import FreeCAD as App
import Part
import Mesh
import ImportGui
import os
import math

DOC = App.ActiveDocument
ROBOT_NAME = "my_robot"
EXPORT_DIR = f"/Users/harryberg/projects/FreeCAD-Designs/macros/{ROBOT_NAME}"
MESH_FORMAT = "stl"  # or 'dae'
SCALE = 0.001  # mm → m
PLA_DENSITY = 1240  # kg/m^3

def ensure_dir(path):
    if not os.path.exists(path):
        os.makedirs(path)

def radians(deg):
    return deg * math.pi / 180.0

def format_vector(v):
    return f"{v.x * SCALE:.6f} {v.y * SCALE:.6f} {v.z * SCALE:.6f}"

def format_rotation(rot):
    return f"{radians(rot.x):.6f} {radians(rot.y):.6f} {radians(rot.z):.6f}"

def format_placement(placement):
    if placement is None:
        return "0 0 0", "0 0 0"
    pos = placement.Base
    rpy = placement.Rotation.toEuler()
    return f"{pos.x * SCALE:.6f} {pos.y * SCALE:.6f} {pos.z * SCALE:.6f}", f"{radians(rpy[0]):.6f} {radians(rpy[1]):.6f} {radians(rpy[2]):.6f}"

def export_mesh(body, name):
    mesh_path = os.path.join(EXPORT_DIR, "meshes", f"{name}.{MESH_FORMAT}")
    Mesh.export([body], mesh_path)
    return f"package://{ROBOT_NAME}/meshes/{name}.{MESH_FORMAT}"

def get_inertial(body, name=None):
    # Hardcode STS3215 servo values if detected in link name
    if name and "STS3125".lower() in name.lower():
        mass = 0.06  # 60g
        com = App.Vector(0, 0, 0)
        inertia = {
            "ixx": 1e-5,
            "ixy": 0.0,
            "ixz": 0.0,
            "iyy": 1e-5,
            "iyz": 0.0,
            "izz": 1e-5,
        }
        return {"mass": mass, "com": com, "inertia": inertia}
    # Otherwise, use PLA calculation
    volume_mm3 = body.Shape.Volume  # mm^3
    volume_m3 = volume_mm3 * 1e-9   # m^3
    mass = volume_m3 * PLA_DENSITY
    com = body.Shape.CenterOfMass
    props = body.Shape.MatrixOfInertia
    inertia = {
        "ixx": props.A11 * 1e-12 * PLA_DENSITY,
        "ixy": props.A12 * 1e-12 * PLA_DENSITY,
        "ixz": props.A13 * 1e-12 * PLA_DENSITY,
        "iyy": props.A22 * 1e-12 * PLA_DENSITY,
        "iyz": props.A23 * 1e-12 * PLA_DENSITY,
        "izz": props.A33 * 1e-12 * PLA_DENSITY,
    }
    return {"mass": mass, "com": com, "inertia": inertia}

def write_link(f, part):
    # If this is an App::Link, follow to the linked body
    if part.TypeId == "App::Link":
        body = part.LinkedObject
        name = part.Name
    else:
        # Direct PartDesign::Body
        body = part
        name = part.Name
    if not body or body.TypeId != "PartDesign::Body":
        print(f"Skipping {name}: not a PartDesign::Body")
        return
    # Check for valid shape
    if not hasattr(body, "Shape") or body.Shape is None or body.Shape.isNull():
        print(f"Skipping {name}: no valid shape to export")
        return
    print(f"Exporting link: {name}, body: {body}, has shape: {hasattr(body, 'Shape') and not body.Shape.isNull()}")

    mesh_path = export_mesh(body, name)
    inertial = get_inertial(body, name)

    f.write(f'  <link name="{name}">\n')
    f.write(f'    <inertial>\n')
    f.write(f'      <origin xyz="{format_vector(inertial["com"])}" rpy="0 0 0"/>\n')
    f.write(f'      <mass value="{inertial["mass"]:.6f}"/>\n')
    f.write(f'      <inertia ')
    for k, v in inertial["inertia"].items():
        f.write(f'{k}="{v:.6f}" ')
    f.write('/>\n')
    f.write(f'    </inertial>\n')
    f.write(f'    <visual>\n')
    f.write(f'      <origin xyz="0 0 0" rpy="0 0 0"/>\n')
    f.write(f'      <geometry>\n')
    f.write(f'        <mesh filename="{mesh_path}"/>\n')
    f.write(f'      </geometry>\n')
    f.write(f'    </visual>\n')
    f.write(f'    <collision>\n')
    f.write(f'      <origin xyz="0 0 0" rpy="0 0 0"/>\n')
    f.write(f'      <geometry>\n')
    f.write(f'        <mesh filename="{mesh_path}"/>\n')
    f.write(f'      </geometry>\n')
    f.write(f'    </collision>\n')
    f.write(f'  </link>\n')

def write_joint(f, lcs, child_part):
    parent = lcs.LinkedObject
    if not parent:
        return

    parent_name = parent.Name
    child_name = child_part.Name
    placement = lcs.Placement

    f.write(f'  <joint name="{parent_name}_to_{child_name}" type="revolute">\n')  # default: revolute
    f.write(f'    <parent link="{parent_name}"/>\n')
    f.write(f'    <child link="{child_name}"/>\n')
    f.write(f'    <origin xyz="{format_vector(placement.Base)}" rpy="{format_rotation(placement.Rotation.toEuler())}"/>\n')
    f.write(f'    <axis xyz="0 0 1"/>\n')  # default axis
    f.write(f'  </joint>\n')

def collect_parts_recursive(group):
    parts = []
    for obj in group:
        if hasattr(obj, "Group"):
            parts.append(obj)
            parts.extend(collect_parts_recursive(obj.Group))
        elif obj.TypeId == "PartDesign::Body":
            parts.append(obj)
    return parts

def find_joints_group(assembly):
    for obj in assembly.Group:
        if obj.TypeId == "Assembly::JointGroup":
            if "Joints" in obj.Name:
                return obj
    return None

def trygetattr(obj, prop):
    try:
        return getattr(obj, prop)
    except:
        return "<no " + prop + ">"

def get_link_name_from_reference(ref):
    # ref is a tuple: (<Assembly object>, ['LinkName.EdgeX', ...])
    if ref and len(ref) == 2 and len(ref[1]) > 0:
        return ref[1][0].split('.')[0]
    return None

def assemblyToURDF():
    print('running' + '\n'*5)
    ensure_dir(EXPORT_DIR)
    ensure_dir(os.path.join(EXPORT_DIR, "meshes"))

    assembly = [obj for obj in DOC.Objects if obj.TypeId == "Assembly::AssemblyObject"]
    try:
        assembly = assembly[0]
    except:
        print('no assembly found')
        return
    
    # Collect only real robot parts: App::Link to PartDesign::Body, or direct PartDesign::Body
    robot_parts = []
    for obj in collect_parts_recursive([assembly]):
        if obj.TypeId == "App::Link" and getattr(obj, "LinkedObject", None) and getattr(obj.LinkedObject, "TypeId", None) == "PartDesign::Body":
            robot_parts.append(obj)
        elif obj.TypeId == "PartDesign::Body":
            robot_parts.append(obj)
    print('Robot parts to export:')
    for part in robot_parts:
        if part.TypeId == "App::Link":
            print(f'  {part.Name} (App::Link) → {part.LinkedObject.Name} ({part.LinkedObject.TypeId})')
        else:
            print(f'  {part.Name} ({part.TypeId})')
    print('num robot parts', len(robot_parts))

    joints_group = find_joints_group(assembly) if assembly else None
    joint_objs = joints_group.Group if joints_group else []
    print('num joints', len(joint_objs))
    for joint in joint_objs:
        joint_type = getattr(joint, "JointType", "revolute").lower()
        parent = get_link_name_from_reference(getattr(joint, "Reference1", None))
        child = get_link_name_from_reference(getattr(joint, "Reference2", None))
        placement1 = getattr(joint, "Placement1", None)
        placement2 = getattr(joint, "Placement2", None)
        print(f"Joint: {joint.Name}, type: {joint_type}, parent: {parent}, child: {child}, placement1: {placement1}, placement2: {placement2}")
    # (existing extraction logic remains below for reference)
    # for joint in joint_objs:
    #     parent = getattr(joint, "Parent", None)
    #     child = getattr(joint, "Child", None)
    #     joint_type = getattr(joint, "JointType", "revolute")
    #     placement = getattr(joint, "Placement", None)
    #     axis = getattr(joint, "Axis", None)
    #     print(f"Joint: {joint.Name}, type: {joint_type}, parent: {parent}, child: {child}, placement: {placement}, axis: {axis}")

    # Write URDF links and joints (joints writing to be implemented next)
    with open(os.path.join(EXPORT_DIR, "robot.urdf"), "w") as f:
        f.write(f'<robot name="{ROBOT_NAME}">\n\n')

        # Write all links
        for part in robot_parts:
            write_link(f, part)

        # Write all joints
        for joint in joint_objs:
            joint_type = getattr(joint, "JointType", "revolute").lower()
            parent = get_link_name_from_reference(getattr(joint, "Reference1", None))
            child = get_link_name_from_reference(getattr(joint, "Reference2", None))
            placement1 = getattr(joint, "Placement1", None)
            xyz, rpy = format_placement(placement1)
            if joint.Name == "GroundedJoint":
                # Handle grounded joint: connect base link to world as fixed
                base_link = getattr(joint, "ObjectToGround", None)
                base_link_name = base_link.Name if base_link else None
                if base_link_name:
                    f.write(f'  <joint name="world_to_{base_link_name}" type="fixed">\n')
                    f.write(f'    <parent link="world"/>\n')
                    f.write(f'    <child link="{base_link_name}"/>\n')
                    f.write(f'    <origin xyz="{xyz}" rpy="{rpy}"/>\n')
                    f.write(f'  </joint>\n')
            elif parent and child:
                f.write(f'  <joint name="{parent}_to_{child}" type="{joint_type}">\n')
                f.write(f'    <parent link="{parent}"/>\n')
                f.write(f'    <child link="{child}"/>\n')
                f.write(f'    <origin xyz="{xyz}" rpy="{rpy}"/>\n')
                if joint_type == "revolute":
                    f.write(f'    <axis xyz="0 0 1"/>\n')
                f.write(f'  </joint>\n')

        f.write('</robot>\n')

    # Success message and summary
    print(f"\nExport complete!\nURDF exported to: {os.path.join(EXPORT_DIR, 'robot.urdf')}\nExported {len(robot_parts)} robot parts and {len(joint_objs)} joints.")


def main():
    assemblyToURDF()
