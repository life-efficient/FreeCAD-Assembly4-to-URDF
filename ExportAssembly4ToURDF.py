import FreeCAD as App
import Part
import Mesh
import ImportGui
import os
import math

DOC = App.ActiveDocument
ROBOT_NAME = "my_robot"
EXPORT_DIR = os.path.join(os.path.expanduser("~"), "projects/FreeCAD-Designs", "macros", ROBOT_NAME)
MESH_FORMAT = "stl"  # or 'dae'
SCALE = 0.001  # mm → m
PLA_DENSITY = 1240  # kg/m^3
USE_PACKAGE_PREFIX = False  # Set to True for package://, False for absolute path

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
    if USE_PACKAGE_PREFIX:
        return f"package://{ROBOT_NAME}/meshes/{name}.{MESH_FORMAT}"
    else:
        return mesh_path

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
        placement = part.Placement
    else:
        # Direct PartDesign::Body
        body = part
        name = part.Name
        placement = part.Placement
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
    # xyz, rpy = format_placement(placement)  # No longer used for visual/collision

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

def write_joint(f, joint):
    joint_type = getattr(joint, "JointType", "revolute").lower()
    reference1 = getattr(joint, "Reference1", None)
    reference2 = getattr(joint, "Reference2", None)
    placement1 = getattr(joint, "Placement1", None)
    placement2 = getattr(joint, "Placement2", None)
    axis = getattr(joint, "Axis", None) if hasattr(joint, "Axis") else None

    # Special case for grounded joint
    if reference1 is None and reference2 is None:
        semantic_name = "grounded_joint"
        parent = "world"
        obj_to_ground = getattr(joint, "ObjectToGround", None)
        child = getattr(obj_to_ground, "Name", "unknown") if obj_to_ground else "unknown"
    else:
        parent = get_link_name_from_reference(reference1)
        child = get_link_name_from_reference(reference2)
        def sanitize(name):
            return str(name).replace(' ', '_').replace('-', '_') if name else 'none'
        semantic_name = f"{sanitize(parent)}_to_{sanitize(child)}_{joint_type}"

    print(f"Joint: {semantic_name}, type: {joint_type}, parent: {parent}, child: {child}, placement1: {placement1}, placement2: {placement2}, axis: {axis}")
    # Debug print all properties of the joint
    print(f"All properties for joint {joint.Name}:")
    for prop in dir(joint):
        if not prop.startswith('__'):
            try:
                print(f'  {prop}: {getattr(joint, prop)}')
            except Exception as e:
                print(f'  {prop}: <error: {e}>')
    # Compute relative transform for URDF joint origin
    if placement1 and placement2:
        rel = placement1.inverse().multiply(placement2)
        xyz, rpy = format_placement(rel)
        print(f"[DEBUG] {semantic_name} relative transform: xyz={xyz}, rpy={rpy}")
    else:
        xyz, rpy = "0 0 0", "0 0 0"
    # Extract Z axis from Placement1's rotation for revolute joints
    axis_vec = None
    if placement1 is not None:
        axis_vec = placement1.Rotation.multVec(App.Vector(0,0,1))
        print(f"[DEBUG] {semantic_name} extracted axis from Placement1: {axis_vec}")
        if abs(axis_vec.Length - 1.0) > 1e-6:
            print(f"[WARNING] {semantic_name} axis is not a unit vector: {axis_vec}")
        # Warn if axis is flipped (Z component negative)
        if axis_vec.z < 0:
            print(f"[WARNING] {semantic_name} axis appears flipped (z < 0): {axis_vec}")
    else:
        print(f"[DEBUG] {semantic_name} axis: None (defaulting to 0 0 1)")
    # Write the joint to the URDF file
    f.write(f'  <joint name="{semantic_name}" type="{joint_type}">\n')
    f.write(f'    <parent link="{parent}"/>\n')
    f.write(f'    <child link="{child}"/>\n')
    f.write(f'    <origin xyz="{xyz}" rpy="{rpy}"/>\n')
    # Write axis if not fixed
    if joint_type != "fixed":
        if axis_vec is not None:
            f.write(f'    <axis xyz="{axis_vec.x} {axis_vec.y} {axis_vec.z}"/>\n')
        else:
            f.write('    <axis xyz="0 0 1"/>\n')
    # Write joint limits for revolute/prismatic
    if joint_type in ["revolute", "prismatic"]:
        lower = getattr(joint, "LowerLimit", -3.14)
        upper = getattr(joint, "UpperLimit", 3.14)
        effort = getattr(joint, "Effort", 1)
        velocity = getattr(joint, "Velocity", 1)
        f.write(f'    <limit lower="{lower}" upper="{upper}" effort="{effort}" velocity="{velocity}"/>\n')
    f.write('  </joint>\n')

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
    # Write URDF links and joints (joints writing to be implemented next)
    with open(os.path.join(EXPORT_DIR, "robot.urdf"), "w") as f:
        f.write(f'<robot name="{ROBOT_NAME}">\n\n')

        # Write all links
        for part in robot_parts:
            write_link(f, part)

        # Write all joints
        for joint in joint_objs:
            write_joint(f, joint)

        f.write('</robot>\n')

    # Success message and summary
    print(f"\nExport complete!\nURDF exported to: {os.path.join(EXPORT_DIR, 'robot.urdf')}\nExported {len(robot_parts)} robot parts and {len(joint_objs)} joints.")

    # Print the contents of the exported URDF for debugging
    urdf_path = os.path.join(EXPORT_DIR, 'robot.urdf')
    print("\n--- URDF Output ---")
    with open(urdf_path, 'r') as urdf_file:
        print(urdf_file.read())


def main():
    assemblyToURDF()
