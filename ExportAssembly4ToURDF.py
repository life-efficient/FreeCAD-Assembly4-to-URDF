import FreeCAD as App
import Part
import Mesh
import ImportGui
import os
import math

DOC = App.ActiveDocument
ROBOT_NAME = "my_robot"
EXPORT_DIR = "/Users/harryberg/projects/FreeCAD Designs/macros"  # Change this to your target directory
MESH_FORMAT = "dae"  # or 'stl'
SCALE = 0.001  # mm → m

def ensure_dir(path):
    if not os.path.exists(path):
        os.makedirs(path)

def radians(deg):
    return deg * math.pi / 180.0

def format_vector(v):
    return f"{v.x * SCALE:.6f} {v.y * SCALE:.6f} {v.z * SCALE:.6f}"

def format_rotation(rot):
    return f"{radians(rot.x):.6f} {radians(rot.y):.6f} {radians(rot.z):.6f}"

def export_mesh(body, name):
    mesh_path = os.path.join(EXPORT_DIR, "meshes", f"{name}.{MESH_FORMAT}")
    shape = body.Shape.copy()
    shape.scale(SCALE)
    Mesh.export([shape], mesh_path)
    return f"package://{ROBOT_NAME}/meshes/{name}.{MESH_FORMAT}"

def get_inertial(body):
    props = body.Shape.MatrixOfInertia
    mass = body.Shape.Mass
    com = body.Shape.CenterOfMass
    return {
        "mass": mass,
        "com": com,
        "inertia": {
            "ixx": props.A11,
            "ixy": props.A12,
            "ixz": props.A13,
            "iyy": props.A22,
            "iyz": props.A23,
            "izz": props.A33
        }
    }

def write_link(f, part):
    body = next((obj for obj in part.Group if obj.TypeId == "PartDesign::Body"), None)
    if not body:
        return

    name = part.Name
    mesh_path = export_mesh(body, name)
    inertial = get_inertial(body)

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

def main():
    print('running' + '\n'*5)
    ensure_dir(EXPORT_DIR)
    ensure_dir(os.path.join(EXPORT_DIR, "meshes"))

    # Find the top-level assemblies (App::Part)
    # print([(obj.Name, obj.TypeId) for obj in DOC.Objects])
    assembly = [obj for obj in DOC.Objects if obj.TypeId == "Assembly::AssemblyObject"]
    try:
        assembly = assembly[0]
    except:
        print('no assembly found')
        return
    

    # jointgroup = [obj for obj in assembly.Group if obj.TypeId == "Assembly::JointGroup"]
    # try:
    #     jointgroup = jointgroup[0]
    # except:
    #     print('no joint group found')
    #     return
    # # print(jointgroup.Name)
    # # print([(obj.Name, obj.TypeId) for obj in jointgroup.Group])
    # joints = [obj for obj in jointgroup.Group]
    # if len(joints) == 0:
    #     print('no joints found')
    #     return
    # print([(obj.Name, obj.TypeId) for obj in joints]) # [('GroundedJoint', 'App::FeaturePython'), ('Joint', 'App::FeaturePython')]
    # try:
    #     grounded_joint = [obj for obj in joints if obj.Name == 'GroundedJoint'][0]
    # except:
    #     print('no grounded joint found')
    #     return
    # print(grounded_joint.Name)
    # print(grounded_joint.Placement)

    # for joint in joints:
    #     print()
    #     # if joint.Name == 'GroundedJoint':
    #     #     # print('joints have the following properties:')
    #     #     # for prop in joint.PropertiesList:
    #     #     #     print(f'\t{prop}: {getattr(joint, prop, None)}')
    #         # continue
    #     print(joint.Name)
    #     # try:    
    #     #     print(joint.Placement)
    #     # except:
    #     #     print('\tno placement found')
    #     # print('\t', dir(joint))
    #     print('\t', joint.PropertiesList)
    #     try:
    #         print('\t', joint.JointType)
    #     except:
    #         continue
    #     for prop in joint.PropertiesList:
    #         print(f'\t{prop}: {getattr(joint, prop, None)}')
        # print(joint.LinkedObject)
        # print(joint.LinkedObject.Name)
        # print(joint.LinkedObject.Placement)
        # print(joint.LinkedObject.LinkedObject)
    # print(grounded_joint.LinkedObject)
    # print(grounded_joint.LinkedObject.Name)
    # print(grounded_joint.LinkedObject.Placement)
    # print(grounded_joint.LinkedObject.LinkedObject)
    # return 
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
    for part in robot_parts:
        print(part.Name, part.TypeId)
    return
    # for assembly in top_assemblies:
    #     robot_parts.extend(collect_parts_recursive([assembly]))

    # lcs_objs = [obj for obj in DOC.Objects if "LCS" in obj.Name]
    # print('num robot parts', len(robot_parts))
    # print('num lcs objs', len(lcs_objs))

    joints_group = find_joints_group(assembly) if assembly else None
    joint_objs = joints_group.Group if joints_group else []
    print('num joints', len(joint_objs))
    for joint in joint_objs:
        print(f'Joint: {joint.Name}, TypeId: {joint.TypeId}, type: {trygetattr(joint, "JointType")}')
        # print('Attributes:', dir(joint))
        # print('Properties:')
        # for prop in joint.PropertiesList:
        #     print(f'  {prop}: {getattr(joint, prop, None)}')
        # print(joint.PropertiesList)

    with open(os.path.join(EXPORT_DIR, "robot.urdf"), "w") as f:
        f.write(f'<robot name="{ROBOT_NAME}">\n\n')

        # Write all links
        for part in robot_parts:
            write_link(f, part)

        # Write all joints
        for lcs in lcs_objs:
            attached = lcs.getLinkedObject(True)
            if attached and isinstance(attached, App.Part):
                write_joint(f, lcs, attached)

        f.write('</robot>\n')

main()
