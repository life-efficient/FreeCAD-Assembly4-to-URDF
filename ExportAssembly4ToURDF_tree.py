import FreeCAD as App
import Part
import Mesh
import os
import math

from utils_math import radians, format_vector, format_rotation, format_placement
from utils_io import ensure_dir
from freecad_helpers import get_link_name_from_reference, export_mesh, get_inertial

DOC = App.ActiveDocument
ROBOT_NAME = "my_robot"
EXPORT_DIR = os.path.join(os.path.expanduser("~"), "projects/FreeCAD-Designs", "macros", ROBOT_NAME)
MESH_FORMAT = "stl"  # or 'dae'
SCALE = 0.001  # mm → m
PLA_DENSITY = 1240  # kg/m^3
USE_PACKAGE_PREFIX = False  # Set to True for package://, False for absolute path

def write_link(f, part, xyz="0 0 0", rpy="0 0 0"):
    if part.TypeId == "App::Link":
        body = part.LinkedObject
        name = part.Name
    else:
        body = part
        name = part.Name
    if not body or body.TypeId != "PartDesign::Body":
        print(f"Skipping {name}: not a PartDesign::Body")
        return
    if not hasattr(body, "Shape") or body.Shape is None or body.Shape.isNull():
        print(f"Skipping {name}: no valid shape to export")
        return
    print(f"Exporting link: {name}, body: {body}, has shape: {hasattr(body, 'Shape') and not body.Shape.isNull()}")
    from freecad_helpers import export_mesh, get_inertial
    from utils_math import format_vector
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
    f.write(f'      <origin xyz="{xyz}" rpy="{rpy}"/>\n')
    f.write(f'      <geometry>\n')
    f.write(f'        <mesh filename="{mesh_path}"/>\n')
    f.write(f'      </geometry>\n')
    f.write(f'    </visual>\n')
    f.write(f'    <collision>\n')
    f.write(f'      <origin xyz="{xyz}" rpy="{rpy}"/>\n')
    f.write(f'      <geometry>\n')
    f.write(f'        <mesh filename="{mesh_path}"/>\n')
    f.write(f'      </geometry>\n')
    f.write(f'    </collision>\n')
    f.write(f'  </link>\n')

def writeFixedJoint(f, joint, semantic_name, parent, child, xyz, rpy):
    f.write(f'  <joint name="{semantic_name}" type="fixed">\n')
    f.write(f'    <parent link="{parent}"/>\n')
    f.write(f'    <child link="{child}"/>\n')
    f.write(f'    <origin xyz="{xyz}" rpy="{rpy}"/>\n')
    f.write('  </joint>\n')

def writeRevoluteJoint(f, joint, semantic_name, parent, child, xyz, rpy, axis_vec):
    f.write(f'  <joint name="{semantic_name}" type="revolute">\n')
    f.write(f'    <parent link="{parent}"/>\n')
    f.write(f'    <child link="{child}"/>\n')
    f.write(f'    <origin xyz="{xyz}" rpy="{rpy}"/>\n')
    if axis_vec is not None:
        f.write(f'    <axis xyz="{axis_vec.x} {axis_vec.y} {axis_vec.z}"/>\n')
    else:
        f.write('    <axis xyz="0 0 1"/>\n')
    lower = getattr(joint, "LowerLimit", -3.14)
    upper = getattr(joint, "UpperLimit", 3.14)
    effort = getattr(joint, "Effort", 1)
    velocity = getattr(joint, "Velocity", 1)
    f.write(f'    <limit lower="{lower}" upper="{upper}" effort="{effort}" velocity="{velocity}"/>\n')
    f.write('  </joint>\n')

def find_joints_group(assembly):
    for obj in assembly.Group:
        if obj.TypeId == "Assembly::JointGroup":
            if "Joints" in obj.Name:
                return obj
    return None

def build_tree(joint_objs):
    parent_to_joints = {}
    child_to_joint = {}
    for joint in joint_objs:
        reference1 = getattr(joint, "Reference1", None)
        reference2 = getattr(joint, "Reference2", None)
        parent = get_link_name_from_reference(reference1)
        child = get_link_name_from_reference(reference2)
        if parent and child:
            if parent not in parent_to_joints:
                parent_to_joints[parent] = []
            parent_to_joints[parent].append((joint, child))
            child_to_joint[child] = joint
        # Special case: grounded joint
        if reference1 is None and reference2 is None:
            obj_to_ground = getattr(joint, "ObjectToGround", None)
            if obj_to_ground and hasattr(obj_to_ground, "Name"):
                parent_to_joints["world"] = [(joint, obj_to_ground.Name)]
                child_to_joint[obj_to_ground.Name] = joint
    return parent_to_joints, child_to_joint

def find_root_link(joint_objs):
    from freecad_helpers import get_link_name_from_reference
    for joint in joint_objs:
        # Detect grounded joint by missing references or name
        is_grounded = (
            (getattr(joint, "Reference1", None) is None and getattr(joint, "Reference2", None) is None)
            or getattr(joint, "Name", "").lower() == "groundedjoint"
        )
        if is_grounded:
            obj_to_ground = getattr(joint, "ObjectToGround", None)
            if obj_to_ground and hasattr(obj_to_ground, "Name"):
                return obj_to_ground.Name
            reference2 = getattr(joint, "Reference2", None)
            child = get_link_name_from_reference(reference2)
            if child:
                return child
    return None

def build_link_to_joints(joint_objs):
    from freecad_helpers import get_link_name_from_reference
    link_to_joints = {}
    for joint in joint_objs:
        reference1 = getattr(joint, "Reference1", None)
        reference2 = getattr(joint, "Reference2", None)
        link1 = get_link_name_from_reference(reference1)
        link2 = get_link_name_from_reference(reference2)
        for link in [link1, link2]:
            if link:
                if link not in link_to_joints:
                    link_to_joints[link] = []
                link_to_joints[link].append(joint)
        # Special case: grounded joint
        if getattr(joint, "Reference1", None) is None and getattr(joint, "Reference2", None) is None:
            obj_to_ground = getattr(joint, "ObjectToGround", None)
            if obj_to_ground and hasattr(obj_to_ground, "Name"):
                link = obj_to_ground.Name
                if link not in link_to_joints:
                    link_to_joints[link] = []
                link_to_joints[link].append(joint)
    return link_to_joints

def traverse_graph(f, link_name, robot_parts_map, link_to_joints, visited_links, visited_joints, from_joint=None, from_link=None):
    from utils_math import format_placement
    from freecad_helpers import get_link_name_from_reference
    # Compute visual/collision offset from the joint we arrived from
    xyz, rpy = "0 0 0", "0 0 0"
    if from_joint is not None:
        # Use Placement2 if we arrived as Reference2, Placement1 if as Reference1
        if from_link is not None:
            if from_link == get_link_name_from_reference(getattr(from_joint, "Reference2", None)):
                placement = getattr(from_joint, "Placement2", None)
            else:
                placement = getattr(from_joint, "Placement1", None)
            if placement:
                xyz, rpy = format_placement(placement)
    if link_name in visited_links:
        print(f'[DEBUG] Link {link_name} already written, skipping.')
        return
    print(f'[DEBUG] Writing link: {link_name} (from_joint: {getattr(from_joint, "Name", None)})')
    part = robot_parts_map.get(link_name)
    if part:
        write_link(f, part, xyz, rpy)
        visited_links.add(link_name)
    for joint in link_to_joints.get(link_name, []):
        if joint in visited_joints:
            continue
        reference1 = getattr(joint, "Reference1", None)
        reference2 = getattr(joint, "Reference2", None)
        link1 = get_link_name_from_reference(reference1)
        link2 = get_link_name_from_reference(reference2)
        # Special case: grounded joint
        if getattr(joint, "Reference1", None) is None and getattr(joint, "Reference2", None) is None:
            obj_to_ground = getattr(joint, "ObjectToGround", None)
            link1 = "world"
            link2 = obj_to_ground.Name if obj_to_ground and hasattr(obj_to_ground, "Name") else None
        # Determine the other link
        other_link = link2 if link1 == link_name else link1
        if other_link is None or other_link == "world":
            continue
        # Write the joint
        # Determine direction for Placement1/2 and axis
        swap = (link_name == link2)
        if swap:
            placement1 = getattr(joint, "Placement2", None)
            placement2 = getattr(joint, "Placement1", None)
        else:
            placement1 = getattr(joint, "Placement1", None)
            placement2 = getattr(joint, "Placement2", None)
        joint_type = getattr(joint, "JointType", "revolute").lower()
        def sanitize(name):
            return str(name).replace(' ', '_').replace('-', '_') if name else 'none'
        if getattr(joint, "Reference1", None) is None and getattr(joint, "Reference2", None) is None:
            semantic_name = "grounded_joint"
            parent = "world"
            child = link2
        else:
            parent = link1 if not swap else link2
            child = link2 if not swap else link1
            semantic_name = f"{sanitize(parent)}_to_{sanitize(child)}_{joint_type}"
        if placement1 and placement2:
            rel = placement1.inverse().multiply(placement2)
            joint_xyz, joint_rpy = format_placement(rel)
        else:
            joint_xyz, joint_rpy = "0 0 0", "0 0 0"
        axis_vec = None
        if placement1 is not None:
            axis_vec = placement1.Rotation.multVec(App.Vector(0,0,1))
            if swap and axis_vec is not None:
                axis_vec = App.Vector(-axis_vec.x, -axis_vec.y, -axis_vec.z)
        print(f'[DEBUG] Writing joint: {getattr(joint, "Name", None)} (parent: {parent}, child: {child})')
        if joint_type == "fixed":
            writeFixedJoint(f, joint, semantic_name, parent, child, joint_xyz, joint_rpy)
        elif joint_type == "revolute":
            writeRevoluteJoint(f, joint, semantic_name, parent, child, joint_xyz, joint_rpy, axis_vec)
        else:
            print(f"[WARNING] Skipping unsupported joint type: {joint_type}")
        visited_joints.add(joint)
        traverse_graph(f, other_link, robot_parts_map, link_to_joints, visited_links, visited_joints, from_joint=joint, from_link=link_name)

# In assemblyToURDF_tree, use build_link_to_joints and start traversal from the grounded link(s)
def assemblyToURDF_tree():
    print('running (graph traversal)' + '\n'*5)
    ensure_dir(EXPORT_DIR)
    ensure_dir(os.path.join(EXPORT_DIR, "meshes"))
    assembly = [obj for obj in DOC.Objects if obj.TypeId == "Assembly::AssemblyObject"]
    try:
        assembly = assembly[0]
    except:
        print('no assembly found')
        return
    robot_parts = []
    for obj in assembly.Group:
        if obj.TypeId == "App::Link" and getattr(obj, "LinkedObject", None) and getattr(obj.LinkedObject, "TypeId", None) == "PartDesign::Body":
            robot_parts.append(obj)
        elif obj.TypeId == "PartDesign::Body":
            robot_parts.append(obj)
    # Also collect recursively
    for obj in assembly.Group:
        if hasattr(obj, "Group"):
            for sub in obj.Group:
                if sub.TypeId == "App::Link" and getattr(sub, "LinkedObject", None) and getattr(sub.LinkedObject, "TypeId", None) == "PartDesign::Body":
                    robot_parts.append(sub)
                elif sub.TypeId == "PartDesign::Body":
                    robot_parts.append(sub)
    robot_parts_map = {part.Name: part for part in robot_parts}
    joints_group = find_joints_group(assembly) if assembly else None
    joint_objs = joints_group.Group if joints_group else []
    print('Joint names and JointType values:')
    for joint in joint_objs:
        print(f"  {getattr(joint, 'Name', '<no name>')}: JointType = {getattr(joint, 'JointType', '<none>')}")
    link_to_joints = build_link_to_joints(joint_objs)
    print('link_to_joints map:')
    for link, joints in link_to_joints.items():
        print(f'  Link: {link}')
        for joint in joints:
            print(f'    Joint: {getattr(joint, "Name", "<no name>")}')
    # Find all grounded links (those attached to world)
    grounded_links = []
    for joint in joint_objs:
        if getattr(joint, "Reference1", None) is None and getattr(joint, "Reference2", None) is None:
            obj_to_ground = getattr(joint, "ObjectToGround", None)
            if obj_to_ground and hasattr(obj_to_ground, "Name"):
                grounded_links.append(obj_to_ground.Name)
    with open(os.path.join(EXPORT_DIR, "robot_tree.urdf"), "w") as f:
        f.write(f'<robot name="{ROBOT_NAME}">\n\n')
        visited_links = set()
        visited_joints = set()
        for root_link in grounded_links:
            traverse_graph(f, root_link, robot_parts_map, link_to_joints, visited_links, visited_joints)
        f.write('</robot>\n')
    print(f"\nExport complete!\nURDF exported to: {os.path.join(EXPORT_DIR, 'robot_tree.urdf')}")

def main():
    assemblyToURDF_tree() 