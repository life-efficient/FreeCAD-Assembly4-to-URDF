import FreeCAD as App
import Part
import Mesh
import os
import math

from utils_io import ensure_dir
from freecad_helpers import get_link_name_from_reference, export_mesh, get_inertial, get_joint_urdf_transform, get_joint_axis_in_urdf_frame

DOC = App.ActiveDocument
ROBOT_NAME = "my_robot"
EXPORT_DIR = os.path.join(os.path.expanduser("~"), "projects/FreeCAD-Designs", "macros", ROBOT_NAME)
MESH_FORMAT = "stl"  # or 'dae'
SCALE = 1.0  # mm → m (set to 1.0 for no scaling)
PLA_DENSITY = 1240  # kg/m^3
USE_PACKAGE_PREFIX = False  # Set to True for package://, False for absolute path
MANUAL_CHECK = True  # Set to True to print human-readable transform checks

# Add a global variable to track if we've already cleared the manual check file this run
MANUAL_CHECK_FILE = os.path.join(os.path.dirname(__file__), "manual_check.txt")
_manual_check_file_cleared = False

def print_manual_check(name, parent, xyz, rpy, kind):
    global _manual_check_file_cleared
    x_raw, y_raw, z_raw = xyz.split()
    # Remove trailing zeros and unnecessary decimal points
    def clean_num(val):
        try:
            fval = float(val)
            if fval.is_integer():
                return str(int(fval))
            else:
                return str(fval).rstrip('0').rstrip('.') if '.' in str(fval) else str(fval)
        except Exception:
            return val
    x_raw_c = clean_num(x_raw)
    y_raw_c = clean_num(y_raw)
    z_raw_c = clean_num(z_raw)
    rx_deg = math.degrees(float(rpy.split()[0]))
    ry_deg = math.degrees(float(rpy.split()[1]))
    rz_deg = math.degrees(float(rpy.split()[2]))
    lines = [
        f"Does this look right for {kind} '{name}' relative to '{parent}'?",
        f"  Translated by: X: {x_raw_c} Y: {y_raw_c} Z: {z_raw_c}",
        f"  Rotated by:   X: {rx_deg:.1f}° Y: {ry_deg:.1f}° Z: {rz_deg:.1f}°"
    ]
    for line in lines:
        print(line)
    print()
    # Overwrite the file at the start of each run, append for subsequent calls
    mode = 'w' if not _manual_check_file_cleared else 'a'
    with open(MANUAL_CHECK_FILE, mode, encoding='utf-8') as f:
        for line in lines:
            f.write(line + '\n')
        f.write('\n')
    _manual_check_file_cleared = True

def write_link(f, part, mesh_offset_placement=None, is_root=False, joint_name=None, parent_name=None):
    from freecad_helpers import export_mesh, get_inertial
    from utils_math import format_vector, format_placement
    import FreeCAD as App
    if part.TypeId == "App::Link":
        body = part.LinkedObject
        name = part.Name
    else:
        body = part
        name = part.Name
    if not body or body.TypeId != "PartDesign::Body":
        raise RuntimeError(f"ERROR: {name} is not a PartDesign::Body")
    if not hasattr(body, "Shape") or body.Shape is None or body.Shape.isNull():
        raise RuntimeError(f"ERROR: {name} has no valid shape to export")
    mesh_path = export_mesh(body, name)
    inertial = get_inertial(body, name)
    xyz, rpy = "0 0 0", "0 0 0"
    if is_root:
        if MANUAL_CHECK:
            print_manual_check(name, parent_name or "world", xyz, rpy, kind="LINK (root)")
    else:
        if mesh_offset_placement is None:
            raise RuntimeError(f"ERROR: Could not find joint attachment frame (Placement) for link '{name}' (joint: '{joint_name}'). This usually means the joint reference is invalid or not supported by the exporter. Please check your Assembly4 joint definition.")
        # Links: use inverse of child_placement
        mesh_offset = mesh_offset_placement.inverse()
        xyz, rpy = format_placement(mesh_offset, scale=SCALE)
        if MANUAL_CHECK:
            print_manual_check(name, parent_name, xyz, rpy, kind="LINK")
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

def traverse_graph(f, link_name, robot_parts_map, link_to_joints, visited_links, visited_joints, from_joint=None, from_link=None, mesh_offset_placement=None, is_root=False, parent_name=None):
    from utils_math import format_placement
    from freecad_helpers import get_link_name_from_reference
    import FreeCAD as App
    if link_name in visited_links:
        return
    part = robot_parts_map.get(link_name)
    if part:
        write_link(f, part, mesh_offset_placement, is_root=is_root, joint_name=getattr(from_joint, 'Name', None), parent_name=parent_name)
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
        if link1 == link_name:
            parent_link = link1
            child_link = link2
            parent_placement = getattr(joint, "Placement1", None)
            child_placement = getattr(joint, "Placement2", None)
        else:
            parent_link = link2
            child_link = link1
            parent_placement = getattr(joint, "Placement2", None)
            child_placement = getattr(joint, "Placement1", None)
        if child_link is None or child_link == "world" or child_link in visited_links:
            continue
        axis_vec = None
        if parent_placement is not None:
            axis_vec = parent_placement.Rotation.multVec(App.Vector(0,0,1))
            if parent_link != link_name and axis_vec is not None:
                axis_vec = App.Vector(-axis_vec.x, -axis_vec.y, -axis_vec.z)
        joint_type = getattr(joint, "JointType", "revolute").lower()
        def sanitize(name):
            return str(name).replace(' ', '_').replace('-', '_') if name else 'none'
        if getattr(joint, "Reference1", None) is None and getattr(joint, "Reference2", None) is None:
            semantic_name = "grounded_joint"
            parent = "world"
            child = child_link
        else:
            parent = parent_link
            child = child_link
            semantic_name = f"{sanitize(parent)}-{sanitize(child)}_{joint_type}"
        if parent_placement and child_placement:
            # Use helper to get joint transform (parent frame to joint frame)
            joint_placement = get_joint_urdf_transform(parent_placement, child_placement)
            joint_xyz, joint_rpy = format_placement(joint_placement, scale=SCALE)
            # Optionally, get the joint axis in parent frame (for revolute/prismatic)
            joint_axis = get_joint_axis_in_urdf_frame(joint_placement)
            print(f"[DEBUG] Joint: {semantic_name}")
            print(f"  joint_placement.Base: {joint_placement.Base}")
            print(f"  joint_placement.Rotation (Euler): {joint_placement.Rotation.toEuler()}")
            print(f"  joint_axis (parent frame): {joint_axis}")
            if MANUAL_CHECK:
                print_manual_check(semantic_name, parent, joint_xyz, joint_rpy, kind="JOINT")
        else:
            raise RuntimeError(f"ERROR: Could not find joint attachment frames (Placement1/2) for joint '{getattr(joint, 'Name', None)}' (parent: '{parent}', child: '{child}').")
        if joint_type == "fixed":
            writeFixedJoint(f, joint, semantic_name, parent, child, joint_xyz, joint_rpy)
        elif joint_type == "revolute":
            writeRevoluteJoint(f, joint, semantic_name, parent, child, joint_xyz, joint_rpy, axis_vec)
        else:
            raise RuntimeError(f"ERROR: Unsupported joint type: {joint_type}")
        visited_joints.add(joint)
        # Links: pass child_placement as mesh_offset_placement for the child link
        traverse_graph(f, child_link, robot_parts_map, link_to_joints, visited_links, visited_joints, from_joint=joint, from_link=parent_link, mesh_offset_placement=child_placement, is_root=False, parent_name=parent)

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
    urdf_file = os.path.join(EXPORT_DIR, "robot.urdf")
    with open(urdf_file, "w") as f:
        f.write(f'<robot name="{ROBOT_NAME}">\n\n')
        visited_links = set()
        visited_joints = set()
        for root_link in grounded_links:
            traverse_graph(f, root_link, robot_parts_map, link_to_joints, visited_links, visited_joints, mesh_offset_placement=None, is_root=True, parent_name="world")
        f.write('</robot>\n')
    print(f"\nExport complete!\nURDF exported to: {urdf_file}")

def main():
    assemblyToURDF_tree() 