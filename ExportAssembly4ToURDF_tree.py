import FreeCAD as App
import Part
import Mesh
import os
import math

from utils_io import ensure_dir
from freecad_helpers import get_link_name_from_reference, export_mesh, get_inertial, get_joint_urdf_transform, get_joint_axis_in_urdf_frame, get_joint_alignment
from utils_math import format_vector, format_placement

# --- Logging helpers ---
MANUAL_CHECK_FILE = os.path.join(os.path.dirname(__file__), "manual_check.txt")
with open(MANUAL_CHECK_FILE, 'w', encoding='utf-8') as f:
    f.write('')  # Clear file at start

def log_message(msg):
    # Safety check: prevent infinite loops/runaway logging
    MANUAL_CHECK_FILE = os.path.join(os.path.dirname(__file__), "manual_check.txt")
    try:
        with open(MANUAL_CHECK_FILE, 'r', encoding='utf-8') as f:
            line_count = sum(1 for _ in f)
        assert line_count < 100, f"manual_check.txt exceeds 100 lines ({line_count}) -- possible infinite loop!"
    except FileNotFoundError:
        pass
    print(msg)
    with open(MANUAL_CHECK_FILE, 'a', encoding='utf-8') as f:
        f.write(msg + '\n')

def log_newline():
    with open(MANUAL_CHECK_FILE, 'a', encoding='utf-8') as f:
        f.write('\n')

# --- Assembly tree printer ---
def print_assembly_tree(root_link):
    log_message('--- ASSEMBLY TREE ---')
    def print_link(link, depth=0, visited=None):
        if visited is None:
            visited = set()
        if link.name in visited:
            log_message('  ' * depth + f'[CYCLE] {link.name}')
            return
        log_message('  ' * depth + f'Link: {link.name}')
        visited.add(link.name)
        print(link.joints)
        for joint in link.joints:
            joint_name = getattr(joint, 'name', '<no name>')
            joint_type = getattr(joint, 'joint_type', '<none>')
            child_link = joint.child_link
            child_link_name = child_link.name if child_link else '<no child>'
            log_message('  ' * (depth + 1) + f'Joint: {joint_name} (type: {joint_type}) -> {child_link_name}')
            if child_link:
                print_link(child_link, depth + 2, visited.copy())
    print_link(root_link)
    log_newline()

DOC = App.ActiveDocument
ROBOT_NAME = "my_robot"
EXPORT_DIR = os.path.join(os.path.expanduser("~"), "projects/FreeCAD-Designs", "macros", ROBOT_NAME)
MESH_FORMAT = "stl"  # or 'dae'
SCALE = 1.0  # mm → m (set to 1.0 for no scaling)
PLA_DENSITY = 1240  # kg/m^3
USE_PACKAGE_PREFIX = False  # Set to True for package://, False for absolute path
MANUAL_CHECK = True  # Set to True to print human-readable transform checks

# Add a global variable to track if we've already cleared the manual check file this run
# _manual_check_file_cleared = False # This variable is no longer needed as log_message handles clearing

# def print_manual_check(name, parent, xyz, rpy, kind): # This function is no longer needed
#     global _manual_check_file_cleared
#     x_raw, y_raw, z_raw = xyz.split()
#     # Remove trailing zeros and unnecessary decimal points
#     def clean_num(val):
#         try:
#             fval = float(val)
#             if fval.is_integer():
#                 return str(int(fval))
#             else:
#                 return str(fval).rstrip('0').rstrip('.') if '.' in str(fval) else str(fval)
#         except Exception:
#             return val
#     x_raw_c = clean_num(x_raw)
#     y_raw_c = clean_num(y_raw)
#     z_raw_c = clean_num(z_raw)
#     rx_deg = math.degrees(float(rpy.split()[0]))
#     ry_deg = math.degrees(float(rpy.split()[1]))
#     rz_deg = math.degrees(float(rpy.split()[2]))
#     lines = [
#         f"Does this look right for {kind} '{name}' relative to '{parent}'?",
#         f"  Translated by: X: {x_raw_c} Y: {y_raw_c} Z: {z_raw_c}",
#         f"  Rotated by:   X: {rx_deg:.1f}° Y: {ry_deg:.1f}° Z: {rz_deg:.1f}°"
#     ]
#     for line in lines:
#         print(line)
#     print()
#     # Overwrite the file at the start of each run, append for subsequent calls
#     mode = 'w' if not _manual_check_file_cleared else 'a'
#     with open(MANUAL_CHECK_FILE, mode, encoding='utf-8') as f:
#         for line in lines:
#             f.write(line + '\n')
#         f.write('\n')
#     _manual_check_file_cleared = True

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
    if is_root or mesh_offset_placement is None:
        xyz, rpy = "0 0 0", "0 0 0"
        if MANUAL_CHECK:
            msg = f"Does this look right for LINK (root) '{name}' relative to '{parent_name or 'world'}'?\n  Translated by: X: 0 Y: 0 Z: 0\n  Rotated by:   X: 0.0° Y: 0.0° Z: 0.0°"
            log_message(msg)
            log_newline()
    else:
        xyz, rpy = format_placement(mesh_offset_placement.inverse(), scale=SCALE)
        if MANUAL_CHECK:
            msg = f"Does this look right for LINK '{name}' relative to '{parent_name}'?\n  Translated by: X: {xyz.split()[0]} Y: {xyz.split()[1]} Z: {xyz.split()[2]}\n  Rotated by:   X: {math.degrees(float(rpy.split()[0])):.1f}° Y: {math.degrees(float(rpy.split()[1])):.1f}° Z: {math.degrees(float(rpy.split()[2])):.1f}°"
            log_message(msg)
            log_newline()
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
        log_message(joint.Name)
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
            if obj_to_ground and hasattr(obj_to_ground, "Name") and obj_to_ground.Name in links:
                grounded_links.append(links[obj_to_ground.Name])
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

# --- New helper functions ---
def compute_joint_transform(parent_placement, child_placement):
    # Returns the URDF joint <origin> transform
    return parent_placement.inverse().multiply(child_placement)

def compute_mesh_offset(child_placement):
    # Returns the mesh offset transform for the link
    return child_placement.inverse()

# --- Placement extraction helper ---
def get_joint_placements(joint, link_name):
    reference1 = getattr(joint, "Reference1", None)
    reference2 = getattr(joint, "Reference2", None)
    from freecad_helpers import get_link_name_from_reference
    link1 = get_link_name_from_reference(reference1)
    link2 = get_link_name_from_reference(reference2)
    if link1 == link_name:
        parent_placement = getattr(joint, "Placement1", None)
        child_placement = getattr(joint, "Placement2", None)
    else:
        parent_placement = getattr(joint, "Placement2", None)
        child_placement = getattr(joint, "Placement1", None)
    return parent_placement, child_placement

# --- Simplified handle_link and handle_joint ---
class FreeCADLink:
    def __init__(self, part):
        if part.TypeId == "App::Link":
            self.body = part.LinkedObject
            self.name = part.Name
        else:
            self.body = part
            self.name = part.Name
        self.type_id = self.body.TypeId if self.body else None
        self.shape = self.body.Shape if self.body and hasattr(self.body, "Shape") else None
        if not self.body or self.type_id != "PartDesign::Body":
            raise RuntimeError(f"ERROR: {self.name} is not a PartDesign::Body")
        if not self.shape or self.shape.isNull():
            raise RuntimeError(f"ERROR: {self.name} has no valid shape to export")
        self.joints = []  # List of FreeCADJoint objects

class URDFLink:
    def __init__(self, freecad_link, mesh_offset=None, is_root=False, parent_name=None):
        from freecad_helpers import export_mesh, get_inertial
        from utils_math import format_vector, format_placement
        self.name = freecad_link.name
        self.body = freecad_link.body
        self.mesh_path = export_mesh(self.body, self.name)
        self.inertial = get_inertial(self.body, self.name)
        self.is_root = is_root
        self.parent_name = parent_name
        if is_root or mesh_offset is None:
            self.xyz, self.rpy = "0 0 0", "0 0 0"
        else:
            self.xyz, self.rpy = format_placement(mesh_offset, scale=SCALE)

# Refactor handle_link to use FreeCADLink and URDFLink

def handle_link(f, freecad_link, prev_joint=None, is_root=False, joint_name=None, parent_name=None):
    log_message(f"[handle_link] Handling link: {freecad_link.name} (is_root={is_root}, parent={parent_name})")
    if is_root or prev_joint is None or not hasattr(prev_joint, 'parent_placement') or prev_joint.parent_placement is None:
        mesh_offset = None
    else:
        mesh_offset = prev_joint.parent_placement.inverse()
    urdf_link = URDFLink(freecad_link, mesh_offset, is_root=is_root, parent_name=parent_name)
    if MANUAL_CHECK:
        msg = f"Does this look right for LINK (root) '{urdf_link.name}' relative to '{urdf_link.parent_name or 'world'}'?\n  Translated by: X: {urdf_link.xyz.split()[0]} Y: {urdf_link.xyz.split()[1]} Z: {urdf_link.xyz.split()[2]}\n  Rotated by:   X: {math.degrees(float(urdf_link.rpy.split()[0])):.1f}° Y: {math.degrees(float(urdf_link.rpy.split()[1])):.1f}° Z: {math.degrees(float(urdf_link.rpy.split()[2])):.1f}°"
        log_message(msg)
        log_newline()
    f.write(f'  <link name="{urdf_link.name}">\n')
    f.write(f'    <inertial>\n')
    f.write(f'      <origin xyz="{format_vector(urdf_link.inertial["com"])}" rpy="0 0 0"/>\n')
    f.write(f'      <mass value="{urdf_link.inertial["mass"]:.6f}"/>\n')
    f.write(f'      <inertia ')
    for k, v in urdf_link.inertial["inertia"].items():
        f.write(f'{k}="{v:.6f}" ')
    f.write('/>\n')
    f.write(f'    </inertial>\n')
    f.write(f'    <visual>\n')
    f.write(f'      <origin xyz="{urdf_link.xyz}" rpy="{urdf_link.rpy}"/>\n')
    f.write(f'      <geometry>\n')
    f.write(f'        <mesh filename="{urdf_link.mesh_path}"/>\n')
    f.write(f'      </geometry>\n')
    f.write(f'    </visual>\n')
    f.write(f'    <collision>\n')
    f.write(f'      <origin xyz="{urdf_link.xyz}" rpy="{urdf_link.rpy}"/>\n')
    f.write(f'      <geometry>\n')
    f.write(f'        <mesh filename="{urdf_link.mesh_path}"/>\n')
    f.write(f'      </geometry>\n')
    f.write(f'    </collision>\n')
    f.write(f'  </link>\n')

class FreeCADJoint:
    def __init__(self, joint, link_name, child_link=None):
        self.joint = joint
        self.link_name = link_name
        reference1 = getattr(joint, "Reference1", None)
        reference2 = getattr(joint, "Reference2", None)
        from freecad_helpers import get_link_name_from_reference
        link1 = get_link_name_from_reference(reference1)
        link2 = get_link_name_from_reference(reference2)
        if link1 == link_name:
            self.parent_placement = getattr(joint, "Placement1", None)
            self.child_placement = getattr(joint, "Placement2", None)
        else:
            self.parent_placement = getattr(joint, "Placement2", None)
            self.child_placement = getattr(joint, "Placement1", None)
        self.from_parent_origin = self.parent_placement
        self.from_child_origin = self.child_placement
        self.joint_type = getattr(joint, "JointType", "revolute").lower()
        self.name = getattr(joint, "Name", None)
        self.reference1 = reference1
        self.reference2 = reference2
        self.child_link = child_link  # Should be a FreeCADLink or None

class URDFJoint:
    def __init__(self, prev_joint, curr_joint):
        self.freecad_joint = curr_joint
        self.joint_type = curr_joint.joint_type
        self.name = curr_joint.name
        # Compose the transform: prev_joint.from_parent_origin.inverse() * curr_joint.from_child_origin
        if prev_joint is not None and hasattr(prev_joint, 'from_parent_origin') and prev_joint.from_parent_origin is not None and curr_joint.from_child_origin is not None:
            self.urdf_transform = prev_joint.from_parent_origin.inverse().multiply(curr_joint.from_child_origin)
        else:
            self.urdf_transform = curr_joint.from_child_origin
        # Axis in parent frame
        if curr_joint.from_child_origin is not None:
            import FreeCAD as App
            self.axis = curr_joint.from_child_origin.Rotation.multVec(App.Vector(0,0,1))
        else:
            self.axis = None

# Update handle_joint to use new URDFJoint signature

def handle_joint(f, prev_joint, curr_joint, parent_link, child_link):
    from utils_math import format_placement
    log_message(f"[handle_joint] Handling joint: {getattr(curr_joint, 'name', '<no name>')} (type={getattr(curr_joint, 'joint_type', '<none>')}) parent={parent_link} child={child_link}")
    joint_type = curr_joint.joint_type
    def sanitize(name):
        return str(name).replace(' ', '_').replace('-', '_') if name else 'none'
    if getattr(curr_joint, "reference1", None) is None and getattr(curr_joint, "reference2", None) is None:
        semantic_name = "grounded_joint"
        parent = "world"
        child = child_link
    else:
        parent = parent_link
        child = child_link
        semantic_name = f"{sanitize(parent)}-{sanitize(child)}_{joint_type}"
    # Compute URDF joint transform
    urdf_joint = URDFJoint(prev_joint, curr_joint)
    joint_xyz, joint_rpy = format_placement(urdf_joint.urdf_transform, scale=SCALE)
    axis_vec = urdf_joint.axis
    if parent_link != parent and axis_vec is not None:
        import FreeCAD as App
        axis_vec = App.Vector(-axis_vec.x, -axis_vec.y, -axis_vec.z)
    if MANUAL_CHECK:
        msg = f"Does this look right for JOINT '{semantic_name}' relative to '{parent}'?\n  Translated by: X: {joint_xyz.split()[0]} Y: {joint_xyz.split()[1]} Z: {joint_xyz.split()[2]}\n  Rotated by:   X: {math.degrees(float(joint_rpy.split()[0])):.1f}° Y: {math.degrees(float(joint_rpy.split()[1])):.1f}° Z: {math.degrees(float(joint_rpy.split()[2])):.1f}°"
        log_message(msg)
        log_newline()
    if joint_type == "fixed":
        writeFixedJoint(f, curr_joint.joint, semantic_name, parent, child, joint_xyz, joint_rpy)
    elif joint_type == "revolute":
        writeRevoluteJoint(f, curr_joint.joint, semantic_name, parent, child, joint_xyz, joint_rpy, axis_vec)
    else:
        raise RuntimeError(f"ERROR: Unsupported joint type: {joint_type}")

# --- New object graph initialization and traversal ---
def find_grounded_joint_and_link(joint_objs, links):
    from freecad_helpers import get_link_name_from_reference
    for joint_obj in joint_objs:
        if getattr(joint_obj, "Reference1", None) is None and getattr(joint_obj, "Reference2", None) is None:
            obj_to_ground = getattr(joint_obj, "ObjectToGround", None)
            grounded_name = getattr(obj_to_ground, "Name", None)
            if grounded_name in links:
                return joint_obj, links[grounded_name]
    return None, None

def build_assembly_tree(robot_parts, joint_objs):
    from freecad_helpers import get_link_name_from_reference
    # Build all FreeCADLink objects
    links = {part.Name: FreeCADLink(part) for part in robot_parts}
    log_message('[DEBUG] Link names in links dict: ' + ', '.join(links.keys()))
    # 1. Identify the root joint (grounded)
    root_joint = None
    for joint_obj in joint_objs:
        name = getattr(joint_obj, 'Name', '').lower()
        joint_type = getattr(joint_obj, 'JointType', '').lower()
        if 'ground' in name or 'ground' in joint_type:
            root_joint = joint_obj
            break
    if not root_joint:
        log_message('[DEBUG] No grounded joint found!')
        return None, links, []
    # 2. Find the single link this joint attaches to (from Placement1 or Placement2)
    ref1 = get_link_name_from_reference(getattr(root_joint, 'Reference1', None))
    ref2 = get_link_name_from_reference(getattr(root_joint, 'Reference2', None))
    candidates = [ref for ref in [ref1, ref2] if ref in links]
    if getattr(root_joint, 'ObjectToGround', None):
        obj_to_ground = getattr(root_joint, 'ObjectToGround', None)
        if hasattr(obj_to_ground, 'Name') and obj_to_ground.Name in links:
            candidates.append(obj_to_ground.Name)
    candidates = list(set(candidates))
    if len(candidates) != 1:
        raise RuntimeError(f"ERROR: Expected exactly one grounded link, found: {candidates}")
    root_link = links[candidates[0]]
    log_message(f"[DEBUG] Grounded joint found: {getattr(root_joint, 'Name', None)} grounding {root_link.name}")
    # 3. Recursively build the tree
    visited_links = set()
    def build_tree(link):
        if link.name in visited_links:
            return
        visited_links.add(link.name)
        link.joints = []
        for joint_obj in joint_objs:
            ref1 = get_link_name_from_reference(getattr(joint_obj, 'Reference1', None))
            ref2 = get_link_name_from_reference(getattr(joint_obj, 'Reference2', None))
            if link.name == ref1 or link.name == ref2:
                # 4. Attach this joint to the link
                joint = FreeCADJoint(joint_obj, link.name)
                # 5. Find the OTHER link
                other_link_name = ref2 if link.name == ref1 else ref1
                if other_link_name and other_link_name in links:
                    joint.child_link = links[other_link_name]
                    link.joints.append(joint)
                    # 6. Recurse
                    build_tree(joint.child_link)
    build_tree(root_link)
    return root_link, links, []

# New traversal using the object graph

def traverse_link(f, link, parent_joint=None, is_root=False, parent_name=None, visited_links=None, visited_joints=None):
    if visited_links is None:
        visited_links = set()
    if visited_joints is None:
        visited_joints = set()
    # If we've already fully processed this link, skip
    if link.name in visited_links:
        log_message(f'[CYCLE] Skipping already visited link: {link.name}')
        return
    handle_link(f, link, prev_joint=parent_joint, is_root=is_root, parent_name=parent_name)
    for joint in link.joints:
        joint_id = id(joint)
        if joint_id in visited_joints:
            continue
        visited_joints_new = visited_joints.copy()
        visited_joints_new.add(joint_id)
        # Write the joint
        handle_joint(f, parent_joint, joint, link.name, joint.child_link.name)
        # Recursively traverse the child link
        traverse_link(f, joint.child_link, parent_joint=joint, is_root=False, parent_name=link.name, visited_links=visited_links.copy(), visited_joints=visited_joints_new)
    # Only mark the link as visited after all joints/children are processed
    visited_links.add(link.name)

# --- Replace old traversal in assemblyToURDF_tree ---
def assemblyToURDF_tree():
    print('running (assembly tree traversal)' + '\n'*5)
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
    joints_group = find_joints_group(assembly) if assembly else None
    joint_objs = joints_group.Group if joints_group else []
    print('Joint names and JointType values:')
    for joint in joint_objs:
        print(f"  {getattr(joint, 'Name', '<no name>')}: JointType = {getattr(joint, 'JointType', '<none>')}")
    # Build the assembly tree
    root_link, links, joints = build_assembly_tree(robot_parts, joint_objs)
    if root_link:
        print_assembly_tree(root_link)
    urdf_file = os.path.join(EXPORT_DIR, "robot.urdf")
    with open(urdf_file, "w") as f:
        f.write(f'<robot name="{ROBOT_NAME}">\n\n')
        if root_link:
            traverse_link(f, root_link, parent_joint=None, is_root=True, parent_name="world")
        f.write('</robot>\n')
    print(f"\nExport complete!\nURDF exported to: {urdf_file}")

def main():
    assemblyToURDF_tree() 