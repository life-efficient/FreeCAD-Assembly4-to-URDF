import FreeCAD as App
import os

from utils_io import ensure_dir
from freecad_helpers import export_mesh, get_inertial, get_link_name_from_reference, get_mesh_offset, get_joint_transform, get_joint_axis, are_joint_z_axes_opposed, get_global_placement
from utils_math import format_vector, format_placement
from logging_utils import log_message, log_newline

# --- Logging helpers ---
MANUAL_CHECK_FILE = os.path.join(os.path.dirname(__file__), "manual_check.txt")
with open(MANUAL_CHECK_FILE, 'w', encoding='utf-8') as f:
    f.write('')  # Clear file at start

# --- Assembly tree printer ---
def print_assembly_tree(root_link):
    log_message('--- ASSEMBLY TREE ---')
    def print_link(link, prefix='', is_last=True, visited=None):
        if visited is None:
            visited = set()
        if link.name in visited:
            log_message(f'{prefix}{"└─ " if is_last else "├─ "}[CYCLE] {link.name}')
            return
        log_message(f'{prefix}{"└─ " if is_last else "├─ "}Link: {link.name}')
        visited.add(link.name)
        joints = link.joints
        for idx, joint in enumerate(joints):
            joint_is_last = (idx == len(joints) - 1)
            joint_prefix = prefix + ('    ' if is_last else '│   ')
            joint_name = getattr(joint, 'name', '<no name>')
            joint_type = getattr(joint, 'joint_type', '<none>')
            child_link = joint.child_link
            child_link_name = child_link.name if child_link else '<no child>'
            log_message(f'{joint_prefix}{"└─ " if joint_is_last else "├─ "}Joint: {joint_name} (type: {joint_type}) -> {child_link_name}')
            if child_link:
                next_prefix = joint_prefix + ('    ' if joint_is_last else '│   ')
                print_link(child_link, next_prefix, True, visited.copy())
    print_link(root_link, '', True)
    log_newline()

DOC = App.ActiveDocument
ROBOT_NAME = "my_robot"
EXPORT_DIR = os.path.join(os.path.expanduser("~"), "projects/FreeCAD-Designs", "macros", ROBOT_NAME)
MESH_FORMAT = "stl"  # or 'dae'
SCALE = 1.0  # mm → m (set to 1.0 for no scaling)
PLA_DENSITY = 1240  # kg/m^3
USE_PACKAGE_PREFIX = False  # Set to True for package://, False for absolute path
MANUAL_CHECK = True  # Set to True to print human-readable transform checks

def find_joints_group(assembly):
    for obj in assembly.Group:
        if obj.TypeId == "Assembly::JointGroup":
            if "Joints" in obj.Name:
                return obj
    return None

# # --- New helper functions ---
# def compute_joint_transform(parent_placement, child_placement):
#     # Returns the URDF joint <origin> transform
#     return parent_placement.inverse().multiply(child_placement)

# def compute_mesh_offset(child_placement):
#     # Returns the mesh offset transform for the link
#     return child_placement.inverse()

# --- Placement extraction helper ---
def get_joint_placements(joint, link_name):
    reference1 = getattr(joint, "Reference1", None)
    reference2 = getattr(joint, "Reference2", None)
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
    def __init__(self, freecad_link, is_root=False, parent_name=None, parent_joint=None):
        self.name = freecad_link.name
        self.body = freecad_link.body
        self.mesh_path = export_mesh(self.body, self.name)
        self.inertial = get_inertial(self.body, self.name)
        self.is_root = is_root
        self.parent_name = parent_name
        if is_root:
            log_message("root link - skipping alignment")
            self.xyz, self.rpy = "0 0 0", "0 0 0"
        else:
            assert parent_joint is not None, f"Non-root link {self.name} requires a parent_joint for alignment"
            assert hasattr(parent_joint, 'from_parent_origin') and parent_joint.from_parent_origin is not None, f"parent_joint for {self.name} missing from_parent_origin"
            assert hasattr(parent_joint, 'from_child_origin') and parent_joint.from_child_origin is not None, f"parent_joint for {self.name} missing from_child_origin"
            log_message(f"[DEBUG] parent name: {parent_name}")
            mesh_offset = get_mesh_offset(parent_joint)
            self.xyz, self.rpy = format_placement(mesh_offset, scale=SCALE)

    def write(self, f):
        f.write(f'  <link name="{self.name}">\n')
        f.write(f'    <inertial>\n')
        f.write(f'      <origin xyz="{format_vector(self.inertial["com"])}" rpy="0 0 0"/>\n')
        f.write(f'      <mass value="{self.inertial["mass"]:.6f}"/>\n')
        f.write(f'      <inertia ')
        for k, v in self.inertial["inertia"].items():
            f.write(f'{k}="{v:.6f}" ')
        f.write('/>\n')
        f.write(f'    </inertial>\n')
        f.write(f'    <visual>\n')
        f.write(f'      <origin xyz="{self.xyz}" rpy="{self.rpy}"/>\n')
        f.write(f'      <geometry>\n')
        f.write(f'        <mesh filename="{self.mesh_path}"/>\n')
        f.write(f'      </geometry>\n')
        f.write(f'    </visual>\n')
        f.write(f'    <collision>\n')
        f.write(f'      <origin xyz="{self.xyz}" rpy="{self.rpy}"/>\n')
        f.write(f'      <geometry>\n')
        f.write(f'        <mesh filename="{self.mesh_path}"/>\n')
        f.write(f'      </geometry>\n')
        f.write(f'    </collision>\n')
        f.write(f'  </link>\n')

    def __str__(self):
        def clean(val):
            try:
                fval = float(val)
                return 0.0 if abs(fval) < 1e-8 else fval
            except Exception:
                return val
        xyz_clean = ' '.join(str(clean(x)) for x in self.xyz.split())
        rpy_clean = ' '.join(str(clean(x)) for x in self.rpy.split())
        return (f"""URDFLink(
  name={self.name},
  xyz=\"{xyz_clean}\"
  rpy=\"{rpy_clean}\"
)""")

class FreeCADJoint:
    def __init__(self, joint, link_name, child_link=None, parent_link=None):
        self.joint = joint
        self.link_name = link_name
        reference1 = getattr(joint, "Reference1", None)
        reference2 = getattr(joint, "Reference2", None)
        link1 = get_link_name_from_reference(reference1)
        link2 = get_link_name_from_reference(reference2)
        if link1 == link_name:
            self.from_parent_origin = getattr(joint, "Placement1", None)
            self.from_child_origin = getattr(joint, "Placement2", None)
        else:
            self.from_parent_origin = getattr(joint, "Placement2", None)
            self.from_child_origin = getattr(joint, "Placement1", None)
        self.joint_type = getattr(joint, "JointType", "revolute").lower()
        self.name = getattr(joint, "Name", None)
        self.reference1 = reference1
        self.reference2 = reference2
        self.child_link = child_link  # Should be a FreeCADLink or None
        are_joint_z_axes_opposed(parent_link, self, child_link)

class URDFJoint:
    def __init__(self, prev_joint, curr_joint, parent_link=None, child_link=None):
        self.parent_link = parent_link if parent_link is not None else curr_joint.link_name
        self.child_link = child_link if child_link is not None and hasattr(child_link, 'name') else (curr_joint.child_link.name if curr_joint.child_link is not None else None)
        def sanitize(name):
            return str(name).replace(' ', '_').replace('-', '_') if name else 'none'
        self.name = f"{sanitize(self.parent_link)}-{sanitize(self.child_link)}_{curr_joint.joint_type}"
        log_message(f'[PROCESSING JOINT] {self.name}')
        self.freecad_joint = curr_joint
        self.joint_type = curr_joint.joint_type
        self.urdf_transform = get_joint_transform(prev_joint, curr_joint)
        if self.joint_type == "revolute":
            self.axis = get_joint_axis(prev_joint, curr_joint)
        else:
            self.axis = None

    def write(self, f, parent_link, child_link):
        joint_type = self.joint_type
        parent = parent_link
        child = child_link
        semantic_name = self.name
        joint_xyz, joint_rpy = format_placement(self.urdf_transform, scale=SCALE)
        if joint_type == "fixed":
            self._write_fixed(f, semantic_name, parent, child, joint_xyz, joint_rpy)
        elif joint_type == "revolute":
            axis_vec = self.axis if self.joint_type == "revolute" else None
            if parent_link != parent and axis_vec is not None:
                axis_vec = App.Vector(-axis_vec.x, -axis_vec.y, -axis_vec.z)
            self._write_revolute(f, semantic_name, parent, child, joint_xyz, joint_rpy, axis_vec)
        else:
            raise RuntimeError(f"ERROR: Unsupported joint type: {joint_type}")

    def _write_fixed(self, f, semantic_name, parent, child, joint_xyz, joint_rpy):
        f.write(f'  <joint name="{semantic_name}" type="fixed">\n')
        f.write(f'    <parent link="{parent}"/>\n')
        f.write(f'    <child link="{child}"/>\n')
        f.write(f'    <origin xyz="{joint_xyz}" rpy="{joint_rpy}"/>\n')
        f.write('  </joint>\n')

    def _write_revolute(self, f, semantic_name, parent, child, joint_xyz, joint_rpy, axis_vec):
        f.write(f'  <joint name="{semantic_name}" type="revolute">\n')
        f.write(f'    <parent link="{parent}"/>\n')
        f.write(f'    <child link="{child}"/>\n')
        f.write(f'    <origin xyz="{joint_xyz}" rpy="{joint_rpy}"/>\n')
        if self.joint_type == "revolute" and axis_vec is not None:
            f.write(f'    <axis xyz="{axis_vec.x} {axis_vec.y} {axis_vec.z}"/>\n')
        lower = getattr(self.freecad_joint.joint, "LowerLimit", -3.14)
        upper = getattr(self.freecad_joint.joint, "UpperLimit", 3.14)
        effort = getattr(self.freecad_joint.joint, "Effort", 1)
        velocity = getattr(self.freecad_joint.joint, "Velocity", 1)
        f.write(f'    <limit lower="{lower}" upper="{upper}" effort="{effort}" velocity="{velocity}"/>\n')
        f.write('  </joint>\n')

    def __str__(self):
        def clean(val):
            try:
                fval = float(val)
                return 0.0 if abs(fval) < 1e-8 else fval
            except Exception:
                return val
        xyz, rpy = format_placement(self.urdf_transform, scale=1.0)
        xyz_clean = ' '.join(str(clean(x)) for x in xyz.split())
        rpy_clean = ' '.join(str(clean(x)) for x in rpy.split())
        if self.joint_type == "revolute" and self.axis is not None:
            axis_vec = self.axis
            axis_clean = f"{clean(axis_vec.x)} {clean(axis_vec.y)} {clean(axis_vec.z)}"
        else:
            axis_clean = "None"
        return (f"""URDFJoint(
  name={self.name},
  parent_link={self.parent_link},
  child_link={self.child_link},
  xyz=\"{xyz_clean}\"
  rpy=\"{rpy_clean}\"
  axis=\"{axis_clean}\"
)""")

# --- New object graph initialization and traversal ---
def find_grounded_joint_and_link(joint_objs, links):
    for joint_obj in joint_objs:
        if getattr(joint_obj, "Reference1", None) is None and getattr(joint_obj, "Reference2", None) is None:
            obj_to_ground = getattr(joint_obj, "ObjectToGround", None)
            grounded_name = getattr(obj_to_ground, "Name", None)
            if grounded_name in links:
                return joint_obj, links[grounded_name]
    return None, None

def build_assembly_tree(robot_parts, joint_objs):
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
    visited_joints = set()
    def build_tree(link):
        if link.name in visited_links:
            return
        visited_links.add(link.name)
        link.joints = []
        for joint_obj in joint_objs:

            ref1 = get_link_name_from_reference(getattr(joint_obj, 'Reference1', None))
            ref2 = get_link_name_from_reference(getattr(joint_obj, 'Reference2', None))
            # Only add the joint to the current link if the link is the parent/root in this recursion
            if link.name == ref1:
                other_link_name = ref2
            elif link.name == ref2:
                other_link_name = ref1
            else:
                continue
            if other_link_name and other_link_name in links:
                joint_name = getattr(joint_obj, 'Name', '<no name>')
                joint_type = getattr(joint_obj, 'JointType', '<none>')
                # log_message(f"[TREE BUILD] Link '{link.name}' found joint '{joint_name}' (type: {joint_type}) to '{other_link_name}'")
                joint = FreeCADJoint(joint_obj, link.name, child_link=links[other_link_name], parent_link=link)
                joint.child_link = links[other_link_name]

                # SKIP JOINTS ALREADY VISITED - these are the joints that are already in the tree and are (grand)parents of this one
                joint_id = id(joint_obj)
                if joint_id in visited_joints:
                    # log_message(f"[TREE BUILD] Skipping already visited joint '{joint_name}'")
                    continue
                visited_joints.add(joint_id)

                # ADD JOINT TO LINK AND RECURSE
                # log_message(f"[TREE BUILD] Adding joint '{joint_name}' to link '{link.name}'")
                # log_message(f"[TREE BUILD] Setting child_link of joint '{joint_name}' to '{other_link_name}'")
                link.joints.append(joint)
                build_tree(joint.child_link)
    build_tree(root_link)
    return root_link, links, []

def create_urdf(f, link, parent_joint=None, is_root=False, parent_name=None, visited_links=None, visited_joints=None):
    if visited_links is None:
        visited_links = set()
    if visited_joints is None:
        visited_joints = set()
    # If we've already fully processed this link, skip
    if link.name in visited_links:
        log_message(f'[CYCLE] Skipping already visited link: {link.name}')
        return
    # Add a newline and log the link being processed
    log_newline()
    log_message(f'[PROCESSING LINK] {link.name}')
    # Print URDFLink state before writing
    urdf_link = URDFLink(link, is_root=is_root, parent_name=parent_name, parent_joint=parent_joint)
    log_message(str(urdf_link))
    urdf_link.write(f)
    for joint in link.joints:
        joint_id = id(joint)
        if joint_id in visited_joints:
            continue
        visited_joints_new = visited_joints.copy()
        visited_joints_new.add(joint_id)
        # Add a newline and log the joint being processed
        log_newline()
        # Create URDFJoint before any alignment or transform logs
        urdf_joint = URDFJoint(parent_joint, joint, parent_link=link.name, child_link=joint.child_link.name)
        log_message(str(urdf_joint))
        urdf_joint.write(f, link.name, joint.child_link.name)
        # Recursively traverse the child link
        create_urdf(f, joint.child_link, parent_joint=joint, is_root=False, parent_name=link.name, visited_links=visited_links.copy(), visited_joints=visited_joints_new)
    # Only mark the link as visited after all joints/children are processed
    visited_links.add(link.name)

# --- Replace old traversal in assemblyToURDF_tree ---
def convert_assembly_to_urdf():
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
            log_message(f"[DEBUG] Found robot part: {obj.Name}, {get_global_placement(obj)}")
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
    # print('Joint names and JointType values:')
    # for joint in joint_objs:
    #     print(f"  {getattr(joint, 'Name', '<no name>')}: JointType = {getattr(joint, 'JointType', '<none>')}")
    # Build the assembly tree
    root_link, links, joints = build_assembly_tree(robot_parts, joint_objs)
    if root_link:
        print_assembly_tree(root_link)
    urdf_file = os.path.join(EXPORT_DIR, "robot.urdf")
    with open(urdf_file, "w") as f:
        f.write(f'<robot name="{ROBOT_NAME}">\n\n')
        if root_link:
            create_urdf(f, root_link, parent_joint=None, is_root=True, parent_name="world")
        f.write('</robot>\n')
    print(f"\nExport complete!\nURDF exported to: {urdf_file}")

def main():
    convert_assembly_to_urdf() 