import FreeCAD as App
import os

from utils_io import ensure_dir
from freecad_helpers import export_mesh, get_inertial, get_link_name_from_reference, get_link_names_from_reference_expanded, get_mesh_offset, get_joint_transform, get_joint_axis, get_global_placement, resolve_object_to_link_names
from utils_math import format_vector, format_placement
from logging_utils import log_message, log_newline

EXPORT_HERE = True
EXPORT_HERE = False

def get_export_dir():
    """Get the export directory based on configuration"""

    if EXPORT_HERE:
        return os.path.join(os.path.dirname(__file__), "exports")

    # Print the active document information
    if DOC:
        print(f"Active document name: {DOC.Name}")
        print(f"Active document file path: {DOC.FileName}")
        print(f"Active document label: {DOC.Label}")
    else:
        print("No active document found")
    
    # Load environment configuration
    env_file = os.path.join(os.path.dirname(__file__), "export_config.env")
    external_dir = None
    
    if os.path.exists(env_file):
        with open(env_file, 'r') as f:
            for line in f:
                line = line.strip()
                if line and not line.startswith('#'):
                    if '=' in line:
                        key, value = line.split('=', 1)
                        if key.strip() == 'EXTERNAL_EXPORT_DIR':
                            external_dir = value.strip()
                            break
    
    # Get external directory from environment, fallback to local exports directory
    if external_dir:
        base_dir = external_dir
        print(f"Using external export directory: {base_dir}")
    else:
        # Fallback to local directory in repo
        base_dir = os.path.join(os.path.dirname(__file__), "exports")
        print(f"Using local export directory: {base_dir}")
    
    # Get the FreeCAD document name without extension
    if not DOC:
        raise RuntimeError("No active FreeCAD document found. Please open a document before running the export.")
    if not DOC.Name:
        raise RuntimeError("Active FreeCAD document has no name. Please save the document before running the export.")
    
    fcdoc_name = DOC.Name.replace('.FCStd', '')  # Remove .FCStd extension if present
    
    # Create export directory path
    export_dir = os.path.join(base_dir, fcdoc_name)
    
    # Check if directory exists, create if it doesn't
    if not os.path.exists(export_dir):
        print(f"Creating export directory: {export_dir}")
        os.makedirs(export_dir, exist_ok=True)
    else:
        print(f"Export directory exists: {export_dir}")
    
    return export_dir

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
MESH_FORMAT = "stl"  # or 'dae'
SCALE = 0.001  # mm → m (set to 0.001 for mm to meters conversion)
PLA_DENSITY = 1240  # kg/m^3
USE_PACKAGE_PREFIX = False  # Set to True for package://, False for absolute path
MANUAL_CHECK = True  # Set to True to print human-readable transform checks

def find_joints_group(assembly):
    if not assembly or not getattr(assembly, "Group", None):
        return None
    for obj in assembly.Group:
        if obj.TypeId == "Assembly::JointGroup" and "Joints" in getattr(obj, "Name", ""):
            return obj
    return None


def collect_all_joints(assembly):
    """Collect joints from main assembly and any subassemblies that have Joints groups."""
    joint_objs = []
    seen_joints = set()
    seen_assemblies = set()

    def collect(obj):
        if obj is None or id(obj) in seen_assemblies:
            return
        seen_assemblies.add(id(obj))
        joints_group = find_joints_group(obj)
        if not joints_group and getattr(obj, "LinkedObject", None):
            joints_group = find_joints_group(obj.LinkedObject)
        if joints_group and joints_group.Group:
            for j in joints_group.Group:
                if id(j) not in seen_joints:
                    seen_joints.add(id(j))
                    joint_objs.append(j)
        if getattr(obj, "Group", None):
            for child in obj.Group:
                if getattr(child, "Group", None):
                    collect(child)
        # AssemblyLink: also process LinkedObject to find joints in the linked assembly
        if getattr(obj, "LinkedObject", None):
            lo = obj.LinkedObject
            if getattr(lo, "Group", None) and getattr(lo, "TypeId", "") in ("Assembly::AssemblyObject", "App::Part", "Part::Feature"):
                collect(lo)

    collect(assembly)
    return joint_objs

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
    link1_candidates = get_link_name_from_reference(reference1)
    link2_candidates = get_link_name_from_reference(reference2)
    if link_name in link1_candidates:
        parent_placement = getattr(joint, "Placement1", None)
        child_placement = getattr(joint, "Placement2", None)
    else:
        parent_placement = getattr(joint, "Placement2", None)
        child_placement = getattr(joint, "Placement1", None)
    return parent_placement, child_placement

# --- Simplified handle_link and handle_joint ---
class FreeCADLink:
    def __init__(self, part):
        self.part = part
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
        self.global_placement = get_global_placement(part)
        # log_message(f"[DEBUG] {self.name} global placement: {self.global_placement}")

class URDFLink:
    def __init__(self, freecad_link, export_dir, is_root=False, parent_name=None, parent_joint=None):
        self.name = freecad_link.name
        self.body = freecad_link.body
        self.mesh_path = export_mesh(self.body, self.name, export_dir)
        self.inertial = get_inertial(self.body, self.name)
        self.is_root = is_root
        self.parent_name = parent_name
        if is_root:
            self.xyz, self.rpy = "0 0 0", "0 0 0"
        else:
            assert parent_joint is not None, f"Non-root link {self.name} requires a parent_joint for alignment"
            assert hasattr(parent_joint, 'from_parent_origin') and parent_joint.from_parent_origin is not None, f"parent_joint for {self.name} missing from_parent_origin"
            assert hasattr(parent_joint, 'from_child_origin') and parent_joint.from_child_origin is not None, f"parent_joint for {self.name} missing from_child_origin"
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
    def __init__(self, joint, link_name, child_link=None, parent_link=None, ref1_candidates=None, ref2_candidates=None):
        self.joint = joint
        self.link_name = link_name
        reference1 = getattr(joint, "Reference1", None)
        reference2 = getattr(joint, "Reference2", None)
        link1_candidates = ref1_candidates if ref1_candidates is not None else get_link_name_from_reference(reference1)
        link2_candidates = ref2_candidates if ref2_candidates is not None else get_link_name_from_reference(reference2)
        if link_name in link1_candidates:
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
        self.parent_link = parent_link  # Should be a FreeCADLink or None
        # are_joint_z_axes_opposed(parent_link, self, child_link)

class URDFJoint:
    def __init__(self, prev_joint, curr_joint, parent_link=None, child_link=None):
        self.parent_link = parent_link if parent_link is not None else curr_joint.link_name
        self.child_link = child_link if child_link is not None and hasattr(child_link, 'name') else (curr_joint.child_link.name if curr_joint.child_link is not None else None)
        def sanitize(name):
            return str(name).replace(' ', '_').replace('-', '_') if name else 'none'
        self.name = f"{sanitize(self.parent_link)}-{sanitize(self.child_link)}_{curr_joint.joint_type}"
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

    visited_links = set()
    visited_joints = set()

    def build_tree(link):
        if link.name in visited_links:
            return
        visited_links.add(link.name)
        link.joints = []
        for joint_obj in joint_objs:
            ref1 = getattr(joint_obj, 'Reference1', None)
            ref2 = getattr(joint_obj, 'Reference2', None)
            ref1_candidates = get_link_names_from_reference_expanded(ref1, links)
            ref2_candidates = get_link_names_from_reference_expanded(ref2, links)
            if link.name in ref1_candidates:
                other_candidates = ref2_candidates
            elif link.name in ref2_candidates:
                other_candidates = ref1_candidates
            else:
                continue
            other_link_name = next((r for r in other_candidates if r in links and r != link.name), None)
            if other_link_name and other_link_name not in visited_links:  # skip cycle-forming joints
                joint = FreeCADJoint(
                    joint_obj, link.name,
                    child_link=links[other_link_name], parent_link=link,
                    ref1_candidates=ref1_candidates, ref2_candidates=ref2_candidates
                )
                joint.child_link = links[other_link_name]
                joint_id = id(joint_obj)
                if joint_id in visited_joints:
                    continue
                visited_joints.add(joint_id)
                link.joints.append(joint)
                build_tree(joint.child_link)

    # 1. Identify the root joint (grounded)
    root_joint = None
    for joint_obj in joint_objs:
        name = getattr(joint_obj, 'Name', '').lower()
        joint_type = getattr(joint_obj, 'JointType', '').lower()
        if 'ground' in name or 'ground' in joint_type:
            root_joint = joint_obj
            break
    if not root_joint:
        raise RuntimeError(
            "ERROR: No grounded joint found. Assembly must have a joint with 'ground' in its Name or JointType "
            "(e.g. 'GroundedJoint' or type 'Ground') that identifies which link is fixed to the world."
        )
    # 2. Find the single link this joint attaches to (from Reference1, Reference2, or ObjectToGround)
    ref1 = getattr(root_joint, 'Reference1', None)
    ref2 = getattr(root_joint, 'Reference2', None)
    obj_to_ground = getattr(root_joint, 'ObjectToGround', None)
    ref1_candidates = get_link_names_from_reference_expanded(ref1, links)
    ref2_candidates = get_link_names_from_reference_expanded(ref2, links)
    candidates = [r for r in ref1_candidates + ref2_candidates if r in links]
    if obj_to_ground:
        obj_candidates = resolve_object_to_link_names(obj_to_ground, links)
        candidates.extend(obj_candidates)
    candidates = list(dict.fromkeys(c for c in candidates if c in links))
    if len(candidates) == 0:
        msg = (
            f"Grounded joint must identify at least one link, but got 0. "
            f"Link names: {list(links.keys())}. "
            f"Ref1={repr(ref1)[:80]}, Ref2={repr(ref2)[:80]}, "
            f"ObjectToGround={getattr(obj_to_ground, 'Name', obj_to_ground) if obj_to_ground else None} ({type(obj_to_ground).__name__ if obj_to_ground else 'None'})"
        )
        raise RuntimeError(f"ERROR: {msg}")
    if len(candidates) > 1:
        candidates = [candidates[0]]
    root_link = links[candidates[0]]
    log_message(f"[root] {root_link.name}")
    # 3. Recursively build the tree
    build_tree(root_link)
    return root_link, links, []

def create_urdf(f, link, export_dir, parent_joint=None, is_root=False, parent_name=None, visited_links=None, visited_joints=None):
    if visited_links is None:
        visited_links = set()
    if visited_joints is None:
        visited_joints = set()
    if link.name in visited_links:
        log_message(f'[CYCLE] Skipping {link.name}')
        return
    visited_links.add(link.name)  # Mark before recurse so cycles skip
    log_message(f'[LINK] {link.name}')
    urdf_link = URDFLink(link, export_dir, is_root=is_root, parent_name=parent_name, parent_joint=parent_joint)
    urdf_link.write(f)
    for joint in link.joints:
        joint_id = id(joint.joint)  # Dedupe by joint object, not FreeCADJoint
        if joint_id in visited_joints:
            continue
        visited_joints.add(joint_id)
        urdf_joint = URDFJoint(parent_joint, joint, parent_link=link.name, child_link=joint.child_link.name)
        log_message(f'[JOINT] {urdf_joint.name}')
        urdf_joint.write(f, link.name, joint.child_link.name)
        create_urdf(f, joint.child_link, export_dir, parent_joint=joint, is_root=False, parent_name=link.name, visited_links=visited_links, visited_joints=visited_joints)

# --- Replace old traversal in assemblyToURDF_tree ---
def convert_assembly_to_urdf(export_dir):
    print('running (assembly tree traversal)' + '\n'*5)
    ensure_dir(export_dir)
    ensure_dir(os.path.join(export_dir, "meshes"))
    assembly = [obj for obj in DOC.Objects if obj.TypeId == "Assembly::AssemblyObject"]
    try:
        assembly = assembly[0]
    except:
        print('no assembly found')
        return
    robot_parts = []
    seen_parts = set()  # id(obj) to avoid duplicates

    def collect_parts(obj, depth=0):
        if obj is None:
            return
        name = getattr(obj, "Name", None)
        type_id = getattr(obj, "TypeId", None)
        linked = getattr(obj, "LinkedObject", None)
        if linked:
            linked_type = getattr(linked, "TypeId", None)
            if linked_type == "PartDesign::Body":
                if id(obj) not in seen_parts:
                    seen_parts.add(id(obj))
                    robot_parts.append(obj)
            elif getattr(linked, "Group", None):
                for child in linked.Group:
                    collect_parts(child, depth + 1)
        elif type_id == "PartDesign::Body":
            if id(obj) not in seen_parts:
                seen_parts.add(id(obj))
                robot_parts.append(obj)
        # Recurse into our own Group (for subassemblies) - but NOT PartDesign::Body (features like Pad, Sketch)
        if type_id not in ("PartDesign::Body",) and getattr(obj, "Group", None):
            for child in obj.Group:
                collect_parts(child, depth + 1)

    for obj in assembly.Group:
        collect_parts(obj)
    log_message(f"[export] {len(robot_parts)} parts: {', '.join(getattr(p, 'Name', '?') for p in robot_parts)}")
    joint_objs = collect_all_joints(assembly) if assembly else []
    log_message(f"[export] {len(joint_objs)} joints")
    # print('Joint names and JointType values:')
    # for joint in joint_objs:
    #     print(f"  {getattr(joint, 'Name', '<no name>')}: JointType = {getattr(joint, 'JointType', '<none>')}")
    # Build the assembly tree
    root_link, links, joints = build_assembly_tree(robot_parts, joint_objs)
    if root_link:
        print_assembly_tree(root_link)
    urdf_file = os.path.join(export_dir, "robot.urdf")
    with open(urdf_file, "w") as f:
        f.write(f'<robot name="{ROBOT_NAME}">\n\n')
        if root_link:
            create_urdf(f, root_link, export_dir, parent_joint=None, is_root=True, parent_name="world")
        f.write('</robot>\n')
    print(f"\nExport complete!\nURDF exported to: {urdf_file}")
    # Print URDF contents for debugging
    print("\n--- URDF FILE CONTENTS ---")
    with open(urdf_file, "r") as f:
        print(f.read())
    print("--- END URDF ---")

def main():
    # Get export directory (includes all logging)
    export_dir = get_export_dir()
    
    convert_assembly_to_urdf(export_dir) 