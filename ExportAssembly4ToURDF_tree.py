"""
Assembly4 to URDF export. Flattened link model: all links from all assemblies
are collected into a single dict. Grounded assembly/link is resolved; refs
pointing to assemblies resolve to the grounded link of that assembly.
"""
import FreeCAD as App
import os

from utils_io import ensure_dir
from freecad_helpers import (
    export_mesh, get_inertial, get_link_names_from_reference_expanded,
    get_mesh_offset, get_joint_transform, get_joint_axis, get_global_placement,
    resolve_object_to_link_names,
)
from utils_math import format_vector, format_placement

DOC = App.ActiveDocument
ROBOT_NAME = "my_robot"
SCALE = 0.001  # mm → m


def get_export_dir():
    """Get the export directory from config or environment."""
    if not DOC:
        raise RuntimeError("No active FreeCAD document")
    env_file = os.path.join(os.path.dirname(__file__), "export_config.env")
    external_dir = None
    if os.path.exists(env_file):
        with open(env_file) as f:
            for line in f:
                line = line.strip()
                if line and "=" in line and not line.startswith("#"):
                    k, v = line.split("=", 1)
                    if k.strip() == "EXTERNAL_EXPORT_DIR":
                        external_dir = v.strip()
                        break
    base = external_dir or os.path.join(os.path.dirname(__file__), "exports")
    if external_dir:
        print(f"Using external export directory: {base}")
    fcdoc = DOC.Name.replace(".FCStd", "")
    export_dir = os.path.join(base, fcdoc)
    os.makedirs(export_dir, exist_ok=True)
    print(f"Export directory exists: {export_dir}")
    return export_dir


# --- Collection ---

def _is_body_or_link_to_body(obj):
    """True if obj is a PartDesign::Body or App::Link chain ending in PartDesign::Body."""
    t = getattr(obj, "TypeId", "")
    lo = getattr(obj, "LinkedObject", None)
    if t == "PartDesign::Body":
        return True, obj
    if t == "App::Link" and lo:
        # Follow chain: Link->Link->Body or Link->Body
        target = lo
        while target and getattr(target, "TypeId", "") == "App::Link":
            target = getattr(target, "LinkedObject", None)
        if target and getattr(target, "TypeId", "") == "PartDesign::Body":
            return True, obj
    return False, None


def collect_links_flattened(assembly):
    """Recursively collect all PartDesign::Body parts (as App::Link) from assembly and subassemblies. Returns flat list."""
    result = []
    seen = set()

    def visit(obj):
        if obj is None:
            return
        if id(obj) in seen:
            return
        seen.add(id(obj))
        ok, part = _is_body_or_link_to_body(obj)
        if ok:
            result.append(part)
            return
        for child in getattr(obj, "Group", []) or []:
            visit(child)
        lo = getattr(obj, "LinkedObject", None)
        if lo:
            visit(lo)
            for c in getattr(lo, "Group", []) or []:
                visit(c)

    for child in getattr(assembly, "Group", []) or []:
        visit(child)
    return result


def find_joints_group(assembly):
    if not assembly or not getattr(assembly, "Group", None):
        return None
    for obj in assembly.Group:
        if getattr(obj, "TypeId", "") == "Assembly::JointGroup" and "Joints" in getattr(obj, "Name", ""):
            return obj
    return None


def collect_joints_with_assembly(assembly):
    """Collect (joint, owning_assembly) from assembly and subassemblies."""
    result = []
    seen_joints = set()
    seen_assemblies = set()

    def collect(obj):
        if obj is None or id(obj) in seen_assemblies:
            return
        seen_assemblies.add(id(obj))
        jg = find_joints_group(obj)
        if not jg and getattr(obj, "LinkedObject", None):
            jg = find_joints_group(obj.LinkedObject)
        if jg and getattr(jg, "Group", None):
            for j in jg.Group:
                if id(j) not in seen_joints:
                    seen_joints.add(id(j))
                    result.append((j, obj))
        for child in getattr(obj, "Group", []) or []:
            if getattr(child, "Group", None):
                collect(child)
        lo = getattr(obj, "LinkedObject", None)
        if lo and getattr(lo, "Group", None) and getattr(lo, "TypeId", "") in ("Assembly::AssemblyObject", "App::Part", "Part::Feature"):
            collect(lo)

    collect(assembly)
    return result


def build_assembly_grounded_map(joint_list, links_dict):
    """
    For each assembly that has a grounded joint, map assembly -> grounded link name.
    When ObjectToGround is an assembly, recurse to find grounded link within it.
    Returns {id(assembly): link_name}.
    """
    world_names = {"origin", "origin001", "world", "ground"}
    grounded = {}

    def resolve_obj_to_link(obj):
        if obj is None:
            return None
        if hasattr(obj, "Name") and obj.Name in links_dict:
            return obj.Name
        resolved = resolve_object_to_link_names(obj, links_dict)
        candidates = [r for r in resolved if r.lower() not in world_names and r in links_dict]
        return candidates[0] if candidates else None

    def get_grounded_for_assembly(assem):
        """Find grounded link for assembly by its grounded joint."""
        for j, a in joint_list:
            if a is not assem and getattr(assem, "LinkedObject", None) is not a:
                continue
            n = getattr(j, "Name", "").lower()
            t = getattr(j, "JointType", "").lower()
            if "ground" not in n and "ground" not in t:
                continue
            otg = getattr(j, "ObjectToGround", None)
            if not otg:
                return None
            ln = resolve_obj_to_link(otg)
            if ln:
                return ln
            # otg is assembly - recurse
            return get_grounded_for_assembly(otg)

    for joint_obj, assembly in joint_list:
        name = getattr(joint_obj, "Name", "").lower()
        jtype = getattr(joint_obj, "JointType", "").lower()
        if "ground" not in name and "ground" not in jtype:
            continue
        obj_to_ground = getattr(joint_obj, "ObjectToGround", None)
        if not obj_to_ground:
            continue
        resolved = resolve_object_to_link_names(obj_to_ground, links_dict)
        candidates = [r for r in resolved if r.lower() not in world_names and r in links_dict]
        if len(candidates) == 1:
            grounded[id(assembly)] = candidates[0]
        elif len(candidates) > 1:
            ln = get_grounded_for_assembly(obj_to_ground)
            if ln:
                grounded[id(obj_to_ground)] = ln
                grounded[id(assembly)] = ln
        else:
            ln = get_grounded_for_assembly(obj_to_ground)
            if ln:
                grounded[id(obj_to_ground)] = ln
                grounded[id(assembly)] = ln

    return grounded


# --- Link and joint classes ---

class FreeCADLink:
    def __init__(self, part):
        self.part = part
        self.name = part.Name
        lo = getattr(part, "LinkedObject", None)
        if part.TypeId == "App::Link" and lo:
            # Follow Link->Link->Body chain
            target = lo
            while target and getattr(target, "TypeId", "") == "App::Link":
                target = getattr(target, "LinkedObject", None)
            self.body = target if getattr(target, "TypeId", "") == "PartDesign::Body" else lo
        else:
            self.body = part
        if not self.body or getattr(self.body, "TypeId", "") != "PartDesign::Body":
            raise RuntimeError(f"{self.name} is not a PartDesign::Body")
        shape = getattr(self.body, "Shape", None)
        if not shape or shape.isNull():
            raise RuntimeError(f"{self.name} has no valid shape")
        self.joints = []
        self.global_placement = get_global_placement(part)


class FreeCADJoint:
    def __init__(self, joint_obj, parent_link_name, child_link, links_dict, assembly_grounded_map):
        self.joint = joint_obj
        self.link_name = parent_link_name
        self.child_link = child_link
        self.parent_link = links_dict[parent_link_name]
        ref1 = getattr(joint_obj, "Reference1", None)
        ref2 = getattr(joint_obj, "Reference2", None)
        c1 = get_link_names_from_reference_expanded(ref1, links_dict, assembly_grounded_map)
        c2 = get_link_names_from_reference_expanded(ref2, links_dict, assembly_grounded_map)
        if parent_link_name in c1:
            self.from_parent_origin = getattr(joint_obj, "Placement1", None)
            self.from_child_origin = getattr(joint_obj, "Placement2", None)
        else:
            self.from_parent_origin = getattr(joint_obj, "Placement2", None)
            self.from_child_origin = getattr(joint_obj, "Placement1", None)
        self.joint_type = getattr(joint_obj, "JointType", "revolute").lower()
        self.name = getattr(joint_obj, "Name", None)


class URDFLink:
    def __init__(self, freecad_link, export_dir, is_root=False, parent_joint=None):
        self.name = freecad_link.name
        self.body = freecad_link.body
        self.mesh_path = export_mesh(self.body, self.name, export_dir)
        self.inertial = get_inertial(self.body, self.name)
        if is_root:
            self.xyz, self.rpy = "0 0 0", "0 0 0"
        else:
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


class URDFJoint:
    def __init__(self, prev_joint, curr_joint):
        self.freecad_joint = curr_joint
        self.parent_link = curr_joint.link_name
        self.child_link = curr_joint.child_link.name
        self.joint_type = curr_joint.joint_type
        self.urdf_transform = get_joint_transform(prev_joint, curr_joint)
        self.axis = get_joint_axis(prev_joint, curr_joint) if curr_joint.joint_type == "revolute" else None
        self.name = f"{self.parent_link}-{self.child_link}_{curr_joint.joint_type}".replace(" ", "_").replace("-", "_")

    def write(self, f):
        xyz, rpy = format_placement(self.urdf_transform, scale=SCALE)
        f.write(f'  <joint name="{self.name}" type="{self.joint_type}">\n')
        f.write(f'    <parent link="{self.parent_link}"/>\n')
        f.write(f'    <child link="{self.child_link}"/>\n')
        f.write(f'    <origin xyz="{xyz}" rpy="{rpy}"/>\n')
        if self.joint_type == "revolute" and self.axis:
            f.write(f'    <axis xyz="{self.axis.x} {self.axis.y} {self.axis.z}"/>\n')
            lo = getattr(self.freecad_joint.joint, "LowerLimit", -3.14)
            up = getattr(self.freecad_joint.joint, "UpperLimit", 3.14)
            f.write(f'    <limit lower="{lo}" upper="{up}" effort="1" velocity="1"/>\n')
        f.write('  </joint>\n')


# --- Tree building ---

def _get_grounded_link_for_assembly(joint_list, assem, links_dict):
    """Find grounded link name for an assembly by locating its grounded joint."""
    world = {"origin", "origin001", "world", "ground"}
    for j, a in joint_list:
        if a is not assem and getattr(assem, "LinkedObject", None) is not a:
            continue
        if "ground" not in getattr(j, "Name", "").lower():
            continue
        otg = getattr(j, "ObjectToGround", None)
        if not otg:
            return None
        if hasattr(otg, "Name") and otg.Name in links_dict:
            return otg.Name
        r = resolve_object_to_link_names(otg, links_dict)
        cands = [x for x in r if x.lower() not in world and x in links_dict]
        return cands[0] if cands else None
    return None


def find_root_link(joint_list, links_dict, assembly_grounded_map):
    """Find the grounded link from the grounded joint(s)."""
    world_names = {"origin", "origin001", "world", "ground"}

    def resolve_to_link(obj):
        if obj is None:
            return None
        if hasattr(obj, "Name") and obj.Name in links_dict:
            return obj.Name
        r = resolve_object_to_link_names(obj, links_dict)
        cands = [x for x in r if x.lower() not in world_names and x in links_dict]
        return cands[0] if cands else None

    for joint_obj, assembly in joint_list:
        name = getattr(joint_obj, "Name", "").lower()
        jtype = getattr(joint_obj, "JointType", "").lower()
        if "ground" not in name and "ground" not in jtype:
            continue
        obj_to_ground = getattr(joint_obj, "ObjectToGround", None)
        if not obj_to_ground:
            c1 = get_link_names_from_reference_expanded(
                getattr(joint_obj, "Reference1", None), links_dict, assembly_grounded_map
            )
            c2 = get_link_names_from_reference_expanded(
                getattr(joint_obj, "Reference2", None), links_dict, assembly_grounded_map
            )
            candidates = [r for r in (c1 + c2) if r.lower() not in world_names and r in links_dict]
            if candidates:
                return links_dict[candidates[0]]
            continue
        resolved = resolve_object_to_link_names(obj_to_ground, links_dict)
        candidates = [r for r in resolved if r.lower() not in world_names and r in links_dict]
        if len(candidates) == 1:
            return links_dict[candidates[0]]
        if len(candidates) > 1 and id(obj_to_ground) in assembly_grounded_map:
            return links_dict[assembly_grounded_map[id(obj_to_ground)]]
        if len(candidates) > 1:
            ln = _get_grounded_link_for_assembly(joint_list, obj_to_ground, links_dict)
            if ln:
                return links_dict[ln]
        link_name = resolve_to_link(obj_to_ground)
        if link_name:
            return links_dict[link_name]
        if id(obj_to_ground) in assembly_grounded_map:
            return links_dict[assembly_grounded_map[id(obj_to_ground)]]
        otg = obj_to_ground
        while otg:
            ln = _get_grounded_link_for_assembly(joint_list, otg, links_dict)
            if ln:
                return links_dict[ln]
            if id(otg) in assembly_grounded_map:
                return links_dict[assembly_grounded_map[id(otg)]]
            otg = getattr(otg, "LinkedObject", None)

    # Last resort: use first link from assembly_grounded_map (a sub-assembly's grounded link)
    if assembly_grounded_map:
        for aid, link_name in assembly_grounded_map.items():
            if link_name in links_dict:
                return links_dict[link_name]
    raise RuntimeError("No grounded joint found. Assembly must have a GroundedJoint with ObjectToGround set.")


def build_tree(root_link, links_dict, joint_list, assembly_grounded_map):
    """Attach .joints to each link via DFS from root."""
    visited = set()
    used_joints = set()

    def visit(link):
        if link.name in visited:
            return
        visited.add(link.name)
        pending = []
        for joint_obj, _ in joint_list:
            if id(joint_obj) in used_joints:
                continue
            ref1 = getattr(joint_obj, "Reference1", None)
            ref2 = getattr(joint_obj, "Reference2", None)
            c1 = get_link_names_from_reference_expanded(ref1, links_dict, assembly_grounded_map)
            c2 = get_link_names_from_reference_expanded(ref2, links_dict, assembly_grounded_map)
            if link.name not in c1 and link.name not in c2:
                continue
            other = c2 if link.name in c1 else c1
            other_name = next((r for r in other if r in links_dict and r != link.name), None)
            if not other_name or other_name in visited:
                continue
            fc_joint = FreeCADJoint(joint_obj, link.name, links_dict[other_name], links_dict, assembly_grounded_map)
            pending.append(fc_joint)
            used_joints.add(id(joint_obj))
        link.joints = pending
        for j in link.joints:
            visit(j.child_link)

    visit(root_link)
    return root_link


def create_urdf(f, link, export_dir, parent_joint=None, is_root=False, visited_links=None, visited_joints=None):
    visited_links = visited_links or set()
    visited_joints = visited_joints or set()
    if link.name in visited_links:
        return
    visited_links.add(link.name)
    urdf_link = URDFLink(link, export_dir, is_root=is_root, parent_joint=parent_joint)
    urdf_link.write(f)
    for j in link.joints:
        if id(j.joint) in visited_joints:
            continue
        visited_joints.add(id(j.joint))
        urdf_joint = URDFJoint(parent_joint, j)
        urdf_joint.write(f)
        create_urdf(f, j.child_link, export_dir, parent_joint=j, is_root=False, visited_links=visited_links, visited_joints=visited_joints)


# --- Main ---

def convert_assembly_to_urdf(export_dir):
    print("Exporting...")
    ensure_dir(export_dir)
    ensure_dir(os.path.join(export_dir, "meshes"))

    assembly = None
    for obj in getattr(DOC, "Objects", []) or []:
        if getattr(obj, "TypeId", "") == "Assembly::AssemblyObject":
            grp = getattr(obj, "Group", []) or []
            names = [getattr(c, "Name", "") for c in grp]
            if "Core Assembly" in names or "Core_Assembly" in names:
                assembly = obj
                break
    if not assembly:
        assembly = next((o for o in (getattr(DOC, "Objects", []) or []) if getattr(o, "TypeId", "") == "Assembly::AssemblyObject"), None)
    if not assembly:
        print("No assembly found")
        return

    parts = collect_links_flattened(assembly)
    joint_list = collect_joints_with_assembly(assembly)
    links_dict = {p.Name: FreeCADLink(p) for p in parts}
    assembly_grounded_map = build_assembly_grounded_map(joint_list, links_dict)

    print(f"Links: {len(links_dict)}, Joints: {len(joint_list)}")

    root_link = find_root_link(joint_list, links_dict, assembly_grounded_map)
    print(f"[root] {root_link.name}")

    build_tree(root_link, links_dict, joint_list, assembly_grounded_map)

    urdf_path = os.path.join(export_dir, "robot.urdf")
    with open(urdf_path, "w") as f:
        f.write(f'<robot name="{ROBOT_NAME}">\n\n')
        create_urdf(f, root_link, export_dir, parent_joint=None, is_root=True)
        f.write('</robot>\n')

    print(f"Export complete: {urdf_path}")
    print("\n--- URDF FILE CONTENTS ---")
    with open(urdf_path) as f:
        print(f.read())
    print("--- END URDF ---")


def main():
    export_dir = get_export_dir()
    convert_assembly_to_urdf(export_dir)
