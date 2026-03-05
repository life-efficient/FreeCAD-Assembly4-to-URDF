import FreeCAD as App
import Mesh
import os

# --- Logging helpers (moved from ExportAssembly4ToURDF_tree.py) ---
# Remove log_message and log_newline from this file and import from logging_utils
from logging_utils import log_message, log_newline, log_joint_transform

MESH_FORMAT = "stl"
PLA_DENSITY = 1240


def get_link_name_from_reference(ref):
    """Extract the link name from a FreeCAD joint reference tuple.
    Returns a list of possible names to try (for subassembly refs like 'SubAssembly.BodyName').
    Handles string paths, object references, and (Object, SubElement) tuples."""
    if not ref or len(ref) < 2 or not ref[1]:
        return []
    first = ref[1][0]
    candidates = []
    obj = first
    if isinstance(first, (list, tuple)) and len(first) > 0:
        obj = first[0]  # (Object, SubElement) -> Object
    if hasattr(obj, "Name"):
        candidates.append(obj.Name)
        if getattr(obj, "LinkedObject", None):
            candidates.append(obj.LinkedObject.Name)
    if isinstance(first, str):
        for sep in (".", "#"):
            parts = first.split(sep)
            if parts:
                candidates.append(parts[0])
            if len(parts) > 1:
                candidates.append(parts[-1])
        if first not in candidates:
            candidates.append(first)
    return [c for c in candidates if c]


def get_link_name_from_reference_single(ref):
    """Return first candidate or None (for backward compatibility)."""
    candidates = get_link_name_from_reference(ref)
    return candidates[0] if candidates else None


def _get_obj_from_ref(ref):
    """Extract the document object from an Assembly4 reference tuple."""
    if not ref or len(ref) < 2 or not ref[1]:
        return None
    first = ref[1][0]
    if isinstance(first, (list, tuple)) and first:
        return first[0]
    if hasattr(first, "Name"):
        return first
    if isinstance(first, str):
        # e.g. "Shoulder_V001.Edge97" or "Rack.LCS" - part before dot is link/part name
        doc = ref[0].Document if ref and hasattr(ref[0], "Document") else None
        if doc:
            name = first.split(".")[0].split("#")[0]
            return doc.getObject(name) if name else None
    return None


def _get_support_part(obj):
    """Follow Support to get the Part/Body this LCS (or similar) is attached to.
    Returns the object that owns the geometry, or obj if no Support."""
    if not obj or not hasattr(obj, "Support") or not obj.Support:
        return obj
    sup = obj.Support
    if isinstance(sup, (list, tuple)) and sup:
        s = sup[0]
    else:
        s = sup
    ref_obj = s.Object if hasattr(s, "Object") else (s[0] if isinstance(s, (list, tuple)) and s else None)
    if ref_obj:
        return ref_obj
    return obj


def get_link_names_from_reference_precise(ref, links_dict):
    """
    Resolve a reference to the EXACT link(s) it attaches to.
    - Direct match: ref points to a link name in links_dict -> return that.
    - LCS: ref points to LCS with Support -> follow to the Part, match to link.
    - Container (subassembly): only expand when ref points to container; return links whose
      part has the ref obj as ancestor (contained-in relationship).
    Returns [] if no precise match.
    """
    if not ref:
        return []
    obj = _get_obj_from_ref(ref)
    if obj is None:
        return []

    # Direct match: obj.Name is a link
    if hasattr(obj, "Name") and obj.Name in links_dict:
        return [obj.Name]

    # LCS or similar: follow Support to get the part the ref is attached to
    part_obj = _get_support_part(obj)
    if part_obj and part_obj is not obj:
        if hasattr(part_obj, "Name") and part_obj.Name in links_dict:
            return [part_obj.Name]
        # part_obj might be a Body; links_dict keys are Link names. Check link.part and link.body.
        for name, link in links_dict.items():
            if getattr(link, "part", None) is part_obj or getattr(link, "body", None) is part_obj:
                return [name]

    # Ref points to container (e.g. Core_Assembly): resolve to links contained in it
    resolved = resolve_object_to_link_names(obj, links_dict)
    return list(resolved) if resolved else []


def get_link_names_from_reference_expanded(ref, links_dict, assembly_grounded_map=None):
    """
    Resolve a joint reference to link names.
    - Link/LCS on link: return that link.
    - Assembly: return grounded link of that assembly (from assembly_grounded_map).
    - LCS on assembly: same as assembly.
    assembly_grounded_map: {id(assembly): link_name} - grounded link per assembly.
    """
    if not ref:
        return []
    obj = _get_obj_from_ref(ref)
    if obj is None:
        return []

    # Direct link match
    if hasattr(obj, "Name") and obj.Name in links_dict:
        return [obj.Name]

    # LCS: follow Support to get the part it's attached to
    part_obj = _get_support_part(obj)
    if part_obj and part_obj is not obj:
        if hasattr(part_obj, "Name") and part_obj.Name in links_dict:
            return [part_obj.Name]
        for name, link in links_dict.items():
            if getattr(link, "part", None) is part_obj or getattr(link, "body", None) is part_obj:
                return [name]

    # Assembly or LCS on assembly: use grounded link
    if assembly_grounded_map:
        target = part_obj if part_obj is not obj else obj
        if target and id(target) in assembly_grounded_map:
            name = assembly_grounded_map[id(target)]
            if name in links_dict:
                return [name]

    # Container: resolve to contained links (resolve_object_to_link_names)
    resolved = resolve_object_to_link_names(obj, links_dict)
    return list(resolved) if resolved else []


def get_top_level_container(obj, assembly):
    """Return the direct child of assembly.Group that contains obj (the 'subassembly').
    Used to order traversal: internal links (same container) before external.
    Tries getParentGeoFeatureGroup first, then falls back to Group membership."""
    if not obj or not assembly or not getattr(assembly, "Group", None):
        return None
    # Method 1: walk parent chain via getParentGeoFeatureGroup
    for child in assembly.Group:
        if _has_ancestor(obj, child):
            return id(child)
    # Method 2: Assembly4/App::Link may not use getParentGeoFeatureGroup; search Group tree
    for child in assembly.Group:
        if _obj_in_group_tree(obj, child):
            return id(child)
    return None


def _obj_in_group_tree(obj, container):
    """True if obj is container itself or is in container's Group (recursively)."""
    if obj is None or container is None:
        return False
    if obj is container:
        return True
    grp = getattr(container, "Group", None)
    if not grp:
        return False
    for c in grp:
        if obj is c or _obj_in_group_tree(obj, c):
            return True
    return False


def _has_ancestor(child_obj, ancestor_obj):
    """Check if ancestor_obj is in the parent chain of child_obj (Group or GeoFeatureGroup)."""
    if child_obj is None or ancestor_obj is None:
        return False
    p = child_obj
    seen = set()
    while p:
        if id(p) in seen:
            break
        seen.add(id(p))
        if p is ancestor_obj:
            return True
        p = getattr(p, "getParentGeoFeatureGroup", lambda: None)()
    return False


def resolve_object_to_link_names(obj, links_dict):
    """Resolve an object (e.g. ObjectToGround) to matching link names.
    Handles App::Part, App::Link, nested Groups - traverses to find bodies/links.
    Matches by: (1) name in links_dict, (2) object identity with link.body, (3) link.part.
    Also matches by (doc, name) for cross-document links.
    links_dict: {link_name: FreeCADLink}"""
    if obj is None:
        return []
    obj_name = getattr(obj, "Name", None)
    found = set()
    visited = set()
    link_names = set(links_dict.keys())
    body_to_name = {id(link.body): name for name, link in links_dict.items() if link.body}
    part_to_name = {id(link.part): name for name, link in links_dict.items() if getattr(link, "part", None)}
    docname_name_to_link = {}
    for name, link in links_dict.items():
        for o in (getattr(link, "part", None), getattr(link, "body", None)):
            if o and hasattr(o, "Document") and hasattr(o, "Name"):
                doc = getattr(o, "Document", None)
                doc_name = getattr(doc, "Name", None) if doc else None
                if doc_name is not None and o.Name:
                    docname_name_to_link[(doc_name, o.Name)] = name

    def collect(o, depth=0):
        if o is None or id(o) in visited:
            return
        visited.add(id(o))
        name = getattr(o, "Name", None)
        type_id = getattr(o, "TypeId", type(o).__name__)
        match_reason = None
        if name and name in link_names:
            found.add(name)
            match_reason = "name"
        if id(o) in body_to_name:
            found.add(body_to_name[id(o)])
            match_reason = "body_id"
        if id(o) in part_to_name:
            found.add(part_to_name[id(o)])
            match_reason = "part_id"
        doc_name = getattr(getattr(o, "Document", None), "Name", None)
        if doc_name and name and (doc_name, name) in docname_name_to_link:
            found.add(docname_name_to_link[(doc_name, name)])
            match_reason = "doc_name"
        # Assembly4 LCS: get parent part from Support (can be (Object, SubElement) or object with .Object)
        if hasattr(o, "Support") and o.Support:
            for s in (o.Support if isinstance(o.Support, (list, tuple)) else [o.Support]):
                ref = s.Object if hasattr(s, "Object") else (s[0] if isinstance(s, (list, tuple)) and s else None)
                if ref:
                    collect(ref, depth + 1)
        lo = getattr(o, "LinkedObject", None)
        if lo:
            collect(lo, depth + 1)
        lo_type = getattr(lo, "TypeId", None) if lo else None
        skip_group = type_id == "PartDesign::Body" or (type_id == "App::Link" and lo_type == "PartDesign::Body")
        grp = getattr(o, "Group", None)
        if grp and not skip_group:
            for child in grp:
                collect(child, depth + 1)

    collect(obj)
    return list(found)

def export_mesh(body, name, export_dir):
    """Export a FreeCAD body as a mesh and return the relative mesh path."""
    mesh_path = os.path.join(export_dir, "meshes", f"{name}.{MESH_FORMAT}")
    
    # Import the scale factor
    import ExportAssembly4ToURDF_tree
    scale_factor = ExportAssembly4ToURDF_tree.SCALE
    
    # Create a scaled transform matrix
    scale_matrix = App.Matrix(
        scale_factor, 0, 0, 0,
        0, scale_factor, 0, 0,
        0, 0, scale_factor, 0,
        0, 0, 0, 1
    )
    
    # Apply scaling to the shape
    scaled_shape = body.Shape.transformGeometry(scale_matrix)
    
    # Create a temporary document object to hold the scaled shape
    temp_doc = App.newDocument("TempMeshExport")
    temp_part = temp_doc.addObject("Part::Feature", "ScaledMesh")
    temp_part.Shape = scaled_shape
    
    # Export the scaled mesh
    Mesh.export([temp_part], mesh_path)
    
    # Clean up temporary document
    App.closeDocument("TempMeshExport")
    
    # Return relative path instead of absolute path
    return f"meshes/{name}.{MESH_FORMAT}"

def get_inertial(body, name=None):
    """Calculate inertial properties for a FreeCAD body."""
    if name and "STS3125".lower() in name.lower():
        mass = 0.06
        com = App.Vector(0, 0, 0)
        inertia = {
            "ixx": 1e-5, "ixy": 0.0, "ixz": 0.0,
            "iyy": 1e-5, "iyz": 0.0, "izz": 1e-5,
        }
        return {"mass": mass, "com": com, "inertia": inertia}
    volume_mm3 = body.Shape.Volume
    volume_m3 = volume_mm3 * 1e-9
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


# def get_joint_urdf_transform(parent_placement, child_placement):
#     """
#     Compute the URDF joint origin transform (parent frame to joint frame) as parent_placement * joint_alignment * child_placement.inverse().
#     Returns a FreeCAD.Placement.
#     """
#     # Original formula (no alignment):
#     # return parent_placement.multiply(child_placement.inverse())
#     # New: include joint alignment (rotation-only)
#     joint_alignment = get_joint_alignment(parent_placement, child_placement)
#     return parent_placement.multiply(joint_alignment).multiply(child_placement.inverse())


# def get_joint_axis_in_urdf_frame(joint_placement):
#     """
#     Get the Z axis of the joint's local frame, expressed in the parent link's frame.
#     Returns a tuple (x, y, z).
#     """
#     # The Z axis in the joint's local frame is (0, 0, 1)
#     z_axis_local = App.Vector(0, 0, 1)
#     # Rotate it into the parent frame
#     z_axis_parent = joint_placement.Rotation.multVec(z_axis_local)
#     return (z_axis_parent.x, z_axis_parent.y, z_axis_parent.z) 


# def get_joint_alignment(joint_placement1, joint_placement2):
#     """
#     Compute the rotation-only transform from the parent's joint attachment LCS to the child's joint attachment LCS.
#     Both placements should be in global coordinates.
#     Returns a FreeCAD.Placement representing the rotation-only transform from parent joint LCS to child joint LCS (translation set to zero).
#     """
#     difference = joint_placement1.inverse().multiply(joint_placement2)
#     rotation_difference = difference.Rotation
#     rotation_only_transform = App.Placement(App.Vector(0, 0, 0), rotation_difference)
#     return rotation_only_transform

def clean(val):
    try:
        fval = float(val)
        return 0.0 if abs(fval) < 1e-8 else fval
    except Exception:
        return val

def clean_placement(placement):
    # Returns a string with negligible values set to 0.0 for readability
    pos = placement.Base
    rot = placement.Rotation
    pos_str = f"({clean(pos.x)}, {clean(pos.y)}, {clean(pos.z)})"
    rot_str = f"({clean(rot.Axis.x)}, {clean(rot.Axis.y)}, {clean(rot.Axis.z)}), angle={clean(rot.Angle)}"
    return f"Placement [Pos={pos_str}, Axis/Angle={rot_str}]"

def get_origin_alignment(from_placement, to_placement):
    """Rotation-only alignment from 'from_placement' to 'to_placement'."""
    difference = to_placement.multiply(from_placement.inverse())
    return App.Placement(App.Vector(0, 0, 0), difference.Rotation)


def get_mesh_offset(parent_joint):
    """
    Compute the mesh offset placement for URDF export.
    This version removes the alignment step: just use the inverse of from_child_origin.
    Returns a FreeCAD.Placement.
    """
    # return zero offset
    # return App.Placement(App.Vector(0,0,0), App.Rotation(0,0,0,0))
    # servo points right

    # OPTION 0: Just transform
    # return parent_joint.from_child_origin
    # up instead of across

    # OPTION 1: Just inverse 
    return parent_joint.from_child_origin.inverse()
    # flipped to incorrect side

    alignment = get_origin_alignment(
        parent_joint.from_parent_origin, 
        parent_joint.from_child_origin
    )

    # return alignment

    # return alignment.multiply(parent_joint.from_child_origin)

    # OPTION 2: Align and then inverse (transformations happen from right to left)
    return alignment.multiply(parent_joint.from_child_origin.inverse())
    # this works - positioning the servo correctly
    # to me, this indicates that the order of application is from left to right - firstly align the axes, then move along the transform

    # OPTION 3: Align and then inverse
    # return parent_joint.from_child_origin.multiply(alignment).inverse()
    # seems to work for servo but not for the other joints - likely a coincidence

    # OPTION 4: Inverse then align
    # return alignment.multiply(parent_joint.from_child_origin)
    # servo translated down instead of out -Z by what should be +Y

    # OPTION 5: 
    return parent_joint.from_child_origin.multiply(alignment)
    # servo upside down and rotates about its center

    # OPTION 6: 
    return parent_joint.from_child_origin.inverse().multiply(alignment)

    # from_child_origin = parent_joint.from_child_origin
    # # log_message(f"[DEBUG][mesh_offset] from_child_origin: Placement [Pos=({clean(from_child_origin.Base.x)}, {clean(from_child_origin.Base.y)}, {clean(from_child_origin.Base.z)}), Axis/Angle=({clean(from_child_origin.Rotation.Axis.x)}, {clean(from_child_origin.Rotation.Axis.y)}, {clean(from_child_origin.Rotation.Axis.z)}), angle={clean(from_child_origin.Rotation.Angle)}]")
    # # # --- Old implementation with alignment ---
    # from_parent_origin = parent_joint.from_parent_origin
    # log_message(f"[DEBUG][mesh_offset] from_parent_origin: Placement [Pos=({clean(from_parent_origin.Base.x)}, {clean(from_parent_origin.Base.y)}, {clean(from_parent_origin.Base.z)}), Axis/Angle=({clean(from_parent_origin.Rotation.Axis.x)}, {clean(from_parent_origin.Rotation.Axis.y)}, {clean(from_parent_origin.Rotation.Axis.z)}), angle={clean(from_parent_origin.Rotation.Angle)}]")
    # joint_to_child_origin = from_child_origin.inverse()
    # # # TODO should these values below be inversed for computing alignment?
    # log_message(f"[DEBUG][mesh_offset] alignment: axis={clean(alignment.Rotation.Axis.x)}, {clean(alignment.Rotation.Axis.y)}, {clean(alignment.Rotation.Axis.z)}, angle={clean(alignment.Rotation.Angle)}")
    # aligned_child = alignment.mu
    # log_message(f"[DEBUG][mesh_offset] aligned_child: Placement [Pos=({clean(aligned_child.Base.x)}, {clean(aligned_child.Base.y)}, {clean(aligned_child.Base.z)}), Axis/Angle=({clean(aligned_child.Rotation.Axis.x)}, {clean(aligned_child.Rotation.Axis.y)}, {clean(aligned_child.Rotation.Axis.z)}), angle={clean(aligned_child.Rotation.Angle)}]")
    # return aligned_child

    # --- New implementation: just inverse ---
    return alignment.multiply(parent_joint.from_child_origin)
    log_message(f"[DEBUG][mesh_offset] from_child_origin: {clean_placement(parent_joint.from_child_origin)}")
    mesh_offset = parent_joint.from_child_origin.inverse()
    log_message(f"[DEBUG][mesh_offset] mesh_offset: Placement [Pos=({clean(mesh_offset.Base.x)}, {clean(mesh_offset.Base.y)}, {clean(mesh_offset.Base.z)}), Axis/Angle=({clean(mesh_offset.Rotation.Axis.x)}, {clean(mesh_offset.Rotation.Axis.y)}, {clean(mesh_offset.Rotation.Axis.z)}), angle={clean(mesh_offset.Rotation.Angle)}]")
    return mesh_offset


def get_joint_transform(prev_joint, curr_joint):
    """Compute the URDF joint origin transform (parent frame to joint frame)."""
    alignment_transform = get_joint_frame_alignment(curr_joint)
    if prev_joint is None:
        transform = curr_joint.from_parent_origin
    else:
        assert hasattr(prev_joint, 'from_child_origin') and prev_joint.from_child_origin is not None and curr_joint.from_parent_origin is not None
        product = prev_joint.from_child_origin.inverse().multiply(curr_joint.from_parent_origin)
        transform = product
        # transform = joint_to_joint_in_parent_joint_frame.multiply(alignment)
        # option 1: 
        # transform = alignment.multiply(prev_joint.from_child_origin.inverse()).multiply(curr_joint.from_parent_origin)
        # # option 2: 
        # transform = alignment.multiply(prev_joint.from_child_origin.inverse().multiply(curr_joint.from_parent_origin))
        # # option 3:
        # transform = curr_joint.from_parent_origin.multiply(prev_joint.from_child_origin.inverse()).multiply(alignment)
        # # option 5:
        # transform = alignment.multiply(curr_joint.from_parent_origin.multiply(prev_joint.from_child_origin.inverse()))
        # this makes sense if transforms are applied from right to left
        # however, the working mesh offset calculation seems to work with the opposite order
        # transform = alignment.multiply(prev_joint.from_child_origin.inverse()).multiply(curr_joint.from_parent_origin)
        
        # transform = alignment.multiply(prev_joint.from_child_origin.inverse().multiply(curr_joint.from_parent_origin))
        # ^GOOD ONE leaves joints in correct position but rotated incorrectly - may just be mesh orientation missing

        # transform = prev_joint.from_child_origin.inverse().multiply(curr_joint.from_parent_origin).multiply(alignment)
        # ^ totally messed up
        # transform = curr_joint.from_parent_origin.multiply(prev_joint.from_child_origin.inverse().multiply(alignment))
        # ^ messed up
    transform = transform.multiply(alignment_transform)
    jname = f"{getattr(curr_joint.parent_link, 'name', '?')}-{getattr(curr_joint.child_link, 'name', '?')}"
    log_joint_transform(jname, curr_joint.from_parent_origin, curr_joint.from_child_origin, alignment_transform, transform)
    return transform


def get_joint_axis(prev_joint, curr_joint):
    """Joint axis in joint local frame (Z for Assembly4)."""
    return App.Vector(0, 0, 1)

def get_global_placement(obj):
    placement = obj.Placement
    parent = obj.getParentGeoFeatureGroup()
    while parent:
        placement = parent.Placement.multiply(placement)
        parent = parent.getParentGeoFeatureGroup()
    return placement

def get_joint_frame_alignment(joint):
    """
    Given:
      - joint: FreeCADJoint object (connecting parent and child)
    Returns:
      A FreeCAD.Placement representing the rotation-only transform from the parent joint frame to the child joint frame.
    """
    parent_link = getattr(joint, 'parent_link', None)
    child_link = getattr(joint, 'child_link', None)
    if parent_link is None or child_link is None:
        raise ValueError("joint must have parent_link and child_link attributes")
    parent_joint_global = parent_link.global_placement.multiply(joint.from_parent_origin)
    child_joint_global = child_link.global_placement.multiply(joint.from_child_origin)
    # Compute the transform from parent joint frame to child joint frame
    difference = parent_joint_global.inverse().multiply(child_joint_global)
    rotation_difference = difference.Rotation
    alignment_transform = App.Placement(App.Vector(0, 0, 0), rotation_difference)
    # log_message(f"[DEBUG][get_joint_frame_alignment] alignment_transform: {alignment_transform}")
    return alignment_transform