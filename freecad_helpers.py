import FreeCAD as App
import Mesh
import os

# --- Logging helpers (moved from ExportAssembly4ToURDF_tree.py) ---
# Remove log_message and log_newline from this file and import from logging_utils
from logging_utils import log_message, log_newline

MESH_FORMAT = "stl"
PLA_DENSITY = 1240


def get_link_name_from_reference(ref):
    """Extract the link name from a FreeCAD joint reference tuple."""
    if ref and len(ref) == 2 and len(ref[1]) > 0:
        return ref[1][0].split('.')[0]
    return None

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
    """
    Compute the rotation-only alignment from 'from_placement' to 'to_placement'.
    Returns a FreeCAD.Placement representing the rotation-only transform.
    This is the rotation that, when applied to 'from_placement', aligns it with 'to_placement'.
    """
    log_message(f"[DEBUG] get_origin_alignment: \n\tfrom_placement: {clean_placement(from_placement)}\n\tto_placement: {clean_placement(to_placement)}")
    # Compute the transform that brings 'from_placement' to 'to_placement'
    difference = to_placement.multiply(from_placement.inverse())
    rotation_difference = difference.Rotation
    log_message(f"[DEBUG] get_origin_alignment: {clean(rotation_difference.Angle)} radians about ({clean(rotation_difference.Axis.x)}, {clean(rotation_difference.Axis.y)}, {clean(rotation_difference.Axis.z)})")
    return App.Placement(App.Vector(0, 0, 0), rotation_difference)


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
    """
    Compute the URDF joint origin transform (parent frame to joint frame).
    Returns a FreeCAD.Placement.
    """
    alignment_transform = get_joint_frame_alignment(curr_joint)
    if prev_joint is None:
        log_message(f"\t[DEBUG][get_joint_transform] - this joint must be attached to the root link")
        transform = curr_joint.from_parent_origin
    else:
        assert hasattr(prev_joint, 'from_child_origin') and prev_joint.from_child_origin is not None and curr_joint.from_parent_origin is not None
        log_message(f"\t[DEBUG][get_joint_transform] prev_joint.from_child_origin: {clean_placement(prev_joint.from_child_origin)}")
        log_message(f"\t[DEBUG][get_joint_transform] curr_joint.from_parent_origin: {clean_placement(curr_joint.from_parent_origin)}")
        product = prev_joint.from_child_origin.inverse().multiply(curr_joint.from_parent_origin)
        joint_to_joint_in_parent_joint_frame = product
        log_message(f"\t[DEBUG][get_joint_transform] unaligned transform {clean_placement(product)}")
        # transform = joint_to_joint_in_parent_joint_frame
        transform = joint_to_joint_in_parent_joint_frame
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
    log_message(f"\t[DEBUG][get_joint_transform] transform: {clean_placement(transform)}")
    return transform


def get_joint_axis(prev_joint, curr_joint):
    """
    Compute the joint axis in the joint's local frame (URDF expects axis after <origin> is applied).
    Returns a FreeCAD.Vector.
    Logs the transform and resulting axis.
    """
    # Always return Z axis in joint local frame
    z_axis = App.Vector(0, 0, 1)
    log_message(f"[DEBUG][get_joint_axis] axis in joint local frame: (0, 0, 1)")
    return z_axis

def get_global_placement(obj):
    # return obj.getGlobalPlacement()
    placement = obj.Placement
    parent = obj.getParentGeoFeatureGroup()
    while parent:
        placement = parent.Placement.multiply(placement)
        parent = parent.getParentGeoFeatureGroup()
    log_message(f"[DEBUG][get_global_placement] {obj.Name} global placement: {placement}")
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