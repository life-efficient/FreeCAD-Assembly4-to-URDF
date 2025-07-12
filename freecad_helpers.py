import FreeCAD as App
import Mesh
import os

# --- Logging helpers (moved from ExportAssembly4ToURDF_tree.py) ---
# Remove log_message and log_newline from this file and import from logging_utils
from logging_utils import log_message, log_newline

ROBOT_NAME = "my_robot"
EXPORT_DIR = os.path.join(os.path.expanduser("~"), "projects/FreeCAD-Designs", "macros", ROBOT_NAME)
MESH_FORMAT = "stl"
PLA_DENSITY = 1240


def get_link_name_from_reference(ref):
    """Extract the link name from a FreeCAD joint reference tuple."""
    if ref and len(ref) == 2 and len(ref[1]) > 0:
        return ref[1][0].split('.')[0]
    return None

def export_mesh(body, name):
    """Export a FreeCAD body as a mesh and return the mesh path."""
    mesh_path = os.path.join(EXPORT_DIR, "meshes", f"{name}.{MESH_FORMAT}")
    Mesh.export([body], mesh_path)
    return mesh_path

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
    # Compute the transform that brings 'from_placement' to 'to_placement'
    difference = to_placement.multiply(from_placement.inverse())
    rotation_difference = difference.Rotation
    log_message(f"[DEBUG] get_origin_alignment: axis=({clean(rotation_difference.Axis.x)}, {clean(rotation_difference.Axis.y)}, {clean(rotation_difference.Axis.z)}), angle={clean(rotation_difference.Angle)}")
    return App.Placement(App.Vector(0, 0, 0), rotation_difference)


def get_mesh_offset(parent_joint):
    """
    Compute the mesh offset placement for URDF export.
    This version removes the alignment step: just use the inverse of from_child_origin.
    Returns a FreeCAD.Placement.
    """
    # return zero offset
    # return App.Placement(App.Vector(0,19,0), App.Rotation(0,0,0,0))

    # OPTION 0: Just transform
    # return parent_joint.from_child_origin

    # OPTION 1: Just inverse 
    return parent_joint.from_child_origin.inverse()

    alignment = get_origin_alignment(
        parent_joint.from_parent_origin, 
        parent_joint.from_child_origin
    )

    # OPTION 2: Align and then inverse (transformations happen from right to left)
    # return alignment.multiply(parent_joint.from_child_origin.inverse())
    # servo translated -Z by what should be +Y

    # OPTION 3: Align and then inverse
    # return parent_joint.from_child_origin.inverse().multiply(alignment)
    # servo translated -Z by what should be +Y

    # OPTION 4: Inverse then align
    return alignment.multiply(parent_joint.from_child_origin)
    # servo translated -Z by what should be +Y

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
    if prev_joint is None:
        log_message(f"\t[DEBUG][get_joint_transform] - this joint must be attached to the root link")
        transform = curr_joint.from_parent_origin
    else:
        assert hasattr(prev_joint, 'from_child_origin') and prev_joint.from_child_origin is not None and curr_joint.from_parent_origin is not None
        axis_alignment = get_origin_alignment(prev_joint.from_child_origin, curr_joint.from_parent_origin)
        transform = prev_joint.from_child_origin.inverse().multiply(axis_alignment).multiply(curr_joint.from_parent_origin)
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