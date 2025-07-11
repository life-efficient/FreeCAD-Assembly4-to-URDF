import FreeCAD as App
import Mesh
import os

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


def get_joint_urdf_transform(parent_placement, child_placement):
    """
    Compute the URDF joint origin transform (parent frame to joint frame) as parent_placement * child_placement.inverse().
    Returns a FreeCAD.Placement.
    """
    return parent_placement.multiply(child_placement.inverse())


def get_joint_axis_in_urdf_frame(joint_placement):
    """
    Get the Z axis of the joint's local frame, expressed in the parent link's frame.
    Returns a tuple (x, y, z).
    """
    # The Z axis in the joint's local frame is (0, 0, 1)
    z_axis_local = App.Vector(0, 0, 1)
    # Rotate it into the parent frame
    z_axis_parent = joint_placement.Rotation.multVec(z_axis_local)
    return (z_axis_parent.x, z_axis_parent.y, z_axis_parent.z) 