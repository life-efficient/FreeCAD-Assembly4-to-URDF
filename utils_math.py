import math

def radians(deg):
    """Convert degrees to radians."""
    return deg * math.pi / 180.0

def format_vector(v, scale=0.001):
    """Format a FreeCAD Vector as a scaled string for URDF (default mm to m)."""
    return f"{v.x * scale:.6f} {v.y * scale:.6f} {v.z * scale:.6f}"

def format_rotation(rot):
    """Format a FreeCAD Rotation as a string of radians (roll, pitch, yaw)."""
    return f"{radians(rot.x):.6f} {radians(rot.y):.6f} {radians(rot.z):.6f}"

def format_placement(placement, scale=0.001):
    """Format a FreeCAD Placement as xyz and rpy strings for URDF."""
    if placement is None:
        return "0 0 0", "0 0 0"
    pos = placement.Base
    rpy = placement.Rotation.toEuler()
    return (
        f"{pos.x * scale:.6f} {pos.y * scale:.6f} {pos.z * scale:.6f}",
        f"{radians(rpy[0]):.6f} {radians(rpy[1]):.6f} {radians(rpy[2]):.6f}"
    ) 