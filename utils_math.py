import math
from logging_utils import log_message

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
    """Format a FreeCAD Placement as xyz and rpy strings for URDF.
    Note: FreeCAD's toEuler() returns (yaw, pitch, roll) in degrees (Z, Y, X order),
    but URDF expects (roll, pitch, yaw) in radians (X, Y, Z order).
    So we must swap the order to match URDF's convention.
    """
    if placement is None:
        return "0 0 0", "0 0 0"
    pos = placement.Base
    yaw, pitch, roll = placement.Rotation.toEuler()  # FreeCAD: (Z, Y, X)
    log_message(f"[DEBUG][format_placement] roll: {roll}, pitch: {pitch}, yaw: {yaw}")
    return (
        f"{pos.x * scale:.6f} {pos.y * scale:.6f} {pos.z * scale:.6f}",
        f"{radians(roll):.6f} {radians(pitch):.6f} {radians(yaw):.6f}"
    ) 