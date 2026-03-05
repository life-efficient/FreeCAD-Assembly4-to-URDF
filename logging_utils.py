import os

MANUAL_CHECK_FILE = os.path.join(os.path.dirname(__file__), "manual_check.txt")
MAX_LOG_LINES = 15000
DEBUG = False  # Set True for verbose subassembly/transform logging
DEBUG_JOINT_TRANSFORM = False  # Set True to log joint transform details for debugging
DEBUG_REF_RESOLUTION = True   # Set True to log ref->link resolution for shoulder/ground joints

def log_message(msg, force=False):
    if not force and not DEBUG and msg.startswith(("[DEBUG]", "[resolve]", "[collect_parts]", "\t", "[CYCLE]", "[LINK #", "[JOINT]")):
        return
    try:
        with open(MANUAL_CHECK_FILE, 'r', encoding='utf-8') as f:
            line_count = sum(1 for _ in f)
        assert line_count < MAX_LOG_LINES, f"manual_check.txt exceeds {MAX_LOG_LINES} lines -- possible infinite loop!"
    except FileNotFoundError:
        pass
    print(msg)
    with open(MANUAL_CHECK_FILE, 'a', encoding='utf-8') as f:
        f.write(msg + '\n')

def log_newline():
    if not DEBUG:
        return
    with open(MANUAL_CHECK_FILE, 'a', encoding='utf-8') as f:
        f.write('\n')

def log_joint_transform(joint_name, from_parent, from_child, alignment, result, expected_from_global=None):
    """Targeted log for joint transform debugging. Only prints when DEBUG_JOINT_TRANSFORM."""
    if not DEBUG_JOINT_TRANSFORM:
        return
    if "Shoulder_V001" not in str(joint_name):
        return
    def fmt(p):
        if p is None:
            return "None"
        b = p.Base
        y, p_, r = p.Rotation.toEuler()
        return f"xyz=({b.x:.2f}, {b.y:.2f}, {b.z:.2f}) yaw={y:.1f} pitch={p_:.1f} roll={r:.1f} deg"
    print(f"\n--- [JOINT TRANSFORM] {joint_name} ---")
    print(f"  from_parent_origin: {fmt(from_parent)}")
    print(f"  from_child_origin:  {fmt(from_child)}")
    print(f"  alignment (rot):    {fmt(alignment)}")
    print(f"  result (urdf):     {fmt(result)}")
    if expected_from_global is not None:
        print(f"  expected (global):  {fmt(expected_from_global)}")
    print("---\n") 