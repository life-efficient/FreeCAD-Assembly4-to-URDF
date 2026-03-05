import os

MANUAL_CHECK_FILE = os.path.join(os.path.dirname(__file__), "manual_check.txt")
MAX_LOG_LINES = 15000
DEBUG = False  # Set True for verbose subassembly/transform logging

def log_message(msg, force=False):
    if not force and not DEBUG and msg.startswith(("[DEBUG]", "[resolve]", "[collect_parts]", "\t")):
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