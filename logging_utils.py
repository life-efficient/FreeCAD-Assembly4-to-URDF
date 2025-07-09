import os

MANUAL_CHECK_FILE = os.path.join(os.path.dirname(__file__), "manual_check.txt")

def log_message(msg):
    try:
        with open(MANUAL_CHECK_FILE, 'r', encoding='utf-8') as f:
            line_count = sum(1 for _ in f)
        assert line_count < 500, f"manual_check.txt exceeds 500 lines ({line_count}) -- possible infinite loop!"
    except FileNotFoundError:
        pass
    print(msg)
    with open(MANUAL_CHECK_FILE, 'a', encoding='utf-8') as f:
        f.write(msg + '\n')

def log_newline():
    with open(MANUAL_CHECK_FILE, 'a', encoding='utf-8') as f:
        f.write('\n') 