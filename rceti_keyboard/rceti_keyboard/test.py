import sys
import termios
import tty
from select import select

def detect_key():
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select([sys.stdin], [], [], 5)  # 5-second timeout
    if rlist:
        key = sys.stdin.read(1)
        if key == '\x1b':  # Escape character
            key += sys.stdin.read(2)  # Read the next two characters
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, termios.tcgetattr(sys.stdin))
        print(f"Key pressed: {repr(key)}")
    else:
        print("No key pressed")
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, termios.tcgetattr(sys.stdin))

print("Press any key (timeout in 5 seconds):")
detect_key()