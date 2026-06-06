import sys
import termios
import select
import logging

from motorcontrol.dcmotor import Chassis

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger("chassis")

chassis = Chassis()
chassis.enable()

# -----------------------
# Config
# -----------------------
chassis_pwr = 1.0

# -----------------------
# Terminal setup
# -----------------------
fd = sys.stdin.fileno()
old = termios.tcgetattr(fd)
new = termios.tcgetattr(fd)

new[3] &= ~(termios.ICANON | termios.ECHO)
termios.tcsetattr(fd, termios.TCSADRAIN, new)

# -----------------------
# Key handler
# -----------------------
def handle_key(ch):
    if ch == 'w':
        chassis.setVector(0,  chassis_pwr, 0)
    elif ch == 's':
        chassis.setVector(0, -chassis_pwr, 0)
    elif ch == 'd':
        chassis.setVector(-chassis_pwr, 0, 0)
    elif ch == 'a':
        chassis.setVector( chassis_pwr, 0, 0)
    elif ch == 'q':
        chassis.setVector(0, 0,  chassis_pwr)
    elif ch == 'e':
        chassis.setVector(0, 0, -chassis_pwr)
    elif ch == 'x':
        return False

    return True


# -----------------------
# Main loop
# -----------------------
try:
    print("WASD = move | Q/E = rotate | X = exit")

    running = True

    while running:
        dr, _, _ = select.select([sys.stdin], [], [], 0.1)

        if dr:
            ch = sys.stdin.read(1)
            termios.tcflush(fd, termios.TCIFLUSH)
            running = handle_key(ch)
        else:
            # stop when no key pressed
            chassis.setVector(0, 0, 0)

finally:
    termios.tcsetattr(fd, termios.TCSADRAIN, old)
    chassis.stop()
    print("Clean exit")