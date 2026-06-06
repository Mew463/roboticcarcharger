import sys
import termios
import tty
import select
import logging
from time import sleep

from settings.config import *
from robot import Robot

logging.basicConfig(level=logging.DEBUG)
logger = logging.getLogger("robot")

robot = Robot()
robot.chassis.enable()

# -----------------------
# Config
# -----------------------
chassis_pwr = 1
stepper_curpos = ElevatorConfig.CHARGE_PORT_HEIGHT_CM
stepper_increment = 1
servo_increment = 5
servo_curpos = ArmConfig.chargerServoStartPos
camservo_curpos = ArmConfig.CAMBEHINDPOS
suctmotor_tog = False
valve_solenoid_tog = False

# -----------------------
# Terminal setup
# -----------------------
fd = sys.stdin.fileno()
old = termios.tcgetattr(fd)
new = termios.tcgetattr(fd)

new[3] &= ~(termios.ICANON | termios.ECHO)
termios.tcsetattr(fd, termios.TCSADRAIN, new)

# -----------------------
# Helpers
# -----------------------
def handle_key(ch):
    global stepper_curpos, servo_curpos, camservo_curpos, suctmotor_tog, valve_solenoid_tog

    if ch == 'w':
        robot.chassis.setVector(0, chassis_pwr, 0)
    elif ch == 's':
        robot.chassis.setVector(0, -chassis_pwr, 0)
    elif ch == 'd':
        robot.chassis.setVector(-chassis_pwr, 0, 0)
    elif ch == 'a':
        robot.chassis.setVector(chassis_pwr, 0, 0)
    elif ch == 'q':
        robot.chassis.setVector(0, 0, chassis_pwr)
    elif ch == 'e':
        robot.chassis.setVector(0, 0, -chassis_pwr)

    elif ch == 'h':
        robot.stepper.home(4000)

    elif ch == 'u':
        stepper_curpos += stepper_increment
        robot.stepper.moveTo(stepper_curpos)
        print(stepper_curpos)

    elif ch == 'j':
        stepper_curpos -= stepper_increment
        robot.stepper.moveTo(stepper_curpos)
        print(stepper_curpos)

    elif ch == 'o':  # CAM SERVO
        camservo_curpos += servo_increment
        robot.cam_servo.set_angle(camservo_curpos)
        print(camservo_curpos)

    elif ch == 'l':
        camservo_curpos -= servo_increment
        robot.cam_servo.set_angle(camservo_curpos)
        print(camservo_curpos)
    elif ch == 'p':
        robot.charger_servo.enable()
        servo_curpos += servo_increment
        robot.charger_servo.set_angle(servo_curpos)

    elif ch == ';':
        robot.charger_servo.enable()
        servo_curpos -= servo_increment
        robot.charger_servo.set_angle(servo_curpos)

    elif ch == 'r':
        print("Front:", robot.lidar_mgr.get_angle(0))

    elif ch == 'f':
        print("Back:", robot.lidar_mgr.get_angle(180))

    elif ch == '[':
        suctmotor_tog = not suctmotor_tog
        robot.suct_motor.setSpeed(1 if suctmotor_tog else 0)
    elif ch == ']':
        valve_solenoid_tog = not valve_solenoid_tog
        if valve_solenoid_tog:
             robot.valve_solenoid.setSpeed(1)
             robot.valve_solenoid.setSpeed(0.5)
        else:
            robot.valve_solenoid.setSpeed(0)
        # robot.valve_solenoid.setSpeed(1 if valve_solenoid_tog else 0)
    elif ch == '1':
        print(robot.charuco_tracking.get_frame(True))
        robot.charuco_tracking.show_frame()

    elif ch == '2':
        print(robot.charuco_tracking.get_frame(False))
        robot.charuco_tracking.show_frame()

    elif ch == 'x':
        return False  # signal exit

    return True


# -----------------------
# Main loop
# -----------------------
try:
    print("Press 'x' to quit")

    running = True

    while running:
        dr, _, _ = select.select([sys.stdin], [], [], 0.1)

        if dr:
            ch = sys.stdin.read(1)
            termios.tcflush(fd, termios.TCIFLUSH)

            running = handle_key(ch)
        else:
            robot.chassis.stop()
            # robot.chassis.setVector(0,0,0)

finally:
    termios.tcsetattr(fd, termios.TCSADRAIN, old)
    robot.lidar_mgr.stop()
    robot.chassis.stop()
    print("Clean exit")