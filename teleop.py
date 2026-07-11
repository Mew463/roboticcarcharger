import sys
import termios
import tty
import select
import logging
import time

from settings.config import *
from robot import Robot
from algorithms.moving_avg import *

logging.basicConfig(level=logging.DEBUG)
logger = logging.getLogger("robot")

robot = Robot()
robot.chassis.enable()

# -----------------------
# Config
# -----------------------
chassis_pwr = 1
stepper_curpos = ElevatorConfig.CHARGE_PORT_HEIGHT_MM
stepper_increment = 10
servo_increment = 5
servo_curpos = ArmConfig.chargerServoStartPos
camservo_curpos = ArmConfig.CAMHOMINGPOS
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
        robot.chassis.setVector(0, chassis_pwr, 0, accel = False)
    elif ch == 's':
        robot.chassis.setVector(0, -chassis_pwr, 0, accel = False)
    elif ch == 'd':
        robot.chassis.setVector(-chassis_pwr, 0, 0, accel = False)
    elif ch == 'a':
        robot.chassis.setVector(chassis_pwr, 0, 0, accel = False)
    elif ch == 'q':
        robot.chassis.setVector(0, 0, chassis_pwr, accel = False)
    elif ch == 'e':
        robot.chassis.setVector(0, 0, -chassis_pwr, accel = False)

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

    elif ch == 'o':  # CAM SERVO UP
        camservo_curpos += servo_increment
        robot.cam_servo.set_angle(camservo_curpos)
        print(camservo_curpos)

    elif ch == 'l': # CAM SERVO DOWN
        camservo_curpos -= servo_increment
        robot.cam_servo.set_angle(camservo_curpos)
        print(camservo_curpos)
    elif ch == 'p': # CHARGE SERVO INSERT
        robot.charger_servo.enable()
        servo_curpos -= servo_increment
        robot.charger_servo.set_angle(servo_curpos)

    elif ch == ';': # CHARGE SERVO REMOVE
        robot.charger_servo.enable()
        servo_curpos += servo_increment
        robot.charger_servo.set_angle(servo_curpos)

    elif ch == 'r':
        print("Front:", robot.lidar_mgr.get_angle(300))

    elif ch == 'f':
        print("Back:", robot.lidar_mgr.get_angle(180))

    elif ch == '[':
        suctmotor_tog = not suctmotor_tog
        robot.suct_motor.setSpeed(1 if suctmotor_tog else 0)
    elif ch == '-':
        suct_motor_avg = MovingAverage(25)
        for i in range(1000):
            suct_motor_avg.add(robot.suct_motor_cur.analogRead())
            avg = suct_motor_avg.get_avg()
            print(avg)
    elif ch == ']':
        valve_solenoid_tog = not valve_solenoid_tog
        if valve_solenoid_tog:
             robot.valve_solenoid.setSpeed(1)
             robot.valve_solenoid.setSpeed(0.5)
        else:
            robot.valve_solenoid.setSpeed(0)
            
    elif ch == '1': # Take photo of wall board
        print(robot.charuco_tracking.get_frame(True))
        robot.charuco_tracking.show_frame()

    elif ch == '2': # Take photo of car tracking board
        print(robot.charuco_tracking.get_frame(False))
        robot.charuco_tracking.show_frame()
        
    elif ch == '3': # Increase camera brightness
        print(robot.charuco_tracking.get_frame(False))
        robot.charuco_tracking.show_frame()
        
    elif ch == '4': # Decrease camera brightness
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