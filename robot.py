### Initializes all robot actuators and sensors
### Defines higher level actions that are important for the robot to do
###

import serial
import time
import logging
from motorcontrol.dcmotor import Chassis, DCMotor
from sensors.vision import CharucoTracking
from motorcontrol.esp32_interfaces import *
from sensors.lidar import LidarManager
from settings import pins
from gpiozero import Button
from algorithms.robot_states import *
from algorithms.moving_avg import *
from settings.config import *
from api.tesla_control import TeslaControl

class Robot():
    def __init__(self):
        self.ser = serial.Serial( '/dev/ttyACM0', 115200, timeout=1)
        self.chassis = Chassis()
        self.chassis.enable()
        self.charuco_tracking = CharucoTracking() 
        self.cam_servo = Servo(self.ser, pins.CAM_SERVO)
        self.stepper = Stepper(self.ser)
        self.suct_motor = DCMotor(pins.SUCT_MOTOR, None)
        self.suct_motor_cur = Esp32AnalogRead(self.ser, pins.SUCT_MOTOR_CUR)
        self.valve_solenoid = DCMotor(pins.VALVE_SOLENOID, None)
        self.charger_servo_cur = Esp32AnalogRead( self.ser, pins.SERVO_CUR)
        self.charger_servo = HighPowerServo(self.ser, pins.SERV0, pins.SERVO_EN, self.charger_servo_cur)
        self.button0 = Button(pins.BUT0, pull_up = True)    
        self.button0.when_pressed = self.button0_is_pushed
        self.button0_val = False
        self.last_but_push = time.time()
        self.lidar_mgr = LidarManager()
        self.lidar_mgr.start()
        self.leds = RGBLeds(self.ser)
        
        self.logger = logging.getLogger(f"{__name__}")
        
        self.state = RobotStates.IDLE_PARKED
        self.insertion_state = None
        self.removal_state = RemovalStates.REMOVING_CHARGER
        
    def button0_is_pushed(self):
        if time.time() - self.last_but_push > 0.5:
            self.button0_val = True
            self.logger.info("Button interrupt triggered")
        self.last_but_push = time.time()

    def button0_was_pushed(self):
        if (self.button0_val):
            self.button0_val = False
            return True
        else:
            return False    
    def _move_lateral(self, is_reversed):
        dir = -1
        if is_reversed:
            dir = 1
        self.chassis.move_vector_smooth(dir, 0, 0) 
        time.sleep(0.75)
    
    def _open_valve(self):
        self.valve_solenoid.setSpeed(1)
        self.valve_solenoid.setSpeed(0.5)
    
    def approach(self):
        self.leds.set_circle(Colors.BLUE)
        self.stepper.home(4000)
        self.chassis.enable()
        while (self.lidar_mgr.get_angle(0) > 550): # Drive until we are at the car
            time.sleep(0.1) # Necessary to allow lidar to update
            self.chassis.setVector(0, 1, 0) 
            if (self.lidar_mgr.get_angle(180) + self.lidar_mgr.get_angle(0) > CarConfig.MAX_MOVING_DIST):
                # Switch to homing state cause something has gone wrong
                pass
        self._move_lateral(False)
        self.leds.set_circle(Colors.PURPLE)
        while (self.lidar_mgr.get_angle(0) > 300): # Drive until we are close to the car
            time.sleep(0.1) # Necessary to allow lidar to update
            self.chassis.setVector(0, 0.5, 0) 

        self.chassis.move_vector_smooth(0, 0, 0)
        
    def align(self, Movement, tesla_control:TeslaControl):
        self.cam_servo.set_angle(ArmConfig.CAMINFRONTPOS)
        self.stepper.moveTo(ElevatorConfig.CHARGE_PORT_HEIGHT_CM)
        tesla_control.open_or_unlatch_charge_port() # A better use of elevator delay 
        cur_led_brightness = CameraConfig.INITIAL_BRIGHTNESS
        self.leds.set_static(Colors.WHITE, brightness = cur_led_brightness)
        # Move left suction cup suck more = yaw more negative 
        # Move robot leftwards = x more positive 
        target_x = -0.0798 - 0.005
        target_y = 0.308
        target_yaw = 0
        num_attempts = 3
        for i in range(num_attempts): # ALIGNING LOOP
            
            while (not Movement.move_to_tag_position(x = target_x, y = target_y, yaw = target_yaw, use_wall_board = False).is_success(precision_multiplier = 2)): 
                
                cur_frame_brightness = self.charuco_tracking.get_frame_brightness()
                if cur_frame_brightness is not None:
                    led_error = CameraConfig.SETPOINT_BRIGHTNESS - cur_frame_brightness
                    cur_led_brightness += led_error * 0.1 # Only using I term 
                    cur_led_brightness = max(0, min(cur_led_brightness, 100))
                    self.leds.set_static(Colors.WHITE, cur_led_brightness)
                    # print(f"{cur_led_brightness} {cur_frame_brightness}")
                pass

            while (not Movement.move_to_tag_position(x = target_x, y = target_y - 0.05, yaw = target_yaw, use_wall_board = False).is_success(precision_multiplier = 1.25)): 
                print( Movement.get_z_median())
                pass
            self.suct_motor.setSpeed(1)
            
            start_time = time.time()
            success = False
            suct_motor_avg = MovingAverage(25)
            while (time.time() - start_time < 3 and not success):
                if (self.lidar_mgr.get_angle(0) < 100): # We are too close to the car 
                    self.chassis.stop()
                else:
                    self.chassis.setVector(0, 0.65, 0) 
                suct_motor_avg.add(self.suct_motor_cur.analogRead())
                avg = suct_motor_avg.get_avg()
                if (avg is not None):
                    if (avg < ArmConfig.sucMotorThresholdCurrent):
                        self.logger.info("suck-ccess!")
                        success = True
                    
            if (success):
                break    
            self.chassis.stop()
            self.suct_motor.setSpeed(0)
            self._open_valve()
            self.logger.info("failed insertion to car, trying again")
            self.chassis.move_vector_smooth(0, -0.65, 0)
            self.valve_solenoid.setSpeed(0)
            Movement.reset_errors()
            if (i == num_attempts-1):
                raise Exception(f"Failed to align after {num_attempts} tries")
            
        self.chassis.move_vector_smooth(0, -0.25, 0)
        self.stepper.moveTo(ElevatorConfig.CHARGE_PORT_HEIGHT_CM + 0.25)
        time.sleep(0.25)
        self.chassis.disable()
        
    def insert_charger(self):
        self.charger_servo.enable()
        curPos = ArmConfig.chargerServoStartPos
        
        while (curPos > ArmConfig.chargerServoEndPos): # SERVO INSERTION
            curPos -= 1
            self.charger_servo.set_angle(curPos)
            time.sleep(0.01)

        self.charger_servo.disable()
        self.leds.set_breathing(Colors.BLUE)
        self.suct_motor.setSpeed(0)
        
    def remove_charger(self):
        self.charger_servo.enable()
        self._open_valve()
        curPos = ArmConfig.chargerServoEndPos
        while (curPos < ArmConfig.chargerServoStartPos):
            curPos += 1
            self.charger_servo.set_angle(curPos)
            time.sleep(0.01)
        self.charger_servo.disable()

    def get_car_clearance(self):
        self.chassis.enable()
        self.leds.set_circle(Colors.BLUE)
        
        while (self.lidar_mgr.get_angle(0) < 500): # Drive until we far enough away from car
            time.sleep(0.1) # Necessary to allow lidar to update
            self.chassis.setVector(0, -1, 0) 
        self.valve_solenoid.setSpeed(0)
        self.cam_servo.set_angle(ArmConfig.CAMBEHINDPOS)
        self.stepper.home(4000)
        self._move_lateral(True)
        
    def home(self, Movement):
        self.leds.set_static(Colors.WHITE)
        self.chassis.move_vector_smooth(0, -1, 0) # Go a bit closer to the wall so that leds can hit the target
        time.sleep(1)
        self.chassis.stop()
        # Uses yaw - More positive is robot left side closer to wall
        while (not Movement.move_to_tag_position(x = -0.053, y = 0.75, yaw = 0.5, use_wall_board = True).is_success(precision_multiplier = 1.5)): # Home itself
            pass
        
        while (self.lidar_mgr.get_angle(180) > 100): # Drive until we are pretty close to the wall
            time.sleep(0.1) # Necessary to allow lidar to update
            self.chassis.setVector(0, -0.6, 0) 
        
        self.chassis.disable()
        