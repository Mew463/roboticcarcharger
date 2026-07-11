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
        dir = -1 if is_reversed else 1
        self.chassis.move_vector_smooth(dir, 0, 0) 
        time.sleep(2.5)
    
    def _open_valve(self):
        self.valve_solenoid.setSpeed(1)
        self.valve_solenoid.setSpeed(0.5)
    
    def approach(self, tesla_control:TeslaControl):
        tesla_control.open_or_unlatch_charge_port(wait_for_completion = "false") # Make sure the car opens door ASAP
        
        self.leds.set_circle(Colors.BLUE)
        self.stepper.home(4000)
        self.chassis.enable()
        
        self.chassis.move_vector_smooth(0, -1, 0) # backup
        time.sleep(1)
        
        self._move_lateral(False)
        self.leds.set_circle(Colors.PURPLE)
        
        self.chassis.move_vector_smooth(0, 0, 0.65) # Rotate a little to align better with car
        time.sleep(0.5)
            
        while (self.lidar_mgr.get_angle(0) > 300): # Drive until we are close to the car
            time.sleep(0.1) # Necessary to allow lidar to update
            self.chassis.setVector(0, 0.5, 0) 

        self.chassis.move_vector_smooth(0, 0, 0)
        
    def align(self, movement, tesla_control:TeslaControl):
        self.cam_servo.set_angle(ArmConfig.CAMTESLAPOS)
        self.stepper.moveTo(ElevatorConfig.CHARGE_PORT_HEIGHT_MM)
        cur_led_brightness = CameraConfig.INITIAL_BRIGHTNESS
        self.leds.set_static(Colors.WHITE, brightness = cur_led_brightness)
        # Move left suction cup suck more = yaw more negative 
        # Move robot leftwards = x more positive 
        target_x = -0.0798 - 0.005
        target_y = 0.308
        target_yaw = 0 - 0.1
        target_z = -0.177
        num_attempts = 3
        for i in range(num_attempts): # ALIGNING LOOP
            
            while (not movement.move_to_tag_position(x = target_x, y = target_y, yaw = target_yaw, use_wall_board = False).is_success(precision_multiplier = 2)): 
                
                cur_frame_brightness = self.charuco_tracking.get_frame_brightness()
                if cur_frame_brightness is not None:
                    led_error = CameraConfig.SETPOINT_BRIGHTNESS - cur_frame_brightness
                    cur_led_brightness += led_error * 0.1 # Only using I term 
                    cur_led_brightness = max(0, min(cur_led_brightness, 100))
                    self.leds.set_static(Colors.WHITE, cur_led_brightness)
                    print(f"led bright:{cur_led_brightness} frame bright: {cur_frame_brightness}")
                pass
            
            self.suct_motor.setSpeed(1)
            suct_motor_vals = MovingAverage(30)
            while (not movement.move_to_tag_position(x = target_x, y = target_y - 0.05, yaw = target_yaw, use_wall_board = False).is_success(precision_multiplier = 1.25)): 
                suct_motor_vals.add(self.suct_motor_cur.analogRead())
                med_z_val = movement.get_z_median()
                if med_z_val is not None:
                    delta_z = (target_z - med_z_val) * 1000
                    print(delta_z)
                    if (abs(delta_z) > ElevatorConfig.Z_TOL_MM):
                        print("Z adjustment!")
                        movement.z_vals.clear()
                        self.stepper.moveRelative(delta_z)
                
                pass
            
            start_time = time.time()
            success = False
            
            while (time.time() - start_time < 3 and not success):
                if (self.lidar_mgr.get_angle(0) < 100): # We are too close to the car 
                    self.chassis.stop()
                else:
                    self.chassis.setVector(0, 0.65, 0) 
                suct_motor_vals.add(self.suct_motor_cur.analogRead())
                med = suct_motor_vals.get_med()
                print(med)
                if (med is not None):
                    if (med < ArmConfig.sucMotorThresholdCurrent):
                        self.logger.info("suck-ccess!")
                        success = True
                    
            if (success):
                break    
            self.chassis.stop()
            self.suct_motor.setSpeed(0)
            self._open_valve()
            self.logger.info("failed insertion to car, trying again")
            self.chassis.move_vector_smooth(0, -0.65, 0)
            time.sleep(1)
            self.chassis.stop()
            self.valve_solenoid.setSpeed(0)
            movement.reset_errors()
            if (i == num_attempts-1):
                raise Exception(f"Failed to align after {num_attempts} tries")
            
        self.chassis.move_vector_smooth(0, -0.25, 0)
        self.stepper.moveRelative(2.5)
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
        
        while (self.lidar_mgr.get_angle(0) < 250): # Drive until we far enough away from car
            time.sleep(0.1) # Necessary to allow lidar to update
            self.chassis.setVector(0, -1, 0) 
        self.chassis.move_vector_smooth(0, 0, -.65) # Rotate a little to not fall into the crack
        time.sleep(0.5)
        self.valve_solenoid.setSpeed(0)
        self.cam_servo.set_angle(ArmConfig.CAMHOMINGPOS)
        self.stepper.home(4000)
        self._move_lateral(True)
        self.chassis.move_vector_smooth(0, 1, 0)
        time.sleep(0.5)
        
    def home(self, movement):
        self.leds.set_static(Colors.WHITE)
        self.chassis.stop()
        # Uses yaw - More positive is robot left side closer to wall
        while (not movement.move_to_tag_position(x = -0.192, y = 0.912, yaw = -8.8, use_wall_board = True).is_success(precision_multiplier = 1.5)): # Home itself
            pass
        
        self.chassis.disable()
        