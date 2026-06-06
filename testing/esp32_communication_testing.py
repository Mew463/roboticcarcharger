from motorcontrol.esp32_interfaces import *
from settings.config import ArmConfig
import logging
from settings import pins
import serial
import time

logging.basicConfig(level=logging.DEBUG)
logger = logging.getLogger("Robot")
ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
cam_servo = Servo(ser, pins.CAM_SERVO)
servos_cur = Esp32AnalogRead(ser, pins.SERVO_CUR)
serv0 = HighPowerServo(ser, pins.SERV0, pins.SERVO_EN, servos_cur)
my_stepper = Stepper(ser)
rgb_leds = RGBLeds(ser)

 
val = 1000
# my_stepper.moveTo(val)
# time.sleep(10)
# my_stepper.moveTo(-val)
# my_stepper.home(1000)
# time.sleep(5)
# my_stepper.moveTo(-val)
 
# print(servos_cur.analogRead())
# time.sleep(1)

def printCur():
    val = servos_cur.analogRead() - 2000
    
    max_val = 4095 - 2000
    max_width = 150
    
    scaled = int((val / max_val) * max_width)
    bar = "=" * scaled
    
    print(f"{val:4d} |{bar}")

# rgb_leds.set_circle(Colors.GREEN)
# rgb_leds.set_breathing(Colors.WHITE)
rgb_leds.set_static(Colors.BLACK, 100)
# for i in range(6):
    
#     time.sleep(3)

# serv0.enable()
# stepsize = 1

# while True:
#     for i in range(ArmConfig.chargerServoStartPos, ArmConfig.chargerServoEndPos, -stepsize):
#         serv0.set_angle(i)
#         printCur()
#         time.sleep(0.05)
#     for i in range(ArmConfig.chargerServoEndPos, ArmConfig.chargerServoStartPos, stepsize):
#         serv0.set_angle(i)
#         printCur()
#         time.sleep(0.05)    

