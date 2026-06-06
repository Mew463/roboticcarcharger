import serial
from sensors.vision import CharucoTracking
from motorcontrol.esp32_interfaces import RGBLeds
from settings.config import Colors

my_tracker = CharucoTracking()
ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
rgb_leds = RGBLeds(ser)

cur_led_brightness = 100
rgb_leds.set_static(Colors.WHITE, brightness = 100)
while True:
    # True = Wall charuco , False = tesla charuco
    print(my_tracker.get_frame(False))
    # cur_frame_brightness = my_tracker.get_frame_brightness()
    # if cur_frame_brightness is not None:
    #     setpoint_brightness = 50
    #     led_error = setpoint_brightness - cur_frame_brightness
    #     cur_led_brightness += led_error * 0.1 # Only using I term 
    #     cur_led_brightness = max(0, min(cur_led_brightness, 100))
    #     rgb_leds.set_static(Colors.WHITE, cur_led_brightness)
    #     print(f"{cur_led_brightness} {cur_frame_brightness}")
    my_tracker.show_frame()

