from sensors.adxl343 import *
import time
imu = IMUSensor()
while True:
    x, y, z = imu.get_acceleration()
    
    print(f"{x:0.2f} {y:0.2f} {z:0.2f}")
    time.sleep(0.1
               )
    # avg_z = accel_filter.get_avg()
    # if avg_z is not None and avg_z < 8.0:
    #     print("Warning: Robot is tipping over!")