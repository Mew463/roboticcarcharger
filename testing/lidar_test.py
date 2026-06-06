from sensors.lidar import LidarManager
import time
import logging

logging.basicConfig(level=logging.DEBUG)
logger = logging.getLogger("__name__")

lidar_mgr = LidarManager()
lidar_mgr.start()

print(lidar_mgr.get_angle(0))
print(lidar_mgr.get_angle(180))
# print(lidar_mgr._get_subset((340, 20)))
while True:
    print(lidar_mgr.get_median_from_subset((340, 20)))
    time.sleep(0.1)
lidar_mgr.stop()