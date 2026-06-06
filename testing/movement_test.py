import time
import logging
from robot import Robot
from settings.config import CarConfig, ArmConfig, Colors
from algorithms.movement import FusedMovement
from algorithms.moving_avg import *
from algorithms.robot_states import *
from api.tesla_control import TeslaControl
from api.telegram_control import TelegramControl, MessagingReceiverWrapper

logging.basicConfig(level=logging.DEBUG)
logger = logging.getLogger("Robot")
logging.getLogger("urllib3").setLevel(logging.WARNING) # Surpress noisy loggers
logging.getLogger("requests").setLevel(logging.WARNING) # Surpress noisy loggers

robot = Robot()
messaging = TelegramControl()
msg_receiver = MessagingReceiverWrapper(messaging)
msg_receiver.start()
logger.addHandler(messaging)
my_blue_panther = TeslaControl()
Movement = FusedMovement(robot.chassis, robot.charuco_tracking)

try:
    # robot.align(Movement, my_blue_panther)

    robot.home(Movement)


except KeyboardInterrupt:
    robot.lidar_mgr.stop()

except Exception as e:
    import traceback
    traceback.print_exc()
    robot.lidar_mgr.stop()