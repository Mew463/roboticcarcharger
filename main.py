import time
import logging
import threading
from robot import Robot
from settings.config import CarConfig, ArmConfig, Colors, DistanceThreshold
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
stop_event = threading.Event()
messaging = TelegramControl(stop_event, robot.charuco_tracking)
msg_receiver = MessagingReceiverWrapper(messaging)
msg_receiver.start()
logger.addHandler(messaging)
my_blue_panther = TeslaControl()
Movement = FusedMovement(robot.chassis, robot.charuco_tracking)

# Variables to help determine when car can/should charge
was_car_gone = False
car_ready_to_charge = True

check_time = time.time()
car_detected_start_time = time.time()
car_gone_start_time = time.time()

def is_car_in_distance(vals: DistanceThreshold) -> bool:
    return (abs(robot.lidar_mgr.get_angle(0) - vals.target_distance) < vals.threshold)

if (is_car_in_distance(CarConfig.CAR_PLUGGED_DIST)):
    robot.state = RobotStates.IDLE_CHARGING
    logger.info("SET TO IDLE_CHARGING!")
    messaging.send_message("SET TO IDLE_CHARGING!")

try:
    while not stop_event.is_set(): 
        if robot.state == RobotStates.IDLE_PARKED:
            time.sleep(0.1) # Need for lidar to update
            car_parked_in_distance = is_car_in_distance(CarConfig.CAR_PARKED_DIST)
            if robot.button0_was_pushed() == True or msg_receiver.get_cmd() == "/charge": # Manual activation
                if (car_parked_in_distance and car_ready_to_charge):
                    robot.state = RobotStates.INSERTING
                    robot.leds.set_static(Colors.YELLOW)    
                else:
                    robot.leds.set_static(Colors.RED)
                    time.sleep(1)
                    robot.leds.set_static(Colors.BLACK)
                    logger.info("Manual car charging criteria not met!")
                    if not car_parked_in_distance:
                        messaging.send_message(f"Car not detected in range ({robot.lidar_mgr.get_angle(0)} mm)")
                    if not car_ready_to_charge:
                        messaging.send_message("Required car telemetry not met")
            
            if (time.time() - check_time > 60): # Only check every minute
                check_time = time.time()
                car_ready_to_charge = my_blue_panther.car_ready_to_be_charged()
                
            if (was_car_gone == False): # Handle resetting the car_gone flag
                if (robot.lidar_mgr.get_angle(0) > CarConfig.CAR_PARKED_DIST.target_distance + CarConfig.CAR_PARKED_DIST.threshold * 2 and time.time() - car_gone_start_time > 10):
                    car_gone_start_time = time.time()
                    logger.info("Lidar detected car has left!")
                    car_state = my_blue_panther.get_charging_related_data()
                    if (car_state is not None and car_state["is_at_charger"] == False):
                        logger.info("Car has left geofence")
                        messaging.send_message("Car has left geofence")
                        was_car_gone = True

            if (car_parked_in_distance and was_car_gone): # Automatic detection loop
                robot.leds.set_static(Colors.GREEN)
                car_idle_time = time.time() - car_detected_start_time
                if (car_idle_time > 2): # Only check every x number of seconds
                    car_detected_start_time = time.time() # Reset so we don't check continuously
                    if (my_blue_panther.car_ready_to_be_charged()):
                        logger.info("Car needs to be plugged in")
                        messaging.send_message("Car plugging in automatically!")
                        robot.state = RobotStates.INSERTING
            else:
                car_detected_start_time = time.time()
            
        
        elif robot.state == RobotStates.INSERTING:
            messaging.send_message("Plugging in!")
        
            robot.approach()
            
            robot.align(Movement, my_blue_panther)
            
            robot.insert_charger()

            messaging.send_message("Successfully Inserted")
            robot.state = RobotStates.IDLE_CHARGING
            
        elif robot.state == RobotStates.IDLE_CHARGING:
            time.sleep(0.1) # Actually might be important
            if (robot.button0_was_pushed() == True or msg_receiver.get_cmd() == "/unplug"):
                robot.leds.set_static(Colors.YELLOW)
                robot.state = RobotStates.REMOVING
                robot.removal_state = RemovalStates.REMOVING_CHARGER
            
        elif robot.state == RobotStates.REMOVING:
            messaging.send_message("Unplugging!")

            if (my_blue_panther.open_or_unlatch_charge_port() == False):
                logger.error("Failed to unlock charger")
                raise Exception("Failed to unlock charger")
        
            robot.remove_charger()

            robot.get_car_clearance()

            robot.home(Movement)
        
            robot.cam_servo.set_angle(ArmConfig.CAMINFRONTPOS)
            robot.leds.set_static(Colors.GREEN)
            time.sleep(1)
            robot.leds.set_static(Colors.BLACK)
            robot.state = RobotStates.IDLE_PARKED
            was_car_gone = False
            messaging.send_message("Successfully Homed")

except KeyboardInterrupt:
    robot.lidar_mgr.stop()

except Exception as e:
    import traceback
    traceback.print_exc()
    robot.lidar_mgr.stop()