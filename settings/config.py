from dataclasses import dataclass
import numpy as np 

@dataclass(frozen=True)
class ChassisConfig:
    ## PID CONFIG
    kP_x = 10
    kP_y = 10
    kP_yaw = 0.1
    
    kI_x = 0.1
    kI_y = 0.1
    kI_yaw = 2
    
    ## ACCURACY
    positionAccuracyX = 0.01
    positionAccuracyY = 0.01
    yawAccuracy = 0.5
    
    farDistanceThresholdX = 0.1
    secondsPerMeterX = 6.8
    
    farDistanceThresholdY = 0.1
    secondsPerMeterY = 5.77
    
    farAngleThresholdYaw = 999
    secondsPerAngle = 3.3/90

class ElevatorConfig:
    STEPS_PER_CM = 418.75
    CHARGE_PORT_HEIGHT_CM = 23.5

class ArmConfig:
    sucMotorThresholdCurrent = 2320
    servoThresholdcurrent = 2800
    
    chargerServoStartPos = 157
    chargerServoEndPos = 39
    
    CAMBEHINDPOS = 45
    CAMINFRONTPOS = 170
    
class CameraConfig:
    ### 99$ Camera 720p 60fps
    # K = np.array( 
    # [
    #     [500.7157106383602, 0.0, 682.3971711610658],
    #     [0.0, 499.69894450486436, 386.46595623898685],
    #     [0.0, 0.0, 1.0]
    # ], dtype=np.float32)
    
    # dist = np.array([-0.030042772960827475, -0.03431442937850516, 0.00026535055662068333, -0.0013406833405413719, 0.009221957389354092], dtype=np.float32)  
    
    ### Camera at 1080p 30fps    
    K = np.array( 
    [
        [773.442885134213, 0.0, 1024.8263951684214],
        [0.0, 771.3313880947502, 583.8150132012087],
        [0.0, 0.0, 1.0]
    ], dtype=np.float32)
    
    dist = np.array([-0.01196480038446067, -0.04646810566273238, -0.000800085816789049, -0.000435858940564193, 0.01317977322544761], dtype=np.float32)  
    INITIAL_BRIGHTNESS = 0
    SETPOINT_BRIGHTNESS = 35

@dataclass
class DistanceThreshold:
    target_distance: int
    threshold: int

class CarConfig:
    GEOFENCE_RADIUS_METERS = 10
    CAR_PARKED_DIST = DistanceThreshold(
        target_distance=1258,
        threshold=350
    )
    CAR_PLUGGED_DIST = DistanceThreshold(
        target_distance=98.5,
        threshold=20
    )
    MAX_MOVING_DIST = 1500
    
class MessagingConfig:
    HELP_MESSAGE = """Welcome to the robotic car charger!
    
    /charge = charge car
    
    /unplug = unplug car
    
    /kill   = Kill program
    
    /photo  = Take a photo
    """
    
class Colors:
    BLACK = 0
    RED = 1
    GREEN = 2
    BLUE = 3
    YELLOW = 4
    PURPLE = 5
    WHITE = 500