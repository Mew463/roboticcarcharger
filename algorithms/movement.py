from collections import deque
from statistics import median
import time
import logging 

class PositionController:
    def __init__(self, kP, kI, kD, far_distance_threshold, seconds_per_unit):
        """_summary_

        Args:
            kP (float): P value
            kI (float): I value (scaled by 0.001)
            kD (float): D value
            far_distance_threshold (float): After this threshold, a max value will be returned instead of a PID calcluation
            seconds_per_unit (float): Rate of change, used to determine how long the max value should be returned for
        """
        self.kP = kP
        self.kI = kI * 0.001 # Perform scaling
        self.kD = kD
        self.last_error = 0
        self.integral = 0
        self.far_distance_threshold = far_distance_threshold
        self.seconds_per_unit = seconds_per_unit
        self.translation_end_time = 0
        self.max_pwr_value = 1
        self.the_max_value = self.max_pwr_value
        self.logger = logging.getLogger(f"{__name__}")
        
    def do_translate(self, error):
        """_summary_

        Args:
            error (float): The calculated error

        Returns:
            float: A value that can be given directly to the chassis motors
        """
        now = time.time()
        
        if (error is not None):
            if (error < 0):
                self.the_max_value = -self.max_pwr_value
            else:
                self.the_max_value = self.max_pwr_value
            
            if (abs(error) > self.far_distance_threshold):
                translate_time = abs(error) * (self.seconds_per_unit)
                self.translation_end_time = time.time() + translate_time
                self.logger.info(f"Set translation end time for: {translate_time}")
                return self.the_max_value
            if (now > self.translation_end_time):
                return self._calc_PID(error)
            else:
                return self.the_max_value
            
        else:
            if (now < self.translation_end_time):
                return self.the_max_value
            else:
                return 0
        
    def reset(self):
        """_summary_
        Resets all calculations that are time-related
        """
        self.last_error = 0
        self.integral = 0
        self.translation_end_time = 0

    def _calc_PID(self, error):
        """_summary_
        PID Calculation
        """
        max_i = 0.3
        if (self.integral * self.kI > max_i):
            self.integral = max_i / self.kI
        if (self.integral * self.kI < -max_i):
            self.integral = -max_i / self.kI
        self.integral += error
        
        derivative = (error - self.last_error) 
        self.last_error = error
        return self.kP * error + self.kI * self.integral + self.kD * derivative


from dataclasses import dataclass
@dataclass(frozen=True)
class TrackingTypes:
    POSE = 0
    POSE_EST = 1
    BRIEF_LOST = 2
    LOST = 3
    
TRACKING_NAMES = {
    TrackingTypes.POSE: "POSE",
    TrackingTypes.POSE_EST: "POSE_EST",
    TrackingTypes.BRIEF_LOST: "BRIEF_LOST",
    TrackingTypes.LOST: "LOST",
}

from settings.config import ChassisConfig as CFG
class FusedMovement:
    def __init__(self, chassis, camera_tracker):
        self.myTracker = camera_tracker
        self.chassis = chassis
        self.logger = logging.getLogger(f"{__name__}")
        self.errorx = self.errory = self.erroryaw = 999
        
        self.PositionControllerX = PositionController(CFG.kP_x, CFG.kI_x, 0, 
                                                      CFG.farDistanceThresholdX, 
                                                      CFG.secondsPerMeterX)
        self.PositionControllerY = PositionController(CFG.kP_y, CFG.kI_y, 0, 
                                                      CFG.farDistanceThresholdY, 
                                                      CFG.secondsPerMeterY) 
        self.RotationControllerYaw = PositionController(CFG.kP_yaw, CFG.kI_yaw, 0, 
                                                      CFG.farAngleThresholdYaw, 
                                                      CFG.secondsPerAngle) 
        self.is_success_tracker = deque (maxlen = 10)
        self.am_lost = deque(maxlen = 60)
        self.z_vals = deque(maxlen = 20)
        self.am_lost.append(False)
        self.last_tracking_type = None
        
    def _determine_tracking_mode(self, charuco_res, loss_ratio):
        if (charuco_res["pose"] is not None):
            tracking_type = TrackingTypes.POSE
            self.am_lost.append(False)
        else:
            if loss_ratio > 0.5:
                if (charuco_res["pose_est"] is not None):
                    tracking_type = TrackingTypes.POSE_EST
                else:
                    tracking_type = TrackingTypes.LOST
            else: # loss_ratio < 0.5
                tracking_type = TrackingTypes.BRIEF_LOST
                self.am_lost.append(True)
        if (self.last_tracking_type != tracking_type): # Only print right when we switch
            self.logger.info(f"Using tracking type: {TRACKING_NAMES[tracking_type]}")
        self.last_tracking_type = tracking_type
        return tracking_type
    
    def get_z_median(self):
        if len(self.z_vals) == self.z_vals.maxlen:
            med = median(self.z_vals)
            return med
        
    def move_to_tag_position(self, x, y, yaw, use_wall_board : bool):
        charuco_res = self.myTracker.get_frame(use_wall_board)
        dir_mult = -1 if use_wall_board else 1
        
        loss_ratio = sum(self.am_lost) / len(self.am_lost)
        
        tracking_type = self._determine_tracking_mode(charuco_res, loss_ratio)
        
        if (tracking_type == TrackingTypes.POSE):
            self.am_lost.append(False)
            res = charuco_res["pose"]
            self.errorx = x - res["x"]
            self.errory = y - res["y"]
            self.erroryaw = yaw - res["yaw"]
            self.z_vals.append(res["z"])
            self.chassis.setVector(self.PositionControllerX.do_translate(self.errorx) * 1 * dir_mult, 
                                   self.PositionControllerY.do_translate(self.errory) * -1 * dir_mult, 
                                   self.RotationControllerYaw.do_translate(self.erroryaw) * 1 * dir_mult)
        elif (tracking_type == TrackingTypes.POSE_EST):
            res = charuco_res["pose_est"]
            max_val = 0.5
            x_pwr = min(max_val, max(res["x"] * -1 * dir_mult, -max_val))
            y_pwr = min(max_val, max((res["y"] - y) * dir_mult, -max_val))
            self.chassis.setVector(x_pwr, y_pwr, 0)
        elif (tracking_type == TrackingTypes.BRIEF_LOST):
            self.am_lost.append(True)
            self.chassis.setVector(self.PositionControllerX.do_translate(None) * 1 * dir_mult, 
                                self.PositionControllerY.do_translate(None) * -1 * dir_mult, 
                                self.RotationControllerYaw.do_translate(None) * 1 * dir_mult)
        elif (tracking_type == TrackingTypes.LOST):
            self.chassis.stop()
        return self
    
    def reset_errors(self):
        self.errorx = self.errory = self.erroryaw = 999
        self.am_lost.append(False)
        self.is_success_tracker.clear()
        self.PositionControllerX.reset()
        self.PositionControllerY.reset()
        self.RotationControllerYaw.reset()

    def is_success(self, precision_multiplier = 1):
        was_success = (abs(self.errorx) < CFG.positionAccuracyX * precision_multiplier and 
            abs(self.errory) < CFG.positionAccuracyY * precision_multiplier and
            abs(self.erroryaw) < CFG.yawAccuracy * precision_multiplier)
        self.is_success_tracker.append(was_success)
        if len(self.is_success_tracker) < self.is_success_tracker.maxlen:
            return False
        for success in self.is_success_tracker:
            if (not success):
                return False
        
        self.reset_errors()
        return True