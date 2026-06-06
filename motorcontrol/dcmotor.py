from gpiozero import PWMLED, LED, Button
from motorcontrol.safe_actuator import SafeActuatorBase
from settings import pins
import logging
import time
class DCMotor:
    def __init__(self, plusPin, minusPin, isReversed=False):
        self.plusMotor = PWMLED(plusPin)
        if minusPin is not None:
            self.minusMotor = PWMLED(minusPin)
        else:
            self.minusMotor = None
        self.isReversed = isReversed
    
    def setSpeed(self, value):
        if (self.isReversed):
            value = -value
        if (self.minusMotor is not None):
            if value >= 0:
                self.plusMotor.value = value
                self.minusMotor.value = 0
            elif value < 0:
                self.plusMotor.value = 0
                self.minusMotor.value = -value
        else:
            self.plusMotor.value = value
            
class Chassis(SafeActuatorBase):
    def __init__(self):
        self.logger = logging.getLogger(f"{__name__}")
        super().__init__(self.logger)
        self.frontRight = DCMotor(pins.CHASSIS_F_R_P, pins.CHASSIS_F_R_M, True)
        self.backRight = DCMotor(pins.CHASSIS_B_R_P, pins.CHASSIS_B_R_M, True)
        self.frontLeft = DCMotor(pins.CHASSIS_F_L_P, pins.CHASSIS_F_L_M)
        self.backLeft = DCMotor(pins.CHASSIS_B_L_P, pins.CHASSIS_B_L_M)
        self.enablePin = LED(pins.CHASSIS_EN)
        self.faultPin = Button(pins.CHASSIS_FAULT_L)
        
        self.current_vals = [0, 0, 0, 0]
    
    def _constrain(self, val, min_val, max_val):
        return max(min_val, min(val, max_val))
    
    def setSpeeds(self, fr, fl, br, bl):
        if (self.enablePin.value == False):
            self.logger.info("Motors disabled but trying to run")
            return
        self.frontRight.setSpeed(fr)
        self.backRight.setSpeed(br)
        self.frontLeft.setSpeed(fl)
        self.backLeft.setSpeed(bl)
    def enable(self):
        self.enablePin.on()
    def stop(self):
        self.current_vals[:] = [0] * len(self.current_vals)
        self.setSpeeds(0, 0, 0, 0)
    def smooth_stop(self):
        move_vector_smooth(0,0,0)
        
    def disable(self):
        self.stop()
        self.enablePin.off()
    def monitor(self):
        if (self.faultPin.value):
            self.logger.critical("Motor fault")
            
    def _targets_reached(self, target_vals, tolerance=0.01):
        return all(
            abs(current - target) <= tolerance
            for current, target in zip(self.current_vals, target_vals)
        )
    
    def move_vector_smooth(
        self,
        x,
        y,
        rot,
        update_delay=0.05,
        tolerance=0.01,
        timeout=5
    ):
        """
        Keep calling setVector until all wheel speeds
        reach their targets.
        """

        FRval = y + x + rot
        FLval = y - x - rot
        BRval = y - x + rot
        BLval = y + x - rot

        target_vals = [FRval, FLval, BRval, BLval]

        start_time = time.time()

        while not self._targets_reached(target_vals, tolerance):

            self.setVector(x, y, rot)

            if (time.time() - start_time) > timeout:
                self.logger.warning("moveVectorSmooth timeout")
                break

            time.sleep(update_delay)
    
    def setVector(self, x, y, rot):
        FRval = y + x + rot
        FLval = y - x - rot
        BRval = y - x + rot
        BLval = y + x - rot
        base_acceleration = 0.05
        max_acceleration = 0.3
        ramp_factor = 0.05
        
        target_vals = [FRval, FLval, BRval, BLval]
        
        for i, target_val in enumerate(target_vals):
            current_val = self.current_vals[i]
            # prevent divide-by-zero / explosion
            speed_mag = max(abs(current_val), 0.05)

            # nonlinear ramp
            adj_acceleration = base_acceleration + (
                ramp_factor / speed_mag
            )

            # cap acceleration
            adj_acceleration = min(adj_acceleration, max_acceleration)            
            if current_val < target_val:
                current_val = min(current_val + adj_acceleration, target_val)
            elif current_val > target_val:
                current_val = max(current_val - adj_acceleration, target_val)
           
            current_val = self._constrain(current_val, -1, 1)
            self.current_vals[i] = current_val

        self.setSpeeds(self.current_vals[0], self.current_vals[1], self.current_vals[2], self.current_vals[3])