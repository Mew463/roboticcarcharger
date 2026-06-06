from motorcontrol.esp32_comm import Esp32CommBase
from motorcontrol.safe_actuator import SafeActuatorBase
from gpiozero import LED
from settings.config import ArmConfig, Colors, ElevatorConfig

class Servo(Esp32CommBase):
    def __init__(self, ser, pin):
        super().__init__(ser)
        self.pin = pin

    def set_angle(self, angle: int):
        if not (0 <= angle <= 180):
            self.logger.warning(f"Angle {angle} out of range")
            angle = max(0, min(180, angle))
        
        super().sendCommand('S', self.pin, angle, wait_for_completion = False) # Make it go faster for the higher power servo

class Esp32AnalogRead(Esp32CommBase):
    def __init__(self, ser, pin):
        super().__init__(ser)
        self.pin = pin
        self.value = 0
    def analogRead(self) -> int:
        val = super().sendCommand('R', self.pin, self.value, expect_data = True)
        try: 
            newval = int(val)
            return newval
        except ValueError:
            return None
        
class HighPowerServo(Servo, SafeActuatorBase):
    def __init__(self, ser, sig_pin, enable_pin, current_read_obj: Esp32AnalogRead):
        super().__init__(ser, sig_pin)
        self.enable_pin = LED(enable_pin)
        self.current_read_obj = current_read_obj
    def set_angle(self, angle: int):
        if (not self.enable_pin.value):
            self.logger.warning("Angle set without being enabled!")
        super().set_angle(angle)
        
    def monitor(self): # Placeholder
        if (self.current_read_obj.analogRead() > ArmConfig.servoThresholdcurrent):
            pass
        
    def enable(self):
        self.enable_pin.on()
        
    def disable(self):
        self.enable_pin.off()

class Stepper(Esp32CommBase):
    def __init__(self, ser):
        super().__init__(ser)
        self.curPos = None
    
    def moveTo(self, height_cm: int):
        self.curPos = height_cm * ElevatorConfig.STEPS_PER_CM
        super().sendCommand('E', 0, self.curPos)

    def home(self, speed: int):
        self.curPos = 0
        super().sendCommand('E', 1, speed)

class RGBLeds(Esp32CommBase):
    def __init__(self, ser):
        super().__init__(ser)
    def set_breathing(self, color : Colors):
        super().sendCommand('L', 0, color)
    def set_circle(self, color : Colors):
        super().sendCommand('L', 1, color)
    def set_static(self, color : Colors, brightness : int = 100):
        if color == Colors.WHITE:
            super().sendCommand('L', 2, color + brightness)
        else: 
            super().sendCommand('L', 2, color)