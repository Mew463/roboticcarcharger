import board
import busio
inport math
import adafruit_adxl34x

class IMUSensor:
    def __init__(self):
        """Initializes the ADXL343 over I2C."""
        try:
            self.i2c = busio.I2C(board.SCL, board.SDA)
            self.sensor = adafruit_adxl34x.ADXL343(self.i2c)
     
            self.sensor.range = adafruit_adxl34x.Range.RANGE_2_G
            
        except Exception as e:
            print(f"IMU Error: Could not initialize ADXL343: {e}")
            self.sensor = None

    def get_acceleration(self):
        """Returns (x, y, z) acceleration in m/s^2."""
        if self.sensor:
            return self.sensor.acceleration
        return (0.0, 0.0, 0.0)

    def get_tilt(self):
        """
        Returns (roll, pitch) in degrees
        roll  = tilt around X axis
        pitch = tilt around Y axis
        """
        x, y, z = self.get_acceleration()

        # avoid divide-by-zero
        if z == 0:
            z = 1e-6

        # Roll (X tilt)
        roll = math.atan2(y, z)

        # Pitch (Y tilt)
        pitch = math.atan2(-x, math.sqrt(y**2 + z**2))

        return math.degrees(roll), math.degrees(pitch)