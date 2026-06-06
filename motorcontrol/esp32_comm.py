import serial
import logging 

class Esp32CommBase():
    def __init__(self, ser):
        self.ser = ser
        self.logger = logging.getLogger(
            f"{__name__}.{self.__class__.__name__}"
        )

    # Type = R, W, S, E,  Pin   , # value
    def formulateCommand(self, type: str, pin: int, value: int):
        if len(type) != 1:
            raise ValueError("Must be 1 char")
        return (f"{type}:{pin:02d}:{value}|")
    
    def getChecksum(self, message: str) -> str:
        checksum = 0
        for char in message:
            checksum ^= ord(char)
        return f"{checksum:02X}"
    
    def sendCommand(self, type: str, pin: int, value: int, expect_data: bool = False, wait_for_completion: bool = True):
        msg = self.formulateCommand(type, pin, value)
        msg += self.getChecksum(msg) + '\n'
        # self.logger.debug(f"Sending {msg}")
        self.ser.write(msg.encode())
        if (wait_for_completion):
            line = self.ser.readline().decode().strip()
            if (not line):
                self.logger.error("Did not receive an ESP32 response")
            if (line.find("*") != -1): # We have a log message
                LOG_MAP = {
                    'D': logging.DEBUG,
                    'I': logging.INFO,
                    'W': logging.WARNING,
                    'E': logging.ERROR,
                    'C': logging.CRITICAL,
                }
                self.logger.log(LOG_MAP.get(line[1]), line[3:])
            if expect_data:
                return line
        
