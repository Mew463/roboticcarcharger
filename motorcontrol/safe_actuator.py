from abc import ABC, abstractmethod
"""
Base class of an actuator that is "safe"
A safe actuator has a clear fault condition, additionally enable and disable methods.

"""
class SafeActuatorBase(ABC):
    def __init__(self, logger):
        if logger is None:
            raise ValueError("Logger not provided!")
        self.logger = logger.getChild(self.__class__.__name__)
    
    @abstractmethod
    def monitor(self):
        pass

    @abstractmethod
    def disable(self):
        pass
    
    @abstractmethod
    def enable(self):
        pass