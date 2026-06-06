from enum import Enum, auto 

class InsertionStates(Enum):
    APPROACHING   = auto()
    ALIGNING      = auto()
    INSERTING_CHARGER = auto()
    CHARGING      = auto()

class RemovalStates(Enum):
    REMOVING_CHARGER = auto()
    CAR_CLEARANCE = auto()
    HOMING        = auto()

class RobotStates(Enum):
    IDLE_PARKED   = auto()
    INSERTING     = auto()
    IDLE_CHARGING = auto()
    REMOVING      = auto()
    FATALERROR    = auto()