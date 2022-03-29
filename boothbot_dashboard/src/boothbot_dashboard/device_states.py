from enum import Enum
from rich.text import Text
from rich import print

class DeviceStates(Enum):
    """
    Enum for device states
    """
    OFFLINE = 0
    ON = 1
    OFF = 2
    PULL = 3
    PUSH = 4
    HOLD = 5
    ERROR = 6
    UNKNOWN = 7

if __name__ == "__main__":
    print(DeviceStates.ON, DeviceStates.ON.name, DeviceStates.ON.value)
    print(DeviceStates.OFF, DeviceStates.OFF.name, DeviceStates.OFF.value)
    print(DeviceStates.OFFLINE, DeviceStates.OFFLINE.name, DeviceStates.OFFLINE.value)
