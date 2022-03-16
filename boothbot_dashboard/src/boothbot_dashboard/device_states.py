from enum import Enum
from rich.text import Text
from rich import print

class DeviceStates(Enum):
    """
    Enum for device states
    """
    #ON = Text("打开/on", style='green')
    #OFF = Text("关闭/off", style='red')
    #OFFLINE = Text("离线/offline", style='grey46')
    OFFLINE = 0
    ON = 1
    OFF = 2
    UP = 3
    DOWN = 4
    HOLD = 5
    IN = 6
    OUT = 7
    PRESSED = 8
    RELEASED = 9
    UNKNOWN = 10

if __name__ == "__main__":
    print(DeviceStates.ON, DeviceStates.ON.name, DeviceStates.ON.value)
    print(DeviceStates.OFF, DeviceStates.OFF.name, DeviceStates.OFF.value)
    print(DeviceStates.OFFLINE, DeviceStates.OFFLINE.name, DeviceStates.OFFLINE.value)
