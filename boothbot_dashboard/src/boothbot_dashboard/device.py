
from device_states import DeviceStates
from rich.text import Text

class Device:
    def __init__(self, name):
        self._label = name 
        #self._label = Text(name) 
        self._state = DeviceStates.OFFLINE

    @property
    def label(self):
        return self._label

    @property
    def state(self):
        return self._state

    @state.setter
    def state(self, value: DeviceStates):
        self._state = value

    @property
    def show_text(self):
        return self._state.name

if __name__ == "__main__":
    import unittest

    class TestDevice(unittest.TestCase):
        def test_device_states(self):
            device = Device("test")
            #self.assertEqual(device.label, Text("test"))
            self.assertEqual(device.label, "test")
            self.assertEqual(device.state, DeviceStates.OFFLINE)
            device.state = DeviceStates.ON
            self.assertEqual(device.state, DeviceStates.ON)
            device.state = DeviceStates.OFF
            self.assertEqual(device.state, DeviceStates.OFF)
            device.state = DeviceStates.OFFLINE
            self.assertEqual(device.state, DeviceStates.OFFLINE)

    unittest.main()

