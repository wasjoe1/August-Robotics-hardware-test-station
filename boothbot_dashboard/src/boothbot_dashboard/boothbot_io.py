
import time
import rospy
from boothbot_driver.io_driver_client import (
    IODriverClient,
    IO_NUMBER
)

from boothbot_msgs.ros_interfaces import (
    DRIVERS_CHASSIS_IO,
)

class IO:
    def __init__(self):
        self._io_value = [False] * IO_NUMBER
        self._io_timeouts = [0.0] * IO_NUMBER
        self.io_client = IODriverClient()
        self.io_client.connect()

        rospy.Subscriber(
            DRIVERS_CHASSIS_IO.name,
            DRIVERS_CHASSIS_IO.type,
            self._chassis_io_cb)

    def _chassis_io_cb(self, msg):
        def is_set(x, n):
            return x & (1 << n) != 0
        value = int(msg.io_state)
        for i in range(IO_NUMBER):
            self._io_value[i] = is_set(value, i)

    def set_io(self, io_number, update=True, timeout=0.0):
        if not self._io_value[io_number]:
            self._io_value[io_number] = True
            self._io_timeouts[io_number] = timeout
            if update:
                self.update()

    def reset_io(self, io_number, update=True):
        if self._io_value[io_number]:
            self._io_value[io_number] = False
            if update:
                self.update()

    def toggle_io(self, io_number, update=True):
        self._io_value[io_number] = not self._io_value[io_number]
        if update:
            self.update()

    def update(self):
        set_io = self._io_value
        reset_io = [ not elem for elem in set_io ]
        self.io_client.send_goal(set_io=set_io, reset_io=reset_io, auto_reset_config=self._io_timeouts)
        while not self.io_client.is_done():
            time.sleep(0.1)
        # FIXME: sleep more 0.2s to aovoid io module to wrong state
        time.sleep(0.2)
        self._io_timeouts = [0.0] * IO_NUMBER

    def io_status_cb(self, msg):
        def is_set(x, n):
            return x & (1 << n) != 0

        value = int(msg.io_status)
        for i in range(IO_NUMBER):
            if is_set(value, i):
                self._io_value[i] = True
            else:
                self._io_value[i] = False
