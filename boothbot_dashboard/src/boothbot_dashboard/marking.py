import os
import cv2
import time
import rospy

from device import Device
from device_states import DeviceStates
from device_module import DeviceModule, Image

from boothbot_msgs.ros_interfaces import (
    DRIVERS_CHASSIS_IO,
    MODULES_PERCEPT_CHECK_STATUS,
)

from boothbot_marking.settings import (
    BIT_EDGE_1,
    BIT_EDGE_2,
    BIT_EDGE_3,
    BIT_EDGE_4,
    BIT_LINEAR_LONG_DOWN,
    BIT_LINEAR_LONG_UP,
    BIT_BRUSH_DOWN,
    BIT_BRUSH_UP,
    BIT_STENCIL_OUT,
    BIT_STENCIL_IN,
    BIT_PUMP_IN_TRAY,
    BIT_DRAIN_OUT_TRAY,
)

from boothbot_perception.check_client import MarkChecker

class Marking(DeviceModule):
    def __init__(self, io, *args, **kwargs):
        super().__init__(*args, **kwargs)

        # Marking
        self.K1 = Device("<1> | K1")
        self.K2 = Device("<2> | K2")
        self.K3 = Device("<3> | K3")
        self.K4 = Device("<4> | K4")
        self.l_actuator = Device("<5> | Mark")
        self.s_actuator = Device("<6> | Stencil")
        self.in_pump = Device("<7> | In Pump")
        self.out_pump = Device("<8> | Out Pump")
        self.brush = Device("<9> | Brush")
        self.camera = Device("Marking Camera")

        self.append(self.K1)
        self.append(self.K2)
        self.append(self.K3)
        self.append(self.K4)
        self.append(self.l_actuator)
        self.append(self.s_actuator)
        self.append(self.in_pump)
        self.append(self.out_pump)
        self.append(self.brush)
        self.append(self.camera)

        self.io = io
        self.last_l_actuator_cmd = "up"
        self.last_s_actuator_cmd = "in"
        self.last_brush_cmd = "up"


        rospy.Subscriber(
            MODULES_PERCEPT_CHECK_STATUS.name,
            MODULES_PERCEPT_CHECK_STATUS.type,
            self._percept_check_status_cb)

        rospy.Subscriber(
            DRIVERS_CHASSIS_IO.name,
            DRIVERS_CHASSIS_IO.type,
            self._chassis_io_cb)

        self.l_actuator_up()
        self.stencil_in()
        self.brush_up()
        
    def _chassis_io_cb(self, msg):
        def is_set(x, n):
            return x & (1 << n) != 0
        value = int(msg.io_state)

        if is_set(value, BIT_EDGE_1):
            self.K1.state = DeviceStates.ON
        else:
            self.K1.state = DeviceStates.OFF

        if is_set(value, BIT_EDGE_2):
            self.K2.state = DeviceStates.ON
        else:
            self.K2.state = DeviceStates.OFF

        if is_set(value, BIT_EDGE_3):
            self.K3.state = DeviceStates.ON
        else:
            self.K3.state = DeviceStates.OFF

        if is_set(value, BIT_EDGE_4):
            self.K4.state = DeviceStates.ON
        else:
            self.K4.state = DeviceStates.OFF

        if is_set(value, BIT_PUMP_IN_TRAY):
            self.in_pump.state = DeviceStates.ON
        else:
            self.in_pump.state = DeviceStates.OFF

        if is_set(value, BIT_DRAIN_OUT_TRAY):
            self.out_pump.state = DeviceStates.ON
        else:
            self.out_pump.state = DeviceStates.OFF

        long_up = is_set(value, BIT_LINEAR_LONG_UP)
        long_down = is_set(value, BIT_LINEAR_LONG_DOWN)
        if long_up == long_down:
            self.l_actuator.state = DeviceStates.HOLD
        elif long_up and not long_down:
            self.l_actuator.state = DeviceStates.UP
        else:
            self.l_actuator.state = DeviceStates.DOWN

        brush_up = is_set(value, BIT_BRUSH_UP)
        brush_down = is_set(value, BIT_BRUSH_DOWN)
        if brush_up == brush_down:
            self.brush.state = DeviceStates.HOLD
        elif brush_up and not brush_down:
            self.brush.state = DeviceStates.UP
        else:
            self.brush.state = DeviceStates.DOWN

        stencil_in = is_set(value, BIT_STENCIL_IN)
        stencil_out = is_set(value, BIT_STENCIL_OUT)
        if stencil_in == stencil_out:
            self.s_actuator.state = DeviceStates.HOLD
        elif stencil_in and not stencil_out:
            self.s_actuator.state = DeviceStates.IN
        else:
            self.s_actuator.state = DeviceStates.OUT

        self.refresh()

    def _percept_check_status_cb(self, msg):
        if msg.state == "RUNNING":
            self.camera.state = DeviceStates.ON
        else:
            if os.path.exists("/dev/camera_marking"):
                self.camera.state = DeviceStates.OFF
            else:
                self.camera.state = DeviceStates.OFFLINE
        self.refresh()

    def toggle_k1(self):
        self.io.toggle_io(BIT_EDGE_1)

    def toggle_k2(self):
        self.io.toggle_io(BIT_EDGE_2)

    def toggle_k3(self):
        self.io.toggle_io(BIT_EDGE_3)

    def toggle_k4(self):
        self.io.toggle_io(BIT_EDGE_4)

    def toggle_in_pump(self):
        self.io.toggle_io(BIT_PUMP_IN_TRAY)

    def toggle_out_pump(self):
        self.io.toggle_io(BIT_DRAIN_OUT_TRAY)

    def l_actuator_up(self):
        self.last_l_actuator_cmd = "up"
        self.io.set_io(BIT_LINEAR_LONG_UP, update=False, timeout=2)
        self.io.reset_io(BIT_LINEAR_LONG_DOWN, update=False)
        self.io.update()

    def l_actuator_down(self):
        self.last_l_actuator_cmd = "down"
        self.io.set_io(BIT_LINEAR_LONG_DOWN, update=False, timeout=2)
        self.io.reset_io(BIT_LINEAR_LONG_UP, update=False)
        self.io.update()

    def toggle_l_actuator(self):
        if self.last_l_actuator_cmd == "down":
            self.l_actuator_up()
        else:
            self.l_actuator_down()

    def stencil_in(self):
        self.last_s_actuator_cmd = "in"
        self.io.set_io(BIT_STENCIL_IN, update=False, timeout=2)
        self.io.reset_io(BIT_STENCIL_OUT, update=False)
        self.io.update()

    def stencil_out(self):
        self.last_s_actuator_cmd = "out"
        self.io.set_io(BIT_STENCIL_OUT, update=False, timeout=2)
        self.io.reset_io(BIT_STENCIL_IN, update=False)
        self.io.update()

    def toggle_stencil(self):
        if self.last_l_actuator_cmd == "down":
            self.l_actuator_up()

        if self.last_s_actuator_cmd == "out":
            self.stencil_in()
        else:
            self.stencil_out()  

    def brush_up(self):
        self.io.set_io(BIT_BRUSH_UP, update=False, timeout=1)
        self.io.reset_io(BIT_BRUSH_DOWN, update=False)
        self.io.update()

    def brush_down(self):
        self.io.set_io(BIT_BRUSH_DOWN, update=False, timeout=1)
        self.io.reset_io(BIT_BRUSH_UP, update=False)
        self.io.update()

    def toggle_brush(self):
        if self.last_brush_cmd == "down":
            self.last_brush_cmd = "up"
            self.brush_up()
        else:
            self.last_brush_cmd = "down"

class MarkingCamera(Image):
    def __init__(self, *args, **kwargs):
        super().__init__(None, *args, **kwargs)
        self.checker = MarkChecker()
        self.checker.connect()

    def on_mount(self):
        self.set_interval(10, self.check)

    def wait_for_server(self, state, timeout):
        i = 0
        while self.checker.state != state and i < timeout*10:
            time.sleep(0.1)
            i += 1
        if i >= timeout*10:
            return False
        return True

    def check(self):
        self.checker.reset()
        if not self.wait_for_server("WAITING_BG", 1):
            return
        self.checker.check("0", "C-____",  self.on_check_result)
        if not self.wait_for_server("WAITING_BG", 1):
            return
        self.checker.capture_bg()
        #time.sleep(1)
        if not self.wait_for_server("WAITING_MK", 3):
            return
        self.checker.capture_mk()
        while self.checker.state != "WAITING_BG":
            time.sleep(1)


    def on_check_result(self, state, result):
        fname = result.fname
        if os.path.exists(fname):
            image = cv2.imread(fname)
            self.update(image)

