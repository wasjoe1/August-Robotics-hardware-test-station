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

class CtrlIO(Device):
    def __init__(self, io, bit, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.io = io
        self.bit = bit

    def toggle(self):
        self.io.toggle_io(self.bit)

    def update(self, value):
        def is_set(x, n):
            return x & (1 << n) != 0
        self.state = DeviceStates.ON if is_set(value, self.bit) else DeviceStates.OFF

class CtrlIOPair(Device):
    def __init__(self, io, push_bit, pull_bit,  *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.io = io
        self.push_bit = push_bit
        self.pull_bit = pull_bit
        self.pushed = False

    def push(self):
        if not self.pushed:
            self.io.set_io(self.push_bit, update=False, timeout=2)
            self.io.reset_io(self.pull_bit, update=False)
            self.io.update()
            self.pushed = True

    def pull(self):
        if self.pushed:
            self.io.set_io(self.pull_bit, update=False, timeout=2)
            self.io.reset_io(self.push_bit, update=False)
            self.io.update()
            self.pushed = False

    def update(self, value):
        def is_set(x, n):
            return x & (1 << n) != 0
        pull_state = is_set(value, self.pull_bit)
        push_state = is_set(value, self.push_bit)

        if pull_state == push_state:
            self.state = DeviceStates.HOLD
        elif pull_state:
            self.state = DeviceStates.PULL
        else:
            self.state = DeviceStates.PUSH

    def toggle(self):
        if self.pushed:
            self.pull()
        else:
            self.push()

class LActuator(CtrlIOPair):
    def __init__(self, io, *args, **kwargs):
        super().__init__(io, BIT_LINEAR_LONG_DOWN, BIT_LINEAR_LONG_UP, *args, **kwargs)
        self.up = self.pull

    @property
    def show_text(self):
        if self.state == DeviceStates.HOLD:
            return "HOLD"
        elif self.state == DeviceStates.PULL:
            return "PUSHING DOWN"
        else:
            return "PULLING UP"


class SActuator(CtrlIOPair):
    def __init__(self, io, *args, **kwargs):
        super().__init__(io, BIT_STENCIL_OUT, BIT_STENCIL_IN, *args, **kwargs)

    @property
    def show_text(self):
        if self.state == DeviceStates.HOLD:
            return "HOLD"
        elif self.state == DeviceStates.PULL:
            return "PULLING IN"
        else:
            return "PUSHING OUT"


class Brush(CtrlIOPair):
    def __init__(self, io, *args, **kwargs):
        super().__init__(io, BIT_BRUSH_DOWN, BIT_BRUSH_UP, *args, **kwargs)

    @property
    def show_text(self):
        if self.state == DeviceStates.HOLD:
            return "HOLD"
        elif self.state == DeviceStates.PULL:
            return "UP"
        else:
            return "DOWN"


class CameraState(Device):
    def __init__(self, device, message, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.device = device

        rospy.Subscriber(
            message.name,
            message.type,
            self._cb)

    def _cb(self, msg):
        if msg.state == "RUNNING":
            self.state = DeviceStates.ON
        else:
            if os.path.exists(self.device):
                self.state = DeviceStates.OFF
            else:
                self.state = DeviceStates.OFFLINE

class Marking(DeviceModule):
    def __init__(self, io, *args, **kwargs):
        super().__init__(*args, **kwargs)

        # Marking
        self.k1         = CtrlIO(io, BIT_EDGE_1, name="<1> | K1")
        self.k2         = CtrlIO(io, BIT_EDGE_2, name="<2> | K2")
        self.k3         = CtrlIO(io, BIT_EDGE_3, name="<3> | K3")
        self.k4         = CtrlIO(io, BIT_EDGE_4, name="<4> | K4")
        self.l_actuator = LActuator(io, name="<5> | Mark")
        self.s_actuator = SActuator(io, name="<6> | Stencil")
        self.in_pump    = CtrlIO(io, BIT_PUMP_IN_TRAY,   name="<7> | In Pump")
        self.out_pump   = CtrlIO(io, BIT_DRAIN_OUT_TRAY, name="<8> | Out Pump")
        self.brush      = Brush(io, name="<9> | Brush")
        self.camera     = CameraState("/dev/camera_marking", MODULES_PERCEPT_CHECK_STATUS, "Marking Camera")

        self.append(self.k1)
        self.append(self.k2)
        self.append(self.k3)
        self.append(self.k4)
        self.append(self.l_actuator)
        self.append(self.s_actuator)
        self.append(self.in_pump)
        self.append(self.out_pump)
        self.append(self.brush)
        self.append(self.camera)

        rospy.Subscriber(
            DRIVERS_CHASSIS_IO.name,
            DRIVERS_CHASSIS_IO.type,
            self._chassis_io_cb)

        self.l_actuator.pull()
        self.s_actuator.pull()
        self.brush.pull()

        self.last_io_state = 0

    def on_mount(self):
        self.set_interval(1, self.refresh)

    def _chassis_io_cb(self, msg):
        value = int(msg.io_state)
        if value != self.last_io_state:
            self.k1.update(value)
            self.k2.update(value)
            self.k3.update(value)
            self.k4.update(value)
            self.in_pump.update(value)
            self.out_pump.update(value)
            self.l_actuator.update(value)
            self.s_actuator.update(value)
            self.brush.update(value)
            self.last_io_state = value
            self.refresh()

    def toggle_k1(self):
        self.k1.toggle()

    def toggle_k2(self):
        self.k2.toggle()

    def toggle_k3(self):
        self.k3.toggle()

    def toggle_k4(self):
        self.k4.toggle()

    def toggle_in_pump(self):
        self.in_pump.toggle()

    def toggle_out_pump(self):
        self.out_pump.toggle()

    def toggle_l_actuator(self):
        self.l_actuator.toggle()

    def toggle_stencil(self):
        self.l_actuator.up()
        self.s_actuator.toggle()

    def toggle_brush(self):
        self.brush.toggle()


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

    # Trigger the makring camera to take a picture
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

