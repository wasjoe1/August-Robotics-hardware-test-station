import os
import cv2
import math
import rospy
import ros_numpy
import numpy as np
from device_module import DeviceModule, Image
from device_states import DeviceStates
from device import Device
from boothbot_perception.track_client import TargetTracker
from boothbot_driver.stepper_client import StepperPlatformINF
from boothbot_common.error_code import ErrCode
from boothbot_msgs.ros_interfaces import (
    MODULES_PERCEPT_TRACK_LONG_IMAGE,
    MODULES_PERCEPT_TRACK_SHORT_IMAGE,
    MODULES_PERCEPT_TRACK_STATUS,
    DRIVERS_STEPPER_JOINT,
    DRIVERS_STEPPER_STATUS,
    DRIVERS_CHASSIS_IO,
)

from boothbot_driver.settings import (
    BIT_LED,
)

class Encoder(Device):
    def __init__(self, *args, **kwargs):
        super(Encoder, self).__init__(*args, **kwargs)
        self.encoder_value = 0

    @property
    def show_text(self):
        if self.state == DeviceStates.OFFLINE:
            return self.state.name
        else:
            return f"{self.state.name:7s} {math.degrees(self.encoder_value):.02f}\u00b0"

class CameraBeacon(DeviceModule):
    def __init__(self, io, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.io = io

        self.led = Device("<L> | LED")
        self.h_motor = Device("Horizontal Motor")
        self.h_encoder = Encoder("Horizontal Encoder")
        self.v_motor = Device("Vertical Motor")
        self.v_encoder = Encoder("Vertical Encoder")
        self.l_camera = Device("Long Camera")
        self.s_camera = Device("Short Camera")

        self.stepper = StepperPlatformINF()

        self.append(self.led)
        self.append(self.h_motor)
        self.append(self.h_encoder)
        self.append(self.v_motor)
        self.append(self.v_encoder)
        self.append(self.l_camera)
        self.append(self.s_camera)

        rospy.Subscriber(
            MODULES_PERCEPT_TRACK_STATUS.name,
            MODULES_PERCEPT_TRACK_STATUS.type,
            self._track_status_callback)

        rospy.Subscriber(
            DRIVERS_STEPPER_JOINT.name,
            DRIVERS_STEPPER_JOINT.type,
            self._stepper_joint_callback)

        rospy.Subscriber(
            DRIVERS_STEPPER_STATUS.name,
            DRIVERS_STEPPER_STATUS.type,
            self._stepper_status_callback)

        rospy.Subscriber(
            DRIVERS_CHASSIS_IO.name,
            DRIVERS_CHASSIS_IO.type,
            self._chassis_io_cb)

    def _chassis_io_cb(self, msg):
        def is_set(x, n):
            return x & (1 << n) != 0
        value = int(msg.io_state)

        if is_set(value, BIT_LED):
            self.led.state = DeviceStates.ON
        else:
            self.led.state = DeviceStates.OFF

    def _stepper_joint_callback(self, msg):
        if len(msg.enc_val) == 2:
            self.h_encoder.encoder_value= msg.enc_pos[0]
            self.v_encoder.encoder_value = msg.enc_pos[1]

    def _stepper_status_callback(self, msg):
        if ErrCode.STEPPER_ERR_H_MOTOR.value in msg.errorcodes:
            self.h_motor.state = DeviceStates.OFFLINE
        else:
            self.h_motor.state = DeviceStates.ON

        if ErrCode.STEPPER_ERR_V_MOTOR.value in msg.errorcodes:
            self.v_motor.state = DeviceStates.OFFLINE
        else:
            self.v_motor.state = DeviceStates.ON

        if ErrCode.STEPPER_ERR_H_ENCODER.value in msg.errorcodes:
            self.h_encoder.state = DeviceStates.OFFLINE
        else:
            self.h_encoder.state = DeviceStates.ON

        if ErrCode.STEPPER_ERR_V_ENCODER.value in msg.errorcodes:
            self.v_encoder.state = DeviceStates.OFFLINE
        else:
            self.v_encoder.state = DeviceStates.ON

    def _track_status_callback(self, msg):
        if msg.state == "RUNNING":
            self.l_camera.state = DeviceStates.ON
            self.s_camera.state = DeviceStates.ON
        else:
            if os.path.exists("/dev/camera_long"):
                self.l_camera.state = DeviceStates.OFF
            else:
                self.l_camera.state = DeviceStates.OFFLINE

            if os.path.exists("/dev/camera_short"):
                self.s_camera.state = DeviceStates.OFF
            else:
                self.s_camera.state = DeviceStates.OFFLINE

    def toggle_led(self):
        self.io.toggle_io(BIT_LED)

    def move(self, direction, degrees=5):
        if self.h_motor.state == DeviceStates.OFFLINE or self.v_motor.state == DeviceStates.OFFLINE:
            return

        h = self.h_encoder.encoder_value
        v = self.v_encoder.encoder_value

        if direction == "up":
            v -= math.radians(degrees)
            if v <= math.radians(-8):
                v = math.radians(-8)
        elif direction == "down":
            v += math.radians(degrees)
            if v >= math.radians(8):
                v = math.radians(8)
        elif direction == "left":
            h += math.radians(degrees)
            if h >= math.radians(180):
                h = math.radians(180)
        elif direction == "right":
            h -= math.radians(degrees)
            if h <= -math.radians(180):
                h = -math.radians(180)

        self.stepper.move_absolute(h, v)

    def on_mount(self):
        self.set_interval(1, self.refresh)

class TrackingCameras(Image):
    def __init__(self, *args, **kwargs):
        super().__init__(None, *args, **kwargs)

        self.long_image = None
        self.short_image = None
        self.tracker = TargetTracker()
        self.tracker.connect()
        self.tracker.capture(True)

        rospy.Subscriber(
            MODULES_PERCEPT_TRACK_STATUS.name,
            MODULES_PERCEPT_TRACK_STATUS.type,
            self._track_status_callback)

        rospy.Subscriber(
            MODULES_PERCEPT_TRACK_LONG_IMAGE.name,
            MODULES_PERCEPT_TRACK_LONG_IMAGE.type,
            self._track_long_image_callback)

        rospy.Subscriber(
            MODULES_PERCEPT_TRACK_SHORT_IMAGE.name,
            MODULES_PERCEPT_TRACK_SHORT_IMAGE.type,
            self._track_short_image_callback)

    def _track_status_callback(self, msg):
        if msg.state == "RUNNING":
            self.state = DeviceStates.ON
        else:
            self.state = DeviceStates.ERROR

    def _track_long_image_callback(self, msg):
        self.long_image = ros_numpy.numpify(msg)

    def _track_short_image_callback(self, msg):
        self.short_image = ros_numpy.numpify(msg)

    def concat_images(self):
        if self.long_image is None or self.short_image is None:
            return
        image = np.concatenate((self.long_image, self.short_image), axis=0)
        self.update(image)

    def track(self):
        if self.state == DeviceStates.ON:
            self.tracker.send_goal()

    def on_mount(self):
        self.set_interval(10, self.track)
        self.set_interval(10, self.concat_images)
