#!/usr/bin/python
# -*- coding: utf-8 -*-

from enum import Enum

class CalibrationStates(Enum):
    ERROR = 0
    INIT = 1
    RESETING = 2
    IDLE = 3

    RUNNING = 4

class CalibrationCommand(Enum):
    NONE = 0
    RESET = 1
    SERVOS_DISABLE = 2
    SERVOS_ENABLE = 3
    LASER_ON = 4
    LASER_OFF = 5
    RUN = 6
    SAVE = 7
    DONE = 8


    INITIALIZE_SERVO = 20
    CAMERA_SHARPNESS = 21
    CAMERAS_ALIGNMENT = 22
    CAMERA_LASER_ALIGNMENT = 23
    CAMERAS_ANGLE = 24
    VERTICAL_SERVO_ZERO = 25
    IMU_CALIBRATION = 26

    USE_LONG_CAMERA = 50
    USE_SHORT_CAMERA = 51