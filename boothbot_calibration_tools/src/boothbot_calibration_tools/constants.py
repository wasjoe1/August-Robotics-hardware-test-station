#!/usr/bin/python
# -*- coding: utf-8 -*-

from enum import Enum

class CalibrationStates(Enum):
    ERROR = 0
    INIT = 1
    RESETING = 2
    IDLE = 3

    RUNNING = 4

class JobStatus(Enum):
    INIT = 0
    RUNNING = 1
    DONE = 2

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
    TEST_TRACK=9


    INITIALIZE_SERVO = 20
    CAMERA_SHARPNESS = 21
    CAMERAS_ALIGNMENT = 22
    CAMERA_LASER_ALIGNMENT = 23
    CAMERAS_ANGLE = 24
    VERTICAL_SERVO_ZERO = 25
    IMU_CALIBRATION = 26
    HORIZONTAL_OFFSET = 27
    MARKING_ROI = 28
    CB_INCLINATION = 29
    DEPTH_CAMERA = 30

    USE_LONG_CAMERA = 50
    USE_SHORT_CAMERA = 51

CB_INCLI_CMD = "/cb_incli/cmd"
CB_INCLI_STATE = "/cb_incli/state"
CB_INCLI_RES =  "/cb_incli/res"