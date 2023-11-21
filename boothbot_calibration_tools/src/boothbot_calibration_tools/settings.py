#!/usr/bin/python
# -*- coding: utf-8 -*-

from boothbot_calibration_tools.constants import CalibrationStates as MS
from boothbot_calibration_tools.constants import CalibrationCommand as CS

LEVEL_PORT = "/dev/incli"
LEVEL_INCLINOMETER_UNIT = 1
DEFAULT_TIMES_PER_2PI = 17
DEFAULT_STABLIZE_TIMEOUT = 5.0


JOB_DATA = {
    CS.INITIALIZE_SERVO.name: ["servo_h", "servo_v", "measurement_time"],
    CS.CAMERA_SHARPNESS.name: ["long_sharpness_score", "long_sharpness_result", "long_color_data", "long_color_result",
                               "short_sharpness_score", "short_sharpness_result", "short_color_data", "short_color_result"],
    CS.CAMERAS_ALIGNMENT.name: ["cameras_offset", "measurement_time", "cameras_offset_op_info"],
    CS.CAMERA_LASER_ALIGNMENT.name: ["camera_laser_alignment"],
    CS.CAMERAS_ANGLE.name: ["cameras_angle", "measurement_time"],
    CS.VERTICAL_SERVO_ZERO.name: ["vertical_offset", "measurement_time"],
    CS.IMU_CALIBRATION.name: ["measurement_time", "inclinometer_x", "imu_x", "imu_y", "imu_z", "imu_w", "inclinometer_y","base_offset_x","base_offset_y"],
    CS.HORIZONTAL_OFFSET.name: ["measurement_time", "horizontal_offset"],
    CS.MARKING_ROI.name: ["measurement_time", "x", "y", "w", "h"],
    CS.CB_INCLINATION.name: ["measurement_time", "cb_offset_x", "cb_offset_y", "base_offset_x", "base_offset_y", "roll", "pitch"],
    CS.DEPTH_CAMERA.name: ["measurement_time","roll", "pitch","yaw", "x", "y", "z", "tag_size", "has_set_tag"],
}

SAVE_DATA_TITLE = {
    CS.INITIALIZE_SERVO.name: ["servo_h", "servo_v", "measurement_time"],
    CS.CAMERA_SHARPNESS.name: ["long_sharpness_score", "long_sharpness_result", "long_color_data", "long_color_result",
                               "short_sharpness_score", "short_sharpness_result", "short_color_data", "short_color_result",
                               "measurement_time"],
    CS.CAMERAS_ALIGNMENT.name: ["cameras_offset", "measurement_time"],
    CS.CAMERA_LASER_ALIGNMENT.name: [],
    CS.CAMERAS_ANGLE.name: ["cameras_angle", "measurement_time"],
    CS.VERTICAL_SERVO_ZERO.name: ["vertical_offset", "measurement_time"],
    CS.IMU_CALIBRATION.name: ["measurement_time", "inclinometer_x", "inclinometer_y", "imu_x", "imu_y", "imu_z", "imu_w","base_offset_x","base_offset_y"],
    CS.HORIZONTAL_OFFSET.name: ["measurement_time", "horizontal_offset"],
    CS.MARKING_ROI.name: ["measurement_time", "x", "y", "w", "h"],
    CS.CB_INCLINATION.name: ["measurement_time", "cb_offset_x", "cb_offset_y", "base_offset_x", "base_offset_y", "roll", "pitch"],
    CS.DEPTH_CAMERA.name: ["measurement_time", "roll", "pitch","yaw", "x", "y", "z", "tag_size", "has_set_tag"],

}

JOS_SETTINGS = {
    CS.INITIALIZE_SERVO.name: {},
    CS.CAMERA_SHARPNESS.name: {"camera": "long", "exp_dis": {"long": 48, "short": 5}},
    CS.CAMERAS_ALIGNMENT.name: {"default_h": 1.38},
    CS.CAMERA_LASER_ALIGNMENT.name: {},
    CS.CAMERAS_ANGLE.name: {"default_h": 1.38},
    CS.VERTICAL_SERVO_ZERO.name: [1.57, -1.57],
    CS.IMU_CALIBRATION.name: {},
    CS.CB_INCLINATION.name: {},
    CS.DEPTH_CAMERA.name: {},
}

LAST_SAVE_TILE = [CS.INITIALIZE_SERVO.name,
                CS.CAMERAS_ANGLE.name,
                CS.VERTICAL_SERVO_ZERO.name,
                CS.CAMERA_SHARPNESS.name,
                CS.CAMERAS_ALIGNMENT.name,
                CS.HORIZONTAL_OFFSET.name,
                CS.MARKING_ROI.name,
                CS.CB_INCLINATION.name,
                CS.DEPTH_CAMERA.name,
                ]

JOB_DONE_STATUS = {
    CS.INITIALIZE_SERVO : 90,
    CS.CAMERA_SHARPNESS : 91,
    CS.CAMERAS_ALIGNMENT : 92,
    CS.CAMERA_LASER_ALIGNMENT : 93,
    CS.CAMERAS_ANGLE : 6,
    CS.VERTICAL_SERVO_ZERO : 5,
    CS.IMU_CALIBRATION : 94,
    CS.HORIZONTAL_OFFSET : 4,
    CS.MARKING_ROI : 4,
    CS.CB_INCLINATION : 3,
    CS.DEPTH_CAMERA : 3
}

# SETUP="SETUP"
CALI_ARG = "CALIBRATE"
SAVE_ARG = "SAVE"
LONG = "long"
SHORT = "short"
COLOR = "CALI"
CAMERA_FILTER_COUNT = 3

IMU_SERVICE = "/drivers/chassis/srv_cmd"
APPS_CALIBRATION_SET_PARAM = '/apps/calibration/set_param'


TRANS_BEACON = [0, 0, 0] # [0, 0, 0]
TRANS_BEACON_RCENTER  = [0, 0, 0]# [0, 0, 0]

LASER_HEIGHT = 0.78

# LIONEL INLCINATION
CB_INCLI_PORT_NAME = "/dev/incli"
LIONEL_INCLI_PORT_NAME = "/dev/incli_lionel"

TRANSITIONS_TOP = [
    {
        # Error state manual entrance
        "trigger": "to_ERROR",
        "source": "*",
        "dest": MS.ERROR,
        "unless": ["is_ERROR"],
    },
    {
        # now we reset the machinesrv_cmd_inf
        "trigger": "reset",
        "source": "*",
        "dest": MS.RESETING,
        "unless": ["is_RESETING"],
    },
    {
        # to idle state after reseting
        "trigger": "to_IDLE",
        "source": [MS.RESETING],
        "dest": MS.IDLE,
    },
    {
        # Will back to INIT if initialize failed
        "trigger": "to_INIT",
        "source": [MS.RESETING],
        "dest": MS.INIT,
    },
]

TRANSITIONS = TRANSITIONS_TOP + [
    {
        "trigger": "to_RUNNING",
        "source": MS.IDLE,
        "dest": MS.RUNNING,
    },
]

