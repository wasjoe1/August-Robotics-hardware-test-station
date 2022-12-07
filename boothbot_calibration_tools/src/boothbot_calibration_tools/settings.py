#!/usr/bin/python
# -*- coding: utf-8 -*-

from boothbot_calibration_tools.constants import CalibrationStates as MS
from boothbot_calibration_tools.constants import CalibrationCommand as CS

LEVEL_PORT = "/dev/incli"
LEVEL_INCLINOMETER_UNIT = 1

JOB_DATA = {
    CS.INITIALIZE_SERVO.name: ["servo_h", "servo_v", "measurement_time"],
    CS.CAMERA_SHARPNESS.name: ["long_sharpness_score", "long_sharpness_result", "long_color_data", "long_color_result",
                               "short_sharpness_score", "short_sharpness_result", "short_color_data", "short_color_result"],
    CS.CAMERAS_ALIGNMENT.name: ["cameras_offset", "measurement_time", "cameras_offset_op_info"],
    CS.CAMERA_LASER_ALIGNMENT.name: ["camera_laser_alignment"],
    CS.CAMERAS_ANGLE.name: ["cameras_angle", "measurement_time"],
    CS.VERTICAL_SERVO_ZERO.name: ["vertical_offset", "measurement_time"],
    CS.IMU_CALIBRATION.name: ["measurement_time", "inclinometer_x", "imu_x", "imu_y", "imu_z", "imu_w", "inclinometer_y","offset_x","offset_y"],
    CS.HORIZONTAL_OFFSET.name: ["measurement_time", "horizontal_offset"],
    CS.MARKING_ROI.name: []
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
    CS.IMU_CALIBRATION.name: ["measurement_time", "inclinometer_x", "inclinometer_y", "imu_x", "imu_y", "imu_z", "imu_w","offset_x","offset_y"],
    CS.HORIZONTAL_OFFSET.name: ["measurement_time", "horizontal_offset"],
    CS.MARKING_ROI.name: []
}

JOS_SETTINGS = {
    CS.INITIALIZE_SERVO.name: {},
    CS.CAMERA_SHARPNESS.name: {"camera": "long", "exp_dis": {"long": 40, "short": 5}},
    CS.CAMERAS_ALIGNMENT.name: {"default_h": 1.38},
    CS.CAMERA_LASER_ALIGNMENT.name: {},
    CS.CAMERAS_ANGLE.name: {"default_h": 1.38},
    CS.VERTICAL_SERVO_ZERO.name: [1.57, -1.57],
    CS.IMU_CALIBRATION.name: {},
}

LAST_SAVE_TILE = [CS.INITIALIZE_SERVO.name,
                CS.CAMERAS_ANGLE.name,
                CS.VERTICAL_SERVO_ZERO.name,
                CS.CAMERA_SHARPNESS.name,
                CS.CAMERAS_ALIGNMENT.name,
                CS.HORIZONTAL_OFFSET.name,
                CS.MARKING_ROI.name]

# SETUP="SETUP"
CALI_ARG = "CALIBRATE"
SAVE_ARG = "SAVE"

IMU_SERVICE = "/drivers/chassis/srv_cmd"

# CMD="roslaunch ../launch/base_production.launch"
# RECORD="rostopic echo -c /drivers/chassis/imu"
# SETUPCMD="rosservice call /drivers/chassis/srv_cmd '{command: "IMU", parameter: ${SETUP}}'"
# CALICMD="rosservice call /drivers/chassis/srv_cmd '{command: "IMU", parameter: ${CALI_ARG}}'"
# SAVECMD="rosservice call /drivers/chassis/srv_cmd '{command: "IMU", parameter: ${SAVE_ARG}}'"


TRANS_BEACON = [0, 0, 0] # [0, 0, 0]
TRANS_BEACON_RCENTER  = [0, 0, 0]# [0, 0, 0]

LASER_HEIGHT = 0.78

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
