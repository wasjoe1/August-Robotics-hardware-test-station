#!/usr/bin/python3
# -*- coding: utf-8 -*-

import time
import math
from pprint import PrettyPrinter
import rospy
logger = rospy
import binascii

from serial import Serial
from std_msgs.msg import String
from meterial_inspection_tools.srv import IMUcontrol

from meterial_inspection_tools.ros_interface import (
    IMU_DATA,
    IMU_SRV_CMD
)
from boothbot_msgs.srv import (Command, CommandRequest, CommandResponse)
import json


# Settings 
# Refer to section 12.2 datasheet
NODE_RATE = 10
MOTOR_TOLERANCE = 0.0001
FIXED_BAUDRATE = 115200

WIT_IMU_CMD_CALI_ACC = b"\xff\xaa\x01\x01\x00" 
WIT_IMU_CMD_SAVE_CFG = b"\xff\xaa\x00\x00\x00"
WIT_IMU_CMD_UNLOCK = b"\xff\xaa\x69\x88\xb5"
WIT_IMU_CMD_SET_BAUDRATE = b"\xff\xaa\x04\x06\x00"  # 115200

ACC_SPEED = {
    "2": b"\xff\xaa\x21\x00\x00", #2g
    "4": b"\xff\xaa\x21\x01\x00", #4g
    "8": b"\xff\xaa\x21\x02\x00", #8g
    "16": b"\xff\xaa\x21\x03\x00", #16g
}

#double check GYRO DEGREES
GYRO_DEGREES = { 
    "250": b"\xff\xaa\x20\x00\x00",
    "500": b"\xff\xaa\x20\x01\x00",
    "1000": b"\xff\xaa\x20\x02\x00",
    "2000": b"\xff\xaa\x20\x03\x00", 
}


BANDWIDTH_HZ = {
    "200": b"\xff\xaa\x1f\x01\x00",
    "98" : b"\xff\xaa\x1f\x02\x00",
    "42": b"\xff\xaa\x1f\x03\x00",
    "20": b"\xff\xaa\x1f\x04\x00",
    "10": b"\xff\xaa\x1f\x05\x00",
    "5": b"\xff\xaa\x1f\x06\x00",
}


SENDBACK_RATE_HZ = {
    "0.1":b"\xff\xaa\x03\x01\x00", 
    "0.5": b"\xff\xaa\x03\x02\x00",
    "1": b"\xff\xaa\x03\x03\x00",
    "2": b"\xff\xaa\x03\x04\x00",
    "5": b"\xff\xaa\x03\x05\x00",
    "10":b"\xff\xaa\x03\x06\x00",
    "20":b"\xff\xaa\x03\x07\x00",
    "50":b"\xff\xaa\x03\x08\x00" ,
    "100":b"\xff\xaa\x03\x09\x00",
    "125":b"\xff\xaa\x03\x0a\x00",
    "200":b"\xff\xaa\x03\x0b\x00",
    "Once":b"\xff\xaa\x03\x0c\x00",
    "None":b"\xff\xaa\x03\x0d\x00",
}

WIT_IMU_CMD_SET_SENDBACK_CONTENT = b"\xff\xaa\x02\x0e\x02"
WIT_IMU_CMD_RESET_YAW = b"\xff\xaa\x01\x04\x00"

# convert 8bit hex to signed int
def c8h2sint(value):
    return -(value & 0x80) | (value & 0x7F)


def imu_feedback_parse(frame): #calculations equations given in datasheet
    [
        # convert linear acc
        _, _, _AxL, _AxH, _AyL, _AyH, _AzL, _AzH, _TL, _TH, _,
        # convert angular velocity
        _, _, _wxL, _wxH, _wyL, _wyH, _wzL, _wzH, _TL, _TH, _,
        # convert Roll Pitch Yaw
        _, _, _RollL, _RollH, _PitchL, _PitchH, _YawL, _YawH, _TL, _TH, _,
        # _,_,_,_,_,_,_,_,_,_,__,
        # convert quaternion
        _, _, _Q0L, _Q0H, _Q1L, _Q1H, _Q2L, _Q2H, _Q3L, _Q3H, _,
    ] = frame

    # m/s^2
    Ax = float((c8h2sint(_AxH) << 8) | _AxL) / 32768.0 * 16 * 9.8
    Ay = float((c8h2sint(_AyH) << 8) | _AyL) / 32768.0 * 16 * 9.8
    Az = float((c8h2sint(_AzH) << 8) | _AzL) / 32768.0 * 16 * 9.8

    # deg/s to rad/s
    wx = float((c8h2sint(_wxH) << 8) | _wxL) / 32768.0 * 2000 * math.pi / 180
    wy = float((c8h2sint(_wyH) << 8) | _wyL) / 32768.0 * 2000 * math.pi / 180
    wz = float((c8h2sint(_wzH) << 8) | _wzL) / 32768.0 * 2000 * math.pi / 180

    # deg/s to rad/s
    roll = float((c8h2sint(_RollH) << 8) | _RollL) / 32768.0 * 180 * math.pi / 180
    pitch = float((c8h2sint(_PitchH) << 8) | _PitchL) / 32768.0 * 180 * math.pi / 180
    yaw = float((c8h2sint(_YawH) << 8) | _YawL) / 32768.0 * 180 * math.pi / 180

    # quaternion from IMU
    q0 = float((c8h2sint(_Q0H) << 8) | _Q0L) / 32768
    q1 = float((c8h2sint(_Q1H) << 8) | _Q1L) / 32768
    q2 = float((c8h2sint(_Q2H) << 8) | _Q2L) / 32768
    q3 = float((c8h2sint(_Q3H) << 8) | _Q3L) / 32768

    return {
        "acceleration": [
            Ax,
            Ay,
            Az,
        ],
        "angle_speed": [
            wx,
            wy,
            wz,
        ],
        "radians": [
            roll,
            pitch,
            yaw,
        ],
        "quaternion": [q0, q1, q2, q3],
    }



class IMUCHECK(object):
    def __init__(self):
        super(IMUCHECK,self).__init__()
        # self.port = "/dev/ttyUSB0"
        self.port = "/dev/imu"
        self.baud = None
        self.button = None
        self.state = None
        self.parameter1 = None
        self.parameter2 = None
        self.parameter3 = None
        self.parameter4 = None
        # self.parameter1 = ACC_SPEED[self.parameter1] #ACC
        # self.parameter2 = GYRO_DEGREES[self.parameter2] #GYRO
        # self.parameter3 = BANDWIDTH_HZ[self.parameter3] #Bandwidth
        # self.parameter4 = SENDBACK_RATE_HZ[self.parameter4] #Sendback rate

        self.imu_msg = IMU_DATA.type
        self.imu_puber = IMU_DATA.Publisher()

        IMU_SRV_CMD.Services(self.srv_cb) # place the funcion as the handler

    def srv_cb(self, srv):
        logger.loginfo("Executing service call...")
        logger.loginfo("srv")
        response = CommandResponse()
        print("if response == 1 means success, if response == 0 means fail")
        print("response:")
        print(response)
        logger.loginfo("Service call back was executed")
    
    def initialize(self):
        self.state = "IDLE"

    def run(self):
        logger.logwarn("Running")
        pp = PrettyPrinter(indent=2)
        l = rospy.Rate(NODE_RATE)
        while not rospy.is_shutdown():
            rospy.loginfo_throttle(5, "imu checker running...")
            l.sleep()

    
if __name__ == "__main__":
    print("imu_check node is initialzing")
    rospy.init_node("imu_check")
    imu_checker = IMUCHECK()
    imu_checker.initialize()
    print("imu_check node is initialzed")
    imu_checker.run()
