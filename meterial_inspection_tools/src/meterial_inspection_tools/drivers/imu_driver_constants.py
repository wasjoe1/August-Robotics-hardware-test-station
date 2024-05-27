#!/usr/bin/python3
# -*- coding: utf-8 -*-

import math
import binascii
from enum import Enum
from serial import Serial



class WIT_IMU_CONSTANTS(Enum):
    WIT_IMU_CMD_CALI_ACC = b"\xff\xaa\x01\x01\x00" 
    WIT_IMU_CMD_SAVE_CFG = b"\xff\xaa\x00\x00\x00"
    WIT_IMU_CMD_UNLOCK = b"\xff\xaa\x69\x88\xb5"
    WIT_IMU_CMD_SET_BAUDRATE = b"\xff\xaa\x04\x06\x00"  # 115200
    WIT_IMU_CMD_SET_SENDBACK_CONTENT = b"\xff\xaa\x02\x0e\x02"
    WIT_IMU_CMD_RESET_YAW = b"\xff\xaa\x01\x04\x00"
    WIT_IMU_CMD_SET_SCALE_ACC = b"\xff\xaa\x21\x03\x00"
    WIT_IMU_CMD_SET_SCALE_GYRO = b"\xff\xaa\x20\x03\x00"
    WIT_IMU_CMD_SET_BANDWIDTH = b"\xff\xaa\x1f\x00\x00"
    WIT_IMU_CMD_SET_SENDBACK_RATE = b"\xff\xaa\x03\x0b\x00" 


    BAUDRATE_TABLE = {
        "9600" : b"\xff\xaa\x04\x02\x00",
        "19200": b"\xff\xaa\x04\x03\x00",
        "57600": b"\xff\xaa\x04\x05\x00",
        "115200":b"\xff\xaa\x04\x06\x00",
    }

'''
    ACC_SPEED = {
        "2": b"\xff\xaa\x21\x00\x00", #2g
        "4": b"\xff\xaa\x21\x01\x00", #4g
        "8": b"\xff\xaa\x21\x02\x00", #8g
        "16": b"\xff\xaa\x21\x03\x00", #16g
    }


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
        "11":b"\xff\xaa\x03\x0c\x00", #onces
        "0":b"\xff\xaa\x03\x0d\x00", #none
    }
'''



class RION_IMU_CONSTANTS(Enum):

    RION_IMU_CMD_SET_BAUDRATE = b"\x68\x05\x00\x0b\x05\x13"
    RION_IMU_READ = b"\x65\x05\x00\x8c\x00\x91"
    RION_IDENTIFIER = b"\x68"

    BAUDRATE_TABLE = {
        "9600" : b"\x68\x05\x00\x0b\x02\x13",
        "19200": b"\x68\x05\x00\x0b\x03\x13",
        "115200":b"\x68\x05\x00\x0b\x05\x13",
    }










