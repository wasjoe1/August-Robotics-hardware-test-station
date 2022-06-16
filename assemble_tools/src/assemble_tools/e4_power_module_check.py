#!/usr/bin/env python3

import time
import math
from pprint import PrettyPrinter
import rospy

logger = rospy

import binascii
from serial import Serial
from assemble_tools.get_key import GetKey

INSTRUCTION = """
Key functions:
i: Initialize power module
c: Get current readings
s: Update settings to power module and save
q: Exiting...
"""

NODE_NAME = "e4_power_module_check"
NODE_RATE = 5.0

BATTERY_CMD_CUT_POWER = b"AT+POLAR+0\r\n"
BATTERY_CMD_CHECK_V = b"AT+V\r\n"
BATTERY_CMD_CHECK_C = b"AT+C\r\n"
BATTERY_CMD_CHECK_P = b"AT+P\r\n"

BATTERY_CMD_SET_RESISTANCE = b"AT+R+2\r\n"
BATTERY_CMD_SET_SCALE = b"AT+MAXC+40\r\n"

BATTERY_CMD_SET_OVERV = b"AT+MODE+OVERV\r\n"
BATTERY_CMD_SET_UNDERV = b"AT+MODE+UNDERV\r\n"
BATTERY_CMD_SET_UNDERV_LINE = b"AT+UNDERVERR+21\r\n" # protect battery when too low
BATTERY_CMD_SET_OVERC = b"AT+MODE+OVERC\r\n"
BATTERY_CMD_SET_UNDERC = b"AT+MODE+UNDERC\r\n"
BATTERY_CMD_SET_OVERP = b"AT+MODE+OVERP\r\n"
BATTERY_CMD_SET_UNDERP = b"AT+MODE+UNDERP\r\n"

BATTERY_CMD_SET_D_NC = b"AT+POLAR+1\r\n" # disconnect NC and COM pin when trigger the alarm
BATTERY_CMD_SET_D_NO = b"AT+POLAR+0\r\n" # disconnect NO and COM pin when trigger the alarm

class E4PowerModuleCheck(object):
    def __init__(self):
        super(E4PowerModuleCheck, self).__init__()
        self.get_key = None
        self.port = "/dev/ttyUSB0"
        self.baud = 9600

    def initialize(self):
        self.get_key = GetKey()

    def run(self):
        logger.logwarn(INSTRUCTION)
        l = rospy.Rate(NODE_RATE)
        while not rospy.is_shutdown():
            key = self.get_key.get_key()
            if key in ("i", "I"):
                logger.logwarn("Trying to initialize...")
                logger.loginfo("Detecting power module...")
                # TODO: Check if port existed
                with Serial(port=self.port, baudrate=self.baud, timeout=1.0) as p:
                    p.write(b"AT\r\n")
                    # p.write("BATTERY_CMD_CHECK_V")
                    time.sleep(0.5)
                    if p.read_all() == b"OK\r\n":
                        logger.logwarn("Power module detected!")

            if key in ("s", "S"):
                with Serial(port=self.port, baudrate=self.baud, timeout=1.0) as p:
                    time.sleep(0.2)
                    logger.logwarn("Setting resistance to: 2 Ohm!")
                    p.write(BATTERY_CMD_SET_RESISTANCE)
                    time.sleep(0.2)
                    logger.logwarn("Setting scale to: 40A!")
                    p.write(BATTERY_CMD_SET_SCALE)
                    time.sleep(0.2)
                    logger.logwarn("Setting alarm under: 21V!")
                    p.write(BATTERY_CMD_SET_UNDERV)
                    time.sleep(0.2)
                    p.write(BATTERY_CMD_SET_UNDERV_LINE)
                    time.sleep(0.2)
                    logger.logwarn("Setting NO mode!")
                    p.write(BATTERY_CMD_SET_D_NO)
                    time.sleep(0.2)
                    logger.logwarn("CFG saved!")

            if key in ("c", "C"):
                with Serial(port=self.port, baudrate=self.baud, timeout=1.0) as p:
                    p.write(BATTERY_CMD_CHECK_V)
                    time.sleep(0.2)
                    vol = p.read_all()
                    p.write(BATTERY_CMD_CHECK_C)
                    time.sleep(0.2)
                    cur = p.read_all()
                    p.write(BATTERY_CMD_CHECK_P)
                    time.sleep(0.2)
                    pow = p.read_all()
                    logger.logwarn("Current data: {}".format([vol, cur, pow]))

            elif key in ("q", "Q"):
                logger.logwarn("Exiting...")
                self.get_key.end_get_key()
                break


if __name__ == "__main__":
    rospy.init_node(NODE_NAME, log_level=rospy.INFO)
    e4 = E4PowerModuleCheck()
    e4.initialize()
    e4.run()
