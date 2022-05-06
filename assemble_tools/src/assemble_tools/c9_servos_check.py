#!/usr/bin/env python
# encoding=utf-8

import math
import rospy

logger = rospy

from assemble_tools.get_key import GetKey

from boothbot_driver.stepper_client import StepperPlatformINF

INSTRUCTION = """
This script just control the CB platform to move:
Supported operation:
Type following key for the function:
"i": initialize current script, connect to nodes...
"w": move platform to (0., 0.)
"a": move platform to (pi/2, 0.)
"s": move platform to (pi, 0.)
"d": move platform to (-pi/2, 0.)
"e": exit
"""


NODE_NAME = "c9_servos_check"
NODE_RATE = 5.0
MOTOR_TOLERANCE = 0.0001


class C9ServosCheck(object):
    def __init__(self):
        super(C9ServosCheck, self).__init__()
        self.get_key = None
        self.stepper_client = StepperPlatformINF()

    def initialize(self):
        self.get_key = GetKey()

    def run(self):
        logger.logwarn(INSTRUCTION)
        l = rospy.Rate(NODE_RATE)
        while not rospy.is_shutdown():
            key = self.get_key.get_key()
            if key in ("i", "I"):
                logger.logwarn("Trying to initialize...")
                self.stepper_client.connect()

            if key in ("w", "W"):
                self.stepper_client.move_absolute(0., 0., tolerance=MOTOR_TOLERANCE)

            if key in ("a", "A"):
                self.stepper_client.move_absolute(math.pi/2, 0., tolerance=MOTOR_TOLERANCE)

            if key in ("s", "S"):
                self.stepper_client.move_absolute(math.pi, 0., tolerance=MOTOR_TOLERANCE)

            if key in ("d", "D"):
                self.stepper_client.move_absolute(-math.pi/2, 0., tolerance=MOTOR_TOLERANCE)

            elif key in ("e", "E"):
                logger.logwarn("Exiting...")
                self.get_key.end_get_key()
                break


if __name__ == "__main__":
    rospy.init_node(NODE_NAME, log_level=rospy.INFO)
    c9 = C9ServosCheck()
    c9.initialize()
    c9.run()
