#!/usr/bin/env python
# encoding=utf-8

import rospy

logger = rospy

from assemble_tools.get_key import GetKey

import std_msgs.msg as stmsgs
import sensor_msgs.msg as semsgs

from common.ros_topics import CMDWORD_fn, GS_HWS_JOINT_fn

NODE_NAME = "b3_vertical_offset"
NODE_RATE = 5.0

TARGET_DEVICE = "gs_0"

INSTRUCTION = """
"""


class B3VerticalZeroOffset(object):
    CMD_CALI_MANUAL = "calimanual"
    CMD_CALI_TRACK = "calitrack"

    def __init__(self):
        self.get_key = None
        self._msg_joint = None
        self._pub_cmd = rospy.Publisher(
            CMDWORD_fn(TARGET_DEVICE), stmsgs.String, queue_size=1
        )

    def initialize(self):
        self.get_key = GetKey()

    def _setup_subsriber(self):
        rospy.Subscriber(
            GS_HWS_JOINT_fn(TARGET_DEVICE), semsgs.JointState, self._joint_cb
        )

    def run(self):
        logger.logwarn(INSTRUCTION)
        while not rospy.is_shutdown():
            key = self.get_key.get_key()
            if key in ("n", "N"):
                logger.logwarn("Publishing 'calimanual'")
                self._pub_cmd.publish(self.CMD_CALI_MANUAL)

            elif key in ("f", "F"):
                logger.logwarn("Current horizontal/vertical encoder are:")
                logger.logwarn("{}".format(self._msg_joint.effort))

            elif key in ("e", "E"):
                logger.logwarn("Exiting...")
                self.get_key.end_get_key()
                break

    def _joint_cb(self, msg):
        self._msg_joint = msg


if __name__ == "__main__":
    rospy.init_node(NODE_NAME, log_level=rospy.INFO)
    b3 = B3VerticalZeroOffset()
    b3.initialize()
    b3.run()
