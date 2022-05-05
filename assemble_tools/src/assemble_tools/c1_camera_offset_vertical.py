#!/usr/bin/env python
# encoding=utf-8

import math
import rospy

logger = rospy

from assemble_tools.get_key import GetKey


from boothbot_perception.track_client import TargetTracker, Status
from boothbot_msgs.ros_interfaces import MODULES_PERCEPT_SRV_TRACK
from client_hub import GuidingStationClientHub

INSTRUCTION = """
This script should be used to calibrate the vertical offset between cameras on CB
Current parameters:
Cameras_vertical_offset: {} , type "o" to change it
Tracking_distance: {} , type "d" to change it
Tracking_color: {} , type "c" to change it
Steps:
1. Type "i" to connect nodes
2. Type "t" to start tracking, CB should now tracking at (0., 0.) with predefined color and distance
3. If step 2 succeeded, type "s" to calculate suggested cameras_vertical_offset.
4. Use the suggested value to update "device_settings.yaml", done!
"""


NODE_NAME = "c1_camera_vertical_offset"
NODE_RATE = 5.0
FRAME_CAMERA_BEACON = "cb_0"
# GS_CAMERA_VERTICAL_DIST = 0.058 + 0.062 # For GS
GS_CAMERA_VERTICAL_DIST = 0.03 + 0.035  # For CB
TRACKING_DISTANCE = 4.0


def get_current_offset(exp_dist, short_cam_offset):
    return short_cam_offset - math.atan2(GS_CAMERA_VERTICAL_DIST, exp_dist)


class C1CameraVerticalOffset(object):
    def __init__(self):
        super(C1CameraVerticalOffset, self).__init__()
        self.get_key = None
        self.cb = None
        self.tracker_client = TargetTracker()
        self.cl_hub = GuidingStationClientHub()
        self.gid = 99
        self.color = "BOG"
        self.state = "INIT"
        self.current_offset = 0.0
        self.tracking_distance = TRACKING_DISTANCE

    def initialize(self):
        self.get_key = GetKey()
        o = raw_input(
                "Enter current camera vertical offset\n Leave empty with default {}:".format(
                    self.current_offset
                )
            )
        if o:
            self.current_offset = float(o)
        logger.logwarn("Current offset is set to: {}".format(self.current_offset))
        d = raw_input(
                "Enter tracking distance(distance from CB to beacon)\n Leave empty with default {}:".format(
                    self.tracking_distance
                )
            )
        if d:
            self.tracking_distance = float(d)
        logger.logwarn(
            "Current tracking distance is set to: {}".format(self.tracking_distance)
        )
        c = raw_input(
            "Set beacon color as\n Leave empty with default {}:".format(self.color)
        )
        if c:
            self.color = c
        logger.logwarn("Current beacon color is: {}".format(self.color))

    def run(self):
        logger.logwarn(INSTRUCTION)
        l = rospy.Rate(NODE_RATE)
        while not rospy.is_shutdown():
            if self.state == "TRACKING":
                logger.logwarn_throttle(1.0, "Tracking...")
                if self.cb.located:
                    logger.logwarn("Tracking done!!")
                    self.cb.reset()
                    self.state = "IDLE"
            else:
                key = self.get_key.get_key()
                if key in ("i", "I"):
                    logger.logwarn("Trying to initialize...")
                    self.cb = self.cl_hub.detect_server(FRAME_CAMERA_BEACON)
                    self.cb.reset()
                    self.tracker_client.connect()
                    self.state = "IDLE"

                elif key in ("f", "F"):
                    if self.state != "IDLE":
                        logger.logerr("Not initialized yet!")
                        continue
                    logger.logwarn("Toggle track server to show images on rviz...")
                    MODULES_PERCEPT_SRV_TRACK.service_call()

                elif key in ("t", "T"):
                    if self.state != "IDLE":
                        logger.logerr("Not initialized yet!")
                        continue
                    self.state = "TRACKING"
                    self.gid += 1
                    self.cb.targeting(
                        gid=self.gid,
                        color="BOG",
                        distance=self.tracking_distance,
                        rad_hor=0.0,
                        rad_ver=0.0,
                        triggered=True,
                        is_initialpose=True,
                    )

                elif key in ("s", "S"):
                    if self.state != "IDLE":
                        logger.logerr("Not initialized yet!")
                        continue
                    long_vertical_offset = 0.0
                    logger.logwarn("Triggering long...")
                    cam_status, offset = self.tracker_client._beacon_track(
                        self.tracking_distance, "LO", self.color
                    )
                    logger.logwarn("{}, {}".format(cam_status, offset))
                    if cam_status == "LONG":
                        long_vertical_offset = offset[1]
                    elif cam_status == "OUT":
                        logger.logerr("Long cam tracking failed, please try again!!")
                        continue

                    logger.logwarn("Triggering short...")
                    cam_status, offset = self.tracker_client._beacon_track(
                        self.tracking_distance, "SO", self.color
                    )
                    logger.logwarn("{}, {}".format(cam_status, offset))
                    if cam_status == "SHORT":
                        short_vertical_offset = offset[
                            1
                        ] - self.short_cam_pitch_compensation_fn(self.tracking_distance)
                        logger.logwarn(
                            "Current short tracking offset is: {}".format(
                                short_vertical_offset
                            )
                        )
                        suggest_offset = (
                            long_vertical_offset
                            - short_vertical_offset
                            - math.atan2(
                                GS_CAMERA_VERTICAL_DIST, self.tracking_distance
                            )
                        )
                        logger.logwarn(
                            "Suggested camera vertical offset is: {}".format(
                                suggest_offset
                            )
                        )

                elif key in ("c", "C"):
                    c = input("Set beacon color as:")
                    self.color = c
                    logger.logwarn("Current beacon color is: {}".format(self.color))

                elif key in ("o", "O"):
                    o = float(input("Set current offset as:"))
                    self.current_offset = o
                    logger.logwarn(
                        "Current offset is set to: {}".format(self.current_offset)
                    )

                elif key in ("d", "D"):
                    d = float(input("Set tracking distance as:"))
                    self.tracking_distance = d
                    logger.logwarn(
                        "Current tracking distance is set to: {}".format(
                            self.tracking_distance
                        )
                    )

                elif key in ("e", "E"):
                    logger.logwarn("Exiting...")
                    self.get_key.end_get_key()
                    break

    def short_cam_pitch_compensation_fn(self, exp_dist):
        return math.atan2(GS_CAMERA_VERTICAL_DIST, exp_dist) + self.current_offset


if __name__ == "__main__":
    rospy.init_node(NODE_NAME, log_level=rospy.INFO)
    c1 = C1CameraVerticalOffset()
    c1.initialize()
    c1.run()
