#!/usr/bin/python

import os
import time
import rospy
import math
import yaml
import numpy as np
import tf.transformations as tftrans
import boothbot_common.ros_logger as logger
from boothbot_msgs.ros_interfaces import (
    DEBUG_RVIZ_SIMPLE_GOAL,
    DEBUG_NAV_4D,
    DEBUG_NAV_4D_AUTO_CALI_POINT,
    DEBUG_NAV_4D_RESULT,
    DEBUG_MANUAL_COMMAND,
    DEBUG_NAV_UPDATE_CB_ZERO_OFFSET,
)

from boothbot_perception.check_client import MarkChecker
from boothbot_driver.io_driver_client import IODriverClient
from boothbot_common.settings import MARKING_SYSTEM_OFFSET, LOCAL_TMP_CONFIG_FOLDER_PATH_LIONEL
from boothbot_driver.settings import SERVO_H_CONFIG


TEST_DATA_PATH = os.path.join(LOCAL_TMP_CONFIG_FOLDER_PATH_LIONEL, "nav_4d_cali")
if not os.path.exists(TEST_DATA_PATH):
    os.makedirs(TEST_DATA_PATH)

def get_rotate_2d(theta):
    return np.array([[np.cos(theta), -np.sin(theta)],
                     [np.sin(theta), np.cos(theta)]])

def get_matrix_from_tag(tag):
    x, y, z = (
        tag.pose.pose.pose.position.x,
        tag.pose.pose.pose.position.y,
        tag.pose.pose.pose.position.z,
    )
    quaternion = (
        tag.pose.pose.pose.orientation.x,
        tag.pose.pose.pose.orientation.y,
        tag.pose.pose.pose.orientation.z,
        tag.pose.pose.pose.orientation.w,
    )
    angles = tftrans.euler_from_quaternion(quaternion)
    matrix = tftrans.compose_matrix(translate=(x, y, z), angles=angles)
    return matrix


ROTATE_ROW_180 = tftrans.compose_matrix(angles=(np.pi, 0.0, 0.0))
CURRENT_MARK_OFFSET = MARKING_SYSTEM_OFFSET
CURRENT_ZERO_OFFSET = SERVO_H_CONFIG["zero_offset"]

LED_BELT_IO = [False] * 14 + [True]


class NAV4DAutoCali(object):
    def __init__(self):
        self.auto_cali_point = None
        self.auto_cali_orientations = [
            0.0,
            np.pi / 4,
            np.pi / 2,
            3 * np.pi / 4,
            np.pi,
            -3 * np.pi / 4,
            -np.pi / 2,
            -np.pi / 4,
        ]
        self.auto_cali_tags = [None] * len(self.auto_cali_orientations)
        self.state = 0
        self.calculated_offset_differences = [0.0, 0.0]
        self.average_d_a = [0.0, 0.0]
        self.mark_offset = CURRENT_MARK_OFFSET
        self.zero_offset = CURRENT_ZERO_OFFSET

        self.client_checker = MarkChecker()
        self.client_io = IODriverClient()

        self.msg_nav_4d_result = None
        DEBUG_NAV_4D_AUTO_CALI_POINT.Subscriber(callback=self._nav_4d_auto_cali_point_cb)
        DEBUG_NAV_4D_RESULT.Subscriber(callback=self._nav_4d_result_cb)
        self.pub_nav_4d = DEBUG_NAV_4D.Publisher()
        self.pub_manual_command = DEBUG_MANUAL_COMMAND.Publisher()
        self.pub_zero_offset = DEBUG_NAV_UPDATE_CB_ZERO_OFFSET.Publisher()

    @property
    def auto_cali_point(self):
        return self._auto_cali_point

    @auto_cali_point.setter
    def auto_cali_point(self, value):
        self._auto_cali_point = value
        logger.logwarn("Auto cali point is updated to: {}".format(self._auto_cali_point))

    def _nav_4d_auto_cali_point_cb(self, msg):
        if len(msg.data) != 2:
            logger.logwarn("Auto cali target point invalid!")
        self.auto_cali_point = list(msg.data)

    def _nav_4d_result_cb(self, msg):
        self.msg_nav_4d_result = msg

    def _checker_done_cb(self, state, result):
        logger.logwarn("Checker done with: {} and {}".format(state, result))

    def update_mark_offset(self, mark_offset):
        self.mark_offset = mark_offset
        logger.logwarn("Updating mark_offset to: {}".format(self.mark_offset))
        self.pub_manual_command.publish("MARK_OFFSET:{}".format(self.mark_offset))

    def update_zero_offset(self, zero_offset):
        self.zero_offset = zero_offset
        logger.logwarn("Updating zero_offset to: {}".format(self.zero_offset))
        self.pub_zero_offset.publish(self.zero_offset)

    def clean_dynamic_for_next_run(self):
        logger.logwarn("Cleaning data for next run...")
        self.state = 0
        self.calculated_offset_differences = [0.0, 0.0]
        self.average_d_a = [0.0, 0.0]
        self.auto_cali_tags = [None] * len(self.auto_cali_orientations)

    @property
    def state(self):
        return self._state

    @state.setter
    def state(self, value):
        self._state = value
        logger.loginfo("State is changed to: {}".format(self._state))

    def run(self):
        # 1 set each pose to nav client
        # if SUCCEEDED, get tag
        # 2
        # Calculate d and a as offsets
        # Get encoder zero_offset and mark_offset differences
        if not self.client_checker.connect():
            logger.logfatal("Checker connect failed")
            return False
        if not self.client_io.connect():
            logger.logfatal("IO connect failed")
            return False
        index_ori = 0
        cali_pose = None
        loop = rospy.Rate(1.0)
        logger.loginfo("Working with mark_offset: {}".format(self.mark_offset))
        logger.loginfo("Working with zero_offset: {}".format(self.zero_offset))
        result_file_name = "{}/{}-result.yaml".format(TEST_DATA_PATH, time.strftime("%Y%m%d-%H_%M"))
        while not rospy.is_shutdown():
            # Wait target point
            if self.state == 0:
                if self.auto_cali_point is None:
                    logger.logwarn_throttle(5., "Waiting for auto cali target point!")
                else:
                    logger.logwarn("Received auto cali target point: {}".format(self.auto_cali_point))
                    self.client_io.send_goal(set_io=LED_BELT_IO)
                    self.state = 1
            # Trigger navi
            elif self.state == 1:
                cali_pose = self.auto_cali_point + [
                    self.auto_cali_orientations[index_ori]
                ]
                self.msg_nav_4d_result = None
                self.pub_nav_4d.publish(DEBUG_NAV_4D.type(data=cali_pose))
                self.state = 2
            # Wait navi
            elif self.state == 2:
                if self.msg_nav_4d_result is None:
                    logger.loginfo("Waiting navigation...")
                elif self.msg_nav_4d_result.result == "SUCCEEDED":
                    self.client_checker.check("0", "", self._checker_done_cb)
                    self.state = 3
                else:
                    logger.logwarn("Navigation failed!")
                    return False
            # Trigger checker cali
            elif self.state == 3:
                # Trigger the cali tag capture
                self.client_checker.cali()
                self.state = 4
            # Wait tag
            elif self.state == 4:
                if self.client_checker.tag_detections is None:
                    logger.loginfo("Waiting checker...")
                elif self.client_checker.tag_detections.detections:
                    if len(self.client_checker.tag_detections.detections) == 0:
                        logger.logwarn("Checker tag detection wrong!")
                        return False
                    else:
                        tag = self.client_checker.tag_detections.detections[-1]
                        if len(tag.id) != 4:
                            logger.logwarn("No valid tag for cali found!")
                            return False

                    self.auto_cali_tags[index_ori] = tag
                    logger.logwarn(
                        "Got tag:\n{}".format(self.auto_cali_tags[index_ori])
                    )
                    index_ori += 1
                    if index_ori < len(self.auto_cali_orientations):
                        self.state = 1
                        logger.loginfo("Moving to next loop")
                    else:
                        self.state = 5
                        logger.loginfo("Going to calculate a and d")
            # Calculate result
            elif self.state == 5:
                self.average_d_a = self.get_d_a_from_tags(self.auto_cali_tags)
                self.calculated_offset_differences = self.get_offsets_from_d_a(*self.average_d_a)
                self.state = 6
            # Saving result
            elif self.state == 6:
                with open(result_file_name, "w") as f:
                    data = {
                        "stamp": time.time(),
                        "point": list(self.auto_cali_point),
                        "running_offsets": [float(self.mark_offset), int(self.zero_offset)],
                        "calculated_offset_differences": list(self.calculated_offset_differences),
                        "average_d_a": list(self.average_d_a),
                        #"calculated_offset": [(self.calculated_offset_differences[0] + self.mark_offset), (self.calculated_offset_differences[1] + self.zero_offset)],
                        "orientations": list(self.auto_cali_orientations),
                        "tags": self.auto_cali_tags,
                        "tags_trans": [tftrans.translation_from_matrix(np.dot(ROTATE_ROW_180, get_matrix_from_tag(x))).tolist() for x in self.auto_cali_tags],
                    }
                    yaml.dump(data, f, Dumper=yaml.CDumper)
                    logger.loginfo("Result saved at: {}".format(result_file_name))
                return True
            loop.sleep()

    def get_d_a_from_tags(self, tags):
        # oppsite_tag_pairs = [(0, 2), (1, 3)]  # 0,180 and 90,-90
        oppsite_tag_pairs = [(0, 4), (2, 6)]  # 0,180 and 90,-90
        oppsite_tag_orien = [0., np.pi / 2]
        da_pairs = [None] * len(oppsite_tag_pairs)

        for i, tag_p in enumerate(oppsite_tag_pairs):
            trans_1 = tftrans.translation_from_matrix(
                np.dot(ROTATE_ROW_180, get_matrix_from_tag(tags[tag_p[0]]))
            )
            trans_2 = tftrans.translation_from_matrix(
                np.dot(ROTATE_ROW_180, get_matrix_from_tag(tags[tag_p[1]]))
            )
            v_offset = trans_1[:2] - trans_2[:2]
            ori = oppsite_tag_orien[i]
            da_pairs[i] = np.dot(get_rotate_2d(ori), v_offset)
        logger.loginfo("Offsets got: {}".format(da_pairs))

        # Get average of each column
        avg_a , avg_d = np.average(da_pairs, axis=0)
        return float(avg_d), float(avg_a)

    def get_offsets_from_d_a(self, d, a):
        logger.logwarn("Calculating offsets using d: {}, a: {}".format(d, a))
        zero_offset_radians = np.arctan2(-a/2,abs(CURRENT_MARK_OFFSET + d/2))
        mark_offset = l - a / 2 / np.sin(zero_offset_radians)
        current_mark_offset = CURRENT_MARK_OFFSET + mark_offset



        logger.logwarn(
            "Calculated yaw_offset compensation is: {}".format(
                zero_offset_radians
            )
        )
        logger.logwarn(
            "Calculated mark_offet compensation is: {}".format(current_mark_offset)
        )
        return current_mark_offset, zero_offset_radians


if __name__ == "__main__":
    rospy.init_node("test_4d_auto_cali", log_level=rospy.INFO)
    n4ac = NAV4DAutoCali()
    dx = 1
    dy = 1
    i = 0
    i = i + 1
    logger.loginfo("Running {} times".format(i + 1))
    ret = n4ac.run()
    logger.logwarn("Succeeded? {}".format(ret))
    if rospy.is_shutdown():
        logger.logfatal("Is shutting down....")
    if ret is False:
        logger.logfatal("Running failed, will exit...")
    else:
        cod_d, cod_a = n4ac.calculated_offset_differences
        #n4ac.update_mark_offset(n4ac.mark_offset + cod_d)
        #n4ac.update_zero_offset(n4ac.zero_offset + int(cod_a))
        dx, dy = n4ac.average_d_a
        n4ac.auto_cali_point = [
            n4ac.auto_cali_point[0] + dx / 2,
            n4ac.auto_cali_point[1] - dy / 2,
        ]
        n4ac.clean_dynamic_for_next_run()
