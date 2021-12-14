#!/usr/bin/env python

"""
Mark detection camera action server.
"""
import os
import rospy
import threading
import actionlib
from datetime import datetime

import boothbot_msgs.msg as bbmsg
from boothbot_msgs.ros_interfaces import \
    MODULES_PERCEPT_ACT_CHECK, \
    MODULES_PERCEPT_SRV_CHECK, \
    MODULES_PERCEPT_CHECK_STATUS
# from common.errcode import ErrCode        # Not exist anymore
from boothbot_perception.settings import MARKING_IMG_FOLDER as LIONEL_SAVE_FOLDER
from common import Logging

# NOTE: This code is copied from boothbot_perception/scripts/boothbot_perception/check_server.py with only minor changes.

class MarkDetectServer(Logging):
    img_parent_folder = os.path.join(LIONEL_SAVE_FOLDER, 'mark_imgs')
    is_camera_params = {'h': 120, 'w': 160}
    bn_camera_params = {'h': 480, 'w': 640}
    _feedback = bbmsg.CameraMarkDetectFeedback()
    _result   = bbmsg.CameraMarkDetectResult()
    def __init__(self, port='/dev/camera_marking', is_boothnumber=False):
        super(MarkDetectServer, self).__init__('MarkDetectServer')
        # init camera state machine
        self.is_boothnumber = is_boothnumber
        if self.is_boothnumber:
            camera_params = self.bn_camera_params
        else:
            camera_params = self.is_camera_params
        ctrls = {"exposure_auto":1, "exposure_absolute":226}

        # use lock to protect command read/write
        self._lock = threading.Lock()
        self.command=""

        # make a new dir to save the images
        if not os.path.exists(self.img_parent_folder):
            os.makedirs(self.img_parent_folder)
        init_time = datetime.now().strftime('%Y-%m-%d_%H-%M-%S')
        self.img_folder = os.path.join(self.img_parent_folder, init_time)
        if not os.path.exists(self.img_folder):
            os.makedirs(self.img_folder)

        # init the trigger service
        rospy.Service(
            MODULES_PERCEPT_SRV_CHECK.name,
            MODULES_PERCEPT_SRV_CHECK.type,
            self.set_command
        )

        # init action server
        self.server = actionlib.SimpleActionServer(
            MODULES_PERCEPT_ACT_CHECK.name,
            MODULES_PERCEPT_ACT_CHECK.type,
            self.execute,
            False)
        self.server.start()

        self.pub = rospy.Publisher(
            MODULES_PERCEPT_CHECK_STATUS.name,
            MODULES_PERCEPT_CHECK_STATUS.type,
            queue_size=1)

    def set_command(self, req):
        self.logwarn("Check server receive command {}".format(req.command))
        if req.command in ["TRI_BG", "TRI_MK"]:
            with self._lock:
                self.command = req.command
                return True
        return False

    def send_result(self, goal_id, fname, result):
        self._result.stamp = rospy.Time.now()
        self._result.fname = fname
        self._result.goal_id = goal_id
        self._result.result = result
        self.server.set_succeeded(self._result)
        self._feedback.state = "WAITING_BG"
        self.server.publish_feedback(self._feedback)

    def execute(self, goal):
        print(goal.stamp)
        print(goal.goal_id)
        print(goal.offset_yaw)
        print(goal.exp_type)

        # background and mark image stored in memory
        bg = None
        mk = None

        while True:
            with self._lock:
                command = self.command
                self.command = ""

            if command == "TRI_BG":
                self._feedback.state = "WAITING_MK"
                self.server.publish_feedback(self._feedback)
                fname = os.path.join(
                    self.img_folder,
                    '{goal_id}_{suffix}.png'.format(
                        goal_id=goal.goal_id,
                        suffix='bg',
                    ))

                self.server.set_aborted()
                break

            elif command == "TRI_MK":
                self._feedback.state = "WAITING_DONE"
                self.server.publish_feedback(self._feedback)
                fname = os.path.join(
                    self.img_folder,
                    '{goal_id}_{suffix}.png'.format(
                        goal_id=goal.goal_id,
                        suffix='mk',
                    ))
                self.send_result(goal.goal_id, fname, 'GOOD')
                break

            else:
                rospy.sleep(1)

    def heartbeat(self):
        msg = MODULES_PERCEPT_CHECK_STATUS.type()
        msg.stamp = rospy.Time.now()
        msg.state = "INIT"
        msg.current_task = ""
        msg.errorcodes = []
        msg.state = "RUNNING"
        self.pub.publish(msg)

    def shutdown(self):
        pass

if __name__ == '__main__':
    import time
    rospy.init_node('check_server')
    is_boothnumber = rospy.get_param('~is_boothnumber')
    status_hz = rospy.get_param('~status_hz')
    boot_delay = rospy.get_param('~boot_delay')
    time.sleep(boot_delay)
    server = MarkDetectServer(is_boothnumber=is_boothnumber)
    rate = rospy.Rate(status_hz)
    while not rospy.is_shutdown():
        server.heartbeat()
        rate.sleep()
    server.shutdown()
