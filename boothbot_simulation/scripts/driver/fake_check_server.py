#!/usr/bin/env python

"""
Fake mark detection camera action server.
"""
import time
import rospy
import threading
import actionlib

from common import Logging
import boothbot_msgs.msg as bbmsg
from boothbot_msgs.ros_interfaces import \
    MODULES_PERCEPT_ACT_CHECK, \
    MODULES_PERCEPT_SRV_CHECK, \
    MODULES_PERCEPT_CHECK_STATUS


class MarkDetectServer(Logging):
    _feedback = bbmsg.CameraMarkDetectFeedback()
    _result   = bbmsg.CameraMarkDetectResult()
    def __init__(self, port='/dev/camera_marking', is_boothnumber=False):
        super(MarkDetectServer, self).__init__('MarkDetectServer')

        # use lock to protect command read/write
        self._lock = threading.Lock()
        self._command=""
        self._goal_done = False
        self._goal = None

        # init the message publisher
        self.pub = rospy.Publisher(
            MODULES_PERCEPT_CHECK_STATUS.name,
            MODULES_PERCEPT_CHECK_STATUS.type,
            queue_size=1)

        # init the trigger service
        rospy.Service(
            MODULES_PERCEPT_SRV_CHECK.name,
            MODULES_PERCEPT_SRV_CHECK.type,
            self.service_cb
        )

        # init action server
        self.server = actionlib.SimpleActionServer(
            MODULES_PERCEPT_ACT_CHECK.name,
            MODULES_PERCEPT_ACT_CHECK.type,
            self.action_execute,
            False)
        self.server.start()

    @property
    def command(self):
        with self._lock:
            command = self._command
        self._command = ""
        return command

    @command.setter
    def command(self, req):
        if req.command in ["TRI_BG", "TRI_MK"]:
            with self._lock:
                self._command = req.command
        else:
            self.logerr("Wrong req {}".format(req.command))

    @property
    def goal(self):
        with self._lock:
            return self._goal

    @goal.setter
    def goal(self, value):
        self.logdebug("Check server receive goal {}".format(value))
        with self._lock:
            self._goal = value

    @property
    def goal_done(self):
        with self._lock:
            goal_done = self._goal_done
            self._goal_done = False
        return goal_done

    @goal_done.setter
    def goal_done(self, value):
        self.logdebug("Check server goal done")
        with self._lock:
            self._goal_done = value

    def service_cb(self, req):
        self.loginfo("Check server receive command {}".format(req.command))
        self.command = req
        return True

    def send_result(self, goal_id, fname, result):
        self._result.stamp = rospy.Time.now()
        self._result.fname = fname
        self._result.goal_id = goal_id
        self._result.result = result
        self.server.set_succeeded(self._result)

    def action_execute(self, goal):
        self.loginfo(goal.stamp)
        self.loginfo(goal.goal_id)
        self.loginfo(goal.offset_yaw)
        self.loginfo(goal.exp_type)

        self.goal = goal

        while True:
            if self.goal_done:
                break
            else:
                rospy.sleep(0.1)

    def goal_execute(self):
        goal = self.goal
        if goal is not None:
            # get command with lock
            command = self.command
            if command == "TRI_BG":
                self._feedback.state = "WAITING_MK"
                self.server.publish_feedback(self._feedback)

            elif command == "TRI_MK":
                self._feedback.state = "WAITING_DONE"
                self.server.publish_feedback(self._feedback)
                time.sleep(1)
                self.send_result(goal.goal_id, "", 'GOOD')
                self.goal_done = True
                self.goal = None

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
    def main():
        rospy.init_node('fake_check_server')
        is_boothnumber = rospy.get_param('~is_boothnumber')
        status_hz = rospy.get_param('~status_hz')
        boot_delay = rospy.get_param('~boot_delay')
        time.sleep(boot_delay)
        server = MarkDetectServer(is_boothnumber=is_boothnumber)
        rate = rospy.Rate(status_hz)
        while not rospy.is_shutdown():
            server.heartbeat()
            server.goal_execute()
            rate.sleep()
        server.shutdown()
    main()
