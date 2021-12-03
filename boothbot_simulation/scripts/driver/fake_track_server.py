#!/usr/bin/env python

"""
Camera Beacon Detection action server.
"""
import os
import rospy
import rospkg
import actionlib
import threading

from boothbot_msgs.msg import CameraTrackResult
from boothbot_msgs.ros_interfaces import \
    MODULES_PERCEPT_ACT_TRACK, \
    MODULES_PERCEPT_SRV_TRACK, \
    MODULES_PERCEPT_TRACK_LONG_IMAGE, \
    MODULES_PERCEPT_TRACK_SHORT_IMAGE, \
    MODULES_PERCEPT_TRACK_STATUS
from common import Logging
import common.gs_settings as gbsets
from common.errcode import ErrCode
from std_srvs.srv import TriggerResponse
from sensor_msgs.msg import Image, CameraInfo

# NOTE: This code is copied from boothbot_perception/scripts/boothbot_perception/track_server.py with only minor changes.

MIX_COLORS = {
    # name: [below color, above color]
    'ROG': ['GREEN', 'RED'],
    'ROB': ['BLUE', 'RED'],
    'GOR': ['RED', 'GREEN'],
    'GOB': ['BLUE', 'GREEN'],
    'BOR': ['RED', 'BLUE'],
    'BOG': ['GREEN', 'BLUE'],
}

class TrackServer(Logging):
    def __init__(self):
        super(TrackServer, self).__init__('TargetTracker')

        # init the detector
        rospack = rospkg.RosPack()
        model_path = os.path.join(
            rospack.get_path('boothbot_perception'),
            'models',
            'linear_gray.p'
        )

        # use lock to protect the flag
        self._lock = threading.Lock()
        self.show_image = False

        # init the trigger service
        rospy.Service(
            MODULES_PERCEPT_SRV_TRACK.name,
            MODULES_PERCEPT_SRV_TRACK.type,
            self.set_command
        )

        # init action server
        self.server = actionlib.SimpleActionServer(
            MODULES_PERCEPT_ACT_TRACK.name,
            MODULES_PERCEPT_ACT_TRACK.type,
            self.execute,
            False)
        self.server.start()

        self.pub_status = rospy.Publisher(
            MODULES_PERCEPT_TRACK_STATUS.name,
            MODULES_PERCEPT_TRACK_STATUS.type,
            queue_size=1)

        self.pub_image_long = rospy.Publisher(
            MODULES_PERCEPT_TRACK_LONG_IMAGE.name,
            MODULES_PERCEPT_TRACK_LONG_IMAGE.type,
            queue_size=1)

        self.pub_image_short= rospy.Publisher(
            MODULES_PERCEPT_TRACK_SHORT_IMAGE.name,
            MODULES_PERCEPT_TRACK_SHORT_IMAGE.type,
            queue_size=1)

    def set_command(self, req):
        self.logwarn("Track server receive trigger command ")
        with self._lock:
            self.show_image = not self.show_image
            self.logwarn("Track server show image {}".format(self.show_image))
            return TriggerResponse(
                True,
                "Track server show image {}".format(self.show_image)
            )

    def get_angle(self, frame, bt, exp_dist, draw=False):
        return None

    def _draw_tracking_frame(self, frame, res):
        """
        Drawing tracking info on frame such that we can have visiable tool
        to debug tracking.
        """
        pass

    # As we use python3 in melodic, we need to do this by our own
    @staticmethod
    def img_to_msg(image, stamp, encoding='bgr8', frame_id='camera_left_link'):
        msg = Image()
        msg.height = image.shape[0]
        msg.width = image.shape[1]
        msg.step = image.strides[0]
        msg.encoding = encoding
        msg.header.frame_id = frame_id
        msg.header.stamp = stamp
        msg.data = image.flatten().tolist()
        return msg

    def execute(self, goal):
        self.loginfo("goal.goal_id :{}".format(goal.goal_id))
        self.loginfo("goal.stamp :{}".format(goal.stamp))
        self.loginfo("goal.exp_dist :{}".format(goal.exp_dist))
        self.loginfo("goal.camera_type :{}".format(goal.camera_type))
        self.loginfo("goal.tracking_type :{}".format(goal.tracking_type))

        # Set tracking type
        c_type = 'mix' if goal.tracking_type in MIX_COLORS else 'single'

        with self._lock:
            show_image = self.show_image

        # Prepare the result
        result   = CameraTrackResult()
        result.stamp = rospy.Time.now()
        result.goal_id = goal.goal_id
        result.result = "OUT"
        result.offset_yaw = 0.0
        result.offset_pitch = 0.0

        self.server.set_succeeded(result)

    def heartbeat(self):
        msg = MODULES_PERCEPT_TRACK_STATUS.type()
        msg.stamp = rospy.Time.now()
        msg.state = "INIT"
        msg.current_task = ""
        msg.errorcodes = []
        msg.state = "RUNNING"
        self.pub_status.publish(msg)

    def shutdown(self):
        self.long_cam.exit()
        self.short_cam.exit()

if __name__ == '__main__':
    rospy.init_node('track_server')
    status_hz = rospy.get_param('~status_hz')
    server = TrackServer()
    rate = rospy.Rate(status_hz)
    while not rospy.is_shutdown():
        server.heartbeat()
        rate.sleep()
    server.shutdown()
