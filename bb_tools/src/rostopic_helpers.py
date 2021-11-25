#!/usr/bin/python3

import rospy
from collections import deque
from timeit import default_timer as timer

class BufferedTopicSubscriber():

    def __init__(self, topic_name, msg_type, buf_size=1):
        self.info = (topic_name, msg_type)
        self.buffer = deque(maxlen=buf_size)
        self.sub = rospy.Subscriber(topic_name, msg_type, self.callback)

    def callback(self, msg):
        tup = (msg, timer())
        self.buffer.append(tup)

    def get_latest_msg(self, expire_interval=None):
        if len(self.buffer) == 0:
            return None, True

        msg, _ = self.buffer[-1]
        if expire_interval is None or self.isfresh(expire_interval):
            return msg, True
        else:
            return msg, False
    
    def isfresh(self, interval):
        if len(self.buffer) == 0:
            return False
        _, timestamp = self.buffer[-1]
        return timer() - timestamp <= interval


import math
def euler_from_quaternion(x, y, z, w):
        """
        https://automaticaddison.com/how-to-convert-a-quaternion-into-euler-angles-in-python/
        Convert a quaternion into euler angles (roll, pitch, yaw)
        roll is rotation around x in radians (counterclockwise)
        pitch is rotation around y in radians (counterclockwise)
        yaw is rotation around z in radians (counterclockwise)
        """
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)
     
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)
     
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)
     
        return roll_x, pitch_y, yaw_z # in radians

def rpy_from_pose(pose_msg):
    return rpy_from_quaternion(pose_msg.orientation)

def rpy_from_quaternion(quat_msg):
    return euler_from_quaternion(
        quat_msg.x,
        quat_msg.y,
        quat_msg.z,
        quat_msg.w
    )



if __name__=="__main__":

    from std_msgs.msg import UInt16, Int32, Float32, String, Empty, Bool

    rospy.init_node("helper", anonymous=True)
    crs = BufferedTopicSubscriber("/marking/state", Int32)

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        print(f"{crs.get_latest_msg()}")
        rate.sleep()
