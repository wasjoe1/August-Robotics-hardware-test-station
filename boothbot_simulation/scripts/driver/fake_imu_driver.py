#!/usr/bin/python

import serial
import binascii
import math
import time
import struct

import rospy
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, Empty
import std_msgs.msg as stmsgs
import boothbot_msgs.msg as bbmsgs
import tf

import numpy as np
from common import Logging
from common.errcode import IMU_FID, ErrCode
from common.ros_topics import IMU_HB, IMU_RESET_YAW_CMD, IMU_RAW_DATA, \
    IMU_WIT, CHASSIS_OFFLINE_STATUS
from common.record_usage import HWName, HWRecord

# NOTE: 
# This code is copied from boothbot_driver/scripts/imu_driver_node.py with only minor changes.
# This node currently only publishes empty IMU messages, which should not be used!

class FakeIMUDriver(Logging):
    def __init__(self, port='/dev/ttyUSB0', baudrate=115200, timeout=0.5, pub_topic=IMU_WIT):
        super(FakeIMUDriver, self).__init__("IMU_D")
        self.port = rospy.get_param('~imu_port', port)
        self.rate = float(rospy.get_param('~rate', 150.))
        self.baudrate = baudrate
        self.timeout = timeout
        self.pub = rospy.Publisher(pub_topic, Imu, queue_size=1)

        #for robot_pose_ekf
        self.pub_remap = rospy.Publisher(IMU_RAW_DATA, Imu, queue_size=1)
        self.sub_reset = rospy.Subscriber(IMU_RESET_YAW_CMD, Bool, self._resetYaw_cb)
        #self.srv = rospy.Service('IMU_yaw_reset', reset_yaw, self._resetYaw)
        self._yaw = 0
        self.count = 0
        self.hearbeat_pub = rospy.Publisher(IMU_HB, Empty, queue_size=1)
        self.recorder = HWRecord(HWName.IMU)

        # initalize msg
        self._msg = Imu()
        self._msg.header.frame_id = "imu_link"
        self._msg.orientation_covariance = []
        self._msg.angular_velocity_covariance = []
        self._msg.linear_acceleration_covariance = []

        rospy.Subscriber('/debug_imu_calibrate', stmsgs.String, self._cmd_cb)

        self._msg.orientation_covariance = [
                3e-6 , 0 , 0,
                0, 3e-6, 0,
                0, 0, 3e-6
                ]
        self._msg.angular_velocity_covariance = [
                0.02, 0 , 0,
                0 , 0.02, 0,
                0 , 0 , 0.02
                ]
        self._msg.linear_acceleration_covariance = [
                0.0001 , 0 , 0,
                0 , 0.0001, 0,
                0 , 0 , 0.0001
                ]

    def _getYawDeg(self):
        _,_,_yaw = tf.transformations.euler_from_quaternion([
           self._msg.orientation.x,
           self._msg.orientation.y,
           self._msg.orientation.z,
           self._msg.orientation.w])
        return round(math.degrees(_yaw),2)

    def _resetYaw_cb(self,reset_msg):
        """
        Callback to reset Yaw angle to 0
        """
        raise NotImplementedError

    def _parser2(self, frame):
        # m/s^2
        Ax = 0.0
        Ay = 0.0
        Az = 0.0

        # deg/s to rad/s
        wx = 0.0
        wy = 0.0
        wz = 0.0

        # deg/s to rad/s
        roll = 0.0
        pitch = 0.0
        self._yaw = 0.0

        # quaternion from IMU
        Q0 = 0.0
        Q1 = 0.0
        Q2 = 0.0
        Q3 = 0.0

        #self.loginfo("wx %f wy %f wz %f",wx,wy,wz)
        #fill in the msg
        self.count += 1

        self._msg.header.stamp =  rospy.Time.now()
        self._msg.header.seq = self.count

        self._msg.linear_acceleration.x = Ax
        self._msg.linear_acceleration.y = Ay
        self._msg.linear_acceleration.z = Az
        self._msg.angular_velocity.x = wx
        self._msg.angular_velocity.y = wy
        self._msg.angular_velocity.z = wz

        self._msg.orientation.x = Q1
        self._msg.orientation.y = Q2
        self._msg.orientation.z = Q3
        self._msg.orientation.w = Q0

    def start(self):
        self.loginfo("Connecting to IMU on port {} ...".format(self.port))
        self.loginfo("IMU is connected!")
        self.loginfo("IMU is initilized!")

        self.logerrcode(IMU_FID, ErrCode.OK.value)
        self.loop = rospy.Rate(self.rate)

        # the main loop
        while not rospy.is_shutdown():
            self._parser2(None)
            self.loginfo_throttle(5,"IMU is working, current yaw is {:+.2f} deg".format(math.degrees(self._yaw)))
            self.pub.publish(self._msg)
            self.hearbeat_pub.publish(Empty())
            self.loop.sleep()

        # node is shutting down, cleanup
        self.loginfo("IMU Driver Node is shutting down...")
        self.recorder.shutdown()

    def _cmd_cb(self, msg):
        if msg.data == 'CALIBRATE':
            self.calibrate_accelerometer()
        elif msg.data == 'SAVE':
            self.save_param()

    def calibrate_accelerometer(self):
        print('Calibrating accelerometer...')
        print('Ignored in simulation')

    def save_param(self):
        print('Saving params...')
        print('Ignored in simulation.')

if __name__ == '__main__':
    rospy.init_node("imu_node", anonymous=True)
    test_imu = FakeIMUDriver()
    test_imu.start()
