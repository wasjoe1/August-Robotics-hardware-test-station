#!/usr/bin/python
# -*- coding: utf-8 -*-

import rospy
import geometry_msgs.msg as gmmsg
import tf.transformations as tftfs
import tf
import sensor_msgs.msg as smmsg
import nav_msgs.msg as nmmsg
import math
import boothbot_msgs.srv as bmsrv


CMD_VEL = "cmd_vel"
ODOM = "odom"
BASE_FOOTPRINT = "base_footprint"
IMU_WIT = "imu_wit"
FAKE_BASE = "fake_base"
FAKE_BASE_RESET_SERVICE = "fake_base/reset"
MAP = "map"
RESET_COMMAND = "RESET"
RATE = 200
START_TIME = 0.001
IMU_ORIENTATION_COVARIANCE = [1e6, 0, 0, 0, 1e6, 0, 0, 0, 1e-6]
IMU_ANGULAR_VELOCITY_COVARIANCE = [1e6, 0, 0, 0, 1e6, 0, 0, 0, 1e-6]
IMU_LINEAR_ACCELERATION_COVARIANCE = [-1, 0, 0, 0, 0, 0, 0, 0, 0]

ODOM_POSE_COVARIANCE = [1e-3, 0, 0, 0, 0, 0,
                        0, 1e-3, 0, 0, 0, 0,
                        0, 0, 1e6, 0, 0, 0,
                        0, 0, 0, 1e6, 0, 0,
                        0, 0, 0, 0, 1e6, 0,
                        0, 0, 0, 0, 0, 1e3]
ODOM_TWIST_COVARIANCE = [1e-3, 0, 0, 0, 0, 0,
                         0, 1e-3, 0, 0, 0, 0,
                         0, 0, 1e6, 0, 0, 0,
                         0, 0, 0, 1e6, 0, 0,
                         0, 0, 0, 0, 1e6, 0,
                         0, 0, 0, 0, 0, 1e3]


class FakeBase:
    def __init__(self):
        rospy.Subscriber(CMD_VEL,
                         gmmsg.Twist,
                         self.cmd_vel_cb)
        self.rate = RATE
        self.odom_puber = rospy.Publisher(ODOM, nmmsg.Odometry, queue_size=1)
        self.imu_wit_puber = rospy.Publisher(IMU_WIT, smmsg.Imu, queue_size=1)
        self.current_time = None
        self.last_time = None
        self.br = tf.TransformBroadcaster()
        self.cmd_vel_data = gmmsg.Twist()
        self.cmd_vel_data.angular.x = 0.0
        self.cmd_vel_data.angular.y = 0.0
        self.cmd_vel_data.angular.z = 0.0
        self.cmd_vel_data.linear.x = 0.0
        self.cmd_vel_data.linear.y = 0.0
        self.cmd_vel_data.linear.z = 0.0
        self.is_pub_map_to_odom = rospy.get_param("~pub_map_to_odom")
        self.reset_service = rospy.Service(
            FAKE_BASE_RESET_SERVICE, bmsrv.SIMBoothBotCMD, self.handle_fake_base_reset)
        self._x = 0.0
        self._y = 0.0
        self._th = 0.0
        self.last_v_x = 0.0
        self.last_v_y = 0.0

    def handle_fake_base_reset(self, req):
        if req.command == "RESET":
            odom_topic = nmmsg.Odometry()
            odom_topic.header.stamp = rospy.Time.now()
            odom_topic.header.frame_id = ODOM
            odom_topic.child_frame_id = BASE_FOOTPRINT
            odom_topic.pose.pose.position.x = 0.0
            odom_topic.pose.pose.position.y = 0.0
            odom_topic.pose.pose.position.z = 0.0
            odom_topic.pose.pose.orientation.x = 0.0
            odom_topic.pose.pose.orientation.y = 0.0
            odom_topic.pose.pose.orientation.z = 0.0
            odom_topic.pose.pose.orientation.w = 1.0
            odom_topic.twist.twist.linear.x = 0.0
            odom_topic.twist.twist.linear.y = 0.0
            odom_topic.twist.twist.angular.z = 0.0
            odom_topic.pose.covariance = ODOM_POSE_COVARIANCE
            odom_topic.twist.covariance = ODOM_TWIST_COVARIANCE
            self.odom_puber.publish(odom_topic)

            imu_topic = smmsg.Imu()
            imu_topic.header.frame_id = BASE_FOOTPRINT
            imu_topic.header.stamp = rospy.Time.now()
            imu_topic.orientation.x = 0.0
            imu_topic.orientation.y = 0.0
            imu_topic.orientation.z = 0.0
            imu_topic.orientation.w = 1.0
            imu_topic.orientation_covariance = IMU_ORIENTATION_COVARIANCE
            imu_topic.angular_velocity.x = 0.0
            imu_topic.angular_velocity.y = 0.0
            imu_topic.angular_velocity.z = 0.0
            imu_topic.angular_velocity_covariance = IMU_ANGULAR_VELOCITY_COVARIANCE
            imu_topic.linear_acceleration.x = 0.0
            imu_topic.linear_acceleration.y = 0.0
            imu_topic.linear_acceleration.z = 0.0
            imu_topic.linear_acceleration_covariance = IMU_LINEAR_ACCELERATION_COVARIANCE
            self.imu_wit_puber.publish(imu_topic)
            self._x = 0.0
            self._y = 0.0
            self._th = 0.0
            self.last_v_x = 0.0
            self.last_v_y = 0.0
            q = tftfs.quaternion_from_euler(0.0, 0.0, 0.0)
            self.br.sendTransform([0.0, 0.0, 0.0], q,
                                rospy.Time.now(), BASE_FOOTPRINT, ODOM)
            return True
        else:
            return False

    def cmd_vel_cb(self, msg):
        self.cmd_vel_data = msg

    def data_process(self):
        self.current_time = rospy.Time.now()
        if self.last_time is None:
            self.last_time = self.current_time - \
                rospy.Duration.from_sec(START_TIME)
        v_x = self.cmd_vel_data.linear.x
        v_y = self.cmd_vel_data.linear.y
        v_th = self.cmd_vel_data.angular.z
        dt = (self.current_time - self.last_time).to_nsec()/1000000000.0
        delta_x = (v_x * math.cos(self._th) - v_y * math.sin(self._th)) * dt
        delta_y = (v_x * math.sin(self._th) + v_y * math.cos(self._th)) * dt
        delta_th = v_th * dt
        self._x = self._x + delta_x
        self._y = self._y + delta_y
        self._th = self._th + delta_th

        q = tftfs.quaternion_from_euler(0, 0, self._th)
        self.br.sendTransform([self._x, self._y, 0.0], q,
                              self.current_time, BASE_FOOTPRINT, ODOM)

        odom_topic = nmmsg.Odometry()
        odom_topic.header.stamp = self.current_time
        odom_topic.header.frame_id = ODOM
        odom_topic.child_frame_id = BASE_FOOTPRINT
        odom_topic.pose.pose.position.x = self._x
        odom_topic.pose.pose.position.y = self._y
        odom_topic.pose.pose.position.z = 0.0
        odom_topic.pose.pose.orientation.x = q[0]
        odom_topic.pose.pose.orientation.y = q[1]
        odom_topic.pose.pose.orientation.z = q[2]
        odom_topic.pose.pose.orientation.w = q[3]
        odom_topic.twist.twist.linear.x = v_x
        odom_topic.twist.twist.linear.y = v_y
        odom_topic.twist.twist.angular.z = v_th
        odom_topic.pose.covariance = ODOM_POSE_COVARIANCE
        odom_topic.twist.covariance = ODOM_TWIST_COVARIANCE
        self.odom_puber.publish(odom_topic)

        imu_topic = smmsg.Imu()
        imu_topic.header.frame_id = BASE_FOOTPRINT
        imu_topic.header.stamp = self.current_time
        imu_topic.orientation.x = q[0]
        imu_topic.orientation.y = q[1]
        imu_topic.orientation.z = q[2]
        imu_topic.orientation.w = q[3]
        imu_topic.orientation_covariance = IMU_ORIENTATION_COVARIANCE
        imu_topic.angular_velocity.x = 0.0
        imu_topic.angular_velocity.y = 0.0
        imu_topic.angular_velocity.z = v_th
        imu_topic.angular_velocity_covariance = IMU_ANGULAR_VELOCITY_COVARIANCE
        imu_topic.linear_acceleration.x = (v_x - self.last_v_x)/dt
        imu_topic.linear_acceleration.y = (v_y - self.last_v_y)/dt
        imu_topic.linear_acceleration.z = 0.0
        imu_topic.linear_acceleration_covariance = IMU_LINEAR_ACCELERATION_COVARIANCE
        self.imu_wit_puber.publish(imu_topic)

        self.last_time = self.current_time
        self.last_v_x = v_x
        self.last_v_y = v_y

    def run(self):
        fake_base_loop = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            self.data_process()
            fake_base_loop.sleep()


if __name__ == "__main__":
    # creating ROS node
    rospy.init_node(FAKE_BASE, log_level=rospy.INFO)
    fb = FakeBase()
    fb.run()
