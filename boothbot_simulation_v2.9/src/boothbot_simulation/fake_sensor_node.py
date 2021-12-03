#!/usr/bin/python
# -*- coding: utf-8 -*-

# from unittest.loader import _SortComparisonMethod
import rospy
import sensor_msgs.msg as smmsg
import std_msgs.msg as stdmsgs

LIDAR_RATE = 10
SONAR_RATE = 100
RATE = 10
FAKE_SENSOR = "fake_sensor"
P_LIDAR_TOPIC_NAME = "~lidar_topic_name"
P_SONAR_TOPIC_LIST = "~sonar_topic_list"
P_SONAR_FRAME_LIST = "~sonar_frame_list"
P_LIDAR_FRAME = "~lidar_frame"
SONAR_HEAET_TOPIC_NAME = "/sonar/heartbeat"
SONAR_REAR_TOPIC_NAME = "/sonar_rear_group"


class FakeSensor:
    def __init__(self):
        # get lidar topic name and frame_id
        self.lidar_topic_name = rospy.get_param(P_LIDAR_TOPIC_NAME)
        self.lidar_puber = rospy.Publisher(
            self.lidar_topic_name, smmsg.LaserScan, queue_size=1)
        self.sonar_heart_puber = rospy.Publisher(
            SONAR_HEAET_TOPIC_NAME, stdmsgs.Empty, queue_size=1)
        self.lidar_frame = rospy.get_param(P_LIDAR_FRAME)
        self.sonar_group_puber = rospy.Publisher(
            SONAR_REAR_TOPIC_NAME, stdmsgs.Float32MultiArray, queue_size=1)

        # get sonar topic name and frame_id
        self.sonar_puber_list = []
        sonar_topic_list = rospy.get_param(P_SONAR_TOPIC_LIST)
        sonar_frame_list = rospy.get_param(P_SONAR_FRAME_LIST)
        if len(sonar_topic_list) != len(sonar_frame_list):
            print("sonar topic and frame setting error.")
        for sonar in range(len(sonar_topic_list)):
            self.sonar_puber_list.append([rospy.Publisher(
                sonar_topic_list[sonar], smmsg.Range, queue_size=1),
                sonar_frame_list[sonar]])

        self.laser_data = smmsg.LaserScan()
        self.sonar_data = smmsg.Range()
        self.sonar_heartbeat = stdmsgs.Empty()
        self.sonar_rear_group_data = stdmsgs.Float32MultiArray(data=[1, 2])
        # self.sonar_rear_group_data
        self.rate = RATE

    def pub(self):
        self.laser_data.header.frame_id = self.lidar_frame
        self.laser_data.header.stamp = rospy.Time.now()
        self.lidar_puber.publish(self.laser_data)
        for sonar_puber in self.sonar_puber_list:
            self.sonar_data.header.stamp = rospy.Time.now()
            self.sonar_data.header.frame_id = sonar_puber[1]
            self.sonar_data.min_range = 0.1
            self.sonar_data.max_range = 1.0
            self.sonar_data.range = float("inf")
            sonar_puber[0].publish(self.sonar_data)
        # pub sonar heart
        self.sonar_heart_puber.publish(self.sonar_heartbeat)
        self.sonar_group_puber.publish(self.sonar_rear_group_data)

    def run(self):
        ros_rate = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            self.pub()
            ros_rate.sleep()


if __name__ == "__main__":
    # creating ROS node
    rospy.init_node(FAKE_SENSOR, log_level=rospy.INFO)
    fb = FakeSensor()
    fb.run()
