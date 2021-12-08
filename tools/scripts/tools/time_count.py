#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os
import time
import math
import argparse

import rospy
import std_msgs.msg as stmsgs
import boothbot_msgs.msg as bbmsgs
import geometry_msgs.msg as gemsgs
import tf.transformations as tftrans

from common import Logging

class Time_count(Logging):
    """
    Time counter will use topic information to record Lionel's moving, locating time for each goal.
    How to use:
        It can run seperatly, just run `python time_count.py`
    ros_param:
        `data_dir`: where data storage.
                    data format: Goal pos, Lionel pos, Moving time, Locating time.
    """
    def __init__(self, data_dir=None):
        rospy.init_node('time_counter', log_level=rospy.DEBUG)
        super(Time_count, self).__init__('TC')

        if data_dir == None:
            self.data_dir = rospy.get_param('data_dir', default='/tmp/time_count')
        else:
            self.data_dir = data_dir
        if not os.path.exists(os.path.dirname(self.data_dir)):
            raise ValueError('Folder not exists')
        rospy.Subscriber('map_task/goal', bbmsgs.BoothBotGoal, self._goal_cb)
        rospy.Subscriber('/robot_pose_ekf/odom_combined',
                gemsgs.PoseWithCovarianceStamped, self._odom_combined_cb)
        rospy.Subscriber('io_driver/set_do', stmsgs.UInt16, self._io_set_cb)
        rospy.Subscriber('io_driver/reset_do', stmsgs.UInt16, self._io_reset_cb)
        rospy.Subscriber('marking/cmdword', stmsgs.String, self._marking_cb)

        rate = 15
        self._loop = rospy.Rate(rate)
        self.state = 'INIT'
        self.get_new_goal = False  # state sign: if get new goal
        self.moving = False  # state sign: if moving done
        self.mark = False  # state sign: if located
        self.goal_count = 0
        self.info = ''

        while not rospy.is_shutdown():
            self.poll()
            self._loop.sleep()

    def poll(self):
        state = self.state
        if state == 'INIT' or state == 'DONE':
            if self.get_new_goal == True:
                self.loginfo('get goal: {}'.format(self.goal.coords))
                self.loginfo('from pos: {}'.format(self.pose))
                self.info = ''
                self.info = self.info + str(self.goal_count)
                self.info = self.info + ' ' + str(self.pose)
                self.info = self.info + ' ' + str(self.goal.coords)
                self.goal_count += 1
                self.time_start = time.time()
                self.time_stamp = self.time_start
                self.moving = True
                self.state = 'MOVE'
        elif state == 'MOVE':
            self.get_new_goal = False
            if self.moving == False:  # Light on
                move_time = time.time() - self.time_stamp
                self.time_stamp = time.time()
                self.info = self.info + ' ' + str(round(move_time, 2))
                self.mark = False
                self.state = 'LOCATE'
        elif state == 'LOCATE':
            if self.get_new_goal == True:
                self.info = self.info + ' ' + 'SKIP'
                with open(self.data_dir, 'a') as f:
                    f.write(self.info)
                    f.write('\n')
                self.state = 'DONE'
            elif self.moving == True:
                locate_time = time.time() - self.time_stamp
                self.time_stamp = time.time()
                self.info = self.info + ' ' + str(round(locate_time, 2))
                self.state = 'MOVE'
            elif self.mark == True:
                locate_time = time.time() - self.time_stamp
                self.info = self.info + ' ' + str(round(locate_time, 2))
                with open(self.data_dir, 'a') as f:
                    f.write(self.info)
                    f.write('\n')
                self.loginfo('DONE!')
                self.state = 'DONE'

    def _marking_cb(self, mark_cmd):
        if len(mark_cmd.data.split('_')) == 2:
            self.mark = True

    def _io_set_cb(self, msg):
        if msg.data == 256:
            self.moving = False

    def _io_reset_cb(self, msg):
        if msg.data == 256:
            self.moving = True

    def _goal_cb(self, goal):
        self.get_new_goal = True
        self.goal = goal
        self.goal.coords = [round(i, 3) for i in self.goal.coords]

    def _odom_combined_cb(self, odom):
        self._odom_combined = odom
        self.x = round(odom.pose.pose.position.x, 3)
        self.y = round(odom.pose.pose.position.y, 3)

        rz = tftrans.euler_from_quaternion((
            odom.pose.pose.orientation.x,
            odom.pose.pose.orientation.y,
            odom.pose.pose.orientation.z,
            odom.pose.pose.orientation.w))[2]
        self.rz = round(math.degrees(rz), 2)  # rad to degree
        self.pose = (self.x, self.y, self.rz)

if __name__=='__main__':
    parser = argparse.ArgumentParser(
            description="Time counter will use topic information to record \
                         Lionel's moving, locating time for each goal.")
    parser.add_argument("-d",
                        "--data_dir",
                        type=str,
                        help='where time count data store',
                        default='/tmp/data_dir')
    arg = parser.parse_args()
    timecount = Time_count(arg.data_dir)
