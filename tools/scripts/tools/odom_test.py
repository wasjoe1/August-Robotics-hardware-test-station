#!/usr/bin/env python
# -*- coding: utf-8 -*-
#usage: python odom_test.py [distance]
# Lionel will move alone x or y axis by [distance] meters, make sure there is enough place.

import os
import sys
import csv
import math
import time
import rospy
import std_msgs.msg as stmsgs
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from move_base_msgs.msg import MoveBaseActionGoal


def _pose_cb(msg):
    global Curr_pose
    Curr_pose = msg.pose.pose.position

def send_goal(x,y):
    goal = MoveBaseActionGoal()
    goal.goal.target_pose.header.frame_id = 'odom'
    goal.goal.target_pose.pose.position.x = x
    goal.goal.target_pose.pose.position.y = y
    goal.goal.target_pose.pose.orientation.w = 1
    move_pub.publish(goal)

if __name__ == '__main__':
    if len(sys.argv) == 2:
        l = int(sys.argv[1])
    else:
        l = 4
    data_dir = '/home/augbooth/benchmark_result/'
    filename = 'odom_test_{}.csv'.format(time.strftime("%Y%m%d%H%M",time.localtime()))
    if not os.path.exists(os.path.dirname(data_dir)):
        os.mkdir(data_dir)
    rospy.init_node('odom_test')
    stop = Twist()
    rospy.Subscriber('odom', Odometry, _pose_cb)
    move_pub = rospy.Publisher('move_base/goal', MoveBaseActionGoal, queue_size=2)

    headers = ['cmd','odometry','real_movement']
    with open(data_dir + filename, 'a') as f:
        f_csv = csv.writer(f)
        f_csv.writerow(headers)

    while not rospy.is_shutdown():
        cmd = raw_input('Type the cmd:')
        start_x = Curr_pose.x
        start_y = Curr_pose.y
        x = start_x
        y = start_y
        if cmd == 'x':
            x =  start_x + l
        elif cmd == '-x':
            x =  start_x - l
        elif cmd == 'y':
            y =  start_y + l
        elif cmd == '-y':
            y =  start_y - l
	send_goal(x,y)
        dist = raw_input('Type Lionel\'s movement:')
        odom_dist = math.sqrt((Curr_pose.x - start_x) * (Curr_pose.x - start_x) + (Curr_pose.y - start_y) * (Curr_pose.y - start_y))
        odom_dist = round(odom_dist,3)
        with open(data_dir + filename, 'a') as f:
		    f_csv = csv.writer(f)
		    f_csv.writerow([cmd,odom_dist,dist])
