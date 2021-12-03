#!/usr/bin/env python
# -*- coding: utf-8 -*-
#Run this script after RUN Lionel. It will start counting from 2nd goal
#The result will be saved at ~/benchmark_result 
import os
import time
import math
import csv
import argparse

import rospy
import std_msgs.msg as stmsgs
import boothbot_msgs.msg as bbmsgs
import geometry_msgs.msg as gemsgs
import tf.transformations as tftrans
import matplotlib.pyplot as plt

from common import Logging

class Locating_Evaluate(Logging):
    """
    Time counter will use topic information to record Lionel's moving, locating time for each goal.
    ros_param:
        `data_dir`: where data storage.
                    data format: Count_Number Start_Pos Goal_Pos Distance Lookback Locating_Time Once_Time.
    """
    def __init__(self, data_dir=None):
        rospy.init_node('locating_evaluate', log_level=rospy.DEBUG)
        super(Locating_Evaluate, self).__init__('LE')

        if data_dir == None:
            self.data_dir = rospy.get_param('data_dir', default='/home/augbooth/benchmark_result/')
        else:
            self.data_dir = data_dir
        if not os.path.exists(os.path.dirname(self.data_dir)):
            os.mkdir(self.data_dir)
        self.filename =  'benchmark' + str(time.strftime("%Y%m%d%H%M",time.localtime())) + '.csv'
        rospy.Subscriber('map_task/goal', bbmsgs.BoothBotGoal, self._goal_cb)
        rospy.Subscriber('/robot_pose_ekf/odom_combined', gemsgs.PoseWithCovarianceStamped, self._odom_combined_cb)
        rospy.Subscriber('nav/state', stmsgs.String, self.navstate_cb)
        rospy.Subscriber('marking/state', stmsgs.Int32, self._marking_cb)
	
	headers = ['Count_Number', 'Start_Pos', 'Goal_Pos', 'Distance', 'Lookback', 'Locating_Time', 'Short_Moving_Time', 'TimeCost']
        self.laststate = 'None'
	self.newGoal = False
        self.goal_count = 0
	self.total_lookback = 0
	self.total_locating = 0
	self.total_time = 0
	self.total_distance = 0
	
	with open(self.data_dir + self.filename, 'a') as f:
	    f_csv = csv.writer(f)
            f_csv.writerow(headers)
	while not rospy.is_shutdown():	    
	    rospy.spin()
	
	average_locate_partion = 100 *round(self.total_locating / self.total_time,3)
	average_locate_time = round(self.total_locating / self.goal_count,2)
	average_speed = self.total_distance / self.total_time


	fig = plt.figure(figsize=(14,3))
	text = fig.text(0.05,0.5, 'Finish{} goals in {}s.\nAverage locating time is {}s per goal.\nLocalization cost {}% time.\nGeneral speed is {}m/s'
			.format	(self.goal_count,round(self.total_time,2),average_locate_time,average_locate_partion,average_speed),
			ha='left',va='center',size=35)
	plt.show()

    def _marking_cb(self, state):
	if self.newGoal:
            if state.data == 4:
		self.newGoal = False
		self.time_done = time.time()
		once_time = round(self.time_done - self.time_start,2)
		self.total_lookback += self.lookback
		self.total_locating += self.locate_time
		self.total_time += once_time		
		self.loginfo('Look back {} times'.format(self.lookback))
		self.loginfo('Locating time: {} s'.format(self.locate_time))
		with open(self.data_dir + self.filename, 'a') as f:
		    f_csv = csv.writer(f)
		    f_csv.writerow([self.goal_count, str(self.pose), str(self.goal_coords), self.distance, self.lookback, self.locate_time/self.lookback, self.move_time/self.lookback, once_time])
		self.loginfo('DONE! \r\n')

    def navstate_cb(self, msg):
        if self.goal_count > 0:
            state = msg.data
            if self.laststate == 'pending' and state == 'moving' and self.lookback > 0:
                self.time_move = time.time()
	        self.loginfo('Moving.')
            elif self.laststate == 'moving' and state == 'locating':
                self.lookback += 1    
                self.time_locate = time.time()
                if self.lookback >1:
                    self.move_time += round(self.time_locate - self.time_move,2)
	        self.loginfo('Locating.')
            elif self.laststate == 'moving' and state == 'done':
                self.move_time += round(time.time() - self.time_move,2)
            elif self.laststate == 'locating' and state != 'locating':
                self.locate_time += round(time.time() - self.time_locate,2)
            self.laststate = state
    def _goal_cb(self, goal):
	if self.newGoal:
	    self.goal_count -=1 #Last Goal Failure
	self.newGoal = True
	self.lookback = 0
        self.locate_time = 0
        self.move_time = 0
        self.goal_count += 1
        goal.coords = [round(i, 3) for i in goal.coords]
	self.goal_coords = goal.coords
	self.loginfo('get goal {}: {}'.format(self.goal_count,goal.coords))
	x_diff = goal.coords[0] - self.x	
	y_diff = goal.coords[1] - self.y	
	self.distance = math.sqrt(x_diff * x_diff + y_diff * y_diff)
	self.total_distance += self.distance
        self.time_start = time.time()

    def _odom_combined_cb(self, odom):
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
    EO = Locating_Evaluate()
