#!/usr/bin/env python
# -*- coding: utf-8 -*-
from __future__ import print_function, division

import time
import rospy
from actionlib import SimpleActionClient
import geometry_msgs.msg as gemsgs
import std_msgs.msg as stmsgs
import tf.transformations as tftrans

import guiding_beacon.msg as gbmsgs
from common import Logging
from station_client import GuidingBeaconClient
from common.ros_topics import GS_FEEDBACK_fn, GS_GOAL_fn, CMDWORD_fn


class GuidingStationClientHub(Logging):
    '''
    action client hub, managing cb actions
    '''
    mapper = None
    def __init__(self):
        super(GuidingStationClientHub, self).__init__('CL_HUB')
        self.mapper = {}
        self.__data_mapper = {}

    def detect_server(self, station_name):
        '''
        try to detect the gs and return a valid instance of Guiding Station Client
        '''
        station = GuidingBeaconClient(station_name, self)
        self.__data_mapper[station_name] = {
            'connected':    False,
            'feedback_cb':  station._feedback_cb,
        }
        rospy.Subscriber(
            GS_FEEDBACK_fn(station_name),
            gbmsgs.GuidingBeaconFeedback,
            lambda y: self.__feedback_cb_by_name(station_name, y)
        )
        self.logwarn('Detecting dev {}!'.format(station_name))
        for _ in range(240):
            time.sleep(0.5)
            if self.__data_mapper[station_name]['connected']:
                break
        if self.__data_mapper[station_name]['connected']:
            self.logwarn('Found dev {}!'.format(station_name))
            self.mapper[station_name] = station
            self.__data_mapper[station_name]['pub_goal'] = rospy.Publisher(
                GS_GOAL_fn(station_name), gbmsgs.GuidingBeaconGoal, queue_size=1
            )
            self.__data_mapper[station_name]['pub_cmdword'] = rospy.Publisher(
                CMDWORD_fn(station_name), stmsgs.String, queue_size=1
            )
            return station
        self.logerr('Cannot connect to station: {}'.format(station_name))
        return None

    def __pose_cb_by_name(self, station_name, pose_msg):
        trans = (pose_msg.pose.pose.position.x,
                 pose_msg.pose.pose.position.y,
                 pose_msg.pose.pose.position.z)
        quate = (pose_msg.pose.pose.orientation.x,
                 pose_msg.pose.pose.orientation.y,
                 pose_msg.pose.pose.orientation.z,
                 pose_msg.pose.pose.orientation.w)
        pose = {
            'x': trans[0],
            'y': trans[1],
            'rz': tftrans.euler_from_quaternion(quate)[2],
        }
        self.mapper[station_name]._pose_cb(pose)

    def __feedback_cb_by_name(self, station_name, feedback):
        self.__data_mapper[station_name]['connected'] = True
        self.__data_mapper[station_name]['feedback_cb']({
            'timestamp':feedback.header.stamp,
            'gid':      feedback.gid,
            'located':  feedback.located,
            'state':    feedback.state,
            'distance': feedback.distance,
            'rad_hor':  feedback.rad_hor,
            'rad_ver':  feedback.rad_ver,
            'circle':   feedback.circle,
            'rad_circle':feedback.rad_circle,
        })

    def targeting_by_name(self, station_name, info):
        goal = gbmsgs.GuidingBeaconGoal()
        goal.header.stamp = rospy.Time.now()
        goal.header.frame_id = station_name
        goal.gid = info['gid']
        goal.triggered = info['trigger']
        goal.color = info['color']
        goal.distance = info['distance']
        goal.rad_hor = info['rad_hor']
        goal.rad_ver = info['rad_ver']
        goal.is_initialpose = info['is_initialpose']
        self.__data_mapper[station_name]['pub_goal'].publish(goal)

    def get_pose_by_name(self, station_name, info):
        pass

    def reset_by_name(self, station_name, info):
        self.__data_mapper[station_name]['pub_cmdword'].publish(stmsgs.String('reset'))


if __name__ == "__main__":
    rospy.init_node('gs_hub_test', log_level=rospy.DEBUG)
    gs_hub = GuidingStationClientHub()
    frame_id = 'camera_beacon'
    ac = gs_hub.detect_server('camera_beacon')
    if ac and ac.connect():
        rospy.loginfo('connection test successed, reseting station')
        ac.reset()
        time.sleep(2.0)
        rospy.loginfo('now for tracking test')
        ac.targeting(
            1,
            'ROG',
            10.0,
            0.1,
            0.002,
            False,
        )
        rospy.loginfo('cmd sent to the gs')
        time.sleep(3.0)
        rospy.loginfo('start tracking')
        for _ in range(20):
            if not ac.located:
                ac.trigger(True)
            elif ac.located:
                rospy.loginfo('Tracking finished. Located: {}'.format(ac.located))
                ac.reset()
                break
            time.sleep(2.0)

