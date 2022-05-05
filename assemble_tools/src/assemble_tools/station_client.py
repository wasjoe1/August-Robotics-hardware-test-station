#!/usr/bin/env python
# -*- coding: utf-8 -*-
from __future__ import print_function, division

import time
import math
import numpy as np
import tf.transformations as tftrans
import geometry_msgs.msg as gemsgs
# import guiding_beacon.msg as gbmsgs
import guiding_beacon_system_msgs.msg as gbmsgs


from boothbot_common.utils import inverse_matrix_hard
# from common import Logging
from boothbot_common.ros_logger_wrap import ROSLogging as Logging

from common.states import GuidingStationControllerStates as GSCS
from boothbot_nav.constants import LOCALIZATION_POSE_COVARIANCE, LOCALIZATION_UNLOCATED_COVARIANCE
from boothbot_nav.settings import LED_BEACON_RADIUS

class PoseWithTransformations(object):
    translate = None
    quaternion = None
    angles = None
    transformation = None
    inv_transformation = None
    covariance = None
    def __init__(
            self,
            translate=(0., 0., 0.),
            quaternion=None,
            angles=(0., 0., 0.),
            covariance=LOCALIZATION_UNLOCATED_COVARIANCE,
        ):
        self.translate = translate
        if quaternion is not None:
            self.quaternion = quaternion
            self.angles = tftrans.euler_from_quaternion(quaternion)
        else:
            self.angles = angles
            self.quaternion = tftrans.quaternion_from_euler(*angles)
        self.transformation = tftrans.compose_matrix(translate=self.translate, angles=self.angles)
        # self.inv_transformation = np.array(np.mat(self.transformation).I)
        self.inv_transformation = inverse_matrix_hard(self.transformation)
        self.covariance = covariance

class GuidingBeaconClient(Logging):
    dev_name = None
    __pose = None
    relative_height = 0.0

    _gs_hub = None
    _last_command = None
    goal = None
    feedback = None
    def __init__(self, name, gs_hub):
        super(GuidingBeaconClient, self).__init__(name.upper())
        self.dev_name = name
        self._gs_hub = gs_hub
        self.__pose = PoseWithTransformations()
        self.goal = {
            'gid':      0,
            'trigger':  False,
            'color':    'ROG',
            'distance': 0.0,
            'rad_hor':  0.0,
            'rad_ver':  0.0,
        }
        self.feedback = {
            'timestamp':0.0,
            'gid':      0,
            'located':  False,
            'state':    'INIT',
            'distance': 0.0,
            'rad_hor':  0.0,
            'rad_ver':  0.0,
            'circle':   False,
            'rad_circle':0.0,
        }

    def _feedback_cb(self, feedback):
        # Special case that we are not using DTU instead directly get feedback cb
        if isinstance(feedback, gbmsgs.GuidingBeaconFeedback):
            feedback = {
                'gid': feedback.gid,
                'located': feedback.located,
                'state': feedback.state,
                'distance': feedback.distance,
                'rad_hor': feedback.rad_hor,
                'rad_ver': feedback.rad_ver,
                'circle': feedback.circle,
                'rad_circle': feedback.rad_circle,
            }

        self.feedback.update(feedback)

    def _pose_cb(self, pose):
        # Special case that we are not using DTU instead directly get pose cb
        if isinstance(pose, gemsgs.PoseWithCovarianceStamped):
            trans = (pose.pose.pose.position.x,
                     pose.pose.pose.position.y,
                     pose.pose.pose.position.z)
            quate = (pose.pose.pose.orientation.x,
                     pose.pose.pose.orientation.y,
                     pose.pose.pose.orientation.z,
                     pose.pose.pose.orientation.w)
            # angles = tftrans.euler_from_quaternion(quate)
            self.__pose = PoseWithTransformations(
                translate=trans,
                quaternion=quate,
            )

        elif 'valid' in pose and pose['valid']:
            self.__pose = PoseWithTransformations(
                translate=(pose['x'], pose['y'], 0.0),
                angles=(0.0, 0.0, pose['rz'])
            )

    def _state_cb(self, state):
        self.feedback['state'] = state

    ##################
    # used internal
    def get_target_position(self):
        """
        The results of the current measurement. Usually this should be called after the target
        was located. Or the results are useless.

        Returns
        -------
        float
            The value of x calculated based on the GB's pose.
        float
            The value of y calculated based on the GB's pose.
        float
            The value of z calculated based on the GB's pose.
        """
        (gx, gy), grz = self.pose.translate[0:2], self.pose.angles[2]
        dis, rad_h = self.get_measurement_distance(), self.feedback['rad_hor']

        # already did in get_measurement_distance()
        # dis += LED_BEACON_RADIUS
        ta = grz + rad_h
        tx = gx + dis * math.cos(ta)
        ty = gy + dis * math.sin(ta)

        return tx, ty, 0.0

    def get_target_orientation(self):
        return self.feedback['rad_hor'] + self.pose.angles[2]

    # NOTE only be used when calibration between master and slave
    def set_pose(self, translate, quaternion=None, angles=None):
        """
        Set the GB's pose.

        Parameters
        ----------
        translate:  (float, float, float)
        quanternion:  (float, float, float, float)
        angles:  (float, float, float)
        """
        if angles is None:
            angles = tftrans.euler_from_quaternion(quaternion)

        self._gs_hub.set_pose_by_name(
            self.dev_name,
            {
                'valid': True,
                'x': translate[0],
                'y': translate[1],
                'rz': angles[2]
            }
        )

    def update_relative_height(self, dis=None):
        assert self.located
        # try to get more accurate estimation
        if not dis:
            dis = self.get_measurement_distance()
        rad_v = self.feedback['rad_ver']
        self.relative_height = (self.relative_height - math.tan(rad_v) * dis) / 2
        self.logdebug("Relative beacon height was updated to: {}".format(self.relative_height))

    ##################
    # used by the gbm
    def connect(self):
        return self._gs_hub.connect_by_name(self.dev_name)

    def set_command(self, command):
        '''
        This method will publish the input string as command to station
        '''
        self._gs_hub.commanding_by_name(self.dev_name, {'command': command})

    def reset(self):
        '''
        This method will reset the client to ready, in any cases.
        '''
        self.goal['gid'] = 0
        self._gs_hub.reset_by_name(self.dev_name, {})
        self.feedback['circle'] = False
        self.feedback['rad_circle'] = 0.0

    def targeting(self, gid, color, distance, rad_hor, rad_ver, triggered, is_initialpose=False):
        self.goal = {
            'gid':      gid,
            'trigger':  triggered,
            'color':    color,
            'distance': distance,
            'rad_hor':  rad_hor,
            'rad_ver':  rad_ver,
            'is_initialpose': is_initialpose,
        }
        self._gs_hub.targeting_by_name(
            self.dev_name,
            self.goal,
        )
        if gid != self.gid:
            self.loginfo_throttle(
                1.,
                'gid: {}, assigning {} tracking target at {}'.format(
                    gid, self.dev_name, (distance, rad_hor)
                )
            )

    def trigger(self, triggered=True):
        self.goal['trigger'] = triggered
        self._gs_hub.trigger_by_name(
            self.dev_name,
            self.goal,
        )

    def get_circle_result(self):
        return (
            self.feedback['timestamp'],
            self.feedback['circle'],
            self.feedback['rad_circle'],
        )

    def get_measurement_distance(self):
        return self.feedback['distance'] * math.cos(self.feedback['rad_ver']) + LED_BEACON_RADIUS

    def get_measurement_rz(self):
        return self.feedback['rad_hor']

    def get_measurements(self):
        return (
            self.feedback['timestamp'],
            self.goal['color'],
            self.get_target_position()[0:2],
            self.feedback['rad_hor'] + self.angles[2],
        )

    # def publish_pose(self, timestamp):
    #     # DTU: publish the gb's pose here
    #     self._pose_msg.header.stamp = timestamp
    #     self._pose_msg.pose.pose.position = gemsgs.Point(*self.translate)
    #     self._pose_msg.pose.pose.orientation = gemsgs.Quaternion(*self.quaternion)

    #     self._pose_pub.publish(self._pose_msg)

    @property
    def pose(self):
        return self.__pose

    @property
    def stamp(self):
        return self.feedback['timestamp']

    @property
    def triggered(self):
        return self.state == GSCS.TRACKING.name

    @property
    def gid(self):
        # if self.goal is not None:
        #     return self.goal['gid']
        # return -1
        return self.feedback['gid']

    @property
    def idle(self):
        return self.state == GSCS.IDLE.name

    @property
    def failed(self):
        return self.state == GSCS.FAILED.name

    @property
    def error(self):
        return self.state == GSCS.ERROR.name

    @property
    def state(self):
        return self.feedback['state']

    @property
    def located(self):
        if self.feedback['gid'] != self.goal['gid']:
            return False
        # laser_diff = self.feedback['distance'] - self.goal['distance']
        # if abs(laser_diff) > LASER_WIRED_LIMIT:
        #     self.logwarn("Got laser diff is {} longer than {}, which is filted".format(
        #         laser_diff, LASER_WIRED_LIMIT))
        #     return False
        return self.feedback['located']
