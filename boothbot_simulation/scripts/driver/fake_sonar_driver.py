#!/usr/bin/env python
# -*- coding: utf-8 -*-
from __future__ import print_function, division

import rospy
import serial
import time
import os
import binascii
from scipy.signal import butter, lfilter
from collections import OrderedDict

from sensor_msgs.msg import Range
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import MultiArrayDimension
from std_msgs.msg import Empty, Bool

from boothbot_msgs.ros_interfaces import DRIVERS_SONARS_STATUS, DRIVERS_SONARS_REAR_GROUP, DRIVERS_SONARS_REAR_ON, \
    SONAR_GROUP_PREFIX
from common.ros_topics import SONAR_SINGLE_fn, SONAR_HB, SONAR_REAR_G, REAR_SONAR_ON
from common.utils import reject_outliers
from common import Logging
from common.errcode import ErrCode, SONAR_FID
from common.record_usage import HWName, HWRecord

# address config
from boothbot_driver.ks103_config import *

from boothbot_driver.sonar_driver_ros import KS103ChainDriver
from boothbot_driver.sonar_driver_ros import MAX_SONAR_RANGE_M, SMALL_MAX_SONAR_RANGE_M, \
    MIN_SONAR_RANGE_M, SONAR_FOV_RADIAN

from overrides import overrides

# NOTE: This code is copied from boothbot_driver/src/boothbot_driver/sonar_driver_ros.py with minor changes.

class FakeSonarDriver(Logging):

    def __init__(self, port="/dev/ttyS0", baud=9600, rate=20):
        rospy.init_node('sonar_KS103_driver_node', anonymous=False)
        super(FakeSonarDriver, self).__init__("Sonar_D")

        # initialize params from ks103_config
        self.sonar_name = []  # used sonar names
        self._pub = OrderedDict()  # ros publishers
        self.addr_name = SONAR_BUS_ADDR

        # initialize params from launch file
        self.port = rospy.get_param("~port", port)
        self.baud = baud
        self.rate = rospy.get_param("~sonar_update_rate", rate)  # rostopic rate in hz
        self._front_enable = rospy.get_param("~enable_front", False)
        self._rear_enable = rospy.get_param("~enable_rear", False)
        self._side_enable = rospy.get_param("~enable_side", False)
        # If True, we will use CMD_PROBE[5] for range value of side sonar
        self.use_small_probe = bool(rospy.get_param("~use_small_probe", False))

        if not (self._front_enable or self._rear_enable or self._side_enable):
            rospy.signal_shutdown("No sonar enabled.")

        # not used for now
        self._use_low_pass_filter = rospy.get_param("~use_low_pass_filter", False)

        # interal use params
        self.rosmsg_max_sonar_range_ = MAX_SONAR_RANGE_M if not self.use_small_probe else SMALL_MAX_SONAR_RANGE_M
        self.rosmsg_min_sonar_range_ = MIN_SONAR_RANGE_M
        self.rosmsg_sonar_fov_ = SONAR_FOV_RADIAN

        # self.heartbeat_pub = rospy.Publisher(SONAR_HB, Empty, queue_size=1)
        self.pub_status = rospy.Publisher(DRIVERS_SONARS_STATUS.name,
                                          DRIVERS_SONARS_STATUS.type,
                                          queue_size=1)
        self.msg_status = DRIVERS_SONARS_STATUS.type()
        self.msg_status.stamp = rospy.Time.now()
        self.msg_status.state = 'INIT'

        self._record = HWRecord(HWName.SONARS)
        # initialize sonar msgs and bus address
        # Will add `self.sonar_name` value as well.
        self._initialize_sonar_address()

        # update sonar names
        rospy.set_param("~raw_sonar_topic_names", self.sonar_name)

        # initialize publishers
        for _i in self.sonar_name:
            self._pub[_i] = rospy.Publisher(SONAR_GROUP_PREFIX + _i, Range, queue_size=5)

        # counter for rosmsg seq
        self.count = 0

        self.loginfo("Sonar Sensors to connect {}".format(self.sonar_name))
        self.loginfo("Sonar is Connected at port {}!".format(self.port))
        self.loginfo("Sonar port will be ignored!")

    def _initialize_rear_sonar_rosmsg(self):
        # publisher for rear sonars
        self._rear_pub = rospy.Publisher(DRIVERS_SONARS_REAR_GROUP.name,
                                         DRIVERS_SONARS_REAR_GROUP.type,
                                         queue_size=1)
        # rospy.Subscriber(DRIVERS_SONARS_REAR_ON.name, DRIVERS_SONARS_REAR_ON.type, self._rear_on_cb)
        DRIVERS_SONARS_REAR_ON.Subscriber(self._rear_on_cb)
        self._rear_range = Float32MultiArray()
        self._rear_range_dim = MultiArrayDimension()
        self._rear_range_dim.label = "sonar_rear_group"
        self._rear_range_dim.size = len(SONAR_BUS_ADDR_Rear.values())
        self._rear_range_dim.stride = 0
        self._rear_range.layout.dim.append(self._rear_range_dim)

    def _initialize_sonar_address(self):
        if self._front_enable:
            self.sonar_name += SONAR_BUS_ADDR_Front.keys()
        if self._rear_enable:
            self._initialize_rear_sonar_rosmsg()
            # Cause whether to use rear soanr or front sonar is controlled by
            # _rear_on_cb now, so here we comment the following out
            #self.sonar_name += SONAR_BUS_ADDR_Rear.keys()
        if self._side_enable:
            self.sonar_name += SONAR_BUS_ADDR_Side.keys()

        # For recording component usage calculation.
        for n in self.sonar_name:
            self._record.add_component(n)

    def start(self):
        """
        Main loop
        """
        # initialize sonar with filter to reduce noise
        # use filter level 5
        self._initialize_sonar()

        # get low pass filter
        if self._use_low_pass_filter:
            self._initialize_filter()

        # main loop
        self._scan()
        self._record.shutdown()

    def _reduce_sonar_noise(self, address):
        pass

    def _measure(self, address, reg, command):
        return self.rosmsg_max_sonar_range_

    def _scan(self):
        _msg = Range()
        _msg.radiation_type = Range.ULTRASOUND
        _msg.max_range = self.rosmsg_max_sonar_range_
        _msg.min_range = self.rosmsg_min_sonar_range_
        _msg.field_of_view = self.rosmsg_sonar_fov_
        l = rospy.Rate(self.rate)
        has_range_error = False
        while not rospy.is_shutdown():
            raw_rear_range = {}
            # self.heartbeat_pub.publish(Empty())
            self.msg_status.stamp = rospy.Time.now()
            if has_range_error:
                # self.logerrcode(SONAR_FID, ErrCode.OA_ERR_SONAR.value, throttle_period=5.0)
                self.msg_status.state = 'ERROR'
                self.msg_status.errorcodes = [ErrCode.OA_ERR_SONAR.value]
            else:
                # self.logerrcode(SONAR_FID, ErrCode.OK.value)
                self.msg_status.state = 'RUNNING'
                self.msg_status.errorcodes = []
            self.pub_status.publish(self.msg_status)

            has_range_error = False
            for _i in self.sonar_name:
                self.count += 1
                _msg.header.seq = self.count
                _msg.header.frame_id = _i

                # read sonar measurement 1 by 1 in the chain
                # TODO we can use different filter `PROBE` here for side sonar
                _probe = CMD_PROBE[1]
                if self.use_small_probe is True and (_i.startswith("left_") or _i.startswith("right_")):
                    # Only when testing in small area.
                    _probe = CMD_PROBE[0]

                _range = self._measure(self.addr_name[_i], CMD_REG2, _probe)
                # TODO confirm whether we really need to filter this out?!?!?!
                if _range < 0:
                    has_range_error = True
                    continue

                if self._use_low_pass_filter:
                    new_range = float(self._low_pass_filter(_range))
                    self.logdebug("Raw is {} filtered is {}".format(_range, new_range))
                    _range = new_range

                _msg.header.stamp = rospy.Time.now()
                _msg.range = _range if _range < _msg.max_range else _msg.max_range

                if _i.startswith("r_0"):
                    raw_rear_range[_i] = _range
                else:
                    self._pub[_i].publish(_msg)

            if self._rear_enable:
                self._rear_range.data = list(raw_rear_range.values())
                self._rear_pub.publish(self._rear_range)

            self._record.aggregate(1)
            l.sleep()

    def _write_sonar_addr(self, old_addr, new_addr):
        self.loginfo("Changing address from {} to {}".format(old_addr, new_addr))
        self.loginfo("Finshed! Please reboot the KS103")
        return True

    def _initialize_sonar(self):
        '''
        Initialize all sonar sensors with correct setting
        '''
        self.loginfo("Initializing Sonars")
        for _i in self.sonar_name:
            self._reduce_sonar_noise(self.addr_name[_i])
            time.sleep(0.001)

        # required by KS103 User Manual
        time.sleep(2)
        self.loginfo("Sonars are initialized!")
        return

    # low pass filter to reduce sonar noise
    def _calculate_filter_params(self, cutoff, fs, order=5):
        nyq = 0.5 * fs
        normal_cutoff = cutoff / nyq
        b, a = butter(order, normal_cutoff, btype='low', analog=False)
        return b, a

    def _low_pass_filter(self, rawData):
        filterData = lfilter(self._filter_b, self._filter_a, [rawData])
        return filterData

    def _initialize_filter(self):
        # Filter requirements.absorder = 6
        filter_order = 1
        filter_fs = 50.0  # sample rate, Hz
        filter_cutoff = 20  # desired cutoff frequency of the filter, Hz

        # Get the filter coefficients so we can check its frequency response.
        self._filter_b, self._filter_a = self._calculate_filter_params(
            filter_cutoff, filter_fs, filter_order)

    def _rear_on_cb(self, msg):
        if msg.data:
            self.sonar_name = SONAR_BUS_ADDR_Rear.keys()
        else:
            self.sonar_name = SONAR_BUS_ADDR_Front.keys()

if __name__ == '__main__':
    sonar = FakeSonarDriver()
    sonar.start()
