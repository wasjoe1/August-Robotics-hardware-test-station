#!/usr/bin/env python
# -*- coding: utf-8 -*-
from __future__ import print_function, division

import rospy
from common import Logging
from boothbot_marking.settings import IO_UPDATE_RATE
from boothbot_msgs.ros_interfaces import DRIVERS_CHASSIS_IO

# Fake clock for triggering IOAutoOffTimer.
class FakeIOClock(Logging):
    DEFAULT_IO_STATE_MAX = 2**32 - 1
    DEFAULT_IO_STATE_MIN = 0

    def __init__(self):
        super(FakeIOClock, self).__init__('FakeIOClock')
        self._rate = IO_UPDATE_RATE / 2
        self._io_state = self.DEFAULT_IO_STATE_MAX
        self._io_pub = rospy.Publisher(DRIVERS_CHASSIS_IO.name, DRIVERS_CHASSIS_IO.type, queue_size=1)

    def run(self):
        pacemaker = rospy.Rate(self._rate)
        while not rospy.is_shutdown():
            msg = DRIVERS_CHASSIS_IO.type()
            msg.stamp = rospy.Time.now()
            msg.io_state = self._io_state
            self._io_pub.publish(msg)
            self._io_state = (
                self.DEFAULT_IO_STATE_MAX 
                if self._io_state == 0 
                else 0)
            pacemaker.sleep()

if __name__ == '__main__':
    rospy.init_node('io_clock', log_level=rospy.INFO)
    clock = FakeIOClock()
    clock.run()
