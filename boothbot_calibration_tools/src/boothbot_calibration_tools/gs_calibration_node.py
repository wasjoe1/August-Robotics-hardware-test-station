#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy

from guiding_beacon_system.drivers.laser_driver_v3 import LaserRangeFinderGenerator
from boothbot_calibration_tools.calibration_controller import CalibrationController


class GSCalibration(CalibrationController):
    def __init__(self, name, rate, states=None, transitions=None, commands=None, status_inf=None, srv_cmd_inf=None, need_robot_status=False, error_codes=None, laser=None):
        laser = LaserRangeFinderGenerator.detect_laser_range_finder()
        super().__init__(name, rate, states, transitions, commands, status_inf, srv_cmd_inf, need_robot_status, error_codes, laser)


if __name__ == "__main__":
    rospy.init_node("gs_calibration_controller")
    c = GSCalibration("gs_calibration_controller", 10)
    c.run()
