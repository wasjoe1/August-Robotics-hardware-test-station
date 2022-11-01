#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
import os
import time

from guiding_beacon_system.drivers.laser_driver_py3 import LaserRangeFinder
from boothbot_calibration_tools.calibration_controller import CalibrationController
# TOLERANCE = (0.0005, 0.001)


class LionelCalibration(CalibrationController):
    def __init__(self, name, rate, states=None, transitions=None, commands=None, status_inf=None, srv_cmd_inf=None, need_robot_status=False, error_codes=None, laser=None):
        laser = LaserRangeFinder("")
        super().__init__(name, rate, states, transitions, commands,status_inf, srv_cmd_inf, need_robot_status, error_codes, laser)

    #TODO
    def _do_cameras_angle(self):
        pass
        self.laser.set_fake_data(4, 0)
        super(LionelCalibration, self)._do_cameras_angle()

    def kill_servos_node(self):
        self.logerr("kill lionel servos node.")
        os.system("rosnode kill /servos_stepper_driver")
        super(LionelCalibration, self).kill_servos_node()

    def servos_enable(self):
        self.loginfo("stepper enable")
        target = super(LionelCalibration, self).servos_enable()
        self.servo_move(target)
        time.sleep(0.1)
        super(LionelCalibration, self).servos_enable()


if __name__ == "__main__":
    rospy.init_node("lionel_calibration_controller")
    c = LionelCalibration("lionel_calibration_controller", 10)
    c.run()
