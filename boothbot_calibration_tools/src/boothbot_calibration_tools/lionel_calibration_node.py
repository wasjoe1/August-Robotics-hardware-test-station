#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
import os
import time

from boothbot_calibration_tools.drivers.laser_driver_py3 import LaserRangeFinder
from boothbot_calibration_tools.calibration_controller import CalibrationController
from boothbot_calibration_tools.utils import have_short_camera

from boothbot_painter.painter_client import PainterClient

STEPPER_MAX_ENCODING = (1 << 14) - 1

class LionelCalibration(CalibrationController):
    def __init__(self, name, rate, states=None, transitions=None, commands=None, status_inf=None, srv_cmd_inf=None, need_robot_status=False, error_codes=None, laser=None):
        laser = LaserRangeFinder("")
        painter = PainterClient()
        use_short_camera = have_short_camera()
        super().__init__(name, rate, states, transitions, commands,status_inf, srv_cmd_inf, need_robot_status, error_codes, laser, STEPPER_MAX_ENCODING, painter, use_short_camera)

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
        self.loginfo("lionel stepper enable")
        super(LionelCalibration, self).get_servo_radians()
        self.servo_move(self.track_target)
        time.sleep(0.1)
        super(LionelCalibration, self).servos_enable()


if __name__ == "__main__":
    rospy.init_node("lionel_calibration_controller")
    c = LionelCalibration("lionel_calibration_controller", 10)
    c.run()
