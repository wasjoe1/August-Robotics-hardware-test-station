#!/usr/bin/env python
#encoding=utf-8
from __future__ import division, print_function

import time
import numpy as np

ODOM_POSE_COVARIANCE = [1e-4, 0, 0, 0, 0, 0,
                        0, 1e-4, 0, 0, 0, 0,
                        0, 0, 1e6, 0, 0, 0,
                        0, 0, 0, 1e6, 0, 0,
                        0, 0, 0, 0, 1e6, 0,
                        0, 0, 0, 0, 0, 1e-1]

ODOM_TWIST_COVARIANCE = [1e-4, 0, 0, 0, 0, 0,
                         0, 1e-4, 0, 0, 0, 0,
                         0, 0, 1e6, 0, 0, 0,
                         0, 0, 0, 1e6, 0, 0,
                         0, 0, 0, 0, 1e6, 0,
                         0, 0, 0, 0, 0, 1e-1]

# NOTE: This code is copied from boothbot_driver/scripts/cq2_driver_fake.py

class FakeIMDRDriverSerial(object):
    ''' Configuration Parameters
    '''
    def __init__(self, port="/dev/ttyUSB0", baudrate=9600, timeout=0.5):
        self.SUCCESS = 0
        self.FAIL = -1
        self.OVERTIME = -2

        self.vel_act = np.array([0., 0., 0.])
        self.vel_set = np.array([0., 0., 0.])
        self.acc_set = np.array([0.75, 0.4, 1.5])
        self._last_time = time.time()

    def connect(self):
        return True

    def close(self):
        ''' Close the serial port.
        '''
        return True

    def get_voltage(self):
        return self.SUCCESS, (29000.,)

    def drive_twist(self, v_x, v_y, v_rz):
        self.vel_set = np.array([v_x, v_y, v_rz])
        return self.SUCCESS, None

    def get_twist(self):
        now = time.time()
        dt = now - self._last_time
        self._last_time = now
        dv_max = self.acc_set * dt
        # print(dv_max)
        dv = self.vel_set - self.vel_act
        dv = map(min, dv_max, dv)
        dv = map(max, -dv_max, dv)
        # print(dv)
        self.vel_act += dv
        # print(self.vel_act)
        v_act_x, v_act_y, v_act_rz = self.vel_act
        return self.SUCCESS, (v_act_x, v_act_y, v_act_rz)

    def stop(self):
        ''' Stop all motors.
        '''
        return True
