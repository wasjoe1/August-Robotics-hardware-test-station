#!/usr/bin/env python
# -*- coding: utf-8 -*-
from __future__ import print_function, division
from os import spawnlp

import sys
import time
import struct
import numpy

from numpy.lib.function_base import average, median
import rospy

from guiding_beacon_system.drivers.inclinometer_driver import \
    InclinometerDriver

from guiding_beacon_system.drivers.stepper_base import \
    ModbusDriver

from guiding_beacon_system.settings import NODE_NAME, SERVO_ACTION_NS

import actionlib

from guiding_beacon_system_msgs.msg import ServoMoveAction, ServoMoveGoal
import rospy

from guiding_beacon_system.drivers.stepper_base import VinceMotorDriver

from guiding_beacon_system.drivers.driver_constants import MotorCMD
from rospy.timer import sleep
import math

# -30 and 150
DEGREE60 = -0.5235987755982988
DEGREE_120 = 2.6179938779914944

# DEGREE60 = -1.0471975511965976
# DEGREE_120 = 2.0943951023931953

I60 = "60"
I_120 = "-120"

THRE = 0.003
SPEED = 5000
RECORD_DATA_TIME = 10
WAITING_TIME = 1.5
NUM_REFERENCE_DATA = 10
LITTLE_SLEEP_TIME = 0.1
STOP_SPEED = 0.0
ARRIVED_COUNT = 3


class InclinometerCali():
    def __init__(self,
                 modbus_client=None,
                 name="inclinometer_driver"):
        self._stepper_mb = ModbusDriver()
        self.incli_ctl = InclinometerDriver(self._stepper_mb.client)

        self._ac = actionlib.SimpleActionClient(SERVO_ACTION_NS,
                                                ServoMoveAction)

        self.rad = 0.0
        self._steppers = {1: None, 2: None}

        self.incli_data = {I60: 99.99, I_120: 99.99}
        self._arrived_count = {1: 0, 2: 0}
        self.current_pos = None

        self.zero_list = []

    def init(self):
        self.enable_motor()

    def move_absolute(self, rad_hor, tolerance=0.001):
        self._ac.wait_for_server()
        print("Got new target {},{}".format(rad_hor, 0.0))
        goal = ServoMoveGoal(loop_id=1, target=[
                             rad_hor, 0.0], tolerance=tolerance)
        self._ac.send_goal(goal)
        time.sleep(LITTLE_SLEEP_TIME)

    def first_loop(self):

        if self.rad == 0.0:
            self.rad = DEGREE60
        elif self.rad == DEGREE60:
            self.rad = DEGREE_120
        elif self.rad == DEGREE_120:
            self.rad = DEGREE60
        (x, y) = self.get_average_data()
        self.record_x(x)
        self.move_absolute(self.rad)

    def get_average_data(self):
        time.sleep(WAITING_TIME)
        total_x = 0.0
        total_y = 0.0
        # get averager
        for _ in range(RECORD_DATA_TIME):
            self.get_xy()
            total_x += self.incli_ctl.x
            total_y += self.incli_ctl.y
            time.sleep(LITTLE_SLEEP_TIME)
        x = total_x/RECORD_DATA_TIME
        y = total_y/RECORD_DATA_TIME
        return x, y

    def record_x(self, x):
        if self.rad == DEGREE60:
            self.incli_data[I60] = x
            print("in 60 degree x data {}".format(self.incli_data[I60]))
        elif self.rad == DEGREE_120:
            self.incli_data[I_120] = x
            print("in -120 degree x data {}".format(self.incli_data[I_120]))

    def loop(self):
        # rotate two direction and record the x data
        self.one_loop()
        goal = self.compute_goal()
        time.sleep(WAITING_TIME)
        self.motor_motion(1, self.rad, goal)
        self._steppers[1].move_speed(STOP_SPEED)
        self.calc_record()
        if self.rad == 0.0:
            self.rad = DEGREE60
        elif self.rad == DEGREE60:
            self.rad = DEGREE_120
        elif self.rad == DEGREE_120:
            self.rad = DEGREE60
        self.move_absolute(self.rad)
        time.sleep(WAITING_TIME)
        self.motor_motion(1, self.rad, goal)
        self._steppers[1].move_speed(STOP_SPEED)
        time.sleep(WAITING_TIME)
        (x, y) = self.get_average_data()
        self.zero_list.append(x)
        print(self.zero_list)

        if len(self.zero_list) > NUM_REFERENCE_DATA:
            a = numpy.median(self.zero_list)
            print("median zeor is {}".format(a))
            if math.fabs(x-a) < THRE:
                print("ok, set zero point. now zero point {}".format(x))
                self.incli_ctl.set_zero_point()
                print("save all settings.")
                self.incli_ctl.save_settings()
                exit("calibration inclinometer done.")
        else:
            print("zero point list is less than 10.")

    def calc_record(self):
        total_x = 0.0
        total_y = 0.0
        for _ in range(10):
            self.get_xy()
            total_x += self.incli_ctl.x
            total_y += self.incli_ctl.y
            time.sleep(0.1)
        x = total_x/10
        # y = total_y/10
        print("x {}".format(x))
        # print("y {}".format(y))
        self.record_x(x)

    def motor_motion(self, id, where, goal):
        self._arrived_count[id] = 0
        while True:
            self.get_xy()
            if self.rad == DEGREE_120:
                degree = "-120"
                offset = self.incli_ctl.x - goal
                # offset = goal - self.incli_ctl.x
            else:
                degree = "60"
                offset = goal - self.incli_ctl.x
                # offset = self.incli_ctl.x - goal

            # if offset > 0:
            speed = SPEED*offset
            # else:
            # speed = SPEED*offset
            if math.fabs(offset) > THRE:
                self._steppers[id].move_speed(speed)
                self._arrived_count[id] = 0
                print("in {}, x {}, goal is {}, offset is {}, speed is {}, arrive count {}".format(
                    degree, self.incli_ctl.x, goal, offset, speed, self._arrived_count[id]))
            else:
                print("in {}, x {}, goal is {}, offset is {}, speed is 0.0, arrive count {}".format(
                    degree, self.incli_ctl.x, goal, offset, self._arrived_count[id]))
                self._arrived_count[id] += 1

            if self._arrived_count[id] > ARRIVED_COUNT:
                self._steppers[id].move_speed(STOP_SPEED)
                break
            self.run_motor(id)
            time.sleep(0.05)

    def compute_goal(self):
        print("compute_goal now.")
        goal = (self.incli_data[I60] + self.incli_data[I_120])/2
        print("goal is {}".format(goal))
        print("compute_goal done.")
        return goal

    def run_motor(self, id):
        self._steppers[id].speed_mode_move()

    def enable_motor(self):
        self._steppers[1] = VinceMotorDriver(modbus_client=self._stepper_mb.client,
                                             unit=1)
        self._steppers[2] = VinceMotorDriver(modbus_client=self._stepper_mb.client,
                                             unit=2)

        self.set_division()
        # self.set_io_type()

        self._steppers[1].set_current(2.0, 2.0, 1.0)  # maximum 2.8
        self._steppers[1].set_dynamic(4.8e5, 4.8e5)
        self._steppers[1]._move_speed(0.0)

        self._steppers[2].set_current(2.0, 2.0, 1.0)  # maximum 2.8
        self._steppers[2].set_dynamic(4.8e5, 4.8e5)
        self._steppers[2]._move_speed(0.0)

        print("motor 1 enable {}".format(
            self._steppers[1].enable(MotorCMD.MODE_CONTINUOUS.name)))
        time.sleep(WAITING_TIME)
        print("motor 2 enable {}".format(
            self._steppers[2].enable(MotorCMD.MODE_CONTINUOUS.name)))
        time.sleep(WAITING_TIME)

    def get_x(self):
        self.incli_ctl.get_inclinometer_data()

    def get_xy(self):
        self.incli_ctl.get_inclinometer_data()

    def set_division(self):
        time.sleep(1)
        print("1 motor the division is {}".format(
            self._steppers[1]._division))
        print("2 motor the division is {}".format(
            self._steppers[2]._division))

        self._steppers[1]._refresh_params()
        self._steppers[2]._refresh_params()

    def one_loop(self):
        self.first_loop()
        time.sleep(WAITING_TIME)
        self.first_loop()
        time.sleep(WAITING_TIME)


if __name__ == "__main__":
    rospy.init_node("cali_incli")
    inc_cali = InclinometerCali()
    inc_cali.init()
    print("{}".format(inc_cali.incli_data))
    while True:
        inc_cali.loop()
