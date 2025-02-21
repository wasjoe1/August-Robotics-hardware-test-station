#!/usr/bin/env python
# -*- coding: utf-8 -*-
from __future__ import print_function, division

from re import X
import time
import struct
import boothbot_common.ros_logger as logger

import numpy as np
import yaml
import os

import math
import rospy
import std_msgs.msg as stmsgs
from  guiding_beacon_system_msgs.msg import ServosMoveActionGoal
from std_msgs.msg import String, Int16

from boothbot_common.ros_logger_wrap import ROSLogging as Logging
from pymodbus.exceptions import ModbusIOException

from boothbot_calibration_tools.settings \
    import LEVEL_INCLINOMETER_UNIT

from boothbot_calibration_tools.drivers.base import ModbusDriver
from boothbot_common.settings import  LOCAL_TMP_CONFIG_FOLDER_PATH_LIONEL

from std_msgs.msg import Float32MultiArray

import tf.transformations as tftrans
from boothbot_calibration_tools.utils import (
    sincurve,
    get_estimated_inclination,
    generate_yaw_array,
)


from guiding_beacon_system_msgs.ros_interfaces import (
    DRIVERS_INCLINOMETER_INCLINATION_FILTERED_RAD,
)

from boothbot_calibration_tools.settings import(
    TRANS_BEACON,
    TRANS_BEACON_RCENTER,
    LASER_HEIGHT
)

from boothbot_calibration_tools.constants import(
    CB_INCLI_CMD,
    CB_INCLI_STATE,
    CB_INCLI_RES
)

from boothbot_common.interface_with_type import InterfaceWithType

DRIVERS_INCLINOMETER_INCLINATION_CB = InterfaceWithType('/drivers/inclinometer/inclination_cb', stmsgs.Float32MultiArray)
DRIVERS_INCLINOMETER_INCLINATION_CB_FILTERED = InterfaceWithType('/drivers/inclinometer/inclination_cb_filtered', stmsgs.Float32MultiArray)
DRIVERS_INCLINOMETER_INCLINATION_CB_FILTERED_RAD = InterfaceWithType('/drivers/inclinometer/inclination_cb_filtered_rad', stmsgs.Float32MultiArray)


#######################################
from boothbot_driver.servos_client import ServosClient

DEFAULT_TIMES_PER_2PI = 17
DEFAULT_STABLIZE_TIMEOUT = 5.0

######################################

TEST_DATA_PATH = os.path.join(LOCAL_TMP_CONFIG_FOLDER_PATH_LIONEL, "inclination")
if not os.path.exists(TEST_DATA_PATH):
    os.makedirs(TEST_DATA_PATH)

class InclinometerDriver(Logging):
    def __init__(self,
                 modbus_client=None,
                 servo_cli=None,
                n_times_per_2pi=DEFAULT_TIMES_PER_2PI,
                stablize_timeout=DEFAULT_STABLIZE_TIMEOUT,
                 name="c"):
        self.name = name
        super(InclinometerDriver, self).__init__(self.name)

        self.servos_cli = ServosClient()

        #set radians valus
        self.N = n_times_per_2pi
        self.stablize_timeout = stablize_timeout
        self.gs_yaw_array = generate_yaw_array(self.N)
        rospy.Subscriber(CB_INCLI_CMD, String, self.cb_cli)
        self._puber_state = rospy.Publisher(CB_INCLI_STATE, Int16, queue_size=10)
        self._puber_result = rospy.Publisher(CB_INCLI_RES, String, queue_size=10)
        self.measured_inclinations = [

        ]

        self.start = None
        self.state = 0
        self.result = None
        self.inc_x = None
        self.inc_y = None

        self.x_data = 99.99
        self.y_data = 99.99
        self.mb_id = LEVEL_INCLINOMETER_UNIT

        self.inverse = True
        self._with_rotation = True
        # self.tf_xyz =
        self.offset_x_data = None
        self.offset_x_data = None

        self.offset_pub = rospy.Publisher(
            '/inclinometer', stmsgs.Float64MultiArray, queue_size=1)
        self.offset = stmsgs.Float64MultiArray()
        self.vearth = np.array([[0., 0., 0., 1.]]).T
        # self.vtarget = np.array([[0., 0., 0., 1.]]).T
        self.cba = 0.0
        self.msg_inclinometer_filtered_rad = (
            DRIVERS_INCLINOMETER_INCLINATION_FILTERED_RAD.type()
        )

        # DRIVERS_INCLINOMETER_INCLINATION_FILTERED_RAD.Subscriber(
        #     callback=self._inclination_cb
        # )
        DRIVERS_INCLINOMETER_INCLINATION_CB_FILTERED_RAD.Subscriber(self._inclination_cb)

        self.timu = tftrans.compose_matrix(
            translate=(0., 0., 0.), angles=(0.0, 0.0, 0.0))
        self.tbase_rcenter = tftrans.compose_matrix(
            translate=(0., 0., LASER_HEIGHT), angles=(0., 0., 0.))
        if self._with_rotation:  # True
            self.tbeacon = tftrans.compose_matrix(
                translate=TRANS_BEACON, angles=(0., 0., 0.))
        else:
            self.tbeacon_rcenter = tftrans.compose_matrix(
                translate=(0.0, 0.0, 0.0), angles=(0., 0., 0.))
            self.tbeacon = tftrans.compose_matrix(
                translate=(0.0, 0.0, 0.0), angles=(0., 0., 0.))

    def get_next_servos_radians(self):
        if self.gs_yaw_array:
            return (self.gs_yaw_array[0], 0.0)
        return None
    def add_measurement(self, CB_radians, inclinations):
        logger.loginfo("Got inclination: {} at servos_radians: {}".format(inclinations, CB_radians))
        self.measured_inclinations.append(
            {
                "CB_radians": CB_radians,
                "inclinations": inclinations,
            }
        )

    def cb_cli(self, msg):
        cmd = msg.data
        if cmd.startswith("start"):
            logger.loginfo("CB inclination start...")
            self.start = True
            self.state = 1
        if cmd == "reset":
            self.sub_state = 0
            self.inc_x = None
            self.inc_y = None
            self.gs_yaw_array = generate_yaw_array(self.N)
            self.start = False
            self.measured_inclinations = []

    def get_inclination_params(self):
        gs_yaw_radians = []
        inclinations_x = []
        inclinations_y = []
        for r in self.measured_inclinations:
            gs_yaw_radians.append(r["CB_radians"][0])
            inc_x, inc_y = r["inclinations"]
            inclinations_x.append(inc_x)
            inclinations_y.append(inc_y)
        return [
            get_estimated_inclination(gs_yaw_radians, inclinations_x),
            get_estimated_inclination(gs_yaw_radians, inclinations_y),
        ]

    def gen_action_result(self):
        param_x, param_y = self.get_inclination_params()
        inc_x = (
            sincurve(0.0, param_x[0], param_x[1], 0.0)
            + sincurve(-np.pi / 2, param_y[0], param_y[1], 0.0)
        ) / 2
        inc_y = (
            sincurve(np.pi / 2, param_x[0], param_x[1], 0.0)
            + sincurve(0.0, param_y[0], param_y[1], 0.0)
        ) / 2
        self.inc_x = "inc_x_"+str(inc_x)
        self.inc_y = "inc_y_"+str(inc_y)
        return [inc_x, inc_y]

    def save_log(self):
        self.row_pitch = self.gen_action_result()
        result_file_name = "{}/{}-result.yaml".format(TEST_DATA_PATH, time.strftime("%Y%m%d-%H_%M"))
        with open(result_file_name, "w") as f:
            data = {
                "stamp": time.time(),
                "Row": float(self.row_pitch[0]),
                "Pitch": float(self.row_pitch[1]),
                "inclinations": list(self.measured_inclinations),
            }
            yaml.dump(data, f, Dumper=yaml.CDumper)
            logger.loginfo("Get the result! Row: {}, Pitch:{}".format(self.row_pitch[0],self.row_pitch[1]))
            logger.loginfo("Result saved at: {}".format(result_file_name))


    def set_cb_radians(self):

        if self.sub_state == 0:
            if self.servos_cli.is_done():
                target_radians = self.get_next_servos_radians()
                if target_radians is not None:
                    self.servos_cli.move_to(target_radians)
                    self.sub_state = 1
                else:
                #finish getting radians to roll and pitch
                    self.save_log()
                    return True


        elif self.sub_state == 1:
            if self.servos_cli.is_succeeded():
                self.stablize_timer = rospy.Time.now() + rospy.Duration(
                self.stablize_timeout
                )
                self.sub_state = 2
            elif self.servos_cli.is_failed():
                logger.logwarn(
                    "This shouldn't happen, the servos must into errors here!"
                )
                return False
            elif self.servos_cli.is_active():
                logger.loginfo_throttle(1.0, "Waiting for servos to arrived...")
        elif self.sub_state == 2:
            logger.loginfo_throttle(1.0, "Waiting for inclinometer to stablize...")
            if rospy.Time.now() > self.stablize_timer:
                self.gs_yaw_array.pop(0)
                self.sub_state = 0
                self.add_measurement(
                    self.servos_cli.get_arrived_radians(),
                    self.msg_inclinometer_filtered_rad.data,
                )
        return None


    def _inclination_cb(self, msg):
        self.msg_inclinometer_filtered_rad = msg


    def get_tf(self):


        self.timu = tftrans.compose_matrix(translate=(
            0., 0., 0.), angles=(-self.y_data, -self.x_data, 0.0))


        if self._with_rotation:

            self.tbeacon_rcenter = tftrans.compose_matrix(
                translate=TRANS_BEACON_RCENTER, angles=(0., 0., self.cba))
            target2 = np.dot(
                tftrans.concatenate_matrices(
                    self.timu, self.tbase_rcenter, self.tbeacon_rcenter, self.tbeacon),
                self.vearth
            )
            self.offset_x_data = target2[0]
            self.offset_y_data = target2[1]



        self.offset.data = [self.offset_x_data, self.offset_y_data]
        self.offset_pub.publish(self.offset)


    def pub_res(self):
        if self.inc_x is not None and self.inc_y is not None:
            self._puber_result.publish(self.inc_x)
            rospy.sleep(0.1)
            self._puber_result.publish(self.inc_y)



    def bin_to_float(self, binary):
        return struct.unpack('!f', struct.pack('!I', int(binary, 2)))[0]

if __name__ == "__main__":
    mb = ModbusDriver()
    rospy.init_node("cb_inclinometer_node")
    inc = InclinometerDriver(mb.client)
    inc.servos_cli.connect()
    inc.sub_state = 0
    while True:
        rospy.sleep(0.1)
        if inc.start is True:
            ret = inc.set_cb_radians()
            if rospy.is_shutdown():
                logger.logfatal("Is shutting down....")
                break
            if ret is None:
                logger.loginfo("not done...")
            elif ret is False:
                logger.logfatal("Running failed, will exit...")
                inc.start = False
                inc.state = 99
                # break
            elif ret is True:
                logger.loginfo("All pose inclination got, now finishing...")
                inc.start = False
                inc.state = 2
                # break
        inc._puber_state.publish(inc.state)
        # inc._puber_state.publish(inc.)
        inc.pub_res()
