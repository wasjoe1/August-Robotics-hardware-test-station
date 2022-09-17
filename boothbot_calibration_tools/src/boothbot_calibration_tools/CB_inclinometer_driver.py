#!/usr/bin/env python
# -*- coding: utf-8 -*-
from __future__ import print_function, division

from re import X
import time
import struct
from xml.dom.minidom import ReadOnlySequentialNamedNodeMap
import numpy as np

import math
import rospy
import std_msgs.msg as stmsgs
from  guiding_beacon_system_msgs.msg import ServosMoveActionGoal


from boothbot_common.ros_logger_wrap import ROSLogging as Logging
from pymodbus.exceptions import ModbusIOException

from boothbot_calibration_tools.settings \
    import LEVEL_INCLINOMETER_UNIT

from boothbot_calibration_tools.drivers.base import ModbusDriver

import tf.transformations as tftrans
from guiding_beacon_system_measurement.utils import (
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




#######################################
from boothbot_driver.servos_client import ServosClient

DEFAULT_TIMES_PER_2PI = 17
DEFAULT_STABLIZE_TIMEOUT = 5.0

######################################

class InclinometerDriver(Logging):
    def __init__(self,
                 modbus_client=None,
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

        self.measured_inclinations = [

        ]

        #set inclinometer drivers     
        self.__mb = modbus_client
        self.mb_client = self.__mb
        

        # md = ModbusDriver()
        # self.mb_client = md.client

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

        DRIVERS_INCLINOMETER_INCLINATION_FILTERED_RAD.Subscriber(
            callback=self._inclination_cb
        )

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
        rospy.loginfo("Got inclination: {} at servos_radians: {}".format(inclinations, CB_radians))
        # self.measured_inclinations.append(
        #     {
        #         "CB_radians": CB_radians,
        #         "inclinations": inclinations,
        #     }
        # )

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


    def set_cb_radians(self):
        
        if self.sub_state == 0:
            if self.servos_cli.is_done():
                target_radians = self.get_next_servos_radians()
                if target_radians is not None:
                    self.servos_cli.move_to(target_radians)
                    self.sub_state = 1
            else:
                #finish getting radians to roll and pitch
                rospy.loginfo("All pose inclination got, now finishing...")  


        elif self.sub_state == 1:
            if self.servos_cli.is_succeeded(): 
                self.stablize_timer = rospy.Time.now() + rospy.Duration(
                self.stablize_timeout
                )
                self.sub_state = 2
            elif self.servos_cli.is_failed():
                rospy.logwarn(
                    "This shouldn't happen, the servos must into errors here!"
                ) 
            elif self.servos_cli.is_active():
                rospy.loginfo_throttle(1.0, "Waiting for servos to arrived...")
        elif self.sub_state == 2:
            rospy.loginfo_throttle(1.0, "Waiting for inclinometer to stablize...")
            if rospy.Time.now() > self.stablize_timer:
                self.gs_yaw_array.pop(0)
                self.sub_state = 0
                self.add_measurement(
                    self.servos_cli.get_arrived_radians(),
                    self.msg_inclinometer_filtered_rad.data,
                )


    def _inclination_cb(self, msg):
        self.msg_inclinometer_filtered_rad = msg

                    





    def get_inclinometer_data(self):
        xy_raw_list = self.mb_client.read_holding_registers(
            0x01, 4, unit=self.mb_id)

        time.sleep(0.01)

        if not (isinstance(xy_raw_list, ModbusIOException)):

            x_raw = (xy_raw_list.registers[0] << 16) | xy_raw_list.registers[1]
            y_raw = (xy_raw_list.registers[2] << 16) | xy_raw_list.registers[3]

            self.x_data = math.radians(
                self.bin_to_float("{0:b}".format(x_raw)))
            self.y_data = math.radians(
                self.bin_to_float("{0:b}".format(y_raw)))
            self.get_tf()
            return True


        else:
            self.loginfo("read inclinometer error")
            return False

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





    def bin_to_float(self, binary):
        return struct.unpack('!f', struct.pack('!I', int(binary, 2)))[0]

if __name__ == "__main__":
    mb = ModbusDriver()
    rospy.init_node("cb_inclinometer_node")
    inc = InclinometerDriver(mb.client)
    inc.servos_cli.connect()
    inc.sub_state = 0
    while True:
        inc.set_cb_radians()
    

