#!/usr/bin/env python
# -*- coding: utf-8 -*-

import math
import rospy

logger = rospy

from boothbot_driver.modbus_driver import ModbusDriver
from boothbot_calibration_tools.drivers.inclinometer_driver_base import (
    InclinometerDriverBase,
    InclinometerDriver,
)
from boothbot_calibration_tools.first_order_kalman_filter import (
    FirstOrderKalmanFilter,
)


AUTOLEVEL_INCLINOMETER_UNIT = 1
LIONEL_LEVEL_INCLINOMETER_UNIT = 3

import std_msgs.msg as stmsgs
from std_msgs.msg import Float32MultiArray

from boothbot_common.interface_with_type import InterfaceWithType

DRIVERS_INCLINOMETER_INCLINATION_CB = InterfaceWithType('/drivers/inclinometer/inclination_cb', stmsgs.Float32MultiArray)
DRIVERS_INCLINOMETER_INCLINATION_CB_FILTERED = InterfaceWithType('/drivers/inclinometer/inclination_cb_filtered', stmsgs.Float32MultiArray)
DRIVERS_INCLINOMETER_INCLINATION_CB_FILTERED_RAD = InterfaceWithType('/drivers/inclinometer/inclination_cb_filtered_rad', stmsgs.Float32MultiArray)

from guiding_beacon_system_msgs.ros_interfaces import (
    DRIVERS_INCLINOMETER_INCLINATION,
    DRIVERS_INCLINOMETER_INCLINATION_FILTERED,
    DRIVERS_INCLINOMETER_INCLINATION_FILTERED_RAD,
)

from boothbot_calibration_tools.settings import (
    CB_INCLI_PORT_NAME,
    LIONEL_INCLI_PORT_NAME,
    D_INCLI_INCLI_CB_RAD_FILTERED
)

from boothbot_calibration_tools.utils import two_incli

NODE_NAME = "inclinometer_driver"
NODE_RATE = 10.0

PORT_PREFIX = "/dev/ttyUSB"

class MKInclinometerDriverROS(object):
    def __init__(self, fake_driver=False, two_drivers=None):
        self.incli_feature = {1:"Lionel_level", 3:"cb_calibration"}
        self.two_drivers = two_drivers
        self.fake_driver = fake_driver
        self.modbus_driver = [None, None]
        self.sensor_driver = []
        self.msg_cmdword = None

        self.x_kf = [FirstOrderKalmanFilter(Q=1e-5, R=2e-3), FirstOrderKalmanFilter(Q=1e-5, R=2e-3)]
        self.y_kf = [FirstOrderKalmanFilter(Q=1e-5, R=2e-3), FirstOrderKalmanFilter(Q=1e-5, R=2e-3)]

    def start(self):
        self.sensor_driver.append(self.find_inclinometer(1))
        self.sensor_driver.append(self.find_inclinometer(3))
        self.initialize()
        self.run()

    def find_inclinometer(self, unit):
        baudrate_list = [9600, 115200]
        # find device /dev/ttyUSB* in range(0-10), the id 1 means lionel base, id 3 means lionel cb.
        for b in baudrate_list:
            for i in range(0,10):
                modbus_config = {
                    "method": "rtu",
                    "port": PORT_PREFIX+str(i),
                    "baudrate": b,
                    "parity": "N",
                    "timeout": 0.5,
                }
                modbus_driver = ModbusDriver(**modbus_config)
                # if            
                senser_driver = InclinometerDriver(modbus_driver.client, unit_id=unit)
                x = None
                y = None
                try:
                    x, y = senser_driver.get_inclinometer_data_xy_deg()
                except Exception as e:
                    logger.loginfo(e)
                if x is not None:
                    logger.loginfo("ttyUSB: {} has selected for unit {}, for {}".format(i, unit, self.incli_feature[unit]))
                    return senser_driver
        return None

    def initialize(self):
        if self.fake_driver:
            logger.logwarn("Inclinometer driver: Node running with fake driver!!!")
            self.sensor_driver = [InclinometerDriverBase(),InclinometerDriverBase()]
        else:
            pass
            # if self.two_drivers is True:
            #     logger.loginfo("Two inclinometers drivers!!!")
            #     self.modbus_driver = [ModbusDriver(**(self.modbus_config[0])), ModbusDriver(**(self.modbus_config[1]))]
            #     self.sensor_driver = [
            #         InclinometerDriver(
            #             modbus_client=self.modbus_driver[0].client, unit_id=AUTOLEVEL_INCLINOMETER_UNIT
            #         ),
            #         InclinometerDriver(
            #             modbus_client=self.modbus_driver[1].client, unit_id=LIONEL_LEVEL_INCLINOMETER_UNIT
            #         )
            #     ]
            # # if self.two_drivers is False:
            #     logger.loginfo("One inclinometer drivers!!!")
            #     self.modbus_driver = [ModbusDriver(**(self.modbus_config[0])), None]
            #     self.sensor_driver = [
            #         InclinometerDriver(
            #             modbus_client=self.modbus_driver[0].client, unit_id=AUTOLEVEL_INCLINOMETER_UNIT
            #         ),
            #         None
            #     ]
            # if self.two_drivers is None:
            #     logger.loginfo("No inclinometer driver!!!")

        rospy.Subscriber(
            "/debug/incli_cmdword", stmsgs.String, callback=self._cmdword_cb
        )

        DRIVERS_INCLINOMETER_INCLINATION_CB = InterfaceWithType('/drivers/inclinometer/inclination_cb', stmsgs.Float32MultiArray)
        DRIVERS_INCLINOMETER_INCLINATION_CB_FILTERED = InterfaceWithType('/drivers/inclinometer/inclination_cb_filtered', stmsgs.Float32MultiArray)
        DRIVERS_INCLINOMETER_INCLINATION_CB_FILTERED_RAD = InterfaceWithType('/drivers/inclinometer/inclination_cb_filtered_rad', stmsgs.Float32MultiArray)

        self.pub_data_lionel_incli = DRIVERS_INCLINOMETER_INCLINATION_CB.Publisher()
        self.pub_data_lionel_incli_filtered = DRIVERS_INCLINOMETER_INCLINATION_CB_FILTERED.Publisher()
        self.pub_data_lionel_incli_rad_filtered = DRIVERS_INCLINOMETER_INCLINATION_CB_FILTERED_RAD.Publisher()

        self.pub_data = DRIVERS_INCLINOMETER_INCLINATION.Publisher()
        self.pub_data_filtered = DRIVERS_INCLINOMETER_INCLINATION_FILTERED.Publisher()
        self.pub_data_rad_filtered = (
            DRIVERS_INCLINOMETER_INCLINATION_FILTERED_RAD.Publisher()
        )
        self.pub = [self.pub_data, self.pub_data_lionel_incli]
        self.pub_filtered = [self.pub_data_filtered, self.pub_data_lionel_incli_filtered]
        self.pub_rad_filtered = [self.pub_data_rad_filtered, self.pub_data_lionel_incli_rad_filtered]

    def run(self):
        l = rospy.Rate(NODE_RATE)
        while not rospy.is_shutdown():
            # if self.msg_cmdword is not None:
            #     if self.msg_cmdword.data == "SAVE":
            #         self.sensor_driver.save_settings()
            #     elif self.msg_cmdword.data == "SET_ZERO":
            #         self.sensor_driver.set_zero_point()
            #     self.msg_cmdword = None
            for sid, sensor in enumerate(self.sensor_driver):
                if sensor is None:
                    continue
                x, y = sensor.get_inclinometer_data_xy_deg()
                if x is not None:
                    # filter result
                    kf_x = self.x_kf[sid].update_observed(x)
                    kf_y = self.y_kf[sid].update_observed(y)

                    # inclination original data (degree)
                    msg = stmsgs.Float32MultiArray()
                    msg.data = [x, y]
                    self.pub[sid].publish(msg)

                    # inclination filtered data (degree)
                    msg = stmsgs.Float32MultiArray()
                    msg.data = [kf_x, kf_y]
                    self.pub_filtered[sid].publish(msg)

                    # inclination filtered data (radian)
                    msg = stmsgs.Float32MultiArray()
                    msg.data = [math.radians(kf_x), math.radians(kf_y)]
                    self.pub_rad_filtered[sid].publish(msg)
            l.sleep()

    def _cmdword_cb(self, msg):
        self.msg_cmdword = msg


if __name__ == "__main__":
    rospy.init_node(NODE_NAME, log_level=rospy.INFO)
    two_drivers = two_incli(CB_INCLI_PORT_NAME,LIONEL_INCLI_PORT_NAME)
    fake_driver = bool(rospy.get_param("~fake_driver", False))
    auto_level_enabled = bool(rospy.get_param("~auto_level_enabled", True))
    driver = MKInclinometerDriverROS(fake_driver=(fake_driver or not auto_level_enabled), two_drivers=two_drivers)
    driver.start()
