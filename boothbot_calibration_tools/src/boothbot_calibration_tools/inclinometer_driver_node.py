#!/usr/bin/env python
# -*- coding: utf-8 -*-

import math
import rospy

logger = rospy

from boothbot_driver.modbus_driver import ModbusDriver
from guiding_beacon_system_measurement.inclinometer_driver_base import (
    InclinometerDriverBase,
    InclinometerDriver,
)
from guiding_beacon_system_measurement.first_order_kalman_filter import (
    FirstOrderKalmanFilter,
)

AUTOLEVEL_INCLINOMETER_UNIT = 1

import std_msgs.msg as stmsgs
from guiding_beacon_system_msgs.ros_interfaces import (
    DRIVERS_INCLINOMETER_STATUS,
    DRIVERS_INCLINOMETER_SRV_CMD,
    DRIVERS_INCLINOMETER_INCLINATION,
    DRIVERS_INCLINOMETER_INCLINATION_FILTERED,
    DRIVERS_INCLINOMETER_INCLINATION_FILTERED_RAD,
)

PORT_NAME = "/dev/incli"
NODE_NAME = "inclinometer_driver"
NODE_RATE = 10.0


class MKInclinometerDriverROS(object):
    def __init__(self, fake_driver=False):
        self.modbus_config = {
            "method": "rtu",
            "port": PORT_NAME,
            "baudrate": 9600,
            "parity": "N",
            "timeout": 0.5,
        }
        self.fake_driver = fake_driver
        self.modbus_driver = None
        self.sensor_driver = None
        self.msg_cmdword = None

        self.x_kf = FirstOrderKalmanFilter(Q=1e-5, R=2e-3)
        self.y_kf = FirstOrderKalmanFilter(Q=1e-5, R=2e-3)

    def start(self):
        self.initialize()
        self.run()

    def initialize(self):
        if self.fake_driver:
            logger.logwarn("Inclinometer driver: Node running with fake driver!!!")
            self.sensor_driver = InclinometerDriverBase()
        else:
            self.modbus_driver = ModbusDriver(**self.modbus_config)
            self.sensor_driver = InclinometerDriver(
                modbus_client=self.modbus_driver.client, unit_id=AUTOLEVEL_INCLINOMETER_UNIT
            )
        rospy.Subscriber(
            "/debug/incli_cmdword", stmsgs.String, callback=self._cmdword_cb
        )
        self.pub_data = DRIVERS_INCLINOMETER_INCLINATION.Publisher()
        self.pub_data_filtered = DRIVERS_INCLINOMETER_INCLINATION_FILTERED.Publisher()
        self.pub_data_rad_filtered = (
            DRIVERS_INCLINOMETER_INCLINATION_FILTERED_RAD.Publisher()
        )

    def run(self):
        l = rospy.Rate(NODE_RATE)
        while not rospy.is_shutdown():
            if self.msg_cmdword is not None:
                if self.msg_cmdword.data == "SAVE":
                    self.sensor_driver.save_settings()
                elif self.msg_cmdword.data == "SET_ZERO":
                    self.sensor_driver.set_zero_point()
                self.msg_cmdword = None

            x, y = self.sensor_driver.get_inclinometer_data_xy_deg()
            if x is not None:
                # filter result
                kf_x = self.x_kf.update_observed(x)
                kf_y = self.y_kf.update_observed(y)

                # inclination original data (degree)
                msg = stmsgs.Float32MultiArray()
                msg.data = [x, y]
                self.pub_data.publish(msg)

                # inclination filtered data (degree)
                msg = stmsgs.Float32MultiArray()
                msg.data = [kf_x, kf_y]
                self.pub_data_filtered.publish(msg)

                # inclination filtered data (radian)
                msg = stmsgs.Float32MultiArray()
                msg.data = [math.radians(kf_x), math.radians(kf_y)]
                self.pub_data_rad_filtered.publish(msg)
            l.sleep()

    def _cmdword_cb(self, msg):
        self.msg_cmdword = msg


if __name__ == "__main__":
    rospy.init_node(NODE_NAME, log_level=rospy.INFO)
    fake_driver = bool(rospy.get_param("~fake_driver", False))
    auto_level_enabled = bool(rospy.get_param("~auto_level_enabled", True))
    driver = MKInclinometerDriverROS(fake_driver=(fake_driver or not auto_level_enabled))
    driver.start()
