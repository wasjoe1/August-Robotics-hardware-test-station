#!/usr/bin/env python
# -*- coding: utf-8 -*-

import struct
import rospy

logger = rospy


AUTOLEVEL_INCLINOMETER_UNIT = 3

BAUDRATE_TABLE = {
    0x0000: 2400,
    0x0001: 4800,
    0x0002: 9600,
    0x0003: 19200,
    0x0004: 115200,
}


def reg2f(reg_1, reg_2):
    return struct.unpack("!f", struct.pack("!HH", reg_1, reg_2))[0]


class InclinometerDriverBase(object):
    def __init__(self, modbus_client=None, unit_id=None):
        super(InclinometerDriverBase, self).__init__()
        self.modbus_client = modbus_client
        self.unit_id = unit_id

    def get_inclinometer_data_xy_deg(self):
        return 0.0, 0.0

    def set_zero_point(self):
        return True

    def set_baudrate(self, baudrate_id):
        return True

    def set_unit_id(self, unit_id):
        return True

    def save_settings(self):
        return True


class InclinometerDriver(InclinometerDriverBase):
    def __init__(self, modbus_client, unit_id):
        super(InclinometerDriver, self).__init__(
            modbus_client=modbus_client, unit_id=unit_id
        )

    def get_inclinometer_data_xy_deg(self):
        rhr = self.modbus_client.read_holding_registers(0x01, 4, unit=self.unit_id)
        if rhr.isError():
            return None, None
        x = reg2f(*rhr.registers[0:2])
        y = reg2f(*rhr.registers[2:4])
        return x, y

    def set_zero_point(self):
        data = 0x01
        logger.loginfo("Setting relative zero point")
        rwr = self.modbus_client.write_register(0x0B, data, unit=self.unit_id)
        if not rwr.isError():
            logger.loginfo(
                "Set {} zero point".format(["Relative" if data else "Absolute"])
            )
            return True
        return False

    def set_baudrate(self, baudrate_id):
        if baudrate_id not in BAUDRATE_TABLE:
            return False
        logger.loginfo("Setting baudrate to: {}".format(BAUDRATE_TABLE[baudrate_id]))
        rwr = self.modbus_client.write_register(0x0C, baudrate_id, unit=self.unit_id)
        if not rwr.isError():
            logger.loginfo("Set baudrate to: {}".format(BAUDRATE_TABLE[baudrate_id]))
            return True
        return False

    def set_unit_id(self, unit_id):
        logger.loginfo("Setting unit_id to: {}".format(unit_id))
        rwr = self.modbus_client.write_register(0x0D, unit_id, unit=self.unit_id)
        if not rwr.isError():
            logger.loginfo("Set unit_id to: {}".format(unit_id))
            self.unit_id = unit_id
            return True
        return False

    def save_settings(self):
        data = 0x00
        logger.loginfo("Saving all settings.")
        rwr = self.modbus_client.write_register(0x0F, data, unit=self.unit_id)
        if not rwr.isError():
            logger.loginfo("Settings saved!")
            return True
        return False
