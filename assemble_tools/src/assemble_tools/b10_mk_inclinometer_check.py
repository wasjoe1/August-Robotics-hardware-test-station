#!/usr/bin/env python3

import time
import struct
import rospy

logger = rospy

from pymodbus.client.sync import ModbusSerialClient as ModbusClient
from assemble_tools.get_key import GetKey

INSTRUCTION = """
Key functions:
i: Initialize
c: Get current readings
s: Update settings and save
q: Exiting...
"""

AUTOLEVEL_INCLINOMETER_BAUDRATE = 115200
AUTOLEVEL_INCLINOMETER_UNIT = 3
BAUDRATE_CHECKLIST = (9600, 115200)
UNIT_ID_CHECKLIST = (1, 2, 3)

BAUDRATE_TABLE = {
    0x0000: 2400,
    0x0001: 4800,
    0x0002: 9600,
    0x0003: 19200,
    0x0004: 115200,
}


def reg2f(reg_1, reg_2):
    return struct.unpack("!f", struct.pack("!HH", reg_1, reg_2))[0]


class InclinometerDriver(object):
    def __init__(self, modbus_client, unit_id):
        super(InclinometerDriver, self).__init__()
        self.modbus_client = modbus_client
        self.unit_id = unit_id

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
        logger.loginfo("Set unit_id to: {}".format(unit_id))
        self.unit_id = unit_id
        return True

    def save_settings(self):
        data = 0x00
        logger.loginfo("Saving all settings.")
        rwr = self.modbus_client.write_register(0x0F, data, unit=self.unit_id)
        if not rwr.isError():
            logger.loginfo("Settings saved!")
            return True
        return False


NODE_NAME = "b10_mk_inclinometer_check"
NODE_RATE = 5.0


class B10InclinometerCheck(object):
    def __init__(self):
        super(B10InclinometerCheck, self).__init__()
        self.get_key = None
        self.driver = None
        self.modbus_config = {
            "method": "rtu",
            "port": "/dev/ttyUSB0",
            "baudrate": 115200,
            "parity": "N",
            "timeout": 0.5,
        }

    def initialize(self):
        self.get_key = GetKey()

    def run(self):
        logger.logwarn(INSTRUCTION)
        l = rospy.Rate(NODE_RATE)
        while not rospy.is_shutdown():
            key = self.get_key.get_key()
            if key in ("i", "I"):
                logger.logwarn("Trying to initialize...")
                logger.loginfo("Detecting module...")
                self.driver = None
                for baudrate in BAUDRATE_CHECKLIST:
                    self.modbus_config.update({"baudrate": baudrate})
                    for unit_id in UNIT_ID_CHECKLIST:
                        driver = InclinometerDriver(
                            ModbusClient(**self.modbus_config),
                            unit_id=unit_id,
                        )
                        x, y = driver.get_inclinometer_data_xy_deg()
                        if x is not None:
                            self.driver = driver
                            logger.loginfo(
                                "Connected inclinometer using unit_id: {}\nwith config: {}".format(
                                    unit_id,
                                    self.modbus_config
                                )
                            )
                        if self.driver is not None:
                            break
                    if self.driver is not None:
                        break

            if key in ("s", "S"):
                if self.driver is None:
                    logger.logwarn("Not initialized yet!!")
                else:
                    self.driver.set_baudrate(0x0004)
                    time.sleep(0.1)
                    self.driver.set_unit_id(3)
                    time.sleep(0.1)
                    self.driver.save_settings()
                    time.sleep(0.1)

            if key in ("c", "C"):
                if self.driver is None:
                    logger.logwarn("Not initialized yet!!")
                else:
                    logger.loginfo("Current reading is: {}".format(self.driver.get_inclinometer_data_xy_deg()))

            elif key in ("q", "Q"):
                logger.logwarn("Exiting...")
                self.get_key.end_get_key()
                break


if __name__ == "__main__":
    rospy.init_node(NODE_NAME, log_level=rospy.INFO)
    e4 = B10InclinometerCheck()
    e4.initialize()
    e4.run()
