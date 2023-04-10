#!/usr/bin/env python
# -*- coding: utf-8 -*-

from re import X
import time
import struct
import numpy as np

import math
import rospy
import std_msgs.msg as stmsgs

from boothbot_common.ros_logger_wrap import ROSLogging as Logging
from pymodbus.exceptions import ModbusIOException

from boothbot_calibration_tools.settings \
    import LEVEL_INCLINOMETER_UNIT

# from boothbot_calibration_tools.drivers.base import ModbusDriver

import tf.transformations as tftrans

from boothbot_calibration_tools.settings import(
    TRANS_BEACON,
    TRANS_BEACON_RCENTER,
    LASER_HEIGHT
)

import geometry_msgs.msg as gemsgs


class InclinometerDriver(Logging):
    def __init__(self,
                 modbus_client=None,
                 name="inclinometer_driver"):
        self.name = name
        super(InclinometerDriver, self).__init__(self.name)
        # self.__mb = modbus_client
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

    def get_inclinometer_data(self):
        xy_raw_list = self.mb_client.read_holding_registers(
            0x01, 4, unit=self.mb_id)

        time.sleep(0.01)

        if not (isinstance(xy_raw_list, ModbusIOException)):
            # time.sleep(0.01)
            # print(xy_raw_list.registers[0: 4])
            # self.loginfo("get inclinometer data {} ".format(
            #     xy_raw_list.registers[0: 4]))
            x_raw = (xy_raw_list.registers[0] << 16) | xy_raw_list.registers[1]
            y_raw = (xy_raw_list.registers[2] << 16) | xy_raw_list.registers[3]

            # self.loginfo("x data {}".format(x_raw))
            self.x_data = math.radians(
                self.bin_to_float("{0:b}".format(x_raw)))
            # self.loginfo("y data {}".format(y_raw))
            self.y_data = math.radians(
                self.bin_to_float("{0:b}".format(y_raw)))
            self.get_tf()
            return True

            # print("---------------------")
            # print(self.bin_to_float(b_x))
            # print(self.bin_to_float(b_y))
            # print("---------------------")

            # return float(self.bin_to_float(b_x)), float(self.bin_to_float(b_y))

        else:
            self.loginfo("read inclinometer error")
            return False

    def get_tf(self):

        # if self.inverse:
        #     euler = -self.y_data, -self.x_data, 0.0

        # a = gemsgs.Quaternion(*tftrans.quaternion_from_euler(*euler))
        # row, pitch, _ = tftrans.euler_from_quaternion(
        #     (a.x,
        #      a.y,
        #      a.z,
        #      a.w)
        # )
        self.timu = tftrans.compose_matrix(translate=(
            0., 0., 0.), angles=(-self.y_data, -self.x_data, 0.0))
        # print("imu: ", self.timu)

        if self._with_rotation:  # True
            # self._cba_count += 8
            # self._cba_count %= self._cba_resolution
            # self.cba = self._cba_count / self._cba_resolution * 2 * math.pi
            self.tbeacon_rcenter = tftrans.compose_matrix(
                translate=TRANS_BEACON_RCENTER, angles=(0., 0., self.cba))
            target2 = np.dot(
                tftrans.concatenate_matrices(
                    self.timu, self.tbase_rcenter, self.tbeacon_rcenter, self.tbeacon),
                self.vearth
            )
            self.offset_x_data = target2[0]
            self.offset_y_data = target2[1]
            # print("x", self.offset_x_data)
            # print("y", self.offset_y_data)

        # data = stmsgs.Float64MultiArray()
        self.offset.data = [self.offset_x_data, self.offset_y_data]
        self.offset_pub.publish(self.offset)
        # print("z", target2[2])

    def get_linclinometer_x(self):
        x_raw_list = self.mb_client.read_holding_registers(
            0x01, 2, unit=self.mb_id)
        time.sleep(0.01)

        if not (isinstance(x_raw_list, ModbusIOException)):
            x_raw = (x_raw_list.registers[0] << 16) | x_raw_list.registers[1]

        self.x_data = self.bin_to_float("{0:b}".format(x_raw))

    def set_zero_point(self):
        data = 0x01
        print("start relative zero point.")
        rwr = self.mb_client.write_register(0x0B, data, unit=self.mb_id)
        time.sleep(0.2)
        # return not (isinstance(rwr, ModbusIOException) or rwr.function_code > 0x80)    time.sleep(0.2)
        print(rwr)

    def save_settings(self):
        data = 0x00
        print("save all settings.")
        rwr = self.mb_client.write_register(0x0F, data, unit=self.mb_id)
        time.sleep(0.2)
        # return not (isinstance(rwr, ModbusIOException) or rwr.function_code > 0x80)    time.sleep(0.2)
        print(rwr)

    # def bin_to_float(self, b):
    #     """ Convert binary string to a float. """
    #     # bf = int_to_bytes(int(b, 2), 8)  # 8 bytes needed for IEEE 754 binary64.
    #     print(struct.unpack('>d', b)[0])
    #     return struct.unpack('>d', b)[0]

    def bin_to_float(self, binary):
        return struct.unpack('!f', struct.pack('!I', int(binary, 2)))[0]

    # def ieee745(self, N):  # ieee-745 bits (max 32 bit)
    #     a = int(N[0])        # sign,     1 bit
    #     b = int(N[1:9], 2)    # exponent, 8 bits
    #     c = int("1"+N[9:], 2)  # fraction, len(N)-9 bits

    #     print(float(-1)**a * c / (1 << (len(N)-9 - (b-127))))

    #     return float((-1)**a * c / (1 << (len(N)-9 - (b-127))))

    # N = "110000011010010011"  # str of ieee-745 bits
    # print( ieee745(N)  )  # -->  -20.59375

    @property
    def x(self):
        return self.x_data

    @property
    def y(self):
        return self.y_data

    @property
    def offset_x(self):
        return self.offset_x_data

    @property
    def offset_y(self):
        return self.offset_y_data


if __name__ == "__main__":
    mb = ModbusDriver()
    rospy.init_node("inclinometer_node")
    inc = InclinometerDriver(mb.client)
    while True:
        inc.get_inclinometer_data()
        # print("x", inc.x)
        # print("y", inc.y)
        time.sleep(0.1)
