#!/usr/bin/env python
# -*- coding: utf-8 -*-
from __future__ import print_function, division

# MODBUS_RTU = True
# UNIT = 0x1
import time
import rospy
import dynamic_reconfigure.server as dyn
from boothbot_marking.cfg import IODriverConfig
from serial import SerialException
from pymodbus.exceptions import ModbusIOException
from pymodbus.client.sync import ModbusSerialClient
from pymodbus.client.sync import ModbusTcpClient
# from pymodbus.diag_message import ReturnQueryDataRequest
# from common import Logging
# from common.ros_topics import IO_SET_DO, IO_RESET_DO, IO_OUTPUTS, IO_INPUTS, LED_FEEDBACK

# V3.0 archi
from boothbot_msgs.ros_interfaces import DRIVERS_IO_STATUS, DRIVERS_IO_PDO, DRIVERS_IO_SRV_CMD
import boothbot_msgs.srv as bbsrvs
from boothbot_marking.constants import IOStates
from boothbot_marking.io_auto_off_timer import IOAutoOffTimer

# For simulation purposes
from common import Logging
from overrides import overrides

# NOTE: This code is derived from boothbot_marking/src/boothbot_marking/io_driver.py.

DEFAULT_UNIT = 1
BIT_LED_VALUE = 256
FF16 = 2 ** 16 - 1

def bool2uint12(boollist):
    assert(len(boollist) == 12)
    ret = 0
    for i, b in enumerate(boollist):
        ret += int(b) << i
    return ret

def uint2bool12(unit16):
    assert(unit16 < (1<<12))
    ret = []
    for i in range(12):
        tmp = bool(unit16%2)
        ret.append(tmp)
        unit16 = unit16 >> 1
    return ret

def get_true_index(boollist):
    return [
        i+1
        for i, v in enumerate(boollist)
        if v
    ]

def get_param(params, key, default):
    if key in params:
        return params[key]
    return default

class FakeIODriver(Logging):
    _modbus_cfg = {}
    _last_outputs = 0
    _last_inputs = 0
    _last_set = 0
    _last_reset = 0
    _auto_off_time = [0]*12
    _reconfigured = False
    _failure_count = 0
    _timers = None
    _using_internal_timer = True

    def __init__(self):
        Logging.__init__(self, "IOD")
        rospy.init_node('io', log_level=rospy.INFO)
        rospy.on_shutdown(self._shutdown)

        rate = float(rospy.get_param("~rate", 20.0))
        l = rospy.Rate(rate)
        com_delay = rospy.Duration(0.015)

        MODBUS_RTU = bool(rospy.get_param("~rtu", True))
        self.unit = int(rospy.get_param("~unit", DEFAULT_UNIT))

        if MODBUS_RTU:
            self.loginfo("Using ModbusSerialClient.")
        else:
            self.loginfo("Using ModbusTcpClient.")

        self.loginfo("io driver is connected.")

        pub_status = rospy.Publisher(DRIVERS_IO_STATUS.name, DRIVERS_IO_STATUS.type, queue_size=1)
        self.msg_status = DRIVERS_IO_STATUS.type()
        self.msg_status.stamp = rospy.Time.now()
        self.msg_status.state = IOStates.INIT.name

        pub_pdo = rospy.Publisher(DRIVERS_IO_PDO.name, DRIVERS_IO_PDO.type, queue_size=1)
        msg_pdo = DRIVERS_IO_PDO.type()
        msg_pdo.stamp = rospy.Time.now()

        rospy.Service(DRIVERS_IO_SRV_CMD.name, DRIVERS_IO_SRV_CMD.type, self._srv_cmd_handler)

        # FIXME: hardware might not respond so fast, a hard delay trying to wait the hardware.
        # rospy.sleep(rospy.Duration(3.0))
        dyn.Server(IODriverConfig, self._reconfigure_cb)
        self._timers = IOAutoOffTimer()

        while not rospy.is_shutdown():
            self.loginfo_throttle(5.0, "Running...")

            try:
                if not self._check_failure(None):
                    self._last_inputs = bool2uint12([False for i in range(12)])
            except SerialException:
                pass

            self.msg_status.stamp = msg_pdo.stamp = rospy.Time.now()
            msg_pdo.inputs = self._last_inputs

            if not self._check_failure(None):
                self._last_outputs = bool2uint12([False for i in range(12)])

            msg_pdo.outputs = self._last_outputs

            msg_pdo.cmd_reset_io = self._last_reset
            msg_pdo.cmd_set_io = self._last_set
            pub_pdo.publish(msg_pdo)
            pub_status.publish(self.msg_status)

            # for plugins
            set_outputs = self._last_outputs

            # for auto off feature
            self._last_reset |= self._timers.get_need_reset()

            if self._last_set != 0:
                set_outputs |= self._last_set
                out_set = get_true_index(uint2bool12(self._last_set))
                self.loginfo('Set {}'.format(out_set))
            if self._last_reset != 0:
                set_outputs &= ~self._last_reset
                out_reset = get_true_index(uint2bool12(self._last_reset))
                self.loginfo('Reset {}'.format(out_reset))
            if set_outputs != self._last_outputs:
                self._timers.set_outputs(set_outputs)
                if not self._check_failure(None):
                    self._last_set = 0
                    self._last_reset = 0
            else:
                self._last_set = 0
                self._last_reset = 0

            if self._reconfigured is True:
                if self._using_internal_timer is False:
                    if not self._check_failure(None):
                        self._reconfigured = False
                else:
                    self._timers.update_auto_off_time(self._auto_off_time)
                    self._reconfigured = False

                if not self._reconfigured:
                    self.loginfo("Auto off time updated as: {}.".format(self._auto_off_time))

            l.sleep()

    def _shutdown(self):
        self.loginfo("Shutting down.")

    def _check_failure(self, result):
        self.msg_status.state = IOStates.IDLE.name
        return False

    def _srv_cmd_handler(self, req):
        res = bbsrvs.IOCtrlCommandResponse()
        if bbsrvs.IOCtrlCommandRequest.UPDATE == req.command:
            self._last_reset |= req.reset_io
            self._last_set |= req.set_io
            res.accepted = True

        return res

    def _set_do_cb(self, msg):
        self._last_set |= msg.data
        self._last_reset &= ~msg.data

    def _reset_do_cb(self, msg):
        self._last_reset |= msg.data
        self._last_set &= ~msg.data

    def _reconfigure_cb(self, config, level):
        rospy.loginfo('IOD: receive reconfigure with level: {}'.format(level))
        # print(config)
        if level == -1:
            # initial setting
            for i in range(1, 12):
                config['DO{:0>2d}'.format(i)] = 0

        self._auto_off_time = [
            config['DO01'],
            config['DO02'],
            config['DO03'],
            config['DO04'],
            config['DO05'],
            config['DO06'],
            config['DO07'],
            config['DO08'],
            config['DO09'],
            config['DO10'],
            config['DO11'],
            config['DO12'],
        ]

        self._reconfigured = True
        return config

if __name__ == '__main__':
    iod = FakeIODriver()
