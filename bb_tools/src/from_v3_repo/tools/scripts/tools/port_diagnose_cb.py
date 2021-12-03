#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os
import redis
import struct
import time
from pymodbus.client.sync import ModbusSerialClient as ModbusClient

from common.enums import ConnectStatus
from utils import format_print


SERVO_PORT = '/dev/servos'
LONGCAM_PORT = '/dev/camera_long'
SHORTCAM_PORT = '/dev/camera_short'


def _check_port(port):
    if os.path.exists(port):
        return ConnectStatus.CONNECTED
    else:
        return ConnectStatus.NOTCONNECTED


def check_longcam(): return _check_port(LONGCAM_PORT)
def check_shortcam(): return _check_port(SHORTCAM_PORT)


def _check_unit(port, device, unit):
    client = ModbusClient(method='rtu', port=port, baudrate=57600, parity='N', timeout=0.1)
    if device == 'bed':
        temp_res = client.read_holding_registers(0xc, 1, unit=unit)
        if not temp_res.isError() and temp_res.registers[0] == 14:
            return True
    else:
        temp_res = client.read_input_registers(0x6, 2, unit=unit)
        if not temp_res.isError() and struct.pack('<HH', *temp_res.registers) == 'VSMD':
            return True
    return False


def check_servos():
    ret = []
    units = [('vmd', 1), ('bed', 2),
             ('vmd', 3), ('bed', 4), ]
    for (d, u) in units:
        ret.append(_check_unit(SERVO_PORT, d, u))
        time.sleep(0.1)
    if all(ret):
        return ConnectStatus.CONNECTED
    else:
        return ConnectStatus.NOTCONNECTED


def cb_check_diagnose():
    """ For simple call on cb, add this wrap fn to collect all diagnose results on CB """
    port_name_map = [
        ('Servos', check_servos),
        ('Longcam', check_longcam),
        ('Shortcam', check_shortcam),
    ]
    data = []
    for (name, fn) in port_name_map:
        try:
            res = fn()
            connected = res == ConnectStatus.CONNECTED
        except Exception:  # TODO make this more specific
            connected = False

        data.append({
            'name': name, 'connected': connected
        })

    return data


if __name__ == "__main__":
    results = cb_check_diagnose()
    print(format_print(results))

