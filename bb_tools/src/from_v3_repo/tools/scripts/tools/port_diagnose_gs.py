#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os
import serial
import usb.core
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

check_longcam = lambda: _check_port(LONGCAM_PORT)
check_shortcam = lambda: _check_port(SHORTCAM_PORT)

def check_servos():
    client = ModbusClient(
        method='rtu',
        port=SERVO_PORT,
        baudrate=115200,
        parity='E',
        timeout=0.5
    )
    rst_1 = client.read_holding_registers(0x4418, 7, unit=1)  # For servo 1
    if rst_1.isError():
        return ConnectStatus.NOTCONNECTED
    rst_2 = client.read_holding_registers(0x4418, 7, unit=2)  # For servo 2
    if rst_2.isError():
        return ConnectStatus.NOTCONNECTED
    return ConnectStatus.CONNECTED

def check_laser():
    """
    If SWQ120 or SKD100 detected, return True
    """
    # SWQ120
    swq120 = usb.core.find(idVendor=0x0483, idProduct=0x5710)
    if swq120:
        return ConnectStatus.CONNECTED
    # SKD100
    try:
        skd100 = serial.Serial(port='/dev/skd100d',
                               baudrate=9600,
                               parity='N',
                               timeout=0.002)
        if skd100.isOpen():
            return ConnectStatus.CONNECTED
    except serial.SerialException as e:
        pass
    return ConnectStatus.NOTCONNECTED

def gs_check_diagnose():
    """ Add this wrapper fn to collect all diagnose results on GS """
    port_name_map = [
        ('Servos', check_servos),
        ('Longcam', check_longcam),
        ('Shortcam', check_shortcam),
        ('Laser', check_laser),
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


if __name__=="__main__":
    results = gs_check_diagnose()
    print(format_print(results))
