#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os
import time
import subprocess
import crcmod
from serial import Serial, SerialException
from pymodbus.client.sync import ModbusSerialClient
from pymodbus.exceptions import ModbusIOException

from common.enums import ConnectStatus
from common.gs_settings import HAS_AUX_PC, NEW_BASE, IMU_AS_INCLINOMETER, \
    IS_BOOTHNUMBER
from port_diagnose_cb import cb_check_diagnose
from utils import format_print


LIDAR_PORT = '/dev/lidar'
IMU_PORT = '/dev/imu'
SONAR1_PORT = '/dev/sonar1'
MARKING_CAMERA_PORT = '/dev/camera_marking'
IO_PORT = '/dev/io_module'
HANGFA_PORT = '/dev/mobile_base'
CLINOMETER_PORT = '/dev/clinometer'
ROBOMASTER_PORT = '/dev/ttyACM0'
PAINTER_PORT = '/dev/painter'
DEPTH_CAMERA_ID = "8086:0b07" # For Intel Realsense D435

# Lidar, IMU, Marking_Camera, Robomaster
def _check_port(port):
    if os.path.exists(port):
        return ConnectStatus.CONNECTED
    else:
        return ConnectStatus.NOTCONNECTED

# IO Module
def _check_io(port):
    modbus_config = {}
    modbus_config['method'] = 'rtu'
    modbus_config['port'] = port
    modbus_config['baudrate'] = 115200
    modbus_config['parity'] = 'N'
    modbus_config['timeout'] = 0.5
    client = ModbusSerialClient(**modbus_config)
    client.connect()

    # Try at most 5 times, in case other part (eg., Bringup Hardware) is reading.
    # Anyway, it is better to stop Bringup Hardware before checking io port.
    result = ConnectStatus.NOTCONNECTED
    for _ in range(5):
        try:
            rdi = client.read_discrete_inputs(0x0000, 12, unit=1)
            if not isinstance(rdi, ModbusIOException):
                result = ConnectStatus.CONNECTED
                break
        except SerialException:
            pass
        time.sleep(0.5)

    client.close()
    return result

# Hangfa Chassis
CRC_CALC = crcmod.mkCrcFun(0x11021, initCrc=0x0000, xorOut=0x0000, rev=False)
UNIT_ID = [0x01]
CMD_HEAD = [0xaa, 0x40] + UNIT_ID
def _get_cmd_tail(msg):
    if isinstance(msg, bytes):
        pass
    elif isinstance(msg, list):
        msg = bytes(bytearray(msg))
    elif isinstance(msg, bytearray):
        msg = bytes(msg)
    else:
        raise TypeError("Not a supported type:[list, bytearray, bytes].")

    crc_ret = CRC_CALC(msg)
    cmd_tail = crc_ret&0xff, crc_ret>>8, 0x0d
    return bytes(bytearray(cmd_tail))

def recv(port):
    readl = port.read(5)
    if readl[3] == b"\xff":
        port.flushInput()
        return -1, None
    else:
        datanum = int(readl[4].encode("hex"), 16)
        msg_all = readl
        readl = port.read(datanum + 3)
        msg_all += readl
        if _get_cmd_tail(msg_all[:-3]) == msg_all[-3:]:
            return 0, readl[:-3]
        return -1, None

def get_baud(port):
    ''' Get the current baud rate on the serial port.
    '''
    baud_lookup = {
        b"\x00": 300,
        b"\x01": 1200,
        b"\x02": 2400,
        b"\x03": 4800,
        b"\x04": 9600,
        b"\x05": 19200,
        b"\x06": 38400,
        b"\x07": 57600,
        b"\x08": 115200,
    }
    cmd_bya = bytearray(CMD_HEAD + [0x1d] + [0x01, 0x2d])
    cmd_crc_ret = CRC_CALC(bytes(cmd_bya))
    cmd_tail = cmd_crc_ret&0xff, cmd_crc_ret>>8, 0x0d
    cmd_bya = cmd_bya + bytes(bytearray(cmd_tail))
    try:
        port.flushInput()
        port.write(cmd_bya)
        ret, res = recv(port)
    except Exception as e:
        ret, res = -1, None
    if ret == 0:
        # print baud_lookup[res[-1]]
        return ret, baud_lookup[res[-1]]
    else:
        return ret, 0

def _check_hangfa(hangfa_port):
    try:
        port = Serial(port=hangfa_port, baudrate=115200, timeout=0.5, writeTimeout=None)
        # The next line is necessary to give the firmware time to wake up.
        time.sleep(1)
        ret, val = get_baud(port)
        if ret != 0:
            raise SerialException
        if val != 115200:
            time.sleep(1)
            ret, val = get_baud(port)
            if val != 115200:
                raise SerialException
        return ConnectStatus.CONNECTED

    except SerialException:
        return ConnectStatus.NOTCONNECTED

# Inclinometer
def _check_clinometer(clinometer_port):
    cli_client = ModbusSerialClient('rtu', port=clinometer_port, baudrate=9600, timeout=0.1)
    read = cli_client.read_holding_registers(0x01, 4, unit=1)
    if read.isError():
        return ConnectStatus.NOTCONNECTED
    else:
        return ConnectStatus.CONNECTED


# wrap functions such that all port settings are used from this script.
check_lidar = lambda: _check_port(LIDAR_PORT)
check_imu = lambda: _check_port(IMU_PORT)
check_sonar = lambda: _check_port(SONAR1_PORT)
check_marking_camera = lambda: _check_port(MARKING_CAMERA_PORT)
check_robomaster = lambda: _check_port(ROBOMASTER_PORT)
check_painter = lambda: _check_port(PAINTER_PORT)
check_io = lambda: _check_io(IO_PORT)
check_hangfa = lambda: _check_hangfa(HANGFA_PORT)
check_clinometer = lambda: _check_clinometer(CLINOMETER_PORT)
check_base = check_robomaster if NEW_BASE else check_hangfa

def check_depth_camera():
    """
    Check if the specified depth camera is connected via lsusb.

    NOTE: 1) It is not reliable to check /dev/video? for the existence of
    depth camera, as its ports under /dev will be disconnected once the camera
    is initialized. 2) The DEPTH_CAMERA_ID constant may subject to change if
    other camera is used.
    """
    cmd_lsusb = [
        "/usr/bin/lsusb",
        "-d",
        DEPTH_CAMERA_ID
    ]
    try:
        subprocess.check_output(args=cmd_lsusb)
        return ConnectStatus.CONNECTED
    except subprocess.CalledProcessError:
        return ConnectStatus.NOTCONNECTED

def lionel_check_diagnose():
    """ Add this wrapper fn to collect all diagnose results on Lionel (excludes CB) """
    port_name_map = [
        ('Lidar', check_lidar),
        ('Depth Camera', check_depth_camera),
        ('Marking Camera', check_marking_camera),
        ('Sonar', check_sonar),
        ('IO', check_io),
        ('MoveBase', check_base),
    ]
    if not IMU_AS_INCLINOMETER:
        port_name_map.append(('IMU', check_imu))
        port_name_map.append(('Inclinometer', check_clinometer))
    if IS_BOOTHNUMBER:
        port_name_map.append(('Painter', check_painter))

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
    results = lionel_check_diagnose()
    if HAS_AUX_PC is False:
        cb_results = cb_check_diagnose()
        results.extend(cb_results)
    print(format_print(results))
