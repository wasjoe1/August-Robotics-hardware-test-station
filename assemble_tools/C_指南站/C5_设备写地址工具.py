#!/usr/bin/env python
# -*- coding: utf-8 -*-
from __future__ import print_function, division

import sys
import time
import struct
import argparse
import serial.tools.list_ports
from pymodbus.client.sync import ModbusSerialClient as ModbusClient

DEVICE_TYPE = ['bed', 'vmd']
BAUDRATES = [9600, 57600, 115200]
ENCODER_BAUDRATES_TABLE = {
    9600: 0x00,
    19200: 0x01,
    38400: 0x02,
    57600: 0x03,
    115200: 0x04
}
POSSIBLE_IDS = [4]
FF16 = (1<<16)-1
VMD_DEFAULT_DYN_1 = 1.92e5, 9.6e5, 9.6e5, 2.0, 2.0, 0.5
VMD_DEFAULT_DYN_3 = 1.92e5, 9.6e5, 9.6e5, 1.3, 1.3, 0.8
DEFAULT_BAUDRATE = 57600
DEFAULT_SETTINGS = {
    'vmd_1': (
        (
            u'''\
设置设备速度: {0[0]:.2e}
    加减速度: {0[1]:.2e}, {0[2]:.2e}
    启动电流: {0[3]} A
    匀速电流: {0[4]} A
    保持电流: {0[5]} A\
            '''.format(VMD_DEFAULT_DYN_1),
            3,
            struct.unpack(
                '<'+'H'*12,
                struct.pack('<'+'f'*6, *VMD_DEFAULT_DYN_1)
            )
        ),
        (
            u'''\
设置设备ID为: {}
    波特率为: {}
  步进细分为: {}\
            '''.format(1, DEFAULT_BAUDRATE, 256),
            31,
            (1, DEFAULT_BAUDRATE & FF16, DEFAULT_BAUDRATE >> 16, 8)
        ),
        (
            u'保存设置',
            0,
            (0x0500,)
        ),
    ),
    'bed_2': (
        (
            u'''\
设置设备ID为: {}
    波特率为: {}\
            '''.format(2, DEFAULT_BAUDRATE),
            4,
            (2, ENCODER_BAUDRATES_TABLE[DEFAULT_BAUDRATE])
        ),
    ),
    'vmd_3': (
        (
            u'''\
设置设备速度: {0[0]:.2e}
    加减速度: {0[1]:.2e}, {0[2]:.2e}
    启动电流: {0[3]} A
    匀速电流: {0[4]} A
    保持电流: {0[5]} A\
            '''.format(VMD_DEFAULT_DYN_3),
            3,
            struct.unpack(
                '<'+'H'*12,
                struct.pack('<'+'f'*6, *VMD_DEFAULT_DYN_3)
            )
        ),
        (
            u'''\
设置设备ID为: {}
    波特率为: {}
  步进细分为: {}\
            '''.format(3, DEFAULT_BAUDRATE, 256),
            31,
            (3, DEFAULT_BAUDRATE & FF16, DEFAULT_BAUDRATE >> 16, 8)
        ),
        (
            u'保存设置',
            0,
            (0x0500,)
        ),
    ),
    'bed_4':(
        (
            u'''\
设置设备ID为: {}
    波特率为: {}\
            '''.format(4, DEFAULT_BAUDRATE),
            4,
            (4, ENCODER_BAUDRATES_TABLE[DEFAULT_BAUDRATE])
        ),
    ),
}

def detect_baudrate_and_unit_id(port):
    for b in BAUDRATES:
        with ModbusClient(method='rtu', port=port, baudrate=b, parity='N', timeout=0.1) as client:
            for i in POSSIBLE_IDS:
                # check if connected successfully by validating specical data, for bed is resolution
                respond = client.read_holding_registers(0x0, 1, unit=i)
                if not respond.isError():
                    respond = client.read_holding_registers(0x6, 1, unit=i)
                    if respond.isError():
                        return 'bed', b, i
                respond = client.read_input_registers(0x6, 2, unit=i)
                if not respond.isError() and struct.pack('<HH', *respond.registers) == 'VSMD':
                    return 'vmd', b, i
        # client.close()
        time.sleep(0.5)
    return None, None, None

def write_default_setting(given_type, port, dev_type, baudrate, unit_id):
    wl = DEFAULT_SETTINGS[given_type]
    for description, address, data_list in wl:
        with ModbusClient(method='rtu', port=port, baudrate=baudrate, parity='N', timeout=0.1) as client:
            print(description)
            if given_type in ['vmd_1', 'vmd_3']:
                result = client.write_registers(address, data_list, unit=unit_id)
                if not result.isError():
                    print(u'操作成功！')
                time.sleep(0.5)

            # Thanks for the new protocol!!! -_-
            if given_type in ['bed_2', 'bed_4']:
                for seq, data in enumerate(data_list):
                    result = client.write_register(address+seq, data, unit=unit_id)
                    time.sleep(0.5)
                client.close()
                detected_again = detect_baudrate_and_unit_id(port)
                print(detected_again)
                if detected_again[0] == 'bed' and detected_again[1] == DEFAULT_BAUDRATE:
                    print(u'操作成功！')
                time.sleep(0.5)

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('given_type', choices=DEFAULT_SETTINGS.keys(),
                        help='Defined device_types')
    parser.add_argument("-p", "--port",
                        help='Specified port for setting.')
    args = parser.parse_args()

    dev_type, baudrate, dev_id = detect_baudrate_and_unit_id(args.port)
    if dev_type is None:
        print(u'未检测到设备')
        sys.exit(1)
    print(u'检测到设备：{}'.format((dev_type, baudrate, dev_id)))
    print(u'请确认是否写入设备默认设置: {} ([y]/n)'.format(args.given_type))
    user_in = raw_input()
    if user_in in ('', 'y', 'Y'):
        if not DEFAULT_SETTINGS.has_key(args.given_type) or args.given_type[:3] != dev_type:
            sys.exit(1)
        else:
            write_default_setting(args.given_type, args.port, dev_type, baudrate, dev_id)
    else:
        sys.exit(0)
