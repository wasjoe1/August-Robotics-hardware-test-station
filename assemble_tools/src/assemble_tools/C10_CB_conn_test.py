#!/usr/bin/env python
# -*- coding: utf-8 -*-
from __future__ import print_function, division

import sys
import time
import struct
from pymodbus.client.sync import ModbusSerialClient as ModbusClient

DEVICE_TYPE = ['bed', 'vmd']
BAND = 57600
TEST_CONUT = 300
PORT = "/dev/servos"


devices = []


def print_devices(devices):
    print("All devices:")
    for dev in devices:
        print("id : {} : {}".format(dev["id"], dev["type"]))


def detect_baudrate_and_unit_id(port, devices):
    print("Start detecting devices.")
    with ModbusClient(method='rtu', port=port, baudrate=BAND, parity='N', timeout=0.1) as client:
        for i in range(1, 5):
            print("------- detecting device No. {} -------".format(i))
            # print(i)
            # check if connected successfully by validating specical data, for bed is resolution
            respond = client.read_holding_registers(0x0, 1, unit=i)
            if not respond.isError():
                print("Device No.{} detected.".format(i))
                respond = client.read_holding_registers(0x2A, 1, unit=i)
                if respond.isError():
                    print("Device No.{} is bed.".format(i))
                    device = {}
                    device["id"] = i
                    device["type"] = 'bed'
                    devices.append(device)
                else:
                    # for x in respond.registers:
                    #     print(x)
                    print("Device No.{} is not bed".format(i))
                    # return 'bed', b, i
                    respond = client.read_input_registers(0x6, 2, unit=i)
                    if not respond.isError() and struct.pack('<HH', *respond.registers) == 'VSMD':
                        print("Device No.{} is vmd.".format(i))
                        device = {}
                        device["id"] = i
                        device["type"] = 'vmd'
                        devices.append(device)
                # return 'vmd', b, i
        # client.close()
        time.sleep(0.5)
    return None, None, None


def connectivity_test(port, devices):
    print("Start testing connectivity.")
    with ModbusClient(method='rtu', port=port, baudrate=BAND, parity='N', timeout=0.1) as client:
        for dev in devices:
            print("Testing connectivity of device No.{}.".format(dev["id"]))
            success_count = 0
            for num in range(300):
                if dev["type"] == "vmd":
                    # vmd set mcs
                    result = client.write_registers(0x22, [8], unit=dev["id"])
                    if not result.isError():
                        success_count += 1
                        print('.', end='', file=sys.stderr)
                    else:
                        print('!', end='', file=sys.stderr)
                elif dev["type"] == "bed":
                    # bed set baud
                    result = client.read_holding_registers(0, 1, unit=dev["id"])
                    if not result.isError():
                        success_count += 1
                        print('.', end='', file=sys.stderr)
                    else:
                        print('!', end='', file=sys.stderr)
                    # bed if not sleep here, next loop will fail.
                    # time.sleep(0.1)
            print("")
            dev["connectivity"] = success_count/TEST_CONUT
            print("Device No.{} {} connectivity: {:.2%}.".format(
                dev["id"], dev["type"], success_count/TEST_CONUT))


def print_connectivity(devices):
    for dev in devices:
        print("Device No.{} : {} : connectivity: {:.2%}.".format(
            dev["id"],
            dev["type"],
            dev["connectivity"]))


if __name__ == "__main__":
    detect_baudrate_and_unit_id(PORT, devices)
    print("--------------------------------------")
    print("")
    if len(devices) == 0:
        print("No device found.")
        sys.exit(0)
    print_devices(devices)
    print("")
    print("--------------------------------------")
    print("")
    connectivity_test(PORT, devices)
    print("")
    print("--------------------------------------")
    print("")
    print_connectivity(devices)
