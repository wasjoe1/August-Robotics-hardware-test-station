#!/usr/bin/env python3
# -*- coding: utf-8 -*-
from boothbot_driver.modbus_driver import ModbusDriver
from pymodbus.client.sync import ModbusSerialClient as ModbusClient
import os
import time
BAUDRATES = [9600, 115200]

PORT = ["/dev/ttyUSB0", "/dev/ttyUSB1", "/dev/ttyUSB2"]
UNIT_DICT = [0xe6 ,0xe8 ,0xd0,0xfa,0xfe,0xea,0xe4,0xe2,0xd2,0xec ]

def detect_baudrate_and_unit_id(port):
    for baud in BAUDRATES:
        with ModbusClient(method='rtu', port=port, baudrate=baud, parity='N', timeout=0.1) as client:
            response=client.read_holding_registers(0x200, 1, unit=0xFF)
            if not response.isError():
                print(baud,port)
                return baud, port
        time.sleep(0.5)

class DYP_SONAR_UNIT_WRITTER(object):
    def __init__(self, modbus_driver):
        super(DYP_SONAR_UNIT_WRITTER, self).__init__()
        self.modbus_driver = modbus_driver
        self.dis = [None]*10

    def print_distance(self):
        self.distance_image = """



            ___________________________________________
            |     |  0  |     |  1  |     |  2  |     |
            |     |{} |     |{} |     |{} |     |
            |_____|_____|_____|_____|_____|_____|_____|
            |     |                             |     |
            |     |                             |     |
            |_____|                             |_____|
            |  9  |                             |  3  |
            |{} |                             |{} |
            |_____|                             |_____|
            |     |                             |     |
            |     |                             |     |
            |_____|                             |_____|
            |  8  |                             |  4  |
            |{} |                             |{} |
            |_____|                             |_____|
            |     |                             |     |
            |     |                             |     |
            |_____|_____ _____ _____ _____ _____|_____|
            |     |  7  |     |  6  |     |  5  |     |
            |     |{} |     |{} |     |{} |     |
            |_____|_____|_____|_____|_____|_____|_____|

            Crtl + c 退出

""".format(
    format(self.dis[0]),
    format(self.dis[1]),
    format(self.dis[2]),
    format(self.dis[9]),
    format(self.dis[3]),
    format(self.dis[8]),
    format(self.dis[4]),
    format(self.dis[7]),
    format(self.dis[6]),
    format(self.dis[5])
    )
        return self.distance_image

    def distance_pub(self,unit):
        response=self.modbus_driver.read_holding_registers(0x0101, 1, unit=unit)
        try:
            distance = response.registers[0]
        except:
            distance = None
        return distance

    def loop_distance(self):
        for i, unit in enumerate(UNIT_DICT):
            distance = self.distance_pub(unit)
            if distance is not None:
                self.dis[i] = format(distance/1000.,'.2f')
            else:
                self.dis[i] = None
            os.system('clear')
            print(self.print_distance())

if __name__ == '__main__':
    for port in PORT:
        try:
            baud, _port = detect_baudrate_and_unit_id(port)
        except:
            print()
    print(baud,_port)
    modbus_config = {
    "port": _port,
    "baudrate": baud,
    "parity": "N",
    "timeout": 0.2,
}
    mbd = ModbusDriver(**modbus_config).client
    DYP = DYP_SONAR_UNIT_WRITTER(mbd)
    while True:
        DYP.loop_distance()
        DYP.print_distance()





