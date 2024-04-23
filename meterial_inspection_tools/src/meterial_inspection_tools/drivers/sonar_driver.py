#!/usr/bin/env python3
# -*- coding: utf-8 -*-


"""
INSTRUCTIONS: 
1.ensure pymodbus is in 1.5.2 version
2. this script can test and set parameters for multiple sonars together
3. sonars ID do not have to be set beforehand
3. SAVE and CLOSE buttons are not required, but for user experience
"""

import threading
from pymodbus.client.sync import ModbusSerialClient as ModbusClient
import rospy 
logger = rospy
from enum import Enum, auto
import json 
from meterial_inspection_tools.ros_interface import (
    SONAR_SRV_CMD,
    SONAR_DATA,
    SONAR_CONFIGS,
    SONAR_INFO,
    SONAR_STATE,
)

import pymodbus
print(pymodbus.__version__)

class SONARCommands(Enum):
    NONE = auto()
    RESET = auto()
    SCAN = auto()
    CONNECT = auto()
    DISCONNECT = auto()
    SET_DEFAULT = auto()
    SAVE = auto()

class SonarCheckerStates(Enum): 
    INIT = auto()
    IDLE = auto()
    SCANNING = auto()
    CONNECTING = auto()
    CONNECTED = auto()
    ERROR = auto()

class DYP_SONAR():
    UNIT_DICT_CHECKER = [0xe6 ,0xe8 ,0xd0,0xfa,0xfe,0xea,0xe4,0xe2,0xd2,0xec] #for ID that has already been set
    UNIT_DICT_CHECKER_DEFAULT = [0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF] # for default ID 
    DEFAULT_BAUDRATE = 3 #9600
    BAUDRATE_CHECKLIST = [115200,57600, 9600]
    UNIT_CHECKER = []


    def parse_reading(self, modbus_client):
    
        def print_distance(dis,unit_box):
            distance_image = """


                ___________________________________________
                |     |  0  |     |  1  |     |  2  |     |
                |     |{}   |   |{} |     |{} |     |
                |_____|{}___|___|_{}__|___|_{}__|___|
                |     |                             |     |
                |     |                             |     |
                |_____|                             |_____|
                |  9  |                             |  3  |
                |{} |                             |{}|
                |_{}_|                             |_{}|
                |     |                             |     |
                |     |                             |     |
                |_____|                             |_____|
                |  8  |                             |  4  |
                |{} |                             |{} |
                |_{}__|                             |_{}|
                |     |                             |     |
                |     |                             |     |
                |_____|_____ _____ _____ _____ _____|_____|
                |     |  7  |     |  6  |     |  5  |     |
                |     |{} |      |{} |     |{} |     |
                |_____|{}__|_____|_{}_|____|_{}_|___|

                

    """.format(
        format(dis[0]),format(dis[1]),format(dis[2]), 
        format(unit_box[0]),format(unit_box[1]),format(unit_box[2]),
        format(dis[9]),format(dis[3]),
        format(unit_box[9]),format(unit_box[3]),
        format(dis[8]),format(dis[4]),
        format(unit_box[8]),format(unit_box[4]),
        format(dis[7]),format(dis[6]),format(dis[5]),
        format(unit_box[7]),format(unit_box[6]),format(unit_box[5])
        )
        
            logger.loginfo(distance_image)
            return distance_image

        
        def loop_distance(self,modbus_client):
            dis = [None]*10
            unit_box = [None]*10
            for i,unit in enumerate(self.UNIT_CHECKER):
                response = modbus_client.read_holding_registers(0x0101, 1, unit = unit)
                if not response.isError():
                    distance  = response.registers[0]
                    if distance is not None:
                        dis[i] = format(distance/1000.,'.2f')
                        unit_box[i] = format(hex(unit))
                    else: 
                        dis[i] = None
                        unit_box[i] = None
                if response.isError():
                    pass
            return dis,unit_box

        dis_input,unit_box = loop_distance(self=DYP_SONAR,modbus_client=modbus_client)
        return print_distance(dis_input,unit_box)
    

    def connect(self,configs):
        modbus_client : ModbusClient = None
        temp_client = ModbusClient(**configs)
        logger.loginfo(temp_client)
        sonar_connected_count = 0
        for default_unit in self.UNIT_DICT_CHECKER_DEFAULT:
            respond_default = temp_client.read_holding_registers(0x200, 1,unit= default_unit)
            if not respond_default.isError(): 
                modbus_client = temp_client
                sonar_connected_count += 1
                self.UNIT_CHECKER.append(default_unit)
                logger.loginfo("Sonar connected on default unit 0xFF " + str(sonar_connected_count))
        for unit in self.UNIT_DICT_CHECKER:
            respond = temp_client.read_holding_registers(0x200, 1,unit= unit)
            if not respond.isError():
                modbus_client = temp_client
                sonar_connected_count += 1
                self.UNIT_CHECKER.append(unit)
                logger.loginfo("Sonar connected " + str(sonar_connected_count))
            if respond.isError():
                logger.loginfo(("SONAR WITH CORRESPONDING UNIT ID: ") + str(unit) + (" NOT FOUND, CHECK IF SONAR IS CONNECTED"))
        return modbus_client


    def scan(self, configs):
        modbus_client : ModbusClient = None
        sonar_connected_count = 0
        for baudrate in self.BAUDRATE_CHECKLIST:
            configs.update({"baudrate":baudrate})
            logger.loginfo(baudrate)
            temp_client = ModbusClient(**configs)
            logger.loginfo(temp_client)
            for default_unit in self.UNIT_DICT_CHECKER_DEFAULT:
                respond_default = temp_client.read_holding_registers(0x200, 1,unit= default_unit)
                if not respond_default.isError(): 
                    modbus_client = temp_client
                    sonar_connected_count += 1
                    self.UNIT_CHECKER.append(default_unit)
                    logger.loginfo("Sonar connected on default unit 0xFF " + str(sonar_connected_count))
            for unit in self.UNIT_DICT_CHECKER:
                respond = temp_client.read_holding_registers(0x200, 1,unit= unit)
                if not respond.isError():
                    modbus_client = temp_client
                    sonar_connected_count += 1
                    self.UNIT_CHECKER.append(unit)
                    logger.loginfo("Sonar connected " + str(sonar_connected_count))
                if respond.isError():
                    logger.loginfo(("SONAR WITH CORRESPONDING UNIT ID: ") + str(unit) + (" NOT FOUND, CHECK MODBUS CONFIGS"))
        return modbus_client

    def set_default_settings(self,modbus_client):
        succeeded = False
        for unit_default in self.UNIT_DICT_CHECKER_DEFAULT:
            rwr_baudrate = modbus_client.write_register(0x201,self.DEFAULT_BAUDRATE,unit=unit_default) #set baudrate to 9600
            if not rwr_baudrate.isError():
                logger.loginfo("Set baudrate to 9600")
                for num in range(0,10):
                    num_str = str(num)
                    UNIT = self.UNIT_DICT_SETTER[num_str]
                    rwr_unit = modbus_client.write_register(0x200,UNIT, unit=unit_default) #set unit
                    if not rwr_unit.isError(): 
                        logger.loginfo("Set unit ID to " + str(unit))
                    if rwr_unit.isError():
                        pass
        for unit in self.UNIT_DICT_CHECKER:
            rwr_baudrate = modbus_client.write_register(0x201,self.DEFAULT_BAUDRATE,unit=unit) #set baudrate to 9600
            if not rwr_baudrate.isError():
                 logger.loginfo("Set baudrate to 9600, unit ID has been set previously")
            if rwr_baudrate.isError():
                pass
        logger.loginfo("HERE")
        succeeded = True
        return succeeded
        
    @staticmethod
    def save_parameters(modbus_client):
        return True

    @staticmethod
    def close(modbus_client):
        return True

class SonarChecker:
    NODE_RATE = 5.0
    sonar_model = DYP_SONAR
    _command = None
    _state = None
    modbus_client : ModbusClient = None
    

    modbus_configs = {
        "method": "rtu",
        "port": "/dev/sonar",
        "baudrate": 9600,
        "parity": "N",
        "timeout": 0.5,
    }


    def __init__(self) -> None:
        self.command = "NONE"
        self.state = SonarCheckerStates.INIT
        self.pub_state = SONAR_STATE.Publisher()
        self.pub_info = SONAR_INFO.Publisher()
        self.pub_reading = SONAR_DATA.Publisher()
        self.pub_configs = SONAR_CONFIGS.Publisher()
        SONAR_SRV_CMD.Services(self.srv_cb)
        self.parse_thread = None


        self.__STATES_METHODS = {
        (SONARCommands.NONE, SonarCheckerStates.INIT): self.initialize, # to IDLE
        (SONARCommands.SCAN, SonarCheckerStates.IDLE): self.scan, # to CONNECTED or stay
        (SONARCommands.CONNECT, SonarCheckerStates.IDLE): self.connect, # to CONNECTED or stay
        (SONARCommands.DISCONNECT, SonarCheckerStates.CONNECTED): self.disconnect, # to IDLE
        (SONARCommands.NONE, SonarCheckerStates.CONNECTED): self.parse_reading, # stay
        (SONARCommands.SET_DEFAULT, SonarCheckerStates.CONNECTED): self.set_default_settings, # stay
        (SONARCommands.SAVE, SonarCheckerStates.CONNECTED): self.save_parameters, # stay
    }

    def srv_cb(self, srv):
        self.command = srv.button

    @property
    def command(self):
        return self._command
    @command.setter
    def command(self, value: str):
        try:
            self._command = SONARCommands[value.upper()]
            logger.loginfo(f"Command set as: {self._command}")
        except Exception as e:
            self.log_with_frontend(f"Received wrong command: {value}!!")
            self._command = SONARCommands.NONE

    @property
    def state(self):
        return self._state
    @state.setter
    def state(self, value: SonarCheckerStates):
        if self._state != value:
            self._state = value
            logger.logwarn(f"State changed to: {self._state.name}")
            self.pub_state = SONAR_STATE.Publisher()
            self.pub_state.publish(self.state.name)
            

    def log_with_frontend(self, log):
        self.pub_info.publish(log)

    def start(self):
        l = rospy.Rate(self.NODE_RATE)
        while not rospy.is_shutdown():
            state_method = self.__STATES_METHODS.get((self.command,self.state))
            if state_method:
                state_method()
            self.command = "NONE"
            l.sleep()

    def initialize(self):
        self.state = SonarCheckerStates.IDLE
    
    def _get_current_SONAR_settings(self):
        return {
            "baudrate": self.modbus_configs["baudrate"],
            }

    def connect(self):
        self.state = SonarCheckerStates.CONNECTING
        self. modbus_client = self.sonar_model.connect(self=self.sonar_model,configs=self.modbus_configs)
        if self.modbus_client:
            self.log_with_frontend("Connected on baudrate: 9600")
            self.state = SonarCheckerStates.CONNECTED
            self.pub_configs.publish(json.dumps(self._get_current_SONAR_settings()))
            return True
        self.log_with_frontend(f"Failed to connect CB driver!")
        self.state = SonarCheckerStates.IDLE
        return False
        
    def scan(self):
        self.state = SonarCheckerStates.SCANNING
        self.modbus_client = self.sonar_model.scan(self=self.sonar_model,configs=self.modbus_configs)
        if self.modbus_client:
            self.log_with_frontend(f'Scanned CB driver on baudrate: {self.modbus_configs["baudrate"]}')
            self.state = SonarCheckerStates.CONNECTED
            self.pub_configs.publish(json.dumps(self._get_current_SONAR_settings()))
            return True
        self.state = SonarCheckerStates.IDLE
        return False

    def disconnect(self):
        if self.parse_thread and self.parse_thread.is_alive():
            self.parse_thread.join()
        self.modbus_client  = None
        self.log_with_frontend("DISCONNECTING")
        self.state = SonarCheckerStates.IDLE
        return True

    def parse_reading(self):
        if self.parse_thread and self.parse_thread.is_alive():
            return False
        def parse_target():
            cb_msg = self.sonar_model.parse_reading(self=self.sonar_model,modbus_client = self.modbus_client)
            self.pub_reading.publish(json.dumps(cb_msg))
        self.parse_thread = threading.Thread(target=parse_target)
        self.parse_thread.start()
    
    def set_default_settings(self):
        if self.parse_thread and self.parse_thread.is_alive():
            self.parse_thread.join()
        if self.sonar_model.set_default_settings(self= self.sonar_model,modbus_client=self.modbus_client):
            self.log_with_frontend("Set default settings")
            return True
    
    def save_parameters(self): 
        if self.parse_thread and self.parse_thread.is_alive():
            self.parse_thread.join()
        self.log_with_frontend("CFG saved")
        return True
        
if __name__ == "__main__":
    rospy.init_node("sonar_driver_node")
    sonar_driver_checker = SonarChecker()
    sonar_driver_checker.start()