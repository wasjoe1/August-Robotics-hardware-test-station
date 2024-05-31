#!/usr/bin/env python3
# -*- coding: utf-8 -*-


"""
INSTRUCTIONS: 
1.ensure pymodbus is in 1.5.2 version
2. this script can test for multiple sonars, but can only set one sonar at a time
"""

"""
Code logic: 
flag_check_set_method_available == None
1. scan for multiple sonars --> if more than 1 --> can only read and therefore not set --> flag ==False for multiple sonars
 if flag == false, pass set settings 
2. if have one --> flag == True. can set settings (take current unit_id) --> set unit_id

"""
import os
import datetime
import csv
import threading
import serial
from pymodbus.client.sync import ModbusSerialClient as ModbusClient
import rospy 
logger = rospy
from enum import Enum, auto
import json 
from meterial_inspection_tools.ros_interface import (
    SONAR_SRV_CMD,
    SONAR_DATA,
    SONAR_CONFIGS,
    SONAR_CONFIGS_CHINESE,
    SONAR_INFO,
    SONAR_INFO_CHINESE,
    SONAR_STATE,
)

import pymodbus
print(pymodbus.__version__)

class SONARCommands(Enum):
    NONE = auto()
    RESET = auto()
    CONNECT = auto()
    SET_DEFAULT = auto()

class SonarCheckerStates(Enum): 
    INIT = auto()
    IDLE = auto()
    SCANNING = auto()
    CONNECTED = auto()
    ERROR = auto()

class DYP_SONAR():
    #8 sonars
    UNIT_DICT_CHECKER = [0xe6 ,0xe8 ,0xd0,0xfa,0xfe,0xea,0xe4,0xe2,0xd2,0xec] #for ID that has already been set
    UNIT_DICT_CHECKER_DEFAULT = 0xFF # for default ID  
    DEFAULT_BAUDRATE = 3 #9600
    UNIT_DICT_SETTER = {
            "0": 0xe6,
            "1": 0xe8,
            "2": 0xd0,
            "3": 0xfa,
            "4": 0xfe,
            "5": 0xea,
            "6": 0xe4,
            "7": 0xe2,
            "8": 0xd2,
            "9": 0xec,
        }
    BAUDRATE_CHECKLIST = [115200,57600, 9600]
    UNIT_CHECKER = []
    sonar_counter = 0

    def parse_reading(self, modbus_client):
        
        #TODO: check how many sonars are there on lionel
        def print_distance(dis,unit_box): 
            distance_image = """
        {} : {}
        {} : {}
        {} : {}
        {} : {}
        {} : {}
        {} : {}
        {} : {}
        {} : {}
        """.format(
            format(unit_box[0]), format(dis[0]),
            format(unit_box[1]), format(dis[1]),
            format(unit_box[2]), format(dis[2]),
            format(unit_box[3]), format(dis[3]),
            format(unit_box[4]), format(dis[4]),
            format(unit_box[5]), format(dis[5]),
            format(unit_box[6]), format(dis[6]),
            format(unit_box[7]), format(dis[7]),
        )
            
        '''
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
                |_{}__|                             |{}|
                |     |                             |     |
                |     |                             |     |
                |_____|_____ _____ _____ ___________|_____|
                |     |  7  |     |  6  |     |  5  |     |
                |     | {} |     |{} |     |{} |     |
                |_____|_{}___|____|{}___|___|_{}__|_____|

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
        '''
        
        def loop_distance(self,modbus_client):
            dis = [None]*10
            unit_box = [None]*10
            paired_values = []
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
                    paired_values.append({"distance":dis[i],"unit" : unit_box[i]})
                if response.isError():
                    logger.loginfo("error")
                    logger.loginfo(response)
            return dis,unit_box,paired_values

        dis_input,unit_box,paired_values = loop_distance(self=DYP_SONAR,modbus_client=modbus_client)
        #distance_image = print_distance(dis_input,unit_box)
        timestamp = datetime.datetime.now()
        header = ["timestamp", "distance", "unit"]
        file_path = "sonar_data.csv"
        with open(file_path, 'a', newline='') as csv_file:
            file_empty = os.stat(file_path).st_size == 0
            writer = csv.writer(csv_file)
            if file_empty:
                writer.writerow(header)

            for entry in paired_values:
                writer.writerow([timestamp, entry["distance"], entry["unit"]])
        return print_distance(dis_input,unit_box)
    

    def scan(self, configs):
        modbus_client : ModbusClient = None
        
        #checks for number of sonars
        sonar_connected_count = 0
        global set_available_flag
        set_available_flag = None

        for baudrate in self.BAUDRATE_CHECKLIST:
            configs.update({"baudrate":baudrate})
            logger.loginfo(baudrate)
            temp_client = ModbusClient(**configs)
            logger.loginfo(temp_client)
            for unit in self.UNIT_DICT_CHECKER:
                respond = temp_client.read_holding_registers(0x200, 1,unit= unit)
                if not respond.isError():
                    modbus_client = temp_client
                    sonar_connected_count += 1
                    self.sonar_counter = sonar_connected_count
                    self.UNIT_CHECKER.append(unit)
                    logger.loginfo("Sonar connected " + str(sonar_connected_count))
                if respond.isError():
                    logger.loginfo(("SONAR WITH CORRESPONDING UNIT ID: ") + str(unit) + (" NOT FOUND, CHECK MODBUS CONFIGS"))
            if sonar_connected_count == 1: 
                set_availble_flag = True
            elif sonar_connected_count >1:
                set_available_flag = False
                for default_unit in self.UNIT_DICT_CHECKER_DEFAULT:
                    respond_default = temp_client.read_holding_registers(0x200, 1,unit= default_unit)
                    if not respond_default.isError(): 
                        modbus_client = temp_client
                        sonar_connected_count += 1
                        self.UNIT_CHECKER.append(default_unit)
                        logger.loginfo("Sonar connected on default unit 0xFF " + str(sonar_connected_count))
        return modbus_client

    def set_default_settings(self,modbus_client):
        succeeded = False
        address_counter = 0
        for unit in self.UNIT_DICT_CHECKER:
            rwr_baudrate = modbus_client.write_register(0x201,self.DEFAULT_BAUDRATE,unit=unit) #set baudrate to 9600
            if not rwr_baudrate.isError():
                 logger.loginfo("Set baudrate to 9600, unit ID has been set previously")
            if rwr_baudrate.isError():
                logger.loginfo("PROBLEM SETTING BAUDRATE, check if sonar is connected")
        if self.sonar_counter >1:
            for unit_default in self.UNIT_DICT_CHECKER_DEFAULT:
                rwr_baudrate = modbus_client.write_register(0x201,self.DEFAULT_BAUDRATE,unit=unit_default) #set baudrate to 9600
                if not rwr_baudrate.isError():
                    logger.loginfo("Set baudrate to 9600")
                    UNIT = self.UNIT_DICT_SETTER[str(address_counter)]
                    rwr_unit = modbus_client.write_register(0x200,UNIT, unit=unit_default) #set unit
                    if not rwr_unit.isEconfigsr():
                        pass
        succeeded = True
        return succeeded
        


class SonarChecker:
    NODE_RATE = 5.0
    sonar_model = DYP_SONAR
    _command = None
    _state = None
    modbus_client : ModbusClient = None
    unit_id = None
    

    modbus_configs = {
        "method": "rtu",
        "port": "/dev/sonar",
        "baudrate": 9600,
        "parity": "N",
        "timeout": 0.5,
    }


    def __init__(self) -> None:
        self.command = "NONE"
        self.cmd_params = ""
        self.state = SonarCheckerStates.INIT
        self.pub_state = SONAR_STATE.Publisher()
        self.pub_info = SONAR_INFO.Publisher()
        self.pub_info_chinese = SONAR_INFO_CHINESE.Publisher()
        self.pub_reading = SONAR_DATA.Publisher()
        self.pub_configs = SONAR_CONFIGS.Publisher()
        self.pub_configs_chinese = SONAR_CONFIGS_CHINESE.Publisher()
        SONAR_SRV_CMD.Services(self.srv_cb)
        self.parse_thread = None


        self.__STATES_METHODS = {
        (SONARCommands.NONE, SonarCheckerStates.INIT): self.initialize, # to IDLE
        (SONARCommands.CONNECT, SonarCheckerStates.IDLE): self.scan, # to CONNECTED or stay
        (SONARCommands.NONE, SonarCheckerStates.CONNECTED): self.parse_reading, # stay
        (SONARCommands.SET_DEFAULT, SonarCheckerStates.CONNECTED): self.set_default_settings, # stay

    }

    def srv_cb(self, srv):
        self.command = srv.button
        self.cmd_params = srv.ID
        return True

    @property
    def command(self):
        return self._command
    @command.setter
    def command(self, value: str):
        try:
            self._command = SONARCommands[value.upper()]
            logger.loginfo(f"Command set as: {self._command}")
        except Exception as e:
            self.log_with_frontend(f"Received wrong command: {value}!!",f"命令错误: {value}")
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
            
    def log_with_frontend(self, log, log_chinese):
        logger.loginfo(log)
        logger.loginfo(log_chinese)
        self.pub_info.publish(log)
        self.pub_info_chinese.publish(log_chinese)

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
            "ID" : self.unit_id,
            }
    
    def _get_current_SONAR_settings_chinese(self):
        return {
            "波特率": self.modbus_configs["baudrate"],
            "ID": self.unit_id,
            }

    def scan(self):
        self.state = SonarCheckerStates.SCANNING
        self.modbus_client = self.sonar_model.scan(self=self.sonar_model,configs=self.modbus_configs)
        if self.modbus_client:
            self.log_with_frontend(f'Scanned CB driver on baudrate: {self.modbus_configs["baudrate"]}',f'波特率: {self.modbus_configs["baudrate"]}')
            self.state = SonarCheckerStates.CONNECTED
            self.pub_configs.publish(json.dumps(self._get_current_SONAR_settings()))
            self.pub_configs_chinese.publish(json.dumps(self._get_current_SONAR_settings_chinese()))
            return True
        self.log_with_frontend(f"Failed to connect sonar driver!",f"无法连上 sonar ")
        self.state = SonarCheckerStates.IDLE
        return False

    def parse_reading(self):
        if self.parse_thread and self.parse_thread.is_alive():
            return False
        def parse_target():
            try:
                with serial.Serial(port=self.modbus_configs["port"]) as ser:   
                    sonar_msg = self.sonar_model.parse_reading(self=self.sonar_model,modbus_client = self.modbus_client)
                    rospy.sleep(0.001) # same as sonar reading code on lionel
                    self.pub_reading.publish(json.dumps(sonar_msg))
            except serial.SerialException:
                #TODO: unsure if thread should be joined 
                if self.parse_thread and self.parse_thread.is_alive():
                    self.parse_thread.join()
                #will work
                self.log_with_frontend("Sonar unplugged! Check connection","无法连接IMU，请确保电源再连接")
                self.sonar_model.UNIT_CHECKER = []
                self.state = SonarCheckerStates.IDLE
        self.parse_thread = threading.Thread(target=parse_target)
        self.parse_thread.start()
    
    def set_default_settings(self):
        if self.parse_thread and self.parse_thread.is_alive():
            self.parse_thread.join()
        if self.sonar_model.set_default_settings(self= self.sonar_model,modbus_client=self.modbus_client):
            self.log_with_frontend("SETTING SET","设置成功")
            self.log_with_frontend("CFG SAVED","设置保存成功")
            return True
    

if __name__ == "__main__":
    rospy.init_node("sonar_driver_node")
    sonar_driver_checker = SonarChecker()
    sonar_driver_checker.start()