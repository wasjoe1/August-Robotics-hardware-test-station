#!/usr/bin/env python3
# -*- coding: utf-8 -*-


"""
INSTRUCTIONS: 
1.ensure pymodbus is in 1.5.2 version
2. this script can read data for multiple sonars, but can only set one sonar at a time
3. CN, EN version
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


#-------------------------------------------------------------------------------------------------------
#-------------------------------------------------------------------------------------------------------
class SonarOperations:
    """
    Interface class
    """

    def parse_reading(self,modbus_client):
        """
        def print_distance(dis,unit_box):
            return distance_image
        """
        def loop_distance(self,modbus_client):
            return dis,unit_box,paired_values_display, paired_values_csv
        
        dis_input,unit_box,paired_values_display, paired_values_csv = loop_distance(modbus_client)
        return paired_values_display    
        
    
    def scan (self,configs):
        modbus_client: ModbusClient = None
        global set_available_flag
        set_available_flag = None
        return modbus_client
    
    def set_default_settings(self,modbus_client,unit_id):
        succeeded = False
        return succeeded
    

#-------------------------------------------------------------------------------------------------------
#-------------------------------------------------------------------------------------------------------
class DYP_SONAR():
    #8 sonars
    UNIT_DICT_CHECKER = [0xe6 ,0xe8 ,0xd0,0xfa,0xfe,0xea,0xe4,0xe2,0xd2,0xec,0x01] #for ID that has already been set
    #UNIT_DICT_CHECKER_DEFAULT = 0xFF # BROADCAST ID  
    UNIT_DICT_CHECKER_DEFAULT = 0x01 #DEFAULT ID
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
    BAUDRATE_CHECKLIST = [9600,57600,115200]
    UNIT_CHECKER = []
    sonar_counter = 0

    def parse_reading(self, modbus_client): # current version of lionel has 8 sonars, previous version has 10

        
        def loop_distance(self,modbus_client):
            dis = [None]*10
            unit_box = [None]*10
            paired_values_csv = []
            paired_values_display = []
            
            for i,unit in enumerate(self.UNIT_CHECKER):
                response = modbus_client.read_holding_registers(0x0101, 1, unit = unit)
                if not response.isError():
                    distance  = response.registers[0]
                    if distance is not None:
                        dis[i] = format(distance/1000.,'.2f')
                        unit_box[i] = format(hex(unit))
                    paired_values_display.append({unit_box[i]:dis[i]})
                    paired_values_csv.append({"distance":dis[i],"unit" : unit_box[i]})
                    logger.loginfo(paired_values_display)
                else:
                    logger.loginfo("error")
                    logger.loginfo(response)
            
            logger.loginfo(paired_values_display)
            return dis,unit_box,paired_values_display,paired_values_csv

        dis_input,unit_box,paired_values_display, paired_values_csv = loop_distance(self,modbus_client)
        #dis_input,unit_box,paired_values_display, paired_values_csv = loop_distance(self=DYP_SONAR,modbus_client=modbus_client)
        #distance_image = print_distance(dis_input,unit_box)
        timestamp = datetime.datetime.now()
        header = ["timestamp", "distance", "unit"]
        file_path = "sonar_data.csv"
        with open(file_path, 'a', newline='') as csv_file:
            file_empty = os.stat(file_path).st_size == 0
            writer = csv.writer(csv_file)
            if file_empty:
                writer.writerow(header)

            for entry in paired_values_csv:
                writer.writerow([timestamp, entry["distance"], entry["unit"]])
        #return print_distance(dis_input,unit_box)
        return paired_values_display
 
    
    def scan(self, configs): 

        modbus_client : ModbusClient = None
        global set_available_flag
        set_available_flag = None

        for baudrate in self.BAUDRATE_CHECKLIST:
            # ORIGINAL
            configs.update({"baudrate":baudrate})
            logger.loginfo(baudrate)
            temp_client = ModbusClient(**configs)
            logger.loginfo(temp_client)
            
            for unit in self.UNIT_DICT_CHECKER: # Check the no. of already set sonars
                sonar_unit = temp_client.read_holding_registers(0x200, 1,unit=unit)
                if sonar_unit.isError():
                    logger.loginfo(f"Failed to connect to unit ID {unit}")
                else:
                    self.UNIT_CHECKER.append(unit)
                    modbus_client = temp_client
            set_available_flag = len(self.UNIT_CHECKER) == 1 
            logger.loginfo(self.UNIT_CHECKER)

        return modbus_client

    # PROPOSED function creation 
    # def is_checked_and_appended_sonar_unit(temp_client, checker_list, unit_to_check):
    #     sonar_unit = temp_client.read_holding_registers(0x200, 1, unit=unit_to_check)
    #     if not sonar_unit.isError():
    #         checker_list.append(unit_to_check)
    #         modbus_client = temp_client
    #         return True
    #     return False

        
    def set_default_settings(self,modbus_client,unit_id):
        succeeded = False
        for unit in self.UNIT_CHECKER:
            #set baudrate
            rwr_baudrate = modbus_client.write_register(0x201,self.DEFAULT_BAUDRATE,unit=unit) #set baudrate to 9600
            logger.loginfo("PROBLEM SETTING BAUDRATE") if rwr_baudrate.isError() else logger.loginfo("Set baudrate to 9600")


            #set unit
            unit_set = self.UNIT_DICT_SETTER[unit_id]
            rospy.sleep(0.1)
            rwr_unit = modbus_client.write_register(0x200,unit_set, unit=unit)
            #logger.loginfo("PROBLEM SETTING UNIT") if rwr_unit.isError() else logger.loginfo("unit set")

            if not rwr_unit.isError():
                logger.loginfo("unit set")
                self.UNIT_CHECKER = [unit_set] #TODO: test if this refreshes the unit set
            else:
                logger.loginfo("PROBLEM SETTING UNIT")
                logger.loginfo(rwr_unit)


        succeeded = True 
        return succeeded
    


#-------------------------------------------------------------------------------------------------------
#-------------------------------------------------------------------------------------------------------
class SonarChecker:
    NODE_RATE = 5.0
    sonar_model:SonarOperations = DYP_SONAR()
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
        #self.modbus_client = self.sonar_model.scan(self=self.sonar_model,configs=self.modbus_configs)
        self.modbus_client = self.sonar_model.scan(self.modbus_configs)
        logger.loginfo(self.modbus_client)
        if self.modbus_client:
            self.log_with_frontend(f'Scanned sonar on baudrate: {self.modbus_configs["baudrate"]}',f'波特率: {self.modbus_configs["baudrate"]}')
            self.state = SonarCheckerStates.CONNECTED
            self.pub_configs.publish(json.dumps(self._get_current_SONAR_settings()))
            self.pub_configs_chinese.publish(json.dumps(self._get_current_SONAR_settings_chinese()))
            return True
        self.log_with_frontend(f"Failed to connect sonar driver!, Check connection",f"无法连上 声纳")
        self.state = SonarCheckerStates.IDLE
        return False

    def parse_reading(self):
        if self.parse_thread and self.parse_thread.is_alive():
            return False
        def parse_target():
            try:
                #sonar_msg = self.sonar_model.parse_reading(self=self.sonar_model,modbus_client = self.modbus_client)
                sonar_msg = self.sonar_model.parse_reading(self.modbus_client)
                rospy.sleep(0.5)
                # rospy.sleep(0.001) # same as sonar reading code on lionel
                self.pub_reading.publish(json.dumps(sonar_msg))
            except serial.SerialException:
                self.log_with_frontend("Sonar unplugged! Check connection","无法连接IMU，请确保电源再连接")
                self.sonar_model.UNIT_CHECKER = []
                self.sonar_model.sonar_counter = 0
                self.state = SonarCheckerStates.IDLE
    

        self.parse_thread = threading.Thread(target=parse_target)
        self.parse_thread.start()
    
    def set_default_settings(self):
        if self.parse_thread and self.parse_thread.is_alive():
            self.parse_thread.join()
        if set_available_flag == True:
            #if self.sonar_model.set_default_settings(self= self.sonar_model,modbus_client=self.modbus_client, unit_id=self.cmd_params):
            if self.sonar_model.set_default_settings(self.modbus_client, self.cmd_params):
                #self.sonar_model.UNIT_CHECKER = [self.sonar_model.UNIT_DICT_SETTER[self.cmd_params]] 
                self.log_with_frontend("SETTING SET","设置成功")
                self.log_with_frontend("CFG SAVED","设置保存成功")
                return True
        else: 
            self.log_with_frontend("Ensure there is only one sonar connected", "系统只能同时设置一个sonar")
            
    
#-------------------------------------------------------------------------------------------------------
if __name__ == "__main__":
    rospy.init_node("sonar_driver_node")
    sonar_driver_checker = SonarChecker()
    sonar_driver_checker.start()