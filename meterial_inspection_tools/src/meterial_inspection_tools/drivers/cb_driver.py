#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
INSTRUCTIONS: 
1.ensure pymodbus is in 1.5.2 version
2. this script can test and set parameters for multiple VSMD (驱动机) or BRITER （编码器） together
3. key in VSMD or BRITER in parameters to specify which component is being tested 
4. SAVE and CLOSE buttons are not required, but for user experience
"""

from __future__ import print_function, division
import sys
import time
import struct
import serial
from pymodbus.client.sync import ModbusSerialClient as ModbusClient

import rospy 
logger = rospy
from enum import Enum, auto
import json 
import math
from meterial_inspection_tools.ros_interface import (
    CB_DATA,
    CB_INFO,
    CB_INFO_CHINESE,
    CB_STATE,
    CB_CONFIGS,
    CB_CONFIGS_CHINESE,
    CB_SRV_CMD,
    CB_DATA_CHECK,
)

import pymodbus
print(pymodbus.__version__)

class CBCommands(Enum):
    NONE = auto()
    RESET = auto()
    #SCAN = auto()
    #CONNECT = auto()
    #DISCONNECT = auto()
    AUTO_DETECT = auto()
    SET_DEFAULT = auto()
    #SAVE = auto()

class CBCheckerStates(Enum): 
    INIT = auto()
    IDLE = auto()
    SCANNING = auto()
    #CONNECTING = auto()
    CONNECTED = auto()
    ERROR = auto()


class CBOperations: 
    """
    Interface class
    """
    CB_TYPE = None
    DEFAULT_BAUDRATE = 57600
    UNIT_ID_CHECKLIST = (1, 2, 3, 4)
    UNIT_ID_CHECKLIST_VDSM = []
    UNIT_ID_CHECKLIST_BRITER = []
    BAUDRATE_CHECKLIST = (9600,57600)

    
    def parse_reading(modbus_client):
        return get_encoded_data(modbus_client)

    
    
    def scan(configs):
        modbus_client: ModbusClient = None
        return modbus_client

                
    def set_default_settings(modbus_client):
        succeeded = False
        return succeeded


class CB_VSMD114(CBOperations): #UNIT ID 1,3
    CB_TYPE = "VSMD"

    def parse_reading(self,modbus_client):
        def get_encoded_data(modbus_client):
            pass
        return get_encoded_data(modbus_client)

    def scan(self,configs):
        modbus_client: ModbusClient = None
        VSDM_connected_count = 0
        for baudrate in self.BAUDRATE_CHECKLIST:
            configs.update({"baudrate":baudrate})
            for unit_id  in self.UNIT_ID_CHECKLIST:
                temp_client = ModbusClient(**configs)
                respond = temp_client.read_holding_registers(0x34, 1, unit= unit_id)
                if not respond.isError():
                    modbus_client = temp_client
                    self.UNIT_ID_CHECKLIST_VDSM.append(unit_id)
                    VSDM_connected_count +=1 
                    logger.loginfo("No. of VDSM connected " + str(VSDM_connected_count))
                    logger.loginfo("ID " + str(unit_id))
                if respond.isError():
                    logger.loginfo("Device that is not VDSM has been detected but not connected")
        return modbus_client

        
    def set_default_settings(self,modbus_client):
        succeeded = False
        logger.loginfo("Setting baudrate")
        baudrate = 57600
        counter_VDSM_set_number = 0
        for unit_id in self.UNIT_ID_CHECKLIST_VDSM:
            rwr = modbus_client.write_registers(0x20,baudrate,unit= unit_id)
            if not rwr.isError():
                logger.loginfo(unit_id)
                logger.loginfo("Set baudrate to 57600")
            if rwr.isError():
                logger.loginfo("ERROR in setting baudrate")
            # additional for iteration cycle 1
            if (unit_id != 1) and (unit_id != 3) and (counter_VDSM_set_number == 0):  
                    rwr_unit_ID2 = modbus_client.write_registers(0x1f,1, unit=unit_id)
                    if not rwr_unit_ID2.isError():
                        logger.loginfo("UNIT ID 1 SET")
                        counter_VDSM_set_number +=1
            elif (unit_id != 1) and (unit_id != 3) and (counter_VDSM_set_number ==1):
                rwr_unit_ID4 = modbus_client.write_registers(0x1f,3, unit=unit_id)
                if not rwr_unit_ID4.isError():
                    logger.loginfo("UNIT ID 3 SET")
            succeeded = True 
        return succeeded

    
class CB_BRITER(CBOperations): #UNIT ID 2,4
    CB_TYPE = "BRITER"
   
    def parse_reading(self,modbus_client):
        def get_encoded_data(modbus_client):
            register_values = {}
            for unit in self.UNIT_ID_CHECKLIST_BRITER:
                rhr =modbus_client.read_holding_registers(0x0,1,unit= unit)
                if rhr.isError():
                    logger.loginfo("ERROR READING")
                if not rhr.isError():
                    logger.loginfo("SUCCEEDED IN READING UNIT ID " + str(unit))
                    register_value = rhr.registers[0]
                    register_values[unit] = register_value
            register_values_str = str(register_values)
            logger.loginfo(register_values_str)
            return register_values_str
        return get_encoded_data(modbus_client)                   


    def scan(self,configs):
        modbus_client: ModbusClient = None
        BED_connected_count = 0
        for baudrate in self.BAUDRATE_CHECKLIST:
            configs.update({"baudrate":baudrate})
            for unit_id  in self.UNIT_ID_CHECKLIST:
                temp_client = ModbusClient(**configs)
                respond_VDSM = temp_client.read_holding_registers(0x34, 1, unit= unit_id)
                if respond_VDSM.isError():
                    respond = temp_client.read_holding_registers(0x2,1, unit = unit_id)
                    if not respond.isError(): 
                        modbus_client = temp_client
                        self.UNIT_ID_CHECKLIST_BRITER.append(unit_id)
                        BED_connected_count += 1
                        logger.loginfo("No. of BED connected " + str(BED_connected_count) + " ID " + str(unit_id))
                    if not respond.isError():
                        logger.loginfo("Device that is not BED and not VDSM has been detected but not connected")
        return modbus_client


    def set_default_settings(self,modbus_client):
        succeeded = False
        logger.loginfo("Setting baudrate")
        data_baudrate = 0x03
        counter_BRITER_set_number = 0
        for index,unit_id in enumerate(self.UNIT_ID_CHECKLIST_BRITER):
            rwr = modbus_client.write_register(0x0005,data_baudrate, unit = unit_id) 
            if not rwr.isError():
                logger.loginfo("Set baudrate to 57600 on ID: " + str(unit_id))
            if rwr.isError():
                logger.loginfo("Error in setting baudrate")
                logger.loginfo("ERROR ID" + str(unit_id))
            rospy.sleep(0.5)
            #iteration 1
            if (unit_id != 2) and (unit_id != 4) and (counter_BRITER_set_number == 0):
                rwr_unit_ID1 = modbus_client.write_registers(0x0005,2,unit=unit_id)
                counter_BRITER_set_number +=1
                if not rwr_unit_ID1.isError():
                    logger.loginfo("UNIT ID 2 SET")
                    self.UNIT_ID_CHECKLIST_BRITER[index] = 2
            elif (unit_id != 2) and (unit_id != 4) and (counter_BRITER_set_number == 1):
                rwr_unit_ID3 = modbus_client.write_registers(0x0005,4,unit=unit_id)
                if not rwr_unit_ID3.isError():
                    logger.loginfo("UNIT ID 4 SET")
                    self.UNIT_ID_CHECKLIST_BRITER[index]=4
        succeeded = True
        return succeeded



class CBChecker: 
    NODE_RATE = 5.0
    cb_model: CBOperations = None
    _command = None
    _state = None
    modbus_client: ModbusClient = None
    unit_id = None

    CB_DRIVER_MODEL_TABLE = { 
        CB_BRITER.CB_TYPE: CB_BRITER,
        CB_VSMD114.CB_TYPE: CB_VSMD114,
    }

    modbus_configs = {
        "method": "rtu",
        "port": "/dev/CB",
        "baudrate": 57600,
        "parity": "N",
        "timeout": 0.5,
    }

    def __init__(self) -> None:
        self.command = "NONE"
        self.cmd_params = ""
        self.state = CBCheckerStates.INIT
        self.pub_state = CB_STATE.Publisher()
        self.pub_info = CB_INFO.Publisher()
        self.pub_info_chinese = CB_INFO_CHINESE.Publisher()
        self.pub_reading = CB_DATA.Publisher()
        self.pub_configs = CB_CONFIGS.Publisher()
        self.pub_configs_chinese = CB_CONFIGS_CHINESE.Publisher()
        self.pub_data_check = CB_DATA_CHECK.Publisher()
        CB_SRV_CMD.Services(self.srv_cb)

        self.__STATES_METHODS = {
        (CBCommands.NONE, CBCheckerStates.INIT): self.initialize, # to IDLE
        #(CBCommands.SCAN, CBCheckerStates.IDLE): self.scan, # to CONNECTED or stay
        #(CBCommands.CONNECT, CBCheckerStates.IDLE): self.connect, # to CONNECTED or stay
        #(CBCommands.DISCONNECT, CBCheckerStates.CONNECTED): self.disconnect, # to IDLE
        (CBCommands.NONE, CBCheckerStates.CONNECTED): self.parse_reading, # stay
        (CBCommands.SET_DEFAULT, CBCheckerStates.CONNECTED): self.set_default_settings, # stay
        #(CBCommands.SAVE, CBCheckerStates.CONNECTED): self.save_parameters, # stay
        (CBCommands.AUTO_DETECT, CBCheckerStates.IDLE): self.auto_detect,
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
            self._command = CBCommands[value.upper()]
            logger.loginfo(f"Command set as: {self._command}")
        except Exception as e:
            self.log_with_frontend(f"Received wrong command: {value}!!",f"命令错误: {value}")
            self._command = CBCommands.NONE

    @property
    def state(self):
        return self._state
    @state.setter
    def state(self, value: CBCheckerStates):
        if self._state != value:
            self._state = value
            logger.logwarn(f"State changed to: {self._state.name}")
            self.pub_state = CB_STATE.Publisher()
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
        self.state = CBCheckerStates.IDLE
    
    def _get_current_CB_settings(self):
        return {
            "baudrate": self.modbus_configs["baudrate"],
            "ID": self.unit_id,
            }
    
    def _get_current_CB_settings_chinese(self):
        return {
            "波特率": self.modbus_configs["baudrate"],
            "ID": self.unit_id,
            }
    """
    def scan(self):
        self.state = CBCheckerStates.SCANNING
        if self._determine_CB_type():
            self.modbus_client = self.cb_model.scan(self=self.cb_model,configs=self.modbus_configs)
            if self.modbus_client:
                self.log_with_frontend(f'Scanned CB driver on baudrate: {self.modbus_configs["baudrate"]}')
                self.state = CBCheckerStates.CONNECTED
                self.pub_configs.publish(json.dumps(self._get_current_CB_settings()))
                return True
            self.state = CBCheckerStates.IDLE
            return False

    """
    
    def parse_reading(self):
        try:
            with serial.Serial(port=self.modbus_configs["port"]) as ser:  
                cb_msg = self.cb_model.parse_reading(self=self.cb_model,modbus_client = self.modbus_client)
                self.pub_reading.publish(json.dumps(cb_msg))
                check_NG_or_G = self.cb_model.check_reading(cb_msg)
                if check_NG_or_G: 
                    self.pub_data_check.publish(json.dumps("OK"))
                else:
                    self.pub_data_check.publish(json.dumps("NOT OK"))

                return True
        except (serial.SerialException, BrokenPipeError) as e:
            self.state = CBCheckerStates.IDLE
            return False
    
    def set_default_settings(self):
        if self.cb_model.set_default_settings(self= self.cb_model,modbus_client=self.modbus_client):
            self.log_with_frontend("SETTING SET","设置成功")
            self.log_with_frontend("CFG SAVED","设置保存成功")
            return True
    
    
    def auto_detect(self):
        self.state = CBCheckerStates.SCANNING
        for models in self.CB_DRIVER_MODEL_TABLE.values:
            self.modbusclient = models.scan(self=CBOperations,configs = self.modbus_configs) #will it be able to reconfigure the baudrate?
            if self.modbusclient: 
                if models == CB_BRITER:
                    self.log_with_frontend(f"Found {models.CB_TYPE} with baudrate: {self.modbusclient['baudrate']}! Unit_ID = {models.UNIT_ID_CHECKLIST_BRITER} ", f"IMU 类型 {models.CB_TYPE}, 波特率: {self.modbusclient['baudrate']}!")      
                elif models == CB_VSMD114:
                    self.log_with_frontend(f"Found {models.CB_TYPE} with baudrate: {self.modbusclient['baudrate']}! Unit_ID = {models.UNIT_ID_CHECKLIST_VDSM}", f"IMU 类型 {models.CB_TYPE}, 波特率: {self.modbusclient['baudrate']}!")
                self.pub_configs.publish(json.dumps(models.CB_TYPE))
                self.pub_configs_chinese.publish(json.dumps(models.CB_TYPE))
                self.cb_model = models
                self.state = CBCheckerStates.CONNECTED
                self.pub_configs.publish(json.dumps(self._get_current_CB_settings()))
                self.pub_configs_chinese.publish(json.dumps(self._get_current_CB_settings_chinese()))
                return True
            self.state =CBCheckerStates.IDLE
            return False


if __name__ == "__main__":
    rospy.init_node("cb_driver_node")
    cb_driver_checker = CBChecker()
    cb_driver_checker.start()




    


    