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
from pymodbus.exceptions import ConnectionException
from collections import deque # for moving window filter 
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
    CONNECT = auto()
    SET_DEFAULT = auto()

class CBCheckerStates(Enum): 
    INIT = auto()
    IDLE = auto()
    SCANNING = auto()
    CONNECTED = auto()
    ERROR = auto()


class CBOperations: 
    """
    Interface class
    """
    CB_TYPE = None
    DEFAULT_BAUDRATE = 57600
    #UNIT_ID_CHECKLIST = (1, 2, 3, 4)
    UNIT_ID_CHECKLIST = (1,2)
    UNIT_ID_CHECKLIST_VDSM = []
    UNIT_ID_CHECKLIST_BRITER = []
    BAUDRATE_CHECKLIST = (9600,115200,57600)

    def check_reading():
        data_ok_flag = False
        return data_ok_flag
    
    def parse_reading(modbus_client):
        return get_encoded_data(modbus_client)
    
    def scan(configs):
        modbus_client: ModbusClient = None
        return modbus_client

                
    def set_default_settings(modbus_client,set_ID):
        succeeded = False
        return succeeded

    def create_frame(cb_msg):
        pass
    
    def frame_shift(cb_msg_new):
        pass

class CB_VSMD114(CBOperations): #UNIT ID 1,3, # do not need to parse data
    CB_TYPE = "VSMD"

    #no need to check data
    def check_reading(cb_msg):
        data_ok_flag = False
        data_ok_flag ==True 
        return data_ok_flag
    
    def create_frame(cb_msg):
        pass
    
    def frame_shift(cb_msg_new):
        pass

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

        
    def set_default_settings(self,modbus_client,set_ID):
        succeeded = False
        logger.loginfo("Setting baudrate VSMD")
        baudrate = 57600
        counter_VDSM_set_number = 0
        set_ID_int = int(set_ID)
        for unit_id in self.UNIT_ID_CHECKLIST_VDSM:
            #set baudrate
            rwr = modbus_client.write_registers(0x20,baudrate,unit= unit_id)
            if not rwr.isError():
                logger.loginfo(str(unit_id)+"Set baudrate to 57600")
                modbus_client.baudrate = baudrate
            elif rwr.isError():
                logger.loginfo("ERROR in setting baudrate")
            
            #set ID
            if (unit_id !=set_ID_int):
                rwr_unit_ID2 = modbus_client.write_registers(0x1f,1, unit=unit_id)
                if not rwr_unit_ID2.isError():
                    logger.loginfo(set_ID + "UNIT ID SET")
                    counter_VDSM_set_number +=1
                    logger.loginfo(str(counter_VDSM_set_number) + "number set")

            """
            if (unit_id != 1) and (unit_id != 3) and (counter_VDSM_set_number == 0):  
                    rwr_unit_ID2 = modbus_client.write_registers(0x1f,1, unit=unit_id)
                    if not rwr_unit_ID2.isError():
                        logger.loginfo("UNIT ID 1 SET")
                        counter_VDSM_set_number +=1
            elif (unit_id != 1) and (unit_id != 3) and (counter_VDSM_set_number ==1):
                rwr_unit_ID4 = modbus_client.write_registers(0x1f,3, unit=unit_id)
                if not rwr_unit_ID4.isError():
                    logger.loginfo("UNIT ID 3 SET")
            """
            succeeded = True 
        return succeeded

    
class CB_BRITER(CBOperations): #UNIT ID 2,4
    CB_TYPE = "BRITER"
    window_size = 3
    window = None
    frame = []
    tolerance = 2 
    data_ok_flag = None
    
    """ 
    1. create window with window_size of 3
    2. create frame by appending cb_msg value x 4 times
    3. add frame to window, window fits first 3 values
    4. get average of window     
    5. get newest/lsat value in frame
    6. compare the newest/last -average window value to check if less than 2
    7. remove the oldest value and add in a new reading value
    8. repeat step 1 -7
    """

    #add encoder value into list 
    def create_frame(self,cb_msg):
        if len(self.frame) <4: 
            self.frame.append(cb_msg)
    
    def add_frame_to_window(self):
        window_frame = self.frame[:3]
        self.window= window_frame

    def average_frame(self,window):
        logger.loginfo(window)
        total_sum = 0
        for item in window:
            total_sum +=item
        average = total_sum / len(window)
        return average
    
    def check_within_tolerance(self,average_frame):
        current_value = list(self.frame)[-1]
        difference = current_value-average_frame
        if abs(difference) <= self.tolerance:
            return True
        else: 
            return False
    
    def frame_shift(self,cb_msg_new): 
        removed_value = self.frame.pop(0)
        logger.loginfo(removed_value)
        self.frame.append(cb_msg_new)
        logger.loginfo(self.frame)

    def check_reading(self):
        self.add_frame_to_window(self)
        average_frame = self.average_frame(self,window=self.window)
        check_result = self.check_within_tolerance(self,average_frame)
        return check_result


    
    def parse_reading(self,modbus_client):
        def get_encoded_data(modbus_client):
            logger.loginfo(modbus_client)
            register_values = {}
            for unit in self.UNIT_ID_CHECKLIST_BRITER:
                rhr =modbus_client.read_holding_registers(0x0,1,unit= unit)
                if rhr.isError():
                    logger.loginfo("ERROR READING")
                if not rhr.isError():
                    logger.loginfo("SUCCEEDED IN READING UNIT ID " + str(unit))
                    register_value = rhr.registers[0]
                    #register_values[unit] = register_value
            #register_values_str = str(register_values)
            #logger.loginfo(register_values_str)
            #return register_values_str
                    return register_value
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
                        logger.loginfo("Device that is not BED has been detected but not connected")
        return modbus_client


    def set_default_settings(self,modbus_client,set_ID):
        succeeded = False
        logger.loginfo("Setting baudrate")
        data_baudrate = 0x03
        set_ID_int = int(set_ID)
        counter_BRITER_set_number = 0
        for unit_id in self.UNIT_ID_CHECKLIST_BRITER:
            #set baudrate
            rwr = modbus_client.write_register(0x0005,data_baudrate, unit = unit_id) 
            if not rwr.isError():
                logger.loginfo("Set baudrate to 57600 on ID: " + str(unit_id))
            elif rwr.isError():
                logger.loginfo("Error in setting baudrate")
                logger.loginfo("ERROR ID" + str(unit_id))

            #set unit ID
            if (unit_id !=set_ID_int):
                rwr_unit_ID1 = modbus_client.write_registers(0x0005,set_ID_int,unit=unit_id)
                if not rwr_unit_ID1.isError():
                    counter_BRITER_set_number +=1
                    logger.loginfo(set_ID +" UNIT ID SET")

            """
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
            """
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
        (CBCommands.NONE, CBCheckerStates.CONNECTED): self.parse_reading, # stay
        (CBCommands.SET_DEFAULT, CBCheckerStates.CONNECTED): self.set_default_settings, # stay
        (CBCommands.CONNECT, CBCheckerStates.IDLE): self.auto_detect,
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
    
    def parse_reading(self):
        try:
            with serial.Serial(port=self.modbus_configs["port"]) as ser:  
                logger.loginfo("parsereadinglogger")
                #logger.loginfo(self.modbus_client)
                for i in range(4):
                    cb_msg = self.cb_model.parse_reading(self=self.cb_model,modbus_client = self.modbus_client)
                    if cb_msg is not None:
                        self.pub_reading.publish(json.dumps(cb_msg))
                        self.cb_model.create_frame(self=self.cb_model,cb_msg=cb_msg)
                check_reading = self.cb_model.check_reading(self.cb_model)
                cb_msg_new = cb_msg = self.cb_model.parse_reading(self=self.cb_model,modbus_client = self.modbus_client)
                self.cb_model.frame_shift(self=self.cb_model,cb_msg_new=cb_msg_new)
                logger.loginfo("frameshifted")
                logger.loginfo(check_reading)
                if check_reading == True: 
                    self.pub_data_check.publish("OK")
                elif check_reading == False: 
                    self.pub_data_check.publish("NOT OK")
                else:
                    self.pub_data_check.publish("ERROR")
                return True
        except (serial.SerialException, BrokenPipeError, ConnectionException) as e:
            self.log_with_frontend("CB unplugged! Check connection","无法连接CB，请确保电源在连接")
            self.state = CBCheckerStates.IDLE
            return False
    
    def set_default_settings(self):
        if self.cb_model.set_default_settings(self= self.cb_model,modbus_client=self.modbus_client,set_ID=self.cmd_params):
            self.log_with_frontend("SETTING SET","设置成功")
            self.log_with_frontend("CFG SAVED","设置保存成功")
            return True
    
    
    def auto_detect(self):
        self.state = CBCheckerStates.SCANNING
        for models in self.CB_DRIVER_MODEL_TABLE.values():
            self.modbus_client = models.scan(self=CBOperations,configs = self.modbus_configs) 
            if self.modbus_client: 
                if models == CB_BRITER:
                    self.log_with_frontend(f"Found {models.CB_TYPE} with baudrate: {self.modbus_client.baudrate}! Unit_ID = {models.UNIT_ID_CHECKLIST_BRITER} ", f"IMU 类型 {models.CB_TYPE}, 波特率: {self.modbus_client.baudrate}!")      
                elif models == CB_VSMD114:
                    self.log_with_frontend(f"Found {models.CB_TYPE} with baudrate: {self.modbus_client.baudrate}! Unit_ID = {models.UNIT_ID_CHECKLIST_VDSM}", f"IMU 类型 {models.CB_TYPE}, 波特率: {self.modbus_client.baudrate}!")
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




    


    