#!/usr/bin/python3
# -*- coding: utf-8 -*-


"""
INSTRUCTIONS: 
parameters is not required to be keyed in 

1. press the big webpage imu button --> == send connect command --> ROS system automatically auto_detect + connect
2. select baudrate
2. will check if data is NG or G 
3. just pull to unplug

"""


import serial
import rospy
import struct
logger = rospy
from pymodbus.client.sync import ModbusSerialClient as ModbusClient
from enum import Enum, auto
import json
from meterial_inspection_tools.ros_interface import (
   INCLINOMETER_STATE,
   INCLINOMETER_INFO,
   INCLINOMETER_INFO_CHINESE,
   INCLINOMETER_DATA,
   INCLINOMETER_CONFIGS,
   INCLINOMETER_CONFIGS_CHINESE,
   INCLINOMETER_DATA_CHECK,
   INCLINOMETER_SRV_CMD,
)
import pymodbus
print(pymodbus.__version__)


class InclinCommands(Enum):
    NONE = auto()
    RESET = auto()
    CONNECT = auto()
    SET_DEFAULT = auto()


class InclinCheckerStates(Enum):
    INIT = auto()
    IDLE = auto()
    SCANNING = auto()
    CONNECTING = auto()
    CONNECTED = auto()
    ERROR = auto()


class InclinOperations:

    """
    Interface class
    """

    AUTOLEVEL_INCLINOMETER_BAUDRATE = 115200
    BAUDRATE_CHECKLIST = (9600,115200)

    def connect(configs):
        modbus_client: ModbusClient = None
        unit_id_constant = None
        return modbus_client,unit_id_constant
    
    
    def scan(configs):
        modbus_client: ModbusClient = None
        unit_id = None
        return modbus_client,unit_id
    
    @staticmethod
    def parse_reading(modbus_client,unit_id):
        """
        return latest readings, only x and y applicable
        """
        def get_inclinometer_data_xy_deg(modbus_client):
            return x,y 
        return get_inclinometer_data_xy_deg(modbus_client,unit_id)
    
    @staticmethod
    def set_default_settings(modbus_client,unit_id,command_params):
        succeeded = False
        return succeeded,modbus_client.unit_id
    
    @staticmethod
    def save_parameters(modbus_client,unit_id):
        return True
    
    @staticmethod
    def close(modbus_client):
        return True
    
    def check_reading(x_y_reading):
        return True
    
class InclinSVT626T (InclinOperations):
    AUTOLEVEL_INCLINOMETER_UNIT = 3
    UNIT_ID_CHECKLIST = (1, 2, 3)
    INCLINOMETER_TYPE = "SVT"
    BAUDRATE_TABLE = {
    0x0000: 2400,
    0x0001: 4800,
    0x0002: 9600,
    0x0003: 19200,
    0x0004: 115200,
    0x0005: 14400,
    0x0006: 38400,
    0x0007: 57600
    }

    def parse_reading(modbus_client,unit_id):
        def reg2f(reg_1, reg_2):
            return struct.unpack("!f", struct.pack("!HH", reg_1, reg_2))[0]
        
        def get_inclinometer_data_xy_deg(modbus_client,unit_id):
            if not modbus_client.connect():
                logger.loginfo("CLIENT NOT CONNECTED")
            rhr = modbus_client.read_holding_registers(0x01,4,slave=unit_id)
            if rhr.isError():
                logger.loginfo(rhr)
                return None, None
            x = reg2f(*rhr.registers[0:2])
            y = reg2f(*rhr.registers[2:4])
            return x, y
    
        return get_inclinometer_data_xy_deg(modbus_client,unit_id)

    def connect(self,configs):
        modbus_client: ModbusClient = None
        unit_id_constant = None
        for unit_id in self.UNIT_ID_CHECKLIST: 
            temp_client = ModbusClient(**configs)
            x,y = self.parse_reading(temp_client,unit_id)
            logger.loginfo(unit_id)
            if x is not None:
                modbus_client = temp_client
                unit_id_constant = unit_id
        return modbus_client,unit_id_constant

    
    def scan(self,configs):
        modbus_client: ModbusClient = None
        unit_id_constant = None
        for baudrate in self.BAUDRATE_CHECKLIST:
            configs.update({"baudrate":baudrate})
            for unit_id in self.UNIT_ID_CHECKLIST: 
                temp_client = ModbusClient(**configs)
                x,y = self.parse_reading(temp_client,unit_id)
                if x is not None:
                    modbus_client = temp_client
                    unit_id_constant = unit_id
        return modbus_client,unit_id_constant
    

    def set_default_settings(self,modbus_client,unit_id,command_params):
        succeeded = False
        data_zero_point = 0x01
        logger.loginfo("Setting relative zero point")
        rwr = modbus_client.write_register(0x0B, data_zero_point, unit=unit_id)
        if not rwr.isError():
            logger.loginfo(
                "Set {} zero point".format(["Relative" if data_zero_point else "Absolute"])
            )
        baudrate_set = int(command_params)
        rwr = modbus_client.write_register(0x0C, baudrate_set, slave=unit_id)
        if not rwr.isError():
            logger.loginfo("Set baudrate to: {}".format(InclinSVT626T.BAUDRATE_TABLE[baudrate_set]))
        rwr = modbus_client.write_register(0x0D,3, unit=unit_id)
        logger.loginfo("Set unit_id to: {}".format(3))

        #update modbus_client with set baudrate
        baudrate = self.BAUDRATE_TABLE.get(baudrate_set)
        modbus_client.baudrate = baudrate

        #save
        data = 0x00
        rwr = modbus_client.write_register(0x0F, data, unit=3)
        succeeded = True
        return succeeded

        
    @staticmethod
    def save_parameters(modbus_client,unit_id):
        data = 0x00
        rwr = modbus_client.write_register(0x0F, data, unit=unit_id)
        return True

    
    def check_reading(x_y_reading):
        x, y = x_y_reading
        if x == 0 and y == 0: 
            return True
        return False

class InclinChecker:
    NODE_RATE = 5.0
    _command = None
    _state = None
    modbus_client: ModbusClient = None
    unit_id = 1

    #Set as None if more than 1 inclinometer model and add in model in inclinometer_model_table
    inclinometer_model:InclinOperations = InclinSVT626T
    INCLINOMETER_MODEL_TABLE = {
        InclinSVT626T.INCLINOMETER_TYPE:InclinSVT626T,
    }

    modbus_configs = {
        "method": "rtu",
        "port": "/dev/inclinometer",
        "baudrate": 115200,
        "parity": "N",
        "timeout": 0.5,
    }

    def __init__(self) -> None:
        self.command = "NONE"
        self.cmd_params = ""
        self.state = InclinCheckerStates.INIT
        self.pub_state = INCLINOMETER_STATE.Publisher()
        self.pub_info = INCLINOMETER_INFO.Publisher()
        self.pub_info_chinese = INCLINOMETER_INFO_CHINESE.Publisher()
        self.pub_configs =INCLINOMETER_CONFIGS.Publisher()
        self.pub_configs_chinese = INCLINOMETER_CONFIGS_CHINESE.Publisher()
        self.pub_data = INCLINOMETER_DATA.Publisher()
        self.pub_data_check = INCLINOMETER_DATA_CHECK.Publisher()
        INCLINOMETER_SRV_CMD.Services(self.srv_cb)

        self.__STATES_METHODS = {
            (InclinCommands.NONE, InclinCheckerStates.INIT): self.initialize, # to IDLE
            (InclinCommands.NONE, InclinCheckerStates.CONNECTED): self.parse_reading, # stay
            (InclinCommands.CONNECT, InclinCheckerStates.IDLE): self.connect_scan,
            (InclinCommands.SET_DEFAULT, InclinCheckerStates.CONNECTED): self.set_default_settings
        }

    
    def srv_cb(self,srv): 
        self.command = srv.button
        self.cmd_params = srv.baudrate
        return True

    @property
    def command(self):
        return self._command
    @command.setter
    def command(self, value:str):
        try: 
            self._command = InclinCommands[value.upper()]
            logger.loginfo(f"Command set as {self._command}")
        except Exception as e:
            self.log_with_frontend(f"Received wrong command: {value}", f"命令错误: {value}")
            self._command = InclinCommands.NONE


    @property
    def state(self):
        return self._state
    @state.setter
    def state(self, value: InclinCheckerStates):
        if self._state != value:
            self._state = value
            logger.logwarn(f"State changed to: {self._state.name}")
            self.pub_state = INCLINOMETER_STATE.Publisher()
            self.pub_state.publish(self.state.name)

    def log_with_frontend(self,log,log_chinese):
        logger.loginfo (log)
        self.pub_info.publish(log)
        self.pub_info_chinese.publish(log_chinese)



    def start(self):
        l =rospy.Rate(self.NODE_RATE)
        while not rospy.is_shutdown():
            state_method = self. __STATES_METHODS.get((self.command, self.state))
            if state_method:
                state_method()
                self.command = "NONE"
                l.sleep()
    
    def initialize(self):
        self.state = InclinCheckerStates.IDLE

    def _get_current_inclin_settings(self):
        return {
            "current baudrate": self.modbus_configs["baudrate"],
            "current Unit_id": self.unit_id,
            }
    
    def _get_current_inclin_settings_chinese(self):
        return {
            "波特率": self.modbus_configs["baudrate"],
            "ID": self.unit_id,
        }

    def parse_reading(self):
        try:
            with serial.Serial(port=self.modbus_configs["port"]) as ser:   
                incline_msg = self.inclinometer_model.parse_reading(self.modbus_client,self.unit_id) 
                self.pub_data.publish(json.dumps(incline_msg))
                check_NG_or_G = self.inclinometer_model.check_reading(incline_msg)
                if check_NG_or_G:
                    self.pub_data_check.publish(json.dumps("OK"))
                else:
                    self.pub_data_check.publish(json.dumps("NOT OK"))
                return True
        except (serial.SerialException, BrokenPipeError) as e:
            self.log_with_frontend("Inclinometer unplugged! Check connection","无法连接倾角仪，请确保电源再连接")
            self.state = InclinCheckerStates.IDLE
            return False
       
    
    def set_default_settings(self):
        if self.inclinometer_model.set_default_settings(self = self.inclinometer_model,modbus_client=self.modbus_client,unit_id=self.unit_id, command_params=self.cmd_params):
            self.log_with_frontend("SETTING SET","设置成功")

             #save set settings
            if self.inclinometer_model.save_parameters(self.modbus_client,self.unit_id):
                self.log_with_frontend("CFG saved","设置保存成功")
                return True

    def connect_scan(self):
        #connect and scan merged
        self.state = InclinCheckerStates.SCANNING
        self. modbus_client,self.unit_id= self.inclinometer_model.scan(self=self.inclinometer_model,configs = self.modbus_configs)
        if self.modbus_client:
            self.log_with_frontend(f'Scanned Inclinometer on baudrate: {self.modbus_configs["baudrate"]}', f'波特率 {self.modbus_configs["baudrate"]}')
            self.state = InclinCheckerStates.CONNECTED
            self.pub_configs.publish(json.dumps(self._get_current_inclin_settings()))
            self.pub_configs_chinese.publish(json.dumps(self._get_current_inclin_settings_chinese()))
            return True
        self.log_with_frontend(f"Cannot found the Inclinometer!")
        self.state = InclinCheckerStates.IDLE
        return False


if __name__ == "__main__":
    rospy.init_node("inclinometer_driver_node")
    incline_checker = InclinChecker()
    incline_checker.start()




