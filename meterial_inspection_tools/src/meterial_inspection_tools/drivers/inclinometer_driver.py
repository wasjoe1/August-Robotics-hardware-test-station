#!/usr/bin/python3
# -*- coding: utf-8 -*-


"""
INSTRUCTIONS: 
parameters is not required to be keyed in 

1. press the big webpage imu button --> == send connect command --> ROS system automatically connect + set + save
2. will check if data is NG or G 
3. press disconnect/just pull to unplug

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
   INCLINOMETER_DATA,
   INCLINOMETER_SRV_CMD,
)
import pymodbus
print(pymodbus.__version__)


class InclinCommands(Enum):
    NONE = auto()
    RESET = auto()
    #SCAN = auto()
    CONNECT = auto()
    DISCONNECT = auto()
    #SET_DEFAULT = auto()
    #SAVE = auto()

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
    def set_default_settings(modbus_client,unit_id):
        succeeded = False
        return succeeded,modbus_client.unit_id
    
    @staticmethod
    def save_parameters(modbus_client,unit_id):
        return True
    
    @staticmethod
    def close(modbus_client):
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
    
    @staticmethod
    def set_default_settings(modbus_client,unit_id):
        succeeded = False
        data_zero_point = 0x01
        logger.loginfo("Setting relative zero point")
        rwr = modbus_client.write_register(0x0B, data_zero_point, unit=unit_id)
        if not rwr.isError():
            logger.loginfo(
                "Set {} zero point".format(["Relative" if data_zero_point else "Absolute"])
            )
        baudrate_set = 0x0004
        rwr = modbus_client.write_register(0x0C, baudrate_set, slave=unit_id)
        if not rwr.isError():
            logger.loginfo("Set baudrate to: {}".format(InclinSVT626T.BAUDRATE_TABLE[baudrate_set]))
        rwr = modbus_client.write_register(0x0D,3, unit=unit_id)
        logger.loginfo("Set unit_id to: {}".format(3))
        succeeded = True
        return succeeded

        
    @staticmethod
    def save_parameters(modbus_client,unit_id):
        data = 0x00
        rwr = modbus_client.write_register(0x0F, data, unit=unit_id)
        return True
    
    @staticmethod
    def close(modbus_client):
        return True
    
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
        #self.pub_configs =INCLINOMETER_CONFIGS.Publisher()
        self.pub_data = INCLINOMETER_DATA.Publisher()
        INCLINOMETER_SRV_CMD.Services(self.srv_cb)

        self.__STATES_METHODS = {
            (InclinCommands.NONE, InclinCheckerStates.INIT): self.initialize, # to IDLE
            #(InclinCommands.SCAN, InclinCheckerStates.IDLE): self.scan, # to CONNECTED or stay
            #(InclinCommands.CONNECT, InclinCheckerStates.IDLE): self.connect, # to CONNECTED or stay
            #(InclinCommands.DISCONNECT, InclinCheckerStates.CONNECTED): self.disconnect, # to IDLE
            (InclinCommands.NONE, InclinCheckerStates.CONNECTED): self.parse_reading, # stay
            #(InclinCommands.SET_DEFAULT, InclinCheckerStates.CONNECTED): self.set_default_settings, # stay
            #(InclinCommands.SAVE, InclinCheckerStates.CONNECTED): self.save_parameters, # stay
            #(InclinCommands.NONE, InclinCheckerStates.IDLE):self.check_connection
            (InclinCommands.CONNECT, InclinCheckerStates.IDLE): self.connect_set_save, #all in one step --> iteration 1
        }

    
    def srv_cb(self,srv): 
        self.command = srv.button
        #self.cmd_params = srv.parameter
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
            self.log_with_frontend(f"Received wrong command: {value}")
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

    def log_with_frontend(self,log):
        logger.loginfo (log)
        self.pub_info.publish(log)

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
            "baudrate": self.modbus_configs["baudrate"]
            }
    
    """
    def connect(self):
        self.state = InclinCheckerStates.CONNECTING
        self. modbus_client,self.unit_id = self.inclinometer_model.connect(self=self.inclinometer_model,configs=self.modbus_configs)
        if self.modbus_client:
            self.log_with_frontend(f"Connected on baudrate: 115200")
            self.state = InclinCheckerStates.CONNECTED
            self.log_with_frontend(json.dumps(self._get_current_inclin_settings()))
            return True
        self.log_with_frontend(f"Failed to connect Inclinometer!")
        self.state = InclinCheckerStates.IDLE
        return False
    
    def scan(self):
        self.state = InclinCheckerStates.SCANNING
        self. modbus_client,self.unit_id= self.inclinometer_model.scan(self=self.inclinometer_model,configs = self.modbus_configs)
        if self.modbus_client:
            self.log_with_frontend(f'Scanned Inclinometer on baudrate: {self.modbus_configs["baudrate"]}')
            self.state = InclinCheckerStates.CONNECTED
            self.log_with_frontend(json.dumps(self._get_current_inclin_settings()))
            return True
        self.log_with_frontend(f"Cannot found the Inclinometer!")
        self.state = InclinCheckerStates.IDLE
        return False
    """

    def disconnect(self):
        self.modbus_client  = None
        self.log_with_frontend("DISCONNECTING")
        self.log_with_frontend("DISCONNECTED")
        self.state = InclinCheckerStates.IDLE
        return True
    
    def parse_reading(self):
        try:
            with serial.Serial(port=self.modbus_configs["port"]) as ser:   
                incline_msg = self.inclinometer_model.parse_reading(self.modbus_client,self.unit_id) 
                self.pub_data.publish(json.dumps(incline_msg))
                return True
        except serial.SerialException:
            self.log_with_frontend("Inclinometer unplugged! Check connection")
            self.state = InclinCheckerStates.IDLE
            return False
       
    """
    def set_default_settings(self):
        if self.inclinometer_model.set_default_settings(self.modbus_client,self.unit_id):
            self.log_with_frontend("SETTING SET")
            return True
    
    def save_parameters(self): 
        if self.inclinometer_model.save_parameters(self.modbus_client,self.unit_id):
            self.log_with_frontend("CFG saved")
            return True
    """

    def connect_set_save(self):
        #scan
        self.state = InclinCheckerStates.SCANNING
        self. modbus_client,self.unit_id= self.inclinometer_model.scan(self=self.inclinometer_model,configs = self.modbus_configs)
        if self.modbus_client:
            self.log_with_frontend(f'Scanned Inclinometer on baudrate: {self.modbus_configs["baudrate"]}')
            self.state = InclinCheckerStates.CONNECTED
            self.log_with_frontend(json.dumps(self._get_current_inclin_settings()))

            #set 
            if self.inclinometer_model.set_default_settings(self.modbus_client,self.unit_id):
                self.log_with_frontend("SETTING SET")

            #save
            if self.inclinometer_model.save_parameters(self.modbus_client,self.unit_id):
                self.log_with_frontend("CFG saved")
            
            return True
        self.log_with_frontend(f"Cannot found the Inclinometer!")
        self.state = InclinCheckerStates.IDLE
        return False

    """
    def check_connection(self):
        try:
            with serial.Serial(port=self.modbus_configs["port"]) as ser:
                logger.loginfo("no problem detecting port")
                return True
        except serial.SerialException:
            logger.loginfo("problem detecting port")
            self.log_with_frontend("INCLINOMETER_USB NOT CONNECTED")
            return False
    """


if __name__ == "__main__":
    rospy.init_node("inclinometer_driver_node")
    incline_checker = InclinChecker()
    incline_checker.start()




