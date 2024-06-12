#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
INSTRUCTIONS: 
1.ensure pymodbus is in 1.5.2 version
2. this script can read data for multiple sonars, but can only set one sonar at a time

Code logic: 
flag_check_set_method_available == None
1. scan for multiple sonars --> if more than 1 --> can only read and therefore not set --> flag ==False for multiple sonars
 if flag == false, pass set settings 
2. if have one --> flag == True. can set settings (take current unit_id) --> set unit_id

"""

# -------------------------------------------------------------------------------------------------
# -------------------------------------------------------------------------------------------------
import os
import datetime
from enum import Enum, auto
import json
import csv
import threading
import serial
import rospy
logger = rospy
import pymodbus
from pymodbus.client.sync import ModbusSerialClient as ModbusClient
from meterial_inspection_tools.ros_interface import (
    SONAR_SRV_CMD,
    SONAR_DATA,
    SONAR_CONFIGS,
    SONAR_CONFIGS_CHINESE,
    SONAR_INFO,
    SONAR_INFO_CHINESE,
    SONAR_STATE,
)

# -------------------------------------------------------------------------------------------------
# -------------------------------------------------------------------------------------------------
# ENUM CLASSES
class SonarCommands(Enum):
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

# -------------------------------------------------------------------------------------------------
# -------------------------------------------------------------------------------------------------
class DYPSonar():
    BAUDRATE_CHECKLIST = [9600,57600,115200] # is this still necessary(?)
    DEFAULT_BAUDRATE = 0x0003 # corresponds to 9600 bps
    # DEFAULT_BAUDRATE = 3 # corresponds to 9600 bps
    # total 10 available IDs but 8 slots to connect sonars
    UNIT_DICT_CHECKER = [0xe6 ,0xe8 ,0xd0,0xfa,0xfe,0xea,0xe4,0xe2,0xd2,0xec] #for ID that has already been set
    UNIT_DICT_SETTER = {
        "0xe6": 0xe6,
        "0xe8": 0xe8,
        "0xd0": 0xd0,
        "0xfa": 0xfa,
        "0xfe": 0xfe,
        "0xea": 0xea,
        "0xe4": 0xe4,
        "0xe2": 0xe2,
        "0xd2": 0xd2,
        "0xec": 0xec,
    }
    UNIT_CHECKER = set() # use a set to check
    SET_AVAILABLE_FLAG = False

    # ---------------------------------------------------------------------------------------------
    # FUNCTIONS
    def loop_distance(self, modbus_client):
        distances = [None]*10
        unitIDs = [None]*10
        paired_values = []
        
        for i, unit in enumerate(DYPSonar.UNIT_CHECKER): # not sure if you intend to have this as a class or instance variable, but u declared as a class variable
            unit = self.UNIT_DICT_SETTER[unit]
            response = modbus_client.read_holding_registers(0x0101, 1, unit=unit)
            if not response.isError(): # can find the unit ID
                distance = response.registers[0]
                if distance is not None: # ignore when distance is not valid since distances[i] & unitIDs[i] are already set to be None initially
                    distances[i] = format(distance/1000.,'.2f')
                    unitIDs[i] = format(hex(unit)) # 
                paired_values.append({ "distance":distances[i], "unit" : unitIDs[i] })
            else: 
                logger.loginfo("error")

        return distances, unitIDs, paired_values
    
    def format_and_print_sonars(self, distances, unitIDs): 
        formatted_data = """
                {} : {}
                {} : {}
                {} : {}
                {} : {}
                {} : {}
                {} : {}
                {} : {}
                {} : {}
                """.format(
                    format(unitIDs[0]), format(distances[0]),
                    format(unitIDs[1]), format(distances[1]),
                    format(unitIDs[2]), format(distances[2]),
                    format(unitIDs[3]), format(distances[3]),
                    format(unitIDs[4]), format(distances[4]),
                    format(unitIDs[5]), format(distances[5]),
                    format(unitIDs[6]), format(distances[6]),
                    format(unitIDs[7]), format(distances[7]),)
        logger.loginfo(formatted_data)
        return formatted_data
    # ---------------------------------------------------------------------------------------------
    # PROCESSES
    def scan(self, configs):
        modbus_client : ModbusClient = None

        for baudrate in self.BAUDRATE_CHECKLIST:
            configs.update({"baudrate":baudrate})
            logger.loginfo(baudrate)

            temp_client = ModbusClient(**configs)
            logger.loginfo(temp_client)

            for unit_string, unit in self.UNIT_DICT_SETTER.items(): # put UNIT_DICT_CHECKER_DEFAULT into UNIT_DICT_CHECKER also
                sonar_unit = temp_client.read_holding_registers(0x200, 1,unit=unit)
                if not sonar_unit.isError(): # if valid
                    self.UNIT_CHECKER.add(unit_string)
                    modbus_client = temp_client
                else: # else
                    logger.loginfo(f"Failed to connect to unit ID {unit_string}")
            logger.loginfo(f"There are currently {len(self.UNIT_CHECKER)} sonars connected")
            self.SET_AVAILABLE_FLAG = len(self.UNIT_CHECKER) == 1 # false if UNIT CHECKER has more than 1 sonar_unit

        return modbus_client

    def parse_readings(self, modbus_client): # self.sonar_model.parse_readings(self=self.sonar_model, modbus_client=self.modbus_client)
        distances_inputs, unitIDs, paired_values = self.loop_distance(modbus_client)
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
        return self.format_and_print_sonars(distances_inputs, unitIDs)
    
    # version 1: only 1 unit_id
    def set_default_settings(self, modbus_client, id_to_set_string):
        if (not self.SET_AVAILABLE_FLAG):
            logger.loginfo(f"set_available flag is False")
            return False
        curr_id_string = self.UNIT_CHECKER.pop()
        logger.loginfo(f"TEST: {curr_id_string}")
        curr_id = self.UNIT_DICT_SETTER[curr_id_string] # returns curr id in hex
        logger.loginfo(f"TEST: {id_to_set_string}")
        id_to_set = self.UNIT_DICT_SETTER[id_to_set_string] # change to the hex val
        logger.loginfo(f"TEST: {self.DEFAULT_BAUDRATE}")
        
        # Set baudrate TODO: Find out if we do need to set baudrate
        rwr_baudrate = modbus_client.write_register(0x201, self.DEFAULT_BAUDRATE, unit=curr_id) # set baudrate to 9600
        if rwr_baudrate.isError():
            self.UNIT_CHECKER.add(curr_id_string)
            logger.loginfo("PROBLEM SETTING BAUDRATE")
            return False
        logger.loginfo("Baudrate set to 9600")
        
        # Set unit
        rospy.sleep(0.1)
        rwr_unit = modbus_client.write_register(0x200, id_to_set, unit=curr_id)
        if rwr_unit.isError():
            self.UNIT_CHECKER.add(curr_id_string)
            logger.loginfo("PROBLEM SETTING UNIT")
            return False

        self.UNIT_CHECKER.add(id_to_set_string)
        logger.loginfo(f"unit ID was set from {curr_id} to {id_to_set}")
        return True
    
    # version 2: mutltiple unit_id => python has no overloading
    # def set_default_settings(self, modbus_client, curr_id_string, id_to_set_string): # all ids here are string
    #     if (curr_id not in self.UNIT_CHECKER) or (id_to_set in self.UNIT_CHECKER):
    #         return False
    #     id_to_set = self.UNIT_DICT_SETTER[id_to_set_string] # change to the hex val
    #     curr_id = self.UNIT_DICT_SETTER[curr_id_string] # change to the hex val
        
    #     # Set baudrate TODO: Find out if we do need to set baudrate
    #     rwr_baudrate = modbus_client.write_register(0x201, self.DEFAULT_BAUDRATE, unit=curr_id) # set baudrate to 9600
    #     if rwr_baudrate.isError():
    #         logger.loginfo("PROBLEM SETTING BAUDRATE")
    #         return False
    #     logger.loginfo("Baudrate set to 9600")
        
    #     # Set unit
    #     rospy.sleep(0.1)
    #     rwr_unit = modbus_client.write_register(0x200, id_to_set, unit=curr_id)
    #     if rwr_unit.isError():
    #         logger.loginfo("PROBLEM SETTING UNIT")
    #         return False
    #     logger.loginfo(f"unit ID was set from {curr_id} to {id_to_set}")

    #     # if true, remove the old id & add the new id
    #     self.UNIT_CHECKER.remove(curr_id_string)
    #     self.UNIT_CHECKER.add(id_to_set_string)
    #     return True
        
# -------------------------------------------------------------------------------------------------
# -------------------------------------------------------------------------------------------------
class SonarChecker:
    NODE_RATE = 5.0 # in start
    sonar_model = DYPSonar()
    modbus_client : ModbusClient = None
    modbus_configs = {
        "method": "rtu",
        "port": "/dev/sonar",
        "baudrate": 9600,
        "parity": "N",
        "timeout": 0.5,
    }
    _command = None
    _state = None

    def __init__(self) -> None:
        # cmd & state
        self.command = "NONE"
        self.state = SonarCheckerStates.INIT
        
        # service calls
        self.cmd_params = ""
        SONAR_SRV_CMD.Services(self.srv_cb)
        
        # publishers
        self.pub_state = SONAR_STATE.Publisher()
        self.pub_info = SONAR_INFO.Publisher()
        self.pub_info_chinese = SONAR_INFO_CHINESE.Publisher()
        self.pub_reading = SONAR_DATA.Publisher()
        self.pub_configs = SONAR_CONFIGS.Publisher()
        self.pub_configs_chinese = SONAR_CONFIGS_CHINESE.Publisher()
        
        # processes
        self.parse_thread = None
        self.__STATES_METHODS = {
            (SonarCommands.NONE, SonarCheckerStates.INIT): self.initialize, # From INIT to IDLE; NONE
            (SonarCommands.CONNECT, SonarCheckerStates.IDLE): self.scan, # From IDLE to CONNECTED / IDLE; CONNECT
            (SonarCommands.NONE, SonarCheckerStates.CONNECTED): self.get_readings, # CONNECTED; NONE
            (SonarCommands.SET_DEFAULT, SonarCheckerStates.CONNECTED): self.set_default_settings, # CONNECTED; SET_DEFAULT
        }

    # Service call cb
    def srv_cb(self, srv):
        self.command = srv.button
        self.cmd_params = srv.ID
        return True

    # Set command for system
    @property
    def command(self):
        return self._command
    @command.setter
    def command(self, value: str):
        try:
            self._command = SonarCommands[value.upper()]
            logger.loginfo(f"Command set as: {self._command}")
        except Exception as e:
            self.log_to_web(f"Received wrong command: {value}!!",f"命令错误: {value}")
            self._command = SonarCommands.NONE

    # Set state for system
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

    # start the ROS loop
    def start(self):
        l = rospy.Rate(self.NODE_RATE)
        while not rospy.is_shutdown():
            state_method = self.__STATES_METHODS.get((self.command,self.state))
            if state_method:
                state_method()
            self.command = "NONE"
            l.sleep()

    # ---------------------------------------------------------------------------------------------
    # PROCESSES
    def initialize(self):
        self.state = SonarCheckerStates.IDLE

    def scan(self):
        self.state = SonarCheckerStates.SCANNING # IDLE b4
        self.modbus_client = self.sonar_model.scan(configs=self.modbus_configs)
        if self.modbus_client:
            self.log_to_web(f'Scanned sonar on baudrate: {self.modbus_configs["baudrate"]}',f'波特率: {self.modbus_configs["baudrate"]}')
            self.pub_configs.publish(json.dumps(self.get_current_baudrate()))
            self.pub_configs_chinese.publish(json.dumps(self.get_current_baudrate()))
            self.state = SonarCheckerStates.CONNECTED # set state last so web & topics are updated with curr settings
            return True
        self.log_to_web(f"Failed to connect sonar driver!, Check connection",f"无法连上 sonar")
        self.state = SonarCheckerStates.IDLE
        return False

    def get_readings(self):
        if self.parse_thread and self.parse_thread.is_alive():
            return False
        def parse_target():
            try:
                sonar_msg = self.sonar_model.parse_readings(modbus_client=self.modbus_client)
                rospy.sleep(0.5) # Previously rospy.sleep(0.001), same as sonar reading code on lionel
                self.pub_reading.publish(json.dumps(sonar_msg))
            except serial.SerialException:
                self.sonar_model.UNIT_CHECKER = []
                self.log_to_web("Sonar unplugged! Check connection","无法连接IMU，请确保电源再连接")
                self.state = SonarCheckerStates.IDLE

        self.parse_thread = threading.Thread(target=parse_target)
        self.parse_thread.start()
    
    def set_default_settings(self):
        if self.parse_thread and self.parse_thread.is_alive():
            self.parse_thread.join()
        logger.loginfo(f"TEST: {self.cmd_params}")
        if self.sonar_model.set_default_settings(modbus_client=self.modbus_client, id_to_set_string=self.cmd_params):
            self.log_to_web("DEFAULT SETTINGS SET Baudrate & Unit ID was updated","设置成功")
            self.log_to_web("CFG SAVED","设置保存成功")
        else: 
            self.log_to_web(f"Error in setting Baudrate/ Unit ID for {self.cmd_params}", "系统只能同时设置一个sonar") # TODO: Change the chinese

    # ---------------------------------------------------------------------------------------------
    # FUNCTIONS
    # Logging to web
    def log_to_web(self, log_english, log_chinese):
        logger.loginfo(log_english)
        logger.loginfo(log_chinese)
        self.pub_info.publish(log_english)
        self.pub_info_chinese.publish(log_chinese)
    
    # Get baudrate
    def get_current_baudrate(self):
        return {
            "baudrate": self.modbus_configs["baudrate"],
            "波特率": self.modbus_configs["baudrate"],
        }        
    
# -------------------------------------------------------------------------------------------------
# -------------------------------------------------------------------------------------------------
# INIT MAIN
if __name__ == "__main__":
    rospy.init_node("sonar_driver_node")
    sonar_checker = SonarChecker()
    sonar_checker.start()