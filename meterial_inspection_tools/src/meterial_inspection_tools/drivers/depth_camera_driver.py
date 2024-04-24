#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
logger = rospy
from enum import Enum,auto
import json
from sensor_msgs import point_cloud2
from sensor_msgs.msg import Image
from meterial_inspection_tools import (
    DEPTH_SRV_CMD,
    DEPTH_DATA,
    DEPTH_CONFIGS,
    DEPTH_INFO,
    DEPTH_STATE, 
)



class DEPTHCommands(Enum):
    NONE = auto()
    RESET = auto()
    SCAN = auto()
    CONNECT = auto()
    DISCONNECT = auto()
    SET_DEFAULT = auto()
    SAVE = auto()


class DepthCheckerStates(Enum): 
    INIT = auto()
    IDLE = auto()
    SCANNING = auto()
    CONNECTING = auto()
    CONNECTED = auto()
    ERROR = auto()

class ASTRA_CAMERA():
    
    def parse_reading(configs):
        pass

    def connect(configs):
        pass

    def scan(configs):
        pass
    
    def set_default_settings(configs):
        succeeded = False
        succeeded = True
        return succeeded
    
    @staticmethod
    def save_parameters(configs):
        return True
    
    @staticmethod
    def close(configs):
        return True

class DepthChecker:
    NODE_RATE = 5.0
    depthcamera_model = ASTRA_CAMERA
    _command = None
    _state = None
    

    def __init__(self) -> None:
        self.command = "NONE"
        self.state = DepthCheckerStates.INIT
        self.pub_state = DEPTH_STATE.Publisher()
        self.pub_info = DEPTH_INFO.Publisher()
        self.pub_reading = DEPTH_DATA.Publisher()
        self.pub_configs = DEPTH_CONFIGS.Publisher()
        DEPTH_SRV_CMD.Services(self.srv_cb)

        self.__STATES_METHODS = {
        (DEPTHCommands.NONE, DepthCheckerStates.INIT): self.initialize, # to IDLE
        (DEPTHCommands.SCAN, DepthCheckerStates.IDLE): self.scan, # to CONNECTED or stay
        (DEPTHCommands.CONNECT, DepthCheckerStates.IDLE): self.connect, # to CONNECTED or stay
        (DEPTHCommands.DISCONNECT, DepthCheckerStates.CONNECTED): self.disconnect, # to IDLE
        (DEPTHCommands.NONE, DepthCheckerStates.CONNECTED): self.parse_reading, # stay
        (DEPTHCommands.SET_DEFAULT, DepthCheckerStates.CONNECTED): self.set_default_settings, # stay
        (DEPTHCommands.SAVE, DepthCheckerStates.CONNECTED): self.save_parameters, # stay
    }   
        
    
    def srv_cb(self, srv):
        self.command = srv.button

    @property
    def command(self):
        return self._command
    @command.setter
    def command(self, value: str):
        try:
            self._command = DEPTHCommands[value.upper()]
            logger.loginfo(f"Command set as: {self._command}")
        except Exception as e:
            self.log_with_frontend(f"Received wrong command: {value}!!")
            self._command = DEPTHCommands.NONE

    @property
    def state(self):
        return self._state
    @state.setter
    def state(self, value: DepthCheckerStates):
        if self._state != value:
            self._state = value
            logger.logwarn(f"State changed to: {self._state.name}")
            self.pub_state = DEPTH_STATE.Publisher()
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
        self.state = DepthCheckerStates.IDLE


    def _get_current_DEPTH_settings(self):
        return {
            "baudrate": self.modbus_configs["baudrate"],
            }
    
