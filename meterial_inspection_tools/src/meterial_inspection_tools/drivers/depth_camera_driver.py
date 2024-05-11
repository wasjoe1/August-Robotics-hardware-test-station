#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from sensor_msgs.msg import Image
import rospy
#import message_filters
#import cv2
#import numpy as np
#from cv_bridge import CvBridge
#from tf2_ros import TransformListener,Buffer
#import math
#import tf2_ros
#from geometry_msgs.msg import PointStamped
#from tf2_geometry_msgs import do_transform_point
from sensor_msgs.msg import PointCloud2
import os

import rospy
logger = rospy
from enum import Enum,auto
import json
from meterial_inspection_tools.ros_interface import (
    DEPTH_SRV_CMD,
    DEPTH_DATA,
    DEPTH_INFO,
    DEPTH_DATA,
    DEPTH_STATE
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
    # DATA publish value is in pointcloud2 format

    def connect(port):
        flag:bool = None
        flag = os.path.exists(port)
        if flag == True:
            logger.loginfo("Connected")
        elif flag == False: 
            logger.loginfo("problem connecting")
        return flag

class DepthChecker:
    port = '/dev/astrauvc'
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
        DEPTH_SRV_CMD.Services(self.srv_cb)
        self.subscriber = rospy.Subscriber('/camera/depth_registered/points', PointCloud2, self.get_data_subscriber)

        self.__STATES_METHODS = {
        (DEPTHCommands.NONE, DepthCheckerStates.INIT): self.initialize, # to IDLE
        (DEPTHCommands.CONNECT, DepthCheckerStates.IDLE): self.connect, # to CONNECTED or stay
        (DEPTHCommands.DISCONNECT, DepthCheckerStates.CONNECTED): self.disconnect, # to IDLE
        (DEPTHCommands.NONE, DepthCheckerStates.CONNECTED): self.parse_reading, # stay
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

    def connect(self):
        self.state = DepthCheckerStates.CONNECTING
        connected = self.depthcamera_model.connect(self.port)
        if connected:
            self.log_with_frontend("Connected to astra camera")
            self.state = DepthCheckerStates.CONNECTED
            return True
        self.log_with_frontend(f"Failed to connect to astra camera!")
        self.state = DepthCheckerStates.IDLE
        return False

    def disconnect(self):
        self.log_with_frontend("DISCONNECTING")
        self.state = DepthCheckerStates.IDLE
        return True
    
    def get_data_subscriber(self,data):
        global data_global
        data_global = data
        return data
    
    def parse_reading(self):
        #logger.loginfo(data_global)
        logger.loginfo(type(data_global))
        self.pub_reading.publish(data_global)
        return True
    
        
if __name__ == "__main__":
    rospy.init_node("depth_camera_driver_node")
    depth_driver_checker = DepthChecker()
    depth_driver_checker.start()