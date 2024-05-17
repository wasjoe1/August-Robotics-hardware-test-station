#!/usr/bin/env python3
# -*- coding: utf-8 -*-


"""

INSTRUCTIONS: 
1. connect to ylidar
2. get laser scan
3. get pointcloud
4. format (?)

"""

from sensor_msgs.msg import LaserScan, PointCloud2
import os
import rospy
logger = rospy
from enum import Enum,auto

import json

from meterial_inspection_tools.ros_interface import (
    LIDAR_SRV_CMD,
    LIDAR_STATE,
    LIDAR_DATA_LASERSCAN,
    #LIDAR_DATA_POINTCLOUD,
    LIDAR_INFO

)
    
class YDLIDARCommands(Enum):
    NONE = auto()
    RESET = auto()
    CONNECT = auto()
    DISCONNECT= auto()
    GET_LASERSCAN = auto()
    GET_POINTCLOUD = auto()




class YdlidarCheckerStates(Enum):
    INIT = auto()
    IDLE = auto()
    CONNECTING = auto()
    CONNECTED = auto()
    ERROR = auto()


class YDLIDAR_G2(): 
    def connect(port): # port only exists when depth cam is plugged in
        flag:bool = None
        flag = os.path.exists(port)
        if flag == True:
            logger.loginfo("Connected")
        elif flag == False: 
            logger.loginfo("problem connecting")
        return flag
    pass

class YdlidarChecker:
    port = '/dev/ydliar'
    NODE_RATE = 5.0
    ydliar_model = None
    _command = None
    _state = None

    def __init__(self) -> None:
        self.command = "NONE"
        self.state = YdlidarCheckerStates.INIT
        self.pub_state = LIDAR_STATE.Publisher()
        self.pub_info = LIDAR_INFO.Publisher()
        self.pub_reading_laserscan = LIDAR_DATA_LASERSCAN.Publisher()
        #self.pub_reading_pointcloud = LIDAR_DATA_POINTCLOUD.Publisher()
        LIDAR_SRV_CMD.Services(self.srv_cb)
        #self.subscriber_pointcloud = rospy.Subscriber('/point_cloud', PointCloud2, self.get_pointcloud_subscriber)
        self.subscriber_laserscan = rospy.Subscriber('/scan', LaserScan, self.get_laserscan_subscriber)

        self.__STATES_METHODS = {
        (YDLIDARCommands.NONE, YdlidarCheckerStates.INIT): self.initialize, # to IDLE
        (YDLIDARCommands.CONNECT, YdlidarCheckerStates.IDLE): self.connect, # to CONNECTED or stay
        (YDLIDARCommands.DISCONNECT, YdlidarCheckerStates.CONNECTED): self.disconnect, # to IDLE
        #(YDLIDARCommands.GET_POINTCLOUD, YdlidarCheckerStates.CONNECTED): self.get_pointcloud,  #get one instance of pointcloud msg
        (YDLIDARCommands.GET_LASERSCAN,YdlidarCheckerStates.CONNECTED): self.get_laserscan, #get one instance of laserscan msg
        (YDLIDARCommands.NONE, YdlidarCheckerStates.CONNECTED): self.parse_reading, # stay and stream laserscan + pointcloud readings continuously

   } 
        
    def srv_cb(self,srv):
        self.command = srv.button
        self.parameters = srv.parameter
        return True
    

    @property
    def command(self):
        return self._command
    @command.setter
    def command(self, value: str):
        try:
            self._command = YDLIDARCommands[value.upper()]
            logger.loginfo(f"Command set as: {self._command}")
        except Exception as e:
            self.log_with_frontend(f"Received wrong command: {value}!!")
            self._command = YDLIDARCommands.NONE

    @property
    def state(self):
        return self._state
    @state.setter
    def state(self, value: YdlidarCheckerStates):
        if self._state != value:
            self._state = value
            logger.logwarn(f"State changed to: {self._state.name}")
            self.pub_state = LIDAR_STATE.Publisher()
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
        self.state = YdlidarCheckerStates.IDLE

    def connect(self):
        self.state = YdlidarCheckerStates.CONNECTING
        connected = self.ydliar_model.connect(self.port)
        if connected:
            self.log_with_frontend("Connected to LIDAR")
            self.state = YdlidarCheckerStates.CONNECTED
            return True
        self.log_with_frontend(f"Failed to connect!")
        self.state = YdlidarCheckerStates.IDLE
        return False
    
    def get_pointcloud(self):
        pass


    def get_laserscan(self):
        pass

    def disconnect(self):
        self.log_with_frontend("DISCONNECTING")
        self.log_with_frontend("DISCONNECTED")
        self.state = YdlidarCheckerStates.IDLE
        return True
    

    def get_pointcloud_subscriber(self,data):
        global pointcloud_data_global
        pointcloud_data_global = data
        return data
    

    def get_laserscan_subscriber(self,data):
        global laserScan_data_global
        laserScan_data_global = data
        return data
    
    def parse_reading(self):
        self.pub_reading_laserscan.publish(laserScan_data_global)
        #self.pub_reading_pointcloud.publish(pointcloud_data_global)
        return True
    
if __name__ == "__main__":
    rospy.init_node("lidar_driver_node")
    lidar_driver_checker = YdlidarChecker()
    lidar_driver_checker.start()