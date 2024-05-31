#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
INSTRUCTIONS: 
** need commands and model_ type in parameters
1. connect to ylidar
2. get laser scan (optional)
3. get pointcloud
"""

from ctypes import *
from sensor_msgs.msg import LaserScan, PointCloud2
from laser_geometry import LaserProjection
import os
import rospy
logger = rospy
from enum import Enum,auto
import json
import sensor_msgs.point_cloud2 as pc2
from meterial_inspection_tools.ros_interface import (
    LIDAR_SRV_CMD,
    LIDAR_STATE,
    LIDAR_DATA_LASERSCAN,
    LIDAR_DATA_POINTCLOUD,
    LIDAR_INFO,
    LIDAR_INFO_CHINESE,

)
    
class YDLIDARCommands(Enum):
    NONE = auto()
    RESET = auto()
    CONNECT = auto()
    GET_LASERSCAN = auto()
    GET_POINTCLOUD = auto()




class YdlidarCheckerStates(Enum):
    INIT = auto()
    IDLE = auto()
    CONNECTING = auto()
    CONNECTED = auto()
    ERROR = auto()

class YDLIAROperations: 

    """
    Interface class
    """

    YDLIDAR_TYPE = None

    def connect(port):
        flag:bool = None
        return flag
    
    def get_laserscan():
        return laserscan_one_instance_data


    def get_pointcloud(): 
        return pointcloud_one_instance_data

    def formatting_pointcloud(ros_cloud):
        return convertCloudFromRosToPoints(ros_cloud)

    def check_connection(port):
        connection_flag = os.path.exists(port)
        return connection_flag
    
class YDLIDAR_G2(YDLIAROperations): 
    YDLIDAR_TYPE = "G2"

    def connect(port): # port only exists when lidar is plugged in
        flag:bool = None
        flag = os.path.exists(port)
        if flag == True:
            logger.loginfo("Connected")
        elif flag == False: 
            logger.loginfo("problem connecting")
        return flag
    
    
    def get_laserscan():
        laser_scan_one_instance_data = rospy.wait_for_message('/scan',LaserScan,timeout=None)
        global laserscan_data_debugging
        laserscan_data_debugging = laser_scan_one_instance_data
        return laser_scan_one_instance_data
    
    
    def get_pointcloud(data):
        laserProj = LaserProjection()
        pointcloud_one_instance_data =laserProj.projectLaser(data)
        return pointcloud_one_instance_data
    
    
    def formatting_pointcloud(ros_cloud):

        #for formatting if required by frontend
        def format_data(data):
            pass

        def convertCloudFromRosToPoints(ros_cloud):

            # Get cloud data from ros_cloud
            field_names=[field.name for field in ros_cloud.fields]
            logger.loginfo(field_names)
            cloud_data = list(pc2.read_points(ros_cloud, skip_nans=True, field_names = field_names))
            logger.loginfo(cloud_data)

            # Check empty
            if len(cloud_data)==0:
                print("Converting an empty cloud")
                return None
            
            # Get xyz
            xyz = [[x,y,z] for x,y,z, _, _ in cloud_data ]
            intensity = [[intensity]for _,_,_, intensity,_ in cloud_data] 
            index = [[index]for _,_,_,_,index in cloud_data] 

            return json.dumps({ "coords": xyz, "intensity": intensity, "index": index})

        return convertCloudFromRosToPoints(ros_cloud)

    def check_connection(port):
        connection_flag = os.path.exists(port)
        return connection_flag
    
class YdlidarChecker:
    port = '/dev/ydlidar'
    NODE_RATE = 5.0
    ydliar_model = None
    _command = None
    _state = None

    YDLIAR_MODEL_TABLE = {
        YDLIDAR_G2.YDLIDAR_TYPE: YDLIDAR_G2,
    }

    def __init__(self) -> None:
        self.command = "NONE"
        self.cmd_params = ""
        self.state = YdlidarCheckerStates.INIT
        self.pub_state = LIDAR_STATE.Publisher()
        self.pub_info = LIDAR_INFO.Publisher()
        self.pub_info_chinese = LIDAR_INFO_CHINESE.Publisher()
        self.pub_reading_laserscan = LIDAR_DATA_LASERSCAN.Publisher()
        self.pub_reading_pointcloud = LIDAR_DATA_POINTCLOUD.Publisher()
        LIDAR_SRV_CMD.Services(self.srv_cb)
        self.subscriber_laserscan = rospy.Subscriber('/scan', LaserScan, self.get_laserscan_subscriber)

        self.__STATES_METHODS = {
        (YDLIDARCommands.NONE, YdlidarCheckerStates.INIT): self.initialize, # to IDLE
        (YDLIDARCommands.CONNECT, YdlidarCheckerStates.IDLE): self.connect, # to CONNECTED or stay
        (YDLIDARCommands.GET_POINTCLOUD, YdlidarCheckerStates.CONNECTED): self.get_pointcloud,  #get one instance of pointcloud msg
        (YDLIDARCommands.GET_LASERSCAN,YdlidarCheckerStates.CONNECTED): self.get_laserscan, #get one instance of laserscan msg
        (YDLIDARCommands.NONE,YdlidarCheckerStates.CONNECTED): self.check_connection, # check for unplug 
   } 
        
    def srv_cb(self,srv):
        self.command = srv.button
        self.cmd_params = srv.model
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
            self.log_with_frontend(f"Received wrong command: {value}", f"命令错误: {value}")
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
    

    def log_with_frontend(self, log,log_chinese):
        self.pub_info.publish(log)
        self.pub_info_chinese.publish(log_chinese)

    def _determine_ylidar_type(self): 
        self.ydliar_model = self.YDLIAR_MODEL_TABLE.get(self.cmd_params.upper())
        if self.ydliar_model:
            self.log_with_frontend(f"YDLIDAR model: {self.ydliar_model.YDLIDAR_TYPE}", f"YDLIDAR 类型: {self.ydliar_model.YDLIDAR_TYPE}")
            return True
        self.log_with_frontend(f"LIDAR model: {self.cmd_params} not supported", f"不支持 {self.cmd_params}")
        return False

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
        if self._determine_ylidar_type():
            connected = self.ydliar_model.connect(self.port)
            if connected:
                self.log_with_frontend("Connected to LIDAR", "LIDAR 连接成功")
                self.state = YdlidarCheckerStates.CONNECTED
                return True
        self.log_with_frontend(f"Failed to connect!", "无法连接")
        self.state = YdlidarCheckerStates.IDLE
        return False
    


    def get_laserscan(self):
        laserscan_one_message_data = self.ydliar_model.get_laserscan()
        self.log_with_frontend("one frame of laserscan data captured", "以获取激光扫描数据")
        self.pub_reading_laserscan.publish(laserscan_one_message_data)
        return True

    def get_pointcloud(self):
        laserscan_one_message_data = self.ydliar_model.get_laserscan()
        formatted_pointcloud = self.ydliar_model.get_pointcloud(laserscan_one_message_data)
        frontend_formatted_pointcloud = self.ydliar_model.formatting_pointcloud(formatted_pointcloud)
        logger.loginfo(frontend_formatted_pointcloud) 
        self.pub_reading_pointcloud.publish(frontend_formatted_pointcloud)
        return formatted_pointcloud
    

    def get_laserscan_subscriber(self,data):
        global laserScan_data_global
        laserScan_data_global = data
        return data
    

    def check_connection(self):
        check_connection_result = self.ydliar_model.check_connection(self.port)
        if check_connection_result == False: 
            self.log_with_frontend("Ydliar unplugged, check connection!", "无法连接lidar,请确保电源再连接")
            self.state = YdlidarCheckerStates.IDLE


if __name__ == "__main__":
    rospy.init_node("lidar_driver_node")
    lidar_driver_checker = YdlidarChecker()
    lidar_driver_checker.start()