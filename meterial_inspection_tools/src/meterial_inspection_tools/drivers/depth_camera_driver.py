#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# from sensor_msgs.msg import Image
#import message_filters
#import cv2
#from cv_bridge import CvBridge
#from tf2_ros import TransformListener,Buffer
#import math
#import tf2_ros
#from geometry_msgs.msg import PointStamped
#from tf2_geometry_msgs import do_transform_point

import os
import rospy
logger = rospy
from enum import Enum,auto

import numpy as np
from ctypes import * # convert float to uint32
from sensor_msgs.msg import PointCloud2
import json
import sensor_msgs.point_cloud2 as pc2


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

convert_rgbUint32_to_tuple = lambda rgb_uint32: (
    (rgb_uint32 & 0x00ff0000)>>16, (rgb_uint32 & 0x0000ff00)>>8, (rgb_uint32 & 0x000000ff) # each rgb number is represented by 8 bits (2 x 4 bits)
)

convert_rgbFloat_to_tuple = lambda rgb_float: convert_rgbUint32_to_tuple(
    int(cast(pointer(c_float(rgb_float)), POINTER(c_uint32)).contents.value) # TODO: not exactly sure
)

def convertCloudFromRosToPoints(ros_cloud):
    
    # Get cloud data from ros_cloud
    field_names=[field.name for field in ros_cloud.fields]
    cloud_data = list(pc2.read_points(ros_cloud, skip_nans=True, field_names = field_names))

    # Check empty
    # open3d_cloud = open3d.PointCloud()
    if len(cloud_data)==0:
        print("Converting an empty cloud")
        return None

    # Set open3d_cloud
    if "rgb" in field_names:
        IDX_RGB_IN_FIELD=3 # x, y, z, rgb
        
        # Get xyz
        xyz = [[x,y,z] for x,y,z,rgb in cloud_data ] # [(x,y,z), (x,y,z), (x,y,z) ...]

        # Get rgb
        # Check whether int or float
        if type(cloud_data[0][IDX_RGB_IN_FIELD])==float: # if float (from pcl::toROSMsg)
            rgb = [convert_rgbFloat_to_tuple(rgb) for x,y,z,rgb in cloud_data ]
        else:
            rgb = [convert_rgbUint32_to_tuple(rgb) for x,y,z,rgb in cloud_data ]
        
        rgb = [list(ele) for ele in rgb]
        # combine
        # open3d_cloud.points = open3d.Vector3dVector(np.array(xyz))
        # open3d_cloud.colors = open3d.Vector3dVector(np.array(rgb)/255.0)
    else:
        xyz = [[x,y,z] for x,y,z in cloud_data ] # get xyz
        # np.array(xyz) # [[x,y,z], [x,y,z], [x,y,z] ...]
        # open3d_cloud.points = open3d.Vector3dVector(np.array(xyz))

    # return
    # xyz = np.array(xyz)
    # rgb = np.array(rgb) # array dont work, they just make the string have an extra array string
    dataToReturn = { "coords": xyz, "colors": rgb } # TEST check if this still adds \ to " => yes it still does
    # dataToReturn = { "coords": "xyz", "colors": "rgb" }
    # return str(dataToReturn)
    return json.dumps(dataToReturn)


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
    
    def get_data_subscriber(self, data):
        global data_global
        data_global = data
        # field_names=[field.name for field in data.fields]
        # data_global = list(pc2.read_points(data, skip_nans=True, field_names = field_names))
        return data
    
    def parse_reading(self):
        #logger.loginfo(data_global)
        # logger.loginfo(type(data_global))
        # logger.loginfo(convertCloudFromRosToPoints(data_global))
        self.pub_reading.publish(convertCloudFromRosToPoints(data_global))
        return True
        
if __name__ == "__main__":
    rospy.init_node("depth_camera_driver_node")
    depth_driver_checker = DepthChecker()
    depth_driver_checker.start()