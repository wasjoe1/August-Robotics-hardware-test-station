#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Instructions: 
These are the steps to take 
1. Connect to astra(check if camera is plugged in)
3. get image --> grabs the pointcloud and then formats it automatically --> desired endgoal
"""

import datetime
from sensor_msgs.msg import Image
import rospy
import open3d as o3d
import numpy as np
from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import PointCloud2, PointField #defines pointcloud2 message type 
import sensor_msgs.point_cloud2 as pc2 # prodvides functions and utilities for working with pointcloud 
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
    DEPTH_STATE,
    DEPTH_IMAGE,
)
from std_srvs.srv import Empty

class DEPTHCommands(Enum):
    NONE = auto()
    RESET = auto()
    CONNECT = auto()
    DISCONNECT = auto()
    GET_POINTCLOUD = auto()
    GET_IMAGE = auto()



class DepthCheckerStates(Enum): 
    INIT = auto()
    IDLE = auto()
    SCANNING = auto()
    CONNECTING = auto()
    CONNECTED = auto()
    ERROR = auto()

class ASTRA_CAMERA():
    # DATA publish value is in pointcloud2 format
    """
    def retrieve_latest_pointcloud_ply(directory):
        last_modified_time = 0
        last_modified_file = None
        # Recursively traverse the directory tree
        for root, _, files in os.walk(directory):
            for file in files:
                # Get the full path of the file
                file_path = os.path.join(root, file)
                # Get the modification time of the file
                modified_time = os.path.getmtime(file_path)
                # Check if the current file has a more recent modification time
                if modified_time > last_modified_time:
                    last_modified_time = modified_time
                    last_modified_file = file_path
                    
        return last_modified_file
    """

    def formatPointCloud(point_cloud):
        # Convert Open3D PointCloud to NumPy array
        points_array = np.asarray(point_cloud.points)
        colors_array = np.asarray(point_cloud.colors)

        # Create a sensor_msgs/PointCloud2 message
        msg = PointCloud2()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "your_frame_id"  # Replace with your desired frame ID
        msg.height = 1
        msg.width = len(points_array)
        msg.fields.append(PointField(
            name="x",
            offset=0,
            datatype=PointField.FLOAT32,
            count=1
        ))
        msg.fields.append(PointField(
            name="y",
            offset=4,
            datatype=PointField.FLOAT32,
            count=1
        ))
        msg.fields.append(PointField(
            name="z",
            offset=8,
            datatype=PointField.FLOAT32,
            count=1
        ))
        msg.fields.append(PointField(
            name="rgb",
            offset=12,
            datatype=PointField.FLOAT32,
            count=1
        ))
        msg.is_bigendian = False
        msg.point_step = 16  # 4 bytes per float (x, y, z, rgb)
        msg.row_step = msg.point_step * msg.width
        msg.is_dense = True
        msg.data = np.column_stack((points_array, colors_array)).astype(np.float32).tobytes()
        logger.loginfo(msg)
        return msg

    #def pointcloud_to_2D(file):
        #point_cloud = o3d.io.read_point_cloud(file)
    def pointcloud_to_2D():
        point_cloud = o3d.io.read_point_cloud('.ros/point_cloud/points_xyz_rgb_20240430_075055.ply') # include path & file name here
        points_x_frame = []
        points_y_frame = [] 
        points_z_frame = [] 
        colour_frame = []
        #project points onto 2D planes:
        for colour in point_cloud.colors:
            converting_to_rgb = colour*255
            colour_frame.append(converting_to_rgb)

        for point in point_cloud.points:
            x,y,z = point
            x = point[0]
            y = point[1]
            z = point[2]            

            # for x plane image
            points_x_frame.append((0,y,z))         
            # for y plane image
            points_y_frame.append((x,0,z)) 
            #for z plane image
            points_z_frame.append((x,y,0)) 

        colour_frame_np = np.array(colour_frame)
        points_x_frame_np = np.array(points_x_frame)
        points_y_frame_np = np.array(points_y_frame)
        points_z_frame_np = np.array(points_z_frame)
        #logger.loginfo(points_x_frame_np)
       
        #logger.loginfo(type(points_x_frame_np))
        #logger.loginfo(points_y_frame_np)
        
        point_cloud_x = o3d.geometry.PointCloud()
        point_cloud_y = o3d.geometry.PointCloud()
        point_cloud_z = o3d.geometry.PointCloud()
        point_cloud_x.points = o3d.utility.Vector3dVector(points_x_frame_np) # expects tuple
        point_cloud_y.points = o3d.utility.Vector3dVector(points_y_frame_np)
        point_cloud_z.points = o3d.utility.Vector3dVector(points_z_frame_np)
        point_cloud_x.colors = o3d.utility.Vector3dVector(colour_frame_np)
        point_cloud_y.colors = o3d.utility.Vector3dVector(colour_frame_np)
        point_cloud_z.colors = o3d.utility.Vector3dVector(colour_frame_np)

        logger.loginfo(point_cloud_x)
        #logger.loginfo(type(point_cloud_x))
        return point_cloud_x,point_cloud_y,point_cloud_z

    def get_pointcloud():
        succeeded = False
        rospy.wait_for_service('/camera/save_point_cloud_xyz_rgb')
        try:
            save_pointcloud = rospy.ServiceProxy('/camera/save_point_cloud_xyz_rgb', Empty)
            response = save_pointcloud()
            return response
        except rospy.ServiceException as e:
            logger.loginfo("error in service",e)
        succeeded = True
        return succeeded
    
    def connect(port): # port only exists when depth cam is plugged in
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
        #self.pub_image = DEPTH_IMAGE.Publisher()
        DEPTH_SRV_CMD.Services(self.srv_cb)
        self.subscriber = rospy.Subscriber('/camera/depth_registered/points', PointCloud2, self.get_data_subscriber)
        self.__STATES_METHODS = {
        (DEPTHCommands.NONE, DepthCheckerStates.INIT): self.initialize, # to IDLE
        (DEPTHCommands.CONNECT, DepthCheckerStates.IDLE): self.connect, # to CONNECTED or stay
        (DEPTHCommands.DISCONNECT, DepthCheckerStates.CONNECTED): self.disconnect, # to IDLE
        (DEPTHCommands.GET_POINTCLOUD, DepthCheckerStates.CONNECTED): self.get_pointcloud_ply, # get image and send 3 images
        (DEPTHCommands.GET_IMAGE,DepthCheckerStates.CONNECTED): self.get_image,
    }   
        
    
    def srv_cb(self, srv):
        self.command = srv.button
        return True

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

    def get_image(self):
        #pointcloud_directory = os.path.join(os.path.expanduser("~"), "point_cloud")
        #last_modified_file = self.depthcamera_model.retrieve_latest_pointcloud_ply(pointcloud_directory)
        #point_cloud_image,filename = self.depthcamera_model.get_pointcloud()
        #point_cloud_image_to_2D = self.depthcamera_model.pointcloud_to_2D(last_modified_file)
        point_cloud_x,point_cloud_y,point_cloud_z = self.depthcamera_model.pointcloud_to_2D()
        msg_x = self.depthcamera_model.formatPointCloud(point_cloud_x)
        msg_y = self.depthcamera_model.formatPointCloud(point_cloud_y)
        msg_z = self.depthcamera_model.formatPointCloud(point_cloud_z)
        #o3d.visualization.draw_geometries([point_cloud_image_to_2D])
        self.pub_reading.publish(msg_x)
        rospy.sleep(0.5)
        self.pub_reading.publish(msg_y)
        rospy.sleep(0.5)
        self.pub_reading.publish(msg_z)
        #return point_cloud_image_to_2D

    def get_pointcloud_ply(self): 
        point_cloud_image = self.depthcamera_model.get_pointcloud()
        logger.loginfo("This is the point_cloud_image under get_pointcloud_ply")
        logger.loginfo(point_cloud_image)
        logger.loginfo(type(point_cloud_image))
        return point_cloud_image

    def disconnect(self):
        self.log_with_frontend("DISCONNECTING")
        self.log_with_frontend("DISCONNECTED")
        self.state = DepthCheckerStates.IDLE
        return True
    
    def get_data_subscriber(self,data):
        global data_global
        data_global = data
        return data

"""
    def parse_reading(self):
        self.pub_reading.publish(xyz_plane_images)
        logger.loginfo(xyz_plane_images)
        return True
"""
        
if __name__ == "__main__":
    rospy.init_node("depth_camera_driver_node")
    depth_driver_checker = DepthChecker()
    depth_driver_checker.start()