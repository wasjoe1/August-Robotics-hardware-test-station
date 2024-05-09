#!/usr/bin/env python3
# -*- coding: utf-8 -*-


"""

INSTRUCTIONS: 
1. connect to ylidar
2. lidar scan
3. get pointcloud

"""

from sensor_msgs.msg import LaserScan, PointCloud2
import os
import rospy
logger = rospy
from enum import Enum,auto

import json

