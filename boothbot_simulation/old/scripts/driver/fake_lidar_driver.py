#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import math
from sensor_msgs.msg import LaserScan
from std_srvs.srv import Empty
from common import Logging

# Fake lidar driver based on rplidarNode (node.cpp)
class FakeLidarDriver(Logging):
    DEFAULT_NODE_NAME = 'rplidarNode'
    DEFAULT_MIN_DISTANCE = 0.15
    DEFAULT_MAX_DISTANCE = 8.0
    DEFAULT_ANGLE_COMPENSATE_MULTIPLE = 1
    DEFAULT_RATE = 1.0 

    def __init__(self):
        rospy.init_node(self.DEFAULT_NODE_NAME, anonymous=False)
        super(FakeLidarDriver, self).__init__(self.DEFAULT_NODE_NAME)

        self.loginfo('Initialising...')

        self.serial_port = rospy.get_param('~serial_port', '/dev/lidar')
        self.serial_baudrate = int(rospy.get_param('~serial_baudrate', 256000))
        self.frame_id = rospy.get_param('~frame_id', 'laser_frame')
        self.inverted = bool(rospy.get_param('~inverted', False))
        self.angle_compensate = bool(rospy.get_param('~angle_compensate', True))
        self.scan_mode = rospy.get_param('~scan_mode', 'Stability')

        self.max_distance = float(rospy.get_param('~max_distance', self.DEFAULT_MAX_DISTANCE))
        self.min_distance = self.DEFAULT_MIN_DISTANCE

        self.rate = float(rospy.get_param('~rate', self.DEFAULT_RATE))

        self.scan_pub = rospy.Publisher('scan', LaserScan, queue_size=1)
        self.stop_motor_srv = rospy.Service(self.DEFAULT_NODE_NAME + '/stop_motor', Empty, self.stop_motor)
        self.start_motor_srv = rospy.Service(self.DEFAULT_NODE_NAME + '/start_motor', Empty, self.start_motor)

        self.scan_count = 0
        self.angle_compensate_multiple = self.DEFAULT_ANGLE_COMPENSATE_MULTIPLE

        self.loginfo('Finished initialising.')

    def publish_scan(self, node_ranges, start_scan_time, scan_duration, angle_min, angle_max):
        scan_msg = LaserScan()
        scan_msg.header.stamp = start_scan_time
        scan_msg.header.frame_id = self.frame_id

        self.scan_count += 1
        node_count = len(node_ranges)

        reversed = angle_max > angle_min
        scan_msg.angle_min = math.pi - (angle_max if reversed else angle_min)
        scan_msg.angle_max = math.pi - (angle_min if reversed else angle_max)

        scan_msg.angle_increment = (scan_msg.angle_max - scan_msg.angle_min) / (node_count-1)

        scan_msg.scan_time = scan_duration
        scan_msg.time_increment = scan_duration / (node_count-1)
        scan_msg.range_min = self.min_distance
        scan_msg.range_max = self.max_distance

        scan_msg.intensities = []
        scan_msg.ranges = node_ranges
        
        reverse_data = (reversed and not self.inverted) or (self.inverted and not reversed)
        if reverse_data:
            scan_msg.ranges.reverse()

        self.scan_pub.publish(scan_msg)

    def stop_motor(self, request):
        self.logerr('Stop motor not yet implemented!')
        raise NotImplementedError

    def start_motor(self, request):
        self.logerr('Start motor not yet implemented!')
        raise NotImplementedError
    
    def run(self):
        self.loginfo('Running...')

        pacemaker = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            start_scan_time = rospy.Time.now()
            end_scan_time = rospy.Time.now()
            scan_duration = (end_scan_time - start_scan_time).to_sec()

            angle_min = 0.0
            angle_max = 359.0 / 180.0 * math.pi

            node_count = 360 * 8
            node_ranges = [self.max_distance for i in range(node_count)]

            self.publish_scan(node_ranges, start_scan_time, scan_duration, angle_min, angle_max)

            pacemaker.sleep()
        
        self.loginfo('Shutting down.')

if __name__ == '__main__':
    lidar = FakeLidarDriver()
    lidar.run()
