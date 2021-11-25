#!/usr/bin/env python3
"""
Test script for interacting with message buffer
"""

import sys, os, time
import rospy

def anymsg_cb(msg):
    print(msg._buff)

if __name__ == "__main__":
    rospy.init_node("any_sub", anonymous=True)
    rospy.Subscriber("/test", rospy.AnyMsg, anymsg_cb)
    rospy.spin()
    # In another terminal, try:
    # `rostopic pub -r 1 /battery std_msgs/UInt8 "data: 255"``
    # And the result should be b'\xff'

