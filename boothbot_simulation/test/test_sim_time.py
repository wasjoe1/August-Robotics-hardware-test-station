#!/usr/bin/python3

import rospy
from collections import deque
from timeit import default_timer as timer
import boothbot_common.ros_logger as logger
import time
from rosgraph_msgs.msg import Clock


class TestSimTime():

    def __init__(self):
        rospy.init_node("test_sim_clock")
        self.sub = rospy.Subscriber("/clock", Clock , callback = self.get_sim_time)

    def get_sim_time(self, msg):
        self.sim_time = msg


    def sim_time_delay(self):
        start_time = time.time()
        rospy.sleep(2)
        end_time = time.time()
        logger.logwarn("sim_time_delay:{}".format(end_time-start_time))

    def wall_time_delay(self):
        start_time = time.time()
        time.sleep(2)
        end_time = time.time()
        logger.logwarn("wall_time_delay:{}".format(end_time-start_time))

if __name__=="__main__":
    tst = TestSimTime()
    while not rospy.is_shutdown():
        tst.sim_time_delay()
        time.sleep(1)
        tst.wall_time_delay()
        time.sleep(1)