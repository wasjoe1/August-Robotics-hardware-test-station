#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import socket
import json
import os
import yaml
import time
from boothbot_driver.lrf_drivers import LaserRangeFinderGenerator, LaserRangeFinderBase, LDMS60Driver


print("init servos")

class json_handler():
    def __init__(self):
        self.file_name = socket.gethostname() + "_" + str(time.strftime(
            "%Y-%m-%d-%H-%M-%S", time.localtime())) + ".json"
        with open(self.file_name, "w+") as file:
            file_data = {}
            file.seek(0)
            json.dump(file_data, file, indent=4)

    def write_json(self, id, save_data):

        file_data = {}
        if os.path.exists(self.file_name):
            with open(self.file_name, "r") as file:
                file_data = json.load(file)

        with open(self.file_name, "w+") as file:
            file_data[id] = save_data
            file.seek(0)
            json.dump(file_data, file, indent=4)

    def msg2json(self, msg):
        y = yaml.load(str(msg))
        return json.dumps(y, indent=4)

if __name__ == "__main__":
    print("start.")
    rospy.init_node("test_gs")
    print("init node done.")
    print("init laser")
    laser = LaserRangeFinderGenerator.detect_laser_range_finder()

    print("laser connnect")
    s_time = time.time()
    laser.connect()
    e_time = time.time()
    used_time = e_time - s_time
    print("used time {}".format(used_time))



    jh = json_handler()


    id = 1
    while not rospy.is_shutdown():

        print("laser on")
        # rospy.sleep(1)
        laser.laser_on()
        rospy.sleep(0.5)
        located, laser_distance, laser_angle = laser.get_distance_full()
        print("laser_distance: {}".format(laser_distance))
        jh.write_json(id, laser_distance)
        id += 1
        rospy.sleep(1)
        laser.reset()
