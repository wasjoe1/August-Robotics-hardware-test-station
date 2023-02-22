#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import rospy
# from boothbot_driver.servos_client import ServosClient
from gbs_measurement.gs_measurement_module_client import GuidingStationMeasurementClient
from guiding_beacon_system.guiding_station_calibration_app_client import GuidingStationCalibrationAppClient
from boothbot_common.ros_logger_wrap import ROSLogging
from std_msgs.msg import String
from boothbot_msgs.ros_interfaces import (
    DRIVERS_SERVOS_PDO,
    DRIVERS_SERVOS_SRV_CMD
)
from guiding_beacon_system_msgs.ros_interfaces import (
    MODULES_MEASUREMENT_SRV_CMD
)

INIT = "init"
COLLECT = "collect"
CALI = "cali"
DONE = "done"

COLOR = "BOG"

NUM_RBS = 4

NUM_DIRS = 8

class Collect8Dir(ROSLogging):
    def __init__(self, name):
        super(Collect8Dir, self).__init__(name)
        self.state = INIT
        self.m_raw_data = {}
        self.rb_pose = {}
        self.current_radians = []
        self.current_servos_states = []
        self.next_m = None
        self.current_m = None
        self.current_c = None
        self.next_c = None
        self.num_rbs = NUM_RBS
        self.cali_count = 0
        self.reset_flag = False
        self.one_dir_done = False
        self.current_dir = 0

        # ros
        rospy.Subscriber("~cmd", String, self.cmd_cb)
        self.gsmc = GuidingStationMeasurementClient()
        self.gscc = GuidingStationCalibrationAppClient(False)
        DRIVERS_SERVOS_PDO.Subscriber(self.servos_pdo_cb)

    def cmd_cb(self, msg):
        self.loginfo("Got msg {}".format(msg.data))
        cmd = msg.data
        if cmd.startswith("m"):
            m_id = int(cmd.split("_")[1])
            if m_id <0 or (m_id >self.num_rbs-1):
                self.loginfo("Got invalid measure id.. {}".format(m_id))
                return
            self.next_m = m_id
            self.loginfo("set next_m as {}".format(self.next_m))
        # elif cmd.startswith("s"):
        #     self.next_c = 0
        #     self.loginfo("run now...")
        elif cmd.startswith("reset"):
            self.reset_flag = True

        

    def init(self):
        self.loginfo("init.....")
        if self.gsmc.connect() and self.gscc.connect():
            self.loginfo("connect done... resetting.....")
            self.gsmc.reset()
            self.gscc.reset()
            self.loginfo("resetting......")
            return True
        return False

    def collect(self):
        if len(self.m_raw_data) == self.num_rbs:
            self.disable_state = False
            self.loginfo("all measurements done... waiting to start calibration..")
            self.calc_poses()
            self.next_c = 0
            return True

        if not self.gsmc.is_done():
            self.logdebug_throttle(5, "current m_data {}".format(self.m_raw_data))
            return

        [hor, ver] = self.current_radians
        # [hor_state, ver_state] = self.current_servos_states
        # ver = 0.0

        if self.next_m is not None: 
            self.loginfo("enabling servos..")
            if self.current_servos_states == ["DISABLED", "DISABLED"]:
                rospy.sleep(1)
                # DRIVERS_SERVOS_SRV_CMD.service_call("ENABLE")
                self.gsmc.reset()
                rospy.sleep(1)
            self.loginfo("enable servo done...")
            self.loginfo("set {} {}".format(hor, ver))
            self.current_m = self.next_m
            self.next_m = None
            self.gsmc.measure(COLOR, 0.0, hor, ver, True)
            return

        # if GS has job and measure done.
        if self.current_m is not None:
            if self.gsmc.is_succeeded():
                self.loginfo("measure succeeded.")
                measurement_res = self.gsmc.get_result()
                self.loginfo("measument data {} {} {}".format(measurement_res.distance, measurement_res.rad_hor, measurement_res.rad_ver))
                self.m_raw_data[self.current_m] = [measurement_res.distance, measurement_res.rad_hor, measurement_res.rad_ver]
                self.current_m = None
                self.loginfo("current data {}".format(self.m_raw_data))
            else:
                self.loginfo("measure failed.")
                self.current_m = None
                self.gsmc.reset()
        else:
            self.loginfo_throttle(2,"not measurement now... laser on..")
            MODULES_MEASUREMENT_SRV_CMD.service_call("LASER_ON")
            # self.loginfo("servos state {}".format(self.current_servos_states))
            if self.current_servos_states != ["DISABLED", "DISABLED"]:
                self.loginfo("disable servos now.")
                DRIVERS_SERVOS_SRV_CMD.service_call("DISABLE")
                rospy.sleep(1)
            # MODULES_MEASUREMENT_SRV_CMD.service_call("DISABLE")
            # self.gsmc.set_command("DISABLE")
            return

        # TODO
    
    def calc_poses(self):
        offset = 0.05
        for k, v in self.m_raw_data.items():
            laser_dis = v[0]+ offset
            hor = v[1]
            ver = v[2]
            self.rb_pose[k] = (laser_dis*math.cos(hor)*math.cos(ver), laser_dis*math.sin(hor)*math.cos(ver))
        self.loginfo("calc rbs poses {}".format(self.rb_pose))
    
    def cali(self):
        if not self.gscc.is_done():
            self.loginfo_throttle(2, "cali app is not done..")
        # pass
        # if self.next_c is None:
        #     self.loginfo("waiting user to start...")
        #     return

        if self.next_c is not None:
            (rb1, rb2) = self.get_rb_ids(self.next_c)
            self.loginfo("next_c {}".format(self.next_c))
            self.loginfo("set cali rb ids {}, {}".format(rb1, rb2))
            self.gscc.calibrate(
                (0.0, 0.0, 0.0),
                0.03,
                COLOR,
                (self.rb_pose[rb1]),
                COLOR,
                (self.rb_pose[rb2])
            )
            self.current_c = self.next_c
            self.next_c = None
            return

        if self.current_c is not None:
            if self.gscc.is_done():
                #TODO
                if self.gscc.is_succeeded():
                    self.loginfo("calibration done... result {}".format(self.gscc.get_result()))
                    # self.current_c += 1
                    self.loginfo("calibration done..")
                    self.cali_count +=1
                    self.next_c = self.current_c
                elif self.gscc.is_failed():
                    self.loginfo("calibration failed....set next_c")
                    self.next_c = self.current_c
            else:
                self.loginfo_throttle(2,"waitting calibration done..")

        if self.cali_count == 2:
            self.loginfo("all done ... calibration count {}".format(self.cali_count))
            self.next_c = self.current_c + 1
            self.current_c = None
            # self.current_c = None
            self.loginfo("set next cali id {},clear cali count..".format(self.next_c))
            self.cali_count = 0
            # self.current_c = 


        if self.next_c == 4:
            self.loginfo("do all calibration done...")
            self.current_dir += 1
            if self.current_dir == 8:
                self.loginfo("{} dirs cali done.. script exit..".format(self.current_dir))
                exit(0)
                # self.one_dir_done = False
            else:
                self.one_dir_done = True
            return True

        return False


    def get_rb_ids(self, i):
        if i >= self.num_rbs - 1:
            return (i, 0)
        return (i, i+1)

    def reset_handle(self):
        if self.one_dir_done:
            self.loginfo("one dir cali done..reset....")
            self.reset()
            self.one_dir_done = False
            return
        if self.reset_flag:
            self.reset()
            self.reset_flag = False
        
    def reset(self):
        self.gscc.reset()
        self.gsmc.reset()
        self.state = INIT
        self.m_raw_data = {}
        self.rb_pose = {}
        self.current_radians = []
        self.current_servos_states = []
        self.next_m = None
        self.current_m = None
        self.current_c = None
        self.next_c = None
        self.next_c = None
        self.num_rbs = NUM_RBS
        self.cali_count = 0
        self.one_dir_done = False

    def run(self):
        self.reset_handle()
        if self.state == INIT:
            if self.init():
                self.state = COLLECT
        elif self.state == COLLECT:
            if self.collect():
                self.state = CALI
        elif self.state == CALI:
            if self.cali():
                self.state = DONE
        elif self.state == DONE:
            self.loginfo("calibration done..")
        

    def servos_pdo_cb(self, msg):
        self.current_radians = msg.radians
        self.current_servos_states = msg.driver_states




if __name__ == "__main__":
    rospy.init_node("8_dirs_collect")
    c8d = Collect8Dir("8_dirs_collect")
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        c8d.run()
        rate.sleep()
