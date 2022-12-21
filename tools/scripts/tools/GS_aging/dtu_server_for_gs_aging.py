#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from boothbot_portal.gs_hub_client import GuidingStationHubClient
import guiding_beacon_system_msgs.msg as gbmsgs
from boothbot_msgs.ros_interfaces import (
    # MODULES_GS_HUB_STATUS,
    MODULES_GS_HUB_SRV_CMD,
    MODULES_GS_HUB_PDO
)
from boothbot_msgs.msg import std_msgs

from boothbot_common.ros_logger_wrap import ROSLogging as Logging

COLOR = "BOG"

class cali_cont(Logging):
    def __init__(self, name):
        super(cali_cont, self).__init__("cali_cont")        
        self.gshc = GuidingStationHubClient()

        self.gshc.connect()
        self.gshc.get_valid_gs(["gs_1"], 2)
        # self.gshc._using_new_gs
        # self.gshc.current_gs_serial_code

        self.state = 0
        self.detail_state = "INIT"
        self.dis = [14.9,50,9]
        self.hor = [-1.495,0.03,1.1]
        self.gid = 2
        self.gshc.cancel_goal()
        self.gshc.reset()
        self.i = 0
        self.gs_current_pose = self.gshc.get_current_gs_pose()
        self.gs_last_pose = None
        self.task_done = False
        self.adding = False
        self.deling = False
        self.gid = 1
        self.num_total_gs = 0
        self.adding_fail_count = 0

        MODULES_GS_HUB_PDO.Subscriber(self._cb_gs)
        rospy.Subscriber("set_rb", std_msgs.msg.String, self._cb_set_rb)

    def _cb_gs(self, msg):
        # print(msg)
        self.num_total_gs = len(msg.gs_info)

    def _cb_set_rb(self, msg):
        self.loginfo("Got msg {}..".format(msg))
        data = msg.split("_")
        rb_id = int(data[0])
        self.hor[rb_id] = float(data[1])

    def run(self):
        if not self.gshc.is_done():
            return
        
        ### 
        # When last measurement task done or start a new aging test, DTU aging server will go into add/delete loop.
        # After calibration done, GS would update pose via DUT.
        # If the server found the GS pose changed, the server will start a new task.

        if self.task_done:
            # gss = self.gshc.get_valid_gs(["gs_1"], self.i)
            gss = self.gshc.get_valid_gs(["gs_1"], self.gid)
            if gss is None and self.adding is False and self.deling is False:
                self.logwarn(".................adding gs....")
                MODULES_GS_HUB_SRV_CMD.service_call("ags")
                self.adding_fail_count = 0
                self.adding = True
                return

            if self.adding is True:
                self.adding_fail_count += 1
                if gss is not None:
                    self.loginfo("set adding to {}".format(True))
                    self.adding = False

                    self.gs_current_pose = self.gshc.get_current_gs_pose()
                    self.loginfo("got gs pose {}".format(self.gs_current_pose))
                    self.loginfo("last pose is {}, set last pose {}".format(self.gs_last_pose, self.gs_current_pose))
                    if self.gs_current_pose ==  (0.0, 0.0, 0.0):
                        self.loginfo_throttle(3, "Got error pose {}".format(self.gs_current_pose))
                        return
                    if self.gs_last_pose != self.gs_current_pose:
                        self.gs_last_pose = self.gs_current_pose
                        self.logwarn("add gs done, set new task")
                        self.adding_fail_count = 0
                        self.task_done = False
                        self.i = 0
                        return
                    else:
                        self.logwarn("pose not changed.")
                if self.adding_fail_count >= 200:
                    self.logerr("Add gs  timeout. should retry...")
                    self.adding = False
                    self.adding_fail_count = 0


            if gss is not None and self.deling is False and self.adding is False:
                self.loginfo(gss)
                self.loginfo(".................delete gs....")
                MODULES_GS_HUB_SRV_CMD.service_call("dgs")
                self.deling = True
                return

            if self.deling is True:
                if gss is None:
                    self.loginfo("set deling to {}".format(False))
                    self.deling = False

        else:
            ###
            # The server start the measurement task, it would measure 3 RBs

            if self.state == 0:
                self.loginfo("Targeting")
                gss = self.gshc.get_valid_gs(["gs_1"], self.gid)
                if gss is None:
                    self.loginfo("not valid gs.")
                    if self.num_total_gs == 0:
                        self.loginfo("no gs found. add gs...")
                        self.task_done = True
                    self.gid += 1
                    return
                self.gshc.targeting(self.gid,COLOR,self.dis[self.i],self.hor[self.i],0,True,False)
                self.state = 1
            elif self.state == 1:
                if not self.gshc.is_failed() and not self.gshc.is_succeeded():
                    self.loginfo("not any task, set task")
                    self.detail_state = "TASKING"
                elif self.gshc.is_succeeded():
                    self.loginfo("measure succeeded {}".format(self.gshc.get_measurement()))
                    self.i += 1
                    self.gid += 1
                    # self.gshc.reset()
                    self.state = 0
                elif self.gshc.is_failed():
                    self.loginfo("fail {}".format(self.gshc.is_failed()))
                    self.i += 1
                    self.gid += 1
                    # self.gshc.reset()
                    self.state = 0
                else:
                    self.loginfo("dont know..")
            if self.i >= 3:
                self.task_done = True

            if self.gid >= 255:
                self.gid = 1

        # if self.i>= 255:
        #     self.i = 1
        # else:
        #     self.i += 1
rospy.init_node("test_gs_hub_client")

rate = rospy.Rate(0.5)
cc = cali_cont("cali_cont")

while not rospy.is_shutdown():

    cc.run()

    rate.sleep()

