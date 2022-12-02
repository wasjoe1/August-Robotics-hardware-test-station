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

from boothbot_common.ros_logger_wrap import ROSLogging as Logging


class cali_cont(Logging):
    def __init__(self,name):
        super(cali_cont, self).__init__("cali_cont")        
        self.gshc = GuidingStationHubClient()

        self.gshc.connect()
        self.gshc.get_valid_gs(["gs_1"], 2)
        # self.gshc._using_new_gs
        # self.gshc.current_gs_serial_code

        self.state = 0
        self.detail_state = "INIT"
        self.dis = [14.9,50,9]
        self.hor = [-1.51396,0.03396,1.1]
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

        MODULES_GS_HUB_PDO.Subscriber(self._cb_gs)

    def _cb_gs(self, msg):
        # print(msg)
        self.num_total_gs = len(msg.gs_info)


    def run(self):
        if not self.gshc.is_done():
            return
        
        if self.task_done:
            # gss = self.gshc.get_valid_gs(["gs_1"], self.i)
            gss = self.gshc.get_valid_gs(["gs_1"], self.gid)
            if gss is None and self.adding is False and self.deling is False:
                self.loginfo(".................adding gs....")
                MODULES_GS_HUB_SRV_CMD.service_call("ags")
                self.adding = True
                return

            if self.adding is True:
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
                        self.task_done = False
                        self.i = 0
                        return
                    else:
                        self.loginfo("pose not changed.")

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
                self.gshc.targeting(self.gid,"BOG",self.dis[self.i],self.hor[self.i],0,True,False)
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
    # if task_done:
    #     print("task done.")
    #     gs_current_pose = gshc.get_current_gs_pose()
    #     if gs_last_pose != gs_current_pose:
    #         print("gs_last_pose: {}, gs_current_pose: {}".format(gs_last_pose, gs_current_pose))
    #         i = 0
    #         task_done = False
    #         gs_last_pose = gs_current_pose
    # else:
    #     if state == 0:
    #         print("Targeting")
    #         gs_valid = gshc.get_valid_gs(["gs_1"], gid)
    #         if gs_valid is None:
    #             gid += 1 
    #             if gid >= 255:
    #                 gid = 1
    #         else:
    #             print("current rb id {}".format(i))
    #             gshc.targeting(1,"ROG",dis[i],hor[i],0,True,False)
    #             state = 1
    #     elif state == 1:
    #         if not gshc.is_done():
    #             print("not done")
    #         elif not gshc.is_failed() and not gshc.is_succeeded():
    #             print("not any task, set task")
    #             detail_state = "TASKING"
    #         elif gshc.is_succeeded():
    #             print("measure succeeded {}".format(gshc.get_measurement()))
    #             i += 1
    #             # gshc.reset()
    #             state = 0
    #         elif gshc.is_failed():
    #             print("fail {}".format(gshc.is_failed()))
    #             i += 1
    #             # gshc.reset()
    #             state = 0
    #         else:
    #             print("dont know..")
    #     if i >= 3:
    #         task_done = True

    rate.sleep()

