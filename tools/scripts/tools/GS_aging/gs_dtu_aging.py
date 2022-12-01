#!/usr/bin/python
# -*- coding: utf-8 -*-


import os
import rospy
import json
import time
import yaml
import datetime
import socket
# logger = rospy
from rospy_message_converter import json_message_converter

# from boothbot_common import ros_logger_wrap as

from boothbot_common.ros_logger_wrap import ROSLogging as Logging
from guiding_beacon_system.guiding_station_measurement_module_client import GuidingStationMeasurementClient
# from gbs_measurement.gs_measurement_module_client import GuidingStationMeasurementClient
import guiding_beacon_system_msgs.msg as gbmsgs

from guiding_beacon_system_msgs.ros_interfaces import (
    GS_0_CMDWORD,
    GS_0_FEEDBACK,
    GS_0_GOAL,
    GS_0_STATE,
    GS_0_SET_POSE,
    GS_0_UPDATE_POSE,
    MODULES_MEASUREMENT_ACT_MEASURE
)

from guiding_beacon_system_msgs.msg import (GBMeasureGoal,
GBMeasureResult,
GBMeasureActionGoal,
GBMeasureActionResult
)

from threading import Lock

from guiding_beacon_system.guiding_station_calibration_app_client import GuidingStationCalibrationAppClient

COLOR = "BOG"
GUESS_POSE = (-0.2303,	-0.4552,	-1.549097)


# TEST_GOAL_0 = {"x": 2.901149754, "y": 22.77927683}
# TEST_GOAL_1 = {"x": 12.13942357, "y": -1.18506726}
# TEST_GOAL_2 = {"x": -3.416903625, "y": -0.138446967}

# TEST_GOAL_0 = {"x": -15.54288662, "y": -1.659957859} #15
# TEST_GOAL_1 = {"x": 2.445583913, "y": -48.48100228} #48
# TEST_GOAL_2 = {"x": 8.566780444, "y": -3.244379861} #9

TEST_GOAL_0 = {"x": -15.54288662, "y": -1.659951443} # 15
TEST_GOAL_1 = {"x": 2.445430569, "y": -48.48078833} # 48
TEST_GOAL_2 = {"x": 8.566331757, "y": -3.244249486} # 9



goal_list = []
goal_list.append(TEST_GOAL_0)
goal_list.append(TEST_GOAL_1)
goal_list.append(TEST_GOAL_2)

M_TEST_GOAL_0 = gbmsgs.GBMeasureGoal(
    gid=1,
    color=COLOR,
    distance=14.9,
    rad_hor=-1.59,
    rad_ver=-0.05,
    is_initialpose=False,
    unique_id=1,
)

M_TEST_GOAL_1 = gbmsgs.GBMeasureGoal(
    gid=1,
    color=COLOR,
    distance=48.,
    rad_hor=-0.06,
    rad_ver=0.005,
    is_initialpose=False,
    unique_id=1,
)

M_TEST_GOAL_2 = gbmsgs.GBMeasureGoal(
    gid=1,
    color=COLOR,
    distance=9,
    rad_hor=1.103,
    rad_ver=0.05,
    is_initialpose=False,
    unique_id=1,
)

measurement_goal_list = []
measurement_goal_list.append(M_TEST_GOAL_0)
measurement_goal_list.append(M_TEST_GOAL_1)
measurement_goal_list.append(M_TEST_GOAL_2)

INIT = "init"
RUNNING = "running"
CALIBRATING = "calibrating"
MEASURING = "measuring"

goal = None
goal_id = None
result = None
# temp_lock = Lock()

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


class LOGMGS(Logging):
    def __init__(self, name):
        super(LOGMGS, self).__init__(name)

def get_goal(msg):
    # global temp_lock
    global goal
    global goal_id
    # with temp_lock:
    if msg.goal.is_initialpose is True:
        pass
    else:
        goal = msg
        goal_id = goal.goal_id.id
        # print(goal)

def get_result(msg):
    # global temp_lock
    global result
    # global goal_id
    # global goal
    # with temp_lock:

        # if msg.status.goal_id.id == goal_id:
    if msg.status.goal_id.id.startswith("/guiding_station_portal"):
        result = msg
        print("Got topic")
        print(result)

if __name__ == "__main__":
    rospy.init_node("gs_measurement_test")
    gsc = GuidingStationMeasurementClient()

    hz = 5
    rate = rospy.Rate(hz)
    gid = 1

    lm = LOGMGS("GS")

    jh = json_handler()

    state = INIT
    sub_state = 0
    sub_count = 0
    lm.loginfo("j1900-1057")

    # new calibration test

    gscc = GuidingStationCalibrationAppClient(using_fleet_rb=False)
    # /modules/measurement/act_measure/goal
    # /modules/measurement/act_measure/result
    # GBMeasureGoal
    rospy.Subscriber("/modules/measurement/act_measure/goal", GBMeasureActionGoal, get_goal, queue_size=1)
    rospy.Subscriber("/modules/measurement/act_measure/result", GBMeasureActionResult, get_result, queue_size=1)


    set_pose = GS_0_SET_POSE.Publisher()
    set_pose_type = GS_0_SET_POSE.type()

    rospy.sleep(3)
    gscc.reset()
    rospy.sleep(3)

    cali_state = INIT
    cali_sub_state = 0
    mesu_sub_state = 0
    cali_group = 0
    rb_id = 0
    c_t = 0
    m_t = 0

    # gid = 0
    # gscc.calibrate(
    #     (75, 2.3, 0.),
    #     0.03,
    #     COLOR,
    #     (28.767, 10.976),
    #     COLOR,
    #     (81.532, 10.976)
    # )
    cali_rate = rospy.Rate(2)
    while not rospy.is_shutdown():
        if cali_state == INIT:
            if gscc.connect():
                lm.loginfo("gscc connect succeeded.")
                cali_sub_state = 0
                cali_state = CALIBRATING
        elif cali_state == CALIBRATING:
            c_t += 1
            lm.loginfo_throttle(5, "cali time {}".format(c_t))
            if c_t >= 500:
                save_data['c']["calibration_is_succeeded"] = "false"
                lm.loginfo("save data: {}".format(save_data))
                jh.write_json(gid, save_data)   
                lm.loginfo("cali timeout...")             
                gscc.reset()
                c_t = 0
                cali_sub_state = 0

            if cali_sub_state == 0:
                lm.loginfo("start to calibration")
                cali_id = gid % 3
                cali_id_f = 0 if (cali_id + 1) > 2 else cali_id + 1
                cali_id_r = 2 if (cali_id - 1) < 0 else cali_id - 1
                lm.loginfo("gid: {}, cali_id: {}, cali_id_f: {}, cali_id_r: {}".format(
                    gid, cali_id,  cali_id_f, cali_id_r))
                # if cali_id == 0:
                gscc.calibrate(
                    GUESS_POSE,
                    0.03,
                    COLOR,
                    (goal_list[cali_id_f]["x"],
                     goal_list[cali_id_f]["y"]),
                    COLOR,
                    (goal_list[cali_id_r]["x"],
                     goal_list[cali_id_r]["y"])
                )
                save_data = {}
                dic = {"c": {}}
                save_data.update(dic)
                save_data['c']["cali_id_f_x"] = goal_list[cali_id_f]["x"]
                save_data['c']["cali_id_f_y"] = goal_list[cali_id_f]["y"]
                save_data['c']["cali_id_r_x"] = goal_list[cali_id_r]["x"]
                save_data['c']["cali_id_r_y"] = goal_list[cali_id_r]["y"]
                c_start_time = datetime.datetime.now()
                save_data['c']["c_start_time"] = str(c_start_time)
                save_data['c']["type"] = "calibration"
                cali_sub_state = 1
            elif cali_sub_state == 1:
                if gscc.is_done():
                    c_end_time = datetime.datetime.now()
                    lm.loginfo("gs calibration done.")
                    if gscc.is_failed():
                        lm.loginfo("gs calibration failed.")
                        save_data['c']["calibration_is_succeeded"] = "false"
                        lm.loginfo("save data: {}".format(save_data))
                        jh.write_json(gid, save_data)
                        gid += 1
                    elif gscc.is_succeeded():
                        lm.loginfo("gs calibration succeeded.")
                        cali_res = gscc.get_result()
                        lm.loginfo(cali_res)
                        set_pose_type.pose = cali_res.calibrated_pose
                        for x in range(5):
                            set_pose.publish(set_pose_type)
                            time.sleep(0.3)
                        cali_state = MEASURING
                        # save_data = {"type": "calibration",
                        #              "calibration_is_succeeded": "true",
                        #              "calibration_data": jh.msg2json(cali_res)}
                        save_data['c']["calibration_is_succeeded"] = "true"
                        save_data['c']["calibration_data"] = json_message_converter.convert_ros_message_to_json(
                            cali_res)
                        dic = {"m": {}}
                        save_data.update(dic)
                        #  "calibration_is_succeeded": "true",
                        #  "calibration_data": jh.msg2json(cali_res)}
                    # jh.write_json(gid,save_data)
                    # cali_sub_state = 2
                    save_data["c_end_time"] = str(c_end_time)
                    save_data["c_diff_time"] = (
                        c_end_time - c_start_time).total_seconds()
                    gscc.reset()
                    c_t = 0
                    cali_sub_state = 0
                else:
                    lm.loginfo_throttle(5, "waiting for calibration done. {}".format(gscc.is_done()))


        elif cali_state == MEASURING:
            lm.loginfo_throttle(5, "measure time {}".format(m_t))
            m_t += 1
            # for m in measurement_goal_list:
            # lm.loginfo((goal,result))
            if mesu_sub_state == 0:
                # with temp_lock:
                if goal is not None:
                    mesu_sub_state = 1
                    lm.loginfo("get goal {}".format(goal))
                    goal_b = goal
                    goal = None
                    # lm.loginfo("measure goal {}".format(measurement_goal_list[rb_id]))
                    # gsc.send_goal(measurement_goal_list[rb_id])
                    save_data["m"][str(rb_id)] = {}
                    cali_sub_state = 1
                    m_start_time = datetime.datetime.now()
                    save_data["m"][str(rb_id)]["m_start_time"] = str(m_start_time)
            if mesu_sub_state == 1:
                # with temp_lock:
                if result is not None:
                    mesu_sub_state = 0
                    # if result.status.goal_id.id != goal_id:
                    #     cali_rate.sleep()
                    #     continue
                        
                    lm.loginfo("get result {}".format(result.result))
                    # if not gsc.is_succeeded() and not gsc.is_failed():
                    #     lm.loginfo("waitting measure done.")
                    #     cali_rate.sleep()
                    #     continue
                    # else:
                    #     lm.logwarn("measurment done..")

                    # if gsc.is_done():
                    if not save_data["m"].has_key(str(rb_id)):
                        save_data["m"][str(rb_id)] = {}
                        m_start_time = datetime.datetime.now()
                        save_data["m"][str(rb_id)]["m_start_time"] = str(m_start_time)                            
                    result_b = result
                    result = None
                    m_end_time = datetime.datetime.now()
                    save_data["m"][str(rb_id)]["m_end_time"] = str(m_end_time)
                    if result_b.result.located:
                        goal_id = None
                        lm.loginfo("measurement succeeded.")
                        # measurement_res = gsc.get_result()
                        # lm.loginfo(result.result)
                        # save_data["m"][str(rb_id)] = {"measurement_is_succeeded": "true",
                        #              "measurement_data": jh.msg2json(measurement_res)}
                        save_data["m"][str(rb_id)]["measurement_is_succeeded"] = "true"
                        save_data["m"][str(rb_id)]["measurement_data"] = json_message_converter.convert_ros_message_to_json(
                            result_b.result)
                    else:
                        lm.loginfo("measurement failed.")
                        # save_data["m"][str(rb_id)] = {"measurement_is_succeeded": "false"}
                        save_data["m"][str(rb_id)]["measurement_is_succeeded"] = "false"
                    save_data["m"][str(rb_id)]["m_diff_time"] = (
                        m_start_time - m_end_time).total_seconds()
                    gsc.reset()
                    lm.loginfo_throttle(2, "rb_id {}, cali_sub_state {}".format(rb_id, cali_sub_state))
                    rb_id += 1
                    cali_sub_state = 0
                
            if m_t >= 500:
                lm.loginfo("measure timeout")
                save_data["m"][str(rb_id)] = {}
                save_data["m"][str(rb_id)]["measurement_is_succeeded"] = "false"
                # save_data["m"][str(rb_id)]["m_diff_time"] = (
                #     m_start_time - m_end_time).total_seconds()
                rb_id += 1
                cali_sub_state = 0

            if rb_id == 3:
                rb_id = 0
                m_t = 0
                lm.loginfo("save data: {}".format(save_data))
                jh.write_json(gid, save_data)
                gid += 1
                cali_state = CALIBRATING

        cali_rate.sleep()
