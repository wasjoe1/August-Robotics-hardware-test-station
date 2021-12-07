#!/usr/bin/env python
# -*- coding: utf-8 -*-

from __future__ import print_function

import re
import rospy
import sys
import json
import math
import os
from datetime import datetime

import boothbot_msgs.msg as bbmsgs
from boothbot_msgs.ros_interfaces import ROBOT_ACT_TASK

### Script that runs fake Lionel.

# Initialize variables.
map_id = ""
init_pose = ""
gs_id_list = ""

init_goal_result = None
gotomark_goal_result = None

# Env. variable from os
HOME_PATH = os.environ.get('HOME')

# TODO: Check that the map exists in the database.
def is_map_id_valid(input_string):
    pattern = re.compile('(0|([1-9][0-9]*))$')
    return pattern.match(input_string) is not None

# TODO: Check that the coordinates are within the map (and plausible).
def is_init_pose_valid(input_string):
    num_re_string = '-?((0|([1-9][0-9]*))(\.[0-9]+)?)'
    pattern = re.compile(num_re_string + ' ' + num_re_string + ' ' + num_re_string + '$')
    return pattern.match(input_string) is not None

# TODO: Ensure that the guiding station(s) declared are the same as those launched.
def is_gs_list_valid(input_string):
    base_pattern = re.compile('(0|([1-9][0-9]*))$')
    is_none_list = [base_pattern.match(s) is None for s in re.split(' ',input_string)]
    return not reduce(lambda x, y: x or y, is_none_list)

def init_goal_cb(status, result):
    global init_goal_result
    init_goal_result = result

def gotomark_goal_cb(status, result):
    global gotomark_goal_result
    gotomark_goal_result = result

if __name__ == "__main__":
    # Request user inputs.
    while not is_map_id_valid(map_id):
        print("Please enter map id (0 or greater): ")
        map_id = raw_input()
    map_id = int(map_id)

    while not is_init_pose_valid(init_pose):
        print("Please enter init pose in the form of x y rz (degree): ")
        print("E.g., 5 10 90")
        init_pose = raw_input()
    init_pose = [float(n) for n in re.split(' ', init_pose)]
    init_pose[2] = init_pose[2] * math.pi / 180

    while not is_gs_list_valid(gs_id_list):
        print("Please enter the list of guiding stations to use: ")
        print("E.g., 1 2")
        gs_id_list = raw_input()
    gs_id_list = [int(n) for n in re.split(' ', gs_id_list)]

    # Set up robot action client.
    rospy.init_node("run_simulation")
    robot_client = ROBOT_ACT_TASK.SimpleActionClient()
    if not robot_client.wait_for_server(timeout=rospy.Duration(10.0)):
        rospy.logerr('Cannot connect to server... Please make sure fake Lionel is running.')
        sys.exit(1)

    # Run init pose.
    rospy.loginfo("Running init pose...")
    init_goal = bbmsgs.BoothbotTaskGoal()
    init_goal.stamp = rospy.Time.now()
    init_goal.command = bbmsgs.BoothbotTaskGoal.INIT_MAP
    init_goal.parameter = json.dumps({
        "valid_gs": gs_id_list,
        "guess_pose": init_pose,
        "map_id": map_id
    })

    # TODO: Allow user to specify timeout.
    robot_client.send_goal(init_goal, done_cb = init_goal_cb)
    init_goal_finished = robot_client.wait_for_result()

    if not init_goal_finished:
        rospy.logerr("Init pose timed out!")
        sys.exit(1)

    if not init_goal_result.result == bbmsgs.BoothbotTaskResult.SUCCEEDED:
        rospy.logerr("Init pose failed!")
        sys.exit(1)

    rospy.loginfo("Init pose succeeded!")

    # Run goto-mark
    # TODO: Allow user to specify goal parameters.
    rospy.loginfo("Running goto-mark...")
    gotomark_goal = bbmsgs.BoothbotTaskGoal()
    gotomark_goal.stamp = rospy.Time.now()
    gotomark_goal.command = bbmsgs.BoothbotTaskGoal.GOTO_MARK
    gotomark_goal.parameter = json.dumps({
        "default_goal_tolerance": [0.005, 0.005, 0.05],
        "command": "RUN",
        "default_last_moving_factor": 1
    })

    # TODO: Allow user to specify timeout.
    robot_client.send_goal(gotomark_goal, done_cb = gotomark_goal_cb)
    gotomark_goal_finished = robot_client.wait_for_result()

    if not gotomark_goal_finished:
        rospy.logerr("Goto-mark timed out!")
        sys.exit(1)

    if not gotomark_goal_result.result == bbmsgs.BoothbotTaskResult.SUCCEEDED:
        rospy.logerr("Goto-mark failed!")
        sys.exit(1)

    rospy.loginfo("Goto-mark succeeded!")

    # Create link to log.
    # TODO: Use environment variables to store the correct path.
    _log_dir = HOME_PATH + ".ros/log/latest"
    # log_dir = os.path.realpath("/home/augbooth/.ros/log/latest") #Aviod using absolute dir
    log_dir = os.path.realpath(_log_dir)
    log_path = log_dir + "/rosout.log"

    target_dir = "../reports"
    target_log_suffix = datetime.now().strftime("%Y%m%d%H%M%S")
    target_path = target_dir + "/rosout-" + target_log_suffix + ".log"
    os.symlink(log_path, target_path)
    rospy.loginfo("Log file link: " + target_path)
