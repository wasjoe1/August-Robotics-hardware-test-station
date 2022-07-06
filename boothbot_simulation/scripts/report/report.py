#!/usr/bin/env python
# -*- coding: utf-8 -*-

from __future__ import print_function
from cgitb import reset
import numpy as np

from enum import Enum
from transitions import Machine

from common.py_logger import PyLogging
from boothbot_common.datacode import DataCode

class ReportState(Enum):
    """
    The report can be in 5 different states, each of which governs what data
    can be added into the report. In particular, each `job` is assumed to
    comprise two stages:
    - init_pose, where all setup-related data, such as the map,
      boothbot pose and guiding station pose(s), are loaded into the system.
    - gotomark, where boothbot begins navigating to and marking each corner.
    """
    PRE_INIT = 0
    INIT_SETUP = 1
    PRE_GOTOMARK = 2
    GOTOMARK_EXEC = 3
    POST_GOTOMARK = 4

class Report(PyLogging):
    """
    The root class for creating a report from the logged data items. It
    recursively calls other classes to process input data items according
    to their type, and it implements the visitor pattern to allow the
    report to be printed.
    """
    UNEXPECTED_DATA_MSG = "{}: Ignoring unexpected message! \n\t [{}] {}"

    def __init__(self):
        super(Report, self).__init__("Report")
        self.add_data = self._add_data_PRE_INIT
        self.machine = Machine(
            model = self,
            states = ReportState,
            initial = ReportState.PRE_INIT,
            auto_transitions = True,
            send_event = False
        )

        self.init_pose = InitPose()
        self.map_setup = MapSetup()
        self.boothbot_setup = BoothbotSetup()
        self.guiding_station_map = {}
        self.localization_list = []
        self.error_list = []
        self.gotomark = GotoMark()
        self.navigation_list = []
        self.marking_list = []

    ### PRE_INIT State ###
    ### The report waits for the start of init_pose, which is indicated by the
    ### arrival of the INIT_POSE_START_TIME data.

    def on_enter_PRE_INIT(self):
        self.add_data = self._add_data_PRE_INIT
        self.guiding_station_map["TEMP"] = GuidingStationSetup()

    def _add_data_PRE_INIT(self, code, data):
        is_data_consumed = False
        if True: #code == DataCode.INIT_POSE_START_TIME:
            # Proceed to INIT_SETUP state and try adding data again.
            self.to_INIT_SETUP()
            self.add_data(code, data)
            is_data_consumed = True
        else:
            self.logerr(self.UNEXPECTED_DATA_MSG.format(self.state.name, code, data))
            is_data_consumed = False
        return is_data_consumed

    ### INIT State ###
    ### The report starts collecting init_pose data. Init_pose is completed when
    ### the INIT_POSE_END_TIME data arrives.
    def on_enter_INIT_SETUP(self):
        self.add_data = self._add_data_INIT_SETUP
        self.localization_list.append(LocalizationAttempt())
        self.guiding_station_map["TEMP"] = GuidingStationSetup()
        self.error_list.append(ErrorMessage())

    def on_exit_INIT_SETUP(self):
        locAttempt = self.localization_list[-1]
        if locAttempt.is_empty():
            self.localization_list.pop()

        gsTemp = self.guiding_station_map["TEMP"]
        if gsTemp.name is not None and gsTemp.name not in self.guiding_station_map:
            self.guiding_station_map[gsTemp.id] = gsTemp
        self.guiding_station_map.pop("TEMP")

        errMsg = self.error_list[-1]
        if errMsg.is_empty():
            self.error_list.pop()

    def _add_data_INIT_SETUP(self, code, data):
        is_data_consumed = False
        locAttempt = self.localization_list[-1]
        gsTemp = self.guiding_station_map["TEMP"]
        errMsg = self.error_list[-1]

        component_list = [
            self.init_pose,
            self.map_setup,
            self.boothbot_setup,
            locAttempt,
            gsTemp,
            errMsg
        ]
        for p in component_list:
            is_data_consumed = is_data_consumed or p.add_data(code, data)

        if not is_data_consumed:
            self.logerr(self.UNEXPECTED_DATA_MSG.format(self.state.name, code, data))
        elif locAttempt.is_complete():
            self.localization_list.append(LocalizationAttempt())
        elif gsTemp.is_complete():
            self.guiding_station_map[gsTemp.id] = gsTemp
            self.guiding_station_map["TEMP"] = GuidingStationSetup()
        elif errMsg.is_complete():
            self.error_list.append(ErrorMessage())
        elif self.init_pose.is_complete():
            self.to_PRE_GOTOMARK()

        return is_data_consumed

    ### PRE_GOTOMARK State ###
    ### The report waits for the start of gotomark, which is indicated by the
    ### arrival of the GOTOMARK_START_TIME data.
    def on_enter_PRE_GOTOMARK(self):
        self.add_data = self._add_data_PRE_GOTOMARK
        self.guiding_station_map["TEMP"] = GuidingStationSetup()

    def _add_data_PRE_GOTOMARK(self, code, data):
        is_data_consumed = False
        if code == DataCode.GOTOMARK_START_TIME:
            # Proceed to GOTOMARK_EXEC state and try adding data again.
            self.to_GOTOMARK_EXEC()
            self.add_data(code, data)
            is_data_consumed = True
        else:
            self.logerr(self.UNEXPECTED_DATA_MSG.format(self.state.name, code, data))
            is_data_consumed = False
        return is_data_consumed

    ### GOTOMARK_EXEC State ###
    ### The report starts collecting gotomark data. Gotomark is completed when
    ### the GOTOMARK_END_TIME data arrives.
    def on_enter_GOTOMARK_EXEC(self):
        self.add_data = self._add_data_GOTOMARK_EXEC
        self.localization_list.append(LocalizationAttempt())
        # self.guiding_station_map["TEMP"] = GuidingStationSetup()
        self.error_list.append(ErrorMessage())
        self.navigation_list.append(NavigationGoal())
        self.marking_list.append(MarkingGoal())

    def on_exit_GOTOMARK_EXEC(self):
        locAttempt = self.localization_list[-1]
        if locAttempt.is_empty():
            self.localization_list.pop()

        gsTemp = self.guiding_station_map["TEMP"]
        if gsTemp.id is not None and gsTemp.id not in self.guiding_station_map:
            self.guiding_station_map[gsTemp.id] = gsTemp
        self.guiding_station_map.pop("TEMP")

        errMsg = self.error_list[-1]
        if errMsg.is_empty():
            self.error_list.pop()

        navGoal = self.navigation_list[-1]
        if navGoal.is_empty():
            self.navigation_list.pop()

        mrkGoal = self.marking_list[-1]
        if mrkGoal.is_empty():
            self.marking_list.pop()

    def _add_data_GOTOMARK_EXEC(self, code, data):
        is_data_consumed = False
        locAttempt = self.localization_list[-1]
        gsTemp = self.guiding_station_map["TEMP"]
        errMsg = self.error_list[-1]
        navGoal = self.navigation_list[-1]
        mrkGoal = self.marking_list[-1]

        component_list = [
            self.gotomark,
            self.map_setup,
            self.boothbot_setup,
            locAttempt,
            gsTemp,
            errMsg,
            navGoal,
            mrkGoal
        ]
        for p in component_list:
            is_data_consumed = is_data_consumed or p.add_data(code, data)

        if not is_data_consumed:
            self.logerr(self.UNEXPECTED_DATA_MSG.format(self.state.name, code, data))
        elif locAttempt.is_complete():
            self.localization_list.append(LocalizationAttempt())
        elif errMsg.is_complete():
            self.error_list.append(ErrorMessage())
        elif navGoal.is_complete():
            self.navigation_list.append(NavigationGoal())
        elif mrkGoal.is_complete():
            self.marking_list.append(MarkingGoal())
        elif self.gotomark.is_complete():
            self.to_POST_GOTOMARK()
        elif gsTemp.is_complete():
            self.guiding_station_map[gsTemp.id] = gsTemp
            self.guiding_station_map["TEMP"] = GuidingStationSetup()

        return is_data_consumed

    ### POST_GOTOMARK State ###
    ### This is an absorbing state. All data received in this state are assumed
    ### to be irrelevant for the job in question.
    def on_enter_POST_GOTOMARK(self):
        self.add_data = self._add_data_POST_GOTOMARK

    def _add_data_POST_GOTOMARK(self, code, data):
        self.logerr(self.UNEXPECTED_DATA_MSG.format(self.state.name, code, data))
        return False

    def print(self, printer):
        try:
            return printer.print_report(self)
        except Exception as e:
            print(e)
            return

    @property
    def total_nav_goal(self):
        try:
            return len(self.navigation_list)
        except Exception as e:
            print(e)
            return

    @property
    def total_nav_duration(self):
        try:
            return sum([x for x in map(lambda n: n.duration, self.navigation_list) if isinstance(x,float)])
        except Exception as e:
            print(e)
            return

    @property
    def total_nav_success(self):
        try:
            return sum([x for x in map(lambda n: n.success_flag == True, self.navigation_list) if isinstance(x,bool)])
        except Exception as e:
            print(e)
            return

    @property
    def total_success_nav_duration(self):
        try:
            return sum([x for x in map(lambda n: n.successful_duration, self.navigation_list) if isinstance(x,float)])
        except Exception as e:
            print(e)
            return


    @property
    def total_mrk_goal(self):
        try:
            return len(self.marking_list)
        except Exception as e:
            print(e)
            return

    @property
    def total_mrk_duration(self):
        try:
            return sum([x for x in map(lambda n: n.duration, self.marking_list) if isinstance(x,float)])
        except Exception as e:
            print(e)
            return

    @property
    def total_mrk_success(self):
        try:
            return sum([x for x in map(lambda n: n.success_flag == True, self.marking_list) if isinstance(x,bool)])
        except Exception as e:
            print(e)
            return

    @property
    def total_success_mrk_duration(self):
        try:
            return sum([x for x in map(lambda n: n.successful_duration, self.marking_list) if isinstance(x,float)])
        except Exception as e:
            print(e)
            return

    @property
    def total_loc_goal(self):
        try:
            return len(self.localization_list)
        except Exception as e:
            print(e)
            return

    @property
    def total_loc_duration(self):
        try:
            return sum([x for x in map(lambda n: n.duration, self.localization_list) if isinstance(x,float) or isinstance(x,int)])
        except Exception as e:
            print(e)
            return

    @property
    def total_loc_success(self):
        try:
            return sum([x for x in map(lambda n: n.success_flag == True, self.localization_list) if isinstance(x,bool)])
        except Exception as e:
            print(e)
            return

    @property
    def total_success_loc_duration(self):
        try:
            # return sum([x for x in map(lambda n: n.successful_duration, self.localization_list) if isinstance(x,float)])
            return sum([x.successful_duration for x in self.localization_list if isinstance(x.successful_duration,float)])
        except Exception as e:
            print(e)
            return

    @property
    def cb_total_failed_times(self):
        try:
            return sum([x for x in map(lambda n: n.cb_located != True, self.localization_list) if isinstance(x,bool)])
        except Exception as e:
            print(e)
            return
    @property
    def gs_total_failed_times(self):
        try:
            return sum([x for x in map(lambda n: n.gs_located != True, self.localization_list) if isinstance(x,bool)])
        except Exception as e:
            print(e)
            return

    @property
    def total_times_of_cb_located_first(self):
        return sum([x for x in map(lambda n: n.cb_first_located == True, self.localization_list) if isinstance(x,bool)])
    @property
    def total_times_of_gs_located_first(self):
        return sum([x for x in map(lambda n: n.cb_first_located != True, self.localization_list) if isinstance(x,bool)])

    @property
    def avg_gs_cb_located_time_diff(self):
        return np.average([x.gs_cb_located_time_diff for x in self.localization_list if isinstance(x.gs_cb_located_time_diff,float)])

    @property
    def avg_mrk_rate(self):
        try:
            return self.total_mrk_goal / self.gotomark.duration * 60 * 60
        except Exception as e:
            print(e)
            return

    @property
    def avg_loc_times_per_nav_goal(self):
        try:
            total_nav_success =  sum([x for x in map(lambda n: n.success_flag == True, self.navigation_list) if isinstance(x,bool)])
            total_loc_with_nav_success = sum([x.location_times for x in self.navigation_list if (x.success_flag == True)])
            return total_loc_with_nav_success / float(total_nav_success)
        except Exception as e:
            print(e)
            return

    @property
    def std_sucess_loc_duration(self):
        try:
            return np.std([x for x in map(lambda n: n.successful_duration, self.localization_list) if isinstance(x,float)])
        except Exception as e:
            print(e)
            return
    @property
    def std_sucess_nav_duration(self):
        try:
            return np.std([x for x in map(lambda n: n.successful_duration, self.navigation_list) if isinstance(x,float)])
        except Exception as e:
            print(e)
            return
    @property
    def std_sucess_mrk_duration(self):
        try:
            # return np.std([x for x in map(lambda n: n.successful_duration, self.marking_list) if isinstance(x,float)])
            return np.std([x for x in map(lambda n: n.duration, self.marking_list) if isinstance(x,float)])
        except Exception as e:
            print(e)
            return

    @property
    def median_sucess_loc_duration(self):
        try:
            return np.median([x for x in map(lambda n: n.successful_duration, self.localization_list) if isinstance(x,float)])
        except Exception as e:
            print(e)
            return
    @property
    def median_sucess_nav_duration(self):
        try:
            return np.median([x for x in map(lambda n: n.successful_duration, self.navigation_list) if isinstance(x,float)])
        except Exception as e:
            print(e)
            return
    @property
    def median_sucess_mrk_duration(self):
        try:
            # return np.std([x for x in map(lambda n: n.successful_duration, self.marking_list) if isinstance(x,float)])
            return np.median([x for x in map(lambda n: n.duration, self.marking_list) if isinstance(x,float)])
        except Exception as e:
            print(e)
            return


### The following classes are all helper classes for processing the
### corresponding types of data. In general, they all:
### - Contain attributes for storing the relevant data.
### - Implement an add_data method to populate the attributes.
### - Implement the visitor pattern for pretty-printing.
### - Implement methods for checking their status or computing statistics
###   (where necessary).

class MapSetup:
    def __init__(self):
        self.id = None
        self.occupancy = None
        self.resolution = None
        self.width = None
        self.height = None
        self.origin = None

    def add_data(self, code, data):
        is_data_consumed = False
        if code == DataCode.MAP_ID:
            self.id = data
            is_data_consumed = True
        elif code == DataCode.MAP_OCCUPANCY:
            self.occupancy = data
            is_data_consumed = True
        elif code == DataCode.MAP_RESOLUTION:
            self.resolution = data
            is_data_consumed = True
        elif code == DataCode.MAP_WIDTH:
            self.width = data
            is_data_consumed = True
        elif code == DataCode.MAP_HEIGHT:
            self.height = data
            is_data_consumed = True
        elif code == DataCode.MAP_ORIGIN:
            self.origin = data
            is_data_consumed = True
        return is_data_consumed

    def print(self, printer):
        try:
            return printer.print_map_setup(self)
        except Exception as e:
            print(e)
            return

class BoothbotSetup:
    def __init__(self):
        self.init_pose = None
        self.machine_hostname = None
        self.git_info = None
        self.battery_level = None
        self.machine_uptime = None

    def add_data(self, code, data):
        is_data_consumed = False
        if code == DataCode.INIT_GUESS_POSE:
            self.init_pose = data
            is_data_consumed = True
        elif code == DataCode.MACHINE_HOSTNAME:
            self.machine_hostname = data
            is_data_consumed = True
        elif code == DataCode.GIT_INFO:
            self.git_info = data
            is_data_consumed = True
        elif code == DataCode.BATTERY_LEVEL:
            self.battery_level = data
            is_data_consumed = True
        elif code == DataCode.MACHINE_UPTIME:
            self.machine_uptime = data
            is_data_consumed = True
        return is_data_consumed

    def print(self, printer):
        try:
            return printer.print_boothbot_setup(self)
        except Exception as e:
            print(e)
            return

class GuidingStationSetup:
    def __init__(self):
        self.id = None
        self.pose = None
        self.name = None

    def add_data(self, code, data):
        is_data_consumed = False
        if code == DataCode.GS_ID:
            self.id = data
            is_data_consumed = True
        elif code == DataCode.GS_POSE:
            self.pose = data
            is_data_consumed = True
        elif code == DataCode.GS_NAME:
            self.name = data
            is_data_consumed = True
        return is_data_consumed

    def is_complete(self):
        return self.id is not None and self.pose is not None and self.name is not None

    def print(self, printer):
        try:
            return printer.print_guiding_station(self)
        except Exception as e:
            print(e)
            return

class LocalizationAttempt:
    def __init__(self):
        self.start_time = None
        self.reset_params()

    def reset_params(self):
        self.start_pose = None
        self.end_time = None
        self.end_pose = None
        self.success_flag = None
        self.exp_dist = None
        self.measured_dist = None
        self.used_gss = None
        self.gs_located_time = None
        self.cb_located_time = None

    def add_data(self, code, data):
        is_data_consumed = False
        if code == DataCode.LOC_GOAL_START_TIME:
            self.start_time = data
            is_data_consumed = True
            self.reset_params()
        elif code == DataCode.LOC_GOAL_DETECTED_POSE:
            if self.start_pose is None:
                self.start_pose = data
            else:
                self.end_pose = data
            is_data_consumed = True
        elif code == DataCode.LOC_GOAL_SUCCEEDED:
            self.success_flag = data
            if data == False:
                self.end_time = self.start_time
            is_data_consumed = True
        elif code == DataCode.LOC_GOAL_END_TIME:
            self.end_time = data
            is_data_consumed = True
        elif code == DataCode.LOC_GOAL_EXP_DIST:
            self.exp_dist = data
            is_data_consumed = True
        elif code == DataCode.LOC_GOAL_DETECTED_DIST:
            self.measured_dist = data
            is_data_consumed = True
        elif code == DataCode.LOC_USED_GSS:
            self.used_gss = data
            is_data_consumed = True
        elif code == DataCode.LOC_CB_LOCATED_TIME:
            if self.cb_located_time == None:
                self.cb_located_time = data
                is_data_consumed = True
        elif code == DataCode.LOC_GS_LOCATED_TIME:
            if self.gs_located_time == None:
                self.gs_located_time = data
                is_data_consumed = True
        return is_data_consumed

    def is_empty(self):
        return (self.start_time is None and
            self.start_pose is None and
            self.end_time is None and
            self.end_pose is None and
            self.success_flag is None)

    def is_complete(self):
        return self.start_time is not None and self.end_time is not None

    def print(self, printer):
        try:
            return printer.print_localization_attempt(self)
        except Exception as e:
            print(e)
            return

    @property
    def start_time(self):
        return self.start_time

    @property
    def exp_pose(self):
        return self.start_pose

    @property
    def exp_dist(self):
        return self.exp_dist

    @property
    def measured_dist(self):
        return self.measured_dist

    @property
    def located_pose(self):
        return self.end_pose

    @property
    def pose_diff(self):
        try:
            return [self.end_pose[i] - self.start_pose[i] for i in range(3)]
        except Exception as e:
            print(e)
            return

    @property
    def gs_located(self):
        return not self.gs_located_time == None
    @property
    def cb_located(self):
        return not self.cb_located_time == None

    @property
    def cb_first_located(self):
        try:
            return self.cb_located_time < self.gs_located_time
        except:
            return

    @property
    def gs_cb_located_time_diff(self):
        try:
            return self.gs_located_time - self.cb_located_time
        except:
            return

    @property
    def duration(self):
        # is_valid = False
        # if isinstance(self.end_time,float) and isinstance(self.start_time,float):
        #     is_valid = True
        # if is_valid:
        #     return self.end_time - self.start_time
        # else:
        #     return 0
        try:
            return self.end_time - self.start_time
        except Exception as e:
            print(e)
            return

    @property
    def successful_duration(self):
        try:
            if self.success_flag:
                return self.end_time - self.start_time
            else:
                return
        except Exception as e:
            print(e)
            return

class ErrorMessage:
    def __init__(self):
        self.msg = None
        self.raw_error_code = None

    def add_data(self, code, data):
        is_data_consumed = False
        if code == DataCode.ERROR:
            if data != "[]":
                self.msg = data
            is_data_consumed = True
        if code == DataCode.ERROR_CODE:
            self.raw_error_code = data
            is_data_consumed = True
        return is_data_consumed

    def is_complete(self):
        return self.msg is not None

    def is_empty(self):
        return self.msg is None

    def print(self, printer):
        return printer.print_error_message(self)

    def print_code(self,printer):
        return printer.print_error_code_list(self)

class InitPose:
    # TODO: Should add input pose, and located pose
    def __init__(self):
        self.start_time = None
        self.reset_params()

    def reset_params(self):
        self.end_time = None

    def add_data(self, code, data):
        is_data_consumed = False
        if code == DataCode.INIT_POSE_START_TIME:
            try:
                self.start_time = float(data)
                is_data_consumed = True
                self.reset_params()
            except Exception as e:
                self.logerr("Error with add_data [INIT_POSE_START_TIME]: {}".format(e))
        elif code == DataCode.INIT_POSE_END_TIME:
            try:
                self.end_time = float(data)
                is_data_consumed = True
            except Exception as e:
                self.logerr("Error with add_data [INIT_POSE_END_TIME]: {}".format(e))
        return is_data_consumed

    def is_complete(self):
        return self.start_time is not None and self.end_time is not None

    def has_begun(self):
        return self.start_time is not None

    def print(self, printer):
        try:
            return printer.print_init_pose(self)
        except Exception as e:
            print(e)
            return

    @property
    def start_time(self):
        return self.start_time

    @property
    def duration(self):
        try:
            return self.end_time - self.start_time
        except Exception as e:
            print(e)
            return

class GotoMark:
    def __init__(self):
        self.start_time = None
        self.reset_params()

    def reset_params(self):
        self.end_time = None
        self.success_flag = None

    def add_data(self, code, data):
        is_data_consumed = False
        if code == DataCode.GOTOMARK_START_TIME:
            if self.start_time is None:
                self.start_time = data
            is_data_consumed = True
            self.reset_params()
        elif code == DataCode.GOTOMARK_END_TIME:
            self.end_time = data
            is_data_consumed = True
        elif code == DataCode.GOTOMARK_SUCCEEDED:
            self.success_flag = data
            is_data_consumed = True
        return is_data_consumed

    def is_complete(self):
        return self.start_time is not None and self.end_time is not None

    def has_begun(self):
        return self.start_time is not None

    def print(self, printer):
        try:
            return printer.print_gotomark(self)
        except Exception as e:
            print(e)
            return

    @property
    def duration(self):
        try:
            return self.end_time - self.start_time
        except Exception as e:
            print(e)
            return

    @property
    def start_time(self):
        return self.start_time

    def end_time(self):
        return self.end_time

class NavigationGoal:
    def __init__(self):
        self.start_time = None
        self.reset_params()

    def reset_params(self):
        self.start_pose = None
        self.end_time = None
        self.end_pose = None
        self.target_pose = None
        self.success_flag = None
        self.map_goal_id = None
        self.times_of_locating_using_gs = None

    def add_data(self, code, data):
        is_data_consumed = False
        if code == DataCode.NAV_GOAL_START_TIME:
            self.start_time = data
            is_data_consumed = True
            self.reset_params()
        elif code == DataCode.NAV_GOAL_START_POSE:
            self.start_pose = data
            is_data_consumed = True
        elif code == DataCode.NAV_GOAL_END_TIME:
            self.end_time = data
            is_data_consumed = True
        elif code == DataCode.NAV_GOAL_END_POSE:
            self.end_pose = data
            is_data_consumed = True
        elif code == DataCode.NAV_GOAL_TARGET_POSE:
            self.target_pose = data
            is_data_consumed = True
        elif code == DataCode.NAV_GOAL_SUCCEEDED:
            self.success_flag = data
            is_data_consumed = True
        elif code == DataCode.NAV_GOAL_BC_ID:
            self.map_goal_id = data
            is_data_consumed = True
        elif code == DataCode.NAV_TIMES_OF_GS_LOC:
            self.times_of_locating_using_gs = data
            is_data_consumed = True
        return is_data_consumed

    def is_complete(self):
        return self.start_time is not None and self.end_time is not None

    def is_empty(self):
        return (self.start_time is None and
            self.start_pose is None and
            self.end_time is None and
            self.end_pose is None and
            self.target_pose is None and
            self.success_flag is None and
            self.map_goal_id is None and
            self.times_of_locating_using_gs is None)

    def print(self, printer):
        try:
            return printer.print_navigation_goal(self)
        except Exception as e:
            print(e)
            return

    @property
    def duration(self):
        try:
            return self.end_time - self.start_time
        except Exception as e:
            print(e)
            return

    @property
    def successful_duration(self):
        try:
            if self.success_flag:
                return self.end_time - self.start_time
            else:
                return
        except Exception as e:
            print(e)
            return

    @property
    def accuracy(self):
        try:
            return [self.end_pose[i] - self.target_pose[i] for i in range(3)]
        except Exception as e:
            print(e)
            return

    @property
    def avg_speed(self):
        try:
            x_dist = self.end_pose[0] - self.start_pose[0]
            y_dist = self.end_pose[1] - self.start_pose[1]
            total_dist = (x_dist**2 + y_dist**2) ** 0.5

            return total_dist / self.duration
        except Exception as e:
            print(e)
            return

    @property
    def map_goal_id(self):
        return self.map_goal_id

    @property
    def start_time(self):
        return self.start_time

    @property
    def location_times(self):
        return self.times_of_locating_using_gs

class MarkingGoal:
    def __init__(self):
        self.start_time = None
        self.reset_params()

    def reset_params(self):
        self.end_time = None
        self.corner_id = None
        self.end_pose = None
        self.target_pose = None
        self.success_flag = None
        self.pic_name = None
        self.score = None
        self.mark_content = None


    def add_data(self, code, data):
        is_data_consumed = False
        if code == DataCode.MRK_GOAL_START_TIME:
            self.start_time = data
            is_data_consumed = True
            self.reset_params()
        elif code == DataCode.MRK_GOAL_END_TIME:
            self.end_time = data
            is_data_consumed = True
        elif code == DataCode.MRK_GOAL_BC_ID:
            self.corner_id = data
            is_data_consumed = True
        elif code == DataCode.MRK_GOAL_END_POSE:
            self.end_pose = data
            is_data_consumed = True
        elif code == DataCode.MRK_GOAL_TARGET_POSE:
            self.target_pose = data
            is_data_consumed = True
        elif code == DataCode.MRK_GOAL_SUCCEEDED:
            self.success_flag = data
            is_data_consumed = True
        elif code == DataCode.MRK_GOAL_PIC_NAME:
            self.pic_name = data
            is_data_consumed = True
        elif code == DataCode.MRK_GOAL_SCORE:
            self.score = data
            is_data_consumed = True
        elif code == DataCode.MRK_GOAL_MARK_CONTENT:
            self.mark_content = data
        return is_data_consumed

    def is_complete(self):
        return self.start_time is not None and self.end_time is not None

    def is_empty(self):
        return (self.start_time is None and
            self.end_time is None and
            self.corner_id is None and
            self.end_pose is None and
            self.target_pose is None and
            self.success_flag is None and
            self.pic_name is None and
            self.score is None)

    def print(self, printer):
        try:
            return printer.print_marking_goal(self)
        except Exception as e:
            print(e)
            return

    @property
    def mark_content(self):
        return self.mark_content

    @property
    def start_time(self):
        return self.start_time

    @property
    def duration(self):
        try:
            return self.end_time - self.start_time
        except Exception as e:
            print(e)
            return

    @property
    def successful_duration(self):
        try:
            if self.success_flag:
                return self.end_time - self.start_time
            else:
                return
        except Exception as e:
            print(e)
            return

    @property
    def accuracy(self):
        try:
            return [self.end_pose[i] - self.target_pose[i] for i in range(3)]
        except Exception as e:
            print(e)
            return