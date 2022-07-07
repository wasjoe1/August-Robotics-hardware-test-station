#!/usr/bin/env python
# -*- coding: utf-8 -*-

from __future__ import print_function

import json
import uuid

class JsonReportFormatter(json.JSONEncoder):
    """
    A vistor that creates a JSON representation of the visited report. Based
    on https://stackoverflow.com/a/25935321 to produce custom indentation.
    """

    class NoIndent(object):
        """
        Wrapper class to prevent the specified contents from being indented.
        """
        def __init__(self, value):
            self.value = value

    def __init__(self):
        super(JsonReportFormatter, self).__init__(
            sort_keys=True,
            indent=2,
            separators=(', ', ': ')
        )
        self._replacement_map = {}

    def default(self, o):
        if isinstance(o, JsonReportFormatter.NoIndent):
            key = uuid.uuid4().hex
            self._replacement_map[key] = json.dumps(
                o.value,
                sort_keys=True,
                separators=(', ', ': '))
            return "@@%s@@" % (key,)
        else:
            return super(JsonReportFormatter, self).default(o)

    def encode(self, o):
        result = super(JsonReportFormatter, self).encode(o.print(self))
        for k, v in self._replacement_map.iteritems():
            result = result.replace('"@@%s@@"' % (k,), v)
        return result

    def print_report(self, report):
        result = {}

        gs_list = [gs.print(self) for gs in report.guiding_station_map.values()]
        loc_list = [loc.print(self) for loc in report.localization_list]
        nav_list = [nav.print(self) for nav in report.navigation_list]
        mrk_list = [mrk.print(self) for mrk in report.marking_list]
        err_list = [err.print(self) for err in report.error_list]
        error_code_record_list = [err_code.print_code(self) for err_code in report.error_list if err_code.print_code(self) is not None]

        try:
            _avg_nav_rate = report.total_success_nav_duration/report.total_nav_success
        except:
            _avg_nav_rate = None

        try:
            _avg_mrk_rate = report.total_success_mrk_duration/report.total_mrk_success
        except:
            _avg_mrk_rate = None

        try:
            _avg_loc_rate = report.total_success_loc_duration/report.total_loc_success
        except:
            _avg_loc_rate = None

        try:
            avg_nav_rate = report.total_nav_duration/report.total_nav_goal
        except:
            avg_nav_rate = None

        try:
            avg_mrk_rate = report.total_mrk_duration/report.total_mrk_goal
        except:
            avg_mrk_rate = None

        try:
            avg_loc_rate = report.total_loc_duration/report.total_loc_goal
        except:
            avg_loc_rate = None

        try:
            ideal_avg_mrk_rate_per_hr = 3600 / (_avg_nav_rate + avg_mrk_rate)
        except:
            ideal_avg_mrk_rate_per_hr = None

        setup = {}
        setup.update({"map": report.map_setup.print(self)})
        setup.update({"boothbot": report.boothbot_setup.print(self)})
        setup.update({"guiding_stations": gs_list})

        init_pose = report.init_pose.print(self)

        gotomark = report.gotomark.print(self)
        gotomark.update({
            "total_nav_goal": report.total_nav_goal,
            "total_nav_duration": report.total_nav_duration,
            "total_nav_success": report.total_nav_success,
            "total_success_nav_duration": report.total_success_nav_duration,
            "total_mrk_goal": report.total_mrk_goal,
            "total_mrk_duration": report.total_mrk_duration,
            "total_mrk_success": report.total_mrk_success,
            "total_success_mrk_duration": report.total_success_mrk_duration,
            "total_loc_goal": report.total_loc_goal,
            "total_loc_duration": report.total_loc_duration,
            "total_loc_success": report.total_loc_success,
            "total_success_loc_duration": report.total_success_loc_duration
        })

        statistic = {}
        statistic.update({
            "1 - avg_nav_duration (when success)": _avg_nav_rate,
            "1 - avg_mrk_duration (when success)": avg_mrk_rate,
            "1 - avg_loc_duration (when success)": _avg_loc_rate,
            "1 - avg_loc_times_per_nav_goal": report.avg_loc_times_per_nav_goal,
            "2 - std_nav_duration (when success)": report.std_sucess_nav_duration,
            "2 - std_mrk_duration (when success)": report.std_sucess_mrk_duration,
            "2 - std_loc_duration (when success)": report.std_sucess_loc_duration,
            "3 - med_nav_duration (when success)": report.median_sucess_nav_duration,
            "3 - med_mrk_duration (when success)": report.median_sucess_mrk_duration,
            "3 - med_loc_duration (when success)": report.median_sucess_loc_duration,
            "4 - cb_locating_failed_attemps": report.cb_total_failed_times,
            "4 - gs_locating_failed_attemps": report.gs_total_failed_times,
            "4 - times_of_cb_first_located": report.total_times_of_cb_located_first,
            "4 - times_of_gs_first_located": report.total_times_of_gs_located_first,
            "4 - gs_cb_located_time_diff": report.avg_gs_cb_located_time_diff,
            "5 - gotomark_avg_mrk_rate_per_hr": report.avg_mrk_rate,
            "5 - ideal_avg_mrk_rate_per_hr (when all success)": ideal_avg_mrk_rate_per_hr
            # "gotomark_avg_mrk_rate (when success)": report.avg_mrk_rate
        })

        result.update({"0 - setup": setup})
        result.update({"3 - init_pose": init_pose})
        result.update({"2 - gotomark": gotomark})
        result.update({"4 - nav_list": nav_list})
        result.update({"5 - mrk_list": mrk_list})
        result.update({"6 - loc_list": loc_list})
        result.update({"8 - err_list": err_list})
        result.update({"1 - statistic": statistic})
        result.update({"7 - err_code_list": error_code_record_list})
        return result

    def print_map_setup(self, map_setup):
        return {
            "id": map_setup.id,
            "resolution": map_setup.resolution,
            "width": map_setup.width,
            "height": map_setup.height,
            "origin": JsonReportFormatter.NoIndent(map_setup.origin)
        }

    def print_boothbot_setup(self, boothbot_setup):
        return {
            "init_pose": JsonReportFormatter.NoIndent(boothbot_setup.init_pose),
            "name": boothbot_setup.machine_hostname,
            "git_info": boothbot_setup.git_info,
            "initial_battery_voltage": boothbot_setup.battery_level,
            "machine_uptime": boothbot_setup.machine_uptime
        }

    def print_guiding_station(self, guiding_station):
        return {
            "id": guiding_station.id,
            "pose": JsonReportFormatter.NoIndent(guiding_station.pose),
            "name": guiding_station.name
        }

    def print_localization_attempt(self, loc_attempt):
        return {
            "1 - start_time": loc_attempt.start_time,
            "1 - duration": loc_attempt.duration,
            "1 - success": loc_attempt.success_flag,
            "3 - exp_pose": loc_attempt.exp_pose,
            "3 - located_pose": loc_attempt.located_pose,
            "4 - pose_diff": JsonReportFormatter.NoIndent(loc_attempt.pose_diff),
            "2 - laser_exp_dist": loc_attempt.exp_dist,
            "2 - laser_measured_dist": loc_attempt.measured_dist,
            "1 - used_gss": loc_attempt.used_gss,
            "3 - gs_located": str(loc_attempt.gs_located) + " --- " + str(loc_attempt.gs_located_time),
            "3 - cb_located": str(loc_attempt.cb_located) + " --- " + str(loc_attempt.cb_located_time)
        }

    def print_error_message(self, errmsg):
        return errmsg.msg

    def print_error_code_list(self,errmsg):
        return errmsg.raw_error_code

    def print_init_pose(self, init_pose):
        return {
            "trigger_time": init_pose.start_time,
            "duration": init_pose.duration
        }

    def print_gotomark(self, gotomark):
        return {
            "1 - start_time": gotomark.start_time,
            "1 - end_time": gotomark.end_time,
            "0 - duration": gotomark.duration,
            "0 - success": gotomark.success_flag
        }

    def print_navigation_goal(self, nav_goal):
        return {
            "1 - start_time": nav_goal.start_time,
            "1 - map_goal_id": nav_goal.map_goal_id,
            "1 - duration": nav_goal.duration,
            "2 - start_pose": JsonReportFormatter.NoIndent(nav_goal.start_pose),
            "2 - end_pose": JsonReportFormatter.NoIndent(nav_goal.end_pose),
            "2 - times_of_loc": JsonReportFormatter.NoIndent(nav_goal.location_times),
            "3 - accuracy": JsonReportFormatter.NoIndent(nav_goal.accuracy),
            "1 - success": nav_goal.success_flag,
            "1 - avg_speed": nav_goal.avg_speed
        }

    def print_marking_goal(self, mrk_goal):
        return {
            "1 - map_goal_id": mrk_goal.corner_id,
            "1 - mark_content": mrk_goal.mark_content,
            "2 - start_time": mrk_goal.start_time,
            "2 - marker_pose": JsonReportFormatter.NoIndent(mrk_goal.end_pose),
            "accuracy": JsonReportFormatter.NoIndent(mrk_goal.accuracy),
            "3 - duration": mrk_goal.duration,
            "3 - success" : mrk_goal.success_flag,
            "3 - pic_name": mrk_goal.pic_name,
            "3 - score": mrk_goal.score
        }
