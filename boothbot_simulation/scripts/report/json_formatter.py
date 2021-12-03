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
            "total_mrk_goal": report.total_mrk_goal,
            "total_mrk_duration": report.total_mrk_duration,
            "total_mrk_success": report.total_mrk_success,
            "total_loc_goal": report.total_loc_goal,
            "total_loc_duration": report.total_loc_duration,
            "total_loc_success": report.total_loc_success,
            "avg_mrk_rate": report.avg_mrk_rate
        })

        result.update({"setup": setup})
        result.update({"init_pose": init_pose})
        result.update({"gotomark": gotomark})
        result.update({"nav_list": nav_list})
        result.update({"mrk_list": mrk_list})
        result.update({"loc_list": loc_list})
        result.update({"err_list": err_list})
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
            "init_pose": JsonReportFormatter.NoIndent(boothbot_setup.init_pose)
        }

    def print_guiding_station(self, guiding_station):
        return {
            "id": guiding_station.id,
            "pose": JsonReportFormatter.NoIndent(guiding_station.pose)
        }

    def print_localization_attempt(self, loc_attempt):
        return {
            "duration": loc_attempt.duration,
            "success": loc_attempt.success_flag,
            "pose_diff": JsonReportFormatter.NoIndent(loc_attempt.pose_diff)
        }

    def print_error_message(self, errmsg):
        return errmsg.msg

    def print_init_pose(self, init_pose):
        return {
            "duration": init_pose.duration
        }

    def print_gotomark(self, gotomark):
        return {
            "duration": gotomark.duration,
            "success": gotomark.success_flag
        }

    def print_navigation_goal(self, nav_goal):
        return {
            "duration": nav_goal.duration,
            "start_pose": JsonReportFormatter.NoIndent(nav_goal.start_pose),
            "end_pose": JsonReportFormatter.NoIndent(nav_goal.end_pose),
            "accuracy": JsonReportFormatter.NoIndent(nav_goal.accuracy),
            "success": nav_goal.success_flag,
            "avg_speed": nav_goal.avg_speed
        }

    def print_marking_goal(self, mrk_goal):
        return {
            "id": mrk_goal.corner_id,
            "marker_pose": JsonReportFormatter.NoIndent(mrk_goal.end_pose),
            "accuracy": JsonReportFormatter.NoIndent(mrk_goal.accuracy),
            "duration": mrk_goal.duration,
            "success" : mrk_goal.success_flag,
            "pic_name": mrk_goal.pic_name,
            "score": mrk_goal.score
        }
