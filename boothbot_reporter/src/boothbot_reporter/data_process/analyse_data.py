#!/usr/bin/evn python3
# -*- coding: utf-8 -*-

""" filter raw data from roslogs files and use datacode to classify them. """

__author__ = "Jiancheng Zhang"

import re
import datetime
from boothbot_common.datacode import DataCode
from boothbot_reporter.data_process.entity_data import Entity

data_lines = []


def partition(pred, iterable):
    trues = []
    falses = []
    for item in iterable:
        if pred(item):
            trues.append(item)
        else:
            falses.append(item)
    return trues, falses


def filter_DATA(raw_lines):
    global data_lines
    results = list(filter(lambda x: "[DATA]" in x, raw_lines))
    data_lines = data_lines + results
    return data_lines


def filter_ERROR(DATA_lines):
    all_error, no_error = partition(lambda x: "[ERROR_CODE]" in x or "[ERROR]" in x, DATA_lines)
    return list(all_error), list(no_error)


def analyse_data(DATA_lines):
    global data_lines
    all_error_lines, no_error_lines = filter_ERROR(DATA_lines)

    # we will return a set of entities
    entities = []

    # class varaibles
    lionel_name = None
    gs_name = None
    communication_method = None
    version = None
    gotomark_map_id = None
    date = None
    start_time = None
    end_time = None

    navigation_entities = []
    navigation_entity = {"start_time": None, "end_time": None, "nav_goal_succeeded": None,
                         "nav_times_of_gs_loc": None}

    mark_entities = []
    mark_entity = {"start_time": None, "end_time": None, "mark_goal_succeeded": None}

    localisation_entities = []
    localisation_entity = {"start_time": None, "end_time": None, "loc_goal_succeeded": None}

    movement_entities = []
    movement_entity = {"start_time": None, "end_time": None, "move_goal_succeeded": None}

    get_goal_entities = []
    get_goal_entity = {"start_time": None, "end_time": None, "get_goal_succeeded": None}

    switch_map_entities = []
    switch_map_entity = {"start_time": None, "end_time": None, "switch_map_succeeded": None}

    datacode_regex = "\[(([A-Z]+_?)+)\]"
    data_pattern = re.compile("^(.*\[DATA\] " + datacode_regex + " (.*))$")
    for line in no_error_lines:
        # match datacode
        match = data_pattern.match(line)
        code = DataCode.code2enum(match.group(2))
        data = code.parse(match.group(4))

        if code == DataCode.MACHINE_HOSTNAME:
            lionel_name = data
        elif code == DataCode.GS_NAME:
            gs_name = data
        elif code == DataCode.GIT_INFO:
            version = data
        elif code == DataCode.GRPC_CONNECTION_TYPE:
            communication_method = data
        elif code == DataCode.GOTOMARK_MAP_ID:
            gotomark_map_id = data
        elif code == DataCode.GOTOMARK_START_TIME:
            start_time = data
            date = datetime.datetime.utcfromtimestamp(data).strftime('%Y-%m-%d')
            # reset data sets
            get_goal_entities = []
            navigation_entities = []
            movement_entities = []
            localisation_entities = []
            mark_entities = []
            switch_map_entities = []
        elif code == DataCode.GOTOMARK_END_TIME:
            end_time = datetime.datetime.utcfromtimestamp(data).strftime('%H:%M:%S')
            total_time = data - start_time
            start_time = datetime.datetime.utcfromtimestamp(start_time).strftime('%H:%M:%S')
            # construct the object of Entity class
            entity = Entity(lionel_name, gs_name, communication_method, version, gotomark_map_id, date, start_time, end_time,
                            total_time)
            entity.nav_data = navigation_entities
            entity.mark_data = mark_entities
            entity.loc_data = localisation_entities
            entity.move_data = movement_entities
            entity.get_goal_data = get_goal_entities
            entity.switch_map_data = switch_map_entities
            get_goal_entities = []
            navigation_entities = []
            movement_entities = []
            localisation_entities = []
            mark_entities = []
            switch_map_entities = []
            entities.append(entity)

        if code == DataCode.GET_GOAL_START_TIME:
            get_goal_entity = dict.fromkeys(get_goal_entity, None)
            get_goal_entity["start_time"] = data
        elif code == DataCode.GET_GOAL_SUCCEEDED:
            get_goal_entity["get_goal_succeeded"] = data
        elif code == DataCode.GET_GOAL_END_TIME:
            get_goal_entity["end_time"] = data
            get_goal_entities.append(get_goal_entity)
            get_goal_entity = dict.fromkeys(get_goal_entity, None)

        if code == DataCode.NAV_GOAL_START_TIME:
            navigation_entity = dict.fromkeys(navigation_entity, None)
            navigation_entity["start_time"] = data
        elif code == DataCode.NAV_GOAL_SUCCEEDED:
            navigation_entity["nav_goal_succeeded"] = data
        elif code == DataCode.NAV_TIMES_OF_GS_LOC:
            navigation_entity["nav_times_of_gs_loc"] = data
        elif code == DataCode.NAV_GOAL_END_TIME:
            navigation_entity["end_time"] = data
            navigation_entities.append(navigation_entity)
            navigation_entity = dict.fromkeys(navigation_entity, None)

        if code == DataCode.MRK_GOAL_START_TIME:
            mark_entity = dict.fromkeys(mark_entity, None)
            mark_entity["start_time"] = data
        elif code == DataCode.MRK_GOAL_SUCCEEDED:
            mark_entity["mark_goal_succeeded"] = data
        elif code == DataCode.MRK_GOAL_END_TIME:
            mark_entity["end_time"] = data
            mark_entities.append(mark_entity)
            mark_entity = dict.fromkeys(mark_entity, None)

        if code == DataCode.LOC_GOAL_START_TIME:
            localisation_entity = dict.fromkeys(localisation_entity, None)
            localisation_entity["start_time"] = data
        elif code == DataCode.LOC_GOAL_SUCCEEDED:
            localisation_entity["loc_goal_succeeded"] = data
        elif code == DataCode.LOC_GOAL_END_TIME:
            localisation_entity["end_time"] = data
            localisation_entities.append(localisation_entity)
            localisation_entity = dict.fromkeys(localisation_entity, None)

        if code == DataCode.MOVING_START_TIME:
            movement_entity = dict.fromkeys(movement_entity, None)
            movement_entity["start_time"] = data
        elif code == DataCode.MOVING_SUCCEEDED:
            movement_entity["move_goal_succeeded"] = data
        elif code == DataCode.MOVING_END_TIME:
            movement_entity["end_time"] = data
            movement_entities.append(movement_entity)
            movement_entity = dict.fromkeys(movement_entity, None)

        if code == DataCode.SWITCH_MAP_START_TIME:
            switch_map_entity = dict.fromkeys(switch_map_entity, None)
            switch_map_entity["start_time"] = data
        elif code == DataCode.SWITCH_MAP_SUCCEEDED:
            switch_map_entity["switch_map_succeeded"] = data
        elif code == DataCode.SWITCH_MAP_END_TIME:
            switch_map_entity["end_time"] = data
            switch_map_entities.append(switch_map_entity)
            switch_map_entity = dict.fromkeys(switch_map_entity, None)

    data_lines = []
    return entities
