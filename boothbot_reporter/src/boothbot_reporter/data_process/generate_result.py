#!/usr/bin/evn python3
# -*- coding: utf-8 -*-

""" iterate over all entities, and do some math """

__author__ = "Jiancheng Zhang"


def generate_result(entities):
    """
    this method will generate result based on entities.
    if you want to add more type of results by using current data that an entity hold,
    you can only edit this method.

    Args:
        entities (Entity): a list of entities

    Returns:
        _type_: data rows
    """
    # the header
    results = [['order', 'goto_mark_start', 'switch_map', '', 'navigating', '', 'goto_mark_end'],
               [
                   '', '', '', 'moving', 'locating', 'marking'
               ],
               [
                   #add a blank line
               ],
               [
                   'Name_of_Lionel',
                   'Name_of_GS',
                   'Communication_method',
                   'Version',
                   'Date_of_aging',
                   'Starting_time',
                   'Ending_time',

                   'marks/hour',
                   'seconds/mark_goal',

                   'GOTO_MARK_start',
                   'Pull_submap_done',
                   'Moving_done',
                   'Locationg_done',
                   'Marking_done',
                   'time_diff',
                   'Separating Rows',

                   'seconds/navigation',
                   'seconds/move',

                   'seconds/marking',
                   'seconds/submap',
                   'seconds/localisation',
                   'localisation_times/nav',

                   'total_hours',
                   'total_localisation_times',
                   'loc_fail_times',
                   'total_mark_times',
                   'mark_fail_times',
                   'total_move_times',
                   'move_fail_times',
                   'total_navigation_times',
                   'navigation_fail_times',
                   'total_number_of_pull_submap',
                   'Map_ID',
               ]]

    lionel_name = ""
    for entity in entities:
        result = []
        # use zero as the start time of 1 round
        time_line = 0

        result.append(entity.lionel_name)
        result.append(entity.gs_name)
        result.append(entity.communication_method)
        result.append(entity.version)
        result.append(entity.date)
        result.append(entity.start_time)
        result.append(entity.end_time)

        result.append(str(len(entity.mark_data) / (entity.total_time / 3600)))
        result.append(str(cal_avg(entity.nav_data, lambda x: x["end_time"] - x["start_time"]) +
                          cal_avg(entity.mark_data, lambda x: x["end_time"] - x["start_time"])))

        result.append(str(time_line))
        time_line = cal_avg(entity.submap_data, lambda x: x["end_time"] - x["start_time"]) + time_line
        result.append(str(time_line))
        time_line = cal_avg(entity.move_data, lambda x: x["end_time"] - x["start_time"]) + time_line
        result.append(str(time_line))
        time_line = cal_avg(entity.loc_data, lambda x: x["end_time"] - x["start_time"]) + time_line
        result.append(str(time_line))
        time_line = cal_avg(entity.mark_data, lambda x: x["end_time"] - x["start_time"]) + time_line
        result.append(str(time_line))
        result.append(str(cal_diff(entity)))
        result.append('')

        result.append(str(cal_avg(entity.nav_data, lambda x: x["end_time"] - x["start_time"])))
        result.append(str(cal_avg(entity.move_data, lambda x: x["end_time"] - x["start_time"])))

        result.append(str(cal_avg(entity.mark_data, lambda x: x["end_time"] - x["start_time"])))
        result.append(str(cal_avg(entity.submap_data, lambda x: x["end_time"] - x["start_time"])))
        result.append(str(cal_avg(entity.loc_data, lambda x: x["end_time"] - x["start_time"])))
        result.append(str(cal_avg(entity.nav_data, lambda x: x["nav_times_of_gs_loc"])))

        result.append(str(entity.total_time / 3600))
        result.append(str(len(entity.loc_data)))
        result.append(str(cal_fail_count(entity.loc_data, lambda x: x["loc_goal_succeeded"] == False)))
        result.append(str(len(entity.mark_data)))
        result.append(str(cal_fail_count(entity.mark_data, lambda x: x["mark_goal_succeeded"] == False)))
        result.append(str(len(entity.move_data)))
        result.append(str(cal_fail_count(entity.move_data, lambda x: x["move_goal_succeeded"] == False)))
        result.append(str(len(entity.nav_data)))
        result.append(str(cal_fail_count(entity.nav_data, lambda x: x["nav_goal_succeeded"] == False)))
        result.append(str(len(entity.submap_data)))
        result.append(entity.gotomark_map_id)


        results.append(result)

        lionel_name = str(entity.lionel_name)
    # the results contain all data rows
    return results, lionel_name


def cal_avg(list_dict, function):
    try:
        return sum(function(x) for x in list_dict) / len(list_dict)
    except ZeroDivisionError:
        return 0


def cal_sum(list_dict, function):
    try:
        return sum(function(x) for x in list_dict)
    except ZeroDivisionError:
        return 0


def cal_diff(log_data):
    mark_time = cal_sum(log_data.mark_data, lambda x: x['end_time'] - x["start_time"])
    submap_time = cal_sum(log_data.submap_data, lambda x: x["end_time"] - x["start_time"])
    loc_time = cal_sum(log_data.loc_data, lambda x: x["end_time"] - x["start_time"])
    move_time = cal_sum(log_data.move_data, lambda x: x["end_time"] - x["start_time"])
    time_diff = log_data.total_time - submap_time - move_time - loc_time - mark_time

    return time_diff


def cal_fail_count(list_dict, function):
    try:
        return sum(function(x) for x in list_dict)
    except ZeroDivisionError:
        return 0
