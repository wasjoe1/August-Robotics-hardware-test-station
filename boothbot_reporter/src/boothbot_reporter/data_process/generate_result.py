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
    results = [[
        'Name_of_Lionel',
        'Name_of_GS',
        'Communication_method',
        'Date_of_aging',
        'Starting_time',
        'Ending_time',

        'marks/hour',
        'seconds/mark_goal',

        'seconds/navigation',
        'seconds/moving',

        'seconds/marking',
        'seconds/submap',
        'seconds/localisation',
        'localisation_times/nav',

        'total_hours',
        'total_localisation',
        'total_mark',
        'total_moving',
        'total_navigation',
        'total_number_of_pull_submap',
        'Map_ID',
        'time_diff'
    ]]

    lionel_name = ""
    for entity in entities:
        result = []

        result.append(entity.lionel_name)
        result.append(entity.gs_name)
        result.append(entity.communication_method)
        result.append(entity.date)
        result.append(entity.start_time)
        result.append(entity.end_time)

        result.append(str(len(entity.mark_data) / (entity.total_time / 3600)))
        result.append(str(cal_avg(entity.nav_data, lambda x: x["end_time"] - x["start_time"]) +
                          cal_avg(entity.mark_data, lambda x: x["end_time"] - x["start_time"])))

        result.append(str(cal_avg(entity.nav_data, lambda x: x["end_time"] - x["start_time"])))
        result.append(str(cal_avg(entity.move_data, lambda x: x["end_time"] - x["start_time"])))

        result.append(str(cal_avg(entity.mark_data, lambda x: x["end_time"] - x["start_time"])))
        result.append(str(cal_avg(entity.submap_data, lambda x: x["end_time"] - x["start_time"])))
        result.append(str(cal_avg(entity.loc_data, lambda x: x["end_time"] - x["start_time"])))
        result.append(str(cal_avg(entity.nav_data, lambda x: x["nav_times_of_gs_loc"])))

        result.append(str(entity.total_time / 3600))
        result.append(str(len(entity.loc_data)))
        result.append(str(len(entity.mark_data)))
        result.append(str(len(entity.move_data)))
        result.append(str(len(entity.nav_data)))
        result.append(str(len(entity.submap_data)))
        result.append(entity.gotomark_map_id)
        result.append(str(cal_diff(entity)))

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


def cal_diff(entity):
    return str((entity.total_time / 3600) - cal_sum(entity.mark_data, lambda x: x['end_time']
                                                                         - x["start_time"]) - cal_sum(
        entity.loc_data, lambda x: x["end_time"] - x["start_time"]) -
        cal_sum(entity.submap_data, lambda x: x["end_time"] - x["start_time"]) - cal_sum(entity.move_data,
                                                                                         lambda x: x["end_time"] - x[
                                                                                             "start_time"]))
