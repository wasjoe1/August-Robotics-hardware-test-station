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
    base_columns = [''] * 10
    count_columns = 3
    mainmoduleline = base_columns + ['GOTOMARK']
    submoduleline = base_columns + ['']
    minmoduleline = base_columns + ['']
    submodule = {
        'GET_GOAL': 2,
        'NAV': 3,
        'MARK': 1,
    }#key as submodule,value is count of next level module of this submodule + 1
    minmodule = {
        'SWITCH_MAP': 0,
        'MOVE': 0,
        'LOCATE': -1,
    }#key as minmodule,key = 0 means is the first minmodule of the submodule,others' key use whatever except 0
    #and the order must follow the order of submodule
    for i in submodule.keys():
        submoduleline += ['{}'.format(i)] + [''] * (count_columns * submodule[i] - 1)
    for i in minmodule.keys():
        if minmodule[i] == 0:
            minmoduleline += [''] * 3 + ['{}'.format(i)] + [''] * 2
        else:
            minmoduleline += ['{}'.format(i)] + [''] * 2
    count_list = ['Ratio', 'Total', 'Fail']
    comment1 = "Mark's time partially overlaps with the time of get_goal"
    comment2 = "The first time of SWITCH_MAP is not in the GET_GOAL"
    comment3 = "time diff is use Total time of GOTO - Total time of get_goal" \
               ",move,locate and mark"

    results = [
        ['comments:', '{}'.format(comment1), '{}'.format(comment2), '{}'.format(comment3)],
        mainmoduleline,
        submoduleline,
        minmoduleline,
        [
            'Name_of_Lionel',
            'Name_of_GS',
            'Map_ID',
            'Communication_method',
            'Version',
            'Date_of_aging',
            'Starting_time',
            'total_hours',
            'marks/hour',
            'seconds/mark_goal',
            ''
        ]
        + count_list * (len(submodule) + len(minmodule))
        + ['GOTO_MARK_start',
           'Get_goal_done',
           'Moving_done',
           'Locationg_done',
           'Marking_done',
           'time_diff'
           ]
    ]

    lionel_name = ""
    for entity in entities:
        result = []
        # use zero as the start time of 1 round
        time_line = 0

        result.append(entity.lionel_name)
        result.append(entity.gs_name)
        result.append(entity.gotomark_map_id)
        result.append(entity.communication_method)
        result.append(entity.version)
        result.append(entity.date)
        result.append(entity.start_time)
        result.append(str(entity.total_time / 3600))
        result.append(str(len(entity.mark_data) / (entity.total_time / 3600)))
        result.append(str(cal_avg(entity.nav_data, lambda x: x["end_time"] - x["start_time"]) +
                          cal_avg(entity.mark_data, lambda x: x["end_time"] - x["start_time"])))
        result.append('')

        result.append(str(cal_avg(entity.get_goal_data, lambda x: x["end_time"] - x["start_time"])))
        result.append(str(len(entity.get_goal_data)))
        result.append(str(cal_fail_count(entity.get_goal_data, lambda x: x["get_goal_succeeded"] == False)))

        result.append(str(cal_avg(entity.switch_map_data, lambda x: x["end_time"] - x["start_time"])))
        result.append(str(len(entity.switch_map_data)))
        result.append(str(cal_fail_count(entity.switch_map_data, lambda x: x["switch_map_succeeded"] == False)))

        result.append(str(cal_avg(entity.nav_data, lambda x: x["end_time"] - x["start_time"])))
        result.append(str(len(entity.nav_data)))
        result.append(str(cal_fail_count(entity.nav_data, lambda x: x["nav_goal_succeeded"] == False)))

        result.append(str(cal_avg(entity.move_data, lambda x: x["end_time"] - x["start_time"])))
        result.append(str(len(entity.move_data)))
        result.append(str(cal_fail_count(entity.move_data, lambda x: x["move_goal_succeeded"] == False)))

        result.append(str(cal_avg(entity.loc_data, lambda x: x["end_time"] - x["start_time"])))
        result.append(str(len(entity.loc_data)))
        result.append(str(cal_fail_count(entity.loc_data, lambda x: x["loc_goal_succeeded"] == False)))

        result.append(str(cal_avg(entity.mark_data, lambda x: x["end_time"] - x["start_time"])))
        result.append(str(len(entity.mark_data)))
        result.append(str(cal_fail_count(entity.mark_data, lambda x: x["mark_goal_succeeded"] == False)))

        result.append(str(time_line))
        time_line += cal_avg(entity.get_goal_data, lambda x: x["end_time"] - x["start_time"])
        result.append(str(time_line))
        time_line += cal_avg(entity.move_data, lambda x: x["end_time"] - x["start_time"])
        result.append(str(time_line))
        time_line += cal_avg(entity.loc_data, lambda x: x["end_time"] - x["start_time"])
        result.append(str(time_line))
        time_line += cal_avg(entity.mark_data, lambda x: x["end_time"] - x["start_time"])
        result.append(str(time_line))
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


def cal_diff(log_data):
    mark_time = cal_sum(log_data.mark_data, lambda x: x['end_time'] - x["start_time"])
    get_goal_time = cal_sum(log_data.get_goal_data, lambda x: x["end_time"] - x["start_time"])
    loc_time = cal_sum(log_data.loc_data, lambda x: x["end_time"] - x["start_time"])
    move_time = cal_sum(log_data.move_data, lambda x: x["end_time"] - x["start_time"])
    time_diff = log_data.total_time - get_goal_time - move_time - loc_time - mark_time

    return time_diff


def cal_fail_count(list_dict, function):
    try:
        return sum(function(x) for x in list_dict)
    except ZeroDivisionError:
        return 0
