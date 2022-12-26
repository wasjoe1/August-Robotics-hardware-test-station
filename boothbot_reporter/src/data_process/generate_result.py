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
    results = [['Name_of_Lionel', 'Name_of_GS', 'Date_of_aging', 'Starting_time', 'Ending_time',
               'Avg_localisation_duration', 'Avg_mark_duration', 'Avg_navigation_duration', 'Avg_localisation_times',
               'marks_per_hour', 'total_hours', 'total_localisation', 'total_mark', 'total_navigation']]
    
    for entity in entities:
        result = []
        
        result.append(entity.lionel_name)
        result.append(entity.gs_name)
        result.append(entity.date)
        result.append(entity.start_time)
        result.append(entity.end_time)
        result.append(cal_avg(entity.loc_data, lambda x: x["end_time"] - x["start_time"]))
        result.append(cal_avg(entity.mark_data, lambda x: x["end_time"] - x["start_time"]))
        result.append(cal_avg(entity.nav_data, lambda x: x["end_time"] - x["start_time"]))
        result.append(cal_avg(entity.nav_data, lambda x: x["nav_times_of_gs_loc"]))
        result.append(str(len(entity.mark_data) / (entity.total_time / 3600)))
        result.append(str(entity.total_time / 3600))
        result.append(str(len(entity.loc_data)))
        result.append(str(len(entity.mark_data)))
        result.append(str(len(entity.nav_data)))

        results.append(result)

    # the results contains all data rows
    return results


def cal_avg(list_dict, function):
    try:
        return str(sum(function(x) for x in list_dict) / len(list_dict))
    except ZeroDivisionError:
        return "0"
