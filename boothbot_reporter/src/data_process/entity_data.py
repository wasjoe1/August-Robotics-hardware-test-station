#!/usr/bin/evn python3
# -*- coding: utf-8 -*-

""" defined entity class. each entity represent a data row in the final csv file. 
    the entity holds the raw data we need """

__author__ = "Jiancheng Zhang"

class Entity:
    def __init__(self, lionel_name, gs_name, date, start_time, end_time, total_time):
        self.lionel_name = lionel_name
        self.gs_name = gs_name
        self.date = date
        self.start_time = start_time
        self.end_time = end_time
        self.total_time = total_time
        self.nav_data = []
        self.mark_data = []
        self.loc_data = []
