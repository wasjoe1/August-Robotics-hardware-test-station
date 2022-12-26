#!/usr/bin/evn python3
# -*- coding: utf-8 -*-

""" write to a csv file. """

__author__ = "Jiancheng Zhang"

import datetime
import sys
import os
import csv

def write_to_csv(results, target_path):
    filename = str("Lionel_Aging_Report_" + datetime.datetime.now().strftime('%d-%h-%y_%H:%M:%S')+".csv")
    filename = os.path.join(target_path, filename)
    if not os.path.exists(target_path):
        os.makedirs(target_path)
    with open(filename, "w", newline='') as file:
        writer = csv.writer(file)
        writer.writerows(results)