#!/usr/bin/evn python3
# -*- coding: utf-8 -*-

""" write to a csv file. """

__author__ = "Jiancheng Zhang"

import datetime
import os
import sys
import csv

def write_to_csv(results, lionel_name, target_path):
    
    filename = str(lionel_name + "_Aging_Report_" + datetime.datetime.now().strftime('%d-%h-%y_%H-%M-%S')+".csv")
    filename = os.path.join(target_path, filename)
    
    if not os.path.exists(target_path):
        os.makedirs(target_path)
        
    if sys.version_info[0] == 3:
        with open(filename, "w", newline='') as file:
            writer = csv.writer(file)
            writer.writerows(results)
    elif sys.version_info[0] == 2:
        with open(filename, "w") as file:
            writer = csv.writer(file)
            writer.writerows(results)
            