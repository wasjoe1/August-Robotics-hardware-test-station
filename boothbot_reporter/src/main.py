#!/usr/bin/evn python3
# -*- coding: utf-8 -*-

""" Main, the entrance of the program. """

__author__ = "Jiancheng Zhang"

import sys
import os
import getopt

from fileIO import load_logs, write_csv
from data_process import analyse_data, generate_result


def help():
    print("#############################################################################################################\n")
    print("options:")
    print("\t-p  --localpath:    give a path you want the result .csv file save to, or just using the default (Optional)")
    print("\t-h  --help:         ask for help")
    print("\t-q  --quit:         exit immediately\n")
    print("#############################################################################################################")

HOME_PATH = os.environ.get('HOME')
DEFAULT_LOGS_PATH = os.path.join(HOME_PATH, 'catkin_ws/local/roslogs/')

if __name__ == "__main__":
    try:
        opts, args = getopt.getopt(sys.argv[1:], 
                                   'p:hq',
                                   ['localpath=', 'help', 'quit'])
        
        localpath = os.path.join(HOME_PATH, 'catkin_ws/local/report/')
        
        for opt_name, opt_value in opts:
            if opt_name in ('-p', '--localpath'):
                localpath = opt_value
                    
            elif opt_name in ('-h', '--help'):
                help()
                sys.exit()
                
            elif opt_name in ('-q', '--quit'):
                sys.exit()
        
    except getopt.GetoptError as error:
        print(error, "\n")
        help()
        sys.exit(1)


    data_lines = []
    for chunk in load_logs.chunks(DEFAULT_LOGS_PATH):
        data_lines = analyse_data.filter_DATA(chunk)

    entities = analyse_data.analyse_data(data_lines)
    results = generate_result.generate_result(entities)
    write_csv.write_to_csv(results, localpath)