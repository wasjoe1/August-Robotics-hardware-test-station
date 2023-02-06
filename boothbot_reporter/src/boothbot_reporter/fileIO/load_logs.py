#!/usr/bin/evn python3
# -*- coding: utf-8 -*-

""" read all rosout.log files in the folder. """

__author__ = "Jiancheng Zhang"

import sys
import os


def chunks(logs_path, size=50000):
    """
    due to the roslogs files can be large, we read the files chunk by chunk in order to avoid OOM error.

    Args:
        logs_path (_type_): a default path: catkin_ws/local/roslogs/
        size (int, optional): can be changed according to memory size. Defaults to 50000.

    Yields:
        _type_: chunk of lines
    """
    count_line = 0
    count_file = 0

    try:
        for root, dirs, files in os.walk(logs_path, topdown=True):
            # read in date of modified order, i.e. the latest modified file is the last to read
            # because if there are more than one rosout.log file, e.g. rosout.log.1, 
            # some infomation only shows in the oldest log file, like machine name.
            # so make sure the reading order is same as the log written order
            files = [os.path.join(root, x) for x in files]
            files.sort(key=os.path.getmtime)
            
            for filename in files:
                if "rosout.log" in filename:
                    if sys.version_info[0] == 3:
                        with open(filename, 'r', encoding='utf8', errors='ignore') as file:
                            lines = file.readlines()
                            count_file += 1
                            count_line += len(lines)
                            print("total number of lines of the file:", filename, "is: ", len(lines))
                            for i in range(0, len(lines), size):
                                yield lines[i: i + size]
                    elif sys.version_info[0] ==2:
                        with open(filename, 'r') as file:
                            lines = file.readlines()
                            count_file += 1
                            count_line += len(lines)
                            print("total number of lines of the file:", filename, "is: ", len(lines))
                            for i in range(0, len(lines), size):
                                yield lines[i: i + size]

    except IOError:
        print("\nCannot open the file ", filename)
    except Exception as e:
        print(e)
        sys.exit(1)
    finally:
        print("How many files were read: ", count_file)
        print("How many lines were read: ", count_line)
