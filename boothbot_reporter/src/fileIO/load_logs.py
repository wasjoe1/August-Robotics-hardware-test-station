#!/usr/bin/evn python3
# -*- coding: utf-8 -*-

""" read all rosout.log files in the folder. """

__author__ = "Jiancheng Zhang"

import sys
import os


def chunks(logs_path, size=50000):

    count_line = 0
    count_file = 0

    try:
        for root, dirs, files in os.walk(logs_path, topdown=True):
            files = [os.path.join(root, x) for x in files]
            files.sort(key=os.path.getmtime)
            for filename in files:
                if "rosout.log" in filename:
                    with open(filename, 'r', encoding='utf8') as file:
                        lines = file.readlines()
                        count_file += 1
                        count_line += len(lines)
                        print("total number of lines of the file:", filename, "is: ", len(lines))
                        for i in range(0, len(lines), size):
                            yield lines[i: i + size]

    except IOError:
        print("\nCannot open the file ", filename)
        sys.exit(1)
    except Exception as e:
        print(e)
        sys.exit(1)
    finally:
        print("how many files were read: ", count_file)
        print("how many lines were read: ", count_line)
