#!/usr/bin/evn python3
# -*- coding: utf-8 -*-

""" read all rosout.log files in the folder. """

__author__ = "Jiancheng Zhang"

import sys
import os
import math

def chunks(logs_path, size=50000):
    count_line = 0
    count_file = 0
    chunk = []
    try:
        for root, dirs, files in os.walk(logs_path, topdown=False):
            for filename in files:
                if "rosout.log" in filename:
                    file_path = os.path.join(root, filename)
                    
                    with open(file_path, 'r', encoding='utf8') as file:
                        lines = file.readlines()
                        
                        count_file += 1
                        count_line += len(lines)
                        print("total length of file: ", file_path, "is: ",len(lines))
                        for i in range (size, len(lines)+size, size):
                            print(i)
                            chunk = lines[i-size: i+i//size]
                            yield chunk
                            chunk = []
        # # if still some data in the chunk after reading, return again
        # if len(chunk) != 0:
        #     yield chunk
    except IOError:
        print("\nCannot open the file ", file_path)
        sys.exit(1)
    except Exception as e:
        print(e)
        sys.exit(1)
    finally:
        print("how many files were read: ", count_file)
