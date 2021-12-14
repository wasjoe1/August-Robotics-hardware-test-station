#!/usr/bin/env python
# -*- coding: utf-8 -*-

from __future__ import print_function

import re
import sys

from boothbot_common.datacode import DataCode
from report.report import Report
from report.json_formatter import JsonReportFormatter

import os
from datetime import datetime

# Env. variable from os
HOME_PATH = os.environ.get('HOME')
GIT_HEAD = os.environ.get('GIT_HEAD')
YMD = os.environ.get('TODAY')

if GIT_HEAD is not None and YMD is not None:
    REPORT_PATH = HOME_PATH + "/Simulation_Reports/" + YMD + "-" + GIT_HEAD
    # try:
    #     os.mkdir(HOME_PATH + "/Simulation_Reports/" + YMD + "-" + GIT_HEAD)
    # except OSError:
    #     pass
else:
    REPORT_PATH = HOME_PATH + "/Simulation_Reports/UnknownBranch"
try:
    os.mkdir(REPORT_PATH)
except OSError:
    pass

def main():
    file = None
    file_path = None
    is_file_valid = False
    while not is_file_valid:
        try:
            print("Please enter file path: ")
            file_path = raw_input()
            file = open(file_path, "r")
            is_file_valid = True
        except KeyboardInterrupt:
            print("Exiting...")
            sys.exit()
        except:
            print("File not valid!")

    report = Report()
    datacode_regex = "\[(([A-Z]+_?)+)\]"
    data_pattern = re.compile("^(.*\[DATA\] " + datacode_regex + " (.*))$")
    for line in file:
        match = data_pattern.match(line)
        if match is None:
            continue

        code = DataCode.code2enum(match.group(2))
        data = code.parse(match.group(4))
        if data is None:
            report.logerr("Type check failed. Data will be ignored: \n\t {}".format(line))
            continue
        report.add_data(code, data)

    file.close()

    json_encoder = JsonReportFormatter()
    return json_encoder.encode(report)

if __name__ == "__main__":
    report = main()

    # Store report to file.
    # TODO: Use environment variables to store the correct path.
    target_dir = REPORT_PATH #"../reports"
    target_report_suffix = datetime.now().strftime("%Y%m%d%H%M%S")
    target_report_path = target_dir + "/report-" + target_report_suffix + ".json"

    report_file = open(target_report_path, "w")
    report_file.write(report)
    report_file.close()
    print("Report file path: " + target_report_path)

