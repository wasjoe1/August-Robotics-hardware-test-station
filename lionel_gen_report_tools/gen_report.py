#!/usr/bin/env python
# -*- coding: utf-8 -*-
from __future__ import print_function

import re
import sys
import os
from datetime import datetime

from boothbot_common.datacode import DataCode

HOME_PATH = os.environ.get('HOME')
CURRENT_PATH = os.getcwd()

REPORT_DIR = CURRENT_PATH + "/report_storage"
BACKUP_DIR = CURRENT_PATH + "/temp_storage"

GEN_REPORT_SCRIPT_DIR = HOME_PATH + '/catkin_ws/src/augustbot-tools/boothbot_simulation/scripts/'
GEN_REPORT_SCRIPT_PATH = HOME_PATH + '/catkin_ws/src/augustbot-tools/boothbot_simulation/scripts/generate_report.py'

sys.path.insert(0, GEN_REPORT_SCRIPT_DIR)

from report.report import Report
from report.json_formatter import JsonReportFormatter

CURRENT_TIME = datetime.now().strftime("%Y%m%d%H%M%S")
LOG_DIR = CURRENT_PATH + "/roslog_storage"+'/'+CURRENT_TIME

# Create necessary folders
def Initialize():
    try:
        os.mkdir(LOG_DIR)
    except Exception as e:
        print(e)
    try:
        os.mkdir(REPORT_DIR)
    except Exception as e:
        print(e)
    try:
        os.mkdir(LOG_DIR+'/'+CURRENT_TIME)
    except Exception as e:
        print(e)

def main():
    _current_time = datetime.now().strftime("%Y%m%d%H%M%S")
    _files_list = os.listdir(CURRENT_PATH)

    if not _files_list:
        print("Nothings exists in log directory....")
        return

    _files_list = [x for x in _files_list if ".log" in x]

    if not _files_list:
        print("Nothings exists in log directory....")
        return

    print("There is {} logs file due to process...".format(len(_files_list)))

    for f in _files_list:
        _f = CURRENT_PATH +'/' +f
        try:
            _report, _filename, _report_path = gen_report(_f)
            # Store report to file.
            # TODO: Use environment variables to store the correct path.
            target_dir = _report_path
            target_report_suffix = _filename
            target_report_path = target_dir + "/" + target_report_suffix + ".json"

            report_file = open(target_report_path, "w")
            report_file.write(_report)
            report_file.close()
            print("Report file path: " + target_report_path)

            os.rename(_f, LOG_DIR +'/' +f)
        except:
            pass


def gen_report(file_path=None):

    print("The current processing file: {}".format(file_path))

    file = None
    is_file_valid = False
    if not file_path:
        try:
            print("Please enter file path: ")
            file_path = raw_input()
        except KeyboardInterrupt:
            print("Exiting...")
            sys.exit()
        except Exception as e:
            print("1 - File not valid!")
            print(e)

    try:
        file = open(file_path, "r")
        is_file_valid = True
    except Exception as e:
        print("2 - File not valid!")
        print(e)

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

    _machine_name = str(report.boothbot_setup.machine_hostname)
    _task_start_time = "None"
    try:
        _task_start_time = str(datetime.fromtimestamp(report.gotomark.start_time).strftime('%d-%h-%y_%H:%M:%S'))
    except:
        pass
    FILENAME = _machine_name + "-" + _task_start_time

    _report_path = REPORT_DIR+'/'+_machine_name

    try:
        os.mkdir(_report_path)
    except:
        pass

    json_encoder = JsonReportFormatter()

    return json_encoder.encode(report), FILENAME, _report_path

if __name__ == "__main__":
    Initialize()
    main()
