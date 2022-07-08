#!/usr/bin/env python
# -*- coding: utf-8 -*-
from __future__ import print_function
from importlib import import_module

import re
import sys
import os
import paramiko
from datetime import datetime

import subprocess

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
_LOG_DIR = CURRENT_PATH + "/roslog_storage"
LOG_DIR = _LOG_DIR+'/'+CURRENT_TIME

REMOTE_LOGS_PATH = "/home/augbooth/catkin_ws/local/roslogs"
TEMP_FILE = CURRENT_PATH+'/temp.log'

# subprocess.call(["ssh", "augbooth@lsim-0010.local", "find", "/home/augbooth/catkin_ws/local/roslogs/*/rosout.log" ,"-type", "f", "-newermt", "'1/07/2022'"])
# a = subprocess.Popen(["ssh", "augbooth@lsim-0010.local", "find", "/home/augbooth/catkin_ws/local/roslogs/*/rosout.log" ,"-type", "f", "-newermt", "'1/07/2022'"])

def get_input():
    supported_hostname = ["lnp5","lcp3"]

    _hostname_valid = False
    _start_date_valid = False
    _end_date_valid = False

    while not (_hostname_valid):
        try:
            print("Please enter hostname path: [currently only support LCP3 and LNP5 e.g. lnp5-0010]")
            _hostanme = str(raw_input())
            for k in supported_hostname:
                if k in _hostanme:
                    _hostname_valid = True
                    break
            if not _hostname_valid:
                _hostanme = None
                print('Input hostname not valid, please try again')
        except KeyboardInterrupt:
            print("Exiting...")
            sys.exit()
        except Exception as e:
            print("Input not valid!")
            print(e)

    while not (_start_date_valid):
        try:
            print("Please enter start date [yyyymmdd] (e.g. 20220701):")
            _start_date = str(raw_input())
            if len(_start_date) == 8:
                _start_date_valid = True
            if not _start_date_valid:
                _start_date = None
                print('Input start data not valid')
                raise KeyboardInterrupt
        except KeyboardInterrupt:
            print("Exiting...")
            sys.exit()
        except Exception as e:
            print("Input not valid!")
            print(e)

    while not (_end_date_valid):
        try:
            print("Please enter end date [yyyymmdd]:")
            _end_date = str(raw_input())
            if _end_date == '':
                _end_date = -1
            if _end_date == -1:
                _end_date_valid = True
            elif len(_end_date) == 8:
                _end_date_valid = True
            if not _end_date_valid:
                print('Input end data not valid')
                raise KeyboardInterrupt
        except KeyboardInterrupt:
            print("Exiting...")
            sys.exit()
        except Exception as e:
            print("Input not valid!")
            print(e)

    return _hostanme, _start_date, _end_date

# def test_remote(_server):
#     if not (_server):
#         print("ERROR!")
#         return
#     _server = str(_server) + '.local'
#     ssh = paramiko.SSHClient()
#     ssh.load_host_keys(os.path.expanduser(os.path.join("~",".ssh","known_hosts")))
#     ssh.connect(_server, username='augbooth', password='aug')
#     sftp = ssh.open_sftp()
#     sftp.get(CURRENT_PATH, remotepath)

def get_valid_logs_name(_remote_hostname,start_date, end_date):

    _start_date = '{0:04d}-{1:02d}-{2:02d}'.format(int(start_date[:4]),int(start_date[4:6]),int(start_date[6:8]))

    if end_date == -1:
        _temp = subprocess.Popen(["ssh", "augbooth@"+_remote_hostname+".local", "find", "/home/augbooth/catkin_ws/local/roslogs/*/rosout*" ,"-type", "f", "-newermt", _start_date],stdout=subprocess.PIPE)
    else:
        _end_date = '{0:04d}-{1:02d}-{2:02d}'.format(int(end_date[:4]),int(end_date[4:6]),int(end_date[6:8]))
        _temp = subprocess.Popen(["ssh", "augbooth@"+_remote_hostname+".local", "find", "/home/augbooth/catkin_ws/local/roslogs/*/rosout*" ,"-type", "f", "-newermt", _start_date, "!", "-newermt", _end_date],stdout=subprocess.PIPE)

    _files_list = _temp.communicate()[0].split('\n')

    _files_list = [x for x in _files_list if not "stdout" in x]
    files_list = [x for x in _files_list if ".log" in x]

    return files_list

def get_logs_from_remote(_remote_hostname, _files_list):
    if not (_remote_hostname):
        print("ERROR!")
        return
    _server = str(_remote_hostname) + '.local'
    print("There is {} file due to copy...".format(len(_files_list)))

    for f in _files_list:
        print("Now copying from remote: {}".format(f))
        subprocess.call(["touch", TEMP_FILE])
        subprocess.call(["scp",  "augbooth@"+_server+":"+f, TEMP_FILE])

        _c_d = subprocess.Popen(["ssh", "augbooth@"+_remote_hostname+".local", "ls", "-la", f, "--time-style=full-iso", "|" ,"awk" ,"'{print $6" "$7}'"],stdout=subprocess.PIPE)

        _creation = _c_d.communicate()[0].replace('\n','')
        _creation = _creation.replace('\n','').replace(' ','-').replace(':','-').split('.')[0]
        _creation = '{}_{}'.format(_creation[:10],_creation[10:-1])

        # _creation = datetime.datetime.fromtimestamp(os.path.getctime(TEMP_FILE)).strftime("%Y%m%d%H%M%S")
        # _creation = datetime.datetime.fromtimestamp(os.path.getctime(CURRENT_PATH+'/rosout.log')).strftime("%Y%m%d%H%M%S")

        _filename = CURRENT_PATH + '/'+_creation+'_'+str(_remote_hostname)+".log"

        os.rename(TEMP_FILE, _filename)

def filter_files():
    _files_list = os.listdir(CURRENT_PATH)

    if not _files_list:
        print("Nothings exists in log directory....")
        return

    _files_list = [x for x in _files_list if ".log" in x]

    if not _files_list:
        print("Nothings exists in log directory....")
        return

    return _files_list

if __name__ == "__main__":
    _h, _s, _e = get_input()
    _filename_list = get_valid_logs_name(_h,_s,_e)
    get_logs_from_remote(_h,_filename_list)
