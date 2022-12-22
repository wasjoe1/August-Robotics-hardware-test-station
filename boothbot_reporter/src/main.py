#!/usr/bin/evn python3
# -*- coding: utf-8 -*-

""" Main, the entrance of the program. """

__author__ = "Jiancheng Zhang"

import sys
import os
import getopt

from fileIO import load_logs

def help():
    print("#############################################################################################################\n")
    print("options:")
    print("\t-p  --localpath:    give a path you want the result .csv file save to, or just using the default (Optional)")
    print("\t-o  --copyremotely: do you want to copy to a remote computer, y/n? (Optional)")
    print("\t-r  --remotename:   name of the remote computer (necessary if you want to copy to a remote computer)")
    print("\t-k  --remotePW:     password of the remote computer (necessary if you want to copy to a remote computer)")
    print("\t-n  --remotepath:   destination path of the remote computer (Optional)")
    print("\t-h  --help:         ask for help")
    print("\t-q  --quit:         exit immediately\n")
    print("#############################################################################################################")

HOME_PATH = os.environ.get('HOME')
DEFAULT_LOGS_PATH = os.path.join(HOME_PATH, 'roslogs/')

if __name__ == "__main__":
    try:
        opts, args = getopt.getopt(sys.argv[1:], 
                                   'p:o:r:k:n:hq',
                                   ['localpath=', 'copyremotely=', 'remotename=', 'remotePW=','remotepath=',  'help', 'quit'])
        
        localpath = os.path.join(HOME_PATH + 'catkin_ws/local/report/')
        copyremotely = False
        remotename = ""
        remotePW = ""
        remotepath = "~/"
        
        for opt_name, opt_value in opts:
            if opt_name in ('-p', '--localpath'):
                localpath = opt_value
                
            elif opt_name in ('-o', '--copyremotely'):
                if opt_value == "y":
                    copyremotely = True
                elif opt_value == "n":
                    copyremotely = False
                else:
                    print("Unrecongnised value of -o, please type y or n")
                    sys.exit(1)
                
            elif opt_name in ('-r', '--remotename'):
                if copyremotely is True:
                    remotename = opt_value
                else:
                    print("copyremotely is False, but remote computer information is given")
                    sys.exit(1)
                    
            elif opt_name in ('-k', '--remotePW'):
                if copyremotely is True:
                    remotePW = opt_value
                else:
                    print("copyremotely is False, but remote computer information is given")
                    sys.exit(1)
                    
            elif opt_name in ('-n', '--remotepath'):
                if copyremotely is True:
                    remotepath = opt_value
                else:
                    print("copyremotely is False, but remote computer information is given")
                    sys.exit(1)
                    
            elif opt_name in ('-h', '--help'):
                help()
                sys.exit()
                
            elif opt_name in ('-q', '--quit'):
                sys.exit()
        
    except getopt.GetoptError as error:
        print(error, "\n")
        help()
        sys.exit(1)
        
    for chunk in load_logs.chunks(DEFAULT_LOGS_PATH):
        print("done")