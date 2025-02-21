#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import csv
import math
import os
import shutil
import re
import sys

from get_file import Get_file


class Get8Dir():
    def __init__(self) -> None:
        self.file = None
        self.data = None
        self.device = None
        self.time = None
        self.res = {}
        self.new_res = {}
        self.measure_data = {}
        self.cvs =None
        self.init()

    def init(self):
        self.get_file()
        self.get_data()
        self.get_device()

    def get_file(self):
        data = os.listdir(os.getcwd())
        for d in data:
            if d.startswith("cali"):
                self.file = d
                print(d)

    def get_data(self):
        print("Using readlines()")
        with open(self.file) as fp:
            self.data = fp.readlines()
            # for line in Lines:
            #     count += 1
            #     print("Line{}: {}".format(count, line.strip()))
  

    def get_device(self):
        for l in self.data:
            d = re.search( r'GSP(.)-\d*', l, re.M|re.I)
            t = re.search(r'((\d\d\d\d(.*)),)', l, re.M|re.I)

            if d is not None:
                self.device = d.group()
                print(d.group())
                time = t.group(2)
                time = time.replace(" ","_").replace(":","")
                self.time = time
                print(time)
                break
            # break

    def get_res(self):
        for i, l in enumerate(self.data):
            su = self.get_successed(i)
            if su is None:
                continue
            print("-------------------")
            print(su)
            print(self.get_error(i))
            self.res[su] = self.get_error(i)
            self.measure_data[su] = self.get_measure_data(i)
            print("``````````````````")
            # print(self.measure_data)

    def get_successed(self,i):
        # return the measure radian...
        d1 = re.search("Registering last succeeded result as: \[(-?\d+)(\.\d+), (-?\d+)(\.\d+), (-?\d+)(\.\d+)\]", self.data[i], re.M|re.I)
        if d1 is None:
            return None
        d2 = re.search("Calibration is done successfully!", self.data[i+1], re.M|re.I)
        d3 = re.search("Calibration is done successfully!", self.data[i+4], re.M|re.I)
        if d2 is None and d3 is None:
            return None
        # return the measure radian...
        return float(d1.group(3)+d1.group(4))

    def get_error(self,i):
        # return the calibration error..
        for index in range(20):
            d1 = re.search("beacause the error is: (-?\d+)(\.\d+)?m",self.data[index+i],re.M|re.I)
            if d1 is None:
                continue
            # print(d1.group())
            return float(d1.group(1)+d1.group(2))
        return None

    def get_measure_data(self,i):
        data = []
        l = 0
        while True:
            if len(data) == 6:
                # print(data)
                data.reverse()
                return data
            d1 = re.search("Registering last succeeded result as: \[(-?\d+)(\.\d+)?, (-?\d+)(\.\d+)?, (-?\d+)(\.\d+)?\]",self.data[i-l],re.M|re.I)
            if d1 is not None:
                data.append([float(d1.group(1)+d1.group(2)), float(d1.group(3)+d1.group(4)), float(d1.group(5)+d1.group(6))])
            l += 1


    def handle_res(self):
        # print(self.res)
        last_rad = 99.
        for k, v in self.res.items():
            if not math.isclose(last_rad, k, rel_tol=0.1):
                self.new_res[k] = []
                self.new_res[k].append(v)
                last_rad = k
            else:
                self.new_res[last_rad].append(v)
        # print(self.new_res)


    def convert_cvs(self):
        self.cvs = self.device + "_" + self.time + '.csv'
        with open(self.cvs, 'w', newline='') as csvfile:
        # spamwriter = csv.writer(csvfile, delimiter=' ',
        #                         quotechar='|', quoting=csv.QUOTE_MINIMAL)
            spamwriter = csv.writer(csvfile)            
            spamwriter.writerow(["radian"])
            i = 2
            for k, v in self.new_res.items():
                data = []
                data.append(k)
                for d in range(10):
                    # if v[d] is not None:
                    try:
                        data.append(v[d])
                    except IndexError:
                        data.append("")
                data.append("=AVERAGE(B"+str(i)+":K"+str(i)+")")
                i += 1
                spamwriter.writerow(data)
            spamwriter.writerow(["","","","","","","","","","","","=MAX(L2:L9)-MIN(L2:L9)","=AVERAGE(B2:K9)"])

    def move_data(self):
        dir = "./8_dir"
        has_handle = "./8_dir/has_handle"
        if not os.path.exists(has_handle):
            os.makedirs(has_handle)
        shutil.copy(self.cvs, os.path.join(dir))
        new_file_name = self.device+"_"+self.file
        os.rename(self.file, new_file_name)
        shutil.copy(new_file_name,has_handle)
        os.remove(self.cvs)




if __name__ == "__main__":
    # args = sys.argv
    # print(args)
    # hostname = args[1] + ".local"
    # print("Getting file from .... {}".format(hostname))
    # gf = Get_file(hostname)
    # fn = gf.get_calibration_file_name()
    # gf.get_file(fn)
    gd = Get8Dir()
    gd.get_res()
    gd.handle_res()
    gd.convert_cvs()
    gd.move_data()
