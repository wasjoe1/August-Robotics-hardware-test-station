#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import yaml
import os
import matplotlib.pyplot as plt
import numpy as np
from scipy import optimize
import math

LOCAL = "/home/sunjilin/catkin_ws/local/encoder_calibration"

MAX_E = (1<<23)

def encodingtoradian(d, max_encoding=MAX_E):
    # r:encodings = 2*pi:max_encodings
    return (2*math.pi*d)/max_encoding

def radiantoencoding(d, max_encoding=MAX_E):
    return max_encoding*d/(2*math.pi)

class DataFit():
    def __init__(self, sn=None, fn = None ,load_file=None) -> None:
        self.fn = fn
        self.sn = sn
        self.raw_compen_data = None
        # self.get_data()
        self.xdata = None
        self.ydata = None
        self.z = None
        self.x = None
        self.get_data(load_file)
        # self.load_file = load_file
        self.load_file_data = None
        self.fit()

    def get_data(self, file_name=None):
        if self.sn == None:
            yaml_file = None
            if file_name is None:
                data_files = os.listdir()
                print(data_files)
                for i in data_files:
                    if ".yaml" in i:
                        yaml_file = i
            else:
                yaml_file = file_name
            with open(yaml_file, "r") as f:
                self.raw_compen_data =  yaml.safe_load(f)
                self.xdata = encodingtoradian(np.array(self.raw_compen_data[2]))
                self.ydata = np.array(self.raw_compen_data[1])
        else:
            yaml_file = LOCAL + "/" + self.sn + "/" + self.fn + ".yaml"
            with open(yaml_file, "r") as f:
                self.raw_compen_data =  yaml.safe_load(f)
                self.xdata = encodingtoradian(np.array(self.raw_compen_data[2]))
                self.ydata = np.array(self.raw_compen_data[1])

    def fit(self):
        param, pc = optimize.curve_fit(self.f1, self.xdata, self.ydata)
        self.z,self.x = param[0],param[1]
        print("Get the param a: {} b: {} ".format(param[0], param[1]))

    # xdata = encodingtoradian(np.array(yamldata[2]))
    # ydata = np.array(yamldata[1])

    def f1(self, x,a,c):
        # return a*np.sin(b*x*np.pi/180+c)+d
        return a*np.sin(x*np.pi/math.pi+c)

    def gen_comp_dict(self, z=None, xw=None):
        offset = []
        encodings = []
        zf = None
        if z is None:
            zf = self.z
        else:
            zf = z*self.z

        for i in range(0,100):
            i = i*0.0628
            y = self.f1(i,zf,xw)
            offset.append(int(y))
            encodings.append(int(radiantoencoding(i)))
        # print(offset)
        return [encodings,offset,encodings]



# def get_data(file_name=None):
#     yaml_file = None
#     if file_name is None:
#         data_files = os.listdir()
#         print(data_files)
#         for i in data_files:
#             if ".yaml" in i:
#                 yaml_file = i
#     else:
#         yaml_file = file_name
#     with open(yaml_file, "r") as f:
#         return yaml.safe_load(f)
    
# yamldata = get_data()

# import numpy as np
# from scipy import optimize
# import math

# print(1<<23)




# xdata = encodingtoradian(np.array(yamldata[2]))
# ydata = np.array(yamldata[1])

# def f1(x,a,c):
#     # return a*np.sin(b*x*np.pi/180+c)+d
#     return a*np.sin(x*np.pi/math.pi+c)

# param, pc = optimize.curve_fit(f1, xdata, ydata)
# a,c = param[0],param[1]
# c = c%(2*math.pi)
# print(a,c)
# x1 = np.linspace(min(xdata),max(xdata),)
# y1 = a*np.sin(x1*np.pi/math.pi+c)
# for i in x1:
#     if f1(i,a,c) == max(y1):
#         xmax = round(i,2)
#     elif f1(i,a,c) == min(y1):
#         xmin = round(i,2)

# plt.scatter(xdata,ydata,c='g',label='datapoint')
# plt.plot(x1,y1,'b--',label='fitting')
# plt.xlabel('x')
# plt.ylabel('y')
# plt.text(xmax,max(y1),(xmax,round(max(y1),2)))
# plt.text(xmin,min(y1),(xmin,round(min(y1),2)))
# plt.legend()
# plt.show()