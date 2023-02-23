#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import re
import csv
# from guiding_beacon_system._calibration.algorithm import do_calibration
import numpy as np
import math
from encoder_compensate import EncoderCompensate
import time

MINAS_SERVO_RESOLUTION = 8388607

def polar2pose(dis, yaw, pitch):
    dis *= math.cos(pitch)
    x = dis * math.cos(yaw)
    y = dis * math.sin(yaw)
    return x, y

rb1 = (-15.54288662, -1.659951443)
rb2 = (2.445430569, -48.48078833)

def in_range(n, start, end = 0):
  return start <= n <= end if end >= start else end <= n <= start

class get_file():
    def __init__(self, file_type=None, hostname=None):
        self.file_type =file_type
        self.hostname =None
        self.filename = None
        self.raw_log_data = None
        self.device = None
        self.time = None
        self.get_file()
        self.get_data()
        self.get_device()

    def get_file(self):
        data = os.listdir(os.getcwd())
        for d in data:
            if d.startswith("cali"):
                self.filename = d
                print(d)

    def get_data(self):
        print("Using readlines()")
        with open(self.filename) as fp:
            self.raw_log_data = fp.readlines()

    def get_device(self):
        for l in self.raw_log_data:
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


class HandleData():
    def __init__(self, raw_log_data=None):
        self.raw_log_data= raw_log_data
        self.hostname = None
        self.date_time = None
        self.data_iter = 0
        self.res_data = {}
        pass

    def get_res(self):
        for i, l in enumerate(self.raw_log_data):
            calibration_successed_res = self.get_calibration_successed(i)
            if calibration_successed_res is None:
                continue
            print("-------------------")
            print("found successed in {}".format(i))
            # print(calibration_successed_res)
            # print(self.get_error(i))
            # self.res[calibration_successed_res] = self.get_error(i)
            # self.measure_data[calibration_successed_res] = self.get_measure_data(i)
            self.res_data["cali_id_"+str(self.data_iter)] = {}
            self.res_data["cali_id_"+str(self.data_iter)]["radian_laser"] = self.find_detail_data(i, "R")
            self.res_data["cali_id_"+str(self.data_iter)]["encodings"] = self.find_detail_data(i, "E")
            
            # self.find_detail_data(i, "E")
            self.data_iter += 1
            print("``````````````````")


    def search(self,raw_data, regular):
        return re.search(regular, raw_data, re.M|re.I)

    def get_calibration_successed(self,i):
        reg = "Calibration is done successfully!"
        res = self.search(self.raw_log_data[i], reg)
        return res

    def search_radian_laser(self, raw_data):
        radian_and_laser_reg = "Registering last succeeded result as: \[(.*), (.*), (.*)\]"
        res = re.search(radian_and_laser_reg, raw_data, re.M|re.I)
        if res is None:
            return res
        # print(res.groups()[0], res.groups()[1], res.groups()[2])
        return (res.groups()[0], res.groups()[1], res.groups()[2])

    def search_encoder(self, raw_data):
        encoder_reg = "Succeeded encodings: \[(.*), (.*), (.*), (.*)]"
        res = re.search(encoder_reg, raw_data, re.M|re.I)
        if res is None:
            return res
        # print(res.groups()[0], res.groups()[1], res.groups()[2])
        return (res.groups()[0], res.groups()[1], res.groups()[2], res.groups()[3])

    def find_detail_data(self, i, data_type):
        # radian_and_laser_reg = "Registering last succeeded result as: \[(.*), (.*), (.*)\]"
        # encoder_reg = "Succeeded encodings: \[(.*), (.*), (.*), (.*)]"
        end_reg = "Received goal:"
        res_data_iter = 100
        res_data = {}
        ite = i
        RB_data_name = ""
        while True:
            if data_type == "R":
                RB_data_name = "laser_radian_"
                res = self.search_radian_laser(self.raw_log_data[ite])
            elif data_type == "E":
                RB_data_name = "encodings_"
                res = self.search_encoder(self.raw_log_data[ite])
            

            if self.search(self.raw_log_data[ite],end_reg) is not None:
                # print("Found calibration start. end this loop.")
                break

            ite -= 1

            if res is None:
                continue
            else:
                res_data_iter = res_data_iter - 1
                # print("res_data_iter: {}".format(res_data_iter))
                res_data[RB_data_name+str(res_data_iter)] = res
        
            
        return res_data

            # if res_data_iter == 0:
            #     print("data find all. end this loop..")
            #     break


    # def get_successed(self,i):
    #     # return the measure radian...
    #     d1 = re.search("Registering last succeeded result as: \[(-?\d+)(\.\d+), (-?\d+)(\.\d+), (-?\d+)(\.\d+)\]", self.data[i], re.M|re.I)
    #     if d1 is None:
    #         return None
    #     d2 = re.search("Calibration is done successfully!", self.data[i+1], re.M|re.I)
    #     d3 = re.search("Calibration is done successfully!", self.data[i+4], re.M|re.I)
    #     if d2 is None and d3 is None:
    #         return None
    #     # return the measure radian...
    #     return float(d1.group(3)+d1.group(4))        




class OutputCSV():
    def __init__(self, dict, device_name, test_time):
        self.dict = dict
        self.device =device_name
        self.test_time =test_time
        self.save()

    def save(self, ):
        pass
    
    def convert_cvs(self):
        self.cvs = self.device + "_" + self.test_time + '.csv'
        with open(self.cvs, 'w', newline='') as csvfile:
        # spamwriter = csv.writer(csvfile, delimiter=' ',
        #                         quotechar='|', quoting=csv.QUOTE_MINIMAL)
            spamwriter = csv.writer(csvfile)            
            spamwriter.writerow(["test_id"])
            # i = 2
            csv_data = []
            for test_id in range(0,1000):
                
                data = []
                cali_id = "cali_id_"+str(test_id)
                cali_exist = self.dict.get(cali_id, None)
                if cali_exist is None:
                    print("cali_")
                    break
                data.append(cali_id)
                print(self.dict[cali_id])
                for measure_id in range(0,100):
                    # laser
                    measure_exist = self.dict[cali_id]["radian_laser"].get("laser_radian_"+str(measure_id), None)
                    if measure_exist is not None:
                        print(float(self.dict[cali_id]["radian_laser"]["laser_radian_"+str(measure_id)][0]))
                        data.append(float(self.dict[cali_id]["radian_laser"]["laser_radian_"+str(measure_id)][0]))
                        data.append(int(self.dict[cali_id]["encodings"]["encodings_"+str(measure_id)][2]))
                        data.append(int(self.dict[cali_id]["encodings"]["encodings_"+str(measure_id)][1]))
                    else:
                        continue
                spamwriter.writerow(data)
                # csv_data.append(data)
            # spamwriter.writerow(csv_data)

class Get8results():
    def __init__(self, data, compen=None):
        self.data = data
        self.comp = compen+".yaml"
        self._encoder_compensate = EncoderCompensate("test", MINAS_SERVO_RESOLUTION-1, self.comp)
        self.raw_data = []
        self.cvs_res = []

    def get_result(self, comp=True):
        
        results = []

        for k,v in self.data.items():
            if k == "cali_id_13":
                continue
            m1 = []
            m2 = []
            rad_ver1 = []
            rad_ver2 = []

            rad_hor1 = []
            rad_hor2 = []

            encoding1 = []
            encoding2 = []
            for k1,v1 in v.items():
                i = 0
                i2 = 0
                if k1.startswith("r"):
                    # get laser
                    for k2,v2 in v1.items():
                        # print(k2, v2)
                        if i < 3:
                            # print(i)
                            m1.append(float(v2[0]))
                            rad_ver1.append(float(v2[2]))
                        else:
                            # if v2[0] != str(47.9959):
                            m2.append(float(v2[0]))
                            rad_ver2.append(float(v2[2]))
                        i += 1
                if k1.startswith("e"):
                    for k2, v2 in v1.items():
                        if i2 < 3:
                            d = int(v2[2])
                            if comp:
                                d +=  self._encoder_compensate.enc_compensate(int(v2[2]), "show_compen")
                            # print(d)
                            encoding1.append(d)
                            rad_hor1.append((2*math.pi*d)/(1<<23))
                            
                        else:
                            d = int(v2[2])
                            if comp:
                                d +=  self._encoder_compensate.enc_compensate(int(v2[2]), "show_compen")
                            encoding2.append(d)
                            rad_hor2.append((2*math.pi*d)/(1<<23))
                                    
                        i2 += 1
            
            rb1_list = []
            rb2_list = []
            raw_data1=[]
            raw_data2=[]
            for i in range(3):
                r1 = rad_hor1[i]
                r2 = rad_hor2[i]
                if rad_hor1[i] > math.pi:
                    r1 = rad_hor1[i] - 2*math.pi
                # print("r1",rad_hor1[i],r1)
                if rad_hor2[i]> math.pi:
                    r2 = rad_hor2[i] - 2*math.pi
                # print("r2",rad_hor2[i],r2)
                rb1_list.append((m1[i],r1,rad_ver1[i]))
                raw_data1.append((m1[i],encoding1[i],r1,rad_ver1[i]))
            # for i in range(3,6):
                rb2_list.append((m2[i],r2,rad_ver2[i]))
                raw_data2.append((m2[i],encoding2[i],r2,rad_ver2[i]))

                # for i2 in range()
            # print(rb1_list)
            # print(rb2_list)
            # print(math.sqrt(rb2_list[0][0]*rb2_list[0][0] + rb1_list[0][0]*rb1_list[0][0]
            #                 - 2*rb2_list[0][0]*rb1_list[0][0]*math.cos(rb2_list[0][1]-rb1_list[0][1])))
            rb1_measurement = np.array(rb1_list)
            rb2_measurement = np.array(rb2_list)

            raw_data1 = np.array(raw_data1)
            raw_data2 = np.array(raw_data2)
            
            rb1_measurement_avg = (rb1_measurement[:, 0].mean() + 0.05, rb1_measurement[:, 1].mean(), rb1_measurement[:, 2].mean())
            rb2_measurement_avg = (rb2_measurement[:, 0].mean() + 0.05, rb2_measurement[:, 1].mean(), rb2_measurement[:, 2].mean())
            raw_data1_avg = (raw_data1[:,0].mean()+0.05, raw_data1[:,1].mean(), raw_data1[:,2].mean(), raw_data1[:,3].mean())
            raw_data2_avg = (raw_data2[:,0].mean()+0.05, raw_data2[:,1].mean(), raw_data2[:,2].mean(), raw_data2[:,3].mean())
            # print(rb1_measurement_avg, rb2_measurement_avg)
            # print(raw_data1_avg,raw_data2_avg)
            distance_of_rb = np.linalg.norm(np.array(rb2) - np.array(rb1))
            distance_of_rb_m = np.linalg.norm(np.array(polar2pose(*rb2_measurement_avg) - np.array(polar2pose(*rb1_measurement_avg))))
            distance_err = distance_of_rb_m - distance_of_rb
            print(k,distance_of_rb, distance_of_rb_m, rb2_measurement[:, 1].mean(), distance_err)
            self.cvs_res.append((rb2_measurement[:, 1].mean(), distance_err))


            results.append(distance_err)
            self.raw_data.append((raw_data1_avg,raw_data2_avg,distance_err, distance_of_rb_m))

        # save 
        # with open("test.csv", 'w', newline='') as csvfile:
        #     spamwriter = csv.writer(csvfile)            
        #     spamwriter.writerow(["radian"])
        #     for d in self.cvs_res:
        #         spamwriter.writerow([d[0],d[1]])

        print(max(results)-min(results))


class EncoderComp():
    def __init__(self,data):
        self.data = data
        self.dis_err_mean = None
        self.dis_rbs = None
        self.calc_mean_err()
        self.real_dis()
        self.compen_data = []
        self.merge_data = []
        self.yaml_data = []
        self.all_first_data = []
        # self.merge_data

    def real_dis(self):
        self.dis_rbs = np.linalg.norm(np.array(rb2) - np.array(rb1)) + self.dis_err_mean

    def calc_mean_err(self):
        data = np.array(self.data)
        self.dis_err_mean = data[:,2].mean()
        print("mean error is {}".format(self.dis_err_mean))

    def calc_comp(self):
        for d in self.data:
            offset = d[3] - self.dis_rbs
            a = d[0][0]
            b = d[1][0]
            c = d[3]
            std_c = self.dis_rbs
            radian_c = math.acos((a*a+b*b-c*c)/(2*a*b))
            std_radian_c = math.acos((a*a+b*b-std_c*std_c)/(2*a*b))
            radian_offset = radian_c - std_radian_c
            encoding_offset = (1<<23)*radian_offset/(2*math.pi)
            # print("{} to {} has radian_offset {}, encoding offset {}".format(d[0][1], d[1][1], radian_offset, encoding_offset))
            self.compen_data.append((int(d[0][1]), int(d[1][1]), encoding_offset))
        
        print(self.compen_data)
            # c*c = a*a + b*b - 2*a*b*cos(C)
            # C = acos((a*a+b*b-c*c)/2*a*b)
            # std_radian = math.acos((a*a+b*b-c*c)/2*a*b))

    def handle_res(self):
        last_rad = 9999999999
        for v in self.compen_data:
            print(v[0])
            if not math.isclose(last_rad, v[0], abs_tol=2e2):
                # self.new_res[k] = []
                self.merge_data.append(v)
                last_rad = v[0]
            # else:
            #     self.new_res[last_rad].append(v)
        # print(self.new_res)
        print(self.merge_data)


    def out_compen(self):
        target = [0]
        offset = [0]
        real = [0]
        self.yaml_data = [target, offset, real]
        # find min in data
        # all_data = []
        for d in self.merge_data:
            # all_data.append(d[0])
            self.all_first_data.append(d[1])
        print(self.all_first_data)
        self.all_first_data.sort()

        real.append(self.all_first_data[0])
        offset.append(0)
        target.append(self.all_first_data[0])
        ind, ind_ = self.find_index(self.all_first_data[0])
        if ind_ == 1:
            ind_ = 0
        else:
            ind_ = 1
        real.append(self.merge_data[ind][ind_])
        offset.append(int(self.merge_data[ind][2]))
        # TODO
        target.append(self.merge_data[ind][ind_] + self.merge_data[ind][2])

        del target[0]
        del offset[0]
        del real[0]

        print(self.all_first_data)

        # for d in self.all_first_data:
        #     if self.has_in_yaml(d):
        #         continue
        #     ind, ind_ = self.find_index(d)
        self.tmp()

        print(self.yaml_data)

    def tmp(self):
        # while len(self.yaml_data[0]) == 16:
        for d in self.all_first_data:
            if self.has_in_yaml(d):
                continue
            self.find_in_yaml(d)
            


    def find_index(self, data):
        print("find data {}".format(data))
        for ind, val in enumerate(self.merge_data):
            for i in range(2):
                if data == val[i]:
                    return ind, i
        return None

    def find_in_yaml(self, data):
        #now, the yaml data length
        l = len(self.yaml_data[0])
        for i in range(l-1):
            print(data)
            if in_range(data, self.yaml_data[2][i], self.yaml_data[2][i+1]):
                print("data {} in range {} and {}".format(data, self.yaml_data[2][i], self.yaml_data[2][i+1]))


    
    def has_in_yaml(self, data):
        print("find data {} in merge_data".format(data))
        for d in self.yaml_data[2]:
            if d == data:
                return True

        # for d in data2:
        #     if d == data:
        #         return True
        
        return False
        # for ind, val in enumerate(self.merge_data):
        #     for i in range(2):
        #         if data == val[i]:
        #             return True
        return False

        # at last, handle the  first data 0

    # def handle_data(self):
    #     # last_data = None
    #     for val in self.compen_data:
    #         if len(self.merge_data) is 0:
    #             self.merge_data.append(val)
    #             # last_data = 
    #             continue
    #         for ind, val2 in enumerate(self.merge_data):
    #             if math.isclose(val[0], val2[0],abs_tol=200) and math.isclose(val[1], val2[1],abs_tol=200):
    #                 print(math.isclose(val[0], val2[0],abs_tol=200))
    #                 print(math.isclose(val[1], val2[1],abs_tol=200))
    #                 print("close data {} and {}".format(val, val2))
    #                 self.merge_data[ind] = ((val[0]+self.merge_data[ind][0])/2,
    #                                         (val[1]+self.merge_data[ind][0])/2,
    #                                         (val[2]+self.merge_data[ind][0])/2)
    #             else:
    #                 self.merge_data.append(val2)
    #     print(self.merge_data)


if __name__ == "__main__":
    gf = get_file()
    print("device: {}".format(gf.device))
    print("time: {}".format(gf.time))
    hd = HandleData(gf.raw_log_data)
    hd.get_res()

    out = OutputCSV(hd.res_data,gf.device,gf.time)
    out.convert_cvs()
    # g8r = Get8results(hd.res_data, "calibration-gsp1-0042-20220220")
    # g8r.get_result(False)
    # print(g8r.raw_data)

    # ec = EncoderComp(g8r.raw_data)
    # # ec.calc_mean_err()
    # ec.calc_comp()
    # ec.handle_res()
    # ec.out_compen()
