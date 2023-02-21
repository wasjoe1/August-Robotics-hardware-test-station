#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import re
import csv
# from guiding_beacon_system._calibration.algorithm import do_calibration
import numpy as np
import math
from encoder_compensate import EncoderCompensate

MINAS_SERVO_RESOLUTION = 8388607

def polar2pose(dis, yaw, pitch):
    dis *= math.cos(pitch)
    x = dis * math.cos(yaw)
    y = dis * math.sin(yaw)
    return x, y

rb1 = (-15.54288662, -1.659951443)
rb2 = (2.445430569, -48.48078833)

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
            

                    # data.append(float(self.dict["cali_id_"+str(test_id)]["radian_laser"]["laser_radian_"+str(measure_id)][0]))
            # for k, v in self.dict.items():
            #     print(k,v)
            #     data = []
            #     for k1, v1 in v.items():
            #         print(v1)
            #         for k2,v2 in v1.items():
            #             print()
                    # data.append()
                # data.append
                # exit(0)
                # data.append(k)
                # for d in range(10):
                #     # if v[d] is not None:
                #     try:
                #         data.append(v[d])
                #     except IndexError:
                #         data.append("")
                # data.append("=AVERAGE(B"+str(i)+":K"+str(i)+")")
                # i += 1
                # spamwriter.writerow(data)        

if __name__ == "__main__":
    gf = get_file()
    print("device: {}".format(gf.device))
    print("time: {}".format(gf.time))
    hd = HandleData(gf.raw_log_data)
    hd.get_res()
    results = []
    compen_file = "encoder_alignment_20-Feb-23_11-12-21"+".yaml"
    _encoder_compensate = EncoderCompensate("test", MINAS_SERVO_RESOLUTION-1, compen_file)

    for k,v in hd.res_data.items():
        if k == "cali_id_13":
            continue
        # print(k, v)
        # print(k)
        m1 = []
        m2 = []

        rad_ver1 = []
        rad_ver2 = []

        rad_hor1 = []
        rad_hor2 = []

        encoding1 = []
        encoding2 = []
            # radian_laser or encodings:
        for k1,v1 in v.items():
            # print(k1,v1)
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
                        if v2[0] != str(47.9959):
                            m2.append(float(v2[0]))
                            rad_ver2.append(float(v2[2]))
                    i += 1
            # print("eeeeeeeeee",k1)
            if k1.startswith("e"):
                for k2,v2 in v1.items():
                    if i2 < 3:
                        d = int(v2[2])
                        d +=  _encoder_compensate.enc_compensate(int(v2[2]), "show_compen") 
                        print(d)
                        encoding1.append(d)
                        # print()
                        # rad_hor1.append((1<<23)/(2*math.pi*d))
                        rad_hor1.append((2*math.pi*d)/(1<<23))
                        
                    else:
                        d = int(v2[2])
                        d +=  _encoder_compensate.enc_compensate(int(v2[2]), "show_compen") 
                        print(d)
                        # print(d)
                        encoding2.append(d)
                        # print(v2[2])
                        # print((1<<23)/(2*math.pi*d))
                        rad_hor2.append((2*math.pi*d)/(1<<23))
                        # rad_hor2.append((1<<23)/(2*math.pi*int(v2[2])))
                                
                    i2 += 1
        # print(encoding1)
        # print(encoding2)
        
        rb1_list = []
        rb2_list = []
        # print(m1, encoding1, rad_hor1)
        # print(m2, encoding2, rad_hor2)
        # print(1<<23)
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
        # for i in range(3,6):
            rb2_list.append((m2[i],r2,rad_ver2[i]))
            # for i2 in range()
        # print(rb1_list)
        # print(rb2_list)
        # print(math.sqrt(rb2_list[0][0]*rb2_list[0][0] + rb1_list[0][0]*rb1_list[0][0]
        #                 - 2*rb2_list[0][0]*rb1_list[0][0]*math.cos(rb2_list[0][1]-rb1_list[0][1])))
        rb1_measurement = np.array(rb1_list)
        rb2_measurement = np.array(rb2_list)
        rb1_measurement_avg = (rb1_measurement[:, 0].mean() + 0.05, rb1_measurement[:, 1].mean(), rb1_measurement[:, 2].mean())
        rb2_measurement_avg = (rb2_measurement[:, 0].mean() + 0.05, rb2_measurement[:, 1].mean(), rb2_measurement[:, 2].mean())

        # try:
        #     calibrated_pose = do_calibration({
        #         "coordinates": [
        #             rb1,
        #             rb2,
        #         ],
        #         "measurements": [
        #             rb1_measurement_avg,
        #             rb2_measurement_avg,
        #         ],
        #     }, tolerance=0.05)
        # except Exception as e:
        #     print("calibration error {}".format(e))

        # Scoring the result
        distance_of_rb = np.linalg.norm(np.array(rb2) - np.array(rb1))
        distance_of_rb_m = np.linalg.norm(np.array(polar2pose(*rb2_measurement_avg) - np.array(polar2pose(*rb1_measurement_avg))))
        distance_err = distance_of_rb_m - distance_of_rb
        print(k,distance_of_rb, distance_of_rb_m,distance_err)
        results.append(distance_err)
        
    print(max(results)-min(results))
        # print(m1)
        # print(m2)
        # print(encoding1)
        # print(encoding2)
                    # print(v2)
        # exit(0)
    # print(hd.res_data)
    # ocsv = OutputCSV(hd.res_data, gf.device, gf.time)
    # ocsv.convert_cvs()