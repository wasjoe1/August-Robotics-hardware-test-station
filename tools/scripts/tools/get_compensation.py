#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import re
import csv

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
    # print(hd.res_data)
    ocsv = OutputCSV(hd.res_data, gf.device, gf.time)
    ocsv.convert_cvs()