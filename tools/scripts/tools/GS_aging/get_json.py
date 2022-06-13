#!/usr/bin/env python
# -*- coding: utf-8 -*-

import json
import csv
import ast
import math
import pandas as pd


cvs_list = []
writer_list = []
sm_cvs_list = []
sm_writer_list = []


class JsonHandle(object):
    def __init__(self):
        pass
        self.data = None
        self._cvs_execl = None
        self.cvs_data = []
        self.json_file = "GSP4-0057_2022-06-08-13-32-24"

    def load_json(self):
        f = open(self.json_file + ".json")
        self.data = json.load(f)

    def print_json(self):
        for key, value in self.data.items():
            print(key)
            print(value)
            # for b in a:
            #     # print(b["c_state"])
            #     print(b["tracking"])
            #     print(b["result"])

    def hand_data(self):
        # data_name = "test_data.csv"
        for a in range(3):
            cvs_list.append(open(str(a)+".csv", 'wb'))
            writer_list.append(csv.writer(cvs_list[a]))
            sm_cvs_list.append(open('s' + str(a)+'.csv', 'wb'))
            sm_writer_list.append(csv.writer(sm_cvs_list[a]))
            writer_list[a].writerow(['gid',
                                     'calibration_is_succeeded',
                                     'c_start_time',
                                     'c_end_time',
                                     'c_diff_time',
                                     'cali_id_f_x',
                                     'cali_id_f_y',
                                     'cali_id_r_x',
                                     'cali_id_r_y',
                                     'calibration_score',
                                     'distance_of_rbs',
                                     'c_x',
                                     'c_y',
                                     'c_rad',
                                     'measurement_is_succeeded',
                                     'm_start_time',
                                     'm_end_time',
                                     'm_diff_time',
                                     'rad_hor',
                                     'rad_ver',
                                     'distance',
                                     'x',
                                     'y',
                                     'side1',
                                     'sede2',
                                     'side3'])
            sm_writer_list[a].writerow(['gid',
                                        'measurement_is_succeeded',
                                        'm_start_time',
                                        'm_end_time',
                                        'm_diff_time',
                                        'rad_hor',
                                        'rad_ver',
                                        'distance'])
        for key, value in self.data.items():
            try:
                id = int(key) % 3
                if value["calibration_is_succeeded"] == "true":
                    value['calibration_data'] = ast.literal_eval(
                        str(value['calibration_data']))
                    if value["measurement_is_succeeded"] == "true":
                        value["measurement_data"] = json.loads(
                            str(value['measurement_data']))
                        # h = float(value["measurement_data"]["rad_hor"])
                        v = float(value["measurement_data"]["rad_ver"])
                        # distance * math.cos(theta) + LED_BEACON_RADIUS
                        dis = float(value["measurement_data"]
                                    ["distance"])*math.cos(v) + 0.05
                        h = float(value["measurement_data"]["rad_hor"]) + float(
                            list(value['calibration_data']['calibrated_pose'])[2])
                        print(h, v, dis)
                        print("dis: {}".format(dis))
                        print("x: {}".format(dis*math.cos(h)))
                        print("y: {}".format(dis*math.sin(h)))
                        x = float(list(value['calibration_data']['calibrated_pose'])[
                            0]) + dis * (math.cos(h))
                        y = float(list(value['calibration_data']['calibrated_pose'])[
                            1]) + dis * (math.sin(h))
                        p1_x = float(value["cali_id_f_x"])
                        p1_y = float(value["cali_id_f_y"])
                        p2_x = float(value["cali_id_r_x"])
                        p2_y = float(value["cali_id_r_y"])
                        p3_x = x
                        p3_y = y
                        side1 = math.sqrt((p1_x - p3_x)**2 + (p1_y - p3_y)**2)
                        side2 = math.sqrt((p2_x - p3_x)**2 + (p2_y - p3_y)**2)
                        side3 = math.sqrt((p1_x - p2_x)**2 + (p1_y - p2_y)**2)
                        # x = d
                        writer_list[id].writerow([key,
                                                  value["calibration_is_succeeded"],
                                                  value['c_start_time'],
                                                  value['c_end_time'],
                                                  float(value['c_diff_time']),
                                                  float(value['cali_id_f_x']),
                                                  float(value['cali_id_f_y']),
                                                  float(value['cali_id_r_x']),
                                                  float(value['cali_id_r_y']),
                                                  float(value['calibration_data']
                                                        ['calibration_score']),
                                                  float(value['calibration_data']
                                                        ['distance_of_rbs']),
                                                  list(value['calibration_data']
                                                       ['calibrated_pose'])[0],
                                                  list(value['calibration_data']
                                                       ['calibrated_pose'])[1],
                                                  list(value['calibration_data']
                                                       ['calibrated_pose'])[2],
                                                  value["measurement_is_succeeded"],
                                                  value["m_start_time"],
                                                  value["m_end_time"],
                                                  float(value["m_diff_time"]),
                                                  float(
                                                      value["measurement_data"]["rad_hor"]),
                                                  float(
                                                      value["measurement_data"]["rad_ver"]),
                                                  float(
                                                      value["measurement_data"]["distance"]),
                                                  x,
                                                  y,
                                                  side1,
                                                  side2,
                                                  side3])
                        
                        sm_writer_list[id].writerow([key,
                                                    value["measurement_is_succeeded"],
                                                    value["m_start_time"],
                                                    value["m_end_time"],
                                                    float(
                                                        value["m_diff_time"]),
                                                    float(
                                                        value["measurement_data"]["rad_hor"]),
                                                    float(
                                                        value["measurement_data"]["rad_ver"]),
                                                    float(
                                                        value["measurement_data"]["distance"])]
                                                    )
                    else:
                        writer_list[id].writerow([key,
                                                  value["calibration_is_succeeded"],
                                                  value['c_start_time'],
                                                  value['c_end_time'],
                                                  float(value['c_diff_time']),
                                                  float(value['cali_id_f_x']),
                                                  float(value['cali_id_f_y']),
                                                  float(value['cali_id_r_x']),
                                                  float(value['cali_id_r_y']),
                                                  float(value['calibration_data']
                                                        ['calibration_score']),
                                                  float(value['calibration_data']
                                                        ['distance_of_rbs']),
                                                  list(value['calibration_data']
                                                       ['calibrated_pose'])[0],
                                                  list(value['calibration_data']
                                                       ['calibrated_pose'])[1],
                                                  list(value['calibration_data']
                                                       ['calibrated_pose'])[2],
                                                  value["measurement_is_succeeded"],
                                                  '',
                                                  '',
                                                  '',
                                                  '',
                                                  '',
                                                  '',
                                                  '',
                                                  '',
                                                  '',
                                                  '',
                                                  ''])
                else:
                    writer_list[id].writerow([key,
                                              value["calibration_is_succeeded"],
                                              value['c_start_time'],
                                              value['c_end_time'],
                                              float(value['c_diff_time']),
                                              float(value['cali_id_f_x']),
                                              float(value['cali_id_f_y']),
                                              float(value['cali_id_r_x']),
                                              float(value['cali_id_r_y']),
                                              '',
                                              '',
                                              '',
                                              '',
                                              '',
                                              '',
                                              '',
                                              '',
                                              '',
                                              '',
                                              '',
                                              '',
                                              '',
                                              '',
                                              '',
                                              '',
                                              ''])
            except Exception as e:
                print("error")

        for a in range(3):
            # cvs_list.append(open(self.json_file+str(a)+".csv", 'wb'))
            cvs_list[a].close()
            sm_cvs_list[a].close()

    def save_execl(self):
        excel_path = self.json_file
        for v in range(3):
            cvs_path = str(v) + '.csv'

            if self._cvs_execl is None:
                _csv = pd.read_csv(cvs_path, encoding='utf-8')
                with pd.ExcelWriter(excel_path + '.xlsx', mode="w", engine="openpyxl") as writer:
                    _csv.to_excel(writer, sheet_name=str(v))
                self._cvs_execl = True
            else:
                _csv = pd.read_csv(cvs_path, encoding='utf-8')
                with pd.ExcelWriter(excel_path + '.xlsx', mode="a", engine="openpyxl") as writer:
                    _csv.to_excel(writer, sheet_name=str(v))

        for v in range(3):
            cvs_path = "s" + str(v) + '.csv'
            print(cvs_path)
            if self._cvs_execl is None:
                _csv = pd.read_csv(cvs_path, encoding='utf-8')
                with pd.ExcelWriter(excel_path + '.xlsx', mode="w", engine="openpyxl") as writer:
                    _csv.to_excel(writer, sheet_name=str(v))
                self._cvs_execl = True
            else:
                _csv = pd.read_csv(cvs_path, encoding='utf-8')
                with pd.ExcelWriter(excel_path + '.xlsx', mode="a", engine="openpyxl") as writer:
                    _csv.to_excel(writer, sheet_name=str(v))


if __name__ == "__main__":
    j = JsonHandle()
    j.load_json()
    # j.print_json()
    j.hand_data()
    j.save_execl()
