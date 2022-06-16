#!/usr/bin/env python
# -*- coding: utf-8 -*-

import json
import csv
import ast
import math
import pandas as pd
import os
import shutil
import time


cvs_list = []
writer_list = []
single_point_cvs_list = []
single_point_writer_list = []


C = "c"
M = "m"
M_RB_LIST = ["0", "1", "2"]
sheet_name = ["c-r2-r3", "c-r3-r1", "c-r1-r2"]
origin_sheet_name = ["ori_c-r2-r3", "ori_c-r3-r1", "ori_c-r1-r2"]


class JsonHandle(object):
    def __init__(self, name):
        pass
        self.data = None
        self._cvs_execl = None
        self.cvs_data = []
        self.json_file = name[:-5]
        self.divece_name = name[:9]

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

    def get_x_y(self, rb_pose):
        s1 = math.sqrt((rb_pose[0][0] - rb_pose[1][0])
                       ** 2 + (rb_pose[0][1] - rb_pose[1][1]) ** 2)
        s2 = math.sqrt((rb_pose[1][0] - rb_pose[2][0])
                       ** 2 + (rb_pose[1][1] - rb_pose[2][1]) ** 2)
        s3 = math.sqrt((rb_pose[2][0] - rb_pose[0][0])
                       ** 2 + (rb_pose[2][1] - rb_pose[0][1]) ** 2)
        return (s1, s2, s3)

    def hand_data(self):
        # data_name = "test_data.csv"
        for a in range(3):
            cvs_list.append(open(str(a)+".csv", 'wb'))
            writer_list.append(csv.writer(cvs_list[a]))
            single_point_cvs_list.append(open("sp_"+str(a)+".csv", 'wb'))
            single_point_writer_list.append(
                csv.writer(single_point_cvs_list[a]))
            time.sleep(0.01)

            writer_list[a].writerow(['gid',
                                     'calibration_is_succeeded',
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
                                     'p1_measurement_is_succeeded',
                                     'p1_m_diff_time',
                                     'p1_rad_hor',
                                     'p1_rad_ver',
                                     'p1_distance',
                                     'p1_x',
                                     'p1_y',
                                     'p1_measurement_is_succeeded',
                                     'p2_m_diff_time',
                                     'p2_rad_hor',
                                     'p2_rad_ver',
                                     'p2_distance',
                                     'p2_x',
                                     'p2_y',
                                     'p3_measurement_is_succeeded',
                                     'p3_m_diff_time',
                                     'p3_rad_hor',
                                     'p3_rad_ver',
                                     'p3_distance',
                                     'p3_x',
                                     'p3_y',
                                     'side1',
                                     'sede2',
                                     'side3'])
            single_point_writer_list[a].writerow(['gid',
                                                  'diff_time',
                                                  'servo_h',
                                                  'servo_v',
                                                  'laser'])
        for key, value in self.data.items():
            gs_c_x = 99.99
            gs_c_y = 99.99
            gs_c_rz = 99.99
            # try:
            id = int(key) % 3
            rb_pose = []
            iid = 0
            print("handler data {}".format(key))
            print(value[C]["calibration_is_succeeded"])
            if value[C]["calibration_is_succeeded"] == "true":
                value[C]['calibration_data'] = ast.literal_eval(
                    str(value[C]['calibration_data']))

                gs_c_x = float(list(value[C]['calibration_data']
                                    ['calibrated_pose'])[0])
                gs_c_y = float(list(value[C]['calibration_data']
                                    ['calibrated_pose'])[1])
                gs_c_rz = float(list(value[C]['calibration_data']
                                     ['calibrated_pose'])[2])

                write_value = []
                write_value.append(key)
                write_value.append(value[C]["calibration_is_succeeded"])
                write_value.append(float(value['c_diff_time']))
                write_value.append(float(value[C]['cali_id_f_x']))
                write_value.append(float(value[C]['cali_id_f_y']))
                write_value.append(float(value[C]['cali_id_r_x']))
                write_value.append(float(value[C]['cali_id_r_y']))
                write_value.append(float(value[C]['calibration_data']
                                         ['calibration_score']))
                write_value.append(float(value[C]['calibration_data']
                                         ['distance_of_rbs']))
                write_value.append(gs_c_x)
                write_value.append(gs_c_y)
                write_value.append(gs_c_rz)

                # writer_list[id].writerow([key,
                #                           value["calibration_is_succeeded"],
                #                           float(value['c_diff_time']),
                #                           float(value['cali_id_f_x']),
                #                           float(value['cali_id_f_y']),
                #                           float(value['cali_id_r_x']),
                #                           float(value['cali_id_r_y']),
                #                           float(value['calibration_data']
                #                                 ['calibration_score']),
                #                           float(value['calibration_data']
                #                                 ['distance_of_rbs']),
                #                           list(value['calibration_data']
                #                                ['calibrated_pose'])[0],
                #                           list(value['calibration_data']
                #                                ['calibrated_pose'])[1],
                #                           list(value['calibration_data']
                #                                ['calibrated_pose'])[2],

                m_succeeded = []
                for m in M_RB_LIST:
                    if value[M][m]["measurement_is_succeeded"] == "true":
                        m_succeeded.append(True)
                #     else:
                #         m_succeeded.append(False)

                # for m_s in m_succeeded:
                #     if m_s:
                        print(str(value[M][m]['measurement_data']))
                        value[M][m]["measurement_data"] = json.loads(
                            str(value[M][m]['measurement_data']))
                        h = float(value[M][m]["measurement_data"]["rad_hor"])
                        v = float(value[M][m]["measurement_data"]["rad_ver"])
                        laser = float(
                            value[M][m]["measurement_data"]["distance"])
                        # distance * math.cos(theta) + LED_BEACON_RADIUS
                        dis = float(value[M][m]["measurement_data"]
                                    ["distance"])*math.cos(v) + 0.05
                        pos_h = float(
                            value[M][m]["measurement_data"]["rad_hor"]) + gs_c_rz
                        print(h, v, dis)
                        # print("dis: {}".format(dis))
                        # print("x: {}".format(dis*math.cos(h)))
                        # print("y: {}".format(dis*math.sin(h)))
                        x = gs_c_x + dis * (math.cos(pos_h))
                        y = gs_c_y + dis * (math.sin(pos_h))
                        rb_pose.append([x, y])
                        use_time = float(value[M][str(id)]["m_diff_time"])
                        write_value.append("TRUE")
                        write_value.append(use_time)
                        write_value.append(h)
                        write_value.append(v)
                        write_value.append(laser)
                        write_value.append(x)
                        write_value.append(y)

                        # wirte
                        single_write_value = []
                        single_write_value.append(key)
                        single_write_value.append(use_time)
                        single_write_value.append(h)
                        single_write_value.append(v)
                        single_write_value.append(laser)
                        single_point_writer_list[iid].writerow(
                            single_write_value)
                    else:
                        for i in range(7):
                            write_value.append("")
                    iid += 1


            else:
                # print(float(value['c_diff_time']))
                writer_list[id].writerow([key,
                                          value[C]["calibration_is_succeeded"],
                                          '',
                                          float(value[C]['cali_id_f_x']),
                                          float(value[C]['cali_id_f_y']),
                                          float(value[C]['cali_id_r_x']),
                                          float(value[C]['cali_id_r_y']),
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
            # except Exception as e:
            #     print(e)
            if len(rb_pose) == 3:
                (s1, s2, s3) = self.get_x_y(rb_pose)
                write_value.append(s1)
                write_value.append(s2)
                write_value.append(s3)

                writer_list[id].writerow(write_value)
            # if single_write_value is not None:
            #     single_point_writer_list[id].writerow(single_write_value)


        for a in range(3):
            # cvs_list.append(open(self.json_file+str(a)+".csv", 'wb'))
            cvs_list[a].close()
            single_point_cvs_list[a].close()
            # sm_cvs_list[a].close()

    def save_execl(self):
        excel_path = self.json_file
        for v in range(3):
            cvs_path = str(v) + '.csv'

            if self._cvs_execl is None:
                _csv = pd.read_csv(cvs_path, encoding='utf-8')
                with pd.ExcelWriter(excel_path + '.xlsx', mode="w", engine="openpyxl") as writer:
                    # _csv.to_excel(writer, sheet_name=sheet_name[v])
                    _csv.to_excel(writer, sheet_name=origin_sheet_name[v])
                self._cvs_execl = True
            else:
                _csv = pd.read_csv(cvs_path, encoding='utf-8')
                with pd.ExcelWriter(excel_path + '.xlsx', mode="a", engine="openpyxl") as writer:
                    # _csv.to_excel(writer, sheet_name=sheet_name[v])
                    _csv.to_excel(writer, sheet_name=origin_sheet_name[v])

        for v in range(3):
            cvs_path = str(v) + '.csv'
            if self._cvs_execl is None:
                _csv = pd.read_csv(cvs_path, encoding='utf-8')
                with pd.ExcelWriter(excel_path + '.xlsx', mode="w", engine="openpyxl") as writer:
                    _csv.to_excel(writer, sheet_name=sheet_name[v])
                    # _csv.to_excel(writer, sheet_name=origin_sheet_name[v])
                self._cvs_execl = True
            else:
                _csv = pd.read_csv(cvs_path, encoding='utf-8')
                with pd.ExcelWriter(excel_path + '.xlsx', mode="a", engine="openpyxl") as writer:
                    _csv.to_excel(writer, sheet_name=sheet_name[v])
                    # _csv.to_excel(writer, sheet_name=origin_sheet_name[v])

        for v in range(3):
            cvs_path = "sp_" + str(v) + '.csv'
            if self._cvs_execl is None:
                _csv = pd.read_csv(cvs_path, encoding='utf-8')
                with pd.ExcelWriter(excel_path + '.xlsx', mode="w", engine="openpyxl") as writer:
                    # _csv.to_excel(writer, sheet_name=sheet_name[v])
                    _csv.to_excel(writer, sheet_name="sp_" + str(v+1))
                self._cvs_execl = True
            else:
                _csv = pd.read_csv(cvs_path, encoding='utf-8')
                with pd.ExcelWriter(excel_path + '.xlsx', mode="a", engine="openpyxl") as writer:
                    # _csv.to_excel(writer, sheet_name=sheet_name[v])
                    _csv.to_excel(writer, sheet_name="sp_" + str(v+1))


    def remove_cvs(self):
        for a in range(3):
            os.remove(str(a)+".csv")
            os.remove("sp_" + str(a)+".csv")


if __name__ == "__main__":
    if not os.path.exists("has_handler"):
        os.mkdir("has_handler")
    if not os.path.exists("error_json"):
        os.mkdir("error_json")

    for log in os.listdir(os.getcwd()):
        if (".json" in log):

            # try:
            j = JsonHandle(log)
            if not os.path.exists(j.divece_name):
                os.mkdir(j.divece_name)

            j.load_json()
            # j.print_json()
            j.hand_data()
            j.save_execl()
            j.remove_cvs()
            shutil.move(log, "has_handler/")
            shutil.move(log[:-5]+".xlsx", j.divece_name+"/")

            time.sleep(1)

            # except Exception as e:
                # shutil.move(log, "error_json/"+log)


            # try:
            #     j = JsonHandle(log)
            # if not os.path.exists(j.divece_name):
            #     os.mkdir(j.divece_name)            
            #     j.load_json()
            #     # j.print_json()
            #     j.hand_data()
            #     j.save_execl()
            #     j.remove_cvs()
            #     shutil.move(log, "has_handler/"+log)

            # except Exception as e:
            #     shutil.move(log, "error_json/"+log)
