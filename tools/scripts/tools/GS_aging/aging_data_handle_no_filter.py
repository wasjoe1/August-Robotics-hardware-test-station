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
        self.json_file = name[:-5]
        self.divece_name = name[:9]
        self._filter_list = []

    def load_json(self):
        f = open(self.json_file + ".json")
        self.data = json.load(f)

    def get_mean_data(self, data):
        data.sort()
        half = len(data) // 2
        return (data[half] + data[~half]) / 2

    def should_be_filter(self, key, mean, data, threshold):
        if math.fabs(data-mean) > threshold:
            self._filter_list.append(key)
            return True
        return False

    # def filter_row(self, data, threshold):
    #     mean = self.get_mean_data(data)
    #     for value in data:
    #         if self.should_be_filter(mean, value, threshold):
    #             pass

    def filter_data(self):
        c0_x = []
        c0_y = []
        c0_horizontal = []

        c1_x = []
        c1_y = []
        c1_horizontal = []

        c2_x = []
        c2_y = []
        c2_horizontal = []

        p1_dis = []
        p1_hor = []
        p1_ver = []

        p2_dis = []
        p2_hor = []
        p2_ver = []

        p3_dis = []
        p3_hor = []
        p3_ver = []

    # "532": {
    #     "c_end_time": "2022-08-29 07:18:40.894613",
    #     "c": {
    #         "calibration_data": "{\"stamp\": {\"secs\": 1661757520, \"nsecs\": 749172925}, \"calibration_score\": 81, \"calibrated_pose\": [-0.01195, -0.09006, -1.5584254985260193], \"distance_of_rbs\": 0.0}",
    #         "calibration_is_succeeded": "true",
    #         "c_start_time": "2022-08-29 07:18:06.912514",
    #         "cali_id_f_x": 8.566331757,
    #         "cali_id_f_y": -3.244249486,
    #         "cali_id_r_x": -15.54288662,
    #         "cali_id_r_y": -1.659951443,
    #         "type": "calibration"
    #     },
    #     "m": {
    #         "1": {
    #             "m_diff_time": -8.594604,
    #             "measurement_data": "{\"distance\": 48.4547, \"rad_circle\": 0.0, \"rad_hor\": 0.040052028647758246, \"located\": true, \"color\": \"ROG\", \"header\": {\"stamp\": {\"secs\": 1661757536, \"nsecs\": 509772062}, \"frame_id\": \"\", \"seq\": 0}, \"state\": \"DONE\", \"gid\": 1, \"specified_dev_name\": \"\", \"rad_ver\": 0.0036628347812947245, \"circle\": false, \"unique_id\": 0}",
    #             "m_start_time": "2022-08-29 07:18:48.100045",
    #             "measurement_is_succeeded": "true",
    #             "m_end_time": "2022-08-29 07:18:56.694649"
    #         },
    #         "0": {
    #             "m_diff_time": -6.791298,
    #             "measurement_data": "{\"distance\": 15.5609, \"rad_circle\": 0.0, \"rad_hor\": -1.4802734839722858, \"located\": true, \"color\": \"ROG\", \"header\": {\"stamp\": {\"secs\": 1661757527, \"nsecs\": 797692060}, \"frame_id\": \"\", \"seq\": 0}, \"state\": \"DONE\", \"gid\": 1, \"specified_dev_name\": \"\", \"rad_ver\": 0.010222481426431106, \"circle\": false, \"unique_id\": 0}",
    #             "m_start_time": "2022-08-29 07:18:41.103425",
    #             "measurement_is_succeeded": "true",
    #             "m_end_time": "2022-08-29 07:18:47.894723"
    #         },
    #         "2": {
    #             "m_diff_time": -17.392203,
    #             "measurement_data": "{\"distance\": 9.0942, \"rad_circle\": 0.0, \"rad_hor\": 1.206053445744143, \"located\": true, \"color\": \"ROG\", \"header\": {\"stamp\": {\"secs\": 1661757554, \"nsecs\": 194425106}, \"frame_id\": \"\", \"seq\": 0}, \"state\": \"DONE\", \"gid\": 1, \"specified_dev_name\": \"\", \"rad_ver\": 0.021452386758968753, \"circle\": false, \"unique_id\": 0}",
    #             "m_start_time": "2022-08-29 07:18:56.902835",
    #             "measurement_is_succeeded": "true",
    #             "m_end_time": "2022-08-29 07:19:14.295038"
    #         }
    #     },
    #     "c_diff_time": 33.982099
    # },
        # print(self.data)
        copy_dict = {**self.data}
        for key, value in copy_dict.items():
            print(key)
            # print(value)
            if value["c"]["calibration_is_succeeded"] == "false":
                del self.data[key]
                continue

            if value[C]["calibration_is_succeeded"] == "true":
                value[C]['calibration_data'] = ast.literal_eval(
                    str(value[C]['calibration_data']))

            # print(value["c"]["calibration_data"])
            # print(value["c"]["calibration_data"])

            if float(value["c"]["calibration_data"]["calibration_score"]) > 0 and value["m"]["0"]["measurement_is_succeeded"] == "true" and value["m"]["1"]["measurement_is_succeeded"] == "true" and value["m"]["2"]["measurement_is_succeeded"] == "true":
                mod = int(key) % 3
                if mod == 0:
                    c0_x.append(list(value["c"]["calibration_data"]["calibrated_pose"])[0])
                    c0_y.append(list(value["c"]["calibration_data"]["calibrated_pose"])[1])
                    c0_horizontal.append(list(value["c"]["calibration_data"]["calibrated_pose"])[2])
                elif mod == 1:
                    c1_x.append(list(value["c"]["calibration_data"]["calibrated_pose"])[0])
                    c1_y.append(list(value["c"]["calibration_data"]["calibrated_pose"])[1])
                    c1_horizontal.append(list(value["c"]["calibration_data"]["calibrated_pose"])[2])
                elif mod == 2:
                    c2_x.append(list(value["c"]["calibration_data"]["calibrated_pose"])[0])
                    c2_y.append(list(value["c"]["calibration_data"]["calibrated_pose"])[1])
                    c2_horizontal.append(list(value["c"]["calibration_data"]["calibrated_pose"])[2])

                value[M]["0"]["measurement_data"] = json.loads(
                    str(value[M]["0"]['measurement_data']))

                value[M]["1"]["measurement_data"] = json.loads(
                    str(value[M]["1"]['measurement_data']))

                value[M]["2"]["measurement_data"] = json.loads(
                    str(value[M]["2"]['measurement_data']))

                p1_dis.append(value["m"]["0"]["measurement_data"]["distance"])
                p1_hor.append(value["m"]["0"]["measurement_data"]["rad_hor"])
                p1_ver.append(value["m"]["0"]["measurement_data"]["rad_ver"])

                p2_dis.append(value["m"]["1"]["measurement_data"]["distance"])
                p2_hor.append(value["m"]["1"]["measurement_data"]["rad_hor"])
                p2_ver.append(value["m"]["1"]["measurement_data"]["rad_ver"])

                p3_dis.append(value["m"]["2"]["measurement_data"]["distance"])
                p3_hor.append(value["m"]["2"]["measurement_data"]["rad_hor"])
                p3_ver.append(value["m"]["2"]["measurement_data"]["rad_ver"])

            else:
                # self._filter_list.append(key)
                del self.data[key]
        
        mean_c0_x = self.get_mean_data(c0_x)
        mean_c0_y = self.get_mean_data(c0_y)
        mean_c0_horizontal = self.get_mean_data(c0_horizontal)

        mean_c1_x = self.get_mean_data(c1_x)
        mean_c1_y = self.get_mean_data(c1_y)
        mean_c1_horizontal = self.get_mean_data(c1_horizontal)

        mean_c2_x = self.get_mean_data(c2_x)
        mean_c2_y = self.get_mean_data(c2_y)
        mean_c2_horizontal = self.get_mean_data(c2_horizontal)

        mean_p1_dis = self.get_mean_data(p1_dis)
        mean_p1_hor = self.get_mean_data(p1_hor)
        mean_p1_ver = self.get_mean_data(p1_ver)

        mean_p2_dis = self.get_mean_data(p2_dis)
        mean_p2_hor = self.get_mean_data(p2_hor)
        mean_p2_ver = self.get_mean_data(p2_ver)

        mean_p3_dis = self.get_mean_data(p3_dis)
        mean_p3_hor = self.get_mean_data(p3_hor)
        mean_p3_ver = self.get_mean_data(p3_ver)

        for key, value in self.data.items():
            mod = int(key) % 3
            if mod == 0:
                self.should_be_filter(key, mean_c0_x, list(value["c"]["calibration_data"]["calibrated_pose"])[0], 0.003)
                self.should_be_filter(key, mean_c0_y, list(value["c"]["calibration_data"]["calibrated_pose"])[1], 0.003)
                self.should_be_filter(key, mean_c0_horizontal, list(value["c"]["calibration_data"]["calibrated_pose"])[2], 0.0005)
                # c0_x.append(list(value["c"]["calibration_data"]["calibrated_pose"])[0])
                # c0_y.append(list(value["c"]["calibration_data"]["calibrated_pose"])[1])
                # c0_horizontal.append(list(value["c"]["calibration_data"]["calibrated_pose"])[2])
            elif mod == 1:
                self.should_be_filter(key, mean_c1_x, list(value["c"]["calibration_data"]["calibrated_pose"])[0], 0.003)
                self.should_be_filter(key, mean_c1_y, list(value["c"]["calibration_data"]["calibrated_pose"])[1], 0.003)
                self.should_be_filter(key, mean_c1_horizontal, list(value["c"]["calibration_data"]["calibrated_pose"])[2], 0.0005)
                # c1_x.append(list(value["c"]["calibration_data"]["calibrated_pose"])[0])
                # c1_y.append(list(value["c"]["calibration_data"]["calibrated_pose"])[1])
                # c1_horizontal.append(list(value["c"]["calibration_data"]["calibrated_pose"])[2])
            elif mod == 2:
                self.should_be_filter(key, mean_c2_x, list(value["c"]["calibration_data"]["calibrated_pose"])[0], 0.003)
                self.should_be_filter(key, mean_c2_y, list(value["c"]["calibration_data"]["calibrated_pose"])[1], 0.003)
                self.should_be_filter(key, mean_c2_horizontal, list(value["c"]["calibration_data"]["calibrated_pose"])[2], 0.0005)
                # c2_x.append(list(value["c"]["calibration_data"]["calibrated_pose"])[0])
                # c2_y.append(list(value["c"]["calibration_data"]["calibrated_pose"])[1])
                # c2_horizontal.append(list(value["c"]["calibration_data"]["calibrated_pose"])[2])

            # p1_dis.append(value["m"]["0"]["measurement_data"]["distance"])
            # p1_hor.append(value["m"]["0"]["measurement_data"]["rad_hor"])
            # p1_ver.append(value["m"]["0"]["measurement_data"]["rad_ver"])

            # p2_dis.append(value["m"]["1"]["measurement_data"]["distance"])
            # p2_hor.append(value["m"]["1"]["measurement_data"]["rad_hor"])
            # p2_ver.append(value["m"]["1"]["measurement_data"]["rad_ver"])

            # p3_dis.append(value["m"]["2"]["measurement_data"]["distance"])
            # p3_hor.append(value["m"]["2"]["measurement_data"]["rad_hor"])
            # p3_ver.append(value["m"]["2"]["measurement_data"]["rad_ver"])

            self.should_be_filter(key, mean_p1_dis, value["m"]["0"]["measurement_data"]["distance"], 0.005)
            self.should_be_filter(key, mean_p1_hor, value["m"]["0"]["measurement_data"]["rad_hor"], 0.0001)
            self.should_be_filter(key, mean_p1_ver, value["m"]["0"]["measurement_data"]["rad_ver"], 0.002)

            self.should_be_filter(key, mean_p2_dis, value["m"]["1"]["measurement_data"]["distance"], 0.005)
            self.should_be_filter(key, mean_p2_hor, value["m"]["1"]["measurement_data"]["rad_hor"], 0.0001)
            self.should_be_filter(key, mean_p2_ver, value["m"]["1"]["measurement_data"]["rad_ver"], 0.002)

            self.should_be_filter(key, mean_p3_dis, value["m"]["2"]["measurement_data"]["distance"], 0.005)
            self.should_be_filter(key, mean_p3_hor, value["m"]["2"]["measurement_data"]["rad_hor"], 0.0001)
            self.should_be_filter(key, mean_p3_ver, value["m"]["2"]["measurement_data"]["rad_ver"], 0.002)

        copy_dict == {**self.data}
        for key, value in copy_dict.items():
            if key in self._filter_list:
                del self.data[key]

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
            cvs_list.append(open(str(a)+".csv", 'w'))
            writer_list.append(csv.writer(cvs_list[a]))
            single_point_cvs_list.append(open("sp_"+str(a)+".csv", 'w'))
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
                        md = json.loads(value[M][m]['measurement_data'])
                        print(str(value[M][m]['measurement_data']))
                        # value[M][m]["measurement_data"] = json.loads(
                        #     str(value[M][m]['measurement_data']))
                        # print(type(value[M][m]["measurement_data"]))

                        h = float(md["rad_hor"])
                        v = float(md["rad_ver"])
                        laser = float(
                            md["distance"])
                        # distance * math.cos(theta) + LED_BEACON_RADIUS
                        dis = float(md
                                    ["distance"])*math.cos(v) + 0.05
                        pos_h = float(
                            md["rad_hor"]) + gs_c_rz
                        print(h, v, dis)
                        # print("dis: {}".format(dis))
                        # print("x: {}".format(dis*math.cos(h)))
                        # print("y: {}".format(dis*math.sin(h)))
                        x = gs_c_x + dis * (math.cos(pos_h))
                        y = gs_c_y + dis * (math.sin(pos_h))
                        rb_pose.append([x, y])
                        use_time = float(value[M][m]["m_diff_time"])
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
            # j.filter_data()
            # j.print_json()
            j.hand_data()
            j.save_execl()
            j.remove_cvs()
            shutil.copy(log, "has_handler/")
            shutil.copy(log[:-5]+".xlsx", j.divece_name+"/")
            os.remove(log)
            os.remove(log[:-5]+".xlsx")

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
