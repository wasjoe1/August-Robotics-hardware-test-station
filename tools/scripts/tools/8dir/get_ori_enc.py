#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from get_8_dir_res_get_data import Get8Dir
import os
import yaml
import math
import sys
import numpy as np
import csv
import shutil

from encoder_compensate import EncoderCompensate

from guiding_beacon_system._calibration.algorithm import do_calibration
# from guiding_beacon_system.utils import polar2pose

def polar2pose(dis, yaw, pitch):
    dis *= math.cos(pitch)
    x = dis * math.cos(yaw)
    y = dis * math.sin(yaw)
    return x, y

home = os.path.expanduser('~')

MINAS_SERVO_RESOLUTION = 8388607

rb1 = (-15.54288662, -1.659951443)
rb2 = (2.445430569, -48.48078833)

# gd = Get8Dir()
# gd.get_res()
# gd.handle_res()

# print(gd.device)
# print(os.path.join(home, "catkin_ws/src/boothbot-config/boothbot_config/config/device_settings"),gd.device,"device_settings.yaml")

class GetOriEnc():
    def __init__(self) -> None:
        self.gd = Get8Dir()
        self.gd.get_res()
        self.gd.handle_res()
        self._settings_file = os.path.join(home, "catkin_ws/src/boothbot-config/boothbot_config/config/device_settings",self.gd.device,"device_settings.yaml")
        self.ori_enc = {}
        self.hori_offset = self.get_hori_offset()
        
    def get_hori_offset(self):
        with open(self._settings_file, "r") as s_f:
            data = yaml.safe_load(s_f)
            # print(data)
        return data["servos_driver"]["servo_parameter"]["horizontal"]["zero_offset"]

    def get_ori_enc(self):
        print("the origin radian data ")
        print(self.gd.measure_data)
        for k,v in self.gd.measure_data.items():
            self.ori_enc[k] = v
            # print(v[1])
            for k1,d in enumerate(v):
                ori_d = int(d[1]* (MINAS_SERVO_RESOLUTION/(2*math.pi)) + self.hori_offset)
                if ori_d > MINAS_SERVO_RESOLUTION:
                    ori_d = ori_d - MINAS_SERVO_RESOLUTION
                self.ori_enc[k][k1][1] = ori_d
        print("convert origin radian to encoding")
        print(self.ori_enc)



class GetComp():
    def __init__(self,name) -> None:
        self.local_dir = os.path.join(home, "catkin_ws/local/encoder_calibration")
        # print(name)
        self.compen_file = os.path.join(self.local_dir,name[0:9],name+".yaml")
        print("load compensation file...")
        print(self.compen_file)
        self._encoder_compensate = EncoderCompensate(name+".yaml", MINAS_SERVO_RESOLUTION-1, self.compen_file)
        self.compensated_data = {}

    def get_compen_data(self,data,hori_offset):
        for k,v in data.items():
            self.compensated_data[k] = v
            # print(v[1])
            for k1,d in enumerate(v):
                ori_d = d[1]
                ori_d += self._encoder_compensate.enc_compensate(ori_d, "show_compen")
                ori_c = ori_d - hori_offset
                if ori_c > (MINAS_SERVO_RESOLUTION>>1):
                    ori_c = ori_c - MINAS_SERVO_RESOLUTION
                rad = ori_c/MINAS_SERVO_RESOLUTION*2*math.pi
                self.compensated_data[k][k1][1] = rad
        print("after compensation... the radian data ")
        print(self.compensated_data)



class Get8DirResult():
    def __init__(self,data) -> None:
        self.data = data
        self.cvs = None
        self._8dir_data = {}
        self.final_data = {}

    def get_8dir_result(self):
        for k, v in self.data.items():
            rb1_list = []
            rb2_list = []
            for i in range(3):
                rb1_list.append((v[i][0],v[i][1],v[i][2]))
            for i in range(3,6):
                rb2_list.append((v[i][0],v[i][1],v[i][2]))
                # for i2 in range()
            rb1_measurement = np.array(rb1_list)
            rb2_measurement = np.array(rb2_list)
            rb1_measurement_avg = (rb1_measurement[:, 0].mean() + 0.05, rb1_measurement[:, 1].mean(), rb1_measurement[:, 2].mean())
            rb2_measurement_avg = (rb2_measurement[:, 0].mean() + 0.05, rb2_measurement[:, 1].mean(), rb2_measurement[:, 2].mean())

            # try:
            calibrated_pose = do_calibration({
                "coordinates": [
                    rb1,
                    rb2,
                ],
                "measurements": [
                    rb1_measurement_avg,
                    rb2_measurement_avg,
                ],
            }, tolerance=0.05)
            # except RefsDistanceNotMatch as e:
            #     logger.logerr(e)
            #     calibrated_pose = [0., 0., 0.]
            # except ACosDataRangeError as e:
            #     logger.logerr(e)
            #     calibrated_pose = [0., 0., 0.]

            # Scoring the result
            distance_of_rb = np.linalg.norm(np.array(rb2) - np.array(rb1))
            distance_of_rb_m = np.linalg.norm(np.array(polar2pose(*rb2_measurement_avg) - np.array(polar2pose(*rb1_measurement_avg))))
            distance_err = distance_of_rb_m - distance_of_rb
            print(k, distance_err)
            self._8dir_data[k] = distance_err

    def save_cvs(self):
            # def handle_res(self):
        # print(self.res)
        last_rad = 99.
        for k, v in self._8dir_data.items():
            if not math.isclose(last_rad, k, rel_tol=0.1):
                self.final_data[k] = []
                self.final_data[k].append(v)
                last_rad = k
            else:
                self.final_data[last_rad].append(v)
        # print(self._8dir_data)


    def convert_cvs(self,device, comp_file):
        self.cvs = device + "_" + comp_file + '.csv'
        with open(self.cvs, 'w', newline='') as csvfile:
        # spamwriter = csv.writer(csvfile, delimiter=' ',
        #                         quotechar='|', quoting=csv.QUOTE_MINIMAL)
            spamwriter = csv.writer(csvfile)            
            spamwriter.writerow(["radian"])
            i = 2
            for k, v in self.final_data.items():
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

    def move_data(self, device):
        dir = "./8_dir"
        has_handle = "./8_dir/" + device
        if not os.path.exists(has_handle):
            os.makedirs(has_handle)
        shutil.copy(self.cvs, has_handle)
        # new_file_name = device+"_"+self.file
        # os.rename(self.cvs, new_file_name)
        shutil.copy(self.cvs, has_handle)
        # os.remove(self.cvs)


if __name__ == "__main__":
    args = sys.argv
    print(args)
    # hostname = args[1] + ".local"
    # print("Getting file from .... {}".format(hostname))

    goe = GetOriEnc()
    # print(goe.get_hori_offset())
    goe.get_ori_enc()
    # print("convert ")
    # print(goe.ori_enc)


    gc = GetComp(args[1])
    gc.get_compen_data(goe.ori_enc, goe.hori_offset)
    print(gc.compensated_data)

    g8dr = Get8DirResult(gc.compensated_data)
    g8dr.get_8dir_result()
    g8dr.save_cvs()
    g8dr.convert_cvs(goe.gd.device, args[1])
    g8dr.move_data(goe.gd.device)


        # score = self.scoring(distance_err, self.action_goal.tolerance)
        # logger.logwarn("Calibrated score is: {} beacause the error is: {}m".format(score, distance_err))


# goe = GetOriEnc()
# # print(goe.get_hori_offset())
# goe.get_ori_enc()
# print(goe.enc)

