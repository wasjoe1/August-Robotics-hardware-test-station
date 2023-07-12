#!/usr/bin/env python
# -*- coding: utf-8 -*-

# from boothbot_driver.servo_drivers.utils import get_compensate

import os
import yaml

def get_compensate(path):
    try:
        # print(path)
        with open(path, "r") as ecdf:
            return True, yaml.load(ecdf, Loader=yaml.CLoader)
    except IOError:
        print("Can not find encoder_compensate.yaml...")
    return False, None


class EncoderCompensate():
    def __init__(self, compensate_data_name, max_encoding, path, comp_dict=None):
        self.disable = False
        if (path is None) and (comp_dict is None):
            print("set disable true.....")
            self.disable = True
        # print(comp_dict)
        self.comp_dict = comp_dict
        self.max_ending = max_encoding
        self.compensate_data_name = compensate_data_name
        self.get_compensate_dict(path)

    def get_compensate_dict(self, path):
        if path is None:
            return False
            # self.comp_dict = dict
        res, self.comp_dict = get_compensate(path)
        return res

    def check_id_enc(self, selected_dict, dict_id, data):
        # This is recursive function for finding the index of the corresponding encoder from the compenstation dictionary.
        dict_index = dict_id

        if dict_index >= len(self.comp_dict[selected_dict])-1:
            if data > self.comp_dict[selected_dict][dict_index] and data < self.comp_dict[selected_dict][0] + self.max_ending:
                return (True, dict_index)

        if data >= self.comp_dict[selected_dict][dict_index] and data < self.comp_dict[selected_dict][dict_index+1]:
            return (True, dict_index)
        else:
            if data < self.comp_dict[selected_dict][dict_index]:
                return self.check_id_enc(selected_dict, dict_index-1, data)
            else:
                return self.check_id_enc(selected_dict, dict_index+1, data)

    # This function use to get compensation from dictionary.
    def enc_compensate(self, enc, mode="target_compen"):
        if self.disable is True:
            return 0
        # Param: enc
        # The input encoding.

        # Param: mode
        # If "target_compen". Servo move to the target position which compensated from the HD_encoder dict.
        # If "show_compen". Servo show the position which compensated from the servo_encoding dict.

        # Return: r
        # Return the compenstation

        selected_dict = 0

        if mode == "target_compen":
            selected_dict = 0
        elif mode == "show_compen":
            selected_dict = 2

        if enc >= self.max_ending:
            enc = enc - self.max_ending
        if enc < 0:
            enc = enc + self.max_ending

        # Find the index of the corresponding encoder.
        if enc <= min(self.comp_dict[selected_dict]):
            enc = self.max_ending - enc
            dict_index = len(self.comp_dict[selected_dict])-1
        elif enc >= max(self.comp_dict[selected_dict]):
            dict_index = len(self.comp_dict[selected_dict])-1
        else:
            k = self.max_ending/len(self.comp_dict[0])
            d = enc - self.comp_dict[selected_dict][0]

            dict_index = int(d/k)
            (res, index) = self.check_id_enc(selected_dict, dict_index, enc)
            if res:
                dict_index = index

        # Find the compensation between two encoder values for more precise compensation.
        if dict_index >= len(self.comp_dict[0])-1:
            scale = float((enc - self.comp_dict[selected_dict][dict_index])) / \
                (self.comp_dict[selected_dict][0]+self.max_ending-self.comp_dict[selected_dict][dict_index])
            r = int(self.comp_dict[1][dict_index] + scale *
                    (self.comp_dict[1][0]-self.comp_dict[1][dict_index]))
        else:
            scale = float((enc - self.comp_dict[selected_dict][dict_index])) / \
                (self.comp_dict[selected_dict][dict_index+1]-self.comp_dict[selected_dict][dict_index])
            r = int(self.comp_dict[1][dict_index] + scale *
                    (self.comp_dict[1][dict_index+1]-self.comp_dict[1][dict_index]))

        return r
