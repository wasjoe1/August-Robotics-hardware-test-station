#!/usr/bin/env python3

import oyaml
import json

from boothbot_config.device_settings import DEVICE_SETTINGS_FILE_PATH

import os

home = os.path.expanduser('~')


def get_file():
    dir_list = os.listdir(".")

    json_list = []
    for file_name in dir_list:
        if ".json" in file_name:
            json_list.append(file_name)
    return json_list


def get_device_name(file_name):
    return file_name[0:9]


def get_device_setting_path(device_name):
    return home + "/catkin_ws/src/boothbot-config/boothbot_config/config/device_settings/" + device_name + "/device_settings.yaml"


def replace_setting(device_settings_path, json_path):
    print("loading json.... {}".format(json_path))
    with open(json_path, "r") as jf:
        json_data = json.load(jf)

    servo_h = json_data["servos"]["servo_h"]
    servo_v = json_data["servos"]["servo_v"]

    with open(device_settings_path, "r") as f:
        print(device_settings_path)
        doc = oyaml.safe_load(f)
        print(doc)
        doc['servos_driver']['servo_parameter']['horizontal']['zero_offset'] = servo_h
        doc['servos_driver']['servo_parameter']['vertical']['zero_offset'] = servo_v

    with open(device_settings_path, "w") as f:
        oyaml.dump(doc, f)


if __name__ == "__main__":
    print(get_file())
    for file_name in get_file():
        device_name = get_device_name(file_name)
        print(device_name)
        device_settings_path = get_device_setting_path(device_name)
        print(device_settings_path)
        replace_setting(device_settings_path, file_name)
