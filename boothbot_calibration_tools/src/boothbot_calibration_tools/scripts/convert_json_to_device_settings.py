#!/usr/bin/env python3

import oyaml
import json
import shutil

# from boothbot_config.device_settings import DEVICE_SETTINGS_FILE_PATH

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
    return home + "/catkin_ws/src/boothbot-config/boothbot_config/config/device_settings/" + device_name.upper() + "/device_settings.yaml"


def gs_replace_setting(device_settings_path, json_path):
    print("loading json.... {}".format(json_path))
    with open(json_path, "r") as jf:
        json_data = json.load(jf)

    servo_h = None
    servo_v = None
    cameras_angle = None
    vertical_offset = None
    if 'INITIALIZE_SERVO' in json_data.keys():
        servo_h = json_data["INITIALIZE_SERVO"]["servo_h"]
    # if 'INITIALIZE_SERVO' in json_data.keys():
    #     servo_v = json_data["INITIALIZE_SERVO"]["servo_v"]
    if 'CAMERAS_ANGLE' in json_data.keys():
        cameras_angle = json_data["CAMERAS_ANGLE"]["cameras_angle"]
    if 'VERTICAL_SERVO_ZERO' in json_data.keys():
        vertical_offset = json_data["VERTICAL_SERVO_ZERO"]["vertical_offset"]

    with open(device_settings_path, "r") as f:
        print(device_settings_path)
        doc = oyaml.safe_load(f)
        print(doc)
        if servo_h is not None:
            doc['servos_driver']['servo_parameter']['horizontal']['zero_offset'] = int(
                servo_h)
        # if servo_v is not None:
        #     doc['servos_driver']['servo_parameter']['vertical']['zero_offset'] = int(
        #         servo_v)
        if cameras_angle is not None:
            doc['tracker_driver']['long_short_cam_angle_offset'] = cameras_angle
        if vertical_offset is not None:
            doc['servos_driver']['servo_parameter']['vertical']['zero_offset'] = int(
                vertical_offset)

    with open(device_settings_path, "w") as f:
        oyaml.dump(doc, f)


def ln_replace_setting(device_settings_path, json_path):
    print("loading json.... {}".format(json_path))
    with open(json_path, "r") as jf:
        json_data = json.load(jf)

    servo_h = None
    servo_v = None
    cameras_angle = None
    # vertical_offset = None
    if 'HORIZONTAL_OFFSET' in json_data.keys():
        servo_h = json_data["HORIZONTAL_OFFSET"]["horizontal_offset"]
    if 'VERTICAL_SERVO_ZERO' in json_data.keys():
        servo_v = json_data["VERTICAL_SERVO_ZERO"]["vertical_offset"]
    if 'CAMERAS_ANGLE' in json_data.keys():
        cameras_angle = json_data["CAMERAS_ANGLE"]["cameras_angle"]
    # if 'VERTICAL_SERVO_ZERO' in json_data.keys():
    #     vertical_offset = json_data["VERTICAL_SERVO_ZERO"]["vertical_offset"]

    with open(device_settings_path, "r") as f:
        print(device_settings_path)
        doc = oyaml.safe_load(f)
        print(doc)
        if servo_h is not None:
            doc['servos_driver']['servo_parameter']['horizontal']['zero_offset'] = int(
                servo_h)
        if servo_v is not None:
            doc['servos_driver']['servo_parameter']['vertical']['zero_offset'] = int(
                servo_v)
        if cameras_angle is not None:
            doc['tracker_driver']['long_short_cam_angle_offset'] = cameras_angle
        # if vertical_offset is not None:
        #     doc['servos_driver']['servo_parameter']['vertical']['zero_offset'] = int(vertical_offset)

    with open(device_settings_path, "w") as f:
        oyaml.dump(doc, f)


if __name__ == "__main__":
    print(get_file())
    for file_name in get_file():
        device_name = get_device_name(file_name)
        print(device_name)
        device_settings_path = get_device_setting_path(device_name)
        if device_name.lower().startswith("gs"):
            print(device_settings_path)
            gs_replace_setting(device_settings_path, file_name)
        else:
            ln_replace_setting(device_settings_path, file_name)
        shutil.copy(file_name, "./has_handle/")
        os.remove(file_name)
