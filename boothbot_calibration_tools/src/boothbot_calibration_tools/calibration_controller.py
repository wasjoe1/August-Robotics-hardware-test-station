#!/usr/bin/env python3
# -*- coding: utf-8 -*-


import rospy

import rospy as logger

from boothbot_common.ros_logger_wrap import ROSLogging as Logging

import os
import time
import oyaml
import socket
import glob

import ros_numpy
import base64
from PIL import Image
from io import BytesIO
from datetime import datetime

from boothbot_calibration_tools.constants import CalibrationStates as MS
from boothbot_calibration_tools.constants import CalibrationCommand as CS

# from boothbot_common.settings import BOOTHBOT_GET_CONFIG
from boothbot_config.device_settings import DEVICE_SETTINGS_FILE_PATH
from boothbot_config.device_settings import CONFIG_PATH
from boothbot_perception.tracking_camera import TrackingCamera

import json
import socket

from boothbot_common.module_base import ModuleBase

from guiding_beacon_system.drivers.laser_driver import LaserRangeFinderGenerator, LaserRangeFinder

from boothbot_driver.servos_client import ServosClient
# from boothbot_perception.track_client import TargetTracker

from boothbot_msgs.ros_interfaces import (
    APPS_CALIBRATION_SRV_CMD,
    APPS_CALIBRATION_STATUS,
    APPS_CALIBRATION_DATA,
    DRIVERS_SERVOS_PDO,
    DRIVERS_TRACKER_STATUS,
    DRIVERS_SERVOS_SRV_CMD,
    # DRIVERS_TRACKER_LONG_IMAGE,
    # DRIVERS_TRACKER_SHORT_IMAGE,
)


TRANSITIONS_TOP = [
    {
        # Error state manual entrance
        "trigger": "to_ERROR",
        "source": "*",
        "dest": MS.ERROR,
        "unless": ["is_ERROR"],
    },
    {
        # now we reset the machinesrv_cmd_inf
        "trigger": "reset",
        "source": "*",
        "dest": MS.RESETING,
        "unless": ["is_RESETING"],
    },
    {
        # to idle state after reseting
        "trigger": "to_IDLE",
        "source": [MS.RESETING],
        "dest": MS.IDLE,
    },
    {
        # Will back to INIT if initialize failed
        "trigger": "to_INIT",
        "source": [MS.RESETING],
        "dest": MS.INIT,
    },
]

TRANSITIONS = TRANSITIONS_TOP + [
    {
        "trigger": "to_RUNNING",
        "source": MS.IDLE,
        "dest": MS.RUNNING,
    },
]

JOB_SETTING = {
    CS.INITIALIZE_SERVO.name: {},
    CS.CAMERA_SHARPNESS.name: {"camera": "long", "exp_dis": {"long": 14, "short": 5}},
    CS.CAMERAS_ALIGNMENT.name: {},
    CS.CAMERA_LASER_ALIGNMENT.name: {},
    CS.CAMERAS_ANGLE.name: {},
    CS.VERTICAL_SERVO_ZERO.name: {},
    CS.IMU_CALIBRATION.name: {},
}

LONG = "long"
SHORT = "short"

# class Camera(object):
#     def __init__() :
#         self.


class CalibrationController(ModuleBase):
    def __init__(self, name, rate, states=None, transitions=None, commands=None, status_inf=None, srv_cmd_inf=None, need_robot_status=False, error_codes=None):
        super(CalibrationController, self).__init__(
            name=name,
            rate=rate,
            states=MS,
            transitions=TRANSITIONS,
            commands=CS,
            status_inf=APPS_CALIBRATION_STATUS,
            srv_cmd_inf=APPS_CALIBRATION_SRV_CMD,
        )
        self.ON_STATE_METHODS.update(
            {
                MS.INIT: [self.on_INIT],
                MS.RESETING: [self.on_RESETING],
                MS.IDLE: [self.on_IDLE],
                MS.RUNNING: [self.on_RUNNING],
                MS.ERROR: [],
            }
        )
        self.puber_data = APPS_CALIBRATION_DATA.Publisher()

        DRIVERS_SERVOS_PDO.Subscriber(self.servo_pdo_cb)
        # DRIVERS_TRACKER_LONG_IMAGE.Subscriber(self.long_camera_cb)
        # DRIVERS_TRACKER_SHORT_IMAGE.Subscriber(self.short_camera_cb)

        self._data = {}
        self._data["data"] = {}
        self._job_data = {}
        self._save_data = {}
        self._job = None
        self._data["data"]["host_name"] = socket.gethostname()

        self._done_list = {}
        # self._done_list["IMU_CALIBRATION"] = "true"

        # self.send_image = False
        self.long_camera_img_data = None
        self.short_camera_img_data = None

        # driver
        # self.tracker = TargetTracker()
        # self.camera_long = TrackingCamera("/dev/camera_long", laser_dist=4)
        # self.camera_short = TrackingCamera("/dev/camera_short", laser_dist=50)
        self.camera_long = None
        self.camera_short = None
        self.servos = ServosClient()
        self.laser = LaserRangeFinderGenerator.detect_laser_range_finder()

        # config yaml
        self.config_dir = CONFIG_PATH + "/calibration_data"
        self.json_file = ""
        self.last_json_file = ""

        # state
        self.sub_state = 0

        # cameras
        self.cameras = {LONG: None, SHORT: None}
        self.cameras_frame = {LONG: None, SHORT: None}

    def update_data(self):
        if self._job is not None:
            self._data["data"]["step"] = self._job.name
        self._data["data"]["job_data"] = self._job_data
        self._data["data"]["save_data"] = self._save_data
        self._data["data"]["done"] = self._done_list

    def pub_data(self):
        # short_img_data = self.handler_img_data(
        #     self.short_camera_img_data, "short")
        # if short_img_data is not None:
        #     # self.loginfo("pub short img")
        #     self.puber_data.publish(short_img_data)
        # # else:
        # #     self.logwarn("short img is None")

        # long_img_data = self.handler_img_data(
        #     self.long_camera_img_data, "long")
        # if long_img_data is not None:
        #     # self.loginfo("pub long img")
        #     self.puber_data.publish(long_img_data)
        # # else:
        #     self.logwarn("long img is None")

        for k, v in self.cameras_frame.items():
            img_res = self.handler_img_data(k, v)
            if img_res is not None:
                self.loginfo("pub {} img ".format(k))
                self.puber_data.publish(img_res)
            rospy.sleep(0.05)
        self.reset_image_flag()

        self.update_data()
        msg = json.dumps(self._data)
        self.puber_data.publish(msg)

    def on_INIT(self):
        if self.initialize():
            self.initialized = True
        return True

    def on_RESETING(self):
        self.loginfo("reset, to idle")
        # self.tracker.reset()
        self.servos.reset()
        self.laser.reset()
        self.to_IDLE()
        return True

    def on_IDLE(self):
        self.pub_data()
        if self.is_job():
            self.to_RUNNING()

    def on_RUNNING(self):
        self.pub_data()
        if self._job == CS.INITIALIZE_SERVO:
            self._do_initialize_servo()
        elif self._job == CS.CAMERA_SHARPNESS:
            self.loginfo_throttle(5, "prepare image.")
            self._do_sharpness()
            # self.tracker.trigger("BOG", cali=True)
            pass
        elif self._job == CS.CAMERAS_ALIGNMENT:
            pass
        elif self._job == CS.CAMERA_LASER_ALIGNMENT:
            pass
        elif self._job == CS.CAMERAS_ANGLE:
            pass
        elif self._job == CS.VERTICAL_SERVO_ZERO:
            pass
        elif self._job == CS.IMU_CALIBRATION:
            pass

    def initialize(self):
        # TODO
        logger.loginfo("initializing ....")
        self.last_json_file = self.get_last_json()
        self.get_last_data()
        self.check_yaml_dir()
        now = datetime.now()
        date_time = now.strftime("%m-%d-%Y-%H-%M-%S")
        self.json_file = self.config_dir + '/' + socket.gethostname() + "-" + \
            date_time + ".json"

        for client in (self.servos, self.laser):
            self.loginfo("connecting to {}".format(client.name))
            if not client.connect(timeout=0.5):
                logger.logwarn("Initializing {} failed!!".format(client.name))
                # initialized = False
                return False
            else:
                self.loginfo("connected succeeded{}".format(client.name))
        # self.tracker.capture(True)
        self.reset()
        return True

    def handle_inputs(self, command):
        if command != CS.NONE:
            self.logwarn("Got command. {}".format(command))
            # return
        if command == CS.NONE:
            pass
        elif CS.RESET == command:
            self.reset()
        elif CS.SERVOS_DISABLE == command:
            self.logwarn("servo disable")
            self.servos.disable()
            # if self.goal is not None:
            #     self.goal.got_manual_cali_command = "RB1"
        elif CS.SERVOS_ENABLE == command:
            self.logwarn("servo enable")
            self.servos.enable()
            pass
        elif CS.LASER_ON == command:
            self.laser.laser_on()
        elif CS.LASER_OFF == command:
            self.laser.laser_off()
        elif CS.RUN == command:
            self.job_init_servo_replace_setting()

        elif CS.SAVE == command:
            pass
        elif CS.DONE == command:
            self.job_done()
        else:
            self.turn_to_step(command)
            # if self.goal is not None:
            #     self.goal.got_manual_cali_command = "RB2"
        return True
        # logger.loginfo("handler input ....")

    # def reset(self):
    #     self.loginfo("resetting.")

    def job_done(self):
        if self._job is None:
            return
        if self._job == CS.INITIALIZE_SERVO:
            self.logwarn("killing node ")
            # rosnode.kill_nodes("servos_driver")
            os.system("rosnode kill /servos_driver")
        self._done_list[self._job.name] = "true"
        self.sub_state = 0
        self.save_json()
        # TODO

    def is_job(self):
        return self._job is not None

    # callback function

    def servo_pdo_cb(self, msg):
        # update servo data
        self._job_data["servo_h"] = msg.encodings_origin[0]
        self._job_data["servo_v"] = msg.encodings_origin[1]

    # def long_camera_cb(self, msg):
    #     # self.loginfo_throttle(5, "Got long img")
    #     self.long_camera_img_data = msg

    # def short_camera_cb(self, msg):
    #     # self.loginfo_throttle(5, "Got short img")
    #     self.short_camera_img_data = msg

    def turn_to_step(self, CS):
        if self._job is None:
            self.loginfo("step change to {}".format(CS.name))
            self._job = CS
            self.to_RUNNING()
            return
        if self._job != CS:
            self.loginfo("step change to {}".format(CS.name))
            self.reset()
            self._job = CS
            self.to_RUNNING()
            return

    # image handler for get image data from rostopic
    # def img2textfromtopic(self, msg):
    #     im = ros_numpy.numpify(msg)
    #     # From BGR to RGB
    #     im = im[:, :, ::-1]
    #     im = Image.fromarray(im)
    #     buf = BytesIO()
    #     im.save(buf, format="JPEG")
    #     im_binary = base64.b64encode(buf.getvalue())
    #     im_text = im_binary.decode()
    #     return im_text

    # image handler for get image data from rostopic
    def img2textfromcv2(self, frame):
        # im = ros_numpy.numpify(msg)
        # From BGR to RGB
        im = frame[:, :, ::-1]
        im = Image.fromarray(im)
        buf = BytesIO()
        im.save(buf, format="JPEG")
        im_binary = base64.b64encode(buf.getvalue())
        im_text = im_binary.decode()
        return im_text

    def reset_image_flag(self):
        for k , v in self.cameras_frame.items():
            self.cameras_frame[k] = None
        # self.long_camera_img_data = None
        # self.short_camera_img_data = None

    def handler_img_data(self, type, img_data):
        if img_data is not None:
            self.loginfo("Got {} msg".format(type))
            pre_data = {}
            # img_data = self.img2text(msg)
            pre_data[type] = {"time": time.time(), "data": img_data}
            return json.dumps(pre_data)
        else:
            self.logwarn("{} is None".format(type))
            return None

    # yaml handler
    def get_last_json(self):
        # file_type = r'\*json'
        self.loginfo("config file {}".format(self.config_dir))
        files = glob.glob(self.config_dir + "/*")
        if len(files) != 0:
            max_file = max(files, key=os.path.getctime)
            self.loginfo("last file is {}".format(max_file))
            return max_file
        else:
            self.logwarn("there is no json file in config directory")
            return None

    def get_last_data(self):
        if self.last_json_file is None:
            self._data["last_data"] = {}
        else:
            with open(self.last_json_file, "r") as f:
                # last_data = json.loads
                last_data = json.load(f)
                self._data["data"]["last_data"] = last_data

    def save_json(self):
        with open(self.json_file, 'w') as f:
            json.dump(self._save_data, f)

    def check_yaml_dir(self):
        if not os.path.exists(self.config_dir):
            self.loginfo("Directory is created.")
            os.mkdir(self.config_dir)

    def job_init_servo_replace_setting(self):
        with open(DEVICE_SETTINGS_FILE_PATH) as f:
            doc = oyaml.load(f)

        doc['servos_driver']['servo_parameter']['horizontal']['zero_offset'] = self._job_data["servo_h"]
        doc['servos_driver']['servo_parameter']['vertical']['zero_offset'] = self._job_data["servo_v"]

        servo_save_data = {
            "servos": {
                "time": time.time(),
                "servo_h": self._job_data["servo_h"],
                "servo_v": self._job_data["servo_v"],
            }
        }

        self._save_data.update(servo_save_data)

        with open(DEVICE_SETTINGS_FILE_PATH, 'w') as f:
            oyaml.dump(doc, f)

    # JOB

    def _do_initialize_servo(self):
        pass

    def _do_sharpness(self):
        camera_type = JOB_SETTING[CS.CAMERA_SHARPNESS.name]['camera']
        if self.sub_state == 0:
            self.cameras[LONG] = TrackingCamera(
                "/dev/camera_long", laser_dist=40)
            self.cameras[SHORT] = TrackingCamera(
                "/dev/camera_short", laser_dist=5)
            time.sleep(10)
            self._sub_state = 1
        if self.sub_state == 1:
            self.get_sharpness_result(camera_type)

        # RB 4m for short, RB 40 for long
        pass

    def get_sharpness_result(self, type):
        frame = self.cameras[type].cap()
        # frame = camera.cap()
        if frame is not None:
            self.loginfo("Got {} frame".format(type))
            beacon_res = self.cameras[type].find_beacon(frame, 0.0, "BOG")
            self.cameras[type].draw_beacon(frame, beacon_res)
            self.cameras_frame[type] = self.img2textfromcv2(frame)
            self.loginfo("beacon_res {}".format(beacon_res))
            sharpness_result = self.cameras[type].get_sharpness(
                frame, beacon_res, JOB_SETTING[CS.CAMERA_SHARPNESS.name]["exp_dis"]["long"])
            self.loginfo("sharpness good {}".format(sharpness_result))
            color_result = self.cameras[type].get_color_value(
                frame, beacon_res, color="green")
            self.loginfo("color good {}".format(color_result))


if __name__ == "__main__":
    rospy.init_node("calibration_controller")
    c = CalibrationController("calibration_controller", 10)
    c.run()
