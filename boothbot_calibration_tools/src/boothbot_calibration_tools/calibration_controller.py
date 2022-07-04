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

from boothbot_common.module_base_py3 import ModuleBase

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

JOB_DATA = {
    CS.INITIALIZE_SERVO.name: ["servo_h", "servo_v"],
    CS.CAMERA_SHARPNESS.name: ["long_sharpness_score", "long_sharpness_result", "long_color_data", "long_color_result",
                               "short_sharpness_score", "short_sharpness_result", "short_color_data", "short_color_result"],
    CS.CAMERAS_ALIGNMENT.name: ["cameras_offset","measurement_time"],
    CS.CAMERA_LASER_ALIGNMENT.name: ["camera_laser_alignment"],
    CS.CAMERAS_ANGLE.name: ["cameras_angle"],
    CS.VERTICAL_SERVO_ZERO.name: [],
    CS.IMU_CALIBRATION.name: [],
}

LONG = "long"
SHORT = "short"
COLOR = "BOG"
TOLERANCE = (1e-5, 5e-5)
CAMERA_FILTER_COUNT = 3

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

        self._data = {}
        self._data["data"] = {}
        self._job_data = {}
        self._save_data = {}
        self._job = None
        self._data["data"]["host_name"] = socket.gethostname()

        self._done_list = {}

        # self.send_image = False
        self.long_camera_img_data = None
        self.short_camera_img_data = None

        # driver
        self.camera_long = None
        self.camera_short = None
        self.servos = ServosClient()
        self.laser = LaserRangeFinderGenerator.detect_laser_range_finder()

        # config yaml
        self.config_dir = CONFIG_PATH + "/calibration_data"
        self.json_file = ""
        self.last_json_file = ""

        self.run_flag = False

        # state
        self.sub_state = 0

        self.track_target = None
        self.camera_filter_count = 0

        # cameras
        self.cameras = {LONG: None, SHORT: None}
        self.cameras_frame = {LONG: None, SHORT: None}
        self.client_status = {"servos": None, "cameras": None}

        self.job_setting = {
            CS.INITIALIZE_SERVO.name: {},
            CS.CAMERA_SHARPNESS.name: {"camera": "long", "exp_dis": {"long": 40, "short": 5}},
            CS.CAMERAS_ALIGNMENT.name: {"default_h": 1.48},
            CS.CAMERA_LASER_ALIGNMENT.name: {},
            CS.CAMERAS_ANGLE.name: {},
            CS.VERTICAL_SERVO_ZERO.name: {},
            CS.IMU_CALIBRATION.name: {},
        }

    def update_data(self):
        if self._job is not None:
            self._data["data"]["step"] = self._job.name
        # self._data["data"]["job_data"] = self._job_data
            self._data["data"]["job_data"] = {}
            for k, v in self._job_data.items():
                if k in JOB_DATA[self._job.name]:
                    self._data["data"]["job_data"][k] = v
            self._data["data"]["client_status"] = {}
            self._data["data"]["client_status"].update(
                self.client_status)
            self._data["data"]["save_data"] = self._save_data
            self._data["data"]["done"] = self._done_list

    def pub_data(self):
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
        self.reset_camera()
        self.servos.reset()
        self.laser.reset()
        self.run_flag = False
        self.clear_data()
        self.to_IDLE()
        return True

    def on_IDLE(self):
        self.update_client_status()
        self.pub_data()
        if self.is_job():
            self.to_RUNNING()

    def on_RUNNING(self):
        self.update_client_status()
        self.pub_data()
        if self._job == CS.INITIALIZE_SERVO:
            self._do_initialize_servo()
        elif self._job == CS.CAMERA_SHARPNESS:
            self._do_sharpness()
        elif self._job == CS.CAMERAS_ALIGNMENT:
            self._do_cameras_alignment()
        elif self._job == CS.CAMERA_LASER_ALIGNMENT:
            self._do_camera_laser_alignment()
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
        if command == CS.NONE:
            pass
        elif CS.RESET == command:
            self.reset()
        elif CS.SERVOS_DISABLE == command:
            self.logwarn("servo disable")
            self.servos.disable()
        elif CS.SERVOS_ENABLE == command:
            self.logwarn("servo enable")
            self.servos.enable()
        elif CS.LASER_ON == command:
            self.laser.laser_on()
        elif CS.LASER_OFF == command:
            self.laser.laser_off()
        elif CS.RUN == command:
            self.run_job()
        elif CS.SAVE == command:
            self.loginfo("save data..")
            self.save_data()
        elif CS.DONE == command:
            self.job_done()
        elif CS.USE_LONG_CAMERA == command:
            self.loginfo("use long camera")
            self.job_setting[CS.CAMERA_SHARPNESS.name]["camera"] = LONG
            pass
        elif CS.USE_SHORT_CAMERA == command:
            self.loginfo("use short camera")
            self.job_setting[CS.CAMERA_SHARPNESS.name]["camera"] = SHORT
            pass
        else:
            self.turn_to_step(command)
            # if self.goal is not None:
            #     self.goal.got_manual_cali_command = "RB2"
        return True
        # logger.loginfo("handler input ....")

    def reset_camera(self):
        self.camera_filter_count = 0
        for k, v in self.cameras.items():
            if self.cameras[k] is not None:
                self.cameras[k].shutdown()
                self.cameras[k] = None
                time.sleep(0.1)

    def update_client_status(self):
        if self.servos.is_ready():
            self.client_status["servos"] = "OK"
        else:
            self.client_status["servos"] = "not ready"

        if self.cameras_idle():
            self.client_status["cameras"] = "OK"
        else:
            self.client_status["cameras"] = "not ready"

    def clear_data(self):
        self._job_data = {}

    def init_dict_and_update_job_time(self):
        self._save_data[self._job.name] = {}
        self._save_data[self._job.name].update({"time": time.time()})

    def save_data(self):
        self.loginfo("preparing data.")
        if self._job == CS.INITIALIZE_SERVO:
            pass
        else:
            self._save_data[self._job.name] = {}
            self._save_data[self._job.name].update({"time": time.time()})
            for k, v in self._job_data.items():
                if k in JOB_DATA[self._job.name]:
                    self._save_data[self._job.name].update({k: v})
        self.save_json()

    def run_job(self):
        if self._job == CS.INITIALIZE_SERVO:
            self.job_init_servo_replace_setting()
        else:
            self.run_flag = True
        # elif self._job

    def job_done(self):
        if self._job is None:
            return
        if self._job == CS.INITIALIZE_SERVO:
            self.logwarn("killing node ")
            # rosnode.kill_nodes("servos_driver")
            os.system("rosnode kill /servos_driver")
        elif self.job == CS.CAMERA_SHARPNESS:
            self.loginfo("sharpness done.")
        self._done_list[self._job.name] = "true"

    def is_job(self):
        return self._job is not None

    # callback function
    def servo_pdo_cb(self, msg):
        # update servo data
        self._job_data["servo_h"] = msg.encodings_origin[0]
        self._job_data["servo_v"] = msg.encodings_origin[1]

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
    def img2textfromcv2(self, frame):
        # From BGR to RGB
        im = frame[:, :, ::-1]
        im = Image.fromarray(im)
        buf = BytesIO()
        im.save(buf, format="JPEG")
        im_binary = base64.b64encode(buf.getvalue())
        im_text = im_binary.decode()
        return im_text

    def reset_image_flag(self):
        for k, v in self.cameras_frame.items():
            self.cameras_frame[k] = None

    def handler_img_data(self, type, img_data):
        if img_data is not None:
            self.loginfo("Got {} msg".format(type))
            pre_data = {}
            pre_data[type] = {"time": time.time(), "data": img_data}
            return json.dumps(pre_data)
        else:
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
        self.logwarn("save data to {}".format(self.json_file))
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
            CS.INITIALIZE_SERVO.name: {
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

    def cameras_idle(self):
        if self._job == CS.CAMERA_LASER_ALIGNMENT:
            if self.cameras[LONG] is None:
                return False
            else:
                return self.cameras[LONG].is_camera_idle()
        else:
            if (self.cameras[LONG] is None) or (self.cameras[SHORT] is None):
                return False
            else:
                return self.cameras[LONG].is_camera_idle() and self.cameras[SHORT].is_camera_idle()

    def _do_sharpness(self):
        camera_type = self.job_setting[CS.CAMERA_SHARPNESS.name]['camera']
        if self.sub_state == 0:
            self.cameras[LONG] = TrackingCamera(
                "/dev/camera_long", laser_dist=40)
            self.cameras[SHORT] = TrackingCamera(
                "/dev/camera_short", laser_dist=5)
            self._sub_state = 1
        elif self.sub_state == 1:
            if not self.cameras_idle():
                return
            if not self.run_flag:
                return
            self.get_sharpness_result(camera_type)

        # RB 4m for short, RB 40 for long
        pass

    def get_sharpness_result(self, type):
        frame = self.cameras[type].cap()
        # frame = camera.cap()
        if frame is not None:
            self.loginfo("Got {} frame".format(type))
            beacon_res = self.cameras[type].find_beacon(frame, 0.0, COLOR)
            self.cameras[type].draw_beacon(frame, beacon_res)
            self.cameras_frame[type] = self.img2textfromcv2(frame)
            self.loginfo("beacon_res {}".format(beacon_res))
            sharpness_result = self.cameras[type].get_sharpness(
                frame, beacon_res, self.job_setting[CS.CAMERA_SHARPNESS.name]["exp_dis"][type])
            self.loginfo("sharpness good {}".format(sharpness_result))
            color_data, color_result = self.cameras[type].get_color_value(
                frame, beacon_res, color="green")

            self._job_data[type+"_sharpness_score"] = "0" if sharpness_result is None else sharpness_result[0]
            self._job_data[type+"_sharpness_result"] = "BAD" if sharpness_result is None else str(
                sharpness_result[1])
            self._job_data[type+"_color_result"] = str(color_result)
            self._job_data[type+"_color_data"] = str(color_data)[1:-1]
            self.loginfo("color good {}".format(color_result))

    def check_camera_filter(self, offset):
        if abs(offset[0]) < 2e-5 and abs(offset[1]) < 1e-3:
            self.loginfo("!!!!!!!!!!!!!!!!!!!!! arrived {}".format(
                self.camera_filter_count))
            self.camera_filter_count += 1
        else:
            self.camera_filter_count = 0

    def track_done(self, cameras_offset):
        return self.camera_filter_count > cameras_offset

    def _do_cameras_alignment(self):
        if self.sub_state == 0:
            self.cameras[LONG] = TrackingCamera(
                "/dev/camera_long", laser_dist=10)
            self.cameras[SHORT] = TrackingCamera(
                "/dev/camera_short", laser_dist=10)
            self._sub_state = 1
        elif self.sub_state == 1:
            if not self.cameras_idle():
                return
            if not self.run_flag:
                return
            self.laser.laser_on()
            self.sub_state = 2
        elif self.sub_state == 2:
            if self.servos.done:
                self.track_target = (
                    self.job_setting[self._job.name]["default_h"], 0.0)
                self.servo_move(self.track_target, TOLERANCE)
                self.sub_state = 3
        elif self.sub_state == 3:
            if self.servos.done:
                long_res, long_offset = self.track_beacon()
                if long_res:
                    self.sub_state = 4
                    return
        #         self.sub_state = 4
        elif self.sub_state == 4:
            self.loginfo_throttle(
                2, "long camera track done. track with short camera track")
            long_res, long_offset = self.track_beacon(LONG)
            if not long_res:
                self.loginfo(
                    "long camera detected moved. return to substate 3")
                self.sub_state = 3
                return
            self.loginfo("check short camera now")
            short_res, short_offset = self.track_beacon(SHORT)
            if(long_offset is not None) and (short_offset is not None):
                cameras_offset = self.get_camreras_offset(
                    long_offset, short_offset)
                self.loginfo("Got offset. {}".format(cameras_offset))
                # self.init_dict_and_update_job_time()
                # self._job_data["time"] = time.time()
                self._job_data["cameras_offset"] = cameras_offset
                self._job_data["measurement_time"] = time.time()

    def get_camreras_offset(self, long_offset, short_offset):
        return (long_offset[0]-short_offset[0])

    def track_beacon(self, type=LONG):
        offset = self.cameras_handle(type)
        if offset is None:
            return False, None
        if type == LONG:
            self.check_camera_filter(offset)
        self.loginfo(
            "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!the offset is {}".format(offset))
        if self.track_done(1):
            return True, offset
        self.track_target = self.update_target(offset)
        self.loginfo(
            "!!!!!!!!!!!!!!!!!!!!!!!!target is {} now".format(self.track_target))
        self.servos.move_to(self.track_target, TOLERANCE)
        return False, offset

    def update_target(self, offset):
        return (self.servos.radians[0] + offset[0], self.servos.radians[1] + offset[1])

    def servo_move(self, target, tolerances=TOLERANCE):
        self.servos.move_to(target, tolerances)

    def _do_camera_laser_alignment(self):
        if self.sub_state == 0:
            self.cameras[LONG] = TrackingCamera(
                "/dev/camera_long", laser_dist=49)
            self._sub_state = 1
        elif self.sub_state == 1:
            if not self.cameras_idle():
                return
            if not self.run_flag:
                return
            self.laser.laser_on()
            frame = self.cameras[LONG].cap()
            print("Got long frame")
            # beacon_res = self.cameras[LONG].find_beacon(frame, 0.0, "BOG")
            # print(f"beacon_res {beacon_res}")
            # self.cameras[LONG].draw_beacon(frame, beacon_res)
            # angle = self.cameras[LONG].get_beacon_angle(beacon_res)
            # print(f"angle {angle}")
            # sharpness_result = self.cameras[LONG].get_sharpness(frame, beacon_res, exp_dist)
            # print(f"sharpness good {sharpness_result}")
            # color_result = self.cameras[LONG].get_color_value(frame, beacon_res, color="green")
            # print(f"color good {color_result}")
            laser_dot = self.cameras[LONG].find_laser_dot(frame)
            self.loginfo("laser_dot {}".format(laser_dot))
            result = self.cameras[LONG].get_laser_result(frame, laser_dot)
            self.loginfo("result  {}".format(result))
            self._job_data["camera_laser_alignment"] = result
            self.cameras_frame[LONG] = self.img2textfromcv2(frame)

    def get_camera_result(self, type):
        frame = self.cameras[type].cap()
        beacon_res = self.cameras[type].find_beacon(frame, 0.0, COLOR)
        self.cameras[type].draw_beacon(frame, beacon_res)
        self.cameras_frame[type] = self.img2textfromcv2(frame)
        angle = self.cameras[type].get_beacon_angle(beacon_res)
        return angle

    def cameras_handle(self, type=LONG):
        if type != LONG:
            return self.get_camera_result(SHORT)
        else:
            long_anle = self.get_camera_result(LONG)
            if long_anle is None:
                return self.get_camera_result(SHORT)
            else:
                return long_anle


if __name__ == "__main__":
    rospy.init_node("calibration_controller")
    c = CalibrationController("calibration_controller", 10)
    c.run()
