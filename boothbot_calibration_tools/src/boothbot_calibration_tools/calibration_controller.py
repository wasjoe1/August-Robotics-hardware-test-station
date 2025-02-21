#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import math
import numpy as np
import os
import time
import oyaml
import socket
import glob
import json
import socket
import shutil


from datetime import datetime

from boothbot_calibration_tools.constants import CalibrationStates as MS
from boothbot_calibration_tools.constants import CalibrationCommand as CS
from boothbot_calibration_tools.constants import JobStatus as JS

# from boothbot_common.settings import BOOTHBOT_GET_CONFIG
from boothbot_config.device_settings import DEVICE_SETTINGS_FILE_PATH
from boothbot_config.device_settings import CONFIG_PATH
# from boothbot_perception.track.tracking_camera import TrackingCamera
from boothbot_calibration_tools.calibration_camera_tracking import CaliTrackingCamera
from boothbot_calibration_tools.calibration_camera_tracking_base import CaliTrackingCameraBase
from boothbot_calibration_tools.dep_camera_cali_tool import ImageProcessing

import boothbot_msgs.srv as bbsrvs
import boothbot_msgs.msg as bbmsgs
import std_msgs.msg as stmsgs

from boothbot_perception.track import settings

from boothbot_perception.check.roi_calibration import roi_calibration

from boothbot_common.settings import BOOTHBOT_GET_CONFIG

from boothbot_common.module_base import ModuleBase

from boothbot_driver.servos_client import ServosClient
from boothbot_msgs.ros_interfaces import (
    APPS_CALIBRATION_SRV_CMD,
    APPS_CALIBRATION_STATUS,
    APPS_CALIBRATION_DATA,
    DRIVERS_SERVOS_PDO,
    DRIVERS_CHASSIS_SRV_CMD,
    DRIVERS_CHASSIS_IMU,
)


from guiding_beacon_system_msgs.ros_interfaces import (
    DRIVERS_INCLINOMETER_INCLINATION_FILTERED_RAD
)

from boothbot_common.interface_with_type import InterfaceWithType

DRIVERS_INCLINOMETER_INCLINATION_CB = InterfaceWithType('/drivers/inclinometer/inclination_cb', stmsgs.Float32MultiArray)
DRIVERS_INCLINOMETER_INCLINATION_CB_FILTERED = InterfaceWithType('/drivers/inclinometer/inclination_cb_filtered', stmsgs.Float32MultiArray)
DRIVERS_INCLINOMETER_INCLINATION_CB_FILTERED_RAD = InterfaceWithType('/drivers/inclinometer/inclination_cb_filtered_rad', stmsgs.Float32MultiArray)

from boothbot_msgs.srv import (
    Command
)

from augustbot_msgs.srv import (
    CommandRequest
)

from std_msgs.msg import String, Int16
# from boothbot_msgs.srv import Command

from boothbot_calibration_tools.settings import (
    JOB_DATA,
    SAVE_DATA_TITLE,
    TRANSITIONS,
    CALI_ARG,
    SAVE_ARG,
    JOS_SETTINGS,
    LAST_SAVE_TILE,
    SHORT,
    LONG,
    COLOR,
    CAMERA_FILTER_COUNT,
    JOB_DONE_STATUS,
    APPS_CALIBRATION_SET_PARAM
)

from boothbot_calibration_tools.constants import(
    CB_INCLI_CMD,
    CB_INCLI_STATE,
    CB_INCLI_RES
)

from boothbot_calibration_tools.utils import (
    get_tolerance,
    img2textfromcv2,
    occlusion_image
)

TRACKER_CONFIG = BOOTHBOT_GET_CONFIG(name="tracker_driver")
GS_CAMERA_VERTICAL_DIST = TRACKER_CONFIG["long_cam_laser_dist"] + \
    TRACKER_CONFIG["short_cam_laser_dist"]


class CalibrationController(ModuleBase):
    def __init__(self,
                 name, rate, states=None, transitions=None, commands=None, status_inf=None,
                 srv_cmd_inf=None, need_robot_status=False, error_codes=None,
                 laser=None, max_encoding=None, painter=None, have_short_camera=True):
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
        self.max_encoding = max_encoding

        # ROS
        self.puber_data = APPS_CALIBRATION_DATA.Publisher()
        DRIVERS_SERVOS_PDO.Subscriber(self.servo_pdo_cb)
        DRIVERS_CHASSIS_IMU.Subscriber(self.imu_cb)
        DRIVERS_INCLINOMETER_INCLINATION_FILTERED_RAD.Subscriber(self._incli_base_cb)
        DRIVERS_INCLINOMETER_INCLINATION_CB_FILTERED_RAD.Subscriber(self._incli_cb_cb)
        self.cb_incli_pub = rospy.Publisher(CB_INCLI_CMD, String, queue_size=10)
        rospy.Subscriber(CB_INCLI_STATE, Int16, self.cb_cb_incli_state)
        rospy.Subscriber(CB_INCLI_RES, String, self.cb_cb_incli_res)
        self.cb_incli_state = None

        rospy.Service(APPS_CALIBRATION_SET_PARAM, Command, self._set_param)

        # JOB
        self._data = {}
        self._data["data"] = {}
        self._job_data = {}
        self._save_data = {}
        self._user_gui_save_data = {}
        self._job = None
        self._data["data"]["host_name"] = socket.gethostname()
        self.is_gs = socket.gethostname().lower().startswith("gs")

        self._done_list = {}

        # driver
        self.servos = ServosClient()
        # self.laser = LaserRangeFinderGenerator.detect_laser_range_finder()
        self.laser = laser

        self.color_range = [(getattr(settings, "CALI_LOWER"),getattr(settings, "CALI_UPPER"))]

        # config yaml
        self.config_dir = CONFIG_PATH + "/calibration_data"
        self.json_file = ""
        self.last_json_file = ""

        self.run_flag = False
        self.test_track = False
        # state
        self.sub_state = 0

        self.track_target = None
        self.camera_filter_count = 0

        # cameras
        self.cameras = {LONG: None, SHORT: None}
        self.cameras_frame = {LONG: None, SHORT: None}
        self.client_status = {"servos": None, "cameras": None, "job_status": JS.INIT.name}
        self.servos_save_encoder = []
        self.current_rad = (0.0, 0.0)
        self._tolerance = get_tolerance()
        self.logwarn("tolerance auto seleted {}.".format(self._tolerance))
        self.have_short_camera = have_short_camera

        self.cameras_angle = []
        self.vertical_encoder = []
        self.horizontal_offset = []
        self._job_vertical_iter = 0
        self._job_dep_cam_iter = 0
        self.euler_camera_base_to_base_list = []
        self.translation_camera_base_to_base_list = []
        # CB and inclination
        self.incli_base_data = None
        self.incli_cb_data = None
        self.cb_row = None
        self.cb_pitch = None
        self.imu_data = {
            "imu_x": None,
            "imu_y": None,
            "imu_z": None,
            "imu_w": None
        }

        self.painter = painter
        self.image_processing = None

        self.job_setting = JOS_SETTINGS

        self.save_data_title = LAST_SAVE_TILE

    def update_data(self):
        if self._job is not None:
            self._data["data"]["step"] = self._job.name
        # self._data["data"]["job_data"] = self._job_data
            self._data["data"]["job_data"] = {}
            if len(self._job_data) > 1:
                for k, v in self._job_data.items():
                    if k in JOB_DATA[self._job.name]:
                        self._data["data"]["job_data"][k] = v
            self._data["data"]["client_status"] = {}
            self._data["data"]["client_status"].update(
                self.client_status)
            # self._data["data"]["save_data"] = self._save_data
            self._data["data"]["save_data"] = self._user_gui_save_data
            self._data["data"]["done"] = self._done_list

    def pub_data(self):
        for k, v in self.cameras_frame.items():
            img_res = self.handler_img_data(k, v)
            if img_res is not None:
                self.loginfo("pub {} img ".format(k))
                self.puber_data.publish(img_res)
            rospy.sleep(0.02)
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
        self.clear_data()
        self.to_IDLE()
        self.cb_incli_pub.publish(String("reset"))
        return True

    def clear_data(self):
        self.run_flag = False
        self._job_data = {}
        self.cameras_angle = []
        self.cameras_angle = []
        self.vertical_encoder = []
        self.horizontal_offset = []
        self.euler_camera_base_to_base_list = []
        self.translation_camera_base_to_base_list = []
        self._job_vertical_iter = 0
        self._job_dep_cam_iter = 0
        self.camera_filter_count = 0
        self.test_track = False
        self.cb_incli_state = None

        # settable param
        self.laser_distance = None
        self.long_camera_exposure = None
        self.depth_camera_cali_param = None
        self.laser_align_no_occlusion_size = None

        self.set_job_status(JS.INIT.name)


    def set_job_status(self, status):
        self.client_status["job_status"] = status

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
        elif self._job == CS.CAMERAS_ALIGNMENT and self.have_short_camera:
            self._do_cameras_alignment()
        elif self._job == CS.CAMERA_LASER_ALIGNMENT:
            self._do_camera_laser_alignment()
        elif self._job == CS.CAMERAS_ANGLE and self.have_short_camera:
            self._do_cameras_angle()
        elif self._job == CS.VERTICAL_SERVO_ZERO:
            self._do_vertical_offset()
        elif self._job == CS.IMU_CALIBRATION:
            if not self.is_gs:
                self._do_imu_calibration()
        elif self._job == CS.HORIZONTAL_OFFSET:
            if not self.is_gs:
                self._do_horizontal_offset()
        elif self._job == CS.MARKING_ROI:
            if not self.is_gs:
                self._do_marking_camera_roi()
        elif self._job == CS.CB_INCLINATION:
            if not self.is_gs:
                self._do_cb_inclination()
        elif self._job == CS.DEPTH_CAMERA:
            if not self.is_gs:
                self._do_depth_camera()

    def initialize(self):
        self.loginfo("initializing ....")
        self.last_json_file = self.get_last_json()
        self.check_yaml_dir()
        try:
            self.get_last_data()
        except Exception as e:
            self.logerr("Got last data error....")
        now = datetime.now()
        date_time = now.strftime("%m-%d-%Y-%H-%M-%S")
        self.json_file = self.config_dir + '/' + socket.gethostname() + "-" + \
            date_time

        for client in (self.servos, self.laser):
            self.loginfo("connecting to {}".format(client.name))
            if not client.connect(timeout=0.5):
                self.logwarn("Initializing {} failed!!".format(client.name))
                return False
            else:
                self.loginfo("connected succeeded{}".format(client.name))
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
            self.servos_enable()
        elif CS.LASER_ON == command:
            self.laser.laser_on()
        elif CS.LASER_OFF == command:
            self.laser.laser_off()
        elif CS.RUN == command:
            self.run_job()
        elif CS.SAVE == command:
            self.save_data()
        elif CS.DONE == command:
            self.job_done()
        elif CS.USE_LONG_CAMERA == command:
            self.loginfo("use long camera")
            self.job_setting[CS.CAMERA_SHARPNESS.name]["camera"] = LONG
        elif CS.USE_SHORT_CAMERA == command:
            self.loginfo("use short camera")
            self.job_setting[CS.CAMERA_SHARPNESS.name]["camera"] = SHORT
        elif CS.TEST_TRACK == command:
            self.test_track = True
            # self.loginfo()
        else:
            self.turn_to_step(command)
        return True

    def servos_enable(self):
        self.servos.enable()
        return True

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
        elif self.servos.error:
            self.client_status["servos"] = "ERROR"
        else:
            self.client_status["servos"] = "not ready"

        if self.cameras_idle():
            self.client_status["cameras"] = "OK"
        else:
            self.client_status["cameras"] = "not ready"

        if self.run_flag:
            if JOB_DONE_STATUS[self._job] == self.sub_state:
                self.set_job_status(JS.DONE.name)
            else:
                self.set_job_status(JS.RUNNING.name)

    def set_job_current_time(self):
        self._job_data["measurement_time"] = time.time()

    def save_data(self):
        self.loginfo("preparing data.")
        self._save_data = {}
        self._user_gui_save_data[self._job.name] = {}
        self._save_data[self._job.name] = {}
        if self._job.name in (CS.INITIALIZE_SERVO.name, CS.CAMERA_SHARPNESS.name):
            self.set_job_current_time()
        for k, v in self._job_data.items():
            if k in SAVE_DATA_TITLE[self._job.name]:
                self._save_data[self._job.name].update({k: v})
                self._user_gui_save_data[self._job.name].update({k: v})
        self.save_json()

    def run_job(self):
        if self._job == CS.INITIALIZE_SERVO:
            self.job_init_servo_replace_setting()
        elif self._job == CS.IMU_CALIBRATION:
            self.loginfo("imu calibration now.")
            self.cali_imu(CALI_ARG)
            self.loginfo("imu calibration now.")
        else:
            self.run_flag = True
        # elif self._job

    def job_done(self):
        if self._job is None:
            return
        if self._job == CS.INITIALIZE_SERVO:
            self.logwarn("killing node ")
            self.kill_servos_node()
        elif self._job == CS.IMU_CALIBRATION:
            self.cali_imu(SAVE_ARG)
            self.logwarn("save imu calibration")
            # rosnode.kill_nodes("servos_driver")
            # os.system("rosnode kill /servos_driver")
        self._done_list[self._job.name] = "true"

    def kill_servos_node(self):
        return True

    def is_job(self):
        return self._job is not None

    # callback function
    def servo_pdo_cb(self, msg):
        # update servo data
        # self.set_job_current_time()
        self._job_data["servo_h"] = msg.encodings_origin[0]
        self._job_data["servo_v"] = msg.encodings_origin[1]
        self.current_rad = (msg.radians[0], msg.radians[1])

    def imu_cb(self, msg):
        self.imu_data["imu_x"] = msg.orientation.x
        self.imu_data["imu_y"] = msg.orientation.y
        self.imu_data["imu_z"] = msg.orientation.z
        self.imu_data["imu_w"] = msg.orientation.w

    def turn_to_step(self, CS):
        if (self._job) != CS or (self._job is None):
            self.loginfo("step change to {}, resetting".format(CS.name))
            self.reset()
            self._job = CS
            self.to_RUNNING()
            return

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
        list_dir = os.listdir(self.config_dir)
        self._data["data"]["last_data"] = {}
        for f in list_dir:
            for t in self.save_data_title:
                if t in f:
                    self.loginfo("open file {}".format(f))
                    file_path = self.config_dir+"/" + f
                    try:
                        with open(file_path, "r") as data_file:
                            # last_data = json.loads
                            last_data = json.load(data_file)
                            self._data["data"]["last_data"].update(last_data)
                    except FileNotFoundError as not_found_e:
                        self.loginfo("{} not found...".format(file_path))
                    except json.decoder.JSONDecodeError as file_e:
                        self.loginfo("load file {} error...".format(file_path))

    def save_json(self):
        save_path = self.json_file+"_"+self._job.name+".json"
        self.logwarn("save data to {}".format(save_path))

        list_dir = os.listdir(self.config_dir)
        for f in list_dir:
            if self._job.name+".json" in f:
                shutil.move(self.config_dir+"/"+f,
                            self.config_dir+"/old_data/"+f)

        with open(save_path, 'w') as f:
            json.dump(self._save_data, f)

    def check_yaml_dir(self):
        if not os.path.exists(self.config_dir):
            self.loginfo("Directory is created.")
            os.mkdir(self.config_dir)
        if not os.path.exists(self.config_dir+"/old_data"):
            self.loginfo("Directory is created.")
            os.mkdir(self.config_dir+"/old_data")

    def job_init_servo_replace_setting(self):
        with open(DEVICE_SETTINGS_FILE_PATH) as f:
            doc = oyaml.load(f)

        doc['servos_driver']['servo_parameter']['horizontal']['zero_offset'] = self._job_data["servo_h"]
        doc['servos_driver']['servo_parameter']['vertical']['zero_offset'] = self._job_data["servo_v"]

        with open(DEVICE_SETTINGS_FILE_PATH, 'w') as f:
            oyaml.dump(doc, f, default_flow_style=False)

    # JOB

    def save_angle(self, data):
        with open(DEVICE_SETTINGS_FILE_PATH) as f:
            doc = oyaml.load(f)
        self.loginfo("save data to device settting {}".format(data))
        doc['tracker_driver']['long_short_cam_angle_offset'] = float(data)
        with open(DEVICE_SETTINGS_FILE_PATH, 'w') as f:
            oyaml.dump(doc, f, default_flow_style=False)

    def _do_initialize_servo(self):
        pass

    def cameras_idle(self):
        if (self.cameras[LONG] is None) or (self.cameras[SHORT] is None):
            return False
        else:
            return self.cameras[LONG].is_camera_idle() and self.cameras[SHORT].is_camera_idle()

    def get_cameras_frames(self):
        for type in (LONG, SHORT):
            frame = self.cameras[type].cap()
            if frame is None:
                continue
            if self._job in [CS.CAMERA_LASER_ALIGNMENT]:
                # self.loginfo("Got {} frame {}".format(type, frame))
                self.cameras_frame[type] = img2textfromcv2(frame, False)
            else:
                self.cameras_frame[type] = img2textfromcv2(frame, True)

    def _do_sharpness(self):
        camera_type = self.job_setting[CS.CAMERA_SHARPNESS.name]['camera']
        # self.set_camera_expo(LONG, 4000)
        if self.sub_state == 0:
            if self.init_cameras(40, 5):
                self.sub_state = 1
        elif self.sub_state == 1:
            if not self.cameras_idle():
                return
            self.sub_state = 2
        elif self.sub_state == 2:
            self.set_camera_expo(LONG, 4000)
            self.sub_state = 3
        elif self.sub_state == 3:
            if not self.run_flag:
                self.get_cameras_frames()
                return
            self.get_sharpness_result(camera_type)

    def get_sharpness_result(self, type):
        frame = self.cameras[type].cap()
        if frame is not None:
            self.loginfo("Got {} frame".format(type))
            beacon_res = self.cameras[type].find_beacon(frame, 0.0, COLOR, self.color_range)
            self.cameras[type].draw_beacon(frame, beacon_res)
            self.cameras_frame[type] = img2textfromcv2(frame, True)
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
        if abs(offset[0]) < self._tolerance[0] and abs(offset[1]) < self._tolerance[1]:
            self.loginfo("!!!!!!!!!!!!!!!!!!!!! arrived {}".format(
                self.camera_filter_count))
            self.camera_filter_count += 1
        else:
            self.camera_filter_count = 0

    def track_done(self, cameras_offset):
        return self.camera_filter_count > cameras_offset

    def _do_cameras_alignment(self):
        if self.sub_state == 0:
            self.init_cameras(5, 5)
            self.sub_state = 1
        elif self.sub_state == 1:
            if not self.cameras_idle():
                return
            if not self.run_flag:
                self.get_cameras_frames()
                return
            self.laser.laser_on()
            self.sub_state = 2
        elif self.sub_state == 2:
            if self.servos.done:
                long_res, long_offset = self.track_beacon(LONG, 1)
                if long_res:
                    self.sub_state = 3
                    return
        elif self.sub_state == 3:
            self.loginfo_throttle(
                2, "long camera track done. track with short camera track")
            long_res, long_offset = self.track_beacon(LONG, 1)
            if not long_res:
                self.loginfo(
                    "long camera detected moved. return to substate 3")
                self.sub_state = 2
                return
            self.loginfo("check short camera now")
            short_res, short_offset = self.track_beacon(SHORT, 1)
            if(long_offset is not None) and (short_offset is not None):
                cameras_offset = self.get_camreras_offset(
                    long_offset, short_offset)
                self.loginfo("Got offset. {}".format(cameras_offset))
                self.set_job_current_time()
                self._job_data["cameras_offset"] = cameras_offset
                op_info = {}
                if cameras_offset >= 0:
                    op_info = {"small": "right"}
                else:
                    op_info = {"small": "left"}
                self._job_data["cameras_offset_op_info"] = op_info

    def get_camreras_offset(self, long_offset, short_offset):
        return (long_offset[0]-short_offset[0])

    def track_beacon(self, type=LONG, camera_filter_time=3, dis=0.0, compensation=False, angle=None):
        offset = self.cameras_handle(type, dis, compensation, angle)
        if offset is None:
            return False, None
        if type == LONG:
            self.check_camera_filter(offset)
        self.loginfo(
            "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!the offset is {}".format(offset))
        if self.track_done(camera_filter_time):
            return True, offset
        self.track_target = self.update_target(offset)
        self.loginfo(
            "!!!!!!!!!!!!!!!!!!!!!!!!target is {} now".format(self.track_target))
        self.servos.move_to(self.track_target, self._tolerance)
        return False, offset

    def update_target(self, offset):
        return (self.servos.radians[0] + offset[0], self.servos.radians[1] + offset[1])

    def servo_move(self, target):
        self.servos.move_to(target, self._tolerance)
    
    def get_servo_radians(self):
        self.track_target = (self.servos.radians[0], self.servos.radians[1])

    def _do_camera_laser_alignment(self):
        if self.have_short_camera:
            self.laser.laser_on()
        if self.cameras[LONG] is not None and self.long_camera_exposure is not None:
            if self.cameras_idle():
                # self.loginfo("")
                self.set_camera_expo(LONG, int(self.long_camera_exposure))
                self.long_camera_exposure = None
                return
        # check laser distance changed by user..
        if self.laser_distance is not None:
            self.loginfo("param laser distance changed, will reset laser camera alignment..")
            self.reset_camera()
            self.sub_state = 0
        if self.sub_state == 0:
            if self.laser_distance is None:
                self.loginfo("set laser distance to default 49")
                if self.init_cameras(49, 5):
                    self._sub_state = 1
            else:
                self.loginfo("set laser distance to {}".format(self.laser_distance))
                if self.init_cameras(self.laser_distance, 5):
                    self.laser_distance = None
                    self._sub_state = 1
        elif self._sub_state == 1:
            self.laser.laser_on()
            self._sub_state = 2
        elif self._sub_state == 2:
            if not self.have_short_camera:
                self.set_camera_expo(LONG, 8000)
            self._sub_state = 3
        elif self.sub_state == 3:
            if not self.cameras_idle():
                return
            if not self.run_flag:
                self.get_cameras_frames()
                return
            frame = self.cameras[LONG].cap()
            # Add a occlusion on image to cover noise which can affect the detection of red dot. 
            frame = occlusion_image(frame, self.laser_align_no_occlusion_size)
            print("Got long frame")
            laser_dot = self.cameras[LONG].find_laser_dot(frame)
            # self.loginfo("laser_dot {}".format(laser_dot))
            result = self.cameras[LONG].get_laser_result(frame, laser_dot)
            # self.loginfo("result  {}".format(result))
            self._job_data["camera_laser_alignment"] = result
            self.cameras_frame[LONG] = img2textfromcv2(frame, False)
            if self.test_track:
                pass
                if self.servos.done:
                    self.track_beacon(LONG, CAMERA_FILTER_COUNT)

    def init_camera(self, type, dis, ena):
        # TODO, support lnp6
        try:
            if self.cameras[type] is None:
                # TODO, use local Camera driver
                if ena is False:
                    self.cameras[type] = CaliTrackingCameraBase(
                        "/dev/camera_"+type.lower(), laser_dist=dis)
                else:
                    self.cameras[type] = CaliTrackingCamera(
                        "/dev/camera_"+type.lower(), laser_dist=dis)

        except Exception as e:
            self.logerr(e)
            self.logerr("{} camera init error".format(type))
            self.cameras[type] = None
            return False
        return True

    def init_cameras(self, long_dist, short_dist):
        long_init_res = self.init_camera(LONG, long_dist, True)
        short_init_res = self.init_camera(SHORT, short_dist, self.have_short_camera)
        if long_init_res and short_init_res:
            return True
        else:
            return False
    
    def set_camera_expo(self, type, expo):
        if expo is not None:
            self.loginfo("Set {} exposure to {}".format(type, expo))
            self.cameras[type].set_expo(expo)

    def get_camera_result(self, type, dis=0.0, compensation=False, long_shrot_angle=None):
        frame = self.cameras[type].cap()
        beacon_res = self.cameras[type].find_beacon(frame, dis, COLOR, self.color_range)
        self.cameras[type].draw_beacon(frame, beacon_res)
        self.cameras_frame[type] = img2textfromcv2(frame, True)
        self.loginfo("long short angle {}".format(long_shrot_angle))
        angle = self.cameras[type].get_beacon_angle(
            beacon_res, dis, compensation, long_shrot_angle)
        return angle

    def cameras_handle(self, type=LONG, dis=0.0, compensation=False, angle=None):
        # TODO, support lnp6
        if type != LONG:
            return self.get_camera_result(SHORT, dis, compensation, angle)
        else:
            long_anle = self.get_camera_result(LONG, dis, compensation, angle)
            if long_anle is None and self.have_short_camera:
                return self.get_camera_result(SHORT, dis, compensation, angle)
            else:
                return long_anle

    def _do_cameras_angle(self):
        # TODO, support lnp6
        if self.sub_state == 0:
            if self.init_cameras(4, 4):
                self.sub_state = 1
        elif self.sub_state == 1:
            if not self.cameras_idle():
                return
            if not self.run_flag:
                self.get_cameras_frames()
                return
            self.laser.laser_on()
            self.sub_state = 2
        elif self.sub_state == 2:
            if self.servos.done:
                long_res, long_offset = self.track_beacon(camera_filter_time=CAMERA_FILTER_COUNT)
                if long_res:
                    self.sub_state = 3
                    return
        elif self.sub_state == 3:
            self.loginfo_throttle(
                2, "long camera track done. track with short camera track")
            long_res, long_offset = self.track_beacon(LONG, CAMERA_FILTER_COUNT)
            if not long_res:
                self.loginfo(
                    "long camera detected moved. return to substate 3")
                self.sub_state = 2
                return
            self.loginfo("check short camera now")
            short_res, short_offset = self.track_beacon(SHORT, CAMERA_FILTER_COUNT)
            try:
                res, dis = self.laser.get_distance()
            except Exception as e:
                self.logerr("laser measure error. {}".format(e))
                return
            projection_dis = dis*math.cos(long_offset[1])
            self.loginfo("the laser dis is {}".format(dis))
            if not res:
                return

            if(long_offset is not None) and (short_offset is not None):
                d2 = math.atan2(GS_CAMERA_VERTICAL_DIST, projection_dis)
                d1 = short_offset[1]
                cameras_angle = -d1-d2

                self.loginfo("d1: {}, d2: {}, d3: {}".format(
                    d1, d2, (cameras_angle)))
                self.cameras_angle.append(cameras_angle)

            self.laser.reset()
            if len(self.cameras_angle) > 10:
                arr = np.array(self.cameras_angle)
                avg = np.average(arr)
                self._job_data["cameras_angle"] = avg
                self.set_job_current_time()
                self.sub_state = 4
        elif self.sub_state == 4:
            self.loginfo_throttle(5, "{} job done".format(self._job.name))
            if self.test_track:
                self.save_angle(self._job_data["cameras_angle"])
                self.sub_state = 5
        elif self.sub_state == 5:
            self.sub_state = 6
        elif self.sub_state == 6:
            if not self.cameras_idle():
                return
            else:
                self.sub_state = 7
        elif self.sub_state == 7:
            self.laser.laser_on()
            self.reset_camera_filter()
            self.track_beacon(SHORT, 3, 4, True, float(
                self._job_data["cameras_angle"]))

    def reset_camera_filter(self):
        self.camera_filter_count = 0

    def _do_vertical_offset(self):
        if self.sub_state == 0:
            if self.init_cameras(7, 7):
                self.sub_state = 1
        elif self.sub_state == 1:
            if not self.cameras_idle():
                return
            if not self.run_flag:
                self.get_cameras_frames()
                return
            # if not self.run_flag:
            #     return
            self.laser.laser_on()
            self.sub_state = 2
        elif self.sub_state == 2:
            # for k, v in self.job_setting[CS.VERTICAL_SERVO_ZERO.name].items():
            if self.servos.done:
                self.track_target = (
                    self.job_setting[self._job.name][self._job_vertical_iter], 0.0)
                self.servo_move(self.track_target)
                self.sub_state = 3
        elif self.sub_state == 3:
            if self.servos.done:
                long_res, long_offset = self.track_beacon(camera_filter_time=CAMERA_FILTER_COUNT)
                if long_res:
                    self.sub_state = 4
                    return
        elif self.sub_state == 4:
            self.loginfo_throttle(
                2, "long camera track done.")
            long_res, long_offset = self.track_beacon(LONG, CAMERA_FILTER_COUNT)
            if not long_res:
                self.loginfo(
                    "long camera detected moved. return to substate 3")
                self.sub_state = 3
                return
            if long_res:
                self.vertical_encoder.append(self._job_data["servo_v"])
                self.loginfo(self.servos.encoders)
                if len(self.vertical_encoder) >= 5*(self._job_vertical_iter+1):
                    self._job_vertical_iter += 1
                    self.sub_state = 2
            self.loginfo("job vertical iter: {}, len(vertical_encoder): {}, {}".format(
                self._job_vertical_iter, len(self.vertical_encoder), self.vertical_encoder))
            if self._job_vertical_iter >= 2:
                arr1 = np.array(self.vertical_encoder[0:4])
                avg1 = np.average(arr1)
                arr2 = np.array(self.vertical_encoder[5:9])
                avg2 = np.average(arr2)
                arr = np.array(self.vertical_encoder)
                avg = np.average(arr)
                if math.fabs(avg1-avg2) > self.max_encoding/2:
                    avg = avg + self.max_encoding/2
                if avg >= self.max_encoding:
                    avg = avg - self.max_encoding
                self.loginfo("verticalencoder is {}".format(avg))
                self._job_data["vertical_offset"] = avg
                self.set_job_current_time()
                self.sub_state = 5
        elif self.sub_state == 5:
            self.loginfo_throttle(2, "{} job done.".format(self._job.name))

    def set_incli_data(self):
        if self.incli_base_data is not None:
            self._job_data["base_offset_x"] = self.incli_base_data.data[0]
            self._job_data["base_offset_y"] = self.incli_base_data.data[1]
        if self.incli_cb_data is not None:
            self._job_data["cb_offset_x"] = self.incli_cb_data.data[0]
            self._job_data["cb_offset_y"] = self.incli_cb_data.data[1]

    def _do_imu_calibration(self):
        self.set_incli_data()
        if self.sub_state == 0:
            self.sub_state = 1
        elif self.sub_state == 1:
            self._job_data.update(self.imu_data)

    def _do_cb_inclination(self):
        self.set_incli_data()
        if self.sub_state == 0:
            if not self.run_flag:
                return
            self.sub_state = 1
        elif self.sub_state == 1:
            self.loginfo("start this job...")
            data = String()
            data.data = "start"
            self.cb_incli_pub.publish(data)
            self.sub_state = 2
        elif self.sub_state == 2:
            self.set_job_state("RUNNING")
            if self.cb_incli_state == 2:
                self.sub_state = 3
            elif self.cb_cb_incli_state == 99:
                self.logerr_throttle(2, "cb incli error...")
        elif self.sub_state == 3:
            self.set_job_state("DONE")
            if self.cb_row is not None and self.cb_pitch is not None:
                self._job_data["roll"] = self.cb_row
                self._job_data["pitch"] = self.cb_pitch
                self.set_job_current_time()
            self.loginfo_throttle(2,"cb inclination successed..")
        
    def cb_cb_incli_state(self,msg):
        self.cb_incli_state = msg.data

    def cb_cb_incli_res(self, msg):
        data = msg.data
        if data.startswith("inc_x"):
            self.cb_row = float(data.split('_')[2])
        if data.startswith("inc_y"):
            self.cb_pitch = float(data.split('_')[2])

    def cali_imu(self, prarm):
        cmd = CommandRequest()
        cmd.command = "IMU"
        cmd.parameter = prarm
        res = DRIVERS_CHASSIS_SRV_CMD.service_call(cmd)
        # if res.

    def _do_horizontal_offset(self):
        self.loginfo("horizontal offset calibration deprecate")
        
    #     if self.sub_state == 0:
    #         if self.init_cameras(4, 4):
    #             self.sub_state = 1
    #     elif self.sub_state == 1:
    #         if not self.cameras_idle():
    #             return
    #         if not self.run_flag:
    #             self.get_cameras_frames()
    #             return
    #         self.laser.laser_on()
    #         self.sub_state = 2
    #     elif self.sub_state == 2:
    #         if self.servos.done:
    #             long_res, long_offset = self.track_beacon(camera_filter_time=CAMERA_FILTER_COUNT)
    #             if long_res:
    #                 self.sub_state = 3
    #                 return
    #     elif self.sub_state == 3:
    #         self.loginfo_throttle(
    #             2, "long camera track done.")
    #         long_res, long_offset = self.track_beacon(camera_filter_time=CAMERA_FILTER_COUNT)
    #         if long_res:
    #             self.horizontal_offset.append(self._job_data["servo_h"])
    #         else:
    #             self.sub_state = 2
    #         if len(self.horizontal_offset) > 5:
    #             arr = np.array(self.horizontal_offset)
    #             avg = np.average(arr)
    #             self.loginfo("verticalencoder is {}".format(avg))
    #             self.set_job_current_time()
    #             self._job_data["horizontal_offset"] = avg
    #             self.sub_state = 4
    #     elif self.sub_state == 4:
    #         self.loginfo_throttle(2, "horizontal job done.")


    def _incli_base_cb(self, msg):
        self.incli_base_data = msg
    
    def _incli_cb_cb(self, msg):
        self.incli_cb_data = msg

    def _set_param(self, cmd):
        self.loginfo("Got cammand: {}".format(cmd))
        if "=" in cmd.command:
            [k,v] = cmd.command.split("=")
            try:
                if k == "L":
                    self.loginfo("set {} to {}m on laser camera alignment.".format(v, k))
                    self.laser_distance = float(v)
                elif k == "E":
                    self.loginfo("set camera exposure to {}.".format(v))
                    self.long_camera_exposure = float(v)
                elif k == "D":
                    self.loginfo("set depth camera cali param to {}.".format(v))
                    self.depth_camera_cali_param = float(v)
                elif k == "LM":
                    self.loginfo("set laser occlusion pixel param to {}.".format(v))
                    self.laser_align_no_occlusion_size = int(float(v))
            except Exception as e:
                self.logerr(e)
                self.logerr("exception occur when got param...")                    

    def _do_marking_camera_roi(self):
        if self.sub_state == 0:
            if not self.painter.connect():
                return
            # camera init
            self.sub_state = 1
        elif self.sub_state == 1:
            # waiting painter home
            if not self.painter.is_done():
                return
            self.painter.set_goal_enable()
            self.sub_state = 2
        elif self.sub_state == 2:
            if not self.painter.is_done():
                return
            if not self.run_flag:
                return
            self.sub_state = 3
        elif self.sub_state == 3:
            self.set_job_state("RUNNING")
            self.loginfo("start to roi calibration")
            res = roi_calibration()
            if res is None:
                self.loginfo("Got None result...")
                return
            # calculate the param..
            self.set_job_current_time()
            (x,y,w,h) = res
            self._job_data["x"] = x
            self._job_data["y"] = y
            self._job_data["w"] = w
            self._job_data["h"] = h
            self.sub_state = 4
        elif self.sub_state == 4:
            self.set_job_state("DONE")
            self.loginfo_throttle(4, "roi  calibration done.")

    def _do_depth_camera(self):
        # self._job_data["tag_size"] = self.depth_camera_cali_param
        if self.sub_state == 0:
            self.image_processing = ImageProcessing()
            self.sub_state = 1
            return
        #TODO
        elif self.sub_state == 1:
            if self.depth_camera_cali_param is None:
                return
            self._job_data["tag_size"] = self.depth_camera_cali_param
            self._job_data["has_set_tag"] = "has set"
            self.sub_state = 2
        elif self.sub_state == 2:
            if not self.run_flag:
                return
            self.sub_state = 3
        elif self.sub_state == 3:
            self.set_job_state("RUNNING, cali time: {}".format(self._job_dep_cam_iter))
            if self._job_dep_cam_iter <= 10:
                image_name = "{}.jpg".format(self._job_dep_cam_iter)
                res = self.image_processing.save_image(image_name)
                time.sleep(0.5)
                if res is True:
                    euler_camera_base_to_base, translation_camera_base_to_base = self.image_processing.get_tf_from_apriltag(image_name, self.depth_camera_cali_param)
                    if euler_camera_base_to_base is None:
                        return
                    self._job_dep_cam_iter+=1
                    self.euler_camera_base_to_base_list.append(euler_camera_base_to_base)
                    self.translation_camera_base_to_base_list.append(translation_camera_base_to_base)
                    res = False
            else:
                self.sub_state = 4
        elif self.sub_state == 4:
            self.set_job_state("DONE")
            res1, res2 = self.image_processing.compute_the_average(self.euler_camera_base_to_base_list,self.translation_camera_base_to_base_list)
            self._job_data["yaw"] = 0.0
            self._job_data["roll"] = res1[0]
            self._job_data["pitch"] = res1[1]
            self._job_data["x"] = res2[0]
            self._job_data["y"] = res2[1]
            self._job_data["z"] = res2[2]
            self.loginfo_throttle(2, "depth camera done. results {}, {}".format(res1, res2))
            # self._job_data[]
        
    # set job state
    def set_job_state(self, str):
        self._job_data["job_state"] = str



if __name__ == "__main__":
    rospy.init_node("calibration_controller")
    c = CalibrationController("calibration_controller", 4)
    c.run()
