#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from boothbot_calibration_tools.settings import (
    LONG,
    SHORT,
    COLOR
)

class CaliTrackingCameraBase():
    def __init__(self, camera_type, laser_dist=50):
        pass

    def cap(self):
        return None

    def is_camera_idle(self):
        return True

    def set_expo(self):
        return True

    def find_beacon(self, frame, dis, color, color_range):
        return None

    def get_sharpness(self, frame, beacon_res, dis):
        return None

    def get_color_value(self, frame, beacon_res, color):
        return None

    def get_beacon_angle(self, beacon_res, dis, compensation, long_shrot_angle):
        return None

    def find_laser_dot(self, frame):
        return None

    def get_laser_result(self, frame, laser_dot):
        return None

    def draw_beacon(self, frame, beacon_res):
        return None

    def shutdown(self):
        return True
