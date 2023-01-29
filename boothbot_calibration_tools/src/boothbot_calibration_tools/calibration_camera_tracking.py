#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from boothbot_perception.track.tracking_camera import TrackingCamera
from boothbot_calibration_tools.calibration_camera_tracking_base import CaliTrackingCameraBase
from boothbot_calibration_tools.settings import (
    LONG,
    SHORT,
    COLOR
)

class CaliTrackingCamera(CaliTrackingCameraBase):
    def __init__(self, camera_type, laser_dist=50):
        self.ontology = TrackingCamera(camera_type, laser_dist)

    def cap(self):
        return self.ontology.cap()

    def is_camera_idle(self):
        return self.ontology.is_camera_idle()

    def find_beacon(self, frame, dis, color, color_range):
        return self.ontology.find_beacon(frame, dis, color, color_range)

    def get_sharpness(self, frame, beacon_res, dis):
        return self.ontology.get_sharpness(frame, beacon_res, dis)

    def get_color_value(self, frame, beacon_res, color):
        return self.ontology.get_color_value(frame, beacon_res, color)

    def get_beacon_angle(self, beacon_res, dis, compensation, long_shrot_angle):
        return self.ontology.get_beacon_angle(beacon_res, dis, compensation, long_shrot_angle)

    def find_laser_dot(self, frame):
        return self.ontology.find_laser_dot(frame)

    def get_laser_result(self, frame, laser_dot):
        return self.ontology.get_laser_result(frame, laser_dot)

    def draw_beacon(self, frame, beacon_res):
        return self.ontology.draw_beacon(frame, beacon_res)

    def shutdown(self):
        return self.ontology.shutdown()

