#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from boothbot_perception.track.tracking_camera import TrackingCamera

class CaliTrackingCamera():
    def __init__(self, camera_type, dis=50, enable=True):
        self.ontology = TrackingCamera(camera_type, dis)
        self.enable = enable

    def cap(self):
        if self.enable:
            return self.ontology.cap()
        else:
            return None

    def is_camera_idle(self):
        if self.enable:
            return self.ontology.is_camera_idle()
        else:
            return True

    def find_beacon(self, frame, dis, color, color_range):
        return self.ontology.find_beacon(frame, dis, color, color_range)

    def get_sharpness(self, frame, beacon_res, dis):
        return self.ontology.get_sharpness(frame, beacon_res, dis)

    def get_color_value(self, frame, beacon_res, color):
        return self.ontology.get_color_value(frame, beacon_res, color)

    def find_laser_dot(self, frame):
        return self.ontology.find_laser_dot(frame)

    def get_laser_result(self, frame, laser_dot):
        return self.ontology.get_laser_result(frame, laser_dot)
