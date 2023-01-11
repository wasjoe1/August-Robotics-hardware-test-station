#!/usr/bin/env python3

import rospy
import ros_numpy
import numpy as np

import grpc
from PIL import Image
from io import BytesIO
import json
import socket
import base64
# from device_states import DeviceStates
# # from device import Device
from boothbot_perception import TargetTracker
from boothbot_common.error_code import ErrCode
from boothbot_msgs.ros_interfaces import (
    DRIVERS_TRACKER_LONG_IMAGE,
    DRIVERS_TRACKER_SHORT_IMAGE,
    DRIVERS_TRACKER_STATUS,
    DRIVERS_SERVOS_PDO,
    DRIVERS_SERVOS_STATUS,
    DRIVERS_CHASSIS_IO,
)

from proto_msg import (
    data_pb2,
    data_pb2_grpc
)


def img2text(msg):
    im = ros_numpy.numpify(msg)
    # From BGR to RGB
    im = im[:, :, ::-1]
    im = Image.fromarray(im)
    buf = BytesIO()
    im.save(buf, format="JPEG")
    im_binary = base64.b64encode(buf.getvalue())
    im_text = im_binary.decode()
    return im_text


def send_data(data):
    grpc_data = json.dumps(data)
    h_name = socket.gethostname()
    IP_addres = socket.gethostbyname(h_name)
    with grpc.insecure_channel(IP_addres + ':50051') as channel:
        # stub = command_pb2_grpc.Command_serviceStub(channel)
        stub = data_pb2_grpc.data_ServiceStub(channel=channel)
        response = stub.GetMsg(data_pb2.dataRequest(request_data=grpc_data))
    print("Client received: " + response.response_data)


def _track_long_image_callback(msg):
    # long_image = ros_numpy.numpify(msg)
    # str_encode = long_image.tostring()
    # img_64 = base64.b64encode(str_encode)
    img_data = img2text(msg)
    grpc_data = {}
    grpc_data["long"] = img_data
    send_data(grpc_data)

    # channel = grpc.insecure_channel('127.0.0.1:5001')
    # stub = img_pb2_grpc.dataStub(channel)

    # response = stub.serving(img_pb2.data_request(img_requeset=img_64))


def _track_short_image_callback(msg):
    img_data = img2text(msg)
    grpc_data = {}
    grpc_data["short"] = img_data
    send_data(grpc_data)


if __name__ == "__main__":
    rospy.init_node("test_tracker_node")
    tracker = TargetTracker()
    tracker.connect()
    tracker.capture(True)

    # rospy.Subscriber(
    #     DRIVERS_TRACKER_STATUS.name,
    #     DRIVERS_TRACKER_STATUS.type,
    #     _track_status_callback)

    rospy.Subscriber(
        DRIVERS_TRACKER_LONG_IMAGE.name,
        DRIVERS_TRACKER_LONG_IMAGE.type,
        _track_long_image_callback)

    rospy.Subscriber(
        DRIVERS_TRACKER_SHORT_IMAGE.name,
        DRIVERS_TRACKER_SHORT_IMAGE.type,
        _track_short_image_callback)

    rate = rospy.Rate(1)

    while not rospy.is_shutdown():
        tracker.trigger("BOG", cali=True)
        rate.sleep()


# class TrackingCameras(Image):
#     def __init__(self, *args, **kwargs):
#         super().__init__(None, *args, **kwargs)

#         self.long_image = None
#         self.short_image = None
#         self.tracker = TargetTracker()
#         self.tracker.connect()
#         self.tracker.capture(True)

#         rospy.Subscriber(
#             DRIVERS_TRACKER_STATUS.name,
#             DRIVERS_TRACKER_STATUS.type,
#             self._track_status_callback)

#         rospy.Subscriber(
#             DRIVERS_TRACKER_LONG_IMAGE.name,
#             DRIVERS_TRACKER_LONG_IMAGE.type,
#             self._track_long_image_callback)

#         rospy.Subscriber(
#             DRIVERS_TRACKER_SHORT_IMAGE.name,
#             DRIVERS_TRACKER_SHORT_IMAGE.type,
#             self._track_short_image_callback)

#     def _track_status_callback(self, msg):
#         if msg.state == "RUNNING":
#             self.state = DeviceStates.ON
#         else:
#             self.state = DeviceStates.ERROR

#     def _track_long_image_callback(self, msg):
#         self.long_image = ros_numpy.numpify(msg)

#     def _track_short_image_callback(self, msg):
#         self.short_image = ros_numpy.numpify(msg)

#     def concat_images(self):
#         if self.long_image is None or self.short_image is None:
#             return
#         image = np.concatenate((self.long_image, self.short_image), axis=0)
#         self.update(image)

#     def track(self):
#         if self.state == DeviceStates.ON:
#             self.tracker.send_goal()

#     def on_mount(self):
#         self.set_interval(10, self.track)
#         self.set_interval(10, self.concat_images)
