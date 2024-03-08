#!/usr/bin/env python3

import rospy
import rospy as logger
from meterial_inspection_tools.ros_interface import (
    # MODULES_MATERIAL_INSPECTION_DATA,
    # MODULES_MATERIAL_INSPECTION_SRV_CMD
    LDLIDAR_DATA,
    LDLIDAR_SRV_CMD,
    msg_dict
)

from rospy_message_converter.message_converter import convert_ros_message_to_dictionary


import grpc

import _thread
from concurrent import futures
# import socket
import time
import json
import rospy
import rospy as logger
# from rospy_message_converter import message_converter

# from boothbot_calibration_tools.constants import CalibrationCommand as CS

from boothbot_msgs.srv import (Command, CommandRequest)

from proto_msg import (
    data_pb2,
    data_pb2_grpc
)

PORT = 50052
FASTAPI_GRPC_PORT = 50051
COMMAND = "command"

class DataService(data_pb2_grpc.data_ServiceServicer):
    def GetMsg(self, request, context):
        logger.loginfo("Got new msg {}".format(request.request_data))
        data = json.loads(request.request_data)
        for k, v in data.items():
            if k == COMMAND:
                self.command(k,v)
            else:
                logger.loginfo("set command: {}".format(v))
                self.set_command(v)

        return data_pb2.dataResponse()
    
    def command(self,k,v):
        cmd_dict = json.loads(v)
        for sub_node, sub_node_cmd in cmd_dict.items():
            cmd = CommandRequest()
            cmd.command = sub_node_cmd
            # return True
            logger.loginfo("service call {}".format(cmd))
            return  msg_dict[sub_node].service_call(cmd)


    def set_command(self, v):
        logger.loginfo("set command: {}".format(v))



class GRPCROSCOMMUNITE():
    def __init__(self):

        # MODULES_MATERIAL_INSPECTION_DATA.Subscriber(self.data_cb)
        self.data_to_send = None
        self.long_camera_data = None
        self.short_camera_data = None

        ### ROS
        for sub_node_name, content in msg_dict.items():
            content["topic"].Subscriber(self.data_cb, callback_args={"name":LDLIDAR_DATA.name})
        # LDLIDAR_DATA.Subscriber()

    def serve(self):
        _ONE_DAY_IN_SECONDS = 60 * 60 * 24
        server = grpc.server(futures.ThreadPoolExecutor(max_workers=10))
        # step_pb2_grpc.add_Step_serviceServicer_to_server(StepService(), server)
        data_pb2_grpc.add_data_ServiceServicer_to_server(DataService(), server)
        # command_pb2_grpc.add_Command_serviceServicer_to_server(
        #     CommandService(), server)
        server.add_insecure_port('0.0.0.0:50052')
        server.start()
        try:
            while True:
                # self.send_data()
                time.sleep(_ONE_DAY_IN_SECONDS)
        except KeyboardInterrupt:
            server.stop(0)

    def run(self):
        try:
            _thread.start_new_thread(self.serve, ())
            # self.serve()
        except Exception as e:
            logger.info("Error: cannot init thread. {}".format(e))

        rate = rospy.Rate(100)
        while not rospy.is_shutdown():
            self.send_data()
            rate.sleep()

    def data_cb(self, msg, cb_args):
        # logger.loginfo("get msg and cb_args {}".format(cb_args))
        func = lambda a: a.replace("/","").replace("data","")
        self.data_to_send = json.dumps({func(cb_args["name"]): convert_ros_message_to_dictionary(msg)})

    def send_data(self):
        if self.data_to_send is not None:
            # data = {}
            # data["data"] = self.data_to_send
            # logger.loginfo(type(self.data_to_send))
            # logger.loginfo(self.data_to_send[0:20])
            grpc_data = json.dumps(self.data_to_send)
            self.send_to_fastapi(grpc_data)
            self.data_to_send = None

    def send_to_fastapi(self, grpc_data):
        try:
            with grpc.insecure_channel('0.0.0.0:'+str(FASTAPI_GRPC_PORT)) as channel:
                stub = data_pb2_grpc.data_ServiceStub(channel)
                response = stub.GetMsg(
                    data_pb2.dataRequest(request_data=grpc_data))
        except Exception as e:
            logger.loginfo(e)


if __name__ == "__main__":
    rospy.init_node("grpc_ros")
    g = GRPCROSCOMMUNITE()
    g.run()
