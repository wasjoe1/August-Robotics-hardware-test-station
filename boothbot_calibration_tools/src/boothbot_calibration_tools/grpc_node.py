#!/usr/bin/env python3

from boothbot_common.ros_logger_wrap import ROSLogging as Logging
from boothbot_msgs.ros_interfaces import APPS_CALIBRATION_DATA

import grpc

import _thread
from concurrent import futures
# import socket
import time
import json
import rospy
import rospy as logger
# from rospy_message_converter import message_converter

from boothbot_msgs.ros_interfaces import(
    APPS_CALIBRATION_SRV_CMD,
    # APPS_CALIBRATION_STATUS,
    APPS_CALIBRATION_DATA
)

from boothbot_calibration_tools.constants import CalibrationCommand as CS

from boothbot_calibration_tools.settings import APPS_CALIBRATION_SET_PARAM
from boothbot_msgs.srv import (Command, CommandRequest)

from proto_msg import (
    data_pb2,
    data_pb2_grpc
)

PORT = 50052
FASTAPI_GRPC_PORT = 50051
SET_PARAM = "set_param"

class DataService(data_pb2_grpc.data_ServiceServicer):
    def GetMsg(self, request, context):
        logger.loginfo("Got new msg {}".format(request.request_data))
        data = json.loads(request.request_data)
        for k, v in data.items():
            # if k == "step":
            #     logger.loginfo("send new step {}".format(v))
            # # APPS_CALIBRATION_SRV_CMD.service_call(v)
            # elif k == "command":
            if v.startswith(SET_PARAM):
                [set_param,k_tmp,v_tmp] = v.split("=")
                logger.loginfo("set param: {} {}".format(k_tmp, v_tmp))
                self.set_param(k_tmp,v_tmp)
            else:
                logger.loginfo("set command: {}".format(v))
                # logger.loginfo("set command: {} {}".format(k_tmp, v_tmp))
                self.set_command(v)

        return data_pb2.dataResponse()
    
    def set_param(self,k,v):
        try:
            v = float(v)
        except ValueError:
            logger.logerr("cannot convert {} to float".format(v))
            return False
        logger.loginfo("service call {} {}".format(k, v))
        # return APPS_CALIBRATION_SRV_CMD.service_call(command=(k + "=" +v)) 
        call_srv = rospy.ServiceProxy(APPS_CALIBRATION_SET_PARAM, Command)
        cmd = CommandRequest()
        cmd.command = str(k) + "=" + str(v)
        return  call_srv(cmd)

    def set_command(self, cmd):
        """
        Sending the command to server

        Parameters
        ----------
        cmd : boothbot_marking.constants.MarkingControllerCommand
            The cmd

        Returns
        -------
        bool
            If the cmd was received or not
        """
        # legal check
        try:
            if not isinstance(cmd, CS):
                cmd = CS[cmd.upper()]
            return APPS_CALIBRATION_SRV_CMD.service_call(command=cmd.name)
        except KeyError:
            self.logwarn("No such command: {}".format(cmd))
            return False


class GRPCtoROS(Logging):
    def __init__(self):
        super(GRPCtoROS, self).__init__('GRPCtoROS')
        APPS_CALIBRATION_DATA.Subscriber(self.data_cb)
        self.data_to_send = None
        self.long_camera_data = None
        self.short_camera_data = None

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

    def data_cb(self, msg):
        self.data_to_send = msg.data

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
            self.loginfo(e)


if __name__ == "__main__":
    rospy.init_node("grpc_ros")
    g = GRPCtoROS()
    g.run()
