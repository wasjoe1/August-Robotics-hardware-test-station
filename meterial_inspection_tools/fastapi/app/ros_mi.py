import json

import rospy
import rospy as logger
from meterial_inspection_tools.ros_interface import (
    msg_dict
)
from rospy_message_converter.message_converter import convert_ros_message_to_dictionary
from boothbot_msgs.srv import (Command, CommandRequest)
from meterial_inspection_tools.srv import (IMUcontrolRequest,IMUcontrol)


class MeterialInspection():
    def __init__(self) -> None:
        logger.loginfo("init meterial inspection...")
        self.send_queue = []
        for sub_node_name, content in msg_dict.items():
            try:
                content["topic_data"].Subscriber(self.data_cb, callback_args={"name": sub_node_name})
                logger.loginfo("subscription should have succeded")
            except Exception as e:
                logger.loginfo("subscription failed")
                logger.logerr(e)

    def data_cb(self, msg, cb_args):
        func = lambda a: a.replace("/","").replace("data","")
        self.data_to_send = json.dumps({func(cb_args["name"]): convert_ros_message_to_dictionary(msg)})
        self.send_queue.append(self.data_to_send)

    def send_srv(self, srv):
        srv_dict = json.loads(srv)
        logger.loginfo("send service {}".format(srv_dict))
        for sub_node_name, request in srv_dict.items(): # sub node name is "device_name"; request is the "req data"
            command = IMUcontrolRequest() # get the srv req object
            command.button = "button" # put the parameters into the req obj
            command.parameter1 = 0
            command.parameter2 = 0
            command.parameter3 = 0
            command.parameter4 = 0
            logger.loginfo("request {} to {}".format(command, sub_node_name))
            logger.loginfo("actual request data: {}".format(request))
            logger.loginfo("call service to {}".format(msg_dict[sub_node_name]["srv"])) # should print out the service name
            try:
                msg_dict[sub_node_name]["srv"].service_call(command) # only 2 sub nodes name now => ldlidar & imu
                logger.loginfo("service call should have succedded")
            except Exception as e:
                logger.logerr(e) # prints out the error if service call fails
        pass

    def has_msg(self):
        return len(self.send_queue) > 0

    def pop_msg(self):
        if len(self.send_queue) >0:
            del self.send_queue[0]