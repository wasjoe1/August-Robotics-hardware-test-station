import json

import rospy
import rospy as logger
from meterial_inspection_tools.ros_interface import (
    LDLIDAR_DATA,
    LDLIDAR_SRV_CMD,
    msg_dict
)
from rospy_message_converter.message_converter import convert_ros_message_to_dictionary
from boothbot_msgs.srv import (Command, CommandRequest)


class MeterialInspection():
    def __init__(self) -> None:
        logger.loginfo("init meterial inspection...")
        self.send_queue = []
        for sub_node_name, content in msg_dict.items():
            content["topic"].Subscriber(self.data_cb, callback_args={"name": sub_node_name})

    def data_cb(self, msg, cb_args):
        func = lambda a: a.replace("/","").replace("data","")
        self.data_to_send = json.dumps({func(cb_args["name"]): convert_ros_message_to_dictionary(msg)})
        self.send_queue.append(self.data_to_send)

    def send_srv(self, srv):
        srv_dict = json.loads(srv)
        # logger.loginfo("send service {}".format(srv_dict))
        for sub_node_name, request in srv_dict.items(): # sub node name is "_step_"; request is the "cmd_string"
            command = CommandRequest()
            command.command = request
            logger.loginfo("request {} to {}".format(command, sub_node_name))        
            msg_dict[sub_node_name]["srv"].service_call(command)
        pass

    def has_msg(self):
        return len(self.send_queue) > 0

    def pop_msg(self):
        if len(self.send_queue) >0:
            del self.send_queue[0]