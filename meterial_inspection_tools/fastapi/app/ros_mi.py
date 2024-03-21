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
        self.topic_data_send_queue = []
        self.topic_state_send_queue = []
        self.topic_info_send_queue = []

        for sub_node_name, content in msg_dict.items():
            try:
                content["topic_data"].Subscriber(self.topic_data_cb, callback_args={"name": sub_node_name})
                logger.loginfo('Sub-ed to "topic_data" have succeded')
            except Exception as e:
                logger.loginfo('Subscription to "topic_data" failed')
                logger.logerr(e)
            
            try:
                content["topic_state"].Subscriber(self.topic_state_cb, callback_args={"name": sub_node_name})
                logger.loginfo('Sub-ed to "topic_state" have succeded')
            except Exception as e:
                logger.loginfo('Subscription to "topic_state" failed')
                logger.logerr(e)

            try:
                content["topic_info"].Subscriber(self.topic_info_cb, callback_args={"name": sub_node_name}) # will be used for the cb_args in the def below
                logger.loginfo('Sub-ed to "topic_info" have succeded')
            except Exception as e:
                logger.loginfo('Subscription to "topic_state" failed')
                logger.logerr(e)

    # -------------------------------------------------------------------------------------------------
    # Subscriber call back functions & msg functions
    # For topic_data
    def topic_data_cb(self, msg, cb_args):
        func = lambda a: a.replace("/","").replace("data","")
        # func(cb_args["name"]) => returns "imu", msg is wtv data there is in the topic
        data_to_send = json.dumps({func(cb_args["name"]): convert_ros_message_to_dictionary(msg)})
        self.topic_data_send_queue.append(data_to_send)

    def has_topic_data_msg(self):
        return len(self.topic_data_send_queue) > 0

    def pop_topic_data_msg(self):
        if len(self.topic_data_send_queue) >0:
            del self.topic_data_send_queue[0]
    
    # For topic_state
    def topic_state_cb(self, msg, cb_args):
        func = lambda a: a.replace("/","").replace("data","")
        # func(cb_args["name"]) => returns "imu", msg is wtv data there is in the topic
        data_to_send = json.dumps({func(cb_args["name"]): convert_ros_message_to_dictionary(msg)})
        self.topic_state_send_queue.append(data_to_send)

    def has_topic_state_msg(self):
        return len(self.topic_state_send_queue) > 0

    def pop_topic_state_msg(self):
        if len(self.topic_state_send_queue) >0:
            del self.topic_state_send_queue[0]

    # For topic_info
    def topic_info_cb(self, msg, cb_args):
        func = lambda a: a.replace("/","").replace("data","")
        # func(cb_args["name"]) => returns "imu", msg is wtv data there is in the topic
        data_to_send = json.dumps({func(cb_args["name"]): convert_ros_message_to_dictionary(msg)})
        self.topic_info_send_queue.append(data_to_send)

    def has_topic_info_msg(self):
        return len(self.topic_info_send_queue) > 0

    def pop_topic_info_msg(self):
        if len(self.topic_info_send_queue) >0:
            del self.topic_info_send_queue[0]

    # -------------------------------------------------------------------------------------------------
    # service calls
    def send_srv(self, srv):
        srv_dict = json.loads(srv)
        logger.loginfo("send service {}".format(srv_dict))
        for sub_node_name, request in srv_dict.items(): # sub node name is "device_name"; request is the "req data"
            command = IMUcontrolRequest() # get the srv req object
            logger.loginfo("request in rosmi {}".format(request))
            command.button = request["button"] # put the parameters into the req obj
            command.parameter1 = request["parameter1"]
            command.parameter2 = request["parameter2"]
            command.parameter3 = request["parameter3"]
            command.parameter4 = request["parameter4"]
            logger.loginfo("request {} to {}".format(command, sub_node_name))
            logger.loginfo("actual request data: {}".format(request))
            logger.loginfo("call service to {}".format(msg_dict[sub_node_name]["srv"])) # should print out the service name
            try:
                msg_dict[sub_node_name]["srv"].service_call(command) # only 1 sub node name now => imu
                logger.loginfo("service call should have succedded")
            except Exception as e:
                logger.logerr(e) # prints out the error if service call fails
        pass