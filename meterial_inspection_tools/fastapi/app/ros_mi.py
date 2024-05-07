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
        self.topic_configs_send_queue = {
            "imu": [],
            "inclinometer": [],
            "cb": [],
            "sonar": [],
        }
        self.topic_data_send_queue = {
            "imu": [],
            "inclinometer": [],
            "cb": [],
            "sonar": [],
        }
        self.topic_info_send_queue = {
            "imu": [],
            "inclinometer": [],
            "cb": [],
            "sonar": [],
        }
        self.topic_state_send_queue = {
            "imu": [],
            "inclinometer": [],
            "cb": [],
            "sonar": [],
        }

        for sub_node_name, content in msg_dict.items():
            try:
                content["topic_configs"].Subscriber(self.topic_configs_cb, callback_args={"name": sub_node_name}) # will be used for the cb_args in the def below
                logger.loginfo(f"Sub-ed to {sub_node_name}'s topic_configs have succeded")
            except Exception as e:
                logger.loginfo(f"Subscription to {sub_node_name}'s topic_configs failed")
                logger.logerr(e)

            try:
                content["topic_data"].Subscriber(self.topic_data_cb, callback_args={"name": sub_node_name})
                logger.loginfo(f"Sub-ed to {sub_node_name}'s topic_data have succeded")
            except Exception as e:
                logger.loginfo(f"Subscription to {sub_node_name}'s topic_data failed")
                logger.logerr(e)
            
            try:
                content["topic_info"].Subscriber(self.topic_info_cb, callback_args={"name": sub_node_name}) # will be used for the cb_args in the def below
                logger.loginfo(f"Sub-ed to {sub_node_name}'s topic_info have succeded")
            except Exception as e:
                logger.loginfo(f"Subscription to {sub_node_name}'s topic_info failed")
                logger.logerr(e)
            
            try:
                content["topic_state"].Subscriber(self.topic_state_cb, callback_args={"name": sub_node_name})
                logger.loginfo(f"Sub-ed to {sub_node_name}'s topic_state have succeded")
            except Exception as e:
                logger.loginfo(f"Subscription to {sub_node_name}'s topic_state failed")
                logger.logerr(e)

    # -------------------------------------------------------------------------------------------------
    # Subscriber call back functions & msg functions
    # CONFIGS TOPIC
    def has_topic_configs_msg(self, component):
        return len(self.topic_configs_send_queue[component]) > 0
    
    def pop_topic_configs_msg(self, component):
        if len(self.topic_configs_send_queue[component]) > 0:
            del self.topic_configs_send_queue[component][0]
    
    def get_topic_configs_msg(self, component):
        return self.topic_configs_send_queue[component][0]

    # DATA TOPIC
    def has_topic_data_msg(self, component):
        return len(self.topic_data_send_queue[component]) > 0
    
    def get_topic_data_msg(self, component):
        return self.topic_data_send_queue[component][0]

    def pop_topic_data_msg(self, component):
        if len(self.topic_data_send_queue[component]) > 0:
            del self.topic_data_send_queue[component][0]
    
    # INFO TOPIC
    def get_topic_info_msg(self, component):
        return self.topic_info_send_queue[component][0]

    def has_topic_info_msg(self, component):
        return len(self.topic_info_send_queue[component]) > 0
    
    def pop_topic_info_msg(self, component):
        if len(self.topic_info_send_queue[component]) > 0:
            del self.topic_info_send_queue[component][0]

    # STATE TOPIC
    def get_topic_state_msg(self, component):
        return self.topic_state_send_queue[component][0]

    def has_topic_state_msg(self, component):
        return len(self.topic_state_send_queue[component]) > 0

    def pop_topic_state_msg(self, component):
        if len(self.topic_state_send_queue[component]) > 0:
            del self.topic_state_send_queue[component][0]
            
    # -------------------------------------------------------------------------------------------------
    # CALLBACKS
    def topic_configs_cb(self, msg, cb_args):
        func = lambda a: a.replace("/","").replace("data","")
        print("call back args in topics", cb_args["name"])
        print("call back args filtered in topics", func(cb_args["name"]))
        # func(cb_args["name"]) => returns "imu", msg is wtv data there is in the topic
        data_to_send = json.dumps({func(cb_args["name"]): convert_ros_message_to_dictionary(msg)})
        self.topic_configs_send_queue[func(cb_args["name"])].append(data_to_send)

    def topic_data_cb(self, msg, cb_args):
        func = lambda a: a.replace("/","").replace("data","")
        print("call back args in topics", cb_args["name"])
        print("call back args filtered in topics", func(cb_args["name"]))
        # func(cb_args["name"]) => returns "imu", msg is wtv data there is in the topic
        data_to_send = json.dumps({func(cb_args["name"]): convert_ros_message_to_dictionary(msg)})
        self.topic_data_send_queue[func(cb_args["name"])].append(data_to_send)
    
    def topic_info_cb(self, msg, cb_args):
        func = lambda a: a.replace("/","").replace("data","")
        print("call back args in topics", cb_args["name"])
        print("call back args filtered in topics", func(cb_args["name"]))
        # func(cb_args["name"]) => returns "imu", msg is wtv data there is in the topic
        data_to_send = json.dumps({func(cb_args["name"]): convert_ros_message_to_dictionary(msg)})
        self.topic_info_send_queue[func(cb_args["name"])].append(data_to_send)
    
    def topic_state_cb(self, msg, cb_args):
        func = lambda a: a.replace("/","").replace("data","")
        print("call back args in topics", cb_args["name"])
        print("call back args filtered in topics", func(cb_args["name"]))
        # func(cb_args["name"]) => returns "imu", msg is wtv data there is in the topic
        data_to_send = json.dumps({func(cb_args["name"]): convert_ros_message_to_dictionary(msg)})
        self.topic_state_send_queue[func(cb_args["name"])].append(data_to_send)

    # -------------------------------------------------------------------------------------------------
    # service calls
    def send_srv(self, srv):
        srv_dict = json.loads(srv)
        logger.loginfo(f"send service {srv_dict}")
        for sub_node_name, request in srv_dict.items(): # sub node name is "device_name"; request is the "req data"
            logger.loginfo(f"test 0")
            logger.loginfo(f"subnode name is {sub_node_name}")
            command = IMUcontrolRequest() # get the srv req object
            logger.loginfo(f"{command}")
            logger.loginfo(f"test 1")
            command.button = request["button"] # put the parameters into the req obj
            logger.loginfo(f"test 2")
            if request.get("parameter"):
                logger.loginfo(f"test 3")
                # command.parameter = request["parameter"]
            logger.loginfo(f"test 4")
            logger.loginfo(f"request data: {request}")
            logger.loginfo(f"command {command} to {sub_node_name}")
            logger.loginfo(f"call service to {msg_dict[sub_node_name]['srv']}") # should print out the service name
            try:
                msg_dict[sub_node_name]["srv"].service_call(command) # only 1 sub node name now => imu
                logger.loginfo("service call should have succedded")
            except Exception as e:
                logger.error("hi error is actually detected in the service call")
                logger.logerr(e) # prints out the error if service call fails
        