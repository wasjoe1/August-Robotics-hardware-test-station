import json

import rospy
import rospy as logger
from meterial_inspection_tools.ros_interface import (
    msg_dict
)
from rospy_message_converter.message_converter import convert_ros_message_to_dictionary
from boothbot_msgs.srv import (Command, CommandRequest)
from meterial_inspection_tools.srv import (IMUControl, IMUControlRequest)
from meterial_inspection_tools.srv import (SonarControl, SonarControlRequest)
from meterial_inspection_tools.srv import (InclinometerControl, InclinometerControlRequest)
from meterial_inspection_tools.srv import (CBControl, CBControlRequest)

ServiceRequestTypes = {
    "imu": IMUControlRequest,
    "sonar": SonarControlRequest,
    "inclinometer": InclinometerControlRequest,
    "cb": CBControlRequest,
}

class MeterialInspection():
    def __init__(self) -> None:
        logger.loginfo("init meterial inspection...")
        self.send_queue = {
            "configs": {
                "imu": [],
                "inclinometer": [],
                "cb": [],
                "sonar": [],
            },
            "data": {
                "imu": [],
                "inclinometer": [],
                "cb": [],
                "sonar": [],
            },
            "info": {
                "imu": [],
                "inclinometer": [],
                "cb": [],
                "sonar": [],
            },
            "state": {
                "imu": [],
                "inclinometer": [],
                "cb": [],
                "sonar": [],
            },
            "reading_checker": {
                "imu": [],
                "inclinometer": [],
                "cb": [],
                "sonar": [],
            },
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
    
    # want to get, pop, and check has msg
    # depends on the component & topic
    # components => imu, sonar, inclinometer, cb
    # topics are state, info, data, configs
    def get_topic_msg(self, topic, component):
        return self.send_queue[topic][component][0]

    def has_topic_msg(self, topic, component):
        return len(self.send_queue[topic][component]) > 0

    def pop_topic_msg(self, topic, component):
        if len(self.send_queue[topic][component]) > 0:
            del self.send_queue[topic][component][0]

    # -------------------------------------------------------------------------------------------------
    # CALLBACKS
    def topic_configs_cb(self, msg, cb_args):
        func = lambda a: a.replace("/","").replace("data","")
        print("call back args in topics", cb_args["name"])
        print("call back args filtered in topics", func(cb_args["name"]))
        # func(cb_args["name"]) => returns "imu", msg is wtv data there is in the topic
        data_to_send = json.dumps({func(cb_args["name"]): convert_ros_message_to_dictionary(msg)})
        self.send_queue["configs"][func(cb_args["name"])].append(data_to_send)

    def topic_data_cb(self, msg, cb_args):
        func = lambda a: a.replace("/","").replace("data","")
        print("call back args in topics", cb_args["name"])
        print("call back args filtered in topics", func(cb_args["name"]))
        # func(cb_args["name"]) => returns "imu", msg is wtv data there is in the topic
        data_to_send = json.dumps({func(cb_args["name"]): convert_ros_message_to_dictionary(msg)})
        self.send_queue["data"][func(cb_args["name"])].append(data_to_send)
    
    def topic_info_cb(self, msg, cb_args):
        func = lambda a: a.replace("/","").replace("data","")
        print("call back args in topics", cb_args["name"])
        print("call back args filtered in topics", func(cb_args["name"]))
        # func(cb_args["name"]) => returns "imu", msg is wtv data there is in the topic
        data_to_send = json.dumps({func(cb_args["name"]): convert_ros_message_to_dictionary(msg)})
        self.send_queue["info"][func(cb_args["name"])].append(data_to_send)
    
    def topic_state_cb(self, msg, cb_args):
        func = lambda a: a.replace("/","").replace("data","")
        print("call back args in topics", cb_args["name"])
        print("call back args filtered in topics", func(cb_args["name"]))
        # func(cb_args["name"]) => returns "imu", msg is wtv data there is in the topic
        data_to_send = json.dumps({func(cb_args["name"]): convert_ros_message_to_dictionary(msg)})
        self.send_queue["state"][func(cb_args["name"])].append(data_to_send)

    # -------------------------------------------------------------------------------------------------
    # service calls
    def send_srv(self, srv):
        srv_dict = json.loads(srv)
        logger.loginfo(f"send service {srv_dict}")
        for device_name, request in srv_dict.items(): # sub node name is "device_name"; request is the "req data"
            logger.loginfo(f"component name is {device_name}")
            logger.loginfo(f"request data: {request}")
            
            try:
                command = ServiceRequestTypes[device_name]() # get the srv req object
                command.button = request["button"] # put the parameters into the req obj
                if request.get("parameter"):
                    command.parameter = request["parameter"]
                logger.loginfo(f"command data: {command}")
                
                logger.loginfo(f"Service call to {msg_dict[device_name]['srv']}...")                
                msg_dict[device_name]["srv"].service_call(command)
                logger.loginfo("Service call succedded")
            except Exception as e:
                logger.logerr("Service call failed")
                logger.logerr(e)
                raise rospy.ServiceException("Service call failed")
        