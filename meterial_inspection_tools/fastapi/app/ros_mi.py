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
        self.send_queue = {} # {component: {topic1: [], topic2: []...}, ... }

        for component, topics in msg_dict.items():
            logger.loginfo(f"All Topics in {component} component: {topics}")
            for topic in topics:
                # for all the topics in respective components
                # subscribe to respective topics
                # call the respective callback functions
                # skip srv calls
                if topic == "srv":
                    continue
                try:
                    # create msg storage for component & topic
                    if not self.send_queue.get(component): # if send q has no such topic
                        self.send_queue[component] = {}
                    if not self.send_queue[component].get(topic): # if send q topic has no such component
                        self.send_queue[component][topic] = []
                    
                    topics[topic].Subscriber(self.topic_cb, callback_args={"component": component, "topic": topic}) # will be used for the cb_args in the def below
                    logger.loginfo("after node subscribed")
                    logger.loginfo(f"Sub-ed to {component}'s {topic} have succeded")
                except TypeError as e:
                    logger.loginfo(f"Subscription to {component}'s {topic} failed")
                    logger.logerr(e)

    # -------------------------------------------------------------------------------------------------
    # Subscriber call back functions & msg functions
    
    # want to get, pop, and check has msg
    # depends on the component & topic
    # components => imu, sonar, inclinometer, cb
    # topics are state, info, data, configs
    def get_topic_msg(self, component, topic):
        return self.send_queue[component][topic][0]

    def has_topic_msg(self, component, topic):
        return len(self.send_queue[component][topic]) > 0

    def pop_topic_msg(self, component, topic):
        if len(self.send_queue[component][topic]) > 0:
            del self.send_queue[component][topic][0]
    
    def get_topic_to_component_dict(self):
        logger.loginfo("printing out send q ...")
        logger.loginfo(self.send_queue)
        
        res = []
        for component in self.send_queue:
            for topic in self.send_queue[component]:
                res.append((component, topic))
        
        logger.loginfo("printing out component to topic ...")
        logger.loginfo(res)
        return res

    # -------------------------------------------------------------------------------------------------
    # CALLBACKS
    # TODO Double check that subscribers can use the same cb func
    def topic_cb(self, msg, cb_args):
        print("call back component arg: ", cb_args["component"]) # should return 'imu', 'inclinometer', ...
        print("call back topic arg: ", cb_args["topic"]) # should return 'topic_info', 'topic_data', ... etc.
        data_to_send = json.dumps({cb_args["component"]: convert_ros_message_to_dictionary(msg)})
        if not self.send_queue.get(cb_args["component"]): # if send q has no such component
            logger.logerr(f"{cb_args['component']} component does not exist")
        if not self.send_queue[cb_args["component"]].get(cb_args["topic"]): # if send q component has no such topic
            logger.logerr(f"{cb_args['component']} does not have {cb_args['topic']}")
        self.send_queue[cb_args["component"]][cb_args["topic"]].append(data_to_send)

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
        