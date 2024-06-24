from enum import Enum, auto
from serial import Serial
import os
import rospy
logger = rospy

class BaseCommands(Enum):
    NONE = auto()
    RESET = auto()
    CONNECT = auto()
    SET_DEFAULT = auto()

class BaseCheckerStates(Enum):
    INIT = auto()
    IDLE = auto()
    SCANNING = auto()
    CONNECTED = auto()
    ERROR = auto()



class BaseOperations_modbus: #TODO: check again
    """
    Base for interface class
    """
    def scan(configs):
        modbus_client: ModbusClient = None
        unit_id_constant = None
        return modbus_client,unit_id_constant
    
    def parse_reading(modbus_client,unit_id):
        return get_data(modbus_client,unit_id)
    
    def set_default_setting(modbus_client,unit_id,command_params):
        succeeded = False
        return succeeded,modbus_client, unit_id
    


class BaseOperations_serial: #TODO: check again
    TYPE = None
    
    def scan(port):
        serial_port: Serial = None
        return serial_port
    
    def set_default_settings(serial_port):
        succeeded = False
        return succeeded
    
    def check_reading(msg_data):
        return True
    


class BaseOperations_connection_thirdparty:

    def connect(port):
        flag:bool = None
        flag:bool = None
        flag = os.path.exists(port)
        if flag == True:
            logger.loginfo("Connected")
        elif flag == False: 
            logger.loginfo("problem connecting")
        return flag
    
    def check_connection(port):
        connection_flag = os.path.exists(port)
        return connection_flag        



class BaseChecker: 
    NODE_RATE = 5.0

    def __init__(self, command, state_cls,info_cls,info_chinese_cls, srv_cmd_cls,additional_publishers = None):
        self.command = command
        self.pub_info = info_cls.Publisher()
        self.pub_info_chinese = info_chinese_cls.Publisher()
        self.pub_state = state_cls.Publisher()
        srv_cmd_cls.Services(self.srv_method) #TODO: for all srv, change to srv_method common naming



    def log_with_frontend(self, log,log_chinese):
        logger.loginfo(log)
        logger.loginfo(log_chinese)
        self.pub_info.publish(log)
        self.pub_info_chinese.publish(log_chinese)

    @property
    def state(self):
        return self._state
    @state.setter
    def state(self, value: BaseCheckerStates):
        if self._state != value:
            self._state = value
            logger.logwarn(f"State changed to: {self._state.name}")
            #self.pub_state = state_cls.Publisher()
            self.pub_state.publish(self.state.name)

    def start(self):
        l = rospy.Rate(self.NODE_RATE)
        while not rospy.is_shutdown():
            state_method = self.__STATES_METHODS.get((self.command,self.state))
            if state_method:
                state_method()
            self.command = "NONE"
            l.sleep()



        