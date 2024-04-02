#!/usr/bin/python3
# -*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import Imu
logger = rospy
from enum import Enum, auto
from serial import Serial
import json
import numpy as np
import math
from meterial_inspection_tools.ros_interface import (
    IMU_DATA,
    IMU_INFO,
    IMU_STATE,
    IMU_SRV_CMD,
)

from imu_driver_constants import (
    WIT_IMU_CONSTANTS,
    RION_IMU_CONSTANTS,
)   


class IMUCommands(Enum):
    NONE = auto()
    RESET = auto()
    SCAN = auto()
    CONNECT = auto()
    AUTO_DETECT = auto()
    DISCONNECT = auto()
    SET_DEFAULT = auto()
    SAVE = auto()

class IMUCheckerStates(Enum):
    INIT = auto()
    IDLE = auto()
    SCANNING = auto()
    CONNECTING = auto()
    CONNECTED = auto()
    ERROR = auto()


class IMUOperations:
    """
    Interfaces class
    """
    IMU_TYPE = None
    SCAN_BAUDRATES = (9600, 115200)
    DEFAULT_BAUDRATE = 115200

    @staticmethod
    def connect(port):
        serial_port: Serial = None
        return serial_port

    @staticmethod
    def scan(port):
        serial_port: Serial = None
        return serial_port
     

    @staticmethod
    def parse_reading(serial_port):
        """
        Return the latest frame of reading
        """
        if IMUChecker.imu_model == IMURion.IMU_TYPE:
            return Imu()
        elif IMUChecker.imu_model == IMUWIT.IMU_TYPE:
            pass 
            
    
    @staticmethod
    def set_default_settings(serial_port):
        succeeded = False
        return succeeded
    
    @staticmethod
    def save_parameters(port):
        return True
    
    @staticmethod
    def close(port):
        return True
        


class IMURion(IMUOperations):
    IMU_TYPE = "RION"
    G = 9.80665
    OFFSET  = 3
    ROLL = 4
    PITCH = 7
    YAW = 10
    ACC_X = 13
    ACC_Y = 16
    ACC_Z = 19
    GYRO_X = 22
    GYRO_Y = 25
    GYRO_Z = 28


    @staticmethod
    def parse_reading(serial_port):

        def get_quaternion_from_euler(roll, pitch, yaw):
                """
                Convert an Euler angle to a quaternion.

                Inputf self.serial_port:tation around z-axis) angle in radians.

                Output
                :return qx, qy, qz, qw: The orientation in quaternion [x,y,z,w] format
                """
                qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
                qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
                qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
                qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
                return [qx, qy, qz, qw]
            
        def reflash():
            s_data = None
            reset_input_buffer = Serial.reset_input_buffer()
            reset_output_buffer = Serial.reset_output_buffer()
            s_data = Serial.read(32)
            return s_data

        def byte_str(val):
        # print("got val {}".format(val))
            if len(str(hex(val)[2:])) == 1:
                # print("length {}".format(len(str(hex(val)[2:]))))
                return "0" + str(hex(val)[2:])
            return str(hex(val)[2:])   
        
        def factor(s):
            f = 1
            if s[0] == "1":
                f = f * -1
            return f

        def orien():
            roll_ori = split_data(IMURion.ROLL)
            print(roll_ori)
            pitch_ori = split_data(IMURion.PITCH)
            yaw_ori = split_data(IMURion.YAW)
            roll = int(roll_ori[1:6])/100.0 * factor(roll_ori)
            pitch = int(pitch_ori[1:6])/100.0 * factor(pitch_ori)
            yaw = int(yaw_ori[1:6])/100.0 * factor(yaw_ori)
            roll = math.radians(roll)
            pitch = math.radians(pitch)
            yaw = math.radians(yaw)
            return (roll, pitch, yaw)

        def acc():
            acc_x_ori = split_data(IMURion.ACC_X)
            acc_y_ori = split_data(IMURion.ACC_Y)
            acc_z_ori = split_data(IMURion.ACC_Z)
            acc_x = int(acc_x_ori[1:6])/1000.0 * factor(acc_x_ori) * IMURion.G
            acc_y = int(acc_y_ori[1:6])/1000.0 * factor(acc_y_ori) * IMURion.G
            acc_z = int(acc_z_ori[1:6])/1000.0 * factor(acc_z_ori) * IMURion.G
            return (acc_x, acc_y, acc_z)

        def gyro():
            gyro_x_ori = split_data(IMURion.GYRO_X)
            gyro_y_ori = split_data(IMURion.GYRO_Y)
            gyro_z_ori = split_data(IMURion.GYRO_Z)    
        
        def split_data(s_data, start, offset=3):
            s = ""
            for i in range(0, offset):
                s += byte_str(s_data[start+i])
            # return byte_str(res[4]) + byte_str(res[5]) + byte_str(res[6])
            return s


        reflash()
        (roll,pitch,yaw) = orien()
        (acc_x, acc_y, acc_z) = acc()
        (gyro_x, gyro_y, gyro_z) = gyro()

        (x, y, z, w) = get_quaternion_from_euler(roll, pitch, yaw)

        Imu.orientation.x = x
        Imu.orientation.y = y
        Imu.orientation.z = z
        Imu.orientation.w = w
        Imu.angular_velocity.x = gyro_x
        Imu.angular_velocity.y = gyro_y
        Imu.angular_velocity.z = gyro_z
        Imu.linear_acceleration.x = acc_x
        Imu.linear_acceleration.y = acc_y
        Imu.linear_acceleration.z = acc_z
        return Imu()
    
    @staticmethod
    def connect(port):
        serial_port: Serial = None
        with Serial(port = port, baudrate=IMUOperations.DEFAULT_BAUDRATE, timeout=1.0) as p: 
            timestart = 1.0
            rion_imu_baudrate= p.read_until(RION_IMU_CONSTANTS.RION_IMU_CMD_SET_BAUDRATE.value)
            if rospy.Time.now().to_sec - timestart < 1.0:
                serial_port =p
        return serial_port
            
            
    
    @staticmethod
    def scan (port):
        serial_port: Serial = None
        for baud in IMUOperations.SCAN_BAUDRATES:
            with Serial(port = port, baudrate=baud, timeout=1.0) as p: 
                timestart = 1.0
                rion_imu_baudrate= p.read_until(RION_IMU_CONSTANTS.RION_IMU_CMD_SET_BAUDRATE.value)
                if rospy.Time.now().to_sec - timestart < 1.0:
                    serial_port = p
        return serial_port
        
        
    @staticmethod
    def set_default_settings(serial_port):
        succeeded = False
        try:
            with Serial(port = serial_port, baudrate=IMUOperations.DEFAULT_BAUDRATE, timeout=1.0) as  p: 
                p.write(RION_IMU_CONSTANTS.RION_IMU_CMD_SET_BAUDRATE.value)
                succeeded = True
        except Exception as e: 
            pass
        return succeeded

    @staticmethod
    def save_parameters(port):
        return True

    @staticmethod
    def close(port):
        return True

class IMUWIT(IMUOperations):
    IMU_TYPE = "WIT"

    @staticmethod
    def parse_reading(serial_port):
        def c8h2sint(value):
                return -(value & 0x80) | (value & 0x7F)
        def imu_feedback_parse(frame):
            [
                # convert linear acc
                _, _, _AxL, _AxH, _AyL, _AyH, _AzL, _AzH, _TL, _TH, _,
                # convert angular velocity
                _, _, _wxL, _wxH, _wyL, _wyH, _wzL, _wzH, _TL, _TH, _,
                # convert Roll Pitch Yaw
                _, _, _RollL, _RollH, _PitchL, _PitchH, _YawL, _YawH, _TL, _TH, _,
                # _,_,_,_,_,_,_,_,_,_,__,
                # convert quaternion
                _, _, _Q0L, _Q0H, _Q1L, _Q1H, _Q2L, _Q2H, _Q3L, _Q3H, _,
            ] = frame

            # m/s^2
            Ax = float((c8h2sint(_AxH) << 8) | _AxL) / 32768.0 * 16 * 9.8
            Ay = float((c8h2sint(_AyH) << 8) | _AyL) / 32768.0 * 16 * 9.8
            Az = float((c8h2sint(_AzH) << 8) | _AzL) / 32768.0 * 16 * 9.8

            # deg/s to rad/s
            wx = float((c8h2sint(_wxH) << 8) | _wxL) / 32768.0 * 2000 * math.pi / 180
            wy = float((c8h2sint(_wyH) << 8) | _wyL) / 32768.0 * 2000 * math.pi / 180
            wz = float((c8h2sint(_wzH) << 8) | _wzL) / 32768.0 * 2000 * math.pi / 180

            # deg/s to rad/s
            roll = float((c8h2sint(_RollH) << 8) | _RollL) / 32768.0 * 180 * math.pi / 180
            pitch = float((c8h2sint(_PitchH) << 8) | _PitchL) / 32768.0 * 180 * math.pi / 180
            yaw = float((c8h2sint(_YawH) << 8) | _YawL) / 32768.0 * 180 * math.pi / 180

            # quaternion from IMU
            q0 = float((c8h2sint(_Q0H) << 8) | _Q0L) / 32768
            q1 = float((c8h2sint(_Q1H) << 8) | _Q1L) / 32768
            q2 = float((c8h2sint(_Q2H) << 8) | _Q2L) / 32768
            q3 = float((c8h2sint(_Q3H) << 8) | _Q3L) / 32768

            return {
                "acceleration": [
                    Ax,
                    Ay,
                    Az,
                ],
                "angle_speed": [
                    wx,
                    wy,
                    wz,
                ],
                "radians": [
                    roll,
                    pitch,
                    yaw,
                ],
                "quaternion": [q0, q1, q2, q3],
            }
           
        with Serial(port=serial_port, baudrate= IMUOperations.DEFAULT_BAUDRATE, timeout=1.0) as p:
                for t in range(3):
                    _imu_raw_data = p.read_until(b"\x55\x51")
                    if len(_imu_raw_data) != 44:
                        frame_int = [int(x) for x in _imu_raw_data]
                        frame_int = frame_int[-2:] + frame_int[:-2]
                        result = imu_feedback_parse(frame_int)
                        result_json =json.dumps(result)
    
        return result_json

    @staticmethod
    def connect(port):
        serial_port: Serial = None
        with Serial(port =port,baudrate=IMUOperations.DEFAULT_BAUDRATE,timeout=1.0) as p:
            timestart = rospy.Time.now().to_sec()
            imu_raw_data = p.read_until(b"\x55\x51")
            if rospy.Time.now().to_sec() - timestart < 1.0:
                serial_port = p
        return serial_port


            
    @staticmethod
    def scan(port):
        serial_port: Serial = None
        for baud in IMUOperations.SCAN_BAUDRATES:
            with Serial(port =port,baudrate=baud,timeout=1.0) as p:
                timestart = rospy.Time.now().to_sec()
                imu_raw_data = p.read_until(b"\x55\x51")
                if rospy.Time.now().to_sec() - timestart < 1.0:
                    serial_port = p 
        return serial_port
    
    @staticmethod
    def set_default_settings(serial_port):
        succeeded = False
        try: 
            with Serial(port = serial_port, baudrate=IMUOperations.DEFAULT_BAUDRATE,timeout =1.0) as p:
                        p.write(WIT_IMU_CONSTANTS.WIT_IMU_CMD_UNLOCK.value)
                        p.write(WIT_IMU_CONSTANTS.WIT_IMU_CMD_UNLOCK.value)
                        p.write(WIT_IMU_CONSTANTS.WIT_IMU_CMD_UNLOCK.value) 
                        p.write(WIT_IMU_CONSTANTS.WIT_IMU_CMD_UNLOCK.value)
                        p.write(WIT_IMU_CONSTANTS.WIT_IMU_CMD_SET_SCALE_ACC) #WIT_IMU_CMD_SET_SCALE_ACC
                        p.write(WIT_IMU_CONSTANTS.WIT_IMU_CMD_SET_SCALE_GYRO) #WIT_IMU_CMD_SET_SCALE_GYRO
                        p.write(WIT_IMU_CONSTANTS.WIT_IMU_CMD_SET_BANDWIDTH) #WIT_IMU_CMD_SET_BANDWIDTH
                        p.write(WIT_IMU_CONSTANTS.WIT_IMU_CMD_SET_SENDBACK_RATE) #WIT_IMU_CMD_SET_SENDBACK_RATE)
                        p.write(WIT_IMU_CONSTANTS.WIT_IMU_CMD_SET_SENDBACK_CONTENT.value)
                        succeeded = True
        except Exception as e:
            pass
        return succeeded
    
    @staticmethod
    def save_parameters(port):
        with Serial(port =port,baudrate=IMUOperations.DEFAULT_BAUDRATE,timeout=1.0) as p:
            p.write(WIT_IMU_CONSTANTS.WIT_IMU_CMD_SAVE_CFG.value)
            return True
    
    @staticmethod
    def close(port):
        return True
    

class IMUChecker:
    NODE_RATE = 5.0
    PORT = "/dev/imu"
    imu_model: IMUOperations = None
    _command = None
    _state = None
    serial_port: Serial = None

    IMU_MODEL_TABLE = {
        IMURion.IMU_TYPE: IMURion,
        IMUWIT.IMU_TYPE: IMUWIT,
    }

    def __init__(self) -> None:
        self.command = "NONE"
        self.cmd_params = ""
        self.state = IMUCheckerStates.INIT

        self.__STATES_METHODS = {
            (IMUCommands.NONE, IMUCheckerStates.INIT): self.initialize, # to IDLE
            (IMUCommands.SCAN, IMUCheckerStates.IDLE): self.scan, # to CONNECTED or stay
            (IMUCommands.CONNECT, IMUCheckerStates.IDLE): self.connect, # to CONNECTED or stay
            (IMUCommands.AUTO_DETECT, IMUCheckerStates.IDLE): self.auto_detect, # to CONNECTED or stay
            (IMUCommands.DISCONNECT, IMUCheckerStates.CONNECTED): self.disconnect, # to IDLE
            (IMUCommands.NONE, IMUCheckerStates.CONNECTED): self.parse_reading, # stay
            (IMUCommands.SET_DEFAULT, IMUCheckerStates.CONNECTED): self.set_default_settings, # stay
            (IMUCommands.SAVE, IMUCheckerStates.CONNECTED): self.save_parameters, # stay
        }

        #self.pub_state = rospy.Publisher()
        self.pub_state = IMU_STATE.Publisher()
        #self.pub_info = rospy.Publisher()
        self.pub_info = IMU_INFO.Publisher()
        #self.pub_reading = rospy.Publisher()
        self.pub_reading = IMU_DATA.Publisher()
        self.pub_configs = rospy.Publisher()
        #rospy.Service(handler=self._srv_command_handler)
        IMU_SRV_CMD.Services(self.srv_cb)


    def srv_cb(self, srv):
        self.command = srv.command
        self.cmd_params = srv.parameters
        


    @property
    def command(self):
        return self._command
    @command.setter
    def command(self, value: str):
        try:
            self._command = IMUCommands[value.upper()]
            logger.loginfo(f"Command set as: {self._command}")
        except Exception as e:
            self.log_with_frontend(f"Received wrong command: {value}!!")
            self._command = IMUCommands.NONE

    @property
    def state(self):
        return self._state
    @state.setter
    def state(self, value: IMUCheckerStates):
        if self._state != value:
            self._state = value
            logger.logwarn(f"State changed to: {self._state.name}")
            self.pub_state.publish(self.state.name)

    def log_with_frontend(self, log):
        logger.loginfo(log)
        self.pub_info.publish(log)

    def start(self):
        l = rospy.Rate(self.NODE_RATE)
        while not rospy.is_shutdown():
            state_method = self.__STATES_METHODS.get((self.command, self.state))
            if state_method:
                state_method()
            self.command = "NONE"
            l.sleep()

    def initialize(self):
        self.state = IMUCheckerStates.IDLE

    def _get_current_imu_settings(self):
        return {
            "baudrate": self.serial_port.baudrate
        }

    def _determine_imu_type(self):
        self.imu_model = self.IMU_MODEL_TABLE.get(self.cmd_params.upper())
        if self.imu_model:
            self.log_with_frontend(f"IMU model: {self.imu_model.IMU_TYPE}")
            return True
        self.log_with_frontend(f"IMU model: {self.cmd_params} not supported")
        return False

    def connect(self):
        self.state = IMUCheckerStates.CONNECTING
        if self._determine_imu_type():
            self.serial_port = self.imu_model.connect(self.PORT)
            if self.serial_port:
                self.log_with_frontend(f"Connected IMU on baudrate: {self.serial_port.baudrate}")
                self.state = IMUCheckerStates.CONNECTED
                self.pub_configs.publish(json.dumps(self._get_current_imu_settings()))
                return True
        self.log_with_frontend(f"Failed to connect IMU!")
        self.state = IMUCheckerStates.IDLE
        return False

    def scan(self):
        self.state = IMUCheckerStates.SCANNING
        if self._determine_imu_type():
            self.serial_port = self.imu_model.scan(self.PORT)
            if self.serial_port:
                self.log_with_frontend(f"Scanned IMU on baudrate: {self.serial_port.baudrate}")
                self.state = IMUCheckerStates.CONNECTED
                self.pub_configs.publish(json.dumps(self._get_current_imu_settings()))
                return True
        self.log_with_frontend(f"Cannot found the IMU!")
        self.state = IMUCheckerStates.IDLE
        return False

    def auto_detect(self):
        self.state = IMUCheckerStates.SCANNING
        for imu_model in self.IMU_MODEL_TABLE.values():
            serial_port: Serial = imu_model.scan(self.PORT)
            if serial_port:
                self.log_with_frontend(f"Found IMU model: {imu_model.IMU_TYPE} with baudrate: {serial_port.baudrate}!!")
                self.imu_model = imu_model
                self.serial_port = serial_port
                self.state = IMUCheckerStates.CONNECTED
                return True
        self.log_with_frontend(f"Cannot found any IMU...")
        self.state = IMUCheckerStates.IDLE
        return False

    def disconnect(self):
        if self._determine_imu_type():
            self.serial_port: Serial = self.imu_model.close(self.PORT) 
            if self.serial_port: 
                self.serial_port.close()
                self.serial_port = None
                self.imu_model = None
                self.state = IMUCheckerStates.IDLE
                return True

    def parse_reading(self):
        imu_msg = self.imu_model.parse_reading(self.serial_port)
        self.pub_reading.publish(imu_msg)
        return True

    def set_default_settings(self):
        if self._determine_imu_type():
            self.serial_port: Serial = self.imu_model.set_default_settings(self.PORT)
            if self.serial_port:
                return self.parse_reading()

    def save_parameters(self):
        if self._determine_imu_type():
            self.serial_port: Serial = self.imu_model.save_parameters(self.PORT)
            if self.serial_port:
                self.log_with_frontend("CFG SAVED")
                return True

if __name__ == "__main__":
    rospy.init_node("imu_checker_node")
    imu_checker = IMUChecker()
    imu_checker.start()
