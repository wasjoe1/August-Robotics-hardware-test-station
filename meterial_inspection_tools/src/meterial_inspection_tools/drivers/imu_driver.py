#!/usr/bin/python3
# -*- coding: utf-8 -*-

import time
import math
import json
import rospy
logger = rospy
import binascii
import os
from enum import Enum
from serial import Serial
from std_msgs.msg import String
from meterial_inspection_tools.srv import IMUcontrol
from meterial_inspection_tools.ros_interface import (
    IMU_DATA,
    IMU_SRV_CMD,
    IMU_STATE,
    IMU_INFO
)
from sensor_msgs.msg import Imu
from boothbot_msgs.srv import (Command, CommandRequest,CommandResponse)
from imu_driver_constants import (
    IMU_CONSTANTS,
    WIT_IMU_CONSTANTS,
    RION_IMU_CONSTANTS
)

#RION sensor readings
bps = 115200
timex = 5

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

#RION sensor readings
def byte_str(val):
    # print("got val {}".format(val))
    if len(str(hex(val)[2:])) == 1:
        # print("length {}".format(len(str(hex(val)[2:]))))
        return "0" + str(hex(val)[2:])
    return str(hex(val)[2:])


import numpy as np # Scientific computing library for Python
 
def get_quaternion_from_euler(roll, pitch, yaw):
    """
    Convert an Euler angle to a quaternion.

    Input
    :param roll: The roll (rotation around x-axis) angle in radians.
    :param pitch: The pitch (rotation around y-axis) angle in radians.
    :param yaw: The yaw (rotation around z-axis) angle in radians.

    Output
    :return qx, qy, qz, qw: The orientation in quaternion [x,y,z,w] format
    """
    qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
    qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
    qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)

    return [qx, qy, qz, qw]
   

timeout = 1.0
# convert 8bit hex to signed int
def c8h2sint(value):
    return -(value & 0x80) | (value & 0x7F)


#WIT sensor readings
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

class statemanager(object):
        def __init__(self):
            self.current_state = "IDLE"
            self.commands = { 
                (WIT_IMU_CONSTANTS.CONNECT.value, IMU_CONSTANTS.STATE_IDLE.value): WIT.connect, 
                (WIT_IMU_CONSTANTS.SCAN.value, IMU_CONSTANTS.STATE_IDLE.value) : WIT.scan,
                (WIT_IMU_CONSTANTS.CLOSE.value, IMU_CONSTANTS.STATE_CONNECTED.value) :WIT.close,
                (WIT_IMU_CONSTANTS.SAVE.value, IMU_CONSTANTS.STATE_CONNECTED.value) : WIT.save,
                (WIT_IMU_CONSTANTS.SET.value, IMU_CONSTANTS.STATE_CONNECTED.value) :WIT.set,
                (RION_IMU_CONSTANTS.RSCAN.value, IMU_CONSTANTS.STATE_IDLE.value): RION.scan,
                (RION_IMU_CONSTANTS.RSET.value, IMU_CONSTANTS.STATE_CONNECTED.value):RION.set,
                (RION_IMU_CONSTANTS.RCONNECT.value,IMU_CONSTANTS.STATE_IDLE.value):RION.connect,
                }
        
        #update state    
        def change_state(self,new_state):
            self.current_state = new_state
            imu.publisher_logging_state(self.current_state)
        
        # if current state is the given state and input is the command given, then state change_method
        def execute_command(self, command):
            command_key = (command,self.current_state)
            if command_key in self.commands:
                    state_change_method =self.commands[command_key]
                    state_change_method()
            else:
                imu.publisher_logging_info("Invalid command and state")


class RION(object):

    """
    used only when usb checks that it is RION model
    CONNECT assumes baudrate of 115200 and reads current sensor values
    SCAN is available to find baudrate and reads current sensor values
    SET to set baudrate to 115200
    CLOSE to disconnect usb
    """
        
    def reflash(self):
        self.s_data = None
        Serial(IMU_CONSTANTS.PORT.value,IMU_CONSTANTS.FIXED_BAUDRATE.value,timeout=timeout).reset_input_buffer()
        Serial(IMU_CONSTANTS.PORT.value,IMU_CONSTANTS.FIXED_BAUDRATE.value,timeout=timeout).reset_output_buffer()
        self.s_data = Serial(IMU_CONSTANTS.PORT.value,IMU_CONSTANTS.FIXED_BAUDRATE.value,timeout=timeout).read(32)

    def factor(self,s):
        f = 1
        if s[0] == "1":
            f = f * -1
        return f

    def orien(self):
        roll_ori = self.split_data(ROLL)
        pitch_ori = self.split_data(PITCH)
        yaw_ori = self.split_data(YAW)
        roll = int(roll_ori[1:6])/100.0 * self.factor(roll_ori)
        pitch = int(pitch_ori[1:6])/100.0 * self.factor(pitch_ori)
        yaw = int(yaw_ori[1:6])/100.0 * self.factor(yaw_ori)
        roll = math.radians(roll)
        pitch = math.radians(pitch)
        yaw = math.radians(yaw)
        return (roll, pitch, yaw)

    def acc(self):
        acc_x_ori = self.split_data(ACC_X)
        acc_y_ori = self.split_data(ACC_Y)
        acc_z_ori = self.split_data(ACC_Z)
        acc_x = int(acc_x_ori[1:6])/1000.0 * self.factor(acc_x_ori) * G
        acc_y = int(acc_y_ori[1:6])/1000.0 * self.factor(acc_y_ori) * G
        acc_z = int(acc_z_ori[1:6])/1000.0 * self.factor(acc_z_ori) * G
        return (acc_x, acc_y, acc_z)

    def gyro(self):
        gyro_x_ori = self.split_data(GYRO_X)
        gyro_y_ori = self.split_data(GYRO_Y)
        gyro_z_ori = self.split_data(GYRO_Z)
        gyro_x = int(gyro_x_ori[1:6])/100.0 * self.factor(gyro_x_ori)
        gyro_y = int(gyro_y_ori[1:6])/100.0 * self.factor(gyro_y_ori)
        gyro_z = int(gyro_z_ori[1:6])/100.0 * self.factor(gyro_z_ori)
        gyro_x = math.radians(gyro_x)
        gyro_y = math.radians(gyro_y)
        gyro_z = math.radians(gyro_z)
        return (gyro_x, gyro_y, gyro_z)

    def split_data(self, start, offset=3):
        s = ""
        for i in range(0, offset):
            s += byte_str(self.s_data[start+i])
        # return byte_str(res[4]) + byte_str(res[5]) + byte_str(res[6])
        return s

    @staticmethod
    def connect():
        if imu.model == IMU_CONSTANTS.MODEL_RION.value:
            baud = IMU_CONSTANTS.FIXED_BAUDRATE.value
            imu.publisher_logging_info("Checking baudrate {}".format(baud))
            with Serial(port=IMU_CONSTANTS.PORT.value, baudrate=baud, timeout=1.0) as p:
                            rion_instance.reflash()
                            imu.publisher_logging_info("Baudrate at {}".format(baud))
                            imu.publisher_logging_state(IMU_CONSTANTS.STATE_CONNECTED.value)
                            state_manager.change_state(IMU_CONSTANTS.STATE_CONNECTED.value) 
                            #publish readings
                            (roll,pitch,yaw) = rion_instance.orien()
                            (acc_x, acc_y, acc_z) =rion_instance.acc()
                            (gyro_x, gyro_y, gyro_z) = rion_instance.gyro()

                            (x, y, z, w) = get_quaternion_from_euler(roll, pitch, yaw)

                            imu_sensor.orientation.x = x
                            imu_sensor.orientation.y = y
                            imu_sensor.orientation.z = z
                            imu_sensor.orientation.w = w
                            imu_sensor.angular_velocity.x = gyro_x
                            imu_sensor.angular_velocity.y = gyro_y
                            imu_sensor.angular_velocity.z = gyro_z
                            imu_sensor.linear_acceleration.x = acc_x
                            imu_sensor.linear_acceleration.y = acc_y
                            imu_sensor.linear_acceleration.z = acc_z
                            seralized_imu_sensor = str(imu_sensor)
                            seralized_imu_sensor_json = json.dumps(seralized_imu_sensor)
                            imu.publisher_logging_data(seralized_imu_sensor_json)


        else:
            imu.publisher_logging_info(RION_IMU_CONSTANTS.ERROR_to_Rion.value)
            raise ValueError(RION_IMU_CONSTANTS.ERROR_to_Rion.value)


    @staticmethod
    def scan():
        if imu.model ==IMU_CONSTANTS.MODEL_RION.value:
            for baud in (9600,115200):
                 with Serial(port=IMU_CONSTANTS.PORT.value, baudrate=baud, timeout=1.0) as p:
                    rion_instance.reflash() 
                    #publish readings
                    (roll,pitch,yaw) = rion_instance.orien()
                    (acc_x, acc_y, acc_z) =rion_instance.acc()
                    (gyro_x, gyro_y, gyro_z) = rion_instance.gyro()

                    (x, y, z, w) = get_quaternion_from_euler(roll, pitch, yaw)

                    imu_sensor.orientation.x = x
                    imu_sensor.orientation.y = y
                    imu_sensor.orientation.z = z
                    imu_sensor.orientation.w = w
                    imu_sensor.angular_velocity.x = gyro_x
                    imu_sensor.angular_velocity.y = gyro_y
                    imu_sensor.angular_velocity.z = gyro_z
                    imu_sensor.linear_acceleration.x = acc_x
                    imu_sensor.linear_acceleration.y = acc_y
                    imu_sensor.linear_acceleration.z = acc_z
                    seralized_imu_sensor = str(imu_sensor)
                    seralized_imu_sensor_json = json.dumps(seralized_imu_sensor)
                    imu.publisher_logging_data(seralized_imu_sensor_json)
                    imu.publisher_logging_info("Baudrate at {}".format(baud))
                    imu.publisher_logging_state(IMU_CONSTANTS.STATE_CONNECTED.value)
                    state_manager.change_state(IMU_CONSTANTS.STATE_CONNECTED.value)
                    return baud
                     
        else:
            imu.publisher_logging_info(RION_IMU_CONSTANTS.ERROR_to_Rion.value)
            raise ValueError(RION_IMU_CONSTANTS.ERROR_to_Rion.value)
                    
    @staticmethod
    def set():
        if imu.model ==IMU_CONSTANTS.MODEL_RION.value:
            baud_rate = RION.scan()
            with Serial(port=IMU_CONSTANTS.PORT.value, baudrate=baud_rate, timeout=1.0) as p:
                p.write(RION_IMU_CONSTANTS.RION_IMU_CMD_SET_BAUDRATE.value)
                imu.publisher_logging_info("baudrate is set to {}".format(IMU_CONSTANTS.FIXED_BAUDRATE.value))
        else:
            imu.publisher_logging_info(RION_IMU_CONSTANTS.ERROR_to_Rion.value)
            raise ValueError(RION_IMU_CONSTANTS.ERROR_to_Rion.value)



class WIT(object):

    """
    used only when usb checks that it is WIT model
    CONNECT assumes baudrate of 115200 and reads current sensor values
    SCAN is available to find baudrate and reads current sensor values
    SET to set baudrate to 115200 and other keyed-in parameters
    SAVE to save changes
    CLOSE to disconnect usb
    """

    @staticmethod
    def connect():
        if imu.model == IMU_CONSTANTS.MODEL_WIT.value:
            detectedbaud = None
            baud = IMU_CONSTANTS.FIXED_BAUDRATE.value
            imu.publisher_logging_info("Checking baudrate {}".format(baud))
            with Serial(port=IMU_CONSTANTS.PORT.value, baudrate=baud, timeout=1.0) as p:
                        timestart = rospy.Time.now().to_sec()
                        imu_raw_data = p.read_until(b"\x55\x51")
                        if rospy.Time.now().to_sec() - timestart < timeout: 
                            detectedbaud = baud
                            imu.publisher_logging_info("Baudrate at {}".format(detectedbaud))
                            imu.publisher_logging_state(IMU_CONSTANTS.STATE_CONNECTED.value)
                            state_manager.change_state(IMU_CONSTANTS.STATE_CONNECTED.value) 
                            #publish readings
                            for t in range(3):
                                imu_raw_data = p.read_until(b"\x55\x51")
                                if len(imu_raw_data) != 44:
                                    logger.loginfo("Got {} data".format(len(imu_raw_data)))
                                    rospy.Rate(5)
                                    continue
                                frame_int = [int(x) for x in imu_raw_data]
                                frame_int = frame_int[-2:] + frame_int[:-2]
                                result = imu_feedback_parse(frame_int)
                                result_json = json.dumps(result)
                                imu.publisher_logging_data("Got: {}".format(result_json))
                        else:
                            imu.publisher_logging_info("Wrong baudrate, press SCAN to set") 

        else: 
            imu.publisher_logging_info(WIT_IMU_CONSTANTS.ERROR_to_wit.value)
            raise ValueError(WIT_IMU_CONSTANTS.ERROR_to_wit.value)
            

    @staticmethod
    def scan():
        if imu.model == IMU_CONSTANTS.MODEL_WIT.value:
            detectedbaud = None
            for baud in (115200,9600):
                imu.publisher_logging_info("Checking baudrate {}".format(baud))
                with Serial(port=IMU_CONSTANTS.PORT.value, baudrate=baud, timeout=1.0) as p:
                    timestart = rospy.Time.now().to_sec()
                    imu_raw_data = p.read_until(b"\x55\x51")
                    if rospy.Time.now().to_sec() - timestart < timeout: 
                        detectedbaud = baud
                        imu.publisher_logging_info("Baudrate at {}".format(detectedbaud))
                        imu.publisher_logging_state(IMU_CONSTANTS.STATE_CONNECTED.value)
                        state_manager.change_state(IMU_CONSTANTS.STATE_CONNECTED.value)
                        #publish readings
                        for t in range(3):
                            imu_raw_data = p.read_until(b"\x55\x51")
                            if len(imu_raw_data) != 44:
                                logger.loginfo("Got {} data".format(len(imu_raw_data)))
                                rospy.Rate(5)
                                continue
                            frame_int = [int(x) for x in imu_raw_data]
                            frame_int = frame_int[-2:] + frame_int[:-2]
                            result = imu_feedback_parse(frame_int)
                            result_json = json.dumps(result)
                            imu.publisher_logging_data("Got: {}".format(result_json)) 
        else: 
            imu.publisher_logging_info(WIT_IMU_CONSTANTS.ERROR_to_wit.value)
            raise ValueError(WIT_IMU_CONSTANTS.ERROR_to_wit.value)
            


    @staticmethod
    def set():
        if imu.model == IMU_CONSTANTS.MODEL_WIT.value:
            imu.publisher_logging_info("Setting parameters")
            ACC =str(imu.receivedparameters[1])
            DICT_ACC = WIT_IMU_CONSTANTS.ACC_SPEED.value
            GYRO = str(imu.receivedparameters[2])
            DICT_GYRO = WIT_IMU_CONSTANTS.GYRO_DEGREES.value
            BANDWIDTH = str(imu.receivedparameters[3])
            DICT_BANDWIDTH = WIT_IMU_CONSTANTS.BANDWIDTH_HZ.value
            SENDBACK = str(imu.receivedparameters[4])
            DICT_SENDBACK =WIT_IMU_CONSTANTS.SENDBACK_RATE_HZ.value
            
            with Serial(port=IMU_CONSTANTS.PORT.value, baudrate=IMU_CONSTANTS.FIXED_BAUDRATE.value, timeout=1.0) as p:
                        p.write(WIT_IMU_CONSTANTS.WIT_IMU_CMD_UNLOCK.value)
                        p.write(WIT_IMU_CONSTANTS.WIT_IMU_CMD_UNLOCK.value)
                        p.write(WIT_IMU_CONSTANTS.WIT_IMU_CMD_UNLOCK.value) 
                        p.write(WIT_IMU_CONSTANTS.WIT_IMU_CMD_UNLOCK.value)
                        p.write(WIT_IMU_CONSTANTS.WIT_IMU_CMD_SET_BAUDRATE.value)
                        p.write(DICT_ACC[ACC]) #WIT_IMU_CMD_SET_SCALE_ACC
                        p.write(DICT_GYRO[GYRO]) #WIT_IMU_CMD_SET_SCALE_GYRO
                        p.write(DICT_BANDWIDTH[BANDWIDTH]) #WIT_IMU_CMD_SET_BANDWIDTH
                        p.write(DICT_SENDBACK[SENDBACK]) #WIT_IMU_CMD_SET_SENDBACK_RATE)
                        p.write(WIT_IMU_CONSTANTS.WIT_IMU_CMD_SET_SENDBACK_CONTENT.value)
                        imu.publisher_logging_info(
                        "Baudrate to 115200 \n Setting ACC scale to: 16g/s2! \n Setting gyro scale to: 2000deg/s \n Setting bandwidth to: 250Hz \n Setting feedback rate to: 200Hz!\n Setting feedback content as:\n0x51->acc 0x52->w 0x53->angle 0x59->quaternion" 
                        )
                        imu.publisher_logging_state(IMU_CONSTANTS.STATE_CONNECTED.value)
                        state_manager.change_state(IMU_CONSTANTS.STATE_CONNECTED.value) 
                        
                
                    #publish readings
                        for t in range(3):
                            imu_raw_data = p.read_until(b"\x55\x51")
                            if len(imu_raw_data) != 44:
                                logger.loginfo("Got {} data".format(len(imu_raw_data)))
                                rospy.Rate(5)
                                continue
                            frame_int = [int(x) for x in imu_raw_data]
                            frame_int = frame_int[-2:] + frame_int[:-2]
                            result = imu_feedback_parse(frame_int)
                            result_json = json.dumps(result)
                            imu.publisher_logging_data("Got: {}".format(result_json))

        else: 
            imu.publisher_logging_info(WIT_IMU_CONSTANTS.ERROR_to_wit.value)
            raise ValueError(WIT_IMU_CONSTANTS.ERROR_to_wit.value)
            



    @staticmethod
    def save():
        if imu.model == IMU_CONSTANTS.MODEL_WIT.value:
            with Serial(port=IMU_CONSTANTS.PORT.value, baudrate=IMU_CONSTANTS.FIXED_BAUDRATE.value, timeout=1.0) as p:
                p.write(WIT_IMU_CONSTANTS.WIT_IMU_CMD_SAVE_CFG.value)
                imu.publisher_logging_info("CFG SAVED")

        else: 
            imu.publisher_logging_info(WIT_IMU_CONSTANTS.ERROR_to_wit.value)
            raise ValueError(WIT_IMU_CONSTANTS.ERROR_to_wit.value)
    
      

    @staticmethod
    def close():
        imu.publisher_logging_info("EXITING")
        state_manager.change_state(IMU_CONSTANTS.STATE_DISCONNECTED.value)
        imu.publisher_logging_state(IMU_CONSTANTS.STATE_DISCONNECTED.value)





class IMUCHECK(object):
    def __init__(self):
        """
        Checks IMU_model, checks USB connections
        Publisher instance, logger and publish methods, service method
        main loop
        """

        super(IMUCHECK,self).__init__()
        self.button = None
        IMU_SRV_CMD.Services(self.srv_cb)
        self.imu_data_pub = IMU_DATA.Publisher()
        self.imu_info_pub = IMU_INFO.Publisher()
        self.imu_state_pub = IMU_STATE.Publisher()
        self.check_imu_model_once = False
        self.model = None


    #process service
    def srv_cb(self, srv):
        self.receivedparameters = [srv.button,srv.parameter1, srv.parameter2, srv.parameter3, srv.parameter4]
        self.button = self.receivedparameters[0]
        self.response = CommandResponse()
        self.publisher_logging_info("Service executed")
        state_manager.execute_command(self.button)
        return True


# PUBLISHERS
    def publisher_logging_data(self,message):
        logger.loginfo(message)
        self.imu_data_pub.publish(message)

    def publisher_logging_info(self,message):
        logger.loginfo(message)
        self.imu_info_pub.publish(message) 

    def publisher_logging_state(self,message):
        logger.loginfo(message)
        self.imu_state_pub.publish(message)
        
    def check_imu_model(self):
         for baud in (115200,9600):
            with Serial(port=IMU_CONSTANTS.PORT.value, baudrate=baud, timeout=1.0) as p:
                p.write(WIT_IMU_CONSTANTS.WIT_IDENTIFIER.value)
                rospy.sleep(0.1)
                response = p.read(1)
                if response == WIT_IMU_CONSTANTS.WIT_IDENTIFIER.value:
                    logger.loginfo_throttle(3,"WIT_IMU_connected")
                    self.model = IMU_CONSTANTS.MODEL_WIT.value
            
                p.write(RION_IMU_CONSTANTS.RION_IDENTIFIER.value)
                rospy.sleep(0.1)
                if response == RION_IMU_CONSTANTS.RION_IDENTIFIER.value:
                    logger.loginfo_throttle(3,"RION_IMU_connected")
                    self.model = IMU_CONSTANTS.MODEL_RION.value
                    
        
         
         
    def check_usb_connections(self):
        #check if USB is connected
        if os.path.exists(IMU_CONSTANTS.PORT.value):
            logger.loginfo_throttle(2, "USB device connected")
            #state_manager.change_state("IDLE")
            self.publisher_logging_state(state_manager.current_state)
        else:
            logger.loginfo_throttle(2, "USB device disconnected")
            state_manager.change_state(IMU_CONSTANTS.STATE_DISCONNECTED.value)
            self.publisher_logging_state(IMU_CONSTANTS.STATE_DISCONNECTED.value)

        
    def main_loop(self):
        logger.loginfo("starting main loop")
        rospy.Rate(5)
        while not rospy.is_shutdown():
            self.check_usb_connections()
            if state_manager.current_state != IMU_CONSTANTS.STATE_DISCONNECTED.value:
                self.check_imu_model()
            rospy.Rate(5)
     

if __name__ == "__main__":
    rospy.init_node("imu_check")
    imu = IMUCHECK()
    state_manager = statemanager()
    imu_sensor = Imu()
    rion_instance = RION()
    imu.main_loop()


    