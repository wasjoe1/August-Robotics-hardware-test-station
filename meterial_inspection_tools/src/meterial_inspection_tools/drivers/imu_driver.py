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

from boothbot_msgs.srv import (Command, CommandRequest,CommandResponse)
from imu_driver_constants import (
    IMU_CONSTANTS,
    WIT_IMU_CONSTANTS,
    RION_IMU_CONSTANTS
)

timeout = 1.0
# convert 8bit hex to signed int
def c8h2sint(value):
    return -(value & 0x80) | (value & 0x7F)


#refer to datasheet for calculations method 
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




class IMUCHECK(object):
    def __init__(self):
        super(IMUCHECK,self).__init__()
        self.port = "/dev/imu"
        self.baud = None
        self.button = None
        self.state_manager = self.statemanager()
        IMU_SRV_CMD.Services(self.srv_cb)
        self.imu_data_pub = IMU_DATA.Publisher()
        self.imu_info_pub = IMU_INFO.Publisher()
        self.imu_state_pub = IMU_STATE.Publisher()


    #process service
    def srv_cb(self, srv):
        self.receivedparameters = [srv.button,srv.parameter1, srv.parameter2, srv.parameter3, srv.parameter4]
        self.button = self.receivedparameters[0]
        self.response = CommandResponse()
        logger.loginfo("Service call back was executed")
        self.execute_command(self.button)
        return True

    # statemanager
    class statemanager:
        def __init__(self):
            self.current_state = "IDLE"
            self.commands = { 
                ("CONNECT", "IDLE"): IMUCHECK.connect, 
                ("SCAN", "IDLE") : IMUCHECK.scan,
                ("CLOSE", "CONNECTED") :IMUCHECK.close,
                ("SAVE", "CONNECTED") : IMUCHECK.save,
                ("SET", "CONNECTED") :IMUCHECK.set,
                }
        
        #update state
        def change_state(self,new_state):
            self.current_state = new_state
            logger.loginfo(self.current_state)
        
        # if current state is the given state and input is the command given, then state change_method
    def execute_command(self, command):
        command_key = (command,self.state_manager.current_state)
        if command_key in self.state_manager.commands:
                state_change_method =self.state_manager.commands[command_key]
                state_change_method(self)
        else:
            logger.logwarn("Invalid command and state")


# PUBLISHERS
    def publisher_data(self,message):
        self.imu_data_pub.publish(message)

    def publisher_info(self,message):
        self.imu_info_pub.publish(message) 

    def publisher_state(self,message):
        self.imu_state_pub.publish(message)

    def check_imu_model(self):
         for baud in (115200,9600):
            with Serial(port=self.port, baudrate=baud, timeout=1.0) as p:
                p.write(WIT_IMU_CONSTANTS.WIT_IDENTIFIER.value)
                rospy.sleep(0.1)
                response = p.read(1)
                if response == WIT_IMU_CONSTANTS.WIT_IDENTIFIER.value:
                    logger.loginfo_throttle(2,"WIT_IMU_connected")
                    return "WIT_IMU_connected"
                p.write(RION_IMU_CONSTANTS.RION_IDENTIFIER.value)
                rospy.sleep(0.1)
                if response == RION_IMU_CONSTANTS.RION_IDENTIFIER.value:
                    logger.loginfo_throttle(2,"RION_IMU_connected")
                    return "RION_IMU_connected"
                    


         
         
    def check_usb_connections(self):
        #check if USB is connected
        if os.path.exists(self.port):
            logger.loginfo_throttle(2, "USB device connected")
            #self.state_manager.change_state("IDLE")
            #self.publisher_state("IDLE")
        else:
            logger.loginfo_throttle(2, "USB device disconnected")
            self.state_manager.change_state("DISCONNECTED")
            self.publisher_state("DISCONNECTED")
         

# buttons and functions
    def connect(self):
        baud = IMU_CONSTANTS.FIXED_BAUDRATE.value
        self.publisher_info("Checking baudrate {}".format(baud))
        with Serial(port=self.port, baudrate=baud, timeout=1.0) as p:
                    timestart = rospy.Time.now().to_sec()
                    if rospy.Time.now().to_sec() - timestart < timeout: 
                        self.baud = baud
                        logger.loginfo("Baudrate detected {}".format(self.baud))
                        self.publisher_info("Baudrate at {}".format(self.baud))
                        self.publisher_state("CONNECTED")
                        self.state_manager.change_state("CONNECTED") 
                        

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
                            logger.loginfo("Got: \n {}".format(result_json))
                            self.publisher_data("Got: {}".format(result_json))
                    else:
                        logger.loginfo("ERROR, not at correct baudrate")
                        self.publisher_info("Wrong baudrate, press SCAN to set") 


        
    def scan(self):
        for baud in (115200,9600):
            logger.loginfo("Checking baudrate {}".format(baud))
            with Serial(port=self.port, baudrate=baud, timeout=1.0) as p:
                timestart = rospy.Time.now().to_sec()
                imu_raw_data = p.read_until(b"\x55\x51")
                if rospy.Time.now().to_sec() - timestart < timeout: 
                    self.baud = baud
                    logger.loginfo("Baudrate detected {}".format(self.baud))
                    self.publisher_info("Baudrate at {}".format(self.baud))
                    if self.baud == IMU_CONSTANTS.FIXED_BAUDRATE.value: 
                        self.publisher_state("CONNECTED")
                        self.state_manager.change_state("CONNECTED") 
                        logger.loginfo("CONNECTED")
                    else:
                        p.write(WIT_IMU_CONSTANTS.WIT_IMU_CMD_UNLOCK.value)
                        p.write(WIT_IMU_CONSTANTS.WIT_IMU_CMD_UNLOCK.value)
                        p.write(WIT_IMU_CONSTANTS.WIT_IMU_CMD_UNLOCK.value) 
                        p.write(WIT_IMU_CONSTANTS.WIT_IMU_CMD_UNLOCK.value)
                        p.write(WIT_IMU_CONSTANTS.WIT_IMU_CMD_SET_BAUDRATE.value)
                        self.baud = 115200
                        p.write(WIT_IMU_CONSTANTS.WIT_IMU_CMD_SAVE_CFG.value)   
                        self.publisher_info("baudrate is set to {}".format(self.baud))
                        self.publisher_state("CONNECTED")
                        self.state_manager.change_state("CONNECTED") 
                        logger.loginfo("CONNECTED")

    def set(self):
        logger.loginfo("Setting parameters")
        self.publisher_info("Setting parameters")
        ACC =str(self.receivedparameters[1])
        DICT_ACC = WIT_IMU_CONSTANTS.ACC_SPEED.value
        GYRO = str(self.receivedparameters[2])
        DICT_GYRO = WIT_IMU_CONSTANTS.GYRO_DEGREES.value
        BANDWIDTH = str(self.receivedparameters[3])
        DICT_BANDWIDTH = WIT_IMU_CONSTANTS.BANDWIDTH_HZ.value
        SENDBACK = str(self.receivedparameters[4])
        DICT_SENDBACK =WIT_IMU_CONSTANTS.SENDBACK_RATE_HZ.value
        
        with Serial(port=self.port, baudrate=IMU_CONSTANTS.FIXED_BAUDRATE.value, timeout=1.0) as p:
                    p.write(WIT_IMU_CONSTANTS.WIT_IMU_CMD_UNLOCK.value)
                    p.write(WIT_IMU_CONSTANTS.WIT_IMU_CMD_UNLOCK.value)
                    p.write(WIT_IMU_CONSTANTS.WIT_IMU_CMD_UNLOCK.value) 
                    p.write(WIT_IMU_CONSTANTS.WIT_IMU_CMD_UNLOCK.value)
                    p.write(DICT_ACC[ACC]) #WIT_IMU_CMD_SET_SCALE_ACC
                    p.write(DICT_GYRO[GYRO]) #WIT_IMU_CMD_SET_SCALE_GYRO
                    p.write(DICT_BANDWIDTH[BANDWIDTH]) #WIT_IMU_CMD_SET_BANDWIDTH
                    p.write(DICT_SENDBACK[SENDBACK]) #WIT_IMU_CMD_SET_SENDBACK_RATE)
                    p.write(WIT_IMU_CONSTANTS.WIT_IMU_CMD_SET_SENDBACK_CONTENT.value)
                    self.publisher_info(
                       "Setting ACC scale to: 16g/s2! \n Setting gyro scale to: 2000deg/s \n Setting bandwidth to: 250Hz \n Setting feedback rate to: 200Hz!\n Setting feedback content as:\n0x51->acc 0x52->w 0x53->angle 0x59->quaternion" 
                       )
                    logger.loginfo("Setting ACC scale to: 16g/s2! \n Setting gyro scale to: 2000deg/s \n Setting bandwidth to: 250Hz \n Setting feedback rate to: 200Hz!\n Setting feedback content as:\n0x51->acc 0x52->w 0x53->angle 0x59->quaternion" 
                       )
                    
                    
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
                        logger.loginfo("Got: \n {}".format(result_json))
                        self.publisher_data("Got: {}".format(result_json))




    def save(self):
        with Serial(port=self.port, baudrate=IMU_CONSTANTS.FIXED_BAUDRATE.value, timeout=1.0) as p:
            p.write(WIT_IMU_CONSTANTS.WIT_IMU_CMD_SAVE_CFG.value)
            logger.loginfo("CFG saved!")
            self.publisher_info("CFG SAVED")
      


    def close(self):
        logger.loginfo("EXITING")
        self.publisher_info("EXITING")
        self.state_manager.change_state("DISCONNECTED")
        self.publisher_state("DISCONNECTED")



    def main_loop(self):
        logger.loginfo("starting main loop")
        rospy.Rate(5)
        while not rospy.is_shutdown():
            self.check_usb_connections()
            self.check_imu_model()
            rospy.Rate(5)
     

    

if __name__ == "__main__":
    rospy.init_node("imu_check")
    imu = IMUCHECK()
    imu.main_loop()


    