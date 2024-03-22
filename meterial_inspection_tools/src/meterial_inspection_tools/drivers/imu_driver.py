#!/usr/bin/python3
# -*- coding: utf-8 -*-


import time
import math
from pprint import PrettyPrinter
import rospy
logger = rospy
import binascii
import os

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



# Settings 
NODE_RATE = 10
MOTOR_TOLERANCE = 0.0001
FIXED_BAUDRATE = 115200


WIT_IMU_CMD_CALI_ACC = b"\xff\xaa\x01\x01\x00" 
WIT_IMU_CMD_SAVE_CFG = b"\xff\xaa\x00\x00\x00"
WIT_IMU_CMD_UNLOCK = b"\xff\xaa\x69\x88\xb5"
WIT_IMU_CMD_SET_BAUDRATE = b"\xff\xaa\x04\x06\x00"  # 115200

ACC_SPEED = {
    "2": b"\xff\xaa\x21\x00\x00", #2g
    "4": b"\xff\xaa\x21\x01\x00", #4g
    "8": b"\xff\xaa\x21\x02\x00", #8g
    "16": b"\xff\xaa\x21\x03\x00", #16g
}


GYRO_DEGREES = { 
    "250": b"\xff\xaa\x20\x00\x00",
    "500": b"\xff\xaa\x20\x01\x00",
    "1000": b"\xff\xaa\x20\x02\x00",
    "2000": b"\xff\xaa\x20\x03\x00", 
}


BANDWIDTH_HZ = {
    "200": b"\xff\xaa\x1f\x01\x00",
    "98" : b"\xff\xaa\x1f\x02\x00",
    "42": b"\xff\xaa\x1f\x03\x00",
    "20": b"\xff\xaa\x1f\x04\x00",
    "10": b"\xff\xaa\x1f\x05\x00",
    "5": b"\xff\xaa\x1f\x06\x00",
}


SENDBACK_RATE_HZ = {
    "0.1":b"\xff\xaa\x03\x01\x00", 
    "0.5": b"\xff\xaa\x03\x02\x00",
    "1": b"\xff\xaa\x03\x03\x00",
    "2": b"\xff\xaa\x03\x04\x00",
    "5": b"\xff\xaa\x03\x05\x00",
    "10":b"\xff\xaa\x03\x06\x00",
    "20":b"\xff\xaa\x03\x07\x00",
    "50":b"\xff\xaa\x03\x08\x00" ,
    "100":b"\xff\xaa\x03\x09\x00",
    "125":b"\xff\xaa\x03\x0a\x00",
    "200":b"\xff\xaa\x03\x0b\x00",
    "11":b"\xff\xaa\x03\x0c\x00", #once
    "0":b"\xff\xaa\x03\x0d\x00", #none
}

WIT_IMU_CMD_SET_SENDBACK_CONTENT = b"\xff\xaa\x02\x0e\x02"
WIT_IMU_CMD_RESET_YAW = b"\xff\xaa\x01\x04\x00"

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

#Check IMU
class IMUCHECK(object):
    def __init__(self):
        super(IMUCHECK,self).__init__()
        self.port = "/dev/imu"
        self.baud = None
        self.button = None
        self.state = None
        self.parameter1=None
        self.parameter2=None
        self.parameter3=None
        self.parameter4=None
        self.imu_msg_data = IMU_DATA.get_new_msg() # returns 0 0 0 0
        self.imu_msg_state = IMU_STATE.get_new_msg()
        self.imu_msg_info = IMU_INFO.get_new_msg()
        self.imu_puber = IMU_DATA.Publisher()
        self.imu_puber_string = IMU_STATE.Publisher()
        self.imu_puber_string_info = IMU_INFO.Publisher()
        
        
        IMU_SRV_CMD.Services(self.srv_cb)
     
    #process service
    def srv_cb(self, srv):
        logger.loginfo("srv")
        logger.loginfo(srv)
        self.button = srv.button
        self.parameter1= str(srv.parameter1) #stored in self.parameters in case value is needed for "SET"
        self.parameter2 =str(srv.parameter2)
        self.parameter3 = str(srv.parameter3)
        self.parameter4 = str(srv.parameter4)
        response = CommandResponse()
        logger.loginfo("Service call back was executed")
        return True
    

    def initialize(self):
        self.state = "IDLE"  


    def run(self):
        logger.logwarn("Running")
        pp = PrettyPrinter(indent=2)
        rate = rospy.Rate(5)  
        rate_quick = rospy.Rate(1) 
        rate_wait_for_refresh = rospy.Rate(3) 
        timeout =1.0

        while not rospy.is_shutdown():    
            #check if USB is connected
            if os.path.exists(self.port):
                logger.loginfo_throttle(2, "USB device connected")
            else:
                logger.loginfo_throttle(2, "USB device disconnected")
                self.state = "DISCONNECTED"
                self.imu_msg_state = "DISCONNECTED"
                self.imu_puber_string.publish(self.imu_msg_state)

            #CONNECT: Check if baudrate is set at 115200 and return current readings
            if self.button == "CONNECT" and (self.state == "IDLE"):
                logger.logwarn("Connect button executed")
                logger.logwarn("Trying to initialize....")
                logger.loginfo("Detecting IMU...")
                self.imu_msg_info = "CONNECTING"
                self.imu_puber_string_info.publish(self.imu_msg_info)
                baud = FIXED_BAUDRATE
                logger.loginfo("Checking baudrate {}".format(baud))
                with Serial(port=self.port, baudrate=baud, timeout=1.0) as p:
                    timestart = rospy.Time.now().to_sec()
                    if rospy.Time.now().to_sec() - timestart < timeout: 
                        self.baud = baud
                        logger.logwarn("Baudrate detected {}".format(self.baud))
                        self.imu_msg_info = "Baudrate at {}".format(self.baud)
                        self.imu_puber_string_info.publish(self.imu_msg_info)
                        self.imu_msg_info = "Correct baudrate"
                        self.imu_puber_string_info.publish(self.imu_msg_info)
                        self.imu_msg_state= "CONNECTED"
                        self.imu_puber_string.publish(self.imu_msg_state)
                        self.state = "CONNECTED" 
                        logger.loginfo("CONNECTED")
                        

                        #publish current readings
                        for t in range(3):
                            imu_raw_data = p.read_until(b"\x55\x51")
                            if len(imu_raw_data) != 44:
                                logger.logwarn("Got {} data".format(len(imu_raw_data)))
                                rate_quick.sleep()
                                continue
                            frame_int = [int(x) for x in imu_raw_data]
                            frame_int = frame_int[-2:] + frame_int[:-2]
                            result = imu_feedback_parse(frame_int)
                            logger.loginfo("Got: \n {}".format(pp.pformat(result)))
                            self.imu_msg_data = ("Got: {}".format(pp.pformat(result)))
                        self.imu_puber.publish(str(self.imu_msg_data))
                        logger.loginfo("imu msg data is published")
                        rate_wait_for_refresh.sleep()
                    else:
                        logger.logwarn ("ERROR, not at correct baudrate")
                        self.imu_msg_info= "Wrong baudrate, press SCAN to set" #publish to info
                        self.imu_puber_string_info.publish(self.imu_msg_info)
                        self.imu_msg_state="IDLE"
                        self.imu_puber_string.publish(self.imu_msg_state) 


        
                                    
            #SCAN: Check baudrate and set to 115200
            elif self.button == "SCAN" and (self.state =="IDLE"):
                for baud in (115200,9600):
                    logger.loginfo("Checking baudrate {}".format(baud))
                    with Serial(port=self.port, baudrate=baud, timeout=1.0) as p:
                        timestart = rospy.Time.now().to_sec()
                        imu_raw_data = p.read_until(b"\x55\x51")
                        if rospy.Time.now().to_sec() - timestart < timeout: 
                            self.baud = baud
                            logger.logwarn("Baudrate detected {}".format(self.baud))
                            self.imu_msg_info = "Baudrate at {}".format(self.baud)
                            self.imu_puber_string_info.publish(self.imu_msg_info)
                            if self.baud == FIXED_BAUDRATE: 
                                self.imu_msg_state= "CONNECTED"
                                self.state = "CONNECTED"
                                self.imu_puber_string.publish(self.imu_msg_state) 
                                logger.loginfo("CONNECTED")
                            else:
                                p.write(WIT_IMU_CMD_UNLOCK)
                                p.write(WIT_IMU_CMD_UNLOCK)
                                p.write(WIT_IMU_CMD_UNLOCK) 
                                p.write(WIT_IMU_CMD_UNLOCK)
                                p.write(WIT_IMU_CMD_SET_BAUDRATE)
                                self.baud = 115200
                                p.write(WIT_IMU_CMD_SAVE_CFG)   
                                self.imu_msg_info="baudrate is set to {}".format(self.baud)
                                self.imu_puber_string_info.publish(self.imu_msg_info)
                                self.imu_msg_state= "CONNECTED"
                                self.imu_puber_string.publish(self.imu_msg_state)
                                self.state = "CONNECTED" 
                                logger.loginfo("CONNECTED")


            #SET: set desired readings from available readings given in dictionary
            elif self.button == "SET" and (self.state =="CONNECTED"):
                with Serial(port=self.port, baudrate=baud, timeout=1.0) as p:
                    p.write(WIT_IMU_CMD_UNLOCK)
                    p.write(WIT_IMU_CMD_UNLOCK)
                    p.write(WIT_IMU_CMD_UNLOCK) 
                    p.write(WIT_IMU_CMD_UNLOCK)
                    

                    logger.logwarn("Setting ACC scale to: 16g/s2!")
                    logger.loginfo(self.parameter1)
                    self.imu_msg_info= ("Setting ACC scale to: 16g/s2!")
                    self.imu_puber_string_info.publish(self.imu_msg_info)
                    p.write(ACC_SPEED[self.parameter1]) #WIT_IMU_CMD_SET_SCALE_ACC
                
                    logger.logwarn("Setting gyro scale to: 2000deg/s!")
                    self.imu_msg_info=("Setting gyro scale to: 2000deg/s!")
                    self.imu_puber_string_info.publish(self.imu_msg_info)
                    p.write(GYRO_DEGREES[self.parameter2]) #WIT_IMU_CMD_SET_SCALE_GYRO
        
                    logger.logwarn("Setting bandwidth to: 250Hz!")
                    self.imu_msg_info=("Setting bandwidth to: 250Hz!")
                    self.imu_puber_string_info.publish(self.imu_msg_info)
                    p.write(BANDWIDTH_HZ[self.parameter3]) #WIT_IMU_CMD_SET_BANDWIDTH
            
                    logger.logwarn("Setting feedback rate to: 200Hz!")
                    self.imu_msg_info=("Setting feedback rate to: 200Hz!")
                    self.imu_puber_string_info.publish(self.imu_msg_info)
                    p.write(SENDBACK_RATE_HZ[self.parameter4]) #WIT_IMU_CMD_SET_SENDBACK_RATE)

                    logger.logwarn(
                    "Setting feedback content as:\n0x51->acc 0x52->w 0x53->angle 0x59->quaternion"
                    )
                    self.imu_msg_info = (
                    "Setting feedback content as:\n0x51->acc 0x52->w 0x53->angle 0x59->quaternion"
                    )
                    p.write(WIT_IMU_CMD_SET_SENDBACK_CONTENT)
                    self.imu_puber_string_info.publish(self.imu_msg_info)
                   
                    
                    
                    #publish readings
                    for t in range(3):
                        imu_raw_data = p.read_until(b"\x55\x51")
                        if len(imu_raw_data) != 44:
                            logger.logwarn("Got {} data".format(len(imu_raw_data)))
                            rate_quick.sleep()
                            continue
                        frame_int = [int(x) for x in imu_raw_data]
                        frame_int = frame_int[-2:] + frame_int[:-2]
                        result = imu_feedback_parse(frame_int)
                        logger.loginfo("Got: \n {}".format(pp.pformat(result)))
                        self.imu_msg_data = ("Got: {}".format(pp.pformat(result)))
                    self.imu_puber.publish(str(self.imu_msg_data))
                    rate_wait_for_refresh.sleep()

                    self.button = None

                #SAVE
            elif (self.button =="SAVE") and (self.state == "CONNECTED"):
                        with Serial(port=self.port, baudrate=FIXED_BAUDRATE, timeout=1.0) as p:
                            p.write(WIT_IMU_CMD_SAVE_CFG)
                            logger.logwarn("CFG saved!")
                            self.imu_msg_info= ("CFG saved!")
                            self.imu_puber_string_info.publish(self.imu_msg_info)
                            self.button = None
  



                #CLOSE
            elif self.button == 'CLOSE' and self.state != "DISCONNECTED":
                logger.logwarn("Exiting")
                self.imu_msg_info=("Exiting")
                self.imu_puber_string_info.publish(self.imu_msg_info)
                self.imu_msg_state=("DISCONNECTED")
                self.imu_puber_string.publish(self.imu_msg_state)
                self.state == "DISCONNECTED"
                break

            rate.sleep()

                        
        
if __name__ == "__main__":
    rospy.init_node("imu_check")
    imu_checker = IMUCHECK()
    imu_checker.initialize()
    logger.loginfo("testinit")
    imu_checker.run()
    logger.loginfo("testrun")


    
