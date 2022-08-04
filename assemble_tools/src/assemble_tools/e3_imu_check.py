#!/usr/bin/env python3

import time
import math
from pprint import PrettyPrinter
import rospy

logger = rospy

import binascii
from serial import Serial
from assemble_tools.get_key import GetKey

INSTRUCTION = """
Key functions:
i: Initialize connection to IMU, will detect baudrate
b: Set baudrate to 115200 to IMU
s: Set all settings to IMU
c: Check current readings
"""


NODE_NAME = "e3_imu_check"
NODE_RATE = 5.0
MOTOR_TOLERANCE = 0.0001

WIT_IMU_CMD_CALI_ACC = b"\xff\xaa\x01\x01\x00"
WIT_IMU_CMD_SAVE_CFG = b"\xff\xaa\x00\x00\x00"
WIT_IMU_CMD_UNLOCK = b"\xff\xaa\x69\x88\xb5"
WIT_IMU_CMD_SET_BAUDRATE = b"\xff\xaa\x04\x06\x00"  # 115200
# WIT_IMU_CMD_SET_SCALE_ACC = b"\xff\xaa\x21\x00\x00"  # 2g/s2
# WIT_IMU_CMD_SET_SCALE_ACC = b"\xff\xaa\x21\x01\x00"  #
# WIT_IMU_CMD_SET_SCALE_ACC = b"\xff\xaa\x21\x02\x00"  #
WIT_IMU_CMD_SET_SCALE_ACC = b"\xff\xaa\x21\x03\x00"  # 16g/s2
# WIT_IMU_CMD_SET_SCALE_GYRO = b"\xff\xaa\x20\x00\x00"  # 250deg/s
# WIT_IMU_CMD_SET_SCALE_GYRO = b"\xff\xaa\x20\x01\x00"  #
# WIT_IMU_CMD_SET_SCALE_GYRO = b"\xff\xaa\x20\x02\x00"  #
WIT_IMU_CMD_SET_SCALE_GYRO = b"\xff\xaa\x20\x03\x00"  # 2000deg/s
WIT_IMU_CMD_SET_BANDWIDTH = b"\xff\xaa\x1f\x00\x00"  # 250Hz
# WIT_IMU_CMD_SET_BANDWIDTH = b"\xff\xaa\x1f\x01\x00"  # 200Hz
# WIT_IMU_CMD_SET_BANDWIDTH = b"\xff\xaa\x1f\x02\x00"  # 98Hz
# WIT_IMU_CMD_SET_BANDWIDTH = b"\xff\xaa\x1f\x03\x00"  # 42Hz
# WIT_IMU_CMD_SET_BANDWIDTH = b"\xff\xaa\x1f\x04\x00"  # 20Hz
# WIT_IMU_CMD_SET_BANDWIDTH = b"\xff\xaa\x1f\x05\x00"  # 10Hz
# WIT_IMU_CMD_SET_BANDWIDTH = b"\xff\xaa\x1f\x06\x00"  # 5Hz
# WIT_IMU_CMD_SET_SENDBACK_RATE = b"\xff\xaa\x03\x01\x00"  # 0.1Hz
# WIT_IMU_CMD_SET_SENDBACK_RATE = b"\xff\xaa\x03\x02\x00"  # 0.5Hz
# WIT_IMU_CMD_SET_SENDBACK_RATE = b"\xff\xaa\x03\x03\x00"  # 1Hz
# WIT_IMU_CMD_SET_SENDBACK_RATE = b"\xff\xaa\x03\x04\x00"  # 2Hz
# WIT_IMU_CMD_SET_SENDBACK_RATE = b"\xff\xaa\x03\x05\x00"  # 5Hz
# WIT_IMU_CMD_SET_SENDBACK_RATE = b"\xff\xaa\x03\x06\x00"  # 10Hz
# WIT_IMU_CMD_SET_SENDBACK_RATE = b"\xff\xaa\x03\x07\x00"  # 20Hz
# WIT_IMU_CMD_SET_SENDBACK_RATE = b"\xff\xaa\x03\x08\x00"  # 50Hz
# WIT_IMU_CMD_SET_SENDBACK_RATE = b"\xff\xaa\x03\x09\x00"  # 100Hz
# WIT_IMU_CMD_SET_SENDBACK_RATE = b"\xff\xaa\x03\x0a\x00"  # 125Hz
WIT_IMU_CMD_SET_SENDBACK_RATE = b"\xff\xaa\x03\x0b\x00"  # 200Hz
# WIT_IMU_CMD_SET_SENDBACK_RATE = b"\xff\xaa\x03\x0c\x00"  # Once
# WIT_IMU_CMD_SET_SENDBACK_RATE = b"\xff\xaa\x03\x0d\x00"  # None

# enable 0x51->acc 0x52->w 0x53->angle 0x59->quaternion
WIT_IMU_CMD_SET_SENDBACK_CONTENT = b"\xff\xaa\x02\x0e\x02"
WIT_IMU_CMD_RESET_YAW = b"\xff\xaa\x01\x04\x00"

# convert 8bit hex to signed int
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


class E3IMUCheck(object):
    def __init__(self):
        super(E3IMUCheck, self).__init__()
        self.get_key = None
        self.port = "/dev/ttyUSB0"
        self.baud = None

    def initialize(self):
        self.get_key = GetKey()

    def run(self):
        logger.logwarn(INSTRUCTION)
        pp = PrettyPrinter(indent=2)
        l = rospy.Rate(NODE_RATE)
        while not rospy.is_shutdown():
            key = self.get_key.get_key()
            if key in ("i", "I"):
                logger.logwarn("Trying to initialize...")
                logger.loginfo("Detecting IMU...")
                # TODO: Check if port existed
                for baud in (115200, 9600):
                    timeout = 1.0
                    logger.loginfo("Check baudrate {}".format(baud))
                    with Serial(port=self.port, baudrate=baud, timeout=1.0) as p:
                        time_start = time.time()
                        _imu_raw_data = p.read_until(b"\x55\x51")
                        if time.time() - time_start < timeout:
                            self.baud = baud
                            logger.logwarn(
                                "Baudrate detected that a device is on {}".format(
                                    self.baud
                                )
                            )
                            break

            if key in ("b", "B"):
                if self.baud is None:
                    logger.logwarn("No baudrate is set, please check!")
                    continue
                with Serial(port=self.port, baudrate=self.baud, timeout=1.0) as p:
                    p.write(WIT_IMU_CMD_UNLOCK)
                    p.write(WIT_IMU_CMD_UNLOCK)
                    p.write(WIT_IMU_CMD_UNLOCK)
                    p.write(WIT_IMU_CMD_UNLOCK)
                    time.sleep(0.2)
                    p.write(WIT_IMU_CMD_SET_BAUDRATE)
                    time.sleep(0.2)
                    p.write(WIT_IMU_CMD_SAVE_CFG)
                    time.sleep(0.2)
                    self.baud = 115200
                    logger.loginfo("Baudrate is set to: {}".format(self.baud))
                with Serial(port=self.port, baudrate=self.baud, timeout=1.0) as p:
                    p.write(WIT_IMU_CMD_SAVE_CFG)
                    logger.logwarn("CFG saved!")

            if key in ("s", "S"):
                if self.baud is None:
                    logger.logwarn("No baudrate is set, please check!")
                    continue
                with Serial(port=self.port, baudrate=self.baud, timeout=1.0) as p:
                    p.write(WIT_IMU_CMD_UNLOCK)
                    p.write(WIT_IMU_CMD_UNLOCK)
                    p.write(WIT_IMU_CMD_UNLOCK)
                    p.write(WIT_IMU_CMD_UNLOCK)
                    time.sleep(0.2)
                    logger.logwarn("Setting ACC scale to: 16g/s2!")
                    p.write(WIT_IMU_CMD_SET_SCALE_ACC)
                    time.sleep(0.2)
                    logger.logwarn("Setting gyro scale to: 2000deg/s!")
                    p.write(WIT_IMU_CMD_SET_SCALE_GYRO)
                    time.sleep(0.2)
                    logger.logwarn("Setting bandwidth to: 250Hz!")
                    p.write(WIT_IMU_CMD_SET_BANDWIDTH)
                    time.sleep(0.2)
                    logger.logwarn("Setting feedback rate to: 200Hz!")
                    p.write(WIT_IMU_CMD_SET_SENDBACK_RATE)
                    time.sleep(0.2)
                    logger.logwarn(
                        "Setting feedback content as:\n0x51->acc 0x52->w 0x53->angle 0x59->quaternion"
                    )
                    p.write(WIT_IMU_CMD_SET_SENDBACK_CONTENT)
                    time.sleep(0.2)
                    p.write(WIT_IMU_CMD_SAVE_CFG)
                    time.sleep(0.2)
                    logger.logwarn("CFG saved!")

            if key in ("c", "C"):
                if self.baud is None:
                    logger.logwarn("No baudrate is set, please check!")
                    continue
                with Serial(port=self.port, baudrate=self.baud, timeout=1.0) as p:
                    for t in range(3):
                        _imu_raw_data = p.read_until(b"\x55\x51")
                        if len(_imu_raw_data) != 44:
                            # logger.logwarn("Got {} data".format(len(_imu_raw_data)))
                            time.sleep(0.2)
                            continue
                        # print(_imu_raw_data)
                        # frame_ = map(binascii.b2a_hex, _imu_raw_data)
                        # print([int(x) for x in _imu_raw_data])
                        frame_int = [int(x) for x in _imu_raw_data]
                        frame_int = frame_int[-2:] + frame_int[:-2]
                        result = imu_feedback_parse(frame_int)
                        logger.loginfo("Got: \n {}".format(pp.pformat(result)))
                        if result["quaternion"][0] != 0.0 and self.baud == 115200:
                            logger.logwarn("This IMU is OK!")
                        else:
                            logger.logerr("This IMU is not OK!")
                        break

            elif key in ("q", "Q"):
                logger.logwarn("Exiting...")
                self.get_key.end_get_key()
                break


if __name__ == "__main__":
    rospy.init_node(NODE_NAME, log_level=rospy.INFO)
    e3 = E3IMUCheck()
    e3.initialize()
    e3.run()
