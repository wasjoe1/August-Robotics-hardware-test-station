from enum import Enum
import time
from serial import Serial, SerialException
import boothbot_common.ros_logger as logger
from boothbot_driver.lrf_drivers.lrf_base import LaserRangeFinderBase

DEFAULT_SERIAL_CONFIG = {
    "port": "/dev/ldms_60",
    "timeout": 1.0,
}

LDMS_DISTANCE_RATIO = 1e-4
LDMS_COMMAND_HARD_DELAY = 0.02
LDMS_DEVICE_ADDRESS = "01"

LDMS_OPERATIONS_TABLE = {
    # Command: timeout_times
    "WS": 1,
    "GP": 1,
    "GT": 2,
    "OL": 3,
    "SL": 3,
    "CS": 2,
    "RM": 3,
    "SO": 1,
    "GO": 1,
    "S1": 14,
    "GH": 1,
    "HS": 1,
}

class LDMSMeasureMode(Enum):
    Unknown = "0"
    Default = "1"
    Fast = "2"
    Precise = "3"

class LDMSECInternal(Enum):
    Timeout = "200"
    ReturnNotValid = "201"
    OverLimit = "205"
    TemperatureHigh = "252"
    TemperatureLow = "253"
    LightWeak = "255"
    LightStrong = "256"
    ChecksumPC = "301"
    ChecksumSL = "302"
    HarewareError = "601"
    FirmwareError = "602"

class LDMS60Driver(LaserRangeFinderBase):
    def __init__(self):
        super(LDMS60Driver, self).__init__()
        self.port = None
        self.name = "ldms_60"
        self.serial_config = DEFAULT_SERIAL_CONFIG.copy()
        # self.serial_config.update(serial_config)
        self.runtime_data = {
            "mode": LDMSMeasureMode.Unknown,
        }

    def connect(self, timeout=None):
        try:
            self.port = Serial(**self.serial_config)
            self.port.close()
            self.port = Serial(**self.serial_config)
            if self.port.isOpen():
                if self.get_measure_mode() != LDMSMeasureMode.Unknown:
                    return True
        except SerialException as e:
            logger.logwarn("SerialException as: {}".format(e))
        return False

    def initialize(self):
        return self.set_measure_mode(LDMSMeasureMode.Default)

    def switch_laser(self, on=True):
        if on:
            cmd = "OL"
        else:
            cmd = "SL"
        ret = self._execute_command(cmd)
        if ret in LDMSECInternal:
            return False
        return True

    def get_distance(self):
        ret = self._execute_command("S1")
        if ret in LDMSECInternal:
            return False, 0.0
        # Laser measured exception occurred, so got blank result.
        if ret == "":
            return False, 0.0
        return True, float(ret[1:8]) * LDMS_DISTANCE_RATIO

    def get_distance_full(self):
        ret, distance = self.get_distance()
        return ret, distance, 0.0

    def get_measure_mode(self):
        ret = self._execute_command("RM")
        if ret not in LDMSECInternal:
            self.runtime_data["mode"] = LDMSMeasureMode(ret[1])
        else:
            self.runtime_data["mode"] = LDMSMeasureMode.Unknown
        return self.runtime_data["mode"]

    def set_measure_mode(self, mode=LDMSMeasureMode.Precise):
        ret = self._execute_command("CS", "+{}".format(mode.value))
        if ret not in LDMSECInternal:
            time.sleep(LDMS_COMMAND_HARD_DELAY)
            return self.get_measure_mode()

    def _execute_command(self, command, *args):
        command_str = ":{}{}{}".format(
            LDMS_DEVICE_ADDRESS, command, args[0] if args else ""
        )
        command_str = "{}{}\r\n".format(command_str, self.gen_checksum(command_str))
        ret = self._execute(command_str, LDMS_OPERATIONS_TABLE[command])
        if ret is None:
            return ""
        return ret

    def _execute(self, command_str, timeout_times, return_valid=True):
        self.port.reset_input_buffer()
        self.port.reset_output_buffer()
        logger.loginfo("Executing command_str: {}".format(command_str))
        self.port.write(command_str.encode())
        incom_val = None
        if return_valid:
            for _ in range(timeout_times):
                ret = self.port.read_until(expected="\r\n")
                _ret = ret.decode("utf-8")
                if _ret:
                    logger.loginfo("Received return value: {}".format(_ret))
                    if incom_val is not None:
                        logger.logwarn("add {} to old data {}".format(_ret, incom_val))
                        incom_val += _ret
                        return incom_val
                    if _ret.startswith(command_str[:5]):
                        ret_tail = _ret[5:]
                        # Detect laser whether received incomplete value.
                        measure_head = ":" + LDMS_DEVICE_ADDRESS + "S1"
                        if _ret[:5] == measure_head:
                            if len(ret_tail) != 12:
                                incom_val = ret_tail
                                logger.logwarn("Received incomplete value: {} , len: {} set value to {}".format(ret_tail, len(ret_tail), incom_val))
                                continue
                                # error_code = LDMSECInternal.ReturnNotValid
                                # return error_code
                            else:
                                logger.logwarn("Got full distance {}".format(ret_tail))
                                return ret_tail
                        if ret_tail[0] == "@" or len(ret_tail) == 0:
                            error_code = LDMSECInternal(ret_tail[1:4])
                            logger.logwarn("Failed due to: {}".format(error_code))
                            return error_code
                        return ret_tail
                    else:
                        error_code = LDMSECInternal.ReturnNotValid
                        logger.logwarn("Failed due to: {}".format(error_code))
                        return error_code
            logger.logwarn("Execution timeout!")
            return LDMSECInternal.Timeout
        logger.logwarn("Executed commmand but got None return.")
        return None

    @staticmethod
    def gen_checksum(cmd):
        cs = 0
        for s in cmd:
            cs += ord(s)
        checksum = hex(cs % 0x100).upper()
        if len(checksum) < 4:
            return "0{}".format(checksum[-1])
        return checksum[-2:]
