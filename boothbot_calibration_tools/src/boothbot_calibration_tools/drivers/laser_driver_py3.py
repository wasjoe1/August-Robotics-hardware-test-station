#!/usr/bin/env python3

import time
import math
import threading
import usb.core
import usb.util
from serial import Serial, SerialException
from boothbot_common.constants import FF32half
from boothbot_common.ros_logger_wrap import ROSLogging as Logging
from boothbot_perception.usb import USB
import struct

class LaserRangeFinder(Logging):
    def __init__(self, name):
        super(LaserRangeFinder, self).__init__(name)

    def connect(self, timeout=0.5):
        return True

    def close(self):
        return True

    def reset(self):
        return True

    def enable(self):
        if self.connect():
            if self.error:
                self.reset()
        return self.is_ready()

    def disable(self):
        return self.close()

    def laser_on(self, timeout=None):
        return True

    def laser_off(self):
        return True

    def get_distance(self):
        return True, self._fake_distance

    def get_distance_full(self):
        return True, self._fake_distance, self._fake_ver_angle

    def is_ready(self):
        return not self.error

    @property
    def error(self):
        return False

    @property
    def ready(self):
        return not self.error

    def set_fake_data(self, dis, angle):
        self._fake_distance = dis
        self._fake_ver_angle = angle

class LaserRangeFinderGenerator(object):
    def __init__(self):
        pass

    #to detect if swq laser is existed, return LaserRangeFinder if true
    @staticmethod
    def detect_swq_laser():
        swq_lrf = SWQ120LaserRangeFinder()
        if swq_lrf.connect():
            return swq_lrf
        return None

    #to detect if skd laser is existed, return LaserRangeFinder if true
    @staticmethod
    def detect_skd_laser():
        skd_lrf = SKD100DLaserRangeFinder()
        if skd_lrf.connect():
            return skd_lrf
        return None

    #return a LaserRangeFinder instance, first skd, if not swq, if not None
    @classmethod
    def detect_laser_range_finder(cls):
        current_lrf = cls.detect_swq_laser()
        if current_lrf:
            return current_lrf

        current_lrf = cls.detect_skd_laser()
        if current_lrf:
            return current_lrf

        return None


# def check_ready(func):
#     def wrapper(self, *args, **kwargs):
#         if self._status['READY'] is True:
#             return func(self, *args, **kwargs)
#         print('Device not ready!')
#         return False
#     return wrapper

# def check_lost(func):
#     def wrapper(self, *args, **kwargs):
#         if self._error is False:
#             return func(self, *args, **kwargs)
#         # print('Device not ready!')
#         return False
#     return wrapper

class SWQ120LaserRangeFinder(LaserRangeFinder):
    _dev = None
    _epin = None
    _epout = None
    _laser_timer = threading.Timer(1., lambda x: print('empty'))

    _error = True
    _is_on = False


    port = '/dev/swq120'
    def __init__(self):
        super(SWQ120LaserRangeFinder, self).__init__('SWQ120')

    def connect(self, timeout=0.5):
        if self._dev is not None:
            self.close()
        self._usb = USB(self.port)
        dev = usb.core.find(idVendor=0x0483, idProduct=0x5710)
        if dev is None:
            self.logerr("Laser module not found!")
            # raise AttributeError("Laser module not found!")
            # sys.exit("Device not found!")
            self._error = True
            return False

        try:
            if dev.is_kernel_driver_active(0) is True:
                dev.detach_kernel_driver(0)
        except usb.core.USBError as e:
            self.logerr("Kernel driver won't give up control over device: {}".format(e))
            # raise AttributeError("Kernel driver won't give up control over device: {}".format(e))
            # sys.exit("Kernel driver won't give up control over device: {}".format(e))
            self._error = True
            return False

        try:
            dev.set_configuration()
            dev.reset()
        except usb.core.USBError as e:
            self.logerr("Cannot set configuration the device: {}".format(e))
            # raise AttributeError("Cannot set configuration the device: {}".format(e))
            # sys.exit("Cannot set configuration the device: {}".format(e))
            self._error = True
            return False

        self._dev = dev
        self._epin = dev[0][(0, 0)][0]
        self._epout = dev[0][(0, 0)][1]
        self._error = False
        return True

    def close(self):
        if self._dev is None:
            return False
        self.laser_off()
        self._dev.finalize()
        self._dev = None
        self._epin = None
        self._epout = None
        return True

    def reset(self):
        # by adding if to reduce delay, doing reset port each time is time consuming
        if self.error:
            # Reset USB port
            self._usb.reset()
            # Wait 1.0s for hardware
            # time.sleep(1.0)
        self.connect()
        # close laser
        self.laser_off()
        # clean the waiting zone of laser
        cmd = 'ATD001#'
        self._get_response_of(cmd, 500)
        return self.is_ready()

    def _flushinput(self):
        try:
            while(1):
                _ = self._dev.read(self._epin.bEndpointAddress,
                                   self._epin.wMaxPacketSize,
                                   timeout=100)
        except usb.core.USBError as e:
            if e.errno == 110:
                return True
            elif e.errno == 19:
                self._error = True
            return False

    def _get_response_of(self, cmd, timeout=3000):
        try:
            self._epout.write(cmd)
            recv = self._dev.read(self._epin.bEndpointAddress,
                                  self._epin.wMaxPacketSize,
                                  timeout=timeout)
            if recv is not None:
                return recv.tostring()
        except usb.core.USBError as e:
            # e.errno == 110 is a timeout error!
            if e.errno == 110:
                pass
            elif e.errno == 19:
                self.logerr('Device lost!')
                self._error = True
            else:
                self.logerr('Unkown error: {}'.format(e))
            return None

    def _execute(self, cmd):
        self._flushinput()
        # if self._error:
        #     self.connect()
        return self._get_response_of(cmd)

    def laser_on(self, timeout=110.):
        # FIXME: currently the device will turn off the laser automatically after 120s
        timeout = 110. if timeout > 110. else timeout
        cmd = 'ATK001#'
        if self._is_on is False:
            data = self._execute(cmd)
            if data is None:
                return False
            self._is_on = True
            self._laser_timer = threading.Timer(timeout, self.laser_off)
            self._laser_timer.setDaemon(True)
            self._laser_timer.start()
        return self.is_ready()

    def laser_off(self):
        self.clear_cmd()
        self._is_on = False
        if self._laser_timer.isAlive() is True:
            self._laser_timer.cancel()
        return True

    def clear_cmd(self):
        cmd = 'ATK009#'
        return self._execute(cmd) is not None

    def _measure(self):
        cmd = 'ATK001#'
        if self._is_on is True:
            self._execute(cmd)
            if self._laser_timer.isAlive() is True:
                self._laser_timer.cancel()
            self._is_on = False
        else:
            for _ in range(2):
                self._execute(cmd)
        cmd = 'ATD001#'
        return self._execute(cmd)

    def get_distance(self):
        data = self._measure()
        if data is None:
            return False, 0.0
        # return True, float(int(data[3:7].decode(encoding='hex'), base=16))/1e4
        # print(int.from_bytes(data[3:7], "little"))
        return True, float(int.from_bytes(data[3:7], "big")/1e4)
        # return True, struct.unpack('f', data[3:7])

    # deprecated since the inclinometer inside swq is useless
    def get_distance_full(self):
        data = self._measure()
        if data is None:
            return False, 0.0, 0.0
        ang1 = int(data[7:11].decode(encoding='hex'), base=16)
        if ang1 > FF32half:
            ang1 = ang1 - (1<<32)
        ang2 = int(data[11:15].decode(encoding='hex'), base=16)
        if ang2 > FF32half:
            ang2 = ang2 - (1<<32)
        return (True,
                float(int(data[3:7].decode(encoding='hex'), base=16))/1e4,
                math.radians(float(ang1)/10))

    @property
    def error(self):
        return self._error

class SKD100DLaserRangeFinder(LaserRangeFinder):
    _PORT = '/dev/skd100d'
    _t_laser_off = None
    _t_laser_worker = None
    _t_laser_working = False
    _worker_period = 1./10.
    port = None
    _error = True

    def __init__(self):
        super(SKD100DLaserRangeFinder, self).__init__('skd100d')
        self.__laser_worker_lock = threading.Lock()
        self._last_measurement = (time.time(), 0.0)
        self.MEASUREMENT_TIME_TOLERANCE = self._worker_period
        self.GET_DISTANCE_TRY_TIMES = 50 # 50 for 50*self._worker_period == 5. seconds

    def reset(self):
        if self.connect():
            self.laser_off()
            return True
        return False

    def connect(self, timeout=0.5):
        if self.port is None:
            try:
                self.port = Serial(port=self._PORT, baudrate=9600, parity='N', timeout=0.002)
            except SerialException as e:
                self.logwarn(e)
                if e.errno == 2:
                    self._error = True
                    return False
        if self.port.isOpen():
            # send msg to skd and check feedback
            self._error = False
            return True
        return False

    def close(self):
        if self.port is not None:
            self.laser_off()
            self.port.close()
        return True

    def laser_on(self, timeout=110.):
        with self.__laser_worker_lock:
            self.__t_lw_end_time = time.time() + timeout
        if self._t_laser_worker is None or not self._t_laser_worker.is_alive():
            self._t_laser_worker = threading.Thread(target=self._laser_worker, name='laser_worker')
            self._t_laser_worker.start()

    def _laser_worker(self):
        self.port.flushInput()
        self.port.write('C')
        while True:
            time.sleep(self._worker_period)
            with self.__laser_worker_lock:
                if time.time() < self.__t_lw_end_time:
                    rall = self.port.readall().split('\xff')
                    if len(rall) >= 2 and len(rall[-1]) == 6:
                        try:
                            self._last_measurement = (time.time(), float(rall[-2])*1e-3)
                        except ValueError as e:
                            # self.logerr(e)
                            # self._last_measurement = (time.time(), 0.0)
                            pass
                    else:
                        self.port.write('C') # in case it's not open correctly
                else:
                    self.port.write('U')
                    return

    def laser_off(self):
        self.__t_lw_end_time = time.time() + 1. # remain some buffer in case multiple get_distance

    def get_distance(self):
        try:
            # just in case
            self.laser_on()
            for i in range(self.GET_DISTANCE_TRY_TIMES):
                if (time.time() - self._last_measurement[0] < self.MEASUREMENT_TIME_TOLERANCE
                    and self._last_measurement[1] != 0.0):
                    return True, self._last_measurement[1]
                time.sleep(self._worker_period)
            return False, 0.0
        finally:
            self.laser_off()

    def get_distance_full(self):
        res = self.get_distance()
        return res[0], res[1], 0.0

    @property
    def error(self):
        return self._error
