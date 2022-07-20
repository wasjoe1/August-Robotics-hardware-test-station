#!/usr/bin/env python
# -*- coding: utf-8 -*-
from __future__ import print_function, division

import time
import os
import subprocess
from pymodbus.client.sync import ModbusSerialClient as ModbusClient
from pymodbus.exceptions import ModbusIOException

# from common import Logging
from boothbot_common.ros_logger_wrap import ROSLogging as Logging

# local setting and constants
from boothbot_calibration_tools.settings import LEVEL_PORT

class DevDetect():
    def __init__(self, file):
        """__init__

        constructor function.
        this class judge if file modify.


        Parameters
        ----------
        file : str
            get last modify time of file.
        """
        self._file = file
        self._current_file_time = self.get_time(file)
        print("now init file time:", self._current_file_time)

    def get_time(self, file):
        """get_time

        get file last modify time.

        Parameters
        ----------
        file : str
            file path

        Returns
        -------
        str
            linux command "ls -la" show time.  
        """
        return subprocess.check_output("ls -la " + file + " | awk '{print $6,$7,$8}'", shell=True)[:-1]

    def file_change(self):
        """file_change

        return whether file change.

        Returns
        -------
        bool
            true for file changed. false for not changed.
        """
        print(self._current_file_time)
        print(self.get_time(self._file))
        if os.path.exists(self._file):
            if self.get_time(self._file) == self._current_file_time:
                print("file not change.")
                return False
            else:
                print("file change.")
                return True
        else:
            print("file not exist now.")
            return True


class ModbusDriver(Logging):
    _modbus_cfg = {}

    def __init__(self, name='mbd', method='rtu', **kwargs):
        """__init__

        constructor function.

        Parameters
        ----------
        name : str, optional
            name, by default 'mbd'
        method : str, optional
            modbus type, by default 'rtu'
        """
        super(ModbusDriver, self).__init__(name)

        self._modbus_cfg['method'] = method
        self._modbus_cfg['port'] = kwargs.get("port", LEVEL_PORT)
        self._modbus_cfg['baudrate'] = kwargs.get("baudrate", 9600)
        self._modbus_cfg['parity'] = 'N'
        self._modbus_cfg['timeout'] = kwargs.get("timeout", 0.5)

        i = 0

        self._client = []

        while True:
            if os.path.exists(LEVEL_PORT):
                self._client = ModbusClient(**self._modbus_cfg)
                self.__dd = DevDetect(LEVEL_PORT)

                # if os.path.exists(DEV_PATH):
                #     self.__dd = DevDetect(DEV_PATH)
                if self._client.connect():
                    self.loginfo("Modbus device is connected.")
                else:
                    self.logerr("Cannot connect to the Modbus device.")
                    # if connect fail, fsm will go into init loop.
                    # sleep 0.5 second.
                    time.sleep(0.01)
                # else:
                #     self.logerr("Cannot find device now.")
                #     # if dev lost, fsm will go into init loop.
                #     # sleep 1 second.
                #     time.sleep(1)
                break
            else:
                i += 1
                if i == 100:
                    break

    def is_dev_changed(self):
        """is_dev_changed

        whether file use by modbus is modify.

        Returns
        -------
        bool
            true for file changed. false for not changed.
        """
        return self.__dd.file_change()

    def close(self):
        """close

        close client of modbus.

        Returns
        -------
        bool
            false for exception ocurred.
        """
        try:
            self._client.close()
            return True
        except Exception:
            return False

    @property
    def client(self):
        return self._client
