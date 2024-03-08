#!/usr/bin/python
# -*- coding: utf-8 -*-

from enum import Enum

class CalibrationStates(Enum):
    ERROR = 0
    INIT = 1
    RESETING = 2
    IDLE = 3

    RUNNING = 4
