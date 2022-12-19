#!/usr/bin/env python
# -*- coding: utf-8 -*-
from __future__ import print_function, division

import numpy as np
import scipy.optimize as optimize

def sincurve(x, offset=0, volume=1., shift=0.):
    return np.sin(x + offset) * volume + shift

def get_estimated_inclination(x_axis, y_axis, offset0=0., volume0=1., shift0=0.):
    # use the collected inclination with yaw data to curve_fit
    param, _ = optimize.curve_fit(sincurve, x_axis, y_axis, p0=[offset0, volume0, shift0])
    return param

def generate_yaw_array(N):
    # Generate from 0 to np.pi
    positive = np.linspace(0., np.pi, int(N/2) + 1).tolist()

    # np.pi to -np.pi
    to_minus_pi = np.linspace(np.pi, -np.pi, N)[1:-1].tolist()

    # -np.pi to 0
    negative = np.linspace(-np.pi, 0, int(N/2) + 1).tolist()

    return positive + to_minus_pi + negative
