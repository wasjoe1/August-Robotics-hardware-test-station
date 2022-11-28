from __future__ import print_function, division
import socket
from boothbot_common.settings import(
    SERVOS_DRIVER_CONFIG,
    SERVO_PARAMETER
)
import numpy as np
import scipy.optimize as optimize
def get_host_ip():
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(('8.8.8.8', 80))
        ip = s.getsockname()[0]
    finally:
        s.close()

    return ip


def get_tolerance():
    gs_type = SERVO_PARAMETER["platform"].get('servo_types', ['stepper', 'stepper'])
    if gs_type == ['minas', 'minas']:
        return (1e-5, 5e-5)
        # PLATFORM_DEFAULT_TOLERANCE = SERVO_PARAMETER["platform"].get('tolerance', (1e-5, 5e-5))
    elif gs_type == ['minas', 'stepper']:
        return (1e-5, 0.001)
    elif gs_type == ['stepper', 'stepper']:
        return (0.0005, 0.001)
    else:
        return (1e-5, 5e-5)

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
    
def get_gs_type():
    gs_type = SERVO_PARAMETER["platform"].get('servo_types', ['stepper', 'stepper'])
    if gs_type == ['minas', 'minas']:
        return "minas"
        # PLATFORM_DEFAULT_TOLERANCE = SERVO_PARAMETER["platform"].get('tolerance', (1e-5, 5e-5))
    elif gs_type == ['minas', 'stepper']:
        return "stepper"
    else:
        return "default"

def get_max_encoder():
    if socket.gethostname().lower().startswith("gs"):
        return 8388608
    else:
        return 16384


