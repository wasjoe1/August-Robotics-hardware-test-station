from __future__ import print_function, division
import socket
from boothbot_common.settings import(
    SERVOS_DRIVER_CONFIG,
    SERVO_PARAMETER
)
import numpy as np
import scipy.optimize as optimize
import cv2
import base64
from PIL import Image
from io import BytesIO

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


def get_gs_type():
    gs_type = SERVO_PARAMETER["platform"].get('servo_types', ['stepper', 'stepper'])
    if gs_type == ['minas', 'minas']:
        return "minas"
        # PLATFORM_DEFAULT_TOLERANCE = SERVO_PARAMETER["platform"].get('tolerance', (1e-5, 5e-5))
    elif gs_type == ['minas', 'stepper']:
        return "stepper"
    else:
        return "default"


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

# image handler for get image data from rostopic
def img2textfromcv2(frame, draw_line):
    # From BGR to RGB
    frame = cv2.resize(frame, None, fx=0.25, fy=0.25,
                        interpolation=cv2.INTER_LINEAR)
    if draw_line:
        width = frame.shape[0]
        height = frame.shape[1]
        cv2.line(frame,(0,int(width/2)),(int(height),int(width/2)),(0x00,0xA5,0xFF),2)
        cv2.line(frame,(int(height/2),0),(int(height/2),int(width)),(0x00,0xA5,0xFF),2)
    im = frame[:, :, ::-1]
    im = Image.fromarray(im)
    buf = BytesIO()
    im.save(buf, format="JPEG")
    im_binary = base64.b64encode(buf.getvalue())
    im_text = im_binary.decode()
    return im_text

def have_short_camera():
    try:
        # TOLERANCE = (0.0005, 0.001)
        from boothbot_perception.track.settings import HAVE_SHORT_CAMERA
        print("have short camera {}".format(HAVE_SHORT_CAMERA))
        return HAVE_SHORT_CAMERA
    except ImportError as e:
        print("import HAVE_SHORT_CAMERA error {}.".format(e))
        return True
    except:
        print("something error occured that we have not tested..")