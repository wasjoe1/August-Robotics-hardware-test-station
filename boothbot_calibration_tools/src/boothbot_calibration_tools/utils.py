import socket
from boothbot_common.settings import(
    SERVOS_DRIVER_CONFIG,
    SERVO_PARAMETER
)

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

