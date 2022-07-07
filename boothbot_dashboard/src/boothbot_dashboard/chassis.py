import rospy, rostopic
from device_module import DeviceModule
from device_states import DeviceStates
from device import Device

from boothbot_msgs.ros_interfaces import (
    DRIVERS_CHASSIS_SRV_CMD,
    DRIVERS_CHASSIS_STATUS,
    DRIVERS_SONARS_SET_REAR,
    DRIVERS_SONARS_STATUS,
    DRIVERS_DEPTH_CAMERA_DEPTH_REGISTERED_POINTS,
    DRIVERS_LIDAR_SCAN_FILTERED,
)

from boothbot_common.error_code import ErrCode

class SonarStatus(Device):
    def __init__(self, code, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.code = code
        self.data = ["N/A"] * len(code)

        rospy.Subscriber(
            DRIVERS_SONARS_STATUS.name,
            DRIVERS_SONARS_STATUS.type,
            self._cb)

    @property
    def show_text(self):
        if self.state == DeviceStates.OFF:
            return f"{self.state.name:7s}"
        else:
            return f"{self.state.name:7s} {self.data}"

    def _cb(self, msg):
        if self.state == DeviceStates.OFF:
            return

        error_set = set(msg.errorcodes)

        for i, each in enumerate(self.code):
            if each in error_set:
                self.data[i] = "ERR"
            else:
                self.data[i] = "OK"

        if "ERR" in self.data:
            self.state = DeviceStates.ERROR
        else:
            self.state = DeviceStates.ON


class EStop(Device):
    @property
    def show_text(self):
        return "RELEASED" if self.state == DeviceStates.ON else "PRESSED" if self.state == DeviceStates.OFF else self.state.name

    def update(self, msg):
        if msg.dyn_power:
            self.state = DeviceStates.OFF if not msg.estop_off else DeviceStates.ON
        else:
            self.state = DeviceStates.UNKNOWN


class Power(Device):
    @property
    def show_text(self):
        return "ON" if self.state == DeviceStates.ON else "OFF"

    def update(self, msg):
        self.state = DeviceStates.ON if msg.dyn_power else DeviceStates.OFF

    def toggle(self):
        if self.state == DeviceStates.ON:
            self.state = DeviceStates.OFF
            DRIVERS_CHASSIS_SRV_CMD.service_call(
                command='POWER',
                parameter='OFF'
            )
        else:
            self.state = DeviceStates.ON
            DRIVERS_CHASSIS_SRV_CMD.service_call(
                command='POWER',
                parameter='ON'
            )


class Wheels(Device):
    @property
    def show_text(self):
        return "ON" if self.state == DeviceStates.ON else "OFF"

    def update(self, msg):
        self.state = DeviceStates.ON if msg.enabled else DeviceStates.OFF

    def toggle(self):
        if self.state == DeviceStates.ON:
            DRIVERS_CHASSIS_SRV_CMD.service_call(
                command='ENABLE',
                parameter='OFF'
            )
        else:
            DRIVERS_CHASSIS_SRV_CMD.service_call(
                command='ENABLE',
                parameter='ON'
            )


class FreqChecker(Device):
    def __init__(self, topic, *args, **kwargs):
        super().__init__(*args, **kwargs)
        # window size to 5
        self.checker = rostopic.ROSTopicHz(5)
        self.topic = topic
        rospy.Subscriber(
            self.topic.name,
            self.topic.type,
            self.checker.callback_hz,
            callback_args=self.topic.name)

    def update(self):
        freq = self.checker.get_hz(self.topic.name)
        if freq is not None and freq[0] > 0:
            self.state = DeviceStates.ON
        else:
            self.state = DeviceStates.OFFLINE


class Chassis(DeviceModule):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

        # CHASSIS
        self.estop = EStop("E-Stop")
        self.power = Power("<R> | Power")
        self.wheels = Wheels("<0> | Wheels")

        left_sonar_error_codes = [
            ErrCode.OA_ERR_SONAR_LEFT_01_DATA.value,
            ErrCode.OA_ERR_SONAR_LEFT_02_DATA.value
        ]
        self.left_sonar = SonarStatus(left_sonar_error_codes, name="Left Sonars")

        right_sonar_error_codes = [
            ErrCode.OA_ERR_SONAR_RIGHT_01_DATA.value,
            ErrCode.OA_ERR_SONAR_RIGHT_02_DATA.value
        ]
        self.right_sonar = SonarStatus(right_sonar_error_codes, name="Right Sonars")

        front_sonar_error_codes = [
            ErrCode.OA_ERR_SONAR_FRONT_01_DATA.value,
            ErrCode.OA_ERR_SONAR_FRONT_02_DATA.value,
            ErrCode.OA_ERR_SONAR_FRONT_03_DATA.value
        ]
        self.front_sonar = SonarStatus(front_sonar_error_codes, name="Front Sonars")

        rear_sonar_error_codes = [
            ErrCode.OA_ERR_SONAR_REAR_01_DATA.value,
            ErrCode.OA_ERR_SONAR_REAR_02_DATA.value,
            ErrCode.OA_ERR_SONAR_REAR_03_DATA.value
        ]
        self.rear_sonar = SonarStatus(rear_sonar_error_codes, name="Rear Sonars")

        self.imu = Device("IMU")
        self.depth_camera = FreqChecker(DRIVERS_DEPTH_CAMERA_DEPTH_REGISTERED_POINTS, name="Depth Camera")
        self.lidar = FreqChecker(DRIVERS_LIDAR_SCAN_FILTERED, name="LIDAR")

        self.append(self.estop)
        self.append(self.power)
        self.append(self.wheels)
        self.append(self.left_sonar)
        self.append(self.right_sonar)
        self.append(self.front_sonar)
        self.append(self.rear_sonar)
        self.append(self.imu)
        self.append(self.depth_camera)
        self.append(self.lidar)

        rospy.Subscriber(
            DRIVERS_CHASSIS_STATUS.name,
            DRIVERS_CHASSIS_STATUS.type,
            self._chassis_status_cb)

        self.set_rear_sonar = rospy.ServiceProxy(
            DRIVERS_SONARS_SET_REAR.name,
            DRIVERS_SONARS_SET_REAR.type)

        self._rear_sonar_enabled = False
        self.rear_sonar.state = DeviceStates.OFF
        self.set_rear_sonar(self.rear_sonar_enabled)

    @property
    def rear_sonar_enabled(self):
        return self._rear_sonar_enabled

    @rear_sonar_enabled.setter
    def rear_sonar_enabled(self, value):
        self._rear_sonar_enabled = value

    def on_mount(self):
        self.set_interval(5, self.depth_camera.update)
        self.set_interval(5, self.lidar.update)
        self.set_interval(1, self.refresh)

    def _chassis_status_cb(self, msg):
        self.estop.update(msg)
        self.power.update(msg)
        self.wheels.update(msg)
        #self.left_sonar.state = DeviceStates.ON if msg.side_sonar else DeviceStates.OFF
        self.imu.state = DeviceStates.ON if msg.imu_online else DeviceStates.OFF

    def toggle_enable(self):
        self.wheels.toggle()

    def toggle_power(self):
        self.power.toggle()

    def toggle_rear_sonar(self):
        result = self.set_rear_sonar(not self.rear_sonar_enabled)
        if result.success:
            self.rear_sonar_enabled = not self.rear_sonar_enabled

        if self.rear_sonar_enabled:
            self.front_sonar.state = DeviceStates.OFF
            self.rear_sonar.state = DeviceStates.ON
        else:
            self.front_sonar.state = DeviceStates.ON
            self.rear_sonar.state = DeviceStates.OFF



