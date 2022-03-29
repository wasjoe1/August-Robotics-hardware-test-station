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
    def __init__(self, number, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.data = ["N/A"] * number

    @property
    def show_text(self):
        return f"{self.state.name:7s} {self.data}"

    def set_data(self, data, i):
        self.data[i] = data
    
class EStop(Device):
    @property
    def show_text(self):
        return "RELEASED" if self.state == DeviceStates.ON else "PRESSED" if self.state == DeviceStates.OFF else self.state.name

    def update(self, msg):
        if msg.dyn_power:
            self.state = DeviceStates.OFF if not msg.estop_off else DeviceStates.ON
        else:
            self.state = DeviceStates.UNKNOWN

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
        self.power = Device("<R> | Power")
        self.chassis = Device("<0> | Chassis")
        self.left_sonar = SonarStatus(number=2, name="Left Sonars")
        self.right_sonar = SonarStatus(number=2, name="Right Sonars")
        self.front_sonar = SonarStatus(number=3, name="Front Sonars")
        self.rear_sonar = SonarStatus(number=3, name="Rear Sonars")
        self.battery = Device("Battery")
        self.imu = Device("IMU")
        self.depth_camera = FreqChecker(DRIVERS_DEPTH_CAMERA_DEPTH_REGISTERED_POINTS, name="Depth Camera")
        self.lidar = FreqChecker(DRIVERS_LIDAR_SCAN_FILTERED, name="LIDAR")

        self.append(self.estop)
        self.append(self.power)
        self.append(self.chassis)
        self.append(self.left_sonar)
        self.append(self.right_sonar)
        self.append(self.front_sonar)
        self.append(self.rear_sonar)
        self.append(self.battery)
        self.append(self.imu)
        self.append(self.depth_camera)
        self.append(self.lidar)

        rospy.Subscriber(
            DRIVERS_CHASSIS_STATUS.name,
            DRIVERS_CHASSIS_STATUS.type,
            self._chassis_status_cb)

        rospy.Subscriber(
            DRIVERS_SONARS_STATUS.name,
            DRIVERS_SONARS_STATUS.type,
            self._sonars_status_cb)


        self.hz_checker = rostopic.ROSTopicHz(5)

        rospy.Subscriber(
            DRIVERS_DEPTH_CAMERA_DEPTH_REGISTERED_POINTS.name,
            DRIVERS_DEPTH_CAMERA_DEPTH_REGISTERED_POINTS.type,
            self.hz_checker.callback_hz,
            callback_args=DRIVERS_DEPTH_CAMERA_DEPTH_REGISTERED_POINTS.name)

        rospy.Subscriber(
            DRIVERS_LIDAR_SCAN_FILTERED.name,
            DRIVERS_LIDAR_SCAN_FILTERED.type,
            self.hz_checker.callback_hz,
            callback_args=DRIVERS_LIDAR_SCAN_FILTERED.name)

        self.set_rear_sonar = rospy.ServiceProxy(
            DRIVERS_SONARS_SET_REAR.name,
            DRIVERS_SONARS_SET_REAR.type)

        self.rear_sonar_enabled = False
        self.set_rear_sonar(self.rear_sonar_enabled)

    def on_mount(self):
        self.set_interval(5, self.depth_camera.update)
        self.set_interval(5, self.lidar.update)
        self.set_interval(1, self.refresh)

    def _sonars_status_cb(self, msg):
        text_array = ("OK", "ERR")
        action_array = [
            {"code":ErrCode.OA_ERR_SONAR_FRONT_01_DATA.value, "text":text_array, "dev":self.front_sonar, "index":0},
            {"code":ErrCode.OA_ERR_SONAR_FRONT_02_DATA.value, "text":text_array, "dev":self.front_sonar, "index":1},
            {"code":ErrCode.OA_ERR_SONAR_FRONT_03_DATA.value, "text":text_array, "dev":self.front_sonar, "index":2},
            {"code":ErrCode.OA_ERR_SONAR_REAR_01_DATA.value,  "text":text_array, "dev":self.rear_sonar,  "index":0},
            {"code":ErrCode.OA_ERR_SONAR_REAR_02_DATA.value,  "text":text_array, "dev":self.rear_sonar,  "index":1},
            {"code":ErrCode.OA_ERR_SONAR_REAR_03_DATA.value,  "text":text_array, "dev":self.rear_sonar,  "index":2},
            {"code":ErrCode.OA_ERR_SONAR_LEFT_01_DATA.value,  "text":text_array, "dev":self.left_sonar,  "index":0},
            {"code":ErrCode.OA_ERR_SONAR_LEFT_02_DATA.value,  "text":text_array, "dev":self.left_sonar,  "index":1},    
            {"code":ErrCode.OA_ERR_SONAR_RIGHT_01_DATA.value, "text":text_array, "dev":self.right_sonar, "index":0},
            {"code":ErrCode.OA_ERR_SONAR_RIGHT_02_DATA.value, "text":text_array, "dev":self.right_sonar, "index":1},
        ]
        front_sonar_errors = [
                ErrCode.OA_ERR_SONAR_FRONT_01_DATA.value,
                ErrCode.OA_ERR_SONAR_FRONT_02_DATA.value,
                ErrCode.OA_ERR_SONAR_FRONT_03_DATA.value
        ]
        rear_sonar_errors = [
                ErrCode.OA_ERR_SONAR_REAR_01_DATA.value,
                ErrCode.OA_ERR_SONAR_REAR_02_DATA.value,
                ErrCode.OA_ERR_SONAR_REAR_03_DATA.value
        ]
        left_sonar_errors = [
                ErrCode.OA_ERR_SONAR_LEFT_01_DATA.value,
                ErrCode.OA_ERR_SONAR_LEFT_02_DATA.value
        ]
        right_sonar_errors = [
                ErrCode.OA_ERR_SONAR_RIGHT_01_DATA.value,
                ErrCode.OA_ERR_SONAR_RIGHT_02_DATA.value
        ]

        for each in action_array:
            if each["code"] in msg.errorcodes:
                each["dev"].set_data(each["text"][1], each["index"])
            else:
                each["dev"].set_data(each["text"][0], each["index"])

        error_set = set(msg.errorcodes)

        if self.rear_sonar_enabled:
            self.front_sonar.state = DeviceStates.OFF
        else:
            self.front_sonar.state = DeviceStates.ON

        if self.rear_sonar_enabled:
            self.rear_sonar.state = DeviceStates.ON 
        else:
            self.rear_sonar.state = DeviceStates.OFF

        if error_set & set(left_sonar_errors):
            self.left_sonar.state = DeviceStates.ERROR
        else:
            self.left_sonar.state = DeviceStates.ON

        if error_set & set(right_sonar_errors):
            self.right_sonar.state = DeviceStates.ERROR
        else:
            self.right_sonar.state = DeviceStates.ON

    def _chassis_status_cb(self, msg):
        self.estop.update(msg)
        self.power.state = DeviceStates.ON if msg.dyn_power else DeviceStates.OFF
        self.chassis.state = DeviceStates.ON if msg.enabled else DeviceStates.OFF
        #self.left_sonar.state = DeviceStates.ON if msg.side_sonar else DeviceStates.OFF
        self.imu.state = DeviceStates.ON if msg.imu_online else DeviceStates.OFF

    def toggle_enable(self):
        if self.chassis.state == DeviceStates.ON:
            DRIVERS_CHASSIS_SRV_CMD.service_call(
                command='ENABLE',
                parameter='OFF'
            )
        else:
            DRIVERS_CHASSIS_SRV_CMD.service_call(
                command='ENABLE',
                parameter='ON'
            )

    def toggle_power(self):
        if self.power.state == DeviceStates.ON:
            self.power.state = DeviceStates.OFF
            DRIVERS_CHASSIS_SRV_CMD.service_call(
                command='POWER',
                parameter='OFF'
            )
        else:
            self.power.state = DeviceStates.ON
            DRIVERS_CHASSIS_SRV_CMD.service_call(
                command='POWER',
                parameter='ON'
            )

    def toggle_rear_sonar(self):
        result = self.set_rear_sonar(not self.rear_sonar_enabled)
        if result.success:
            self.rear_sonar_enabled = not self.rear_sonar_enabled



