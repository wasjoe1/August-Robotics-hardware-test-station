import rospy, rostopic
from device_module import DeviceModule
from device_states import DeviceStates
from device import Device

from boothbot_msgs.ros_interfaces import (
    DRIVERS_CHASSIS_SRV_CMD,
    DRIVERS_CHASSIS_STATUS,
    DRIVERS_SONARS_SET_REAR,
    DRIVERS_SONARS_STATUS,
    DRIVERS_SONARS_F_01,
    DRIVERS_SONARS_F_02,
    DRIVERS_SONARS_F_03,
    DRIVERS_SONARS_R_01,
    DRIVERS_SONARS_R_02,
    DRIVERS_SONARS_R_03,
    DRIVERS_SONARS_LEFT_01,
    DRIVERS_SONARS_LEFT_02,
    DRIVERS_SONARS_RIGHT_01,
    DRIVERS_SONARS_RIGHT_02,
    DRIVERS_DEPTH_CAMERA_DEPTH_REGISTERED_POINTS,
    DRIVERS_LIDAR_SCAN_FILTERED,
)

from boothbot_common.error_code import ErrCode

# FIXME: remove this after finishing the sonar
from textual.widgets import Placeholder
from rich.text import Text
from rich.console import RenderableType
from rich.panel import Panel
from rich.align import Align
from rich import box


class SonarSensor:
    def __init__(self, name, msg_type):
        self.min = 0.0
        self.max = 0.0
        self.data = 0.0
        self.name = name

        rospy.Subscriber(
            msg_type.name,
            msg_type.type,
            self._sonar_data_cb)

    def _sonar_data_cb(self, msg):
        self.min = msg.min_range
        self.max = msg.max_range
        self.data = msg.range

class SonarData(Placeholder):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.f_01 = SonarSensor("f_01", DRIVERS_SONARS_F_01)
        self.f_02 = SonarSensor("f_02", DRIVERS_SONARS_F_02)
        self.f_03 = SonarSensor("f_03", DRIVERS_SONARS_F_03)
        self.r_01 = SonarSensor("r_01", DRIVERS_SONARS_R_01)
        self.r_02 = SonarSensor("r_02", DRIVERS_SONARS_R_02)
        self.r_03 = SonarSensor("r_03", DRIVERS_SONARS_R_03)
        self.left_01 = SonarSensor("left_01", DRIVERS_SONARS_LEFT_01)
        self.left_02 = SonarSensor("left_02", DRIVERS_SONARS_LEFT_02)
        self.right_01 = SonarSensor("right_01", DRIVERS_SONARS_RIGHT_01)
        self.right_02 = SonarSensor("right_02", DRIVERS_SONARS_RIGHT_02)

    def _convert(self) -> Text:
        text = Text("Sonar Data")
        return text

    def render(self) -> RenderableType:
        return Panel(
            Align.center(
                self._convert(), vertical="middle"
            ),
            title=self.name,
            border_style="blue",
            box=box.ROUNDED,
            style=self.style,
            height=self.height,
        )


class Sonars(Device):
    def __init__(self, number, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.data = ["N/A"] * number

    @property
    def show_text(self):
        return f"{self.state.name:7s} {self.data}"

    def set_data(self, data, i):
        self.data[i] = data
    

class Chassis(DeviceModule):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

        # CHASSIS
        self.estop = Device("E-Stop")
        self.power = Device("<R> | Power")
        self.chassis = Device("<0> | Chassis")
        self.left_sonar = Sonars(number=2, name="Left Sonars")
        self.right_sonar = Sonars(number=2, name="Right Sonars")
        self.front_sonar = Sonars(number=3, name="Front Sonars")
        self.rear_sonar = Sonars(number=3, name="Rear Sonars")
        self.battery = Device("Battery")
        self.imu = Device("IMU")
        self.depth_camera = Device("Depth Camera")
        self.lidar = Device("LIDAR")

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

    def hz_monitor(self):
        depth_camera_hz = self.hz_checker.get_hz(DRIVERS_DEPTH_CAMERA_DEPTH_REGISTERED_POINTS.name)
        lidar_hz = self.hz_checker.get_hz(DRIVERS_LIDAR_SCAN_FILTERED.name)
        if depth_camera_hz is not None:
            self.depth_camera.state = DeviceStates.ON if depth_camera_hz[0] > 0 else DeviceStates.OFFLINE
        else:
            self.depth_camera.state = DeviceStates.OFFLINE

        if lidar_hz is not None:
            self.lidar.state = DeviceStates.ON if lidar_hz[0] > 0 else DeviceStates.OFFLINE
        else:
            self.lidar.state = DeviceStates.OFFLINE



    def on_mount(self):
        self.set_interval(5, self.hz_monitor)

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

        if error_set & set(front_sonar_errors):
            self.front_sonar.state = DeviceStates.OFFLINE
        else:
            if self.rear_sonar_enabled:
                self.front_sonar.state = DeviceStates.OFF
            else:
                self.front_sonar.state = DeviceStates.ON

        if error_set & set(rear_sonar_errors):
            self.rear_sonar.state = DeviceStates.OFFLINE
        else:
            if self.rear_sonar_enabled:
                self.rear_sonar.state = DeviceStates.ON 
            else:
                self.rear_sonar.state = DeviceStates.OFF

        if error_set & set(left_sonar_errors):
            self.left_sonar.state = DeviceStates.OFFLINE
        else:
            self.left_sonar.state = DeviceStates.ON

        if error_set & set(right_sonar_errors):
            self.right_sonar.state = DeviceStates.OFFLINE
        else:
            self.right_sonar.state = DeviceStates.ON

        self.refresh()


    def _chassis_status_cb(self, msg):
        if msg.dyn_power:
            self.estop.state = DeviceStates.PRESSED if not msg.estop_off else DeviceStates.RELEASED
        else:
            self.estop.state = DeviceStates.UNKNOWN
        self.power.state = DeviceStates.ON if msg.dyn_power else DeviceStates.OFF
        self.chassis.state = DeviceStates.ON if msg.enabled else DeviceStates.OFF
        #self.left_sonar.state = DeviceStates.ON if msg.side_sonar else DeviceStates.OFF
        self.imu.state = DeviceStates.ON if msg.imu_online else DeviceStates.OFF
        self.refresh()

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



