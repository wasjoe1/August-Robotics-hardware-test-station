import rospy

from textual.widgets import Placeholder
from rich.text import Text
from rich.console import RenderableType
from rich.panel import Panel
from rich.align import Align
from rich import box

from boothbot_msgs.ros_interfaces import (
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
)


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
        self.left_01  = SonarSensor("left_01", DRIVERS_SONARS_LEFT_01)
        self.left_02  = SonarSensor("left_02", DRIVERS_SONARS_LEFT_02)
        self.right_01 = SonarSensor("right_01", DRIVERS_SONARS_RIGHT_01)
        self.right_02 = SonarSensor("right_02", DRIVERS_SONARS_RIGHT_02)
        self._rear_sonar_enabled = False

    @property
    def rear_sonar_enabled(self):
        return self._rear_sonar_enabled

    @rear_sonar_enabled.setter
    def rear_sonar_enabled(self, value):
        self._rear_sonar_enabled = value
        self.refresh()

    def _convert(self):
        str = ""
        if self.rear_sonar_enabled:
            str += f"[grey46]Turned Off[/]\n\n\n\n"
        else:
            str += f"[yellow]{self.f_01.data:.2f}{' ':4s}{self.f_02.data:.2f}{' ':4s}{self.f_03.data:.2f}[/]\n\n\n\n"
        str += f"[green]{self.left_01.data:.2f}     \u25b2      {self.right_01.data:.2f}\n\n\n\n\n\n[/]"
        str += f"[green]{self.left_02.data:.2f}            {self.right_02.data:.2f}\n\n\n\n[/]"
        if self.rear_sonar_enabled:
            str += f"[green]{self.r_01.data:.2f}    {self.r_02.data:.2f}    {self.r_03.data:.2f}[/]"
        else:
            str += f"[grey46]Turned Off[/]"

        return str

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
    def on_mount(self):
        self.set_interval(0.2, self.refresh)
