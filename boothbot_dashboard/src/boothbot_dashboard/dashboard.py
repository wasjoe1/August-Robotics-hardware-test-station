#!/usr/bin/env python3.7
from textual.app import App

from boothbot_io import IO

from textual.widgets import Header, Footer
from marking import Marking, MarkingCamera
from chassis import Chassis, SonarData
from camera_beacon import CameraBeacon, TrackingCameras
from comm import Comm
import rospy

# ignore ResourceWarning from rospy
# https://stackoverflow.com/questions/26563711/disabling-python-3-2-resourcewarning
import warnings
warnings.simplefilter("ignore", ResourceWarning)

class HwTestApp(App):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

        self.io = IO()

        self.chassis = Chassis(name="Chassis")
        self.cb = CameraBeacon(self.io, name="Camera Beacon")
        self.marking = Marking(self.io, name="Marking")
        self.comm = Comm(name="Communication")
        self.sonar = SonarData(name="Sonar")
        self.tracking_cameras = TrackingCameras(name="Tracking Cameras")
        self.marking_cameras = MarkingCamera(name="Marking Camera")

    async def action_toggle_led(self):
        self.cb.toggle_led()

    async def action_cb_left(self):
        pass

    async def action_cb_right(self):
        pass

    async def action_cb_up(self):
        pass

    async def action_cb_down(self):
        pass

    async def action_toggle_k1(self):
        self.marking.toggle_k1()

    async def action_toggle_k2(self):
        self.marking.toggle_k2()

    async def action_toggle_k3(self):
        self.marking.toggle_k3()

    async def action_toggle_k4(self):
        self.marking.toggle_k4()

    async def action_toggle_long_actuator(self):
        self.marking.toggle_l_actuator()

    async def action_toggle_short_actuator(self):
        self.marking.toggle_stencil()

    async def action_toggle_in_pump(self):
        self.marking.toggle_in_pump()

    async def action_toggle_out_pump(self):
        self.marking.toggle_out_pump()

    async def action_toggle_brush(self):
        self.marking.toggle_brush()

    async def action_toggle_enable(self):
        self.chassis.toggle_enable()

    async def action_toggle_power(self):
        self.chassis.toggle_power()

    async def action_toggle_rear_sonar(self):
        self.chassis.toggle_rear_sonar()

    async def on_load(self):
        await self.bind("q", "quit", "Quit")
        await self.bind("l", "toggle_led", show=False)
        await self.bind("a", "cb_left", "CB\u2b6f")
        await self.bind("d", "cb_right", "CB\u2b6e")
        await self.bind("w", "cb_up", "CB\u2b67")
        await self.bind("s", "cb_down", "CB\u2b68")
        await self.bind("f", "toggle_rear_sonar", "Switch Front/Rear Sonar")
        await self.bind("1", "toggle_k1", show=False)
        await self.bind("2", "toggle_k2", show=False)
        await self.bind("3", "toggle_k3", show=False)
        await self.bind("4", "toggle_k4", show=False)
        await self.bind("5", "toggle_long_actuator", show=False)
        await self.bind("6", "toggle_short_actuator", show=False)
        await self.bind("7", "toggle_in_pump", show=False)
        await self.bind("8", "toggle_out_pump", show=False)
        await self.bind("9", "toggle_brush", show=False)
        await self.bind("0", "toggle_enable", show=False)

        await self.bind("r", "toggle_power", show=False)

    async def on_mount(self) -> None:
        """Make a simple grid arrangement."""

        grid = await self.view.dock_grid(edge="left", name="left")

        grid.add_column(fraction=1, name="left", min_size=55)
        grid.add_column(fraction=2, name="center", min_size=60)
        grid.add_column(fraction=2, name="right", min_size=60)

        grid.add_row(size=3, name="header")
        grid.add_row(fraction=len(self.cb.devices)+2, name="top", min_size=len(self.cb.devices)+2)
        grid.add_row(fraction=len(self.chassis.devices)+2, name="middle1", min_size=len(self.chassis.devices)+2)
        grid.add_row(fraction=len(self.marking.devices)+2, name="middle2", min_size=len(self.marking.devices)+2)
        grid.add_row(fraction=len(self.comm.devices)+2, name="bottom", min_size=len(self.comm.devices)+2)
        grid.add_row(size=1, name="footer")

        grid.add_areas(
            header = "left-start|right-end,header",
            footer = "left-start|right-end,footer",
            area_cb="left,top",
            area_chassis="left,middle1",
            area_marking="left,middle2",
            area_comm="left,bottom",
            area_sonar="right,top-start|middle1-end",
            area_tracking_cameras="center,top-start|bottom-end",
            area_marking_camera="right,middle2-start|bottom-end",
        )

        grid.place(
            header=Header(),
            footer=Footer(),
            area_cb=self.cb,
            area_chassis=self.chassis,
            area_marking=self.marking,
            area_comm=self.comm,
            area_sonar=self.sonar,
            area_tracking_cameras=self.tracking_cameras,
            area_marking_camera=self.marking_cameras,
        )


rospy.init_node('dashboard', log_level=rospy.INFO)
HwTestApp.run(title="Lionel Hardware Test", log="textual.log")
