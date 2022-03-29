import os
import netifaces

from boothbot_dtu.dtu_config import get_config
from device_module import DeviceModule
from device_states import DeviceStates
from device import Device


class DTU(Device):
    def __init__(self, device_name, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.device = device_name
        self.config = {}
        self.error_message = ""

        self.update_status()

    @property
    def show_text(self):
        if self.state == DeviceStates.ERROR:
            return f"{self.error_message:10s}"
        if self.state == DeviceStates.ON:
            return f"{self.state.name:7s} channel:{self.config['channel']:3d}"
        else:
            return f"{self.state.name:7s}"

    def update_status(self):
        if os.path.exists(self.device):
            self.config, self.error_message = get_config()
            if self.config is not None:
                self.state = DeviceStates.ON
            else:
                self.state = DeviceStates.ERROR
        else:
            self.state = DeviceStates.OFFLINE

class Router(Device):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.interface = self.get_interface_name()
        self.ip_address = ""

    def get_interface_name(self):
        # eth0 for Jetson Nano and raspberry pi
        # enp4s0 for i5 computer
        # enp2s0 for j1900 computer
        interfaces = netifaces.interfaces()
        for each in ["eth0", "enp2s0", "enp4s0"]:
            if each in interfaces:
                return each
        return ""

    @property
    def show_text(self):
        return f"{self.ip_address}"

    def update_status(self):
        if self.interface != "":
            addr = netifaces.ifaddresses(self.interface)
            if netifaces.AF_INET in addr:
                self.ip_address = addr[netifaces.AF_INET][0]["addr"]
                self.state = DeviceStates.ON
            else:
                self.ip_address = ""
                self.state = DeviceStates.OFFLINE

class Comm(DeviceModule):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

        # COMM
        self.dtu = DTU("/dev/dtu", name="DTU")
        self.router = Router("Router")

        self.append(self.dtu)
        self.append(self.router)

    async def update(self):
        self.router.update_status()
        self.dtu.update_status()
        self.refresh()

    def on_mount(self):
        self.set_interval(5, self.update)
