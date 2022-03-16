import os
import netifaces
from device_module import DeviceModule
from device_states import DeviceStates
from device import Device

class Comm(DeviceModule):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

        # COMM
        self.dtu = Device("DTU")
        self.dtu_name = "/dev/dtu"
        self.router = Device("Network")
        # eth0 for Jetson Nano and raspberry pi
        # enp4s0 for i5 computer
        # enp2s0 for j1900 computer
        self.interfaces = ["eth0", "enp2s0", "enp4s0"]
        self.ip_address = ""

        self.append(self.dtu)
        self.append(self.router)

    def dtu_status(self):
        if os.path.exists("/dev/dtu"):
            self.dtu.state = DeviceStates.ON
            self.dtu.state = DeviceStates.OFF
        else:
            self.dtu.state = DeviceStates.OFFLINE

    def router_status(self):
        def is_interface_up(interface):
            addr = netifaces.ifaddresses(interface)
            return netifaces.AF_INET in addr

        def get_ip_address(interface):
            addr = netifaces.ifaddresses(interface)
            return addr[netifaces.AF_INET][0]["addr"]

        interfaces = netifaces.interfaces()
        for each in self.interfaces:
            if each in interfaces:
                if is_interface_up(each):
                    self.router.state = DeviceStates.ON
                    self.ip_address = get_ip_address(each)
                    return

        self.router.state = DeviceStates.OFFLINE
        self.ip_address = ""

    def devices_state(self):
        str = ""
        color = self.get_color(self.dtu)
        str += self.format_output(self.dtu.label, color, self.dtu.state.name)

        color = self.get_color(self.router)
        if self.router.state != DeviceStates.OFFLINE:
            str += self.format_output(self.router.label, color, self.ip_address)
        else:
            str += self.format_output(self.router.label, color, self.router.state.name)
        return str

    async def update(self):
        self.dtu_status()
        self.router_status()
        self.refresh()

    def on_mount(self):
        self.set_interval(1, self.update)
