from rich import box
from rich.align import Align
from rich.console import RenderableType
from rich.panel import Panel
from rich.pretty import Pretty
from rich.text import Text

import rich.repr
from rich.styled import Styled
import numpy as np
import cv2

from device_states import DeviceStates
from textual.widgets import Placeholder

class DeviceModule(Placeholder):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.devices = []

    def append(self, device):
        self.devices.append(device)

    def get_color(self, device):
        color = "green"
        if device.state == DeviceStates.ON:
            color = "green"
        elif device.state == DeviceStates.OFF:
            color = "grey46"
        elif device.state == DeviceStates.OFFLINE or device.state == DeviceStates.ERROR:
            color = "red"
        return color

    def format_output(self, label, color, state):
        return f"{label:18s}: [{color}]{state}[/{color}]\n"

    def devices_text(self):
        str = ""
        for device in self.devices:
            color = self.get_color(device)
            str += self.format_output(device.label, color, device.show_text)
        return str

    def render(self) -> RenderableType:
        return Panel(
            Align.left(
                self.devices_text(), vertical="middle"
            ),
            title=self.name,
            border_style="green",
            box=box.ROUNDED,
            style=self.style,
            height=self.height,
        )

class Image(Placeholder):
    def __init__(self, image, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.image = image

    def _pix_to_rich(self, r, g, b) -> Text:
        return Text("\u2588", style=f"#{r:02x}{g:02x}{b:02x}")

    def _convert(self) -> Text:
        text = Text("")
        if self.image is None:
            return text
        # resize the image to fit the Panel
        # without distorting the aspect ratio
        # and without cropping the image

        width, height = self.size
        scale = min(float(height) / self.image.shape[0], float(width) / self.image.shape[1])

        dim = (int(self.image.shape[1] * scale *2 ), int(self.image.shape[0] * scale))
        image = cv2.resize(self.image, dim, interpolation=cv2.INTER_AREA)
        # convert the image to a Rich Text object
        for i in range(image.shape[0]):
            for j in range(image.shape[1]):
                b, g, r = image[i, j]
                text.append(self._pix_to_rich(r, g, b))
            text.append("\n")
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
        
    def update(self, image: np.ndarray) -> None:
        assert isinstance(image, np.ndarray)
        self.image = image
        self.refresh()


if __name__ == "__main__":
    cb = DeviceModule(name="Camera Beacon")

    led = Device("LED")
    h_motor = Device("Horizontal Motor")
    h_encoder = Device("Horizontal Encoder")
    v_motor = Device("Vertical Motor")
    v_encoder = Device("Vertical Encoder")
    l_camera = Device("Long Camera")
    s_camera = Device("Short Camera")

    cb.append(led)
    cb.append(h_motor)
    cb.append(h_encoder)
    cb.append(v_motor)
    cb.append(v_encoder)
    cb.append(l_camera)
    cb.append(s_camera)

    print(cb.devices_state())
