import time
import numpy as np
from pynput import keyboard
from dataclasses import dataclass, field

from lerobot.teleoperators.teleoperator import Teleoperator, TeleoperatorConfig


@TeleoperatorConfig.register_subclass("mycobot_leader")
@dataclass
class MyCobotLeaderConfig(TeleoperatorConfig):

    # TODO
    port: str = "COM3"
    baudrate: int = 1000000



