import time
import numpy as np
from pynput import keyboard
from dataclasses import dataclass, field

from lerobot.teleoperators.teleoperator import Teleoperator, TeleoperatorConfig


@TeleoperatorConfig.register_subclass("mycobot_keyboard")
@dataclass
class MyCobotKeyboardConfig(TeleoperatorConfig):
    # 每次按键增加/减少的角度（度）
    sensitivity: float = 2.0
    # 初始角度（必须与机械臂初始姿态一致，或者在 connect 时同步）
    init_joints: list[float] = field(
        default_factory=lambda: [
            0.0,  # Joint 1
            -131.3,  # Joint 2
            107.4,  # Joint 3
            -117.7,  # Joint 4
            -90.7,  # Joint 5
            -125.6,  # Joint 6
        ]
    #default_factory = lambda: [
    #    0.0,  # Joint 1
    #    -140.0,  # Joint 2
    #    120.9,  # Joint 3
    #    -85.5,  # Joint 4
    #    -92.5,  # Joint 5
    #    -23.5  # Joint 6
    #]
    )


