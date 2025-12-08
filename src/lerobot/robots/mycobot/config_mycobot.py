import time
import numpy as np
import serial
from dataclasses import dataclass, field
from typing import Dict, Any

from lerobot.cameras import ColorMode
# 引入 Lerobot 基础类
from lerobot.robots import Robot, RobotConfig
from lerobot.cameras.realsense.configuration_realsense import RealSenseCameraConfig

# 引入硬件驱动 (假设使用 pymycobot 作为底层驱动简化 TCP 通信)
# 如果你是手写 socket，请替换这里的 ElephantRobot 调用
from pymycobot import ElephantRobot


# =========================================================
# Step 1: 定义配置类 (Subclass the Robot Interface)
# =========================================================
@RobotConfig.register_subclass("mycobot_pro630")
@dataclass
class MyCobotPro630Config(RobotConfig):
    # 机械臂 TCP 配置
    arm_ip: str = "10.194.18.232"  # 你的机械臂 IP
    arm_port: int = 5001  # Pro 630 默认端口

    # 夹爪 串口 配置
    gripper_port: str = "com3"  # Linux 下通常是这个，Windows 下是 COMx
    gripper_baudrate: int = 115200

    start_position_angle = [
        0.0,  # Joint 1
        -140.0,  # Joint 2
        120.9,  # Joint 3
        -85.5,  # Joint 4
        -92.5,  # Joint 5
        -23.5,  # Joint 6
    ]

    # 默认相机配置
    cameras: dict[str, RealSenseCameraConfig] = field(
        default_factory=lambda: {
            "cam_1": RealSenseCameraConfig(
                serial_number_or_name="344422070499",  # 记得替换成你的真实序列号
                width=640,
                height=480,
                fps=30,
                use_depth=False,
                color_mode=ColorMode.RGB,
                warmup_s=2
            ),
            "cam_2": RealSenseCameraConfig(
                serial_number_or_name="327122078359",  # 记得替换成你的真实序列号
                width=640,
                height=480,
                fps=30,
                use_depth=False,
                color_mode=ColorMode.RGB,
                warmup_s=2
            )
        }
    )
