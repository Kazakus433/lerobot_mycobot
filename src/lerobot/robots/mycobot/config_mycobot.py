
from dataclasses import dataclass, field

from lerobot.cameras import ColorMode
# 引入 Lerobot 基础类
from lerobot.robots import RobotConfig
from lerobot.cameras.realsense.configuration_realsense import RealSenseCameraConfig



# =========================================================
# Step 1: 定义配置类 (Subclass the Robot Interface)
# =========================================================
@RobotConfig.register_subclass("mycobot_pro630")
@dataclass
class MycobotPro630Config(RobotConfig):
    # 机械臂 TCP 配置
    type: str = "mycobot_pro630"
    ip: str = "10.194.21.62"  # 你的机械臂 IP
    port: int = 5001  # Pro 630 默认端口

    # 夹爪 串口 配置
    gripper_port: str = "com3"  # Linux 下通常是这个，Windows 下是 COMx
    gripper_baudrate: int = 115200

    start_position_angle = [
        156.5,  # Joint 1
        106.0,  # Joint 2
        121.5,  # Joint 3
        -59.0,  # Joint 4
        -8.5,  # Joint 5
        -6.2,  # Joint 6
    ]

    # 默认相机配置
    cameras: dict[str, RealSenseCameraConfig] = field(
        default_factory=lambda: {
            "camera1": RealSenseCameraConfig(
                serial_number_or_name="344422070499",  # 记得替换成你的真实序列号
                width=640,
                height=480,
                fps=30,
                use_depth=False,
                color_mode=ColorMode.RGB,
                warmup_s=2
            ),
            "camera2": RealSenseCameraConfig(
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
