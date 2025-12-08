import time
import numpy as np
import serial
from dataclasses import dataclass, field
from typing import Dict, Any

from lerobot.robots import Robot, RobotConfig
from lerobot.cameras import CameraConfig, make_cameras_from_configs
# 假设你的配置文件名字是这个
from lerobot.robots.mycobot.config_mycobot import MyCobotPro630Config
from pymycobot import ElephantRobot


class MyCobotPro630(Robot):
    config_class = MyCobotPro630Config
    name = "mycobot_pro630"

    def __init__(self, config: MyCobotPro630Config):
        super().__init__(config)
        self.config = config

        self.arm = None
        self.gripper = None
        self._is_connected = False
        self.cameras = make_cameras_from_configs(config.cameras)

    @property
    def observation_features(self) -> dict:
        features = {f"joint_{i}.pos": float for i in range(1, 7)}
        features["gripper.pos"] = float
        for cam_name, cam in self.cameras.items():
            features[cam_name] = (cam.height, cam.width, 3)
        return features

    @property
    def action_features(self) -> dict:
        return {f"joint_{i}.pos": float for i in range(1, 7)} | {"gripper.pos": float}

    def connect(self):
        print(f"Connecting to Arm at {self.config.arm_ip}:{self.config.arm_port}...")
        try:
            self.arm = ElephantRobot(self.config.arm_ip, self.config.arm_port)
            self.arm.start_client()
            time.sleep(1)
            self.arm.start_robot()
            time.sleep(1)

            if self.arm.is_client_started:
                print("连接成功")
            else:
                raise ConnectionError("Failed to connect to mycobot")

            print(f"Connecting to Gripper...")
            self.gripper = serial.Serial(
                port=self.config.gripper_port,
                baudrate=self.config.gripper_baudrate,
                timeout=0.1
            )

            for cam in self.cameras.values():
                cam.connect()

            self._is_connected = True
            print("All hardware connected successfully.")

        except Exception as e:
            print(f"Connection failed: {e}")
            self.disconnect()
            raise e

    def disconnect(self):
        if self.arm:
            self.arm = None  # 根据 SDK 情况，可能有专门的 close 方法
        if self.gripper and self.gripper.is_open:
            self.gripper.close()
        for cam in self.cameras.values():
            cam.disconnect()
        self._is_connected = False

    @property
    def is_connected(self) -> bool:
        return self._is_connected

    @property
    def is_calibrated(self) -> bool:
        return True

    def calibrate(self):
        pass

    # =========================================================
    # 修改点 1: 读取时直接使用角度
    # =========================================================
    def get_observation(self) -> dict[str, Any]:
        if not self.is_connected:
            raise ConnectionError("Robot not connected")

        obs_dict = {}

        # --- A. 读取机械臂 ---
        # 这里的 angles_deg 直接就是 [0.0, 90.0, -90.0 ...] 这样的角度值
        angles_deg = self.arm.get_angles()

        if angles_deg is None or len(angles_deg) != 6:
            raise IOError("Failed to read angles from MyCobot Pro 630")

        # 【修改】：直接遍历 angles_deg，不进行 np.deg2rad 转换
        for i, val in enumerate(angles_deg):
            obs_dict[f"joint_{i + 1}.pos"] = float(val)

        # --- B. 读取夹爪 ---
        obs_dict["gripper.pos"] = 0.0

        # --- C. 读取相机 ---
        for cam_key, cam in self.cameras.items():
            obs_dict[cam_key] = cam.async_read()

        return obs_dict

    # =========================================================
    # 修改点 2: 写入时直接发送角度
    # =========================================================
    def send_action(self, action: dict[str, Any]) -> dict[str, Any]:
        if not self.is_connected:
            raise ConnectionError("Robot not connected")

        # --- A. 控制机械臂 ---
        # 1. 直接提取数据，假设 action 里存的已经是角度值
        target_deg = [action[f"joint_{i}.pos"] for i in range(1, 7)]

        # 2. 【修改】：不需要 np.rad2deg，直接发送列表
        speed = 1000
        self.arm.write_angles(target_deg, speed)

        # --- B. 控制夹爪 ---
        if "gripper.pos" in action:
            gripper_val = action["gripper.pos"]
            if gripper_val > 0.5:
                pass  # close
            else:
                pass  # open

        return action

    def reset(self):
        # 确保 start_position_angle 配置里填的也是角度值
        self.arm.write_angles(self.config.start_position_angle, 800)
        # 注意：原代码用了 set_joint，Pro 630 SDK 通常用 write_angles 或 write_coords
        print("回到初始位置")