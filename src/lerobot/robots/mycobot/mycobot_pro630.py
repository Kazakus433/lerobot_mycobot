import time
import numpy as np  # 确保导入 numpy
import serial
from dataclasses import dataclass, field
from typing import Dict, Any

from lerobot.robots import Robot, RobotConfig
from lerobot.cameras import CameraConfig, make_cameras_from_configs
from lerobot.robots.mycobot.elegripper import Gripper
from lerobot.robots.mycobot.config_mycobot import MycobotPro630Config
from pymycobot import ElephantRobot


class MycobotPro630(Robot):
    config_class = MycobotPro630Config
    name = "mycobot_pro630"

    # 让键盘操作去获取
    _instance = None

    def __init__(self, config: MycobotPro630Config):
        super().__init__(config)
        self.config = config

        self.arm = None
        self.gripper = None
        self._is_connected = False
        self.cameras = make_cameras_from_configs(config.cameras)

        MycobotPro630._instance = self

    @classmethod
    def get_instance(cls):
        return cls._instance

    # 指定观测的特征
    @property
    def observation_features(self) -> dict:
        features = {f"joint_{i}.pos": float for i in range(1, 7)}
        features["gripper.pos"] = float
        for cam_name, cam in self.cameras.items():
            features[cam_name] = (cam.height, cam.width, 3)
        return features

    # 指定动作的特征
    @property
    def action_features(self) -> dict:
        return {f"joint_{i}.pos": float for i in range(1, 7)} | {"gripper.pos": float}

    def connect(self, calibrate: bool = True):
        print(f"Connecting to Arm at {self.config.ip}:{self.config.port}...")
        try:
            self.arm = ElephantRobot(self.config.ip, self.config.port)
            self.arm.start_client()
            time.sleep(1)
            self.arm.start_robot()
            time.sleep(1)

            if self.arm.is_client_started:
                print("连接成功")
            else:
                raise ConnectionError("Failed to connect to mycobot")

            print(f"Connecting to Gripper...")
            self.gripper = Gripper(self.config.gripper_port, self.config.gripper_baudrate, id=14)

            for cam in self.cameras.values():
                print(cam)
                cam.connect()

            self._is_connected = True
            print("All hardware connected successfully.")

            if calibrate:
                self.configure()

        except Exception as e:
            print(f"Connection failed: {e}")
            self.disconnect()
            raise e

    def disconnect(self):
        if self.arm:
            self.arm = None
        if self.gripper:
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

    def configure(self) -> None:
        """Apply a minimal runtime configuration for the arm and gripper."""
        if not self.is_connected:
            raise ConnectionError("Robot not connected")

        # if self.config.start_position_angle and len(self.config.start_position_angle) == 6:
        print("Moving to configured start position...")
        # start_position_angle 通常也是角度制，直接发送即可
        self.arm.write_angles(self.config.start_position_angle, self.config.speed)
        self.gripper.set_gripper_value(0, 50)

    # =========================================================
    # 修改点 1: 读取硬件角度 -> 转为弧度 (Deg -> Rad)
    # =========================================================
    def get_observation(self) -> dict[str, Any]:
        if not self.is_connected:
            raise ConnectionError("Robot not connected")
        obs_dict = {}
        angles_deg = self.arm.get_angles()

        if angles_deg is None or len(angles_deg) != 6:
            print(f"⚠️ Warning: Bad data from robot: {angles_deg}")

            if hasattr(self, '_last_valid_angles'):
                angles_deg = self._last_valid_angles
            else:
                raise IOError("Failed to read valid angles from MyCobot Pro 630")
        else:
            self._last_valid_angles = angles_deg
        # 角度 -> 弧度 (np.deg2rad)
        angles_rad = np.deg2rad(angles_deg)
        for i, val in enumerate(angles_rad):
            obs_dict[f"joint_{i + 1}.pos"] = float(val)
        # --- B. 读取夹爪 ---
        obs_dict["gripper.pos"] = self.gripper.get_gripper_value() / 100.0
        # --- C. 读取相机 ---
        for cam_key, cam in self.cameras.items():
            obs_dict[cam_key] = cam.async_read()

        return obs_dict

    def send_action(self, action: dict[str, Any]) -> dict[str, Any]:
        if not self.is_connected:
            raise ConnectionError("Robot not connected")

        target_rad = [action[f"joint_{i}.pos"] for i in range(1, 7)]
        print(f'action:{action}')
        gripper_value = int((action["gripper.pos"].item() if hasattr(action["gripper.pos"], "item") else action["gripper.pos"]) * 100)
        if gripper_value < 0:
            gripper_value = 0
        if gripper_value > 100:
            gripper_value = 100
        target_deg = np.rad2deg(target_rad).tolist()

        # 3. 发送指令
        self.arm.write_angles(target_deg, self.config.speed)
        self.gripper.set_gripper_value(gripper_value, 100)


        return action