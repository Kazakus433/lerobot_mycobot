#!/usr/bin/env python

# Copyright 2025 The HuggingFace Inc. team. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import logging
from functools import cached_property
import numpy as np
import time
from lerobot.teleoperators.so100_leader.config_so100_leader import SO100LeaderConfig
from lerobot.teleoperators.so100_leader.so100_leader import SO100Leader

from pymycobot import MyArmC

from ..teleoperator import Teleoperator
from .config_mycobot_leader import MyCobotLeaderConfig

logger = logging.getLogger(__name__)


class MycobotLeader(Teleoperator):
    """
    [Bimanual SO-100 Leader Arms](https://github.com/TheRobotStudio/SO-ARM100) designed by TheRobotStudio
    This bimanual leader arm can also be easily adapted to use SO-101 leader arms, just replace the SO100Leader class with SO101Leader and SO100LeaderConfig with SO101LeaderConfig.
    """

    config_class = MyCobotLeaderConfig
    name = "mycobot_leader_config"

    def __init__(self, config: MyCobotLeaderConfig):
        super().__init__(config)
        self.config = config
        self.leader = None

        self._is_connected = False


    @cached_property
    def action_features(self) -> dict[str, type]:
        # 定义数据格式：6个关节 + 1个夹爪
        return {f"joint_{i}.pos": float for i in range(1, 7)} | {"gripper.pos": float}

    @cached_property
    def feedback_features(self) -> dict[str, type]:
        return {}

    @property
    def is_connected(self) -> bool:
        return self._is_connected

    # TODO
    def connect(self, calibrate: bool = True) -> None:
        self.leader = MyArmC(self.config.port, self.config.baudrate)
        self._is_connected = True

    @property
    def is_calibrated(self) -> bool:
        return True

    def calibrate(self) -> None:
        pass

    def configure(self) -> None:
        pass


    def setup_motors(self) -> None:
        pass

    def get_action(self) -> dict[str, float]:
        # 1. 获取关节角度列表 (度数)
        angle = self.leader.get_joints_angle()

        fact_angle = [0, 0, 0, 0, 0, 0]

        # 2. 硬件错位对调与零点补偿 (度数)
        fact_angle[0] = angle[0]
        fact_angle[1] = angle[1] - 100
        fact_angle[2] = angle[2] + 100
        fact_angle[4] = -angle[3] - 90
        fact_angle[5] = angle[5] -120
        if angle[4] < 90:
            fact_angle[3] = 90 - angle[4]
            fact_angle[3] = -90 - fact_angle[3]
        elif angle[4] > 90:
            fact_angle[3] = 90 - angle[4]
            fact_angle[3] = -90 + fact_angle[3]
        else:
            fact_angle[3] = -angle[4]

        # 3. 硬件安全限幅（在转为弧度前进行，因为你的 max/min 是度数）
        self.jointlimit(fact_angle)

        fact_angle[4] = -90

        # 4. 转换为弧度
        fact_angle_rad = np.deg2rad(fact_angle).tolist()

        # 5. 保留小数防抖（度数保留2位，弧度建议保留4位，物理精度等同）
        for i in range(len(fact_angle_rad)):
            fact_angle_rad[i] = round(fact_angle_rad[i], 4)

        # 6. 处理夹爪
        grip_value = float(-angle[6])
        if grip_value < 0:
            grip_value = 0.0
        if grip_value > 80:
            grip_value = 100.0

        # 【关键修改】归一化到 0.0 ~ 1.0，因为你的 send_action 里会再乘以 100
        grip_value_normalized = grip_value / 100.0
        time.sleep(0.2)

        # 7. 按照索引映射到字典中并返回
        action_dict = {
            "joint_1.pos": float(fact_angle_rad[0]),
            "joint_2.pos": float(fact_angle_rad[1]),
            "joint_3.pos": float(fact_angle_rad[2]),
            "joint_4.pos": float(fact_angle_rad[3]),
            "joint_5.pos": float(fact_angle_rad[4]),
            "joint_6.pos": float(fact_angle_rad[5]),
            "gripper.pos": grip_value_normalized
        }
        return action_dict

    def jointlimit(self, angles):
        max = [180.0, 90.0, 150.0, 80.0, 168.0, 175.0]
        min = [-180.0, -270, -150.0, -260.0, -168.0, -175.0]
        for i in range(6):
            if (angles[i] > max[i]):
                angles[i] = max[i]
            if (angles[i] < min[i]):
                angles[i] = min[i]


    # TODO
    def send_feedback(self, feedback: dict[str, float]) -> None:
        pass

    # TODO
    def disconnect(self) -> None:
        self._is_connected = False

