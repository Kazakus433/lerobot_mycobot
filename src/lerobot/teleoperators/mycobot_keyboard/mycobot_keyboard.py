import time
import numpy as np
import pygame
from typing import Any

from lerobot.teleoperators.teleoperator import Teleoperator
from lerobot.teleoperators.mycobot_keyboard.config_mycobot_keyboard import MyCobotKeyboardConfig
from lerobot.utils.errors import DeviceAlreadyConnectedError, DeviceNotConnectedError

# 仅在连接时引入 Robot 类，用于同步初始位置
from lerobot.robots.mycobot.mycobot_pro630 import MycobotPro630


class MyCobotKeyboard(Teleoperator):
    config_class = MyCobotKeyboardConfig
    name = "mycobot_keyboard"

    def __init__(self, config: MyCobotKeyboardConfig):
        super().__init__(config)
        self.config = config

        self._is_connected = False

        # --- 核心状态变量 ---
        # virtual_joint_positions: 当前计算出的目标位置（实时变动）
        # initial_joint_positions: 连接时的初始位置（固定不变，用于复位）
        self.virtual_joint_positions = None
        self.initial_joint_positions = None

        self.virtual_gripper_pos = 0  # 0.0(开) ~ 1.0(关)
        self.initial_gripper_pos = 0

        # --- 参数设置 ---
        # 手动控制灵敏度：每次按键调整的弧度值
        self.move_step_rad = 0.02
        self.gripper_step = 0.1

        # 【新增】自动复位参数
        self.is_resetting = False  # 是否正在复位中
        self.reset_step_rad = 0.05  # 复位时的自动移动速度（建议比手动稍快）

        # 关节限制 (弧度)，防止超出机械臂物理极限
        # 对应 J1 到 J6
        self.joint_limits = {
            'min': [-3.14, -3.14, -3.14, -3.14, -3.14, -3.14],
            'max': [3.14, 3.14, 3.14, 3.14, 3.14, 3.14]
        }

        # Pygame 相关
        self.pressed_keys = set()
        self.screen = None
        self.KEY_MAP = {}

    @property
    def action_features(self) -> dict:
        # 定义数据格式：6个关节 + 1个夹爪
        return {f"joint_{i}.pos": float for i in range(1, 7)} | {"gripper.pos": float}

    @property
    def feedback_features(self) -> dict:
        return {}

    @property
    def is_connected(self) -> bool:
        return self._is_connected

    def connect(self):
        if self.is_connected:
            raise DeviceAlreadyConnectedError("Already connected.")

        pygame.init()

        # --- 1. 设置按键映射 (关节控制模式) ---
        self.KEY_MAP = {
            # 关节 1 (底座) - A/D
            'j1_pos': pygame.K_a, 'j1_neg': pygame.K_d,
            # 关节 2 (大臂) - W/S
            'j2_pos': pygame.K_w, 'j2_neg': pygame.K_s,
            # 关节 3 (小臂) - E/Q
            'j3_pos': pygame.K_e, 'j3_neg': pygame.K_q,
            # 关节 4 (旋转) - H/Y
            'j4_pos': pygame.K_h, 'j4_neg': pygame.K_y,
            # 关节 5 (手腕) - T/U
            'j5_pos': pygame.K_t, 'j5_neg': pygame.K_u,
            # 关节 6 (末端) - J/G
            'j6_pos': pygame.K_j, 'j6_neg': pygame.K_g,
            # 夹爪 - F/R
            'gripper_close': pygame.K_f, 'gripper_open': pygame.K_r,
            # 功能键
            'to_init': pygame.K_z,  # 复位键
            'debug': pygame.K_p,  # 复位键
            'stop': pygame.K_ESCAPE,  # 退出键
        }

        # --- 2. 初始化 Pygame 窗口 ---
        self.screen = pygame.display.set_mode((400, 300))
        pygame.display.set_caption("MyCobot Keyboard (Smooth Reset Mode)")
        font = pygame.font.SysFont(None, 24)
        text = font.render("Controls: QWEASD... Z to Smooth Reset", True, (255, 255, 255))
        self.screen.blit(text, (20, 20))
        pygame.display.flip()

        # --- 3. 关键步骤：同步并保存初始状态 ---
        print("正在读取机械臂初始姿态...")
        try:
            # 获取实例读取真实角度
            robot_instance = MycobotPro630.get_instance()

            # 读取真实角度 (角度制 -> 弧度制)
            real_angles = robot_instance.arm.get_angles()
            if not real_angles:
                real_angles = [0.0] * 6
                print("⚠️ 警告: 无法读取初始角度，默认为 0")

            # 1. 初始化当前虚拟状态
            self.virtual_joint_positions = np.deg2rad(real_angles).tolist()

            # 2. 【核心】备份初始状态 (深拷贝)
            self.initial_joint_positions = list(self.virtual_joint_positions)

            # 读取夹爪
            g_val = robot_instance.gripper.get_gripper_value()
            current_g = (g_val / 100.0) if g_val is not None else 0.5

            self.virtual_gripper_pos = current_g
            self.initial_gripper_pos = current_g

            print(f"✅ 同步成功! 初始弧度: {np.round(self.virtual_joint_positions, 2)}")

        except Exception as e:
            print(f"❌ 读取错误 (使用默认零位): {e}")
            self.virtual_joint_positions = [0.0] * 6
            self.initial_joint_positions = [0.0] * 6
            self.virtual_gripper_pos = 0.5
            self.initial_gripper_pos = 0.5

        self._is_connected = True
        self.is_resetting = False

    def disconnect(self):
        if self._is_connected:
            pygame.quit()
            self._is_connected = False
            self.is_resetting = False
            print("Teleop disconnected.")

    def calibrate(self) -> None:
        pass

    def configure(self) -> None:
        pass

    def is_calibrated(self) -> bool:
        return True

    def get_action(self) -> dict[str, Any]:
        """
        计算下一帧的动作：
        - 纯数学计算，不读取硬件 IO
        - 支持 Z 键平滑复位 (避免数据突变)
        """
        if not self.is_connected:
            raise DeviceNotConnectedError("Not connected.")

        # ==========================
        # 1. Pygame 事件处理
        # ==========================
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                self.disconnect()
                return {}
            elif event.type == pygame.KEYDOWN:
                if event.key == self.KEY_MAP['stop']:
                    self.disconnect()
                    return {}

                # 按下 Z 键，开启归位模式，而不是直接赋值
                if event.key == self.KEY_MAP['to_init']:
                    self.is_resetting = True
                    print("🔄 开始平滑复位...")
                if event.key == self.KEY_MAP['debug']:
                    robot_instance = MycobotPro630.get_instance()
                    robot_instance.configure()
                    self.virtual_joint_positions = np.deg2rad([
                        0.0,  # Joint 1
                        -131.3,  # Joint 2
                        107.4,  # Joint 3
                        -117.7,  # Joint 4
                        -90.7,  # Joint 5
                        -125.6,  # Joint 6
                    ])
                    self.virtual_gripper_pos = 0

                self.pressed_keys.add(event.key)
            elif event.type == pygame.KEYUP:
                self.pressed_keys.discard(event.key)

        # ==========================
        # 2. 状态更新逻辑
        # ==========================

        # --- A. 平滑归位逻辑 ---
        if self.is_resetting:
            all_arrived = True  # 标记是否所有关节都到了

            # 关节平滑插值
            for i in range(6):
                target = self.initial_joint_positions[i]
                current = self.virtual_joint_positions[i]
                diff = target - current

                # 如果差距大于步长，就走一步
                if abs(diff) > self.reset_step_rad:
                    all_arrived = False
                    direction = 1.0 if diff > 0 else -1.0
                    self.virtual_joint_positions[i] += direction * self.reset_step_rad
                else:
                    # 差距很小，直接吸附
                    self.virtual_joint_positions[i] = target

            # 夹爪平滑插值
            g_target = self.initial_gripper_pos
            g_current = self.virtual_gripper_pos
            g_diff = g_target - g_current
            if abs(g_diff) > self.gripper_step:
                all_arrived = False
                g_dir = 1.0 if g_diff > 0 else -1.0
                self.virtual_gripper_pos += g_dir * self.gripper_step
            else:
                self.virtual_gripper_pos = g_target

            # 检查是否全部复位完成
            if all_arrived:
                self.is_resetting = False
                print("✅ 复位完成，可继续手动控制")

        # --- B. 手动控制逻辑 (仅在非复位状态下生效) ---
        else:
            # 关节 1
            if self.KEY_MAP['j1_pos'] in self.pressed_keys: self.virtual_joint_positions[0] += self.move_step_rad
            if self.KEY_MAP['j1_neg'] in self.pressed_keys: self.virtual_joint_positions[0] -= self.move_step_rad
            # 关节 2
            if self.KEY_MAP['j2_pos'] in self.pressed_keys: self.virtual_joint_positions[1] += self.move_step_rad
            if self.KEY_MAP['j2_neg'] in self.pressed_keys: self.virtual_joint_positions[1] -= self.move_step_rad
            # 关节 3
            if self.KEY_MAP['j3_pos'] in self.pressed_keys: self.virtual_joint_positions[2] += self.move_step_rad
            if self.KEY_MAP['j3_neg'] in self.pressed_keys: self.virtual_joint_positions[2] -= self.move_step_rad
            # 关节 4
            if self.KEY_MAP['j4_pos'] in self.pressed_keys: self.virtual_joint_positions[3] += self.move_step_rad
            if self.KEY_MAP['j4_neg'] in self.pressed_keys: self.virtual_joint_positions[3] -= self.move_step_rad
            # 关节 5
            if self.KEY_MAP['j5_pos'] in self.pressed_keys: self.virtual_joint_positions[4] += self.move_step_rad
            if self.KEY_MAP['j5_neg'] in self.pressed_keys: self.virtual_joint_positions[4] -= self.move_step_rad
            # 关节 6
            if self.KEY_MAP['j6_pos'] in self.pressed_keys: self.virtual_joint_positions[5] += self.move_step_rad
            if self.KEY_MAP['j6_neg'] in self.pressed_keys: self.virtual_joint_positions[5] -= self.move_step_rad

            # 夹爪
            if self.KEY_MAP['gripper_close'] in self.pressed_keys: self.virtual_gripper_pos -= self.gripper_step
            if self.KEY_MAP['gripper_open'] in self.pressed_keys: self.virtual_gripper_pos += self.gripper_step

        # ==========================
        # 3. 范围限制 (Safety Clip)
        # ==========================
        for i in range(6):
            self.virtual_joint_positions[i] = np.clip(
                self.virtual_joint_positions[i],
                self.joint_limits['min'][i],
                self.joint_limits['max'][i]
            )

        self.virtual_gripper_pos = np.clip(self.virtual_gripper_pos, 0.0, 1.0)

        # ==========================
        # 4. 返回 Action
        # ==========================
        return {
            "joint_1.pos": float(self.virtual_joint_positions[0]),
            "joint_2.pos": float(self.virtual_joint_positions[1]),
            "joint_3.pos": float(self.virtual_joint_positions[2]),
            "joint_4.pos": float(self.virtual_joint_positions[3]),
            "joint_5.pos": float(self.virtual_joint_positions[4]),
            "joint_6.pos": float(self.virtual_joint_positions[5]),
            "gripper.pos": float(self.virtual_gripper_pos),
        }

    def send_feedback(self, feedback: dict[str, Any]) -> None:
        pass