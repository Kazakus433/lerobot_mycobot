import time
import numpy as np
from pynput import keyboard
from dataclasses import dataclass

from lerobot.teleoperators.teleoperator import Teleoperator, TeleoperatorConfig
from lerobot.teleoperators.mycobot_keyboard.config_mycobot_keyboard import MyCobotKeyboardConfig

class MyCobotKeyboard(Teleoperator):
    config_class = MyCobotKeyboardConfig
    name = "mycobot_keyboard"

    def __init__(self, config: MyCobotKeyboardConfig):
        super().__init__(config)
        self.config = config

        # 内部维护的目标角度状态 (6个关节)
        # 如果 Config 里没给，默认为 0
        self.current_joints = np.array(config.init_joints if config.init_joints else [0.0] * 6, dtype=np.float32)
        self.gripper_state = 0.0  # 0=Open, 1=Close

        # 按键状态
        self.pressed_keys = set()
        self.listener = None

    def connect(self):
        # 启动键盘监听
        self.listener = keyboard.Listener(
            on_press=self._on_press,
            on_release=self._on_release
        )
        self.listener.start()
        print("⌨️ Keyboard Teleop connected. Use keys to move arm.")
        print("Joint 1: [1]/[Q] | Joint 2: [2]/[W] | Joint 3: [3]/[E]")
        print("Joint 4: [4]/[R] | Joint 5: [5]/[T] | Joint 6: [6]/[Y]")
        print("Gripper: [Space] (Toggle)")

    def disconnect(self):
        if self.listener:
            self.listener.stop()

    def _on_press(self, key):
        try:
            self.pressed_keys.add(key.char)
        except AttributeError:
            self.pressed_keys.add(key)

    def _on_release(self, key):
        try:
            k = key.char
        except AttributeError:
            k = key
        if k in self.pressed_keys:
            self.pressed_keys.remove(k)

        # 夹爪切换逻辑 (松开空格时切换)
        if key == keyboard.Key.space:
            self.gripper_state = 1.0 if self.gripper_state < 0.5 else 0.0
            state_str = "Closed" if self.gripper_state > 0.5 else "Open"
            print(f"Gripper: {state_str}")

    def get_action(self) -> dict:
        """
        LeRobot 主循环会高频调用此函数 (e.g. 30Hz)
        """
        step = self.config.sensitivity

        # --- 关节映射逻辑 (根据按键更新 self.current_joints) ---
        # Joint 1
        if '1' in self.pressed_keys: self.current_joints[0] += step
        if 'q' in self.pressed_keys: self.current_joints[0] -= step

        # Joint 2
        if '2' in self.pressed_keys: self.current_joints[1] += step
        if 'w' in self.pressed_keys: self.current_joints[1] -= step

        # Joint 3
        if '3' in self.pressed_keys: self.current_joints[2] += step
        if 'e' in self.pressed_keys: self.current_joints[2] -= step

        # Joint 4
        if '4' in self.pressed_keys: self.current_joints[3] += step
        if 'r' in self.pressed_keys: self.current_joints[3] -= step

        # Joint 5
        if '5' in self.pressed_keys: self.current_joints[4] += step
        if 't' in self.pressed_keys: self.current_joints[4] -= step

        # Joint 6
        if '6' in self.pressed_keys: self.current_joints[5] += step
        if 'y' in self.pressed_keys: self.current_joints[5] -= step

        # --- 这里的关键：返回的数据结构必须与 Robot 的 action_features 一致 ---
        action = {}
        for i in range(6):
            action[f"joint_{i + 1}.pos"] = self.current_joints[i]

        action["gripper.pos"] = self.gripper_state

        return action