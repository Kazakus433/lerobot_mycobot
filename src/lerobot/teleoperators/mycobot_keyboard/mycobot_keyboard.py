import time
import numpy as np
import pygame
from typing import Any
from lerobot.teleoperators.teleoperator import Teleoperator
from lerobot.teleoperators.mycobot_keyboard.config_mycobot_keyboard import MyCobotKeyboardConfig
from lerobot.utils.errors import DeviceAlreadyConnectedError, DeviceNotConnectedError

# 引入 Robot 类用于获取实例
from lerobot.robots.mycobot.mycobot_pro630 import MycobotPro630


class MyCobotKeyboard(Teleoperator):
    config_class = MyCobotKeyboardConfig
    name = "mycobot_keyboard"

    # 类变量初始化
    mycobot = None
    ec = None
    gripper_controller = None  # 新增：用于存储夹爪控制器实例
    step = 5
    current_gripper_value = 0

    def __init__(self, config: MyCobotKeyboardConfig):
        super().__init__(config)
        self.config = config

        self._is_connected = False
        self._is_calibrated = False

        # --- 移植旧代码的变量 ---
        self.pressed_keys = set()
        self.last_action_time = {}
        self.KEY_MAP = {}

        # 配置参数
        self.action_interval = 0.05  # 机械臂移动间隔
        self.gripper_interval = 0.1  # 夹爪间隔
        self.global_speed = 2000  # 机械臂速度
        self.gripper_speed = 100   # 夹爪速度 (通常不用太快)

        # 夹爪状态维护
        self.gripper_state = 0.0  # 0.0 ~ 1.0 (LeRobot 格式)

    @property
    def action_features(self) -> dict:
        return {f"joint_{i}.pos": float for i in range(1, 7)} | {"gripper.pos": float}

    @property
    def feedback_features(self) -> dict:
        return {}

    @property
    def is_connected(self) -> bool:
        return self._is_connected

    @property
    def is_calibrated(self) -> bool:
        return self._is_calibrated

    def connect(self, calibrate: bool = True):
        if self.is_connected:
            raise DeviceAlreadyConnectedError("Already connected.")

        # --- 1. 初始化 Pygame ---
        pygame.init()

        # 初始化映射
        self.KEY_MAP = {
            'move_x_forward': pygame.K_w,
            'move_x_backward': pygame.K_s,
            'move_y_right': pygame.K_d,
            'move_y_left': pygame.K_a,
            'move_z_up': pygame.K_q,
            'move_z_down': pygame.K_e,
            'rotate_rx_positive': pygame.K_y,
            'rotate_rx_negative': pygame.K_h,
            'rotate_ry_positive': pygame.K_t,
            'rotate_ry_negative': pygame.K_u,
            'rotate_rz_positive': pygame.K_g,
            'rotate_rz_negative': pygame.K_j,
            'gripper_open': pygame.K_r,
            'gripper_close': pygame.K_f,
            'to_init': pygame.K_z,
            'stop': pygame.K_x,
        }

        # 创建窗口
        self.screen = pygame.display.set_mode((400, 300))
        pygame.display.set_caption("机械臂键盘控制 - LeRobot版")

        font = pygame.font.SysFont(None, 24)
        text = font.render("Click window to focus.", True, (255, 255, 255))
        self.screen.blit(text, (20, 20))
        pygame.display.flip()

        self._is_connected = True
        self._is_calibrated = True
        print("🎮 Pygame Teleop Connected! Logic ported from Cartesian script.")
        self.mycobot = MycobotPro630.get_instance()
        self.ec = self.mycobot.arm
        self.gripper_controller = self.mycobot.gripper

    def disconnect(self):
        if self._is_connected:
            pygame.quit()
            self._is_connected = False

    def calibrate(self) -> None:
        pass

    def configure(self) -> None:
        pass

    def get_action(self) -> dict[str, Any]:
        if not self.is_connected:
            raise DeviceNotConnectedError("Not connected.")

        # 保存到 self 以便其他函数调用

        # 假设 Robot 类里初始化了 self.gripper (MycobotPro630 类中的属性)
        # 如果你的夹爪逻辑是写在 Robot 类里的，这里获取它
        #if hasattr(robot_instance, 'gripper'):
        #    self.gripper_controller = robot_instance.gripper
        #else:
            # 如果没有专门的 gripper 对象，可能是在 arm 对象里直接控制
        #    self.gripper_controller = self.ec

        current_time = time.time()

        # =========================================================
        # 第一部分：事件处理
        # =========================================================
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                self.disconnect()
                return {}

            elif event.type == pygame.KEYDOWN:
                key = event.key
                if key == pygame.K_ESCAPE:
                    self.disconnect()
                    return {}
                elif key == self.KEY_MAP['to_init']:
                    print("回到初始化位置")
                    self.mycobot.configure()

                    pass

                self.pressed_keys.add(key)

            elif event.type == pygame.KEYUP:
                key = event.key
                self.pressed_keys.discard(key)

                # 清理时间记录
                if key == self.KEY_MAP['gripper_open'] or key == self.KEY_MAP['gripper_close']:
                    self.last_action_time.pop(key, None)

                # --- 停止逻辑 ---
                movement_keys = [
                    self.KEY_MAP['move_x_forward'], self.KEY_MAP['move_x_backward'],
                    self.KEY_MAP['move_y_right'], self.KEY_MAP['move_y_left'],
                    self.KEY_MAP['move_z_up'], self.KEY_MAP['move_z_down'],
                    self.KEY_MAP['rotate_rx_positive'], self.KEY_MAP['rotate_rx_negative'],
                    self.KEY_MAP['rotate_ry_positive'], self.KEY_MAP['rotate_ry_negative'],
                    self.KEY_MAP['rotate_rz_positive'], self.KEY_MAP['rotate_rz_negative'],
                ]

                if key in movement_keys:
                    any_movement = any(k in self.pressed_keys for k in movement_keys)
                    if not any_movement:
                        try:
                            self.ec.task_stop()
                        except Exception as e:
                            print(f"Stop error: {e}")

                        for k in movement_keys:
                            self.last_action_time.pop(k, None)

        # =========================================================
        # 第二部分：执行动作 (夹爪部分)
        # =========================================================

        # 检查打开键
        if self.KEY_MAP['gripper_open'] in self.pressed_keys:
            # 恢复节流逻辑
            if self.KEY_MAP['gripper_open'] not in self.last_action_time or \
               (current_time - self.last_action_time[self.KEY_MAP['gripper_open']]) >= self.gripper_interval:

                self.gripper_open()
                self.last_action_time[self.KEY_MAP['gripper_open']] = current_time

        # 检查关闭键
        elif self.KEY_MAP['gripper_close'] in self.pressed_keys:
            # 恢复节流逻辑
            if self.KEY_MAP['gripper_close'] not in self.last_action_time or \
               (current_time - self.last_action_time[self.KEY_MAP['gripper_close']]) >= self.gripper_interval:
                self.gripper_close()
                self.last_action_time[self.KEY_MAP['gripper_close']] = current_time

        # 更新给 LeRobot 的状态 (0.0 - 1.0)
        #self.current_gripper_value = self.gripper_controller.get_gripper_value()
        #self.gripper_state = self.current_gripper_value / 100.0

        # =========================================================
        # 第三部分：笛卡尔移动逻辑
        # =========================================================

        should_move = False
        active_key = None

        for key in self.pressed_keys:
            if key == self.KEY_MAP['gripper_open'] or key == self.KEY_MAP['gripper_close']:
                continue
            if key not in self.last_action_time or (current_time - self.last_action_time[key]) >= self.action_interval:
                should_move = True
                active_key = key
                break

        if should_move and active_key:
            speed = self.global_speed

            if active_key == self.KEY_MAP['move_x_forward']:
                self.ec.jog_coord('X', 1, speed)
            elif active_key == self.KEY_MAP['move_x_backward']:
                self.ec.jog_coord('X', -1, speed)

            elif active_key == self.KEY_MAP['move_y_right']:  # 你的旧代码里 D 是 -Y
                self.ec.jog_coord('Y', -1, speed)
            elif active_key == self.KEY_MAP['move_y_left']:  # A 是 +Y
                self.ec.jog_coord('Y', 1, speed)

            elif active_key == self.KEY_MAP['move_z_up']:
                self.ec.jog_coord('Z', 1, speed)
            elif active_key == self.KEY_MAP['move_z_down']:
                self.ec.jog_coord('Z', -1, speed)

            elif active_key == self.KEY_MAP['rotate_rx_positive']:
                self.ec.jog_coord('RX', 1, speed)
            elif active_key == self.KEY_MAP['rotate_rx_negative']:
                self.ec.jog_coord('RX', -1, speed)

            elif active_key == self.KEY_MAP['rotate_ry_positive']:  # G 是 -RY (旧代码)
                self.ec.jog_coord('RY', -1, speed)
            elif active_key == self.KEY_MAP['rotate_ry_negative']:  # J 是 +RY
                self.ec.jog_coord('RY', 1, speed)

            elif active_key == self.KEY_MAP['rotate_rz_positive']:
                self.ec.jog_coord('RZ', 1, speed)
            elif active_key == self.KEY_MAP['rotate_rz_negative']:
                self.ec.jog_coord('RZ', -1, speed)

            # 更新时间戳
            self.last_action_time[active_key] = current_time

        # =========================================================
        # 第三部分：返回数据 (LeRobot 要求)
        # =========================================================

        # 读取当前状态，返回给 LeRobot 用于录制
        # 因为我们是直连控制，所以返回的就是 jog 后的实时位置
        try:
            angles = self.ec.get_angles()
            if not angles or len(angles) != 6:
                angles = [0.0] * 6
        except Exception:
            angles = [0.0] * 6

        return {
            "joint_1.pos": float(angles[0]),
            "joint_2.pos": float(angles[1]),
            "joint_3.pos": float(angles[2]),
            "joint_4.pos": float(angles[3]),
            "joint_5.pos": float(angles[4]),
            "joint_6.pos": float(angles[5]),
            "gripper.pos": float(self.gripper_controller.get_gripper_value() / 100.0),
        }

    def send_feedback(self, feedback: dict[str, Any]) -> None:
        return None

    def gripper_open(self):
        """打开夹爪 - 增加开度值"""
        self.current_gripper_value = self.gripper_controller.get_gripper_value()
        # 如果当前值还未达到最大值，则进行增加
        if self.current_gripper_value < 100:
            self.current_gripper_value += self.step

            # 边界检查：确保不超过最大值
            if self.current_gripper_value > 100:
                self.current_gripper_value = 100

            # print(f"执行夹爪打开，当前值: {current_gripper_value}") # 注释掉避免刷屏
            try:
                # 调用机械臂接口设定夹爪数值
                self.gripper_controller.set_gripper_value(self.current_gripper_value, self.gripper_speed)
            except Exception as e:
                print(f"夹爪指令发送失败: {e}")
        else:
            pass  # 到达最大值不打印，避免刷屏

    def gripper_close(self):
        """关闭夹爪 - 减小开度值"""
        self.current_gripper_value = self.gripper_controller.get_gripper_value()
        # 如果当前值大于0，则进行减小
        if self.current_gripper_value > 0:
            self.current_gripper_value -= self.step

            # 边界检查：确保不小于0
            if self.current_gripper_value < 0:
                self.current_gripper_value = 0

            # print(f"执行夹爪关闭，当前值: {current_gripper_value}") # 注释掉避免刷屏
            try:
                # 调用机械臂接口设定夹爪数值
                self.gripper_controller.set_gripper_value(self.current_gripper_value, self.gripper_speed)
            except Exception as e:
                print(f"夹爪指令发送失败: {e}")
        else:
            pass  # 到达最小值不打印