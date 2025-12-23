import time
import numpy as np
import pygame
import threading
import queue
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
    gripper_controller = None
    step = 10

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
        self.action_interval = 0.05
        self.gripper_interval = 0.05
        self.global_speed = 3000
        self.rspeed = 1000
        self.gripper_speed = 100

        # 夹爪状态维护
        self.current_gripper_value = 50

        # --- 多线程相关 ---
        self.gripper_queue = queue.Queue()
        self.worker_running = False
        self.worker_thread = None

        # 【新增优化】关节角度缓存与读取线程
        self.reader_thread = None
        self.cached_angles = [0.0] * 6  # 用于存储最新的真实角度

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

    # --- 后台工作线程：夹爪写入 ---
    def _gripper_worker(self):
        """负责写入夹爪指令，防止阻塞主线程"""
        while self.worker_running:
            try:
                val, speed = self.gripper_queue.get(timeout=0.1)

                # 贪婪消费：只执行最新的一条指令
                last_val, last_speed = val, speed
                while not self.gripper_queue.empty():
                    try:
                        last_val, last_speed = self.gripper_queue.get_nowait()
                        self.gripper_queue.task_done()
                    except queue.Empty:
                        break

                if self.gripper_controller:
                    self.gripper_controller.set_gripper_value(last_val, last_speed)

                self.gripper_queue.task_done()

            except queue.Empty:
                continue
            except Exception as e:
                print(f"夹爪线程错误: {e}")

    # --- 【新增优化】后台工作线程：关节读取 ---
    def _joint_reader_worker(self):
        """
        负责不停地读取关节角度。
        get_angles() 是阻塞的，但因为它在独立线程里跑，
        所以不会卡住键盘控制的主循环。
        """
        while self.worker_running:
            try:
                if self.ec:
                    # 这里依然是耗时的，但无所谓，不影响主线程
                    angles = self.ec.get_angles()
                    if angles and len(angles) == 6:
                        self.cached_angles = angles

                # 稍微休眠一下，避免死循环占用过多CPU，同时也给串口喘息机会
                time.sleep(0.01)
            except Exception as e:
                # 忽略读取错误，保持上一次的值
                # print(f"读取角度错误: {e}")
                time.sleep(0.1)

    def connect(self, calibrate: bool = True):
        if self.is_connected:
            raise DeviceAlreadyConnectedError("Already connected.")

        pygame.init()

        self.KEY_MAP = {
            'move_x_forward': pygame.K_w, 'move_x_backward': pygame.K_s,
            'move_y_right': pygame.K_d, 'move_y_left': pygame.K_a,
            'move_z_up': pygame.K_q, 'move_z_down': pygame.K_e,
            'rotate_rx_positive': pygame.K_y, 'rotate_rx_negative': pygame.K_h,
            'rotate_ry_positive': pygame.K_t, 'rotate_ry_negative': pygame.K_u,
            'rotate_rz_positive': pygame.K_g, 'rotate_rz_negative': pygame.K_j,
            'gripper_open': pygame.K_r, 'gripper_close': pygame.K_f,
            'to_init': pygame.K_z, 'stop': pygame.K_x,
        }

        self.screen = pygame.display.set_mode((400, 300))
        pygame.display.set_caption("机械臂键盘控制 - 双线程极速版")

        font = pygame.font.SysFont(None, 24)
        text = font.render("Click window to focus.", True, (255, 255, 255))
        self.screen.blit(text, (20, 20))
        pygame.display.flip()

        self._is_connected = True
        self._is_calibrated = True
        print("🎮 Connected! Dual-threading enabled (Read+Write).")

        self.mycobot = MycobotPro630.get_instance()
        self.ec = self.mycobot.arm
        self.gripper_controller = self.mycobot.gripper

        # 初始同步
        try:
            init_val = self.gripper_controller.get_gripper_value()
            if init_val is not None: self.current_gripper_value = init_val

            init_angles = self.ec.get_angles()
            if init_angles: self.cached_angles = init_angles
        except:
            pass

        self.worker_running = True

        # 启动夹爪写入线程
        self.worker_thread = threading.Thread(target=self._gripper_worker, daemon=True)
        self.worker_thread.start()

        # 【新增优化】启动关节读取线程
        self.reader_thread = threading.Thread(target=self._joint_reader_worker, daemon=True)
        self.reader_thread.start()

    def disconnect(self):
        if self._is_connected:
            self.worker_running = False

            # 等待两个线程结束
            if self.worker_thread: self.worker_thread.join(timeout=1.0)
            if self.reader_thread: self.reader_thread.join(timeout=1.0)

            pygame.quit()
            self._is_connected = False

    def calibrate(self) -> None:
        pass

    def configure(self) -> None:
        pass

    def get_action(self) -> dict[str, Any]:
        if not self.is_connected:
            raise DeviceNotConnectedError("Not connected.")

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
                    try:
                        self.mycobot.configure()
                    except:
                        pass
                self.pressed_keys.add(key)
            elif event.type == pygame.KEYUP:
                key = event.key
                self.pressed_keys.discard(key)
                if key == self.KEY_MAP['gripper_open'] or key == self.KEY_MAP['gripper_close']:
                    self.last_action_time.pop(key, None)

                movement_keys = list(self.KEY_MAP.values())
                if key in movement_keys:
                    ignore = [self.KEY_MAP['gripper_open'], self.KEY_MAP['gripper_close'],
                              self.KEY_MAP['to_init'], self.KEY_MAP['stop']]
                    remaining = [k for k in self.pressed_keys if k not in ignore]
                    if not remaining:
                        try:
                            self.ec.task_stop()
                        except:
                            pass
                        self.last_action_time.clear()

        # =========================================================
        # 第二部分：执行动作 (夹爪)
        # =========================================================
        if self.KEY_MAP['gripper_open'] in self.pressed_keys:
            if self.KEY_MAP['gripper_open'] not in self.last_action_time or \
                    (current_time - self.last_action_time[self.KEY_MAP['gripper_open']]) >= self.gripper_interval:
                self.gripper_open()
                self.last_action_time[self.KEY_MAP['gripper_open']] = current_time

        elif self.KEY_MAP['gripper_close'] in self.pressed_keys:
            if self.KEY_MAP['gripper_close'] not in self.last_action_time or \
                    (current_time - self.last_action_time[self.KEY_MAP['gripper_close']]) >= self.gripper_interval:
                self.gripper_close()
                self.last_action_time[self.KEY_MAP['gripper_close']] = current_time

        # =========================================================
        # 第三部分：笛卡尔移动逻辑
        # =========================================================
        should_move = False
        active_key = None

        for key in self.pressed_keys:
            if key in [self.KEY_MAP['gripper_open'], self.KEY_MAP['gripper_close'], self.KEY_MAP['to_init'],
                       self.KEY_MAP['stop']]:
                continue
            if key not in self.last_action_time or (current_time - self.last_action_time[key]) >= self.action_interval:
                should_move = True
                active_key = key
                break

        if should_move and active_key:
            speed = self.global_speed
            rspeed = self.rspeed
            if active_key == self.KEY_MAP['move_x_forward']:
                self.ec.jog_coord('X', 1, speed)
            elif active_key == self.KEY_MAP['move_x_backward']:
                self.ec.jog_coord('X', -1, speed)
            elif active_key == self.KEY_MAP['move_y_right']:
                self.ec.jog_coord('Y', -1, speed)
            elif active_key == self.KEY_MAP['move_y_left']:
                self.ec.jog_coord('Y', 1, speed)
            elif active_key == self.KEY_MAP['move_z_up']:
                self.ec.jog_coord('Z', 1, speed)
            elif active_key == self.KEY_MAP['move_z_down']:
                self.ec.jog_coord('Z', -1, speed)
            elif active_key == self.KEY_MAP['rotate_rx_positive']:
                self.ec.jog_angle('J4', -1, rspeed)
                #self.ec.jog_coord('RX', 1, speed)
            elif active_key == self.KEY_MAP['rotate_rx_negative']:
                self.ec.jog_angle('J4', 1, rspeed)
                #self.ec.jog_coord('RX', -1, speed)
            elif active_key == self.KEY_MAP['rotate_ry_positive']:
                self.ec.jog_angle('J6', -1, rspeed)
                #self.ec.jog_coord('RY', -1, speed)
            elif active_key == self.KEY_MAP['rotate_ry_negative']:
                self.ec.jog_angle('J6', 1, rspeed)
                #self.ec.jog_coord('RY', 1, speed)
            elif active_key == self.KEY_MAP['rotate_rz_positive']:
                self.ec.jog_angle('J5', 1, rspeed)
                #self.ec.jog_coord('RZ', 1, speed)
            elif active_key == self.KEY_MAP['rotate_rz_negative']:
                self.ec.jog_angle('J5', -1, rspeed)
                #self.ec.jog_coord('RZ', -1, speed)
            self.last_action_time[active_key] = current_time

        # =========================================================
        # 第四部分：返回数据 (优化后)
        # =========================================================

        # 【优化】不再调用 self.ec.get_angles()
        # 而是直接使用 self.cached_angles（由后台线程更新）
        # 这样既保证了返回值是真实的（非0），又消除了主循环的阻塞

        angles_rad = np.deg2rad(self.cached_angles)

        return {
            "joint_1.pos": float(angles_rad[0]),
            "joint_2.pos": float(angles_rad[1]),
            "joint_3.pos": float(angles_rad[2]),
            "joint_4.pos": float(angles_rad[3]),
            "joint_5.pos": float(angles_rad[4]),
            "joint_6.pos": float(angles_rad[5]),
            "gripper.pos": float(self.current_gripper_value / 100.0),
        }

    def send_feedback(self, feedback: dict[str, Any]) -> None:
        return None

    def gripper_open(self):
        """打开夹爪"""
        if self.current_gripper_value < 100:
            self.current_gripper_value += self.step
            if self.current_gripper_value > 100: self.current_gripper_value = 100
            try:
                self.gripper_queue.put((self.current_gripper_value, self.gripper_speed))
            except Exception as e:
                print(f"夹爪指令发送失败: {e}")

    def gripper_close(self):
        """关闭夹爪"""
        if self.current_gripper_value > 0:
            self.current_gripper_value -= self.step
            if self.current_gripper_value < 0: self.current_gripper_value = 0
            try:
                self.gripper_queue.put((self.current_gripper_value, self.gripper_speed))
            except Exception as e:
                print(f"夹爪指令发送失败: {e}")