#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from autoware_auto_control_msgs.msg import AckermannControlCommand
import math
import serial
import struct
import logging
import threading
import queue
import time
import sys
import os
import datetime
import evdev
from evdev import ecodes
from pynput import keyboard

ANG_THRESHLOD = 0.2 
SPEED_THRESHOLD = 0.1

class IntegratedControlPublisher(Node):

    MODE_AUTO = 'AUTO'  # Autoware控制模式
    MODE_MANUAL = 'MANUAL'  # 键盘控制模式
    MODE_WHEEL = 'WHEEL'  # 方向盘控制模式

    # 数据包类型常量
    TIME_SYNC_HEADER = 0x43
    ACK_HEADER = 0x44
    ACK_RECEIVED = 0x10
    ACK_SENT = 0x11
    SYNC_FAILURE_HEADER = 0x45
    MSG_TYPE_COMMAND = 0x01
    MSG_TYPE_NRF_INIT = 0x02  # 修正: nRF初始化命令应该是0x02，与Arduino端一致

    DECODING_STATUS_HEADER = 0x46
    # 方向盘控制设备配置
    WHEEL_DEVICE_NAME = "Logitech G29 Driving Force Racing Wheel"
    # 备选关键词，用于宽松匹配
    WHEEL_KEYWORDS = ["g29", "logitech", "wheel", "driving force"]

    def __init__(self):
        super().__init__('integrated_control_publisher')

        # 创建日志目录
        log_dir = "logs"
        if not os.path.exists(log_dir):
            os.makedirs(log_dir)
        
        # 使用当前日期和时间创建日志文件名
        timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        
        # 配置日志
        comm_log_filename = os.path.join(log_dir, f"comm_details_{timestamp}.log")
        file_handler = logging.FileHandler(comm_log_filename)
        file_handler.setLevel(logging.DEBUG)
        file_format = logging.Formatter('%(asctime)s - %(levelname)s - %(message)s')
        file_handler.setFormatter(file_format)
        
        console_handler = logging.StreamHandler()
        console_handler.setLevel(logging.INFO)
        console_handler.setFormatter(file_format)
        
        self.logger = logging.getLogger(__name__)
        self.logger.setLevel(logging.DEBUG)
        self.logger.addHandler(file_handler)
        self.logger.addHandler(console_handler)
        
        # 记录系统启动信息
        self.logger.info(f"=== 集成控制发布者启动于 {timestamp} ===")
        self.logger.info(f"通信详情日志文件: {comm_log_filename}")
        
        # 初始化时间戳CSV日志文件
        self.timing_log_path = os.path.join(log_dir, f"timing_log_{timestamp}.csv")
        with open(self.timing_log_path, 'w') as f:
            f.write("timestamp,event_type,pc_time,arduino_time,delay_ms,steering_angle,speed,mapped_steering,mapped_speed,reason\n")
        
        self.logger.info(f"时间戳CSV日志文件: {self.timing_log_path}")
        self.log_lock = threading.Lock()

        self.stop_flag = False
        
        # 声明并获取参数
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('timeout', 0.5)
        self.declare_parameter('sync_max_attempts', 3)
        self.declare_parameter('sync_timeout', 0.1) # 增加同步超时时间
        # 添加命令发送速率限制参数
        self.declare_parameter('min_command_interval_ms', 50)  # 默认最小间隔50ms (20Hz)
        # 添加日志级别参数
        self.declare_parameter('console_log_level', 'INFO')
        self.declare_parameter('file_log_level', 'DEBUG')
        self.declare_parameter('wheel_log_level', 'WARNING')  # 方向盘模式特定的日志级别

        # 添加车辆物理模型参数
        self.declare_parameter('vehicle_mass', 1000.0)        # 车辆质量(kg)
        self.declare_parameter('rolling_resistance', 0.015)    # 滚动阻力系数
        self.declare_parameter('air_drag_coeff', 0.3)          # 空气阻力系数
        self.declare_parameter('frontal_area', 2.0)            # 前向投影面积(m^2)
        self.declare_parameter('max_acceleration', 2.5)        # 最大加速度(m/s^2)
        self.declare_parameter('max_deceleration', 5.0)        # 最大减速度(m/s^2)
        self.declare_parameter('idle_deceleration', 0.2)       # 怠速减速度(m/s^2)
        self.declare_parameter('simulation_step', 0.04)        # 物理模拟步长(秒)

        port = self.get_parameter('port').get_parameter_value().string_value
        baudrate = self.get_parameter('baudrate').get_parameter_value().integer_value
        timeout = self.get_parameter('timeout').get_parameter_value().double_value
        self.sync_max_attempts = self.get_parameter('sync_max_attempts').get_parameter_value().integer_value
        self.sync_timeout = self.get_parameter('sync_timeout').get_parameter_value().double_value
        # 获取最小命令间隔并转换为秒
        self.min_command_interval = self.get_parameter('min_command_interval_ms').get_parameter_value().integer_value / 1000.0
        
        # 获取日志级别并应用
        console_log_level = self.get_parameter('console_log_level').get_parameter_value().string_value
        file_log_level = self.get_parameter('file_log_level').get_parameter_value().string_value
        self.wheel_log_level = self.get_parameter('wheel_log_level').get_parameter_value().string_value
        
        # 获取车辆物理参数
        self.vehicle_mass = self.get_parameter('vehicle_mass').get_parameter_value().double_value
        self.rolling_resistance = self.get_parameter('rolling_resistance').get_parameter_value().double_value
        self.air_drag_coeff = self.get_parameter('air_drag_coeff').get_parameter_value().double_value
        self.frontal_area = self.get_parameter('frontal_area').get_parameter_value().double_value
        self.max_acceleration = self.get_parameter('max_acceleration').get_parameter_value().double_value
        self.max_deceleration = self.get_parameter('max_deceleration').get_parameter_value().double_value
        self.idle_deceleration = self.get_parameter('idle_deceleration').get_parameter_value().double_value
        self.simulation_step = self.get_parameter('simulation_step').get_parameter_value().double_value
        
        # 用于车辆物理模型的额外变量
        self.current_velocity = 0.0
        self.target_velocity = 0.0
        self.last_physics_update = time.monotonic()
        self.physics_lock = threading.RLock()
        
        # 设置日志级别
        console_handler.setLevel(getattr(logging, console_log_level))
        file_handler.setLevel(getattr(logging, file_log_level))
        
        # 记录参数信息
        self.logger.info(f"参数配置: 端口={port}, 波特率={baudrate}, 超时={timeout}秒")
        self.logger.info(f"同步配置: 最大尝试次数={self.sync_max_attempts}, 超时={self.sync_timeout}秒")
        self.logger.info(f"命令发送速率限制: 最小间隔={self.min_command_interval*1000}ms")
        self.logger.info(f"日志级别: 控制台={console_log_level}, 文件={file_log_level}, 方向盘模式={self.wheel_log_level}")
        
        # 记录车辆物理模型参数
        self.logger.info(f"车辆物理模型参数: 质量={self.vehicle_mass}kg, 滚动阻力系数={self.rolling_resistance}")
        self.logger.info(f"空气阻力系数={self.air_drag_coeff}, 前向面积={self.frontal_area}m^2")
        self.logger.info(f"最大加速度={self.max_acceleration}m/s^2, 最大减速度={self.max_deceleration}m/s^2")
        self.logger.info(f"怠速减速度={self.idle_deceleration}m/s^2, 物理步长={self.simulation_step}s")
        
        # 初始化变量
        self.pre_steering_tire_angle = 0.0  # 初始化为0，避免None引起的问题
        self.pre_speed = 0.0               # 初始化为0，避免None引起的问题
        self.data_queue = queue.Queue()
        self.stop_event = threading.Event()
        self.cmd_lock = threading.Lock()
        self.steering_angle_deg = 0.0
        self.speed = 0.0
        self.mode = self.MODE_AUTO
        self.mode_lock = threading.RLock()
        self.last_wheel_status_log = 0  # 用于限制方向盘状态日志
        
        # 时间戳相关变量
        self.time_base = int(time.time() * 1000)
        self.time_sync_lock = threading.Lock()
        self.time_synced = False
        self.pc_arduino_offset = 0
        self.last_sync_time = 0
        self.command_timestamps = {}
        self.sync_attempts = 0
        self.sync_failed = False
        self.sync_failure_count = 0
        self.max_sync_failures = 3

        # 存储最近发送的控制命令
        self.last_command_store = {}
        self.last_command_lock = threading.Lock()

        # 记录上次发送的命令
        self.last_sent_commands = {
            'steering_angle_deg': None,
            'speed': None
        }
        # 添加最后命令发送时间，用于速率限制
        self.last_command_send_time = 0

        # 方向盘控制相关变量
        self.wheel_device = None
        self.wheel_thread = None
        self.wheel_lock = threading.Lock()
        self.wheel_values = {
            'wheel': 33000,  # 初始值在中间位置
            'Accelerater': 255,  # 初始值为未踩踏
            'Brake': 255,  # 初始值为未踩踏
            'Gear_Forward': 0,  # 初始状态
            'Gear_Push_Back': 0,  # 初始状态
        }
        
        # 用于跟踪方向盘值的变化，仅当值有变化时才记录日志
        self.last_wheel_values = self.wheel_values.copy()
        
        self.gear_direction = 1  # 1 表示前进, -1 表示后退
        
        # 喇叭状态变量
        self.trumpet_active = False
        self.trumpet_timer = None

        # 初始化键盘监听
        self.key_state = set()

        # R2按钮动态识别
        self.r2_button_codes = [294]  # 默认值
        self.r2_button_detected = False
        
        # 初始化串口
        try:
            self.ser = serial.Serial(port, baudrate, timeout=timeout)
            if self.ser.is_open:
                self.logger.info(f"成功打开串口: {port}")
            else:
                self.logger.error(f"无法打开串口: {port}")
                raise Exception(f"无法打开串口: {port}")
        except Exception as e:
            self.logger.error(f"打开串口时发生错误: {e}")
            # rclpy.shutdown() # 如果串口至关重要，考虑节点是否应该退出
            sys.exit(1)      # 如果串口至关重要则退出

        time.sleep(2)  # 等待串口稳定

        # 定义设备事件到映射函数的字典
        self.EVENT_CODE_TO_KEY = {
            ecodes.EV_ABS: {
                0: {"name": "wheel", "func": self.map_wheel},       # X轴，通常是方向盘转向
                1: {"name": "Accelerater", "func": self.map_accelerator},  # Y轴，可能是油门
                2: {"name": "Brake", "func": self.map_brake},       # Z轴，可能是刹车
                3: {"name": "wheel_alt", "func": self.map_wheel},   # RX轴，备选方向盘
                4: {"name": "Accelerater_alt", "func": self.map_accelerator}, # RY轴，备选油门
                5: {"name": "Brake_alt", "func": self.map_brake},   # RZ轴，备选刹车
            },
            ecodes.EV_KEY: {
                # 用evtest验证G29的这些按钮代码
                ecodes.BTN_TR: {"name": "Right_pick", "meaning": "右转向"}, # 例如: BTN_TR (R1/RB)
                ecodes.BTN_TL: {"name": "Left_pick", "meaning": "左转向"},  # 例如: BTN_TL (L1/LB)
                # 添加多个可能的ENTER按钮代码
                ecodes.BTN_SELECT: {"name": "ENTER", "meaning": "喇叭/蜂鸣器", "func": self.handle_enter_button}, # 可能是BTN_SELECT (299)
                704: {"name": "ENTER2", "meaning": "喇叭/蜂鸣器备选", "func": self.handle_enter_button}, # PS按钮也可以作为喇叭备选
                711: {"name": "ENTER3", "meaning": "喇叭/蜂鸣器备选", "func": self.handle_enter_button}, # 添加更多可能的ENTER按钮代码
                294: {"name": "R2", "meaning": "TO_WHEEL转方向盘", "func": self.switch_to_wheel_mode}, # 常见G29 R2代码
                298: {"name": "R3", "meaning": "TO_AUTO转自动", "func": self.switch_to_auto_mode}, # 常见G29 R3代码
                302: {"name": "Gear_Forward", "meaning": "档位前推", "func": self.set_reverse_gear}, # 换挡paddle
                303: {"name": "Gear_Push_Back", "meaning": "档位后推", "func": self.set_forward_gear}, # 换挡paddle
            }
        }

        # 初始化nRF通信
        if self.init_nrf_communication():
            self.logger.info("nRF通信初始化成功")
        else:
            self.logger.warning("nRF通信初始化失败，但将继续运行")

        self.logger.info("当前模式: AUTO")

        # 启动线程
        self.read_thread = threading.Thread(target=self.read_from_serial, daemon=True)
        self.read_thread.start()
        
        self.process_thread = threading.Thread(target=self.process_serial_data, daemon=True)
        self.process_thread.start()

        # 启动键盘监听线程
        self.keyboard_thread = threading.Thread(target=self.keyboard_input_listener, daemon=True)
        self.keyboard_thread.start()

        # 创建ROS订阅者
        self.subscription = self.create_subscription(
            AckermannControlCommand,
            '/control/command/control_cmd',
            self.listener_callback,
            1)

        # 启动时间同步线程
        self.sync_time_thread = threading.Thread(target=self.periodic_time_sync, daemon=True)
        self.sync_time_thread.start()

        # 初始化方向盘设备
        self.init_wheel_device()

        # 启动方向盘状态检查线程
        self.wheel_status_thread = threading.Thread(target=self.check_wheel_status, daemon=True)
        self.wheel_status_thread.start()

        # 启动R2按钮识别线程
        self.r2_detect_thread = threading.Thread(target=self.r2_button_detection, daemon=True)
        self.r2_detect_thread.start()

        # 发送初始停止命令
        self.logger.info("发送初始停止命令")
        self.send_stop_command()
        
        # 主循环线程在main函数中启动
    
    def calculate_resistance(self, velocity):
        """计算车辆行驶阻力 (牛顿)"""
        # 滚动阻力 + 空气阻力
        v_abs = abs(velocity)
        v_sign = 1.0 if velocity > 0 else -1.0 if velocity < 0 else 0.0
        
        # 计算滚动阻力 (正比于车重，与速度方向相反)
        rolling_force = self.vehicle_mass * 9.8 * self.rolling_resistance * v_sign if v_abs > 0.01 else 0.0
        
        # 计算空气阻力 (正比于速度平方，与速度方向相反)
        air_density = 1.225  # kg/m^3 at sea level
        air_force = 0.5 * air_density * self.frontal_area * self.air_drag_coeff * v_abs * velocity
        
        total_resistance = rolling_force + air_force
        return total_resistance

    def update_vehicle_physics(self, target_speed, brake_force):
        """更新车辆物理状态"""
        with self.physics_lock:
            current_time = time.monotonic()
            dt = min(current_time - self.last_physics_update, 0.1)  # 限制最大步长为100ms
            if dt < 0.001:  # 时间步长太小，跳过更新
                return self.current_velocity
                
            self.last_physics_update = current_time
            
            # 获取当前速度和方向
            current_velocity = self.current_velocity
            v_sign = 1.0 if current_velocity > 0 else -1.0 if current_velocity < 0 else 0.0
            
            # 计算阻力并将其转换为加速度
            resistance_force = self.calculate_resistance(current_velocity)
            resistance_accel = resistance_force / self.vehicle_mass
            
            # 计算目标加速度
            target_accel = 0.0
            
            if brake_force > 0.05:  # 刹车力大于阈值
                # 刹车力转换为减速度，方向与当前速度相反
                brake_decel = brake_force * self.max_deceleration
                if abs(current_velocity) > 0.01:  # 仅当车辆移动时施加制动力
                    target_accel = -v_sign * brake_decel
                else:
                    # 车辆几乎停止，强制停车
                    target_accel = 0.0
                    current_velocity = 0.0
            else:
                # 自然滑行或加速
                speed_diff = target_speed - current_velocity
                
                if abs(speed_diff) > 0.01:  # 有显著速度差
                    # 加速度基于速度差，但受最大加速度/减速度限制
                    if speed_diff > 0:  # 需要加速
                        target_accel = min(self.max_acceleration, speed_diff / 0.5)  # 0.5秒内达到目标
                    else:  # 需要减速
                        target_accel = max(-self.idle_deceleration, speed_diff / 0.5)  # 怠速减速更温和
                elif abs(current_velocity) > 0.01:  # 自然滑行
                    # 怠速减速度，方向与速度相反
                    target_accel = -v_sign * self.idle_deceleration
                else:  # 几乎静止
                    target_accel = 0.0
                    current_velocity = 0.0
            
            # 应用总加速度 (目标加速度 + 阻力)
            effective_accel = target_accel - resistance_accel
            
            # 更新速度 v = v0 + a*t
            new_velocity = current_velocity + effective_accel * dt
            
            # 如果改变方向，则启用更强的阻力模型
            if current_velocity * new_velocity < 0 and abs(current_velocity) > 0.1:
                # 速度经过零的特殊情况，先减至零
                new_velocity = 0.0
            
            # 如果目标速度接近零且当前速度很小，强制为零
            if abs(target_speed) < 0.05 and abs(new_velocity) < 0.1:
                new_velocity = 0.0
                
            # 更新速度状态
            self.current_velocity = new_velocity
            self.target_velocity = target_speed
            
            # 调试物理更新
            v_km_h = abs(new_velocity) * 3.6  # 转换为km/h显示
            if brake_force > 0.05 or abs(new_velocity) > 0.5 or abs(target_speed) > 0.5:
                self.logger.debug(f"物理更新: 速度={new_velocity:.2f}m/s ({v_km_h:.1f}km/h), " +
                                f"目标={target_speed:.2f}m/s, 加速度={effective_accel:.2f}m/s², " +
                                f"阻力={resistance_accel:.3f}m/s², 刹车={brake_force:.2f}")
                
            return new_velocity

    def r2_button_detection(self):
        """自动检测R2按钮代码的线程"""
        # 等待方向盘设备初始化完成
        time.sleep(5)
        
        if not self.r2_button_detected and self.wheel_device:
            self.logger.info("开始R2按钮自动检测过程...")
            self.logger.info("请在接下来15秒内按下方向盘上的R2按钮几次")
            
            start_time = time.monotonic()
            button_counts = {}
            detected = False
            
            # 持续15秒监听按钮
            while time.monotonic() - start_time < 15 and not self.stop_event.is_set() and not detected:
                try:
                    # 非阻塞读取
                    event = self.wheel_device.read_one()
                    if event and event.type == ecodes.EV_KEY and event.value == 1:  # 按钮按下
                        # 记录按钮出现次数
                        if event.code not in button_counts:
                            button_counts[event.code] = 0
                        button_counts[event.code] += 1
                        
                        self.logger.info(f"检测到按钮: 代码={event.code}，按下次数={button_counts[event.code]}")
                        
                        # 我们认为按下次数最多的290-315范围内的按钮是R2
                        if 290 <= event.code <= 315 and button_counts[event.code] >= 3:
                            self.logger.info(f"已识别出R2按钮代码: {event.code}")
                            # 动态更新R2按钮映射
                            with self.mode_lock:  # 防止并发修改
                                # 移除旧的R2按钮映射
                                old_r2_codes = []
                                for code, info in self.EVENT_CODE_TO_KEY[ecodes.EV_KEY].items():
                                    if info.get("name") == "R2":
                                        old_r2_codes.append(code)
                                
                                for code in old_r2_codes:
                                    if code in self.EVENT_CODE_TO_KEY[ecodes.EV_KEY]:
                                        del self.EVENT_CODE_TO_KEY[ecodes.EV_KEY][code]
                                        self.logger.info(f"移除旧的R2映射: {code}")
                                
                                # 添加新的R2按钮映射
                                self.EVENT_CODE_TO_KEY[ecodes.EV_KEY][event.code] = {
                                    "name": "R2", 
                                    "meaning": "TO_WHEEL转方向盘",
                                    "func": self.switch_to_wheel_mode
                                }
                                self.logger.info(f"添加新的R2映射: {event.code}")
                                
                                # 添加到已识别按钮列表
                                self.r2_button_codes = [event.code]
                                self.r2_button_detected = True
                                detected = True
                                
                                # 测试R2按钮功能
                                self.logger.info("测试R2按钮切换功能...")
                                prev_mode = self.mode
                                self.switch_to_wheel_mode()
                                if self.mode == self.MODE_WHEEL and prev_mode != self.MODE_WHEEL:
                                    self.logger.info("R2按钮功能测试成功!")
                                else:
                                    self.logger.warning("R2按钮功能测试失败，请检查switch_to_wheel_mode函数")
                    
                    time.sleep(0.01)  # 短暂休眠以避免CPU占用
                except Exception as e:
                    self.logger.error(f"R2按钮检测错误: {e}")
                    time.sleep(0.1)
            
            if not detected:
                # 如果没有检测到，显示所有按下的按钮，以便手动确认
                if button_counts:
                    self.logger.info("未能自动识别R2按钮，以下是检测到的所有按钮:")
                    for code, count in button_counts.items():
                        self.logger.info(f"按钮代码: {code}, 按下次数: {count}")
                    self.logger.info("请在代码中手动设置正确的R2按钮代码")
                else:
                    self.logger.warning("未检测到任何按钮按下，请确保方向盘连接正常")
            
            self.logger.info("R2按钮检测过程结束")
        elif self.r2_button_detected:
            self.logger.info(f"R2按钮已被识别，代码: {self.r2_button_codes}")
        else:
            self.logger.warning("方向盘设备未初始化，无法进行R2按钮检测")

    def init_nrf_communication(self):
        """初始化nRF通信"""
        try:
            self.logger.info("开始初始化nRF通信...")
            
            # 发送0x02命令来初始化nRF (正确的MSG_TYPE_NRF_INIT值)
            current_time = self.get_timestamp()
            
            # 构建初始化命令包
            # 使用0x42作为头部字节，MSG_TYPE_NRF_INIT作为消息类型，当前时间戳，转向角和速度都为0
            packed_payload = struct.pack('<BIffB', self.MSG_TYPE_NRF_INIT, 
                                    current_time, 0.0, 0.0, 0x00)  # 最后的0x00是喇叭状态位
            
            # 计算校验和
            checksum = 0
            for byte_val in packed_payload:
                checksum ^= byte_val
            
            # 完整消息: 头部(0x42) + 负载 + 校验和
            complete_msg = bytes([0x42]) + packed_payload + bytes([checksum])
            
            # 记录发送详情
            hex_msg = " ".join(f"{byte:02X}" for byte in complete_msg)
            self.logger.info(f"发送nRF初始化命令 HEX: {hex_msg} (Type: {self.MSG_TYPE_NRF_INIT})")
            
            # 发送消息
            self.ser.write(complete_msg)
            self.ser.flush()
            self.last_command_send_time = time.monotonic() # 更新发送时间
            
            self.logger.info("nRF初始化命令已发送")
            
            # 等待响应（可选）
            time.sleep(0.5)
            if self.ser.in_waiting > 0:
                response = self.ser.read(self.ser.in_waiting)
                hex_resp = " ".join(f"{byte:02X}" for byte in response)
                self.logger.info(f"收到nRF初始化响应 (raw): {hex_resp}")
            
            return True
        except Exception as e:
            self.logger.error(f"初始化nRF通信时出错: {e}")
            return False

    def check_wheel_status(self):
        """检查方向盘状态的线程"""
        self.logger.info("方向盘状态检查线程启动")
        while not self.stop_event.is_set():
            # 检查方向盘线程状态
            if hasattr(self, 'wheel_thread') and self.wheel_thread is not None:
                # self.logger.debug(f"方向盘线程状态: {'活跃' if self.wheel_thread.is_alive() else '已停止'}")  # 太频繁
                if not self.wheel_thread.is_alive():
                    self.logger.warning("方向盘线程已停止运行! 尝试重新初始化...")
                    self.init_wheel_device()
            else:
                self.logger.warning("方向盘线程未初始化或为None, 尝试初始化...")
                self.init_wheel_device()
            time.sleep(30)  # 降低检查频率

    def init_wheel_device(self):
        """初始化方向盘设备"""
        try:
            self.logger.info("开始初始化方向盘设备...")
            device_path = self.find_wheel_device()
                
            if device_path:
                try:
                    if self.wheel_device and self.wheel_device.path != device_path:
                        try:
                            self.wheel_device.close()
                        except Exception as e:
                            self.logger.warning(f"关闭现有方向盘设备时出错: {e}")
                    
                    if not self.wheel_device or self.wheel_device.path != device_path:
                        self.wheel_device = evdev.InputDevice(device_path)
                        self.logger.info(f"成功初始化方向盘设备: {self.wheel_device.name} ({self.wheel_device.path})")

                    # 设备能力日志移到DEBUG级别
                    if self.wheel_thread and self.wheel_thread.is_alive():
                        self.logger.info("旧方向盘线程仍在运行，新线程将取代其处理逻辑。")

                    self.wheel_thread = threading.Thread(target=self.wheel_event_listener, daemon=True)
                    self.wheel_thread.start()
                    self.logger.info("已启动方向盘事件监听线程")
                except Exception as e:
                    self.logger.error(f"初始化方向盘设备实例时发生错误: {e}", exc_info=True)
                    self.wheel_device = None
            else:
                self.logger.warning(f"找不到方向盘设备: {self.WHEEL_DEVICE_NAME}，将不启用方向盘控制功能")
                self.wheel_device = None
        except Exception as e:
            self.logger.error(f"初始化方向盘设备时发生错误: {e}", exc_info=True)

    def find_wheel_device(self):
        """查找方向盘设备路径"""
        self.logger.info(f"查找方向盘设备: {self.WHEEL_DEVICE_NAME}")
        try:
            devices = evdev.list_devices()
            if not devices:
                self.logger.warning("系统中未找到任何输入设备 (evdev)。")
                return None

            for path in devices:
                try:
                    device = evdev.InputDevice(path)
                    # self.logger.debug(f"检查设备: {device.name} ({path})") # 降到Debug级别
                    if self.WHEEL_DEVICE_NAME.lower() in device.name.lower():
                        self.logger.info(f"找到方向盘设备: {device.name} ({path})")
                        return path
                    for keyword in self.WHEEL_KEYWORDS:
                        if keyword.lower() in device.name.lower():
                            self.logger.info(f"通过关键词 '{keyword}' 找到可能的方向盘设备: {device.name} ({path})")
                            return path
                except Exception:
                    pass
        except Exception as e:
            self.logger.error(f"列出evdev输入设备时出错: {e}")
        return None

    def wheel_event_listener(self):
        """监听方向盘设备事件的线程"""
        self.logger.info("========== 方向盘事件监听线程已启动 ==========")
        if not self.wheel_device:
            self.logger.error("方向盘设备未初始化，事件监听线程退出。")
            return
        
        self.logger.info("方向盘事件映射配置:")
        for event_type, codes in self.EVENT_CODE_TO_KEY.items():
            event_type_name = "按键(EV_KEY)" if event_type == ecodes.EV_KEY else "轴(EV_ABS)"
            self.logger.info(f"  事件类型: {event_type_name}")
            for code, info in codes.items():
                code_name = ecodes.KEY.get(code, "未知按键") if event_type == ecodes.EV_KEY else ecodes.ABS.get(code, "未知轴")
                func_name = info.get("func").__name__ if "func" in info else "无函数"
                self.logger.info(f"    代码 {code} ({code_name}): {info['name']} ({info.get('meaning', '无描述')}) -> {func_name}")
        
        try:
            # 检测并打印设备的能力
            self.logger.info("设备功能探测开始...")
            capabilities = self.wheel_device.capabilities(verbose=True)
            
            # 遍历设备支持的事件类型
            for event_type, codes in capabilities.items():
                if event_type == ecodes.EV_ABS:  # 轴事件
                    self.logger.info("检测到轴控制器:")
                    for item in codes:
                        if isinstance(item, tuple):
                            code, info = item
                            self.logger.info(f"  轴代码 {code} ({ecodes.ABS.get(code, '未知')}): {info}")
                        else:
                            self.logger.info(f"  轴代码: {item}")
                            
                    # 轴探测和映射确认
                    wheel_mapped = False
                    gas_mapped = False
                    brake_mapped = False
                    
                    g29_mappings = {
                        # 各种G29上常见的轴映射
                        "wheel": [0],                  # 方向盘通常是X轴 (ABS_X)
                        "accelerator": [1, 5, 6],      # 油门通常是Y轴 (ABS_Y) 或 RZ (ABS_RZ)
                        "brake": [2, 4]                # 刹车通常是Z轴 (ABS_Z)
                    }
                    
                    available_abs_codes = []
                    for item in codes:
                        if isinstance(item, tuple):
                            available_abs_codes.append(item[0])
                        else:
                            available_abs_codes.append(item)
                    
                    # 更新轴映射
                    for wheel_code in g29_mappings["wheel"]:
                        if wheel_code in available_abs_codes and not wheel_mapped:
                            if wheel_code in self.EVENT_CODE_TO_KEY[ecodes.EV_ABS]:
                                del self.EVENT_CODE_TO_KEY[ecodes.EV_ABS][wheel_code]
                            self.EVENT_CODE_TO_KEY[ecodes.EV_ABS][wheel_code] = {"name": "wheel", "func": self.map_wheel}
                            self.logger.info(f"方向盘映射到轴代码 {wheel_code} ({ecodes.ABS.get(wheel_code, '未知')})")
                            wheel_mapped = True
                    
                    for accel_code in g29_mappings["accelerator"]:
                        if accel_code in available_abs_codes and not gas_mapped:
                            if accel_code in self.EVENT_CODE_TO_KEY[ecodes.EV_ABS]:
                                del self.EVENT_CODE_TO_KEY[ecodes.EV_ABS][accel_code]
                            self.EVENT_CODE_TO_KEY[ecodes.EV_ABS][accel_code] = {"name": "Accelerater", "func": self.map_accelerator}
                            self.logger.info(f"油门映射到轴代码 {accel_code} ({ecodes.ABS.get(accel_code, '未知')})")
                            gas_mapped = True
                    
                    for brake_code in g29_mappings["brake"]:
                        if brake_code in available_abs_codes and not brake_mapped:
                            if brake_code in self.EVENT_CODE_TO_KEY[ecodes.EV_ABS]:
                                del self.EVENT_CODE_TO_KEY[ecodes.EV_ABS][brake_code]
                            self.EVENT_CODE_TO_KEY[ecodes.EV_ABS][brake_code] = {"name": "Brake", "func": self.map_brake}
                            self.logger.info(f"刹车映射到轴代码 {brake_code} ({ecodes.ABS.get(brake_code, '未知')})")
                            brake_mapped = True
                            
                    # 如果未能映射，尝试更多的可能性
                    if not wheel_mapped or not gas_mapped or not brake_mapped:
                        self.logger.warning("无法完全映射控制轴，使用通用映射")
                        # 添加通用轴映射
                        for axis_code in available_abs_codes:
                            if axis_code not in self.EVENT_CODE_TO_KEY[ecodes.EV_ABS]:
                                if axis_code == 0:  # X轴通常是方向盘
                                    self.EVENT_CODE_TO_KEY[ecodes.EV_ABS][axis_code] = {"name": "wheel", "func": self.map_wheel}
                                    self.logger.info(f"通用映射: X轴(0) -> 方向盘")
                                elif axis_code in [2, 4]:  # Z轴或RY轴通常是刹车
                                    self.EVENT_CODE_TO_KEY[ecodes.EV_ABS][axis_code] = {"name": "Brake", "func": self.map_brake}
                                    self.logger.info(f"通用映射: 轴{axis_code} -> 刹车")
                                elif axis_code in [1, 5, 6]:  # Y轴、RZ轴或其他通常是油门
                                    self.EVENT_CODE_TO_KEY[ecodes.EV_ABS][axis_code] = {"name": "Accelerater", "func": self.map_accelerator}
                                    self.logger.info(f"通用映射: 轴{axis_code} -> 油门")
            
            # 调试消息：确认轴和按钮映射
            self.logger.info("最终轴映射:")
            for axis_code, info in self.EVENT_CODE_TO_KEY[ecodes.EV_ABS].items():
                self.logger.info(f"  轴代码 {axis_code} -> {info['name']}")
                            
            # 循环读取事件
            self.logger.info("开始读取方向盘事件")
            last_debug_time = time.monotonic()
            
            # 方向盘读取主循环
            for event in self.wheel_device.read_loop():
                if self.stop_event.is_set():
                    self.logger.info("收到停止信号，方向盘事件监听线程退出")
                    break
                
                # 定期输出调试信息
                current_time = time.monotonic()
                if current_time - last_debug_time > 30:  # 每30秒输出一次调试信息
                    self.logger.debug("方向盘事件监听线程正在运行中...")
                    last_debug_time = current_time
                
                if event.type == ecodes.EV_ABS:  # 轴事件
                    # 调试所有轴事件
                    self.logger.debug(f"轴事件: 代码={event.code}, 值={event.value}")
                    
                    # 查找此轴是否在我们的映射中
                    if event.code in self.EVENT_CODE_TO_KEY[ecodes.EV_ABS]:
                        control = self.EVENT_CODE_TO_KEY[ecodes.EV_ABS][event.code]
                        with self.wheel_lock:
                            if self.wheel_values.get(control["name"]) != event.value:
                                old_value = self.wheel_values.get(control["name"])
                                self.wheel_values[control["name"]] = event.value
                                
                                # 使用函数名作为调试信息
                                mapped_value = 0
                                if "func" in control:
                                    mapped_value = control["func"](event.value)
                                
                                if control["name"] == "wheel":
                                    self.logger.debug(f"方向盘更新: 原始值={event.value}, 映射值={mapped_value:.2f}度")
                                elif control["name"] == "Accelerater":
                                    if mapped_value > 0.05:  # 只记录明显的踩踏
                                        self.logger.debug(f"油门更新: 原始值={event.value}, 映射值={mapped_value:.2f}")
                                elif control["name"] == "Brake":
                                    if mapped_value > 0.05:  # 只记录明显的踩踏
                                        self.logger.debug(f"刹车更新: 原始值={event.value}, 映射值={mapped_value:.2f}")
                    else:
                        # 未映射的轴，记录以便后续添加
                        self.logger.debug(f"未映射的轴: 代码={event.code}, 值={event.value}")
                        
                elif event.type == ecodes.EV_KEY:  # 按钮事件
                    if event.value == 1:  # 按钮按下
                        self.logger.debug(f"按钮按下: 代码={event.code}")
                        
                        # 查找此按钮是否在我们的映射中
                        if event.code in self.EVENT_CODE_TO_KEY[ecodes.EV_KEY]:
                            control = self.EVENT_CODE_TO_KEY[ecodes.EV_KEY][event.code]
                            message = f"按钮 {control['name']} ({control.get('meaning', '')}) 按下"
                            # 根据模式调整日志级别
                            if self.mode == self.MODE_WHEEL and "ENTER" not in control["name"]:
                                self.logger.debug(message)  # 方向盘模式下普通按钮降为debug
                            else:
                                self.logger.info(message)  # 其他模式或喇叭按钮保持info级别
                                
                            if "func" in control:
                                if "ENTER" in control["name"]:
                                    self.logger.info(f"检测到喇叭按钮按下: {control['name']}, 代码={event.code}")
                                control["func"]()
                        else:
                            # 未映射的按钮，记录以便后续添加
                            self.logger.debug(f"未映射的按钮: 代码={event.code}")
                            
                    elif event.value == 0:  # 按钮释放
                        # 检查是否是喇叭按钮
                        if event.code in self.EVENT_CODE_TO_KEY[ecodes.EV_KEY]:
                            control = self.EVENT_CODE_TO_KEY[ecodes.EV_KEY][event.code]
                            if "ENTER" in control["name"]:
                                self.logger.info(f"检测到喇叭按钮释放: {control['name']}, 代码={event.code}")
                                self.release_trumpet()
                
                # 如果处于方向盘模式，发送更新的控制命令
                if self.mode == self.MODE_WHEEL:
                    # 控制值会由main_loop中的update_wheel_control统一处理和发送
                    pass
                
        except OSError as e:
            self.logger.error(f"方向盘事件监听线程读取错误 (可能设备断开): {e}")
            self.wheel_device = None # 标记设备丢失
            # 从check_wheel_status线程尝试重新初始化
        except Exception as e:
            self.logger.error(f"方向盘事件监听线程发生未知错误: {e}", exc_info=True)
        finally:
            self.logger.info("方向盘事件监听线程结束")

    def handle_enter_button(self):
        """处理Enter按钮按下事件"""
        self.logger.info("ENTER按钮被按下 - 激活喇叭")
        with self.wheel_lock: # 假设trumpet_active是共享的
            self.trumpet_active = True
        # 强制立即发送喇叭命令，避免等待周期性更新
        current_speed = 0.0
        current_steering_angle = 0.0
        
        with self.wheel_lock:
            # 获取当前控制值
            if self.mode == self.MODE_WHEEL:
                wheel_raw = self.wheel_values.get('wheel', 33000) 
                accel_raw = self.wheel_values.get('Accelerater', 255)
                brake_raw = self.wheel_values.get('Brake', 255) 
                
                current_steering_angle = self.map_wheel(wheel_raw)
                accel_mapped = self.map_accelerator(accel_raw)
                brake_mapped = self.map_brake(brake_raw)
                
                # 使用物理模型当前速度
                current_speed = self.current_velocity
            else:
                current_steering_angle = self.steering_angle_deg
                current_speed = self.speed
        
        # 强制发送带喇叭的命令
        self.send_command_with_trumpet(current_steering_angle, current_speed, True, force_send=True)

    def release_trumpet(self):
        """释放喇叭"""
        self.logger.info("喇叭按钮释放 - 关闭喇叭")
        with self.wheel_lock:
            self.trumpet_active = False
        
        # 强制立即发送关闭喇叭命令
        current_speed = 0.0
        current_steering_angle = 0.0
        
        with self.wheel_lock:
            # 获取当前控制值
            if self.mode == self.MODE_WHEEL:
                wheel_raw = self.wheel_values.get('wheel', 33000) 
                accel_raw = self.wheel_values.get('Accelerater', 255)
                brake_raw = self.wheel_values.get('Brake', 255) 
                
                current_steering_angle = self.map_wheel(wheel_raw)
                accel_mapped = self.map_accelerator(accel_raw)
                brake_mapped = self.map_brake(brake_raw)
                
                # 使用物理模型当前速度
                current_speed = self.current_velocity
            else:
                current_steering_angle = self.steering_angle_deg
                current_speed = self.speed
        
        # 强制发送关闭喇叭的命令
        self.send_command_with_trumpet(current_steering_angle, current_speed, False, force_send=True)

    def send_command_with_trumpet(self, steering_angle_deg, speed, trumpet_active, force_send=False):
        """发送带有喇叭状态的控制命令, 包含速率限制"""
        current_monotonic_time = time.monotonic()
        if not force_send and (current_monotonic_time - self.last_command_send_time < self.min_command_interval):
            # self.logger.debug(f"命令发送过于频繁，已跳过。间隔: {current_monotonic_time - self.last_command_send_time:.3f}s")
            return False

        try:
            steering_angle_deg = round(steering_angle_deg, 3)
            speed = round(speed, 3)
            current_time_ms = self.get_timestamp()
            
            with self.last_command_lock:
                self.last_command_store = {
                    'timestamp': current_time_ms,
                    'steering_angle_deg': steering_angle_deg,
                    'speed': speed,
                    'trumpet': trumpet_active
                }

            # 负载: 类型(1), 时间戳(4), 角度(4), 速度(4), 喇叭(1)
            packed_payload = struct.pack('<BIffB', self.MSG_TYPE_COMMAND, 
                                         current_time_ms, steering_angle_deg, speed, 
                                         0x01 if trumpet_active else 0x00)
            
            checksum = 0
            for byte_val in packed_payload:
                checksum ^= byte_val
            
            # 完整消息: 头部(0x42), 负载, 校验和
            complete_msg = bytes([0x42]) + packed_payload + bytes([checksum])
            
            self.log_timing_event("CMD_TRUMPET_SENT" if trumpet_active else "CMD_TRUMPET_RELEASE_SENT", 
                                  current_time_ms, 0, 0, steering_angle_deg, speed)
            
            self.ser.write(complete_msg)
            self.ser.flush()
            self.last_command_send_time = current_monotonic_time # 更新最后发送时间

            hex_msg = " ".join(f"{byte:02X}" for byte in complete_msg)
            log_message = f"串口发送(喇叭): 时间戳={current_time_ms}, 角={steering_angle_deg}, 速={speed}, 喇叭={'开' if trumpet_active else '关'}, HEX={hex_msg}"
            
            # 喇叭命令总是使用INFO级别，确保可见
            self.logger.info(log_message)

            self.last_sent_commands['speed'] = speed
            self.last_sent_commands['steering_angle_deg'] = steering_angle_deg
            return True
                
        except Exception as e:
            self.logger.error(f"发送带喇叭状态的命令时发生错误: {e}")
            return False

    def update_wheel_control(self, force_send=False):
        """基于方向盘和踏板位置更新控制值，包含真实车辆物理模型"""
        if self.mode != self.MODE_WHEEL:
            return
            
        with self.wheel_lock:
            wheel_raw = self.wheel_values.get('wheel', 33000) 
            accel_raw = self.wheel_values.get('Accelerater', 255)
            brake_raw = self.wheel_values.get('Brake', 255) 
            
            # 调试显示当前原始值
            self.logger.debug(f"当前方向盘原始值 - 方向盘:{wheel_raw}, 油门:{accel_raw}, 刹车:{brake_raw}")
            
            current_steering_angle = -self.map_wheel(wheel_raw)
            accel_mapped = self.map_accelerator(accel_raw)
            brake_mapped = self.map_brake(brake_raw)
            
            # 计算目标速度和刹车力
            target_speed = 0.0
            brake_force = 0.0
            
            # 根据刹车和油门确定目标速度和刹车力
            if brake_mapped > 0.05:  # 刹车被踩下
                brake_force = brake_mapped
                target_speed = 0.0   # 刹车时目标速度为零
                
                # 记录刹车动作
                if brake_mapped > 0.1 and time.monotonic() - self.last_wheel_status_log > 1.0:
                    self.logger.debug(f"刹车踏板: 原始值={brake_raw}, 映射值={brake_mapped:.2f}")
                    self.last_wheel_status_log = time.monotonic()
            elif accel_mapped > 0.05:  # 油门被踩下
                target_speed = accel_mapped * 3.6  # 最大速度可达3.6 m/s (约13 km/h)
                brake_force = 0.0
                
                # 记录油门动作
                if accel_mapped > 0.1 and time.monotonic() - self.last_wheel_status_log > 1.0:
                    self.logger.debug(f"油门踏板: 原始值={accel_raw}, 映射值={accel_mapped:.2f}")
                    self.last_wheel_status_log = time.monotonic()
            else:  # 既无油门也无刹车
                target_speed = 0.0   # 在自然滑行中目标速度为零
                brake_force = 0.0    # 无刹车力
                
            # 应用挡位方向
            target_speed *= self.gear_direction
            
            # 更新车辆物理模型以获取实际速度
            current_speed = self.update_vehicle_physics(target_speed, brake_force)
            current_speed = round(max(-3.6, min(3.6, current_speed)), 3)  # 限制速度范围
            current_steering_angle = round(current_steering_angle, 3)
        
        # 更新共享状态（如需要供其他模式使用）
        self.steering_angle_deg = current_steering_angle
        self.speed = current_speed  # 使用物理模型的当前速度，而非目标速度

        value_changed = (abs(self.last_sent_commands.get('speed', 0) - current_speed) > 0.05 or 
                        abs(self.last_sent_commands.get('steering_angle_deg', 0) - current_steering_angle) > 0.5)
        
        if value_changed or force_send:
            if value_changed:  # 仅在实际变化时记录
                # 计算km/h显示，用于日志
                speed_km_h = abs(current_speed) * 3.6
                # 调整方向盘控制日志的输出级别
                log_message = (f"方向盘控制更新: 角={current_steering_angle}°, 速={current_speed}m/s ({speed_km_h:.1f}km/h), " +
                            f"挡位={self.gear_direction}, 喇叭={'开' if self.trumpet_active else '关'}")
                
                if getattr(logging, self.wheel_log_level) <= logging.INFO:
                    self.logger.info(log_message)
                else:
                    self.logger.debug(log_message)
            
            if self.trumpet_active:
                self.send_command_with_trumpet(current_steering_angle, current_speed, True, force_send)
            else:
                self.send_command(current_steering_angle, current_speed, force_send)

    def map_wheel(self, value): 
        """将方向盘值从evdev范围映射到角度范围"""
        # G29的ABS_X范围通常是0-65535。中心约为32767。
        # 归一化位置 (-1到1)
        normalized_pos = (value - 32767.0) / 32768.0 
        normalized_pos = max(-1.0, min(1.0, normalized_pos))
        steering_angle = normalized_pos * 25.0 # 最大25度
        # 调试输出映射结果
        self.logger.debug(f"方向盘映射: 原始值={value}, 归一化值={normalized_pos:.3f}, 映射角度={steering_angle:.2f}度")
        return steering_angle

    def map_brake(self, value): 
        """将刹车值映射到0-1范围，适当非线性化"""
        # G29踏板通常给出255(释放)到0(完全踩下)。
        # 输出: 0.0(无刹车)到1.0(全刹车)
        normalized = (255.0 - value) / 255.0
        mapped_value = max(0.0, min(1.0, normalized))
        result = mapped_value ** 1.5 # 稍微非线性
        # 调试输出映射结果
        if mapped_value > 0.05:  # 只记录有效踩踏
            self.logger.debug(f"刹车映射: 原始值={value}, 归一化值={normalized:.3f}, 映射值={result:.3f}")
        return result

    def map_accelerator(self, value): 
        """将油门值映射到0-1范围，适当非线性化"""
        # 类似刹车: 255(释放)到0(完全踩下)。
        # 输出: 0.0(无加速)到1.0(全油门)
        normalized = (255.0 - value) / 255.0
        mapped_value = max(0.0, min(1.0, normalized))
        result = mapped_value ** 1.5 # 稍微非线性
        # 调试输出映射结果
        if mapped_value > 0.05:  # 只记录有效踩踏
            self.logger.debug(f"油门映射: 原始值={value}, 归一化值={normalized:.3f}, 映射值={result:.3f}")
        return result

    def set_forward_gear(self):
        """设置前进档位"""
        self.logger.info("切换到前进档位")
        self.gear_direction = -1
        # 如果在其他地方使用wheel_values，更新以反映档位状态
        # self.update_wheel_control(force_send=True) # 立即发送更新

    def set_reverse_gear(self):
        """设置后退档位"""
        self.logger.info("切换到后退档位")
        self.gear_direction = 1
        # self.update_wheel_control(force_send=True) # 立即发送更新

    def switch_to_wheel_mode(self):
        """切换到方向盘控制模式"""
        with self.mode_lock:
            if self.mode == self.MODE_WHEEL: return
            prev_mode = self.mode
            self.mode = self.MODE_WHEEL
            self.logger.info(f"\n****切换到 WHEEL 控制模式 (从 {prev_mode})****")
            self.send_stop_command(force_send=True) # 确保停止命令发送
            self.last_sent_commands.update({'speed': None, 'steering_angle_deg': None})
            self.gear_direction = 0 #不挂档则停止 默认前进
            self.update_wheel_control(force_send=True) # 发送当前方向盘状态

    def switch_to_auto_mode(self):
        """切换到自动控制模式"""
        with self.mode_lock:
            if self.mode == self.MODE_AUTO: return
            prev_mode = self.mode
            self.mode = self.MODE_AUTO
            self.logger.info(f"\n****切换到 AUTO 控制模式 (从 {prev_mode})****")
            self.send_stop_command(force_send=True)
            self.last_sent_commands.update({'speed': None, 'steering_angle_deg': None})
            # Autoware将通过listener_callback开始发送命令

    def keyboard_input_listener(self):
        """键盘输入监听线程"""
        self.logger.debug("键盘输入监听线程启动")
        # 以不阻塞shutdown的方式设置监听器
        listener = keyboard.Listener(on_press=self.on_press, on_release=self.on_release)
        listener.start()
        self.logger.info("键盘监听器已启动。按 ESC 退出。")
        self.stop_event.wait() # 等待停止事件停止监听器
        listener.stop()
        self.logger.info("键盘监听器已停止。")

    def on_press(self, key):
        if self.stop_event.is_set(): return
        try:
            key_char = getattr(key, 'char', None)
            # self.logger.debug(f"按键按下: {key}")
            
            if key == keyboard.Key.space:
                self.logger.info("空格键按下 - 发送停止命令")
                self.send_stop_command(force_send=True)
                with self.cmd_lock: # 用于手动模式状态
                    self.speed = 0.0
                    self.steering_angle_deg = 0.0
            elif key == keyboard.Key.f5:
                self.logger.info("F5: 手动触发时间同步...")
                self.sync_attempts = 0; self.sync_failed = False; self.sync_failure_count = 0
                self.sync_time() # sync_time本身受其自己的定时器速率限制
            elif key == keyboard.Key.f6:
                self.logger.info("F6: 手动触发nRF初始化...")
                self.init_nrf_communication()
            elif key == keyboard.Key.f7: 
                self.logger.info("F7: 手动切换到方向盘模式")
                self.switch_to_wheel_mode()
            elif key == keyboard.Key.f8: 
                self.logger.info("F8: 手动切换到自动模式")
                self.switch_to_auto_mode()
            elif key == keyboard.Key.f9:
                self.logger.info("F9: 模拟R2按钮，切换到方向盘模式")
                self.switch_to_wheel_mode()
            elif key == keyboard.Key.f10:
                self.logger.info("F10: 模拟喇叭按钮，激活喇叭")
                self.handle_enter_button()
            elif key == keyboard.Key.f11:
                self.logger.info("F11: 释放喇叭")
                self.release_trumpet()
            elif key == keyboard.Key.f12:
                # 调试方向盘设备状态
                self.logger.info("F12: 输出当前方向盘控制值和轴映射")
                with self.wheel_lock:
                    self.logger.info(f"方向盘值: {self.wheel_values}")
                    self.logger.info(f"当前速度: {self.speed}, 转向角: {self.steering_angle_deg}, 喇叭: {self.trumpet_active}")
                    self.logger.info(f"物理模型: 当前速度={self.current_velocity}, 目标速度={self.target_velocity}")
                    self.logger.info("轴映射:")
                    for axis_code, info in self.EVENT_CODE_TO_KEY[ecodes.EV_ABS].items():
                        self.logger.info(f"  轴代码 {axis_code} -> {info['name']}")
            elif key_char:
                self.key_state.add(key_char.lower())
                if key_char.lower() == 'm': self.toggle_mode()

        except Exception as e:
            self.logger.error(f"处理按键按下事件时出错: {e}")

    def on_release(self, key):
        if self.stop_event.is_set(): return
        try:
            key_char = getattr(key, 'char', None)
            # self.logger.debug(f"按键释放: {key}")
            if key_char:
                self.key_state.discard(key_char.lower())

            if key == keyboard.Key.esc:
                self.logger.info("ESC键按下 - 程序准备退出...")
                self.stop_event.set() # 信号所有线程停止
                rclpy.shutdown() # 启动ROS关闭
        except Exception as e:
            self.logger.error(f"处理按键释放事件时出错: {e}")

    def toggle_mode(self):
        """切换控制模式"""
        with self.mode_lock:
            prev_mode = self.mode
            if self.mode == self.MODE_AUTO:
                self.mode = self.MODE_MANUAL
                self.logger.info(f"\n****切换到 MANUAL 模式 (键盘控制) (从 {prev_mode})****")
            elif self.mode == self.MODE_MANUAL:
                 self.mode = self.MODE_AUTO
                 self.logger.info(f"\n****切换到 AUTO 模式 (Autoware控制) (从 {prev_mode})****")
            # 添加与WHEEL模式间的转换(如果'm'也应在其中循环)
            else: # 当前在WHEEL模式，'m'可以转到AUTO或MANUAL
                self.mode = self.MODE_AUTO 
                self.logger.info(f"\n****切换到 AUTO 模式 (Autoware控制) (从 {prev_mode})****")

            self.send_stop_command(force_send=True)
            self.last_sent_commands.update({'speed': None, 'steering_angle_deg': None})
            with self.cmd_lock: # 重置手动模式状态
                self.speed = 0.0
                self.steering_angle_deg = 0.0

    def periodic_time_sync(self):
        """定期时间同步线程"""
        while not self.stop_event.is_set():
            if self.sync_failed:
                self.logger.error("时间同步失败次数过多，执行安全停车并切换到手动模式")
                self.safety_stop_and_switch_to_manual()
                self.sync_failed = False; self.sync_attempts = 0; self.sync_failure_count = 0
                self.stop_event.wait(5.0) # 在重大故障后等待重试同步
            else:
                self.sync_time()
            
            # 睡眠更长时间，sync_time有自己的尝试逻辑
            self.stop_event.wait(10.0) # 每10秒同步一次

    def safety_stop_and_switch_to_manual(self):
        """安全停车并切换到手动模式"""
        self.logger.warning("执行安全停车并切换到手动模式")
        self.send_stop_command(force_send=True)
        with self.mode_lock:
            if self.mode != self.MODE_MANUAL:
                self.mode = self.MODE_MANUAL
                self.logger.info("\n****因时间同步失败，自动切换到 MANUAL 模式****")
                with self.cmd_lock:
                    self.speed = 0.0; self.steering_angle_deg = 0.0
                self.last_sent_commands.update({'speed': None, 'steering_angle_deg': None})
        self.log_timing_event("SAFETY_STOP_SYNC_FAIL", self.get_timestamp(), 0,0,0,0)

    def get_timestamp(self):
        """获取相对时间戳（毫秒）"""
        return int(time.time() * 1000) - self.time_base

    def log_timing_event(self, event_type, pc_time, arduino_time, delay_ms, 
                         steering_angle=0.0, speed=0.0, mapped_steering=0.0, mapped_speed=0.0, reason=""):
        """记录时间戳事件到日志文件"""
        with self.log_lock:
            try:
                with open(self.timing_log_path, 'a') as f:
                    # 为日志条目本身使用一致的时间戳
                    log_entry_timestamp = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
                    f.write(f"{log_entry_timestamp},{event_type},{pc_time},{arduino_time},{delay_ms},{steering_angle:.3f},{speed:.3f},{mapped_steering},{mapped_speed},{reason}\n")
            except Exception as e:
                self.logger.error(f"写入时间戳日志时出错: {e}")
        # self.logger.debug(f"Event: {event_type}, PC: {pc_time}, Ard: {arduino_time}, Delay: {delay_ms}, Reason: {reason}") # 可能过于详细

    def sync_time(self):
        """与Arduino同步时间"""
        if self.stop_event.is_set(): return
        with self.time_sync_lock: # 确保一次只有一个同步操作
            if self.sync_attempts >= self.sync_max_attempts:
                self.logger.warning(f"已达到最大同步尝试次数 ({self.sync_attempts})，暂时放弃同步。")
                self.sync_failed = True
                # self.sync_failure_count 由Arduino的SYNC_FAILURE消息处理递增
                return

            self.sync_attempts += 1
            current_time_ms = self.get_timestamp()
            
            # 数据包: 头部(TIME_SYNC_HEADER), PC时间戳(4), 校验和(1)
            packed_payload = struct.pack('<I', current_time_ms) # 仅时间戳
            
            checksum = self.TIME_SYNC_HEADER # 校验和包括头部
            for byte_val in packed_payload:
                checksum ^= byte_val
            
            complete_msg = bytes([self.TIME_SYNC_HEADER]) + packed_payload + bytes([checksum])
            
            try:
                self.ser.write(complete_msg)
                self.ser.flush()
                self.logger.info(f"发送时间同步请求: PC 时间={current_time_ms} ms (尝试 {self.sync_attempts}/{self.sync_max_attempts})")
                self.log_timing_event("SYNC_REQUEST_SENT", current_time_ms, 0, 0)
                self.last_sync_time = current_time_ms # 用于RTT计算
                self.command_timestamps['sync_request_sent_at'] = time.monotonic() # 用于超时处理

                # 同步响应超时由串行读取超时和定期检查处理
            except Exception as e:
                self.logger.error(f"发送时间同步请求时出错: {e}")
                # 不要立即设置sync_failed，让尝试用完或Arduino报告失败

    def handle_sync_timeout(self):
        """处理同步超时"""
        with self.time_sync_lock:
            if self.sync_attempts >= self.sync_max_attempts:
                self.logger.error(f"时间同步超时，尝试次数: {self.sync_attempts}/{self.sync_max_attempts}")
                self.sync_failed = True
                self.log_timing_event("SYNC_TIMEOUT", self.last_sync_time, 0, 0, reason="Max attempts reached")

    def calculate_checksum_with_timestamp(self, msg_type, timestamp, steering_tire_angle, speed):
        """计算带时间戳的校验和"""
        data = struct.pack('<BIff', msg_type, timestamp, steering_tire_angle, speed)
        checksum = 0
        for byte_val in data:
            checksum ^= byte_val
        return checksum

    def listener_callback(self, msg: AckermannControlCommand):
        """Autoware控制命令回调处理"""
        with self.mode_lock:
            if self.mode != self.MODE_AUTO:
                # self.logger.debug(f"忽略ROS消息 - 当前模式: {self.mode}")
                return

        steering_tire_angle_deg = round(math.degrees(msg.lateral.steering_tire_angle), 3)
        current_speed = round(msg.longitudinal.speed, 3)
        # acceleration = msg.longitudinal.acceleration # 现在没有直接用于命令

        # self.logger.debug(f"ROS CMD: Angle={steering_tire_angle_deg}, Speed={current_speed}, Accel={acceleration}")

        # 简化停止逻辑: 如果速度很低且角度为零，认为是停止。
        # Autoware应理想地发送明确的零速度/角度以停止。
        is_stop_command = abs(current_speed) < 0.01 and abs(steering_tire_angle_deg) < 0.1

        # 发送条件:
        # 1. 相较于pre_...值有显著变化
        # 2. 或者是停止命令且之前不是停止
        # 3. 或者上一个命令不同(涵盖初始发送和变化)
        significant_change = (abs(steering_tire_angle_deg - self.pre_steering_tire_angle) > ANG_THRESHLOD or
                              abs(current_speed - self.pre_speed) > SPEED_THRESHOLD)
        
        # 避免发送太多相同命令
        repeated_command = (self.last_sent_commands['speed'] == current_speed and
                            self.last_sent_commands['steering_angle_deg'] == steering_tire_angle_deg)

        if significant_change and not repeated_command:
            self.logger.info(f'Autoware CMD变化: 角={steering_tire_angle_deg:.2f} (旧:{self.pre_steering_tire_angle:.2f}), '
                             f'速={current_speed:.2f} (旧:{self.pre_speed:.2f})')
            self.pre_steering_tire_angle = steering_tire_angle_deg
            self.pre_speed = current_speed
            
            # 更新共享状态
            self.steering_angle_deg = steering_tire_angle_deg
            self.speed = current_speed
            
            self.send_command(steering_tire_angle_deg, current_speed)
        # else:
            # self.logger.debug(f'Autoware CMD变化过小或重复，已忽略。')

    def send_command(self, steering_angle_deg, speed, force_send=False):
        """统一的命令发送函数, 包含速率限制"""
        current_monotonic_time = time.monotonic()
        if not force_send and (current_monotonic_time - self.last_command_send_time < self.min_command_interval):
            # self.logger.debug(f"命令发送过于频繁，已跳过。间隔: {current_monotonic_time - self.last_command_send_time:.3f}s")
            return False
        
        try:
            steering_angle_deg = round(steering_angle_deg, 3)
            speed = round(speed, 3)
            current_time_ms = self.get_timestamp() # 相对时间
            
            with self.last_command_lock:
                self.last_command_store = { # 存储我们打算发送的内容
                    'timestamp': current_time_ms,
                    'steering_angle_deg': steering_angle_deg,
                    'speed': speed,
                    'trumpet': self.trumpet_active # 反映当前喇叭状态
                }
            
            # 负载: 类型(1), 时间戳(4), 角度(4), 速度(4), 喇叭(1)
            # Arduino期望喇叭状态即使在普通命令中也存在
            actual_trumpet_state = self.trumpet_active # 如果与wheel线程共享则在锁下读取当前状态
            
            packed_payload = struct.pack('<BIffB', self.MSG_TYPE_COMMAND, 
                                         current_time_ms, steering_angle_deg, speed, 
                                         0x01 if actual_trumpet_state else 0x00)
            
            checksum = 0
            for byte_val in packed_payload:
                checksum ^= byte_val
            
            # 完整消息: 头部(0x42), 负载, 校验和
            complete_msg = bytes([0x42]) + packed_payload + bytes([checksum])
            
            command_type = "STOP_CMD_SENT" if speed == 0 and steering_angle_deg == 0 else "CMD_SENT"
            self.log_timing_event(command_type, current_time_ms, 0, 0, steering_angle_deg, speed)
            
            self.ser.write(complete_msg)
            self.ser.flush()
            self.last_command_send_time = current_monotonic_time # 更新最后发送时间

            hex_msg = " ".join(f"{byte:02X}" for byte in complete_msg)
            source_mode = self.mode # 获取当前模式用于日志
            
            # 根据模式和命令类型调整日志级别
            log_message = f"串口发送({source_mode}): 时间戳={current_time_ms}, 角={steering_angle_deg}, 速={speed}, 喇叭={'开' if actual_trumpet_state else '关'}, HEX={hex_msg}"
            
            if self.mode == self.MODE_WHEEL and command_type != "STOP_CMD_SENT":
                # 在方向盘模式下，非停止命令使用较低的日志级别
                if getattr(logging, self.wheel_log_level) <= logging.INFO:
                    self.logger.info(log_message)
                else:
                    self.logger.debug(log_message)
            else:
                # 其他模式或停止命令始终使用INFO级别
                self.logger.info(log_message)

            self.last_sent_commands['speed'] = speed
            self.last_sent_commands['steering_angle_deg'] = steering_angle_deg
            return True
                
        except serial.SerialException as se:
            self.logger.error(f"串口写入错误: {se}. 尝试关闭并重新打开串口。")
            # 实现串口重连逻辑（如果需要）
            self.stop_event.set() # 暂时停止节点
            rclpy.shutdown()
            return False
        except Exception as e:
            self.logger.error(f"发送命令时发生错误: {e}")
            return False
            
    def send_stop_command(self, force_send=False):
        """发送停止命令: 速度=0, 转向角=0"""
        self.logger.warning("!!! 触发停车命令 !!!")
        stopped = self.send_command(0.0, 0.0, force_send=force_send)
        if stopped:
            with self.cmd_lock: # 用于手动模式和一般状态
                self.pre_steering_tire_angle = 0.0
                self.pre_speed = 0.0
                self.steering_angle_deg = 0.0
                self.speed = 0.0
            # 重置物理模型
            with self.physics_lock:
                self.current_velocity = 0.0
                self.target_velocity = 0.0

    def read_from_serial(self):
        """串口读取线程"""
        self.logger.debug("串口读取线程启动")
        buffer = bytearray()
        while not self.stop_event.is_set() and self.ser and self.ser.is_open:
            try:
                if self.ser.in_waiting > 0:
                    data_chunk = self.ser.read(self.ser.in_waiting)
                    buffer.extend(data_chunk)
                    # self.logger.debug(f"串口原始接收: {data_chunk.hex(' ')}")

                # 处理已知数据包结构的缓冲区
                while True: # 持续处理缓冲区直到没有更多完整数据包
                    processed_packet = False
                    if not buffer: break

                    header_byte = buffer[0]
                    # self.logger.debug(f"Buffer peek: 0x{header_byte:02X}, Buffer len: {len(buffer)}")

                    if header_byte == 0x42: # Arduino状态消息 (11字节)
                        if len(buffer) >= 11:
                            packet = buffer[:11]
                            # self.logger.debug(f"处理0x42包 (Arduino状态): {packet.hex(' ')}")
                            self.data_queue.put(bytes(packet)) # 放入副本
                            buffer = buffer[11:]
                            processed_packet = True
                        else: break # 完整数据包的数据不足

                    elif header_byte == self.TIME_SYNC_HEADER: # 时间同步ACK (11字节)
                        if len(buffer) >= 11: # 头(1)+类型(1)+PC时间戳(4)+Arduino时间戳(4)+校验和(1)
                            packet = buffer[:11]
                            # self.logger.debug(f"处理0x43包 (时间同步ACK): {packet.hex(' ')}")
                            self.process_time_sync_response(bytes(packet)) # 直接处理
                            buffer = buffer[11:]
                            processed_packet = True
                        else: break
                    
                    elif header_byte == self.ACK_HEADER:  # ACK响应 (11字节)
                        if len(buffer) >= 11: # 头(1)+类型(1)+PC时间戳(4)+Arduino时间戳(4)+校验和(1)
                            packet = buffer[:11]
                            # self.logger.debug(f"处理0x44包 (ACK): {packet.hex(' ')}")
                            self.process_ack_response(bytes(packet)) # 直接处理
                            buffer = buffer[11:]
                            processed_packet = True
                        else: break
                    
                    elif header_byte == self.SYNC_FAILURE_HEADER: # 同步失败 (7字节)
                        if len(buffer) >= 7: # 头(1)+原因(1)+Arduino时间戳(4)+校验和(1)
                            packet = buffer[:7]
                            # self.logger.debug(f"处理0x45包 (同步失败): {packet.hex(' ')}")
                            self.process_sync_failure_response(bytes(packet)) # 直接处理
                            buffer = buffer[7:]
                            processed_packet = True
                        else: break

                    elif header_byte == self.DECODING_STATUS_HEADER: # 解码状态 (16字节)
                        if len(buffer) >= 16:
                            packet = buffer[:16]
                            # self.logger.debug(f"处理0x46包 (解码状态): {packet.hex(' ')}")
                            self.process_decoding_status(bytes(packet)) # 直接处理
                            buffer = buffer[16:]
                            processed_packet = True
                        else: break
                    
                    else: # 未知头部或格式错误的数据包
                        self.logger.warning(f"串口收到未知包头: 0x{header_byte:02X}. 丢弃该字节。Buffer: {buffer.hex(' ')}")
                        buffer = buffer[1:] # 丢弃未知字节并重试
                        processed_packet = True # 重新评估循环条件

                    if not processed_packet: # 无法形成或处理已知数据包
                        break
            
            except serial.SerialException as se:
                self.logger.error(f"串口读取错误: {se}. 线程退出。")
                self.stop_event.set() # 通知其他线程
                break
            except Exception as e:
                self.logger.error(f"读取串口时发生未知错误: {e}", exc_info=True)
                # 可能丢弃缓冲区以从解析错误中恢复
                # buffer.clear()
                self.stop_event.wait(0.01) # 重试前短暂延迟
            
            if not self.ser or not self.ser.is_open:
                self.logger.info("串口已关闭，读取线程退出。")
                break
            self.stop_event.wait(0.005) # 短暂睡眠以防止没有数据时忙等

        self.logger.info("串口读取线程已终止。")

    def process_decoding_status(self, data: bytes):
        """处理解码状态信息"""
        try:
            # 头(1)+标志(1)+原始转向(4)+原始速度(4)+映射转向(1)+映射速度(1)+喇叭(1)+保留(1)+校验和(1)+结束(1) = 16
            if len(data) != 16 or data[0] != self.DECODING_STATUS_HEADER or data[15] != 0xAA:
                self.logger.warning(f"解码状态数据格式错误: 长度={len(data)}, 包头=0x{data[0]:02X}, 结尾=0x{data[15]:02X}")
                return
                
            # 对字节1-13的校验和(标志到保留)与data[14]比较
            checksum = 0
            for i in range(1, 14): checksum ^= data[i]
                
            if checksum != data[14]:
                self.logger.warning(f"解码状态消息校验和不正确: 计算得到=0x{checksum:02X}, 收到=0x{data[14]:02X}")
                return
            
            status_flag = data[1]
            orig_steering = struct.unpack('<f', data[2:6])[0]
            orig_speed = struct.unpack('<f', data[6:10])[0]
            mapped_steering = data[10]
            mapped_speed = data[11]
            trumpet_status = data[12]
            
            # 根据当前模式调整日志级别
            log_message = f"解码状态: Orig(S:{orig_steering:.2f}, V:{orig_speed:.2f}), " + \
                          f"Mapped(S:{mapped_steering}, V:{mapped_speed}), Trumpet:{trumpet_status}, Flag:{status_flag}"
                          
            if self.mode == self.MODE_WHEEL:
                # 在方向盘模式下使用debug级别
                self.logger.debug(log_message)
            else:
                # 其他模式使用info级别
                self.logger.info(log_message)
                
            self.log_timing_event("DECODING_STATUS_RECV", 0,0,0, orig_steering, orig_speed, 
                                mapped_steering, mapped_speed, f"Flag:{status_flag}")
        except Exception as e:
            self.logger.error(f"解析解码状态响应失败: {e}")

    def process_sync_failure_response(self, data: bytes):
        """处理同步失败响应"""
        try:
            # 头(1)+原因(1)+Arduino时间(4)+校验和(1) = 7
            if len(data) != 7 or data[0] != self.SYNC_FAILURE_HEADER:
                self.logger.warning(f"同步失败响应数据格式错误: 长度={len(data)}, 包头=0x{data[0]:02X}")
                return

            # 对头部、原因、Arduino时间的校验和与data[6]比较
            checksum = 0
            for i in range(0, 6): checksum ^= data[i]

            if checksum != data[6]:
                self.logger.warning(f"同步失败响应校验和不正确: 计算得到=0x{checksum:02X}, 收到=0x{data[6]:02X}")
                return

            failure_reason = data[1]
            arduino_time = struct.unpack('<I', data[2:6])[0]
            reason_text = {0x01: "超时", 0x02: "缓冲区满"}.get(failure_reason, "未知")
                
            self.logger.error(f"Arduino报告时间同步失败 - 原因: {reason_text} (0x{failure_reason:02X}), Arduino时间: {arduino_time} ms")
            
            with self.time_sync_lock:
                self.sync_failure_count += 1
                if self.sync_failure_count >= self.max_sync_failures:
                    self.logger.critical(f"检测到多次同步失败 ({self.sync_failure_count})。激活紧急安全程序。")
                    self.sync_failed = True # 这将触发safety_stop_and_switch_to_manual
                
            self.log_timing_event("SYNC_FAIL_ARD_RECV", self.get_timestamp(), arduino_time, 0, reason=reason_text)
        except Exception as e:
            self.logger.error(f"解析同步失败响应出错: {e}")

    def process_time_sync_response(self, data: bytes):
        """处理时间同步响应"""
        try:
            # 头(1)+类型(1)+PC时间戳(4)+Arduino时间戳(4)+校验和(1) = 11
            if len(data) != 11 or data[0] != self.TIME_SYNC_HEADER:
                self.logger.warning(f"时间同步响应数据格式错误: 长度={len(data)}, 包头=0x{data[0]:02X}")
                return

            # 对前10个字节的校验和与data[10]比较
            checksum = 0
            for i in range(0, 10): checksum ^= data[i]

            if checksum != data[10]:
                self.logger.warning(f"时间同步响应校验和不正确: 计算得到=0x{checksum:02X}, 收到=0x{data[10]:02X}")
                return

            sync_type = data[1] # 对于ACK应该是0x01
            pc_time_echo = struct.unpack('<I', data[2:6])[0]
            arduino_time_ack = struct.unpack('<I', data[6:10])[0]
            
            if sync_type == 0x01: # ACK
                with self.time_sync_lock:
                    self.time_synced = True
                    # 由于RTT，偏移计算可能复杂。
                    # 简单偏移: pc_time_echo - arduino_time_ack (测量Arduino相对于PC在ACK时的时钟)
                    self.pc_arduino_offset = pc_time_echo - arduino_time_ack 
                    self.sync_attempts = 0 # 成功同步时重置
                    self.sync_failed = False
                    self.sync_failure_count = 0 # 也重置此项
                    
                rtt_ms = (time.monotonic() - self.command_timestamps.get('sync_request_sent_at', time.monotonic())) * 1000
                
                self.logger.info(f"时间同步成功 - PC时间回显: {pc_time_echo}, Arduino确认时间: {arduino_time_ack}, 估算偏移: {self.pc_arduino_offset} ms, RTT: {rtt_ms:.2f} ms")
                self.log_timing_event("SYNC_ACK_RECV", pc_time_echo, arduino_time_ack, rtt_ms)
            else:
                self.logger.warning(f"收到未知的时间同步响应类型: 0x{sync_type:02X}")
                
        except Exception as e:
            self.logger.error(f"解析时间同步响应失败: {e}")
            # 这里不立即设置sync_failed，让尝试用完或Arduino报告失败

    def process_ack_response(self, data: bytes):
        """处理ACK响应"""
        try:
            # 头(1)+类型(1)+PC时间戳(4)+Arduino时间戳(4)+校验和(1) = 11
            if len(data) != 11 or data[0] != self.ACK_HEADER:
                self.logger.warning(f"ACK响应数据格式错误: 长度={len(data)}, 包头=0x{data[0]:02X}")
                return

            checksum = 0
            for i in range(0, 10): checksum ^= data[i]
            if checksum != data[10]:
                self.logger.warning(f"ACK响应校验和不正确: 计算得到=0x{checksum:02X}, 收到=0x{data[10]:02X}")
                return

            ack_type = data[1]
            pc_timestamp_echo = struct.unpack('<I', data[2:6])[0]
            arduino_time_ack = struct.unpack('<I', data[6:10])[0]
            
            # 计算从原始PC命令时间戳到现在的延迟
            current_pc_relative_time = self.get_timestamp()
            processing_delay = current_pc_relative_time - pc_timestamp_echo # 这是命令的粗略RTT
            
            ack_type_str = {self.ACK_RECEIVED: "CMD_RECV_BY_ARD", self.ACK_SENT: "CMD_SENT_BY_NRF"}.get(ack_type, "未知ACK")

            # 根据当前模式调整日志级别
            log_message = f"{ack_type_str}确认 - PC命令时间戳: {pc_timestamp_echo}, Arduino确认时间: {arduino_time_ack}, 总延迟估算: {processing_delay} ms"
            if self.mode == self.MODE_WHEEL:
                # 在方向盘模式下使用debug级别
                self.logger.debug(log_message)
            else:
                # 其他模式使用info级别
                self.logger.info(log_message)
                
            self.log_timing_event(ack_type_str, pc_timestamp_echo, arduino_time_ack, processing_delay)
                
        except Exception as e:
            self.logger.error(f"解析ACK响应失败: {e}")

    def process_serial_data(self): # 此线程主要处理来自data_queue的数据(主要是Arduino状态消息)
        """处理从串口读取的数据队列"""
        self.logger.debug("数据处理线程启动")
        while not self.stop_event.is_set():
            try:
                response = self.data_queue.get(timeout=0.1) # 等待数据
                if len(response) == 11 and response[0] == 0x42: # Arduino状态消息
                    # 头(1)+标志(1)+值1(4)+值2(4)+校验和(1)
                    
                    # 对标志、值1、值2的校验和(字节1到9)与response[10]比较
                    checksum = 0
                    for i in range(1, 10): checksum ^= response[i]

                    if checksum == response[10]:
                        msg_flag = response[1]
                        # 对相关标志，假设值1是mapped_steering，值2是mapped_speed
                        # 对其他标志，这些可能是原始角度/速度或其他数据
                        val1 = struct.unpack('<f', response[2:6])[0]
                        val2 = struct.unpack('<f', response[6:10])[0]
                        
                        # 检索原始命令详情用于日志上下文
                        original_steering, original_speed, cmd_timestamp = 0.0, 0.0, 0
                        with self.last_command_lock:
                            if self.last_command_store:
                                original_steering = self.last_command_store.get('steering_angle_deg', 0.0)
                                original_speed = self.last_command_store.get('speed', 0.0)
                                cmd_timestamp = self.last_command_store.get('timestamp', 0)

                        log_msg_prefix = f"Arduino状态(0x{msg_flag:02X})"
                        if msg_flag == 0x00: # NRF初始化失败
                             self.logger.error(f"{log_msg_prefix}: NRF模块初始化失败。")
                             self.log_timing_event("NRF_INIT_FAIL_ARD", cmd_timestamp,0,0, reason="NRF_BEGIN_FAIL")
                        elif msg_flag == 0x02: # NRF发送成功
                            # 根据当前模式调整日志级别
                            log_message = f"{log_msg_prefix}: NRF发送成功 - 映射角={val1:.2f}, 映射速={val2:.2f}"
                            if self.mode == self.MODE_WHEEL:
                                # 在方向盘模式下使用debug级别
                                self.logger.debug(log_message)
                            else:
                                # 其他模式使用info级别
                                self.logger.info(log_message)
                                
                            self.log_timing_event("NRF_SEND_OK_ARD", cmd_timestamp, 0,0, original_steering, original_speed, val1, val2)
                        elif msg_flag == 0x03: # NRF发送失败
                            self.logger.warning(f"{log_msg_prefix}: NRF发送失败! - 原始角={val1:.2f}, 原始速={val2:.2f}")
                            self.log_timing_event("NRF_SEND_FAIL_ARD", cmd_timestamp,0,0, val1, val2, reason="RADIO_WRITE_FAIL")
                        elif msg_flag == 0x04: # Arduino上的校验和/解析错误
                            self.logger.error(f"{log_msg_prefix}: Arduino报告接收命令校验和或解析错误。")
                            self.log_timing_event("CMD_PARSE_ERR_ARD", cmd_timestamp,0,0, reason="CHECKSUM_FAIL_ON_ARDUINO")
                        elif msg_flag == 0x05: # Arduino上的超时(未收到PC命令)
                            self.logger.error(f"{log_msg_prefix}: Arduino报告5秒内未收到PC命令。")
                            self.log_timing_event("CMD_TIMEOUT_ON_ARD", self.get_timestamp(),0,0, reason="NO_PC_CMD_5S")
                        # 0x06 (Arduino上的SYNC_REQUEST_TIMEOUT)由SYNC_FAILURE_HEADER消息处理
                        else:
                            self.logger.warning(f"{log_msg_prefix}: 未知状态标志 - Val1={val1:.2f}, Val2={val2:.2f}")
                    else:
                        self.logger.error(f"Arduino状态消息校验和不正确! HEX: {response.hex(' ')}")
                # else: # 如果read_from_serial正确过滤，不应发生
                    # self.logger.warning(f"数据处理队列收到未知格式消息: {response.hex(' ')}")
            except queue.Empty:
                continue 
            except Exception as e:
                self.logger.error(f"处理串口数据时发生错误: {e}", exc_info=True)
        self.logger.info("数据处理线程已终止。")

    def main_loop(self): # 处理键盘手动控制和周期性方向盘控制发送
        """主控制循环线程"""
        self.logger.debug("主控制循环线程启动")
        
        wheel_control_interval = 1.0 / 25.0  # 目标25Hz方向盘模式
        last_wheel_control_time = time.monotonic()
        
        manual_control_interval = self.min_command_interval # 使用配置的间隔键盘
        last_manual_control_time = time.monotonic()
        
        # 初始化物理模型时间戳
        self.last_physics_update = time.monotonic()

        while not self.stop_event.is_set() and rclpy.ok():
            current_monotonic_time = time.monotonic()
            
            current_mode = self.mode # 安全读取
                    
            if current_mode == self.MODE_MANUAL:
                if current_monotonic_time - last_manual_control_time >= manual_control_interval:
                    with self.cmd_lock: # 保护self.speed, self.steering_angle_deg, self.key_state
                        # 基于key_state的目标值
                        target_speed = self.speed 
                        target_steering_deg = self.steering_angle_deg

                        # 速度调整因子
                        speed_increment = 0.15 # 每间隔m/s
                        max_manual_speed = 1.0 # m/s

                        if 'w' in self.key_state: target_speed = min(max_manual_speed, self.speed + speed_increment)
                        elif 's' in self.key_state: target_speed = max(-max_manual_speed, self.speed - speed_increment)
                        else: target_speed = 0.0 # 如果没有速度键则逐渐停止

                        # 转向调整
                        angle_increment = 5.0 # 每间隔度数
                        max_manual_angle = 25.0 # 度

                        if 'a' in self.key_state: target_steering_deg = min(max_manual_angle, self.steering_angle_deg + angle_increment)
                        elif 'd' in self.key_state: target_steering_deg = max(-max_manual_angle, self.steering_angle_deg - angle_increment)
                        else: # 逐渐自动回正转向
                            if abs(self.steering_angle_deg) > angle_increment / 2:
                                target_steering_deg = self.steering_angle_deg - math.copysign(angle_increment, self.steering_angle_deg)
                            else:
                                target_steering_deg = 0.0
                        
                        target_speed = round(target_speed, 3)
                        target_steering_deg = round(target_steering_deg, 3)

                        if target_speed != self.speed or target_steering_deg != self.steering_angle_deg or \
                           (target_speed == 0 and self.speed !=0) or \
                           (target_steering_deg == 0 and self.steering_angle_deg !=0) : # 在变化或停止时发送

                            self.speed = target_speed
                            self.steering_angle_deg = target_steering_deg
                            
                            # self.logger.info(f"手动控制: 角={self.steering_angle_deg}, 速={self.speed}")
                            if self.trumpet_active:
                                self.send_command_with_trumpet(self.steering_angle_deg, self.speed, True)
                            else:
                                self.send_command(self.steering_angle_deg, self.speed)
                            last_manual_control_time = current_monotonic_time
            
            elif current_mode == self.MODE_WHEEL:
                if current_monotonic_time - last_wheel_control_time >= wheel_control_interval:
                    # 调试消息：当前方向盘控制值
                    with self.wheel_lock:
                        self.logger.debug(f"方向盘控制值: wheel={self.wheel_values.get('wheel', 0)}, " +
                                          f"Accelerator={self.wheel_values.get('Accelerater', 0)}, " +
                                          f"Brake={self.wheel_values.get('Brake', 0)}")
                    
                    self.update_wheel_control(force_send=True) # force_send以维持25Hz心跳
                    last_wheel_control_time = current_monotonic_time
            
            self.stop_event.wait(0.005) # 短睡眠以提高响应性
        self.logger.info("主控制循环线程已终止。")

    def stop_and_exit(self):
        """停止所有线程并清理资源"""
        if hasattr(self, 'stop_event') and not self.stop_event.is_set():
            self.logger.info("正在停止节点并清理资源...")
            self.stop_event.set() # 通知所有线程

            # 尝试发送最终停止命令（如果串口可用）
            if hasattr(self, 'ser') and self.ser and self.ser.is_open:
                try:
                    self.logger.info("发送最终停止命令...")
                    self.send_stop_command(force_send=True)
                    time.sleep(0.1) # 给发送一些时间
                except Exception as e:
                    self.logger.warning(f"发送最终停止命令时出错: {e}")
            
            # 连接线程(顺序可能对依赖关系很重要)
            thread_timeout = 1.0 # 秒
            threads_to_join = [
                getattr(self, name, None) for name in 
                ['keyboard_thread', 'main_loop_thread', 'read_thread', 'process_thread', 
                 'sync_time_thread', 'wheel_status_thread', 'wheel_thread', 'r2_detect_thread']
            ]

            for thread in threads_to_join:
                if thread and thread.is_alive():
                    self.logger.debug(f"等待线程 {thread.name} 终止...")
                    thread.join(timeout=thread_timeout)
                    if thread.is_alive():
                        self.logger.warning(f"线程 {thread.name} 终止超时。")
            
            if hasattr(self, 'wheel_device') and self.wheel_device:
                try: self.wheel_device.close(); self.logger.info("方向盘设备已关闭")
                except: pass # 忽略关闭时的错误
            
            if hasattr(self, 'ser') and self.ser and self.ser.is_open:
                try: self.ser.close(); self.logger.info(f"串口 {self.ser.portstr} 已关闭")
                except: pass

            self.logger.info("节点清理完成。")

    def destroy_node(self):
        """重写Node的destroy_node方法，确保资源被正确释放"""
        self.logger.info("节点销毁中...")
        self.stop_and_exit()
        super().destroy_node()
        self.logger.info("节点已销毁。")

def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = IntegratedControlPublisher()
        
        # 在单独的线程中启动main_loop
        node.main_loop_thread = threading.Thread(target=node.main_loop, name="MainLoopThread", daemon=True)
        node.main_loop_thread.start()

        print_initial_instructions(node)
        rclpy.spin(node)

    except KeyboardInterrupt:
        node.get_logger().info('接收到键盘中断信号 (Ctrl+C)，正在退出...')
    except Exception as e:
        if node: node.get_logger().fatal(f"节点初始化或运行时发生未捕获的严重错误: {e}", exc_info=True)
        else: logging.fatal(f"节点初始化时发生未捕获的严重错误: {e}", exc_info=True)
    finally:
        if node:
            node.destroy_node() # 这会调用stop_and_exit
        if rclpy.ok(): # 检查是否已经调用了shutdown
            rclpy.shutdown()
        
        print("程序已退出。")
        sys.exit(0) # 确保干净退出

def print_initial_instructions(node):
    print("\n========== Integrated Control Publisher ==========")
    print(f"当前模式: {node.mode}")
    print(f"日志文件位于: {os.path.abspath('logs')}")
    print(f"最小命令间隔: {node.min_command_interval*1000:.0f} ms")
    print("控制指令:")
    print("  - 'm': 切换控制模式 (AUTO <-> MANUAL <-> ?)")
    print("  - 'F7': 切换到方向盘模式 (WHEEL)")
    print("  - 'F9': 备用按键，模拟R2按钮切换到方向盘模式")
    print("  - 'F10': 模拟喇叭按钮，激活喇叭")
    print("  - 'F11': 释放喇叭")
    print("  - 'F12': 调试方向盘设备状态和轴映射")
    print("  - 'R2' (方向盘): 将自动检测并映射到方向盘模式切换")
    print("  - 'F8': 切换到自动模式 (AUTO)")
    print("  - 'R3' (方向盘): 切换到自动模式")
    print("  - 键盘控制 (MANUAL): 'w','s' (速度), 'a','d' (转向)")
    print("  - 方向盘控制 (WHEEL): 使用方向盘、踏板、档位。'ENTER' (喇叭)")
    print("  - 通用: 'Space' (停车), 'F5' (同步时间), 'F6' (NRF初始化)")
    print("  - 'Esc': 退出程序")
    print("===================================================\n")
    print("程序启动后会自动检测方向盘上的R2按钮，请在接下来的15秒内多次按下R2按钮")
    print("如果自动检测失败，可以使用F9键模拟R2按钮功能\n")

if __name__ == '__main__':
    main()