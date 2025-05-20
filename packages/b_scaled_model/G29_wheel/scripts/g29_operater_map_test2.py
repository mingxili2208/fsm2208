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
    MSG_TYPE_NRF_INIT = 0x01  # nRF初始化命令

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
        self.declare_parameter('sync_timeout', 0.05)

        port = self.get_parameter('port').get_parameter_value().string_value
        baudrate = self.get_parameter('baudrate').get_parameter_value().integer_value
        timeout = self.get_parameter('timeout').get_parameter_value().double_value
        self.sync_max_attempts = self.get_parameter('sync_max_attempts').get_parameter_value().integer_value
        self.sync_timeout = self.get_parameter('sync_timeout').get_parameter_value().double_value
        
        # 记录参数信息
        self.logger.info(f"参数配置: 端口={port}, 波特率={baudrate}, 超时={timeout}秒")
        self.logger.info(f"同步配置: 最大尝试次数={self.sync_max_attempts}, 超时={self.sync_timeout}秒")
        
        # 初始化变量
        self.pre_steering_tire_angle = None
        self.pre_speed = None
        self.data_queue = queue.Queue()
        self.stop_event = threading.Event()
        self.cmd_lock = threading.Lock()
        self.steering_angle_deg = 0.0
        self.speed = 0.0
        self.mode = self.MODE_AUTO
        self.mode_lock = threading.RLock()
        
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
            raise

        time.sleep(2)  # 等待串口稳定

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

        # 发送初始停止命令
        self.logger.info("发送初始停止命令")
        self.send_stop_command()

    def init_nrf_communication(self):
        """初始化nRF通信"""
        try:
            self.logger.info("开始初始化nRF通信...")
            
            # 发送0x01命令来初始化nRF
            current_time = self.get_timestamp()
            
            # 构建初始化命令包
            # 使用0x42作为头部字节，MSG_TYPE_NRF_INIT作为消息类型，当前时间戳，转向角和速度都为0
            packed_msg = struct.pack('<BBIffB', 0x42, self.MSG_TYPE_NRF_INIT, 
                                    current_time, 0.0, 0.0, 0x00)  # 最后的0x00是喇叭状态位
            
            # 计算校验和
            checksum = self.calculate_checksum_with_timestamp(self.MSG_TYPE_NRF_INIT, current_time, 0.0, 0.0)
            
            # 添加校验和
            complete_msg = packed_msg + bytes([checksum])
            
            # 记录发送详情
            hex_msg = " ".join(f"{byte:02X}" for byte in complete_msg)
            self.logger.info(f"发送nRF初始化命令 HEX: {hex_msg}")
            
            # 发送消息
            self.ser.write(complete_msg)
            self.ser.flush()
            
            self.logger.info("nRF初始化命令已发送")
            
            # 等待响应（可选）
            time.sleep(0.5)
            if self.ser.in_waiting > 0:
                response = self.ser.read(self.ser.in_waiting)
                hex_resp = " ".join(f"{byte:02X}" for byte in response)
                self.logger.info(f"收到nRF初始化响应: {hex_resp}")
            
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
                self.logger.info(f"方向盘线程状态: {'活跃' if self.wheel_thread.is_alive() else '已停止'}")
                if self.wheel_thread.is_alive():
                    self.logger.info("方向盘设备已连接并正在监听事件")
                else:
                    self.logger.warning("方向盘线程已停止运行! 尝试重新初始化...")
                    self.init_wheel_device()
            else:
                self.logger.warning("方向盘线程未初始化或为None")
                self.init_wheel_device()
                
            # 记录当前所有输入设备
            try:
                devices = [evdev.InputDevice(path) for path in evdev.list_devices()]
                self.logger.info(f"当前系统中的输入设备数量: {len(devices)}")
                for i, device in enumerate(devices):
                    self.logger.info(f"设备 {i+1}: {device.name} (路径: {device.path})")
                    
                    # 检查是否有任何看起来像方向盘的设备但未被识别
                    if not self.wheel_device:
                        for keyword in self.WHEEL_KEYWORDS:
                            if keyword.lower() in device.name.lower():
                                self.logger.info(f"发现可能的方向盘设备: {device.name}")
                                if "g29" in device.name.lower() or "driving force" in device.name.lower():
                                    self.logger.info(f"尝试使用此设备初始化方向盘")
                                    try:
                                        self.wheel_device = device
                                        self.init_wheel_device()
                                        break
                                    except Exception as e:
                                        self.logger.error(f"使用设备初始化方向盘时出错: {e}")
            except Exception as e:
                self.logger.error(f"列出输入设备时出错: {e}")
                
            # 每30秒检查一次
            time.sleep(30)

    def init_wheel_device(self):
        """初始化方向盘设备"""
        try:
            self.logger.info("开始初始化方向盘设备...")
            
            # 如果已经有一个wheel_device，先尝试使用它
            if self.wheel_device:
                self.logger.info(f"使用现有方向盘设备: {self.wheel_device.name}")
                device_path = self.wheel_device.path
            else:
                # 否则查找新设备
                device_path = self.find_wheel_device()
                
            if device_path or self.wheel_device:
                try:
                    # 如果已存在设备实例但不是从device_path创建的，先关闭它
                    if self.wheel_device and (not device_path or self.wheel_device.path != device_path):
                        try:
                            self.wheel_device.close()
                            self.logger.info("已关闭现有方向盘设备")
                        except Exception as e:
                            self.logger.warning(f"关闭现有方向盘设备时出错: {e}")
                    
                    # 如果找到了新设备路径，创建新的设备实例
                    if device_path and (not self.wheel_device or self.wheel_device.path != device_path):
                        self.wheel_device = evdev.InputDevice(device_path)
                        self.logger.info(f"从路径创建方向盘设备: {self.wheel_device.name}")
                    
                    # 打印设备支持的事件类型和代码
                    try:
                        if self.wheel_device:
                            capabilities = self.wheel_device.capabilities(verbose=True)
                            self.logger.info("设备支持的事件类型:")
                            for event_type, codes in capabilities.items():
                                event_type_name = "EV_KEY" if event_type == ecodes.EV_KEY else "EV_ABS" if event_type == ecodes.EV_ABS else str(event_type)
                                self.logger.info(f"  {event_type_name}:")
                                if isinstance(codes, list):
                                    for code in codes:
                                        if isinstance(code, tuple):
                                            self.logger.info(f"    {code[0]}: {code[1]}")
                                        else:
                                            self.logger.info(f"    {code}")
                            
                            # 特别记录按键事件，用于调试R2按钮
                            if ecodes.EV_KEY in capabilities:
                                self.logger.info("支持的按键代码:")
                                for code in capabilities[ecodes.EV_KEY]:
                                    if isinstance(code, tuple):
                                        code_value = code[0]
                                    else:
                                        code_value = code
                                    
                                    # 特别标记出我们关心的按钮
                                    special_note = ""
                                    if code_value == 294:
                                        special_note = " <-- 当前配置为R2按钮"
                                    elif code_value in [292, 293, 298, 302, 303, 711]:
                                        special_note = " <-- 已映射按钮"
                                        
                                    self.logger.info(f"    按键代码: {code_value}{special_note}")
                    except Exception as e:
                        self.logger.error(f"获取设备能力时出错: {e}")
                    
                    # 尝试禁用力反馈
                    try:
                        if self.wheel_device and ecodes.FF_AUTOCENTER in self.wheel_device.capabilities().get(ecodes.EV_FF, []):
                            self.logger.info("尝试禁用力反馈...")
                            self.wheel_device.set_autocenter(0)
                            self.logger.info("已禁用自动居中力反馈")
                    except Exception as e:
                        self.logger.warning(f"禁用力反馈时出错: {e}")
                    
                    # 如果已有线程在运行，先停止它
                    if self.wheel_thread and self.wheel_thread.is_alive():
                        # 无法直接停止线程，但设置stop_event可以让线程退出循环
                        self.logger.info("等待现有方向盘线程终止...")
                        time.sleep(1)  # 给线程一些时间退出
                    
                    # 启动新的方向盘监听线程
                    self.wheel_thread = threading.Thread(target=self.wheel_event_listener, daemon=True)
                    self.wheel_thread.start()
                    self.logger.info("已启动方向盘事件监听线程")
                except Exception as e:
                    self.logger.error(f"初始化方向盘设备实例时发生错误: {e}")
                    self.logger.error(f"错误详情: {type(e).__name__}: {str(e)}")
                    import traceback
                    self.logger.error(f"异常堆栈: {traceback.format_exc()}")
            else:
                self.logger.warning(f"找不到方向盘设备: {self.WHEEL_DEVICE_NAME}，将不启用方向盘控制功能")
        except Exception as e:
            self.logger.error(f"初始化方向盘设备时发生错误: {e}")
            self.logger.error(f"错误详情: {type(e).__name__}: {str(e)}")
            import traceback
            self.logger.error(f"异常堆栈: {traceback.format_exc()}")

    def find_wheel_device(self):
        """查找方向盘设备路径"""
        self.logger.info(f"查找方向盘设备: {self.WHEEL_DEVICE_NAME}")
        try:
            # 列出所有设备并详细记录
            all_devices = evdev.list_devices()
            self.logger.info(f"系统中共有 {len(all_devices)} 个输入设备")
            
            exact_match = None
            best_match = None
            
            for path in all_devices:
                try:
                    device = evdev.InputDevice(path)
                    self.logger.info(f"检查设备: {device.name} ({path})")
                    
                    # 精确匹配
                    if self.WHEEL_DEVICE_NAME.lower() == device.name.lower():
                        self.logger.info(f"找到精确匹配的方向盘设备: {device.name} ({path})")
                        exact_match = path
                        break
                    
                    # 部分匹配
                    if self.WHEEL_DEVICE_NAME.lower() in device.name.lower():
                        self.logger.info(f"找到包含名称的方向盘设备: {device.name} ({path})")
                        best_match = path
                    
                    # 关键词匹配
                    if not best_match:
                        for keyword in self.WHEEL_KEYWORDS:
                            if keyword.lower() in device.name.lower():
                                self.logger.info(f"找到包含关键词的可能方向盘设备: {device.name} ({path})")
                                best_match = path
                                break
                except Exception as e:
                    self.logger.warning(f"无法访问设备 {path}: {e}")
                    
            # 返回最佳匹配
            if exact_match:
                self.logger.info(f"使用精确匹配的设备: {exact_match}")
                return exact_match
            elif best_match:
                self.logger.info(f"使用最佳匹配的设备: {best_match}")
                return best_match
        except Exception as e:
            self.logger.error(f"列出输入设备时出错: {e}")
            
        return None

    def wheel_event_listener(self):
        """监听方向盘设备事件的线程"""
        self.logger.info("========== 方向盘事件监听线程已启动 ==========")
        
        # 用于定期输出心跳信息的时间戳
        last_heartbeat = time.time()
        
        # 记录最后一次检测到的事件时间
        last_event_time = time.time()
        
        # 为调试目的创建一个按键代码到名称的映射
        known_button_codes = {
            292: "右换挡拨片",
            293: "左换挡拨片",
            294: "R2按钮",
            295: "L2按钮",
            296: "△按钮",
            297: "○按钮",
            298: "R3按钮",
            299: "L3按钮",
            300: "SELECT/SHARE按钮",
            301: "START/OPTIONS按钮",
            302: "档位前推(后退)",
            303: "档位后推(前进)",
            704: "PS按钮",
            705: "TOUCHPAD按钮",
            706: "MUTE按钮",
            711: "ENTER按钮",
        }
        
        # 定义设备事件到映射函数的字典
        EVENT_CODE_TO_KEY = {
            ecodes.EV_ABS: {
                0: {"name": "wheel", "func": self.map_wheel},
                2: {"name": "Accelerater", "func": self.map_accelerator},
                5: {"name": "Brake", "func": self.map_brake},
            },
            ecodes.EV_KEY: {
                292: {"name": "Right_pick", "meaning": "右转向"},
                293: {"name": "Left_pick", "meaning": "左转向"},
                711: {"name": "ENTER", "meaning": "喇叭/蜂鸣器", "func": self.handle_enter_button},
                294: {"name": "R2", "meaning": "TO_MANNUR转人工", "func": self.switch_to_wheel_mode},
                298: {"name": "R3", "meaning": "TO_AUTO转自动", "func": self.switch_to_auto_mode},
                302: {"name": "Gear_Forward", "meaning": "档位前推~后退", "func": self.set_reverse_gear},
                303: {"name": "Gear_Push_Back", "meaning": "档位后推~前进", "func": self.set_forward_gear},
            }
        }
        
        self.logger.info("方向盘事件映射配置:")
        for event_type, codes in EVENT_CODE_TO_KEY.items():
            event_type_name = "按键(EV_KEY)" if event_type == ecodes.EV_KEY else "轴(EV_ABS)"
            self.logger.info(f"  事件类型: {event_type_name}")
            for code, info in codes.items():
                func_name = info.get("func").__name__ if "func" in info else "无函数"
                self.logger.info(f"    代码 {code}: {info['name']} ({info.get('meaning', '无描述')}) -> {func_name}")
        
        try:
            # 检查设备是否有效
            if not self.wheel_device:
                self.logger.error("方向盘设备未初始化，事件监听线程退出")
                return
                
            # 验证设备是否可读
            try:
                read_event_success = False
                timeout_start = time.time()
                
                # 尝试在短时间内读取一个事件来验证设备是否正常工作
                # 注意: read_one()通常是非阻塞的，如果没有事件会返回None
                while not read_event_success and time.time() - timeout_start < 2:
                    try:
                        event = self.wheel_device.read_one()
                        if event:
                            self.logger.info(f"成功从设备读取测试事件: {event}")
                            read_event_success = True
                            break
                        time.sleep(0.1)  # 短暂等待后再次尝试
                    except IOError as e:
                        if e.errno == 11:  # 资源暂时不可用，这是正常的
                            pass
                        else:
                            raise
                
                if not read_event_success:
                    self.logger.warning("在验证阶段未能读取任何事件，但这不一定是错误")
            except Exception as e:
                self.logger.error(f"测试读取设备事件时出错: {e}")
                # 尝试重新打开设备
                try:
                    if hasattr(self, 'wheel_device') and self.wheel_device:
                        self.wheel_device.close()
                    device_path = self.find_wheel_device()
                    if device_path:
                        self.wheel_device = evdev.InputDevice(device_path)
                        self.logger.info(f"已重新打开方向盘设备: {self.wheel_device.name}")
                    else:
                        self.logger.error("无法找到方向盘设备，线程退出")
                        return
                except Exception as e2:
                    self.logger.error(f"重新打开设备时出错: {e2}")
                    return
            
            # 开始事件循环
            self.logger.info("开始方向盘事件监听循环...")
            
            # 记录R2按钮是否检测到
            r2_detected = False
            
            event_count = 0  # 用于记录接收到的事件数量
            
            # 持续读取方向盘事件
            for event in self.wheel_device.read_loop():
                event_count += 1
                if event_count % 100 == 0:  # 每100个事件记录一次
                    self.logger.debug(f"已处理 {event_count} 个方向盘事件")
                    
                if self.stop_event.is_set():
                    self.logger.info("收到停止信号，方向盘事件监听线程退出")
                    break
                
                # 更新最后事件时间
                last_event_time = time.time()
                
                # 每10秒输出一次心跳消息
                current_time = time.time()
                if current_time - last_heartbeat > 10:
                    self.logger.info(f"方向盘事件监听线程仍在运行... (最后事件: {current_time - last_event_time:.1f}秒前)")
                    last_heartbeat = current_time
                
                # 记录所有事件类型（调试用）
                event_type_name = "EV_KEY" if event.type == ecodes.EV_KEY else "EV_ABS" if event.type == ecodes.EV_ABS else str(event.type)
                self.logger.debug(f"方向盘事件: 类型={event_type_name}, 代码={event.code}, 值={event.value}")
                
                # 特别记录R2按钮事件
                if event.type == ecodes.EV_KEY and event.code == 294:
                    button_state = "按下" if event.value == 1 else "释放" if event.value == 0 else f"重复 ({event.value})"
                    self.logger.info(f"检测到R2按钮事件: {button_state}")
                    r2_detected = True
                    
                    # 直接调用切换模式函数确认它能正常工作
                    if event.value == 1:  # 按下时
                        self.logger.info("R2按钮按下 - 直接调用切换到方向盘模式函数")
                        self.switch_to_wheel_mode()
                
                # 特别记录所有按键事件
                if event.type == ecodes.EV_KEY and event.value == 1:  # 按键按下
                    button_name = known_button_codes.get(event.code, f"未知按钮({event.code})")
                    self.logger.info(f"检测到按键按下: {button_name} (代码={event.code})")
                
                # 处理相关事件
                if event.type in EVENT_CODE_TO_KEY and event.code in EVENT_CODE_TO_KEY[event.type]:
                    control = EVENT_CODE_TO_KEY[event.type][event.code]
                    
                    # 对于轴事件(方向盘、油门、刹车)
                    if event.type == ecodes.EV_ABS:
                        with self.wheel_lock:
                            # 检查值是否有变化，如果有变化才记录日志
                            old_value = self.wheel_values.get(control["name"])
                            if old_value != event.value:
                                self.wheel_values[control["name"]] = event.value
                                
                                # 记录日志（只在值变化时）
                                if "func" in control:
                                    mapped_value = control["func"](event.value)
                                    self.logger.debug(f"{control['name']}值变化: 原始值={event.value}, 映射值={mapped_value}")
                        
                        # 如果在方向盘控制模式下，更新控制值
                        if self.mode == self.MODE_WHEEL:
                            self.update_wheel_control()
                    
                    # 对于按钮事件(按键)
                    elif event.type == ecodes.EV_KEY:
                        # 按钮按下时触发
                        if event.value == 1:
                            self.logger.info(f"按钮 {control['name']} ({control.get('meaning', '')}) 按下 - 代码: {event.code}")
                            
                            if "func" in control:
                                func_name = control['func'].__name__
                                self.logger.info(f"执行函数: {func_name}")
                                control["func"]()
                            
                            # 更新档位状态
                            if control["name"] in ["Gear_Forward", "Gear_Push_Back"]:
                                with self.wheel_lock:
                                    self.wheel_values[control["name"]] = event.value
                        
                        # 按钮释放，对于Enter键，需要处理释放事件
                        elif event.value == 0:
                            self.logger.debug(f"按钮 {control['name']} 释放")
                            if control["name"] == "ENTER":
                                self.release_trumpet()
                            if control["name"] in ["Gear_Forward", "Gear_Push_Back"]:
                                with self.wheel_lock:
                                    self.wheel_values[control["name"]] = event.value
                
                # 如果我们关心的R2按钮默认没有被检测到，尝试记录所有按钮并找出可能的替代按钮
                if not r2_detected and event.type == ecodes.EV_KEY and event.value == 1:
                    self.logger.info(f"潜在R2按钮候选: 代码={event.code}")
                    
                    # 如果检测到任何按钮代码在290-300范围内，可能是R2按钮
                    if 290 <= event.code <= 300 and event.code != 294:
                        self.logger.info(f"检测到可能的R2按钮替代品: 代码={event.code}")
                        
        except Exception as e:
            self.logger.error(f"方向盘事件监听线程出错: {e}")
            self.logger.error(f"错误详情: {type(e).__name__}: {str(e)}")
            import traceback
            self.logger.error(f"异常堆栈: {traceback.format_exc()}")
        finally:
            self.logger.info("方向盘事件监听线程结束")

    def handle_enter_button(self):
        """处理Enter按钮按下事件"""
        self.logger.info("ENTER按钮被按下 - 激活喇叭")
        
        # 更新喇叭状态
        with self.wheel_lock:
            self.trumpet_active = True
        
        # 获取当前控制值并发送带喇叭信号的命令
        self.send_command_with_trumpet(self.steering_angle_deg, self.speed, True)
        
        # 如果已有计时器在运行，先停止它
        if self.trumpet_timer and self.trumpet_timer.is_alive():
            self.trumpet_timer.cancel()
            
        # 我们不设置自动释放计时器，让用户手动释放喇叭

    def release_trumpet(self):
        """释放喇叭"""
        self.logger.info("喇叭按钮释放 - 关闭喇叭")
        
        # 取消任何可能存在的喇叭定时器
        if self.trumpet_timer and self.trumpet_timer.is_alive():
            self.trumpet_timer.cancel()
            self.trumpet_timer = None
        
        # 更新喇叭状态
        with self.wheel_lock:
            self.trumpet_active = False
        
        # 发送带有喇叭释放信号的控制命令
        self.send_command_with_trumpet(self.steering_angle_deg, self.speed, False)

    def send_command_with_trumpet(self, steering_angle_deg, speed, trumpet_active):
        """发送带有喇叭状态的控制命令"""
        try:
            # 四舍五入到3位小数
            steering_angle_deg = round(steering_angle_deg, 3)
            speed = round(speed, 3)
            current_time = self.get_timestamp()
            
            # 存储当前命令
            with self.last_command_lock:
                self.last_command_store = {
                    'timestamp': current_time,
                    'steering_angle_deg': steering_angle_deg,
                    'speed': speed,
                    'trumpet': trumpet_active
                }

            # 计算校验和（包含喇叭状态）
            checksum = self.calculate_checksum_with_timestamp_and_trumpet(
                self.MSG_TYPE_COMMAND, current_time, steering_angle_deg, speed, trumpet_active)

            # 打包消息（带喇叭状态）
            packed_msg = struct.pack('<BBIffB', 0x42, self.MSG_TYPE_COMMAND, 
                                    current_time, steering_angle_deg, speed, 
                                    0x01 if trumpet_active else 0x00)  # 喇叭状态位
            
            # 添加校验和
            packed_msg += bytes([checksum])
            
            # 记录发送事件
            command_type = "TRUMPET_CMD" if trumpet_active else "TRUMPET_RELEASE"
            self.log_timing_event(command_type, current_time, 0, 0, steering_angle_deg, speed)
            
            # 发送消息
            self.ser.write(packed_msg)
            self.ser.flush()
            
            # 记录详情
            hex_msg = " ".join(f"{byte:02X}" for byte in packed_msg)
            self.logger.debug(f"发送带喇叭状态的命令 HEX: {hex_msg}")
            self.logger.info(f"发送到串口 - 时间戳: {current_time} ms, 速度: {speed} m/s, 转向角度: {steering_angle_deg} 度, 喇叭: {'开启' if trumpet_active else '关闭'}")

            # 更新上次发送的命令
            self.last_sent_commands['speed'] = speed
            self.last_sent_commands['steering_angle_deg'] = steering_angle_deg
                
        except Exception as e:
            self.logger.error(f"发送带喇叭状态的命令时发生错误: {e}")

    def calculate_checksum_with_timestamp_and_trumpet(self, msg_type, timestamp, steering_tire_angle, speed, trumpet_active):
        """计算包含喇叭状态的校验和"""
        # 打包数据为字节
        data = struct.pack('<BIffB', msg_type, timestamp, steering_tire_angle, speed, 
                          0x01 if trumpet_active else 0x00)
        checksum = 0
        for byte in data:
            checksum ^= byte
        return checksum

    def update_wheel_control(self, force_send=False):
        """基于方向盘和踏板位置更新控制值
        
        Args:
            force_send: 如果为True，将强制发送命令，即使与上次命令相同
        """
        if self.mode != self.MODE_WHEEL:
            return
            
        with self.wheel_lock:
            # 获取当前方向盘和踏板值
            wheel_value = self.wheel_values.get('wheel', 33000)
            accel_value = self.wheel_values.get('Accelerater', 255)
            brake_value = self.wheel_values.get('Brake', 255)
            
            # 映射值
            steering_angle = self.map_wheel(wheel_value)
            accel = self.map_accelerator(accel_value)
            brake = self.map_brake(brake_value)
            
            # 计算线速度 (考虑油门和刹车的综合效果)
            # 刹车优先于油门
            if brake > 0.1:  # 有明显刹车时
                raw_speed = -brake * 1.8  # 刹车产生负向加速度
            else:
                raw_speed = accel * 1.8  # 油门产生正向加速度
                
            # 考虑档位方向
            speed = raw_speed * self.gear_direction
            
            # 限制速度范围
            speed = max(-1.8, min(1.8, speed))
            
            # 四舍五入到3位小数
            speed = round(speed, 3)
            steering_angle = round(steering_angle, 3)
        
        # 检查是否需要发送命令
        value_changed = (self.last_sent_commands['speed'] != speed or 
                        self.last_sent_commands['steering_angle_deg'] != steering_angle)
        
        # 仅在调试级别记录常规更新，减少日志量
        if value_changed:
            # 只在值变化时记录调试信息
            self.logger.debug(f"方向盘控制更新: 转向角={steering_angle}°, 速度={speed}m/s, 方向={self.gear_direction}")
        
        if  value_changed:
            # 只有在值变化或强制发送时才记录信息级别日志
            if value_changed:
                self.logger.info(f"方向盘控制值变化: 转向角={steering_angle}°, 速度={speed}m/s")
            
            # 如果喇叭处于激活状态，发送带喇叭的命令
            if self.trumpet_active:
                self.send_command_with_trumpet(steering_angle, speed, True)
            else:
                self.send_command(steering_angle, speed)

    def map_wheel(self, value):
        """
        将方向盘值从evdev范围(0-65535)映射到转向角度(-25到+25度)
        方向盘中心位置约为33000
        物理方向盘范围为0-900度(左450, 右450从中心)
        """
        # 定义方向盘中心和校准参数
        wheel_center = 33000
        evdev_range = 65535
        
        # 计算相对于中心的归一化位置(-1到1)
        normalized_pos = (value - wheel_center) / (evdev_range / 2)
        # 限制在-1到1的范围内
        normalized_pos = max(-1, min(1, normalized_pos))
        
        # 映射到目标范围(-25到+25度)
        steering_angle = normalized_pos * 25
        
        return steering_angle

    def map_brake(self, value):
        """
        刹车的非线性映射
        输入范围: 255(无刹车)到0(全刹车)
        输出范围: 0.0(无刹车)到1.0(全刹车)
        使用指数曲线以便在刹车踩得更深时更敏感
        """
        # 反转值，使0表示无刹车，255表示全刹车
        inverted = 255 - value
        
        # 归一化到0-1范围
        normalized = inverted / 255.0
        
        # 应用非线性映射(指数曲线: x^2)
        # 这使得刹车在踩得更深时更灵敏
        mapped_value = normalized ** 2
        
        return mapped_value

    def map_accelerator(self, value):
        """
        油门的非线性映射
        输入范围: 255(无油门)到0(全油门)
        输出范围: 0.0(无油门)到1.0(全油门)
        使用指数曲线以便在油门踩得更深时更敏感
        """
        # 反转值，使0表示无油门，255表示全油门
        inverted = 255 - value
        
        # 归一化到0-1范围
        normalized = inverted / 255.0
        
        # 应用非线性映射(指数曲线: x^2)
        # 这使得油门在踩得更深时更灵敏
        mapped_value = normalized ** 2
        
        return mapped_value

    def set_forward_gear(self):
        """设置为前进档位"""
        self.logger.info("切换到前进档位")
        self.gear_direction = 1
        with self.wheel_lock:
            self.wheel_values['Gear_Push_Back'] = 1
            self.wheel_values['Gear_Forward'] = 0

    def set_reverse_gear(self):
        """设置为后退档位"""
        self.logger.info("切换到后退档位")
        self.gear_direction = -1
        with self.wheel_lock:
            self.wheel_values['Gear_Forward'] = 1
            self.wheel_values['Gear_Push_Back'] = 0

    def switch_to_wheel_mode(self):
        """切换到方向盘控制模式"""
        self.logger.info("尝试切换到方向盘控制模式...")
        with self.mode_lock:
            prev_mode = self.mode
            self.mode = self.MODE_WHEEL
            self.logger.info("\n\n")
            self.logger.info(f"****切换到 WHEEL 控制模式 (方向盘控制)**** (从 {prev_mode} 模式)")
            # 重置命令
            self.send_stop_command()
            self.last_sent_commands['speed'] = None
            self.last_sent_commands['steering_angle_deg'] = None
            # 初始化为前进档位
            self.gear_direction = 1
            
            # 立即强制更新一次控制值，确保即使用户不操作方向盘也能发送初始命令
            self.logger.info("强制更新初始控制值...")
            self.update_wheel_control(force_send=True)

    def switch_to_auto_mode(self):
        """切换到自动控制模式"""
        self.logger.info("尝试切换到自动控制模式...")
        with self.mode_lock:
            prev_mode = self.mode
            self.mode = self.MODE_AUTO
            self.logger.info("\n\n")
            self.logger.info(f"****切换到 AUTO 控制模式 (Autoware控制)**** (从 {prev_mode} 模式)")
            # 重置命令
            self.send_stop_command()
            self.last_sent_commands['speed'] = None
            self.last_sent_commands['steering_angle_deg'] = None

    def keyboard_input_listener(self):
        """键盘输入监听线程"""
        self.logger.debug("键盘输入监听线程启动")
        with keyboard.Listener(on_press=self.on_press, on_release=self.on_release) as listener:
            listener.join()

    def on_press(self, key):
        try:
            key_char = getattr(key, 'char', None)
            key_name = str(key).replace('Key.', '')
            self.logger.debug(f"按键按下: {key_name}")
            
            if key == keyboard.Key.space:
                # 设置速度和转向角为0
                self.logger.info("捕捉到空格键。正在发送停止命令...")
                self.send_stop_command()
                with self.cmd_lock:
                    self.speed = 0.0
                    self.steering_angle_deg = 0.0
                    # 重置上次发送的命令
                    self.last_sent_commands['speed'] = None
                    self.last_sent_commands['steering_angle_deg'] = None
            elif key == keyboard.Key.f5:
                # F5键触发时间同步
                self.logger.info("手动触发时间同步...")
                # 重置同步尝试计数
                self.sync_attempts = 0
                self.sync_failed = False
                self.sync_failure_count = 0
                self.sync_time()
            elif key == keyboard.Key.f6:
                # F6键触发nRF初始化
                self.logger.info("手动触发nRF初始化...")
                self.init_nrf_communication()
            elif key == keyboard.Key.f7:
                # F7键触发方向盘模式
                self.logger.info("手动触发切换到方向盘模式...")
                self.switch_to_wheel_mode()
            elif key == keyboard.Key.f8:
                # F8键触发自动模式
                self.logger.info("手动触发切换到自动模式...")
                self.switch_to_auto_mode()
            else:
                try:
                    key_char = key.char.lower()
                    self.key_state.add(key_char)

                    if key_char == 'm':
                        # 切换到键盘手动模式
                        self.toggle_mode()
                        # 发送停止命令
                        self.logger.info("Switching state triggers parking")
                        self.send_stop_command()
                        self.stop_flag = False
                        with self.cmd_lock:
                            self.speed = 0.0
                            self.steering_angle_deg = 0.0
                            # 重置上次发送的命令
                            self.last_sent_commands['speed'] = None
                            self.last_sent_commands['steering_angle_deg'] = None
                except AttributeError:
                    pass  # 非字符键忽略
        except Exception as e:
            self.logger.error(f"处理按键按下事件时出错: {e}")

    def on_release(self, key):
        try:
            key_name = str(key).replace('Key.', '')
            self.logger.debug(f"按键释放: {key_name}")
            
            try:
                key_char = key.char.lower()
                self.key_state.discard(key_char)
            except AttributeError:
                pass  # 非字符键忽略

            if key == keyboard.Key.esc:
                self.logger.info("按下了 Esc 键。正在退出...")
                # 触发 ROS2 shutdown
                rclpy.shutdown()
        except Exception as e:
            self.logger.error(f"处理按键释放事件时出错: {e}")

    def toggle_mode(self):
        """切换控制模式"""
        with self.mode_lock:
            prev_mode = self.mode
            if self.mode == self.MODE_AUTO:
                self.mode = self.MODE_MANUAL
                self.logger.info("\n\n")
                self.logger.info(f"****切换到 MANUAL 模式 (键盘控制)**** (从 {prev_mode} 模式)")
                # 重置上次发送的命令
                self.last_sent_commands['steering_angle_deg'] = None
                self.last_sent_commands['speed'] = None
            else:
                self.mode = self.MODE_AUTO
                self.logger.info(f"****切换到 AUTO 模式**** (从 {prev_mode} 模式)")

    def periodic_time_sync(self):
        """定期进行时间同步"""
        while not self.stop_event.is_set():
            # 如果同步失败，则触发安全停车并切换到手动模式
            if self.sync_failed:
                self.logger.error("时间同步失败超过最大尝试次数，切换到手动模式并停车")
                self.safety_stop_and_switch_to_manual()
                # 重置同步失败标志
                self.sync_failed = False
                self.sync_attempts = 0
                self.sync_failure_count = 0
                time.sleep(0.05)
            else:
                self.sync_time()
                time.sleep(10)

    def safety_stop_and_switch_to_manual(self):
        """安全停车并切换到手动模式"""
        self.logger.warning("执行安全停车并切换到手动模式")
        
        # 发送停止命令
        self.send_stop_command()
        
        # 切换到手动模式
        with self.mode_lock:
            prev_mode = self.mode
            if self.mode != self.MODE_MANUAL:
                self.mode = self.MODE_MANUAL
                self.logger.info("\n\n")
                self.logger.info(f"****因时间同步失败，自动切换到 MANUAL 模式**** (从 {prev_mode} 模式)")
                
                # 重置命令
                with self.cmd_lock:
                    self.speed = 0.0
                    self.steering_angle_deg = 0.0
                    self.last_sent_commands['speed'] = None
                    self.last_sent_commands['steering_angle_deg'] = None
        
        # 记录此事件
        self.log_timing_event("TIME_SYNC_FAILED_SAFETY_STOP", 0, 0, 0, 0.0, 0.0)

    def get_timestamp(self):
        """获取相对时间戳（毫秒）"""
        current_time = int(time.time() * 1000)
        relative_time = current_time - self.time_base
        return relative_time

    def log_timing_event(self, event_type, pc_time, arduino_time, delay_ms, 
                         steering_angle=0.0, speed=0.0, mapped_steering=0.0, mapped_speed=0.0, reason=""):
        """记录时间戳事件到日志文件"""
        try:
            with self.log_lock:
                with open(self.timing_log_path, 'a') as f:
                    timestamp = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
                    if reason:
                        f.write(f"{timestamp},{event_type},{pc_time},{arduino_time},{delay_ms},{steering_angle},{speed},{mapped_steering},{mapped_speed},{reason}\n")
                    else:
                        f.write(f"{timestamp},{event_type},{pc_time},{arduino_time},{delay_ms},{steering_angle},{speed},{mapped_steering},{mapped_speed}\n")
            
            # 记录到详细通信日志
            log_message = f"事件: {event_type}, PC时间: {pc_time}, Arduino时间: {arduino_time}, 延迟: {delay_ms}ms, " + \
                         f"转向角: {steering_angle}, 速度: {speed}, 映射转向: {mapped_steering}, 映射速度: {mapped_speed}"
            if reason:
                log_message += f", 原因: {reason}"
            self.logger.debug(log_message)
        except Exception as e:
            self.logger.error(f"写入时间戳日志时出错: {e}")

    def sync_time(self):
        """与Arduino同步时间"""
        try:
            # 增加同步尝试次数
            self.sync_attempts += 1
            
            current_time = self.get_timestamp()
            
            # 打包时间同步消息
            packed_msg = struct.pack('<BI', self.TIME_SYNC_HEADER, current_time)
            
            # 添加校验和
            checksum = 0
            for byte in packed_msg:
                checksum ^= byte
            
            # 完整消息
            complete_msg = packed_msg + bytes([checksum])
            
            # 记录发送详情
            hex_msg = " ".join(f"{byte:02X}" for byte in complete_msg)
            self.logger.debug(f"发送时间同步请求 HEX: {hex_msg}")
            
            # 发送消息
            self.ser.write(complete_msg)
            self.ser.flush()
            
            self.logger.info(f"发送时间同步请求: {current_time} ms (相对时间戳), 尝试次数: {self.sync_attempts}/{self.sync_max_attempts}")
            self.log_timing_event("SYNC_REQUEST", current_time, 0, 0)
            self.last_sync_time = current_time
            
            # 记录发送时间
            with self.time_sync_lock:
                self.command_timestamps['sync'] = current_time
                
            # 设置同步超时定时器
            sync_timeout_timer = threading.Timer(self.sync_timeout, self.handle_sync_timeout)
            sync_timeout_timer.daemon = True
            sync_timeout_timer.start()
                
        except Exception as e:
            self.logger.error(f"时间同步出错: {e}")
            self.handle_sync_timeout()

    def handle_sync_timeout(self):
        """处理同步超时情况"""
        with self.time_sync_lock:
            # 检查是否已经收到同步响应
            if not self.time_synced and self.sync_attempts >= self.sync_max_attempts:
                self.logger.error(f"时间同步超时，尝试次数: {self.sync_attempts}/{self.sync_max_attempts}")
                self.sync_failed = True
                self.log_timing_event("SYNC_TIMEOUT", self.last_sync_time, 0, 0)

    def calculate_checksum(self, msg_type, steering_tire_angle, speed):
        # 打包数据为字节
        data = struct.pack('<Bff', msg_type, steering_tire_angle, speed)
        checksum = 0
        for byte in data:
            checksum ^= byte
        return checksum

    def calculate_checksum_with_timestamp(self, msg_type, timestamp, steering_tire_angle, speed):
        # 打包数据为字节
        data = struct.pack('<BIff', msg_type, timestamp, steering_tire_angle, speed)
        checksum = 0
        for byte in data:
            checksum ^= byte
        return checksum

    def verify_checksum(self, data):
        if len(data) < 11:
            self.logger.info("Invalid checksum")
            return False
        try:
            _, msg_type, steering_tire_angle, speed, recv_checksum = struct.unpack('<BBffB', data[:11])
            calculated_checksum = self.calculate_checksum(msg_type, steering_tire_angle, speed)
            return calculated_checksum == recv_checksum
        except struct.error:
            return False

    def listener_callback(self, msg):
        # 检查模式
        with self.mode_lock:
            if self.mode != self.MODE_AUTO:
                self.logger.debug(f"忽略ROS消息 - 当前模式: {self.mode}")
                return

        # 转换数据
        steering_tire_angle_deg = round(math.degrees(msg.lateral.steering_tire_angle), 3)
        speed = round(msg.longitudinal.speed, 3)
        acceleration = msg.longitudinal.acceleration
        
        self.logger.debug(f"ROS消息 - 转向角度: {msg.lateral.steering_tire_angle}弧度/{steering_tire_angle_deg}度, " +
                        f"速度: {speed}m/s, 加速度: {acceleration}m/s²")
        
        # 触发停止命令的条件
        if acceleration < -1.1 or (self.last_sent_commands['speed'] == speed and 
                self.last_sent_commands['steering_angle_deg'] == steering_tire_angle_deg): 
            if self.stop_flag == False:
                self.logger.debug(f"加速度<-1.1, autoware 控制触发停止命令 - 加速度: {acceleration}, 重复命令: {self.last_sent_commands['speed'] == speed and self.last_sent_commands['steering_angle_deg'] == steering_tire_angle_deg}")
                self.send_stop_command() 
                self.stop_flag = True
                return

        # 判断是否有显著变化
        if (self.pre_steering_tire_angle is None or
            self.pre_speed is None or
            abs(steering_tire_angle_deg - self.pre_steering_tire_angle) > ANG_THRESHLOD or
            abs(speed - self.pre_speed) > SPEED_THRESHOLD):

            self.logger.info('接收到 AckermannControlCommand 消息:')
            self.logger.info(f'  转向角度: {msg.lateral.steering_tire_angle} 弧度')
            self.logger.info(f'  速度: {msg.longitudinal.speed} m/s')
            self.logger.info(f'  加速度: {msg.longitudinal.acceleration} m/s²')

            self.pre_steering_tire_angle = steering_tire_angle_deg
            self.pre_speed = speed
            
            # 再次检查模式
            with self.mode_lock:
                if self.mode != self.MODE_AUTO:
                    self.logger.debug("模式已变更，取消发送命令")
                    return
            
            # 判断是否与上次发送的命令不同，避免重复发送
            if (self.last_sent_commands['speed'] != speed or
                self.last_sent_commands['steering_angle_deg'] != steering_tire_angle_deg):
                self.stop_flag = False
                # 保存当前控制值，供其他模式使用
                self.steering_angle_deg = steering_tire_angle_deg
                self.speed = speed
                self.send_command(steering_tire_angle_deg, speed)
        else:
            self.logger.debug(f'cmd变化过小, angle:{abs(steering_tire_angle_deg - self.pre_steering_tire_angle)}, speed:{abs(speed - self.pre_speed)}')

    def send_command(self, steering_angle_deg, speed):
        """统一的命令发送函数"""
        try:
            # 四舍五入到3位小数
            steering_angle_deg = round(steering_angle_deg, 3)
            speed = round(speed, 3)
            current_time = self.get_timestamp()
            
            # 存储当前命令
            with self.last_command_lock:
                self.last_command_store = {
                    'timestamp': current_time,
                    'steering_angle_deg': steering_angle_deg,
                    'speed': speed,
                    'trumpet': self.trumpet_active
                }

            # 计算校验和
            checksum = self.calculate_checksum_with_timestamp(self.MSG_TYPE_COMMAND, current_time, steering_angle_deg, speed)

            # 打包消息，增加喇叭状态字节（默认为0）
            packed_msg = struct.pack('<BBIffB', 0x42, self.MSG_TYPE_COMMAND, 
                                    current_time, steering_angle_deg, speed, 
                                    0x00)  # 喇叭关闭
            
            # 添加校验和
            packed_msg += bytes([checksum])
            
            # 记录发送事件
            command_type = "STOP_CMD_SENT" if speed == 0 and steering_angle_deg == 0 else "CMD_SENT"
            self.log_timing_event(command_type, current_time, 0, 0, steering_angle_deg, speed)
            
            # 发送消息
            bytes_written = self.ser.write(packed_msg)
            self.ser.flush()
            
            # 记录详情
            hex_msg = " ".join(f"{byte:02X}" for byte in packed_msg)
            
            # 增强命令日志输出，包含更多详细信息
            cmd_info = "停止命令" if (speed == 0 and steering_angle_deg == 0) else "控制命令"
            source = ""
            with self.mode_lock:
                if self.mode == self.MODE_AUTO:
                    source = "自动控制"
                elif self.mode == self.MODE_MANUAL:
                    source = "键盘控制"
                elif self.mode == self.MODE_WHEEL:
                    source = "方向盘控制"
            
            # 记录字节格式和命令来源
            self.logger.debug(f"发送{cmd_info} HEX: {hex_msg}")
            self.logger.info(f"发送到串口 ({source}) - 时间戳: {current_time} ms, 速度: {speed} m/s, 转向角度: {steering_angle_deg} 度, 字节: {bytes_written}")

            # 更新上次发送的命令
            self.last_sent_commands['speed'] = speed
            self.last_sent_commands['steering_angle_deg'] = steering_angle_deg
            return True
                
        except Exception as e:
            self.logger.error(f"发送命令时发生错误: {e}")
            return False
            
    def send_stop_command(self):
        """发送停止命令: 速度=0, 转向角=0"""
        self.send_command(0.0, 0.0)
        # 重置前一个状态
        self.logger.warning(" !!!/n The parking command is triggered, please check whether an error occurs /n !!!")
        with self.cmd_lock:
            self.pre_steering_tire_angle = 0
            self.pre_speed = 0

    def read_from_serial(self):
        """串口读取线程"""
        self.logger.debug("串口读取线程启动")
        while not self.stop_event.is_set() and self.ser.is_open:
            try:
                if self.ser.in_waiting > 0:
                    # 读取头部字节
                    header_byte = self.ser.read(1)
                    if not header_byte:
                        continue
                        
                    header = header_byte[0]
                    self.logger.debug(f"收到消息头: 0x{header:02X}")
                    
                    if header == 0x42:  # 控制命令响应
                        response_data = header_byte + self.ser.read(10)
                        if len(response_data) == 11:
                            hex_data = " ".join(f"{byte:02X}" for byte in response_data)
                            self.logger.debug(f"收到控制响应 HEX: {hex_data}")
                            self.data_queue.put(response_data)
                        else:
                            self.logger.warning(f"控制响应数据不完整: {len(response_data)}字节")
                    
                    elif header == self.TIME_SYNC_HEADER:  # 时间同步响应
                        response_data = header_byte + self.ser.read(10)
                        if len(response_data) == 11:
                            hex_data = " ".join(f"{byte:02X}" for byte in response_data)
                            self.logger.debug(f"收到时间同步响应 HEX: {hex_data}")
                            self.process_time_sync_response(response_data)
                        else:
                            self.logger.warning(f"时间同步响应数据不完整: {len(response_data)}字节")
                    
                    elif header == self.ACK_HEADER:  # ACK响应
                        response_data = header_byte + self.ser.read(10)
                        if len(response_data) == 11:
                            hex_data = " ".join(f"{byte:02X}" for byte in response_data)
                            self.logger.debug(f"收到ACK响应 HEX: {hex_data}")
                            self.process_ack_response(response_data)
                        else:
                            self.logger.warning(f"ACK响应数据不完整: {len(response_data)}字节")
                    
                    elif header == self.SYNC_FAILURE_HEADER:  # 同步失败响应
                        response_data = header_byte + self.ser.read(6)
                        if len(response_data) == 7:
                            hex_data = " ".join(f"{byte:02X}" for byte in response_data)
                            self.logger.debug(f"收到同步失败响应 HEX: {hex_data}")
                            self.process_sync_failure_response(response_data)
                        else:
                            self.logger.warning(f"同步失败响应数据不完整: {len(response_data)}字节")
                    elif header == self.DECODING_STATUS_HEADER:  # Decoding status response
                        response_data = header_byte + self.ser.read(15)
                        if len(response_data) == 16:
                            hex_data = " ".join(f"{byte:02X}" for byte in response_data)
                            self.logger.debug(f"收到解码状态响应 HEX: {hex_data}")
                            self.process_decoding_status(response_data)
                        else:
                            self.logger.warning(f"解码状态响应数据不完整: {len(response_data)}字节")
                    else:
                        # 未知消息类型
                        self.logger.warning(f"收到未知消息头: 0x{header:02X}")
                        if self.ser.in_waiting > 0:
                            self.logger.debug(f"清空缓冲区: {self.ser.in_waiting} 字节")
                        self.ser.read(self.ser.in_waiting)  # 清空缓冲区
                else:
                    time.sleep(0.01)  # 减少CPU占用
            except Exception as e:
                self.logger.error(f"读取串口时发生错误: {e}")
                break
    def process_decoding_status(self, data):
        """处理解码状态信息"""
        try:
            if len(data) < 16 or self.stop_event.is_set():
                self.logger.warning(f"解码状态数据不完整: {len(data)}字节 或节点正在关闭")
                return
                
            # 解析解码状态信息
            status_flag = data[1]
            orig_steering = struct.unpack('<f', data[2:6])[0]
            orig_speed = struct.unpack('<f', data[6:10])[0]
            mapped_steering = data[10]
            mapped_speed = data[11]
            trumpet_status = data[12]
            
            # 检查校验和
            checksum = 0
            for i in range(1, 14):
                checksum ^= data[i]
                
            if checksum != data[14]:
                self.logger.warning("解码状态消息校验和不正确")
                return
                
            # 仅在调试级别记录解码细节
            self.logger.debug(f"解码状态: 原始转向={orig_steering:.3f}, 原始速度={orig_speed:.3f}, " +
                            f"映射转向={mapped_steering}, 映射速度={mapped_speed}, " +
                            f"喇叭状态={'开启' if trumpet_status else '关闭'}")
            
            # 记录到时间戳日志
            self.log_timing_event("DECODING_STATUS", 0, 0, 0, orig_steering, orig_speed, 
                                mapped_steering, mapped_speed, f"Status: {status_flag}")
                                
        except Exception as e:
            self.logger.error(f"解析解码状态响应失败: {e}")
    def process_sync_failure_response(self, data):
        """处理同步失败响应"""
        try:
            if len(data) < 7 or self.stop_event.is_set():
                self.logger.warning(f"同步失败响应数据不完整: {len(data)}字节 或节点正在关闭")
                return
                
            # 解析同步失败响应
            failure_reason = data[1]
            arduino_time = data[2] | (data[3] << 8) | (data[4] << 16) | (data[5] << 24)
            
            reason_text = "未知"
            if failure_reason == 0x01:
                reason_text = "超时"
            elif failure_reason == 0x02:
                reason_text = "缓冲区满"
                
            self.logger.error(f"时间同步失败 - 原因: {reason_text} (0x{failure_reason:02X}), Arduino时间: {arduino_time} ms")
            
            # a增加失败计数
            with self.time_sync_lock:
                self.sync_failure_count += 1
                
                if self.sync_failure_count >= self.max_sync_failures:
                    self.logger.critical(f"检测到多次同步失败 ({self.sync_failure_count})。激活紧急安全程序。")
                    self.sync_failed = True
                
            # 记录事件
            self.log_timing_event("SYNC_FAILURE", 0, arduino_time, 0, 0.0, 0.0, 0.0, 0.0, reason_text)
                
        except Exception as e:
            self.logger.error(f"解析同步失败响应出错: {e}")

    def process_time_sync_response(self, data):
        """处理时间同步响应"""
        try:
            if len(data) < 11 or self.stop_event.is_set():
                self.logger.warning(f"时间同步响应数据不完整: {len(data)} bytes 或节点正在关闭")
                return
                
            # 解析时间同步响应
            sync_type = data[1]
            pc_time = data[2] | (data[3] << 8) | (data[4] << 16) | (data[5] << 24)
            arduino_time = data[6] | (data[7] << 8) | (data[8] << 16) | (data[9] << 24)
            
            # 计算偏移量
            offset = pc_time - arduino_time
            
            self.logger.debug(f"解析同步响应 - 类型: 0x{sync_type:02X}, PC时间: {pc_time}, Arduino时间: {arduino_time}, 偏移: {offset}")
            
            if sync_type == 0x01:  # 同步确认
                with self.time_sync_lock:
                    self.time_synced = True
                    self.pc_arduino_offset = offset
                    # 重置同步计数器
                    self.sync_attempts = 0
                    self.sync_failed = False
                    self.sync_failure_count = 0
                    
                current_time = self.get_timestamp()
                round_trip = current_time - self.last_sync_time
                
                self.logger.info(f"时间同步成功 - PC时间: {pc_time} ms, Arduino时间: {arduino_time} ms, 偏移量: {offset} ms, 延迟: {round_trip} ms")
                
                # 记录时间同步事件
                self.log_timing_event("SYNC_RESPONSE", pc_time, arduino_time, round_trip)
                
        except Exception as e:
            self.logger.error(f"解析时间同步响应失败: {e}")
            # 增加同步尝试失败次数
            with self.time_sync_lock:
                if self.sync_attempts >= self.sync_max_attempts:
                    self.sync_failed = True

    def process_ack_response(self, data):
        """处理ACK响应"""
        try:
            if len(data) < 11 or self.stop_event.is_set():
                self.logger.warning(f"ACK响应数据不完整: {len(data)} bytes 或节点正在关闭")
                return
                
            # 解析ACK响应
            ack_type = data[1]
            pc_timestamp = data[2] | (data[3] << 8) | (data[4] << 16) | (data[5] << 24)
            arduino_time = data[6] | (data[7] << 8) | (data[8] << 16) | (data[9] << 24)
            
            # 计算延迟
            current_relative_time = self.get_timestamp()
            delay = current_relative_time - pc_timestamp
            
            self.logger.debug(f"解析ACK响应 - 类型: 0x{ack_type:02X}, PC时间: {pc_timestamp}, Arduino时间: {arduino_time}, 延迟: {delay}ms")
            
            if ack_type == self.ACK_RECEIVED:
                self.logger.info(f"命令接收确认 - PC时间戳: {pc_timestamp} ms, Arduino时间: {arduino_time} ms, 延迟: {delay} ms")
                self.log_timing_event("CMD_RECEIVED", pc_timestamp, arduino_time, delay)
                
            elif ack_type == self.ACK_SENT:
                self.logger.info(f"命令发送确认 - PC时间戳: {pc_timestamp} ms, Arduino时间: {arduino_time} ms, 总延迟: {delay} ms")
                self.log_timing_event("CMD_SENT_SUCCESS", pc_timestamp, arduino_time, delay)
                
        except Exception as e:
            self.logger.error(f"解析ACK响应失败: {e}")

    def process_serial_data(self):
        """处理从串口读取的数据"""
        self.logger.debug("数据处理线程启动")
        while not self.stop_event.is_set():
            try:
                response = self.data_queue.get(timeout=0.1)
                if len(response) >= 11 and response[0] == 0x42:
                    self.logger.debug(f"处理队列数据: {' '.join(f'{byte:02X}' for byte in response)}")
                    if self.verify_checksum(response):
                        try:
                            _, msg_type, mapped_steering, mapped_speed, _ = struct.unpack('<BBffB', response[:11])
                            
                            # 四舍五入
                            mapped_steering = round(mapped_steering, 3)
                            mapped_speed = round(mapped_speed, 3)
                            
                            # 获取最近发送的原始命令
                            original_steering = 0.0
                            original_speed = 0.0
                            timestamp = 0
                            
                            with self.last_command_lock:
                                if self.last_command_store:
                                    original_steering = self.last_command_store.get('steering_angle_deg', 0.0)
                                    original_speed = self.last_command_store.get('speed', 0.0)
                                    timestamp = self.last_command_store.get('timestamp', 0)
                            
                            self.logger.debug(f"解析控制响应 - 类型: 0x{msg_type:02X}, 映射转向: {mapped_steering}, 映射速度: {mapped_speed}")
                            self.logger.debug(f"原始命令数据 - 时间戳: {timestamp}, 转向: {original_steering}, 速度: {original_speed}")
                            
                            if msg_type == 0x02:
                                self.logger.info(f"Arduino 发送正确: 映射后的转向角度={mapped_steering}, 映射后的速度={mapped_speed}")
                                self.log_timing_event("RADIO_SENT_SUCCESS", timestamp, 0, 0, 
                                                     original_steering, original_speed, mapped_steering, mapped_speed)
                            elif msg_type == 0x03:
                                self.logger.warning(f"!!!!!! Arduino 发送错误!!!!!!!\n映射后的转向角度={mapped_steering}, 映射后的速度={mapped_speed}")
                                self.log_timing_event("RADIO_SENT_FAIL", timestamp, 0, 0, 
                                                     original_steering, original_speed, mapped_steering, mapped_speed)
                            elif msg_type == 0x05:
                                self.logger.error("!!!!!! Arduino 5s 内未接收到消息!!!!!!!")
                                self.log_timing_event("TIMEOUT", timestamp, 0, 0)
                            elif msg_type == 0x06:  # 处理同步超时状态码
                                self.logger.error("!!!!!! Arduino 同步请求超时!!!!!!!")
                                self.log_timing_event("SYNC_TIMEOUT_ARDUINO", timestamp, 0, 0)
                                
                                # 增加同步失败计数
                                with self.time_sync_lock:
                                    self.sync_failure_count += 1
                                    if self.sync_failure_count >= self.max_sync_failures:
                                        self.sync_failed = True
                        except Exception as e:
                            self.logger.error(f"解析响应数据失败: {e}")
                    else:
                        self.logger.error("!!!!!! Arduino 发送的校验和不正确!!!!!!!")
                else:
                    self.logger.error("!!!!!! Arduino 发送的消息格式不正确!!!!!!!")
            except queue.Empty:
                continue  # 没有数据，继续等待
            except Exception as e:
                self.logger.error(f"处理串口数据时发生错误: {e}")

    def main_loop(self):
        """主循环，处理键盘输入并发送数据到串口。以特定频率发送方向盘控制指令。"""
        self.logger.debug("主循环线程启动")
        
        # 计算25Hz对应的循环间隔时间（秒）
        wheel_control_interval = 1.0 / 25.0  # 25Hz = 0.04秒每次
        last_wheel_control_time = time.time()
        
        while rclpy.ok() and not self.stop_event.is_set():
            current_time = time.time()
            
            # 检查当前模式
            with self.mode_lock:
                current_mode = self.mode
                    
            if current_mode == self.MODE_MANUAL:
                # 根据按键更新速度和转向角
                with self.cmd_lock:
                    new_speed = self.speed
                    new_steering_angle_deg = self.steering_angle_deg

                    # 处理速度
                    if 'w' in self.key_state:
                        new_speed = min(self.speed + 0.1, 1.8)
                        self.logger.debug(f"W键按下 - 增加速度到: {new_speed}")
                    elif 's' in self.key_state:
                        new_speed = max(self.speed - 0.1, -1.8)
                        self.logger.debug(f"S键按下 - 减少速度到: {new_speed}")
                    else:
                        new_speed = 0  # 无按键时速度为0
                        if self.speed != 0:
                            self.logger.debug(f"无速度按键 - 重置速度为0")

                    # 处理转向
                    if 'a' in self.key_state:
                        new_steering_angle_deg = min(self.steering_angle_deg + 4.58, 34.38)
                        self.logger.debug(f"A键按下 - 左转到: {new_steering_angle_deg}")
                    elif 'd' in self.key_state:
                        new_steering_angle_deg = max(self.steering_angle_deg - 4.58, -34.38)
                        self.logger.debug(f"D键按下 - 右转到: {new_steering_angle_deg}")
                    else:
                        # 松开转向键时转向角回正
                        if self.steering_angle_deg > 0:
                            new_steering_angle_deg = max(self.steering_angle_deg - 5.73, 0)
                            self.logger.debug(f"无转向按键 - 转向回正到: {new_steering_angle_deg}")
                        elif self.steering_angle_deg < 0:
                            new_steering_angle_deg = min(self.steering_angle_deg + 5.73, 0)
                            self.logger.debug(f"无转向按键 - 转向回正到: {new_steering_angle_deg}")

                    # 四舍五入到3位小数
                    new_speed = round(new_speed, 3)
                    new_steering_angle_deg = round(new_steering_angle_deg, 3)

                    # 检查是否有变化
                    if (new_speed != self.speed) or (new_steering_angle_deg != self.steering_angle_deg):
                        self.logger.debug(f"手动控制值更新 - 速度: {self.speed} -> {new_speed}, 转向: {self.steering_angle_deg} -> {new_steering_angle_deg}")
                        self.speed = new_speed
                        self.steering_angle_deg = new_steering_angle_deg

                        # 判断是否与上次发送的命令不同
                        if (self.last_sent_commands['speed'] != self.speed or
                            self.last_sent_commands['steering_angle_deg'] != self.steering_angle_deg):

                            # 再次检查模式
                            with self.mode_lock:
                                if self.mode == self.MODE_MANUAL:
                                    # 如果喇叭激活，发送带喇叭的命令
                                    if self.trumpet_active:
                                        self.send_command_with_trumpet(self.steering_angle_deg, self.speed, True)
                                    else:
                                        self.send_command(self.steering_angle_deg, self.speed)
            
            # 在方向盘模式下，按照固定频率发送控制指令
            elif current_mode == self.MODE_WHEEL:
                # 以25Hz频率更新控制值，无论值是否变化
                if current_time - last_wheel_control_time >= wheel_control_interval:
                    self.update_wheel_control(force_send=True)
                    last_wheel_control_time = current_time
            
            # 短暂睡眠，减轻CPU负担
            # 使用更短的时间间隔，确保不会错过发送时机
            time.sleep(0.01)  # 10ms，远小于25Hz的40ms周期

    def stop_and_exit(self):
        """停止所有线程并清理资源"""
        self.logger.info("正在停止节点并清理资源...")
        self.send_stop_command()
        self.stop_event.set()
        
        self.logger.debug("等待线程终止...")
        if self.read_thread.is_alive():
            self.read_thread.join(timeout=1)
            self.logger.debug("串口读取线程已终止" if not self.read_thread.is_alive() else "串口读取线程终止超时")
            
        if self.process_thread.is_alive():
            self.process_thread.join(timeout=1)
            self.logger.debug("数据处理线程已终止" if not self.process_thread.is_alive() else "数据处理线程终止超时")
            
        if self.sync_time_thread.is_alive():
            self.sync_time_thread.join(timeout=1)
            self.logger.debug("时间同步线程已终止" if not self.sync_time_thread.is_alive() else "时间同步线程终止超时")
        
        if hasattr(self, 'wheel_status_thread') and self.wheel_status_thread and self.wheel_status_thread.is_alive():
            self.wheel_status_thread.join(timeout=1)
            self.logger.debug("方向盘状态线程已终止" if not self.wheel_status_thread.is_alive() else "方向盘状态线程终止超时")
        
        if hasattr(self, 'wheel_thread') and self.wheel_thread and self.wheel_thread.is_alive():
            self.wheel_thread.join(timeout=1)
            self.logger.debug("方向盘事件线程已终止" if not self.wheel_thread.is_alive() else "方向盘事件线程终止超时")
            
        # 关闭方向盘设备
        if hasattr(self, 'wheel_device') and self.wheel_device:
            try:
                self.wheel_device.close()
                self.logger.info("已关闭方向盘设备")
            except Exception as e:
                self.logger.error(f"关闭方向盘设备出错: {e}")
            
        if hasattr(self, 'ser') and self.ser and self.ser.is_open:
            self.ser.close()
            self.logger.info(f"已关闭串口: {self.ser.port}")
            
        self.logger.info("节点清理完成")

    def destroy_node(self):
        """重写Node的destroy_node方法，确保资源被正确释放"""
        self.logger.info("正在销毁节点并发送停止命令...")
        self.stop_and_exit()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)

    node = None
    try:
        node = IntegratedControlPublisher()
    except Exception as e:
        logging.getLogger(__name__).error(f"节点初始化时发生未预期的错误: {e}")
        sys.exit(1)

    # 启动主循环线程
    main_loop_thread = threading.Thread(target=node.main_loop, daemon=True)
    main_loop_thread.start()

    # 显示提示信息
    print("\n========== Integrated Control Publisher ==========")
    print(f"当前模式: {node.mode}")
    print("控制指令:")
    print("  - 切换控制模式:")
    print("      * 按 'm' 切换到键盘控制模式 (AUTO <-> MANUAL)")
    print("      * R2按钮 切换到方向盘控制模式 (WHEEL)")
    print("      * R3按钮 从方向盘控制模式返回自动模式 (AUTO)")
    print("      * F7键 也可以切换到方向盘控制模式")
    print("      * F8键 也可以切换到自动控制模式")
    print("  - 键盘控制模式:")
    print("      * 'w' 增加速度")
    print("      * 's' 减少速度")
    print("      * 'a' 向左转")
    print("      * 'd' 向右转")
    print("  - 方向盘控制模式:")
    print("      * 使用方向盘控制转向")
    print("      * 使用油门和刹车踏板控制速度")
    print("      * 换挡杆前推控制后退，后推控制前进")
    print("      * ENTER按钮控制喇叭")
    print("  - 通用控制:")
    print("      * 'Space' 设置速度=0且转向角=0")
    print("      * 'F5' 键手动触发时间同步")
    print("      * 'F6' 键手动触发nRF初始化")
    print("      * 'Esc' 键退出程序")
    print(f"时间戳日志将记录到: {os.path.abspath(node.timing_log_path)}")
    print("===================================================\n")

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('接收到键盘中断信号 (Ctrl+C)，正在退出...')
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()
        sys.exit(0)

if __name__ == '__main__':
    main()