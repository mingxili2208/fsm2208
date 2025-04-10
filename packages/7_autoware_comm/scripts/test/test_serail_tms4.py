#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from autoware_auto_control_msgs.msg import AckermannControlCommand
from std_msgs.msg import String
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
from pynput import keyboard

class IntegratedControlPublisher(Node):

    MODE_AUTO = 'AUTO'
    MODE_MANUAL = 'MANUAL'

    # 新增数据包类型常量
    TIME_SYNC_HEADER = 0x43
    ACK_HEADER = 0x44
    ACK_RECEIVED = 0x10
    ACK_SENT = 0x11
    SYNC_FAILURE_HEADER = 0x45  # 新增：同步失败消息的头部标识

    def __init__(self):
        super().__init__('integrated_control_publisher')

        # 创建日志目录
        log_dir = "logs"
        if not os.path.exists(log_dir):
            os.makedirs(log_dir)
        
        # 使用当前日期和时间创建日志文件名
        timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        
        # 配置详细通信日志文件
        comm_log_filename = os.path.join(log_dir, f"comm_details_{timestamp}.log")
        # 配置文件处理器
        file_handler = logging.FileHandler(comm_log_filename)
        file_handler.setLevel(logging.DEBUG)
        file_format = logging.Formatter('%(asctime)s - %(levelname)s - %(message)s')
        file_handler.setFormatter(file_format)
        
        # 配置控制台处理器
        console_handler = logging.StreamHandler()
        console_handler.setLevel(logging.INFO)
        console_handler.setFormatter(file_format)
        
        # 配置根日志记录器
        self.logger = logging.getLogger(__name__)
        self.logger.setLevel(logging.DEBUG)  # 设置为DEBUG级别以捕获所有日志
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

        self.stop_flag=False
        # 声明并获取参数
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('timeout', 0.5)  # 超时时间，单位：秒
        self.declare_parameter('sync_max_attempts', 3)  # 最大同步尝试次数
        self.declare_parameter('sync_timeout', 0.05)  # 同步超时时间(秒)

        port = self.get_parameter('port').get_parameter_value().string_value
        baudrate = self.get_parameter('baudrate').get_parameter_value().integer_value
        timeout = self.get_parameter('timeout').get_parameter_value().double_value
        self.sync_max_attempts = self.get_parameter('sync_max_attempts').get_parameter_value().integer_value
        self.sync_timeout = self.get_parameter('sync_timeout').get_parameter_value().double_value
        self.MSG_TYPE_COMMAND = 0x01
        
        # 记录参数信息
        self.logger.info(f"参数配置: 端口={port}, 波特率={baudrate}, 超时={timeout}秒")
        self.logger.info(f"同步配置: 最大尝试次数={self.sync_max_attempts}, 超时={self.sync_timeout}秒")
        
        # 初始化串口
        try:
            self.ser = serial.Serial(port, baudrate, timeout=timeout)
            if self.ser.is_open:
                self.logger.info(f"成功打开串口: {port}")
            else:
                self.logger.error(f"无法打开串口: {port}")
                raise serial.SerialException(f"无法打开串口: {port}")
        except serial.SerialException as e:
            self.logger.error(f"打开串口时发生错误: {e}")
            # 抛出异常，让主函数处理
            raise e

        time.sleep(2)  # 等待串口稳定

        # 初始化前一个角度和速度
        self.pre_steering_tire_angle = None
        self.pre_speed = None

        # 创建一个队列用于接收数据
        self.data_queue = queue.Queue()

        # 定义事件以控制线程关闭
        self.stop_event = threading.Event()

        # 初始化命令变量及锁
        self.cmd_lock = threading.Lock()
        self.steering_angle_deg = 0.0
        self.speed = 0.0

        # 当前模式，默认自动模式
        self.mode = self.MODE_AUTO
        self.mode_lock = threading.RLock()  # 专用于模式切换的可重入锁
        self.logger.info("当前模式: AUTO")

        # 时间戳相关变量
        self.time_base = int(time.time() * 1000)  # 程序启动时的基准时间
        self.time_sync_lock = threading.Lock()
        self.time_synced = False
        self.pc_arduino_offset = 0  # PC时间与Arduino时间的偏移量
        self.last_sync_time = 0     # 上次同步时间
        self.command_timestamps = {}  # 存储命令发送时间戳
        self.sync_attempts = 0      # 同步尝试次数
        self.sync_failed = False    # 同步是否失败的标志
        self.sync_failure_count = 0  # 同步失败计数器
        self.max_sync_failures = 3  # 连续同步失败的最大次数，超过则进行紧急停车

        # 新增: 存储最近发送的控制命令及其时间戳，用于与接收到的映射值关联
        self.last_command_store = {}
        self.last_command_lock = threading.Lock()

        # 启动串口读取线程
        self.read_thread = threading.Thread(target=self.read_from_serial, daemon=True)
        self.read_thread.start()
        self.logger.debug("串口读取线程已启动")

        # 启动数据处理线程
        self.process_thread = threading.Thread(target=self.process_serial_data, daemon=True)
        self.process_thread.start()
        self.logger.debug("数据处理线程已启动")

        # 创建ROS订阅者
        self.subscription = self.create_subscription(
            AckermannControlCommand,
            '/control/command/control_cmd',
            self.listener_callback,
            1)
        self.subscription  # 防止未使用变量警告
        self.logger.debug("已创建ROS订阅: /control/command/control_cmd")

        # 添加新的发布者，用于发布接收到的串口数据
        self.received_data_publisher = self.create_publisher(String, '/serial/received_data', 10)
        self.logger.debug("已创建ROS发布者: /serial/received_data")
        
        # 添加新的发布者，用于发布时间戳信息
        self.timing_publisher = self.create_publisher(String, '/serial/timing_info', 10)
        self.logger.debug("已创建ROS发布者: /serial/timing_info")

        # 初始化键盘监听相关变量
        self.key_state = set()
        self.keyboard_thread = threading.Thread(target=self.keyboard_input_listener, daemon=True)
        self.keyboard_thread.start()
        self.logger.debug("键盘监听线程已启动")

        # 记录上次发送的命令，避免重复发送
        self.last_sent_commands = {
            'steering_angle_deg': None,
            'speed': None
        }
        
        # 启动时进行时间同步
        self.sync_time_thread = threading.Thread(target=self.periodic_time_sync, daemon=True)
        self.sync_time_thread.start()
        self.logger.debug("定期时间同步线程已启动")

        # 发送初始停止命令
        self.logger.info("发送初始停止命令")
        self.send_stop_command()

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
                time.sleep(0.05)  # 等待一段时间后再尝试同步
            else:
                self.sync_time()
                time.sleep(0.05)  # 每10秒同步一次时间

    def safety_stop_and_switch_to_manual(self):
        """安全停车并切换到手动模式"""
        self.logger.warning("执行安全停车并切换到手动模式")
        
        # 首先发送停止命令
        self.send_stop_command()
        
        # 切换到手动模式
        with self.mode_lock:
            if self.mode != self.MODE_MANUAL:
                self.mode = self.MODE_MANUAL
                self.logger.info("\n\n")
                self.logger.info("****因时间同步失败，自动切换到 MANUAL 模式****\n\n")
                
                # 重置命令
                with self.cmd_lock:
                    self.speed = 0.0
                    self.steering_angle_deg = 0.0
                    # 重置上次发送的命令以允许重新发送
                    self.last_sent_commands['speed'] = None
                    self.last_sent_commands['steering_angle_deg'] = None
        
        # 发布警告消息
        warning_msg = String()
        warning_msg.data = "警告: 时间同步失败! 已切换到手动模式并停车。"
        self.received_data_publisher.publish(warning_msg)
        self.timing_publisher.publish(warning_msg)
        
        # 确保日志记录此事件
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
                    # 如果提供了原因，则将其添加到日志中
                    if reason:
                        f.write(f"{timestamp},{event_type},{pc_time},{arduino_time},{delay_ms},{steering_angle},{speed},{mapped_steering},{mapped_speed},{reason}\n")
                    else:
                        f.write(f"{timestamp},{event_type},{pc_time},{arduino_time},{delay_ms},{steering_angle},{speed},{mapped_steering},{mapped_speed}\n")
            
            # 同时记录到详细通信日志
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
            
            current_time = self.get_timestamp()  # 相对时间戳
            
            # 打包时间同步消息
            packed_msg = struct.pack('<BI', self.TIME_SYNC_HEADER, current_time)
            
            # 添加校验和
            checksum = 0
            for byte in packed_msg:
                checksum ^= byte
            
            # 完整消息
            complete_msg = packed_msg + bytes([checksum])
            
            # 记录发送详情 - 十六进制格式
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
            self.handle_sync_timeout()  # 如果发送同步请求出错，也视为同步失败

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
        data = struct.pack('<Bff',  msg_type, steering_tire_angle, speed)
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
        # 使用专用的模式锁检查模式
        with self.mode_lock:
            if self.mode != self.MODE_AUTO:
                # 如果当前不是自动模式，则忽略ROS消息
                self.logger.debug(f"忽略ROS消息 - 当前模式: {self.mode}")
                return

        # 将转向角度从弧度转换为度，并四舍五入到3位小数
        steering_tire_angle_deg = round(math.degrees(msg.lateral.steering_tire_angle), 3)
        speed = round(msg.longitudinal.speed, 3)
        acceleration=msg.longitudinal.acceleration
        
        # 记录接收到的ROS消息详情
        self.logger.debug(f"ROS消息 - 转向角度: {msg.lateral.steering_tire_angle}弧度/{steering_tire_angle_deg}度, " +
                        f"速度: {speed}m/s, 加速度: {acceleration}m/s²")
        
        if acceleration<-1.1 or (self.last_sent_commands['speed'] == speed and
                self.last_sent_commands['steering_angle_deg'] == steering_tire_angle_deg): 
            if self.stop_flag==False:
                self.logger.debug(f"触发停止命令 - 加速度: {acceleration}, 重复命令: {self.last_sent_commands['speed'] == speed and self.last_sent_commands['steering_angle_deg'] == steering_tire_angle_deg}")
                self.send_stop_command() 
                self.stop_flag=True
        # 判断是否有显著变化
        if (self.pre_steering_tire_angle is None or
            self.pre_speed is None or
            abs(steering_tire_angle_deg - self.pre_steering_tire_angle) > 0.5 or
            abs(speed - self.pre_speed) > 0.1):

            self.logger.info('接收到 AckermannControlCommand 消息:')
            self.logger.info(f'  转向角度: {msg.lateral.steering_tire_angle} 弧度')
            self.logger.info(f'  速度: {msg.longitudinal.speed} m/s')
            self.logger.info(f'  加速度: {msg.longitudinal.acceleration} m/s²')

            self.pre_steering_tire_angle = steering_tire_angle_deg
            self.pre_speed = speed
            self.logger.debug(f"当前转向角度: {steering_tire_angle_deg} 度, 速度: {speed} m/s")
            
            # 获取当前时间戳
            
            # 再次检查模式，确保期间没有切换
            with self.mode_lock:
                if self.mode != self.MODE_AUTO:
                    self.logger.debug("模式已变更，取消发送命令")
                    return
            
            # 判断是否与上次发送的命令不同，避免重复发送
            if (self.last_sent_commands['speed'] != speed or
                self.last_sent_commands['steering_angle_deg'] != steering_tire_angle_deg):
                self.stop_flag=False
                current_time = self.get_timestamp()
            
                # 计算校验和
                checksum = self.calculate_checksum_with_timestamp(self.MSG_TYPE_COMMAND, current_time, steering_tire_angle_deg, speed)

                # 打包消息，包含时间戳
                try:
                    packed_msg = struct.pack('<BBIffB', 0x42, self.MSG_TYPE_COMMAND, 
                                        current_time,  # 添加4字节时间戳
                                        steering_tire_angle_deg, speed, checksum)
                    hex_msg = " ".join(f"{byte:02X}" for byte in packed_msg)
                    self.logger.debug(f"打包消息 HEX: {hex_msg}")
                except struct.error as e:
                    self.logger.error(f"打包消息时发生错误: {e}")
                    return
                # 存储当前命令，用于后续与映射值关联
                with self.last_command_lock:
                    self.last_command_store = {
                        'timestamp': current_time,
                        'steering_angle_deg': steering_tire_angle_deg,
                        'speed': speed
                    }
                
                # 记录发送事件
                self.log_timing_event("CMD_SENT", current_time, 0, 0, steering_tire_angle_deg, speed)
                
                # 发送消息
                try:
                    bytes_written = self.ser.write(packed_msg)
                    self.logger.info(f"send_result: {bytes_written} bytes written")
                    self.ser.flush()  # 确保所有数据都已发送
                except serial.SerialException as e:
                    self.logger.error(f"写入串口时发生错误: {e}")
                    return

                # 在主脚本输出当前发送的speed和angle
                self.logger.info(f"发送到串口 - 时间戳: {current_time} ms, 速度: {speed} m/s, 转向角度: {steering_tire_angle_deg} 度")

                # 更新上次发送的命令
                self.last_sent_commands['speed'] = speed
                self.last_sent_commands['steering_angle_deg'] = steering_tire_angle_deg

                # 发布发送的命令信息到ROS2主题
                if not self.stop_event.is_set():
                    sent_msg = String()
                    sent_msg.data = f"Sent - 时间戳: {current_time} ms, 速度: {speed} m/s, 转向角度: {steering_tire_angle_deg} 度"
                    self.received_data_publisher.publish(sent_msg)    
    
    def calculate_checksum_with_timestamp(self, msg_type, timestamp, steering_tire_angle, speed):
        # 打包数据为字节
        data = struct.pack('<BIff', msg_type, timestamp, steering_tire_angle, speed)
        checksum = 0
        for byte in data:
            checksum ^= byte
        return checksum

    def read_from_serial(self):
        # 移除文件logging，改为只将数据放入队列
        self.logger.debug("串口读取线程启动")
        while not self.stop_event.is_set() and self.ser.is_open:
            try:
                if self.ser.in_waiting > 0:
                    # 首先读取第一个字节来确定消息类型
                    header_byte = self.ser.read(1)
                    if not header_byte:
                        continue
                        
                    header = header_byte[0]
                    self.logger.debug(f"收到消息头: 0x{header:02X}")
                    
                    if header == 0x42:  # 原始控制命令响应
                        # 读取剩余10个字节
                        response_data = header_byte + self.ser.read(10)
                        if len(response_data) == 11:
                            # 记录原始十六进制数据
                            hex_data = " ".join(f"{byte:02X}" for byte in response_data)
                            self.logger.debug(f"收到控制响应 HEX: {hex_data}")
                            self.data_queue.put(response_data)
                        else:
                            self.logger.warning(f"控制响应数据不完整: {len(response_data)}字节")
                    
                    elif header == self.TIME_SYNC_HEADER:  # 时间同步响应
                        # 读取剩余10个字节 (1字节类型 + 4字节PC时间戳 + 4字节Arduino时间戳 + 1字节校验)
                        response_data = header_byte + self.ser.read(10)
                        if len(response_data) == 11:
                            # 记录原始十六进制数据
                            hex_data = " ".join(f"{byte:02X}" for byte in response_data)
                            self.logger.debug(f"收到时间同步响应 HEX: {hex_data}")
                            self.process_time_sync_response(response_data)
                        else:
                            self.logger.warning(f"时间同步响应数据不完整: {len(response_data)}字节")
                    
                    elif header == self.ACK_HEADER:  # ACK响应
                        # 读取剩余10个字节 (1字节类型 + 4字节时间戳 + 4字节Arduino本地时间 + 1字节校验)
                        response_data = header_byte + self.ser.read(10)
                        if len(response_data) == 11:
                            # 记录原始十六进制数据
                            hex_data = " ".join(f"{byte:02X}" for byte in response_data)
                            self.logger.debug(f"收到ACK响应 HEX: {hex_data}")
                            self.process_ack_response(response_data)
                        else:
                            self.logger.warning(f"ACK响应数据不完整: {len(response_data)}字节")
                    
                    elif header == self.SYNC_FAILURE_HEADER:  # 同步失败响应
                        # 读取剩余6个字节 (1字节原因 + 4字节Arduino时间 + 1字节校验)
                        response_data = header_byte + self.ser.read(6)
                        if len(response_data) == 7:
                            # 记录原始十六进制数据
                            hex_data = " ".join(f"{byte:02X}" for byte in response_data)
                            self.logger.debug(f"收到同步失败响应 HEX: {hex_data}")
                            self.process_sync_failure_response(response_data)
                        else:
                            self.logger.warning(f"同步失败响应数据不完整: {len(response_data)}字节")
                    
                    else:
                        # 未知消息类型，尝试读取并丢弃一些数据以重新同步
                        self.logger.warning(f"收到未知消息头: 0x{header:02X}")
                        # 清空缓冲区前记录
                        if self.ser.in_waiting > 0:
                            self.logger.debug(f"清空缓冲区: {self.ser.in_waiting} 字节")
                        self.ser.read(self.ser.in_waiting)  # 清空缓冲区
                else:
                    time.sleep(0.01)  # 减少CPU占用
            except serial.SerialException as e:
                self.logger.error(f"读取串口时发生错误: {e}")
                break

    def process_sync_failure_response(self, data):
        """处理同步失败响应"""
        try:
            if len(data) < 7 or self.stop_event.is_set():
                self.logger.warning(f"同步失败响应数据不完整: {len(data)}字节 或节点正在关闭")
                return
                
            # 解析同步失败响应 (字节顺序为小端序)
            failure_reason = data[1]
            
            # 提取Arduino时间戳 (4字节, 小端序)
            arduino_time = data[2] | (data[3] << 8) | (data[4] << 16) | (data[5] << 24)
            
            reason_text = "未知"
            if failure_reason == 0x01:
                reason_text = "超时"
            elif failure_reason == 0x02:
                reason_text = "缓冲区满"
                
            self.logger.error(f"时间同步失败 - 原因: {reason_text} (0x{failure_reason:02X}), Arduino时间: {arduino_time} ms")
            
            # 增加失败计数
            with self.time_sync_lock:
                self.sync_failure_count += 1
                
                if self.sync_failure_count >= self.max_sync_failures:
                    self.logger.critical(f"检测到多次同步失败 ({self.sync_failure_count})。激活紧急安全程序。")
                    self.sync_failed = True
                
            # 记录带时间信息的事件
            self.log_timing_event("SYNC_FAILURE", 0, arduino_time, 0, 0.0, 0.0, 0.0, 0.0, reason_text)
            
            # 发布失败消息
            if not self.stop_event.is_set():
                failure_msg = String()
                failure_msg.data = f"时间同步失败 - 原因: {reason_text}, Arduino时间: {arduino_time} ms"
                self.timing_publisher.publish(failure_msg)
                
        except Exception as e:
            self.logger.error(f"解析同步失败响应出错: {e}")

    def process_time_sync_response(self, data):
        """处理时间同步响应"""
        try:
            if len(data) < 11 or self.stop_event.is_set():
                self.logger.warning(f"时间同步响应数据不完整: {len(data)} bytes 或节点正在关闭")
                return
                
            # 解析时间同步响应 (字节顺序为小端序)
            sync_type = data[1]
            
            # 提取PC时间戳 (4字节, 小端序)
            pc_time = data[2] | (data[3] << 8) | (data[4] << 16) | (data[5] << 24)
            
            # 提取Arduino时间戳 (4字节, 小端序)
            arduino_time = data[6] | (data[7] << 8) | (data[8] << 16) | (data[9] << 24)
            
            # 在Python端计算偏移量 (PC时间 - Arduino时间)
            offset = pc_time - arduino_time
            
            self.logger.debug(f"解析同步响应 - 类型: 0x{sync_type:02X}, PC时间: {pc_time}, Arduino时间: {arduino_time}, 偏移: {offset}")
            
            if sync_type == 0x01:  # 同步确认
                with self.time_sync_lock:
                    self.time_synced = True
                    self.pc_arduino_offset = offset
                    # 重置同步尝试次数、失败标志和失败计数
                    self.sync_attempts = 0
                    self.sync_failed = False
                    self.sync_failure_count = 0  # 成功同步时重置失败计数
                    
                current_time = self.get_timestamp()
                round_trip = current_time - self.last_sync_time
                
                if not self.stop_event.is_set():
                    sync_msg = String()
                    sync_msg.data = f"时间同步成功 - PC时间: {pc_time} ms, Arduino时间: {arduino_time} ms, 偏移量: {offset} ms, 延迟: {round_trip} ms"
                    self.timing_publisher.publish(sync_msg)
                
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
                
            # 解析ACK响应 (字节顺序为小端序)
            ack_type = data[1]
            
            # 提取PC时间戳 (4字节, 小端序)
            pc_timestamp = data[2] | (data[3] << 8) | (data[4] << 16) | (data[5] << 24)
            
            # 提取Arduino本地时间 (4字节, 小端序)
            arduino_time = data[6] | (data[7] << 8) | (data[8] << 16) | (data[9] << 24)
            
            # 使用相对时间戳计算延迟
            current_relative_time = self.get_timestamp()
            delay = current_relative_time - pc_timestamp
            
            self.logger.debug(f"解析ACK响应 - 类型: 0x{ack_type:02X}, PC时间: {pc_timestamp}, Arduino时间: {arduino_time}, 延迟: {delay}ms")
            
            if ack_type == self.ACK_RECEIVED:
                ack_msg_text = f"命令接收确认 - PC时间戳: {pc_timestamp} ms, Arduino时间: {arduino_time} ms, 延迟: {delay} ms"
                self.logger.info(ack_msg_text)
                
                if not self.stop_event.is_set():
                    ack_msg = String()
                    ack_msg.data = ack_msg_text
                    self.timing_publisher.publish(ack_msg)
                
                # 记录接收确认事件
                self.log_timing_event("CMD_RECEIVED", pc_timestamp, arduino_time, delay)
                
            elif ack_type == self.ACK_SENT:
                ack_msg_text = f"命令发送确认 - PC时间戳: {pc_timestamp} ms, Arduino时间: {arduino_time} ms, 总延迟: {delay} ms"
                self.logger.info(ack_msg_text)
                
                if not self.stop_event.is_set():
                    ack_msg = String()
                    ack_msg.data = ack_msg_text
                    self.timing_publisher.publish(ack_msg)
                
                # 记录发送确认事件
                self.log_timing_event("CMD_SENT_SUCCESS", pc_timestamp, arduino_time, delay)
                
        except Exception as e:
            self.logger.error(f"解析ACK响应失败: {e}")

    def process_serial_data(self):
        self.logger.debug("数据处理线程启动")
        while not self.stop_event.is_set():
            try:
                response = self.data_queue.get(timeout=0.1)  # 等待数据，避免阻塞
                if len(response) >= 11 and response[0] == 0x42:
                    self.logger.debug(f"处理队列数据: {' '.join(f'{byte:02X}' for byte in response)}")
                    if self.verify_checksum(response):
                        try:
                            _, msg_type, mapped_steering, mapped_speed, _ = struct.unpack('<BBffB', response[:11])
                            
                            # Arduino直接发送映射后的值
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
                                log_msg = f"Arduino 发送正确: 映射后的转向角度={mapped_steering}, 映射后的速度={mapped_speed}"
                                self.logger.info(log_msg)
                                # 记录成功事件，同时包含原始值和映射值
                                self.log_timing_event("RADIO_SENT_SUCCESS", timestamp, 0, 0, 
                                                     original_steering, original_speed, mapped_steering, mapped_speed)
                                # 发布到新的ROS2主题
                                if not self.stop_event.is_set():
                                    received_msg = String()
                                    received_msg.data = log_msg
                                    self.received_data_publisher.publish(received_msg)
                            elif msg_type == 0x03:
                                log_msg = f"!!!!!! Arduino 发送错误!!!!!!!\n映射后的转向角度={mapped_steering}, 映射后的速度={mapped_speed}"
                                self.logger.warning(log_msg)
                                # 记录失败事件，同时包含原始值和映射值
                                self.log_timing_event("RADIO_SENT_FAIL", timestamp, 0, 0, 
                                                     original_steering, original_speed, mapped_steering, mapped_speed)
                                # 发布到新的ROS2主题
                                if not self.stop_event.is_set():
                                    received_msg = String()
                                    received_msg.data = log_msg
                                    self.received_data_publisher.publish(received_msg)
                            elif msg_type == 0x05:
                                log_msg = "!!!!!! Arduino 5s 内未接收到消息!!!!!!!"
                                self.logger.error(log_msg)
                                # 记录超时事件
                                self.log_timing_event("TIMEOUT", timestamp, 0, 0)
                                # 发布到新的ROS2主题
                                if not self.stop_event.is_set():
                                    received_msg = String()
                                    received_msg.data = log_msg
                                    self.received_data_publisher.publish(received_msg)
                            elif msg_type == 0x06:  # 新增：处理同步超时状态码
                                log_msg = "!!!!!! Arduino 同步请求超时!!!!!!!"
                                self.logger.error(log_msg)
                                # 记录同步超时事件
                                self.log_timing_event("SYNC_TIMEOUT_ARDUINO", timestamp, 0, 0)
                                # 发布到ROS2主题
                                if not self.stop_event.is_set():
                                    received_msg = String()
                                    received_msg.data = log_msg
                                    self.received_data_publisher.publish(received_msg)
                                    
                                # 增加同步失败计数
                                with self.time_sync_lock:
                                    self.sync_failure_count += 1
                                    if self.sync_failure_count >= self.max_sync_failures:
                                        self.sync_failed = True
                        except struct.error as e:
                            log_msg = f"解析响应数据失败: {e}"
                            self.logger.error(log_msg)
                            # 发布到新的ROS2主题
                            if not self.stop_event.is_set():
                                received_msg = String()
                                received_msg.data = log_msg
                                self.received_data_publisher.publish(received_msg)
                    else:
                        log_msg = "!!!!!! Arduino 发送的校验和不正确!!!!!!!"
                        self.logger.error(log_msg)
                        # 发布到新的ROS2主题
                        if not self.stop_event.is_set():
                            received_msg = String()
                            received_msg.data = log_msg
                            self.received_data_publisher.publish(received_msg)
                else:
                    log_msg = "!!!!!! Arduino 发送的消息格式不正确!!!!!!!"
                    self.logger.error(log_msg)
                    # 发布到新的ROS2主题
                    if not self.stop_event.is_set():
                        received_msg = String()
                        received_msg.data = log_msg
                        self.received_data_publisher.publish(received_msg)
            except queue.Empty:
                continue  # 没有数据，继续等待
            except Exception as e:
                self.logger.error(f"处理串口数据时发生错误: {e}")

    def keyboard_input_listener(self):
        self.logger.debug("键盘输入监听线程启动")
        with keyboard.Listener(on_press=self.on_press, on_release=self.on_release) as listener:
            listener.join()

    def on_press(self, key):
        try:
            key_char = getattr(key, 'char', None)
            key_name = str(key).replace('Key.', '')
            self.logger.debug(f"按键按下: {key_name}")
            
            if key == keyboard.Key.space:
                # 捕捉空格键，设置速度和转向角为0
                self.logger.info("捕捉到空格键。正在发送停止命令...")
                self.send_stop_command()
                with self.cmd_lock:
                    self.speed = 0.0
                    self.steering_angle_deg = 0.0
                    # 重置上次发送的命令以允许重新发送
                    self.last_sent_commands['speed'] = None
                    self.last_sent_commands['steering_angle_deg'] = None
            elif key == keyboard.Key.f5:
                # F5键触发时间同步
                self.logger.info("手动触发时间同步...")
                # 重置同步尝试次数
                self.sync_attempts = 0
                self.sync_failed = False
                self.sync_failure_count = 0
                self.sync_time()
            else:
                try:
                    key_char = key.char.lower()
                    self.key_state.add(key_char)

                    if key_char == 'm':
                        # 切换模式（最高优先级）
                        self.toggle_mode()
                        # 立即发送停止命令
                        self.send_stop_command()
                        self.stop_flag=False
                        with self.cmd_lock:
                            self.speed = 0.0
                            self.steering_angle_deg = 0.0
                            # 重置上次发送的命令以允许重新发送
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
                # 触发 ROS2 shutdown，rclpy.spin 会结束
                rclpy.shutdown()
        except Exception as e:
            self.logger.error(f"处理按键释放事件时出错: {e}")

    def toggle_mode(self):
        # 使用模式专用锁，确保切换模式时的原子性
        with self.mode_lock:
            if self.mode == self.MODE_AUTO:
                self.mode = self.MODE_MANUAL
                self.logger.info("\n\n")
                self.logger.info("****切换到 MANUAL 模式****\n\n")
                # 重置上次发送的命令以便手动模式可以立即发送新指令
                self.last_sent_commands['steering_angle_deg'] = None
                self.last_sent_commands['speed'] = None
            else:
                self.mode = self.MODE_AUTO
                self.logger.info("****切换到 AUTO 模式****\n\n")
                # 不需要重置，因为自动模式依赖ROS消息

    def send_stop_command(self):
        """
        发送停止命令: 速度=0, 转向角=0
        """
        try:
            steering_tire_angle_deg = round(0.0, 3)
            speed = round(0.0, 3)
            current_time = self.get_timestamp()
            
            # 存储停止命令，用于后续与映射值关联
            with self.last_command_lock:
                self.last_command_store = {
                    'timestamp': current_time,
                    'steering_angle_deg': steering_tire_angle_deg,
                    'speed': speed
                }

            self.pre_steering_tire_angle=0
            self.pre_speed=0

            checksum = self.calculate_checksum_with_timestamp(self.MSG_TYPE_COMMAND, current_time, steering_tire_angle_deg, speed)
            packed_msg = struct.pack('<BBIffB', 0x42, self.MSG_TYPE_COMMAND, current_time, steering_tire_angle_deg, speed, checksum)
            
            # 记录十六进制详情
            hex_msg = " ".join(f"{byte:02X}" for byte in packed_msg)
            self.logger.debug(f"发送停止命令 HEX: {hex_msg}")
            self.logger.info(f"发送停止命令 - 时间戳: {current_time}, 速度: 0.0, 转向: 0.0")
            
            # 记录停止命令发送事件
            self.log_timing_event("STOP_CMD_SENT", current_time, 0, 0, 0.0, 0.0)
                
            bytes_written = self.ser.write(packed_msg)
            self.logger.debug(f"停止命令发送结果: {bytes_written} 字节")
            self.ser.flush()
            self.logger.info(f"发送到串口 - 时间戳: {current_time} ms, 速度: {speed} m/s, 转向角度: {steering_tire_angle_deg} 度")

            # 发布发送的停止命令信息到ROS2主题
            if not self.stop_event.is_set():
                sent_msg = String()
                sent_msg.data = f"Sent Stop - 时间戳: {current_time} ms, 速度: {speed} m/s, 转向角度: {steering_tire_angle_deg} 度"
                self.received_data_publisher.publish(sent_msg)

        except serial.SerialException as e:
            self.logger.error(f"发送停止命令时发生串口错误: {e}")
        except struct.error as e:
            self.logger.error(f"打包停止命令时发生错误: {e}")

    def stop_and_exit(self):
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
            
        if self.ser.is_open:
            self.ser.close()
            self.logger.info(f"已关闭串口: {self.ser.port}")
            
        self.logger.info("节点清理完成")

    def destroy_node(self):
        self.logger.info("正在销毁节点并发送停止命令...")
        self.stop_and_exit()
        super().destroy_node()

    def main_loop(self):
        """
        主循环，处理键盘输入并发送数据到串口。
        仅在手动模式下工作。
        """
        self.logger.debug("主循环线程启动")
        while rclpy.ok() and not self.stop_event.is_set():
            # 检查当前模式，仅在手动模式下处理键盘控制
            with self.mode_lock:
                current_mode = self.mode
                
            if current_mode == self.MODE_MANUAL:
                # 根据当前按下的键更新速度和转向角
                with self.cmd_lock:
                    new_speed = self.speed
                    new_steering_angle_deg = self.steering_angle_deg

                    if 'w' in self.key_state:
                        new_speed = min(self.speed + 0.1, 1.8)  # 最大速度限制为1.8
                        self.logger.debug(f"W键按下 - 增加速度到: {new_speed}")
                    elif 's' in self.key_state:
                        new_speed = max(self.speed - 0.1, -1.8)  # 最低速度限制为-1.8
                        self.logger.debug(f"S键按下 - 减少速度到: {new_speed}")
                    else:
                        new_speed = 0 # 无按键时set as 0
                        if self.speed != 0:
                            self.logger.debug(f"无速度按键 - 重置速度为0")

                    if 'a' in self.key_state:
                        new_steering_angle_deg = min(self.steering_angle_deg + 4.58, 34.38)  # 0.6 弧度
                        self.logger.debug(f"A键按下 - 左转到: {new_steering_angle_deg}")
                    elif 'd' in self.key_state:
                        new_steering_angle_deg = max(self.steering_angle_deg - 4.58, -34.38)
                        self.logger.debug(f"D键按下 - 右转到: {new_steering_angle_deg}")
                    else:
                        # 松开转向键时转向角逐渐回到0
                        if self.steering_angle_deg > 0:
                            new_steering_angle_deg = max(self.steering_angle_deg - 5.73, 0)  # ~0.1 弧度
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

                        # 判断是否与上次发送的命令不同，避免重复发送
                        if (self.last_sent_commands['speed'] != self.speed or
                            self.last_sent_commands['steering_angle_deg'] != self.steering_angle_deg):

                            # 再次检查模式，确保发送命令前仍在手动模式
                            with self.mode_lock:
                                if self.mode == self.MODE_MANUAL:
                                    self.send_serial_command(self.steering_angle_deg, self.speed)
                                    self.last_sent_commands['speed'] = self.speed
                                    self.last_sent_commands['steering_angle_deg'] = self.steering_angle_deg
            
            time.sleep(0.05)  # 适当的循环间隔

    def send_serial_command(self, steering_angle_deg, speed):
        try:
            # 对发送的float值进行四舍五入到3位小数
            steering_angle_deg = round(steering_angle_deg, 3)
            speed = round(speed, 3)
            current_time = self.get_timestamp()

            # 存储当前命令，用于后续与映射值关联
            with self.last_command_lock:
                self.last_command_store = {
                    'timestamp': current_time,
                    'steering_angle_deg': steering_angle_deg,
                    'speed': speed
                }

            # 计算校验和
            checksum = self.calculate_checksum_with_timestamp(self.MSG_TYPE_COMMAND, current_time, steering_angle_deg, speed)

            # 打包消息
            packed_msg = struct.pack('<BBIffB', 0x42, 0x01, 
                                    current_time,  # 添加时间戳
                                    steering_angle_deg, speed, checksum)
            
            # 记录十六进制详情
            hex_msg = " ".join(f"{byte:02X}" for byte in packed_msg)
            self.logger.debug(f"发送手动控制命令 HEX: {hex_msg}")

            # 判断是否与上次发送的命令不同，避免重复发送
            if (self.last_sent_commands['speed'] != speed or
                self.last_sent_commands['steering_angle_deg'] != steering_angle_deg):
                
                # 记录手动命令发送事件
                self.log_timing_event("MANUAL_CMD_SENT", current_time, 0, 0, steering_angle_deg, speed)
                
                # 发送消息
                bytes_written = self.ser.write(packed_msg)
                self.logger.debug(f"手动命令发送结果: {bytes_written} 字节")
                self.ser.flush()

                # 在主脚本输出当前发送的speed和angle
                self.logger.info(f"发送到串口 - 时间戳: {current_time} ms, 速度: {speed} m/s, 转向角度: {steering_angle_deg} 度")

                # 发布发送的命令信息到ROS2主题
                if not self.stop_event.is_set():
                    sent_msg = String()
                    sent_msg.data = f"Sent - 时间戳: {current_time} ms, 速度: {speed} m/s, 转向角度: {steering_angle_deg} 度"
                    self.received_data_publisher.publish(sent_msg)
                
                self.pre_steering_tire_angle=0
                self.pre_speed=0
                
                # 更新上次发送的命令
                self.last_sent_commands['speed'] = speed
                self.last_sent_commands['steering_angle_deg'] = steering_angle_deg

        except struct.error as e:
            self.logger.error(f"打包命令时发生错误: {e}")
        except serial.SerialException as e:
            self.logger.error(f"写入串口时发生错误: {e}")

def main(args=None):
    rclpy.init(args=args)

    node = None
    try:
        node = IntegratedControlPublisher()
    except serial.SerialException:
        # 如果串口初始化失败，rclpy已经被关闭，不需要进一步操作
        logging.getLogger(__name__).error("串口初始化失败，程序退出")
        sys.exit(1)
    except Exception as e:
        # 捕捉其他初始化错误
        logging.getLogger(__name__).error(f"节点初始化时发生未预期的错误: {e}")
        sys.exit(1)

    # 启动主循环线程以处理键盘输入
    main_loop_thread = threading.Thread(target=node.main_loop, daemon=True)
    main_loop_thread.start()

    # 显示提示信息
    print("\n========== Integrated Control Publisher ==========")
    print(f"当前模式: {node.mode}")
    print("控制指令:")
    print("  - 按 'm' 切换控制模式 (AUTO <-> MANUAL)")
    print("  - 手动模式下使用以下键盘控制:")
    print("      * 'w' 增加速度")
    print("      * 's' 减少速度")
    print("      * 'a' 向左转")
    print("      * 'd' 向右转")
    print("      * 'Space' 设置速度=0且转向角=0")
    print("  - 按 'F5' 键手动触发时间同步")
    print("  - 按 'Esc' 键退出程序")
    print(f"时间戳日志将记录到: {os.path.abspath(node.timing_log_path)}")
    print("  * 注意: 如果时间同步失败超过最大尝试次数，将自动切换到手动模式并停车")
    print("===================================================\n")

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('接收到键盘中断信号 (Ctrl+C)，正在退出...')
    finally:
        # 无论是通过 Ctrl+C 还是其他方式退出，确保资源被正确释放
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()
        sys.exit(0)  # 确保主线程退出

if __name__ == '__main__':
    main()