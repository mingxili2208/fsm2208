#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from autoware_auto_control_msgs.msg import AckermannControlCommand
from autoware_auto_vehicle_msgs.msg import Engage
from rosgraph_msgs.msg import Clock
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
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


ANG_THRESHLOD = 0.2
SPEED_THRESHOLD= 0.1
class IntegratedControlPublisher(Node):

    MODE_AUTO = 'AUTO'
    MODE_MANUAL = 'MANUAL'

    # 数据包类型常量
    TIME_SYNC_HEADER = 0x43
    ACK_HEADER = 0x44
    ACK_RECEIVED = 0x10
    ACK_SENT = 0x11
    SYNC_FAILURE_HEADER = 0x45
    MSG_TYPE_COMMAND = 0x01

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
        
        # 初始化ROS消息日志文件
        self.ros_msg_log_path = os.path.join(log_dir, f"ros_messages_{timestamp}.log")
        with open(self.ros_msg_log_path, 'w') as f:
            f.write(f"=== ROS消息日志 - 启动于 {timestamp} ===\n")
            f.write("timestamp,message_type,direction,content\n")
        
        self.logger.info(f"时间戳CSV日志文件: {self.timing_log_path}")
        self.logger.info(f"ROS消息日志文件: {self.ros_msg_log_path}")
        self.log_lock = threading.Lock()
        self.ros_log_lock = threading.Lock()

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
            raise e

        time.sleep(2)  # 等待串口稳定

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
        self.logger.info("当前模式: AUTO")

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

        # Engage状态管理
        self.engage_lock = threading.RLock()
        self.system_preset_engage = False  # 系统预设的engage状态
        self.current_engage = False        # 当前的engage状态
        self.last_engage_source = None     # 最后一次engage状态的来源
        self.last_engage_timestamp = None  # 最后一次engage状态更新的时间戳
        self.latest_clock = None           # 最新的clock消息
        self.clock_received = False        # 是否收到过clock消息
        self.engage_override_active = False  # 是否激活了强制模式
        self.engage_override_type = None    # 强制模式类型: "temporary" 或 "permanent"
        self.shutting_down = False         # 标记节点是否正在关闭

        # QoS配置
        engage_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )

        # 创建Engage发布者和订阅者
        self.engage_publisher = self.create_publisher(
            Engage, "/autoware/engage", engage_qos
        )
        
        self.engage_subscription = self.create_subscription(
            Engage, "/autoware/engage", self.engage_callback, engage_qos
        )
        
        # 创建时钟订阅者
        self.clock_subscription = self.create_subscription(
            Clock, '/clock', self.clock_callback, engage_qos
        )

        # 启动线程
        self.read_thread = threading.Thread(target=self.read_from_serial, daemon=True)
        self.read_thread.start()
        
        self.process_thread = threading.Thread(target=self.process_serial_data, daemon=True)
        self.process_thread.start()

        # 创建ROS订阅者
        self.subscription = self.create_subscription(
            AckermannControlCommand,
            '/control/command/control_cmd',
            self.listener_callback,
            1)
        
        # 初始化键盘监听
        self.key_state = set()
        self.keyboard_thread = threading.Thread(target=self.keyboard_input_listener, daemon=True)
        self.keyboard_thread.start()

        # 记录上次发送的命令
        self.last_sent_commands = {
            'steering_angle_deg': None,
            'speed': None
        }
        
        # 启动时间同步线程
        self.sync_time_thread = threading.Thread(target=self.periodic_time_sync, daemon=True)
        self.sync_time_thread.start()

        # 发送初始停止命令
        self.logger.info("发送初始停止命令")
        self.send_stop_command()

    def log_ros_message(self, message_type, direction, content):
        """记录ROS消息到专用日志文件"""
        try:
            with self.ros_log_lock:
                timestamp = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
                
                # 格式化内容为单行
                formatted_content = str(content).replace('\n', ' ').replace(',', ';')
                
                with open(self.ros_msg_log_path, 'a') as f:
                    f.write(f"{timestamp},{message_type},{direction},{formatted_content}\n")
                
                # 同时输出到控制台和日志
                self.logger.info(f"ROS {message_type} {direction}: {content}")
                
        except Exception as e:
            self.logger.error(f"记录ROS消息日志时出错: {e}")

    def clock_callback(self, msg: Clock):
        """处理/clock话题的回调函数"""
        self.latest_clock = msg.clock
        self.clock_received = True
        self.log_ros_message("Clock", "接收", f"{msg.clock.sec}.{msg.clock.nanosec}")

    def engage_callback(self, msg: Engage):
        """处理/autoware/engage话题的回调函数"""
        # 节点关闭时不处理消息
        if self.shutting_down:
            return
            
        current_time = datetime.datetime.now()
        
        # 记录接收到的engage消息
        self.log_ros_message("Engage", "接收", f"engage={msg.engage}, stamp={msg.stamp.sec}.{msg.stamp.nanosec}")
        
        with self.engage_lock:
            # 检查是否是自己发布的消息
            is_self_published = False
            if self.last_engage_timestamp is not None:
                # 如果时间间隔非常小，可能是自己刚刚发布的
                time_diff = (current_time - self.last_engage_timestamp).total_seconds()
                if time_diff < 0.1:  # 100毫秒内
                    is_self_published = True
                    self.logger.debug(f"检测到自己发布的Engage消息 (时间差: {time_diff:.3f}秒)")
            
            if not is_self_published:
                old_system_preset = self.system_preset_engage
                # 更新系统预设状态
                self.system_preset_engage = msg.engage
                self.last_engage_source = "external"
                self.logger.info(f"收到外部Engage状态更新: {old_system_preset} -> {msg.engage}")
                
                # 如果在强制模式下，不更新当前状态
                if self.engage_override_active and self.engage_override_type == "temporary":
                    self.logger.info(f"当前处于临时强制模式，保持当前Engage状态: {self.current_engage}")
                # 如果在强制永久模式下，并且外部状态变为True，解除强制模式
                elif self.engage_override_active and self.engage_override_type == "permanent" and msg.engage:
                    self.logger.info("外部Engage状态变为True，解除永久强制模式")
                    self.engage_override_active = False
                    self.engage_override_type = None
                    self.current_engage = True
                # 如果在AUTO模式下，更新当前状态以匹配系统预设
                elif self.mode == self.MODE_AUTO and not self.engage_override_active:
                    self.current_engage = msg.engage
                    self.logger.info(f"在AUTO模式下同步当前Engage状态为: {self.current_engage}")
            else:
                self.logger.debug(f"忽略自己发布的Engage消息: {msg.engage}")

    def publish_engage(self, engage_state: bool, reason: str = ""):
        """发布engage命令到Autoware"""
        # 检查节点是否正在关闭
        if self.shutting_down:
            self.logger.warning(f"无法发布engage状态，节点正在关闭")
            return False

        # 检查ROS2上下文是否有效
        if not rclpy.ok():
            self.logger.warning(f"无法发布engage状态: ROS2上下文无效 ({reason})")
            return False
            
        try:
            with self.engage_lock:
                # 规则1: 在系统预设为False的情况下，不得进行engage状态由F向T的转变，除非在强制模式
                if not self.system_preset_engage and engage_state and not self.current_engage and not self.engage_override_active:
                    self.logger.warning(f"系统预设为False时尝试将engage设置为True，操作被拒绝({reason})")
                    return False
                
                msg = Engage()
                
                # 使用clock消息作为时间戳，如果可用
                if self.clock_received and self.latest_clock is not None:
                    msg.stamp = self.latest_clock
                else:
                    # 添加更清晰的消息
                    cur_time = self.get_clock().now().to_msg()
                    self.logger.debug(f"/clock不可用，使用节点时间作为时间戳: {cur_time.sec}.{cur_time.nanosec};({reason})")
                    msg.stamp = cur_time
                
                msg.engage = engage_state
                
                # 更新状态
                old_engage = self.current_engage
                self.current_engage = engage_state
                self.last_engage_source = "self"
                self.last_engage_timestamp = datetime.datetime.now()
                
                # 记录发布的engage消息
                override_info = ""
                if self.engage_override_active:
                    override_info = f", 强制模式: {self.engage_override_type}"
                
                self.log_ros_message("Engage", "发布", 
                                  f"状态: {old_engage} -> {engage_state}, 原因: {reason}, " +
                                  f"系统预设: {self.system_preset_engage}{override_info}")
                
                # 发布消息
                self.engage_publisher.publish(msg)
                return True
        except Exception as e:
            self.logger.error(f"发布engage命令时出错: {e}")
            return False

    def force_engage_temporary(self):
        """临时强制engage为True（不改变系统预设）"""
        with self.engage_lock:
            if not self.current_engage:
                self.logger.info("激活临时强制Engage模式")
                self.engage_override_active = True
                self.engage_override_type = "temporary"
                return self.publish_engage(True, "临时强制激活")
            else:
                self.logger.info("Engage已经是True，无需临时强制")
                return True

    def force_engage_permanent(self):
        """永久强制engage为True（通过循环检测来维持）"""
        with self.engage_lock:
            if not self.current_engage:
                self.logger.info("激活永久强制Engage模式（模拟系统预设为True）")
                self.engage_override_active = True
                self.engage_override_type = "permanent"
                return self.publish_engage(True, "永久强制激活")
            else:
                self.logger.info("Engage已经是True，无需永久强制")
                return True

    def disengage(self):
        """停止engage"""
        with self.engage_lock:
            # 如果当前处于强制模式，解除强制
            if self.engage_override_active:
                self.logger.info(f"解除{self.engage_override_type}强制Engage模式")
                self.engage_override_active = False
                self.engage_override_type = None
            
            if self.current_engage:
                return self.publish_engage(False, "手动关闭")
            else:
                self.logger.info("Engage已经是False，无需操作")
                return True

    def update_engage_based_on_mode(self, new_mode, old_mode):
        """根据模式变化更新engage状态"""
        # 规则2: AUTO -> MANUAL 时，engage从True变为False
        if old_mode == self.MODE_AUTO and new_mode == self.MODE_MANUAL and self.current_engage:
            self.logger.info("从AUTO切换到MANUAL模式，将engage设置为False")
            self.publish_engage(False, "Mode changed from AUTO to MANUAL")
            
            # 解除强制模式（如果有）
            if self.engage_override_active:
                self.logger.info(f"解除{self.engage_override_type}强制Engage模式")
                self.engage_override_active = False
                self.engage_override_type = None
        
        # 规则3: MANUAL -> AUTO 时，如果系统预设为True，则engage从False变为True
        elif old_mode == self.MODE_MANUAL and new_mode == self.MODE_AUTO and self.system_preset_engage and not self.current_engage:
            self.logger.info("从MANUAL切换到AUTO模式，同时系统预设为True，将engage设置为True")
            self.publish_engage(True, "Mode changed from MANUAL to AUTO with system preset True")

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
            old_mode = self.mode
            if self.mode != self.MODE_MANUAL:
                self.mode = self.MODE_MANUAL
                self.logger.info("\n\n")
                self.logger.info("****因时间同步失败，自动切换到 MANUAL 模式****\n\n")
                
                # 重置命令
                with self.cmd_lock:
                    self.speed = 0.0
                    self.steering_angle_deg = 0.0
                    self.last_sent_commands['speed'] = None
                    self.last_sent_commands['steering_angle_deg'] = None
                
                # 更新engage状态
                self.update_engage_based_on_mode(self.MODE_MANUAL, old_mode)
        
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
            # 如果节点关闭，不执行同步
            if self.shutting_down:
                return
                
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
        # 如果节点关闭，不执行回调
        if self.shutting_down:
            return
            
        # 记录接收到的控制命令（简化版本，避免日志过大）
        self.log_ros_message("ControlCmd", "接收", 
                          f"steering={round(msg.lateral.steering_tire_angle, 3)}, " +
                          f"speed={round(msg.longitudinal.speed, 3)}, " +
                          f"accel={round(msg.longitudinal.acceleration, 3)}")

        # 检查模式
        with self.mode_lock:
            if self.mode != self.MODE_AUTO:
                self.logger.debug(f"忽略ROS消息 - 当前模式: {self.mode}")
                return

        # 检查engage状态
        with self.engage_lock:
            if not self.current_engage:
                self.logger.debug("忽略ROS消息 - 当前engage状态为False")
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
                self.send_command(steering_tire_angle_deg, speed)
        else:
            self.logger.debug(f'cmd变化过小, angle:{abs(steering_tire_angle_deg - self.pre_steering_tire_angle)}, speed:{abs(speed - self.pre_speed)}')

    def send_command(self, steering_angle_deg, speed):
        """统一的命令发送函数"""
        try:
            # 如果节点关闭，不执行发送
            if self.shutting_down:
                return
                
            # 四舍五入到3位小数
            steering_angle_deg = round(steering_angle_deg, 3)
            speed = round(speed, 3)
            current_time = self.get_timestamp()
            
            # 存储当前命令
            with self.last_command_lock:
                self.last_command_store = {
                    'timestamp': current_time,
                    'steering_angle_deg': steering_angle_deg,
                    'speed': speed
                }

            # 计算校验和
            checksum = self.calculate_checksum_with_timestamp(self.MSG_TYPE_COMMAND, current_time, steering_angle_deg, speed)

            # 打包消息
            packed_msg = struct.pack('<BBIffB', 0x42, self.MSG_TYPE_COMMAND, 
                                    current_time, steering_angle_deg, speed, checksum)
            
            # 记录发送事件
            command_type = "STOP_CMD_SENT" if speed == 0 and steering_angle_deg == 0 else "CMD_SENT"
            self.log_timing_event(command_type, current_time, 0, 0, steering_angle_deg, speed)
            
            # 发送消息
            bytes_written = self.ser.write(packed_msg)
            self.ser.flush()
            
            # 记录详情
            hex_msg = " ".join(f"{byte:02X}" for byte in packed_msg)
            self.logger.debug(f"发送命令 HEX: {hex_msg}")
            self.logger.info(f"发送到串口 - 时间戳: {current_time} ms, 速度: {speed} m/s, 转向角度: {steering_angle_deg} 度")

            # 更新上次发送的命令
            self.last_sent_commands['speed'] = speed
            self.last_sent_commands['steering_angle_deg'] = steering_angle_deg
                
        except serial.SerialException as e:
            self.logger.error(f"发送命令时发生串口错误: {e}")
        except struct.error as e:
            self.logger.error(f"打包命令时发生错误: {e}")

    def send_stop_command(self):
        """发送停止命令: 速度=0, 转向角=0"""
        self.send_command(0.0, 0.0)
        # 重置前一个状态
        self.logger.warning(" !!!\n The parking command is triggered, please check whether an error occurs \n !!!")
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
                    
                    else:
                        # 未知消息类型
                        self.logger.warning(f"收到未知消息头: 0x{header:02X}")
                        if self.ser.in_waiting > 0:
                            self.logger.debug(f"清空缓冲区: {self.ser.in_waiting} 字节")
                        self.ser.read(self.ser.in_waiting)  # 清空缓冲区
                else:
                    time.sleep(0.01)  # 减少CPU占用
            except serial.SerialException as e:
                self.logger.error(f"读取串口时发生错误: {e}")
                break
            except Exception as e:
                if not self.stop_event.is_set():  # 只有在非关闭状态才记录错误
                    self.logger.error(f"读取串口时发生意外错误: {e}")

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
            if not self.stop_event.is_set():  # 只有在非关闭状态才记录错误
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
                        except struct.error as e:
                            self.logger.error(f"解析响应数据失败: {e}")
                    else:
                        self.logger.error("!!!!!! Arduino 发送的校验和不正确!!!!!!!")
                else:
                    self.logger.error("!!!!!! Arduino 发送的消息格式不正确!!!!!!!")
            except queue.Empty:
                continue  # 没有数据，继续等待
            except Exception as e:
                if not self.stop_event.is_set():  # 只有在非关闭状态才记录错误
                    self.logger.error(f"处理串口数据时发生错误: {e}")

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
                # F6键切换engage状态
                with self.engage_lock:
                    new_engage_state = not self.current_engage
                    if self.publish_engage(new_engage_state, "手动切换"):
                        self.logger.info(f"手动切换engage状态为: {new_engage_state}")
                    else:
                        self.logger.warning(f"无法手动切换engage状态为: {new_engage_state}")
            elif key == keyboard.Key.f7:
                # F7键临时强制engage为True（不改变系统预设）
                self.logger.info("捕捉到F7键，尝试临时强制engage为True")
                if self.force_engage_temporary():
                    self.logger.info("临时强制engage为True成功")
                else:
                    self.logger.warning("临时强制engage为True失败")
            elif key == keyboard.Key.f8:
                # F8键永久强制engage为True（模拟系统预设为True）
                self.logger.info("捕捉到F8键，尝试永久强制engage为True")
                if self.force_engage_permanent():
                    self.logger.info("永久强制engage为True成功")
                else:
                    self.logger.warning("永久强制engage为True失败")
            elif key == keyboard.Key.f9:
                # F9键取消任何强制并设置engage为False
                self.logger.info("捕捉到F9键，取消强制并设置engage为False")
                if self.disengage():
                    self.logger.info("已取消强制并设置engage为False")
                else:
                    self.logger.warning("取消强制失败")
            else:
                try:
                    key_char = key.char.lower()
                    self.key_state.add(key_char)

                    if key_char == 'm':
                        # 切换模式
                        old_mode = self.mode
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
                        
                        # 根据模式变化更新engage状态
                        with self.mode_lock:
                            self.update_engage_based_on_mode(self.mode, old_mode)
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
                # 标记正在关闭
                self.shutting_down = True
                # 停止所有线程
                self.stop_event.set()
                # 使用系统退出而不是ROS2关闭，确保完全退出
                os._exit(0)  # 强制退出，不等待其他线程
        except Exception as e:
            self.logger.error(f"处理按键释放事件时出错: {e}")
    def toggle_mode(self):
        """切换控制模式"""
        with self.mode_lock:
            old_mode = self.mode
            
            if self.mode == self.MODE_AUTO:
                self.mode = self.MODE_MANUAL
                self.logger.info("\n\n")
                self.logger.info("****切换到 MANUAL 模式****\n\n")
                # 重置上次发送的命令
                self.last_sent_commands['steering_angle_deg'] = None
                self.last_sent_commands['speed'] = None
            else:
                self.mode = self.MODE_AUTO
                self.logger.info("****切换到 AUTO 模式****\n\n")
            
            # 根据模式变化更新engage状态
            self.update_engage_based_on_mode(self.mode, old_mode)

    def stop_and_exit(self):
        """停止所有线程并清理资源"""
        self.logger.info("正在停止节点并清理资源...")
        
        # 设置关闭标志
        self.shutting_down = True
        
        # 发送停止命令到Arduino (这个不需要ROS2上下文)
        self.send_stop_command()
        
        # 检查ROS2上下文是否仍然有效，再发布最终的engage=False
        try:
            with self.engage_lock:
                if self.current_engage and rclpy.ok():  # 确保ROS2上下文仍然有效
                    try:
                        self.publish_engage(False, "程序关闭")
                        self.logger.info("程序关闭时将engage设置为False")
                    except Exception as e:
                        self.logger.warning(f"无法在关闭时发布engage状态: {e}")
                else:
                    self.logger.info(f"跳过关闭时的engage发布 - ROS2上下文状态为{rclpy.ok()},engage状态为{self.current_engage}")
        except Exception as e:
            self.logger.warning(f"处理关闭时的engage状态时出错: {e}")
        
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
        """重写Node的destroy_node方法，确保资源被正确释放"""
        self.logger.info("正在销毁节点并发送停止命令...")
        self.stop_and_exit()
        super().destroy_node()

    def main_loop(self):
        """主循环，处理键盘输入并发送数据到串口。仅在手动模式下工作。"""
        self.logger.debug("主循环线程启动")
        while rclpy.ok() and not self.stop_event.is_set():
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
                                    self.send_command(self.steering_angle_deg, self.speed)
            
            time.sleep(0.05)  # 循环间隔

def main(args=None):
    # 初始化ROS2标志
    ros2_initialized = False
    ros2_shutdown_attempted = False
    node = None
    
    # 不配置全局日志，让节点自己处理日志
    try:
        # 初始化ROS2
        rclpy.init(args=args)
        ros2_initialized = True
        
        # 创建节点
        try:
            node = IntegratedControlPublisher()
            
            # 启动主循环线程
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
            print("  - 按 'F6' 键手动切换engage状态")
            print("  - 按 'F7' 键临时强制engage为True (不改变系统预设)")
            print("  - 按 'F8' 键永久强制engage为True (模拟系统预设为True)")
            print("  - 按 'F9' 键取消任何强制并设置engage为False")
            print("  - 按 'Esc' 键退出程序")
            print(f"时间戳日志将记录到: {os.path.abspath(node.timing_log_path)}")
            print(f"ROS消息日志将记录到: {os.path.abspath(node.ros_msg_log_path)}")
            print(f"Engage状态: 系统预设={node.system_preset_engage}, 当前={node.current_engage}")
            print("  * 自动模式切换规则:")
            print("    - AUTO -> MANUAL: engage将设置为False")
            print("    - MANUAL -> AUTO: 若系统预设为True，engage将设置为True")
            print("  * 强制engage规则:")
            print("    - 临时强制: 保持engage为True直到外部系统改变状态")
            print("    - 永久强制: 模拟系统预设为True，直到外部系统将预设设为True")
            print("  * 注意: 如果时间同步失败超过最大尝试次数，将自动切换到手动模式并停车")
            print("===================================================\n")

            # 开始ROS2事件循环
            try:
                rclpy.spin(node)
            except KeyboardInterrupt:
                # 只记录一次KeyboardInterrupt
                node.get_logger().info('接收到键盘中断信号 (Ctrl+C)，正在退出...')
            
        except serial.SerialException as e:
            print(f"串口初始化失败: {e}")
            sys.exit(1)
        except Exception as e:
            print(f"节点初始化时发生未预期的错误: {e}")
            import traceback
            traceback.print_exc()
            sys.exit(1)
            
    except Exception as e:
        print(f"程序启动失败: {e}")
        import traceback
        traceback.print_exc()
    finally:
        # 清理资源
        if node is not None and not node.shutting_down:
            try:
                node.shutting_down = True
                node.destroy_node()
            except Exception as e:
                print(f"销毁节点时出错: {e}")
        
        # 关闭ROS2，仅当之前未尝试关闭时
        if ros2_initialized and not ros2_shutdown_attempted:
            try:
                # 标记为已尝试关闭
                ros2_shutdown_attempted = True
                
                # 检查ROS2上下文是否仍有效
                if rclpy.ok():
                    rclpy.shutdown()
            except Exception as e:
                print(f"ROS2关闭时出错: {e}")
        
        # 确保程序退出
        sys.exit(0)

if __name__ == '__main__':
    main()