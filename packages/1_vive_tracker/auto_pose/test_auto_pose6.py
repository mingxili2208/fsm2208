import sys
import time
import struct
import serial
import threading
import queue
import os
import numpy as np
import matplotlib.pyplot as plt
from datetime import datetime
from pynput import keyboard
from vive_tracker import ViveTrackerModule

# Debug级别控制
DEBUG_LEVEL = {
    "SERIAL_RAW": False,     # 串口原始数据
    "PROTOCOL": False,       # 协议解析详情
    "PERFORMANCE": True,     # 性能监控
    "CONNECTION": True,      # 连接状态
    "SAMPLING": True         # 采样相关信息
}

# 系统状态枚举
class SystemState:
    IDLE = "空闲"
    CALIBRATING = "校准中"
    READY_TO_SAMPLE = "准备采样"
    SAMPLING = "采样中"
    PAUSED = "暂停"
    WAITING_FOR_STILLNESS = "等待静止"
    COLLECTING_STILL_DATA = "采集静止数据"

# 移动检测类
class MovementDetector:
    def __init__(self, movement_threshold=0.02):
        self.movement_threshold = movement_threshold
        self.last_position = None
        self.baseline_position = None
        self.total_distance = 0.0
        self.movement_detected = False
        
    def set_baseline(self, lidar_d2, lidar_d3, tracker_x, tracker_z):
        """设置基线位置"""
        self.baseline_position = {
            'lidar_d2': lidar_d2,
            'lidar_d3': lidar_d3,
            'tracker_x': tracker_x,
            'tracker_z': tracker_z
        }
        self.last_position = self.baseline_position.copy()
        self.total_distance = 0.0
        self.movement_detected = False
        write_log(f"[移动检测] 设置基线位置: D2={lidar_d2:.3f}m, D3={lidar_d3:.3f}m, X={tracker_x:.3f}m, Z={tracker_z:.3f}m", "SAMPLING")
    
    def check_movement(self, lidar_d2, lidar_d3, tracker_x, tracker_z):
        """检查是否发生了显著移动"""
        if self.last_position is None:
            return False, 0.0
        
        # 计算各维度的变化
        lidar_d2_change = abs(lidar_d2 - self.last_position['lidar_d2'])
        lidar_d3_change = abs(lidar_d3 - self.last_position['lidar_d3'])
        tracker_x_change = abs(tracker_x - self.last_position['tracker_x'])
        tracker_z_change = abs(tracker_z - self.last_position['tracker_z'])
        
        # 计算综合移动距离（欧几里得距离）
        lidar_movement = np.sqrt(lidar_d2_change**2 + lidar_d3_change**2)
        tracker_movement = np.sqrt(tracker_x_change**2 + tracker_z_change**2)
        
        # 取两种传感器中的最大移动距离
        total_movement = max(lidar_movement, tracker_movement)
        
        # 检查是否超过阈值
        movement_detected = total_movement >= self.movement_threshold
        
        if movement_detected:
            self.total_distance += total_movement
            self.movement_detected = True
            write_log(f"[移动检测] 检测到移动: 激光={lidar_movement:.3f}m, Tracker={tracker_movement:.3f}m, 总移动={self.total_distance:.3f}m", "SAMPLING")
        
        # 更新最后位置
        self.last_position = {
            'lidar_d2': lidar_d2,
            'lidar_d3': lidar_d3,
            'tracker_x': tracker_x,
            'tracker_z': tracker_z
        }
        
        return movement_detected, total_movement
    
    def reset_movement(self):
        """重置移动检测"""
        self.movement_detected = False
        write_log("[移动检测] 重置移动状态", "SAMPLING")

# 静止数据采集类
class StillDataCollector:
    def __init__(self, collection_duration=5.0, min_samples=20):
        self.collection_duration = collection_duration
        self.min_samples = min_samples
        self.reset()
    
    def reset(self):
        self.start_time = None
        self.samples = []
        self.is_collecting = False
    
    def start_collection(self):
        """开始采集静止数据"""
        self.reset()
        self.start_time = time.time()
        self.is_collecting = True
        write_log(f"[静止采集] 开始采集{self.collection_duration}秒静止数据，请保持不动...", "SAMPLING")
    
    def add_sample(self, timestamp, lidar_d2, lidar_d3, tracker_coord):
        """添加样本数据"""
        if not self.is_collecting:
            return False
        
        elapsed = time.time() - self.start_time
        if elapsed > self.collection_duration:
            return self.finish_collection()
        
        self.samples.append({
            'timestamp': timestamp,
            'lidar_d2': lidar_d2,
            'lidar_d3': lidar_d3,
            'tracker_x': tracker_coord[0],
            'tracker_y': tracker_coord[1],
            'tracker_z': tracker_coord[2],
            'tracker_roll': tracker_coord[3],
            'tracker_yaw': tracker_coord[4],
            'tracker_pitch': tracker_coord[5]
        })
        
        # 显示进度
        if len(self.samples) % 10 == 0:
            progress = (elapsed / self.collection_duration) * 100
            write_log(f"[静止采集] 进度: {progress:.1f}% ({len(self.samples)}个样本)", "SAMPLING")
        
        return False
    
    def finish_collection(self):
        """完成采集并计算平均值"""
        if len(self.samples) < self.min_samples:
            write_log(f"[静止采集] 样本不足({len(self.samples)} < {self.min_samples})，采集失败", "SAMPLING")
            self.reset()
            return False
        
        # 计算各项数据的平均值
        avg_data = {
            'timestamp': np.mean([s['timestamp'] for s in self.samples]),
            'lidar_d2': np.mean([s['lidar_d2'] for s in self.samples]),
            'lidar_d3': np.mean([s['lidar_d3'] for s in self.samples]),
            'tracker_x': np.mean([s['tracker_x'] for s in self.samples]),
            'tracker_y': np.mean([s['tracker_y'] for s in self.samples]),
            'tracker_z': np.mean([s['tracker_z'] for s in self.samples]),
            'tracker_roll': np.mean([s['tracker_roll'] for s in self.samples]),
            'tracker_yaw': np.mean([s['tracker_yaw'] for s in self.samples]),
            'tracker_pitch': np.mean([s['tracker_pitch'] for s in self.samples])
        }
        
        # 计算标准差以评估数据质量
        std_d2 = np.std([s['lidar_d2'] for s in self.samples]) * 1000  # 转换为mm
        std_d3 = np.std([s['lidar_d3'] for s in self.samples]) * 1000
        std_x = np.std([s['tracker_x'] for s in self.samples]) * 1000
        std_z = np.std([s['tracker_z'] for s in self.samples]) * 1000
        
        write_log(f"[静止采集] 采集完成: {len(self.samples)}个样本", "SAMPLING")
        write_log(f"[静止采集] 平均值: D2={avg_data['lidar_d2']:.3f}m, D3={avg_data['lidar_d3']:.3f}m, X={avg_data['tracker_x']:.3f}m, Z={avg_data['tracker_z']:.3f}m", "SAMPLING")
        write_log(f"[静止采集] 标准差: D2={std_d2:.1f}mm, D3={std_d3:.1f}mm, X={std_x:.1f}mm, Z={std_z:.1f}mm", "SAMPLING")
        
        self.is_collecting = False
        return avg_data
    
    def get_progress(self):
        """获取采集进度"""
        if not self.is_collecting or self.start_time is None:
            return 0.0
        elapsed = time.time() - self.start_time
        return min((elapsed / self.collection_duration) * 100, 100.0)

# 校准类
class CalibrationManager:
    def __init__(self, calibration_duration=10.0, min_samples=50):
        self.calibration_duration = calibration_duration
        self.min_samples = min_samples
        self.reset()
    
    def reset(self):
        self.start_time = None
        self.samples = []
        self.is_calibrating = False
        self.baseline_established = False
        self.baseline_values = None
    
    def start_calibration(self):
        """开始校准"""
        self.reset()
        self.start_time = time.time()
        self.is_calibrating = True
        write_log(f"[校准] 开始{self.calibration_duration}秒校准，请保持设备静止...", "SAMPLING")
    
    def add_calibration_sample(self, lidar_d2, lidar_d3, tracker_coord):
        """添加校准样本"""
        if not self.is_calibrating:
            return False
        
        elapsed = time.time() - self.start_time
        if elapsed > self.calibration_duration:
            return self.finish_calibration()
        
        self.samples.append({
            'lidar_d2': lidar_d2,
            'lidar_d3': lidar_d3,
            'tracker_x': tracker_coord[0],
            'tracker_y': tracker_coord[1],
            'tracker_z': tracker_coord[2],
            'tracker_roll': tracker_coord[3],
            'tracker_yaw': tracker_coord[4],
            'tracker_pitch': tracker_coord[5]
        })
        
        # 显示进度
        if len(self.samples) % 20 == 0:
            progress = (elapsed / self.calibration_duration) * 100
            write_log(f"[校准] 进度: {progress:.1f}% ({len(self.samples)}个样本)", "SAMPLING")
        
        return False
    
    def finish_calibration(self):
        """完成校准"""
        if len(self.samples) < self.min_samples:
            write_log(f"[校准] 样本不足({len(self.samples)} < {self.min_samples})，校准失败", "SAMPLING")
            self.reset()
            return False
        
        # 计算基线值
        self.baseline_values = {
            'lidar_d2': np.mean([s['lidar_d2'] for s in self.samples]),
            'lidar_d3': np.mean([s['lidar_d3'] for s in self.samples]),
            'tracker_x': np.mean([s['tracker_x'] for s in self.samples]),
            'tracker_y': np.mean([s['tracker_y'] for s in self.samples]),
            'tracker_z': np.mean([s['tracker_z'] for s in self.samples]),
            'tracker_roll': np.mean([s['tracker_roll'] for s in self.samples]),
            'tracker_yaw': np.mean([s['tracker_yaw'] for s in self.samples]),
            'tracker_pitch': np.mean([s['tracker_pitch'] for s in self.samples])
        }
        
        # 计算稳定性
        std_d2 = np.std([s['lidar_d2'] for s in self.samples]) * 1000
        std_d3 = np.std([s['lidar_d3'] for s in self.samples]) * 1000
        std_x = np.std([s['tracker_x'] for s in self.samples]) * 1000
        std_z = np.std([s['tracker_z'] for s in self.samples]) * 1000
        
        write_log(f"[校准] 校准完成: {len(self.samples)}个样本", "SAMPLING")
        write_log(f"[校准] 基线值: D2={self.baseline_values['lidar_d2']:.3f}m, D3={self.baseline_values['lidar_d3']:.3f}m", "SAMPLING")
        write_log(f"[校准] 基线值: X={self.baseline_values['tracker_x']:.3f}m, Z={self.baseline_values['tracker_z']:.3f}m", "SAMPLING")
        write_log(f"[校准] 稳定性: D2±{std_d2:.1f}mm, D3±{std_d3:.1f}mm, X±{std_x:.1f}mm, Z±{std_z:.1f}mm", "SAMPLING")
        
        self.is_calibrating = False
        self.baseline_established = True
        return True
    
    def get_progress(self):
        """获取校准进度"""
        if not self.is_calibrating or self.start_time is None:
            return 0.0
        elapsed = time.time() - self.start_time
        return min((elapsed / self.calibration_duration) * 100, 100.0)

# 数据存储管理类
class DataManager:
    def __init__(self, data_dir, timestamp):
        self.data_dir = data_dir
        self.timestamp = timestamp
        self.raw_data_file = os.path.join(data_dir, f"sampling_raw_data_{timestamp}.csv")
        self.filtered_data_file = os.path.join(data_dir, f"sampling_filtered_data_{timestamp}.csv")
        self.sampling_points = []
        self.init_csv_files()
    
    def init_csv_files(self):
        """初始化CSV文件"""
        header = "Point_ID,Timestamp,Distance_2,Distance_3,X,Y,Z,Roll,Yaw,Pitch\n"
        
        with open(self.raw_data_file, 'w') as f:
            f.write(header)
        
        with open(self.filtered_data_file, 'w') as f:
            f.write(header)
        
        write_log(f"[数据] 初始化数据文件:", "SAMPLING")
        write_log(f"  原始数据: {self.raw_data_file}", "SAMPLING")
        write_log(f"  滤波数据: {self.filtered_data_file}", "SAMPLING")
    
    def save_sampling_point(self, point_id, raw_data, filtered_data):
        """保存采样点数据"""
        # 保存原始数据
        raw_line = f"{point_id},{raw_data['timestamp']:.3f},{raw_data['lidar_d2']:.6f},{raw_data['lidar_d3']:.6f},"
        raw_line += f"{raw_data['tracker_x']:.6f},{raw_data['tracker_y']:.6f},{raw_data['tracker_z']:.6f},"
        raw_line += f"{raw_data['tracker_roll']:.6f},{raw_data['tracker_yaw']:.6f},{raw_data['tracker_pitch']:.6f}\n"
        
        # 保存滤波数据
        filtered_line = f"{point_id},{filtered_data['timestamp']:.3f},{filtered_data['lidar_d2']:.6f},{filtered_data['lidar_d3']:.6f},"
        filtered_line += f"{filtered_data['tracker_x']:.6f},{filtered_data['tracker_y']:.6f},{filtered_data['tracker_z']:.6f},"
        filtered_line += f"{filtered_data['tracker_roll']:.6f},{filtered_data['tracker_yaw']:.6f},{filtered_data['tracker_pitch']:.6f}\n"
        
        with open(self.raw_data_file, 'a') as f:
            f.write(raw_line)
        
        with open(self.filtered_data_file, 'a') as f:
            f.write(filtered_line)
        
        self.sampling_points.append({
            'point_id': point_id,
            'raw': raw_data,
            'filtered': filtered_data
        })
        
        write_log(f"[数据] 保存采样点 {point_id}: 原始D2={raw_data['lidar_d2']:.3f}m, 滤波D2={filtered_data['lidar_d2']:.3f}m", "SAMPLING")
    
    def get_sampling_count(self):
        """获取已采样点数"""
        return len(self.sampling_points)

# 零飘滤波器类
class NoiseFilter:
    def __init__(self, window_size=20, noise_threshold=0.005):
        self.window_size = window_size
        self.noise_threshold = noise_threshold
        self.data_history = []
        self.baseline_value = None
        self.baseline_samples = 0
        self.baseline_required = 50
    
    def add_sample(self, value):
        self.data_history.append(value)
        if len(self.data_history) > self.window_size:
            self.data_history.pop(0)
        
        if self.baseline_samples < self.baseline_required:
            self.baseline_samples += 1
            if self.baseline_samples == self.baseline_required:
                self.baseline_value = np.mean(self.data_history)
                write_log(f"[滤波] 基线建立完成: {self.baseline_value:.4f}m")
    
    def get_filtered_value(self, current_value):
        if self.baseline_value is None:
            return current_value
        
        if len(self.data_history) >= self.window_size:
            recent_mean = np.mean(self.data_history[-10:])
            if abs(recent_mean - self.baseline_value) < self.noise_threshold:
                self.baseline_value = 0.99 * self.baseline_value + 0.01 * recent_mean
        
        deviation = current_value - self.baseline_value
        if abs(deviation) < self.noise_threshold:
            filtered_value = self.baseline_value + 0.3 * deviation
            return filtered_value
        
        return current_value

# 延迟检测与过滤类
class LatencyFilter:
    def __init__(self, max_acceptable_latency_ms=30):
        self.max_latency = max_acceptable_latency_ms / 1000.0
        self.recent_latencies = []
        self.rejected_count = 0
        self.total_requests = 0
        
    def check_lidar_latency(self, request_time, response_time):
        latency = response_time - request_time
        self.recent_latencies.append(latency)
        self.total_requests += 1
        
        if len(self.recent_latencies) > 20:
            self.recent_latencies.pop(0)
            
        if latency > self.max_latency:
            self.rejected_count += 1
            write_log(f"[延迟] 拒绝高延迟数据: {latency*1000:.1f}ms > {self.max_latency*1000}ms", "DEBUG")
            return False
        return True
    
    def check_tracker_freshness(self, tracker_request_time, tracker_response_time):
        processing_time = tracker_response_time - tracker_request_time
        if processing_time > self.max_latency:
            write_log(f"[延迟] 拒绝过慢tracker数据: 获取耗时{processing_time*1000:.1f}ms", "DEBUG")
            return False
        return True

# 串口缓冲区管理类
class SerialBufferManager:
    def __init__(self, serial_port, expected_packet_size=10):
        self.ser = serial_port
        self.expected_size = expected_packet_size
        self.buffer = bytearray()
        self.sync_lost_count = 0
        self.recovery_attempts = 0
        self.last_flush_time = time.time()
        
    def find_valid_packet(self):
        """在缓冲区中查找有效数据包"""
        while len(self.buffer) >= self.expected_size:
            # 查找帧头 0x55 0x7E
            start_idx = -1
            for i in range(len(self.buffer) - 1):
                if self.buffer[i] == 0x55 and self.buffer[i + 1] == 0x7E:
                    start_idx = i
                    break
            
            if start_idx == -1:
                # 没找到帧头，清空一部分缓冲区
                self.buffer = self.buffer[len(self.buffer)//2:]
                write_log(f"未找到帧头，缓冲区剩余: {len(self.buffer)}字节", "SERIAL_DEBUG")
                return None
            
            # 移除帧头前的无效数据
            if start_idx > 0:
                removed_data = self.buffer[:start_idx]
                write_log(f"移除帧头前无效数据: {' '.join([f'{b:02X}' for b in removed_data])}", "DEBUG")
                self.buffer = self.buffer[start_idx:]
            
            # 检查是否有完整数据包
            if len(self.buffer) >= self.expected_size:
                packet = self.buffer[:self.expected_size]
                
                # 验证帧尾
                if packet[8] == 0x7E and packet[9] == 0x55:
                    # 找到有效数据包
                    self.buffer = self.buffer[self.expected_size:]
                    return bytes(packet)
                else:
                    # 帧尾不匹配，继续查找
                    write_log(f"帧尾不匹配: 期望[7E 55], 实际[{packet[8]:02X} {packet[9]:02X}]", "DEBUG")
                    self.buffer = self.buffer[1:]  # 移除第一个字节继续查找
            else:
                # 数据不够，等待更多数据
                break
        
        return None
    
    def read_packet_with_recovery(self, timeout=0.1):
        """带恢复机制的数据包读取"""
        start_time = time.time()
        
        while time.time() - start_time < timeout:
            try:
                # 读取可用数据
                available = self.ser.in_waiting
                if available > 0:
                    new_data = self.ser.read(available)
                    self.buffer.extend(new_data)
                    write_log(f"读取{len(new_data)}字节，缓冲区总长度: {len(self.buffer)}", "SERIAL_DEBUG")
                
                # 尝试解析数据包
                packet = self.find_valid_packet()
                if packet:
                    self.sync_lost_count = 0
                    self.recovery_attempts = 0
                    return packet
                
                # 缓冲区过载保护
                if len(self.buffer) > 100:
                    write_log(f"缓冲区过载({len(self.buffer)}字节)，执行清理", "DEBUG")
                    self.buffer = self.buffer[-50:]  # 保留最新50字节
                
                time.sleep(0.001)  # 短暂等待
                
            except serial.SerialException as e:
                write_log(f"串口读取异常: {e}", "ERROR")
                return None
        
        # 超时处理
        self.sync_lost_count += 1
        if self.sync_lost_count > 3:
            self.attempt_recovery()
        
        return None
    
    def attempt_recovery(self):
        """连接恢复尝试"""
        self.recovery_attempts += 1
        write_log(f"尝试连接恢复 (第{self.recovery_attempts}次)", "DEBUG")
        
        try:
            # 清空串口缓冲区
            self.ser.reset_input_buffer()
            self.ser.reset_output_buffer()
            self.buffer.clear()
            
            # 发送几个测试请求
            for i in range(3):
                self.send_request()
                time.sleep(0.02)
            
            write_log("执行了恢复操作: 清空缓冲区并发送测试请求", "DEBUG")
            
        except Exception as e:
            write_log(f"恢复操作失败: {e}", "ERROR")
    
    def send_request(self):
        """发送A69数据请求"""
        tx_buf = bytearray([0x55, 0x7E, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x7E, 0x55])
        tx_buf[7] = calculate_checksum_r(tx_buf)
        try:
            self.ser.write(tx_buf)
            self.ser.flush()
            write_log(f"发送请求: {' '.join([f'{b:02X}' for b in tx_buf])}", "SERIAL_DEBUG")
        except Exception as e:
            write_log(f"发送请求失败: {e}", "ERROR")

# 创建数据存储目录
def create_data_directory():
    timestamp = datetime.now().strftime("%m%d_%H%M%S")
    data_dir = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "data", f"sampling_{timestamp}")
    os.makedirs(data_dir, exist_ok=True)
    print(f"[系统] 数据将保存到目录: {data_dir}")
    return data_dir, timestamp

# 配置
SERIAL_PORT = "/dev/ttyUSB0"
BAUD_RATE = 9600
TRACKER_NAME = "tracker_1"

DATA_DIR, TIMESTAMP = create_data_directory()
LOG_FILE = os.path.join(DATA_DIR, "sampling_log.txt")

# 初始化日志
with open(LOG_FILE, "w") as log_file:
    log_file.write(f"日志创建时间: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n")
    log_file.write("=== 采样系统日志 ===\n")

def write_log(message, level="INFO"):
    timestamp = datetime.now().strftime('%H:%M:%S.%f')[:-3]  # 包含毫秒
    log_message = f"[{timestamp}] {message}"
    
    with open(LOG_FILE, "a") as log_file:
        log_file.write(log_message + "\n")
    
    # 根据级别决定是否在控制台显示
    if level in ["IMPORTANT", "ERROR", "SAMPLING"]:
        print(message)
    elif level == "DEBUG" and DEBUG_LEVEL["PROTOCOL"]:
        print(f"[DEBUG] {message}")
    elif level == "SERIAL_DEBUG" and DEBUG_LEVEL["SERIAL_RAW"]:
        print(f"[SERIAL] {message}")

# 初始化设备
write_log("[系统] 初始化 ViveTrackerModule...", "IMPORTANT")
vtm = ViveTrackerModule()
vtm.print_discovered_objects()
tracker = vtm.devices.get(TRACKER_NAME)

if tracker is None:
    write_log(f"[错误] Tracker '{TRACKER_NAME}' 未找到!", "ERROR")
    sys.exit(1)
else:
    write_log(f"[系统] Tracker '{TRACKER_NAME}' 连接成功", "IMPORTANT")

# 初始化串口
try:
    ser = serial.Serial(
        port=SERIAL_PORT, 
        baudrate=BAUD_RATE, 
        timeout=0.1,
        write_timeout=0.1,
        bytesize=serial.EIGHTBITS,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        xonxoff=False,
        rtscts=False,
        dsrdtr=False
    )
    
    # 清空串口缓冲区
    ser.reset_input_buffer()
    ser.reset_output_buffer()
    
    if ser.is_open:
        write_log(f"串口配置: {SERIAL_PORT}, {BAUD_RATE}波特率, 超时{ser.timeout}s", "IMPORTANT")
except serial.SerialException as e:
    write_log(f"无法打开串口 {SERIAL_PORT}: {e}", "ERROR")
    sys.exit(1)

# 全局变量初始化
running = True
current_state = SystemState.IDLE
current_sampling_segment = 1
current_point_id = 1

# 初始化各组件
calibration_manager = CalibrationManager(calibration_duration=10.0)
movement_detector = MovementDetector(movement_threshold=0.02)
still_data_collector = StillDataCollector(collection_duration=5.0)
data_manager = DataManager(DATA_DIR, TIMESTAMP)

# 为每个通道创建独立的滤波器
noise_filters = {
    'd2': NoiseFilter(window_size=20, noise_threshold=0.005),
    'd3': NoiseFilter(window_size=20, noise_threshold=0.005)
}

latency_filter = LatencyFilter(max_acceptable_latency_ms=30)

# A69协议函数
def calculate_checksum_r(data):
    return data[3] ^ data[4] ^ data[5] ^ data[6]

def parse_a69_data_with_debug(response):
    """增强版A69数据解析，包含详细debug信息"""
    
    # 记录接收到的原始数据
    hex_data = ' '.join([f'{b:02X}' for b in response])
    write_log(f"收到原始数据: [{hex_data}] (长度: {len(response)})", "SERIAL_DEBUG")
    
    # 检查数据包长度
    if len(response) != 10:
        write_log(f"数据包长度错误: 期望10字节，实际{len(response)}字节", "DEBUG")
        write_log(f"错误数据详情: {hex_data}", "DEBUG")
        return None
    
    # 检查帧头和帧尾
    if response[0] != 0x55:
        write_log(f"帧头1错误: 期望0x55，实际0x{response[0]:02X}", "DEBUG")
        return None
    if response[1] != 0x7E:
        write_log(f"帧头2错误: 期望0x7E，实际0x{response[1]:02X}", "DEBUG")
        return None
    if response[8] != 0x7E:
        write_log(f"帧尾1错误: 期望0x7E，实际0x{response[8]:02X}", "DEBUG")
        return None
    if response[9] != 0x55:
        write_log(f"帧尾2错误: 期望0x55，实际0x{response[9]:02X}", "DEBUG")
        return None

    # 校验和检查
    checksum = response[7]
    computed_checksum = calculate_checksum_r(response)
    
    if checksum != computed_checksum:
        write_log(f"校验和错误: 计算值0x{computed_checksum:02X}, 接收值0x{checksum:02X}", "DEBUG")
        write_log(f"用于校验的数据: {response[3]:02X} {response[4]:02X} {response[5]:02X} {response[6]:02X}", "DEBUG")
        return None

    # 解析距离数据
    distance_2 = (response[3] << 8) | response[4]
    distance_3 = (response[5] << 8) | response[6]
    
    # 距离合理性检查
    d2_meters = distance_2 / 1000.0
    d3_meters = distance_3 / 1000.0
    
    write_log(f"解析成功: D2={distance_2}({d2_meters:.3f}m), D3={distance_3}({d3_meters:.3f}m)", "SERIAL_DEBUG")
    
    # 检查距离合理性
    if d2_meters < 0.02 or d2_meters > 5.0:
        write_log(f"D2距离异常: {d2_meters:.3f}m 超出合理范围[0.02-5.0]", "DEBUG")
    if d3_meters < 0.02 or d3_meters > 5.0:
        write_log(f"D3距离异常: {d3_meters:.3f}m 超出合理范围[0.02-5.0]", "DEBUG")
    
    return d2_meters, d3_meters

def process_sampling_data(timestamp, distance_2, distance_3, cam_coord):
    """处理采样数据的主函数"""
    global current_state, current_point_id, movement_detector, still_data_collector
    global calibration_manager, noise_filters, data_manager
    
    # 应用零飘滤波
    filtered_d2 = noise_filters['d2'].get_filtered_value(distance_2)
    filtered_d3 = noise_filters['d3'].get_filtered_value(distance_3)
    
    # 更新滤波器状态
    noise_filters['d2'].add_sample(distance_2)
    noise_filters['d3'].add_sample(distance_3)
    
    # 根据当前状态处理数据
    if current_state == SystemState.CALIBRATING:
        # 校准阶段
        calibration_finished = calibration_manager.add_calibration_sample(distance_2, distance_3, cam_coord)
        if calibration_finished:
            # 校准完成，设置移动检测基线
            baseline = calibration_manager.baseline_values
            movement_detector.set_baseline(
                baseline['lidar_d2'], baseline['lidar_d3'],
                baseline['tracker_x'], baseline['tracker_z']
            )
            current_state = SystemState.READY_TO_SAMPLE
            write_log("=== 校准完成，可以开始采样！请开始线性移动，当检测到移动超过0.02m时将提示您静止采样 ===", "SAMPLING")
    
    elif current_state == SystemState.READY_TO_SAMPLE or current_state == SystemState.SAMPLING:
        # 采样阶段 - 检测移动
        movement_detected, movement_distance = movement_detector.check_movement(
            distance_2, distance_3, cam_coord[0], cam_coord[2]
        )
        
        if movement_detected:
            current_state = SystemState.WAITING_FOR_STILLNESS
            write_log(f"=== 检测到移动超过0.02m！请在当前位置保持静止，将进行5秒采样... ===", "SAMPLING")
            still_data_collector.start_collection()
            current_state = SystemState.COLLECTING_STILL_DATA
    
    elif current_state == SystemState.COLLECTING_STILL_DATA:
        # 静止数据采集阶段
        raw_data_point = {
            'timestamp': timestamp,
            'lidar_d2': distance_2,
            'lidar_d3': distance_3,
            'tracker_x': cam_coord[0],
            'tracker_y': cam_coord[1],
            'tracker_z': cam_coord[2],
            'tracker_roll': cam_coord[3],
            'tracker_yaw': cam_coord[4],
            'tracker_pitch': cam_coord[5]
        }
        
        filtered_data_point = {
            'timestamp': timestamp,
            'lidar_d2': filtered_d2,
            'lidar_d3': filtered_d3,
            'tracker_x': cam_coord[0],
            'tracker_y': cam_coord[1],
            'tracker_z': cam_coord[2],
            'tracker_roll': cam_coord[3],
            'tracker_yaw': cam_coord[4],
            'tracker_pitch': cam_coord[5]
        }
        
        collection_finished = still_data_collector.add_sample(timestamp, distance_2, distance_3, cam_coord)
        
        if collection_finished:
            # 获取平均数据
            avg_raw_data = still_data_collector.finish_collection()
            
            if avg_raw_data:
                # 创建滤波后的平均数据
                avg_filtered_data = avg_raw_data.copy()
                avg_filtered_data['lidar_d2'] = noise_filters['d2'].get_filtered_value(avg_raw_data['lidar_d2'])
                avg_filtered_data['lidar_d3'] = noise_filters['d3'].get_filtered_value(avg_raw_data['lidar_d3'])
                
                # 保存采样点
                data_manager.save_sampling_point(current_point_id, avg_raw_data, avg_filtered_data)
                
                write_log(f"=== 采样点 {current_point_id} 完成！继续移动以进行下一个采样点... ===", "SAMPLING")
                current_point_id += 1
                
                # 重置移动检测，准备下一个采样点
                movement_detector.reset_movement()
                current_state = SystemState.SAMPLING
            else:
                write_log("=== 采样失败，请重新移动至下一位置... ===", "SAMPLING")
                current_state = SystemState.SAMPLING

def display_help():
    """显示控制说明"""
    help_text = f"""
=== 线性移动采样系统控制说明 ===
当前状态: {current_state}
当前采样段: {current_sampling_segment}
已采样点数: {data_manager.get_sampling_count()}

基本操作:
  'C' - 开始校准（在开始采样前必须校准）
  'P' - 暂停/继续采样（暂停后移动到新位置，再次按P重新校准并继续）
  'S' - 显示当前状态和统计信息
  'D' - 切换Debug详细信息显示
  'H' - 显示此帮助信息
  'Esc' - 退出程序并保存所有数据

采样流程:
1. 按 'C' 开始校准（设备保持静止10秒）
2. 校准完成后开始线性移动
3. 当移动超过0.02m时，系统提示静止5秒进行采样
4. 采样完成后继续移动，重复步骤3
5. 按 'P' 可暂停，移动到新位置后再按 'P' 会重新校准
================================
    """
    write_log(help_text, "SAMPLING")

def display_status():
    """显示当前状态信息"""
    status_info = f"""
=== 系统状态信息 ===
当前状态: {current_state}
当前采样段: {current_sampling_segment}
已采样点数: {data_manager.get_sampling_count()}

校准状态:
  是否已校准: {'是' if calibration_manager.baseline_established else '否'}
  校准进度: {calibration_manager.get_progress():.1f}%

采样状态:
  当前点ID: {current_point_id}
  静止采集进度: {still_data_collector.get_progress():.1f}%
  总移动距离: {movement_detector.total_distance:.3f}m

滤波器状态:
  D2基线值: {noise_filters['d2'].baseline_value:.4f}m (样本: {noise_filters['d2'].baseline_samples})
  D3基线值: {noise_filters['d3'].baseline_value:.4f}m (样本: {noise_filters['d3'].baseline_samples})

数据文件:
  原始数据: {data_manager.raw_data_file}
  滤波数据: {data_manager.filtered_data_file}
====================
    """
    write_log(status_info, "SAMPLING")

# 增强的串口监听函数
def enhanced_serial_listener():
    global running, current_state, latency_filter
    
    # 创建串口缓冲区管理器
    buffer_manager = SerialBufferManager(ser)
    
    # 性能监控
    request_count = 0
    success_count = 0
    last_stats_time = time.time()
    
    while running:
        if current_state in [SystemState.CALIBRATING, SystemState.READY_TO_SAMPLE, 
                           SystemState.SAMPLING, SystemState.COLLECTING_STILL_DATA]:
            
            request_start_time = time.time()
            request_count += 1
            
            # 发送请求
            buffer_manager.send_request()
            
            # 读取响应
            response = buffer_manager.read_packet_with_recovery(timeout=0.15)
            response_time = time.time()
            
            if response:
                # 检查延迟
                latency = response_time - request_start_time
                if not latency_filter.check_lidar_latency(request_start_time, response_time):
                    write_log(f"因延迟过高拒绝数据: {latency*1000:.1f}ms", "DEBUG")
                    continue
                
                # 解析数据
                parsed_data = parse_a69_data_with_debug(response)
                if parsed_data:
                    success_count += 1
                    distance_2, distance_3 = parsed_data
                    
                    # 获取tracker数据
                    tracker_request_time = time.time()
                    try:
                        cam_coord = tracker.get_pose_euler()
                        tracker_response_time = time.time()
                        
                        if not latency_filter.check_tracker_freshness(tracker_request_time, tracker_response_time):
                            write_log(f"Tracker响应过慢: {(tracker_response_time-tracker_request_time)*1000:.1f}ms", "DEBUG")
                            continue
                        
                        # 处理数据
                        process_sampling_data(response_time, distance_2, distance_3, cam_coord)
                        
                    except Exception as e:
                        write_log(f"获取tracker数据失败: {e}", "ERROR")
            
            # 定期输出统计信息
            current_time = time.time()
            if current_time - last_stats_time > 60:  # 每60秒
                success_rate = (success_count / request_count * 100) if request_count > 0 else 0
                write_log(f"串口统计: 请求{request_count}次, 成功{success_count}次, 成功率{success_rate:.1f}%", "IMPORTANT")
                last_stats_time = current_time

        time.sleep(0.015)

# 启动线程
serial_thread = threading.Thread(target=enhanced_serial_listener, daemon=True)
serial_thread.start()

# 增强的按键监听
def on_press(key):
    global current_state, current_sampling_segment, current_point_id
    global calibration_manager, movement_detector, still_data_collector
    
    try:
        if key.char == 'c' and current_state == SystemState.IDLE:
            # 开始校准
            calibration_manager.start_calibration()
            current_state = SystemState.CALIBRATING
            write_log(f"=== 开始校准（采样段 {current_sampling_segment}），请保持设备静止10秒... ===", "SAMPLING")
            
        elif key.char == 'p':
            # 暂停/继续采样
            if current_state in [SystemState.READY_TO_SAMPLE, SystemState.SAMPLING]:
                current_state = SystemState.PAUSED
                write_log("=== 采样已暂停，您可以移动到新位置，再次按 'P' 将重新校准并继续采样 ===", "SAMPLING")
                
            elif current_state == SystemState.PAUSED:
                # 重新开始新的采样段
                current_sampling_segment += 1
                # 重置校准和检测器
                calibration_manager.reset()
                movement_detector = MovementDetector(movement_threshold=0.02)
                still_data_collector.reset()
                # 重置滤波器
                noise_filters['d2'] = NoiseFilter(window_size=20, noise_threshold=0.005)
                noise_filters['d3'] = NoiseFilter(window_size=20, noise_threshold=0.005)
                
                calibration_manager.start_calibration()
                current_state = SystemState.CALIBRATING
                write_log(f"=== 开始新的采样段 {current_sampling_segment}，重新校准中... ===", "SAMPLING")
                
            elif current_state in [SystemState.COLLECTING_STILL_DATA, SystemState.WAITING_FOR_STILLNESS]:
                write_log("=== 正在采集数据，请等待当前采样点完成后再暂停 ===", "SAMPLING")
                
            else:
                write_log(f"=== 当前状态 {current_state} 无法暂停，请先开始校准 ===", "SAMPLING")
                
        elif key.char == 's':
            # 显示状态
            display_status()
            
        elif key.char == 'd':
            # 切换debug模式
            DEBUG_LEVEL["SERIAL_RAW"] = not DEBUG_LEVEL["SERIAL_RAW"]
            DEBUG_LEVEL["PROTOCOL"] = not DEBUG_LEVEL["PROTOCOL"]
            status = "开启" if DEBUG_LEVEL["SERIAL_RAW"] else "关闭"
            write_log(f"[系统] Debug模式已{status}", "SAMPLING")
            
        elif key.char == 'h':
            # 显示帮助
            display_help()
            
        # 状态提示
        if key.char == 'c' and current_state != SystemState.IDLE:
            write_log(f"=== 当前状态为 {current_state}，无法重新校准 ===", "SAMPLING")
                
    except AttributeError:
        pass

def on_release(key):
    global running
    if key == keyboard.Key.esc:
        write_log("=== 程序退出，保存所有数据... ===", "SAMPLING")
        write_log(f"=== 总共采样了 {data_manager.get_sampling_count()} 个点 ===", "SAMPLING")
        write_log(f"=== 数据已保存到: ===", "SAMPLING")
        write_log(f"  原始数据: {data_manager.raw_data_file}", "SAMPLING")
        write_log(f"  滤波数据: {data_manager.filtered_data_file}", "SAMPLING")
        running = False
        return False

listener = keyboard.Listener(on_press=on_press, on_release=on_release)
listener.start()

# 显示初始控制说明
display_help()

try:
    while running:
        time.sleep(0.1)
except KeyboardInterrupt:
    write_log("=== 接收到 Ctrl+C，程序退出... ===", "SAMPLING")
    running = False
finally:
    if ser and ser.is_open:
        ser.close()
        write_log("[系统] 串口已关闭", "IMPORTANT")