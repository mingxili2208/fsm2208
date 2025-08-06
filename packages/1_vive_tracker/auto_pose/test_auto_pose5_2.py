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
    "PERFORMANCE": True,    # 性能监控
    "CONNECTION": True      # 连接状态
}

# 系统状态枚举
class SystemState:
    IDLE = "IDLE"
    RECORDING = "RECORDING"
    PAUSED = "PAUSED"
    TESTING = "TESTING"

# 零飘滤波器类
class NoiseFilter:
    def __init__(self, window_size=20, noise_threshold=0.005):
        self.window_size = window_size
        self.noise_threshold = noise_threshold
        self.data_history = []
        self.baseline_value = None
        self.baseline_samples = 0
        self.baseline_required = 50
    
    def reset_baseline(self):
        """重置基线校准"""
        self.data_history.clear()
        self.baseline_value = None
        self.baseline_samples = 0
        write_log(f"[滤波] 基线已重置，等待重新校准", "IMPORTANT")
    
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
    
    def is_baseline_ready(self):
        """检查基线是否准备就绪"""
        return self.baseline_value is not None

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
            write_log(f"[延迟] 拒绝高延迟数据: {latency*1000:.1f}ms > {self.max_latency*1000}ms", "DEBUG")
            return False
        return True
    
    def check_tracker_freshness(self, tracker_request_time, tracker_response_time):
        processing_time = tracker_response_time - tracker_request_time
        if processing_time > self.max_latency:
            write_log(f"[延迟] 拒绝过慢tracker数据: 获取耗时{processing_time*1000:.1f}ms", "DEBUG")
            return False
        return True
    
    def get_latency_stats(self):
        if self.recent_latencies:
            avg_latency = np.mean(self.recent_latencies) * 1000
            max_latency = np.max(self.recent_latencies) * 1000
            rejection_rate = (self.rejected_count / self.total_requests) * 100 if self.total_requests > 0 else 0
            return f"Avg: {avg_latency:.1f}ms, Max: {max_latency:.1f}ms, Reject: {rejection_rate:.1f}%"
        return "No latency data"

# 增强的零飘验证器类
class ZeroDriftValidator:
    def __init__(self, name=""):
        self.name = name
        self.static_test_data = []
        self.test_duration = 60
        self.start_time = None
        self.is_testing = False
        self.accumulated_time = 0
        self.last_valid_time = None
        self.connection_loss_count = 0
        self.total_disconnection_time = 0
        
    def start_static_test(self):
        self.start_time = time.time()
        self.static_test_data.clear()
        self.is_testing = True
        self.accumulated_time = 0
        self.last_valid_time = self.start_time
        self.connection_loss_count = 0
        self.total_disconnection_time = 0
        write_log(f"[验证-{self.name}] 开始60秒静态零飘测试（支持连接中断后继续）", "IMPORTANT")
        
    def add_static_sample(self, raw_value, filtered_value):
        if not self.is_testing:
            return
            
        current_time = time.time()
        
        if self.last_valid_time and (current_time - self.last_valid_time) > 1.0:
            disconnection_duration = current_time - self.last_valid_time
            self.connection_loss_count += 1
            self.total_disconnection_time += disconnection_duration
            write_log(f"[验证-{self.name}] 检测到连接恢复，断线时长: {disconnection_duration:.1f}秒", "IMPORTANT")
        
        if self.last_valid_time:
            time_since_last = min(current_time - self.last_valid_time, 1.0)
            self.accumulated_time += time_since_last
        
        self.last_valid_time = current_time
        
        if self.accumulated_time >= self.test_duration:
            write_log(f"[验证-{self.name}] 累积测试时间已达到 {self.test_duration}秒，结束测试", "IMPORTANT")
            self.is_testing = False
            self.analyze_static_performance()
            return
        
        self.static_test_data.append({
            'time': self.accumulated_time,
            'raw': raw_value,
            'filtered': filtered_value,
            'real_time': current_time
        })
        
        if len(self.static_test_data) % 200 == 0:
            progress = (self.accumulated_time / self.test_duration) * 100
            remaining = self.test_duration - self.accumulated_time
            write_log(f"[验证-{self.name}] 测试进度: {progress:.1f}% (剩余{remaining:.1f}秒, 样本数:{len(self.static_test_data)})", "IMPORTANT")
                
    def analyze_static_performance(self):
        if len(self.static_test_data) < 50:
            write_log(f"[验证-{self.name}] 测试数据不足({len(self.static_test_data)}个样本)，无法分析")
            return
            
        raw_values = [d['raw'] for d in self.static_test_data]
        filtered_values = [d['filtered'] for d in self.static_test_data]
        
        raw_std = np.std(raw_values) * 1000
        filtered_std = np.std(filtered_values) * 1000
        raw_range = (np.max(raw_values) - np.min(raw_values)) * 1000
        filtered_range = (np.max(filtered_values) - np.min(filtered_values)) * 1000
        
        improvement_std = (raw_std - filtered_std) / raw_std * 100 if raw_std > 0 else 0
        improvement_range = (raw_range - filtered_range) / raw_range * 100 if raw_range > 0 else 0
        
        actual_test_duration = time.time() - self.start_time if self.start_time else 0
        effective_data_rate = len(self.static_test_data) / self.accumulated_time if self.accumulated_time > 0 else 0
        
        report = f"""
=== {self.name} Zero Drift Validation Report ===
Test Configuration:
  Target Duration: {self.test_duration}s
  Effective Duration: {self.accumulated_time:.1f}s
  Actual Duration: {actual_test_duration:.1f}s
  Connection Loss Count: {self.connection_loss_count}
  Total Disconnection Time: {self.total_disconnection_time:.1f}s

Data Statistics:
  Valid Samples: {len(self.static_test_data)}
  Effective Data Rate: {effective_data_rate:.1f} Hz
  Raw Data Std Dev: {raw_std:.2f}mm
  Filtered Data Std Dev: {filtered_std:.2f}mm
  Std Dev Improvement: {improvement_std:.1f}%
  
  Raw Data Range: {raw_range:.2f}mm  
  Filtered Data Range: {filtered_range:.2f}mm
  Range Improvement: {improvement_range:.1f}%
        """
        write_log(report, "IMPORTANT")
        self.plot_drift_comparison()
        return improvement_std > 30
    
    def plot_drift_comparison(self):
        try:
            times = [d['time'] for d in self.static_test_data]
            raw_values = [d['raw']*1000 for d in self.static_test_data]
            filtered_values = [d['filtered']*1000 for d in self.static_test_data]
            
            plt.figure(figsize=(15, 8))
            
            plt.subplot(2, 1, 1)
            plt.plot(times, raw_values, 'b-', alpha=0.7, label='Raw Data', linewidth=1)
            plt.plot(times, filtered_values, 'r-', linewidth=2, label='Filtered Data')
            plt.ylabel('Distance (mm)')
            plt.title(f'{self.name} Drift Processing Comparison (Duration: {self.accumulated_time:.1f}s)')
            plt.legend()
            plt.grid(True, alpha=0.3)
            
            plt.subplot(2, 1, 2)
            differences = [(raw - filtered) * 1000 for raw, filtered in zip([d['raw'] for d in self.static_test_data], [d['filtered'] for d in self.static_test_data])]
            plt.plot(times, differences, 'g-', alpha=0.7, label='Filter Correction', linewidth=1)
            plt.axhline(y=0, color='k', linestyle='--', alpha=0.5)
            plt.ylabel('Correction (mm)')
            plt.xlabel('Accumulated Time (s)')
            plt.legend()
            plt.grid(True, alpha=0.3)
            
            plt.tight_layout()
            plt.savefig(os.path.join(DATA_DIR, f'drift_comparison_{self.name}.png'), dpi=150)
            write_log(f"[验证-{self.name}] 零飘对比图已保存")
            plt.close()
        except Exception as e:
            write_log(f"[验证-{self.name}] 生成图表失败: {e}")
    
    def get_test_status(self):
        if not self.is_testing:
            return "Not testing"
        
        progress = (self.accumulated_time / self.test_duration) * 100
        remaining = self.test_duration - self.accumulated_time
        return f"In progress {progress:.1f}% (Remaining: {remaining:.1f}s, Samples: {len(self.static_test_data)})"
    
    def force_finish_test(self):
        if self.is_testing:
            write_log(f"[验证-{self.name}] 手动结束测试，累积时间: {self.accumulated_time:.1f}秒", "IMPORTANT")
            self.is_testing = False
            if len(self.static_test_data) >= 50:
                self.analyze_static_performance()
            else:
                write_log(f"[验证-{self.name}] 数据不足，无法生成报告")
    
    def emergency_stop(self):
        """紧急停止测试"""
        if self.is_testing:
            self.is_testing = False
            write_log(f"[验证-{self.name}] 紧急停止测试", "IMPORTANT")

# 数据状态管理类
class DataStatus:
    def __init__(self):
        self.invalid_count = 0
        self.had_valid_data = False
        self.last_recorded_data = None
        self.consecutive_failures = 0
        self.recorded_data_raw = []      # 原始数据
        self.recorded_data_filtered = [] # 滤波后数据
        self.pause_count = 0             # 暂停次数计数
        self.current_segment = 1         # 当前采样段编号
    
    def on_invalid_data(self):
        self.invalid_count += 1
        self.consecutive_failures += 1
        return self.invalid_count
    
    def on_valid_data(self):
        recovery_needed = self.invalid_count > 3 and self.had_valid_data
        self.invalid_count = 0
        self.had_valid_data = True
        self.consecutive_failures = 0
        return recovery_needed
    
    def reset_for_recovery(self):
        write_log("[系统] 执行数据恢复重置", "IMPORTANT")
        self.last_recorded_data = None
    
    def reset_for_new_segment(self):
        """为新的采样段重置状态"""
        self.pause_count += 1
        self.current_segment += 1
        self.last_recorded_data = None
        write_log(f"[采样] 开始新的采样段 #{self.current_segment}", "IMPORTANT")
    
    def record_data_point(self, timestamp, distance_2_raw, distance_3_raw, distance_2_filtered, distance_3_filtered, cam_coord):
        # 记录原始数据
        self.recorded_data_raw.append((timestamp, distance_2_raw, distance_3_raw, cam_coord, self.current_segment))
        # 记录滤波后数据
        self.recorded_data_filtered.append((timestamp, distance_2_filtered, distance_3_filtered, cam_coord, self.current_segment))
        
        write_log(f"[数据] 段#{self.current_segment} 记录数据点: 原始d2={distance_2_raw:.3f}m, d3={distance_3_raw:.3f}m | 滤波d2={distance_2_filtered:.3f}m, d3={distance_3_filtered:.3f}m | x={cam_coord[0]:.3f}, z={cam_coord[2]:.3f}")

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
    data_dir = os.path.join(os.getcwd(), "data", f"sampling_{timestamp}")
    os.makedirs(data_dir, exist_ok=True)
    print(f"Data will be saved to directory: {data_dir}")
    return data_dir, timestamp

# 配置
SERIAL_PORT = "/dev/ttyUSB0"
BAUD_RATE = 9600
TRACKER_NAME = "tracker_1"

DATA_DIR, TIMESTAMP = create_data_directory()
CSV_FILE_RAW = os.path.join(DATA_DIR, f"tracker_data_raw_{TIMESTAMP}.csv")
CSV_FILE_FILTERED = os.path.join(DATA_DIR, f"tracker_data_filtered_{TIMESTAMP}.csv")
LOG_FILE = os.path.join(DATA_DIR, "tracker_log.txt")

# 初始化日志
with open(LOG_FILE, "w") as log_file:
    log_file.write(f"日志创建时间: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n")
    log_file.write("=== 系统日志 ===\n")

def write_log(message, level="INFO"):
    timestamp = datetime.now().strftime('%H:%M:%S.%f')[:-3]  # 包含毫秒
    log_message = f"[{timestamp}] {message}"
    
    with open(LOG_FILE, "a") as log_file:
        log_file.write(log_message + "\n")
    
    # 根据级别决定是否在控制台显示
    if level in ["IMPORTANT", "ERROR"]:
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
        write_log(f"Serial port configured: {SERIAL_PORT}, {BAUD_RATE} baud, timeout {ser.timeout}s", "IMPORTANT")
except serial.SerialException as e:
    write_log(f"Cannot open serial port {SERIAL_PORT}: {e}", "ERROR")
    sys.exit(1)

# 全局对象初始化
data_status = DataStatus()
running = True
recording = False
paused = False
current_state = SystemState.IDLE

# 为每个通道创建独立的滤波器和验证器
noise_filters = {
    'd2': NoiseFilter(window_size=20, noise_threshold=0.005),
    'd3': NoiseFilter(window_size=20, noise_threshold=0.005)
}

latency_filter = LatencyFilter(max_acceptable_latency_ms=80)

drift_validators = {
    'd2': ZeroDriftValidator("Distance_2"),
    'd3': ZeroDriftValidator("Distance_3")
}

# 阈值设置
min_change_thresholds = {
    "Distance_2": 0.008,
    "Distance_3": 0.008,
    "X": 0.005,
    "Z": 0.005
}
max_change_threshold = 0.2

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

def save_to_csv(data_raw, data_filtered):
    """分别保存原始数据和滤波后数据到两个CSV文件"""
    
    # 保存原始数据
    with open(CSV_FILE_RAW, 'w') as csv_file:
        csv_file.write("Timestamp,Distance_2,Distance_3,X,Y,Z,Roll,Yaw,Pitch,Segment\n")
        
        for timestamp, dis2, dis3, coord, segment in data_raw:
            csv_entry = f"{timestamp:.3f},{dis2:.3f},{dis3:.3f},{coord[0]:.4f},{coord[1]:.4f},{coord[2]:.4f},{coord[3]:.4f},{coord[4]:.4f},{coord[5]:.4f},{segment}\n"
            csv_file.write(csv_entry)
    
    # 保存滤波后数据
    with open(CSV_FILE_FILTERED, 'w') as csv_file:
        csv_file.write("Timestamp,Distance_2,Distance_3,X,Y,Z,Roll,Yaw,Pitch,Segment\n")
        
        for timestamp, dis2, dis3, coord, segment in data_filtered:
            csv_entry = f"{timestamp:.3f},{dis2:.3f},{dis3:.3f},{coord[0]:.4f},{coord[1]:.4f},{coord[2]:.4f},{coord[3]:.4f},{coord[4]:.4f},{coord[5]:.4f},{segment}\n"
            csv_file.write(csv_entry)
            
    write_log(f"[系统] 保存原始数据 {len(data_raw)} 条到 {CSV_FILE_RAW}", "IMPORTANT")
    write_log(f"[系统] 保存滤波数据 {len(data_filtered)} 条到 {CSV_FILE_FILTERED}", "IMPORTANT")

def format_data_for_log(data_name, prev_value, curr_value, change):
    return f"{data_name}: {prev_value:.4f} -> {curr_value:.4f} (Change: {change:.4f})"

def force_stop_all_operations():
    """强制停止所有操作，返回到空闲状态"""
    global recording, paused, current_state, data_status
    
    # 停止录制
    if recording or paused:
        recording = False
        paused = False
        write_log("[系统] 强制停止录制", "IMPORTANT")
        if len(data_status.recorded_data_raw) > 0 or len(data_status.recorded_data_filtered) > 0:
            save_to_csv(data_status.recorded_data_raw, data_status.recorded_data_filtered)
            write_log(f"[系统] 已保存原始数据 {len(data_status.recorded_data_raw)} 条，滤波数据 {len(data_status.recorded_data_filtered)} 条", "IMPORTANT")
    
    # 停止所有静态测试
    active_tests = []
    for name, validator in drift_validators.items():
        if validator.is_testing:
            active_tests.append(name)
            validator.emergency_stop()
    
    if active_tests:
        write_log(f"[系统] 强制停止静态测试: {', '.join(active_tests)}", "IMPORTANT")
    
    # 重置系统状态
    current_state = SystemState.IDLE
    write_log(f"[系统] 系统状态: {current_state}", "IMPORTANT")
    
    # 显示控制说明
    display_help()

def get_current_state():
    """获取当前系统状态"""
    if paused:
        return SystemState.PAUSED
    elif recording:
        active_tests = [name for name, validator in drift_validators.items() if validator.is_testing]
        if active_tests:
            return SystemState.TESTING
        else:
            return SystemState.RECORDING
    return SystemState.IDLE

def pause_recording():
    """暂停录制并准备新的baseline校准"""
    global paused, current_state
    
    if recording and not paused:
        paused = True
        current_state = SystemState.PAUSED
        
        # 重置滤波器的baseline
        for name, filter_obj in noise_filters.items():
            filter_obj.reset_baseline()
        
        # 为新采样段做准备
        data_status.reset_for_new_segment()
        
        write_log(f"[采样] 已暂停录制，等待基线重新校准完成后继续采样", "IMPORTANT")
        write_log(f"[系统] 系统状态: {current_state}", "IMPORTANT")
        return True
    return False

def resume_recording():
    """恢复录制（需要等待baseline校准完成）"""
    global paused, current_state
    
    if paused:
        # 检查baseline是否准备就绪
        all_baselines_ready = all(filter_obj.is_baseline_ready() for filter_obj in noise_filters.values())
        
        if all_baselines_ready:
            paused = False
            current_state = SystemState.RECORDING
            write_log(f"[采样] 基线校准完成，恢复录制", "IMPORTANT")
            write_log(f"[系统] 系统状态: {current_state}", "IMPORTANT")
            return True
        else:
            baselines_status = []
            for name, filter_obj in noise_filters.items():
                status = "Ready" if filter_obj.is_baseline_ready() else f"Calibrating ({filter_obj.baseline_samples}/{filter_obj.baseline_required})"
                baselines_status.append(f"{name}: {status}")
            
            write_log(f"[采样] 基线校准未完成，无法恢复录制", "IMPORTANT")
            write_log(f"[采样] 基线状态: {', '.join(baselines_status)}", "IMPORTANT")
            return False
    return False

def display_help():
    """显示控制说明"""
    help_text = """
=== Control Instructions ===
Basic Operations:
  'R' - Start recording data (linear movement sampling)
  'E' - Stop recording data  
  'P' - Pause/Resume recording (switch position, recalibrate baseline)
  'T' - Start 60-second static zero drift test
  'S' - Show current performance statistics
  'D' - Toggle Debug detailed information
  'I' - Show test status
  'Q' - Manually end test
  'X' - Force exit current mode, return to idle state
  'H' - Show help information
  'Esc' - Exit program and save data

Sampling Process:
  1. Press 'R' to start recording, perform linear movement
  2. Press 'P' to pause recording, can switch to new position
  3. At new position press 'P' again, system will recalibrate baseline then continue sampling
  4. Finally save raw data and filtered data to two separate CSV files
=========================
    """
    write_log(help_text, "IMPORTANT")

# 简化的数据处理函数
def simplified_process_data(timestamp, distance_2, distance_3, cam_coord, recovery_needed=False):
    global data_status, noise_filters, drift_validators, paused
    
    processing_start = time.time()
    
    # 应用零飘滤波
    filtered_d2 = noise_filters['d2'].get_filtered_value(distance_2)
    filtered_d3 = noise_filters['d3'].get_filtered_value(distance_3)
    
    # 更新滤波器状态
    noise_filters['d2'].add_sample(distance_2)
    noise_filters['d3'].add_sample(distance_3)
    
    # 记录滤波效果
    d2_correction = distance_2 - filtered_d2
    d3_correction = distance_3 - filtered_d3
    if abs(d2_correction) > 0.001 or abs(d3_correction) > 0.001:
        write_log(f"[滤波] D2校正: {d2_correction*1000:.1f}mm, D3校正: {d3_correction*1000:.1f}mm")
    
    # 分别添加到对应的验证器
    drift_validators['d2'].add_static_sample(distance_2, filtered_d2)
    drift_validators['d3'].add_static_sample(distance_3, filtered_d3)
    
    # 使用滤波后的数据进行变化检测
    current_data = (filtered_d2, filtered_d3, cam_coord[0], cam_coord[2])
    change_names = ["Distance_2", "Distance_3", "X", "Z"]
    
    # 记录数据
    write_log(f"[原始] A69: d2={distance_2:.3f}m, d3={distance_3:.3f}m | Vive: x={cam_coord[0]:.3f}, y={cam_coord[1]:.3f}, z={cam_coord[2]:.3f}")
    write_log(f"[滤波] A69: d2={filtered_d2:.3f}m, d3={filtered_d3:.3f}m")
    
    # 如果处于暂停状态，只用于baseline校准，不记录数据
    if paused:
        # 检查是否可以恢复录制
        if all(filter_obj.is_baseline_ready() for filter_obj in noise_filters.values()):
            write_log(f"[采样] 基线校准完成，可以按'P'恢复录制", "IMPORTANT")
        return
    
    # 处理数据恢复
    if recovery_needed:
        write_log(f"[系统] 检测到有效数据恢复，重置比较基准", "IMPORTANT")
        data_status.reset_for_recovery()
    
    # 第一条数据
    if data_status.last_recorded_data is None:
        write_log(f"[系统] 记录基准数据", "IMPORTANT")
        data_status.record_data_point(timestamp, distance_2, distance_3, filtered_d2, filtered_d3, cam_coord)
        data_status.last_recorded_data = current_data
        return
    
    # 计算变化
    changes = [abs(current_data[i] - data_status.last_recorded_data[i]) for i in range(4)]
    
    # 生成变化详情
    change_details = []
    for i in range(4):
        change_details.append(format_data_for_log(
            change_names[i], 
            data_status.last_recorded_data[i], 
            current_data[i], 
            changes[i]
        ))
    write_log(f"[比较] {' | '.join(change_details)}")
    
    # 检查异常变化
    data_too_large = any(changes[i] > max_change_threshold for i in range(4))
    if data_too_large:
        write_log(f"[警告] 数据变化过大，更新比较基准但不记录", "IMPORTANT")
        data_status.last_recorded_data = current_data
        return
    
    # 检查显著变化
    significant_changes = []
    for i in range(4):
        threshold = min_change_thresholds[change_names[i]]
        if changes[i] > threshold:
            significant_changes.append(f"{change_names[i]}={changes[i]:.4f}>{threshold}")
    
    # 核心简化：消除双重滤波
    if significant_changes:
        write_log(f"[信息] 检测到明显变化: {', '.join(significant_changes)}", "IMPORTANT")
        data_status.record_data_point(timestamp, distance_2, distance_3, filtered_d2, filtered_d3, cam_coord)
        data_status.last_recorded_data = current_data
    else:
        write_log("[信息] 无显著变化，跳过记录")
    
    # 记录处理时间
    processing_time = time.time() - processing_start
    if processing_time > 0.005:
        write_log(f"[延迟] 数据处理耗时: {processing_time*1000:.2f}ms", "DEBUG")

# 增强的串口监听函数
def enhanced_serial_listener():
    global recording, paused, data_status, latency_filter, current_state
    
    # 创建串口缓冲区管理器
    buffer_manager = SerialBufferManager(ser)
    
    # 性能监控
    request_count = 0
    success_count = 0
    last_stats_time = time.time()
    
    while running:
        if recording:  # 录制模式包括正常录制和暂停状态
            current_state = get_current_state()
            
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
                        recovery_needed = data_status.on_valid_data()
                        simplified_process_data(response_time, distance_2, distance_3, cam_coord, recovery_needed)
                        
                    except Exception as e:
                        write_log(f"获取tracker数据失败: {e}", "ERROR")
                        data_status.on_invalid_data()
                
                else:
                    # 解析失败
                    data_status.on_invalid_data()
                    write_log("数据解析失败", "DEBUG")
            
            else:
                # 没有收到响应
                consecutive_failures = data_status.on_invalid_data()
                write_log(f"未收到响应 (连续失败: {consecutive_failures}次)", "DEBUG")
                
                # 严重失败时的处理
                if consecutive_failures > 10:
                    write_log("连续失败次数过多，执行深度恢复", "ERROR")
                    buffer_manager.attempt_recovery()
            
            # 定期输出统计信息
            current_time = time.time()
            if current_time - last_stats_time > 30:  # 每30秒
                success_rate = (success_count / request_count * 100) if request_count > 0 else 0
                write_log(f"串口统计: 请求{request_count}次, 成功{success_count}次, 成功率{success_rate:.1f}%", "IMPORTANT")
                last_stats_time = current_time
        else:
            current_state = SystemState.IDLE

        time.sleep(0.015)

def generate_performance_report():
    report = f"""
=== Performance Analysis Report ===
System State: {current_state}
Sampling Segments: {data_status.current_segment}
Pause Count: {data_status.pause_count}
Latency Statistics: {latency_filter.get_latency_stats()}
Filter Statistics:
  D2 Baseline: {noise_filters['d2'].baseline_value:.4f}m (Samples: {noise_filters['d2'].baseline_samples})
  D3 Baseline: {noise_filters['d3'].baseline_value:.4f}m (Samples: {noise_filters['d3'].baseline_samples})
Data Statistics:
  Raw Data Points: {len(data_status.recorded_data_raw)}
  Filtered Data Points: {len(data_status.recorded_data_filtered)}
    """
    write_log(report, "IMPORTANT")
    return report

# 启动线程
serial_thread = threading.Thread(target=enhanced_serial_listener, daemon=True)
serial_thread.start()

# 增强的按键监听
def on_press(key):
    global recording, paused, data_status, current_state
    try:
        if key.char == 'r' and current_state == SystemState.IDLE:
            recording = True
            paused = False
            data_status = DataStatus()
            noise_filters['d2'] = NoiseFilter(window_size=20, noise_threshold=0.005)
            noise_filters['d3'] = NoiseFilter(window_size=20, noise_threshold=0.005)
            current_state = SystemState.RECORDING
            write_log("[系统] 开始录制数据（线性移动采样）...", "IMPORTANT")
            write_log(f"[系统] 系统状态: {current_state}", "IMPORTANT")
            
        elif key.char == 'e' and (current_state == SystemState.RECORDING or current_state == SystemState.PAUSED):
            recording = False
            paused = False
            current_state = SystemState.IDLE
            write_log("[系统] 停止录制数据", "IMPORTANT")
            write_log(f"[系统] 系统状态: {current_state}", "IMPORTANT")
            save_to_csv(data_status.recorded_data_raw, data_status.recorded_data_filtered)
            generate_performance_report()
            
        elif key.char == 'p' and (current_state == SystemState.RECORDING or current_state == SystemState.PAUSED):
            if current_state == SystemState.RECORDING:
                # 暂停录制
                if pause_recording():
                    write_log("[采样] 已暂停，请移动到新位置后再按'P'继续", "IMPORTANT")
            elif current_state == SystemState.PAUSED:
                # 恢复录制
                if resume_recording():
                    write_log("[采样] 已恢复录制，可以继续线性移动", "IMPORTANT")
                else:
                    write_log("[采样] 无法恢复，等待基线校准完成", "IMPORTANT")
                    
        elif key.char == 't' and current_state == SystemState.IDLE:
            # 开始静态测试
            drift_validators['d2'].start_static_test()
            drift_validators['d3'].start_static_test()
            recording = True
            paused = False
            current_state = SystemState.TESTING
            write_log("[系统] 静态测试已开始，即使连接中断也会继续累积时间", "IMPORTANT")
            write_log(f"[系统] 系统状态: {current_state}", "IMPORTANT")
            
        elif key.char == 'q':
            # 手动结束静态测试
            if current_state == SystemState.TESTING:
                active_tests = [name for name, validator in drift_validators.items() if validator.is_testing]
                if active_tests:
                    for name, validator in drift_validators.items():
                        if validator.is_testing:
                            validator.force_finish_test()
                    recording = False
                    paused = False
                    current_state = SystemState.IDLE
                    write_log(f"[系统] 手动结束静态测试: {', '.join(active_tests)}", "IMPORTANT")
                    write_log(f"[系统] 系统状态: {current_state}", "IMPORTANT")
                else:
                    write_log("[系统] 当前没有进行中的静态测试")
            else:
                write_log("[系统] 当前没有进行中的静态测试")
                
        elif key.char == 'i':
            # 显示测试状态信息
            write_log(f"[状态] 当前系统状态: {current_state}", "IMPORTANT")
            write_log(f"[状态] 采样段数: {data_status.current_segment}, 暂停次数: {data_status.pause_count}", "IMPORTANT")
            
            # 显示baseline状态
            for name, filter_obj in noise_filters.items():
                status = "Ready" if filter_obj.is_baseline_ready() else f"Calibrating ({filter_obj.baseline_samples}/{filter_obj.baseline_required})"
                write_log(f"[状态-{name}] 基线: {status}", "IMPORTANT")
            
            for name, validator in drift_validators.items():
                if validator.is_testing:
                    status = validator.get_test_status()
                    write_log(f"[状态-{name}] {status}", "IMPORTANT")
            if not any(v.is_testing for v in drift_validators.values()):
                write_log("[状态] 当前没有进行中的静态测试")
                
        elif key.char == 's':
            generate_performance_report()
            
        elif key.char == 'd':
            # 切换debug模式
            DEBUG_LEVEL["SERIAL_RAW"] = not DEBUG_LEVEL["SERIAL_RAW"]
            DEBUG_LEVEL["PROTOCOL"] = not DEBUG_LEVEL["PROTOCOL"]
            status = "ON" if DEBUG_LEVEL["SERIAL_RAW"] else "OFF"
            write_log(f"[系统] Debug模式已{status}", "IMPORTANT")
            
        elif key.char == 'x':
            # 强制退出当前模式
            if current_state != SystemState.IDLE:
                write_log(f"[系统] 强制退出当前模式: {current_state}", "IMPORTANT")
                force_stop_all_operations()
            else:
                write_log("[系统] 当前已经是空闲状态", "IMPORTANT")
                display_help()
                
        elif key.char == 'h':
            # 显示帮助
            display_help()
            
        # 状态检查提示
        if key.char in ['r', 'e', 't', 'p']:
            if key.char == 'r' and current_state != SystemState.IDLE:
                write_log(f"[警告] 当前状态为 {current_state}，无法开始录制。请先按 'X' 退出当前模式", "IMPORTANT")
            elif key.char == 'e' and current_state not in [SystemState.RECORDING, SystemState.PAUSED]:
                write_log(f"[警告] 当前状态为 {current_state}，无法停止录制", "IMPORTANT")
            elif key.char == 't' and current_state != SystemState.IDLE:
                write_log(f"[警告] 当前状态为 {current_state}，无法开始测试。请先按 'X' 退出当前模式", "IMPORTANT")
            elif key.char == 'p' and current_state not in [SystemState.RECORDING, SystemState.PAUSED]:
                write_log(f"[警告] 当前状态为 {current_state}，无法暂停/恢复录制", "IMPORTANT")
                
    except AttributeError:
        pass

def on_release(key):
    global running, data_status
    if key == keyboard.Key.esc:
        write_log("[系统] 程序退出，保存所有数据...", "IMPORTANT")
        force_stop_all_operations()  # 确保所有操作都被正确停止
        running = False
        return False

listener = keyboard.Listener(on_press=on_press, on_release=on_release)
listener.start()

# 显示初始控制说明
display_help()
write_log(f"[系统] 当前状态: {current_state}", "IMPORTANT")

try:
    while running:
        time.sleep(0.1)
except KeyboardInterrupt:
    write_log("[系统] 接收到 Ctrl+C，程序退出...", "IMPORTANT")
    force_stop_all_operations()
    running = False
finally:
    if ser and ser.is_open:
        ser.close()
        write_log("[系统] 串口已关闭", "IMPORTANT")