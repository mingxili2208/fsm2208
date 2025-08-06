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
            write_log(f"[延迟] 拒绝高延迟数据: {latency*1000:.1f}ms > {self.max_latency*1000}ms")
            return False
        return True
    
    def check_tracker_freshness(self, tracker_request_time, tracker_response_time):
        processing_time = tracker_response_time - tracker_request_time
        if processing_time > self.max_latency:
            write_log(f"[延迟] 拒绝过慢tracker数据: 获取耗时{processing_time*1000:.1f}ms")
            return False
        return True
    
    def get_latency_stats(self):
        if self.recent_latencies:
            avg_latency = np.mean(self.recent_latencies) * 1000
            max_latency = np.max(self.recent_latencies) * 1000
            rejection_rate = (self.rejected_count / self.total_requests) * 100 if self.total_requests > 0 else 0
            return f"平均延迟: {avg_latency:.1f}ms, 最大: {max_latency:.1f}ms, 拒绝率: {rejection_rate:.1f}%"
        return "无延迟数据"

# 增强的零飘验证器类（支持连接中断后继续测试）
class ZeroDriftValidator:
    def __init__(self, name=""):
        self.name = name
        self.static_test_data = []
        self.test_duration = 60
        self.start_time = None
        self.is_testing = False
        self.accumulated_time = 0  # 累积的有效测试时间
        self.last_valid_time = None  # 最后一次收到有效数据的时间
        self.connection_loss_count = 0  # 连接丢失次数
        self.total_disconnection_time = 0  # 总断线时间
        
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
        
        # 如果这是连接恢复后的第一个数据点
        if self.last_valid_time and (current_time - self.last_valid_time) > 1.0:  # 超过1秒认为是连接中断
            disconnection_duration = current_time - self.last_valid_time
            self.connection_loss_count += 1
            self.total_disconnection_time += disconnection_duration
            write_log(f"[验证-{self.name}] 检测到连接恢复，断线时长: {disconnection_duration:.1f}秒", "IMPORTANT")
        
        # 更新累积的有效测试时间
        if self.last_valid_time:
            time_since_last = min(current_time - self.last_valid_time, 1.0)  # 限制单次时间增量
            self.accumulated_time += time_since_last
        
        self.last_valid_time = current_time
        
        # 检查是否达到目标测试时间
        if self.accumulated_time >= self.test_duration:
            write_log(f"[验证-{self.name}] 累积测试时间已达到 {self.test_duration}秒，结束测试", "IMPORTANT")
            self.is_testing = False
            self.analyze_static_performance()
            return
        
        # 添加数据点
        self.static_test_data.append({
            'time': self.accumulated_time,  # 使用累积时间而不是绝对时间
            'raw': raw_value,
            'filtered': filtered_value,
            'real_time': current_time  # 保留真实时间戳用于调试
        })
        
        # 每10秒报告一次进度
        if len(self.static_test_data) % 200 == 0:  # 假设约20Hz采样率
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
        
        # 计算数据采集的时间统计
        actual_test_duration = time.time() - self.start_time if self.start_time else 0
        effective_data_rate = len(self.static_test_data) / self.accumulated_time if self.accumulated_time > 0 else 0
        
        report = f"""
=== {self.name} 零飘处理验证报告 ===
测试配置:
  目标测试时间: {self.test_duration}秒
  累积有效时间: {self.accumulated_time:.1f}秒
  实际耗时: {actual_test_duration:.1f}秒
  连接中断次数: {self.connection_loss_count}
  总断线时间: {self.total_disconnection_time:.1f}秒

数据统计:
  有效样本数: {len(self.static_test_data)}
  有效数据率: {effective_data_rate:.1f} Hz
  原始数据标准差: {raw_std:.2f}mm
  滤波后标准差: {filtered_std:.2f}mm
  标准差改善: {improvement_std:.1f}%
  
  原始数据极差: {raw_range:.2f}mm  
  滤波后极差: {filtered_range:.2f}mm
  极差改善: {improvement_range:.1f}%
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
            
            # 主图：数据对比
            plt.subplot(2, 1, 1)
            plt.plot(times, raw_values, 'b-', alpha=0.7, label='原始数据', linewidth=1)
            plt.plot(times, filtered_values, 'r-', linewidth=2, label='滤波后')
            plt.ylabel('距离 (mm)')
            plt.title(f'{self.name} 零飘处理效果对比 (累积时间: {self.accumulated_time:.1f}s)')
            plt.legend()
            plt.grid(True, alpha=0.3)
            
            # 子图：差值分析
            plt.subplot(2, 1, 2)
            differences = [(raw - filtered) * 1000 for raw, filtered in zip([d['raw'] for d in self.static_test_data], [d['filtered'] for d in self.static_test_data])]
            plt.plot(times, differences, 'g-', alpha=0.7, label='滤波校正量', linewidth=1)
            plt.axhline(y=0, color='k', linestyle='--', alpha=0.5)
            plt.ylabel('校正量 (mm)')
            plt.xlabel('累积时间 (s)')
            plt.legend()
            plt.grid(True, alpha=0.3)
            
            plt.tight_layout()
            plt.savefig(os.path.join(DATA_DIR, f'drift_comparison_{self.name}.png'), dpi=150)
            write_log(f"[验证-{self.name}] 零飘对比图已保存")
            plt.close()
        except Exception as e:
            write_log(f"[验证-{self.name}] 生成图表失败: {e}")
    
    def get_test_status(self):
        """获取当前测试状态"""
        if not self.is_testing:
            return "未进行测试"
        
        progress = (self.accumulated_time / self.test_duration) * 100
        remaining = self.test_duration - self.accumulated_time
        return f"进行中 {progress:.1f}% (剩余{remaining:.1f}s, 样本:{len(self.static_test_data)})"
    
    def force_finish_test(self):
        """手动结束测试"""
        if self.is_testing:
            write_log(f"[验证-{self.name}] 手动结束测试，累积时间: {self.accumulated_time:.1f}秒", "IMPORTANT")
            self.is_testing = False
            if len(self.static_test_data) >= 50:
                self.analyze_static_performance()
            else:
                write_log(f"[验证-{self.name}] 数据不足，无法生成报告")
# **简化后的数据状态管理类**
class DataStatus:
    def __init__(self):
        self.invalid_count = 0
        self.had_valid_data = False
        self.last_recorded_data = None
        self.consecutive_failures = 0
        self.recorded_data = []
    
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
    
    def record_data_point(self, timestamp, distance_2, distance_3, cam_coord):
        self.recorded_data.append((timestamp, distance_2, distance_3, cam_coord))
        write_log(f"[数据] 记录数据点: d2={distance_2:.3f}m, d3={distance_3:.3f}m | x={cam_coord[0]:.3f}, z={cam_coord[2]:.3f}")

# 创建数据存储目录
def create_data_directory():
    timestamp = datetime.now().strftime("%m%d_%H%M%S")
    data_dir = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "data", timestamp)
    os.makedirs(data_dir, exist_ok=True)
    print(f"[系统] 数据将保存到目录: {data_dir}")
    return data_dir, timestamp

# 配置
SERIAL_PORT = "/dev/ttyUSB0"
BAUD_RATE = 9600
TRACKER_NAME = "tracker_1"

DATA_DIR, TIMESTAMP = create_data_directory()
CSV_FILE = os.path.join(DATA_DIR, f"tracker_data_{TIMESTAMP}.csv")
LOG_FILE = os.path.join(DATA_DIR, "tracker_log.txt")

# 初始化日志
with open(LOG_FILE, "w") as log_file:
    log_file.write(f"日志创建时间: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n")
    log_file.write("=== 系统日志 ===\n")

def write_log(message, level="INFO"):
    timestamp = datetime.now().strftime('%H:%M:%S')
    log_message = f"[{timestamp}] {message}"
    
    with open(LOG_FILE, "a") as log_file:
        log_file.write(log_message + "\n")
    
    if level in ["IMPORTANT", "ERROR"]:
        print(message)

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

try:
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1)
    if ser.is_open:
        write_log(f"[系统] 成功打开串口: {SERIAL_PORT}", "IMPORTANT")
except serial.SerialException as e:
    write_log(f"[错误] 无法打开串口 {SERIAL_PORT}: {e}", "ERROR")
    sys.exit(1)

# 全局对象初始化
data_status = DataStatus()
running = True
recording = False

# **按照建议修正：为每个通道创建独立的滤波器和验证器**
noise_filters = {
    'd2': NoiseFilter(window_size=20, noise_threshold=0.005),
    'd3': NoiseFilter(window_size=20, noise_threshold=0.005)
}

latency_filter = LatencyFilter(max_acceptable_latency_ms=30)

drift_validators = {
    'd2': ZeroDriftValidator("Distance_2"),
    'd3': ZeroDriftValidator("Distance_3")
}

# 阈值设置
min_change_thresholds = {
    "Distance_2": 0.008,
    "Distance_3": 0.008,
    "X": 0.003,
    "Z": 0.003
}
max_change_threshold = 0.2

# A69协议函数
def calculate_checksum_r(data):
    return data[3] ^ data[4] ^ data[5] ^ data[6]

def parse_a69_data(response):
    if len(response) != 10 or response[0] != 0x55 or response[1] != 0x7E or response[8] != 0x7E or response[9] != 0x55:
        write_log(f"[错误] 无效数据包: {list(map(hex, response))}", "ERROR")
        return None

    checksum = response[7]
    computed_checksum = calculate_checksum_r(response)

    if checksum != computed_checksum:
        write_log(f"[错误] 校验失败: 计算值 {hex(computed_checksum)}, 收到值 {hex(checksum)}", "ERROR")
        return None

    distance_2 = (response[3] << 8) | response[4]
    distance_3 = (response[5] << 8) | response[6]
    return distance_2 / 1000.0, distance_3 / 1000.0

def send_a69_data_request():
    tx_buf = bytearray([0x55, 0x7E, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x7E, 0x55])
    tx_buf[7] = calculate_checksum_r(tx_buf)
    ser.write(tx_buf)
    ser.flush()

def save_to_csv(data):
    with open(CSV_FILE, 'w') as csv_file:
        csv_file.write("Timestamp,Distance_2,Distance_3,X,Y,Z,Roll,Yaw,Pitch\n")
        
        for timestamp, dis2, dis3, coord in data:
            csv_entry = f"{timestamp:.3f},{dis2:.3f},{dis3:.3f},{coord[0]:.4f},{coord[1]:.4f},{coord[2]:.4f},{coord[3]:.4f},{coord[4]:.4f},{coord[5]:.4f}\n"
            csv_file.write(csv_entry)
            
    write_log(f"[系统] 保存 {len(data)} 条数据到 {CSV_FILE}", "IMPORTANT")

def format_data_for_log(data_name, prev_value, curr_value, change):
    return f"{data_name}: {prev_value:.4f} -> {curr_value:.4f} (变化: {change:.4f})"

# **核心优化：简化的数据处理函数**
def simplified_process_data(timestamp, distance_2, distance_3, cam_coord, recovery_needed=False):
    global data_status, noise_filters, drift_validators
    
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
    
    # **按建议修正：分别添加到对应的验证器**
    drift_validators['d2'].add_static_sample(distance_2, filtered_d2)
    drift_validators['d3'].add_static_sample(distance_3, filtered_d3)
    
    # 使用滤波后的数据进行变化检测
    current_data = (filtered_d2, filtered_d3, cam_coord[0], cam_coord[2])
    change_names = ["Distance_2", "Distance_3", "X", "Z"]
    
    # 记录数据
    write_log(f"[原始] A69: d2={distance_2:.3f}m, d3={distance_3:.3f}m | Vive: x={cam_coord[0]:.3f}, y={cam_coord[1]:.3f}, z={cam_coord[2]:.3f}")
    write_log(f"[滤波] A69: d2={filtered_d2:.3f}m, d3={filtered_d3:.3f}m")
    
    # 处理数据恢复
    if recovery_needed:
        write_log(f"[系统] 检测到有效数据恢复，重置比较基准", "IMPORTANT")
        data_status.reset_for_recovery()
    
    # 第一条数据
    if data_status.last_recorded_data is None:
        write_log(f"[系统] 记录基准数据", "IMPORTANT")
        data_status.record_data_point(timestamp, filtered_d2, filtered_d3, cam_coord)
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
    
    # **核心简化：消除双重滤波**
    if significant_changes:
        write_log(f"[信息] 检测到明显变化: {', '.join(significant_changes)}", "IMPORTANT")
        # 直接记录当前滤波后的数据，无需缓冲区
        data_status.record_data_point(timestamp, filtered_d2, filtered_d3, cam_coord)
        data_status.last_recorded_data = current_data
    else:
        # 无显著变化时，什么都不做（NoiseFilter已经处理了噪声）
        write_log("[信息] 无显著变化，跳过记录")
    
    # 记录处理时间
    processing_time = time.time() - processing_start
    if processing_time > 0.005:
        write_log(f"[延迟] 数据处理耗时: {processing_time*1000:.2f}ms")

# 增强的串口监听函数
def enhanced_serial_listener():
    global recording, data_status, latency_filter
    max_consecutive_failures = 5
    last_connection_loss_warning = 0
    
    while running:
        if recording:
            request_time = time.time()
            send_a69_data_request()

            try:
                response = ser.read(10)
                response_time = time.time()
                
                if len(response) == 10:
                    if not latency_filter.check_lidar_latency(request_time, response_time):
                        continue
                    
                    parsed_data = parse_a69_data(response)
                    if parsed_data:
                        distance_2, distance_3 = parsed_data
                        
                        tracker_request_time = time.time()
                        cam_coord = tracker.get_pose_euler()
                        tracker_response_time = time.time()
                        
                        if not latency_filter.check_tracker_freshness(tracker_request_time, tracker_response_time):
                            continue
                        
                        recovery_needed = data_status.on_valid_data()
                        
                        # 如果是从连接失败中恢复，给出提示
                        if recovery_needed:
                            write_log("[系统] 连接已恢复，继续数据采集", "IMPORTANT")
                            # 显示验证器状态（如果正在测试）
                            for name, validator in drift_validators.items():
                                if validator.is_testing:
                                    status = validator.get_test_status()
                                    write_log(f"[验证-{name}] 测试状态: {status}", "IMPORTANT")
                        
                        simplified_process_data(response_time, distance_2, distance_3, cam_coord, recovery_needed)
                        
                    else:
                        data_status.on_invalid_data()
                else:
                    consecutive_failures = data_status.on_invalid_data()
                    current_time = time.time()
                    
                    # 限制警告频率：每30秒最多警告一次
                    if consecutive_failures >= max_consecutive_failures and (current_time - last_connection_loss_warning) > 30:
                        last_connection_loss_warning = current_time
                        write_log(f"[警告] 连续 {consecutive_failures} 次未收到有效数据，请检查连接", "ERROR")
                        
                        # 如果正在进行静态测试，给出额外说明
                        active_tests = [name for name, validator in drift_validators.items() if validator.is_testing]
                        if active_tests:
                            write_log(f"[信息] 静态测试进行中 ({', '.join(active_tests)})，连接恢复后将自动继续", "IMPORTANT")
                    
                    write_log("[警告] 未收到有效数据，重发请求...")

            except serial.SerialException as e:
                write_log(f"[错误] 串口读取失败: {e}", "ERROR")
                data_status.on_invalid_data()

        time.sleep(0.015)

def generate_performance_report():
    report = f"""
=== 性能分析报告 ===
延迟统计: {latency_filter.get_latency_stats()}
滤波统计:
  D2基线: {noise_filters['d2'].baseline_value:.4f}m (样本: {noise_filters['d2'].baseline_samples})
  D3基线: {noise_filters['d3'].baseline_value:.4f}m (样本: {noise_filters['d3'].baseline_samples})
数据统计:
  总记录数据点: {len(data_status.recorded_data)}
    """
    write_log(report, "IMPORTANT")
    return report

# 启动线程
serial_thread = threading.Thread(target=enhanced_serial_listener, daemon=True)
serial_thread.start()

# 按键监听
# 增强的按键监听
def on_press(key):
    global recording, data_status
    try:
        if key.char == 'r' and not recording:
            recording = True
            data_status = DataStatus()
            noise_filters['d2'] = NoiseFilter(window_size=20, noise_threshold=0.005)
            noise_filters['d3'] = NoiseFilter(window_size=20, noise_threshold=0.005)
            write_log("[系统] 开始录制数据...", "IMPORTANT")
        elif key.char == 'e' and recording:
            recording = False
            write_log("[系统] 停止录制数据", "IMPORTANT")
            save_to_csv(data_status.recorded_data)
            generate_performance_report()
        elif key.char == 't' and not recording:
            # 开始静态测试
            drift_validators['d2'].start_static_test()
            drift_validators['d3'].start_static_test()
            recording = True
            write_log("[系统] 静态测试已开始，即使连接中断也会继续累积时间", "IMPORTANT")
        elif key.char == 'q':
            # 手动结束静态测试
            active_tests = [name for name, validator in drift_validators.items() if validator.is_testing]
            if active_tests:
                for name, validator in drift_validators.items():
                    if validator.is_testing:
                        validator.force_finish_test()
                recording = False
                write_log(f"[系统] 手动结束静态测试: {', '.join(active_tests)}", "IMPORTANT")
            else:
                write_log("[系统] 当前没有进行中的静态测试")
        elif key.char == 'i':
            # 显示测试状态信息
            for name, validator in drift_validators.items():
                if validator.is_testing:
                    status = validator.get_test_status()
                    write_log(f"[状态-{name}] {status}", "IMPORTANT")
            if not any(v.is_testing for v in drift_validators.values()):
                write_log("[状态] 当前没有进行中的静态测试")
        elif key.char == 's' and recording:
            generate_performance_report()
    except AttributeError:
        pass

def on_release(key):
    global running, data_status
    if key == keyboard.Key.esc:
        write_log("[系统] 程序退出，保存所有数据...", "IMPORTANT")
        save_to_csv(data_status.recorded_data)
        generate_performance_report()
        running = False
        return False

listener = keyboard.Listener(on_press=on_press, on_release=on_release)
listener.start()

write_log("=== 控制说明 ===", "IMPORTANT")
write_log("按 'R' 开始录制，按 'E' 停止录制", "IMPORTANT")
write_log("按 'T' 开始60秒静态零飘测试", "IMPORTANT")
write_log("按 'S' 显示当前性能统计", "IMPORTANT")
write_log("按 'Esc' 退出并保存数据", "IMPORTANT")

try:
    while running:
        time.sleep(0.1)
except KeyboardInterrupt:
    write_log("[系统] 接收到 Ctrl+C，程序退出...", "IMPORTANT")
    save_to_csv(data_status.recorded_data)
    generate_performance_report()
    running = False
finally:
    if ser and ser.is_open:
        ser.close()
        write_log("[系统] 串口已关闭", "IMPORTANT")