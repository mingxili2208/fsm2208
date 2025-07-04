import sys
import time
import struct
import serial
import threading
import queue
import os
import numpy as np
from datetime import datetime
from pynput import keyboard
from vive_tracker import ViveTrackerModule

# 数据状态管理类
class DataStatus:
    def __init__(self):
        self.invalid_count = 0
        self.had_valid_data = False
        self.last_recorded_data = None
        self.data_buffer = []
        self.consecutive_failures = 0
        self.recorded_data = []
    
    def on_invalid_data(self):
        """处理无效数据"""
        self.invalid_count += 1
        self.consecutive_failures += 1
        return self.invalid_count
    
    def on_valid_data(self):
        """处理有效数据，返回是否需要恢复"""
        recovery_needed = self.invalid_count > 3 and self.had_valid_data
        
        # 重置状态
        self.invalid_count = 0
        self.had_valid_data = True
        self.consecutive_failures = 0
        
        return recovery_needed
    
    def reset_for_recovery(self):
        """执行恢复重置"""
        write_log("[系统] 执行数据恢复重置", "IMPORTANT")
        self.last_recorded_data = None
        self.data_buffer.clear()
    
    def add_to_buffer(self, timestamp, distance_2, distance_3, cam_coord):
        """添加数据到缓冲区"""
        self.data_buffer.append((timestamp, distance_2, distance_3, cam_coord))
        write_log(f"[缓冲] 数据添加到缓冲区 (当前大小: {len(self.data_buffer)}/{buffer_size})")
    
    def clear_buffer(self):
        """清空缓冲区"""
        buffer_size_before = len(self.data_buffer)
        self.data_buffer.clear()
        if buffer_size_before > 0:
            write_log(f"[缓冲] 清空缓冲区 ({buffer_size_before}个点)")
    
    def get_buffer_average(self):
        """计算缓冲区数据的平均值"""
        if not self.data_buffer:
            return None
            
        # 使用最新数据的时间戳
        new_timestamp = self.data_buffer[-1][0]
        avg_distance_2 = sum(item[1] for item in self.data_buffer) / len(self.data_buffer)
        avg_distance_3 = sum(item[2] for item in self.data_buffer) / len(self.data_buffer)
        
        avg_coords = [0, 0, 0, 0, 0, 0]
        for i in range(6):
            avg_coords[i] = sum(item[3][i] for item in self.data_buffer) / len(self.data_buffer)
        
        return (new_timestamp, avg_distance_2, avg_distance_3, avg_coords)
    
    def compress_buffer_to_average(self):
        """将缓冲区压缩为一个平均值数据点"""
        if not self.data_buffer:
            return
            
        avg_data = self.get_buffer_average()
        if avg_data:
            buffer_size_before = len(self.data_buffer)
            self.data_buffer.clear()
            self.data_buffer.append(avg_data)
            write_log(f"[缓冲] 缓冲区压缩: {buffer_size_before}个点 -> 1个平均值点", "IMPORTANT")
    
    def record_buffer_average_and_clear(self):
        """记录缓冲区平均值并清空缓冲区"""
        if not self.data_buffer:
            return
            
        avg_data = self.get_buffer_average()
        if avg_data:
            write_log(f"[缓冲] 记录缓冲区平均值数据 ({len(self.data_buffer)}个点的平均)", "IMPORTANT")
            self.record_data_point(*avg_data)
            self.clear_buffer()
    
    def record_data_point(self, timestamp, distance_2, distance_3, cam_coord):
        """记录单个数据点"""
        self.recorded_data.append((timestamp, distance_2, distance_3, cam_coord))
        write_log(f"[数据] 记录数据点: d2={distance_2:.3f}m, d3={distance_3:.3f}m | x={cam_coord[0]:.3f}, z={cam_coord[2]:.3f}")
    
    def finalize_recording(self):
        """完成录制，处理剩余数据"""
        if self.data_buffer:
            write_log(f"[系统] 停止录制，记录缓冲区平均值数据 ({len(self.data_buffer)}个点)", "IMPORTANT")
            self.record_buffer_average_and_clear()

# 创建数据存储目录
def create_data_directory():
    timestamp = datetime.now().strftime("%m%d_%H%M%S")
    data_dir = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "data", timestamp)
    os.makedirs(data_dir, exist_ok=True)
    print(f"[系统] 数据将保存到目录: {data_dir}")
    return data_dir, timestamp

# 串口配置
SERIAL_PORT = "/dev/ttyUSB0"
BAUD_RATE = 9600

# 设定追踪器的名称
TRACKER_NAME = "tracker_1"

# 创建数据存储目录和文件名
DATA_DIR, TIMESTAMP = create_data_directory()
CSV_FILE = os.path.join(DATA_DIR, f"tracker_data_{TIMESTAMP}.csv")
LOG_FILE = os.path.join(DATA_DIR, "tracker_log.txt")

# 初始化日志文件
with open(LOG_FILE, "w") as log_file:
    log_file.write(f"日志创建时间: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n")
    log_file.write("=== 系统日志 ===\n")

# 写入日志函数
def write_log(message, level="INFO"):
    timestamp = datetime.now().strftime('%H:%M:%S')
    log_message = f"[{timestamp}] {message}"
    
    with open(LOG_FILE, "a") as log_file:
        log_file.write(log_message + "\n")
    
    if level in ["IMPORTANT", "ERROR"]:
        print(message)

# 初始化 ViveTrackerModule
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
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1)
    if ser.is_open:
        write_log(f"[系统] 成功打开串口: {SERIAL_PORT}", "IMPORTANT")
except serial.SerialException as e:
    write_log(f"[错误] 无法打开串口 {SERIAL_PORT}: {e}", "ERROR")
    sys.exit(1)

# 全局状态对象和控制变量
data_status = DataStatus()
running = True
recording = False

# 数据过滤设置
buffer_size = 5
min_change_thresholds = {
    "Distance_2": 0.01,
    "Distance_3": 0.01,
    "X": 0.003,
    "Z": 0.003
}
max_change_threshold = 0.2

# 计算 A69 校验和
def calculate_checksum_r(data):
    return data[3] ^ data[4] ^ data[5] ^ data[6]

# 解析 A69 设备数据（只负责解析，不管理状态）
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

# 发送 A69 数据请求
def send_a69_data_request():
    tx_buf = bytearray([0x55, 0x7E, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x7E, 0x55])
    tx_buf[7] = calculate_checksum_r(tx_buf)
    ser.write(tx_buf)
    ser.flush()

# 保存数据到CSV
def save_to_csv(data):
    with open(CSV_FILE, 'w') as csv_file:
        csv_file.write("Timestamp,Distance_2,Distance_3,X,Y,Z,Roll,Yaw,Pitch\n")
        
        for timestamp, dis2, dis3, coord in data:
            csv_entry = f"{timestamp:.3f},{dis2:.3f},{dis3:.3f},{coord[0]:.4f},{coord[1]:.4f},{coord[2]:.4f},{coord[3]:.4f},{coord[4]:.4f},{coord[5]:.4f}\n"
            csv_file.write(csv_entry)
            
    write_log(f"[系统] 保存 {len(data)} 条数据到 {CSV_FILE}", "IMPORTANT")

# 格式化数据变化显示
def format_data_for_log(data_name, prev_value, curr_value, change):
    return f"{data_name}: {prev_value:.4f} -> {curr_value:.4f} (变化: {change:.4f})"

# 数据处理主函数
def process_data_with_buffer(timestamp, distance_2, distance_3, cam_coord, recovery_needed=False):
    global data_status
    
    current_data = (distance_2, distance_3, cam_coord[0], cam_coord[2])
    change_names = ["Distance_2", "Distance_3", "X", "Z"]
    
    # 记录原始数据到日志
    write_log(f"[原始] A69: d2={distance_2:.3f}m, d3={distance_3:.3f}m | Vive: x={cam_coord[0]:.3f}, y={cam_coord[1]:.3f}, z={cam_coord[2]:.3f}, r={cam_coord[3]:.2f}, y={cam_coord[4]:.2f}, p={cam_coord[5]:.2f}")
    
    # 处理数据恢复
    if recovery_needed:
        write_log(f"[系统] 检测到有效数据恢复，重置比较基准", "IMPORTANT")
        data_status.reset_for_recovery()
    
    # 第一条数据或恢复后的第一条数据
    if data_status.last_recorded_data is None:
        write_log(f"[系统] 记录基准数据", "IMPORTANT")
        data_status.record_data_point(timestamp, distance_2, distance_3, cam_coord)
        data_status.last_recorded_data = current_data
        return
    
    # 计算变化
    changes = [abs(current_data[i] - data_status.last_recorded_data[i]) for i in range(4)]
    
    # 生成变化详情日志
    change_details = []
    for i in range(4):
        change_details.append(format_data_for_log(
            change_names[i], 
            data_status.last_recorded_data[i], 
            current_data[i], 
            changes[i]
        ))
    write_log(f"[比较] {' | '.join(change_details)}")
    
    # 检查数据是否变化过大（异常数据）
    data_too_large = any(changes[i] > max_change_threshold for i in range(4))
    
    if data_too_large:
        write_log(f"[警告] 数据变化过大，更新比较基准但不记录", "IMPORTANT")
        write_log(f"[详情] {' | '.join(change_details)}", "IMPORTANT")
        # 更新比较基准，以适应这次剧烈变化
        data_status.last_recorded_data = current_data
        return
    
    # 检查是否有显著变化
    significant_changes = []
    for i in range(4):
        threshold = min_change_thresholds[change_names[i]]
        if changes[i] > threshold:
            significant_changes.append(f"{change_names[i]}={changes[i]:.4f}>{threshold}")
    
    # 处理显著变化
    if significant_changes:
        write_log(f"[信息] 检测到明显变化: {', '.join(significant_changes)}", "IMPORTANT")
        write_log(f"[详情] {' | '.join(change_details)}", "IMPORTANT")
        
        # 记录缓冲区平均值数据并清空
        data_status.record_buffer_average_and_clear()
        
        # 记录当前数据
        data_status.record_data_point(timestamp, distance_2, distance_3, cam_coord)
        data_status.last_recorded_data = current_data
        
    else:
        # 无显著变化，添加到缓冲区
        data_status.add_to_buffer(timestamp, distance_2, distance_3, cam_coord)
        
        # 缓冲区满时的处理
        if len(data_status.data_buffer) >= buffer_size:
            # 压缩缓冲区为一个平均值点
            data_status.compress_buffer_to_average()
            
            # 更新比较基准为压缩后的平均值
            if data_status.data_buffer:
                avg_data = data_status.data_buffer[0]  # 压缩后缓冲区只有一个平均值点
                data_status.last_recorded_data = (avg_data[1], avg_data[2], avg_data[3][0], avg_data[3][2])

# 串口监听线程
def serial_listener():
    global recording, data_status
    max_consecutive_failures = 5
    
    while running:
        if recording:
            send_a69_data_request()

            try:
                response = ser.read(10)
                parsed_data = parse_a69_data(response)

                if parsed_data:
                    # 处理有效数据，检查是否需要恢复
                    recovery_needed = data_status.on_valid_data()
                    
                    distance_2, distance_3 = parsed_data
                    timestamp = time.time()

                    # 请求 Vive Tracker 位姿
                    cam_coord = tracker.get_pose_euler()
                    
                    # 处理数据
                    process_data_with_buffer(timestamp, distance_2, distance_3, cam_coord, recovery_needed)
                    
                else:
                    # 处理无效数据
                    consecutive_failures = data_status.on_invalid_data()
                    if consecutive_failures >= max_consecutive_failures:
                        write_log(f"[警告] 连续 {consecutive_failures} 次未收到有效数据，请检查连接", "ERROR")
                    write_log("[警告] 未收到有效数据，重发请求...")

            except serial.SerialException as e:
                write_log(f"[错误] 串口读取失败: {e}", "ERROR")
                data_status.on_invalid_data()

        time.sleep(0.02)  # 20ms 轮询

# 启动串口监听线程
serial_thread = threading.Thread(target=serial_listener, daemon=True)
serial_thread.start()

# 监听按键
def on_press(key):
    global recording, data_status
    try:
        if key.char == 'r' and not recording:
            recording = True
            # 重置数据状态
            data_status = DataStatus()
            write_log("[系统] 开始录制数据...", "IMPORTANT")
            # 显示当前使用的阈值设置
            thresholds_info = ", ".join([f"{k}: {v}" for k, v in min_change_thresholds.items()])
            write_log(f"[系统] 使用的最小变化阈值: {thresholds_info}", "IMPORTANT")
            write_log(f"[系统] 最大变化阈值: {max_change_threshold}", "IMPORTANT")
        elif key.char == 'e' and recording:
            recording = False
            # 处理缓冲区中的数据
            data_status.finalize_recording()
            write_log("[系统] 停止录制数据", "IMPORTANT")
            # 保存当前所有数据
            save_to_csv(data_status.recorded_data)
    except AttributeError:
        pass

# 监听退出
def on_release(key):
    global running, data_status
    if key == keyboard.Key.esc:
        write_log("[系统] 程序退出，保存所有数据...", "IMPORTANT")
        # 处理缓冲区中的数据
        data_status.finalize_recording()
        save_to_csv(data_status.recorded_data)
        write_log(f"[系统] 数据已保存到 {CSV_FILE}", "IMPORTANT")
        running = False
        return False

# 启动监听
listener = keyboard.Listener(on_press=on_press, on_release=on_release)
listener.start()

write_log("按 'R' 开始录制，按 'E' 停止录制，按 'Esc' 退出并保存数据.", "IMPORTANT")
thresholds_info = ", ".join([f"{k}: {v}" for k, v in min_change_thresholds.items()])
write_log(f"[系统] 配置的最小变化阈值: {thresholds_info}", "IMPORTANT")
write_log(f"[系统] 最大变化阈值: {max_change_threshold}", "IMPORTANT")

try:
    while running:
        time.sleep(0.1)
except KeyboardInterrupt:
    write_log("[系统] 接收到 Ctrl+C，程序退出...", "IMPORTANT")
    data_status.finalize_recording()
    save_to_csv(data_status.recorded_data)
    running = False
finally:
    if ser and ser.is_open:
        ser.close()
        write_log("[系统] 串口已关闭", "IMPORTANT")