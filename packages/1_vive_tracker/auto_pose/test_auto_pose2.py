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

# 创建数据存储目录
def create_data_directory():
    # 获取当前时间戳作为目录名
    timestamp = datetime.now().strftime("%m%d_%H%M%S")
    # 创建数据目录路径
    data_dir = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "data", timestamp)
    
    # 确保目录存在
    os.makedirs(data_dir, exist_ok=True)
    
    print(f"[系统] 数据将保存到目录: {data_dir}")
    return data_dir, timestamp

# 串口配置
SERIAL_PORT = "/dev/ttyUSB0"  # 请根据实际情况修改
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

# 写入日志函数 - 区分控制台和文件
def write_log(message, level="INFO"):
    """
    写入日志，根据级别决定是否同时输出到控制台
    level: "INFO" - 仅文件, "IMPORTANT" - 文件和控制台, "ERROR" - 错误信息，文件和控制台
    """
    timestamp = datetime.now().strftime('%H:%M:%S')
    log_message = f"[{timestamp}] {message}"
    
    # 写入日志文件
    with open(LOG_FILE, "a") as log_file:
        log_file.write(log_message + "\n")
    
    # 根据级别决定是否输出到控制台
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

# 线程安全的队列
data_queue = queue.Queue()

# 存储数据
recorded_data = []
running = True
recording = False  # 录制标志

# 数据过滤与缓冲设置
buffer_size = 5  # 缓冲区大小
data_buffer = []  # 数据缓冲区
last_recorded_data = None  # 上一次记录的数据

# 为每个数据设置单独的阈值
min_change_thresholds = {
    "Distance_2": 0.003,
    "Distance_3": 0.01,
    "X": 0.003,
    "Z": 0.003
}

# 最大变化阈值（超过则抛弃）
max_change_threshold = 0.2

# 无效数据处理
invalid_data_count = 0
had_valid_data = False  # 是否曾经有过有效数据

# 计算 A69 校验和（XOR 校验）
def calculate_checksum_r(data):
    """计算 A69 校验和（XOR）"""
    return data[3] ^ data[4] ^ data[5] ^ data[6]

# 解析 A69 设备数据
def parse_a69_data(response):
    """解析 A69 设备数据包"""
    global invalid_data_count, had_valid_data
    
    if len(response) != 10 or response[0] != 0x55 or response[1] != 0x7E or response[8] != 0x7E or response[9] != 0x55:
        invalid_data_count += 1
        write_log(f"[错误] 无效数据包: {list(map(hex, response))} (连续错误: {invalid_data_count})", "ERROR")
        return None

    checksum = response[7]
    computed_checksum = calculate_checksum_r(response)

    if checksum != computed_checksum:
        invalid_data_count += 1
        write_log(f"[错误] 校验失败: 计算值 {hex(computed_checksum)}, 收到值 {hex(checksum)} (连续错误: {invalid_data_count})", "ERROR")
        return None

    # 解析有效数据，重置无效计数
    invalid_data_count = 0
    had_valid_data = True
    
    distance_2 = (response[3] << 8) | response[4]
    distance_3 = (response[5] << 8) | response[6]

    return distance_2 / 1000.0, distance_3 / 1000.0  # 假设数据单位是 mm，转换为 m

# 发送 A69 数据
def send_a69_data(flag, distance_2, distance_3):
    """按照 A69 格式发送数据"""
    distance_2_int = int(distance_2 * 1000)  # 转换为整数
    distance_3_int = int(distance_3 * 1000)

    tx_buf = bytearray(10)
    tx_buf[0] = 0x55
    tx_buf[1] = 0x7E
    tx_buf[2] = flag
    tx_buf[3] = (distance_2_int >> 8) & 0xFF  # 高字节
    tx_buf[4] = distance_2_int & 0xFF        # 低字节
    tx_buf[5] = (distance_3_int >> 8) & 0xFF  # 高字节
    tx_buf[6] = distance_3_int & 0xFF        # 低字节
    tx_buf[7] = tx_buf[3] ^ tx_buf[4] ^ tx_buf[5] ^ tx_buf[6]  # XOR 校验
    tx_buf[8] = 0x7E
    tx_buf[9] = 0x55

    ser.write(tx_buf)
    ser.flush()
    write_log(f"[数据] 发送 A69 数据: {list(map(hex, tx_buf))}")

def send_a69_data_request():
    """按照 A69 格式发送数据请求，并正确计算校验位"""
    tx_buf = bytearray([0x55, 0x7E, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x7E, 0x55])
    
    # 计算校验位
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

# 格式化数据显示
def format_data_for_log(data_name, prev_value, curr_value, change):
    """格式化数据变化的日志显示"""
    return f"{data_name}: {prev_value:.4f} -> {curr_value:.4f} (变化: {change:.4f})"

# 记录单个数据点
def record_data_point(timestamp, distance_2, distance_3, cam_coord):
    global recorded_data
    recorded_data.append((timestamp, distance_2, distance_3, cam_coord))
    write_log(f"[数据] 记录数据点: d2={distance_2:.3f}m, d3={distance_3:.3f}m | x={cam_coord[0]:.3f}, z={cam_coord[2]:.3f}")

# 处理缓冲区数据 - 修复版本
def process_buffer():
    global data_buffer, last_recorded_data
    
    if not data_buffer:
        return
    
    # 计算平均值
    # avg_timestamp = sum(item[0] for item in data_buffer) / len(data_buffer)
    new_timestamp = data_buffer[-1][0]
    avg_distance_2 = sum(item[1] for item in data_buffer) / len(data_buffer)
    avg_distance_3 = sum(item[2] for item in data_buffer) / len(data_buffer)
    
    # 计算位姿的平均值
    avg_coords = [0, 0, 0, 0, 0, 0]  # x, y, z, roll, yaw, pitch
    for i in range(6):
        avg_coords[i] = sum(item[3][i] for item in data_buffer) / len(data_buffer)
    
    # 更新last_recorded_data为平均值数据
    last_recorded_data = (avg_distance_2, avg_distance_3, avg_coords[0], avg_coords[2])
    
    # 清空缓冲区，只保留平均值
    buffer_size_before = len(data_buffer)
    data_buffer.clear()
    
    # 将平均值添加回缓冲区
    data_buffer.append((new_timestamp, avg_distance_2, avg_distance_3, avg_coords))
    
    write_log(f"[缓冲] 计算缓冲区平均值 ({buffer_size_before}个点)，保留在缓冲区: d2={avg_distance_2:.3f}m, d3={avg_distance_3:.3f}m | x={avg_coords[0]:.3f}, z={avg_coords[2]:.3f}")

# 检查数据变化并处理缓冲区
def process_data_with_buffer(timestamp, distance_2, distance_3, cam_coord):
    global last_recorded_data, data_buffer, recorded_data, invalid_data_count, had_valid_data
    
    # 提取需要比较的关键数据
    current_data = (distance_2, distance_3, cam_coord[0], cam_coord[2])
    change_names = ["Distance_2", "Distance_3", "X", "Z"]
    
    # 记录原始数据到日志
    write_log(f"[原始] A69: d2={distance_2:.3f}m, d3={distance_3:.3f}m | Vive: x={cam_coord[0]:.3f}, y={cam_coord[1]:.3f}, z={cam_coord[2]:.3f}, r={cam_coord[3]:.2f}, y={cam_coord[4]:.2f}, p={cam_coord[5]:.2f}")
    
    # 如果之前有连续无效数据，且现在得到有效数据，则重置参考基准
    if invalid_data_count > 3 and had_valid_data:
        write_log(f"[系统] 检测到有效数据恢复，重置比较基准", "IMPORTANT")
        last_recorded_data = None
        invalid_data_count = 0
    
    # 判断是否需要记录这条数据
    should_record = False
    
    if last_recorded_data is None:
        # 第一条数据，直接记录
        should_record = True
        write_log(f"[系统] 首次数据，直接记录", "IMPORTANT")
        # 记录第一个数据点
        record_data_point(timestamp, distance_2, distance_3, cam_coord)
        # 更新比较基准
        last_recorded_data = current_data
        return  # 直接返回，避免重复记录
    
    # 计算每个数据的变化
    changes = [abs(current_data[i] - last_recorded_data[i]) for i in range(4)]
    
    # 生成详细的数据变化日志
    change_details = []
    for i in range(4):
        change_details.append(format_data_for_log(
            change_names[i], 
            last_recorded_data[i], 
            current_data[i], 
            changes[i]
        ))
    
    # 记录比较过程到日志
    write_log(f"[比较] {' | '.join(change_details)}")
    
    # 检查是否有任何一个数据超过其对应的最大阈值
    data_too_large = False
    for i in range(4):
        if changes[i] > max_change_threshold:
            data_too_large = True
            write_log(f"[警告] 数据变化过大 ({change_names[i]}={changes[i]:.4f} > {max_change_threshold})", "IMPORTANT")
            write_log(f"[详情] {' | '.join(change_details)}", "IMPORTANT")
            break
    
    # 如果数据变化过大，更新比较基准然后返回
    if data_too_large:
        # 更新比较基准，以避免持续触发变化过大警告
        last_recorded_data = current_data
        write_log("[系统] 检测到数据变化过大，更新比较基准但不记录数据")
        return
    
    # 检查是否有任何一个数据超过其对应的最小阈值
    significant_changes = []
    
    for i in range(4):
        # 使用对应数据的阈值
        threshold = min_change_thresholds[change_names[i]]
        
        # 检查是否超过阈值
        if changes[i] > threshold:
            significant_changes.append(f"{change_names[i]}={changes[i]:.4f}>{threshold}")
    
    # 如果有任何一个数据超过其对应的最小阈值，需要记录
    if significant_changes:
        should_record = True
        write_log(f"[信息] 检测到明显变化: {', '.join(significant_changes)}", "IMPORTANT")
        write_log(f"[详情] {' | '.join(change_details)}", "IMPORTANT")
        
        # 如果缓冲区有数据，直接记录缓冲区所有数据
        if data_buffer:
            # 记录缓冲区中的所有数据
            for item in data_buffer:
                record_data_point(*item)
            # 清空缓冲区
            write_log(f"[缓冲] 检测到明显变化，记录并清空缓冲区 ({len(data_buffer)}个点)", "IMPORTANT")
            data_buffer.clear()
    
    # 更新比较基准
    last_recorded_data = current_data
    
    # 如果不需要立即记录，则存入缓冲区
    if not should_record:
        data_buffer.append((timestamp, distance_2, distance_3, cam_coord))
        write_log(f"[缓冲] 数据添加到缓冲区 (当前大小: {len(data_buffer)}/{buffer_size})")
        
        # 如果缓冲区已满，计算平均值
        if len(data_buffer) >= buffer_size:
            process_buffer()
    else:
        # 需要立即记录当前数据点
        record_data_point(timestamp, distance_2, distance_3, cam_coord)

# 监听串口
def serial_listener():
    global recording, invalid_data_count, had_valid_data
    consecutive_failures = 0
    max_consecutive_failures = 5
    
    while running:
        if recording:
            send_a69_data_request()  # 发送 0x02 请求数据

            try:
                response = ser.read(10)  # 读取 10 字节
                parsed_data = parse_a69_data(response)

                if parsed_data:
                    distance_2, distance_3 = parsed_data
                    timestamp = time.time()

                    # 请求 Vive Tracker 位姿
                    cam_coord = tracker.get_pose_euler()
                    
                    # 处理数据（过滤、缓冲和记录）
                    process_data_with_buffer(timestamp, distance_2, distance_3, cam_coord)
                    consecutive_failures = 0  # 重置连续失败计数
                else:
                    consecutive_failures += 1
                    if consecutive_failures >= max_consecutive_failures:
                        write_log(f"[警告] 连续 {consecutive_failures} 次未收到有效数据，请检查连接", "ERROR")
                        consecutive_failures = 0  # 重置，避免持续输出警告
                    write_log("[警告] 未收到有效数据，重发请求...")

            except serial.SerialException as e:
                write_log(f"[错误] 串口读取失败: {e}", "ERROR")
                consecutive_failures += 1

        time.sleep(0.02)  # 20ms 轮询

# 停止录制时处理缓冲区数据
def finalize_recording():
    global data_buffer, recorded_data
    
    # 如果缓冲区有数据，记录所有数据
    if data_buffer:
        write_log(f"[系统] 停止录制，记录缓冲区中的所有数据 ({len(data_buffer)}个点)", "IMPORTANT")
        # 记录缓冲区中的所有数据
        for item in data_buffer:
            record_data_point(*item)
        # 清空缓冲区
        data_buffer.clear()

# 启动串口监听线程
serial_thread = threading.Thread(target=serial_listener, daemon=True)
serial_thread.start()

# 监听按键
def on_press(key):
    global recording, last_recorded_data, data_buffer, invalid_data_count, had_valid_data
    try:
        if key.char == 'r' and not recording:
            recording = True
            # 重置数据相关变量
            last_recorded_data = None
            data_buffer.clear()
            invalid_data_count = 0
            had_valid_data = False
            write_log("[系统] 开始录制数据...", "IMPORTANT")
            # 显示当前使用的阈值设置
            thresholds_info = ", ".join([f"{k}: {v}" for k, v in min_change_thresholds.items()])
            write_log(f"[系统] 使用的最小变化阈值: {thresholds_info}", "IMPORTANT")
            write_log(f"[系统] 最大变化阈值: {max_change_threshold}", "IMPORTANT")
        elif key.char == 'e' and recording:
            recording = False
            # 处理缓冲区中的数据
            finalize_recording()
            write_log("[系统] 停止录制数据", "IMPORTANT")
            # 保存当前所有数据
            save_to_csv(recorded_data)
    except AttributeError:
        pass

# 监听退出
def on_release(key):
    global running
    if key == keyboard.Key.esc:
        write_log("[系统] 程序退出，保存所有数据...", "IMPORTANT")
        # 处理缓冲区中的数据
        finalize_recording()
        save_to_csv(recorded_data)
        write_log(f"[系统] 数据已保存到 {CSV_FILE}", "IMPORTANT")
        running = False
        return False  # 停止键盘监听

# 启动监听
listener = keyboard.Listener(on_press=on_press, on_release=on_release)
listener.start()

write_log("按 'R' 开始录制，按 'E' 停止录制，按 'Esc' 退出并保存数据.", "IMPORTANT")
# 显示当前使用的阈值设置
thresholds_info = ", ".join([f"{k}: {v}" for k, v in min_change_thresholds.items()])
write_log(f"[系统] 配置的最小变化阈值: {thresholds_info}", "IMPORTANT")
write_log(f"[系统] 最大变化阈值: {max_change_threshold}", "IMPORTANT")

try:
    while running:
        time.sleep(0.1)  # 降低 CPU 占用
except KeyboardInterrupt:
    write_log("[系统] 接收到 Ctrl+C，程序退出...", "IMPORTANT")
    # 处理缓冲区中的数据
    finalize_recording()
    save_to_csv(recorded_data)
    running = False
finally:
    if ser and ser.is_open:
        ser.close()
        write_log("[系统] 串口已关闭", "IMPORTANT")