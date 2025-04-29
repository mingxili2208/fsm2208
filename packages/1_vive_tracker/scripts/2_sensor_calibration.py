import sys
import time
import struct
import serial
import threading
import queue
import os
import numpy as np
import pandas as pd
from datetime import datetime
from pynput import keyboard
from vive_tracker import ViveTrackerModule
from scipy.spatial.transform import Rotation as R

# 全局参数配置
# 串口配置
SERIAL_PORT = "/dev/ttyUSB0"  # 请根据实际情况修改
BAUD_RATE = 9600

# 设定追踪器的名称
TRACKER_NAME = "tracker_1"
# 使用时间戳命名日志目录
LOG_DIR_PREFIX = "tracker_session_"
RAW_LOG_NAME = "raw.txt"
FILTERED_LOG_NAME = "filtered.txt"
RAW_CORRECTED_NAME = "raw_corrected.csv"
FILTERED_CORRECTED_NAME = "filtered_corrected.csv"
EULER_CALIBRATION_NAME = "euler.csv"
STATS_NAME = "stats.txt"

# 标定参数
DISTANCE2_THRESHOLD = 0.05  # distance_2允许的误差范围(m)
CALIBRATION_MODE = 0  # 0:distance_3/Z校准模式, 1:distance_2/X校准模式
FILTER_WINDOW_SIZE = 5  # 移动平均窗口大小

# 墙面参考距离 - 这些值可以在实际环境中修改
WALL_A_DISTANCE = 4.250  # A墙面的参考距离(m)
WALL_B_DISTANCE = 1.660  # B墙面的参考距离(m)

# 激光器固定补偿
LASER_OFFSET = 0.05  # 激光器距离补偿值(m)

# 标准Yaw角度 - 这些值可以根据实际环境修改
STANDARD_YAW_NEG = 41.2  # 垂直于A墙时的标准反向Yaw角度
STANDARD_YAW_POS = -139.1    # 垂直于A墙时的标准正向Yaw角度
YAW_TOLERANCE = 10         # Yaw角度容许误差(度)

# Euler校准相关参数
EULER_CALIBRATION_MODE = False  # Euler校准模式标志
EULER_DESIRED_ANGLES = [90, -90, 0, 45, -45, 135, -135]  # 期望的角度
EULER_POINTS = 3  # 需要测量的点位数量
EULER_DATA = []  # 存储Euler校准数据的列表
CURRENT_POINT = 0  # 当前点位索引
CURRENT_ANGLE_INDEX = 0  # 当前角度索引

# 状态控制
running = True
recording = False  # 录制标志
standby_mode = True  # 待机模式标志

# 线程安全的队列
data_queue = queue.Queue()

# 线程锁，用于保护对tracker的访问
tracker_lock = threading.Lock()

# 存储数据 - 分别存储原始数据和过滤后的数据
raw_data = []       # 原始数据
filtered_data = []  # 过滤后的数据
distance2_baseline = None  # 记录distance_2的基准值

# 声明全局变量
tracker = None
ser = None
vtm = None

# 全局文件句柄
raw_file_handle = None
filtered_file_handle = None
raw_corrected_file_handle = None
filtered_corrected_file_handle = None
euler_file_handle = None
stats_file_handle = None
log_dir = None

# 初始化设备和连接
def initialize_devices():
    global tracker, ser, vtm
    
    # 初始化 ViveTrackerModule
    print("[系统] 正在初始化 Vive Tracker...")
    try:
        vtm = ViveTrackerModule()
        print("[系统] ViveTrackerModule 初始化成功")
        
        # 列出所有发现的设备
        print("[系统] 正在检索可用的追踪器...")
        vtm.print_discovered_objects()
        
        # 获取指定名称的追踪器
        print(f"[系统] 尝试获取名为 '{TRACKER_NAME}' 的追踪器...")
        tracker = vtm.devices.get(TRACKER_NAME)

        if tracker is None:
            print(f"[错误] Tracker '{TRACKER_NAME}' 未找到!")
            available_trackers = list(vtm.devices.keys())
            if available_trackers:
                print(f"[信息] 可用的追踪器: {available_trackers}")
                print("[提示] 您可以修改代码中的TRACKER_NAME变量以匹配上述追踪器之一")
            else:
                print("[信息] 未发现任何可用的追踪器")
            return False
        
        # 测试获取位姿数据
        print("[系统] 正在测试追踪器位姿数据获取...")
        try:
            with tracker_lock:
                test_pose = tracker.get_pose_euler()
            print(f"[成功] 追踪器位姿测试: {test_pose}")
        except Exception as e:
            print(f"[错误] 无法获取追踪器位姿: {e}")
            print("[信息] 追踪器可能存在问题或未正确连接")
            return False
        
        print(f"[成功] 已成功连接追踪器 '{TRACKER_NAME}'")
    except Exception as e:
        print(f"[错误] 初始化Vive Tracker失败: {e}")
        return False

    # 初始化串口
    try:
        print(f"[系统] 正在尝试连接串口 {SERIAL_PORT}...")
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1)
        if ser.is_open:
            print(f"[成功] 已打开串口: {SERIAL_PORT}")
            return True
        else:
            print(f"[错误] 无法打开串口 {SERIAL_PORT}")
            return False
    except serial.SerialException as e:
        print(f"[错误] 无法打开串口 {SERIAL_PORT}: {e}")
        return False
    
    return False

# 初始化日志和文件
def initialize_logs():
    global log_dir, raw_file_handle, filtered_file_handle, euler_file_handle, stats_file_handle
    global raw_file, filtered_file, raw_corrected_file, filtered_corrected_file, euler_file, stats_file
    
    try:
        # 创建日志目录
        log_dir = create_log_directory()
        print(f"[系统] 创建日志目录: {log_dir}")
        
        # 获取文件路径
        raw_file, filtered_file, raw_corrected_file, filtered_corrected_file, euler_file, stats_file = get_log_file_paths(log_dir)
        
        # 打开文件句柄并写入表头
        raw_file_handle = open(raw_file, "a")
        raw_file_handle.write("Timestamp, Distance_2(Z), Distance_3(X), X, Y, Z, Roll, Yaw, Pitch, CalibrationMode\n")
        
        filtered_file_handle = open(filtered_file, "a")
        filtered_file_handle.write("Timestamp, Distance_2(Z), Distance_3(X), X, Y, Z, Roll, Yaw, Pitch, CalibrationMode, IsValid\n")
        
        # 创建euler校准文件但暂不写入表头，等需要时再写
        euler_file_handle = open(euler_file, "a") 
        
        # 创建统计文件并写入初始信息
        stats_file_handle = open(stats_file, "a")
        stats_file_handle.write(f"=== 校准数据记录开始于 {datetime.now().strftime('%Y-%m-%d %H:%M:%S')} ===\n")
        stats_file_handle.write(f"墙面A参考距离: {WALL_A_DISTANCE}m\n")
        stats_file_handle.write(f"墙面B参考距离: {WALL_B_DISTANCE}m\n")
        stats_file_handle.write(f"激光器距离补偿: {LASER_OFFSET}m\n")
        stats_file_handle.write(f"Z轴校准允许误差: {DISTANCE2_THRESHOLD}m\n")
        stats_file_handle.write(f"移动平均窗口大小: {FILTER_WINDOW_SIZE}个采样点\n")
        stats_file_handle.write(f"标准反向Yaw角度: {STANDARD_YAW_NEG}度\n")
        stats_file_handle.write(f"标准正向Yaw角度: {STANDARD_YAW_POS}度\n")
        stats_file_handle.write(f"Yaw角度容许误差: {YAW_TOLERANCE}度\n")
        stats_file_handle.write("\n=== 数据记录 ===\n")
        stats_file_handle.flush()
        
        print("[成功] 日志文件初始化完成")
        return True
    except Exception as e:
        print(f"[错误] 初始化日志文件失败: {e}")
        return False

# 关闭所有打开的文件句柄
def close_log_files():
    global raw_file_handle, filtered_file_handle, euler_file_handle, stats_file_handle
    
    try:
        if raw_file_handle:
            raw_file_handle.close()
            print("[系统] 已关闭原始数据文件")
        
        if filtered_file_handle:
            filtered_file_handle.close()
            print("[系统] 已关闭过滤数据文件")
        
        if euler_file_handle:
            euler_file_handle.close()
            print("[系统] 已关闭Euler校准数据文件")
        
        if stats_file_handle:
            stats_file_handle.write(f"\n=== 校准数据记录结束于 {datetime.now().strftime('%Y-%m-%d %H:%M:%S')} ===\n")
            stats_file_handle.close()
            print("[系统] 已关闭统计数据文件")
    except Exception as e:
        print(f"[警告] 关闭文件句柄时出错: {e}")

# 修改配置参数
def configure_parameters():
    global DISTANCE2_THRESHOLD, FILTER_WINDOW_SIZE, WALL_A_DISTANCE, WALL_B_DISTANCE
    global LASER_OFFSET, STANDARD_YAW_NEG, STANDARD_YAW_POS, YAW_TOLERANCE, EULER_POINTS
    
    print("\n===== 参数配置 =====")
    print("按回车跳过以保持默认值")
    
    # 墙面参考距离
    try:
        input_value = input(f"墙面A参考距离 [当前值: {WALL_A_DISTANCE}m]: ")
        if input_value.strip():
            WALL_A_DISTANCE = float(input_value)
            print(f"[更新] 墙面A参考距离设为: {WALL_A_DISTANCE}m")
        
        input_value = input(f"墙面B参考距离 [当前值: {WALL_B_DISTANCE}m]: ")
        if input_value.strip():
            WALL_B_DISTANCE = float(input_value)
            print(f"[更新] 墙面B参考距离设为: {WALL_B_DISTANCE}m")
        
        # 激光器补偿
        input_value = input(f"激光器距离补偿 [当前值: {LASER_OFFSET}m]: ")
        if input_value.strip():
            LASER_OFFSET = float(input_value)
            print(f"[更新] 激光器距离补偿设为: {LASER_OFFSET}m")
        
        # 标准Yaw角度
        input_value = input(f"标准反向Yaw角度 [当前值: {STANDARD_YAW_NEG}度]: ")
        if input_value.strip():
            STANDARD_YAW_NEG = float(input_value)
            print(f"[更新] 标准反向Yaw角度设为: {STANDARD_YAW_NEG}度")
        
        input_value = input(f"标准正向Yaw角度 [当前值: {STANDARD_YAW_POS}度]: ")
        if input_value.strip():
            STANDARD_YAW_POS = float(input_value)
            print(f"[更新] 标准正向Yaw角度设为: {STANDARD_YAW_POS}度")
        
        # 过滤参数
        input_value = input(f"移动平均窗口大小 [当前值: {FILTER_WINDOW_SIZE}个采样点]: ")
        if input_value.strip():
            FILTER_WINDOW_SIZE = int(input_value)
            print(f"[更新] 移动平均窗口大小设为: {FILTER_WINDOW_SIZE}个采样点")
        
        input_value = input(f"Z轴校准模式下distance_2允许误差 [当前值: {DISTANCE2_THRESHOLD}m]: ")
        if input_value.strip():
            DISTANCE2_THRESHOLD = float(input_value)
            print(f"[更新] Z轴校准模式下distance_2允许误差设为: {DISTANCE2_THRESHOLD}m")
        
        input_value = input(f"Yaw角度容许误差 [当前值: {YAW_TOLERANCE}度]: ")
        if input_value.strip():
            YAW_TOLERANCE = float(input_value)
            print(f"[更新] Yaw角度容许误差设为: {YAW_TOLERANCE}度")
        
        # Euler校准参数
        input_value = input(f"Euler校准点位数量 [当前值: {EULER_POINTS}个]: ")
        if input_value.strip():
            EULER_POINTS = int(input_value)
            print(f"[更新] Euler校准点位数量设为: {EULER_POINTS}个")
            
        print("\n[完成] 参数配置已更新")
        
        # 更新参数到统计文件
        if stats_file_handle:
            stats_file_handle.write("\n=== 参数更新 ===\n")
            stats_file_handle.write(f"墙面A参考距离: {WALL_A_DISTANCE}m\n")
            stats_file_handle.write(f"墙面B参考距离: {WALL_B_DISTANCE}m\n")
            stats_file_handle.write(f"激光器距离补偿: {LASER_OFFSET}m\n")
            stats_file_handle.write(f"Z轴校准允许误差: {DISTANCE2_THRESHOLD}m\n")
            stats_file_handle.write(f"移动平均窗口大小: {FILTER_WINDOW_SIZE}个采样点\n")
            stats_file_handle.write(f"标准反向Yaw角度: {STANDARD_YAW_NEG}度\n")
            stats_file_handle.write(f"标准正向Yaw角度: {STANDARD_YAW_POS}度\n")
            stats_file_handle.write(f"Yaw角度容许误差: {YAW_TOLERANCE}度\n")
            stats_file_handle.write(f"Euler校准点位数量: {EULER_POINTS}个\n")
            stats_file_handle.flush()
        
        return True
    
    except ValueError as e:
        print(f"[错误] 输入格式不正确: {e}")
        return False

# 计算 A69 校验和（XOR 校验）
def calculate_checksum_r(data):
    """计算 A69 校验和（XOR）"""
    return data[3] ^ data[4] ^ data[5] ^ data[6]

# 解析 A69 设备数据
def parse_a69_data(response):
    """解析 A69 设备数据包"""
    if len(response) != 10 or response[0] != 0x55 or response[1] != 0x7E or response[8] != 0x7E or response[9] != 0x55:
        print(f"[错误] 无效数据包: {list(map(hex, response))}")
        return None

    checksum = response[7]
    computed_checksum = calculate_checksum_r(response)

    if checksum != computed_checksum:
        print(f"[错误] 校验失败: 计算值 {hex(computed_checksum)}, 收到值 {hex(checksum)}")
        return None

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
    print(f"发送 A69 数据: {list(map(hex, tx_buf))}")

def send_a69_data_request():
    """按照 A69 格式发送数据请求，并正确计算校验位"""
    tx_buf = bytearray([0x55, 0x7E, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x7E, 0x55])
    
    # 计算校验位
    tx_buf[7] = calculate_checksum_r(tx_buf)

    ser.write(tx_buf)
    ser.flush()

# 创建带有时间戳的日志目录
def create_log_directory():
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    log_dir = f"{LOG_DIR_PREFIX}{timestamp}"
    
    # 创建目录
    if not os.path.exists(log_dir):
        os.makedirs(log_dir)
    
    return log_dir

# 获取日志文件路径
def get_log_file_paths(log_dir):
    raw_file = os.path.join(log_dir, RAW_LOG_NAME)
    filtered_file = os.path.join(log_dir, FILTERED_LOG_NAME)
    raw_corrected_file = os.path.join(log_dir, RAW_CORRECTED_NAME)
    filtered_corrected_file = os.path.join(log_dir, FILTERED_CORRECTED_NAME)
    euler_file = os.path.join(log_dir, EULER_CALIBRATION_NAME)
    stats_file = os.path.join(log_dir, STATS_NAME)
    
    return raw_file, filtered_file, raw_corrected_file, filtered_corrected_file, euler_file, stats_file

# 检查数据是否符合当前校准模式
def is_valid_calibration_data(distance_2, distance_3):
    global distance2_baseline, CALIBRATION_MODE, DISTANCE2_THRESHOLD
    
    if CALIBRATION_MODE == 0:  # distance_3/Z校准模式
        if distance2_baseline is None:
            # 首次记录，设定基准值
            distance2_baseline = distance_2
            return True
        
        # 检查distance_2是否在允许误差范围内
        if abs(distance_2 - distance2_baseline) <= DISTANCE2_THRESHOLD:
            return True
        else:
            print(f"[警告] distance_2偏离基准值过大: 当前={distance_2:.3f}, 基准={distance2_baseline:.3f}, 差值={abs(distance_2 - distance2_baseline):.3f}")
            return False
    
    return True  # 在distance_2/X校准模式下，不做特殊处理

# 计算移动平均滤波
def moving_average(data_array, new_value, window_size=FILTER_WINDOW_SIZE):
    data_array.append(new_value)
    if len(data_array) > window_size:
        data_array.pop(0)
    return sum(data_array) / len(data_array)

# 显示主菜单
def display_main_menu():
    print("\n===== 传感器标定系统菜单 =====")
    print("1. 开始Z轴校准 (T键)")
    print("2. X/Z轴校准切换或暂停 (P键)")
    print("3. 进入Euler校准模式 (E键)")
    print("4. 修改配置参数 (C键)")
    print("5. 退出并保存数据 (Esc键)")
    print("==============================")
    print("当前配置:")
    print(f"- 墙面A参考距离: {WALL_A_DISTANCE}m")
    print(f"- 墙面B参考距离: {WALL_B_DISTANCE}m")
    print(f"- 激光器距离补偿: {LASER_OFFSET}m")
    print(f"- Z轴校准允许误差: {DISTANCE2_THRESHOLD}m")
    print(f"- 移动平均窗口大小: {FILTER_WINDOW_SIZE}个采样点")
    print("==============================")
    print("请选择操作...")

# 将原始数据写入文件
def write_raw_data_to_file(timestamp, distance_2, distance_3, coord, mode):
    global raw_file_handle
    try:
        if raw_file_handle:
            log_entry = (f"{timestamp:.3f}, {distance_2:.3f}, {distance_3:.3f}, "
                         f"{coord[0]:.4f}, {coord[1]:.4f}, {coord[2]:.4f}, "
                         f"{coord[3]:.4f}, {coord[4]:.4f}, {coord[5]:.4f}, "
                         f"{mode}\n")
            raw_file_handle.write(log_entry)
            raw_file_handle.flush()  # 立即写入磁盘，防止数据丢失
    except Exception as e:
        print(f"[错误] 写入原始数据失败: {e}")

# 将过滤后的数据写入文件
def write_filtered_data_to_file(timestamp, distance_2, distance_3, coord, mode, valid):
    global filtered_file_handle
    try:
        if filtered_file_handle:
            valid_str = "1" if valid else "0"
            log_entry = (f"{timestamp:.3f}, {distance_2:.3f}, {distance_3:.3f}, "
                         f"{coord[0]:.4f}, {coord[1]:.4f}, {coord[2]:.4f}, "
                         f"{coord[3]:.4f}, {coord[4]:.4f}, {coord[5]:.4f}, "
                         f"{mode}, {valid_str}\n")
            filtered_file_handle.write(log_entry)
            filtered_file_handle.flush()  # 立即写入磁盘，防止数据丢失
    except Exception as e:
        print(f"[错误] 写入过滤数据失败: {e}")

# 监听串口
def serial_listener():
    global recording, CALIBRATION_MODE, distance2_baseline, standby_mode
    
    # 用于滤波的数据存储
    distance2_buffer = []
    distance3_buffer = []
    x_buffer = []
    y_buffer = []
    z_buffer = []
    roll_buffer = []
    yaw_buffer = []
    pitch_buffer = []
    
    # 用于定期写入文件的计数器
    record_counter = 0
    flush_interval = 5  # 每5条数据强制刷新一次文件
    
    while running:
        if recording and not EULER_CALIBRATION_MODE and not standby_mode:
            send_a69_data_request()  # 发送 0x02 请求数据

            try:
                response = ser.read(10)  # 读取 10 字节
                parsed_data = parse_a69_data(response)

                if parsed_data:
                    raw_distance_2, raw_distance_3 = parsed_data
                    
                    # 请求 Vive Tracker 位姿（原始数据）
                    try:
                        with tracker_lock:
                            raw_cam_coord = tracker.get_pose_euler()
                        timestamp = time.time()
                        
                        # 存储原始数据
                        raw_data_entry = (timestamp, 
                                          raw_distance_2, 
                                          raw_distance_3, 
                                          raw_cam_coord, 
                                          CALIBRATION_MODE)
                        raw_data.append(raw_data_entry)
                        
                        # 实时写入原始数据到文件
                        write_raw_data_to_file(timestamp, raw_distance_2, raw_distance_3, raw_cam_coord, CALIBRATION_MODE)
                        
                        # 应用移动平均滤波
                        filtered_distance_2 = moving_average(distance2_buffer, raw_distance_2)
                        filtered_distance_3 = moving_average(distance3_buffer, raw_distance_3)
                        
                        # 对Vive Tracker位姿数据也应用滤波
                        filtered_x = moving_average(x_buffer, raw_cam_coord[0])
                        filtered_y = moving_average(y_buffer, raw_cam_coord[1])
                        filtered_z = moving_average(z_buffer, raw_cam_coord[2])
                        filtered_roll = moving_average(roll_buffer, raw_cam_coord[3])
                        filtered_yaw = moving_average(yaw_buffer, raw_cam_coord[4])
                        filtered_pitch = moving_average(pitch_buffer, raw_cam_coord[5])
                        
                        filtered_cam_coord = (
                            filtered_x, filtered_y, filtered_z,
                            filtered_roll, filtered_yaw, filtered_pitch
                        )
                        
                        # 检查滤波后的数据是否符合当前校准模式的要求
                        data_valid = is_valid_calibration_data(filtered_distance_2, filtered_distance_3)
                        
                        calibration_status = "有效" if data_valid else "误差过大"
                        current_mode = "Z轴(distance_3)校准" if CALIBRATION_MODE == 0 else "X轴(distance_2)校准"
                        
                        # 显示原始数据和滤波后的数据对比
                        print(f"\n[数据] 模式: {current_mode} | 状态: {calibration_status}")
                        print(f"[原始] A69: distance_2(Z)={raw_distance_2:.3f}, distance_3(X)={raw_distance_3:.3f}")
                        print(f"[滤波] A69: distance_2(Z)={filtered_distance_2:.3f}, distance_3(X)={filtered_distance_3:.3f}")
                        print(f"[原始] Vive: x={raw_cam_coord[0]:.4f}, y={raw_cam_coord[1]:.4f}, z={raw_cam_coord[2]:.4f}")
                        print(f"[滤波] Vive: x={filtered_x:.4f}, y={filtered_y:.4f}, z={filtered_z:.4f}")
                        
                        # 存储过滤后的数据
                        if data_valid or CALIBRATION_MODE == 1:  # 在X轴校准模式下总是存储
                            filtered_data_entry = (
                                timestamp, 
                                filtered_distance_2, 
                                filtered_distance_3, 
                                filtered_cam_coord, 
                                CALIBRATION_MODE, 
                                data_valid  # 额外存储数据有效性标志
                            )
                            filtered_data.append(filtered_data_entry)
                            
                            # 实时写入过滤后的数据到文件
                            write_filtered_data_to_file(timestamp, filtered_distance_2, filtered_distance_3, 
                                                       filtered_cam_coord, CALIBRATION_MODE, data_valid)
                            
                            # 记录到统计文件
                            record_counter += 1
                            if record_counter % 10 == 0:  # 每10条记录一次统计
                                if stats_file_handle:
                                    now = datetime.now().strftime('%H:%M:%S')
                                    stats_file_handle.write(f"[{now}] 已记录 {record_counter} 条有效数据，当前模式: {current_mode}\n")
                                    stats_file_handle.flush()
                    except Exception as e:
                        print(f"[错误] 获取或处理位姿数据时出错: {e}")
                else:
                    print("[警告] 未收到有效数据，重发请求...")

            except serial.SerialException as e:
                print(f"[错误] 串口读取失败: {e}")

        time.sleep(0.02)  # 20ms 轮询

# 显示Euler校准指引信息
def display_euler_guidance():
    global CURRENT_POINT, CURRENT_ANGLE_INDEX, EULER_DATA, EULER_DESIRED_ANGLES
    
    # 计算已记录的角度
    completed_angles = []
    for i in range(CURRENT_ANGLE_INDEX):
        completed_angles.append(EULER_DESIRED_ANGLES[i])
    
    # 判断当前需要记录的角度
    current_angle = EULER_DESIRED_ANGLES[CURRENT_ANGLE_INDEX]
    
    # 计算本点位还需要的角度
    remaining_angles = []
    for i in range(CURRENT_ANGLE_INDEX, len(EULER_DESIRED_ANGLES)):
        remaining_angles.append(EULER_DESIRED_ANGLES[i])
    
    print("\n===== Euler校准指引 =====")
    print(f"当前点位: {CURRENT_POINT + 1}/{EULER_POINTS}")
    print(f"当前需要的角度: {current_angle}°")
    
    if completed_angles:
        print(f"已完成的角度: {completed_angles}")
    
    if remaining_angles[1:]:  # 排除当前角度
        print(f"本点位剩余角度: {remaining_angles[1:]}")
    
    print(f"本点位进度: {CURRENT_ANGLE_INDEX}/{len(EULER_DESIRED_ANGLES)}")
    print(f"总进度: {len(EULER_DATA)}/{EULER_POINTS * len(EULER_DESIRED_ANGLES)}")
    print("请将传感器放置在指定角度，然后按 'A' 记录数据")
    print("如有错误，可按 'D' 删除上一条记录")
    print("===========================")

# 记录Euler校准数据
def record_euler_data():
    global CURRENT_POINT, CURRENT_ANGLE_INDEX, EULER_DATA, EULER_DESIRED_ANGLES, EULER_CALIBRATION_MODE, tracker
    global euler_file_handle
    
    # 检查tracker是否有效
    if tracker is None:
        print("[错误] Tracker对象不可用，无法记录数据")
        return
    
    try:
        # 获取当前的Vive Tracker位姿
        print("[调试] 正在尝试获取Vive Tracker位姿...")
        with tracker_lock:
            pose = tracker.get_pose_euler()
        print(f"[调试] 成功获取位姿: {pose}")
        
        # 获取当前期望角度
        desired_angle = EULER_DESIRED_ANGLES[CURRENT_ANGLE_INDEX]
        
        # 记录时间戳
        timestamp = time.time()
        
        # 创建记录
        record = (timestamp, pose[0], pose[1], pose[2], pose[3], pose[4], pose[5], desired_angle)
        EULER_DATA.append(record)
        
        # 如果是第一条数据，先写入表头
        if len(EULER_DATA) == 1 and euler_file_handle:
            euler_file_handle.write("Timestamp,X,Y,Z,Roll,Yaw,Pitch,DesiredAngle\n")
        
        # 实时写入Euler数据
        if euler_file_handle:
            log_entry = f"{timestamp:.3f},{pose[0]:.4f},{pose[1]:.4f},{pose[2]:.4f},{pose[3]:.4f},{pose[4]:.4f},{pose[5]:.4f},{desired_angle}\n"
            euler_file_handle.write(log_entry)
            euler_file_handle.flush()
        
        print(f"[已记录] 角度 {desired_angle}° 的位姿数据")
        
        # 更新索引
        CURRENT_ANGLE_INDEX += 1
        
        # 检查是否完成当前点位的所有角度
        if CURRENT_ANGLE_INDEX >= len(EULER_DESIRED_ANGLES):
            CURRENT_ANGLE_INDEX = 0
            CURRENT_POINT += 1
            print(f"[完成] 点位 {CURRENT_POINT}/{EULER_POINTS} 的所有角度!")
            
            # 记录到统计文件
            if stats_file_handle:
                now = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
                stats_file_handle.write(f"[{now}] Euler校准: 完成点位 {CURRENT_POINT}/{EULER_POINTS}\n")
                stats_file_handle.flush()
            
            # 检查是否完成所有点位
            if CURRENT_POINT >= EULER_POINTS:
                print("\n[完成] 所有 Euler 校准数据已采集!")
                print("请检查数据是否正确。若正确，请按 'S' 保存数据并退出校准模式")
                return
        
        # 显示下一个角度的指引
        display_euler_guidance()
    except Exception as e:
        print(f"[错误] 记录Euler数据时发生异常: {e}")
        import traceback
        traceback.print_exc()  # 打印完整的堆栈跟踪

# 删除上一条Euler校准数据
def delete_last_euler_data():
    global CURRENT_POINT, CURRENT_ANGLE_INDEX, EULER_DATA, euler_file_handle
    
    if not EULER_DATA:
        print("[警告] 没有可删除的记录")
        return
    
    # 删除最后一条记录
    deleted_record = EULER_DATA.pop()
    
    # 更新索引
    if CURRENT_ANGLE_INDEX > 0:
        CURRENT_ANGLE_INDEX -= 1
    else:
        if CURRENT_POINT > 0:
            CURRENT_POINT -= 1
            CURRENT_ANGLE_INDEX = len(EULER_DESIRED_ANGLES) - 1
    
    print(f"[已删除] 角度 {deleted_record[7]}° 的位姿数据")
    
    # 这里需要重写整个Euler文件，因为我们要移除最后一行
    if euler_file_handle:
        # 关闭当前文件句柄
        euler_file_handle.close()
        
        # 用写模式重新打开
        euler_file_handle = open(euler_file, "w")
        
        # 写入表头和所有保留的数据
        euler_file_handle.write("Timestamp,X,Y,Z,Roll,Yaw,Pitch,DesiredAngle\n")
        for rec in EULER_DATA:
            log_entry = f"{rec[0]:.3f},{rec[1]:.4f},{rec[2]:.4f},{rec[3]:.4f},{rec[4]:.4f},{rec[5]:.4f},{rec[6]:.4f},{rec[7]}\n"
            euler_file_handle.write(log_entry)
        euler_file_handle.flush()
    
    # 记录到统计文件
    if stats_file_handle:
        now = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
        stats_file_handle.write(f"[{now}] Euler校准: 删除角度 {deleted_record[7]}° 的记录\n")
        stats_file_handle.flush()
    
    # 显示当前状态
    display_euler_guidance()

# 保存Euler校准数据
def save_euler_data():
    global EULER_DATA, stats_file_handle
    
    if not EULER_DATA:
        print("[警告] 没有Euler校准数据可保存")
        return 0
    
    # Euler数据已经实时写入文件，这里只需更新统计信息
    if stats_file_handle:
        now = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
        stats_file_handle.write(f"[{now}] Euler校准完成: 共记录 {len(EULER_DATA)} 条数据\n")
        stats_file_handle.flush()
    
    print(f"[成功] Euler校准数据已保存")
    return len(EULER_DATA)

# 监听按键
def on_press(key):
    global recording, CALIBRATION_MODE, distance2_baseline, EULER_CALIBRATION_MODE, standby_mode
    global CURRENT_POINT, CURRENT_ANGLE_INDEX, EULER_DATA, tracker, stats_file_handle  # 确保global声明在函数开头
    
    try:
        # 待机模式下的键位
        if standby_mode:
            if key.char == 't':
                # 确保tracker可用
                if tracker is None:
                    print("[错误] Tracker未初始化或不可用，请先检查连接")
                    return
                
                # 开始Z轴校准
                standby_mode = False
                recording = True
                CALIBRATION_MODE = 0  # Z轴校准模式
                distance2_baseline = None  # 重置基准值
                print("\n[系统] 开始录制数据... 进入Z轴(distance_3)校准模式")
                print("[提示] 请沿Z轴方向移动，保持X轴(distance_2)稳定")
                
                # 记录到统计文件
                if stats_file_handle:
                    now = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
                    stats_file_handle.write(f"[{now}] 开始Z轴校准\n")
                    stats_file_handle.flush()
                
            elif key.char == 'e':
                # 确保tracker可用
                if tracker is None:
                    print("[错误] Tracker未初始化或不可用，请先检查连接")
                    return
                
                # 进入Euler校准模式
                standby_mode = False
                EULER_CALIBRATION_MODE = True
                # 重置Euler校准参数
                CURRENT_POINT = 0
                CURRENT_ANGLE_INDEX = 0
                EULER_DATA = []
                print("\n[系统] 进入Euler校准模式")
                
                # 记录到统计文件
                if stats_file_handle:
                    now = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
                    stats_file_handle.write(f"[{now}] 进入Euler校准模式\n")
                    stats_file_handle.flush()
                
                display_euler_guidance()
                
            elif key.char == 'c':
                # 修改配置参数
                configure_parameters()
                display_main_menu()
        
        # 普通校准模式的键位
        elif not EULER_CALIBRATION_MODE:
            if key.char == 't' and not recording:
                # 确保tracker可用
                if tracker is None:
                    print("[错误] Tracker未初始化或不可用，请先检查连接")
                    return
                
                # 开始Z轴校准
                recording = True
                CALIBRATION_MODE = 0
                distance2_baseline = None  # 重置基准值
                print("\n[系统] 开始录制数据... 进入Z轴(distance_3)校准模式")
                print("[提示] 请沿Z轴方向移动，保持X轴(distance_2)稳定")
                
                # 记录到统计文件
                if stats_file_handle:
                    now = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
                    stats_file_handle.write(f"[{now}] 开始Z轴校准\n")
                    stats_file_handle.flush()
                
            elif key.char == 'p' and recording:
                # 切换校准模式或暂停
                if CALIBRATION_MODE == 0:
                    CALIBRATION_MODE = 1  # 切换到X轴校准模式
                    print("\n[系统] 切换到X轴(distance_2)校准模式")
                    print("[提示] 现在可以沿X轴方向移动了")
                    
                    # 记录到统计文件
                    if stats_file_handle:
                        now = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
                        stats_file_handle.write(f"[{now}] 切换到X轴校准模式\n")
                        stats_file_handle.flush()
                else:
                    recording = False
                    standby_mode = True
                    print("\n[系统] 暂停录制数据，返回待机状态")
                    
                    # 记录到统计文件
                    if stats_file_handle:
                        now = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
                        stats_file_handle.write(f"[{now}] 暂停录制，返回待机\n")
                        stats_file_handle.flush()
                    
                    display_main_menu()
                
            elif key.char == 'e':
                # 确保tracker可用
                if tracker is None:
                    print("[错误] Tracker未初始化或不可用，请先检查连接")
                    return
                
                # 切换到Euler校准模式
                recording = False
                EULER_CALIBRATION_MODE = True
                # 重置Euler校准参数
                CURRENT_POINT = 0
                CURRENT_ANGLE_INDEX = 0
                EULER_DATA = []
                print("\n[系统] 进入Euler校准模式")
                
                # 记录到统计文件
                if stats_file_handle:
                    now = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
                    stats_file_handle.write(f"[{now}] 进入Euler校准模式\n")
                    stats_file_handle.flush()
                
                display_euler_guidance()
                
            elif key.char == 'c':
                # 返回待机并修改参数
                recording = False
                standby_mode = True
                configure_parameters()
                display_main_menu()
        
        # Euler校准模式的键位
        else:
            if key.char == 'a':
                # 确保tracker可用
                if tracker is None:
                    print("[错误] Tracker未初始化或不可用，请先检查连接")
                    return
                
                # 记录当前姿态
                print("[调试] 开始记录Euler数据...")
                record_euler_data()
                
            elif key.char == 'd':
                # 删除上一条记录
                delete_last_euler_data()
                
            elif key.char == 's' and CURRENT_POINT >= EULER_POINTS:
                # 完成所有点位后，保存数据并退出Euler校准模式
                save_euler_data()
                EULER_CALIBRATION_MODE = False
                standby_mode = True
                print("\n[系统] 退出Euler校准模式，返回待机状态")
                
                # 记录到统计文件
                if stats_file_handle:
                    now = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
                    stats_file_handle.write(f"[{now}] 退出Euler校准模式，返回待机\n")
                    stats_file_handle.flush()
                
                display_main_menu()
                
    except AttributeError:
        # 可能是特殊键
        pass
    except Exception as e:
        print(f"[错误] 按键处理时发生异常: {e}")
        import traceback
        traceback.print_exc()

# 保存数据到文件
def save_data_to_file(filename, data, is_filtered=False):
    with open(filename, "w") as f:
        if is_filtered:
            f.write("Timestamp, Distance_2(Z), Distance_3(X), X, Y, Z, Roll, Yaw, Pitch, CalibrationMode, IsValid\n")
            for timestamp, dis2, dis3, coord, mode, valid in data:
                valid_str = "1" if valid else "0"
                log_entry = (f"{timestamp:.3f}, {dis2:.3f}, {dis3:.3f}, "
                             f"{coord[0]:.4f}, {coord[1]:.4f}, {coord[2]:.4f}, "
                             f"{coord[3]:.4f}, {coord[4]:.4f}, {coord[5]:.4f}, "
                             f"{mode}, {valid_str}\n")
                f.write(log_entry)
        else:
            f.write("Timestamp, Distance_2(Z), Distance_3(X), X, Y, Z, Roll, Yaw, Pitch, CalibrationMode\n")
            for timestamp, dis2, dis3, coord, mode in data:
                log_entry = (f"{timestamp:.3f}, {dis2:.3f}, {dis3:.3f}, "
                             f"{coord[0]:.4f}, {coord[1]:.4f}, {coord[2]:.4f}, "
                             f"{coord[3]:.4f}, {coord[4]:.4f}, {coord[5]:.4f}, "
                             f"{mode}\n")
                f.write(log_entry)
    return len(data)

# 计算修正后的垂直距离
def compute_corrected_distances(laser_x, laser_z, yaw):
    """
    Calculate corrected perpendicular distances based on the yaw angle of the tracker.
    
    When the measurement entity rotates, the laser measurement will no longer be 
    perpendicular to the wall, requiring geometric correction.
    
    Args:
        laser_x: Distance measured in X direction (toward wall A)
        laser_z: Distance measured in Z direction (toward wall B)
        yaw: Current yaw angle of the tracker
    
    Returns:
        Tuple of corrected distances (d_perp_x, d_perp_z)
    """
    # Determine which standard angle to use based on which wall the sensor is facing
    # If yaw is closer to STANDARD_YAW_NEG, we're facing wall A in the negative direction
    # If yaw is closer to STANDARD_YAW_POS, we're facing wall A in the positive direction
    
    if abs(yaw - STANDARD_YAW_POS) < abs(yaw - STANDARD_YAW_NEG):
        # We're closer to the positive standard angle
        theta = np.radians(yaw - STANDARD_YAW_POS)
    else:
        # We're closer to the negative standard angle
        theta = np.radians(yaw - STANDARD_YAW_NEG)
    
    # Apply cosine correction to get perpendicular distances
    d_perp_x = laser_x * np.cos(theta)   # X direction corrected distance
    d_perp_z = laser_z * np.cos(theta)   # Z direction corrected distance
    
    return round(d_perp_x, 3), round(d_perp_z, 3)  # Keep 3 decimal places

# 应用数据后处理并保存修正后的数据 - 处理原始数据
def process_raw_data_corrected(output_filename):
    """处理原始数据，应用激光测距方向修正，并保存为CSV格式"""
    try:
        # 从原始数据生成DataFrame
        records = []
        for timestamp, distance_2, distance_3, coord, mode in raw_data:
            records.append({
                "Timestamp": timestamp,
                "Distance_2": distance_2,  # Z方向
                "Distance_3": distance_3,  # X方向
                "X": coord[0],
                "Y": coord[1],
                "Z": coord[2],
                "Roll": coord[3],
                "Yaw": coord[4],
                "Pitch": coord[5],
                "CalibrationMode": mode,
            })
        
        if not records:
            print("[警告] 没有原始数据用于处理")
            return 0
            
        df = pd.DataFrame(records)
        
        # 重命名列以更清晰地表示物理含义
        df.rename(columns={"Distance_2": "d_laser_z", "Distance_3": "d_laser_x"}, inplace=True)
        
        # 应用激光器固定补偿
        df["d_laser_x"] = df["d_laser_x"] + LASER_OFFSET
        df["d_laser_z"] = df["d_laser_z"] + LASER_OFFSET
        
        # 计算修正后的数据
        corrected_data = [compute_corrected_distances(lx, lz, yaw)
                          for lx, lz, yaw in zip(df["d_laser_x"], df["d_laser_z"], df["Yaw"])]
        
        # 将计算结果添加到DataFrame
        df["laser_x"], df["laser_z"] = zip(*corrected_data)
        
        # 应用墙面参考距离
        df["laser_x"] = WALL_A_DISTANCE - df["laser_x"]
        df["laser_z"] = WALL_B_DISTANCE - df["laser_z"]
        
        # 筛选Yaw角度在有效范围内的数据
        condition1 = abs(df["Yaw"] + STANDARD_YAW_NEG) <= YAW_TOLERANCE  # 筛选yaw在反向范围的数据
        condition2 = abs(df["Yaw"] - STANDARD_YAW_POS) <= YAW_TOLERANCE  # 筛选yaw在正向范围的数据
        df = df[condition1 | condition2]
        
        # 重新排列列顺序并四舍五入
        df = df[["Timestamp", "d_laser_x", "d_laser_z", "laser_x", "laser_z", 
                 "X", "Y", "Z", "Yaw", "Roll", "Pitch", "CalibrationMode"]]
        df = df.round(4)  # 所有数值列保留 4 位小数
        
        # 保存修正后的数据
        df.to_csv(output_filename, index=False, encoding="utf-8")
        
        return len(df)
    
    except Exception as e:
        print(f"[错误] 处理原始数据时发生错误: {e}")
        import traceback
        traceback.print_exc()
        return 0

# 应用数据后处理并保存修正后的数据 - 处理过滤后的数据
def process_filtered_data_corrected(output_filename):
    """处理过滤后的数据，应用激光测距方向修正，并保存为CSV格式"""
    try:
        # 从已过滤的数据生成DataFrame
        records = []
        for timestamp, distance_2, distance_3, coord, mode, valid in filtered_data:
            records.append({
                "Timestamp": timestamp,
                "Distance_2": distance_2,  # Z方向
                "Distance_3": distance_3,  # X方向
                "X": coord[0],
                "Y": coord[1],
                "Z": coord[2],
                "Roll": coord[3],
                "Yaw": coord[4],
                "Pitch": coord[5],
                "CalibrationMode": mode,
                "IsValid": valid
            })
        
        if not records:
            print("[警告] 没有过滤后的数据用于处理")
            return 0
            
        df = pd.DataFrame(records)
        
        # 重命名列以更清晰地表示物理含义
        df.rename(columns={"Distance_2": "d_laser_z", "Distance_3": "d_laser_x"}, inplace=True)
        
        # 应用激光器固定补偿
        df["d_laser_x"] = df["d_laser_x"] + LASER_OFFSET
        df["d_laser_z"] = df["d_laser_z"] + LASER_OFFSET
        
        # 计算修正后的数据
        corrected_data = [compute_corrected_distances(lx, lz, yaw)
                          for lx, lz, yaw in zip(df["d_laser_x"], df["d_laser_z"], df["Yaw"])]
        
        # 将计算结果添加到DataFrame
        df["laser_x"], df["laser_z"] = zip(*corrected_data)
        
        # 应用墙面参考距离
        df["laser_x"] = WALL_A_DISTANCE - df["laser_x"]
        df["laser_z"] = WALL_B_DISTANCE - df["laser_z"]
        
        # 筛选Yaw角度在有效范围内的数据
        condition1 = abs(df["Yaw"] + STANDARD_YAW_NEG) <= YAW_TOLERANCE  # 筛选yaw在反向范围的数据
        condition2 = abs(df["Yaw"] - STANDARD_YAW_POS) <= YAW_TOLERANCE  # 筛选yaw在正向范围的数据
        df = df[condition1 | condition2]
        
        # 重新排列列顺序并四舍五入
        df = df[["Timestamp", "d_laser_x", "d_laser_z", "laser_x", "laser_z", 
                 "X", "Y", "Z", "Yaw", "Roll", "Pitch", "CalibrationMode", "IsValid"]]
        df = df.round(4)  # 所有数值列保留 4 位小数
        
        # 保存修正后的数据
        df.to_csv(output_filename, index=False, encoding="utf-8")
        
        return len(df)
    
    except Exception as e:
        print(f"[错误] 处理过滤后数据时发生错误: {e}")
        import traceback
        traceback.print_exc()
        return 0

# 分析数据质量
def analyze_data_quality(raw_data, filtered_data):
    # 分析原始数据
    raw_z_mode_data = [(dis2, dis3, coord) for _, dis2, dis3, coord, mode in raw_data if mode == 0]
    raw_x_mode_data = [(dis2, dis3, coord) for _, dis2, dis3, coord, mode in raw_data if mode == 1]
    
    # 分析过滤后的数据
    filtered_z_mode_data = [(dis2, dis3, coord) for _, dis2, dis3, coord, mode, valid in filtered_data if mode == 0 and valid]
    filtered_x_mode_data = [(dis2, dis3, coord) for _, dis2, dis3, coord, mode, valid in filtered_data if mode == 1]
    
    print("\n=== 数据质量分析 ===")
    
    # 分析Z模式下distance_2稳定性
    if raw_z_mode_data:
        raw_dis2_values = [dis2 for dis2, _, _ in raw_z_mode_data]
        raw_dis2_std = np.std(raw_dis2_values)
        print(f"[原始] Z校准模式下distance_2标准差: {raw_dis2_std:.5f}m")
        
    if filtered_z_mode_data:
        filtered_dis2_values = [dis2 for dis2, _, _ in filtered_z_mode_data]
        filtered_dis2_std = np.std(filtered_dis2_values)
        print(f"[滤波] Z校准模式下distance_2标准差: {filtered_dis2_std:.5f}m")
        
        # 计算改进比例
        if raw_z_mode_data:
            improvement = ((raw_dis2_std - filtered_dis2_std) / raw_dis2_std) * 100
            print(f"[分析] distance_2稳定性提升: {improvement:.2f}%")
    
    # 分析X模式下distance_3稳定性
    if raw_x_mode_data:
        raw_dis3_values = [dis3 for _, dis3, _ in raw_x_mode_data]
        raw_dis3_std = np.std(raw_dis3_values)
        print(f"[原始] X校准模式下distance_3标准差: {raw_dis3_std:.5f}m")
        
    if filtered_x_mode_data:
        filtered_dis3_values = [dis3 for _, dis3, _ in filtered_x_mode_data]
        filtered_dis3_std = np.std(filtered_dis3_values)
        print(f"[滤波] X校准模式下distance_3标准差: {filtered_dis3_std:.5f}m")
        
        # 计算改进比例
        if raw_x_mode_data:
            improvement = ((raw_dis3_std - filtered_dis3_std) / raw_dis3_std) * 100
            print(f"[分析] distance_3稳定性提升: {improvement:.2f}%")
    
    # 数据点统计
    filtered_z_valid_ratio = 0
    if raw_z_mode_data:
        filtered_z_valid_ratio = len(filtered_z_mode_data) / len(raw_z_mode_data) * 100
    
    print(f"\n[统计] 原始数据总点数: {len(raw_data)}个")
    print(f"[统计] - Z模式: {len(raw_z_mode_data)}个")
    print(f"[统计] - X模式: {len(raw_x_mode_data)}个")
    print(f"[统计] 有效过滤数据总点数: {len(filtered_data)}个")
    print(f"[统计] - Z模式有效点: {len(filtered_z_mode_data)}个 ({filtered_z_valid_ratio:.1f}%保留率)")
    print(f"[统计] - X模式点: {len(filtered_x_mode_data)}个")
    
    return {
        "raw_total": len(raw_data),
        "raw_z_mode": len(raw_z_mode_data),
        "raw_x_mode": len(raw_x_mode_data),
        "filtered_total": len(filtered_data),
        "filtered_z_mode": len(filtered_z_mode_data),
        "filtered_x_mode": len(filtered_x_mode_data),
        "z_mode_retention": filtered_z_valid_ratio
    }

# 生成修正后的数据并定期更新
def generate_corrected_data():
    """定期生成修正后的数据"""
    global raw_corrected_file, filtered_corrected_file
    
    while running:
        # 每30秒生成一次修正后的数据
        if len(raw_data) > 0:
            process_raw_data_corrected(raw_corrected_file)
        
        if len(filtered_data) > 0:
            process_filtered_data_corrected(filtered_corrected_file)
            
        # 等待30秒
        time.sleep(30)

# 监听退出
def on_release(key):
    global running, EULER_CALIBRATION_MODE, standby_mode
    
    try:
        if key == keyboard.Key.esc:
            # 如果在Euler校准模式中，先退出该模式
            if EULER_CALIBRATION_MODE:
                if len(EULER_DATA) > 0:
                    print("\n[警告] 您正在退出Euler校准模式，但有未保存的数据")
                    print("按 'S' 保存数据，或再次按 'Esc' 放弃数据并退出")
                    EULER_CALIBRATION_MODE = False
                    standby_mode = True
                    display_main_menu()
                    return True
            
            print(f"\n[系统] 正在处理最终数据...")
            
            # 处理并保存原始数据的修正版本
            print(f"[系统] 正在处理并生成原始数据的修正版本...")
            raw_corrected_count = process_raw_data_corrected(raw_corrected_file)
            if raw_corrected_count > 0:
                print(f"[系统] 已保存{raw_corrected_count}条原始数据修正版到 {raw_corrected_file}")
            
            # 处理并保存过滤数据的修正版本
            print(f"[系统] 正在处理并生成过滤数据的修正版本...")
            filtered_corrected_count = process_filtered_data_corrected(filtered_corrected_file)
            if filtered_corrected_count > 0:
                print(f"[系统] 已保存{filtered_corrected_count}条过滤数据修正版到 {filtered_corrected_file}")
            
            # 保存Euler校准数据(如果有)
            euler_count = len(EULER_DATA)
            if euler_count > 0:
                print(f"[系统] 已保存{euler_count}条Euler校准数据")
            
            # 分析数据质量
            stats = analyze_data_quality(raw_data, filtered_data)
            
            # 将统计信息也保存到文件中
            if stats_file_handle:
                stats_file_handle.write("\n=== 最终数据统计 ===\n")
                stats_file_handle.write(f"原始数据总点数: {stats['raw_total']}个\n")
                stats_file_handle.write(f"- Z模式: {stats['raw_z_mode']}个\n")
                stats_file_handle.write(f"- X模式: {stats['raw_x_mode']}个\n")
                stats_file_handle.write(f"过滤后有效数据点数: {stats['filtered_total']}个\n")
                stats_file_handle.write(f"- Z模式有效点: {stats['filtered_z_mode']}个 ({stats['z_mode_retention']:.1f}%保留率)\n")
                stats_file_handle.write(f"- X模式点: {stats['filtered_x_mode']}个\n")
                
                if euler_count > 0:
                    stats_file_handle.write(f"Euler校准数据点数: {euler_count}个\n")
                
                stats_file_handle.write("\n滤波设置:\n")
                stats_file_handle.write(f"- 移动平均窗口大小: {FILTER_WINDOW_SIZE}个采样点\n")
                stats_file_handle.write(f"- Z模式下distance_2允许误差范围: {DISTANCE2_THRESHOLD}m\n")
                stats_file_handle.write("\n墙面参考设置:\n")
                stats_file_handle.write(f"- 墙面A(X方向)参考距离: {WALL_A_DISTANCE}m\n")
                stats_file_handle.write(f"- 墙面B(Z方向)参考距离: {WALL_B_DISTANCE}m\n")
                stats_file_handle.write(f"- 激光器固定补偿: {LASER_OFFSET}m\n")
                stats_file_handle.write(f"- 标准Yaw角度(负): {STANDARD_YAW_NEG}度\n")
                stats_file_handle.write(f"- 标准Yaw角度(正): {STANDARD_YAW_POS}度\n")
                stats_file_handle.write(f"- Yaw容许误差: {YAW_TOLERANCE}度\n")
                
                # 添加文件生成信息
                stats_file_handle.write("\n输出文件:\n")
                stats_file_handle.write(f"- 原始数据: {os.path.basename(raw_file)}\n")
                stats_file_handle.write(f"- 过滤数据: {os.path.basename(filtered_file)}\n")
                stats_file_handle.write(f"- 原始数据修正版: {os.path.basename(raw_corrected_file)}\n")
                stats_file_handle.write(f"- 过滤数据修正版: {os.path.basename(filtered_corrected_file)}\n")
                if euler_count > 0:
                    stats_file_handle.write(f"- Euler校准数据: {os.path.basename(euler_file)}\n")
                
            print(f"[系统] 统计信息已记录")

            # 关闭资源前先刷新所有文件
            if raw_file_handle:
                raw_file_handle.flush()
            if filtered_file_handle:
                filtered_file_handle.flush()
            if euler_file_handle:
                euler_file_handle.flush()
            if stats_file_handle:
                stats_file_handle.flush()

            running = False
            return False  # 停止键盘监听
            
    except Exception as e:
        print(f"[错误] 退出处理时发生异常: {e}")
        import traceback
        traceback.print_exc()
        running = False
        return False  # 确保在异常情况下也能退出

# 主函数
def main():
    global running
    
    print("\n===== 传感器标定系统 =====")
    print("正在初始化系统...")
    
    # 初始化设备
    if not initialize_devices():
        print("[错误] 设备初始化失败")
        user_input = input("是否继续运行程序? (y/n): ").strip().lower()
        if user_input != 'y':
            print("[系统] 程序退出")
            return
        print("[警告] 继续运行，但某些功能可能不可用")
    
    # 初始化日志和文件
    if not initialize_logs():
        print("[错误] 日志文件初始化失败")
        user_input = input("是否继续运行程序? (y/n): ").strip().lower()
        if user_input != 'y':
            print("[系统] 程序退出")
            return
        print("[警告] 继续运行，但日志记录功能可能受限")
    
    # 询问是否修改配置参数
    print("\n系统初始化完成，是否需要修改默认配置参数? (y/n)")
    choice = input().strip().lower()
    if choice == 'y':
        configure_parameters()
    
    # 显示主菜单
    display_main_menu()
    
    # 启动串口监听线程
    serial_thread = threading.Thread(target=serial_listener, daemon=True)
    serial_thread.start()
    
    # 启动定期生成修正数据的线程
    corrected_data_thread = threading.Thread(target=generate_corrected_data, daemon=True)
    corrected_data_thread.start()
    
    # 启动键盘监听
    listener = keyboard.Listener(on_press=on_press, on_release=on_release)
    listener.start()
    
    try:
        while running:
            time.sleep(0.1)  # 降低 CPU 占用
    except KeyboardInterrupt:
        print("\n[系统] 接收到键盘中断，正在退出...")
    except Exception as e:
        print(f"\n[错误] 主循环异常: {e}")
        import traceback
        traceback.print_exc()
    finally:
        # 关闭所有文件句柄
        close_log_files()
        
        # 关闭资源
        running = False
        try:
            if ser and ser.is_open:
                ser.close()
                print("[系统] 串口已关闭")
        except Exception as e:
            print(f"[警告] 关闭串口时出错: {e}")
        
        print("[系统] 程序已退出")

# 启动程序
if __name__ == "__main__":
    main()