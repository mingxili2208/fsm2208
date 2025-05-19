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
# 使用时间戳命名日志文件
LOG_FILE_PREFIX = "tracker_log_"
LOG_FILE_EXT = ".txt"
RAW_LOG_SUFFIX = "_raw"
FILTERED_LOG_SUFFIX = "_filtered"
RAW_CORRECTED_SUFFIX = "_raw_corrected"
FILTERED_CORRECTED_SUFFIX = "_filtered_corrected"

# 标定参数
DISTANCE2_THRESHOLD = 0.003  # distance_2允许的误差范围(m)
CALIBRATION_MODE = 0  # 0:distance_3/Z校准模式, 1:distance_2/X校准模式
FILTER_WINDOW_SIZE = 5  # 移动平均窗口大小

# 墙面参考距离 - 这些值可以在实际环境中修改
WALL_A_DISTANCE = 4.250  # A墙面的参考距离(m)
WALL_B_DISTANCE = 1.660  # B墙面的参考距离(m)

# 激光器固定补偿
LASER_OFFSET = 0.05  # 激光器距离补偿值(m)

# 标准Yaw角度 - 这些值可以根据实际环境修改
STANDARD_YAW_NEG = -137.4  # 垂直于A墙时的标准反向Yaw角度
STANDARD_YAW_POS = 40.7    # 垂直于A墙时的标准正向Yaw角度
YAW_TOLERANCE = 10         # Yaw角度容许误差(度)

# 初始化 ViveTrackerModule
vtm = ViveTrackerModule()
vtm.print_discovered_objects()
tracker = vtm.devices.get(TRACKER_NAME)

if tracker is None:
    print(f"Error: Tracker '{TRACKER_NAME}' not found!")
    sys.exit(1)

# 初始化串口
try:
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1)
    if ser.is_open:
        print(f"成功打开串口: {SERIAL_PORT}")
except serial.SerialException as e:
    print(f"Error: 无法打开串口 {SERIAL_PORT}: {e}")
    sys.exit(1)

# 线程安全的队列
data_queue = queue.Queue()

# 存储数据 - 分别存储原始数据和过滤后的数据
raw_data = []       # 原始数据
filtered_data = []  # 过滤后的数据
distance2_baseline = None  # 记录distance_2的基准值
running = True
recording = False  # 录制标志

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

# 生成带有时间戳的日志文件名
def get_timestamped_log_filename():
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    base_filename = f"{LOG_FILE_PREFIX}{timestamp}"
    raw_filename = f"{base_filename}{RAW_LOG_SUFFIX}{LOG_FILE_EXT}"
    filtered_filename = f"{base_filename}{FILTERED_LOG_SUFFIX}{LOG_FILE_EXT}"
    raw_corrected_filename = f"{base_filename}{RAW_CORRECTED_SUFFIX}.csv"
    filtered_corrected_filename = f"{base_filename}{FILTERED_CORRECTED_SUFFIX}.csv"
    return raw_filename, filtered_filename, raw_corrected_filename, filtered_corrected_filename

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

# 监听串口
def serial_listener():
    global recording, CALIBRATION_MODE, distance2_baseline
    
    # 用于滤波的数据存储
    distance2_buffer = []
    distance3_buffer = []
    x_buffer = []
    y_buffer = []
    z_buffer = []
    roll_buffer = []
    yaw_buffer = []
    pitch_buffer = []
    
    while running:
        if recording:
            send_a69_data_request()  # 发送 0x02 请求数据

            try:
                response = ser.read(10)  # 读取 10 字节
                parsed_data = parse_a69_data(response)

                if parsed_data:
                    raw_distance_2, raw_distance_3 = parsed_data
                    
                    # 请求 Vive Tracker 位姿（原始数据）
                    raw_cam_coord = tracker.get_pose_euler()
                    timestamp = time.time()
                    
                    # 存储原始数据
                    raw_data_entry = (timestamp, 
                                      raw_distance_2, 
                                      raw_distance_3, 
                                      raw_cam_coord, 
                                      CALIBRATION_MODE)
                    raw_data.append(raw_data_entry)
                    
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
                else:
                    print("[警告] 未收到有效数据，重发请求...")

            except serial.SerialException as e:
                print(f"[错误] 串口读取失败: {e}")

        time.sleep(0.02)  # 20ms 轮询

# 启动串口监听线程
serial_thread = threading.Thread(target=serial_listener, daemon=True)
serial_thread.start()

# 监听按键
def on_press(key):
    global recording, CALIBRATION_MODE, distance2_baseline
    try:
        if key.char == 'r' and not recording:
            recording = True
            CALIBRATION_MODE = 0  # 开始Z轴校准模式
            distance2_baseline = None  # 重置基准值
            print("\n[系统] 开始录制数据... 进入Z轴(distance_3)校准模式")
            print("[提示] 请沿Z轴方向移动，保持X轴(distance_2)稳定")
            
        elif key.char == 'e' and recording:
            CALIBRATION_MODE = 1  # 切换到X轴校准模式
            print("\n[系统] 切换到X轴(distance_2)校准模式")
            print("[提示] 现在可以沿X轴方向移动了")
            
        elif key.char == 's' and recording:
            recording = False
            print("\n[系统] 停止录制数据")
            
    except AttributeError:
        pass

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
    根据测量实体的旋转(yaw角度)，计算修正后的垂直距离
    当测量实体旋转时，激光测量将不再垂直于墙面，需要进行几何修正
    """
    if yaw < 0:
        theta = np.radians(yaw + STANDARD_YAW_NEG)
    else:
        theta = np.radians(yaw - STANDARD_YAW_POS)  # 计算偏移角度（转换为弧度）

    # 旋转变换，恢复真实垂直距离
    d_perp_x = laser_x * np.cos(theta)   # X 方向的修正距离
    d_perp_z = laser_z * np.cos(theta)   # Z 方向的修正距离

    return round(d_perp_x, 3), round(d_perp_z, 3)  # 保留 3 位小数

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

# 监听退出
def on_release(key):
    global running
    if key == keyboard.Key.esc:
        # 使用带时间戳的日志文件名
        raw_file, filtered_file, raw_corrected_file, filtered_corrected_file = get_timestamped_log_filename()
        print(f"\n[系统] 正在保存数据...")
        
        # 保存原始数据
        raw_count = save_data_to_file(raw_file, raw_data, is_filtered=False)
        print(f"[系统] 已保存{raw_count}条原始数据到 {raw_file}")
        
        # 保存过滤后的数据
        filtered_count = save_data_to_file(filtered_file, filtered_data, is_filtered=True)
        print(f"[系统] 已保存{filtered_count}条过滤后数据到 {filtered_file}")
        
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
        
        # 分析数据质量
        stats = analyze_data_quality(raw_data, filtered_data)
        
        # 将统计信息也保存到文件中
        stats_file = filtered_file.replace(LOG_FILE_EXT, "_stats.txt")
        with open(stats_file, "w") as f:
            f.write("=== 校准数据统计 ===\n")
            f.write(f"原始数据总点数: {stats['raw_total']}个\n")
            f.write(f"- Z模式: {stats['raw_z_mode']}个\n")
            f.write(f"- X模式: {stats['raw_x_mode']}个\n")
            f.write(f"过滤后有效数据点数: {stats['filtered_total']}个\n")
            f.write(f"- Z模式有效点: {stats['filtered_z_mode']}个 ({stats['z_mode_retention']:.1f}%保留率)\n")
            f.write(f"- X模式点: {stats['filtered_x_mode']}个\n")
            f.write("\n滤波设置:\n")
            f.write(f"- 移动平均窗口大小: {FILTER_WINDOW_SIZE}个采样点\n")
            f.write(f"- Z模式下distance_2允许误差范围: {DISTANCE2_THRESHOLD}m\n")
            f.write("\n墙面参考设置:\n")
            f.write(f"- 墙面A(X方向)参考距离: {WALL_A_DISTANCE}m\n")
            f.write(f"- 墙面B(Z方向)参考距离: {WALL_B_DISTANCE}m\n")
            f.write(f"- 激光器固定补偿: {LASER_OFFSET}m\n")
            f.write(f"- 标准Yaw角度(负): {STANDARD_YAW_NEG}度\n")
            f.write(f"- 标准Yaw角度(正): {STANDARD_YAW_POS}度\n")
            f.write(f"- Yaw容许误差: {YAW_TOLERANCE}度\n")
            
            # 添加文件生成信息
            f.write("\n输出文件:\n")
            f.write(f"- 原始数据: {raw_file}\n")
            f.write(f"- 过滤数据: {filtered_file}\n")
            f.write(f"- 原始数据修正版: {raw_corrected_file}\n")
            f.write(f"- 过滤数据修正版: {filtered_corrected_file}\n")
            
        print(f"[系统] 统计信息已保存到 {stats_file}")

        running = False
        return False  # 停止键盘监听

# 启动监听
listener = keyboard.Listener(on_press=on_press, on_release=on_release)
listener.start()

print("\n=== 传感器标定系统 ===")
print("按 'R' 开始录制 (Z轴校准模式)")
print("按 'E' 切换到X轴校准模式")
print("按 'S' 停止录制")
print("按 'Esc' 退出并保存数据")
print("========================")
print(f"滤波窗口大小: {FILTER_WINDOW_SIZE}个采样点")
print(f"Z轴校准模式下distance_2允许误差: {DISTANCE2_THRESHOLD}m")
print("========================")

try:
    while running:
        time.sleep(0.1)  # 降低 CPU 占用
finally:
    # 关闭资源
    if ser.is_open:
        ser.close()
        print("[系统] 串口已关闭")