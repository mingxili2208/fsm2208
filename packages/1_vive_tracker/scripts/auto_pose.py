import sys
import time
import struct
import serial
import threading
import queue
from pynput import keyboard
from vive_tracker import ViveTrackerModule

# 串口配置
SERIAL_PORT = "/dev/ttyUSB0"  # 请根据实际情况修改
BAUD_RATE = 115200

# 设定追踪器的名称
TRACKER_NAME = "tracker_1"
LOG_FILE = "tracker_log.txt"

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

# 存储数据
recorded_data = []
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
def send_a69_data(distance_2, distance_3):
    """按照 A69 格式发送数据"""
    distance_2_int = int(distance_2 * 1000)  # 转换为整数
    distance_3_int = int(distance_3 * 1000)

    tx_buf = bytearray(10)
    tx_buf[0] = 0x55
    tx_buf[1] = 0x7E
    tx_buf[2] = 0x03
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

# 监听串口
def serial_listener():
    global recording
    while running:
        if recording:
            send_a69_data(0.0, 0.0)  # 发送 0x03 请求数据

            try:
                response = ser.read(10)  # 读取 10 字节
                parsed_data = parse_a69_data(response)

                if parsed_data:
                    distance_2, distance_3 = parsed_data
                    timestamp = time.time()

                    print(f"[数据] A69 位姿: distance_2={distance_2:.3f}, distance_3={distance_3:.3f}")

                    # 请求 Vive Tracker 位姿
                    cam_coord = tracker.get_pose_euler()
                    print(f"[数据] Vive Tracker 位姿: x={cam_coord[0]:.4f}, y={cam_coord[1]:.4f}, z={cam_coord[2]:.4f}, "
                          f"roll={cam_coord[3]:.4f}, yaw={cam_coord[4]:.4f}, pitch={cam_coord[5]:.4f}")

                    # 存储数据
                    recorded_data.append((timestamp, distance_2, distance_3, cam_coord))
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
    global recording
    try:
        if key.char == 'r' and not recording:
            recording = True
            print("\n[系统] 开始录制数据...")
        elif key.char == 'e' and recording:
            recording = False
            print("\n[系统] 停止录制数据")
    except AttributeError:
        pass

# 监听退出
def on_release(key):
    global running
    if key == keyboard.Key.esc:
        print("\n[系统] 保存所有数据到日志文件...")
        with open(LOG_FILE, "w") as log_file:
            log_file.write("Timestamp, Distance_2, Distance_3, X, Y, Z, Roll, Yaw, Pitch\n")
            for timestamp, dis2, dis3, coord in recorded_data:
                log_entry = f"{timestamp:.3f}, {dis2:.3f}, {dis3:.3f}, {coord[0]:.4f}, {coord[1]:.4f}, {coord[2]:.4f}, {coord[3]:.4f}, {coord[4]:.4f}, {coord[5]:.4f}\n"
                log_file.write(log_entry)
        print(f"[系统] 数据已保存到 {LOG_FILE}")

        running = False
        return False  # 停止键盘监听

# 启动监听
listener = keyboard.Listener(on_press=on_press, on_release=on_release)
listener.start()

print("按 'R' 开始录制，按 'E' 停止录制，按 'Esc' 退出并保存数据.")

while running:
    time.sleep(0.1)  # 降低 CPU 占用