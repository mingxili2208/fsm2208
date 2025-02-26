import sys
import time
import struct
import serial
import threading
import queue
from pynput import keyboard
from vive_tracker import ViveTrackerModule

MESSAGE_TYPES = {
    0x00: "INITIALIZATION_FAILED",
    0x01: "CONTROL_RECEIVED",
    0x02: "CONTROL_SENT_OK",
    0x03: "CONTROL_SENT_FAILED",
    0x05: "TIMEOUT",
    0x11: "POSE_DATA",
    0x12: "POSE_REQUEST_SUCCESS",
    0x13: "POSE_REQUEST_FAILED"
}
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

# 存储数据
recorded_data = []
running = True

# 线程安全的队列
data_queue = queue.Queue()

# 计算校验和
def calculate_checksum_r(data):
    """计算校验和（从索引 1 到 9）"""
    return sum(data[1:10]) & 0xFF

def calculate_checksum(flag, steering_angle, speed):
    checksum = (0x42 + flag + int(steering_angle * 1000) + int(speed * 1000)) & 0xFF
    return checksum

# 发送串口指令
def send_serial_command(flag, steering_angle_deg=0.0, speed=0.0):
    try:
        steering_angle_deg = round(steering_angle_deg, 3)
        speed = round(speed, 3)

        checksum = calculate_checksum(flag, steering_angle_deg, speed)
        packed_msg = struct.pack('<BBffB', 0x42, flag, steering_angle_deg, speed, checksum)

        ser.write(packed_msg)
        ser.flush()

        print(f"发送指令: flag={flag}, 速度={speed}, 转向角={steering_angle_deg}")

    except struct.error as e:
        print(f"打包命令错误: {e}")
    except serial.SerialException as e:
        print(f"串口写入错误: {e}")

# 串口监听线程
def serial_listener():
    """监听串口数据"""
    while running:
        response = ser.read(11)  # 读取 11 字节完整数据包
        if len(response)>0:
            if len(response) != 11:
                print(f"[错误] 数据包长度错误: 期望 11 字节, 实际收到 {len(response)} 字节")
                if len(response) > 0:
                    print(f"[调试] 收到数据: {[hex(b) for b in response]}")
                continue

            # 检查数据包起始字节是否正确 (0x42)
            if response[0] != 0x42:
                print(f"[错误] 数据包起始字节错误: 预期 0x42, 实际收到 {hex(response[0])}")
                continue

            message_type = response[1]
            checksum = response[10]
            computed_checksum = calculate_checksum_r(response)

            # 校验和验证
            if checksum != computed_checksum:
                print(f"[错误] 校验和错误: 计算值 {hex(computed_checksum)}, 收到值 {hex(checksum)}")
                continue

            # 解析不同的状态码
            if message_type in MESSAGE_TYPES:
                print(f"[状态] 收到消息: {MESSAGE_TYPES[message_type]} ({hex(message_type)})")
            else:
                print(f"[警告] 未知的状态码: {hex(message_type)}")

            # 处理 POSE_DATA (0x11)
            if message_type == 0x11:
                distance_2 = struct.unpack('<f', response[2:6])[0]
                distance_3 = struct.unpack('<f', response[6:10])[0]
                print(f"[数据] POSE_DATA: distance_2={distance_2:.3f}, distance_3={distance_3:.3f}")
                data_queue.put((distance_2, distance_3))  # 存入队列
           

# 启动串口监听线程
serial_thread = threading.Thread(target=serial_listener, daemon=True)
serial_thread.start()

# 监听按键
def on_press(key):
    try:
        if key.char == 'r':
            print("\n开始以 50Hz 频率发送 0x02 指令，并请求位姿数据 (持续 60s)...")
            start_time = time.time()
            elapsed_time = 0

            while elapsed_time < 60:  # 持续 60 秒
                send_serial_command(0x02, steering_angle_deg=0.0, speed=0.0)  # 发送 0x02 指令

                try:
                    distance_2, distance_3 = data_queue.get(timeout=0.02)  # 20ms 内等待队列数据
                    timestamp = time.time()
                    cam_coord = tracker.get_pose_euler()

                    # 记录数据：时间戳、distance_2、distance_3、位姿
                    recorded_data.append((timestamp, distance_2, distance_3, cam_coord))

                    print(f"[{timestamp:.3f}] DIS2={distance_2:.3f}, DIS3={distance_3:.3f} | "
                        f"x={cam_coord[0]:.4f}, y={cam_coord[1]:.4f}, z={cam_coord[2]:.4f}, "
                        f"roll={cam_coord[3]:.4f}, yaw={cam_coord[4]:.4f}, pitch={cam_coord[5]:.4f}")

                except queue.Empty:
                    pass  # 超时无数据

                elapsed_time = time.time() - start_time  # 更新已运行时间
    except AttributeError:
        print("on_press_error")
# 监听退出
def on_release(key):
    global running
    if key == keyboard.Key.esc:
        print("\n保存所有数据到日志文件...")
        with open(LOG_FILE, "w") as log_file:
            log_file.write("Timestamp, Distance_2, Distance_3, X, Y, Z, Roll, Yaw, Pitch\n")
            for timestamp, dis2, dis3, coord in recorded_data:
                log_entry = (f"{timestamp:.3f}, {dis2:.3f}, {dis3:.3f}, {coord[0]:.4f}, {coord[1]:.4f}, {coord[2]:.4f}, "
                             f"{coord[3]:.4f}, {coord[4]:.4f}, {coord[5]:.4f}\n")
                log_file.write(log_entry)
        print(f"数据已保存到 {LOG_FILE}")

        running = False
        return False  # 停止键盘监听

# 启动监听
listener = keyboard.Listener(on_press=on_press, on_release=on_release)
listener.start()

print("按 'R' 以 50Hz 发送 0x02 指令并请求位姿数据（持续 60s），按 'Esc' 退出并保存数据.")

while running:
    time.sleep(0.1)  # 降低 CPU 占用