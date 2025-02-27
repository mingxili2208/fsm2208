import sys
import time
import serial
import threading
import queue
from pynput import keyboard

# 串口配置
SERIAL_PORT = "/dev/ttyUSB0"  # 请根据实际情况修改
BAUD_RATE = 9600
LOG_FILE = "a69_tracker_log.txt"

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
recorded_data = []
running = True
recording = False  # 录制标志

# 计算 XOR 校验和
def calculate_checksum_r(data):
    """计算 A69 设备的 XOR 校验和（适用于数据包）"""
    return data[3] ^ data[4] ^ data[5] ^ data[6]

# 解析 A69 设备数据
def parse_a69_data(response):
    """解析 A69 设备数据包，返回 (distance_2, distance_3)"""
    if len(response) != 10:
        print(f"[错误] 接收到的数据长度错误: {len(response)}，预期 10 字节")
        return None

    if response[0] != 0x55 or response[1] != 0x7E or response[8] != 0x7E or response[9] != 0x55:
        print(f"[错误] 无效数据包格式: {list(map(hex, response))}")
        return None

    checksum = response[7]
    computed_checksum = calculate_checksum_r(response)

    if checksum != computed_checksum:
        print(f"[错误] 校验失败: 计算值 {hex(computed_checksum)}, 收到值 {hex(checksum)}")
        return None

    distance_2 = (response[3] << 8) | response[4]
    distance_3 = (response[5] << 8) | response[6]

    return distance_2 / 1000.0, distance_3 / 1000.0  # 假设数据单位是 mm，转换为 m

# 发送 A69 数据请求
def send_a69_data_request():
    """按照 A69 格式发送数据请求，并正确计算校验位"""
    tx_buf = bytearray([0x55, 0x7E, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x7E, 0x55])
    
    # 计算校验位
    tx_buf[7] = calculate_checksum_r(tx_buf)

    ser.write(tx_buf)
    ser.flush()
    print(f"发送 A69 数据请求的指令码: {hex(tx_buf[2])}")
    #print(f"发送 A69 数据请求: {list(map(hex, tx_buf))}")

# 串口监听线程
def serial_listener():
    global recording
    while running:
        if recording:
            send_a69_data_request()  # 发送数据请求

            try:
                response = ser.read(10)  # 读取 10 字节数据
                if len(response) < 10:
                    print("[警告] 串口返回数据不足 10 字节，丢弃")
                    continue

                parsed_data = parse_a69_data(response)

                if parsed_data:
                    distance_2, distance_3 = parsed_data
                    timestamp = time.time()

                    print(f"[数据] A69 位姿: distance_2={distance_2:.3f}, distance_3={distance_3:.3f}")

                    # 存储数据
                    recorded_data.append((timestamp, distance_2, distance_3))
                else:
                    print("[警告] 收到无效数据，重发请求...")

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
            log_file.write("Timestamp, Distance_2, Distance_3\n")
            for timestamp, dis2, dis3 in recorded_data:
                log_file.write(f"{timestamp:.3f}, {dis2:.3f}, {dis3:.3f}\n")
        print(f"[系统] 数据已保存到 {LOG_FILE}")

        running = False
        return False  # 退出键盘监听

# 启动键盘监听
listener = keyboard.Listener(on_press=on_press, on_release=on_release)
listener.start()

print("按 'R' 开始录制，按 'E' 停止录制，按 'Esc' 退出并保存数据.")

while running:
    time.sleep(0.1)  # 降低 CPU 占用