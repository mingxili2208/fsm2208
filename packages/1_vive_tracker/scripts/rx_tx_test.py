import time
import struct
import serial
import threading

# 串口配置
SERIAL_PORT = "/dev/ttyUSB0"  # 请根据实际情况修改
BAUD_RATE = 115200

# 计算校验和
def calculate_checksum(flag, steering_angle, speed):
    checksum = (0x42 + flag + int(steering_angle * 1000) + int(speed * 1000)) & 0xFF
    return checksum

# 发送串口指令
def send_serial_command(ser, flag, steering_angle_deg=0.0, speed=0.0):
    try:
        checksum = calculate_checksum(flag, steering_angle_deg, speed)
        packed_msg = struct.pack('<BBffB', 0x42, flag, steering_angle_deg, speed, checksum)
        ser.write(packed_msg)
        ser.flush()
        print(f"[发送] 指令: flag={flag}, 速度={speed}, 转向角={steering_angle_deg}")
    except serial.SerialException as e:
        print(f"[错误] 串口写入错误: {e}")

# 监听串口返回数据
def serial_listener(ser):
    while True:
        response = ser.read(11)  # 读取 11 字节完整数据包
        if response:
            print(f"[接收] 原始数据: {[hex(b) for b in response]}")

# 初始化串口
try:
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1)
    print(f"[成功] 打开串口: {SERIAL_PORT}")

    # 启动串口监听线程
    serial_thread = threading.Thread(target=serial_listener, args=(ser,), daemon=True)
    serial_thread.start()

    # 每秒 1 次（10Hz）发送 0x02 指令
    while True:
        send_serial_command(ser, 0x02, 0.0, 0.0)
        time.sleep(1)  # 100ms 间隔，即 10Hz

except serial.SerialException as e:
    print(f"[错误] 无法打开串口 {SERIAL_PORT}: {e}")