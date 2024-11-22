import serial
from pynput import keyboard
import struct
import logging
import threading
import time
import queue
import signal
import sys
import math

# 配置日志
logging.basicConfig(level=logging.DEBUG, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

# 设置串口参数
SERIAL_PORT = '/dev/ttyUSB0'  # 根据实际情况修改串口号
BAUD_RATE = 115200
TIMEOUT = 0.5  # 超时时间，单位：秒，根据需要调整

# 全局变量
data_queue = queue.Queue()
steering_angle = 0.0
speed = 0.0
running = True  # 用于控制键盘监听和串口线程的停止
key_state = set()  # 用于存储当前按下的按键

# 串口读取线程函数
def read_from_serial(ser):
    while running and ser.is_open:
        try:
            if ser.in_waiting > 0:
                response_data = ser.read(11)
                data_queue.put(response_data)
                if response_data:
                    hex_data = " ".join(f"{byte:02X}" for byte in response_data)
                    logger.debug(f"从串口接收到的数据: {hex_data}")
        except serial.SerialException as e:
            logger.error(f"读取串口时发生错误: {e}")
            break
        else:
            time.sleep(0.1)

# 捕捉键盘按下事件
def on_press(key):
    try:
        key_state.add(key.char.lower())  # 将按下的键放入集合，转为小写
    except AttributeError:
        if key == keyboard.Key.space:
            key_state.add('space')

# 捕捉键盘释放事件
def on_release(key):
    try:
        key_state.discard(key.char.lower())  # 将松开的键从集合中移除，转为小写
    except AttributeError:
        if key == keyboard.Key.space:
            key_state.discard('space')

    if key == keyboard.Key.esc:
        # 按下 `esc` 停止程序
        stop_and_exit()

# 启动键盘输入监听
def keyboard_input_listener():
    with keyboard.Listener(on_press=on_press, on_release=on_release) as listener:
        listener.join()

# 发送数据到串口
def send_data_to_serial(steering_angle, speed):
    try:
        # 限制转向角和速度的范围
        steering_angle = math.degrees(max(min(steering_angle, 0.6), -0.6))
        speed = max(min(speed, 1.8), -1.8)
        logger.debug(f"当前速度: {speed} m/s, 转向角: {steering_angle} rad")

        # 打包和发送消息
        msg = struct.pack('<BBffB', 0x42, 0x01, steering_angle, speed, 0)
        hex_msg = " ".join(f"{byte:02X}" for byte in msg)
        logger.debug(f"发送的消息: {hex_msg}")
        bytes_written = ser.write(msg)
        ser.flush()
        time.sleep(0.03)
        logger.debug(f"写入串口的字节数: {bytes_written}")
    except struct.error as e:
        logger.error(f"打包消息时发生错误: {e}")
    except serial.SerialException as e:
        logger.error(f"写入串口时发生错误: {e}")

# 捕捉 SIGINT 信号 (Ctrl + C)
def signal_handler(sig, frame):
    logger.info("捕捉到中断信号，正在退出...")
    stop_and_exit()

# 停止串口并退出程序
def stop_and_exit():
    global ser, running
    running = False
    if ser.is_open:
        send_data_to_serial(0, 0)  # 发送停止命令
        ser.close()
        logger.info(f"已关闭串口: {SERIAL_PORT}")
    sys.exit(0)

# 主程序
if __name__ == "__main__":
    # 注册信号处理函数
    signal.signal(signal.SIGINT, signal_handler)

    # 打开串口
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=TIMEOUT)
        if ser.is_open:
            logger.info(f"成功打开串口: {SERIAL_PORT}")
    except serial.SerialException as e:
        logger.error(f"无法打开串口: {SERIAL_PORT}, 错误: {e}")
        sys.exit(1)

    time.sleep(2)

    # 启动串口读取线程
    read_thread = threading.Thread(target=read_from_serial, args=(ser,), daemon=True)
    read_thread.start()

    # 启动键盘输入监听
    listener_thread = threading.Thread(target=keyboard_input_listener, daemon=True)
    listener_thread.start()

    # 主线程发送数据到串口
    while running:
        # 根据当前按下的键更新速度和转向角
        with threading.Lock():
            if 'w' in key_state:
                speed = min(speed + 0.1, 1.8)  # 最大速度限制为1.8
            elif 's' in key_state:
                speed = max(speed - 0.1, -1.8)  # 最低速度限制为-1.8
            else:
                speed = 0  # 无按键时速度归零

            if 'a' in key_state:
                steering_angle += 0.08
                # 限制转向角度
                steering_angle = min(steering_angle, 0.6)
            elif 'd' in key_state:
                steering_angle -= 0.08
                # 限制转向角度
                steering_angle = max(steering_angle, -0.6)
            else:
                # 松开转向键时转向角逐渐回到0
                if steering_angle > 0:
                    steering_angle = max(steering_angle - 0.1, 0)
                elif steering_angle < 0:
                    steering_angle = min(steering_angle + 0.1, 0)

        # 持续发送最新的速度和转向角度到串口
        send_data_to_serial(steering_angle, speed)
        time.sleep(0.1)