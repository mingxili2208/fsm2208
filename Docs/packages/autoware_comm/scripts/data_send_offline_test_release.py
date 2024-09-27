import serial
import pandas as pd
import time
import struct
import logging
import threading
import queue

# 配置日志
logging.basicConfig(level=logging.DEBUG, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

# 设置串口参数
SERIAL_PORT = '/dev/ttyUSB0'  # 请根据实际情况修改串口号
BAUD_RATE = 115200
TIMEOUT = 0.5  # 超时时间，单位：秒，根据需要调整

# 读取CSV文件
try:
    #csv_path = "~/lmx/Data/topic_record/control_cmd.bag/control_cmd_no_timestap_3.csv"
    csv_path="~/lmx/Data/topic_record/control_cmd_bag_2/control_cmd_no_timestap_1.csv"
    df = pd.read_csv(csv_path)
    logger.debug("读取CSV文件完成")
except Exception as e:
    logger.error(f"读取CSV文件失败: {e}")
    exit(1)

# 初始化前一个角度和速度
pre_steering_tire_angle = 0.0
pre_speed = 0.0
data_queue=queue.Queue()
# 定义读取线程
def read_from_serial(ser):
    while ser.is_open:
        try:
            #time.sleep(0.2)
            if ser.in_waiting > 0:
                response_data = ser.read(11)
                data_queue.put(response_data)
                #time.sleep(0.2)
                if response_data:
                    hex_data = " ".join(f"{byte:02X}" for byte in response_data)
                    logger.debug(f"从串口接收到的数据: {hex_data}")
                    # 可在此处添加解析响应数据的逻辑
            else:
                time.sleep(0.05) 
        except serial.SerialException as e:
            logger.error(f"读取串口时发生错误: {e}")
            break

# 打开串口并启动读取线程
try:
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=TIMEOUT)
    if ser.is_open:
        logger.info(f"成功打开串口: {SERIAL_PORT}")
    else:
        logger.error(f"无法打开串口: {SERIAL_PORT}")
        exit(1)
except serial.SerialException as e:
    logger.error(f"打开串口时发生错误: {e}")
    exit(1)

# 启动读取线程
read_thread = threading.Thread(target=read_from_serial, args=(ser,), daemon=True)
read_thread.start()

time.sleep(2)

# 遍历DataFrame的每一行
for index, row in df.iterrows():
    # 获取当前的转向角度和速度
    cur_steering_tire_angle = row[0]
    cur_speed = row[1]

    # 判断是否有显著变化
    if abs(cur_steering_tire_angle - pre_steering_tire_angle) > 0.5 or abs(cur_speed - pre_speed) > 0.1:
        if abs(cur_steering_tire_angle) > 28 :
            cur_steering_tire_angle = 28*(cur_steering_tire_angle/cur_steering_tire_angle)
        pre_steering_tire_angle = cur_steering_tire_angle
        pre_speed = cur_speed
        logger.debug(f"当前转向角度: {cur_steering_tire_angle}, 速度: {cur_speed}")

        # 打包消息
        try:
            msg = struct.pack('<BBffB', 0x42, 0x01, cur_steering_tire_angle, cur_speed, 0)
            hex_msg = " ".join(f"{byte:02X}" for byte in msg)
            logger.debug(f"发送的消息: {hex_msg}")
        except struct.error as e:
            logger.error(f"打包消息时发生错误: {e}")
            continue

        # 发送消息
        try:
            bytes_written = ser.write(msg)
            ser.flush()  # 确保所有数据都已发送
            time.sleep(0.02)
            logger.debug(f"写入串口的字节数: {bytes_written}")
        except serial.SerialException as e:
            logger.error(f"写入串口时发生错误: {e}")
            continue

        # 等待设备响应（可根据需要调整）
        time.sleep(0.03)  # 200毫秒

        # 继续下一次发送，不再在主线程中读取

        # 等待下一次发送
        #time.sleep(1.0)  # 1秒
    else:
        logger.debug("转向角度和速度无显著变化，无需发送")

# 等待所有数据被处理后关闭串口
msg = struct.pack('<BBffB', 0x42, 0x01, 0, 0, 0)
ser.write(msg)
ser.flush() 
read_thread.stop()
time.sleep(0.1)  # 200毫秒
ser.close()
logger.info(f"已关闭串口: {SERIAL_PORT}")