import serial 
import pandas as pd
import time
import struct
import logging


logging.basicConfig(level=logging.DEBUG, format='%(asctime)s - %(levelname)s - %(message)s')

# 创建一个日志记录器
logger = logging.getLogger(__name__)

# 输出不同级别的日志信息



# 设置串口参数
serial_port = '/dev/ttyUSB0'  # 请根据实际情况修改串口号
baud_rate = 115200

# 打开串口
ser = serial.Serial(serial_port, baud_rate)

# 读取 CSV 文件
df = pd.read_csv("~/lmx/Data/topic_record/control_cmd.bag/control_cmd_no_timestap_3 .csv")
logger.debug("read data from csv finished")
def calculate_checksum(steering_tire_angle, speed):
    data = struct.pack('<BBff', 0x42, 0x01, steering_tire_angle, speed)
    checksum = 0
    for byte in data:
        checksum ^= byte
    return checksum
def verify_checksum(data):
    checksum = calculate_checksum(struct.unpack('<ff', data[2:8])[0], struct.unpack('<ff', data[2:8])[1])
    return checksum == data[8]

# 遍历 DataFrame 的每一行
for index, row in df.iterrows():
    # 获取 steering_angle 和 speed 值
    if ser.is_open:
        logger.info("serial is open")

        steering_tire_angle = row['steering_angle']
        speed = row['speed']

        msg = struct.pack('<BBffB', 0x42, 0x01, steering_tire_angle, speed, calculate_checksum(steering_tire_angle, speed))
        logger.debug("msg: {}".format(msg))
        # send msg
        flag=ser.write(msg)
        if flag is not None:
            logger.debug("send msg to arduino: {}".format(flag))
        else:
            logger.error("send msg to arduino failed")
            # get msg from arduino for callback
        if ser.in_waiting >= 11:
            data = ser.read(11)
            if data[0] == 0x42 and verify_checksum(data):
                # 解析消息
                _, msg_type, steering_tire_angle, speed, _ = struct.unpack('<BBffB', data)
                if msg_type == 0x01:
                    logger.info("arduino received msg succeed!!")
                if msg_type == 0x02:
                    logger.info(f'Arduino send correct as: steering_tire_angle={steering_tire_angle}, speed={speed}')
                elif msg_type == 0x03:
                    logger.info(f'!!!!!!Arduino send error!!!!!!!\nwhile: steering_tire_angle={steering_tire_angle}, speed={speed}')
        # 等待一小段时间，以便 Arduino 处理数据
    else:
        logger.info("serial is not open")
    #time.sleep(0.1)  # 可以根据实际情况调整延时时间

# 关闭串口
ser.close()


