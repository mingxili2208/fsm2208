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
baud_rate = 9600
timeout = 1  # 超时时间，单位：秒

# 打开串口
ser = serial.Serial(serial_port, baud_rate, timeout=timeout)

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

pre_steering_tire_angle=0.0
pre_speed=0.0

# 遍历 DataFrame 的每一行
for index, row in df.iterrows():
    # 获取 steering_angle 和 speed 值
    if ser.is_open:
        logger.info("serial is open")
        ser.reset_input_buffer()
        cur_steering_tire_angle = row['steering_angle']
        cur_speed = row['speed']
        if abs(cur_steering_tire_angle-pre_steering_tire_angle)>5 or abs(cur_speed-pre_speed)>0.1:
            pre_steering_tire_angle=cur_steering_tire_angle
            pre_speed=cur_speed
            logger.debug("cur_steering_tire_angle: {}, speed: {}".format(cur_steering_tire_angle, cur_speed))
            steering_tire_angle=float(cur_steering_tire_angle)
            speed=float(cur_speed)
            msg = struct.pack('<BBffB', 0x42, 0x01, steering_tire_angle, speed, 0)
            hex_msg = " ".join(f"{byte:02X}" for byte in msg)
            logger.debug("msg: {}".format(hex_msg))
            # send msg
            #ser.reset_input_buffer()
        
            flag=ser.write(msg)
            #time.sleep(1)
            if flag is not None:
                logger.debug("send msg to arduino: {}".format(flag))
            else:
                logger.error("flag------send msg to arduino failed")
                # get msg from arduino for callback
            if ser.in_waiting > 0:
                response_data = ser.read(ser.in_waiting)
                hex_data = " ".join(f"{byte:02X}" for byte in response_data)
                logger.debug(f"从串口接收到的数据: {hex_data}")
        # if ser.in_waiting >= 11:
        #     logger.debug("receive msg from arduino: {}".format(data))
        #     data = ser.read(11)
        #     if data[0] == 0x42:# and verify_checksum(data):
        #         # 解析消息
        #         _, msg_type, steering_tire_angle, speed, _ = struct.unpack('<BBffB', data)
        #         if msg_type == 0x00:
        #             logger.info("arduino received msg type succeed!!!")
        #         elif msg_type == 0x01:
        #             logger.info("arduino received msg trans succeed!!")
        #         elif msg_type == 0x02:
        #             logger.info(f'Arduino send correct as: steering_tire_angle={steering_tire_angle}, speed={speed}')
        #         elif msg_type == 0x03:
        #             logger.info(f'!!!!!!Arduino send error!!!!!!!\nwhile: steering_tire_angle={steering_tire_angle}, speed={speed}')
        #         else:
        #             logger.error("!!!!!!arduino received unknown msg error!!!!!!!")
        # else: 
        #     logger.error("ser.in_waiting----receive msg from arduino failed")
        # 等待一小段时间，以便 Arduino 处理数据
    else:
        logger.info("serial is not open")
    time.sleep(0.1)  # 可以根据实际情况调整延时时间

# 关闭串口
ser.close()


