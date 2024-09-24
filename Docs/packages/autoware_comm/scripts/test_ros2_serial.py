#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from autoware_auto_control_msgs.msg import AckermannControlCommand
import math
import serial
import struct
import logging
import threading
import queue
import time

class AckermannControlPublisher(Node):

    def __init__(self):
        super().__init__('ackermann_control_publisher')

        # 配置日志
        logging.basicConfig(level=logging.DEBUG, format='%(asctime)s - %(levelname)s - %(message)s')
        self.logger = logging.getLogger(__name__)

        # 获取参数
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('timeout', 0.5)  # 超时时间，单位：秒

        port = self.get_parameter('port').get_parameter_value().string_value
        baudrate = self.get_parameter('baudrate').get_parameter_value().integer_value
        timeout = self.get_parameter('timeout').get_parameter_value().double_value

        # 初始化串口
        try:
            self.ser = serial.Serial(port, baudrate, timeout=timeout)
            if self.ser.is_open:
                self.logger.info(f"成功打开串口: {port}")
            else:
                self.logger.error(f"无法打开串口: {port}")
                raise serial.SerialException(f"无法打开串口: {port}")
        except serial.SerialException as e:
            self.logger.error(f"打开串口时发生错误: {e}")
            rclpy.shutdown()
            return

        # 初始化前一个角度和速度
        self.pre_steering_tire_angle = 0.0
        self.pre_speed = 0.0

        # 创建一个队列，用于传递数据
        self.data_queue = queue.Queue()

        # 定义用于控制线程关闭的事件
        self.stop_event = threading.Event()

        # 启动读取线程
        self.read_thread = threading.Thread(target=self.read_from_serial, args=(self.ser, self.stop_event, self.data_queue), daemon=True)
        self.read_thread.start()

        # 创建订阅者
        self.subscription = self.create_subscription(
            AckermannControlCommand,
            '/control/command/control_cmd',
            self.listener_callback,
            10)
        self.subscription  # 防止未使用变量警告

    def calculate_checksum(self, steering_tire_angle, speed):
        data = struct.pack('<BBff', 0x42, 0x01, steering_tire_angle, speed)
        checksum = 0
        for byte in data:
            checksum ^= byte
        return checksum

    def verify_checksum(self, data):
        if len(data) < 9:
            return False
        steering_tire_angle, speed = struct.unpack('<ff', data[2:10])
        calculated_checksum = self.calculate_checksum(math.degrees(steering_tire_angle), speed)
        return calculated_checksum == data[10]

    def listener_callback(self, msg):
        self.logger.info('接收到 AckermannControlCommand 消息:')
        self.logger.info(f'  转向角度: {msg.lateral.steering_tire_angle} 弧度')
        self.logger.info(f'  速度: {msg.longitudinal.speed} m/s')
        self.logger.info(f'  加速度: {msg.longitudinal.acceleration} m/s²')

        # 将转向角度从弧度转换为度
        steering_tire_angle_deg = math.degrees(msg.lateral.steering_tire_angle)
        speed = msg.longitudinal.speed

        # 判断是否有显著变化
        if abs(steering_tire_angle_deg - self.pre_steering_tire_angle) > 5 or abs(speed - self.pre_speed) > 0.1:
            self.pre_steering_tire_angle = steering_tire_angle_deg
            self.pre_speed = speed
            self.logger.debug(f"当前转向角度: {steering_tire_angle_deg}, 速度: {speed}")

            # 打包消息
            try:
                checksum = self.calculate_checksum(steering_tire_angle_deg, speed)
                packed_msg = struct.pack('<BBffB', 0x42, 0x01, steering_tire_angle_deg, speed, checksum)
                hex_msg = " ".join(f"{byte:02X}" for byte in packed_msg)
                self.logger.debug(f"发送的消息: {hex_msg}")
            except struct.error as e:
                self.logger.error(f"打包消息时发生错误: {e}")
                return

            # 发送消息
            try:
                bytes_written = self.ser.write(packed_msg)
                self.ser.flush()  # 确保所有数据都已发送
                self.logger.debug(f"写入串口的字节数: {bytes_written}")
            except serial.SerialException as e:
                self.logger.error(f"写入串口时发生错误: {e}")
                return

            # 等待设备响应
            time.sleep(0.2)  # 200毫秒

            # 从队列中获取响应数据
            while not self.data_queue.empty():
                response = self.data_queue.get()
                if len(response) >= 11 and response[0] == 0x42:
                    if self.verify_checksum(response):
                        _, msg_type, steering_tire_angle_resp, speed_resp, checksum_resp = struct.unpack('<BBffB', response[:11])
                        if msg_type == 0x02:
                            self.logger.info(f'Arduino 发送正确: 转向角度={steering_tire_angle_resp} 度, 速度={speed_resp} m/s')
                        elif msg_type == 0x03:
                            self.logger.info(f'!!!!!!Arduino 发送错误!!!!!!!\n转向角度={steering_tire_angle_resp} 度, 速度={speed_resp} m/s')
                        else:
                            self.logger.error("!!!!!!Arduino 发送未知消息类型!!!!!!!")
                    else:
                        self.logger.error("!!!!!!Arduino 发送的校验和不正确!!!!!!!")
                else:
                    self.logger.error("!!!!!!Arduino 发送的消息格式不正确!!!!!!!")

    def read_from_serial(self, ser, stop_event, data_queue):
        while not stop_event.is_set() and ser.is_open:
            try:
                if ser.in_waiting > 0:
                    response_data = ser.read(11)  # 假设响应消息长度为11字节
                    if response_data:
                        data_queue.put(response_data)
                        hex_data = " ".join(f"{byte:02X}" for byte in response_data)
                        self.logger.debug(f"从串口接收到的数据: {hex_data}")
                else:
                    time.sleep(0.01)  # 减少CPU占用
            except serial.SerialException as e:
                self.logger.error(f"读取串口时发生错误: {e}")
                break

    def destroy_node(self):
        self.stop_event.set()  # 通知读取线程停止
        self.read_thread.join()  # 等待读取线程结束
        if self.ser.is_open:
            self.ser.close()
            self.logger.info(f"已关闭串口: {self.ser.port}")
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    ackermann_control_publisher = AckermannControlPublisher()

    try:
        rclpy.spin(ackermann_control_publisher)
    except KeyboardInterrupt:
        ackermann_control_publisher.get_logger().info('接收到中断信号，准备退出...')
    finally:
        ackermann_control_publisher.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()