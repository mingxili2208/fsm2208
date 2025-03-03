#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from autoware_auto_control_msgs.msg import AckermannControlCommand
from std_msgs.msg import String
import math
import serial
import struct
import logging
import threading
import queue
import time
import sys
import numpy as np
from scipy.interpolate import interp1d
from pynput import keyboard

class IntegratedControlPublisher(Node):

    MODE_AUTO = 'AUTO'
    MODE_MANUAL = 'MANUAL'

    def __init__(self):
        super().__init__('integrated_control_publisher')

        # 配置日志
        logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
        self.logger = logging.getLogger(__name__)

        # 读取 ROS2 参数
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 9600)
        self.declare_parameter('timeout', 0.5)

        port = self.get_parameter('port').get_parameter_value().string_value
        baudrate = self.get_parameter('baudrate').get_parameter_value().integer_value
        timeout = self.get_parameter('timeout').get_parameter_value().double_value

        # 初始化串口
        try:
            self.ser = serial.Serial(port, baudrate, timeout=timeout)
            if self.ser.is_open:
                self.logger.info(f"成功打开串口: {port}")
            else:
                raise serial.SerialException(f"无法打开串口: {port}")
        except serial.SerialException as e:
            self.logger.error(f"打开串口时发生错误: {e}")
            raise e

        time.sleep(2)  # 等待串口稳定

        # 状态变量
        self.pre_steering_tire_angle = None
        self.pre_speed = None
        self.data_queue = queue.Queue()
        self.stop_event = threading.Event()
        self.cmd_lock = threading.Lock()
        self.steering_angle_deg = 0.0
        self.speed = 0.0
        self.mode = self.MODE_AUTO

        # 订阅 ROS2 话题
        self.subscription = self.create_subscription(
            AckermannControlCommand,
            '/control/command/control_cmd',
            self.listener_callback,
            1
        )

        # 发布接收到的串口数据
        self.received_data_publisher = self.create_publisher(String, '/serial/received_data', 10)

        # 监听键盘输入
        self.key_state = set()
        self.keyboard_thread = threading.Thread(target=self.keyboard_input_listener, daemon=True)
        self.keyboard_thread.start()

        # 监听串口数据
        self.serial_thread = threading.Thread(target=self.serial_listener, daemon=True)
        self.serial_thread.start()

    def calculate_checksum_r(self, data):
        return data[3] ^ data[4] ^ data[5] ^ data[6]

    def calculate_checksum_interval(self, data, start, end):
        checksum = 0
        for i in range(start, end + 1):
            checksum ^= data[i]
        return checksum

    def angle_interpolation(self, angle):
        x_data = np.array([0, 15, 30, 45, 60, 75, 90])
        y_data = np.array([0, 10, 20, 30, 40, 50, 60])
        interp_func = interp1d(x_data, y_data, kind='linear', fill_value='extrapolate')
        return interp_func(abs(angle))

    def map_speed_sigmoid(self, speed):
        return int(100 / (1 + np.exp(-speed / 2)))

    def parse_ack_response(self, response):
        if len(response) == 10 and response[0] == 0x55 and response[1] == 0x7E and response[2] == 0x01:
            checksum = response[3] ^ response[4] ^ response[5] ^ response[6]
            return checksum == response[7]
        return False

    def send_motion_parameters(self, steering_tire_angle, speed):
        tx_buf = bytearray(10)
        tx_buf[0] = 0x55
        tx_buf[1] = 0x7E
        tx_buf[2] = 0x01  

        if steering_tire_angle < 0:
            tx_buf[3] = int(120 - self.angle_interpolation(-steering_tire_angle))
        else:
            tx_buf[3] = int(120 + self.angle_interpolation(steering_tire_angle))

        tx_buf[4] = 0x00

        if speed < 0:
            tx_buf[5] = int(128 - self.map_speed_sigmoid(-speed))
        else:
            tx_buf[5] = int(128 + self.map_speed_sigmoid(speed))

        tx_buf[6] = 0x00
        tx_buf[7] = self.calculate_checksum_interval(tx_buf, 3, 6)
        tx_buf[8] = 0x7E
        tx_buf[9] = 0x55

        for attempt in range(3):
            self.ser.write(tx_buf)
            self.ser.flush()
            time.sleep(0.01)
            response = self.ser.read(10)
            if self.parse_ack_response(response):
                self.logger.info("A69 设备确认收到指令")
                return
            self.logger.warning(f"A69 未响应，重试 {attempt+1}/3")

    def listener_callback(self, msg):
        if self.mode != self.MODE_AUTO:
            return

        steering_tire_angle_deg = round(math.degrees(msg.lateral.steering_tire_angle), 3)
        speed = round(msg.longitudinal.speed, 3)

        if (self.pre_steering_tire_angle is None or
            self.pre_speed is None or
            abs(steering_tire_angle_deg - self.pre_steering_tire_angle) > 0.5 or
            abs(speed - self.pre_speed) > 0.05):

            self.pre_steering_tire_angle = steering_tire_angle_deg
            self.pre_speed = speed

            self.send_motion_parameters(steering_tire_angle_deg, speed)

    def serial_listener(self):
        while not self.stop_event.is_set():
            try:
                response = self.ser.read(10)
                if self.parse_ack_response(response):
                    self.logger.info("A69 设备确认收到指令")
            except serial.SerialException as e:
                self.logger.error(f"串口读取失败: {e}")

    def keyboard_input_listener(self):
        with keyboard.Listener(on_press=self.on_press, on_release=self.on_release) as listener:
            listener.join()

    def on_press(self, key):
        if key == keyboard.Key.space:
            self.send_motion_parameters(0.0, 0.0)
        elif hasattr(key, 'char') and key.char == 'm':
            self.mode = self.MODE_MANUAL if self.mode == self.MODE_AUTO else self.MODE_AUTO
            self.logger.info(f"切换模式: {self.mode}")

    def on_release(self, key):
        if key == keyboard.Key.esc:
            self.logger.info("按下 Esc，退出程序")
            rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = IntegratedControlPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()