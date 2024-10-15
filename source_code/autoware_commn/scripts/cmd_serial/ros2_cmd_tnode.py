#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from autoware_auto_control_msgs.msg import AckermannControlCommand
import math
import serial
import struct

class AckermannControlSubscriber(Node):

    def __init__(self):
        super().__init__('ackermann_control_subscriber')

        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('timeout', 1)

        port = self.get_parameter('port').get_parameter_value().string_value
        baudrate = self.get_parameter('baudrate').get_parameter_value().integer_value
        timeout = self.get_parameter('timeout').get_parameter_value().integer_value

        self.ser = serial.Serial(port, baudrate, timeout=timeout)
        self.timer = self.create_timer(0.1, self.timer_callback)

        self.steering_tire_angle = 0.0
        self.speed = 0.0


        self.subscription = self.create_subscription(
            AckermannControlCommand,
            '/control/command/control_cmd',  
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
    def calculate_checksum(self, steering_tire_angle, speed):
        data = struct.pack('<BBff', 0x42, 0x01, steering_tire_angle, speed)
        checksum = 0
        for byte in data:
            checksum ^= byte
        return checksum
    def verify_checksum(self, data):
        checksum = self.calculate_checksum(struct.unpack('<ff', data[2:8])[0], struct.unpack('<ff', data[2:8])[1])
        return checksum == data[8]
    def listener_callback(self, msg):
        self.get_logger().info('Received AckermannControlCommand message:')
        self.get_logger().info(f'  Steering Tire Angle: {msg.lateral.steering_tire_angle}')
        self.get_logger().info(f'  Speed: {msg.longitudinal.speed}')
        self.get_logger().info(f'  Acceleration: {msg.longitudinal.acceleration}')
        self.steering_tire_angle=math.degrees(msg.lateral.steering_tire_angle)
        self.speed=msg.longitudinal.speed
        
        # package msg
        msg = struct.pack('<BBffB', 0x42, 0x01, self.steering_tire_angle, self.speed, self.calculate_checksum(self.steering_tire_angle, self.speed))

        # send msg
        self.ser.write(msg)

         # get msg from arduino for callback
        if self.ser.in_waiting >= 9:
            data = self.ser.read(9)
            if data[0] == 0x42 and self.verify_checksum(data):
                # 解析消息
                _, msg_type, steering_tire_angle, speed, _ = struct.unpack('<BBffB', data)

                if msg_type == 0x02:
                    self.get_logger().info(f'Arduino send correct as: steering_tire_angle={steering_tire_angle}, speed={speed}')
                elif msg_type == 0x03:
                    self.get_logger().info(f'!!!!!!Arduino send error!!!!!!!\nwhile: steering_tire_angle={steering_tire_angle}, speed={speed}')

def main(args=None):
    rclpy.init(args=args)

    ackermann_control_subscriber = AckermannControlSubscriber()

    rclpy.spin(ackermann_control_subscriber)

    # Destroy the node explicitly
    ackermann_control_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()