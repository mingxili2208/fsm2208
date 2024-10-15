import rclpy
from rclpy.node import Node
import serial
import struct

class SerialNode(Node):

    def __init__(self):
        super().__init__('serial_node')
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

    def timer_callback(self):
        # 打包消息
        msg = struct.pack('<BBffB', 0x42, 0x01, self.steering_tire_angle, self.speed, self.calculate_checksum(self.steering_tire_angle, self.speed))

        # 发送消息
        self.ser.write(msg)

        # 接收消息
        if self.ser.in_waiting >= 9:
            data = self.ser.read(9)
            if data[0] == 0x42 and self.verify_checksum(data):
                # 解析消息
                _, msg_type, steering_tire_angle, speed, _ = struct.unpack('<BBffB', data)

                if msg_type == 0x02:
                    self.get_logger().info(f'Received from Arduino: steering_tire_angle={steering_tire_angle}, speed={speed}')

    def calculate_checksum(self, steering_tire_angle, speed):
        data = struct.pack('<BBff', 0x42, 0x01, steering_tire_angle, speed)
        checksum = 0
        for byte in data:
            checksum ^= byte
        return checksum

    def verify_checksum(self, data):
        checksum = self.calculate_checksum(struct.unpack('<ff', data[2:8])[0], struct.unpack('<ff', data[2:8])[1])
        return checksum == data[8]

def main(args=None):
    rclpy.init(args=args)
    serial_node = SerialNode()
    rclpy.spin(serial_node)
    serial_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()