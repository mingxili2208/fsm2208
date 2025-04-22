import serial
import struct
import time
import argparse

# 通信协议定义
FRAME_HEADER = 0xAA  # 帧头标识
FRAME_TAIL = 0x55    # 帧尾标识

# 数据包类型定义
class PacketType:
    PACKET_MOTION = 0x01   # 运动数据包（包含线速度、角度等）
    PACKET_COMMAND = 0x02  # 命令数据包
    PACKET_STATUS = 0x03   # 状态数据包
    PACKET_ACK = 0x04      # 确认数据包

class RF24Sender:
    def __init__(self, port, baudrate=115200):
        """初始化RF24发送器"""
        self.serial = serial.Serial(port, baudrate, timeout=1)
        self.sequence_counter = 0
        print(f"Connected to {port} at {baudrate} baud")
        time.sleep(2)  # 等待串口稳定
    
    def calculate_checksum(self, data):
        """计算校验和"""
        return sum(data) & 0xFF
    
    def pack_motion_data(self, device_id, linear_vel, angular_vel, linear_acc, angular_acc):
        """打包运动数据包"""
        # 初始化32字节数据包
        packet = bytearray(32)
        
        # 填充帧头和包类型
        packet[0] = FRAME_HEADER  # 帧头
        packet[1] = PacketType.PACKET_MOTION  # 包类型
        packet[2] = device_id  # 设备ID
        
        # 填充速度和加速度数据 (使用小端字节序，与Arduino兼容)
        struct.pack_into('<h', packet, 3, linear_vel)    # 线速度 (2字节)
        struct.pack_into('<h', packet, 5, angular_vel)   # 角速度 (2字节)
        struct.pack_into('<h', packet, 7, linear_acc)    # 线加速度 (2字节)
        struct.pack_into('<h', packet, 9, angular_acc)   # 角加速度 (2字节)
        
        # 填充序列号
        packet[11] = self.sequence_counter
        self.sequence_counter = (self.sequence_counter + 1) & 0xFF
        
        # 计算校验和 (不包括校验和字段本身和帧尾)
        checksum = self.calculate_checksum(packet[:12])
        packet[12] = checksum
        
        # 填充帧尾
        packet[13] = FRAME_TAIL
        
        return packet
    
    def send_motion_data(self, device_id, linear_vel, angular_vel, linear_acc, angular_acc):
        """发送运动数据包"""
        packet = self.pack_motion_data(device_id, linear_vel, angular_vel, linear_acc, angular_acc)
        bytes_written = self.serial.write(packet)
        self.serial.flush()
        
        # 记录发送的数据
        print(f"发送运动数据: 设备ID={device_id}, 序列号={packet[11]}")
        print(f"  线速度: {linear_vel}, 角速度: {angular_vel}")
        print(f"  线加速度: {linear_acc}, 角加速度: {angular_acc}")
        print(f"  发送字节数: {bytes_written}")
        print(f"  原始数据: {packet.hex(' ')}")
        print()
        
        return bytes_written
    
    def close(self):
        """关闭串口连接"""
        if self.serial and self.serial.is_open:
            self.serial.close()
            print("串口已关闭")

def main():
    """主函数"""
    parser = argparse.ArgumentParser(description='RF24L01 数据发送器')
    parser.add_argument('--port', type=str, required=True, help='串口端口，如 COM3')
    parser.add_argument('--baud', type=int, default=115200, help='波特率，默认115200')
    parser.add_argument('--interval', type=float, default=1.0, help='发送间隔（秒），默认1秒')
    parser.add_argument('--count', type=int, default=0, help='发送包数量，0表示无限发送')
    
    args = parser.parse_args()
    
    sender = RF24Sender(args.port, args.baud)
    
    try:
        count = 0
        while args.count == 0 or count < args.count:
            # 这里可以根据需要修改发送的数据
            # 参数: 设备ID, 线速度, 角速度, 线加速度, 角加速度
            sender.send_motion_data(1, 100, 45, 10, 5)
            
            count += 1
            if args.count > 0:
                print(f"已发送 {count}/{args.count} 个数据包")
            else:
                print(f"已发送 {count} 个数据包")
                
            time.sleep(args.interval)
    
    except KeyboardInterrupt:
        print("\n用户中断，停止发送")
    
    finally:
        sender.close()

if __name__ == "__main__":
    main()