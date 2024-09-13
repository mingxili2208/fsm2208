import serial
import time
import struct

# 配置串口的参数
serial_port = '/dev/ttyUSB0'  # 根据需要更改串口号,例如 Windows 上可以是 'COM3'
baud_rate = 9600  # 波特率
timeout = 1  # 超时时间，单位：秒

# 打开串口
ser = serial.Serial(serial_port, baud_rate, timeout=timeout)

# 检查串口是否已打开
if ser.is_open:
    print(f"串口 {serial_port} 已打开，波特率：{baud_rate}")

try:
    # 构造要发送的字节数据 (例如，发送 [0x42, 0x01, 0xFF, 0x10])
    ser.reset_input_buffer()
    msg = struct.pack('<BBffB', 0x42, 0x01, 0, 0.5, 0)
    #byte_data = bytearray([0x42, 0x01, 0x00, 0x00, 0x00, 0x00, 0x3f, 0x80, 0x00, 0x00,0x00])
    hex_msg = " ".join(f"{byte:02X}" for byte in msg)
    # 发送数据
    print(f"发送字节数据: {hex_msg}")
    ser.write(msg)

    # 等待一段时间以确保数据发送完成
    time.sleep(1)

    # 可选：接收从串口返回的数据
    while(1):
        if ser.in_waiting > 0:
            response_data = ser.read(ser.in_waiting)
            hex_data = " ".join(f"{byte:02X}" for byte in response_data)
            print(f"从串口接收到的数据: {hex_data}")
            print(f"发送字节数据: {hex_msg}")
            ser.write(msg)
            #break

except Exception as e:
    print(f"串口通信出现错误: {e}")

finally:
    # 关闭串口
    ser.close()
    print(f"串口 {serial_port} 已关闭")