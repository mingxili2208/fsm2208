import serial

# 配置串口
PORT = "/dev/ttyUSB0"  # 根据实际情况修改串口名
BAUD_RATE = 9600

def parse_data(data):
    """
    解析手柄数据包
    :param data: 从串口读取的一行数据
    :return: 解析后的字典
    """
    try:
        # 拆分键值对
        key_value_pairs = data.strip().split(",")
        parsed_data = {}
        for pair in key_value_pairs:
            key, value = pair.split(":")
            parsed_data[key] = int(value)  # 转换值为整数
        return parsed_data
    except Exception as e:
        print(f"Error parsing data: {e}")
        return None

def main():
    # 打开串口
    try:
        ser = serial.Serial(PORT, BAUD_RATE, timeout=1)
        print(f"Connected to {PORT} at {BAUD_RATE} baud.")
    except Exception as e:
        print(f"Failed to connect to {PORT}: {e}")
        return

    # 读取并处理数据
    try:
        while True:
            if ser.in_waiting > 0:  # 检查是否有数据可读
                line = ser.readline().decode('utf-8').strip()
                print(f"Raw data: {line}")  # 打印接收到的原始数据

                parsed = parse_data(line)  # 解析数据
                if parsed:
                    print("Parsed data:", parsed)  # 打印解析后的字典
    except KeyboardInterrupt:
        print("Exiting...")
    except Exception as e:
        print(f"Error: {e}")
    finally:
        ser.close()  # 关闭串口

if __name__ == "__main__":
    main()