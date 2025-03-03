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

        # 声明并获取参数
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 9600)
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
            # 抛出异常，让主函数处理
            raise e

        time.sleep(2)  # 等待串口稳定

        # 初始化前一个角度和速度
        self.pre_steering_tire_angle = None
        self.pre_speed = None

        # 创建一个队列用于接收数据
        self.data_queue = queue.Queue()

        # 定义事件以控制线程关闭
        self.stop_event = threading.Event()

        # 初始化命令变量及锁
        self.cmd_lock = threading.Lock()
        self.steering_angle_deg = 0.0
        self.speed = 0.0

        # 当前模式，默认自动模式
        self.mode = self.MODE_AUTO
        self.logger.info("当前模式: AUTO")

        # 启动串口读取线程
        self.read_thread = threading.Thread(target=self.read_from_serial, daemon=True)
        self.read_thread.start()

        # 启动数据处理线程
        self.process_thread = threading.Thread(target=self.process_data_queue, daemon=True)
        self.process_thread.start()

        # 创建ROS订阅者
        self.subscription = self.create_subscription(
            AckermannControlCommand,
            '/control/command/control_cmd',
            self.listener_callback,
            1)
        self.subscription  # 防止未使用变量警告

        # 添加新的发布者，用于发布接收到的串口数据
        self.received_data_publisher = self.create_publisher(String, '/serial/received_data', 10)
        # 添加调试信息发布者
        self.debug_publisher = self.create_publisher(String, '/serial/debug_info', 10)

        # 初始化键盘监听相关变量
        self.key_state = set()
        self.keyboard_thread = threading.Thread(target=self.keyboard_input_listener, daemon=True)
        self.keyboard_thread.start()

        # 记录上次发送的命令，避免重复发送
        self.last_sent_commands = {
            'steering_angle_deg': None,
            'speed': None
        }

    def angle_interpolation(self, angle):
        """
        将角度值通过线性插值映射到控制值，使用与Arduino代码相同的插值点
        """
        # 定义与Arduino代码相同的插值点
        x_data = np.array([0, 5, 10, 15, 20, 25, 30])
        y_data = np.array([0, 15, 30, 50, 60, 100, 120])
        interp_func = interp1d(x_data, y_data, kind='linear', fill_value='extrapolate')
        return interp_func(abs(angle))

    def map_speed_sigmoid(self, speed, min_speed=0.05, max_speed=1.8, min_mapped=50, max_mapped=100):
        """
        使用与Arduino代码相同的sigmoid函数将速度映射到控制值
        """
        if speed <= 0.05:
            return 0
        
        speed = min(speed, max_speed)
        normalized_speed = (speed - min_speed) / (max_speed - min_speed)
        sigmoid_speed = 1.0 / (1.0 + np.exp(-10.0 * (normalized_speed - 0.5)))
        mapped_speed = sigmoid_speed * (max_mapped - min_mapped) + min_mapped
        return int(mapped_speed)

    def calculate_checksum_interval(self, data, start, end):
        """
        计算指定范围内的校验和
        """
        checksum = 0
        for i in range(start, end + 1):
            checksum ^= data[i]
        return checksum

    def parse_ack_response(self, response):
        """
        解析A69设备的响应数据
        """
        if len(response) == 10 and response[0] == 0x55 and response[1] == 0x7E and response[8] == 0x7E and response[9] == 0x55:
            checksum = response[3] ^ response[4] ^ response[5] ^ response[6]
            
            # 发布调试信息
            debug_msg = String()
            debug_msg.data = f"响应解析: ST={response[2]:02X}, DATA=[{response[3]:02X},{response[4]:02X},{response[5]:02X},{response[6]:02X}], CHK={response[7]:02X}, CALC_CHK={checksum:02X}"
            self.debug_publisher.publish(debug_msg)
            
            if response[2] == 0x01 and checksum == response[7]:
                return True
            elif response[7] != checksum:
                self.logger.warning(f"校验和错误: 接收={response[7]:02X}, 计算={checksum:02X}")
                return False
            else:
                self.logger.info(f"收到非确认消息: ST={response[2]:02X}")
                
                # 如果是速度/角度回传消息，解析并记录
                if response[2] == 0x02:  # 假设0x02是速度/角度回传
                    try:
                        # 根据通信协议解析数据
                        received_speed = response[3]
                        received_angle = response[5]
                        self.logger.info(f"接收到速度反馈: {received_speed}, 角度反馈: {received_angle}")
                        
                        # 发布解析后的数据
                        feedback_msg = String()
                        feedback_msg.data = f"反馈: 速度={received_speed}, 角度={received_angle}"
                        self.received_data_publisher.publish(feedback_msg)
                    except Exception as e:
                        self.logger.error(f"解析反馈数据时出错: {e}")
                
                return False
        
        self.logger.warning(f"无效的响应数据: {' '.join(f'{b:02X}' for b in response)}")
        return False

    def send_motion_parameters(self, steering_tire_angle, speed):
        """
        发送运动参数到A69设备，使用与Arduino代码相同的映射方法
        """
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

        # 记录准备发送的数据
        hex_msg = " ".join(f"{byte:02X}" for byte in tx_buf)
        self.logger.info(f"发送数据: {hex_msg}")
        
        # 发布发送的数据内容调试信息
        debug_msg = String()
        debug_msg.data = f"发送: ST={tx_buf[2]:02X}, 角度={tx_buf[3]:02X}, 速度={tx_buf[5]:02X}, CHK={tx_buf[7]:02X}"
        self.debug_publisher.publish(debug_msg)

        for attempt in range(3):
            try:
                bytes_written = self.ser.write(tx_buf)
                self.logger.info(f"send_result {bytes_written}")
                self.ser.flush()
                
                # 等待响应数据
                time.sleep(0.01)
                response = self.ser.read(10)
                
                if len(response) == 10:
                    self.logger.info(f"收到响应: {' '.join(f'{b:02X}' for b in response)}")
                    
                    if self.parse_ack_response(response):
                        self.logger.info("A69 设备确认收到指令")
                        
                        # 发布到ROS2主题
                        sent_msg = String()
                        sent_msg.data = f"Sent - 速度: {speed} m/s, 转向角度: {steering_tire_angle} 度 (已确认)"
                        self.received_data_publisher.publish(sent_msg)
                        
                        # 更新上次发送的命令
                        self.last_sent_commands['speed'] = speed
                        self.last_sent_commands['steering_angle_deg'] = steering_tire_angle
                        return
                    else:
                        self.logger.warning("A69 响应解析失败或非确认响应")
                else:
                    self.logger.warning(f"响应数据长度不正确: {len(response)}")
                
                self.logger.warning(f"A69 未确认接收，重试 {attempt+1}/3")
            except serial.SerialException as e:
                self.logger.error(f"写入串口时发生错误: {e}")
                break
        
        # 发送失败后发布消息
        sent_msg = String()
        sent_msg.data = f"发送失败 - 速度: {speed} m/s, 转向角度: {steering_tire_angle} 度"
        self.received_data_publisher.publish(sent_msg)

    def listener_callback(self, msg):
        if self.mode != self.MODE_AUTO:
            # 如果当前不是自动模式，则忽略ROS消息
            return

        # 将转向角度从弧度转换为度，并四舍五入到3位小数
        steering_tire_angle_deg = round(math.degrees(msg.lateral.steering_tire_angle), 3)
        speed = round(msg.longitudinal.speed, 3)

        # 判断是否有显著变化
        if (self.pre_steering_tire_angle is None or
            self.pre_speed is None or
            abs(steering_tire_angle_deg - self.pre_steering_tire_angle) > 0.5 or
            abs(speed - self.pre_speed) > 0.05):

            self.logger.info('接收到 AckermannControlCommand 消息:')
            self.logger.info(f'  转向角度: {msg.lateral.steering_tire_angle} 弧度')
            self.logger.info(f'  速度: {msg.longitudinal.speed} m/s')
            self.logger.info(f'  加速度: {msg.longitudinal.acceleration} m/s²')

            self.pre_steering_tire_angle = steering_tire_angle_deg
            self.pre_speed = speed
            
            # 判断是否与上次发送的命令不同，避免重复发送
            if (self.last_sent_commands['speed'] != speed or
                self.last_sent_commands['steering_angle_deg'] != steering_tire_angle_deg):
                
                # 发送运动参数到A69设备
                self.send_motion_parameters(steering_tire_angle_deg, speed)

    def process_data_queue(self):
        """
        处理从串口接收到的数据队列
        """
        while not self.stop_event.is_set():
            try:
                if not self.data_queue.empty():
                    data = self.data_queue.get(block=False)
                    
                    # 处理接收到的数据
                    if len(data) == 10 and data[0] == 0x55 and data[1] == 0x7E and data[8] == 0x7E and data[9] == 0x55:
                        # 发布原始接收数据调试信息
                        raw_msg = String()
                        raw_msg.data = f"接收原始数据: {' '.join(f'{b:02X}' for b in data)}"
                        self.debug_publisher.publish(raw_msg)
                        
                        # 计算校验和
                        checksum = data[3] ^ data[4] ^ data[5] ^ data[6]
                        if checksum != data[7]:
                            self.logger.warning(f"接收数据校验和错误: 接收={data[7]:02X}, 计算={checksum:02X}")
                            continue
                        
                        # 根据不同的状态类型处理数据
                        st_type = data[2]
                        
                        if st_type == 0x01:  # 控制确认
                            self.logger.info("收到控制确认")
                        elif st_type == 0x02:  # 速度和角度反馈
                            # 假设数据是速度和角度反馈
                            speed_value = data[3]
                            angle_value = data[5]
                            
                            # 发布解析后的数据
                            feedback_msg = String()
                            feedback_msg.data = f"接收到设备状态反馈: 速度={speed_value}, 角度={angle_value}"
                            self.received_data_publisher.publish(feedback_msg)
                            
                            self.logger.info(f"设备状态反馈: 速度={speed_value}, 角度={angle_value}")
                        else:
                            self.logger.info(f"收到未知类型状态: ST={st_type:02X}")
                    else:
                        self.logger.warning(f"接收到无效的帧格式: {' '.join(f'{b:02X}' for b in data)}")
                        
                else:
                    time.sleep(0.01)  # 减少CPU占用
            except Exception as e:
                self.logger.error(f"处理数据队列时发生错误: {e}")
                time.sleep(0.1)

    def read_from_serial(self):
        while not self.stop_event.is_set() and self.ser.is_open:
            try:
                if self.ser.in_waiting > 0:
                    # 先尝试读取帧头
                    byte1 = self.ser.read(1)
                    if byte1 and byte1[0] == 0x55:
                        byte2 = self.ser.read(1)
                        if byte2 and byte2[0] == 0x7E:
                            # 找到帧头，读取剩余数据
                            remaining_data = self.ser.read(8)  # 读取剩余的8个字节
                            if len(remaining_data) == 8:
                                response_data = byte1 + byte2 + remaining_data
                                if len(response_data) == 10:
                                    # 将完整的10字节数据添加到队列
                                    self.data_queue.put(response_data)
                                    self.logger.debug(f"Received raw data: {' '.join(f'{byte:02X}' for byte in response_data)}")
                        else:
                            # 不是帧头，丢弃并继续
                            pass
                    else:
                        # 不是帧头，丢弃并继续
                        pass
                else:
                    time.sleep(0.01)  # 减少CPU占用
            except serial.SerialException as e:
                self.logger.error(f"读取串口时发生错误: {e}")
                break

    def keyboard_input_listener(self):
        with keyboard.Listener(on_press=self.on_press, on_release=self.on_release) as listener:
            listener.join()

    def on_press(self, key):
        if key == keyboard.Key.space:
            # 捕捉空格键，设置速度和转向角为0
            self.logger.info("捕捉到空格键。正在发送停止命令...")
            self.send_stop_command()
            with self.cmd_lock:
                self.speed = 0.0
                self.steering_angle_deg = 0.0
                # 重置上次发送的命令以允许重新发送
                self.last_sent_commands['speed'] = None
                self.last_sent_commands['steering_angle_deg'] = None
        else:
            try:
                key_char = key.char.lower()
                self.key_state.add(key_char)

                if key_char == 'm':
                    # 切换模式
                    self.toggle_mode()
                    self.send_stop_command()
                    with self.cmd_lock:
                        self.speed = 0.0
                        self.steering_angle_deg = 0.0
                        # 重置上次发送的命令以允许重新发送
                        self.last_sent_commands['speed'] = None
                        self.last_sent_commands['steering_angle_deg'] = None
            except AttributeError:
                pass  # 非字符键忽略

    def on_release(self, key):
        try:
            key_char = key.char.lower()
            self.key_state.discard(key_char)
        except AttributeError:
            pass  # 非字符键忽略

        if key == keyboard.Key.esc:
            self.logger.info("按下了 Esc 键。正在退出...")
            # 触发 ROS2 shutdown，rclpy.spin 会结束
            rclpy.shutdown()

    def toggle_mode(self):
        with self.cmd_lock:
            if self.mode == self.MODE_AUTO:
                self.mode = self.MODE_MANUAL
                self.logger.info("\n\n")
                self.logger.info("****切换到 MANUAL 模式****\n\n")
                # 重置上次发送的命令以便手动模式可以立即发送新指令
                self.last_sent_commands['steering_angle_deg'] = None
                self.last_sent_commands['speed'] = None
            else:
                self.mode = self.MODE_AUTO
                self.logger.info("****切换到 AUTO 模式****\n\n")
                # 不需要重置，因为自动模式依赖ROS消息

    def send_stop_command(self):
        """
        发送停止命令: 速度=0, 转向角=0
        """
        self.logger.info("发送停止命令")
        self.send_motion_parameters(0.0, 0.0)

    def stop_and_exit(self):
        self.send_stop_command()
        self.stop_event.set()
        if self.read_thread.is_alive():
            self.read_thread.join(timeout=1)
        if self.process_thread.is_alive():
            self.process_thread.join(timeout=1)
        if self.keyboard_thread.is_alive():
            # 由于 pynput 的 Listener 会阻塞，无法直接关闭，只能依靠设置 event 和让主程序退出
            pass
        if self.ser.is_open:
            self.ser.close()
            self.logger.info(f"已关闭串口: {self.ser.port}")

    def destroy_node(self):
        self.logger.info("正在销毁节点并发送停止命令...")
        self.stop_and_exit()
        super().destroy_node()

    def main_loop(self):
        """
        主循环，处理键盘输入并发送数据到串口。
        仅在手动模式下工作。
        """
        while rclpy.ok() and not self.stop_event.is_set():
            if self.mode == self.MODE_MANUAL:
                # 根据当前按下的键更新速度和转向角
                with self.cmd_lock:
                    new_speed = self.speed
                    new_steering_angle_deg = self.steering_angle_deg

                    if 'w' in self.key_state:
                        new_speed = min(self.speed + 0.1, 1.8)  # 最大速度限制为1.8
                    elif 's' in self.key_state:
                        new_speed = max(self.speed - 0.1, -1.8)  # 最低速度限制为-1.8
                    else:
                        new_speed = 0 # 无按键时set as 0

                    if 'a' in self.key_state:
                        new_steering_angle_deg = min(self.steering_angle_deg + 4.58, 34.38)  # 0.6 弧度
                    elif 'd' in self.key_state:
                        new_steering_angle_deg = max(self.steering_angle_deg - 4.58, -34.38)
                    else:
                        # 松开转向键时转向角逐渐回到0
                        if self.steering_angle_deg > 0:
                            new_steering_angle_deg = max(self.steering_angle_deg - 5.73, 0)  # ~0.1 弧度
                        elif self.steering_angle_deg < 0:
                            new_steering_angle_deg = min(self.steering_angle_deg + 5.73, 0)

                    # 四舍五入到3位小数
                    new_speed = round(new_speed, 3)
                    new_steering_angle_deg = round(new_steering_angle_deg, 3)

            # 检查是否有变化
            if self.mode == self.MODE_MANUAL:
                with self.cmd_lock:
                    if (new_speed != self.speed) or (new_steering_angle_deg != self.steering_angle_deg):
                        self.speed = new_speed
                        self.steering_angle_deg = new_steering_angle_deg

                        # 判断是否与上次发送的命令不同，避免重复发送
                        if (self.last_sent_commands['speed'] != self.speed or
                            self.last_sent_commands['steering_angle_deg'] != self.steering_angle_deg):

                            self.send_motion_parameters(self.steering_angle_deg, self.speed)
            time.sleep(0.1)  # 100ms 间隔

def main(args=None):
    rclpy.init(args=args)

    node = None
    try:
        node = IntegratedControlPublisher()
    except serial.SerialException:
        # 如果串口初始化失败，rclpy已经被关闭，不需要进一步操作
        sys.exit(1)
    except Exception as e:
        # 捕捉其他初始化错误
        logging.getLogger(__name__).error(f"节点初始化时发生未预期的错误: {e}")
        sys.exit(1)

    # 启动主循环线程以处理键盘输入
    main_loop_thread = threading.Thread(target=node.main_loop, daemon=True)
    main_loop_thread.start()

    # 显示提示信息
    print("\n========== Integrated Control Publisher ==========")
    print(f"当前模式: {node.mode}")
    print("控制指令:")
    print("  - 按 'm' 切换控制模式 (AUTO <-> MANUAL)")
    print("  - 手动模式下使用以下键盘控制:")
    print("      * 'w' 增加速度")
    print("      * 's' 减少速度")
    print("      * 'a' 向左转")
    print("      * 'd' 向右转")
    print("      * 'Space' 设置速度=0且转向角=0")
    print("  - 按 'Esc' 键退出程序")
    print("===================================================\n")

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('接收到键盘中断信号 (Ctrl+C)，正在退出...')
    finally:
        # 无论是通过 Ctrl+C 还是其他方式退出，确保资源被正确释放
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()
        sys.exit(0)  # 确保主线程退出

if __name__ == '__main__':
    main()