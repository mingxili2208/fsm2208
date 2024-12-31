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
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('timeout', 0.5)  # 超时时间，单位：秒

        port = self.get_parameter('port').get_parameter_value().string_value
        baudrate = self.get_parameter('baudrate').get_parameter_value().integer_value
        timeout = self.get_parameter('timeout').get_parameter_value().double_value
        self.MSG_TYPE_COMMAND = 0x01
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
        self.process_thread = threading.Thread(target=self.process_serial_data, daemon=True)
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

        # 初始化键盘监听相关变量
        self.key_state = set()
        self.keyboard_thread = threading.Thread(target=self.keyboard_input_listener, daemon=True)
        self.keyboard_thread.start()

        # 记录上次发送的命令，避免重复发送
        self.last_sent_commands = {
            'steering_angle_deg': None,
            'speed': None
        }

    def calculate_checksum(self, msg_type, steering_tire_angle, speed):
        # 打包数据为字节
        data = struct.pack('<Bff',  msg_type, steering_tire_angle, speed)
        checksum = 0
        for byte in data:
            checksum ^= byte
        return checksum

    def verify_checksum(self, data):
        if len(data) < 11:
            self.logger.info("Invalid checksum")
            return False
        try:
            _, msg_type, steering_tire_angle, speed, checksum = struct.unpack('<BBffB', data[:11])
        except struct.error:
            return False
        calculated_checksum = self.calculate_checksum(msg_type, steering_tire_angle, speed)
        return calculated_checksum == checksum

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
            self.logger.debug(f"当前转向角度: {steering_tire_angle_deg} 度, 速度: {speed} m/s")
            #
            # 计算校验和
            checksum = self.calculate_checksum(self.MSG_TYPE_COMMAND, steering_tire_angle_deg, speed)

            # 打包消息
            try:
                packed_msg = struct.pack('<BBffB', 0x42, self.MSG_TYPE_COMMAND, steering_tire_angle_deg, speed, checksum)
                hex_msg = " ".join(f"{byte:02X}" for byte in packed_msg)
            except struct.error as e:
                self.logger.error(f"打包消息时发生错误: {hex_msg}")
                return

            # 判断是否与上次发送的命令不同，避免重复发送
            if (self.last_sent_commands['speed'] != speed or
                self.last_sent_commands['steering_angle_deg'] != steering_tire_angle_deg):

                # 发送消息
                try:
                    bytes_written = self.ser.write(packed_msg)
                    #time.sleep(0.02)
                    self.logger.info(f"send_result  {bytes_written}")
                    self.ser.flush()  # 确保所有数据都已发送
                except serial.SerialException as e:
                    self.logger.error(f"写入串口时发生错误: {e}")
                    return

                # 将发送的消息写入日志文件（可选，建议移除）
                # self.write_to_log_file(f"Sent: {hex_msg}")  # 移除或保留根据需要

                # 在主脚本输出当前发送的speed和angle
                self.logger.info(f"发送到串口 - 速度: {speed} m/s, 转向角度: {steering_tire_angle_deg} 度")

                # 更新上次发送的命令
                self.last_sent_commands['speed'] = speed
                self.last_sent_commands['steering_angle_deg'] = steering_tire_angle_deg

                # 发布发送的命令信息到ROS2主题
                sent_msg = String()
                sent_msg.data = f"Sent - 速度: {speed} m/s, 转向角度: {steering_tire_angle_deg} 度"
                self.received_data_publisher.publish(sent_msg)

    def read_from_serial(self):
        # 移除文件logging，改为只将数据放入队列
        while not self.stop_event.is_set() and self.ser.is_open:
            try:
                if self.ser.in_waiting > 0:
                    response_data = self.ser.read(11)  # 假设响应消息长度为11字节
                    if response_data:
                        self.data_queue.put(response_data)
                        self.logger.debug(f"Received raw data: {' '.join(f'{byte:02X}' for byte in response_data)}")
                else:
                    time.sleep(0.01)  # 减少CPU占用
            except serial.SerialException as e:
                self.logger.error(f"读取串口时发生错误: {e}")
                break

    def process_serial_data(self):
        while not self.stop_event.is_set():
            try:
                response = self.data_queue.get(timeout=0.1)  # 等待数据，避免阻塞
                if len(response) >= 11 and response[0] == 0x42:
                    if self.verify_checksum(response):
                        try:
                            _, msg_type, steering_tire_angle_resp, speed_resp, _ = struct.unpack('<BBffB', response[:11])
                            steering_tire_angle_resp = round(steering_tire_angle_resp, 3)
                            speed_resp = round(speed_resp, 3)
                            if msg_type == 0x02:
                                log_msg = f"Arduino 发送正确: 转向角度={steering_tire_angle_resp} 度, 速度={speed_resp} m/s"
                                self.logger.info(log_msg)
                                # 发布到新的ROS2主题
                                received_msg = String()
                                received_msg.data = log_msg
                                self.received_data_publisher.publish(received_msg)
                            elif msg_type == 0x03:
                                log_msg = f"!!!!!! Arduino 发送错误!!!!!!!\n转向角度={steering_tire_angle_resp} 度, 速度={speed_resp} m/s"
                                self.logger.warning(response)
                                self.logger.warning(log_msg)
                                # 发布到新的ROS2主题
                                received_msg = String()
                                received_msg.data = log_msg
                                self.received_data_publisher.publish(received_msg)
                            elif msg_type == 0x05:
                                log_msg = "!!!!!! Arduino 5s 内未接收到消息!!!!!!!"
                                self.logger.error(response)
                                self.logger.error(log_msg)
                                # 发布到新的ROS2主题
                                received_msg = String()
                                received_msg.data = log_msg
                                self.received_data_publisher.publish(received_msg)
                        except struct.error as e:
                            log_msg = f"解析响应数据失败: {e}"
                            self.logger.error(log_msg)
                            # 发布到新的ROS2主题
                            received_msg = String()
                            received_msg.data = log_msg
                            self.received_data_publisher.publish(received_msg)
                    else:
                        log_msg = "!!!!!! Arduino 发送的校验和不正确!!!!!!!"
                        self.logger.error(response)
                        self.logger.error(log_msg)
                        # 发布到新的ROS2主题
                        received_msg = String()
                        received_msg.data = log_msg
                        self.received_data_publisher.publish(received_msg)
                else:
                    self.logger.error(response)
                    log_msg = "!!!!!! Arduino 发送的消息格式不正确!!!!!!!"
                    self.logger.error(log_msg)
                    # 发布到新的ROS2主题
                    received_msg = String()
                    received_msg.data = log_msg
                    self.received_data_publisher.publish(received_msg)
            except queue.Empty:
                continue  # 没有数据，继续等待
            except Exception as e:
                self.logger.error(f"处理串口数据时发生错误: {e}")

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
        try:
            steering_tire_angle_deg = round(0.0, 3)
            speed = round(0.0, 3)
            checksum = self.calculate_checksum(self.MSG_TYPE_COMMAND, steering_tire_angle_deg, speed)
            packed_msg = struct.pack('<BBffB', 0x42, self.MSG_TYPE_COMMAND, steering_tire_angle_deg, speed, checksum)
            hex_msg = " ".join(f"{byte:02X}" for byte in packed_msg)
            self.logger.info(f"发送停止命令: {hex_msg}")
            self.ser.write(packed_msg)
            self.ser.flush()
            self.logger.info(f"发送到串口 - 速度: {speed} m/s, 转向角度: {steering_tire_angle_deg} 度")

            # 发布发送的停止命令信息到ROS2主题
            sent_msg = String()
            sent_msg.data = f"Sent Stop - 速度: {speed} m/s, 转向角度: {steering_tire_angle_deg} 度"
            self.received_data_publisher.publish(sent_msg)

            #time.sleep(0.1)  # 确保数据发送完毕
        except serial.SerialException as e:
            self.logger.error(f"发送停止命令时发生串口错误: {e}")
        except struct.error as e:
            self.logger.error(f"打包停止命令时发生错误: {e}")

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

                            self.send_serial_command(self.steering_angle_deg, self.speed)
                            self.last_sent_commands['speed'] = self.speed
                            self.last_sent_commands['steering_angle_deg'] = self.steering_angle_deg
            time.sleep(0.1)  # 100ms 间隔

    def send_serial_command(self, steering_angle_deg, speed):
        try:
            # 对发送的float值进行四舍五入到3位小数
            steering_angle_deg = round(steering_angle_deg, 3)
            speed = round(speed, 3)

            # 计算校验和
            checksum = self.calculate_checksum(self.MSG_TYPE_COMMAND, steering_angle_deg, speed)

            # 打包消息
            packed_msg = struct.pack('<BBffB', 0x42, 0x01, steering_angle_deg, speed, checksum)
            hex_msg = " ".join(f"{byte:02X}" for byte in packed_msg)

            # 判断是否与上次发送的命令不同，避免重复发送
            if (self.last_sent_commands['speed'] != speed or
                self.last_sent_commands['steering_angle_deg'] != steering_angle_deg):

                # 发送消息
                bytes_written = self.ser.write(packed_msg)
                self.ser.flush()

                # 将发送的消息写入日志文件（可选，建议移除）
                # self.write_to_log_file(f"Sent: {hex_msg}")  # 移除或保留根据需要

                # 在主脚本输出当前发送的speed和angle
                self.logger.info(f"发送到串口 - 速度: {speed} m/s, 转向角度: {steering_angle_deg} 度")

                # 发布发送的命令信息到ROS2主题
                sent_msg = String()
                sent_msg.data = f"Sent - 速度: {speed} m/s, 转向角度: {steering_angle_deg} 度"
                self.received_data_publisher.publish(sent_msg)

                # 更新上次发送的命令
                self.last_sent_commands['speed'] = speed
                self.last_sent_commands['steering_angle_deg'] = steering_angle_deg

        except struct.error as e:
            self.logger.error(f"打包命令时发生错误: {e}")
        except serial.SerialException as e:
            self.logger.error(f"写入串口时发生错误: {e}")

    def write_to_log_file(self, message):
        try:
            with self.log_lock:
                with open(self.log_file_path, 'a') as log_file:
                    log_file.write(f"{message}\n")
        except Exception as e:
            self.logger.error(f"写入日志文件时发生错误: {e}")

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