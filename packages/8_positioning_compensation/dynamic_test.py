#!/usr/bin/env python

import os
os.environ['PYGAME_HIDE_SUPPORT_PROMPT'] = "hide"

import sys
sys.path.append("/home/jarvislee-carla/WorkplaceCarlas/op_carla/op_bridge/op_bridge/fsm_lab_simulation")

import time
import struct
import serial
import threading
import queue
import numpy as np
import math
import logging
from datetime import datetime
from pynput import keyboard
from vive_tracker import ViveTrackerModule
from advanced_vr2sx_transformer import AdvancedVR2SandBoxTransformer
from scipy.spatial.transform import Rotation as R

# 设置日志
logging.basicConfig(level=logging.INFO)

class IntegratedDataCollector:
    def __init__(self):
        # 串口配置
        self.SERIAL_PORT = "/dev/ttyUSB0"
        self.BAUD_RATE = 9600
        self.TRACKER_NAME = "tracker_1"
        
        # RBF标定结果目录
        self.CALIBRATION_RESULTS_DIR = "/home/cityu-fsm-lab-carla/lmx/packages/1_vive_tracker/Iteratively_process_data/results/RANSAC_SVD_Deformation_RMSE_3.679_2025-07-07_14-41-59_1to32_scale_RMSE_4.0mm_combined_corrected_tracker_data_2.csv/results"
        
        # 创建数据存储目录
        self.data_dir, self.timestamp = self.create_data_directory()
        self.csv_file = os.path.join(self.data_dir, f"integrated_data_{self.timestamp}.csv")
        self.log_file = os.path.join(self.data_dir, f"collection_log_{self.timestamp}.txt")
        
        # 初始化组件
        self.init_logging()
        self.init_vive_tracker()
        self.init_serial()
        self.init_rbf_transformer()
        
        # 数据收集控制
        self.running = True
        self.recording = False
        self.collected_data = []
        
        # 数据处理设置
        self.buffer_size = 5
        self.data_buffer = []
        self.last_recorded_position = None
        self.min_change_threshold = 0.01  # 1cm位置变化阈值
        
        # 统计信息
        self.total_collections = 0
        self.successful_transforms = 0
        self.failed_transforms = 0
        
    def create_data_directory(self):
        """创建数据存储目录"""
        timestamp = datetime.now().strftime("%m%d_%H%M%S")
        data_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "integrated_data", timestamp)
        os.makedirs(data_dir, exist_ok=True)
        print(f"[系统] 数据将保存到目录: {data_dir}")
        return data_dir, timestamp
    
    def init_logging(self):
        """初始化日志"""
        self.logger = logging.getLogger("integrated_collector")
        self.logger.setLevel(logging.INFO)
        
        # 文件处理器
        file_handler = logging.FileHandler(self.log_file)
        file_handler.setLevel(logging.INFO)
        formatter = logging.Formatter('%(asctime)s - %(levelname)s - %(message)s')
        file_handler.setFormatter(formatter)
        self.logger.addHandler(file_handler)
        
        # 控制台处理器
        console_handler = logging.StreamHandler()
        console_handler.setLevel(logging.INFO)
        console_handler.setFormatter(formatter)
        self.logger.addHandler(console_handler)
        
        self.logger.info("集成数据收集器初始化开始")
    
    def init_vive_tracker(self):
        """初始化Vive Tracker"""
        try:
            self.logger.info("初始化 ViveTrackerModule...")
            self.vtm = ViveTrackerModule()
            self.vtm.print_discovered_objects()
            self.tracker = self.vtm.devices.get(self.TRACKER_NAME)
            
            if self.tracker is None:
                raise RuntimeError(f"Tracker '{self.TRACKER_NAME}' 未找到!")
            
            self.logger.info(f"Tracker '{self.TRACKER_NAME}' 连接成功")
        except Exception as e:
            self.logger.error(f"Vive Tracker初始化失败: {e}")
            raise
    
    def init_serial(self):
        """初始化串口"""
        try:
            self.ser = serial.Serial(self.SERIAL_PORT, self.BAUD_RATE, timeout=0.1)
            if self.ser.is_open:
                self.logger.info(f"成功打开串口: {self.SERIAL_PORT}")
        except serial.SerialException as e:
            self.logger.error(f"无法打开串口 {self.SERIAL_PORT}: {e}")
            raise
    
    def init_rbf_transformer(self):
        """初始化RBF变换器"""
        try:
            self.sandbox_transformer = AdvancedVR2SandBoxTransformer(
                tracker_name=self.TRACKER_NAME,
                calibration_dir=self.CALIBRATION_RESULTS_DIR,
                max_control_points=50,
                rbf_kernel='cubic'
            )
            self.logger.info("RBF变换器初始化成功")
        except Exception as e:
            self.logger.error(f"RBF变换器初始化失败: {e}")
            raise
    
    def calculate_checksum_r(self, data):
        """计算A69校验和"""
        return data[3] ^ data[4] ^ data[5] ^ data[6]
    
    def parse_a69_data(self, response):
        """解析A69激光数据"""
        if len(response) != 10 or response[0] != 0x55 or response[1] != 0x7E or response[8] != 0x7E or response[9] != 0x55:
            return None
        
        checksum = response[7]
        computed_checksum = self.calculate_checksum_r(response)
        
        if checksum != computed_checksum:
            return None
        
        distance_2 = (response[3] << 8) | response[4]
        distance_3 = (response[5] << 8) | response[6]
        return distance_2 / 1000.0, distance_3 / 1000.0
    
    def send_a69_data_request(self):
        """发送A69数据请求"""
        tx_buf = bytearray([0x55, 0x7E, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x7E, 0x55])
        tx_buf[7] = self.calculate_checksum_r(tx_buf)
        self.ser.write(tx_buf)
        self.ser.flush()
    
    def RPY2quaternion(self, roll, pitch, yaw):
        """欧拉角转四元数"""
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)
        
        w = cr * cp * cy + sr * sp * sy
        x = sr * cp * cy - cr * sp * sy
        y = cr * sp * cy + sr * cp * sy
        z = cr * cp * sy - sr * sp * cy
        
        return x, y, z, w
    
    def process_complete_data_point(self, timestamp, laser_data, tracker_data):
        """处理完整的数据点（激光+追踪器+变换后）"""
        try:
            self.total_collections += 1
            
            # 解析数据 - 修正tracker数据解析顺序
            distance_2, distance_3 = laser_data
            
            # tracker_data格式: [x, y, z, roll, yaw, pitch]
            tracker_position = [tracker_data[0], tracker_data[1], tracker_data[2]]
            tracker_orientation = [tracker_data[3], tracker_data[4], tracker_data[5]]  # roll, yaw, pitch
            
            # 获取RBF变换后的坐标
            try:
                sandbox_position, sandbox_yaw = self.sandbox_transformer.get_transformed_coor()
                if sandbox_position is None or sandbox_yaw is None:
                    self.logger.warning("RBF变换返回None值")
                    self.failed_transforms += 1
                    return False
                
                self.successful_transforms += 1
                
            except Exception as e:
                self.logger.error(f"坐标变换失败: {e}")
                self.failed_transforms += 1
                return False
            
            # 检查位置变化
            current_position = np.array([tracker_position[0], tracker_position[2]])  # x, z
            
            if self.last_recorded_position is not None:
                position_change = np.linalg.norm(current_position - self.last_recorded_position)
                if position_change < self.min_change_threshold:
                    # 变化太小，添加到缓冲区
                    self.data_buffer.append({
                        'timestamp': timestamp,
                        'laser': {'distance_2': distance_2, 'distance_3': distance_3},
                        'tracker': {
                            'x': tracker_position[0], 'y': tracker_position[1], 'z': tracker_position[2],
                            'roll': tracker_orientation[0], 'yaw': tracker_orientation[1], 'pitch': tracker_orientation[2]
                        },
                        'sandbox': {
                            'x': sandbox_position[0], 'y': sandbox_position[1], 'z': sandbox_position[2],
                            'yaw': sandbox_yaw
                        }
                    })
                    
                    # 缓冲区满时处理
                    if len(self.data_buffer) >= self.buffer_size:
                        self.process_buffer_average()
                    
                    return True
            
            # 显著变化或第一个数据点
            if self.data_buffer:
                self.process_buffer_average()
            
            # 记录当前数据点
            self.record_data_point({
                'timestamp': timestamp,
                'laser': {'distance_2': distance_2, 'distance_3': distance_3},
                'tracker': {
                    'x': tracker_position[0], 'y': tracker_position[1], 'z': tracker_position[2],
                    'roll': tracker_orientation[0], 'yaw': tracker_orientation[1], 'pitch': tracker_orientation[2]
                },
                'sandbox': {
                    'x': sandbox_position[0], 'y': sandbox_position[1], 'z': sandbox_position[2],
                    'yaw': sandbox_yaw
                }
            })
            
            self.last_recorded_position = current_position
            
            self.logger.info(f"数据收集成功 - Laser: d2={distance_2:.3f}m, d3={distance_3:.3f}m | "
                           f"Tracker: x={tracker_position[0]:.3f}, y={tracker_position[1]:.3f}, z={tracker_position[2]:.3f} | "
                           f"Roll={math.degrees(tracker_orientation[0]):.1f}°, Yaw={math.degrees(tracker_orientation[1]):.1f}°, Pitch={math.degrees(tracker_orientation[2]):.1f}° | "
                           f"Sandbox: x={sandbox_position[0]:.3f}, y={sandbox_position[1]:.3f}, yaw={math.degrees(sandbox_yaw):.1f}°")
            
            return True
            
        except Exception as e:
            self.logger.error(f"数据处理失败: {e}")
            return False
    
    def process_buffer_average(self):
        """处理缓冲区平均值"""
        if not self.data_buffer:
            return
        
        # 计算平均值
        avg_data = {
            'timestamp': self.data_buffer[-1]['timestamp'],  # 使用最新时间戳
            'laser': {
                'distance_2': sum(d['laser']['distance_2'] for d in self.data_buffer) / len(self.data_buffer),
                'distance_3': sum(d['laser']['distance_3'] for d in self.data_buffer) / len(self.data_buffer)
            },
            'tracker': {},
            'sandbox': {}
        }
        
        # 计算各坐标系的平均值
        for coord_sys in ['tracker', 'sandbox']:
            for key in self.data_buffer[0][coord_sys].keys():
                avg_data[coord_sys][key] = sum(d[coord_sys][key] for d in self.data_buffer) / len(self.data_buffer)
        
        self.logger.info(f"缓冲区平均值处理: {len(self.data_buffer)}个点 -> 1个平均值点")
        self.data_buffer.clear()
    
    def record_data_point(self, data_point):
        """记录数据点"""
        self.collected_data.append(data_point)
        self.logger.info(f"记录数据点 #{len(self.collected_data)}")
    
    def data_collection_loop(self):
        """数据收集主循环"""
        consecutive_failures = 0
        max_consecutive_failures = 5
        
        while self.running:
            if self.recording:
                try:
                    # 发送激光数据请求
                    self.send_a69_data_request()
                    
                    # 读取激光数据
                    response = self.ser.read(10)
                    laser_data = self.parse_a69_data(response)
                    
                    if laser_data:
                        # 立即获取tracker数据
                        tracker_data = self.tracker.get_pose_euler()
                        timestamp = time.time()
                        
                        # 处理完整数据点
                        success = self.process_complete_data_point(timestamp, laser_data, tracker_data)
                        
                        if success:
                            consecutive_failures = 0
                        else:
                            consecutive_failures += 1
                    else:
                        consecutive_failures += 1
                        self.logger.warning("激光数据解析失败")
                    
                    if consecutive_failures >= max_consecutive_failures:
                        self.logger.error(f"连续{consecutive_failures}次数据收集失败")
                        consecutive_failures = 0  # 重置计数器
                
                except Exception as e:
                    self.logger.error(f"数据收集过程出错: {e}")
                    consecutive_failures += 1
            
            time.sleep(0.02)  # 50Hz采集频率
    
    def save_to_csv(self):
        """保存数据到CSV文件"""
        if not self.collected_data:
            self.logger.warning("没有数据需要保存")
            return
        
        try:
            with open(self.csv_file, 'w') as f:
                # 写入表头
                header = [
                    "Timestamp",
                    "Laser_Distance_2", "Laser_Distance_3",
                    "Tracker_X", "Tracker_Y", "Tracker_Z", "Tracker_Roll", "Tracker_Yaw", "Tracker_Pitch",
                    "Sandbox_X", "Sandbox_Y", "Sandbox_Z", "Sandbox_Yaw"
                ]
                f.write(",".join(header) + "\n")
                
                # 写入数据
                for data in self.collected_data:
                    row = [
                        f"{data['timestamp']:.6f}",
                        f"{data['laser']['distance_2']:.6f}", f"{data['laser']['distance_3']:.6f}",
                        f"{data['tracker']['x']:.6f}", f"{data['tracker']['y']:.6f}", f"{data['tracker']['z']:.6f}",
                        f"{data['tracker']['roll']:.6f}", f"{data['tracker']['yaw']:.6f}", f"{data['tracker']['pitch']:.6f}",
                        f"{data['sandbox']['x']:.6f}", f"{data['sandbox']['y']:.6f}", f"{data['sandbox']['z']:.6f}",
                        f"{data['sandbox']['yaw']:.6f}"
                    ]
                    f.write(",".join(row) + "\n")
            
            self.logger.info(f"成功保存{len(self.collected_data)}条数据到: {self.csv_file}")
            self.print_statistics()
            
        except Exception as e:
            self.logger.error(f"保存CSV文件失败: {e}")
    
    def print_statistics(self):
        """打印统计信息"""
        self.logger.info("=== 数据收集统计 ===")
        self.logger.info(f"总收集次数: {self.total_collections}")
        self.logger.info(f"成功变换次数: {self.successful_transforms}")
        self.logger.info(f"失败变换次数: {self.failed_transforms}")
        self.logger.info(f"最终记录数据点: {len(self.collected_data)}")
        if self.total_collections > 0:
            success_rate = (self.successful_transforms / self.total_collections) * 100
            self.logger.info(f"变换成功率: {success_rate:.2f}%")
    
    def on_key_press(self, key):
        """按键处理"""
        try:
            if key.char == 'r' and not self.recording:
                self.recording = True
                self.collected_data.clear()
                self.data_buffer.clear()
                self.last_recorded_position = None
                self.total_collections = 0
                self.successful_transforms = 0
                self.failed_transforms = 0
                self.logger.info("开始数据收集...")
                
            elif key.char == 'e' and self.recording:
                self.recording = False
                if self.data_buffer:
                    self.process_buffer_average()
                self.logger.info("停止数据收集")
                self.save_to_csv()
                
        except AttributeError:
            pass
    
    def on_key_release(self, key):
        """按键释放处理"""
        if key == keyboard.Key.esc:
            self.logger.info("程序退出...")
            if self.recording:
                if self.data_buffer:
                    self.process_buffer_average()
                self.save_to_csv()
            self.running = False
            return False
    
    def run(self):
        """运行主程序"""
        try:
            # 启动数据收集线程
            collection_thread = threading.Thread(target=self.data_collection_loop, daemon=True)
            collection_thread.start()
            
            # 启动键盘监听
            listener = keyboard.Listener(on_press=self.on_key_press, on_release=self.on_key_release)
            listener.start()
            
            self.logger.info("=== 集成数据收集器已启动 ===")
            self.logger.info("按 'R' 开始录制，按 'E' 停止录制，按 'Esc' 退出程序")
            self.logger.info(f"位置变化阈值: {self.min_change_threshold*1000:.1f}mm")
            self.logger.info(f"数据缓冲区大小: {self.buffer_size}")
            
            # 主循环
            while self.running:
                time.sleep(0.1)
                
        except KeyboardInterrupt:
            self.logger.info("接收到Ctrl+C，程序退出...")
            if self.recording and self.data_buffer:
                self.process_buffer_average()
            self.save_to_csv()
            self.running = False
            
        finally:
            if hasattr(self, 'ser') and self.ser.is_open:
                self.ser.close()
                self.logger.info("串口已关闭")


if __name__ == "__main__":
    # 检查标定目录是否存在
    calibration_dir = "/home/cityu-fsm-lab-carla/lmx/packages/1_vive_tracker/Iteratively_process_data/results/RANSAC_SVD_Deformation_RMSE_3.679_2025-07-07_14-41-59_1to32_scale_RMSE_4.0mm_combined_corrected_tracker_data_2.csv/results"
    
    if not os.path.exists(calibration_dir):
        print(f"错误: 标定结果目录不存在: {calibration_dir}")
        print("请修改代码中的 CALIBRATION_RESULTS_DIR 变量为正确的路径")
        sys.exit(1)
    
    try:
        collector = IntegratedDataCollector()
        collector.run()
    except Exception as e:
        print(f"程序运行失败: {e}")
        sys.exit(1)