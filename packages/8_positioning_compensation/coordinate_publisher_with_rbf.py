#!/usr/bin/env python

import os
os.environ['PYGAME_HIDE_SUPPORT_PROMPT'] = "hide"

import sys
sys.path.append("/home/jarvislee-carla/WorkplaceCarlas/op_carla/op_bridge/op_bridge/fsm_lab_simulation")
import rclpy
import math
from transforms3d.euler import euler2mat, quat2euler, euler2quat
# 导入新的RBF变换器
from advanced_vr2sx_transformer import AdvancedVR2SandBoxTransformer
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseWithCovariance, Pose, Point, Quaternion
from std_msgs.msg import Header
from rclpy.node import Node 
import numpy as np
from rclpy.time import Time
import carla
import datetime
import logging
import setproctitle
import time

setproctitle.setproctitle('python_update_coordinate_rbf')
VEHICLE_NAME = "follow_adtruck"

# RBF标定结果目录 - 需要根据实际路径修改
CALIBRATION_RESULTS_DIR = "/home/cityu-fsm-lab-carla/lmx/packages/1_vive_tracker/Iteratively_process_data/results_bp3/RANSAC_SVD_Deformation_2025-07-29_15-37-34_1to28_scale_RMSE_4.0mm_final_coordinates.csv/results"  # 修改为实际路径

# Create log directory in the current working directory
LOG_DIR = os.path.join(os.getcwd(), "coordinate_logs_rbf")
os.makedirs(LOG_DIR, exist_ok=True)

class RBFTransformedSandBoxCoorPublisherNode(Node):
    def __init__(self, update_rate=0.02):
        super().__init__("rbf_transformed_SandBox_coordinate_publisher_node")
        self.publisher = self.create_publisher(
            PoseWithCovarianceStamped, f"/real_world/{VEHICLE_NAME}/transformed_with_covariance", 1
        )

        # Set up logging
        timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        log_file = os.path.join(LOG_DIR, f"coordinate_transform_rbf_{timestamp}.log")
        
        self.logger = logging.getLogger("coordinate_transform_rbf")
        self.logger.setLevel(logging.INFO)
        
        # Create file handler
        file_handler = logging.FileHandler(log_file)
        file_handler.setLevel(logging.INFO)
        
        # Create formatter and add to handler
        formatter = logging.Formatter('%(asctime)s - %(message)s')
        file_handler.setFormatter(formatter)
        
        # Add handler to logger
        self.logger.addHandler(file_handler)
        
        # Log header
        self.logger.info("Timestamp, Before_X, Before_Y, Before_Z, Before_Yaw_Deg, After_X, After_Y, After_Z, After_Yaw_Deg")
        
        # Log the path where logs are saved
        self.get_logger().info(f"RBF Coordinate logs will be saved to: {log_file}")

        # 初始化RBF变换器
        try:
            self.sandbox_transformer = AdvancedVR2SandBoxTransformer(
                tracker_name="tracker_1",
                calibration_dir=CALIBRATION_RESULTS_DIR,
                max_control_points=50,  # 优化性能
                rbf_kernel='cubic'   # 更快的核函数
            )
            # 配置选项2: 高速度 (推荐用于实时性要求高的场景)
            # self.sandbox_transformer = AdvancedVR2SandBoxTransformer(
            #     tracker_name="tracker_1",
            #     calibration_dir=CALIBRATION_RESULTS_DIR,
            #     max_control_points=30,
            #     rbf_kernel='linear'  # 最快
            # )
            
            # 配置选项3: 平衡模式 (推荐用于一般场景)
            # self.sandbox_transformer = AdvancedVR2SandBoxTransformer(
            #     tracker_name="tracker_1",
            #     calibration_dir=CALIBRATION_RESULTS_DIR,
            #     max_control_points=40,
            #     rbf_kernel='cubic'
            # )
            self.get_logger().info("RBF变换器初始化成功")
        except Exception as e:
            self.get_logger().error(f"RBF变换器初始化失败: {e}")
            raise
        
        self.previous_position = None
        self.previous_yaw = None
        self.current_position = None
        self.current_yaw = None
        self.log_flag = False
        
        # 记录上一次发布位姿的时间
        self.last_published_time = self.get_clock().now()
        self.similarity_matrix = np.array([
            [ 3.27650037e+01, -9.68207899e-01,  0.00000000e+00, -5.41650474e+01],
            [ 9.68207899e-01,  3.27650037e+01,  0.00000000e+00,  7.28873065e+01],
            [ 0.00000000e+00,  0.00000000e+00,  3.27793059e+01, -5.08468894e-02],
            [ 0.00000000e+00,  0.00000000e+00,  0.00000000e+00,  1.00000000e+00]
        ])
        
        # 创建定时器
        self.timer = self.create_timer(update_rate, self.update_position_yaw)

    def update_position_yaw(self):
        """更新位置和偏航角"""
        self.current_position, self.current_yaw = self.sandbox_transformer.get_transformed_coor()
        
        if self.current_position is None or self.current_yaw is None:
            self.get_logger().warning("RBF变换器返回None值")
            return
            
        if self.check_threshold(self.current_position, self.current_yaw):
            self.get_logger().info(
                f"\n RBF变换后新位置: x: {self.current_position[0]:.4f}, y: {self.current_position[1]:.4f}, z: {self.current_position[2]:.4f}, yaw: {math.degrees(self.current_yaw):.4f}°"
            )  
        self.publish_transformed_coor()

    def check_threshold(self, position, yaw, threshold=0.005):
        """检查位置和角度变化阈值"""
        if self.previous_position is None or self.previous_yaw is None:
            self.previous_position = position
            self.previous_yaw = yaw
            self.log_flag = True
            return True
            
        position_changed = (
            abs(position[0] - self.previous_position[0]) > threshold or
            abs(position[1] - self.previous_position[1]) > threshold or
            abs(position[2] - self.previous_position[2]) > threshold
        )
        yaw_changed = abs(yaw - self.previous_yaw) > np.radians(0.486)  # 约0.486度

        if position_changed or yaw_changed:
            self.previous_position = position
            self.previous_yaw = yaw
            self.log_flag = True
            return True
        else:
            self.current_position = self.previous_position
            self.current_yaw = self.previous_yaw
            self.log_flag = False
            return False
            
    def calculate_B_position(self, A_position, A_yaw):
        """计算B相对于A的位置偏移"""
        offset_A_to_B = np.array([0, 0, 0])

        rotation_matrix = np.array([
            [np.cos(A_yaw), -np.sin(A_yaw), 0],
            [np.sin(A_yaw),  np.cos(A_yaw), 0],
            [0, 0, 1]
        ])

        offset_in_C = np.dot(rotation_matrix, offset_A_to_B)
        B_position = A_position - offset_in_C
        return B_position
        
    def transform_coordinates_from_sandbox2carla(self, transformed_position__, transformed_yaw__):
        """从沙箱坐标系转换到Carla坐标系"""
        B_position_offseted = self.calculate_B_position(
            np.array([transformed_position__[0], transformed_position__[2], 0.0016]), 
            transformed_yaw__
        )

        original_point = np.array([B_position_offseted[0], B_position_offseted[1], B_position_offseted[2]])
        homogeneous_point = np.append(original_point, 1)
        transformed_homogeneous_point = self.similarity_matrix @ homogeneous_point
        transformed_point = transformed_homogeneous_point[:3] / transformed_homogeneous_point[3]

        _transformed_yaw = math.radians(-(math.degrees(transformed_yaw__) - 90))
        transformed_point[1] = -transformed_point[1]
        
        return transformed_point, _transformed_yaw    

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
    
    def publish_transformed_coor(self):
        """发布变换后的坐标"""
        if self.current_position is not None and self.current_yaw is not None:
            transformed_position, transformed_yaw = self.transform_coordinates_from_sandbox2carla(
                self.current_position, self.current_yaw
            )
            
            # 设置消息
            header = Header()
            header.stamp = self.get_clock().now().to_msg()
            header.frame_id = "real_world"

            pose = Pose()
            pose.position = Point(x=transformed_position[0], y=transformed_position[1], z=transformed_position[2])
            pose.orientation = Quaternion(
                x=self.RPY2quaternion(0, 0, transformed_yaw)[0],
                y=self.RPY2quaternion(0, 0, transformed_yaw)[1],
                z=self.RPY2quaternion(0, 0, transformed_yaw)[2],
                w=self.RPY2quaternion(0, 0, transformed_yaw)[3]
            )

            # 创建协方差矩阵
            covariance = [0.0] * 36
            covariance[0] = 0.01  # variance in x
            covariance[7] = 0.01  # variance in y
            covariance[14] = 0.01 # variance in z
            covariance[21] = 0.01 # variance in roll
            covariance[28] = 0.01 # variance in pitch
            covariance[35] = 0.01 # variance in yaw

            pose_with_cov = PoseWithCovariance(pose=pose, covariance=covariance)
            msg = PoseWithCovarianceStamped(header=header, pose=pose_with_cov)

            # 发布消息
            self.publisher.publish(msg)
            
            if self.log_flag:
                # 控制台日志
                self.get_logger().info(
                    f"\n RBF变换发布成功! x: {transformed_position[0]:.4f}, y: {transformed_position[1]:.4f}, z: {transformed_position[2]:.4f}, yaw: {math.degrees(transformed_yaw):.4f}°"
                )
                
                # 文件日志
                self.logger.info(
                    f"{datetime.datetime.now().isoformat()}, "
                    f"{self.current_position[0]}, {self.current_position[1]}, {self.current_position[2]}, {math.degrees(self.current_yaw)}, "
                    f"{transformed_position[0]}, {transformed_position[1]}, {transformed_position[2]}, {math.degrees(transformed_yaw)}"
                )

            self.last_published_time = self.get_clock().now()
        else:
            self.get_logger().error("!!! ERROR !!! RBF变换器当前位置未更新 !!! ")


if __name__ == "__main__":
    # 检查标定目录是否存在
    if not os.path.exists(CALIBRATION_RESULTS_DIR):
        print(f"错误: 标定结果目录不存在: {CALIBRATION_RESULTS_DIR}")
        print("请修改 CALIBRATION_RESULTS_DIR 变量为正确的路径")
        sys.exit(1)
    
    rclpy.init(args=None)

    try:
        # 创建节点
        rbf_transformed_sandbox_coor_publisher_node = RBFTransformedSandBoxCoorPublisherNode()
        
        print("RBF坐标变换节点已启动，开始发布变换后的坐标...")
        
        # 运行节点
        rclpy.spin(rbf_transformed_sandbox_coor_publisher_node)
        
    except Exception as e:
        print(f"节点运行失败: {e}")
    finally:
        rclpy.shutdown()