#!/usr/bin/env python

import os
os.environ['PYGAME_HIDE_SUPPORT_PROMPT'] = "hide"

import sys
import math
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseWithCovariance, Pose, Point, Quaternion
from std_msgs.msg import Header

# 添加项目路径
sys.path.append("/home/jarvislee-carla/Workspace/Carlas/op_carla/op_bridge/op_bridge/fsm_lab_simulation")
from vr2sx_pysurvive import VR2SandBoxTransformer

VEHICLE_NAME = "follow_adtruck"
PUBLISH_RATE = 0.05  # 发布频率，单位秒
POSITION_THRESHOLD = 0.005  # 位置更新阈值
TIMEOUT_THRESHOLD = 0.02  # 超时阈值，单位秒

class CoordinateTransformer:
    """坐标转换工具类，处理各种坐标系之间的转换"""
    
    def __init__(self):
        """初始化坐标转换器"""
        self.similarity_matrix = np.array([
            [3.27650037e+01, -9.68207899e-01, 0.00000000e+00, -5.41650474e+01],
            [9.68207899e-01, 3.27650037e+01, 0.00000000e+00, 7.28873065e+01],
            [0.00000000e+00, 0.00000000e+00, 3.27793059e+01, -5.08468894e-02],
            [0.00000000e+00, 0.00000000e+00, 0.00000000e+00, 1.00000000e+00]
        ])
    
    def sandbox_to_carla(self, sandbox_position, sandbox_yaw):
        """
        将沙盒坐标转换为CARLA坐标
        
        Args:
            sandbox_position (np.array): 沙盒中的位置 [x, y, z]
            sandbox_yaw (float): 沙盒中的偏航角（弧度）
            
        Returns:
            tuple: (转换后的位置, 转换后的偏航角)
        """
        # 重组坐标，沙盒和CARLA坐标系的映射
        original_point = np.array([
            sandbox_position[0],  # x
            sandbox_position[2],  # z作为y
            0.0016               # 固定高度
        ])
        
        # 转换为齐次坐标
        homogeneous_point = np.append(original_point, 1)
        
        # 应用相似性变换
        transformed_homogeneous = self.similarity_matrix @ homogeneous_point
        transformed_point = transformed_homogeneous[:3] / transformed_homogeneous[3]
        
        # 调整y轴方向和偏航角
        transformed_point[1] = -transformed_point[1]  # Carla的y轴方向与沙盒相反
        transformed_yaw = math.radians(-(math.degrees(sandbox_yaw) - 90))
        
        return transformed_point, transformed_yaw
    
    @staticmethod
    def euler_to_quaternion(roll, pitch, yaw):
        """
        将欧拉角转换为四元数
        
        Args:
            roll (float): 横滚角（弧度）
            pitch (float): 俯仰角（弧度）
            yaw (float): 偏航角（弧度）
            
        Returns:
            tuple: 四元数 (x, y, z, w)
        """
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


class TransformedSandboxCoordinatePublisher(Node):
    """发布转换后的沙盒坐标的ROS2节点"""
    
    def __init__(self, update_rate=PUBLISH_RATE):
        """
        初始化节点
        
        Args:
            update_rate (float): 发布频率，单位秒
        """
        super().__init__("transformed_sandbox_coordinate_publisher")
        
        # 创建发布器
        self.publisher = self.create_publisher(
            PoseWithCovarianceStamped, 
            f"/real_world/{VEHICLE_NAME}/transformed_with_covariance", 
            10
        )
        
        # 创建定时器
        self.timer = self.create_timer(update_rate, self.update_coordinates)
        self.publish_timer = self.create_timer(TIMEOUT_THRESHOLD, self.publish_if_needed)
        
        # 初始化工具类
        self.sandbox_transformer = VR2SandBoxTransformer()
        self.coordinate_transformer = CoordinateTransformer()
        
        # 状态变量
        self.current_sandbox_position = None
        self.current_sandbox_yaw = None
        self.current_sandbox_quaternion = None
        self.has_quaternion = False
        
        self.last_published_position = None
        self.last_published_yaw = None
        
        self.last_update_time = self.get_clock().now()
        self.last_publish_time = self.get_clock().now()
        self.has_new_data = False
        self.log_flag=False
        self.updated_flag=False
        self.get_logger().info("沙盒坐标转换发布器已初始化")
    
    def is_significant_update(self, new_position, old_position):
        """
        判断位置更新是否显著
        
        Args:
            new_position: 新位置
            old_position: 旧位置
            
        Returns:
            bool: 是否为显著更新
        """
        if old_position is None:
            return True
        if self.last_published_yaw is None:
            return True
        # 计算均方距离
        mse = np.mean([(a - b) ** 2 for a, b in zip(new_position, old_position)])
        yaw_mse=abs(self.current_sandbox_yaw - self.last_published_yaw)
        if mse>POSITION_THRESHOLD or  yaw_mse > np.radians(0.286):
            self.log_flag=True
            return True
        else:
            self.log_flag=False
            return False
    
    def update_coordinates(self):
        """更新坐标，检查是否有显著变化"""
        try:
            # 检查是否提供四元数
            has_quaternion = self.sandbox_transformer.has_quaternion()
            
            if has_quaternion:
                # 获取带四元数的坐标
                position, quaternion = self.sandbox_transformer.get_transformed_coor_with_quaternion()
                
                if self.is_significant_update(position, self.current_sandbox_position):
                    self.current_sandbox_position = position
                    self.current_sandbox_quaternion = quaternion
                    self.has_quaternion = True
                    self.has_new_data = True
                    self.last_update_time = self.get_clock().now()
            else:
                # 获取带欧拉角的坐标
                position, yaw = self.sandbox_transformer.get_transformed_coor()
                
                if (self.is_significant_update(position, self.current_sandbox_position) or 
                    self.current_sandbox_yaw is None or
                    abs(yaw - self.current_sandbox_yaw) > math.radians(1)):
                    
                    self.current_sandbox_position = position
                    self.current_sandbox_yaw = yaw
                    self.has_quaternion = False
                    self.has_new_data = True
                    self.last_update_time = self.get_clock().now()
                    
        except Exception as e:
            self.get_logger().error(f"更新坐标时出错: {str(e)}")
    
    def publish_if_needed(self):
        """检查是否需要发布坐标"""
        current_time = self.get_clock().now()
        time_since_update = current_time - self.last_update_time
        time_since_publish = current_time - self.last_publish_time
        
        # 如果有新数据或超过超时阈值，则发布
        if self.has_new_data or time_since_publish.nanoseconds > TIMEOUT_THRESHOLD * 1e9:
            if self.has_quaternion:
                self.publish_with_quaternion()
            else:
                self.publish_with_euler()
                
            self.has_new_data = False
            self.last_publish_time = current_time
    
    def create_pose_message(self, position, orientation):
        """
        创建位姿消息
        
        Args:
            position (np.array): 位置
            orientation (Quaternion): 方向四元数
            
        Returns:
            PoseWithCovarianceStamped: ROS2位姿消息
        """
        try:
            # 创建消息头
            header = Header()
            header.stamp = self.get_clock().now().to_msg()
            header.frame_id = "real_world"
            
            # 创建位姿
            pose = Pose()
            pose.position = Point(x=position[0], y=position[1], z=position[2])
            pose.orientation = orientation
            
            # 创建协方差矩阵
            covariance = [0.0] * 36
            covariance[0] = 0.01   # x方差
            covariance[7] = 0.01   # y方差
            covariance[14] = 0.01  # z方差
            covariance[21] = 0.01  # roll方差
            covariance[28] = 0.01  # pitch方差
            covariance[35] = 0.01  # yaw方差
            
            # 创建带协方差的位姿
            pose_with_cov = PoseWithCovariance(pose=pose, covariance=covariance)
            
            # 创建完整消息
            msg = PoseWithCovarianceStamped(header=header, pose=pose_with_cov)
            
            return msg
        except Exception as e:
            self.get_logger().error(f"创建位姿消息失败: {str(e)}")
            return None
    
    def publish_with_euler(self):
        """使用欧拉角发布坐标"""
        if self.current_sandbox_position is None or self.current_sandbox_yaw is None:
            return
            
        try:
            # 转换坐标
            carla_position, carla_yaw = self.coordinate_transformer.sandbox_to_carla(
                self.current_sandbox_position, self.current_sandbox_yaw
            )
            
            # 转换欧拉角为四元数
            quat = self.coordinate_transformer.euler_to_quaternion(0, 0, carla_yaw)
            orientation = Quaternion(x=quat[0], y=quat[1], z=quat[2], w=quat[3])
            
            # 创建并发布消息
            msg = self.create_pose_message(carla_position, orientation)
            if msg:
                self.publisher.publish(msg)
                self.last_published_position = carla_position
                self.last_published_yaw = carla_yaw
                if self.log_flag:
                    self.get_logger().debug(
                        f"已发布欧拉角转换坐标: x={carla_position[0]:.4f}, y={carla_position[1]:.4f}, "
                        f"z={carla_position[2]:.4f}, yaw={math.degrees(carla_yaw):.2f}°"
                    )
        except Exception as e:
            self.get_logger().error(f"发布欧拉角坐标时出错: {str(e)}")
    
    def publish_with_quaternion(self):
        """使用四元数发布坐标"""
        if self.current_sandbox_position is None or self.current_sandbox_quaternion is None:
            return
            
        try:
            # 转换位置坐标
            # 注意：这里我们只转换位置，原始四元数需要单独处理
            original_point = np.array([
                self.current_sandbox_position[0],
                self.current_sandbox_position[2],
                0.0016
            ])
            
            # 转换为齐次坐标
            homogeneous_point = np.append(original_point, 1)
            
            # 应用相似性变换
            transformed_homogeneous = self.coordinate_transformer.similarity_matrix @ homogeneous_point
            carla_position = transformed_homogeneous[:3] / transformed_homogeneous[3]
            carla_position[1] = -carla_position[1]  # 调整y轴方向
            
            # 从四元数中提取并应用到Carla坐标系
            qx, qy, qz, qw = self.current_sandbox_quaternion
            orientation = Quaternion(x=qx, y=qy, z=qz, w=qw)
            
            # 创建并发布消息
            msg = self.create_pose_message(carla_position, orientation)
            if msg:
                self.publisher.publish(msg)
                self.last_published_position = carla_position
                if self.log_flag:
                    self.get_logger().debug(
                        f"已发布四元数转换坐标: x={carla_position[0]:.4f}, y={carla_position[1]:.4f}, "
                        f"z={carla_position[2]:.4f}, quaternion=({qx:.4f}, {qy:.4f}, {qz:.4f}, {qw:.4f})"
                    )
        except Exception as e:
            self.get_logger().error(f"发布四元数坐标时出错: {str(e)}")


def main():
    """主函数"""
    try:
        # 初始化ROS2
        rclpy.init(args=None)
        
        # 创建并运行节点
        node = TransformedSandboxCoordinatePublisher()
        rclpy.spin(node)
    except Exception as e:
        print(f"运行时出错: {str(e)}")
    finally:
        # 清理资源
        rclpy.shutdown()


if __name__ == "__main__":
    main()