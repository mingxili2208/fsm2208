#!/usr/bin/env python

import os
os.environ['PYGAME_HIDE_SUPPORT_PROMPT'] = "hide"

import sys
sys.path.append("/home/jarvislee-carla/Workspace/Carlas/op_carla/op_bridge/op_bridge/fsm_lab_simulation")
import rclpy
import math
from transforms3d.euler import euler2mat, quat2euler, euler2quat
from vr2sx_transformer import VR2SandBoxTransformer
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseWithCovariance, Pose, Point, Quaternion
from std_msgs.msg import Header
from rclpy.node import Node 
import numpy as np
from rclpy.time import Time
import carla

VEHICLE_NAME = "follow_adtruck"

class TransformedSandBoxCoorPublisherNode(Node):
    def __init__(self, update_rate=0.02):
        super().__init__("transformed_SandBox_coordinate_publisher_node")
        self.publisher = self.create_publisher(
            PoseWithCovarianceStamped, f"/real_world/{VEHICLE_NAME}/transformed_with_covariance", 1
        )
        self.timer = self.create_timer(update_rate, self.publish_transformed_coor)
        #self.timeout_checker_timer = self.create_timer(0.02, self.check_timeout)  # 每隔0.1秒检查超时

        self.sandbox_transformer = VR2SandBoxTransformer()
        self.previous_position = None
        self.previous_yaw = None
        
        # 记录上一次发布位姿的时间
        self.last_published_time = self.get_clock().now()
        self.similarity_matrix = np.array([[ 3.27650037e+01, -9.68207899e-01,  0.00000000e+00, -5.41650474e+01],
                             [ 9.68207899e-01,  3.27650037e+01,  0.00000000e+00,  7.28873065e+01],
                             [ 0.00000000e+00,  0.00000000e+00,  3.27793059e+01, -5.08468894e-02],
                             [ 0.00000000e+00,  0.00000000e+00,  0.00000000e+00,  1.00000000e+00]])
    
    def transform_coordinates_from_sandbox2carla(self,transformed_position__,transformed_yaw__):
        
        #pose.position.z=0.0016
        #x=transformed_position[0], y=transformed_position[2], z=transformed_position[1]
        original_point=np.array([transformed_position__[0], transformed_position__[2], 0.0016])
        homogeneous_point = np.append(original_point, 1)
        transformed_homogeneous_point = self.similarity_matrix @ homogeneous_point
        transformed_point = transformed_homogeneous_point[:3] / transformed_homogeneous_point[3]

        #yaw=transformed_yaw
        _transformed_yaw=math.radians(-(math.degrees(transformed_yaw__)-90))#-90#90-transformed_yaw__#+15#-90
        self.get_logger().info(f"-------------------x: {transformed_point[0]}, y: {transformed_point[1]}, z:{transformed_point[2]},yaw: {_transformed_yaw},")
        #new_pose=carla.Transform(carla.Location(x, y, z), carla.Rotation(0, yaw,0 ))
        return transformed_point,_transformed_yaw    

    def RPY2quaternion(self, roll, pitch, yaw):
        """
        Transform Euler angles to quaternion.
        Parameters:
            roll, pitch, yaw
        Return: (x, y, z, w)
        """
        # roll = math.radians(roll)
        # pitch = math.radians(pitch)
        # yaw = math.radians(yaw)

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
    
    def publish_transformed_coor_(self):

        transformed_position_, transformed_yaw_ = self.sandbox_transformer.get_transformed_coor()
        
        if self.previous_position is None or self.previous_yaw is None:
            self.previous_position = transformed_position_
            self.previous_yaw = transformed_yaw_

        transformed_position, transformed_yaw=self.transform_coordinates_from_sandbox2carla(transformed_position_, transformed_yaw_)
        # Set up the PoseWithCovarianceStamped message
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = "real_world"

        pose = Pose()
        # 注意: Carla的y轴和ROS的y轴方向相反, 因此需要对y轴进行翻转
        pose.position = Point(x=transformed_position[0], y=-transformed_position[1], z=transformed_position[2])
        pose.orientation = Quaternion(
            x=self.RPY2quaternion(0, 0, transformed_yaw)[0],
            y=self.RPY2quaternion(0, 0, transformed_yaw)[1],
            z=self.RPY2quaternion(0, 0, transformed_yaw)[2],
            w=self.RPY2quaternion(0, 0, transformed_yaw)[3]
        )

        # Create a covariance matrix for pose (6x6 matrix flattened to 36 elements)
        covariance = [0.0] * 36
        covariance[0] = 0.01  # variance in x
        covariance[7] = 0.01  # variance in y
        covariance[14] = 0.01 # variance in z
        covariance[21] = 0.01 # variance in roll
        covariance[28] = 0.01 # variance in pitch
        covariance[35] = 0.01 # variance in yaw

        # Create PoseWithCovariance object
        pose_with_cov = PoseWithCovariance(pose=pose, covariance=covariance)

        # Create PoseWithCovarianceStamped message
        msg = PoseWithCovarianceStamped(header=header, pose=pose_with_cov)

        # Publish the message
        self.publisher.publish(msg)
        self.get_logger().info(
            f"\n this is coor_ Transformed Published! x is {transformed_position[0]}, y is {transformed_position[1]}, z is {transformed_position[2]}, yaw is {transformed_yaw}"
        )
        self.get_logger().info(
            f"\n Transformed Published with covariance! [{msg.pose.pose.position.x}, {msg.pose.pose.position.y}, {msg.pose.pose.position.z}, {msg.pose.pose.orientation.w}, {msg.pose.pose.orientation.x}, {msg.pose.pose.orientation.y}, {msg.pose.pose.orientation.z}]"
        )

        # 更新最后一次发布的时间
        self.last_published_time = self.get_clock().now()


    def create_transformation_matrix(self,location, yaw_deg):
        """
        创建一个齐次变换矩阵，仅包含yaw旋转。
        
        :param location: 三元组 (x, y, z)
        :param yaw_deg: yaw角度（度）
        :return: 4x4 齐次变换矩阵
        """
        yaw_rad = math.radians(yaw_deg)
        cos_yaw = np.cos(yaw_rad)
        sin_yaw = np.sin(yaw_rad)
        
        T = np.array([
            [cos_yaw, -sin_yaw, 0, location[0]],
            [sin_yaw,  cos_yaw, 0, location[1]],
            [0,        0,       1, location[2]],
            [0,        0,       0, 1]
        ])
        return T

    def invert_transformation_matrix(self,T):
        """
        计算齐次变换矩阵的逆。
        
        :param T: 4x4 齐次变换矩阵
        :return: 4x4 逆齐次变换矩阵
        """
        R = T[0:3, 0:3]
        p = T[0:3, 3]
        R_inv = R.T
        p_inv = -R_inv @ p
        T_inv = np.identity(4)
        T_inv[0:3, 0:3] = R_inv
        T_inv[0:3, 3] = p_inv
        return T_inv

    def extract_pose(self,T):
        """
        从齐次变换矩阵中提取位置和yaw角。
        
        :param T: 4x4 齐次变换矩阵
        :return: (位置, yaw角) 位置为三元组，yaw角为度
        """
        x, y, z = T[0:3, 3]
        yaw_rad = np.arctan2(T[1,0], T[0,0])
        yaw_deg = np.degrees(yaw_rad)
        return (x, y, z), yaw_deg

    def compute_B_pose_in_C(self,location_A, yaw_A):
        """
        计算框架B在框架C中的位置和yaw角。
        
        :param location_A: 框架A在框架C中的位置 (x, y, z)
        :param yaw_A: 框架A在框架C中的yaw角度（度）
        :return: 框架B在框架C中的位置和yaw角
        """
        # 定义 T_A->C
        T_A_in_C = self.create_transformation_matrix(location_A, yaw_A)
        
        # 定义 T_A->B（固定）
        p_A_in_B = (-0.05, 0, 0)  # A在B中的位置
        yaw_A_in_B = 0.0         # A相对于B的yaw角（无旋转）
        T_A_in_B = self.create_transformation_matrix(p_A_in_B, yaw_A_in_B)
        
        # 计算 T_A->B 的逆矩阵
        T_A_in_B_inv = self.invert_transformation_matrix(T_A_in_B)
        
        # 计算 T_B->C
        T_B_in_C = T_A_in_C @ T_A_in_B_inv
        
        # 提取位置和yaw角
        location_B, yaw_B = self.extract_pose(T_B_in_C)
        
        return location_B, yaw_B

    def publish_transformed_coor(self):
        
        transformed_position_, transformed_yaw_ = self.sandbox_transformer.get_transformed_coor()
        
        #transformed_position_[1]-=0.05
        #if self.previous_position is None or self.previous_yaw is None:
        self.previous_position = transformed_position_
        self.previous_yaw = transformed_yaw_
        #else:
            # if abs(transformed_position_[0] - self.previous_position[0]) > 0.005 or \
            #    abs(transformed_position_[2] - self.previous_position[2]) > 0.005 or \
            #    abs(np.degrees(self.previous_yaw) - np.degrees(transformed_yaw_)) > 1:
            #     self.get_logger().info(
            #         f"\n previous_position is {self.previous_position};    the abs in x is {abs(transformed_position_[0] - self.previous_position[0])};  the abs in y is {abs(transformed_position_[2] - self.previous_position[2])} the abs in yaw is {abs(np.degrees(self.previous_yaw) - np.degrees(transformed_yaw_))}")
            #     self.previous_position = transformed_position_
            #     self.previous_yaw = transformed_yaw_
            # else:
            #     return
        location_B_time1, yaw_B_time1 = self.compute_B_pose_in_C(location_A_time1, yaw_A_time1)
        location_B_time2, yaw_B_time2 = self.compute_B_pose_in_C(location_A_time2, yaw_A_time2)
        transformed_position, transformed_yaw=self.transform_coordinates_from_sandbox2carla(transformed_position_, transformed_yaw_)
        # Set up the PoseWithCovarianceStamped message
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = "real_world"

        pose = Pose()
        # 注意: Carla的y轴和ROS的y轴方向相反, 因此需要对y轴进行翻转
        pose.position = Point(x=transformed_position[0], y=-transformed_position[1], z=transformed_position[2])
        pose.orientation = Quaternion(
            x=self.RPY2quaternion(0, 0, transformed_yaw)[0],
            y=self.RPY2quaternion(0, 0, transformed_yaw)[1],
            z=self.RPY2quaternion(0, 0, transformed_yaw)[2],
            w=self.RPY2quaternion(0, 0, transformed_yaw)[3]
        )

        # Create a covariance matrix for pose (6x6 matrix flattened to 36 elements)
        covariance = [0.0] * 36
        covariance[0] = 0.01  # variance in x
        covariance[7] = 0.01  # variance in y
        covariance[14] = 0.01 # variance in z
        covariance[21] = 0.01 # variance in roll
        covariance[28] = 0.01 # variance in pitch
        covariance[35] = 0.01 # variance in yaw

        # Create PoseWithCovariance object
        pose_with_cov = PoseWithCovariance(pose=pose, covariance=covariance)

        # Create PoseWithCovarianceStamped message
        msg = PoseWithCovarianceStamped(header=header, pose=pose_with_cov)

        # Publish the message
        self.publisher.publish(msg)
        self.get_logger().info(
            f"\n Transformed Published! x is {transformed_position[0]}, y is {transformed_position[1]}, z is {transformed_position[2]}, yaw is {transformed_yaw}"
        )
        self.get_logger().info(
            f"\n Transformed Published with covariance! [{msg.pose.pose.position.x}, {msg.pose.pose.position.y}, {msg.pose.pose.position.z}, {msg.pose.pose.orientation.w}, {msg.pose.pose.orientation.x}, {msg.pose.pose.orientation.y}, {msg.pose.pose.orientation.z}]"
        )

        # 更新最后一次发布的时间
        self.last_published_time = self.get_clock().now()

    def check_timeout(self):
        """
        检查是否超过5秒没有发布位姿，如果超过则强制发布一次
        """
        current_time = self.get_clock().now()
        time_since_last_publish = current_time - self.last_published_time

        # 如果超过0.02秒没有发布，则强制发布一次位姿
        if time_since_last_publish.nanoseconds > 0.02 * 1e9:  # 5秒的阈值
            self.get_logger().warn("5 seconds passed without publishing, forcing a publish.")
            self.publish_transformed_coor_()
            #self.last_published_time = self.get_clock().now()


if __name__ == "__main__":
    rclpy.init(args=None)

    # Create the node
    transformed_sandbox_coor_publisher_node= TransformedSandBoxCoorPublisherNode()

    # Spin the node
    rclpy.spin(transformed_sandbox_coor_publisher_node)

    rclpy.shutdown()