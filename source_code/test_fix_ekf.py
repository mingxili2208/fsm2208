#!/usr/bin/env python

import os
os.environ['PYGAME_HIDE_SUPPORT_PROMPT'] = "hide"

import sys
sys.path.append("/home/jarvislee-carla/WorkplaceCarlas/op_carla/op_bridge/op_bridge/fsm_lab_simulation")
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
from matplotlib.path import Path
from shapely.geometry import Polygon 
from shapely.geometry import Point as PPoint

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


    def calculate_B_position(self, A_position, A_yaw, transition_width=0.4):
        """
        根据 A 在 C 中的位置和 yaw 角，计算 B 在 C 中的位置，并通过插值平滑局部额外补偿。
        :param A_position: A 在 C 中的坐标 (x_A, y_A, z_A)
        :param A_yaw: A 在 C 中的 yaw 角（弧度制）
        :param transition_width: 过渡带宽度，用于平滑过渡的范围大小
        :return: B 在 C 中的坐标 (x_B, y_B, z_B)
        """
        x = A_position[0]
        y = A_position[1]
        yaw = A_yaw

        # 全局基础补偿
        global_offset_A_to_B = np.array([-0.0225, 0, 0])

        # 区域补偿定义
        region1_offset_A_to_B = np.array([-0.085, 0, 0])  # 区域 1 补偿
        region2_offset_A_to_B = np.array([-0.0185, 0, 0])  # 区域 2 补偿
        region3_offset_A_to_B = np.array([-0.125, 0, 0])  # 区域 3 补偿
        region4_offset_A_to_B = np.array([+0.00, 0, 0])  # 区域 4 补偿

        # 定义局部区域的坐标范围
        local_region_x_min = -np.inf
        local_region_x_max = 0.66
        local_region_y_min = -3.09
        local_region_y_max = -0.3

        # 区域 3 和 区域 4 多边形定义
        polygon_region3 = Polygon([
            #[3.942,-1.983,]
            #[3.993,-1.557]
            (3.8408, -2.2162),  # A
            (3.5942, -2.1935),  # C 
            (3.1053, -1.7821),   # D
            (3.8339, -1.4712)  # B
        ])
        polygon_region4 = Polygon([
            (1.1455, -1.0390),
            (0.8830, -0.8656),
            (0.8239, -2.0134),
            (1.3043, -2.5719),
            (1.7373, -2.7863),
            (2.4617, -2.9485),
            (3.0875, -2.8259),
            (3.2439, -2.1172),
            (2.7218, -2.5038),
            (1.9899, -2.1543),
            (1.6561, -2.0650),
            (1.6256, -1.6862),
            (1.4803, -1.0702)
        ])

        # 扩展多边形区域（创建过渡带）
        expanded_polygon_region3 = polygon_region3.buffer(transition_width)
        expanded_polygon_region4 = polygon_region4.buffer(transition_width)

        # 判断点是否在局部区域
        def is_in_local_region(x, y):
            """
            检查点是否在局部区域范围内
            """
            return (local_region_x_min <= x <= local_region_x_max) and \
                (local_region_y_min <= y <= local_region_y_max)

        # 计算过渡权重的函数
        def calculate_transition_weight(value, min_value, max_value, transition_width):
            """
            计算在过渡带内的权重。返回值在 [0, 1] 之间，0 表示没有局部补偿，1 表示完全应用局部补偿。
            :param value: 当前值
            :param min_value: 区域的最小边界
            :param max_value: 区域的最大边界
            :param transition_width: 过渡带宽度
            """
            if value < min_value:
                return 0.0
            elif value > max_value:
                return 0.0
            elif min_value <= value <= (min_value + transition_width):
                return (value - min_value) / transition_width
            elif (max_value - transition_width) <= value <= max_value:
                return (max_value - value) / transition_width
            else:
                return 1.0

        # 计算点到多边形边界的距离
        def distance_to_polygon_boundary(point, polygon):
            """
            计算点到多边形边界的距离
            :param point: (x, y) 坐标
            :param polygon: Shapely Polygon 对象
            :return: 距离值
            """
            return polygon.exterior.distance(PPoint(point))

        # 分别计算 x 和 y 的过渡权重
        x_weight = calculate_transition_weight(x, local_region_x_min, local_region_x_max, transition_width)
        y_weight = calculate_transition_weight(y, local_region_y_min, local_region_y_max, transition_width)

        # 区域判断逻辑
        if is_in_local_region(x, y) and -93 <= np.degrees(yaw) <= 88:
            # 区域 1
            yaw_weight = calculate_transition_weight(np.degrees(yaw), -93, 88, transition_width)
            region_weight = x_weight * y_weight * yaw_weight
            region_offset_A_to_B = region1_offset_A_to_B
            self.get_logger().info(f"区域 1: 权重为 {region_weight}, 偏移为 {region_offset_A_to_B}")
        elif is_in_local_region(x, y) and ((88 <= np.degrees(yaw) <= 180) or (-180 <= np.degrees(yaw) <= -93)):
            # 区域 2
            yaw_weight = calculate_transition_weight(np.degrees(yaw), -180, -93, transition_width) + \
                        calculate_transition_weight(np.degrees(yaw), 88, 180, transition_width)
            region_weight = x_weight * y_weight * yaw_weight
            region_offset_A_to_B = region2_offset_A_to_B
            self.get_logger().info(f"区域 2: 权重为 {region_weight}, 偏移为 {region_offset_A_to_B}")
        elif polygon_region3.contains(PPoint(x, y)) and -160 <= np.degrees(yaw) <= 8:
            # 区域 3
            region_weight = 1.0
            region_offset_A_to_B = region3_offset_A_to_B
            self.get_logger().info(f"区域 3: 点在原始多边形内部，权重为 {region_weight},偏移为 {region_offset_A_to_B}")
        elif expanded_polygon_region3.contains(PPoint(x, y)) and -160 <= np.degrees(yaw) <= 8 :
            # 区域 3 过渡带
            distance = distance_to_polygon_boundary((x, y), polygon_region3)
            region_weight = max(0.0, min(1.0, 1 - distance / transition_width))
            region_offset_A_to_B = region3_offset_A_to_B
            self.get_logger().info(f"区域 3 过渡带: 距离原始边界 {distance:.3f}, 权重为 {region_weight},偏移为 {region_offset_A_to_B}")
        elif polygon_region4.contains(PPoint(x, y)):
            # 区域 4
            region_weight = 1.0
            region_offset_A_to_B = region4_offset_A_to_B
            self.get_logger().info(f"区域 4: 点在原始多边形内部，权重为 {region_weight},偏移为 {region_offset_A_to_B}")
        elif expanded_polygon_region4.contains(PPoint(x, y)):
            # 区域 4 过渡带
            distance = distance_to_polygon_boundary((x, y), polygon_region4)
            region_weight = max(0.0, min(1.0, 1 - distance / transition_width))
            region_offset_A_to_B = region4_offset_A_to_B
            self.get_logger().info(f"区域 4 过渡带: 距离原始边界 {distance:.3f}, 权重为 {region_weight},偏移为 {region_offset_A_to_B}")
        else:
            # 全局区域
            region_weight = 0.0
            region_offset_A_to_B = global_offset_A_to_B
            self.get_logger().info(f"全局区域: 点不在任何区域内，权重为 {region_weight},偏移为 {region_offset_A_to_B}")

        # 最终补偿融合
        final_offset_A_to_B = (1 - region_weight) * global_offset_A_to_B + region_weight * region_offset_A_to_B

        # 旋转矩阵 (绕 Z 轴旋转 yaw 角)
        rotation_matrix = np.array([
            [np.cos(A_yaw), -np.sin(A_yaw), 0],
            [np.sin(A_yaw),  np.cos(A_yaw), 0],
            [0, 0, 1]
        ])

        # 计算 A 相对于 B 的偏移
        offset_in_C = np.dot(rotation_matrix, final_offset_A_to_B)

        # 计算 B 在 C 中的坐标
        B_position = A_position - offset_in_C
        return B_position

    def transform_coordinates_from_sandbox2carla(self,transformed_position__,transformed_yaw__):
        
        #pose.position.z=0.0016
        self.get_logger().info(f"this is orginal pose: x: {transformed_position__[0]}, y: {transformed_position__[2]}, z: 0.0016,yaw: {math.degrees(transformed_yaw__)},")
        #x=transformed_position[0], y=transformed_position[2], z=transformed_position[1]
        # 注意: Carla的y轴和ROS的y轴方向相反, 因此需要对y轴进行翻转
        B_position_offseted=self.calculate_B_position(np.array([transformed_position__[0],transformed_position__[2],0.0016]),transformed_yaw__)

        original_point=np.array([B_position_offseted[0], B_position_offseted[1], B_position_offseted[2]])
        #original_point=np.array([transformed_position__[0], transformed_position__[2], 0.0016])

        homogeneous_point = np.append(original_point, 1)
        transformed_homogeneous_point = self.similarity_matrix @ homogeneous_point
        transformed_point = transformed_homogeneous_point[:3] / transformed_homogeneous_point[3]

        #yaw=transformed_yaw
        _transformed_yaw=math.radians(-(math.degrees(transformed_yaw__)-90))#-90#90-transformed_yaw__#+15#-90
        transformed_point[1]=-transformed_point[1]
        self.get_logger().info(f"____transformed___pose x: {transformed_point[0]}, y: {transformed_point[1]}, z:{transformed_point[2]},yaw: {math.degrees(_transformed_yaw)},")
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
        transformed_position, transformed_yaw=self.transform_coordinates_from_sandbox2carla(transformed_position_, transformed_yaw_)
        # Set up the PoseWithCovarianceStamped message
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
        # self.get_logger().info(
        #     f"\n Transformed Published! x is {transformed_position[0]}, y is {transformed_position[1]}, z is {transformed_position[2]}, yaw is {math.degrees(transformed_yaw)}"
        # )
        # self.get_logger().info(
        #     f"\n Transformed Published with covariance! [{msg.pose.pose.position.x}, {msg.pose.pose.position.y}, {msg.pose.pose.position.z}, {msg.pose.pose.orientation.w}, {msg.pose.pose.orientation.x}, {msg.pose.pose.orientation.y}, {msg.pose.pose.orientation.z}]"
        # )

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