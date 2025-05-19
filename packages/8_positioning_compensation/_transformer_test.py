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
import datetime
import logging
import setproctitle
import time

setproctitle.setproctitle('python_update_coordinate')
VEHICLE_NAME = "follow_adtruck"
# Create log directory in the current working directory
LOG_DIR = os.path.join(os.getcwd(), "coordinate_logs")
os.makedirs(LOG_DIR, exist_ok=True)

class TransformedSandBoxCoorPublisherNode(Node):
    def __init__(self, update_rate=0.02):
        super().__init__("transformed_SandBox_coordinate_publisher_node")
        self.publisher = self.create_publisher(
            PoseWithCovarianceStamped, f"/real_world/{VEHICLE_NAME}/transformed_with_covariance", 1
        )

        # Set up logging
        timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        log_file = os.path.join(LOG_DIR, f"coordinate_transform_{timestamp}.log")
        
        self.logger = logging.getLogger("coordinate_transform")
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
        self.get_logger().info(f"Coordinate logs will be saved to: {log_file}")

        self.sandbox_transformer = VR2SandBoxTransformer()
        self.previous_position = None
        self.previous_yaw = None
        self.current_position = None
        self.current_yaw = None
        self.log_flag = False
        
        # 记录上一次发布位姿的时间
        self.last_published_time = self.get_clock().now()
        self.similarity_matrix = np.array([[ 3.27650037e+01, -9.68207899e-01,  0.00000000e+00, -5.41650474e+01],
                             [ 9.68207899e-01,  3.27650037e+01,  0.00000000e+00,  7.28873065e+01],
                             [ 0.00000000e+00,  0.00000000e+00,  3.27793059e+01, -5.08468894e-02],
                             [ 0.00000000e+00,  0.00000000e+00,  0.00000000e+00,  1.00000000e+00]])
        
        # 注意这里不要使用括号，只传递函数引用
        self.timer = self.create_timer(update_rate, self.update_position_yaw)
        #self.timeout_checker_timer = self.create_timer(0.02, self.check_timeout)  # 每隔0.1秒检查超时

    def update_position_yaw(self):
        
        self.current_position, self.current_yaw = self.sandbox_transformer.get_transformed_coor()
        if self.check_threshold(self.current_position, self.current_yaw):
            self.get_logger().info(
                f"\n current new sandbox pos is x : {self.current_position[0]}, y : {self.current_position[1]}, z : {self.current_position[2]}, yaw is {math.degrees(self.current_yaw)}"
            )  
        self.publish_transformed_coor()


    def check_threshold(self, position, yaw, threshold=0.005):
        
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
        yaw_changed = abs(yaw - self.previous_yaw) > np.radians(0.486)  # 约0.286度

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
        """
        根据 A 在 C 中的位置和 yaw 角，计算 B 在 C 中的位置。
        :param A_position: A 在 C 中的坐标 (x_A, y_A, z_A)
        :param A_yaw: A 在 C 中的 yaw 角（弧度制）
        :return: B 在 C 中的坐标 (x_B, y_B, z_B)
        """

        #offset_A_to_B = np.array([-0.0605, 0, 0])
        offset_A_to_B = np.array([-0.005, 0, 0])
        #offset_A_to_B = np.array([-0.0825, 0, 0])

        # 旋转矩阵 (绕 Z 轴旋转 yaw 角)
        rotation_matrix = np.array([
            [np.cos(A_yaw), -np.sin(A_yaw), 0],
            [np.sin(A_yaw),  np.cos(A_yaw), 0],
            [0, 0, 1]
        ])

        # 计算在 C 中 A 相对于 B 的偏移
        offset_in_C = np.dot(rotation_matrix, offset_A_to_B)

        # 计算 B 在 C 中的坐标
        B_position = A_position - offset_in_C
        return B_position
        
    def transform_coordinates_from_sandbox2carla(self, transformed_position__, transformed_yaw__):
        
        #pose.position.z=0.0016
        #self.get_logger().info(f"this is orginal pose: x: {transformed_position__[0]}, y: {transformed_position__[2]}, z: 0.0016,yaw: {math.degrees(transformed_yaw__)},")
        #x=transformed_position[0], y=transformed_position[2], z=transformed_position[1]
        # 注意: Carla的y轴和ROS的y轴方向相反, 因此需要对y轴进行翻转
        B_position_offseted = self.calculate_B_position(np.array([transformed_position__[0], transformed_position__[2], 0.0016]), transformed_yaw__)

        original_point = np.array([B_position_offseted[0], B_position_offseted[1], B_position_offseted[2]])
        #original_point=np.array([transformed_position__[0], transformed_position__[2], 0.0016])

        homogeneous_point = np.append(original_point, 1)
        transformed_homogeneous_point = self.similarity_matrix @ homogeneous_point
        transformed_point = transformed_homogeneous_point[:3] / transformed_homogeneous_point[3]

        #yaw=transformed_yaw
        _transformed_yaw = math.radians(-(math.degrees(transformed_yaw__)-90))  #-90#90-transformed_yaw__#+15#-90
        transformed_point[1] = -transformed_point[1]
        #self.get_logger().info(f"____transformed___pose x: {transformed_point[0]}, y: {transformed_point[1]}, z:{transformed_point[2]},yaw: {math.degrees(_transformed_yaw)},")
        #new_pose=carla.Transform(carla.Location(x, y, z), carla.Rotation(0, yaw,0 ))
        return transformed_point, _transformed_yaw    

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
    
    def publish_transformed_coor(self):
        
        if self.current_position is not None and self.current_yaw is not None:
            transformed_position, transformed_yaw = self.transform_coordinates_from_sandbox2carla(self.current_position, self.current_yaw)
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
            #time.sleep(0.5)
            self.publisher.publish(msg)
            
            if self.log_flag:
                # Log to console
                self.get_logger().info(
                    f"\n Transformed Published! x is {transformed_position[0]}, y is {transformed_position[1]}, z is {transformed_position[2]}, yaw is {math.degrees(transformed_yaw)}"
                )
                
                # Log to file with both original and transformed coordinates
                self.logger.info(
                    f"{datetime.datetime.now().isoformat()}, "
                    f"{self.current_position[0]}, {self.current_position[1]}, {self.current_position[2]}, {math.degrees(self.current_yaw)}, "
                    f"{transformed_position[0]}, {transformed_position[1]}, {transformed_position[2]}, {math.degrees(transformed_yaw)}"
                )
                
            # self.get_logger().info(
            #     f"\n Transformed Published with covariance! [{msg.pose.pose.position.x}, {msg.pose.pose.position.y}, {msg.pose.pose.position.z}, {msg.pose.pose.orientation.w}, {msg.pose.pose.orientation.x}, {msg.pose.pose.orientation.y}, {msg.pose.pose.orientation.z}]"
            # )

            # 更新最后一次发布的时间
            self.last_published_time = self.get_clock().now()
        else:
            self.get_logger().error("!!! ERROR !!! current pos is not updated !!! ")


if __name__ == "__main__":
    rclpy.init(args=None)

    # Create the node
    transformed_sandbox_coor_publisher_node = TransformedSandBoxCoorPublisherNode()

    # Spin the node
    rclpy.spin(transformed_sandbox_coor_publisher_node)

    rclpy.shutdown()