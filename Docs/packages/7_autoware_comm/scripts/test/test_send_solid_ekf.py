#!/usr/bin/env python

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseWithCovariance, Pose, Point, Quaternion
from std_msgs.msg import Header
import math

class FixedPosePublisherNode(Node):
    def __init__(self, update_rate=1):
        super().__init__("fixed_pose_publisher_node")

        # 创建发布器，发送 PoseWithCovarianceStamped 消息
        self.publisher = self.create_publisher(
            PoseWithCovarianceStamped, "/real_world/follow_adtruck/transformed_with_covariance", 1
        )
        
        # 创建定时器，以 0.02 秒的间隔发布消息
        self.timer = self.create_timer(update_rate, self.publish_fixed_pose)

        # 固定的位姿，可以根据你的需求调整
        self.fixed_position = [0.0, 0.0, 0.0016]  # 固定的位置 (x, y, z)
        self.fixed_yaw = math.radians(45)  # 固定的 yaw 角 (以弧度表示)
        self.fixed_quaternion = self.RPY2quaternion(0, 0, self.fixed_yaw)  # 由 yaw 转换为四元数

    def RPY2quaternion(self, roll, pitch, yaw):
        """
        将欧拉角 (roll, pitch, yaw) 转换为四元数 (x, y, z, w)
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

    def publish_fixed_pose(self):
        """
        发布固定的位姿（位置和方向）
        """
        # 创建消息头
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = "real_world"

        # 创建位姿 (Pose) 消息
        pose = Pose()
        pose.position = Point(x=self.fixed_position[0], y=self.fixed_position[1], z=self.fixed_position[2])
        pose.orientation = Quaternion(
            x=self.fixed_quaternion[0],
            y=self.fixed_quaternion[1],
            z=self.fixed_quaternion[2],
            w=self.fixed_quaternion[3]
        )

        # 创建协方差矩阵（6x6矩阵展开成36个元素）
        covariance = [0.0] * 36
        covariance[0] = 0.01  # x 轴上的方差
        covariance[7] = 0.01  # y 轴上的方差
        covariance[14] = 0.01 # z 轴上的方差
        covariance[21] = 0.01 # roll 轴上的方差
        covariance[28] = 0.01 # pitch 轴上的方差
        covariance[35] = 0.01 # yaw 轴上的方差

        # 创建 PoseWithCovariance 对象
        pose_with_cov = PoseWithCovariance(pose=pose, covariance=covariance)

        # 创建 PoseWithCovarianceStamped 消息
        msg = PoseWithCovarianceStamped(header=header, pose=pose_with_cov)

        # 发布消息
        self.publisher.publish(msg)


        # 打印日志
        self.get_logger().info(
            f"Published fixed pose: x={msg.pose.pose.position.x}, y={msg.pose.pose.position.y}, z={msg.pose.pose.position.z}, yaw={math.degrees(self.fixed_yaw)} degrees"
        )

def main(args=None):
    rclpy.init(args=args)

    # 创建节点
    fixed_pose_publisher_node = FixedPosePublisherNode()

    # 让节点运行
    rclpy.spin(fixed_pose_publisher_node)

    # 关闭节点
    rclpy.shutdown()

if __name__ == "__main__":
    main()