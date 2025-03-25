#!/usr/bin/env python

import rclpy
import threading
import time
import math
import sys
import select
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from autoware_auto_vehicle_msgs.msg import Engage
from geometry_msgs.msg import PoseWithCovarianceStamped

class MissionControlNode(Node):
    def __init__(self):
        super().__init__('mission_control_node')

        # 目标位姿列表
        self.waypoints = [
            self.create_pose_stamped(49.233, -51.3687, 0, 0, 0, 0.289784, 0.957092),  # 目标点1
            self.create_pose_stamped(-47.2839, 27.3298, 0, 0, 0, -0.707107, 0.707107), # 目标点2
            self.create_pose_stamped(-48.9462, -44.6544, 0, 0, 0, -0.740755, 0.671775) # 目标点3
        ]
        self.current_waypoint_index = 0  # 当前目标点索引
        self.goal_reached = False  # 是否到达目标点
        self.running = False  # 是否正在执行任务

        # 目标位姿发布器
        self.goal_publisher = self.create_publisher(PoseStamped, "/planning/mission_planning/goal", 10)

        # Engage 控制发布器
        self.engage_publisher = self.create_publisher(Engage, "/autoware/engage", 10)

        # 订阅车辆位姿
        self.subscription = self.create_subscription(
            PoseWithCovarianceStamped,
            "/real_world/follow_adtruck/transformed_with_covariance",
            self.pose_callback,
            10
        )

        # 目标点更新的定时器（防止阻塞主线程）
        self.timer = self.create_timer(0.5, self.check_goal_reached)

        # 监听键盘输入（单独线程）
        self.keyboard_thread = threading.Thread(target=self.keyboard_listener, daemon=True)
        self.keyboard_thread.start()

        self.get_logger().info("MissionControlNode 初始化完成，等待键盘输入 'e' 以启动")

    def create_pose_stamped(self, x, y, z, ox, oy, oz, ow):
        """ 创建 PoseStamped 消息 """
        msg = PoseStamped()
        msg.header.frame_id = "map"
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = z
        msg.pose.orientation.x = ox
        msg.pose.orientation.y = oy
        msg.pose.orientation.z = oz
        msg.pose.orientation.w = ow
        return msg

    def keyboard_listener(self):
        """ 监听键盘输入，按下 'e' 触发任务 """
        self.get_logger().info("等待键盘输入 'e' 以启动任务")
        while rclpy.ok():
            rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
            if rlist:
                key = sys.stdin.read(1)
                if key.lower() == 'e':
                    self.get_logger().info("收到 'e' 键输入，启动任务")
                    self.start_mission()

    def start_mission(self):
        """ 启动任务，按顺序发布目标点 """
        if self.running:
            self.get_logger().warn("任务已在运行中，忽略重复启动")
            return

        self.running = True
        self.current_waypoint_index = 0
        self.publish_next_goal()

    def publish_next_goal(self):
        """ 发布下一个目标点 """
        if self.current_waypoint_index < len(self.waypoints):
            goal_msg = self.waypoints[self.current_waypoint_index]
            goal_msg.header.stamp = self.get_clock().now().to_msg()
            self.goal_publisher.publish(goal_msg)
            self.get_logger().info(f"发布目标点 {self.current_waypoint_index + 1}: {goal_msg.pose.position.x}, {goal_msg.pose.position.y}")

            # 5 秒后启动车辆
            self.create_timer(5.0, self.publish_engage_true)
        else:
            self.get_logger().info("所有目标点已执行完毕，等待新的键盘输入")

    def publish_engage_true(self):
        """ 发送 engage=True 指令 """
        engage_msg = Engage()
        engage_msg.engage = True
        self.engage_publisher.publish(engage_msg)
        self.get_logger().info("发送 engage=True，启动车辆")

    def publish_engage_false(self):
        """ 发送 engage=False 指令 """
        engage_msg = Engage()
        engage_msg.engage = False
        self.engage_publisher.publish(engage_msg)
        self.get_logger().info("发送 engage=False，停止车辆")

    def pose_callback(self, msg):
        """ 订阅车辆位姿，并检查是否到达目标点 """
        if self.running and self.current_waypoint_index < len(self.waypoints):
            current_pose = msg.pose.pose.position
            target_pose = self.waypoints[self.current_waypoint_index].pose.position

            distance = math.sqrt(
                (current_pose.x - target_pose.x) ** 2 +
                (current_pose.y - target_pose.y) ** 2
            )

            self.get_logger().info(f"当前距离目标点 {self.current_waypoint_index + 1}: {distance:.4f} 米")

            if distance < 0.05:
                self.goal_reached = True

    def check_goal_reached(self):
        """ 检查是否到达目标点，等待 10s 后继续任务 """
        if self.goal_reached:
            self.goal_reached = False
            self.get_logger().info(f"到达目标点 {self.current_waypoint_index + 1}，等待 10s")

            # 停止车辆
            self.publish_engage_false()

            # 10s 后发布下一个目标点
            self.create_timer(10.0, self.handle_next_goal)

    def handle_next_goal(self):
        """ 处理下一个目标点 """
        self.current_waypoint_index += 1
        if self.current_waypoint_index < len(self.waypoints):
            self.get_logger().info(f"5s 后发布下一个目标点 {self.current_waypoint_index + 1}")
            self.create_timer(5.0, self.publish_next_goal)
        else:
            self.running = False
            self.get_logger().info("所有目标点已执行完毕，等待新的键盘输入")

def main(args=None):
    rclpy.init(args=args)
    node = MissionControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()