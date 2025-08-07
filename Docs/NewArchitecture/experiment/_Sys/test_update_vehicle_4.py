#!/usr/bin/env python

# used to debug with the coordinator of carla
import os
import cv2
import sys
import time
import signal
import numpy as np
import random
import importlib
import traceback
import math
import carla
import rclpy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from transforms3d.euler import euler2mat, quat2euler, euler2quat
from leaderboard.envs.sensor_interface import SensorReceivedNoData
from leaderboard.autoagents.agent_wrapper import AgentWrapper, AgentError
from srunner.scenariomanager.timer import GameTime
from srunner.scenariomanager.watchdog import Watchdog
from srunner.scenariomanager.carla_data_provider import CarlaDataProvider
from geometry_msgs.msg import PoseStamped, TwistWithCovariance, PoseWithCovariance, TwistStamped, PoseWithCovarianceStamped
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseWithCovariance, Pose, Point, Quaternion
from rclpy.node import Node 
import setproctitle
from sensor_msgs.msg import TimeReference

setproctitle.setproctitle('python_update_vehicle')
class BridgeHelpers(object):
    @staticmethod
    def get_agent_actor(world, role_name):
        actors = world.get_actors().filter("vehicle.*")
        if actors is not None:
            for car in actors:
                print(f"there is a car: {car}, role: {car.attributes['role_name']}")
                if car.attributes["role_name"] == role_name:
                    return car
        else:
            print(f"No vehicle found with role_name: {role_name}")
        return None

class CarlaUpdateVehicleHandler(Node):
    """
    Class to follow the vehicle of real_world.
    """
    def __init__(self, role_name="pygame_adtruck"):

        super().__init__('CarlaUpdateVehicleHandler_Node')

        self.client = carla.Client('localhost', 2000)  # 连接到 Carla 服务器
        self.client.set_timeout(10.0)
        self.world = self.client.get_world()
        self.agent_role_name = role_name

        self.get_logger().info(f"the self.agent_role_name is: {self.agent_role_name}")
        self.get_logger().info(f"world is : {self.world}")
        
        self.ego_vehicle = BridgeHelpers.get_agent_actor(self.world, self.agent_role_name)
        self.get_logger().info(f"ego_vehicle is : {self.ego_vehicle}")
        
        self.pre_pose = None
        self.current_pose = None
        self.topic_name = "/real_world/follow_adtruck/transformed_with_covariance"

        if self.ego_vehicle is None:
            self.get_logger().error(f"[CarlaUpdateVehicleHandler_Node] Can't find Ego Vehicle named {self.agent_role_name}!! Make sure it is spawned!")
            self.stop()
            return  # 停止节点初始化
         # 新增: T1,T2发布器
        self.t1_t2_publisher = self.create_publisher(
            TimeReference, '/fsm_sandbox/timing/t1_t2', 10)
        
        # 新增: 用于跟踪状态的变量
        self.last_received_header = None
        self.pose_update_pending = False
        
        self.subscription = self.create_subscription(
             PoseWithCovarianceStamped,
             self.topic_name,
             self.update_Vehicle_handler_callback,
             1
        )
        
        self.get_logger().info(f"[VehicleFollower] Subscribed to topic: {self.topic_name}")

        # 创建定时器以 50 Hz 频率定时执行 apply_control
        self.timer = self.create_timer(0.033, self.apply_control_50hz)

    def apply_control_50hz(self):
        """
        以 50Hz 的频率应用控制命令，保持车辆停止
        """
        if self.ego_vehicle is not None:
            # 新增: 如果有待处理的位姿更新
            if (self.current_pose is not None and 
                self.last_received_header is not None and 
                self.pose_update_pending):
                
                # 获取T2时间戳（CARLA同步API调用时间）
                t2_stamp = self.get_clock().now()
                
                # 发布T1,T2时间戳对
                timing_msg = TimeReference()
                timing_msg.header.stamp = self.last_received_header.stamp  # T1
                timing_msg.header.frame_id = "t1_t2_pair"
                timing_msg.time_ref = t2_stamp.to_msg()  # T2
                
                self.t1_t2_publisher.publish(timing_msg)
                
                # 执行强制同步
                self.ego_vehicle.set_transform(self.current_pose)
                self.get_logger().debug(f"Vehicle pose updated and timing published")
                
                # 重置状态标记
                self.pose_update_pending = False
                self.last_received_header = None

            self.ego_vehicle.apply_control(carla.VehicleControl(throttle=0.0, brake=1.0, steer=0.0))
        else:
            self.get_logger().error(f"ego_vehicle is None, cannot apply control.")

    def update_Vehicle_handler_callback(self, msg):
        """
        Callback to apply the transform location according to the real vehicle.
        """
        # 检查位姿是否有实际变化
        new_pose = carla.Transform()
        new_pose.location.x = msg.pose.pose.position.x
        new_pose.location.y = -msg.pose.pose.position.y
        new_pose.location.z = msg.pose.pose.position.z

        orientation_q = msg.pose.pose.orientation
        roll, pitch, yaw = quat2euler([orientation_q.w, orientation_q.x, orientation_q.y, orientation_q.z])
        new_pose.rotation.roll = math.degrees(roll)
        new_pose.rotation.pitch = math.degrees(pitch)
        new_pose.rotation.yaw = -math.degrees(yaw)

        # 检查是否超过阈值（使用您的现有方法或简化版本）
        if self.current_pose is None or self.is_pose_changed(new_pose, self.current_pose):
            # 保存新位姿和时间戳
            self.current_pose = new_pose
            self.last_received_header = msg.header
            self.pose_update_pending = True  # 标记有待处理的更新
            
            self.get_logger().debug(f"New pose received, update pending")

    def is_pose_changed(self, new_pose, old_pose, threshold=0.005):
        """
        检查位姿是否发生了显著变化
        """
        if old_pose is None:
            return True
            
        position_diff = math.sqrt(
            pow(new_pose.location.x - old_pose.location.x, 2) + 
            pow(new_pose.location.y - old_pose.location.y, 2) + 
            pow(new_pose.location.z - old_pose.location.z, 2)
        )
        
        yaw_diff = abs(new_pose.rotation.yaw - old_pose.rotation.yaw)
        
        return position_diff > threshold or yaw_diff > 0.5  # 0.5度

    def is_exceeding_threshold(self, cur, pre, threshold=0.02):
        """
        Check if the current position is more than the threshold from the previous position.
        """
        diff = math.sqrt(pow(cur.location.x - pre.location.x, 2) + pow(cur.location.y - pre.location.y, 2))
        diffR = math.sqrt(abs(cur.rotation.yaw - pre.rotation.yaw))

        if diff < threshold and diffR < threshold * 30:
            return False
        else:
            return True

    def run(self):
        """
        Start ROS node loop.
        """
        self.get_logger().info(f"starting spin")
        rclpy.spin(self)

    def stop(self):
        """
        Stop ROS node loop and cleanup.
        """
        self.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":

    rclpy.init(args=None)
    if rclpy.ok():
        print("ROS2 init succeeded")
    else:
        print("ROS2 init failed")

    role_name = os.environ.get('VEHICLE_ROLE_NAME', 'pygame_adtruck')  # 从环境变量获取 role_name
    updating_handler = CarlaUpdateVehicleHandler(role_name)

    try:
        updating_handler.run()
    except Exception as e:
        updating_handler.get_logger().fatal(f"node_destroy")
        updating_handler.stop()
        traceback.print_exc()

    print(f"The vehicle named {os.environ['AGENT_ROLE_NAME']} finished simulation!")