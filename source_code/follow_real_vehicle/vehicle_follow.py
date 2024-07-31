#!/usr/bin/env python

import os
import cv2
import sys
import time
import signal

import random
import importlib
import traceback

import carla
import rclpy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from leaderboard.envs.sensor_interface import SensorReceivedNoData
from leaderboard.autoagents.agent_wrapper import AgentWrapper, AgentError
from srunner.scenariomanager.timer import GameTime
from srunner.scenariomanager.watchdog import Watchdog
from srunner.scenariomanager.carla_data_provider import CarlaDataProvider
from geometry_msgs.msg import PoseStamped, TwistWithCovariance, PoseWithCovariance, TwistStamped, PoseWithCovarianceStamped
import trans_utils as trans
from rclpy.node import Node 

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

class VehicleFolloweHandler(Node):
    """
    Class to follow the vehicle of real_world.
    """
    def __init__(self,role_name="pygame_adtruck"):

        super().__init__('vehicle_follow_control')

        self.client = carla.Client('localhost', 2000)  # 连接到 Carla 服务器
        self.client.set_timeout(10.0)
        self.world = self.client.get_world()
        self.agent_role_name = role_name
        ##########################################
        print(f"the self.agent_role_name is: {self.agent_role_name}")
        ##########################################   
        ##########################################
        self.get_logger().info(f"world is : {self.world}")
        #######################################   
        self.ego_vehicle = BridgeHelpers.get_agent_actor(self.world, self.agent_role_name)
        #########################################
        self.get_logger().info(f"ego_vehicle is : {self.ego_vehicle}")
        ##########################################

        #################################################
        self.topic_name="/real_world/follow_adtruck/transformed"
        if self.ego_vehicle is None:
            self.get_logger().error(f"[VehicleFollower] Can't find Ego Vehicle named {self.agent_role_name}!! Make sure it is spawned!")
            self.stop()
            return  # 停止节点初始化
            
        self.subscription=self.create_subscription(
            PoseStamped,
            "/real_world/follow_adtruck/transformed",
            self.on_real_vehicle_transform_callback,
            #self.listener_callback,
            10)
        self.get_logger().info(f"[VehicleFollower] Subscribed to topic: {self.topic_name}")
    
    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.pose)    


    def on_real_vehicle_transform_callback(self, data):
        """
        Callback to skip to the transform location according to the real vehicle.
        """
        #self.get_logger().info(f"starting transform")
        
        pose = data.pose
        self.get_logger().debug(f"pose is : {pose}")
        carla_pose_transform = trans.ros_pose_to_carla_transform(pose)
        #########################################
        print(carla_pose_transform)
        ##########################################
        if self.ego_vehicle is not None:
            self.ego_vehicle.set_transform(carla_pose_transform)
        else:
            self.get_logger().error(f"[VehicleFollower] Can't find Ego Vehicle named {self.agent_role_name}!! Make sure it is spawned!")
            self.stop()  # 停止 ROS 节点
            raise RuntimeError(f"Can't find Ego Vehicle named {self.agent_role_name}!! Make sure it is spawned!")  # 抛出异常
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
        print("ROS2 运行时环境已成功初始化")
    else:
        print("ROS2 运行时环境初始化失败")

    role_name = os.environ.get('VEHICLE_ROLE_NAME', 'pygame_adtruck')  # 从环境变量获取 role_name
    ego_follow_handler = VehicleFolloweHandler(role_name)

    try:
        ego_follow_handler.run()
    except Exception as e:
        ego_follow_handler.get_logger().fatal(f"node_destroy")
        ego_follow_handler.stop()
        traceback.print_exc()
    
    print(f"The vehicle named {os.environ['AGENT_ROLE_NAME']} finished simulation!")