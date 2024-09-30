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
    def __init__(self,role_name="pygame_adtruck"):

        super().__init__('CarlaUpdateVehicleHandler_Node')

        self.client = carla.Client('localhost', 2000)  # 连接到 Carla 服务器
        self.client.set_timeout(10.0)
        self.world = self.client.get_world()
        self.agent_role_name = role_name
        ##########################################
        self.get_logger().info(f"the self.agent_role_name is: {self.agent_role_name}")
        ##########################################   
        ##########################################
        self.get_logger().info(f"world is : {self.world}")
        #######################################   
        self.ego_vehicle = BridgeHelpers.get_agent_actor(self.world, self.agent_role_name)
        #########################################
        self.get_logger().info(f"ego_vehicle is : {self.ego_vehicle}")
        ##########################################
        self.pre_pose=None
        self.current_pose=None
        #################################################
        self.topic_name="/real_world/follow_adtruck/transformed_with_covariance"


        if self.ego_vehicle is None:
            self.get_logger().error(f"[CarlaUpdateVehicleHandler_Node] Can't find Ego Vehicle named {self.agent_role_name}!! Make sure it is spawned!")
            self.stop()
            return  # 停止节点初始化
        
        #self.ego_vehicle.set_simulate_physics(True) 
                 
        self.subscription=self.create_subscription(
             PoseWithCovarianceStamped,
             self.topic_name,
             #"/real_world/follow_adtruck/transformed",
             self.update_Vehicle_handler_callback,
             #self.listener_callback,
             10)
        
        self.get_logger().info(f"[VehicleFollower] Subscribed to topic: {self.topic_name}")
    
    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.pose)    

   
    def update_Vehicle_handler_callback(self, msg):
        """
        Callback to skip to the transform location according to the real vehicle.
        """
        #self.get_logger().info(f"starting transform")
        #self.get_logger().info(f"Received PoseWithCovarianceStamped: {msg}")
        
        # 使用接收到的位姿消息来设置生成点
        self.current_pose=carla.Transform()
        self.current_pose.location.x = msg.pose.pose.position.x
        self.current_pose.location.y = -msg.pose.pose.position.y #ros =- carla
        self.current_pose.location.z = msg.pose.pose.position.z

        # 四元数转欧拉角（CARLA 使用欧拉角来表示旋转）
        orientation_q = msg.pose.pose.orientation
        roll, pitch, yaw = quat2euler([orientation_q.w,orientation_q.x, orientation_q.y, orientation_q.z])
        self.current_pose.rotation.roll=math.degrees(roll)
        self.current_pose.rotation.pitch=math.degrees(pitch)
        self.current_pose.rotation.yaw = -math.degrees(yaw) #-90
        self.get_logger().info(f"the yaw now is {self.current_pose.rotation.yaw} degrees")
        if  self.pre_pose is  None:
            self.pre_pose = self.current_pose
            if  self.ego_vehicle is not None:
                self.ego_vehicle.set_transform(self.current_pose)
                self.get_logger().info(f"finished init set_transform")
            else:
                self.get_logger().error(f"[VehicleFollower] Can't find Ego Vehicle named {self.agent_role_name}!! Make sure it is spawned!")
                self.stop()  # 停止 ROS 节点
                raise RuntimeError(f"Can't find Ego Vehicle named {self.agent_role_name}!! Make sure it is spawned!")  # 抛出异常
        elif    self.is_exceeding_threshold(self.current_pose,self.pre_pose) is True :
            self.pre_pose = self.current_pose 
            if  self.ego_vehicle is not None:
                self.ego_vehicle.set_transform(self.current_pose)
                self.get_logger().info(f"finished set_transform")
            else:
                self.get_logger().error(f"[VehicleFollower] Can't find Ego Vehicle named {self.agent_role_name}!! Make sure it is spawned!")
                self.stop()  # 停止 ROS 节点
                raise RuntimeError(f"Can't find Ego Vehicle named {self.agent_role_name}!! Make sure it is spawned!")  # 抛出异常

        #self.get_logger().info(f"finished transform")
        if self.ego_vehicle is None:
            self.get_logger().error(f"[VehicleFollower] Can't find Ego Vehicle named {self.agent_role_name}!! Make sure it is spawned!")
            self.stop()  # 停止 ROS 节点
            raise RuntimeError(f"Can't find Ego Vehicle named {self.agent_role_name}!! Make sure it is spawned!")  # 抛出异常

        #########################################
        #print(carla_pose_transformed)
        ##########################################
        # if self.ego_vehicle is not None:
        #     self.ego_vehicle.set_transform(self.current_pose)
        #     #self.get_logger().info(f"finished transform")
        # else:
        #     self.get_logger().error(f"[VehicleFollower] Can't find Ego Vehicle named {self.agent_role_name}!! Make sure it is spawned!")
        #     self.stop()  # 停止 ROS 节点
        #     raise RuntimeError(f"Can't find Ego Vehicle named {self.agent_role_name}!! Make sure it is spawned!")  # 抛出异常
   
    def is_exceeding_threshold(self, cur, pre,threshold = 0.03):
        """
        check if the current position is more than the threshold from the previous position
        """
        diff=math.sqrt(pow(cur.location.x-pre.location.x,2)+pow(cur.location.y-pre.location.y,2))
        
        diffR=math.sqrt(abs(cur.rotation.yaw-pre.rotation.yaw))

        if diff < threshold and diffR < threshold*30:
            return False
        else:
            return True

    
    def ros_point_to_carla_location(self,ros_point):
        return carla.Location(ros_point.x, ros_point.y, ros_point.z)
    
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
        #following_handler.test_coordinates_carla(-61.5,0,0,0)
        #following_handler.test_get_coordinates_carla()
        #following_handler.test_set_coordinates_carla_transformed_from_sandbox(0.8,-0.5,0,0)
        updating_handler.run()
    except Exception as e:
        updating_handler.get_logger().fatal(f"node_destroy")
        updating_handler.stop()
        traceback.print_exc()
    
    print(f"The vehicle named {os.environ['AGENT_ROLE_NAME']} finished simulation!")    
    