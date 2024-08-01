#!/usr/bin/env python

# used to debug with the coordinator of carla

#!/usr/bin/env python

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

class CarlaCoordinatorTest(Node):
    """
    Class to follow the vehicle of real_world.
    """
    def __init__(self,role_name="pygame_adtruck"):

        super().__init__('debug_carla_coordinator')

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
        self.similarity_matrix = np.array([[ 3.27650037e+01, -9.68207899e-01,  0.00000000e+00, -5.41650474e+01],
                             [ 9.68207899e-01,  3.27650037e+01,  0.00000000e+00,  7.28873065e+01],
                             [ 0.00000000e+00,  0.00000000e+00,  3.27793059e+01, -5.08468894e-02],
                             [ 0.00000000e+00,  0.00000000e+00,  0.00000000e+00,  1.00000000e+00]])
            
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
        carla_pose_transformed=self.transform_coordinates_from_sandbox2carl(pose)
        #########################################
        #print(carla_pose_transformed)
        ##########################################
        if self.ego_vehicle is not None:
            self.ego_vehicle.set_transform(carla_pose_transformed)
        else:
            self.get_logger().error(f"[VehicleFollower] Can't find Ego Vehicle named {self.agent_role_name}!! Make sure it is spawned!")
            self.stop()  # 停止 ROS 节点
            raise RuntimeError(f"Can't find Ego Vehicle named {self.agent_role_name}!! Make sure it is spawned!")  # 抛出异常
   
    def transform_coordinates_from_sandbox2carl(self,pose):
        pose.position.z=0.0016
        original_point=np.array([pose.position.x, pose.position.y, pose.position.z])
        homogeneous_point = np.append(original_point, 1)
        transformed_homogeneous_point = self.similarity_matrix @ homogeneous_point
        transformed_point = transformed_homogeneous_point[:3] / transformed_homogeneous_point[3]
        x=transformed_point[0]
        y=transformed_point[1]
        z=transformed_point[2]
        roll, pitch, yaw = quat2euler([
        pose.orientation.w,
        pose.orientation.x,
        pose.orientation.y,
        pose.orientation.z,
            ])
        yaw=math.degrees(yaw)-90
        self.get_logger().info(f"x: {x}, y: {y}, z:{z},yaw: {yaw},")
        new_pose=carla.Transform(carla.Location(x, y, z), carla.Rotation(0, yaw,0 ))
        return new_pose
    
    def test_set_coordinates_carla_transformed_from_sandbox(self,x,y,z,yaw):
        """
        set the transformed coordinates(from sandbox 2 carla & Rotate, translate, scale)
        """
        z=0.0016
        original_point=np.array([x,y,z])
        homogeneous_point = np.append(original_point, 1)
        transformed_homogeneous_point = self.similarity_matrix @ homogeneous_point
        transformed_point = transformed_homogeneous_point[:3] / transformed_homogeneous_point[3]
        x=transformed_point[0]
        y=transformed_point[1]
        z=transformed_point[2]
        self.get_logger().info(f"x: {x}, y: {y}, z:{z},yaw:{yaw}")
        new_pose=carla.Transform(carla.Location(x, y, z), carla.Rotation(0, yaw,0 ))
        self.ego_vehicle.set_transform(new_pose)

    def test_set_coordinates_carla(self,x,y,z,yaw):
        """
        set the coordinates of the vehicle
        test for set of coordinates
        """
        self.get_logger().info(f"x: {x}, y: {y}, z:{z},yaw:{yaw}")
        #carla.Rotation(pitch=0.0, yaw=0.0, roll=0.0)
        new_pose=carla.Transform(carla.Location(x, y, z), carla.Rotation(0, yaw,0 ))
        self.ego_vehicle.set_transform(new_pose)

    def test_get_coordinates_carla(self):
        """
        set the coordinates of the vehicle
        test for set of coordinates
        """
        transform = self.ego_vehicle.get_transform()
        location = transform.location
        x = location.x
        y = location.y
        z = location.z

        rotation = transform.rotation
        pitch = rotation.pitch
        yaw = rotation.yaw
        roll = rotation.roll

        # 打印车辆方向

        self.get_logger().info(f"location: x={x}, y={y}, z={z};\n rotation: pitch={pitch}, yaw={yaw}, roll={roll}")

    
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
    debug_carla_coor = CarlaCoordinatorTest(role_name)

    try:
        #debug_carla_coor.test_coordinates_carla(-61.5,0,0,0)
        #debug_carla_coor.test_get_coordinates_carla()
        #debug_carla_coor.test_set_coordinates_carla_transformed_from_sandbox(0.8,-0.5,0,0)
        debug_carla_coor.run()
    except Exception as e:
        debug_carla_coor.get_logger().fatal(f"node_destroy")
        debug_carla_coor.stop()
        traceback.print_exc()
    
    print(f"The vehicle named {os.environ['AGENT_ROLE_NAME']} finished simulation!")