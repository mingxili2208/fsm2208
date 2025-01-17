#!/usr/bin/env python

import os
import sys
import math
import traceback
import carla
import rclpy
from transforms3d.euler import quat2euler
from geometry_msgs.msg import PoseWithCovarianceStamped
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
    Class to follow the vehicle in real_world.
    """
    def __init__(self, role_name="test_spawner"):
        super().__init__('CarlaUpdateVehicleHandler_Node')

        self.client = carla.Client('localhost', 2000)  # Connect to Carla server
        self.client.set_timeout(10.0)
        self.world = self.client.get_world()
        self.agent_role_name = role_name

        self.get_logger().info(f"The self.agent_role_name is: {self.agent_role_name}")
        self.get_logger().info(f"World is: {self.world}")
        
        self.ego_vehicle = BridgeHelpers.get_agent_actor(self.world, self.agent_role_name)
        self.get_logger().info(f"Ego vehicle is: {self.ego_vehicle}")
        
        self.pre_pose = None  # Previous pose
        self.current_pose = None  # Current pose
        self.topic_name = "/real_world/follow_adtruck/transformed_with_covariance"

        if self.ego_vehicle is None:
            self.get_logger().error(f"[CarlaUpdateVehicleHandler_Node] Can't find Ego Vehicle named {self.agent_role_name}!! Make sure it is spawned!")
            self.stop()
            return  # Stop initialization

        self.subscription = self.create_subscription(
            PoseWithCovarianceStamped,
            self.topic_name,
            self.update_Vehicle_handler_callback,
            1
        )
        
        self.get_logger().info(f"[VehicleFollower] Subscribed to topic: {self.topic_name}")

    def update_Vehicle_handler_callback(self, msg):
        """
        Callback to update the vehicle's transform based on the real-world vehicle data.
        """
        self.current_pose = carla.Transform()
        self.current_pose.location.x = msg.pose.pose.position.x
        self.current_pose.location.y = -msg.pose.pose.position.y  # ROS = -CARLA
        self.current_pose.location.z = msg.pose.pose.position.z + 1.0

        # Convert quaternion to Euler angles (CARLA uses Euler angles for rotation)
        orientation_q = msg.pose.pose.orientation
        roll, pitch, yaw = quat2euler([orientation_q.w, orientation_q.x, orientation_q.y, orientation_q.z])
        self.current_pose.rotation.roll = math.degrees(roll)
        self.current_pose.rotation.pitch = math.degrees(pitch)
        self.current_pose.rotation.yaw = -math.degrees(yaw)

        # Check if the movement exceeds the threshold
        if self.pre_pose is None or self.is_exceeding_threshold(self.current_pose, self.pre_pose):
            self.pre_pose = self.current_pose  # Update previous pose
            if self.ego_vehicle is not None:
                self.ego_vehicle.set_transform(self.current_pose)
                self.get_logger().info(f"Updated vehicle pose.")
            else:
                self.get_logger().error(f"Ego_vehicle is None, cannot update transform.")
        else:
            self.get_logger().info(f"Pose change is within threshold, no update required.")

    def is_exceeding_threshold(self, cur, pre, threshold=0.02):
        """
        Check if the current position/rotation exceeds the threshold compared to the previous position/rotation.
        """
        position_diff = math.sqrt(
            pow(cur.location.x - pre.location.x, 2) + pow(cur.location.y - pre.location.y, 2)
        )
        rotation_diff = abs(cur.rotation.yaw - pre.rotation.yaw)

        self.get_logger().debug(f"Position diff: {position_diff}, Rotation diff: {rotation_diff}")
        
        # Check if either the position or rotation exceeds the threshold
        if position_diff > threshold or rotation_diff > threshold * 30:  # Scaled rotation threshold
            return True
        return False

    def run(self):
        """
        Start ROS node loop.
        """
        self.get_logger().info(f"Starting spin")
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

    role_name = os.environ.get('VEHICLE_ROLE_NAME', 'test_spawner')  # Get role_name from environment variable
    updating_handler = CarlaUpdateVehicleHandler(role_name)

    try:
        updating_handler.run()
    except Exception as e:
        updating_handler.get_logger().fatal(f"Node encountered a fatal error, shutting down.")
        updating_handler.stop()
        traceback.print_exc()

    print(f"The vehicle named {os.environ.get('AGENT_ROLE_NAME', 'unknown')} finished simulation!")