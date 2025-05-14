#!/usr/bin/env python

import os
os.environ['PYGAME_HIDE_SUPPORT_PROMPT'] = "hide"

import sys
sys.path.append("/home/jarvislee-carla/WorkplaceCarlas/op_carla/op_bridge/op_bridge/fsm_lab_simulation")
import rclpy
import math

from vr2sx_transformer import VR2SandBoxTransformer
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from std_msgs.msg import Header
from rclpy.node import Node 
import numpy as np

VEHICLE_NAME = "follow_adtruck"

class TransformedSandBoxCoorPublisherNode(Node):
    def __init__(self,update_rate=0.05):
        #self.node = rclpy.create_node("transformed_SandBox_coordinate_publisher_node")
        super().__init__("transformed_SandBox_coordinate_publisher_node")
        self.publisher = self.create_publisher(
            PoseStamped, f"/real_world/{VEHICLE_NAME}/transformed", 1
        )
        # FOR TESTING set timer as 1 
        self.timer = self.create_timer(update_rate, self.publish_transformed_coor)
        self.sandbox_transformer=VR2SandBoxTransformer()
        self.previous_position=None
        self.previous_yaw = None
        # Get the initial pose.
    def RPY2quaternion(self, roll, pitch, yaw):
        """
        transform eluer to quaternion
        parameter:
            roll
            pitch
            yaw
        return (x, y, z, w)
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
    
    def publish_transformed_coor(self):
        
        
        transformed_position, transformed_yaw=self.sandbox_transformer.get_transformed_coor()
        
        if self.previous_position is  None or self.previous_yaw is None:
            self.previous_position = transformed_position
            self.previous_yaw=transformed_yaw
        else:
            if abs(transformed_position[0] - self.previous_position[0]) > 0.005  or abs(transformed_position[2] - self.previous_position[2]) > 0.005 or abs(np.degrees(self.previous_yaw)-np.degrees(transformed_yaw))>1:
                self.get_logger().info(f"\n previous_position is {self.previous_position};    the abs in x is {abs(transformed_position[0] - self.previous_position[0])};  the abs in y is {abs(transformed_position[2] - self.previous_position[2])} the abs in yaw is {abs(np.degrees(self.previous_yaw)-np.degrees(transformed_yaw))}")
                self.previous_position = transformed_position
                self.previous_yaw=transformed_yaw
            else:
                return
        
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = "real_world"

        pose = Pose()
        # transformed_position is 0---x,1---y,2---z; while y is the upwards; 
        # for unify we set ros2_coor as 0---x,1---y,2---z while z is upwards;
        # which means z_ros2_coor = y_transformed_position  
        pose.position = Point(x=transformed_position[0], y=transformed_position[2], z=transformed_position[1])
        pose.orientation = Quaternion(
                x=self.RPY2quaternion(0,0,transformed_yaw)[0],
                y=self.RPY2quaternion(0,0,transformed_yaw)[1],
                z=self.RPY2quaternion(0,0,transformed_yaw)[2],
                w=self.RPY2quaternion(0,0,transformed_yaw)[3]
        )

        msg = PoseStamped(header=header, pose=pose)

        self.publisher.publish(msg)
        self.get_logger().info(
            f"\n Transformed Published! x is {transformed_position[0]}, y is {transformed_position[2]},z is {transformed_position[1]}, yaw is {transformed_yaw}"
        )
        self.get_logger().info(
            f"\n Transformed Published! [{msg.pose.position.x}, {msg.pose.position.y}, {msg.pose.position.z}, {msg.pose.orientation.w}, {msg.pose.orientation.x}, {msg.pose.orientation.y}, {msg.pose.orientation.z}]"
        )


if __name__ == "__main__":
    # Get the vehicle name.
    #VEHICLE_NAME = input("Please input the vehicle name: ")
    
    rclpy.init(args=None)

    # Transform publisher node.
    transformed_sandbox_coor_publisher_node = TransformedSandBoxCoorPublisherNode()

    # Run the ros2 node.
    rclpy.spin(transformed_sandbox_coor_publisher_node)

    rclpy.shutdown()
