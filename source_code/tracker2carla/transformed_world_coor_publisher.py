#!/usr/bin/env python

import os
os.environ['PYGAME_HIDE_SUPPORT_PROMPT'] = "hide"

import sys
sys.path.append("/home/jarvislee-carla/Workspace/Carlas/op_carla/op_bridge/op_bridge/fsm_lab_simulation")
import rclpy
import trans_utils as trans
from vr2sx_transformer import VR2SandBoxTransformer
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from std_msgs.msg import Header
from rclpy.node import Node 

VEHICLE_NAME = "follow_adtruck"

class TransformedSandBoxCoorPublisherNode(Node):
    def __init__(self):
        #self.node = rclpy.create_node("transformed_SandBox_coordinate_publisher_node")
        super().__init__("transformed_SandBox_coordinate_publisher_node")
        self.publisher = self.create_publisher(
            PoseStamped, f"/real_world/{VEHICLE_NAME}/transformed", 1
        )
        # FOR TESTING set timer as 1 
        self.timer = self.create_timer(0.1, self.publish_transformed_coor)
        self.sandbox_transformer=VR2SandBoxTransformer()
        # Get the initial pose.
        
    
    def publish_transformed_coor(self):
        
        transformed_position, transformed_yaw=self.sandbox_transformer.get_transformed_coor()
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = "real_world"

        pose = Pose()
        # transformed_position is 0---x,1---y,2---z; while y is the upwards; 
        # for unify we set ros2_coor as 0---x,1---y,2---z while z is upwards;
        # which means z_ros2_coor = y_transformed_position  
        pose.position = Point(x=transformed_position[0], y=transformed_position[2], z=transformed_position[1])
        pose.orientation = Quaternion(
                x=trans.RPY2quaternion(0,0,transformed_yaw)[0],
                y=trans.RPY2quaternion(0,0,transformed_yaw)[1],
                z=trans.RPY2quaternion(0,0,transformed_yaw)[2],
                w=trans.RPY2quaternion(0,0,transformed_yaw)[3]
        )

        msg = PoseStamped(header=header, pose=pose)

        self.publisher.publish(msg)
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
