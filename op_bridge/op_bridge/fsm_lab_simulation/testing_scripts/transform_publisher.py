import sys
sys.path.append("/home/jarvislee-carla/Workspace/Carlas/op_carla/op_bridge/op_bridge/fsm_lab_simulation")
import carla
import rclpy
import pygame
import trans_utils as trans
from geometry_msgs.msg import PoseStamped


HOST = "localhost"
PORT = 2000
VEHICLE_NAME = "follow-adtruck"

class TransformPublisherNode:
    def __init__(self):
        self.node = rclpy.create_node("transform_publisher_node")
        self.publisher = self.node.create_publisher(
            PoseStamped, f"/real/{VEHICLE_NAME.replace('-', '_')}/transform", 1
        )
        self.timer = self.node.create_timer(0.001, self.publish_transform)
        pygame.init()
        pygame.display.set_mode((800, 600))
        # Get the initial pose.
        client = carla.Client(HOST, PORT)
        world = client.get_world()
        transform_carla = world.get_map().get_spawn_points()[1]
        self.transform_ros = trans.carla_transform_to_ros_pose(transform_carla)
    
    def publish_transform(self):
        for event in pygame.event.get():
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_f:
                    self.transform_ros.position.x += 10
        msg = PoseStamped()
        msg.pose = self.transform_ros
        self.publisher.publish(msg)
        self.node.get_logger().info(
            f"Transformed Published! [{self.transform_ros.position.x}, {self.transform_ros.position.y}, {self.transform_ros.position.z}, {self.transform_ros.orientation.w}, {self.transform_ros.orientation.x}, {self.transform_ros.orientation.y}, {self.transform_ros.orientation.z}]"
        )


if __name__ == "__main__":
    # Get the vehicle name.
    VEHICLE_NAME = input("Please input the vehicle name: ")
    
    rclpy.init(args=None)

    # Transform publisher node.
    transform_publisher_node = TransformPublisherNode()

    # Run the ros2 node.
    rclpy.spin(transform_publisher_node.node)

    rclpy.shutdown()
