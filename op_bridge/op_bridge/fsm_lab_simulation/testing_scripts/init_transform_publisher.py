import sys
sys.path.append("/home/jarvislee-carla/WorkplaceCarlas/op_carla/op_bridge/op_bridge/fsm_lab_simulation")
import carla
import rclpy
import trans_utils as trans
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool


HOST = "localhost"
PORT = 2000
VEHICLE_NAME = "pygame-adtruck"
SPAWN_POINT = 2

class InitTransformPublisherNode:
    def __init__(self):
        self.node = rclpy.create_node("init_transform_publisher_node")
        self.publisher = self.node.create_publisher(
            PoseStamped, f"/init/{VEHICLE_NAME.replace('-', '_')}/transform", 1
        )
        self.subscriber = self.node.create_subscription(
            Bool, f"/init/{VEHICLE_NAME.replace('-', '_')}/spawn_point_initialization_status", self.spawn_point_initialization_status_callback, 1
        )
        self.timer = self.node.create_timer(0.001, self.publish_transform)
        self.spawn_point_initialization_status = False
        # Get the initial pose.
        client = carla.Client(HOST, PORT)
        world = client.get_world()
        transform_carla = world.get_map().get_spawn_points()[SPAWN_POINT]
        self.transform_ros = trans.carla_transform_to_ros_pose(transform_carla)
    
    def publish_transform(self):
        if not self.spawn_point_initialization_status:
            msg = PoseStamped()
            msg.pose = self.transform_ros
            self.publisher.publish(msg)
            self.node.get_logger().info(
                f"Init Transformed Published! {self.spawn_point_initialization_status} [{self.transform_ros.position.x}, {self.transform_ros.position.y}, {self.transform_ros.position.z}, {self.transform_ros.orientation.w}, {self.transform_ros.orientation.x}, {self.transform_ros.orientation.y}, {self.transform_ros.orientation.z}]"
            )
        else:
            self.node.get_logger().info("Spawn point initialized!")
            raise Exception("Shutdown the node!")
    
    def spawn_point_initialization_status_callback(self, data):
        self.spawn_point_initialization_status = data.data


if __name__ == "__main__":
    # Get the vehicle name.
    VEHICLE_NAME = input("Please input the vehicle name: ")
    SPAWN_POINT = int(eval(input("Please input the spawn point index (Integer only): ")))

    rclpy.init(args=None)

    # Init transform publisher node.
    init_transform_publisher_node = InitTransformPublisherNode()

    # Run the ros2 node.
    try:
        rclpy.spin(init_transform_publisher_node.node)
    except Exception:
        rclpy.shutdown()
