import sys
sys.path.append("/home/cityu-fsm-lab-carla/Workspace/Carla/op_carla/op_bridge/op_bridge/fsm_lab_simulation")
import carla
import rclpy
import trans_utils as trans
from transforms3d.euler import euler2quat
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from std_msgs.msg import Bool
from vive_tracker import ViveTrackerModule


HOST = "localhost"
PORT = 2000
VEHICLE_NAME = "pygame-adtruck"
SPAWN_POINT = 1
TRACKER_NAME = "tracker_1"

import numpy as np


# TODO: change the manually computed transform matrix to a config file
def cam_to_world(cam_coord):
    points_A = np.array([
        [0.0735, -1.256, -0.864],
        [1.2985, -1.2337, -0.7771],
        [2.6265, -1.2138, -1.4418],
        [0.7060, -1.2437, -1.3144],
        [1.7504, -1.2264, -1.3332],
        [2.2345, -1.2219, -1.9145],
        [0.1044, -1.2530, -1.9240],
        [-0.5540, -1.2630, -2.5030],
        [0.1983, -1.2510, -2.6737],
        [-0.7560, -1.2705, -3.1767],
        [0.0263, -1.2603, -3.7120],
        [1.009,-1.188,-3.468], # [1.0052, -1.2399, -3.4513],  
        [1.442,-1.1759,-3.978], # [1.4389, -1.2370, -3.9670],   
        # ------------------------
    ])

    points_A = [
        {'position': [0.1706, -1.2442, -0.8407], 'orientation': [np.radians(-89.5), np.radians(95.9), np.radians(179)]},      # 1
        {'position': [1.4092, -1.2436, -0.2610], 'orientation': [np.radians(-87.8), np.radians(96.0), np.radians(178.5)]},    # 2
        {'position': [2.8897, -1.2410, -0.3832], 'orientation': [np.radians(-89.8), np.radians(-167), np.radians(-179.6)]},    # 3
        {'position': [1.0596, -1.2391, -0.9761], 'orientation': [np.radians(-97.9), np.radians(95.2), np.radians(-172)]},    # 4
        {'position': [2.0351, -1.2394, -0.6069], 'orientation': [np.radians(-90.5), np.radians(98.6), np.radians(-179.4)]},    # 5
        {'position': [2.6981, -1.2376, -0.9729], 'orientation': [np.radians(-91.2), np.radians(97.6), np.radians(-178)]},    # 6
        {'position': [0.7266, -1.2373, -1.7677], 'orientation': [np.radians(-104.6), np.radians(93.2), np.radians(-164.9)]},    # 7
        {'position': [0.3203, -1.2274, -2.5383], 'orientation': [np.radians(-103), np.radians(92.16), np.radians(-166.6)]},   # 8
        {'position': [1.0885, -1.2293, -2.4300], 'orientation': [np.radians(-84.6), np.radians(95.4), np.radians(174.9)]},    # 9
        {'position': [0.3832, -1.2276, -3.2410], 'orientation': [np.radians(-90.0), np.radians(-177.4), np.radians(179.4)]},   # 10
        {'position': [1.3158, -1.2295, -3.4664], 'orientation': [np.radians(-92.0), np.radians(97.8), np.radians(-177.5)]},    # 11
        {'position': [2.1200, -1.2232, -2.8468], 'orientation': [np.radians(-91.7), np.radians(93.4), np.radians(-178.3)]},         # 12
        {'position': [2.7144, -1.2222, -3.1710], 'orientation': [np.radians(-91.7), np.radians(96.0), np.radians(-178.1)]},        # 13
    ]
    points_A = np.array([pt['position'] for pt in points_A])

    points_B = np.array([
        [0.455, 0.755, 0],
        [1.738, 0.283, 0],
        [3.198, 0.532, 0],
        [1.335, 0.966, 0],
        [2.336, 0.68, 0],
        [2.960, 1.11, 0],
        [0.935, 1.725, 0],
        [0.467, 2.47, 0],
        [1.425, 2.426, 0],
        [0.471, 3.175, 0],
        [1.380, 3.461, 0],
        [2.238, 2.927, 0],
        [2.805, 3.299, 0]
    ])

    A_matrix = []
    B_vector = []

    for i in range(len(points_A)):
        x_A, y_A, z_A = points_A[i]
        x_B, y_B, z_B = points_B[i]
        A_matrix.append([x_A, y_A, z_A, 1, 0, 0, 0, 0, 0, 0, 0, 0])
        A_matrix.append([0, 0, 0, 0, x_A, y_A, z_A, 1, 0, 0, 0, 0])
        A_matrix.append([0, 0, 0, 0, 0, 0, 0, 0, x_A, y_A, z_A, 1])
        B_vector.append(x_B)
        B_vector.append(y_B)
        B_vector.append(z_B)

    A_matrix = np.array(A_matrix)
    B_vector = np.array(B_vector)

    params, _, _, _ = np.linalg.lstsq(A_matrix, B_vector, rcond=None)

    a, b, c, d, e, f, g, h, i, j, k, l = params

    transform_matrix = np.array([
        [a, b, c, d],
        [e, f, g, h],
        [i, j, k, l],
        [0, 0, 0, 1]
    ])

    A_point = np.array(cam_coord + [1])
    B_point_computed = np.dot(transform_matrix, A_point)
    return B_point_computed[:3]


ORIGIN_OFFSET = np.array([1.63, 2.33, 0.0])
ORIGIN_OFFSET = np.array([0.0, 0.0, 0.0])


class InitTransformPublisherNode:
    def __init__(self):
        self.node = rclpy.create_node("init_transform_publisher_node")
        self.publisher = self.node.create_publisher(
            PoseStamped, f"/init/{VEHICLE_NAME.replace('-', '_')}/transform", 1
        )
        self.subscriber = self.node.create_subscription(
            Bool, f"/init/{VEHICLE_NAME.replace('-', '_')}/spawn_point_initialization_status", self.spawn_point_initialization_status_callback, 1
        )
        self.timer = self.node.create_timer(0.1, self.publish_transform)
        self.spawn_point_initialization_status = False
        # Get the initial pose from Vive Tracker
        self.vtm = ViveTrackerModule()
        self.vtm.print_discovered_objects()

        # Get the initial pose.
        # client = carla.Client(HOST, PORT)
        # world = client.get_world()
        # transform_carla = world.get_map().get_spawn_points()[SPAWN_POINT]
        # self.transform_ros = trans.carla_transform_to_ros_pose(transform_carla)

    def get_tracker_transform(self):
        tracker = self.vtm.devices[TRACKER_NAME]
        cam_coord = tracker.get_pose_euler()[:3]
        world_coord = cam_to_world(cam_coord)
        world_coord = np.array(world_coord) - ORIGIN_OFFSET
        scale_ratio = 1.0
        self.transform_ros = Pose()
        self.transform_ros.position = Point()
        self.transform_ros.position.x = world_coord[0] * scale_ratio
        self.transform_ros.position.y = world_coord[1] * scale_ratio
        self.transform_ros.position.z = world_coord[2] * scale_ratio + 2
        roll, pitch, yaw = 0.0, 0.0, 0.0
        quat = euler2quat(roll, pitch, yaw)
        self.transform_ros.orientation = Quaternion(w=quat[0], x=quat[1], y=quat[2], z=quat[3])
    
    def publish_transform(self):
        if not self.spawn_point_initialization_status:
            self.get_tracker_transform()
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
    # VEHICLE_NAME = input("Please input the vehicle name: ")
    # SPAWN_POINT = int(eval(input("Please input the spawn point index (Integer only): ")))

    rclpy.init(args=None)

    # Init transform publisher node.
    init_transform_publisher_node = InitTransformPublisherNode()

    # Run the ros2 node.
    try:
        rclpy.spin(init_transform_publisher_node.node)
    except Exception:
        rclpy.shutdown()
