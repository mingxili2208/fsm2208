"""
This module provides a ROS autonomous agent interface to control the ego vehicle via a ROS stack.
"""
import os
import cv2
import time
import math
import numpy
import signal
import datetime
import threading
import subprocess
import trans_utils as trans

import carla
import rclpy
from rclpy.qos import QoSProfile
from cv_bridge import CvBridge
from enum import Enum

from rosgraph_msgs.msg import Clock
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import Header, String
from builtin_interfaces.msg import Time
from transforms3d.euler import euler2quat
from geometry_msgs.msg import PoseStamped, TwistWithCovariance, PoseWithCovariance, TwistStamped, PoseWithCovarianceStamped
from std_msgs.msg import Bool
from sensor_msgs.msg import Image, PointCloud2, NavSatFix, NavSatStatus, CameraInfo, PointField, Imu
from leaderboard.autoagents.autonomous_agent import AutonomousAgent, Track
from srunner.scenariomanager.carla_data_provider import CarlaDataProvider

if os.environ["RUNNING_MODE"] == "record":
    from messages.sensors.lidar import create_cloud
else:
    from sensor_msgs_py.point_cloud2 import create_cloud

if os.environ["CONTROL_MODE"] == "autoware":
    from autoware_auto_control_msgs.msg import AckermannControlCommand
    from autoware_auto_vehicle_msgs.msg import ControlModeReport, GearReport, SteeringReport, TurnIndicatorsReport, HazardLightsReport, VelocityReport


def get_entry_point():
    return "EgoVehicleInit"

# 定义传感器类型枚举
SensorType = Enum('SensorType', 
    {'CAMERA': 1, 'LIDAR': 2, 'GNSS': 3, 'IMU': 4, 'SPEEDOMETER': 5, 'OPENDRIVE_MAP': 6}
)


# 定义传感器数据结构体
class SensorData:
    def __init__(self, data, timestamp, sensor_type: SensorType):
        self.data = data
        self.timestamp = timestamp
        self.sensor_type = sensor_type

# 定义 ROS 节点类
class RosNode:
    def __init__(self, node_name):
        rclpy.init(args=None)
        self.node = rclpy.create_node(node_name)
        self.publishers = {}
        self.subscriptions = {}

    def create_publisher(self, topic_name, msg_type, qos_profile=None):
        self.publishers[topic_name] = self.node.create_publisher(msg_type, topic_name, qos_profile=qos_profile)

    def create_subscription(self, topic_name, msg_type, callback, qos_profile=None):
        self.subscriptions[topic_name] = self.node.create_subscription(msg_type, topic_name, callback, qos_profile=qos_profile)

    def publish(self, topic_name, msg):
        self.publishers[topic_name].publish(msg)

    def spin(self):
        rclpy.spin(self.node)
    
    def destroy(self):
        self.node.destroy_node()
        rclpy.shutdown()

# 定义传感器管理器类
class SensorManager:
    def __init__(self, ros_node: RosNode, world):
        self.ros_node = ros_node
        self.world = world
        self.sensors = {}
        self.sensor_frame_ids = {}
        self.id_to_camera_info_map = {}
        self.cv_bridge = CvBridge()
        self.lidar_publish_prev_time = datetime.datetime.now()
        self.camera_publish_prev_time = {}
        self.imu_publish_prev_time = datetime.datetime.now()
        self.can_publish_prev_time = datetime.datetime.now()
        self.gnss_publish_prev_time = datetime.datetime.now()
        self.ego_vehicle_transform_publish_prev_time = datetime.datetime.now()
        self.lidar_freq = 11
        self.camera_freq = 11
        self.imu_freq = 200 if os.environ["RUNNING_MODE"] == "record" else 50
        self.can_freq = 50
        self.ego_vehicle_transform_freq = 50
        self.gnss_freq = 2
        self.topic_base = "" if os.environ["CONTROL_MODE"].lower() == "autoware" else "/carla/{}"

    def add_sensor(self, sensor_id, sensor_type: SensorType, frame_id=None, attributes=None):
        self.sensors[sensor_id] = {"type": sensor_type, "frame_id": frame_id, "attributes": attributes}
        if sensor_type == SensorType.CAMERA:
            self.camera_publish_prev_time[sensor_id] = datetime.datetime.now()
            self.id_to_camera_info_map[sensor_id] = self.build_camera_info(attributes)
            self.ros_node.create_publisher(
                CameraInfo,
                f"{self.topic_base}/sensing/camera/vehicle_{sensor_id.lower()}/camera_info",
                1,
            )
        elif sensor_type == SensorType.LIDAR:
            self.sensor_frame_ids[sensor_id] = frame_id

    def handle_sensor_data(self, sensor_id, data, timestamp):
        sensor_type = self.sensors[sensor_id]["type"]
        if sensor_type == SensorType.CAMERA:
            self.publish_camera_data(sensor_id, data, timestamp)
        elif sensor_type == SensorType.LIDAR:
            self.publish_lidar_data(sensor_id, data, timestamp)
        elif sensor_type == SensorType.GNSS:
            self.publish_gnss_data(sensor_id, data, timestamp)
        elif sensor_type == SensorType.IMU:
            self.publish_imu_data(sensor_id, data, timestamp)
        elif sensor_type == SensorType.SPEEDOMETER:
            self.publish_can_data(sensor_id, data, timestamp)

    def publish_camera_data(self, sensor_id, data, timestamp):
        if self.checkFrequency(self.camera_publish_prev_time[sensor_id], self.camera_freq):
            return
        
        self.camera_publish_prev_time[sensor_id] = datetime.datetime.now()
        msg = self.cv_bridge.cv2_to_imgmsg(data, encoding="bgr8")
        msg.header = self.get_header(timestamp)
        msg.header.frame_id = self.sensors[sensor_id]["frame_id"]
        self.ros_node.publish(
            f"{self.topic_base}/sensing/camera/vehicle_{sensor_id.lower()}/image_raw",
            msg,
        )
        cam_info = self.id_to_camera_info_map[sensor_id]
        cam_info.header = msg.header
        self.ros_node.publish(
            f"{self.topic_base}/sensing/camera/vehicle_{sensor_id.lower()}/camera_info",
            cam_info,
        )

    def publish_lidar_data(self, sensor_id, data, timestamp):
        if self.checkFrequency(self.lidar_publish_prev_time, self.lidar_freq):
            return
        
        self.lidar_publish_prev_time = datetime.datetime.now()
        header = self.get_header(timestamp)
        lidar_data = numpy.frombuffer(data, dtype=numpy.float32)
        if lidar_data.shape[0] % 4 == 0:
            lidar_data = numpy.reshape(lidar_data, (int(lidar_data.shape[0] / 4), 4))
            lidar_data = lidar_data[..., [1, 0, 2, 3]]
            fields = [
                PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
                PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
                PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
                PointField(name="intensity", offset=12, datatype=PointField.FLOAT32, count=1),
            ]
            if self.sensors[sensor_id]["frame_id"] == "velodyne_top" and os.environ["RUNNING_MODE"] == "record":
                header.frame_id = "velodyne"
            else:
                header.frame_id = self.sensors[sensor_id]["frame_id"]
            msg = create_cloud(header, fields, lidar_data)
            self.ros_node.publish(
                f"/velodyne_points" if os.environ["RUNNING_MODE"] == "record" else f"{self.topic_base}/carla_pointcloud",
                msg,
            )
        else:
            print(f"Cannot Reshape LIDAR Data buffer for {self.sensors[sensor_id]['frame_id']}")

    def publish_gnss_data(self, sensor_id, data, timestamp):
        if self.checkFrequency(self.gnss_publish_prev_time, self.gnss_freq):
            return
        
        self.gnss_publish_prev_time = datetime.datetime.now()
        msg = NavSatFix()
        msg.header = self.get_header(timestamp)
        msg.header.frame_id = self.sensors[sensor_id]["frame_id"]
        msg.latitude = data[0]
        msg.longitude = data[1]
        msg.altitude = data[2]
        msg.status.status = NavSatStatus.STATUS_SBAS_FIX
        msg.status.service = NavSatStatus.SERVICE_GPS | NavSatStatus.SERVICE_GLONASS | NavSatStatus.SERVICE_COMPASS | NavSatStatus.SERVICE_GALILEO
        self.ros_node.publish(
            f"{self.topic_base}/carla_nav_sat_fix",
            msg,
        )

    def publish_imu_data(self, sensor_id, data, timestamp):
        if self.checkFrequency(self.imu_publish_prev_time, self.imu_freq):
            return
        
        self.imu_publish_prev_time = datetime.datetime.now()
        imu_msg = Imu()
        imu_msg.header = self.get_header(timestamp)
        imu_msg.header.frame_id = self.sensors[sensor_id]["frame_id"]

        imu_msg.linear_acceleration.x = data[0]
        imu_msg.linear_acceleration.y = -data[1]
        imu_msg.linear_acceleration.z = data[2]

        imu_msg.angular_velocity.x = -data[3]
        imu_msg.angular_velocity.y = data[4]
        imu_msg.angular_velocity.z = -data[5]

        imu_rotation = data[6]

        quaternion = euler2quat(0, 0, -math.radians(imu_rotation))
        imu_msg.orientation.x = quaternion[0]
        imu_msg.orientation.y = quaternion[1]
        imu_msg.orientation.z = quaternion[2]
        imu_msg.orientation.w = quaternion[3]

        self.ros_node.publish(
            f"/sensing/imu/tamagawa/imu_raw" if os.environ["RUNNING_MODE"] == "record" else f"{self.topic_base}/sensing/imu/tamagawa/imu_raw",
            imu_msg,
        )
#################################question####################################
    def publish_can_data(self, sensor_id, data, timestamp):
        if self.checkFrequency(self.can_publish_prev_time, self.can_freq):
            return
        
        self.can_publish_prev_time = datetime.datetime.now()
        self.speed = data["speed"]
        
        #  Publish Odometry message
        pose_msg = PoseWithCovariance()
        pose_msg.pose = trans.carla_transform_to_ros_pose(self.get_ego_vehicle().get_transform())
        twist_msg = TwistWithCovariance()
        twist_msg.twist.linear.x = data["speed"]
        twist_msg.twist.angular.z = -self.current_control.steer

        odo_msg = Odometry()
        odo_msg.header = self.get_header(timestamp)
        odo_msg.pose = pose_msg
        odo_msg.twist = twist_msg
        self.ros_node.publish(
            f"{self.topic_base}/odo",
            odo_msg,
        )

        if os.environ["CONTROL_MODE"] == "autoware":
            #  Publish Autoware Vehicle Status messages
            vel_rep = VelocityReport()
            vel_rep.header = self.get_header(timestamp)
            vel_rep.header.frame_id = "base_link"
            vel_rep.longitudinal_velocity = data["speed"]
            vel_rep.heading_rate = 0.0
            self.ros_node.publish("/vehicle/status/velocity_status", vel_rep)

            steer_rep = SteeringReport()
            steer_rep.steering_tire_angle = -self.current_control.steer * self.max_steer_angle
            self.ros_node.publish("/vehicle/status/steering_status", steer_rep)

            gear_rep = GearReport()
            gear_rep.stamp = self.get_header(timestamp).stamp
            gear_rep.report = GearReport.DRIVE if self.current_control.gear == 1 else GearReport.PARK
            self.ros_node.publish("/vehicle/status/gear_status", gear_rep)

            control_mode_rep = ControlModeReport()
            control_mode_rep.stamp = self.get_header(timestamp).stamp
            control_mode_rep.mode = ControlModeReport.AUTONOMOUS
            self.ros_node.publish("/vehicle/status/control_mode", control_mode_rep)
    
    def publish_ego_vehicle_transform(self, timestamp):
        if self.checkFrequency(self.ego_vehicle_transform_publish_prev_time, self.ego_vehicle_transform_freq):
            return
        
        self.ego_vehicle_transform_publish_prev_time = datetime.datetime.now()
        ego_vehicle_transform = PoseWithCovariance()
        ego_vehicle_transform.pose = trans.carla_transform_to_ros_pose(self.get_ego_vehicle().get_transform())

        pos_msg = PoseWithCovarianceStamped()
        pos_msg.pose = ego_vehicle_transform
        pos_msg.header = self.get_header(timestamp)
        self.ros_node.publish(
            f"{self.topic_base}/vehicle_transform",
            pos_msg,
        )
    
    def build_camera_info(self, attributes):
        """
        Private function to compute camera info.

        Camera info doesn't change over time.
        """
        
        camera_info = CameraInfo()
        camera_info.width = int(attributes["width"])
        camera_info.height = int(attributes["height"])
        camera_info.distortion_model = "plumb_bob"
        cx = camera_info.width / 2.0
        cy = camera_info.height / 2.0
        fx = camera_info.width / (
            2.0 * math.tan(float(attributes["fov"]) * math.pi / 360.0)
        )
        fy = fx
        camera_info.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
        camera_info.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        camera_info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        camera_info.p = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]

        return camera_info
    
    def get_ego_vehicle(self):
        for vehicle in CarlaDataProvider.get_world().get_actors().filter("vehicle.*"):
            if vehicle.attributes["role_name"] == self.agent_role_name:
                return vehicle
        return None
    
    def get_header(self, timestamp):
        """
        Returns ROS message header.
        """
        
        header = Header()
        seconds = int(timestamp)
        nanoseconds = int((timestamp - int(timestamp)) * 1000000000.0)
        header.stamp = Time(sec=seconds, nanosec=nanoseconds)

        return header
    
    def checkFrequency(self, prev_time, target_freq):
        time_delta = (datetime.datetime.now() - prev_time).microseconds / 1000000.0
        if 1.0 / time_delta >= target_freq:
            return True
        return False

# 定义车辆控制器类
class VehicleController:
    def __init__(self):
        self.current_control = carla.VehicleControl()
        self.steering_factor = 0.45
        self.reverse_steering_factor = 1.7
        self.max_steer_angle = 0.7
        self.speed = 0.0

    def handle_control_command(self, data):
        # FSM Lab Updated controller.
        cmd = carla.VehicleControl()
        if abs(data.longitudinal.acceleration) <= 1.5:
            cmd.gear = 1
        else:
            cmd.gear = 0
        
        if data.longitudinal.speed > 0:
            cmd.reverse = False
            cmd.steer = (-data.lateral.steering_tire_angle / self.max_steer_angle) * self.steering_factor
        elif data.longitudinal.speed < 0:
            cmd.reverse = True
            cmd.steer = (-data.lateral.steering_tire_angle / self.max_steer_angle) * self.reverse_steering_factor
            self.speed = 0.0
        else:
            cmd.reverse = False
            # cmd.steer = 0.0
            self.speed = 0.0
        
        speed_diff = data.longitudinal.speed - self.speed

        if speed_diff > 0:
            cmd.throttle = 0.75
            cmd.brake = 0.0
        elif speed_diff < 0:
            if cmd.reverse:
                cmd.throttle = 0.30
                cmd.brake = 0.0
            else:
                cmd.throttle = 0.0
                if data.longitudinal.speed <= 0.0:
                    cmd.brake = 0.75
                elif speed_diff > -1:
                    cmd.brake = 0.0
                else:
                    cmd.brake = 0.01
        else:
            cmd.throttle = 0.0
            cmd.brake = 0.75
        
        self.current_control = cmd

# 定义地图管理器类
class MapManager:
    def __init__(self, ros_node: RosNode):
        self.ros_node = ros_node
        self.current_map_name = None
        self.topic_base = "" if os.environ["CONTROL_MODE"].lower() == "autoware" else "/carla/{}"

    def handle_map_data(self, map_name, map_data):
        if self.current_map_name != map_name:
            self.current_map_name = map_name
            self.write_opendirve_map_file(self.current_map_name, map_data)
            data_msg = String()
            data_msg.data = map_data
            self.ros_node.publish(
                f"{self.topic_base}/carla/map_file",
                data_msg,
            )
    
    def write_opendirve_map_file(self, map_name, map_data):
        team_code_path = os.environ["OP_AGENT_ROOT"]
        if not team_code_path or not os.path.exists(team_code_path):
            raise IOError("Path '{}' defined by OP_AGENT_ROOT invalid.".format(team_code_path))
        opendrive_map_path = "{}/hdmaps/{}.xodr".format(team_code_path, map_name)
        os.makedirs(f"{team_code_path}/hdmaps/", exist_ok=True)
        f = open(opendrive_map_path, "w")
        f.write(map_data)
        f.close()

# 定义 EgoVehicleInit 类
class EgoVehicleInit(AutonomousAgent):
    def __init__(self):
        super().__init__()
        self.track = Track.MAP
        self.local_host = os.environ["SIMULATOR_LOCAL_HOST"]
        self.port = int(os.environ["SIMULATOR_PORT"])
        self.remote_connection = eval(os.environ["REMOTE_CONNECTION"])
        self.agent_role_name = os.environ["AGENT_ROLE_NAME"].replace("-", "_")
        self.bridge_mode = os.environ["OP_BRIDGE_MODE"]
        self.topic_waypoints = "/carla/{}/waypoints".format(self.agent_role_name)
        self.stack_process = None
        self.carla_gnss_process = None
        self.lidar_slam_process = None
        self.counter = 0
        self.global_plan_published_time = 0
        self.spawn_point_initialized = False
        self._global_plan_world_coord = None
        self.open_drive_map_name = None
        #  创建 ROS 节点
        self.ros_node = RosNode(self.agent_role_name.replace("-", "_"))
        #  创建传感器管理器
        self.sensor_manager = SensorManager(self.ros_node, CarlaDataProvider.get_world())
        #  创建车辆控制器
        self.vehicle_controller = VehicleController()
        #  创建地图管理器
        self.map_manager = MapManager(self.ros_node)
        self.clock_publisher = self.ros_node.create_publisher(Clock, f"/carla/{self.agent_role_name}/clock", 10)
        self.waypoint_publisher = self.ros_node.create_publisher(Path, self.topic_waypoints, 1)
        if self.remote_connection:
            self.spawn_point_initialization_status_publisher = self.ros_node.create_publisher(Bool, f"/init/{self.agent_role_name}/spawn_point_initialization_status", 1)
            self.spawn_point_initialization_status_publisher_timer = self.ros_node.create_timer(0.001, self.on_init_spawn_point_feedback_callback)
        
        # Check the agent root.
        team_code_path = os.environ["OP_AGENT_ROOT"]
        if not team_code_path or not os.path.exists(team_code_path):
            raise IOError("Path '{}' defined by OP_AGENT_ROOT invalid.".format(team_code_path))

        # Get autoware_start_script from environment if 'CONTROL_MODE' is 'autoware'.
        if os.environ["CONTROL_MODE"] == "autoware":
            self.auto_start_script = "{}/start_ros2.sh".format(team_code_path)
            if not os.path.exists(self.auto_start_script):
                raise IOError("File '{}' defined by OP_AGENT_ROOT invalid.".format(self.auto_start_script))
            # Get the carla_gnss_start_script from environment if 'FREE_MAP_NAME' contains 'fsm_lab'.
            if "fsm_lab".lower() in os.environ["FREE_MAP_NAME"]:
                self.carla_gnss_start_script = f"{os.environ['OP_BRIDGE_ROOT']}/op_scripts/fsm_lab_simulation/run_carla_gnss_ros2.sh"
                if not os.path.exists(self.carla_gnss_start_script):
                    raise IOError("File '{}' defined by OP_AGENT_ROOT invalid.".format(self.carla_gnss_start_script))
        
        # Get the lidar_slam_start_script from environment if 'RUNNING_MODE' is 'record'.
        if os.environ["RUNNING_MODE"] == "record":
            self.lidar_slam_start_script = f"{os.environ['OP_BRIDGE_ROOT']}/op_scripts/fsm_lab_simulation/run_lidar_slam_ros2.sh"
            if not os.path.exists(self.lidar_slam_start_script):
                raise IOError("File '{}' defined by OP_AGENT_ROOT invalid.".format(self.lidar_slam_start_script))
    
    def lidar_slam_start(self):
        self.lidar_slam_process = subprocess.Popen(["x-terminal-emulator", "-e", f"{self.lidar_slam_start_script}"])
    
    def carla_gnss_start(self):
        self.carla_gnss_process = subprocess.Popen(["x-terminal-emulator", "-e", f"{self.carla_gnss_start_script}"])
    
    def auto_init_local_agent(self, role_name, map_name, waypoints_topic_name, enable_explore):
        print("Executing stack...", role_name, map_name)
        auto_start_script = self.auto_start_script + " " + role_name + " " + map_name + " " + enable_explore + " " + waypoints_topic_name
        # self.stack_process = subprocess.Popen(auto_start_script, shell=True, preexec_fn=os.setpgrp)
        self.stack_process = subprocess.Popen(["x-terminal-emulator", "-e", f"{auto_start_script}"])

    def setup(self, path_to_conf_file):
        #  初始化传感器
        self.init_sensors()
        #  初始化 ROS 订阅器
        self.init_subscribers()
        # Spin the ROS2 node loop.
        self.spin_thread = threading.Thread(target=self.ros_node.spin)
        self.spin_thread.start()

    def init_sensors(self):
        #  添加传感器
        sensors = [
            {"type": "sensor.camera.rgb", "x": 0.7, "y": 0.0, "z": 1.6, "roll": 0.0, "pitch": 0.0, "yaw": 0.0, "width": 1280, "height": 720, "fov": 100, "id": "Center"},
            {"type": "sensor.camera.rgb", "x": -1.4, "y": 0.0, "z": 1.6, "roll": 0.0, "pitch": 0.0, "yaw": 180, "width": 1280, "height": 720, "fov": 100, "id": "Back"},
            {"type": "sensor.camera.rgb", "x": 0.0, "y": 0.0, "z": 30.0, "roll": 0.0, "pitch": -90.0, "yaw": 0.0, "width": 1280, "height": 720, "fov": 100, "id": "Bev"},
            {"type": "sensor.lidar.ray_cast", "x": 0.0, "y": 0.0, "z": 2.6, "roll": 0.0, "pitch": 0.0, "yaw": -90.0, "id": "LIDAR-Top", "frame_id": "velodyne_top"},
            {"type": "sensor.other.gnss", "x": 0.0, "y": 0.0, "z": 1.6, "id": "GPS", "frame_id": "gnss_link"},
            {"type": "sensor.other.imu", "x": 0.0, "y": 0.0, "z": 1.6, "roll": 0.0, "pitch": 0.0, "yaw": 0.0, "id": "IMU", "frame_id": "tamagawa/imu_link"},
            {"type": "sensor.speedometer", "reading_frequency": 10, "id": "speed"},
        ]
        for sensor in sensors:
            if sensor["type"] == "sensor.camera.rgb":
                self.sensor_manager.add_sensor(sensor["id"], SensorType.CAMERA, f"vehicle_{sensor['id'].lower()}/camera_link", sensor)
            elif sensor["type"] == "sensor.lidar.ray_cast":
                self.sensor_manager.add_sensor(sensor["id"], SensorType.LIDAR, sensor["frame_id"])
            elif sensor["type"] == "sensor.other.gnss":
                self.sensor_manager.add_sensor(sensor["id"], SensorType.GNSS, sensor["frame_id"])
            elif sensor["type"] == "sensor.other.imu":
                self.sensor_manager.add_sensor(sensor["id"], SensorType.IMU, sensor["frame_id"])
            elif sensor["type"] == "sensor.speedometer":
                self.sensor_manager.add_sensor(sensor["id"], SensorType.SPEEDOMETER)

    def init_subscribers(self):
        #  创建 ROS 订阅器
        if self.remote_connection:
            self.ros_node.create_subscription(
                PoseStamped, f"/init/{self.agent_role_name}/transform", self.on_init_spawn_point_callback, 1,
            )
        if os.environ["CONTROL_MODE"] == "autoware":
            self.ros_node.create_subscription(
                AckermannControlCommand, "/control/command/control_cmd", self.vehicle_controller.handle_control_command, qos_profile=QoSProfile(depth=1),
            )
            self.ros_node.create_subscription(
                PoseWithCovarianceStamped, "/initialpose", self.on_auto_vehicle_initialpose_callback, 1,
            )
        elif os.environ["CONTROL_MODE"] == "teleop":
            self.ros_node.create_subscription(
                TwistStamped, f"/carla/{self.agent_role_name}/carla_op_controller_cmd", self.on_vehicle_control_callback, 1,
            )
        elif os.environ["CONTROL_MODE"] == "follow":
            self.ros_node.create_subscription(
                PoseStamped, f"/real/{self.agent_role_name}/transform", self.on_real_vehicle_transform_skip_control_callback, 1,
            )
        elif os.environ["CONTROL_MODE"] == "pygame":
            print("Controlling has been assigned to pygame window, please go to the pygame window and use wasd to control the ego-vehicle!")
            print("Hints: The pygame control mode has two sub-control modes 'Autopilot in Carla' and 'Window (WASD)', please press 'c' or 'p' to switch the sub-control modes respectively!")
            if os.environ["RUNNING_MODE"] == "record":
                print("Hints: You are in 'record' running mode now.")
                print("Hints: After rviz opening, please wait for 2 or 5 seconds to start moving the car.")
                print("Hints: Before moving the car in pygame window, please press 's' to free the brake first to get more robust point cloud.")
        else:
            raise ValueError(f"Invalid CONTROL_MODE={os.environ['CONTROL_MODE']} has been recieved! CONTROL_MODE should only be one of ['autoware', 'pygame', 'follow', 'teleop']")
    
    def on_init_spawn_point_callback(self, data: PoseStamped):
        """
        Callback if a initial spawn point msg is received.
        """

        pose = data.pose
        carla_pose_transform = trans.ros_pose_to_carla_transform(pose)
        if self.remote_connection and not self.spawn_point_initialized:
            if self.sensor_manager.get_ego_vehicle() is not None:
                self.sensor_manager.get_ego_vehicle().set_transform(carla_pose_transform)
                self.spawn_point_initialized = True
                print("Successfully recieve the remote vehicle's pose and initialized spawn point in carla with it.")
            else:
                print(f"[on_init_spawn_point_callback] Can't find Ego Vehicle named {self.agent_role_name}!! Make sure it is spawned!")
        else:
            print("Warning: reinitialize the spawn point from the remote vehicle is not allowed! Please reboot.")

    def on_init_spawn_point_feedback_callback(self):
        """
        Callback if a initial spawn point msg is received.
        """

        self.spawn_point_initialization_status = Bool()
        self.spawn_point_initialization_status.data = self.spawn_point_initialized
        self.spawn_point_initialization_status_publisher.publish(self.spawn_point_initialization_status)

    def on_auto_vehicle_initialpose_callback(self, data: PoseWithCovarianceStamped):
        """
        Callback if a initialpose msg is received.
        """

        pose =data.pose.pose
        pose.position.z += 2.0
        carla_pose_transform = trans.ros_pose_to_carla_transform(pose)
        if self.sensor_manager.get_ego_vehicle() is not None:
            self.sensor_manager.get_ego_vehicle().set_transform(carla_pose_transform)
        else:
            print(f"[on_auto_vehicle_initialpose_callback] Can't find Ego Vehicle named {self.agent_role_name}!! Make sure it is spawned!")
    
    def on_vehicle_control_callback(self, data):
        """
        Callback if a vehicle control command is received (not from autoware).
        """

        cmd = carla.VehicleControl()
        cmd.throttle = data.twist.linear.x / 100.0
        cmd.steer = data.twist.angular.z / 100.0
        cmd.brake = data.twist.linear.y / 100.0
        self.vehicle_controller.current_control = cmd

    def on_real_vehicle_transform_skip_control_callback(self, data: PoseStamped):
        """
        Callback to skip to the transform location according to the real vehicle.
        """

        pose = data.pose
        carla_pose_transform = trans.ros_pose_to_carla_transform(pose)
        if self.sensor_manager.get_ego_vehicle() is not None:
            self.sensor_manager.get_ego_vehicle().set_transform(carla_pose_transform)
        else:
            print(f"[on_real_vehicle_transform_skip_control_callback] Can't find Ego Vehicle named {self.agent_role_name}!! Make sure it is spawned!")

    def run_step(self, input_data, timestamp):
        """
        Execute one step of navigation.
        """

        # Check the remote connection status.
        while self.remote_connection and not self.spawn_point_initialized:
            print("Waiting for remote vehicle's initialization pose...")

        town_map_name = self.map_manager.current_map_name
        if self.stack_process is None and town_map_name is not None and self.open_drive_map_name is not None:
            if os.environ["CONTROL_MODE"] == "autoware":
                if self.carla_gnss_process is None and "fsm_lab".lower() in os.environ["FREE_MAP_NAME"]:
                    self.carla_gnss_start()
                if self.bridge_mode == "free" or self.bridge_mode == "srunner":
                    self.auto_init_local_agent(self.agent_role_name, town_map_name, '', 'true')
                elif self.bridge_mode == "leaderboard":
                    self.auto_init_local_agent(self.agent_role_name, town_map_name, self.topic_waypoints, "false")
                else:
                    self.auto_init_local_agent(self.agent_role_name, town_map_name, self.topic_waypoints, "false")
                
            if self._global_plan_world_coord:
                self.publish_plan(timestamp)
        
        if self.lidar_slam_process is None and os.environ["RUNNING_MODE"] == "record":
            self.lidar_slam_start()
        
        seconds = int(timestamp)
        nanoseconds = int((timestamp - int(timestamp)) * 1000000000.0)
        obj_clock = Clock()
        obj_clock.clock = Time(sec=seconds, nanosec=nanoseconds)

        self.clock_publisher.publish(obj_clock)

        # Check if stack is still running.
        if self.stack_process and self.stack_process.poll() is not None:
            raise RuntimeError("Stack exited with: {} {}".format(self.stack_process.returncode, self.stack_process.communicate()[0]))
        
        # Wait 2 second before publish the global path.
        if self._global_plan_world_coord and (timestamp - self.global_plan_published_time) > 2.0:
            self.global_plan_published_time = timestamp
            self.publish_plan(timestamp)
        
        # Publish the ego vehicle transform.
        self.sensor_manager.publish_ego_vehicle_transform(timestamp)
        
        # Publish data of all sensors.
        for key, val in input_data.items():
            if key == "OpenDRIVE":
                self.open_drive_map_name = self.get_map_name(CarlaDataProvider.get_map().name)
                self.map_manager.handle_map_data(self.open_drive_map_name, val[1]["opendrive"])
            else:
                self.sensor_manager.handle_sensor_data(key, val[1], timestamp)
        
        return self.vehicle_controller.current_control
    
    def publish_plan(self, timestamp):
        """
        Publish the global plan.
        """
        
        msg = Path()
        msg.header = self.sensor_manager.get_header(timestamp)
        msg.header.frame_id = "map"

        for wp in self._global_plan_world_coord:
            pose = PoseStamped()
            pose.pose.position.x = wp[0].location.x
            pose.pose.position.y = -wp[0].location.y
            pose.pose.position.z = wp[0].location.z
            quaternion = euler2quat(0, 0, -math.radians(wp[0].rotation.yaw))
            pose.pose.orientation.x = quaternion[0]
            pose.pose.orientation.y = quaternion[1]
            pose.pose.orientation.z = quaternion[2]
            pose.pose.orientation.w = quaternion[3]
            msg.poses.append(pose)
        
        self.waypoint_publisher.publish(msg)
    
    def destroy(self):
        """
        Cleanup of all ROS publishers.
        """
        self.spin_thread.join()
        if self.stack_process and self.stack_process.poll() is not None:
            # print("Sending SIGTERM to stack...")
            # os.killpg(os.getpgid(self.stack_process.pid), signal.SIGTERM)
            print("Terminating the autoware stack...")
            self.stack_process.terminate()
            print("Waiting for termination of autoware stack...")
            self.stack_process.wait()
            print("Terminated autoware stack in 5 .. 4 .. 3 .. 2 .. 1")
            time.sleep(5)
        
        if self.lidar_slam_process and self.lidar_slam_process.poll() is not None:
            print("Save the recorded lidar point cloud...")
            print(subprocess.run(["ros2", "service", "call", "/map_save", "std_srvs/Empty"], capture_output=True, text=True))
            print("Terminating the lidar slam stack...")
            self.lidar_slam_process.terminate()
            print("Waiting for termination of lidar slam stack...")
            self.lidar_slam_process.wait()
            print("Terminated lidar slam stack in 5 .. 4 .. 3 .. 2 .. 1")
            time.sleep(5)
        
        if self.carla_gnss_process and self.carla_gnss_process.poll() is not None:
            print("Terminating the carla gnss stack...")
            self.carla_gnss_process.terminate()
            print("Waiting for termination of carla gnss stack...")
            self.carla_gnss_process.wait()
            print("Terminated autoware stack in 5 .. 4 .. 3 .. 2 .. 1")
            time.sleep(5)
    
    def get_map_name(self, map_full_name):
        if map_full_name is None:
            return None
        name_start_index = map_full_name.rfind("/")
        if name_start_index == -1:
            name_start_index = 0
        else:
            name_start_index = name_start_index + 1
        
        return map_full_name[name_start_index:len(map_full_name)]
