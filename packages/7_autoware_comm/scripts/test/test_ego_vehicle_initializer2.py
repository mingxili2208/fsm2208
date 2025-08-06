"""
This module provides a ROS autonomous agent interface to control the ego vehicle via a ROS stack.
"""
import os
import cv2
import time
import math
import numpy
import numpy as np
import signal
import datetime
import threading
import subprocess
import trans_utils as trans
import subprocess
import os
import signal
import carla
import rclpy
from rclpy.qos import QoSProfile
from cv_bridge import CvBridge

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
def compass_to_yaw(compass_rad):
    """
    Converts CARLA compass reading (radians, 0-2pi, clockwise, 0 is North)
    to a standard yaw angle (radians, -pi to pi, counter-clockwise, 0 is East).
    """
    # 1. 将顺时针转为逆时针
    yaw = -compass_rad
    
    # 2. 将参考系从"北"校正到"东" (逆时针旋转90度，即加上 pi/2)
    yaw += np.pi / 2
    
    # 3. 将结果归一化到 [-pi, pi] 范围
    if yaw > np.pi:
        yaw -= 2 * np.pi
    elif yaw < -np.pi:
        yaw += 2 * np.pi
        
    return yaw

class EgoVehicleInit(AutonomousAgent):
    """
    Base class for ROS-based stacks.

    Derive from it and implement the sensors() method.

    Please define OP_AGENT_ROOT in your environment.
    The stack is started by executing ${OP_AGENT_ROOT}/start.sh if CONTROL_MODE is 'autoware'.

    The sensor data is published on similar topics as with the carla-ros-bridge.
    You can find details about the utilized datatypes there.

    This agent expects a roscore to be running.
    """

    speed = None
    current_control = None
    stack_process = None
    carla_gnss_process = None
    lidar_slam_process = None
    current_map_name = None
    step_mode_possible = None
    vehicle_info_publisher = None
    global_plan_published_time = None
    start_script = None
    manual_data_debug = False
    counter = 0
    open_drive_map_data = None
    open_drive_map_name = None
    steering_factor = 0.45
    reverse_steering_factor = 1.7
    max_steer_angle = 0.7
    wheelbase = 2.875
    lidar_freq = 11
    camera_freq = 11
    imu_freq = 200 if os.environ["RUNNING_MODE"] == "record" else 50
    can_freq = 50
    ego_vehicle_transform_freq = 50
    gnss_freq = 2

    def setup(self, path_to_conf_file):
        """
        Setup agent.
        """
        self.track = Track.MAP
        self.local_host = os.environ["SIMULATOR_LOCAL_HOST"]
        self.port = int(os.environ["SIMULATOR_PORT"])
        self.remote_connection = eval(os.environ["REMOTE_CONNECTION"])
        self.agent_role_name = os.environ["AGENT_ROLE_NAME"].replace("-", "_")
        self.bridge_mode = os.environ["OP_BRIDGE_MODE"]
        self.topic_base = "" if os.environ["CONTROL_MODE"].lower() == "autoware" else "/carla/{}".format(self.agent_role_name)
        self.topic_waypoints = self.topic_base + "/waypoints"
        self.stack_thread = None
        self.counter = 0
        self.bridge = CvBridge()
        self.center_camera = None
        self.back_camera = None
        self.bev_camera = None
        self.open_drive_map_name = None
        self.open_drive_map_data = None
        self.spawn_point_initialized = False
        self.lidar_attributes_map = {}

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
            if "0516" in os.environ["FREE_MAP_NAME"]:
                self.carla_gnss_start_script = f"{os.environ['OP_BRIDGE_ROOT']}/op_scripts/fsm_lab_simulation/run_carla_gnss_ros2.sh"
                if not os.path.exists(self.carla_gnss_start_script):
                    raise IOError("File '{}' defined by OP_AGENT_ROOT invalid.".format(self.carla_gnss_start_script))
        
        # Get the lidar_slam_start_script from environment if 'RUNNING_MODE' is 'record'.
        if os.environ["RUNNING_MODE"] == "record":
            self.lidar_slam_start_script = f"{os.environ['OP_BRIDGE_ROOT']}/op_scripts/fsm_lab_simulation/run_lidar_slam_ros2.sh"
            if not os.path.exists(self.lidar_slam_start_script):
                raise IOError("File '{}' defined by OP_AGENT_ROOT invalid.".format(self.lidar_slam_start_script))
        
        # Initialize ros2 node.
        #rclpy.init(args=None)
        self.ros2_node = rclpy.create_node(self.agent_role_name.replace("-", "_"))

        self.clock_publisher = self.ros2_node.create_publisher(Clock, f"{self.topic_base}/clock", 10)
        obj_clock = Clock()
        obj_clock.clock = Time(sec=0)
        self.clock_publisher.publish(obj_clock)

        self.timestamp = None
        self.speed = 0
        # Publish global path every 2 seconds.
        self.global_plan_published_time = 0

        self.spawn_point_initialization_status_publisher = None
        self.spawn_point_initialization_status_publisher_timer = None

        self.auto_velocity_status_publisher = None if os.environ["CONTROL_MODE"] == "autoware" else True
        self.auto_steering_status_publisher = None if os.environ["CONTROL_MODE"] == "autoware" else True
        self.auto_gear_status_publisher = None if os.environ["CONTROL_MODE"] == "autoware" else True
        self.auto_control_mode_publisher = None if os.environ["CONTROL_MODE"] == "autoware" else True
        
        self.vehicle_status_publisher = None
        self.vehicle_twist_publisher = None
        self.vehicle_imu_publisher = None
        self.map_file_publisher = None
        self.perception_cloud_publisher = None
        self.localization_cloud_publisher = None
        self.sensing_cloud_publisher = None
        self.current_map_name = None
        self.step_mode_possible = False

        self.publisher_map = {}
        self.id_to_lider_frame_id = {}
        self.id_to_sensor_type_map = {}
        self.id_to_camera_info_map = {}
        self.cv_bridge = CvBridge()

        # Get the ego-vehicle.
        self.ego_vehicle = self.get_ego_vehicle()
        
        # Get the ego-vehicle control.
        self.current_control = carla.VehicleControl()

        # Initialize all subscribers.
        self.init_subscribers()

        # Initialize all publishers except for clock.
        self.init_publishers()
        
        # Spin the ROS2 node loop.
        self.spin_thread = threading.Thread(target=rclpy.spin, args=(self.ros2_node,))
        self.spin_thread.start()
    
    def lidar_slam_start(self):
        # 使用 preexec_fn=os.setsid 来创建一个新的进程组
        self.lidar_slam_process = subprocess.Popen(
            ["x-terminal-emulator", "-e", f"{self.lidar_slam_start_script}"],
            preexec_fn=os.setsid
        )
    def carla_gnss_start(self):
        # 从环境变量获取SANDBOX_ROOT
        sandbox_root = os.environ.get('SANDBOX_ROOT')
        
        # 使用Python生成时间戳
        timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        
        # 构建日志文件路径
        log_dir = os.path.join(sandbox_root, "Logs", "gnss")
        os.makedirs(log_dir, exist_ok=True)
        log_file = os.path.join(log_dir, f"gnss_log_{timestamp}.log")
        
        # 确保命令在bash中执行，并正确重定向所有输出
        bash_command = f"exec {self.carla_gnss_start_script} > {log_file} 2>&1"
        
        # 在终端中执行命令
        self.carla_gnss_process = subprocess.Popen(
            ["x-terminal-emulator", "-e", "bash", "-c", bash_command],
            preexec_fn=os.setsid
        )
    # def carla_gnss_start(self):
    #     log_file = "/home/cityu-fsm-lab-carla/lmx/Logs/gnss/gnss_log.log"
    #     command = f"{self.carla_gnss_start_script} > {log_file} >&1"

    #     # 使用 preexec_fn=os.setsid 来创建一个新的进程组
    #     self.carla_gnss_process = subprocess.Popen(
    #         ["x-terminal-emulator", "-e", command],
    #         preexec_fn=os.setsid
    #     )
        
    def terminate_processes(self):
        # 在上层函数关闭时，发送信号终止所有子进程
        if hasattr(self, 'lidar_slam_process'):
            os.killpg(os.getpgid(self.lidar_slam_process.pid), signal.SIGTERM)
        
        if hasattr(self, 'carla_gnss_process'):
            os.killpg(os.getpgid(self.carla_gnss_process.pid), signal.SIGTERM)
    # def lidar_slam_start(self):
    #     self.lidar_slam_process = subprocess.Popen(["x-terminal-emulator", "-e", f"{self.lidar_slam_start_script}"])
    
    # def carla_gnss_start(self):
    #     log_file = "/home/cityu-fsm-lab-carla/lmx/Docs/packages/autoware_comm/log/gnss_log.log"

    #     command = f"{self.carla_gnss_start_script} > {log_file} >&1"

    #self.carla_gnss_process = subprocess.Popen(["x-terminal-emulator", "-e", command])

    #self.carla_gnss_process = subprocess.Popen(["x-terminal-emulator", "-e", "bash", "-c", command])

    #self.carla_gnss_process = subprocess.Popen(["x-terminal-emulator", "-e", f"{self.carla_gnss_start_script}"])
    
    def auto_init_local_agent(self, role_name, map_name, waypoints_topic_name, enable_explore):
        print("Executing stack...", role_name, map_name)
        auto_start_script = self.auto_start_script + " " + role_name + " " + map_name + " " + enable_explore + " " + waypoints_topic_name
        # self.stack_process = subprocess.Popen(auto_start_script, shell=True, preexec_fn=os.setpgrp)
        self.stack_process = subprocess.Popen(["x-terminal-emulator", "-e", f"{auto_start_script}"])
    
    def sensors(self):
        self.lidar_publish_prev_time = datetime.datetime.now()
        self.camera_publish_prev_time = {
            "Center": datetime.datetime.now(),
            "Back": datetime.datetime.now(),
            "Bev": datetime.datetime.now(),
        }
        self.imu_publish_prev_time = datetime.datetime.now()
        self.can_publish_prev_time = datetime.datetime.now()
        self.gnss_publish_prev_time = datetime.datetime.now()
        self.ego_vehicle_transform_publish_prev_time = datetime.datetime.now()

        sensors = [
            {"type": "sensor.camera.rgb", "x": 0.7, "y": 0.0, "z": 1.6, "roll": 0.0, "pitch": 0.0, "yaw": 0.0, "width": 1280, "height": 720, "fov": 100, "id": "Center"},
            {"type": "sensor.camera.rgb", "x": -1.4, "y": 0.0, "z": 1.6, "roll": 0.0, "pitch": 0.0, "yaw": 180, "width": 1280, "height": 720, "fov": 100, "id": "Back"},
            {"type": "sensor.camera.rgb", "x": 0.0, "y": 0.0, "z": 30.0, "roll": 0.0, "pitch": -90.0, "yaw": 0.0, "width": 1280, "height": 720, "fov": 100, "id": "Bev"},
            {"type": "sensor.lidar.ray_cast", "x": 0.0, "y": 0.0, "z": 2.6, "roll": 0.0, "pitch": 0.0, "yaw": -90, "id": "LIDAR-Top", "frame_id": "velodyne_top"},
            {"type": "sensor.other.gnss", "x": 0.0, "y": 0.0, "z": 1.6, "id": "GPS"},
            {"type": "sensor.opendrive_map", "reading_frequency": 1, "id": "OpenDRIVE"},
            {"type": "sensor.speedometer", "reading_frequency": 10, "id": "speed"},
            {"type": "sensor.other.imu", "x": 0.0, "y": 0.0, "z": 1.6, "roll": 0.0, "pitch": 0.0, "yaw": 0.0, "id": "IMU"},
        ]

        return sensors
    
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
    
    def run_step(self, input_data, timestamp):
        """
        Execute one step of navigation.
        """

        # Check the remote connection status.
        while self.remote_connection and not self.spawn_point_initialized:
            print("Waiting for remote vehicle's initialization pose...")

        town_map_name = self.get_map_name(CarlaDataProvider.get_map().name)
        if self.stack_process is None and town_map_name is not None and self.open_drive_map_name is not None:
            self.write_opendirve_map_file(self.open_drive_map_name, self.open_drive_map_data)
            if os.environ["CONTROL_MODE"] == "autoware":
                if self.carla_gnss_process is None and "fsm_lab".lower() in os.environ["FREE_MAP_NAME"]:
                    self.carla_gnss_start()
                if self.carla_gnss_process is None and "0516" in os.environ["FREE_MAP_NAME"]:
                    self.carla_gnss_start()
                if self.bridge_mode == "free" or self.bridge_mode == "srunner":
                    self.auto_init_local_agent(self.agent_role_name, town_map_name, '', 'true')
                elif self.bridge_mode == "leaderboard":
                    self.auto_init_local_agent(self.agent_role_name, town_map_name, self.topic_waypoints, "false")
                else:
                    self.auto_init_local_agent(self.agent_role_name, town_map_name, self.topic_waypoints, "false")
                
            if self._global_plan_world_coord:
                self.publish_plan()
        
        if self.lidar_slam_process is None and os.environ["RUNNING_MODE"] == "record":
            self.lidar_slam_start()
        
        self.timestamp = timestamp
        seconds = int(self.timestamp)
        nanoseconds = int((self.timestamp - int(self.timestamp)) * 1000000000.0)
        obj_clock = Clock()
        obj_clock.clock = Time(sec=seconds, nanosec=nanoseconds)

        self.clock_publisher.publish(obj_clock)

        # Check if stack is still running.
        if self.stack_process and self.stack_process.poll() is not None:
            raise RuntimeError("Stack exited with: {} {}".format(self.stack_process.returncode, self.stack_process.communicate()[0]))
        
        # Wait 2 second before publish the global path.
        if self._global_plan_world_coord and (self.timestamp - self.global_plan_published_time) > 2.0:
            self.global_plan_published_time = self.timestamp
            self.publish_plan()
        
        # Publish the ego vehicle transform.
        self.publish_ego_vehicle_transform()
        self.publish_initial_pose()
        self.publish_odometry()
        # Publish data of all sensors.
        for key, val in input_data.items():
            sensor_type = self.id_to_sensor_type_map[key]
            if self.manual_data_debug:
                print(key)
            
            if sensor_type == "sensor.camera.rgb":
                self.publish_camera(key, val[1])
            elif sensor_type == "sensor.opendrive_map":
                self.open_drive_map_data = val[1]["opendrive"]
                self.open_drive_map_name = self.get_map_name(CarlaDataProvider.get_map().name)
                self.publish_hd_map(key, val[1], self.open_drive_map_name)
            elif sensor_type == "sensor.other.gnss":
                self.publish_gnss(key, val[1])
            elif sensor_type == "sensor.lidar.ray_cast":
                self.publish_lidar(key, val[1], self.id_to_lider_frame_id[key])
            elif sensor_type == "sensor.speedometer":
                self.publish_can(key, val[1])
            elif sensor_type == "sensor.other.imu":
                self.publish_imu(key, val[1])
            elif self.manual_data_debug:
                print("Additional Sensor !!")
                print(key)
        
        return self.current_control
    
    def destroy(self):
        """
        Cleanup of all ROS publishers.
        """
        rclpy.shutdown()
        self.spin_thread.join()
        self.terminate_processes()
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
            self.stack_process.wait()
            print("Terminated autoware stack in 5 .. 4 .. 3 .. 2 .. 1")
            time.sleep(5)
    


    ###############################################################################################################
    #
    #   Please write all the ROS2 subscribers of the ego-vehicle in the following function.
    #
    ###############################################################################################################
    def init_subscribers(self):
        # Subscribe the initalization spawn point for the vehicle.
        if self.remote_connection:
            self.init_spawn_point_sub = self.ros2_node.create_subscription(
                PoseStamped, f"/init/{self.agent_role_name}/transform", self.on_init_spawn_point_callback, 1,
            )
        
        # Subscribe the vehicle control topic.
        if os.environ["CONTROL_MODE"] == "autoware":
            self.auto_vehicle_control_sub = self.ros2_node.create_subscription(
                AckermannControlCommand, "/control/command/control_cmd", self.on_auto_vehicle_control_callback, qos_profile=QoSProfile(depth=1),
            )
            self.auto_vehicle_initialpose_sub = self.ros2_node.create_subscription(
                PoseWithCovarianceStamped, "/initialpose", self.on_auto_vehicle_initialpose_callback, 1,
            )
        elif os.environ["CONTROL_MODE"] == "teleop":
            self.vehicle_control_sub = self.ros2_node.create_subscription(
                TwistStamped, f"{self.topic_base}/carla_op_controller_cmd", self.on_vehicle_control_callback, 1,
            )
        elif os.environ["CONTROL_MODE"] == "follow":
            self.real_vehicle_transform_sub = self.ros2_node.create_subscription(
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
        
        # Subsribe the vehicle camera topic.
        self.center_camera_sub = self.ros2_node.create_subscription(
            Image, f"{self.topic_base}/sensing/camera/traffic_light/image_raw", self.center_camera_callback, 1
        )
        self.back_camear_sub = self.ros2_node.create_subscription(
            Image, f"{self.topic_base}/sensing/camera/vehicle_back/image_raw", self.back_camera_callback, 1
        )
        self.bev_camera_sub = self.ros2_node.create_subscription(
            Image, f"{self.topic_base}/sensing/camera/vehicle_bev/image_raw", self.bev_camera_callback, 1
        )
    


    ###############################################################################################################
    #
    #   Please write all the ROS2 publishers of the ego-vehicle in the following function.
    #
    ###############################################################################################################
    def init_publishers(self):
        # Publish the spawn point initialization status.
        if self.remote_connection:
            self.spawn_point_initialization_status_publisher = self.ros2_node.create_publisher(Bool, f"/init/{self.agent_role_name}/spawn_point_initialization_status", 1)
            self.spawn_point_initialization_status_publisher_timer = self.ros2_node.create_timer(0.001, self.on_init_spawn_point_feedback_callback)

        # Publish the waypoint of the ego-vehicle.
        self.waypoint_publisher = self.ros2_node.create_publisher(Path, self.topic_waypoints, 1)
        self.initial_pose_publisher_ = self.ros2_node.create_publisher(
            PoseStamped, # <--- 已修正为正确的类型
            '/initial_pose', # <--- 已修正为正确的话题名称
            10)
        
        # self.pose_publish_timer = self.ros2_node.create_timer(1.0, self.publish_initial_pose_once)

        # Publish the transform of the ego-vehicle.
        self.ego_vehicle_transform_publisher = self.ros2_node.create_publisher(PoseWithCovarianceStamped, f"{self.topic_base}/vehicle_transform", 1)

        # Publish all the sensors of the ego-vehicle.
        for sensor in self.sensors():
            self.id_to_sensor_type_map[sensor["id"]] = sensor["type"]
            if sensor["type"] == "sensor.camera.rgb":
                if sensor["id"] == "Center":
                    self.publisher_map[sensor["id"]] = self.ros2_node.create_publisher(
                        Image, f"{self.topic_base}/sensing/camera/traffic_light/image_raw", 1,
                    )
                    self.id_to_camera_info_map[sensor["id"]] = self.build_camera_info(sensor)
                    self.publisher_map[sensor["id"] + "_info"] = self.ros2_node.create_publisher(
                        CameraInfo, f"{self.topic_base}/sensing/camera/traffic_light/camera_info", 1,
                    )
                else:
                    self.publisher_map[sensor["id"]] = self.ros2_node.create_publisher(
                        Image, f"{self.topic_base}/sensing/camera/vehicle_{sensor['id'].lower()}/image_raw", 1,
                    )
                    self.id_to_camera_info_map[sensor["id"]] = self.build_camera_info(sensor)
                    self.publisher_map[sensor["id"] + "_info"] = self.ros2_node.create_publisher(
                        CameraInfo, f"{self.topic_base}/sensing/camera/vehicle_{sensor['id'].lower()}/camera_info", 1,
                    )
            elif sensor["type"] == "sensor.lidar.ray_cast":
                self.id_to_lider_frame_id[sensor["id"]] = sensor["frame_id"]
                if os.environ["RUNNING_MODE"] == "record":
                    self.lidar_attributes_map[sensor["id"]] = sensor.get("attributes", {})
                    self.sensing_cloud_publisher = self.ros2_node.create_publisher(
                        PointCloud2, f"/velodyne_points", 10,
                    )
                else:
                    self.sensing_cloud_publisher = self.ros2_node.create_publisher(
                        PointCloud2, f"{self.topic_base}/carla_pointcloud", 10,
                    )
            elif sensor["type"] == "sensor.other.gnss":
                self.publisher_map[sensor["id"]] = self.ros2_node.create_publisher(
                    NavSatFix, f"{self.topic_base}/carla_nav_sat_fix", 1,
                )
            elif sensor["type"] == "sensor.speedometer":
                if not self.vehicle_status_publisher:
                    self.vehicle_status_publisher = self.ros2_node.create_publisher(
                        Odometry, f"{self.topic_base}/odo", 1,
                    )
                # Initialize the autoware controller topic publisher.
                if not self.auto_velocity_status_publisher:
                    self.auto_velocity_status_publisher = self.ros2_node.create_publisher(
                        VelocityReport, "/vehicle/status/velocity_status", 1,
                    )
                if not self.auto_steering_status_publisher:
                    self.auto_steering_status_publisher = self.ros2_node.create_publisher(
                        SteeringReport, "/vehicle/status/steering_status", 1,
                    )
                if not self.auto_gear_status_publisher:
                    self.auto_gear_status_publisher = self.ros2_node.create_publisher(
                        GearReport, "/vehicle/status/gear_status", 1,
                    )
                if not self.auto_control_mode_publisher:
                    self.auto_control_mode_publisher = self.ros2_node.create_publisher(
                        ControlModeReport, "/vehicle/status/control_mode", 1,
                    )
            elif sensor["type"] == "sensor.other.imu":
                if not self.vehicle_imu_publisher:
                    if os.environ["RUNNING_MODE"] == "record":
                        self.vehicle_imu_publisher = self.ros2_node.create_publisher(
                            Imu, f"/sensing/imu/tamagawa/imu_raw", 1,
                        )
                    else:
                        self.vehicle_imu_publisher = self.ros2_node.create_publisher(
                            Imu, f"{self.topic_base}/sensing/imu/tamagawa/imu_raw", 1,
                        )
            elif sensor["type"] == "sensor.opendrive_map":
                if not self.map_file_publisher:
                    self.map_file_publisher = self.ros2_node.create_publisher(
                        String, f"{self.topic_base}/carla/map_file", 1,
                    )
            else:
                raise TypeError("Invalid sensor type: {}".format(sensor["type"]))
    


    ###############################################################################################################
    #
    #   Please write all the callback functions in the following area.
    #
    ###############################################################################################################
    def on_init_spawn_point_callback(self, data: PoseStamped):
        """
        Callback if a initial spawn point msg is received.
        """

        pose = data.pose
        carla_pose_transform = trans.ros_pose_to_carla_transform(pose)
        if self.remote_connection and not self.spawn_point_initialized:
            if self.ego_vehicle is not None:
                self.ego_vehicle.set_transform(carla_pose_transform)
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

        pose = data.pose.pose
        pose.position.z +=2.0 #0.0016 #2.0
        # pose.position.x += 2.0 #0.0016 #0.0
        # pose.position.y += 0.0 #0.0016 #0.0
        carla_pose_transform = trans.ros_pose_to_carla_transform(pose)
        if self.ego_vehicle is not None:
            self.ego_vehicle.set_transform(carla_pose_transform)
        else:
            print(f"[on_auto_vehicle_initialpose_callback] Can't find Ego Vehicle named {self.agent_role_name}!! Make sure it is spawned!")
    
    def on_auto_vehicle_control_callback(self, data):
        """
        Callback if an autoware vehicle control command is received.
        """

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
        # print(
        #     "acc:", format(data.longitudinal.acceleration),
        #     " speed:", format(data.longitudinal.speed, ".4f"),
        #     " pspeed:", format(self.speed, ".4f"),
        #     " sdiff:", format(speed_diff, ".4f"),
        #     " gear:", format(cmd.gear, ".4f"),
        #     " reverse:", str(cmd.reverse),
        # )

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
        self.step_mode_possible = True

        # Original controller.
        # cmd = carla.VehicleControl()
        # cmd.steer = (-data.lateral.steering_tire_angle / self.max_steer_angle) * self.steering_factor
        # speed_diff = data.longitudinal.speed - self.speed
        # if speed_diff > 0:
        #     cmd.throttle = 0.75
        #     cmd.brake = 0.0
        # elif speed_diff < 0.0:
        #     cmd.throttle = 0.0
        #     if data.longitudinal.speed <= 0.0:
        #         cmd.brake = 0.75
        #     elif speed_diff > -1:
        #         cmd.brake = 0.0
        #     else:
        #         cmd.brake = 0.01
        
        # self.current_control = cmd
        # self.step_mode_possible = True
    
    def on_vehicle_control_callback(self, data):
        """
        Callback if a vehicle control command is received (not from autoware).
        """

        cmd = carla.VehicleControl()
        cmd.throttle = data.twist.linear.x / 100.0
        cmd.steer = data.twist.angular.z / 100.0
        cmd.brake = data.twist.linear.y / 100.0
        self.current_control = cmd
        self.step_mode_possible = True
    
    def on_real_vehicle_transform_skip_control_callback(self, data: PoseStamped):
        """
        Callback to skip to the transform location according to the real vehicle.
        """

        pose = data.pose
        carla_pose_transform = trans.ros_pose_to_carla_transform(pose)
        if self.ego_vehicle is not None:
            self.ego_vehicle.set_transform(carla_pose_transform)
        else:
            print(f"[on_real_vehicle_transform_skip_control_callback] Can't find Ego Vehicle named {self.agent_role_name}!! Make sure it is spawned!")

    def center_camera_callback(self, data):
        """
        Callback to get the center camera image.
        """

        temp_camera = self.rosimg_to_cvimg(data)
        temp_camera = cv2.flip(cv2.rotate(temp_camera, cv2.ROTATE_90_COUNTERCLOCKWISE), 0)
        temp_camera = cv2.resize(temp_camera, (200, 300))
        self.center_camera = temp_camera
    
    def back_camera_callback(self, data):
        """
        Callback to get the back camera image.
        """

        temp_camera = self.rosimg_to_cvimg(data)
        temp_camera = cv2.rotate(temp_camera, cv2.ROTATE_90_COUNTERCLOCKWISE)
        temp_camera = cv2.resize(temp_camera, (200, 300))
        self.back_camera = temp_camera
    
    def bev_camera_callback(self, data):
        """
        Callback to get the bev camera image.
        """

        temp_camera = self.rosimg_to_cvimg(data)
        temp_camera = cv2.flip(cv2.rotate(temp_camera, cv2.ROTATE_90_COUNTERCLOCKWISE), 0)
        temp_camera = cv2.resize(temp_camera, (300, 600))
        self.bev_camera = temp_camera



    ###############################################################################################################
    #
    #   Please write all the publisher handling functions in the following area.
    #
    ###############################################################################################################
    def publish_plan(self):
        """
        Publish the global plan.
        """
        
        msg = Path()
        msg.header = self.get_header()
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
    def publish_lidar(self, sensor_id, data, frame_id="velodyne_top"):
        """
        Function to publish lidar data with x, y, z, intensity, ring, and time.
        This version uses vectorized NumPy operations for high performance.
        """
        
        # 1. 频率检查 (与您原代码相同)
        if hasattr(self, 'lidar_freq') and self.checkFrequency(self.lidar_publish_prev_time, self.lidar_freq):
            return
        self.lidar_publish_prev_time = datetime.datetime.now()

        # 2. 获取LiDAR属性 (与您原代码相同)
        if not hasattr(self, 'lidar_attributes_map'):
            print("Error: Lidar attributes map not found.")
            return

        lidar_attributes = self.lidar_attributes_map.get(sensor_id, {})
        try:
            channels = int(lidar_attributes.get("channels", "32"))
            rotation_frequency = float(lidar_attributes.get("rotation_frequency", "20.0"))
            upper_fov = float(lidar_attributes.get("upper_fov", "10.0"))
            lower_fov = float(lidar_attributes.get("lower_fov", "-30.0"))
        except ValueError as e:
            print(f"Error: Invalid Lidar attribute for {sensor_id}: {e}")
            return

        # --- 核心：向量化计算开始 ---

        # 3. 从缓冲区读取数据并重塑
        # 将一维的字节流数据转换为 (N, 4) 的Numpy数组，N是点的数量
        lidar_data = numpy.frombuffer(data, dtype=numpy.float32)
        if lidar_data.shape[0] % 4 != 0:
            print(f"Cannot Reshape LIDAR Data buffer for {frame_id}")
            return
        lidar_data = numpy.reshape(lidar_data, (-1, 4)) # 列为 [x_carla, y_carla, z_carla, intensity]

        # 4. 坐标系转换 (一次性对所有点操作)
        # 根据我们之前的分析 (CARLA左手系 -> ROS右手系, 且yaw=-90度)
        # ROS_x = CARLA_y, ROS_y = CARLA_x, ROS_z = CARLA_z
        x_ros = lidar_data[:, 1]
        y_ros = lidar_data[:, 0]
        z_ros = lidar_data[:, 2]
        intensity = lidar_data[:, 3]

        # 5. 向量化计算 'ring'
        # 计算每个点到Z轴的水平距离
        horizontal_dist = numpy.sqrt(x_ros**2 + y_ros**2)
        # 计算每个点的垂直角度
        vertical_angle = numpy.arctan2(z_ros, horizontal_dist)
        vertical_angle_deg = numpy.degrees(vertical_angle)
        
        fov_range = upper_fov - lower_fov
        if abs(fov_range) < 1e-6: fov_range = 360.0 # 防止除以零

        # 将垂直角度归一化并映射到ring ID
        normalized_angle = (vertical_angle_deg - lower_fov) / fov_range
        ring = (normalized_angle * (channels - 1)).astype(numpy.uint16)
        # 使用clip确保ring值在有效范围内 [0, channels-1]
        numpy.clip(ring, 0, channels - 1, out=ring)

        # 6. 向量化计算 'time' (每个点的相对时间戳)
        scan_duration = 1.0 / rotation_frequency
        # 计算每个点的水平方位角
        azimuth = numpy.arctan2(y_ros, x_ros)
        # 将方位角从[-pi, pi]映射到[0, 2*pi]，然后归一化到扫描周期内
        time = ((azimuth + numpy.pi) / (2 * numpy.pi)) * scan_duration
        time = time.astype(numpy.float32)

        # 7. 创建结构化数组以封装所有点数据
        # 这是为create_cloud函数准备数据的最高效方式，避免了Python列表的开销
        processed_points = numpy.empty(len(x_ros), dtype=[
            ('x', numpy.float32),
            ('y', numpy.float32),
            ('z', numpy.float32),
            ('intensity', numpy.float32),
            ('ring', numpy.uint16),
            ('time', numpy.float32)
        ])
        processed_points['x'] = x_ros
        processed_points['y'] = y_ros
        processed_points['z'] = z_ros
        processed_points['intensity'] = intensity
        processed_points['ring'] = ring
        processed_points['time'] = time

        # --- 向量化计算结束 ---

        # 8. 定义PointCloud2的字段 (fields)
        # 字段的名称、偏移量、数据类型必须与上面的结构化数组严格对应
        fields = [
            PointField(name='x',         offset=0,  datatype=PointField.FLOAT32, count=1),
            PointField(name='y',         offset=4,  datatype=PointField.FLOAT32, count=1),
            PointField(name='z',         offset=8,  datatype=PointField.FLOAT32, count=1),
            PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
            PointField(name='ring',      offset=16, datatype=PointField.UINT16,  count=1),
            # 注意: time的偏移量是16 + 2(uint16) = 18
            PointField(name='time',      offset=18, datatype=PointField.FLOAT32, count=1) 
        ]

        # 9. 创建并发布消息 (与您原代码相同)
        header = self.get_header()
        if frame_id == "velodyne_top" and os.environ.get("RUNNING_MODE") == "record":
            header.frame_id = "velodyne"
        else:
            header.frame_id = frame_id

        try:
            # 将结构化数组直接传递给create_cloud函数
            msg = create_cloud(header, fields, processed_points)
            if hasattr(self, 'sensing_cloud_publisher') and self.sensing_cloud_publisher is not None:
                self.sensing_cloud_publisher.publish(msg)
        except Exception as e:
            print(f"Error creating PointCloud2 for {sensor_id}: {e}")
    def publish_gnss(self, sensor_id, data):
        """
        Function to publish gnss data.
        """

        if self.checkFrequency(self.gnss_publish_prev_time, self.gnss_freq) == True:
            return
        
        self.gnss_publish_prev_time = datetime.datetime.now()

        msg = NavSatFix()
        msg.header = self.get_header()
        msg.header.frame_id = "gnss_link"
        msg.latitude = data[0]
        msg.longitude = data[1]
        msg.altitude = data[2]
        msg.status.status = NavSatStatus.STATUS_SBAS_FIX
        msg.status.service = NavSatStatus.SERVICE_GPS | NavSatStatus.SERVICE_GLONASS | NavSatStatus.SERVICE_COMPASS | NavSatStatus.SERVICE_GALILEO
        self.publisher_map[sensor_id].publish(msg)

    def publish_camera(self, sensor_id, data):
        """
        Function to publish camera data.
        """
        camera_publish_prev_time = self.camera_publish_prev_time[sensor_id]
        if self.checkFrequency(camera_publish_prev_time, self.camera_freq) == True:
            return
        
        self.camera_publish_prev_time[sensor_id] = datetime.datetime.now()

        msg = self.cv_bridge.cv2_to_imgmsg(data, encoding="bgra8")
        msg.header = self.get_header()
        if sensor_id == "Center":
            msg.header.frame_id = "traffic_light_left_camera/camera_link"
        else:
            msg.header.frame_id = f"vehicle_{sensor_id.lower()}/camera_link"

        cam_info = self.id_to_camera_info_map[sensor_id]
        cam_info.header = msg.header
        self.publisher_map[sensor_id + "_info"].publish(cam_info)
        self.publisher_map[sensor_id].publish(msg)
    
    def publish_imu(self, sensor_id, data):
        """
        Publish IMU data.
        """

        if self.checkFrequency(self.imu_publish_prev_time, self.imu_freq) == True:
            return
        
        self.imu_publish_prev_time = datetime.datetime.now()

        imu_msg = Imu()
        imu_msg.header = self.get_header()
        imu_msg.header.frame_id = "tamagawa/imu_link"

        imu_msg.linear_acceleration.x = data[0]
        imu_msg.linear_acceleration.y = -data[1]
        imu_msg.linear_acceleration.z = data[2]

        imu_msg.angular_velocity.x = -data[3]
        imu_msg.angular_velocity.y = data[4]
        imu_msg.angular_velocity.z = -data[5]

        #imu_rotation = data[6]
        ros_pose = trans.carla_transform_to_ros_pose(self.get_ego_vehicle().get_transform())
        imu_msg.orientation = ros_pose.orientation
        # imu_yaw=compass_to_yaw(data[6])
        # # print(f"imu_rotation, which is compass: {imu_rotation}")
        # quaternion = euler2quat(0, 0, imu_yaw)
        # imu_msg.orientation.x = quaternion[0]
        # imu_msg.orientation.y = quaternion[1]
        # imu_msg.orientation.z = quaternion[2]
        # imu_msg.orientation.w = quaternion[3]

        # imu_msg.orientation.x = imu_rotation[0]
        # imu_msg.orientation.y = imu_rotation[1]
        # imu_msg.orientation.z = imu_rotation[2]
        # imu_msg.orientation.w = 1

        self.vehicle_imu_publisher.publish(imu_msg)
    
    # def publish_can(self, sensor_id, data):
    #     """
    #     Publish can data.
    #     """

    #     if self.checkFrequency(self.can_publish_prev_time, self.can_freq) == True:
    #         return
        
    #     self.can_publish_prev_time = datetime.datetime.now()
    #     self.speed = data["speed"]
    #     pose_msg = PoseWithCovariance()
    #     pose_msg.pose = trans.carla_transform_to_ros_pose(self.get_ego_vehicle().get_transform())
    #     twist_msg = TwistWithCovariance()
    #     twist_msg.twist.linear.x = data["speed"]
    #     # if twist_msg.twist.linear.x < 0.0:
    #     #     twist_msg.twist.linear.x = 0.0
    #     twist_msg.twist.angular.z = -self.current_control.steer
    #     twist_msg.twist.linear.z = 1.0
    #     twist_msg.twist.angular.x = 1.0

    #     odo_msg = Odometry()
    #     odo_msg.header = self.get_header()
    #     odo_msg.pose = pose_msg
    #     odo_msg.twist = twist_msg
    #     self.vehicle_status_publisher.publish(odo_msg)

    #     if os.environ["CONTROL_MODE"] == "autoware":
    #         vel_rep = VelocityReport()
    #         vel_rep.header = self.get_header()
    #         vel_rep.header.frame_id = "base_link"
    #         vel_rep.longitudinal_velocity = data["speed"]
    #         vel_rep.heading_rate = 0.0
    #         self.auto_velocity_status_publisher.publish(vel_rep)

    #         steer_rep = SteeringReport()
    #         if self.current_control.reverse:
    #             steer_rep.steering_tire_angle = (-self.current_control.steer * self.max_steer_angle) / self.reverse_steering_factor
    #         else:
    #             steer_rep.steering_tire_angle = (-self.current_control.steer * self.max_steer_angle) / self.steering_factor
    #         self.auto_steering_status_publisher.publish(steer_rep)

    #         gear_rep = GearReport()
    #         gear_rep.stamp = self.get_header().stamp
    #         if self.current_control.gear == 1:
    #             if self.current_control.reverse:
    #                 gear_rep.report = GearReport.REVERSE
    #             else:
    #                 gear_rep.report = GearReport.DRIVE
    #         else:
    #             gear_rep.report = GearReport.PARK
    #         self.auto_gear_status_publisher.publish(gear_rep)

    #         control_mode_rep = ControlModeReport()
    #         control_mode_rep.stamp = self.get_header().stamp
    #         control_mode_rep.mode = ControlModeReport.AUTONOMOUS
    #         self.auto_control_mode_publisher.publish(control_mode_rep)
    
    def publish_can(self, sensor_id, data):
        """
        这个函数现在只处理与CAN总线相关的原始数据发布，
        例如发布给Autoware的各种状态报告。
        里程计(Odometry)的发布被移到了一个专门的函数中。
        """
        if self.checkFrequency(self.can_publish_prev_time, self.can_freq):
            return
        
        self.can_publish_prev_time = datetime.datetime.now()
        self.speed = data["speed"]

        # --- 发布给 Autoware 的各种状态报告 (这部分逻辑是正确的，予以保留) ---
        if os.environ["CONTROL_MODE"] == "autoware":
            # Velocity Report
            vel_rep = VelocityReport()
            vel_rep.header = self.get_header()
            vel_rep.header.frame_id = "base_link"
            vel_rep.longitudinal_velocity = self.speed
            # heading_rate (偏航角速度) 应该从正确的计算中获取，而不是0.0
            # 我们将在 publish_odometry 中计算它，并在这里复用
            # (为了简化，暂时保留0.0，但理想情况下应该更新)
            vel_rep.heading_rate = 0.0 
            self.auto_velocity_status_publisher.publish(vel_rep)

            # Steering Report
            steer_rep = SteeringReport()
            if self.current_control.reverse:
                steer_rep.steering_tire_angle = (-self.current_control.steer * self.max_steer_angle) / self.reverse_steering_factor
            else:
                steer_rep.steering_tire_angle = (-self.current_control.steer * self.max_steer_angle) / self.steering_factor
            self.auto_steering_status_publisher.publish(steer_rep)

            # Gear Report
            gear_rep = GearReport()
            gear_rep.stamp = self.get_header().stamp
            if self.current_control.gear == 1:
                if self.current_control.reverse:
                    gear_rep.report = GearReport.REVERSE
                else:
                    gear_rep.report = GearReport.DRIVE
            else:
                gear_rep.report = GearReport.PARK
            self.auto_gear_status_publisher.publish(gear_rep)

            # Control Mode Report
            control_mode_rep = ControlModeReport()
            control_mode_rep.stamp = self.get_header().stamp
            control_mode_rep.mode = ControlModeReport.AUTONOMOUS
            self.auto_control_mode_publisher.publish(control_mode_rep)

    def publish_ego_vehicle_transform(self):
        """
        Publish ego vehicle transform.
        """

        if self.checkFrequency(self.ego_vehicle_transform_publish_prev_time, self.ego_vehicle_transform_freq) == True:
            return
        
        ego_vehicle_transform = PoseWithCovariance()
        ego_vehicle_transform.pose = trans.carla_transform_to_ros_pose(self.get_ego_vehicle().get_transform())

        pos_msg = PoseWithCovarianceStamped()
        pos_msg.pose = ego_vehicle_transform
        pos_msg.header = self.get_header()
        self.ego_vehicle_transform_publisher.publish(pos_msg)
    
    def publish_hd_map(self, sensor_id, data, map_name):
        """
        Publish hd map data.
        """

        if self.current_map_name != map_name:
            self.current_map_name = map_name
        if self.map_file_publisher:
            data_msg = String()
            data_msg.data = data["opendrive"]
            self.map_file_publisher.publish(data_msg)

    def publish_odometry(self):
        """
        一个专门的函数，用于获取车辆的真实状态并发布一个正确、健壮的Odometry消息。
        """
        # 从CARLA获取最准确的地面真值数据
        ego_vehicle = self.get_ego_vehicle()
        transform = ego_vehicle.get_transform()
        velocity = ego_vehicle.get_velocity()      # 世界坐标系下的速度矢量
        angular_velocity = ego_vehicle.get_angular_velocity() # 世界坐标系下的角速度矢量

        # -----------------------------------------------------------------
        # 1. 准备 Pose 部分 (与您之前的逻辑相同，是正确的)
        # -----------------------------------------------------------------
        pose_msg = PoseWithCovariance()
        pose_msg.pose = trans.carla_transform_to_ros_pose(transform)
        # 您可以为covariance矩阵填充一些小的对角线值来表示这是一个高精度的位姿
        pose_msg.covariance[0] = 0.01  # x
        pose_msg.covariance[7] = 0.01  # y
        pose_msg.covariance[14] = 0.01 # z
        pose_msg.covariance[21] = 0.001 # roll
        pose_msg.covariance[28] = 0.001 # pitch
        pose_msg.covariance[35] = 0.001 # yaw

        # -----------------------------------------------------------------
        # 2. 准备 Twist 部分 (这是核心修正)
        # -----------------------------------------------------------------
        twist_msg = TwistWithCovariance()
        
        # --- 线速度 (Linear Velocity) ---
        # get_velocity() 返回的是世界坐标系下的速度，我们需要将其转换到车辆的局部坐标系(base_link)下
        # 一个简单且常用的方法是：将世界速度矢量投影到车辆的前进方向矢量上
        forward_vector = transform.get_forward_vector()
        forward_vec_np = np.array([forward_vector.x, forward_vector.y, forward_vector.z])
        velocity_np = np.array([velocity.x, velocity.y, velocity.z])
        
        # twist.linear.x 是车辆在自己前进方向上的速度
        twist_msg.twist.linear.x = np.dot(velocity_np, forward_vec_np)
        # 假设车辆无侧滑和垂直运动
        twist_msg.twist.linear.y = 0.0
        twist_msg.twist.linear.z = 0.0

        # --- 角速度 (Angular Velocity) ---
        # get_angular_velocity() 返回的是世界坐标系下的角速度，也需要转换
        # 幸运的是，对于角速度，从世界坐标系到车辆局部坐标系的转换更直接
        # 我们需要执行从CARLA(左手系)到ROS(右手系)的坐标翻转
        twist_msg.twist.angular.x = -angular_velocity.x # Roll rate
        twist_msg.twist.angular.y = angular_velocity.y  # Pitch rate (注意：这里是正的，因为是绕Y轴转)
        twist_msg.twist.angular.z = -angular_velocity.z # Yaw rate

        # 同样，为twist的covariance填充一些值
        twist_msg.covariance[0] = 0.01  # vx
        twist_msg.covariance[7] = 0.01  # vy
        twist_msg.covariance[35] = 0.01 # wz

        # -----------------------------------------------------------------
        # 3. 组装并发布 Odometry 消息
        # -----------------------------------------------------------------
        odo_msg = Odometry()
        odo_msg.header = self.get_header()
        odo_msg.header.frame_id = "odom"      # 里程计消息的参考系是 odom
        odo_msg.child_frame_id = "base_link"  # 它描述的是 base_link 的状态
        odo_msg.pose = pose_msg
        odo_msg.twist = twist_msg
        
        # 确保publisher存在
        if self.vehicle_status_publisher:
            self.vehicle_status_publisher.publish(odo_msg)

    def publish_initial_pose(self):

        pose_msg = PoseStamped() # <--- 已修正为正确的类型
        pose_msg.header = self.get_header()
        # pose_msg.header.stamp = self.ros2_node.get_clock().now().to_msg()
        pose_msg.header.frame_id = 'map'

        pose_msg.pose=trans.carla_transform_to_ros_pose(self.get_ego_vehicle().get_transform())

        self.initial_pose_publisher_.publish(pose_msg)
        # self.ros2_node.get_logger().info('已成功自动发布初始位姿到 /initial_pose 话题！')


    ###############################################################################################################
    #
    #   Please write all the other processing functions in the following area.
    #
    ###############################################################################################################
    def use_stepping_mode(self):
        """
        Overload this function to use stepping mode.
        """
        return False

    def rosimg_to_cvimg(self, data):
        cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        
        return cv_image

    def checkFrequency(self, prev_time, target_freq):
        time_delta = (datetime.datetime.now() - prev_time).microseconds / 1000000.0
        if 1.0 / time_delta >= target_freq:
            return True
        return False
    
    def write_opendirve_map_file(self, map_name, map_data):
        team_code_path = os.environ["OP_AGENT_ROOT"]
        if not team_code_path or not os.path.exists(team_code_path):
            raise IOError("Path '{}' defined by OP_AGENT_ROOT invalid.".format(team_code_path))
        opendrive_map_path = "{}/hdmaps/{}.xodr".format(team_code_path, map_name)
        os.makedirs(f"{team_code_path}/hdmaps/", exist_ok=True)
        f = open(opendrive_map_path, "w")
        f.write(map_data)
        f.close()
    
    def get_camera_snapshots(self):
        return self.center_camera, self.back_camera, self.bev_camera

    def get_ego_vehicle(self):
        for vehicle in CarlaDataProvider.get_world().get_actors().filter("vehicle.*"):
            if vehicle.attributes["role_name"] == self.agent_role_name:
                return vehicle
        return None
    
    def get_header(self):
        """
        Returns ROS message header.
        """
        
        header = Header()
        seconds = int(self.timestamp)
        nanoseconds = int((self.timestamp - int(self.timestamp)) * 1000000000.0)
        header.stamp = Time(sec=seconds, nanosec=nanoseconds)
        # header.stamp = self.ros2_node.get_clock().now().to_msg()
        return header
    
    def get_map_name(self, map_full_name):
        if map_full_name is None:
            return None
        name_start_index = map_full_name.rfind("/")
        if name_start_index == -1:
            name_start_index = 0
        else:
            name_start_index = name_start_index + 1
        
        return map_full_name[name_start_index:len(map_full_name)]
