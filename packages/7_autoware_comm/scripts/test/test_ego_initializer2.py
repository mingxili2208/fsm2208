#!/usr/bin/env python3

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
import importlib
from abc import ABC, abstractmethod

import carla
import rclpy
from rclpy.qos import QoSProfile
from cv_bridge import CvBridge

from rosgraph_msgs.msg import Clock
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import Header, String, Bool
from builtin_interfaces.msg import Time
from transforms3d.euler import euler2quat
from geometry_msgs.msg import (
    PoseStamped,
    TwistWithCovariance,
    PoseWithCovariance,
    TwistStamped,
    PoseWithCovarianceStamped,
)
from sensor_msgs.msg import Image, PointCloud2, NavSatFix, NavSatStatus, CameraInfo, PointField, Imu
from leaderboard.autoagents.autonomous_agent import AutonomousAgent, Track
from srunner.scenariomanager.carla_data_provider import CarlaDataProvider

# Import trans_utils
try:
    import trans_utils as trans
except ImportError:
    from . import trans_utils as trans

# Conditional imports based on running mode
if os.environ.get("RUNNING_MODE", "normal") == "record":
    from messages.sensors.lidar import create_cloud
else:
    from sensor_msgs_py.point_cloud2 import create_cloud

# Conditional imports based on control mode
if os.environ.get("CONTROL_MODE", "pygame") == "autoware":
    from autoware_auto_control_msgs.msg import AckermannControlCommand
    from autoware_auto_vehicle_msgs.msg import (
        ControlModeReport,
        GearReport,
        SteeringReport,
        TurnIndicatorsReport,
        HazardLightsReport,
        VelocityReport,
    )


def get_entry_point():
    """Returns the entry point class name for the leaderboard."""
    return "EgoVehicleInit"


# Base interface for sensors
class SensorInterface(ABC):
    """Abstract base class for sensor interfaces."""
    
    @abstractmethod
    def initialize(self, ros2_node, topic_base, publisher_map):
        """Initialize the sensor publishers."""
        pass
    
    @abstractmethod
    def process_data(self, sensor_id, data, timestamp):
        """Process sensor data and publish it."""
        pass


# Camera sensor handler
class CameraSensor(SensorInterface):
    """Handler for camera sensors."""
    
    def __init__(self):
        self.cv_bridge = CvBridge()
        self.publisher_map = {}
        self.id_to_camera_info_map = {}
        self.camera_publish_prev_time = {}
        self.camera_freq = 11
    
    def initialize(self, ros2_node, topic_base, publisher_map):
        """Initialize camera publishers."""
        self.ros2_node = ros2_node
        self.topic_base = topic_base
        self.publisher_map = publisher_map
        
    def build_camera_info(self, attributes):
        """Compute camera info that doesn't change over time."""
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
    
    def register_camera(self, sensor_id, sensor_attributes):
        """Register a new camera sensor."""
        if sensor_id == "Center":
            self.publisher_map[sensor_id] = self.ros2_node.create_publisher(
                Image, f"{self.topic_base}/sensing/camera/traffic_light/image_raw", 1,
            )
            self.id_to_camera_info_map[sensor_id] = self.build_camera_info(sensor_attributes)
            self.publisher_map[sensor_id + "_info"] = self.ros2_node.create_publisher(
                CameraInfo, f"{self.topic_base}/sensing/camera/traffic_light/camera_info", 1,
            )
        else:
            self.publisher_map[sensor_id] = self.ros2_node.create_publisher(
                Image, f"{self.topic_base}/sensing/camera/vehicle_{sensor_id.lower()}/image_raw", 1,
            )
            self.id_to_camera_info_map[sensor_id] = self.build_camera_info(sensor_attributes)
            self.publisher_map[sensor_id + "_info"] = self.ros2_node.create_publisher(
                CameraInfo, f"{self.topic_base}/sensing/camera/vehicle_{sensor_id.lower()}/camera_info", 1,
            )
            
        self.camera_publish_prev_time[sensor_id] = datetime.datetime.now()
    
    def process_data(self, sensor_id, data, timestamp):
        """Process and publish camera data."""
        camera_publish_prev_time = self.camera_publish_prev_time[sensor_id]
        if self._check_frequency(camera_publish_prev_time, self.camera_freq):
            return
        
        self.camera_publish_prev_time[sensor_id] = datetime.datetime.now()
        
        msg = self.cv_bridge.cv2_to_imgmsg(data, encoding="bgra8")
        msg.header = self._create_header(timestamp)
        
        if sensor_id == "Center":
            msg.header.frame_id = "traffic_light_left_camera/camera_link"
        else:
            msg.header.frame_id = f"vehicle_{sensor_id.lower()}/camera_link"
            
        cam_info = self.id_to_camera_info_map[sensor_id]
        cam_info.header = msg.header
        self.publisher_map[sensor_id + "_info"].publish(cam_info)
        self.publisher_map[sensor_id].publish(msg)
    
    def _check_frequency(self, prev_time, target_freq):
        """Check if the frequency of publishing should be limited."""
        time_delta = (datetime.datetime.now() - prev_time).microseconds / 1000000.0
        return 1.0 / time_delta >= target_freq
    
    def _create_header(self, timestamp):
        """Create a ROS header with the given timestamp."""
        header = Header()
        seconds = int(timestamp)
        nanoseconds = int((timestamp - seconds) * 1000000000.0)
        header.stamp = Time(sec=seconds, nanosec=nanoseconds)
        return header


# Lidar sensor handler
class LidarSensor(SensorInterface):
    """Handler for lidar sensors."""
    
    def __init__(self):
        self.lidar_publish_prev_time = datetime.datetime.now()
        self.lidar_freq = 11
        self.id_to_frame_id = {}
        
    def initialize(self, ros2_node, topic_base, publisher_map):
        """Initialize lidar publishers."""
        self.ros2_node = ros2_node
        self.topic_base = topic_base
        
        # Create appropriate publisher based on running mode
        if os.environ.get("RUNNING_MODE", "normal") == "record":
            self.sensing_cloud_publisher = ros2_node.create_publisher(
                PointCloud2, "/velodyne_points", 10,
            )
        else:
            self.sensing_cloud_publisher = ros2_node.create_publisher(
                PointCloud2, f"{topic_base}/carla_pointcloud", 10,
            )
    
    def register_lidar(self, sensor_id, frame_id):
        """Register a new lidar sensor."""
        self.id_to_frame_id[sensor_id] = frame_id
    
    def process_data(self, sensor_id, data, timestamp):
        """Process and publish lidar data."""
        if self._check_frequency(self.lidar_publish_prev_time, self.lidar_freq):
            return
        
        self.lidar_publish_prev_time = datetime.datetime.now()
        
        header = self._create_header(timestamp)
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
            
            frame_id = self.id_to_frame_id[sensor_id]
            if frame_id == "velodyne_top" and os.environ.get("RUNNING_MODE", "normal") == "record":
                header.frame_id = "velodyne"
            else:
                header.frame_id = frame_id
                
            msg = create_cloud(header, fields, lidar_data)
            self.sensing_cloud_publisher.publish(msg)
        else:
            print(f"Cannot Reshape LIDAR Data buffer for {self.id_to_frame_id[sensor_id]}")
    
    def _check_frequency(self, prev_time, target_freq):
        """Check if the frequency of publishing should be limited."""
        time_delta = (datetime.datetime.now() - prev_time).microseconds / 1000000.0
        return 1.0 / time_delta >= target_freq
    
    def _create_header(self, timestamp):
        """Create a ROS header with the given timestamp."""
        header = Header()
        seconds = int(timestamp)
        nanoseconds = int((timestamp - seconds) * 1000000000.0)
        header.stamp = Time(sec=seconds, nanosec=nanoseconds)
        return header


# GNSS sensor handler
class GnssSensor(SensorInterface):
    """Handler for GNSS sensors."""
    
    def __init__(self):
        self.gnss_publish_prev_time = datetime.datetime.now()
        self.gnss_freq = 2
        self.publisher_map = {}
        
    def initialize(self, ros2_node, topic_base, publisher_map):
        """Initialize GNSS publishers."""
        self.ros2_node = ros2_node
        self.topic_base = topic_base
        self.publisher_map = publisher_map
    
    def register_gnss(self, sensor_id):
        """Register a new GNSS sensor."""
        self.publisher_map[sensor_id] = self.ros2_node.create_publisher(
            NavSatFix, f"{self.topic_base}/carla_nav_sat_fix", 1,
        )
    
    def process_data(self, sensor_id, data, timestamp):
        """Process and publish GNSS data."""
        if self._check_frequency(self.gnss_publish_prev_time, self.gnss_freq):
            return
        
        self.gnss_publish_prev_time = datetime.datetime.now()
        
        msg = NavSatFix()
        msg.header = self._create_header(timestamp)
        msg.header.frame_id = "gnss_link"
        msg.latitude = data[0]
        msg.longitude = data[1]
        msg.altitude = data[2]
        msg.status.status = NavSatStatus.STATUS_SBAS_FIX
        msg.status.service = (
            NavSatStatus.SERVICE_GPS | 
            NavSatStatus.SERVICE_GLONASS | 
            NavSatStatus.SERVICE_COMPASS | 
            NavSatStatus.SERVICE_GALILEO
        )
        
        self.publisher_map[sensor_id].publish(msg)
    
    def _check_frequency(self, prev_time, target_freq):
        """Check if the frequency of publishing should be limited."""
        time_delta = (datetime.datetime.now() - prev_time).microseconds / 1000000.0
        return 1.0 / time_delta >= target_freq
    
    def _create_header(self, timestamp):
        """Create a ROS header with the given timestamp."""
        header = Header()
        seconds = int(timestamp)
        nanoseconds = int((timestamp - seconds) * 1000000000.0)
        header.stamp = Time(sec=seconds, nanosec=nanoseconds)
        return header


# IMU sensor handler
class ImuSensor(SensorInterface):
    """Handler for IMU sensors."""
    
    def __init__(self):
        self.imu_publish_prev_time = datetime.datetime.now()
        self.imu_freq = 200 if os.environ.get("RUNNING_MODE", "normal") == "record" else 50
        
    def initialize(self, ros2_node, topic_base, publisher_map):
        """Initialize IMU publishers."""
        self.ros2_node = ros2_node
        self.topic_base = topic_base
        
        # Create appropriate publisher based on running mode
        if os.environ.get("RUNNING_MODE", "normal") == "record":
            self.vehicle_imu_publisher = ros2_node.create_publisher(
                Imu, f"/sensing/imu/tamagawa/imu_raw", 1,
            )
        else:
            self.vehicle_imu_publisher = ros2_node.create_publisher(
                Imu, f"{topic_base}/sensing/imu/tamagawa/imu_raw", 1,
            )
    
    def process_data(self, sensor_id, data, timestamp):
        """Process and publish IMU data."""
        if self._check_frequency(self.imu_publish_prev_time, self.imu_freq):
            return
        
        self.imu_publish_prev_time = datetime.datetime.now()
        
        imu_msg = Imu()
        imu_msg.header = self._create_header(timestamp)
        imu_msg.header.frame_id = "tamagawa/imu_link"
        
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
        
        self.vehicle_imu_publisher.publish(imu_msg)
    
    def _check_frequency(self, prev_time, target_freq):
        """Check if the frequency of publishing should be limited."""
        time_delta = (datetime.datetime.now() - prev_time).microseconds / 1000000.0
        return 1.0 / time_delta >= target_freq
    
    def _create_header(self, timestamp):
        """Create a ROS header with the given timestamp."""
        header = Header()
        seconds = int(timestamp)
        nanoseconds = int((timestamp - seconds) * 1000000000.0)
        header.stamp = Time(sec=seconds, nanosec=nanoseconds)
        return header


# CAN/Vehicle status handler
class VehicleStatusSensor(SensorInterface):
    """Handler for vehicle status data (CAN-like)."""
    
    def __init__(self):
        self.can_publish_prev_time = datetime.datetime.now()
        self.can_freq = 50
        self.speed = 0
        
    def initialize(self, ros2_node, topic_base, publisher_map):
        """Initialize vehicle status publishers."""
        self.ros2_node = ros2_node
        self.topic_base = topic_base
        
        # General odometry publisher
        self.vehicle_status_publisher = ros2_node.create_publisher(
            Odometry, f"{topic_base}/odo", 1,
        )
        
        # Autoware-specific publishers if in autoware mode
        if os.environ.get("CONTROL_MODE", "pygame") == "autoware":
            self.auto_velocity_status_publisher = ros2_node.create_publisher(
                VelocityReport, "/vehicle/status/velocity_status", 1,
            )
            self.auto_steering_status_publisher = ros2_node.create_publisher(
                SteeringReport, "/vehicle/status/steering_status", 1,
            )
            self.auto_gear_status_publisher = ros2_node.create_publisher(
                GearReport, "/vehicle/status/gear_status", 1,
            )
            self.auto_control_mode_publisher = ros2_node.create_publisher(
                ControlModeReport, "/vehicle/status/control_mode", 1,
            )
    
    def set_ego_vehicle(self, ego_vehicle):
        """Set the ego vehicle reference."""
        self.ego_vehicle = ego_vehicle
        
    def set_current_control(self, current_control):
        """Set the current control reference."""
        self.current_control = current_control
    
    def process_data(self, sensor_id, data, timestamp):
        """Process and publish vehicle status data."""
        if self._check_frequency(self.can_publish_prev_time, self.can_freq):
            return
        
        self.can_publish_prev_time = datetime.datetime.now()
        self.speed = data["speed"]
        
        pose_msg = PoseWithCovariance()
        pose_msg.pose = trans.carla_transform_to_ros_pose(self.ego_vehicle.get_transform())
        
        twist_msg = TwistWithCovariance()
        twist_msg.twist.linear.x = data["speed"]
        twist_msg.twist.angular.z = -self.current_control.steer
        twist_msg.twist.linear.z = 1.0
        twist_msg.twist.angular.x = 1.0
        
        odo_msg = Odometry()
        odo_msg.header = self._create_header(timestamp)
        odo_msg.pose = pose_msg
        odo_msg.twist = twist_msg
        self.vehicle_status_publisher.publish(odo_msg)
        
        # Publish Autoware-specific messages if in autoware mode
        if os.environ.get("CONTROL_MODE", "pygame") == "autoware":
            self._publish_autoware_status(timestamp)
    
    def _publish_autoware_status(self, timestamp):
        """Publish Autoware-specific status messages."""
        # Velocity status
        vel_rep = VelocityReport()
        vel_rep.header = self._create_header(timestamp)
        vel_rep.header.frame_id = "base_link"
        vel_rep.longitudinal_velocity = self.speed
        vel_rep.heading_rate = 0.0
        self.auto_velocity_status_publisher.publish(vel_rep)
        
        # Steering status
        steer_rep = SteeringReport()
        if self.current_control.reverse:
            steer_rep.steering_tire_angle = (-self.current_control.steer * 0.7) / 1.7  # max_steer_angle / reverse_factor
        else:
            steer_rep.steering_tire_angle = (-self.current_control.steer * 0.7) / 0.45  # max_steer_angle / steering_factor
        self.auto_steering_status_publisher.publish(steer_rep)
        
        # Gear status
        gear_rep = GearReport()
        gear_rep.stamp = self._create_header(timestamp).stamp
        if self.current_control.gear == 1:
            if self.current_control.reverse:
                gear_rep.report = GearReport.REVERSE
            else:
                gear_rep.report = GearReport.DRIVE
        else:
            gear_rep.report = GearReport.PARK
        self.auto_gear_status_publisher.publish(gear_rep)
        
        # Control mode status
        control_mode_rep = ControlModeReport()
        control_mode_rep.stamp = self._create_header(timestamp).stamp
        control_mode_rep.mode = ControlModeReport.AUTONOMOUS
        self.auto_control_mode_publisher.publish(control_mode_rep)
    
    def _check_frequency(self, prev_time, target_freq):
        """Check if the frequency of publishing should be limited."""
        time_delta = (datetime.datetime.now() - prev_time).microseconds / 1000000.0
        return 1.0 / time_delta >= target_freq
    
    def _create_header(self, timestamp):
        """Create a ROS header with the given timestamp."""
        header = Header()
        seconds = int(timestamp)
        nanoseconds = int((timestamp - seconds) * 1000000000.0)
        header.stamp = Time(sec=seconds, nanosec=nanoseconds)
        return header


# Map sensor handler
class MapSensor(SensorInterface):
    """Handler for OpenDRIVE map data."""
    
    def __init__(self):
        self.current_map_name = None
        
    def initialize(self, ros2_node, topic_base, publisher_map):
        """Initialize map publishers."""
        self.ros2_node = ros2_node
        self.topic_base = topic_base
        
        self.map_file_publisher = ros2_node.create_publisher(
            String, f"{topic_base}/carla/map_file", 1,
        )
    
    def process_data(self, sensor_id, data, timestamp):
        """Process and publish map data."""
        map_name = self._get_map_name(CarlaDataProvider.get_map().name)
        
        if self.current_map_name != map_name:
            self.current_map_name = map_name
            
            data_msg = String()
            data_msg.data = data["opendrive"]
            self.map_file_publisher.publish(data_msg)
            
            return data["opendrive"], map_name
        
        return None, None
    
    def _get_map_name(self, map_full_name):
        """Extract the map name from the full path."""
        if map_full_name is None:
            return None
            
        name_start_index = map_full_name.rfind("/")
        if name_start_index == -1:
            name_start_index = 0
        else:
            name_start_index = name_start_index + 1
        
        return map_full_name[name_start_index:len(map_full_name)]


# Vehicle transform handler
class VehicleTransformPublisher:
    """Handler for vehicle transform publishing."""
    
    def __init__(self):
        self.ego_vehicle_transform_publish_prev_time = datetime.datetime.now()
        self.ego_vehicle_transform_freq = 50
        
    def initialize(self, ros2_node, topic_base, ego_vehicle):
        """Initialize transform publisher."""
        self.ros2_node = ros2_node
        self.topic_base = topic_base
        self.ego_vehicle = ego_vehicle
        
        self.ego_vehicle_transform_publisher = ros2_node.create_publisher(
            PoseWithCovarianceStamped, f"{topic_base}/vehicle_transform", 1,
        )
    
    def publish_transform(self, timestamp):
        """Publish the ego vehicle transform."""
        if self._check_frequency(self.ego_vehicle_transform_publish_prev_time, self.ego_vehicle_transform_freq):
            return
        
        self.ego_vehicle_transform_publish_prev_time = datetime.datetime.now()
        
        ego_vehicle_transform = PoseWithCovariance()
        ego_vehicle_transform.pose = trans.carla_transform_to_ros_pose(self.ego_vehicle.get_transform())
        
        pos_msg = PoseWithCovarianceStamped()
        pos_msg.pose = ego_vehicle_transform
        pos_msg.header = self._create_header(timestamp)
        
        self.ego_vehicle_transform_publisher.publish(pos_msg)
    
    def _check_frequency(self, prev_time, target_freq):
        """Check if the frequency of publishing should be limited."""
        time_delta = (datetime.datetime.now() - prev_time).microseconds / 1000000.0
        return 1.0 / time_delta >= target_freq
    
    def _create_header(self, timestamp):
        """Create a ROS header with the given timestamp."""
        header = Header()
        seconds = int(timestamp)
        nanoseconds = int((timestamp - seconds) * 1000000000.0)
        header.stamp = Time(sec=seconds, nanosec=nanoseconds)
        return header


# Abstract controller interface
class ControllerInterface(ABC):
    """Abstract base class for vehicle controllers."""
    
    @abstractmethod
    def initialize(self, ros2_node, ego_vehicle):
        """Initialize the controller."""
        pass
    
    @abstractmethod
    def update_control(self):
        """Update and return the current control command."""
        pass


# Autoware controller
class AutowareController(ControllerInterface):
    """Controller for Autoware mode."""
    
    def __init__(self):
        self.current_control = carla.VehicleControl()
        self.speed = 0
        self.steering_factor = 0.45
        self.reverse_steering_factor = 1.7
        self.max_steer_angle = 0.7
        self.step_mode_possible = False
        
    def initialize(self, ros2_node, ego_vehicle):
        """Initialize the Autoware controller."""
        self.ros2_node = ros2_node
        self.ego_vehicle = ego_vehicle
        
        # Subscribe to Autoware control commands
        self.auto_vehicle_control_sub = ros2_node.create_subscription(
            AckermannControlCommand,
            "/control/command/control_cmd",
            self.on_auto_vehicle_control_callback,
            qos_profile=QoSProfile(depth=1),
        )
        
        # Subscribe to initial pose
        self.auto_vehicle_initialpose_sub = ros2_node.create_subscription(
            PoseWithCovarianceStamped,
            "/initialpose",
            self.on_auto_vehicle_initialpose_callback,
            1,
        )
    
    def on_auto_vehicle_control_callback(self, data):
        """Callback for Autoware vehicle control commands."""
        # FSM Lab Updated controller
        cmd = carla.VehicleControl()
        
        # Set gear based on acceleration
        if abs(data.longitudinal.acceleration) <= 1.5:
            cmd.gear = 1
        else:
            cmd.gear = 0
        
        # Handle steering based on speed direction
        if data.longitudinal.speed > 0:
            cmd.reverse = False
            cmd.steer = (-data.lateral.steering_tire_angle / self.max_steer_angle) * self.steering_factor
        elif data.longitudinal.speed < 0:
            cmd.reverse = True
            cmd.steer = (-data.lateral.steering_tire_angle / self.max_steer_angle) * self.reverse_steering_factor
            self.speed = 0.0
        else:
            cmd.reverse = False
            self.speed = 0.0
        
        # Handle throttle and brake based on speed difference
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
        self.step_mode_possible = True
    
    def on_auto_vehicle_initialpose_callback(self, data):
        """Callback for initial pose messages."""
        pose = data.pose.pose
        pose.position.z += 2.0
        carla_pose_transform = trans.ros_pose_to_carla_transform(pose)
        
        if self.ego_vehicle is not None:
            self.ego_vehicle.set_transform(carla_pose_transform)
        else:
            print("Can't find Ego Vehicle! Make sure it is spawned!")
    
    def update_control(self):
        """Return the current control command."""
        return self.current_control


# Teleop controller
class TeleopController(ControllerInterface):
    """Controller for teleoperation mode."""
    
    def __init__(self):
        self.current_control = carla.VehicleControl()
        self.step_mode_possible = False
        
    def initialize(self, ros2_node, ego_vehicle):
        """Initialize the teleop controller."""
        self.ros2_node = ros2_node
        self.ego_vehicle = ego_vehicle
        
        # Subscribe to twist commands
        self.vehicle_control_sub = ros2_node.create_subscription(
            TwistStamped,
            f"/carla/{ego_vehicle.attributes['role_name']}/carla_op_controller_cmd",
            self.on_vehicle_control_callback,
            1,
        )
    
    def on_vehicle_control_callback(self, data):
        """Callback for teleop vehicle control commands."""
        cmd = carla.VehicleControl()
        cmd.throttle = data.twist.linear.x / 100.0
        cmd.steer = data.twist.angular.z / 100.0
        cmd.brake = data.twist.linear.y / 100.0
        
        self.current_control = cmd
        self.step_mode_possible = True
    
    def update_control(self):
        """Return the current control command."""
        return self.current_control


# Follow controller
class FollowController(ControllerInterface):
    """Controller that follows another vehicle's transform."""
    
    def __init__(self):
        self.current_control = carla.VehicleControl()
        
    def initialize(self, ros2_node, ego_vehicle):
        """Initialize the follow controller."""
        self.ros2_node = ros2_node
        self.ego_vehicle = ego_vehicle
        
        # Subscribe to real vehicle transform
        self.real_vehicle_transform_sub = ros2_node.create_subscription(
            PoseStamped,
            f"/real/{ego_vehicle.attributes['role_name']}/transform",
            self.on_real_vehicle_transform_skip_control_callback,
            1,
        )
    
    def on_real_vehicle_transform_skip_control_callback(self, data):
        """Callback to skip to the transform location of the real vehicle."""
        pose = data.pose
        carla_pose_transform = trans.ros_pose_to_carla_transform(pose)
        
        if self.ego_vehicle is not None:
            self.ego_vehicle.set_transform(carla_pose_transform)
        else:
            print("Can't find Ego Vehicle! Make sure it is spawned!")
    
    def update_control(self):
        """Return the current control command."""
        return self.current_control


# Factory for creating controllers
class ControllerFactory:
    """Factory for creating appropriate controller based on control mode."""
    
    @staticmethod
    def create_controller(control_mode):
        """Create and return a controller based on control mode."""
        if control_mode == "autoware":
            return AutowareController()
        elif control_mode == "teleop":
            return TeleopController()
        elif control_mode == "follow":
            return FollowController()
        elif control_mode == "pygame":
            # Pygame control is handled externally
            return None
        else:
            raise ValueError(f"Invalid control mode: {control_mode}")


# Process launcher for external tools
class ProcessLauncher:
    """Handler for launching and managing external processes."""
    
    def __init__(self):
        self.stack_process = None
        self.carla_gnss_process = None
        self.lidar_slam_process = None
    
    def launch_stack(self, script_path, role_name, map_name, enable_explore, waypoints_topic):
        """Launch the Autoware stack."""
        auto_start_script = f"{script_path} {role_name} {map_name} {enable_explore} {waypoints_topic}"
        self.stack_process = subprocess.Popen(["x-terminal-emulator", "-e", f"{auto_start_script}"])
    
    def launch_gnss(self, script_path):
        """Launch the GNSS process."""
        self.carla_gnss_process = subprocess.Popen(["x-terminal-emulator", "-e", f"{script_path}"])
    
    def launch_lidar_slam(self, script_path):
        """Launch the LiDAR SLAM process."""
        self.lidar_slam_process = subprocess.Popen(["x-terminal-emulator", "-e", f"{script_path}"])
    
    def cleanup(self):
        """Terminate all processes."""
        processes = [
            ("stack", self.stack_process),
            ("GNSS", self.carla_gnss_process),
            ("LiDAR SLAM", self.lidar_slam_process)
        ]
        
        for name, process in processes:
            if process and process.poll() is None:
                try:
                    print(f"Terminating {name} process...")
                    process.terminate()
                    process.wait(timeout=5)
                    print(f"{name} process terminated.")
                except subprocess.TimeoutExpired:
                    print(f"Killing {name} process forcefully...")
                    process.kill()
                    print(f"{name} process killed.")
                except Exception as e:
                    print(f"Error terminating {name} process: {e}")


# Main ego vehicle initialization class
class EgoVehicleInit(AutonomousAgent):
    """Base class for ROS-based stacks."""
    
    def __init__(self, path_to_conf_file):
        """Initialize the ego vehicle."""
        super().__init__(path_to_conf_file)
        
        # Default configuration
        self.speed = None
        self.current_control = None
        self.track = Track.MAP
        self.global_plan_published_time = None
        self.manual_data_debug = False
        self.counter = 0
        self.timestamp = None
        
        # Camera snapshots
        self.center_camera = None
        self.back_camera = None
        self.bev_camera = None
        
        # Map data
        self.open_drive_map_name = None
        self.open_drive_map_data = None
        
        # Control settings
        self.steering_factor = 0.45
        self.reverse_steering_factor = 1.7
        self.max_steer_angle = 0.7
        
        # Sensor handlers
        self.camera_handler = CameraSensor()
        self.lidar_handler = LidarSensor()
        self.gnss_handler = GnssSensor()
        self.imu_handler = ImuSensor()
        self.vehicle_status_handler = VehicleStatusSensor()
        self.map_handler = MapSensor()
        
        # Process launcher
        self.process_launcher = ProcessLauncher()
        
        # Dictionary to store sensor types by ID
        self.id_to_sensor_type_map = {}
        
    def setup(self, path_to_conf_file):
        """Set up the agent."""
        self.track = Track.MAP
        
        # Get environment variables
        self.local_host = os.environ["SIMULATOR_LOCAL_HOST"]
        self.port = int(os.environ["SIMULATOR_PORT"])
        self.remote_connection = eval(os.environ["REMOTE_CONNECTION"])
        self.agent_role_name = os.environ["AGENT_ROLE_NAME"].replace("-", "_")
        self.bridge_mode = os.environ["OP_BRIDGE_MODE"]
        self.control_mode = os.environ["CONTROL_MODE"]
        
        # Set topic base
        self.topic_base = "" if self.control_mode.lower() == "autoware" else f"/carla/{self.agent_role_name}"
        self.topic_waypoints = self.topic_base + "/waypoints"
        
        # Initialize ROS
        self.bridge = CvBridge()
        self.spawn_point_initialized = False
        
        # Check agent root path
        team_code_path = os.environ["OP_AGENT_ROOT"]
        if not team_code_path or not os.path.exists(team_code_path):
            raise IOError(f"Path '{team_code_path}' defined by OP_AGENT_ROOT invalid.")
        
        # Get script paths
        if self.control_mode == "autoware":
            self.auto_start_script = f"{team_code_path}/start_ros2.sh"
            if not os.path.exists(self.auto_start_script):
                raise IOError(f"File '{self.auto_start_script}' defined by OP_AGENT_ROOT invalid.")
                
            # Get GNSS script if necessary
            if "fsm_lab".lower() in os.environ["FREE_MAP_NAME"]:
                self.carla_gnss_start_script = f"{os.environ['OP_BRIDGE_ROOT']}/op_scripts/fsm_lab_simulation/run_carla_gnss_ros2.sh"
                if not os.path.exists(self.carla_gnss_start_script):
                    raise IOError(f"File '{self.carla_gnss_start_script}' invalid.")
        
        # Get LiDAR SLAM script if in record mode
        if os.environ["RUNNING_MODE"] == "record":
            self.lidar_slam_start_script = f"{os.environ['OP_BRIDGE_ROOT']}/op_scripts/fsm_lab_simulation/run_lidar_slam_ros2.sh"
            if not os.path.exists(self.lidar_slam_start_script):
                raise IOError(f"File '{self.lidar_slam_start_script}' invalid.")
        
        # Initialize ROS node
        rclpy.init(args=None)
        self.ros2_node = rclpy.create_node(self.agent_role_name.replace("-", "_"))
        
        # Initialize clock publisher
        self.clock_publisher = self.ros2_node.create_publisher(Clock, f"{self.topic_base}/clock", 10)
        obj_clock = Clock()
        obj_clock.clock = Time(sec=0)
        self.clock_publisher.publish(obj_clock)
        
        # Get the ego vehicle
        self.ego_vehicle = self.get_ego_vehicle()
        self.current_control = carla.VehicleControl()
        
        # Initialize the controller based on control mode
        self.controller = ControllerFactory.create_controller(self.control_mode)
        if self.controller:
            self.controller.initialize(self.ros2_node, self.ego_vehicle)
        
        # Initialize publishers and subscribers
        self.publisher_map = {}
        self.init_subscribers()
        self.init_publishers()
        
        # Initialize transform publisher
        self.transform_publisher = VehicleTransformPublisher()
        self.transform_publisher.initialize(self.ros2_node, self.topic_base, self.ego_vehicle)
        
        # Start ROS spin thread
        self.spin_thread = threading.Thread(target=rclpy.spin, args=(self.ros2_node,))
        self.spin_thread.start()
    
    def sensors(self):
        """Define the sensor suite."""
        # Initialize timestamp tracking for frequency control
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
        
        # Define sensors
        sensors = [
            {"type": "sensor.camera.rgb", "x": 0.7, "y": 0.0, "z": 1.6, "roll": 0.0, "pitch": 0.0, "yaw": 0.0, "width": 1280, "height": 720, "fov": 100, "id": "Center"},
            {"type": "sensor.camera.rgb", "x": -1.4, "y": 0.0, "z": 1.6, "roll": 0.0, "pitch": 0.0, "yaw": 180, "width": 1280, "height": 720, "fov": 100, "id": "Back"},
            {"type": "sensor.camera.rgb", "x": 0.0, "y": 0.0, "z": 30.0, "roll": 0.0, "pitch": -90.0, "yaw": 0.0, "width": 1280, "height": 720, "fov": 100, "id": "Bev"},
            {"type": "sensor.lidar.ray_cast", "x": 0.0, "y": 0.0, "z": 2.6, "roll": 0.0, "pitch": 0.0, "yaw": -90.0, "id": "LIDAR-Top", "frame_id": "velodyne_top"},
            {"type": "sensor.other.gnss", "x": 0.0, "y": 0.0, "z": 1.6, "id": "GPS"},
            {"type": "sensor.opendrive_map", "reading_frequency": 1, "id": "OpenDRIVE"},
            {"type": "sensor.speedometer", "reading_frequency": 10, "id": "speed"},
            {"type": "sensor.other.imu", "x": 0.0, "y": 0.0, "z": 1.6, "roll": 0.0, "pitch": 0.0, "yaw": 0.0, "id": "IMU"},
        ]
        
        return sensors
    
    def init_subscribers(self):
        """Initialize all ROS subscribers."""
        # Subscribe to initialization spawn point if remote connection is enabled
        if self.remote_connection:
            self.init_spawn_point_sub = self.ros2_node.create_subscription(
                PoseStamped,
                f"/init/{self.agent_role_name}/transform",
                self.on_init_spawn_point_callback,
                1,
            )
            
            # Publisher for spawn point initialization status
            self.spawn_point_initialization_status_publisher = self.ros2_node.create_publisher(
                Bool,
                f"/init/{self.agent_role_name}/spawn_point_initialization_status",
                1,
            )
            self.spawn_point_initialization_status_publisher_timer = self.ros2_node.create_timer(
                0.001,
                self.on_init_spawn_point_feedback_callback,
            )
        
        # Camera subscribers
        self.center_camera_sub = self.ros2_node.create_subscription(
            Image,
            f"{self.topic_base}/sensing/camera/traffic_light/image_raw",
            self.center_camera_callback,
            1,
        )
        self.back_camear_sub = self.ros2_node.create_subscription(
            Image,
            f"{self.topic_base}/sensing/camera/vehicle_back/image_raw",
            self.back_camera_callback,
            1,
        )
        self.bev_camera_sub = self.ros2_node.create_subscription(
            Image,
            f"{self.topic_base}/sensing/camera/vehicle_bev/image_raw",
            self.bev_camera_callback,
            1,
        )
    
    def init_publishers(self):
        """Initialize all ROS publishers."""
        # Waypoint publisher
        self.waypoint_publisher = self.ros2_node.create_publisher(
            Path,
            self.topic_waypoints,
            1,
        )
        
        # Initialize sensor handlers
        self.camera_handler.initialize(self.ros2_node, self.topic_base, self.publisher_map)
        self.lidar_handler.initialize(self.ros2_node, self.topic_base, self.publisher_map)
        self.gnss_handler.initialize(self.ros2_node, self.topic_base, self.publisher_map)
        self.imu_handler.initialize(self.ros2_node, self.topic_base, self.publisher_map)
        self.vehicle_status_handler.initialize(self.ros2_node, self.topic_base, self.publisher_map)
        self.map_handler.initialize(self.ros2_node, self.topic_base, self.publisher_map)
        
        # Register sensors with handlers
        for sensor in self.sensors():
            self.id_to_sensor_type_map[sensor["id"]] = sensor["type"]
            
            if sensor["type"] == "sensor.camera.rgb":
                self.camera_handler.register_camera(sensor["id"], sensor)
            elif sensor["type"] == "sensor.lidar.ray_cast":
                self.lidar_handler.register_lidar(sensor["id"], sensor["frame_id"])
            elif sensor["type"] == "sensor.other.gnss":
                self.gnss_handler.register_gnss(sensor["id"])
            elif sensor["type"] == "sensor.other.imu":
                pass  # IMU handler initialization is simple
            elif sensor["type"] == "sensor.speedometer":
                pass  # Vehicle status handler initialization is handled separately
            elif sensor["type"] == "sensor.opendrive_map":
                pass  # Map handler initialization is simple
            else:
                raise TypeError(f"Invalid sensor type: {sensor['type']}")
        
        # Set ego vehicle and current control for vehicle status handler
        self.vehicle_status_handler.set_ego_vehicle(self.ego_vehicle)
        self.vehicle_status_handler.set_current_control(self.current_control)
    
    def run_step(self, input_data, timestamp):
        """Execute one step of navigation."""
        # Check remote connection status
        while self.remote_connection and not self.spawn_point_initialized:
            print("Waiting for remote vehicle's initialization pose...")
        
        # Get map name and initialize external processes if needed
        town_map_name = self.map_handler._get_map_name(CarlaDataProvider.get_map().name)
        if self.process_launcher.stack_process is None and town_map_name is not None and self.open_drive_map_name is not None:
            # Write the OpenDRIVE map file
            self.write_opendirve_map_file(self.open_drive_map_name, self.open_drive_map_data)
            
            # Launch appropriate processes based on control mode
            if self.control_mode == "autoware":
                # Launch GNSS if needed
                if self.process_launcher.carla_gnss_process is None and "fsm_lab".lower() in os.environ["FREE_MAP_NAME"]:
                    self.process_launcher.launch_gnss(self.carla_gnss_start_script)
                
                # Launch stack with appropriate parameters
                if self.bridge_mode == "free" or self.bridge_mode == "srunner":
                    self.process_launcher.launch_stack(self.auto_start_script, self.agent_role_name, town_map_name, 'true', '')
                else:
                    self.process_launcher.launch_stack(self.auto_start_script, self.agent_role_name, town_map_name, 'false', self.topic_waypoints)
            
            # Publish plan if available
            if self._global_plan_world_coord:
                self.publish_plan()
        
        # Launch LiDAR SLAM if in record mode
        if self.process_launcher.lidar_slam_process is None and os.environ.get("RUNNING_MODE", "normal") == "record":
            self.process_launcher.launch_lidar_slam(self.lidar_slam_start_script)
        
        # Update timestamp and publish clock
        self.timestamp = timestamp
        seconds = int(self.timestamp)
        nanoseconds = int((self.timestamp - int(self.timestamp)) * 1000000000.0)
        obj_clock = Clock()
        obj_clock.clock = Time(sec=seconds, nanosec=nanoseconds)
        self.clock_publisher.publish(obj_clock)
        
        # Check if stack is still running
        if (self.process_launcher.stack_process and 
            self.process_launcher.stack_process.poll() is not None):
            raise RuntimeError(f"Stack exited with: {self.process_launcher.stack_process.returncode} "
                               f"{self.process_launcher.stack_process.communicate()[0]}")
        
        # Publish global path periodically
        if self._global_plan_world_coord and (self.timestamp - self.global_plan_published_time) > 2.0:
            self.global_plan_published_time = self.timestamp
            self.publish_plan()
        
        # Publish ego vehicle transform
        self.transform_publisher.publish_transform(self.timestamp)
        
        # Process sensor data
        for key, val in input_data.items():
            sensor_type = self.id_to_sensor_type_map[key]
            
            if self.manual_data_debug:
                print(key)
            
            if sensor_type == "sensor.camera.rgb":
                self.camera_handler.process_data(key, val[1], self.timestamp)
            elif sensor_type == "sensor.opendrive_map":
                self.open_drive_map_data, self.open_drive_map_name = self.map_handler.process_data(key, val[1], self.timestamp)
            elif sensor_type == "sensor.other.gnss":
                self.gnss_handler.process_data(key, val[1], self.timestamp)
            elif sensor_type == "sensor.lidar.ray_cast":
                self.lidar_handler.process_data(key, val[1], self.timestamp)
            elif sensor_type == "sensor.speedometer":
                self.vehicle_status_handler.process_data(key, val[1], self.timestamp)
            elif sensor_type == "sensor.other.imu":
                self.imu_handler.process_data(key, val[1], self.timestamp)
            elif self.manual_data_debug:
                print("Additional Sensor !!")
                print(key)
        
        # Update control from controller if available
        if self.controller:
            self.current_control = self.controller.update_control()
        
        return self.current_control
    
    def destroy(self):
        """Clean up all ROS publishers and processes."""
        # Shutdown ROS
        rclpy.shutdown()
        self.spin_thread.join()
        
        # Clean up external processes
        self.process_launcher.cleanup()
    
    def on_init_spawn_point_callback(self, data):
        """Callback for initial spawn point message."""
        pose = data.pose
        carla_pose_transform = trans.ros_pose_to_carla_transform(pose)
        
        if self.remote_connection and not self.spawn_point_initialized:
            if self.ego_vehicle is not None:
                self.ego_vehicle.set_transform(carla_pose_transform)
                self.spawn_point_initialized = True
                print("Successfully receive the remote vehicle's pose and initialized spawn point in carla with it.")
            else:
                print(f"Can't find Ego Vehicle named {self.agent_role_name}!! Make sure it is spawned!")
        else:
            print("Warning: reinitialize the spawn point from the remote vehicle is not allowed! Please reboot.")
    
    def on_init_spawn_point_feedback_callback(self):
        """Callback to publish spawn point initialization status."""
        spawn_point_status = Bool()
        spawn_point_status.data = self.spawn_point_initialized
        self.spawn_point_initialization_status_publisher.publish(spawn_point_status)
    
    def center_camera_callback(self, data):
        """Callback to get the center camera image."""
        temp_camera = self.rosimg_to_cvimg(data)
        temp_camera = cv2.flip(cv2.rotate(temp_camera, cv2.ROTATE_90_COUNTERCLOCKWISE), 0)
        temp_camera = cv2.resize(temp_camera, (200, 300))
        self.center_camera = temp_camera
    
    def back_camera_callback(self, data):
        """Callback to get the back camera image."""
        temp_camera = self.rosimg_to_cvimg(data)
        temp_camera = cv2.rotate(temp_camera, cv2.ROTATE_90_COUNTERCLOCKWISE)
        temp_camera = cv2.resize(temp_camera, (200, 300))
        self.back_camera = temp_camera
    
    def bev_camera_callback(self, data):
        """Callback to get the bev camera image."""
        temp_camera = self.rosimg_to_cvimg(data)
        temp_camera = cv2.flip(cv2.rotate(temp_camera, cv2.ROTATE_90_COUNTERCLOCKWISE), 0)
        temp_camera = cv2.resize(temp_camera, (300, 600))
        self.bev_camera = temp_camera
    
    def publish_plan(self):
        """Publish the global plan."""
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
    
    def write_opendirve_map_file(self, map_name, map_data):
        """Write OpenDRIVE map data to a file."""
        team_code_path = os.environ["OP_AGENT_ROOT"]
        if not team_code_path or not os.path.exists(team_code_path):
            raise IOError(f"Path '{team_code_path}' defined by OP_AGENT_ROOT invalid.")
            
        opendrive_map_path = f"{team_code_path}/hdmaps/{map_name}.xodr"
        os.makedirs(f"{team_code_path}/hdmaps/", exist_ok=True)
        
        with open(opendrive_map_path, "w") as f:
            f.write(map_data)
    
    def get_camera_snapshots(self):
        """Get camera snapshots for visualization."""
        return self.center_camera, self.back_camera, self.bev_camera
    
    def get_ego_vehicle(self):
        """Get the ego vehicle."""
        for vehicle in CarlaDataProvider.get_world().get_actors().filter("vehicle.*"):
            if vehicle.attributes["role_name"] == self.agent_role_name:
                return vehicle
        return None
    
    def get_header(self):
        """Get a ROS header with the current timestamp."""
        header = Header()
        seconds = int(self.timestamp)
        nanoseconds = int((self.timestamp - int(self.timestamp)) * 1000000000.0)
        header.stamp = Time(sec=seconds, nanosec=nanoseconds)
        return header
    
    def rosimg_to_cvimg(self, data):
        """Convert ROS image to OpenCV image."""
        return self.bridge.imgmsg_to_cv2(data, "bgr8")
    
    def use_stepping_mode(self):
        """Determine if stepping mode should be used."""
        return False