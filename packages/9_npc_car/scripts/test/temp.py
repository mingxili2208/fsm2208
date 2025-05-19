#!/usr/bin/env python

# Copyright (c) 2025 James LI
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""
Autoware-CARLA Integration Script: Manages both NPC traffic and Autoware goal sequences
1. Sets up Traffic Manager and spawns NPC vehicles
2. Checks if vehicle is near npc_test start point, then guides through npc_test waypoints
3. After completing npc_test, removes NPC cars and begins continuous circle navigation
4. Handles proper cleanup on interruption
"""

import glob
import os
import sys
import time
import random
import numpy as np
import argparse
import logging
import inspect
import datetime
import setproctitle
import math
import threading
import signal
from typing import List, Dict, Any, Optional

setproctitle.setproctitle('integrated_scenario_test')
try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

import carla

# Try to import ROS2 libraries
try:
    import rclpy
    from rclpy.node import Node
    from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
    from autoware_auto_vehicle_msgs.msg import Engage
    from builtin_interfaces.msg import Time
    from rclpy.time import Time as rclpy_time
    from rclpy.parameter import Parameter
    from rclpy.clock import ClockType
    from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
    from rosgraph_msgs.msg import Clock
    from transforms3d.euler import quat2euler
    
    ROS2_AVAILABLE = True
except ImportError as e:
    ROS2_AVAILABLE = False
    logging.warning(f"ROS2 or Autoware libraries import failed: {e}")
    logging.warning("Autoware engage functionality will be disabled")


class Logger:
    """Handles logging"""
    
    @staticmethod
    def setup(log_file=None):
        """Set up the logger"""
        timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        
        # If no log file name provided, create a default one with timestamp
        if log_file is None:
            log_dir = "logs"
            # Create log directory if it doesn't exist
            if not os.path.exists(log_dir):
                os.makedirs(log_dir)
            log_file = os.path.join(log_dir, f"integrated_scenario_{timestamp}.log")
        
        # Configure root logger
        logger = logging.getLogger()
        logger.setLevel(logging.INFO)
        
        # Clear any existing handlers
        if logger.handlers:
            logger.handlers.clear()
        
        # Create file handler
        file_handler = logging.FileHandler(log_file)
        file_formatter = logging.Formatter('%(asctime)s - %(levelname)s - %(message)s')
        file_handler.setFormatter(file_formatter)
        logger.addHandler(file_handler)
        
        # Create console handler
        console_handler = logging.StreamHandler(sys.stdout)
        console_formatter = logging.Formatter('%(levelname)s: %(message)s')
        console_handler.setFormatter(console_formatter)
        logger.addHandler(console_handler)
        
        # Log file location
        logger.info(f"Logging to: {os.path.abspath(log_file)}")
        
        return logger


class AutowareGoalPublisher(Node):
    """Handles publishing goals and engage commands to Autoware"""
    
    def __init__(self):
        super().__init__("autoware_goal_publisher")
        
        # Define two sets of goal positions
        self.goal_sets = {
            "npc_test": [
                # Format: [x, y, z, qx, qy, qz, qw]
                [-48.4787, -32.6236, 0.0, 0.0, 0.0, -0.715607, 0.698503],  # start point
                [29.7123, -36.908, 0.0, 0.0, 0.0, 0.248181, 0.968714],     # end point
                [-6.22938, 50.2089, 0.0, 0.0, 0.0, -0.999831, 0.0183731]   # out point
            ],
            "circle": [
                [-48.3957, -19.3472, 0.0, 0.0, 0.0, -0.723422, 0.690406],
                [54.3231, -47.6682, 0.0, 0.0, 0.0, 0.300639, 0.953738],
                [26.589, 50.3549, 0.0, 0.0, 0.0, -0.999824, 0.01874]
            ]
        }
        
        # Parameters
        self.distance_threshold = 1.0  # 1 meter threshold for goal detection
        self.current_pose = None
        self.latest_clock = Time()
        self.clock_received = False
        
        # Create QoS profile for reliable communication
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Create publishers
        self.goal_publisher = self.create_publisher(
            PoseStamped, "/planning/mission_planning/goal", qos
        )
        
        self.engage_publisher = self.create_publisher(
            Engage, "/autoware/engage", qos
        )
        
        # Create subscription to vehicle position
        self.position_subscription = self.create_subscription(
            PoseWithCovarianceStamped,
            "/real_world/follow_adtruck/transformed_with_covariance",
            self.vehicle_position_callback,
            qos
        )
        
        # Add subscription to /clock topic
        self.clock_subscription = self.create_subscription(
            Clock, '/clock', self.clock_callback, qos
        )
        
        self.get_logger().info("AutowareGoalPublisher initialized")
    
    def clock_callback(self, msg: Clock):
        """Process callback from /clock topic"""
        self.latest_clock = msg.clock
        self.clock_received = True
    
    def vehicle_position_callback(self, msg):
        """Callback for vehicle position updates"""
        self.current_pose = msg
    
    def wait_for_clock(self, timeout_sec: float = 5.0) -> bool:
        """Wait to receive at least one clock message"""
        start_time = time.time()
        while not self.clock_received and time.time() - start_time < timeout_sec:
            rclpy.spin_once(self, timeout_sec=0.1)
        
        if self.clock_received:
            self.get_logger().info(f"Received /clock: {self.latest_clock.sec}.{self.latest_clock.nanosec}")
        else:
            self.get_logger().warn(f"Timeout waiting for /clock, will use system time")
        
        return self.clock_received
    
    def publish_goal(self, goal_pos):
        """Publish a goal position to Autoware"""
        msg = PoseStamped()
        
        # Correctly use timestamp from /clock topic
        if self.clock_received:
            msg.header.stamp = self.latest_clock
            self.get_logger().debug(f"Using /clock time for goal: {self.latest_clock.sec}.{self.latest_clock.nanosec}")
        else:
            # If no clock messages received yet, use node clock as fallback
            msg.header.stamp = self.get_clock().now().to_msg()
            self.get_logger().warn("No /clock messages received, using node clock for goal")
        
        msg.header.frame_id = "map"
        
        # Set position - ensure explicit conversion to float
        msg.pose.position.x = float(goal_pos[0])
        msg.pose.position.y = float(goal_pos[1])
        msg.pose.position.z = float(goal_pos[2])
        
        # Set orientation (quaternion) - ensure as float
        msg.pose.orientation.x = float(goal_pos[3])
        msg.pose.orientation.y = float(goal_pos[4])
        msg.pose.orientation.z = float(goal_pos[5])
        msg.pose.orientation.w = float(goal_pos[6])
        
        # Convert quaternion to angle for logging
        _, _, yaw = quat2euler([goal_pos[6], goal_pos[3], goal_pos[4], goal_pos[5]])
        
        self.goal_publisher.publish(msg)
        
        self.get_logger().info(
            f"Setting goal pose: Frame:map, Position({goal_pos[0]}, {goal_pos[1]}, {goal_pos[2]}), "
            f"Orientation({goal_pos[3]}, {goal_pos[4]}, {goal_pos[5]}, {goal_pos[6]}) = Angle: {yaw}"
        )
    
    def publish_engage(self, engage_state):
        """Publish engage command to Autoware"""
        msg = Engage()
        
        # Correctly use timestamp from /clock topic
        if self.clock_received:
            msg.stamp = self.latest_clock
            self.get_logger().debug(f"Using /clock time for engage: {self.latest_clock.sec}.{self.latest_clock.nanosec}")
        else:
            # If no clock messages received yet, use node clock as fallback
            msg.stamp = self.get_clock().now().to_msg()
            self.get_logger().warn("No /clock messages received, using node clock for engage")
        
        msg.engage = engage_state
        
        self.engage_publisher.publish(msg)
        
        self.get_logger().info(f"Published engage state: {engage_state}")
    
    def calculate_distance(self, x1, y1, x2, y2):
        """Calculate Euclidean distance between two points"""
        return math.sqrt((x2 - x1)**2 + (y2 - y1)**2)
    
    def check_if_at_position(self, goal_pos):
        """Check if vehicle is at a specified position"""
        if self.current_pose is None:
            return False
        
        current_x = self.current_pose.pose.pose.position.x
        current_y = self.current_pose.pose.pose.position.y
        
        distance = self.calculate_distance(current_x, current_y, goal_pos[0], goal_pos[1])
        return distance < self.distance_threshold
    
    def get_current_position_distance_to(self, goal_pos):
        """Get vehicle's current distance to a goal position"""
        if self.current_pose is None:
            return float('inf')
        
        current_x = self.current_pose.pose.pose.position.x
        current_y = self.current_pose.pose.pose.position.y
        
        return self.calculate_distance(current_x, current_y, goal_pos[0], goal_pos[1])
    
    def wait_for_position_update(self, timeout_sec=5.0):
        """Wait for at least one position update"""
        start_time = time.time()
        while self.current_pose is None and time.time() - start_time < timeout_sec:
            rclpy.spin_once(self, timeout_sec=0.1)
        
        if self.current_pose is not None:
            pos = self.current_pose.pose.pose.position
            self.get_logger().info(f"Current position: ({pos.x:.2f}, {pos.y:.2f}, {pos.z:.2f})")
            return True
        else:
            self.get_logger().warn(f"No position update received within {timeout_sec} seconds")
            return False


class TrafficManagerInfo:
    """Handles Traffic Manager information display"""
    
    @staticmethod
    def print_info(tm, world):
        """Print detailed information about Traffic Manager and world sync mode"""
        logging.info("\n=== Traffic Manager and World Settings ===")
        
        # Print world settings
        settings = world.get_settings()

        logging.info(f"World settings:")
        logging.info(f"  - Synchronous mode: {settings.synchronous_mode}")
        logging.info(f"  - Fixed delta seconds: {settings.fixed_delta_seconds}")
        logging.info(f"  - Substepping: {settings.substepping}")
        logging.info(f"  - Max substep delta time: {settings.max_substep_delta_time}")
        
        # Print Traffic Manager info
        logging.info(f"Traffic Manager info:")
        logging.info(f"  - Port: {tm.get_port()}")
        
        # Check if Traffic Manager is in synchronous mode
        try:
            is_sync = tm.get_synchronous_mode()
            logging.info(f"  - Synchronous mode: {is_sync}")
        except:
            logging.info(f"  - Synchronous mode: Unable to retrieve")
        
        # Get global settings
        logging.info("\nTraffic Manager global settings:")
        try:
            global_distance = None
            # Try to get global distance to leading vehicle
            if hasattr(tm, 'get_global_distance_to_leading_vehicle'):
                global_distance = tm.get_global_distance_to_leading_vehicle()
            
            if global_distance is not None:
                logging.info(f"  - Global distance to leading vehicle: {global_distance} meters")
            else:
                logging.info(f"  - Global distance to leading vehicle: Unable to retrieve")
        except Exception as e:
            logging.info(f"  - Global distance to leading vehicle: Error retrieving ({e})")
        
        # Check and print currently active vehicles
        vehicles = world.get_actors().filter('vehicle.*')
        logging.info(f"\nCurrent vehicles in the world: {len(vehicles)}")
        
        for i, vehicle in enumerate(vehicles):
            try:
                autopilot = vehicle.get_autopilot()
                role = vehicle.attributes.get('role_name', 'unknown')
                logging.info(f"  Vehicle {i+1}: ID={vehicle.id}, Type={vehicle.type_id}, Role={role}, Autopilot={autopilot}")
            except:
                logging.info(f"  Vehicle {i+1}: ID={vehicle.id}, Type={vehicle.type_id}")
        
        logging.info("=======================================\n")


class CarlaManager:
    """Manages CARLA connection and vehicle spawning"""
    
    def __init__(self, args):
        """Initialize CarlaManager"""
        self.args = args
        self.client = None
        self.world = None
        self.map = None
        self.traffic_manager = None
        self.tm_port = args.tm_port
        self.blueprint_library = None
        self.spawn_points = []
        self.vehicle_list = []
        self.existing_vehicles = {}
        self.original_settings = None
        self.synchronous_master = False
    
    def connect(self):
        """Connect to CARLA server"""
        self.client = carla.Client(self.args.host, self.args.port)
        self.client.set_timeout(10.0)
        
        # Get world and map
        self.world = self.client.get_world()
        self.map = self.world.get_map()
        
        # Log environment info
        logging.info(f"CARLA version: {self.client.get_client_version()}")
        logging.info(f"Map: {self.map.name}")
        logging.info(f"Synchronous mode: {self.world.get_settings().synchronous_mode}")
        logging.info(f"Fixed delta seconds: {self.world.get_settings().fixed_delta_seconds}")
        
        # Save current settings
        self.original_settings = self.world.get_settings()
        
        # Get blueprint library
        self.blueprint_library = self.world.get_blueprint_library()
        
        # Get all available spawn points
        self.spawn_points = self.map.get_spawn_points()
        
        return True
    
    def setup_traffic_manager(self):
        """Set up and connect to Traffic Manager"""
        # Check if Traffic Manager is running
        tm_connected = False
        
        try:
            # Try to connect to default Traffic Manager port
            self.traffic_manager = self.client.get_trafficmanager(self.tm_port)
            logging.info(f"Connected to existing Traffic Manager at port: {self.tm_port}")
            tm_connected = True
        except Exception as e:
            logging.warning(f"Failed to connect to default Traffic Manager (port:{self.tm_port}): {e}")
            # If failed, try alternate port
            try:
                self.tm_port = self.args.alt_tm_port
                self.traffic_manager = self.client.get_trafficmanager(self.tm_port)
                logging.info(f"Created new Traffic Manager at port: {self.tm_port}")
                tm_connected = True
            except Exception as e:
                logging.error(f"Unable to create new Traffic Manager (port:{self.tm_port}): {e}")
                return False
        
        # Print Traffic Manager and world settings info
        if tm_connected:
            TrafficManagerInfo.print_info(self.traffic_manager, self.world)
        
        # Set Traffic Manager parameters - safety settings for coexisting with Autoware
        self.traffic_manager.set_global_distance_to_leading_vehicle(5.0)  # Increase following distance
        self.traffic_manager.set_random_device_seed(self.args.seed)
        
        # Print settings again to confirm changes
        logging.info("\n=== Updated Settings ===")
        TrafficManagerInfo.print_info(self.traffic_manager, self.world)
        
        return tm_connected
    
    def get_existing_vehicles(self):
        """Get and record existing vehicles"""
        # Check current vehicle count and record their info
        existing_actors = self.world.get_actors().filter('vehicle.*')
        logging.info(f'Current world has {len(existing_actors)} existing vehicles')
        
        # Record existing vehicle info to dictionary
        for vehicle in existing_actors:
            self.existing_vehicles[vehicle.id] = {
                'type': vehicle.type_id,
                'role': vehicle.attributes.get('role_name', 'unknown')
            }
            logging.info(f'Recorded existing vehicle: ID={vehicle.id}, Type={vehicle.type_id}, Role={self.existing_vehicles[vehicle.id]["role"]}')
    
    def spawn_vehicle(self, vehicle_id, color, spawn_idx, target_idx):
        """Spawn a single vehicle and set its attributes"""
        # Get specified vehicle blueprint
        bp = self.blueprint_library.find(vehicle_id)
        
        # Output selected vehicle ID and type info
        vehicle_type = vehicle_id.split('.')[1] if '.' in vehicle_id else "unknown"
        logging.info(f'Selected blueprint: ID={vehicle_id}, Type={vehicle_type}')
        
        # Set specified color (if any)
        if color and bp.has_attribute('color'):
            bp.set_attribute('color', color)
            logging.info(f'  Color: {color}')
        
        # Set role name to autopilot
        bp.set_attribute('role_name', 'npc_vehicle')
        
        # Spawn at specified point
        spawn_point = self.spawn_points[spawn_idx]
        
        # Modify spawn_point height to 0.2
        spawn_point.location.z = 0.2
        
        # Spawn vehicle
        vehicle = self.world.try_spawn_actor(bp, spawn_point)
        
        # If unable to spawn at predetermined location, try alternate positions
        spawn_attempts = 0
        while vehicle is None and spawn_attempts < 5:
            spawn_attempts += 1
            logging.info(f"  Trying alternate position {spawn_attempts}...")
            # Slightly adjust position rather than choosing random spawn points
            modified_spawn = carla.Transform(
                carla.Location(
                    x=spawn_point.location.x + random.uniform(-1.0, 1.0),
                    y=spawn_point.location.y + random.uniform(-1.0, 1.0),
                    z=0.2
                ),
                spawn_point.rotation
            )
            vehicle = self.world.try_spawn_actor(bp, modified_spawn)
        
        if vehicle is not None:
            logging.info(f'Vehicle ({vehicle_id}) spawned, ID={vehicle.id}')
            
            # Give vehicle control to Traffic Manager
            vehicle.set_autopilot(True, self.tm_port)
            
            # Set target point
            target_point = self.spawn_points[target_idx].location
            self.traffic_manager.set_path(vehicle, [target_point])
            
            logging.info(f'  Set target point: spawn_point[{target_idx}]')
            logging.info(f'  Target coordinates: x={target_point.x:.2f}, y={target_point.y:.2f}, z={target_point.z:.2f}')
            
            # Set vehicle behavior parameters - safe driving settings
            if self.args.safe_mode:
                try:
                    self.traffic_manager.auto_lane_change(vehicle, False)  # Disable auto lane change
                except:
                    logging.warning("  Note: Unable to set auto lane change parameter")
                
                try:
                    self.traffic_manager.distance_to_leading_vehicle(vehicle, 5.0)  # Set larger following distance
                except:
                    logging.warning("  Note: Unable to set following distance")
                
                try:
                    self.traffic_manager.vehicle_percentage_speed_difference(vehicle, random.uniform(0, 10))  # Reduce speed significantly
                except:
                    logging.warning("  Note: Unable to set speed difference")
                
                try:
                    if hasattr(self.traffic_manager, 'set_desired_speed'):
                        self.traffic_manager.set_desired_speed(vehicle, 25)  # Limit max speed to 25km/h
                except:
                    logging.warning("  Note: Unable to set desired speed")
                
                logging.info(f'  Safe driving mode enabled')
            
            return vehicle
        else:
            logging.error(f'Unable to spawn vehicle ({vehicle_id}), tried multiple positions')
            return None
    
    def spawn_vehicles(self):
        """Spawn all specified vehicles"""
        # Define spawn point and target point indices
        spawn_indices = [15, 40, 21, 44, 29]
        target_indices = [5, 19, 17, 8, 8]
        
        # Define specific vehicle types to spawn
        vehicle_types = [
            "vehicle.carlamotors.european_hgv",
            "vehicle.mini.cooper_s",
            "vehicle.nissan.patrol_2021",
            "vehicle.tesla.cybertruck",
            "vehicle.tesla.cybertruck"
        ]
        
        # Define vehicle colors
        vehicle_colors = [
            "202,88,176",
            "159,0,0",
            "0,38,132",
            None,
            "202,88,176"
        ]
        
        # Ensure we have enough indices
        num_vehicles = min(self.args.num_vehicles, len(vehicle_types))
        
        logging.info(f'Preparing to spawn {num_vehicles} specified vehicles')
        
        # Get and record existing vehicles
        self.get_existing_vehicles()
        
        # Spawn vehicles individually
        if not self.args.batch_spawn:
            for i in range(num_vehicles):
                vehicle = self.spawn_vehicle(
                    vehicle_types[i], 
                    vehicle_colors[i], 
                    spawn_indices[i], 
                    target_indices[i]
                )
                if vehicle:
                    self.vehicle_list.append(vehicle)
        
        # Batch spawn vehicles
        else:
            batch_commands = []
            for i in range(num_vehicles):
                # Get specified vehicle blueprint
                vehicle_id = vehicle_types[i]
                bp = self.blueprint_library.find(vehicle_id)
                
                # Set specified color (if any)
                if vehicle_colors[i] and bp.has_attribute('color'):
                    bp.set_attribute('color', vehicle_colors[i])
                
                # Set role name
                bp.set_attribute('role_name', 'npc_vehicle')
                
                # Prepare spawn point
                spawn_idx = spawn_indices[i]
                spawn_point = self.spawn_points[spawn_idx]
                spawn_point.location.z = 0.2
                
                # Add to command list
                batch_commands.append(carla.command.SpawnActor(bp, spawn_point))
            
            # Execute batch spawn
            logging.info(f"Batch spawning {len(batch_commands)} vehicles...")
            results = self.client.apply_batch_sync(batch_commands, True)
            
            # Process results, get vehicle references
            for i, result in enumerate(results):
                if result.error:
                    logging.error(f"  Vehicle {i+1} spawn failed: {result.error}")
                else:
                    # Get vehicle reference
                    vehicle = self.world.get_actor(result.actor_id)
                    if vehicle:
                        self.vehicle_list.append(vehicle)
                        logging.info(f"  Vehicle {i+1} (ID={result.actor_id}) spawned successfully")
                        
                        # Give vehicle control to Traffic Manager
                        vehicle.set_autopilot(True, self.tm_port)
                        
                        # Set target point
                        target_idx = target_indices[i]
                        target_point = self.spawn_points[target_idx].location
                        self.traffic_manager.set_path(vehicle, [target_point])
                        
                        # Safe driving settings
                        if self.args.safe_mode:
                            try:
                                self.traffic_manager.auto_lane_change(vehicle, False)
                                self.traffic_manager.distance_to_leading_vehicle(vehicle, 5)
                                self.traffic_manager.vehicle_percentage_speed_difference(vehicle, random.uniform(0, 10))
                            except:
                                pass
                    else:
                        logging.error(f"  Unable to get reference to vehicle {i+1} (ID={result.actor_id})")
        
        logging.info(f'Successfully spawned {len(self.vehicle_list)} NPC vehicles')
        return len(self.vehicle_list) > 0
    
    def cleanup_vehicles(self):
        """Clean up spawned vehicles"""
        # First step: Identify which vehicles were created by this script (not previously existing)
        vehicles_to_destroy = []
        current_vehicles = self.world.get_actors().filter('vehicle.*')
        logging.info(f'Current world has {len(current_vehicles)} vehicles')
        
        for vehicle in current_vehicles:
            # If vehicle ID isn't in previously recorded existing vehicles, it was created by this script
            if vehicle.id not in self.existing_vehicles:
                vehicles_to_destroy.append(vehicle)
                logging.info(f"Marked vehicle for removal: ID={vehicle.id}, Type={vehicle.type_id}")
        
        logging.info(f"Step 1: Need to remove {len(vehicles_to_destroy)} vehicles created by this script")
        
        # Step 2: Turn off autopilot for all vehicles to be destroyed
        active_vehicles = []
        for i, vehicle in enumerate(vehicles_to_destroy):
            try:
                if vehicle.is_alive:
                    logging.info(f"  Turning off autopilot for vehicle {i+1} (ID={vehicle.id})")
                    vehicle.set_autopilot(False)
                    active_vehicles.append(vehicle)
                else:
                    logging.info(f"  Vehicle {i+1} no longer exists, skipping")
            except Exception as e:
                logging.error(f"  Failed to turn off autopilot: {e}")
        
        # Wait briefly for Traffic Manager to respond
        time.sleep(0.5)
        
        # Step 3: Destroy vehicles one by one
        destroyed_count = 0
        logging.info(f"Step 3: Destroying {len(active_vehicles)} active vehicles individually")
        for i, vehicle in enumerate(active_vehicles):
            try:
                if vehicle.is_alive:
                    vehicle_id = vehicle.id
                    vehicle.destroy()
                    destroyed_count += 1
                    logging.info(f"  Successfully destroyed vehicle {i+1} (ID={vehicle_id})")
                    # Brief wait to avoid issues with too-rapid destruction
                    time.sleep(0.05)
                else:
                    logging.info(f"  Vehicle {i+1} no longer exists, skipping")
            except Exception as e:
                logging.error(f"  Failed to destroy vehicle: {e}")
        
        # Step 4: Check if any vehicles to be removed still exist and try batch destroy
        logging.info("Step 4: Checking and batch destroying any remaining vehicles")
        try:
            # Get current vehicles again
            final_check = self.world.get_actors().filter('vehicle.*')
            surviving_vehicles = []
            
            # Check which ones should have been destroyed but are still alive
            for vehicle in final_check:
                if vehicle.id not in self.existing_vehicles and vehicle.is_alive:
                    surviving_vehicles.append(vehicle)
            
            if surviving_vehicles:
                logging.info(f"  Attempting batch destroy of remaining {len(surviving_vehicles)} vehicles")
                self.client.apply_batch([carla.command.DestroyActor(v) for v in surviving_vehicles])
                logging.info("  Batch destroy command sent")
            else:
                logging.info("  No remaining vehicles to destroy")
        except Exception as e:
            logging.error(f"  Failed to batch destroy vehicles: {e}")
        
        # Clear vehicle list
        self.vehicle_list = []
        
        time.sleep(0.5)
        return destroyed_count


class IntegratedManager:
    """Integrates CARLA and Autoware functionality"""
    
    def __init__(self, args):
        """Initialize the integrated manager"""
        self.args = args
        self.carla_manager = CarlaManager(args)
        self.autoware_publisher = None
        self.running = True
        self.completion_flags = {
            "npc_test_completed": False,
            "circle_started": False
        }
        
        # Set up signal handler for graceful shutdown
        signal.signal(signal.SIGINT, self.signal_handler)
        signal.signal(signal.SIGTERM, self.signal_handler)
    
    def signal_handler(self, sig, frame):
        """Handle signals for clean shutdown"""
        logging.info(f"Received signal {sig}, initiating cleanup...")
        self.running = False
    
    def initialize(self):
        """Initialize connections and managers"""
        # Connect to CARLA
        if not self.carla_manager.connect():
            logging.error("Failed to connect to CARLA server")
            return False
        
        # Set up Traffic Manager
        if not self.carla_manager.setup_traffic_manager():
            logging.error("Failed to set up Traffic Manager")
            return False
        
        # Initialize ROS2 if available
        if ROS2_AVAILABLE:
            if not rclpy.ok():
                rclpy.init()
            
            # Create Autoware publisher
            self.autoware_publisher = AutowareGoalPublisher()
            
            # Wait for clock and position updates
            self.autoware_publisher.wait_for_clock(5.0)
            self.autoware_publisher.wait_for_position_update(5.0)
            
            logging.info("ROS2 and Autoware publisher initialized")
        else:
            logging.warning("ROS2 not available, skipping Autoware publisher initialization")
            return False
        
        # Spawn NPC vehicles
        if not self.carla_manager.spawn_vehicles():
            logging.error("Failed to spawn NPC vehicles")
            return False
        
        return True
    
    def run_npc_test_sequence(self):
        """Execute the npc_test waypoint sequence"""
        logging.info("Starting npc_test sequence")
        
        # Get goal points
        npc_test_goals = self.autoware_publisher.goal_sets["npc_test"]
        start_point = npc_test_goals[0]
        end_point = npc_test_goals[1]
        out_point = npc_test_goals[2]
        
        # First check if we're already at or near the start point
        logging.info("Checking if vehicle is near the start point...")
        
        # Wait up to 30 seconds checking for position
        max_wait_time = 30  # seconds
        start_time = time.time()
        position_check_interval = 0.5  # seconds
        
        at_start = False
        while time.time() - start_time < max_wait_time and self.running:
            # Process some ROS2 callbacks
            if ROS2_AVAILABLE:
                rclpy.spin_once(self.autoware_publisher, timeout_sec=0.1)
            
            # Check if we're at the start point
            distance = self.autoware_publisher.get_current_position_distance_to(start_point)
            logging.info(f"Distance to start point: {distance:.2f} meters")
            
            if distance < self.autoware_publisher.distance_threshold:
                at_start = True
                logging.info("Vehicle is at the start point!")
                break
            
            time.sleep(position_check_interval)
        
        if not at_start:
            logging.warning(f"Vehicle not at start point after {max_wait_time} seconds")
            logging.info("Continuing with sequence anyway...")
        
        # 1. Publish end point as goal
        logging.info("Publishing end point as goal...")
        self.autoware_publisher.publish_goal(end_point)
        
        # 2. Send engage command
        logging.info("Waiting 5 seconds before sending engage command...")
        time.sleep(5)  # Wait 5 seconds
        logging.info("Sending engage command...")
        self.autoware_publisher.publish_engage(True)
        
        # 3. Wait for vehicle to reach end point
        logging.info("Waiting for vehicle to reach end point...")
        reached_end = False
        
        while not reached_end and self.running:
            # Process some ROS2 callbacks
            if ROS2_AVAILABLE:
                rclpy.spin_once(self.autoware_publisher, timeout_sec=0.1)
            
            # Check if we're at the end point
            reached_end = self.autoware_publisher.check_if_at_position(end_point)
            if reached_end:
                logging.info("Vehicle has reached the end point!")
                break
            
            time.sleep(1)  # Check every second
        
        if not reached_end:
            logging.warning("Vehicle did not reach end point (possible script interruption)")
            return False
        
        # 4. Wait 10 seconds at end point
        logging.info("Waiting 10 seconds at end point...")
        time.sleep(10)
        
        # 5. Publish out point as goal
        logging.info("Publishing out point as goal...")
        self.autoware_publisher.publish_goal(out_point)
        
        # 6. Send engage command
        logging.info("Sending engage command...")
        self.autoware_publisher.publish_engage(True)
        
        # 7. Wait for vehicle to reach out point
        logging.info("Waiting for vehicle to reach out point...")
        reached_out = False
        
        while not reached_out and self.running:
            # Process some ROS2 callbacks
            if ROS2_AVAILABLE:
                rclpy.spin_once(self.autoware_publisher, timeout_sec=0.1)
            
            # Check if we're at the out point
            reached_out = self.autoware_publisher.check_if_at_position(out_point)
            if reached_out:
                logging.info("Vehicle has reached the out point!")
                break
            
            time.sleep(1)  # Check every second
        
        if not reached_out:
            logging.warning("Vehicle did not reach out point (possible script interruption)")
            return False
        
        # 8. Wait 5 seconds at out point
        logging.info("Waiting 5 seconds at out point...")
        time.sleep(5)
        
        # 9. npc_test sequence completed
        logging.info("npc_test sequence completed successfully")
        self.completion_flags["npc_test_completed"] = True
        
        return True
    
    def run_circle_sequence(self):
        """Execute the continuous circle waypoint sequence"""
        logging.info("Starting circle sequence")
        self.completion_flags["circle_started"] = True
        
        # Get circle waypoints
        circle_goals = self.autoware_publisher.goal_sets["circle"]
        
        # Initialize with the first goal
        current_goal_index = 0
        
        # Main loop to cycle through waypoints
        while self.running:
            # Get current goal
            current_goal = circle_goals[current_goal_index]
            next_index = (current_goal_index + 1) % len(circle_goals)
            next_goal = circle_goals[next_index]
            
            # 1. Publish current goal
            logging.info(f"Publishing circle goal {current_goal_index + 1}/{len(circle_goals)}...")
            self.autoware_publisher.publish_goal(current_goal)
            
            # 2. Send engage command
            logging.info("Waiting 3 seconds before sending engage command...")
            time.sleep(3)  # Wait 3 seconds
            logging.info("Sending engage command...")
            self.autoware_publisher.publish_engage(True)
            
            # 3. Wait for vehicle to reach current goal
            logging.info(f"Waiting for vehicle to reach goal {current_goal_index + 1}...")
            reached_goal = False
            
            while not reached_goal and self.running:
                # Process some ROS2 callbacks
                if ROS2_AVAILABLE:
                    rclpy.spin_once(self.autoware_publisher, timeout_sec=0.1)
                
                # Check if we're at the goal
                reached_goal = self.autoware_publisher.check_if_at_position(current_goal)
                if reached_goal:
                    logging.info(f"Vehicle has reached goal {current_goal_index + 1}!")
                    break
                
                time.sleep(1)  # Check every second
            
            if not self.running:
                logging.info("Circle sequence interrupted")
                break
            
            # 4. Wait 5 seconds at goal
            logging.info(f"Waiting 5 seconds at goal {current_goal_index + 1}...")
            time.sleep(5)
            
            # 5. Move to next goal in the circle
            current_goal_index = next_index
            logging.info(f"Moving to next goal: {current_goal_index + 1}/{len(circle_goals)}")
    
    def cleanup(self):
        """Clean up all resources"""
        logging.info("Starting cleanup process...")
        
        # Clean up CARLA resources
        if hasattr(self, 'carla_manager'):
            logging.info("Cleaning up CARLA vehicles...")
            vehicles_destroyed = self.carla_manager.cleanup_vehicles()
            logging.info(f"Destroyed {vehicles_destroyed} vehicles")
        
        # Clean up ROS2 resources
        if ROS2_AVAILABLE and hasattr(self, 'autoware_publisher') and self.autoware_publisher:
            logging.info("Cleaning up ROS2 resources...")
            try:
                self.autoware_publisher.destroy_node()
                if rclpy.ok():
                    rclpy.shutdown()
                logging.info("ROS2 resources cleaned up")
            except Exception as e:
                logging.error(f"Error cleaning up ROS2 resources: {e}")
        
        logging.info("Cleanup completed")


def parse_arguments():
    """Parse command line arguments"""
    argparser = argparse.ArgumentParser(
        description=__doc__)
    argparser.add_argument(
        '--host',
        metavar='H',
        default='127.0.0.1',
        help='IP of the host server (default: 127.0.0.1)')
    argparser.add_argument(
        '-p', '--port',
        metavar='P',
        default=2000,
        type=int,
        help='TCP port to listen to (default: 2000)')
    argparser.add_argument(
        '--tm-port',
        metavar='P',
        default=8100,
        type=int,
        help='Port to communicate with TM (default: 8100)')
    argparser.add_argument(
        '--alt-tm-port',
        metavar='P',
        default=8200,
        type=int,
        help='Alternative TM port if primary is in use (default: 8200)')
    argparser.add_argument(
        '--seed',
        metavar='S',
        type=int,
        default=42,
        help='Set random seed for reproducibility (default: 42)')
    argparser.add_argument(
        '--asynch',
        action='store_true',
        help='Activate asynchronous mode execution')
    argparser.add_argument(
        '--num-vehicles',
        type=int,
        default=5,
        help='Number of NPC vehicles to spawn (default: 5)')
    argparser.add_argument(
        '--safe-mode',
        action='store_true',
        default=True,
        help='Enable safer driving parameters for NPCs')
    argparser.add_argument(
        '--batch-spawn',
        action='store_true',
        help='Spawn vehicles in batches for better performance')
    argparser.add_argument(
        '--config',
        type=str,
        help='Path to configuration JSON file')
    argparser.add_argument(
        '--log-file',
        type=str,
        help='Path to log file')
    
    args = argparser.parse_args()
    
    # If configuration file provided, load from it
    if args.config and os.path.exists(args.config):
        try:
            import json
            with open(args.config, 'r') as f:
                config = json.load(f)
                
                # Update arguments
                args.host = config.get('host', args.host)
                args.port = config.get('port', args.port)
                args.num_vehicles = config.get('number-of-vehicles', args.num_vehicles)
                args.seed = config.get('seed', args.seed)
                args.safe_mode = config.get('safe', args.safe_mode)
                args.asynch = config.get('asynch', args.asynch)
                
                logging.info(f"Loaded settings from config file: {args.config}")
        except Exception as e:
            logging.error(f"Error loading config file: {e}")
    
    return args


def main():
    # Set up logger
    logger = Logger.setup()
    
    # Parse command line arguments
    args = parse_arguments()
    
    # If log file argument provided, reset logger
    if args.log_file:
        logger = Logger.setup(args.log_file)
    
    # Log script start info
    logging.info("="*50)
    logging.info(f"Script start: integrated_scenario_test.py")
    logging.info(f"Start time: {datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
    logging.info(f"Arguments: {vars(args)}")
    logging.info("="*50)
    
    # Create integrated manager
    manager = IntegratedManager(args)
    
    try:
        # Initialize connections and set up
        if not manager.initialize():
            logging.error("Initialization failed")
            return
        
        # Run npc_test sequence
        if not manager.run_npc_test_sequence():
            logging.warning("npc_test sequence did not complete successfully")
        
        # Clean up NPC vehicles after npc_test is complete
        if manager.completion_flags["npc_test_completed"]:
            logging.info("npc_test completed, removing NPC vehicles...")
            manager.carla_manager.cleanup_vehicles()
            logging.info("Waiting 5 seconds after NPC removal...")
            time.sleep(5)
        
        # Start circle sequence
        if manager.running:
            manager.run_circle_sequence()
    
    except KeyboardInterrupt:
        logging.info('\nSimulation interrupted by user')
    except Exception as e:
        logging.error(f'Encountered error: {e}')
        # Print detailed stack trace
        import traceback
        logging.error(traceback.format_exc())
    finally:
        # Clean up resources
        logging.info('Cleaning up simulation...')
        try:
            manager.cleanup()
        except Exception as e:
            logging.error(f'Error during cleanup: {e}')
        
        logging.info('Simulation ended')
        logging.info("="*50)
        logging.info(f"Script end time: {datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
        logging.info("="*50)


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        logging.info('\nScript interrupted by keyboard')
    except Exception as e:
        logging.error(f'Runtime error: {e}')
    finally:
        logging.info('\nScript terminated')