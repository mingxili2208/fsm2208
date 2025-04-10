#!/usr/bin/env python3

"""
This module provides a launcher for spawning and controlling vehicles in CARLA simulator.
"""

import os
import cv2
import sys
import time
import signal
import pygame
import random
import importlib
import threading
import traceback
from abc import ABC, abstractmethod

import carla
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
from transforms3d.euler import quat2euler
import math

from leaderboard.envs.sensor_interface import SensorReceivedNoData
from leaderboard.autoagents.agent_wrapper import AgentWrapper, AgentError
from srunner.scenariomanager.timer import GameTime
from srunner.scenariomanager.watchdog import Watchdog
from srunner.scenariomanager.carla_data_provider import CarlaDataProvider


class BridgeHelpers:
    """Helper class for CARLA-ROS bridge operations."""
    
    @staticmethod
    def get_agent_actor(world, role_name):
        """Find an actor with the specified role name."""
        actors = world.get_actors().filter("vehicle.*")
        for car in actors:
            if car.attributes["role_name"] == role_name:
                return car
        return None


class VehicleSpawner(ABC):
    """Abstract base class for vehicle spawners."""
    
    @abstractmethod
    def initialize(self):
        """Initialize the spawner."""
        pass
    
    @abstractmethod
    def spawn_vehicle(self):
        """Spawn the vehicle."""
        pass
    
    @abstractmethod
    def cleanup(self):
        """Clean up spawned resources."""
        pass


class RemoteVehicleSpawner(Node, VehicleSpawner):
    """Spawner for vehicles that follow a remote vehicle's pose."""
    
    def __init__(self, role_name, model_type, map_name):
        """Initialize the spawner with vehicle details."""
        super().__init__('ego_vehicle_launcher')
        
        self.local_host = os.environ["SIMULATOR_LOCAL_HOST"]
        self.port = int(os.environ["SIMULATOR_PORT"])
        self.agent_role_name = role_name
        self.agent_model_type = model_type
        self.map_name = map_name
        
        # Initialize client and world
        self.client = None
        self.world = None
        
        # Initialize pose tracking
        self.pose_initialized = False
        self.spawn_point = None
        
        # Initialize the pose subscription
        self.subscription = self.create_subscription(
            PoseWithCovarianceStamped,
            '/real_world/follow_adtruck/transformed_with_covariance',
            self.pose_callback,
            10
        )
    
    def initialize(self):
        """Initialize the CARLA client and world."""
        self.client = carla.Client(self.local_host, self.port)
        self.client.set_timeout(20)
        self.world = self.client.get_world()
        
        # Initialize CARLA data provider
        CarlaDataProvider.set_world(self.world)
        CarlaDataProvider.set_client(self.client)
        
        return self.client
    
    def pose_callback(self, msg):
        """Callback for pose updates from the remote vehicle."""
        self.get_logger().info(f"Received PoseWithCovarianceStamped")
        
        # Extract position and orientation from the message
        self.spawn_point = carla.Transform()
        self.spawn_point.location.x = msg.pose.pose.position.x
        self.spawn_point.location.y = -msg.pose.pose.position.y  # ROS to CARLA coordinate conversion
        self.spawn_point.location.z = msg.pose.pose.position.z
        
        # Convert quaternion to Euler angles
        orientation_q = msg.pose.pose.orientation
        roll, pitch, yaw = quat2euler([orientation_q.w, orientation_q.x, orientation_q.y, orientation_q.z])
        
        # Set rotation in CARLA format
        self.spawn_point.rotation.roll = math.degrees(roll)
        self.spawn_point.rotation.pitch = math.degrees(pitch)
        self.spawn_point.rotation.yaw = -math.degrees(yaw)  # Coordinate system conversion
        
        self.pose_initialized = True
    
    def spawn_vehicle(self):
        """Spawn the vehicle at the pose received from the remote vehicle."""
        # Wait for pose to be initialized
        while not self.pose_initialized:
            self.get_logger().info("Waiting for initial pose...")
            rclpy.spin_once(self, timeout_sec=1.0)
        
        # Check if vehicle already exists
        if BridgeHelpers.get_agent_actor(self.world, self.agent_role_name) is None:
            # Generate random color
            color = ",".join([str(random.randint(0, 255)) for _ in range(3)])
            
            # Spawn the vehicle
            CarlaDataProvider.request_new_actor(
                self.agent_model_type, 
                self.spawn_point,
                self.agent_role_name,
                color=color
            )
            
            self.get_logger().info(f"Vehicle {self.agent_role_name} spawned successfully")
        else:
            CarlaDataProvider.cleanup()
            raise RuntimeError(f"Vehicle {self.agent_role_name} already exists!")
    
    def cleanup(self):
        """Clean up spawned resources."""
        vehicle = BridgeHelpers.get_agent_actor(self.world, self.agent_role_name)
        if vehicle is not None:
            vehicle.destroy()
        CarlaDataProvider.cleanup()


class SimpleVehicleSpawner(VehicleSpawner):
    """Spawner for vehicles with a predefined spawn point."""
    
    def __init__(self, role_name, model_type, spawn_point=None):
        """Initialize the spawner with vehicle details."""
        self.local_host = os.environ["SIMULATOR_LOCAL_HOST"]
        self.port = int(os.environ["SIMULATOR_PORT"])
        self.agent_role_name = role_name
        self.agent_model_type = model_type
        self.client = None
        self.world = None
        
        # Parse spawn point if provided
        if spawn_point and spawn_point != "0":
            parts = spawn_point.split(",")
            if len(parts) >= 6:
                self.spawn_point = carla.Transform(
                    carla.Location(float(parts[0]), float(parts[1]), float(parts[2])),
                    carla.Rotation(float(parts[3]), float(parts[4]), float(parts[5]))
                )
            else:
                self.spawn_point = None
        else:
            self.spawn_point = None
    
    def initialize(self):
        """Initialize the CARLA client and world."""
        self.client = carla.Client(self.local_host, self.port)
        self.client.set_timeout(20)
        self.world = self.client.get_world()
        
        # Initialize CARLA data provider
        CarlaDataProvider.set_world(self.world)
        CarlaDataProvider.set_client(self.client)
        
        return self.client
    
    def spawn_vehicle(self):
        """Spawn the vehicle at the predefined or random spawn point."""
        # Check if vehicle already exists
        if BridgeHelpers.get_agent_actor(self.world, self.agent_role_name) is None:
            # Generate random color
            color = ",".join([str(random.randint(0, 255)) for _ in range(3)])
            
            # Get spawn point
            if self.spawn_point is None:
                spawn_points = self.world.get_map().get_spawn_points()
                if spawn_points:
                    self.spawn_point = random.choice(spawn_points)
                else:
                    self.spawn_point = carla.Transform(
                        carla.Location(0, 0, 0),
                        carla.Rotation(0, 0, 0)
                    )
            
            # Spawn the vehicle
            CarlaDataProvider.request_new_actor(
                self.agent_model_type, 
                self.spawn_point,
                self.agent_role_name,
                color=color
            )
            
            print(f"Vehicle {self.agent_role_name} spawned successfully")
        else:
            CarlaDataProvider.cleanup()
            raise RuntimeError(f"Vehicle {self.agent_role_name} already exists!")
    
    def cleanup(self):
        """Clean up spawned resources."""
        vehicle = BridgeHelpers.get_agent_actor(CarlaDataProvider.get_world(), self.agent_role_name)
        if vehicle is not None:
            vehicle.destroy()
        CarlaDataProvider.cleanup()


class VehicleLoop:
    """Manages the main control loop for the vehicle."""
    
    def __init__(self, role_name):
        """Initialize the vehicle loop."""
        self.start_game_time = None
        self.start_system_time = None
        self.debug_mode = False
        self.agent = None
        self.ego_vehicle = None
        self.running = False
        self.agent_vis = None
        self.timestamp_last_run = 0.0
        self.timeout = 20.0
        self.role_name = role_name
        
        # Set up watchdog for agent timeout
        watchdog_timeout = max(5, self.timeout - 2)
        agent_timeout = watchdog_timeout - 1
        self.agent_watchdog = Watchdog(agent_timeout)
    
    def stop_loop(self):
        """Stop the vehicle control loop."""
        self.running = False
    
    def tick_agent(self, timestamp, camera_snapshots):
        """Execute one tick of the agent's control loop."""
        if self.timestamp_last_run < timestamp.elapsed_seconds and self.running:
            self.timestamp_last_run = timestamp.elapsed_seconds
            
            # Update game time and CARLA data provider
            GameTime.on_carla_tick(timestamp)
            CarlaDataProvider.on_carla_tick()
            
            try:
                # Get control action from agent
                ego_action = self.agent()
                
                # Handle pygame visualization if needed
                if os.environ["CONTROL_MODE"] == "pygame":
                    self.agent_vis.control = ego_action
                
                # Update visualization
                self.agent_vis.run(camera_snapshots[0], camera_snapshots[1], camera_snapshots[2])
            
            except SensorReceivedNoData as e:
                raise RuntimeError(e)
            except Exception as e:
                raise AgentError(e)
            
            # Apply control if not in pygame mode
            if os.environ["CONTROL_MODE"] != "pygame":
                self.ego_vehicle.apply_control(ego_action)
            
            # Check if vehicle still exists
            if BridgeHelpers.get_agent_actor(CarlaDataProvider.get_world(), self.role_name) is None:
                self.running = False
        
        # Tick the world if still running
        if self.running:
            CarlaDataProvider.get_world().tick()


class VehicleHandler:
    """Manages the vehicle agent and its lifecycle."""
    
    def __init__(self, client=None):
        """Initialize the vehicle handler."""
        # Get agent details from environment
        self.agent_role_name = os.environ["AGENT_ROLE_NAME"].replace("-", "_")
        
        # Import the agent module
        agent_path = os.environ["TEAM_AGENT"]
        module_name = os.path.basename(agent_path).split(".")[0]
        sys.path.insert(0, os.path.dirname(agent_path))
        module_agent = importlib.import_module(module_name)
        
        # Get agent class and instantiate it
        agent_class_name = getattr(module_agent, "get_entry_point")()
        self.agent_instance = getattr(module_agent, agent_class_name)("")
        self.agent_wrapper = AgentWrapper(self.agent_instance)
        
        # Get the ego vehicle
        self.ego_vehicle = BridgeHelpers.get_agent_actor(
            CarlaDataProvider.get_world(), 
            self.agent_role_name
        )
        
        # Initialize visualization
        self.agent_vis = VehicleTerminal(client)
        
        # Set up the vehicle and sensors
        if self.ego_vehicle is not None:
            print(f"Ego Vehicle: {self.agent_role_name}")
            self.agent_wrapper.setup_sensors(self.ego_vehicle, False)
            self.ego_vehicle.set_simulate_physics(True)
        else:
            print(f"Can't Load Ego Vehicle {self.agent_role_name}!! Agent will exit.")
            
            # Clean up
            CarlaDataProvider.cleanup()
            self.agent_wrapper.cleanup()
            
            if self.agent_instance:
                self.agent_instance.destroy()
                self.agent_instance = None
                
            raise Exception("Can't initialize agent ego_vehicle!")
        
        # Initialize control loop
        self.agent_loop = None
    
    def run_agent(self):
        """Run the agent control loop."""
        try:
            # Set up the agent loop
            self.agent_loop = VehicleLoop(self.agent_role_name)
            self.agent_loop.agent = self.agent_wrapper
            self.agent_loop.ego_vehicle = self.ego_vehicle
            self.agent_loop.start_system_time = time.time()
            self.agent_loop.start_game_time = GameTime.get_time()
            self.agent_loop.role_name = self.agent_role_name
            self.agent_loop.running = True
            self.agent_loop.agent_vis = self.agent_vis
            
            # Run the loop
            while self.agent_loop.running:
                # Get the current timestamp
                timestamp = None
                world = CarlaDataProvider.get_world()
                
                if world:
                    snapshot = world.get_snapshot()
                    if snapshot:
                        timestamp = snapshot.timestamp
                
                # Tick the agent if timestamp is available
                if timestamp:
                    self.agent_loop.tick_agent(
                        timestamp, 
                        self.agent_instance.get_camera_snapshots()
                    )
                    
        except Exception as e:
            traceback.print_exc()
    
    def stop_loop(self, signum, frame):
        """Stop the agent loop in response to a signal."""
        if self.agent_loop:
            self.agent_loop.stop_loop()
    
    def cleanup(self):
        """Clean up all resources."""
        self.agent_wrapper.cleanup()
        self.agent_vis.stop()
        
        if self.ego_vehicle:
            self.ego_vehicle.destroy()
            self.ego_vehicle = None
        
        if self.agent_instance:
            self.agent_instance.destroy()
            self.agent_instance = None


class VehicleTerminal:
    """Handles visualization and keyboard input for the vehicle."""
    
    def __init__(self, client=None):
        """Initialize the vehicle terminal."""
        # Get configuration from environment
        self.agent_role_name = os.environ["AGENT_ROLE_NAME"].replace("-", "_")
        self.control_mode = os.environ["CONTROL_MODE"]
        self.tm_port = int(os.environ["TRAFFIC_MANAGER_PORT"])
        self.frame_rate = float(os.environ["AGENT_FRAME_RATE"])
        
        # Store client reference
        self.client = client
        self.traffic_manager = None
        
        # Set up topic base
        self.topic_base = "" if self.control_mode.lower() == "autoware" else f"/carla/{self.agent_role_name}"
        
        # Initialize pygame for visualization
        pygame.init()
        self.screen_width = 600
        self.screen_height = 500
        self.screen = pygame.display.set_mode((self.screen_width, self.screen_height), pygame.RESIZABLE)
        pygame.display.set_caption(f"Ego-Vehicle {self.agent_role_name}'s terminal")
        
        # Set up pygame clock
        self.clock = pygame.time.Clock()
        
        # Initialize control variables
        self.autopilot = False
        self.control = None
        self.throttle = False
        self.brake = False
        self.steer = None
        self.steer_cache = 0
        
        # Configure throttle limits based on running mode
        self.max_throttle = 0.15 if os.environ.get("RUNNING_MODE", "normal") == "record" else 0.5
        self.reverse_max_throttle = 0.25 if os.environ.get("RUNNING_MODE", "normal") == "record" else 0.5
        
        # Get ego vehicle
        self.ego_vehicle = BridgeHelpers.get_agent_actor(
            CarlaDataProvider.get_world(), 
            self.agent_role_name
        )
    
    def visualization(self, size=None, center_camera=None, back_camera=None, bev_camera=None):
        """Visualize camera feeds in the pygame window."""
        # Combine camera views if available
        if center_camera is not None and back_camera is not None and bev_camera is not None:
            # Create a vertical stack of center and back cameras
            vis_camera = cv2.vconcat([center_camera, back_camera])
            # Add BEV camera alongside
            vis_camera = cv2.hconcat([vis_camera, bev_camera])
        else:
            vis_camera = None
            print("No image received!")
        
        # Display the combined camera view
        if vis_camera is not None:
            if size is not None:
                vis_camera = cv2.resize(vis_camera, size)
            
            # Convert to pygame surface and display
            image_surface = pygame.surfarray.make_surface(
                cv2.cvtColor(vis_camera, cv2.COLOR_BGR2RGB)
            )
            self.screen.blit(image_surface, (0, 0))
    
    def stop(self):
        """Clean up pygame resources."""
        self.ego_vehicle = None
        pygame.quit()
    
    def run(self, center_camera=None, back_camera=None, bev_camera=None):
        """Process input events and update visualization."""
        # Process pygame events
        for event in pygame.event.get():
            # Handle window close
            if event.type == pygame.QUIT:
                pygame.quit()
            
            # Handle window resize
            if event.type == pygame.VIDEORESIZE:
                size = event.size
                self.screen_width, self.screen_height = size
            
            # Handle keyboard input if control is available
            if self.control is not None:
                # Key down events
                if event.type == pygame.KEYDOWN:
                    # Toggle autopilot with 'c'
                    if event.key == pygame.K_c:
                        print("Pygame control mode give to autopilot in carla! "
                              "Please press 'p' to go back to window control (WASD).")
                        self.autopilot = True
                        self.control.throttle = 0.0
                        self.control.brake = 0.0
                        self.ego_vehicle.apply_control(self.control)
                        
                        # Set up traffic manager if client is available
                        if self.client is not None:
                            self._setup_traffic_manager()
                        else:
                            raise ValueError("Carla client failed!")
                        
                        # Enable autopilot for ego vehicle
                        if self.traffic_manager is not None:
                            self.ego_vehicle.set_autopilot(True, self.traffic_manager.get_port())
                    
                    # Disable autopilot with 'p'
                    if event.key == pygame.K_p:
                        print("Pygame control mode give to window (WASD)! "
                              "Please press 'c' to go back to autopilot control in carla (WASD).")
                        self.autopilot = False
                        self.control.throttle = 0.0
                        self.control.brake = 0.0
                        self.ego_vehicle.apply_control(self.control)
                        
                        # Restore settings
                        settings = self.client.get_world().get_settings()
                        settings.synchronous_mode = False
                        settings.fixed_delta_seconds = 1.0 / self.frame_rate
                        self.client.get_world().apply_settings(settings)
                        
                        # Disable autopilot
                        if self.traffic_manager is not None:
                            self.ego_vehicle.set_autopilot(False, self.traffic_manager.get_port())
                        
                        self.traffic_manager = None
                    
                    # WASD controls
                    if event.key == pygame.K_UP or event.key == pygame.K_w:
                        self.throttle = True
                    if event.key == pygame.K_DOWN or event.key == pygame.K_s:
                        self.brake = True
                    if event.key == pygame.K_RIGHT or event.key == pygame.K_a:
                        self.steer = -1
                    if event.key == pygame.K_LEFT or event.key == pygame.K_d:
                        self.steer = 1
                
                # Key up events
                if event.type == pygame.KEYUP:
                    if event.key == pygame.K_UP or event.key == pygame.K_w:
                        self.throttle = False
                    if event.key == pygame.K_DOWN or event.key == pygame.K_s:
                        self.brake = False
                        self.control.reverse = False
                    if event.key == pygame.K_RIGHT or event.key == pygame.K_a:
                        self.steer = None
                    if event.key == pygame.K_LEFT or event.key == pygame.K_d:
                        self.steer = None
                
                # Apply throttle
                if self.throttle:
                    self.control.throttle = min(self.control.throttle + 0.1, self.max_throttle)
                    self.control.gear = 1
                    self.control.brake = False
                elif not self.brake:
                    self.control.throttle = 0.0
                
                # Apply brake
                if self.brake:
                    if self.ego_vehicle.get_velocity().length() < 0.01 and not self.control.reverse:
                        # Switch to reverse
                        self.control.brake = 0.0
                        self.control.gear = 1
                        self.control.reverse = True
                        self.control.throttle = min(self.control.throttle + 0.1, self.max_throttle)
                    elif self.control.reverse:
                        # Apply reverse throttle
                        self.control.throttle = min(self.control.throttle + 0.1, self.reverse_max_throttle)
                    else:
                        # Apply brake
                        self.control.throttle = 0.0
                        if self.ego_vehicle.get_velocity().length() <= 0.0:
                            self.control.brake = 0.75
                        elif self.ego_vehicle.get_velocity().length() > -1:
                            self.control.brake = 0.0
                        else:
                            self.control.brake = 0.01
                else:
                    self.control.brake = 0.0
                
                # Apply steering
                if self.steer is not None:
                    if self.steer == 1:
                        self.steer_cache += 0.03
                    if self.steer == -1:
                        self.steer_cache -= 0.03
                    self.steer_cache = min(0.7, max(-0.7, self.steer_cache))
                    self.control.steer = round(self.steer_cache, 1)
                else:
                    # Gradually reset steering when no input
                    if self.steer_cache > 0.0:
                        self.steer_cache *= 0.2
                    if self.steer_cache < 0.0:
                        self.steer_cache *= 0.2
                    if abs(self.steer_cache) < 0.01:
                        self.steer_cache = 0.0
                    self.control.steer = round(self.steer_cache, 1)
                
                # Apply control if not in autopilot mode
                if not self.autopilot:
                    self.ego_vehicle.apply_control(self.control)
        
        # Update visualization
        self.visualization((self.screen_height, self.screen_width), center_camera, back_camera, bev_camera)
        pygame.display.flip()
    
    def _setup_traffic_manager(self):
        """Set up the traffic manager."""
        self.traffic_manager = self.client.get_trafficmanager(self.tm_port)
        self.traffic_manager.set_global_distance_to_leading_vehicle(100.0)
        self.traffic_manager.set_random_device_seed(0)
        self.traffic_manager.set_synchronous_mode(True)
        
        # Configure world settings
        settings = self.client.get_world().get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = 0.05
        self.client.get_world().apply_settings(settings)
        
        print("Traffic Manager initialized")


def main():
    """Main function to spawn and run the ego vehicle."""
    # Initialize ROS if needed
    if os.environ["CONTROL_MODE"] == "follow":
        rclpy.init(args=None)
    
    # Validate control and running modes
    if os.environ["CONTROL_MODE"] != "pygame" and os.environ["RUNNING_MODE"] == "record":
        raise ValueError("Recording mode is only supported with pygame control mode")
    
    # Choose appropriate vehicle spawner based on control mode
    if os.environ["CONTROL_MODE"] == "follow":
        # Use remote vehicle spawner
        ego_vehicle_spawner = RemoteVehicleSpawner(
            os.environ["AGENT_ROLE_NAME"].replace("-", "_"),
            os.environ["AGENT_MODEL_TYPE"],
            os.environ["FREE_MAP_NAME"]
        )
    else:
        # Use simple vehicle spawner
        ego_vehicle_spawner = SimpleVehicleSpawner(
            os.environ["AGENT_ROLE_NAME"].replace("-", "_"),
            os.environ["AGENT_MODEL_TYPE"],
            os.environ.get("FREE_AGENT_POSE", "0")
        )
    
    # Initialize the spawner and get client
    client = ego_vehicle_spawner.initialize()
    
    try:
        # Spawn the vehicle
        ego_vehicle_spawner.spawn_vehicle()
        
        # Initialize the vehicle handler
        ego_vehicle_handler = VehicleHandler(client)
    except Exception as e:
        ego_vehicle_spawner.cleanup()
        traceback.print_exc()
        sys.exit(1)
    
    # Set up signal handler
    old_handler = signal.signal(signal.SIGINT, ego_vehicle_handler.stop_loop)
    
    try:
        # Run the agent
        ego_vehicle_handler.run_agent()
    except Exception as e:
        traceback.print_exc()
    
    print(f"The vehicle named {os.environ['AGENT_ROLE_NAME']} finished simulation!")
    
    # Restore signal handler
    signal.signal(signal.SIGINT, old_handler)
    
    # Clean up
    ego_vehicle_handler.cleanup()
    ego_vehicle_spawner.cleanup()
    
    # Shut down ROS if initialized
    if os.environ["CONTROL_MODE"] == "follow":
        rclpy.shutdown()


if __name__ == "__main__":
    main()