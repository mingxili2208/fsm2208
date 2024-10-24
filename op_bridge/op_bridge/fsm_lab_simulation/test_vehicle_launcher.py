import os
import cv2
import sys
import time
import signal
import pygame
import random
import importlib
import traceback

import carla
import rclpy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from leaderboard.envs.sensor_interface import SensorReceivedNoData
from leaderboard.autoagents.agent_wrapper import AgentWrapper, AgentError
from srunner.scenariomanager.timer import GameTime
from srunner.scenariomanager.watchdog import Watchdog
from srunner.scenariomanager.carla_data_provider import CarlaDataProvider
from geometry_msgs.msg import PoseWithCovarianceStamped
from transforms3d.euler import quat2euler
from rclpy.node import Node 
import math


class BridgeHelpers(object):
    @staticmethod
    def get_agent_actor(world, role_name):
        actors = world.get_actors().filter("vehicle.*")
        for car in actors:
            if car.attributes["role_name"] == role_name:
                return car
        return None

class EgoVehicleLoop(object):
    def __init__(self, role_name):
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
        watchdog_timeout = max(5, self.timeout - 2)
        agent_timeout = watchdog_timeout - 1
        self.agent_watchdog = Watchdog(agent_timeout)
    
    def stop_loop(self):
        self.running = False
    
    def tick_agent(self, timestamp, camera_snapshots):
        if self.timestamp_last_run < timestamp.elapsed_seconds and self.running:
            self.timestamp_last_run = timestamp.elapsed_seconds

            GameTime.on_carla_tick(timestamp)
            CarlaDataProvider.on_carla_tick()

            try:
                ego_action = self.agent()
                if os.environ["CONTROL_MODE"] == "pygame":
                    self.agent_vis.control = ego_action
                self.agent_vis.run(camera_snapshots[0], camera_snapshots[1], camera_snapshots[2])
            except SensorReceivedNoData as e:
                raise RuntimeError(e)
            except Exception as e:
                raise AgentError(e)
            
            if os.environ["CONTROL_MODE"] != "pygame":
                self.ego_vehicle.apply_control(ego_action)

            if BridgeHelpers.get_agent_actor(CarlaDataProvider.get_world(), self.role_name) is None:
                self.running = False
        
        if self.running:
            CarlaDataProvider.get_world().tick()
class EgoVehicleLauncher(Node):
    def __init__(self):
        super().__init__('ego_vehicle_launcher')

        self.local_host = os.environ["SIMULATOR_LOCAL_HOST"]
        self.port = int(os.environ["SIMULATOR_PORT"])
        self.agent_role_name = os.environ["AGENT_ROLE_NAME"].replace("-", "_")
        self.agent_model_type = os.environ["AGENT_MODEL_TYPE"]
        self.map_name = os.environ["FREE_MAP_NAME"]
        
        # 订阅 PoseWithCovarianceStamped 话题
        self.subscription = self.create_subscription(
            PoseWithCovarianceStamped,
            '/real_world/follow_adtruck/transformed_with_covariance',
            self.pose_callback,
            10)
        
        #self.ego_vehicle = None
        self.world = None
        self.pose_initialized = False  # 标记是否已收到位姿消息
        self.spawn_point = None

    def pose_callback(self, msg):
        """
        回调函数，接收来自 /real_world/follow_adtruck/transformed_with_covariance 话题的位姿信息
        """
        self.get_logger().info(f"x={msg.pose.pose.position.x}, y={msg.pose.pose.position.y}, z={msg.pose.pose.position.z}")
        
        # 使用接收到的位姿消息来设置生成点
        self.spawn_point = carla.Transform()
        self.spawn_point.location.x = msg.pose.pose.position.x
        self.spawn_point.location.y = -msg.pose.pose.position.y #ros =- carla 
        self.spawn_point.location.z = msg.pose.pose.position.z 

        # 四元数转欧拉角（CARLA 使用欧拉角来表示旋转）
        orientation_q = msg.pose.pose.orientation
        roll, pitch, yaw = quat2euler([orientation_q.w,orientation_q.x, orientation_q.y, orientation_q.z])
        self.spawn_point.rotation.roll = math.degrees(roll)
        self.spawn_point.rotation.pitch = math.degrees(pitch)
        self.spawn_point.rotation.yaw = -math.degrees(yaw) #-90

        self.pose_initialized = True  # 标记为已初始化

    def quaternion_to_euler(self, x, y, z, w):
        """
        将四元数转换为欧拉角
        """
        import math
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll = math.atan2(t0, t1)

        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch = math.asin(t2)

        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(t3, t4)

        return roll, pitch, yaw

    def load_vehicle(self):
        """
        加载车辆，确保已经接收到位姿信息
        """
        self.client = carla.Client(self.local_host, self.port)
        self.client.set_timeout(20)
        self.world = self.client.get_world()
        CarlaDataProvider.set_world(self.world)
        CarlaDataProvider.set_client(self.client)

        # 等待直到接收到位姿消息
        while not self.pose_initialized:
            self.get_logger().info("Waiting for initial pose from /real_world/follow_adtruck/transformed_with_covariance...")
            rclpy.spin_once(self, timeout_sec=1.0)

        # 确保车辆未被重复生成
        if BridgeHelpers.get_agent_actor(CarlaDataProvider.get_world(), self.agent_role_name) is None:
            red = str(random.randint(0, 255))
            blue = str(random.randint(0, 255))
            green = str(random.randint(0, 255))
            ego_vehicle_color = ",".join([red, blue, green])

            CarlaDataProvider.request_new_actor(self.agent_model_type, self.spawn_point, self.agent_role_name, color=ego_vehicle_color)

            
        else:
            CarlaDataProvider.cleanup()
            raise RuntimeError(f"The vehicle {self.agent_role_name} has been spawned!")

        # 返回 client
        return self.client

    def cleanup(self):
        if self.client:
            self.client.close()
            self.client = None
        vehicle = BridgeHelpers.get_agent_actor(CarlaDataProvider.get_world(), self.agent_role_name)
        if vehicle is not None:
            vehicle.destroy()
        CarlaDataProvider.cleanup()


class EgoVehicleLoop(object):
    def __init__(self, role_name):
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
        watchdog_timeout = max(5, self.timeout - 2)
        agent_timeout = watchdog_timeout - 1
        self.agent_watchdog = Watchdog(agent_timeout)
    
    def stop_loop(self):
        self.running = False
    
    def tick_agent(self, timestamp, camera_snapshots):
        if self.timestamp_last_run < timestamp.elapsed_seconds and self.running:
            self.timestamp_last_run = timestamp.elapsed_seconds

            GameTime.on_carla_tick(timestamp)
            CarlaDataProvider.on_carla_tick()

            try:
                ego_action = self.agent()
                if os.environ["CONTROL_MODE"] == "pygame":
                    self.agent_vis.control = ego_action
                self.agent_vis.run(camera_snapshots[0], camera_snapshots[1], camera_snapshots[2])
            except SensorReceivedNoData as e:
                raise RuntimeError(e)
            except Exception as e:
                raise AgentError(e)
            
            if os.environ["CONTROL_MODE"] != "pygame":
                self.ego_vehicle.apply_control(ego_action)

            if BridgeHelpers.get_agent_actor(CarlaDataProvider.get_world(), self.role_name) is None:
                self.running = False
        
        if self.running:
            CarlaDataProvider.get_world().tick()


class EgoVehicleHandler(object):
    def __init__(self, client = None):
        self.agent_role_name = os.environ["AGENT_ROLE_NAME"].replace("-", "_")
        agent_path = os.environ["TEAM_AGENT"]
        module_name = os.path.basename(agent_path).split(".")[0]
        sys.path.insert(0, os.path.dirname(agent_path))
        module_agent = importlib.import_module(module_name)
        agent_class_name = getattr(module_agent, "get_entry_point")()
        self.agent_instance = getattr(module_agent, agent_class_name)("")
        self.agent_wrapper = AgentWrapper(self.agent_instance)
        self.ego_vehicle = BridgeHelpers.get_agent_actor(CarlaDataProvider.get_world(), self.agent_role_name)

        #self.ego_vehicle.set_simulate_physics(False)  # autoware need the simulate_physics

        ##########
        print(f"ego_vehicle is : {self.ego_vehicle}")
        ##########
        self.agent_vis = EgoVehicleTerminal(client)
        if self.ego_vehicle is not None:
            print("Ego Vehicle: ", self.ego_vehicle.attributes["role_name"])
            self.agent_wrapper.setup_sensors(self.ego_vehicle, False)
            self.ego_vehicle.set_simulate_physics(True)
        else:
            print(f"Can't Load Ego Vehicle {self.agent_role_name}!! Agent will exit.")
            # Check whether there are other actors in the world.
            # if len(CarlaDataProvider.get_world().get_actors().filter("vehicle.*")) <= 1:
            CarlaDataProvider.cleanup()
            self.agent_wrapper.cleanup()
            if self.agent_instance:
                self.agent_instance.destroy()
                self.agent_instance = None
            raise Exception("Can't initialize agent ego_vehicle!")
    
    def run_agent(self):
        try:
            self.agent_loop = EgoVehicleLoop(self.agent_role_name)
            self.agent_loop.agent = self.agent_wrapper
            self.agent_loop.ego_vehicle = self.ego_vehicle
            self.agent_loop.start_system_time = time.time()
            self.agent_loop.start_game_time = GameTime.get_time()
            self.agent_loop.role_name = self.agent_role_name
            self.agent_loop.running = True
            self.agent_loop.agent_vis = self.agent_vis
            while self.agent_loop.running:
                timestamp = None
                world = CarlaDataProvider.get_world()
                if world:
                    snapshot = world.get_snapshot()
                    if snapshot:
                        timestamp = snapshot.timestamp
                if timestamp:
                    self.agent_loop.tick_agent(timestamp, self.agent_instance.get_camera_snapshots())
        except Exception as e:
            traceback.print_exc()
    
    def stop_loop(self, signum, frame):
        self.agent_loop.stop_loop()
    
    def cleanup(self):
        if self.client:
            self.client.close()
            self.client = None
        self.agent_wrapper.cleanup()
        self.agent_vis.stop()

        if self.ego_vehicle:
            self.ego_vehicle.destroy()
            self.ego_vehicle = None
        
        if self.agent_instance:
            self.agent_instance.destroy()
            self.agent_instance = None


class EgoVehicleTerminal(object):
    def __init__(self, client = None):
        self.agent_role_name = os.environ["AGENT_ROLE_NAME"].replace("-", "_")
        self.control_mode = os.environ["CONTROL_MODE"]
        self.tm_port = int(os.environ["TRAFFIC_MANAGER_PORT"])
        self.frame_rate = float(os.environ["AGENT_FRAME_RATE"])
        self.client = client
        self.traffic_manager = None
        self.topic_base = "" if self.control_mode.lower() == "autoware" else "/carla/{}".format(self.agent_role_name)
        pygame.init()
        self.screen_width = 600
        self.screen_height = 500
        self.screen = pygame.display.set_mode((self.screen_width, self.screen_height), pygame.RESIZABLE)
        pygame.display.set_caption(f"Ego-Vehicle {self.agent_role_name}'s terminal")
        self.clock = pygame.time.Clock()
        self.autopilot = False
        self.control = None
        self.throttle = False
        self.brake = False
        self.steer = None
        self.steer_cache = 0
        self.max_throttle = 0.15 if os.environ["RUNNING_MODE"] == "record" else 0.5
        self.reverse_max_throttle = 0.25 if os.environ["RUNNING_MODE"] == "record" else 0.5
        
        self.ego_vehicle = BridgeHelpers.get_agent_actor(CarlaDataProvider.get_world(), self.agent_role_name)
    
    def visualization(self, size=None, center_camera=None, back_camera=None, bev_camera=None):
        if center_camera is not None and back_camera is not None and bev_camera is not None:
            vis_camera = cv2.vconcat([center_camera, back_camera])
            vis_camera = cv2.hconcat([vis_camera, bev_camera])
        else:
            vis_camera = None
            print("No image recieved!")
        
        if vis_camera is not None:
            if size is not None:
                vis_camera = cv2.resize(vis_camera, size)
            image_surface = pygame.surfarray.make_surface(
                cv2.cvtColor(vis_camera, cv2.COLOR_BGR2RGB)
            )
            self.screen.blit(image_surface, (0, 0))
    
    def stop(self):
        self.ego_vehicle = None
        pygame.quit()

    def run(self, center_camera=None, back_camera=None, bev_camera=None):
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
            if event.type == pygame.VIDEORESIZE:
                size = event.size
                self.screen_width, self.screen_height = size
            if self.control is not None:
                if event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_c:
                        print("Pygame control mode give to autopilot in carla! Please press 'p' to go back to window control (WASD).")
                        self.autopilot = True
                        self.control.throttle = 0.0
                        self.control.brake = 0.0
                        self.ego_vehicle.apply_control(self.control)
                        
                        # Setup the tranffic manager.
                        if self.client is not None:
                            self.traffic_manager = self.client.get_trafficmanager(self.tm_port)
                            self.traffic_manager.set_global_distance_to_leading_vehicle(100.0)
                            self.traffic_manager.set_random_device_seed(0)
                            self.traffic_manager.set_synchronous_mode(True)
                            settings = self.client.get_world().get_settings()
                            settings.synchronous_mode = True
                            settings.fixed_delta_seconds = 0.05
                            self.client.get_world().apply_settings(settings)
                            print("Traffic Manager generated!")
                        else:
                            raise ValueError("Carla client failed!")
                        
                        # Setup autopilot.
                        if self.traffic_manager is not None:
                            self.ego_vehicle.set_autopilot(True, self.traffic_manager.get_port())

                    if event.key == pygame.K_p:
                        print("Pygame control mode give to window (WASD)! Please press 'c' to go back to autopilot control in carla (WASD).")
                        self.autopilot = False
                        self.control.throttle = 0.0
                        self.control.brake = 0.0
                        self.ego_vehicle.apply_control(self.control)
                        settings = self.client.get_world().get_settings()
                        settings.synchronous_mode = False
                        settings.fixed_delta_seconds = 1.0 / self.frame_rate
                        self.client.get_world().apply_settings(settings)
                        if self.traffic_manager is not None:
                            self.ego_vehicle.set_autopilot(False, self.traffic_manager.get_port())
                        self.traffic_manager = None

                    if event.key == pygame.K_UP or event.key == pygame.K_w:
                        self.throttle = True
                    if event.key == pygame.K_DOWN or event.key == pygame.K_s:
                        self.brake = True
                    if event.key == pygame.K_RIGHT or event.key == pygame.K_a:
                        self.steer = -1
                    if event.key == pygame.K_LEFT or event.key == pygame.K_d:
                        self.steer = 1
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
                
                if self.throttle:
                    self.control.throttle = min(self.control.throttle + 0.1, self.max_throttle)
                    self.control.gear = 1
                    self.control.brake = False
                elif not self.brake:
                    self.control.throttle = 0.0
                
                if self.brake:
                    if self.ego_vehicle.get_velocity().length() < 0.01 and not self.control.reverse:
                        self.control.brake = 0.0
                        self.control.gear = 1
                        self.control.reverse = True
                        self.control.throttle = min(self.control.throttle + 0.1, self.max_throttle)
                    elif self.control.reverse:
                        self.control.throttle = min(self.control.throttle + 0.1, self.reverse_max_throttle)
                    else:
                        self.control.throttle = 0.0
                        self.control.brake = min(self.control.brake + 0.3, 1)
                else:
                    self.control.brake = 0.0
                
                if self.steer is not None:
                    if self.steer == 1:
                        self.steer_cache += 0.03
                    if self.steer == -1:
                        self.steer_cache -= 0.03
                    min(0.7, max(-0.7, self.steer_cache))
                    self.control.steer = round(self.steer_cache, 1)
                else:
                    if self.steer_cache > 0.0:
                        self.steer_cache *= 0.2
                    if self.steer_cache < 0.0:
                        self.steer_cache *= 0.2
                    if self.steer_cache < 0.01 and self.steer_cache > -0.01:
                        self.steer_cache = 0.0
                    self.control.steer = round(self.steer_cache, 1)
                
                if not self.autopilot:
                    self.ego_vehicle.apply_control(self.control)
        self.visualization((self.screen_height, self.screen_width), center_camera, back_camera, bev_camera)

        pygame.display.flip()


if __name__ == "__main__":
    
    rclpy.init(args=None)

    if os.environ["CONTROL_MODE"] != "pygame" and os.environ["RUNNING_MODE"] == "record":
        raise ValueError("Only in the pygame control mode, the running mode can be set to record!")
    else:
        ego_vehicle_launcher = EgoVehicleLauncher()
        client = ego_vehicle_launcher.load_vehicle()
        
        try:
            ego_vehicle_handler = EgoVehicleHandler(client)
        except Exception as e:
            ego_vehicle_launcher.cleanup()
            traceback.print_exc()

        old_handler = signal.signal(signal.SIGINT, ego_vehicle_handler.stop_loop)

        try:
            ego_vehicle_handler.run_agent()
        except Exception as e:
            traceback.print_exc()
        
        print(f"The vehicle named {os.environ['AGENT_ROLE_NAME']} finished simulation!")

        signal.signal(signal.SIGINT, old_handler)

        ego_vehicle_handler.cleanup()
        ego_vehicle_launcher.cleanup()
