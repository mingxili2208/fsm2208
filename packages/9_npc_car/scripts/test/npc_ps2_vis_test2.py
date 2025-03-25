#!/usr/bin/env python

import os
import sys
import random
import time
import logging
import pygame
import carla
import serial  # Added for serial communication
from pygame.locals import K_ESCAPE
import numpy as np
import math
import psutil

def print_memory_usage():
    process = psutil.Process(os.getpid())
    print(f"Memory usage: {process.memory_info().rss / 1024 ** 2:.2f} MB")

class EgoVehiclePS2Terminal:
    def __init__(self, client=None, host="127.0.0.1", port=2000, serial_port="/dev/ttyUSB1", baud_rate=9600):
        self.client = client or carla.Client(host, port)
        self.client.set_timeout(10.0)
        self.world = self.client.get_world()

        spawn_points = self.world.get_map().get_spawn_points()
        if len(spawn_points) < 2:
            print("Not enough spawn points to perform navigation.")
            return
        for point in spawn_points:
            point.location.z=0.17

        self.spawn_points=spawn_points

        # Initialize Pygame window
        pygame.init()
        # Camera image size
        self.cam_width = 400
        self.cam_height = 300
        # Pygame window set as front & back up; bev down
        self.screen_width = self.cam_width * 2
        self.screen_height = self.cam_height * 2
        self.screen = pygame.display.set_mode((self.screen_width, self.screen_height), pygame.RESIZABLE)
        pygame.display.set_caption("CARLA Remote_NPC_Vehicle Terminal (PS2 Controller)")
        self.clock = pygame.time.Clock()

        # Control parameters
        self.throttle = 0.0
        self.steer = 0.0
        self.steer_increment = 0.05
        self.vehicle = None
        self.cameras = {}
        self.camera_images = {}

        # Serial port for PS2 controller
        self.serial_port = serial_port
        self.baud_rate = baud_rate
        self.serial_conn = None
        
        # Specify vehicle type and color
        self.vehicle_model = os.environ["NPC_MODEL_TYPE"]
        self.vehicle_color = "255,0,0"  # RGB color, e.g. red
        self.vehicle_role_name = os.environ["NPC_ROLE_NAME"]
        
        # Generate new vehicle
        self.spawn_vehicle()
        self.resize_timer = None  # For delayed window resizing
        self.resizing = False
        self.bev_transfrom = None
        self.REDRAW_EVENT = pygame.USEREVENT + 1
        # 添加起步助推和惯性控制相关变量
        self.starting_forward = False
        self.starting_reverse = False
        self.start_time_forward = 0
        self.start_time_reverse = 0
        self.last_direction_forward = True  # True表示最后一次是前进，False表示最后一次是后退
        self.direction_change_time = 0  # 记录方向变化的时间
        self.target_speed = 0.0  # 目标速度
        
        self.max_speed = 30.0  # 默认最大速度30km/h
        self.last_left_stick_pressed = False
        self.last_right_stick_pressed = False
        self.speed_change_time = 0  # 上次调整速度的时间
        self.speed_change_cooldown = 0.5  # 速度调整的冷却时间(秒)
        # Setup cameras
        self.setup_cameras()
        
        # Setup serial connection
        self.setup_serial()

    def setup_serial(self):
        """Setup serial connection"""
        try:
            self.serial_conn = serial.Serial(self.serial_port, self.baud_rate, timeout=1)
            logging.info(f"Connected to serial port {self.serial_port} at {self.baud_rate} baud.")
        except Exception as e:
            logging.error(f"Failed to connect to serial port {self.serial_port}: {e}")
            self.serial_conn = None

    def parse_serial_data(self, data):
        """Parse received PS2 data packet from serial port"""
        try:
            key_value_pairs = data.strip().split(",")
            parsed_data = {}
            for pair in key_value_pairs:
                key, value = pair.split(":")
                parsed_data[key] = int(value)  # Convert value to integer
            return parsed_data
        except Exception as e:
            logging.error(f"Error parsing serial data: {e}")
            return None
        
    def read_serial_input(self):
        """Read data from serial port and parse it"""
        if self.serial_conn and self.serial_conn.in_waiting > 0:
            try:
                line = self.serial_conn.readline().decode('utf-8').strip()
                logging.info(f"Raw serial data: {line}")
                return self.parse_serial_data(line)
            except Exception as e:
                logging.error(f"Error reading from serial: {e}")
        return None

    def get_nearest_spawn_point(self):
        """
        Get the nearest spawn point for the vehicle.
        
        Returns:
            carla.Transform: The nearest spawn point.
        """
        # Get current vehicle position
        vehicle_location = self.vehicle.get_transform().location

        # Initialize minimum distance and nearest spawn point
        nearest_spawn_point = None
        min_distance = float('inf')

        # Iterate through all spawn points to calculate distance
        for spawn_point in self.spawn_points:
            # Get spawn point location
            spawn_location = spawn_point.location

            # Calculate Euclidean distance
            distance = math.sqrt(
                (vehicle_location.x - spawn_location.x) ** 2 +
                (vehicle_location.y - spawn_location.y) ** 2 +
                (vehicle_location.z - spawn_location.z) ** 2
            )

            # Update if a closer point is found
            if distance < min_distance:
                min_distance = distance
                nearest_spawn_point = spawn_point

        return nearest_spawn_point
    
    def relocate_vehicle_to_nearest_spawn_point(self):
        """
        Relocate the vehicle to the nearest spawn point.
        """
        # Get the nearest spawn point
        nearest_spawn_point = self.get_nearest_spawn_point()

        if nearest_spawn_point:
            # Output debug info
            print(f"Relocating vehicle to nearest spawn point: {nearest_spawn_point.location}")
            
            # Teleport vehicle to nearest spawn point
            self.vehicle.set_transform(nearest_spawn_point)
            control = carla.VehicleControl()
            control.throttle = 0.0
            control.brake = 1.0
            self.vehicle.apply_control(control)
        else:
            print("No nearest spawn point found!")
            
    def spawn_vehicle(self):
        """Generate vehicle and initialize control"""
        blueprint_library = self.world.get_blueprint_library()
        vehicle_blueprints = blueprint_library.filter("vehicle.*")

        if not vehicle_blueprints:
            logging.error("No vehicle blueprints found.")
            sys.exit(1)

        # Try to find the specified vehicle blueprint
        if self.vehicle_model not in [bp.id for bp in vehicle_blueprints]:
            logging.error(f"Vehicle model '{self.vehicle_model}' not found in blueprint library.")
            sys.exit(1)

        vehicle_bp = blueprint_library.find(self.vehicle_model)

        # Set specified color
        if self.vehicle_color:
            # Assume color format is "R,G,B", e.g., "255,0,0"
            color_rgb = self.vehicle_color.split(',')
            if len(color_rgb) != 3:
                logging.error("Vehicle color must be in 'R,G,B' format, e.g., '255,0,0'.")
                sys.exit(1)
            color = ','.join(color_rgb)
            if vehicle_bp.has_attribute("color"):
                vehicle_bp.set_attribute("color", color)
            else:
                logging.warning(f"The vehicle model '{self.vehicle_model}' does not have a color attribute.")

        vehicle_bp.set_attribute("role_name", self.vehicle_role_name)

        spawn_point = self.spawn_points[3]
        self.vehicle = self.world.try_spawn_actor(vehicle_bp, spawn_point)

        if not self.vehicle:
            logging.error("Failed to spawn the vehicle.")
            sys.exit(1)

        logging.info(f"Vehicle '{self.vehicle_model}' spawned successfully and set as {self.vehicle_role_name}.")
        self.vehicle.set_autopilot(False)

    def setup_cameras(self):
        """Setup front, back and bird's eye view cameras and attach them to the vehicle"""
        blueprint_library = self.world.get_blueprint_library()

        # Front camera
        front_camera_bp = blueprint_library.find('sensor.camera.rgb')
        front_camera_bp.set_attribute('image_size_x', f"{self.cam_width}")
        front_camera_bp.set_attribute('image_size_y', f"{self.cam_height}")
        front_camera_bp.set_attribute('fov', '90')  # Adjust field of view

        # Define camera position and orientation
        front_camera_transform = carla.Transform(carla.Location(x=1.5, z=2.4), carla.Rotation(pitch=0))
        self.cameras['front'] = self.world.spawn_actor(
            front_camera_bp,
            front_camera_transform,
            attach_to=self.vehicle)
        self.cameras['front'].listen(lambda image: self.process_image(image, 'front'))
        logging.info(f"Vehicle window_cam_width'{self.cam_width}' was setted.")
        
        # Back camera
        back_camera_bp = blueprint_library.find('sensor.camera.rgb')
        back_camera_bp.set_attribute('image_size_x', f"{self.cam_width}")
        back_camera_bp.set_attribute('image_size_y', f"{self.cam_height}")
        back_camera_bp.set_attribute('fov', '90')  # Adjust field of view

        back_camera_transform = carla.Transform(carla.Location(x=-1.5, z=2.4), carla.Rotation(yaw=180, pitch=0))
        self.cameras['back'] = self.world.spawn_actor(
            back_camera_bp,
            back_camera_transform,
            attach_to=self.vehicle)
        self.cameras['back'].listen(lambda image: self.process_image(image, 'back'))

        # Bird's eye view camera
        bev_camera_bp = blueprint_library.find('sensor.camera.rgb')
        bev_camera_bp.set_attribute('image_size_x', f"{self.cam_width * 2}")
        bev_camera_bp.set_attribute('image_size_y', f"{self.cam_height}")
        bev_camera_bp.set_attribute('fov', '90')  # Adjust field of view
        self.bev_transfrom = carla.Transform(carla.Location(x=-20, z=11.5), carla.Rotation(pitch=-30, yaw=0))
        self.cameras['bev'] = self.world.spawn_actor(
            bev_camera_bp,
            self.bev_transfrom,
            attach_to=self.vehicle)
        self.cameras['bev'].listen(lambda image: self.process_image(image, 'bev'))
    
    def update_bev_camera(self):
        """
        Update the position and orientation of the bird's eye view camera.
        """
        if 'bev' in self.cameras:
            # Stop and destroy the old BEV camera
            self.cameras['bev'].stop()
            self.cameras['bev'].destroy()

            # Create a new BEV camera
            blueprint_library = self.world.get_blueprint_library()
            bev_camera_bp = blueprint_library.find('sensor.camera.rgb')
            bev_camera_bp.set_attribute('image_size_x', f"{self.cam_width * 2}")
            bev_camera_bp.set_attribute('image_size_y', f"{self.cam_height}")
            bev_camera_bp.set_attribute('fov', '90')  # Adjust field of view
            
            # Generate new camera and attach to vehicle
            self.cameras['bev'] = self.world.spawn_actor(
                bev_camera_bp,
                self.bev_transfrom,
                attach_to=self.vehicle
            )
            self.cameras['bev'].listen(lambda image: self.process_image(image, 'bev'))
            logging.info(f"Updated BEV camera to location: {self.bev_transfrom.location}, rotation: {self.bev_transfrom.rotation}")
            
    def process_image(self, image, cam_key):
        """Process camera image data and convert to Pygame displayable format"""
        array = np.frombuffer(image.raw_data, dtype=np.uint8)
        array = np.reshape(array, (image.height, image.width, 4))  # RGBA
        array = array[:, :, :3]  # RGB
        array = array[:, :, ::-1]  # BGR to RGB

        # Store image
        self.camera_images[cam_key] = array

    def visualization(self):
        """Display the three camera images and control information in the Pygame window"""
        # Define display area for each camera image
        cam_positions = {
            'front': (0, 0),
            'back': (self.cam_width, 0),
            'bev': (0, self.cam_height),
        }

        for cam_key, image in self.camera_images.items():
            if cam_key in cam_positions:
                # Convert image to Pygame display format
                surface = pygame.surfarray.make_surface(image.swapaxes(0, 1))
                self.screen.blit(surface, cam_positions[cam_key])

        # Display control hint text in top left corner
        font = pygame.font.Font(None, 24)
        text_surface = font.render("PS2 Controller: L stick for steering, R stick for throttle, R2 for brake", True, (255, 255, 255))
        self.screen.blit(text_surface, (10, 10))  # Display in top left corner

    def stop(self):
        """Stop the vehicle, sensors and exit Pygame"""
        if self.vehicle:
            logging.info("Destroying vehicle.")
            self.vehicle.destroy()
        for camera in self.cameras.values():
            if camera:
                camera.stop()
                camera.destroy()
        if self.serial_conn:
            self.serial_conn.close()
        pygame.quit()

    def run(self):
        """Main loop, handling PS2 controller input and controlling the vehicle"""
        try:
            while True:
                for event in pygame.event.get():
                    if event.type == pygame.QUIT:
                        raise KeyboardInterrupt
                    if event.type == pygame.VIDEORESIZE:
                        self.resizing=True
                        logging.info(f"Resizing window to {event.size[0]}x{event.size[1]}")
                        self.screen_width, self.screen_height = event.size
                        
                        # Update camera image size
                        self.cam_width = self.screen_width // 2
                        self.cam_height = self.screen_height // 2
                        if self.resize_timer:
                            pygame.time.set_timer(self.REDRAW_EVENT, 0)  # Cancel previous timer
                        pygame.time.set_timer(self.REDRAW_EVENT, 500)  # Trigger redraw event after 500ms
                    elif event.type == self.REDRAW_EVENT:
                        logging.info("Window adjustment completed, updating cameras")
                        if self.cameras and self.resizing is True:
                            pygame.time.set_timer(self.REDRAW_EVENT, 0)  # Cancel timer
                            self.resize_timer = None
                            logging.info("destroy camera")
                            for cam_key, cam in self.cameras.items():
                                cam.stop()
                                cam.destroy()
                                logging.info(f"destroy camera {cam_key}")
                            self.cameras.clear()  # Clear camera dictionary
                            self.camera_images = {}
                            self.setup_cameras()
                            self.resizing=False
                    if event.type == pygame.KEYDOWN:
                        if event.key == K_ESCAPE:
                            raise KeyboardInterrupt
                        
                        # Capture number keys 1, 2, 3 to switch BEV angle
                        if event.key == pygame.K_r:
                            self.relocate_vehicle_to_nearest_spawn_point()

                        if event.key == pygame.K_1:
                            self.bev_transfrom=carla.Transform(carla.Location(x=-20, z=11.5),carla.Rotation(pitch=-30, yaw=0))
                            self.update_bev_camera()
                        elif event.key == pygame.K_2:
                            self.bev_transfrom=carla.Transform(carla.Location(x=-15, z=22.5),carla.Rotation(pitch=-50, yaw=0))
                            self.update_bev_camera()
                        elif event.key == pygame.K_3:
                            self.bev_transfrom=carla.Transform(carla.Location(x=-5 , z=40.0),carla.Rotation(pitch=-80, yaw=0))
                            self.update_bev_camera()

                # Initialize vehicle control
                control = carla.VehicleControl()
                control.throttle = 0.0
                control.brake = 0.0
                control.reverse = False
                current_time = time.time()
                velocity = self.vehicle.get_velocity()
                speed = math.sqrt(velocity.x**2 + velocity.y**2 + velocity.z**2)  # 当前速度（m/s）
                speed_kmh = speed * 3.6  # 转换为km/h

                # Read PS2 controller data from serial port
                ps2_data = self.read_serial_input()
                if ps2_data:
                    # L2 controls throttle
                    control = carla.VehicleControl()
                    control.throttle = 0.0
                    control.brake = 0.0
                    control.reverse = False
                    
                    # 检测UP和DOWN按钮状态，用于调整最大速度
                    # UP和DOWN按钮，值为1表示按下，0表示未按下
                    up_button_pressed = ps2_data.get("UP", 0) == 1
                    down_button_pressed = ps2_data.get("DOWN", 0) == 1

                    # 检测DOWN按下（降低速度）
                    if down_button_pressed and not self.last_down_button_pressed and current_time - self.speed_change_time > self.speed_change_cooldown:
                        self.max_speed = max(5.0, self.max_speed - 5.0)  # 最低不低于5km/h
                        self.speed_change_time = current_time
                        logging.info(f"Max speed decreased to {self.max_speed:.1f} km/h")

                    # 检测UP按钮按下（提高速度）
                    if up_button_pressed and not self.last_up_button_pressed and current_time - self.speed_change_time > self.speed_change_cooldown:
                        self.max_speed = min(80.0, self.max_speed + 5.0)  # 最高不超过80km/h
                        self.speed_change_time = current_time
                        logging.info(f"Max speed increased to {self.max_speed:.1f} km/h")

                    # 更新按钮状态
                    self.last_down_button_pressed = down_button_pressed
                    self.last_up_button_pressed = up_button_pressed
                    

                    # Left stick controls steering
                    lx = ps2_data.get("LX", 128)
                    steer_factor = 1.0 if speed_kmh < 30 else 0.6  # 低速时更灵敏，高速时降低灵敏度
                    if lx < 110:
                        self.steer = max(self.steer - self.steer_increment, -1.0) * steer_factor
                    elif lx > 150:
                        self.steer = min(self.steer + self.steer_increment, 1.0) * steer_factor
                    # if lx < 110:  # Turn left
                    #     self.steer = max(self.steer - self.steer_increment, -1.0) * 0.8
                    # elif lx > 150:  # Turn right
                    #     self.steer = min(self.steer + self.steer_increment, 1.0) * 0.8
                    else:  # Stop turning
                        if self.steer > 0:
                            self.steer = max(self.steer - self.steer_increment, 0)
                        elif self.steer < 0:
                            self.steer = min(self.steer + self.steer_increment, 0)

                    control.steer = self.steer

                    # Right stick controls forward and reverse
                    ry = ps2_data.get("RY", 127)
                    current_direction_forward = ry <= 127
                    if current_direction_forward != self.last_direction_forward:
                        self.direction_change_time = current_time
                        self.last_direction_forward = current_direction_forward
                        # 切换方向时立即刹车
                        control.throttle = 0.0
                        control.brake = 1.0
                        self.vehicle.apply_control(control)
                        logging.info(f"Direction changed to {'forward' if current_direction_forward else 'reverse'}")
                        
                        # 等待一小段时间让车辆停下来
                        if speed > 1.0:  # 如果车速大于1m/s
                            # 这里仅应用刹车指令，不要延时
                            self.vehicle.apply_control(control)
                            continue  # 跳过此帧的其余处理
                    
                    # 方向变化后的缓冲期（防止车辆还在运动时就换向）
                    direction_change_buffer = 0.5  # 0.5秒缓冲
                    if current_time - self.direction_change_time < direction_change_buffer and speed > 0.5:
                        control.throttle = 0.0
                        control.brake = 1.0
                        self.vehicle.apply_control(control)
                        continue  # 跳过此帧的其余处理
                    
                    # 前进控制逻辑 (ry < 127)
                   # 前进控制逻辑部分修改为
                    if ry < 127:
                        # 计算基础油门值 (0.0-1.0)
                        base_throttle = ((127 - ry) / 127.0) * 0.7  # 最大油门值为0.7
                        
                        # 判断是否处于起步阶段
                        if speed < 2.0 and not self.starting_forward:
                            self.starting_forward = True
                            self.start_time_forward = current_time
                            logging.info("Starting forward acceleration boost")
                        elif self.starting_forward and speed > 5.0:
                            self.starting_forward = False
                            logging.info("Forward acceleration boost ended")
                        
                        # 更严格的速度控制
                        if speed_kmh >= self.max_speed:
                            # 已经达到或超过最大速度，完全切断油门并轻微刹车
                            control.throttle = 0.0
                            control.brake = 0.3  # 使用更强的刹车力度来确保速度不会超过限制
                            logging.info(f"Speed limit reached: {speed_kmh:.1f} km/h > {self.max_speed:.1f} km/h, applying brake")
                        elif speed_kmh > self.max_speed - 3.0:
                            # 接近最大速度，逐渐减小油门
                            reduction_factor = (self.max_speed - speed_kmh) / 3.0  # 当速度接近最大值时，油门逐渐减小
                            control.throttle = base_throttle * reduction_factor * 0.5
                            logging.info(f"Approaching speed limit: {speed_kmh:.1f} km/h, reducing throttle to {control.throttle:.2f}")
                        else:
                            # 计算目标速度 (km/h)，保持不超过设定的最大速度
                            percentage = ((127 - ry) / 127.0)  # 油门百分比
                            self.target_speed = max(5.0, min(percentage * self.max_speed, self.max_speed))  # 目标速度不超过最大速度
                            
                            # 应用起步助推
                            if self.starting_forward:
                                # 起步阶段提供额外推力，但确保不会导致速度超过限制
                                boost_duration = 2.0  # 助推持续2秒
                                time_factor = max(0, 1.0 - (current_time - self.start_time_forward) / boost_duration)
                                
                                # 根据当前速度动态调整助推量，接近速度限制时减小助推
                                boost_limit_factor = max(0, (self.max_speed - speed_kmh) / self.max_speed)
                                boost_amount = 0.3 * time_factor * boost_limit_factor
                                
                                # 应用助推后的油门值
                                control.throttle = min(0.8, base_throttle + boost_amount)  # 限制最大油门为0.8
                                logging.info(f"Forward boost: {boost_amount:.2f}, Total throttle: {control.throttle:.2f}")
                            else:
                                # 正常驾驶模式下的速度控制
                                if speed_kmh < self.target_speed - 5.0:
                                    # 低于目标速度，应用计算的油门
                                    # 根据距离速度限制的接近程度动态调整油门
                                    throttle_factor = min(1.0, (self.max_speed - speed_kmh) / self.max_speed)
                                    control.throttle = base_throttle * throttle_factor
                                elif speed_kmh > self.target_speed + 2.0:
                                    # 高于目标速度，轻微刹车
                                    control.throttle = 0.0
                                    control.brake = 0.2
                                else:
                                    # 接近目标速度，应用较小的油门以维持速度
                                    control.throttle = base_throttle * 0.3
                        
                        control.reverse = False
                        
                    # 后退控制逻辑 (ry > 127)
                    elif ry > 127:
                        # 计算基础后退油门值 (0.0-1.0)
                        base_reverse_throttle = ((ry - 127) / 127.0) * 0.5  # 最大后退油门值为0.5
                        
                        # 计算后退目标速度 (km/h)，后退最大速度为设定最大速度的一半
                        reverse_max_speed = min(15.0, self.max_speed * 0.5)  # 后退速度为前进最大速度的一半，且不超过15km/h
                        
                        # 更平滑的速度控制逻辑，避免频繁切换刹车状态
                        if speed_kmh >= reverse_max_speed + 1.0:
                            # 已经超过最大后退速度，切断油门并轻微刹车
                            control.throttle = 0.0
                            control.brake = 0.3
                        else:
                            # 判断是否处于后退起步阶段
                            if speed < 1.0 and not self.starting_reverse:
                                self.starting_reverse = True
                                self.start_time_reverse = current_time
                                logging.info("Starting reverse acceleration boost")
                            elif self.starting_reverse and speed > 3.0:
                                self.starting_reverse = False
                                logging.info("Reverse acceleration boost ended")
                            
                            # 计算后退目标速度，使用反比例关系使目标速度更平滑
                            percentage = ((ry - 127) / 127.0)
                            self.target_speed = percentage * reverse_max_speed
                            
                            # 使用更大的缓冲区来避免频繁切换状态
                            if speed_kmh < self.target_speed - 2.0:
                                # 低于目标速度，应用计算的油门
                                control.throttle = base_reverse_throttle
                                control.brake = 0.0
                            elif speed_kmh > self.target_speed + 3.0:
                                # 明显高于目标速度，轻微刹车
                                control.throttle = 0.0
                                control.brake = 0.2
                            else:
                                # 接近目标速度，使用更平滑的油门控制以维持速度
                                # 线性插值计算合适的油门值
                                speed_diff = self.target_speed - speed_kmh
                                throttle_factor = max(0.2, min(1.0, (speed_diff + 2.0) / 4.0))
                                control.throttle = base_reverse_throttle * throttle_factor
                                control.brake = 0.0
                        
                        control.reverse = True
                    
                    if ps2_data.get("L2", 0) is 1:
                        control.throttle = min(1.0, control.throttle + 0.1)
                    # R2 controls brake
                    if ps2_data.get("R2", 0) is 1:
                        control.brake = 1.0
                    # 在屏幕上显示当前速度和控制状态
                    if pygame.time.get_ticks() % 500 == 0:  # 每500ms更新一次
                        logging.info(f"Speed: {speed_kmh:.1f} km/h, Target: {self.target_speed:.1f} km/h, Max: {self.max_speed:.1f} km/h, " +
                                    f"Throttle: {control.throttle:.2f}, Brake: {control.brake:.2f}, " + 
                                    f"Direction: {'Forward' if not control.reverse else 'Reverse'}")

                self.vehicle.apply_control(control)
                self.visualization()
                
                # 在屏幕上显示最大速度信息
                font = pygame.font.Font(None, 30)
                max_speed_text = font.render(f"Max Speed: {self.max_speed:.1f} km/h", True, (255, 255, 0))
                current_speed_text = font.render(f"Current: {speed_kmh:.1f} km/h", True, (255, 255, 0))
                self.screen.blit(max_speed_text, (10, 40))
                self.screen.blit(current_speed_text, (10, 70))
                
                pygame.display.flip()
                if pygame.time.get_ticks() % 500 == 0:
                    print_memory_usage()
                self.clock.tick(60)

        except KeyboardInterrupt:
            logging.info("Exiting...")
        finally:
            self.stop()
            pygame.quit()

def main():
    # Configure logging
    logging.basicConfig(format="%(levelname)s: %(message)s", level=logging.INFO)

    # Set CARLA Python API path
    egg_file = '/home/cityu-fsm-lab-carla/Desktop/Workspace/Carla/carla-0.9.15/PythonAPI/carla/dist/carla-0.9.15-py3.10-linux-x86_64.egg'
    if not os.path.exists(egg_file):
        logging.error(f"CARLA egg file not found at {egg_file}. Please check the path.")
        sys.exit(1)

    sys.path.append(egg_file)

    # Run EgoVehiclePS2Terminal
    terminal = EgoVehiclePS2Terminal(
        host="127.0.0.1",  # Change if CARLA server is on another IP
        port=2000,         # Change if CARLA server uses another port
        serial_port="/dev/ttyUSB1",  # Serial port for PS2 controller
        baud_rate=9600     # Baud rate for serial communication
    )
    terminal.run()


if __name__ == "__main__":
    main()