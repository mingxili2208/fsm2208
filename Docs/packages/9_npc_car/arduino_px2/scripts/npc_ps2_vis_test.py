#!/usr/bin/env python

import os
import sys
import random
import logging
import pygame
import carla
import serial  # 串口通信模块
import numpy as np
from pygame.locals import K_ESCAPE

class EgoVehicleTerminal:
    def __init__(self, client=None, host="127.0.0.1", port=2000, serial_port="/dev/ttyUSB0", baud_rate=9600):
        # CARLA 设置
        self.client = client or carla.Client(host, port)
        self.client.set_timeout(10.0)
        self.world = self.client.get_world()

        # 串口设置
        self.serial_port = serial_port
        self.baud_rate = baud_rate
        self.serial_conn = None

        # 初始化 Pygame 窗口
        pygame.init()
        self.cam_width = 400
        self.cam_height = 300
        self.screen_width = self.cam_width * 2
        self.screen_height = self.cam_height * 2
        self.screen = pygame.display.set_mode((self.screen_width, self.screen_height))
        pygame.display.set_caption("CARLA Ego Vehicle Terminal - Three Cameras")
        self.clock = pygame.time.Clock()

        # 控制相关参数
        self.steer = 0.0
        self.steer_increment = 0.05
        self.vehicle = None
        self.cameras = {}
        self.camera_images = {}

        # 指定车辆类型和颜色
        self.vehicle_model = "vehicle.tesla.model3"
        self.vehicle_color = "255,0,0"

        # 生成车辆并设置摄像头
        self.spawn_vehicle()
        self.setup_cameras()

        # 初始化串口
        self.setup_serial()

    def setup_serial(self):
        """设置串口连接"""
        try:
            self.serial_conn = serial.Serial(self.serial_port, self.baud_rate, timeout=1)
            logging.info(f"Connected to serial port {self.serial_port} at {self.baud_rate} baud.")
        except Exception as e:
            logging.error(f"Failed to connect to serial port {self.serial_port}: {e}")
            self.serial_conn = None

    def parse_serial_data(self, data):
        """解析串口接收到的 PS2 数据包"""
        try:
            key_value_pairs = data.strip().split(",")
            parsed_data = {}
            for pair in key_value_pairs:
                key, value = pair.split(":")
                parsed_data[key] = int(value)  # 转换值为整数
            return parsed_data
        except Exception as e:
            logging.error(f"Error parsing serial data: {e}")
            return None

    def read_serial_input(self):
        """从串口读取数据并解析"""
        if self.serial_conn and self.serial_conn.in_waiting > 0:
            try:
                line = self.serial_conn.readline().decode('utf-8').strip()
                logging.info(f"Raw serial data: {line}")
                return self.parse_serial_data(line)
            except Exception as e:
                logging.error(f"Error reading from serial: {e}")
        return None

    def spawn_vehicle(self):
        """生成车辆并初始化控制"""
        blueprint_library = self.world.get_blueprint_library()
        vehicle_blueprints = blueprint_library.filter("vehicle.*")

        if not vehicle_blueprints:
            logging.error("No vehicle blueprints found.")
            sys.exit(1)

        if self.vehicle_model not in [bp.id for bp in vehicle_blueprints]:
            logging.error(f"Vehicle model '{self.vehicle_model}' not found in blueprint library.")
            sys.exit(1)

        vehicle_bp = blueprint_library.find(self.vehicle_model)

        # 设置指定颜色
        color_rgb = self.vehicle_color.split(',')
        if len(color_rgb) == 3:
            color = ','.join(color_rgb)
            if vehicle_bp.has_attribute("color"):
                vehicle_bp.set_attribute("color", color)

        vehicle_bp.set_attribute("role_name", "ego_vehicle")

        spawn_points = self.world.get_map().get_spawn_points()
        if not spawn_points:
            logging.error("No spawn points available.")
            sys.exit(1)

        spawn_point = random.choice(spawn_points)
        self.vehicle = self.world.try_spawn_actor(vehicle_bp, spawn_point)

        if not self.vehicle:
            logging.error("Failed to spawn the vehicle.")
            sys.exit(1)

        logging.info(f"Vehicle '{self.vehicle_model}' spawned successfully and set as ego_vehicle.")
        self.vehicle.set_autopilot(False)

    def setup_cameras(self):
        """设置摄像头"""
        blueprint_library = self.world.get_blueprint_library()

        # 前视摄像头
        front_camera_bp = blueprint_library.find('sensor.camera.rgb')
        front_camera_bp.set_attribute('image_size_x', f"{self.cam_width}")
        front_camera_bp.set_attribute('image_size_y', f"{self.cam_height}")
        front_camera_bp.set_attribute('fov', '90')

        front_camera_transform = carla.Transform(carla.Location(x=1.5, z=2.4))
        self.cameras['front'] = self.world.spawn_actor(
            front_camera_bp,
            front_camera_transform,
            attach_to=self.vehicle)
        self.cameras['front'].listen(lambda image: self.process_image(image, 'front'))

        # 后视摄像头
        back_camera_bp = blueprint_library.find('sensor.camera.rgb')
        back_camera_bp.set_attribute('image_size_x', f"{self.cam_width}")
        back_camera_bp.set_attribute('image_size_y', f"{self.cam_height}")
        back_camera_bp.set_attribute('fov', '90')

        back_camera_transform = carla.Transform(carla.Location(x=-1.5, z=2.4), carla.Rotation(yaw=180))
        self.cameras['back'] = self.world.spawn_actor(
            back_camera_bp,
            back_camera_transform,
            attach_to=self.vehicle)
        self.cameras['back'].listen(lambda image: self.process_image(image, 'back'))

        # 俯视摄像头（鸟瞰视角）
        bev_camera_bp = blueprint_library.find('sensor.camera.rgb')
        bev_camera_bp.set_attribute('image_size_x', f"{self.cam_width}")
        bev_camera_bp.set_attribute('image_size_y', f"{self.cam_height}")
        bev_camera_bp.set_attribute('fov', '90')

        bev_camera_transform = carla.Transform(carla.Location(x=0, z=10), carla.Rotation(pitch=-90))
        self.cameras['bev'] = self.world.spawn_actor(
            bev_camera_bp,
            bev_camera_transform,
            attach_to=self.vehicle)
        self.cameras['bev'].listen(lambda image: self.process_image(image, 'bev'))

    def process_image(self, image, cam_key):
        """处理摄像头图像数据"""
        array = np.frombuffer(image.raw_data, dtype=np.uint8)
        array = np.reshape(array, (image.height, image.width, 4))[:, :, :3]
        array = array[:, :, ::-1]
        self.camera_images[cam_key] = array.copy()

    def visualization(self):
        """显示摄像头图像"""
        cam_positions = {
            'front': (0, 0),
            'back': (self.cam_width, 0),
            'bev': (self.cam_width // 2, self.cam_height),
        }

        for cam_key, image in self.camera_images.items():
            if cam_key in cam_positions:
                surface = pygame.surfarray.make_surface(image.swapaxes(0, 1))
                self.screen.blit(surface, cam_positions[cam_key])

    def stop(self):
        """停止车辆和摄像头"""
        if self.vehicle:
            self.vehicle.destroy()
        for camera in self.cameras.values():
            if camera:
                camera.stop()
                camera.destroy()
        if self.serial_conn:
            self.serial_conn.close()
        pygame.quit()

    def run(self):
        """主循环，处理串口输入并控制车辆"""
        try:
            while True:
                for event in pygame.event.get():
                    if event.type == pygame.QUIT:
                        raise KeyboardInterrupt
                    elif event.type == pygame.KEYDOWN and event.key == K_ESCAPE:
                        raise KeyboardInterrupt

                # 初始化车辆控制
                control = carla.VehicleControl()
                control.throttle = 0.0
                control.brake = 0.0
                control.reverse = False

                # 从串口读取数据
                ps2_data = self.read_serial_input()
                if ps2_data:
                    # L1 控制油门
                    if ps2_data.get("L1", 0):
                        control.throttle = 1.0

                    # R1 控制刹车
                    if ps2_data.get("R1", 0):
                        control.brake = 1.0

                    # 左摇杆控制方向
                    lx = ps2_data.get("LX", 128)
                    if lx < 100:  # 左转
                        self.steer = max(self.steer - self.steer_increment, -1.0)
                    elif lx > 150:  # 右转
                        self.steer = min(self.steer + self.steer_increment, 1.0)
                    else:  # 停止转向
                        self.steer = 0.0

                    control.steer = self.steer

                    # 右摇杆控制前进和后退
                    ry = ps2_data.get("RY", 128)
                    if ry < 100:  # 前进
                        control.throttle = (128 - ry) / 128.0
                        control.reverse = False
                    elif ry > 150:  # 后退
                        control.throttle = (ry - 128) / 128.0
                        control.reverse = True

                self.vehicle.apply_control(control)
                self.visualization()
                pygame.display.flip()
                self.clock.tick(60)
        except KeyboardInterrupt:
            logging.info("Exiting...")
        finally:
            self.stop()


def main():
    logging.basicConfig(format="%(levelname)s: %(message)s", level=logging.INFO)

    terminal = EgoVehicleTerminal(
        host="127.0.0.1",
        port=2000,
        serial_port="/dev/ttyUSB0",
        baud_rate=9600
    )
    terminal.run()


if __name__ == "__main__":
    main()