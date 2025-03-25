#!/usr/bin/env python

import os
import sys
import random
import time
import logging
import pygame
import carla
from pygame.locals import K_UP, K_DOWN, K_LEFT, K_RIGHT, K_ESCAPE, K_SPACE, K_LSHIFT
import numpy as np
import math
import psutil

def print_memory_usage():
    process = psutil.Process(os.getpid())
    print(f"Memory usage: {process.memory_info().rss / 1024 ** 2:.2f} MB")
class EgoVehicleTerminal:
    def __init__(self, client=None, host="127.0.0.1", port=2000):
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

        # 初始化 Pygame 窗口
        pygame.init()
        # 每个摄像头图像尺寸
        self.cam_width = 400
        self.cam_height = 300
        # Pygame window set as front & back up; bev down
        self.screen_width = self.cam_width * 2
        self.screen_height = self.cam_height * 2
        self.screen = pygame.display.set_mode((self.screen_width, self.screen_height), pygame.RESIZABLE)
        pygame.display.set_caption("CARLA Remote_NPC_Vehicle Terminal ")
        self.clock = pygame.time.Clock()

        # 控制相关参数
        self.throttle = 0.0
        self.steer = 0.0
        self.steer_increment = 0.05
        self.vehicle = None
        self.cameras = {}
        self.camera_images = {}

        # 指定车辆类型和颜色
        self.vehicle_model = os.environ["NPC_MODEL_TYPE"]
        self.vehicle_color = "255,0,0"  # RGB格式颜色，例如红色
        self.vehicle_role_name = os.environ["NPC_ROLE_NAME"]
        # 生成新车辆
        self.spawn_vehicle()
        self.resize_timer = None  # 用于延迟窗口调整
        self.resizing = False #
        self.bev_transfrom = None
        self.REDRAW_EVENT = pygame.USEREVENT + 1
        # 设置三个摄像头
        self.setup_cameras()
    def get_nearest_spawn_point(self):
        """
        获取小车最近的 spawn_point。
        
        Args:
            vehicle (carla.Actor): 当前车辆对象。
            spawn_points (list[carla.Transform]): 地图中的所有 spawn_point。

        Returns:
            carla.Transform: 最近的 spawn_point。
        """
        # 获取车辆的当前位置
        vehicle_location = self.vehicle.get_transform().location

        # 初始化最小距离和最近的 spawn_point
        nearest_spawn_point = None
        min_distance = float('inf')  # 设置为正无穷大

        # 遍历所有 spawn_points，计算距离
        for spawn_point in self.spawn_points:
            # 获取 spawn_point 的位置
            spawn_location = spawn_point.location

            # 计算欧几里得距离
            distance = math.sqrt(
                (vehicle_location.x - spawn_location.x) ** 2 +
                (vehicle_location.y - spawn_location.y) ** 2 +
                (vehicle_location.z - spawn_location.z) ** 2
            )

            # 如果找到更近的点，更新
            if distance < min_distance:
                min_distance = distance
                nearest_spawn_point = spawn_point

        return nearest_spawn_point
    def relocate_vehicle_to_nearest_spawn_point(self):
        """
        将小车刷新到最近的 spawn_point。

        Args:
            vehicle (carla.Actor): 当前车辆对象。
            spawn_points (list[carla.Transform]): 地图中的所有 spawn_point。
        """
        # 获取最近的 spawn_point
        nearest_spawn_point = self.get_nearest_spawn_point()

        if nearest_spawn_point:
            # 输出调试信息
            print(f"Relocating vehicle to nearest spawn point: {nearest_spawn_point.location}")
            
            # 将车辆刷新到最近的 spawn_point
            self.vehicle.set_transform(nearest_spawn_point)
            control = carla.VehicleControl()
            control.throttle = 0.0
            control.brake = 1.0
            self.vehicle.apply_control(control)
        else:
            print("No nearest spawn point found!")
    def spawn_vehicle(self):
        """生成车辆并初始化控制"""
        blueprint_library = self.world.get_blueprint_library()
        vehicle_blueprints = blueprint_library.filter("vehicle.*")

        if not vehicle_blueprints:
            logging.error("No vehicle blueprints found.")
            sys.exit(1)

        # 尝试找到指定的车辆蓝图
        if self.vehicle_model not in [bp.id for bp in vehicle_blueprints]:
            logging.error(f"Vehicle model '{self.vehicle_model}' not found in blueprint library.")
            sys.exit(1)

        vehicle_bp = blueprint_library.find(self.vehicle_model)

        # 设置指定颜色
        if self.vehicle_color:
            # 假设颜色格式为"R,G,B", e.g., "255,0,0"
            color_rgb = self.vehicle_color.split(',')
            if len(color_rgb) != 3:
                logging.error("Vehicle color must be in 'R,G,B' format, e.g., '255,0,0'.")
                sys.exit(1)
            color = ','.join(color_rgb)
            if vehicle_bp.has_attribute("color"):
                vehicle_bp.set_attribute("color", color)
            else:
                logging.warning(f"The vehicle model '{self.vehicle_model}' does not have a color attribute.")

        vehicle_bp.set_attribute("role_name", self.vehicle_role_name)  # 设置角色名称为 'ego_vehicle'


        spawn_point = self.spawn_points[35]
        self.vehicle = self.world.try_spawn_actor(vehicle_bp, spawn_point)

        if not self.vehicle:
            logging.error("Failed to spawn the vehicle.")
            sys.exit(1)

        logging.info(f"Vehicle '{self.vehicle_model}' spawned successfully and set as ego_vehicle.")
        self.vehicle.set_autopilot(False)

    def setup_cameras(self):
        """设置前视、后视和俯视摄像头并附加到ego_vehicle"""
        blueprint_library = self.world.get_blueprint_library()

        # 前视摄像头
        front_camera_bp = blueprint_library.find('sensor.camera.rgb')
        front_camera_bp.set_attribute('image_size_x', f"{self.cam_width}")
        front_camera_bp.set_attribute('image_size_y', f"{self.cam_height}")
        front_camera_bp.set_attribute('fov', '90')  # 调整视野角度

        # 定义摄像头的位置和朝向
        front_camera_transform = carla.Transform(carla.Location(x=1.5, z=2.4), carla.Rotation(pitch=0))
        self.cameras['front'] = self.world.spawn_actor(
            front_camera_bp,
            front_camera_transform,
            attach_to=self.vehicle)
        self.cameras['front'].listen(lambda image: self.process_image(image, 'front'))
        logging.info(f"Vehicle window_cam_width'{self.cam_width}' was setted.")
        # 后视摄像头
        back_camera_bp = blueprint_library.find('sensor.camera.rgb')
        back_camera_bp.set_attribute('image_size_x', f"{self.cam_width}")
        back_camera_bp.set_attribute('image_size_y', f"{self.cam_height}")
        back_camera_bp.set_attribute('fov', '90')  # 调整视野角度

        back_camera_transform = carla.Transform(carla.Location(x=-1.5, z=2.4), carla.Rotation(yaw=180, pitch=0))
        self.cameras['back'] = self.world.spawn_actor(
            back_camera_bp,
            back_camera_transform,
            attach_to=self.vehicle)
        self.cameras['back'].listen(lambda image: self.process_image(image, 'back'))

        # 俯视摄像头（鸟瞰视角）
        bev_camera_bp = blueprint_library.find('sensor.camera.rgb')
        bev_camera_bp.set_attribute('image_size_x', f"{self.cam_width * 2}")
        bev_camera_bp.set_attribute('image_size_y', f"{self.cam_height}")
        bev_camera_bp.set_attribute('fov', '90')  # 调整视野角度
        self.bev_transfrom= carla.Transform(carla.Location(x=-20, z=11.5), carla.Rotation(pitch=-30, yaw=0))
        self.cameras['bev'] = self.world.spawn_actor(
            bev_camera_bp,
            self.bev_transfrom,
            attach_to=self.vehicle)
        self.cameras['bev'].listen(lambda image: self.process_image(image, 'bev'))
    
    def update_bev_camera(self):
        """
        更新鸟瞰摄像头的位置和朝向。

        Args:
            location (carla.Location): 新的摄像头位置。
            rotation (carla.Rotation): 新的摄像头朝向。
        """
        if 'bev' in self.cameras:
            # 停止并销毁旧的鸟瞰摄像头
            self.cameras['bev'].stop()
            self.cameras['bev'].destroy()

            # 创建新的鸟瞰摄像头
            blueprint_library = self.world.get_blueprint_library()
            bev_camera_bp = blueprint_library.find('sensor.camera.rgb')
            bev_camera_bp.set_attribute('image_size_x', f"{self.cam_width * 2}")
            bev_camera_bp.set_attribute('image_size_y', f"{self.cam_height}")
            bev_camera_bp.set_attribute('fov', '90')  # 调整视野角度
            
            # 生成新的摄像头并附加到车辆
            self.cameras['bev'] = self.world.spawn_actor(
                bev_camera_bp,
                self.bev_transfrom,
                attach_to=self.vehicle
            )
            self.cameras['bev'].listen(lambda image: self.process_image(image, 'bev'))
            logging.info(f"Updated BEV camera to location: {self.bev_transfrom.location}, rotation: {self.bev_transfrom.rotation}")
    def process_image(self, image, cam_key):
        """处理摄像头图像数据并转换为Pygame可显示格式"""
        array = np.frombuffer(image.raw_data, dtype=np.uint8)
        array = np.reshape(array, (image.height, image.width, 4))  # RGBA
        array = array[:, :, :3]  # RGB
        array = array[:, :, ::-1]  # BGR to RGB

        # 存储图像
        self.camera_images[cam_key] = array

    def visualization(self):
        """在Pygame窗口中显示三个摄像头的图像和控制信息"""
        # 定义每个摄像头图像的显示区域
        cam_positions = {
            'front': (0, 0),
            'back': (self.cam_width, 0),
            'bev': (0, self.cam_height),
        }

        for cam_key, image in self.camera_images.items():
            if cam_key in cam_positions:
                # 将图像转换为Pygame显示格式
                surface = pygame.surfarray.make_surface(image.swapaxes(0, 1))
                self.screen.blit(surface, cam_positions[cam_key])

        # 在窗口左上角显示控制提示文字
        font = pygame.font.Font(None, 24)
        text_surface = font.render("Controls: Arrow Keys to Drive, SPACE to Brake, ESC to Exit", True, (255, 255, 255))
        self.screen.blit(text_surface, (10, 10))  # 显示在左上角

    def stop(self):
        """停止车辆、传感器并退出 Pygame"""
        if self.vehicle:
            logging.info("Destroying vehicle.")
            self.vehicle.destroy()
        for camera in self.cameras.values():
            if camera:
                camera.stop()
                camera.destroy()
        pygame.quit()

    def run(self):
        """主循环，处理键盘事件并控制车辆"""
        try:
            while True:
                for event in pygame.event.get():
                    if event.type == pygame.QUIT:
                        raise KeyboardInterrupt
                    if event.type == pygame.VIDEORESIZE:
                        self.resizing=True
                        logging.info(f"Resizing window to {event.size[0]}x{event.size[1]}")
                        self.screen_width, self.screen_height = event.size
                        #self.screen = pygame.display.set_mode((self.screen_width, self.screen_height), pygame.RESIZABLE)
                        
                        # 更新摄像头的图像尺寸
                        self.cam_width = self.screen_width // 2
                        self.cam_height = self.screen_height // 2
                        if self.resize_timer:
                            pygame.time.set_timer(self.REDRAW_EVENT, 0)  # 取消之前的定时器
                        pygame.time.set_timer(self.REDRAW_EVENT, 500)  # 500毫秒后触发重绘事件
                        #self.resize_timer = time.time()  # 记录调整时间
                    elif event.type == self.REDRAW_EVENT:
                        logging.info("Window adjustment completed, updating cameras")
                        if self.cameras and self.resizing is True:
                            pygame.time.set_timer(self.REDRAW_EVENT, 0)  # 取消定时器
                            #print(f"Window resized to: {width}x{height}")
                            self.resize_timer = None
                            logging.info("destroy camera")
                            for cam_key, cam in self.cameras.items():
                                cam.stop()
                                cam.destroy()
                                logging.info(f"destroy camera {cam_key}")
                            self.cameras.clear()  # 清空摄像头字典
                            self.camera_images = {}
                            self.setup_cameras()
                            self.resizing=False
                    if event.type == pygame.KEYDOWN:
                        if event.key == K_ESCAPE:
                            raise KeyboardInterrupt
                        
                        # 捕获数字键 1, 2, 3 切换鸟瞰视角
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

                keys = pygame.key.get_pressed()

                # 初始化车辆控制
                control = carla.VehicleControl()
                control.brake = 0.0

                # 控制逻辑：加速、后退、刹车
                if keys[K_SPACE]:  # 刹车
                    self.throttle = 0.0
                    control.brake = 1.0
                elif keys[K_UP]:  # 加速
                    self.throttle = 0.5
                    control.reverse = False
                elif keys[K_DOWN]: #
                    self.throttle = 1.0
                    control.reverse=True
                else:  
                    self.throttle = 0.0
                
                if keys[K_LSHIFT]:
                    control.throttle = self.throttle * 1.5
                else:
                    control.throttle = self.throttle

                # 控制逻辑：方向
                if keys[K_LEFT]:
                    self.steer = max(self.steer - self.steer_increment, -1.0)
                elif keys[K_RIGHT]:
                    self.steer = min(self.steer + self.steer_increment, 1.0)
                else:
                    if self.steer > 0:
                        self.steer = max(self.steer - self.steer_increment, 0)
                    elif self.steer < 0:
                        self.steer = min(self.steer + self.steer_increment, 0)

                control.steer = self.steer * 0.25
                self.vehicle.apply_control(control)

                # 可视化
                # if self.resize_timer > 0 and time.time() - self.resize_timer > self.resize_delay:
                #     self.resize_timer = 0  # 重置计时器
                #     if self.cameras:
                #         logging.info("destroy camera")
                #         for cam_key, cam in self.cameras.items():
                #             cam.stop()
                #             cam.destroy()
                #             logging.info(f"destroy camera {cam_key}")
                #         self.camera_images = {}
                #         self.setup_cameras()
                # if self.cameras:
                #     logging.info("destroy camera")
                #     for cam_key, cam in self.cameras.items():
                #         cam.stop()
                #         cam.destroy()
                #         logging.info(f"destroy camera {cam_key}")
                #     self.camera_images = {}
                #     self.setup_cameras()
                self.visualization()

                # 刷新 Pygame 显示

                pygame.display.flip()
                if pygame.time.get_ticks() % 500 == 0:  #
                    print_memory_usage()
                self.clock.tick(60)

        except KeyboardInterrupt:
            logging.info("Exiting...")
        finally:
            self.stop()


def main():
    # 配置日志
    logging.basicConfig(format="%(levelname)s: %(message)s", level=logging.INFO)

    # 设置CARLA Python API路径（
    egg_file = '/home/cityu-fsm-lab-carla/Desktop/Workspace/Carla/carla-0.9.15/PythonAPI/carla/dist/carla-0.9.15-py3.10-linux-x86_64.egg'
    if not os.path.exists(egg_file):
        logging.error(f"CARLA egg file not found at {egg_file}. Please check the path.")
        sys.exit(1)

    sys.path.append(egg_file)

    # 运行EgoVehicleTerminal
    terminal = EgoVehicleTerminal(
        host="127.0.0.1",  # 如果CARLA服务器在其他IP上，修改此处
        port=2000          # 如果CARLA服务器使用其他端口，修改此处
    )
    terminal.run()


if __name__ == "__main__":
    main()