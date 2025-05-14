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

class VehicleMonitor:
    def __init__(self, client=None, host="127.0.0.1", port=2000, vehicle_id=None, vehicle_role_name="pygame_adtruck"):
        self.client = client or carla.Client(host, port)
        self.client.set_timeout(10.0)
        self.world = self.client.get_world()
        self.vehicle = None
        
        # 使用默认值"pygame_adtruck"或者传入的角色名称
        vehicle_role_name = vehicle_role_name or "pygame_adtruck"
        
        # 寻找已存在的车辆，通过ID或角色名称
        if vehicle_id is not None:
            self.vehicle = self.world.get_actor(vehicle_id)
        elif vehicle_role_name is not None:
            for actor in self.world.get_actors().filter('vehicle.*'):
                if actor.attributes.get('role_name') == vehicle_role_name:
                    self.vehicle = actor
                    break
        
        if not self.vehicle:
            logging.error(f"没有找到ID为{vehicle_id}或角色名为{vehicle_role_name}的车辆")
            sys.exit(1)
        
        logging.info(f"已连接到车辆 {self.vehicle.id} (类型: {self.vehicle.type_id})")
        
        # 获取地图的spawn points用于车辆重置
        self.spawn_points = self.world.get_map().get_spawn_points()
        if len(self.spawn_points) < 2:
            print("地图上没有足够的生成点。")
            return
        for point in self.spawn_points:
            point.location.z = 0.17

        # 初始化 Pygame 窗口
        pygame.init()
        # 每个摄像头图像尺寸
        self.cam_width = 400
        self.cam_height = 300
        # Pygame window set as front & back up; bev down
        self.screen_width = self.cam_width * 2
        self.screen_height = self.cam_height * 2
        self.screen = pygame.display.set_mode((self.screen_width, self.screen_height), pygame.RESIZABLE)
        pygame.display.set_caption("CARLA Vehicle monitor")
        self.clock = pygame.time.Clock()

        # 控制相关参数
        self.throttle = 0.0
        self.steer = 0.0
        self.steer_increment = 0.05
        self.cameras = {}
        self.camera_images = {}

        self.resize_timer = None
        self.resizing = False
        self.bev_transfrom = None
        self.REDRAW_EVENT = pygame.USEREVENT + 1
        
        # 设置三个摄像头
        self.setup_cameras()

    def get_nearest_spawn_point(self):
        """获取小车最近的 spawn_point"""
        vehicle_location = self.vehicle.get_transform().location
        nearest_spawn_point = None
        min_distance = float('inf')

        for spawn_point in self.spawn_points:
            spawn_location = spawn_point.location
            distance = math.sqrt(
                (vehicle_location.x - spawn_location.x) ** 2 +
                (vehicle_location.y - spawn_location.y) ** 2 +
                (vehicle_location.z - spawn_location.z) ** 2
            )

            if distance < min_distance:
                min_distance = distance
                nearest_spawn_point = spawn_point

        return nearest_spawn_point

    def relocate_vehicle_to_nearest_spawn_point(self):
        """将小车刷新到最近的 spawn_point"""
        nearest_spawn_point = self.get_nearest_spawn_point()

        if nearest_spawn_point:
            print(f"将车辆重置到最近的生成点: {nearest_spawn_point.location}")
            self.vehicle.set_transform(nearest_spawn_point)
            control = carla.VehicleControl()
            control.throttle = 0.0
            control.brake = 1.0
            self.vehicle.apply_control(control)
        else:
            print("未找到最近的生成点!")

    def setup_cameras(self):
        """设置前视、后视和俯视摄像头并附加到车辆"""
        blueprint_library = self.world.get_blueprint_library()

        # 前视摄像头 - 现在设置为宽画面，占据底部宽位置
        front_camera_bp = blueprint_library.find('sensor.camera.rgb')
        front_camera_bp.set_attribute('image_size_x', f"{self.cam_width * 2}")  # 宽度为两倍
        front_camera_bp.set_attribute('image_size_y', f"{self.cam_height}")
        front_camera_bp.set_attribute('fov', '90')  # 调整视野角度

        front_camera_transform = carla.Transform(carla.Location(x=1.5, z=2.4), carla.Rotation(pitch=0))
        self.cameras['front'] = self.world.spawn_actor(
            front_camera_bp,
            front_camera_transform,
            attach_to=self.vehicle)
        self.cameras['front'].listen(lambda image: self.process_image(image, 'front'))
        
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

        # 俯视摄像头（鸟瞰视角）- 现在设置为单格位置
        bev_camera_bp = blueprint_library.find('sensor.camera.rgb')
        bev_camera_bp.set_attribute('image_size_x', f"{self.cam_width}")
        bev_camera_bp.set_attribute('image_size_y', f"{self.cam_height}")
        bev_camera_bp.set_attribute('fov', '90')  # 调整视野角度
        self.bev_transfrom = carla.Transform(carla.Location(x=-20, z=11.5), carla.Rotation(pitch=-30, yaw=0))
        self.cameras['bev'] = self.world.spawn_actor(
            bev_camera_bp,
            self.bev_transfrom,
            attach_to=self.vehicle)
        self.cameras['bev'].listen(lambda image: self.process_image(image, 'bev'))
    
    def update_bev_camera(self):
        """更新鸟瞰摄像头的位置和朝向"""
        if 'bev' in self.cameras:
            # 停止并销毁旧的鸟瞰摄像头
            self.cameras['bev'].stop()
            self.cameras['bev'].destroy()

            # 创建新的鸟瞰摄像头
            blueprint_library = self.world.get_blueprint_library()
            bev_camera_bp = blueprint_library.find('sensor.camera.rgb')
            bev_camera_bp.set_attribute('image_size_x', f"{self.cam_width}")  # 单格宽度
            bev_camera_bp.set_attribute('image_size_y', f"{self.cam_height}")
            bev_camera_bp.set_attribute('fov', '90')  # 调整视野角度
            
            # 生成新的摄像头并附加到车辆
            self.cameras['bev'] = self.world.spawn_actor(
                bev_camera_bp,
                self.bev_transfrom,
                attach_to=self.vehicle
            )
            self.cameras['bev'].listen(lambda image: self.process_image(image, 'bev'))
            logging.info(f"更新BEV摄像头位置: {self.bev_transfrom.location}, 旋转: {self.bev_transfrom.rotation}")

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
        # 定义每个摄像头图像的显示区域 - 前视现在在下方占据整行，BEV和后视在上方
        cam_positions = {
            'bev': (0, 0),  # BEV在左上角
            'back': (self.cam_width, 0),  # 后视在右上角
            'front': (0, self.cam_height),  # 前视在下方占据整行
        }

        for cam_key, image in self.camera_images.items():
            if cam_key in cam_positions:
                # 将图像转换为Pygame显示格式
                surface = pygame.surfarray.make_surface(image.swapaxes(0, 1))
                self.screen.blit(surface, cam_positions[cam_key])

        # 显示车辆信息
        font = pygame.font.Font(None, 24)
        vehicle_info = f"vehicle_ID: {self.vehicle.id} | vehicle_Type: {self.vehicle.type_id}"
        text_surface = font.render(vehicle_info, True, (255, 255, 255))
        self.screen.blit(text_surface, (10, 10))
        
        # 显示车速信息
        velocity = self.vehicle.get_velocity()
        speed = 3.6 * math.sqrt(velocity.x**2 + velocity.y**2 + velocity.z**2)  # 转换为km/h
        speed_text = f"Speed: {speed-3.2999:.1f} km/h"
        speed_surface = font.render(speed_text, True, (255, 255, 255))
        self.screen.blit(speed_surface, (10, 40))
        
        # 显示键位控制说明
        controls_text = "control_key: ESC:exit, R:reload_vehicle, 1-3 exchange_BEV"
        controls_surface = font.render(controls_text, True, (255, 255, 255))
        self.screen.blit(controls_surface, (10, self.screen_height - 30))

    def stop(self):
        """停止传感器并退出 Pygame"""
        for camera in self.cameras.values():
            if camera:
                camera.stop()
                camera.destroy()
        pygame.quit()

    def run(self):
        """主循环，处理事件并显示画面"""
        try:
            while True:
                for event in pygame.event.get():
                    if event.type == pygame.QUIT:
                        raise KeyboardInterrupt
                    if event.type == pygame.VIDEORESIZE:
                        self.resizing = True
                        logging.info(f"调整窗口大小为 {event.size[0]}x{event.size[1]}")
                        self.screen_width, self.screen_height = event.size
                        self.screen = pygame.display.set_mode((self.screen_width, self.screen_height), pygame.RESIZABLE)
                        
                        # 更新摄像头的图像尺寸
                        self.cam_width = self.screen_width // 2
                        self.cam_height = self.screen_height // 2
                        if self.resize_timer:
                            pygame.time.set_timer(self.REDRAW_EVENT, 0)  # 取消之前的定时器
                        pygame.time.set_timer(self.REDRAW_EVENT, 500)  # 500毫秒后触发重绘事件
                    
                    elif event.type == self.REDRAW_EVENT:
                        logging.info("窗口调整完成，更新摄像头")
                        if self.cameras and self.resizing is True:
                            pygame.time.set_timer(self.REDRAW_EVENT, 0)  # 取消定时器
                            self.resize_timer = None
                            logging.info("销毁摄像头")
                            for cam_key, cam in self.cameras.items():
                                cam.stop()
                                cam.destroy()
                                logging.info(f"销毁摄像头 {cam_key}")
                            self.cameras.clear()  # 清空摄像头字典
                            self.camera_images = {}
                            self.setup_cameras()
                            self.resizing = False
                    
                    if event.type == pygame.KEYDOWN:
                        if event.key == K_ESCAPE:
                            raise KeyboardInterrupt
                        
                        if event.key == pygame.K_r:
                            self.relocate_vehicle_to_nearest_spawn_point()

                        if event.key == pygame.K_1:
                            self.bev_transfrom = carla.Transform(carla.Location(x=-20, z=11.5), carla.Rotation(pitch=-30, yaw=0))
                            self.update_bev_camera()
                        elif event.key == pygame.K_2:
                            self.bev_transfrom = carla.Transform(carla.Location(x=-15, z=22.5), carla.Rotation(pitch=-50, yaw=0))
                            self.update_bev_camera()
                        elif event.key == pygame.K_3:
                            self.bev_transfrom = carla.Transform(carla.Location(x=-5, z=40.0), carla.Rotation(pitch=-80, yaw=0))
                            self.update_bev_camera()

                # 可视化
                self.visualization()

                # 刷新 Pygame 显示
                pygame.display.flip()
                if pygame.time.get_ticks() % 500 == 0:
                    print_memory_usage()
                self.clock.tick(60)

        except KeyboardInterrupt:
            logging.info("正在退出...")
        finally:
            self.stop()


def main():
    # 配置日志
    logging.basicConfig(format="%(levelname)s: %(message)s", level=logging.INFO)

    # 设置CARLA Python API路径
    egg_file = '/home/cityu-fsm-lab-carla/Desktop/WorkplaceCarla/carla-0.9.15/PythonAPI/carla/dist/carla-0.9.15-py3.10-linux-x86_64.egg'
    if not os.path.exists(egg_file):
        logging.error(f"在{egg_file}未找到CARLA egg文件。请检查路径。")
        sys.exit(1)

    sys.path.append(egg_file)

    # 解析命令行参数
    import argparse
    parser = argparse.ArgumentParser(description='CARLA_vehicle_monitor')
    parser.add_argument('--host', default='127.0.0.1', help='CARLA_sever')
    parser.add_argument('--port', default=2000, type=int, help='CARLA_server_port')
    parser.add_argument('--vehicle-id', type=int, help='vechile_ID')
    parser.add_argument('--role-name', default='pygame_adtruck', help='vehicle_name')
    args = parser.parse_args()

    # 运行VehicleMonitor
    monitor = VehicleMonitor(
        host=args.host,
        port=args.port,
        vehicle_id=args.vehicle_id,
        vehicle_role_name=args.role_name
    )
    monitor.run()


if __name__ == "__main__":
    main()