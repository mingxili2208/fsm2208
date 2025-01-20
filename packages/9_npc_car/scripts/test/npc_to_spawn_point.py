#!/usr/bin/env python

import glob
import os
import sys
import random
import time
import logging
import carla
import pygame
import numpy as np
from carla import VehicleLightState as vls
from pygame.locals import K_UP, K_DOWN, K_LEFT, K_RIGHT, K_ESCAPE, K_SPACE

# 添加 CARLA Python API
try:
    sys.path.append(glob.glob('../carla/PythonAPI/carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

from agents.navigation.basic_agent import BasicAgent

class EgoVehicleTerminal:
    def __init__(self, client=None, host="127.0.0.1", port=2000):
        self.client = client or carla.Client(host, port)
        self.client.set_timeout(10.0)
        self.world = self.client.get_world()

        # 初始化 Pygame 窗口
        pygame.init()
        # 每个摄像头图像尺寸
        self.cam_width = 400
        self.cam_height = 300
        # Pygame 窗口尺寸为两层，上层两个摄像头，下层一个摄像头
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
        self.vehicle_model = "vehicle.tesla.model3"  # 您可以更改为其他车型
        self.vehicle_color = "255,0,0"  # RGB格式颜色，例如红色

        # 选择目标 Spawn Point
        self.target_spawn_point = self.choose_target_spawn_point()

        # 生成新车辆
        self.spawn_vehicle()

        # 设置三个摄像头
        self.setup_cameras()

        # 初始化 Agent
        self.agent = BasicAgent(self.vehicle, target_speed=20.0)  # 设置目标速度为20 m/s

        # 生成路径
        self.generate_route()

    def choose_target_spawn_point(self):
        """选择一个目标 spawn point"""
        spawn_points = self.world.get_map().get_spawn_points()
        if not spawn_points:
            logging.error("No spawn points available.")
            sys.exit(1)
        target = random.choice(spawn_points)
        logging.info(f"Target spawn point chosen at location: {target.location}")
        return target

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

        vehicle_bp.set_attribute("role_name", "ego_vehicle")  # 设置角色名称为 'ego_vehicle'

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
        self.vehicle.set_autopilot(False)  # 禁用自动驾驶

    def setup_cameras(self):
        """设置前视、后视和俯视摄像头并附加到ego_vehicle"""
        blueprint_library = self.world.get_blueprint_library()

        # 通用摄像头设置
        camera_bp = blueprint_library.find('sensor.camera.rgb')
        camera_bp.set_attribute('image_size_x', f"{self.cam_width}")
        camera_bp.set_attribute('image_size_y', f"{self.cam_height}")
        camera_bp.set_attribute('fov', '90')  # 调整视野角度

        # 前视摄像头
        front_camera_transform = carla.Transform(carla.Location(x=1.5, z=2.4), carla.Rotation(pitch=0))
        self.cameras['front'] = self.world.spawn_actor(
            camera_bp,
            front_camera_transform,
            attach_to=self.vehicle)
        self.cameras['front'].listen(lambda image: self.process_image(image, 'front'))

        # 后视摄像头
        back_camera_transform = carla.Transform(carla.Location(x=-1.5, z=2.4), carla.Rotation(yaw=180, pitch=0))
        self.cameras['back'] = self.world.spawn_actor(
            camera_bp,
            back_camera_transform,
            attach_to=self.vehicle)
        self.cameras['back'].listen(lambda image: self.process_image(image, 'back'))

        # 俯视摄像头（鸟瞰视角）
        bev_camera_transform = carla.Transform(carla.Location(x=0, z=10), carla.Rotation(pitch=-90, yaw=0))
        self.cameras['bev'] = self.world.spawn_actor(
            camera_bp,
            bev_camera_transform,
            attach_to=self.vehicle)
        self.cameras['bev'].listen(lambda image: self.process_image(image, 'bev'))

    def process_image(self, image, cam_key):
        """处理摄像头图像数据并转换为Pygame可显示格式"""
        array = np.frombuffer(image.raw_data, dtype=np.uint8)
        array = np.reshape(array, (image.height, image.width, 4))  # RGBA
        array = array[:, :, :3]  # RGB
        array = array[:, :, ::-1]  # BGR to RGB

        # 存储图像
        self.camera_images[cam_key] = array.copy()

    def generate_route(self):
        """生成从当前车辆位置到目标 spawn point 的路径"""
        # 获取当前车辆的位置
        current_location = self.vehicle.get_location()
        current_rotation = self.vehicle.get_transform().rotation

        # 寻找最近的 waypoint
        map = self.world.get_map()
        start_waypoint = map.get_waypoint(current_location, project_to_road=True)
        end_waypoint = map.get_waypoint(self.target_spawn_point.location, project_to_road=True)

        # 计算最短路径的 waypoints
        self.route = start_waypoint.next(50)  # 获取前 50 个 waypoints 作为示例

        # 使用 agent 寻找路径
        self.agent.set_destination(self.target_spawn_point.location)

    def visualization(self):
        """在Pygame窗口中显示三个摄像头的图像和控制信息"""
        # 定义每个摄像头图像的显示区域
        cam_positions = {
            'front': (0, 0),
            'back': (self.cam_width, 0),
            'bev': (self.cam_width // 2, self.cam_height)
        }

        for cam_key, image in self.camera_images.items():
            if cam_key in cam_positions:
                # 将图像转换为Pygame显示格式
                surface = pygame.surfarray.make_surface(image.swapaxes(0, 1))
                # 计算bev的显示位置，确保居中
                if cam_key == 'bev':
                    bev_x = cam_positions[cam_key][0]
                    bev_y = cam_positions[cam_key][1]
                    self.screen.blit(surface, (bev_x, bev_y))
                else:
                    # 上层前后摄像头直接放置
                    self.screen.blit(surface, cam_positions[cam_key])

        # 在窗口左上角显示控制提示文字
        font = pygame.font.Font(None, 24)
        text_surface = font.render("Controls: Arrow Keys to Drive, SPACE to Brake, ESC to Exit", True, (255, 255, 255))
        self.screen.blit(text_surface, (10, 10))  # 显示在左上角

        # 显示帧率
        fps_text = font.render(f"FPS: {int(self.clock.get_fps())}", True, (255, 255, 255))
        self.screen.blit(fps_text, (10, 40))

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
                    elif event.type == pygame.KEYDOWN:
                        if event.key == K_ESCAPE:
                            raise KeyboardInterrupt
                    elif event.type == pygame.VIDEORESIZE:
                        self.screen_width, self.screen_height = event.size
                        self.screen = pygame.display.set_mode((self.screen_width, self.screen_height), pygame.RESIZABLE)
                        if self.cameras:
                            # 更新摄像头的图像尺寸
                            for cam_key, cam in self.cameras.items():
                                cam.stop()
                                cam.destroy()
                            self.camera_images = {}
                            self.setup_cameras()

                keys = pygame.key.get_pressed()

                # 初始化车辆控制
                control = carla.VehicleControl()
                control.throttle = 0.0
                control.brake = 0.0
                control.reverse = False

                # 控制逻辑：加速、后退、刹车
                if keys[K_UP]:
                    control.throttle = 1.0
                    control.reverse = False
                elif keys[K_DOWN]:
                    control.throttle = 1.0  # 后退时节气门较低
                    control.reverse = True
                elif keys[K_SPACE]:
                    control.brake = 1.0

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

                control.steer = self.steer
                self.vehicle.apply_control(control)

                # Agent 控制 NPC
                self.agent.run_step()

                # 可视化
                self.visualization()

                # 刷新 Pygame 显示
                pygame.display.flip()
                self.clock.tick(60)

        except KeyboardInterrupt:
            logging.info("Exiting...")
        finally:
            self.stop()


def main():
    # 配置日志
    logging.basicConfig(format="%(levelname)s: %(message)s", level=logging.INFO)

    # 设置CARLA Python API路径（根据您的实际路径修改）
    egg_file = '/home/cityu-fsm-lab-carla/Desktop/Workspace/Carla/carla-0.9.15/PythonAPI/carla/dist/carla-0.9.15-py3.10-linux-x86_64.egg'
    if not os.path.exists(egg_file):
        logging.error(f"CARLA egg file not found at {egg_file}. Please check the path.")
        sys.exit(1)

    sys.path.append(egg_file)
    import random  # 需要在添加到 sys.path 之后导入

    # 运行 EgoVehicleTerminal
    terminal = EgoVehicleTerminal(
        host="127.0.0.1",  # 如果 CARLA 服务器在其他 IP 上，修改此处
        port=2000          # 如果 CARLA 服务器使用其他端口，修改此处
    )
    terminal.run()


if __name__ == "__main__":
    main()