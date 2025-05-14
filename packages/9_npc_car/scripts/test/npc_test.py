#!/usr/bin/env python

import glob
import os
import sys
import time
import argparse
import logging
import pygame
from pygame.locals import K_UP, K_DOWN, K_LEFT, K_RIGHT, K_ESCAPE, K_SPACE

# 设置CARLA Python API路径（根据你的实际路径修改）
sys.path.append('/home/cityu-fsm-lab-carla/Desktop/WorkplaceCarla/carla-0.9.15/PythonAPI/carla/dist/carla-0.9.15-py3.10-linux-x86_64.egg')
import carla

import random


def main():
    # 解析命令行参数
    argparser = argparse.ArgumentParser(
        description="CARLA Keyboard Control Example")
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
        '--no-rendering',
        action='store_true',
        default=False,
        help='Activate no rendering mode (default: False)')
    args = argparser.parse_args()

    # 配置日志
    logging.basicConfig(format='%(levelname)s: %(message)s', level=logging.INFO)

    # 初始化Pygame
    try:
        pygame.init()
    except pygame.error as e:
        logging.error(f"Failed to initialize pygame: {e}")
        sys.exit("Error: Pygame initialization failed.")

    # 动态设置屏幕大小
    screen_width, screen_height = 400, 300
    try:
        info_object = pygame.display.Info()
        screen_width, screen_height = info_object.current_w // 4, info_object.current_h // 4
    except pygame.error:
        logging.warning("Failed to retrieve display info. Using default screen size.")

    screen = pygame.display.set_mode((screen_width, screen_height))
    pygame.display.set_caption('CARLA Keyboard Control')
    clock = pygame.time.Clock()

    try:
        # 连接到CARLA客户端
        client = carla.Client(args.host, args.port)
        client.set_timeout(10.0)
        world = client.get_world()

        # 确保渲染模式设置正确
        settings = world.get_settings()
        settings.no_rendering_mode = args.no_rendering
        world.apply_settings(settings)

        # 获取蓝图库并选择车辆蓝图
        blueprint_library = world.get_blueprint_library()
        vehicle_blueprints = blueprint_library.filter('vehicle.*')

        if not vehicle_blueprints:
            logging.error("No vehicle blueprints found.")
            return

        # 选择一种随机车辆蓝图
        vehicle_bp = random.choice(vehicle_blueprints)
        if vehicle_bp.has_attribute('color'):
            color = random.choice(vehicle_bp.get_attribute('color').recommended_values)
            vehicle_bp.set_attribute('color', color)
        
        vehicle_bp.set_attribute('role_name', 'test_npc')

        # 获取生成点
        spawn_points = world.get_map().get_spawn_points()
        if not spawn_points:
            logging.error("No spawn points available.")
            return

        spawn_point = random.choice(spawn_points)  # 随机选择一个生成点
        test_npc_vehicle = world.try_spawn_actor(vehicle_bp, spawn_point)

        if not test_npc_vehicle:
            logging.error("Failed to spawn the vehicle.")
            return

        test_npc_vehicle.set_autopilot(False)

        logging.info("test_npc vehicle spawned. Press ESC to exit.")

        # 主循环
        steer_value = 0.0
        steer_increment = 0.05

        while True:
            for event in pygame.event.get():
                if event.type == pygame.QUIT or (event.type == pygame.KEYDOWN and event.key == K_ESCAPE):
                    raise KeyboardInterrupt

            keys = pygame.key.get_pressed()

            # 初始化车辆控制
            control = carla.VehicleControl()
            control.throttle = 0.0
            control.brake = 0.0
            control.reverse = False  # 默认前进模式

            # 控制逻辑：加速、后退、刹车
            if keys[K_UP]:
                control.throttle = 1.0
                control.reverse = False  # 前进
            elif keys[K_DOWN]:
                control.throttle = 1.0
                control.reverse = True  # 后退
            elif keys[K_SPACE]:
                control.brake = 1.0  # 刹车

            # 控制逻辑：转向
            if keys[K_LEFT]:
                steer_value = max(steer_value - steer_increment, -1.0)
            elif keys[K_RIGHT]:
                steer_value = min(steer_value + steer_increment, 1.0)
            else:
                # 方向盘自动回正
                if steer_value > 0:
                    steer_value = max(steer_value - steer_increment, 0)
                elif steer_value < 0:
                    steer_value = min(steer_value + steer_increment, 0)

            control.steer = steer_value
            test_npc_vehicle.apply_control(control)

            if args.no_rendering:
                time.sleep(0.05)
            else:
                screen.fill((0, 0, 0))
                pygame.display.flip()
                clock.tick(60)

    except KeyboardInterrupt:
        logging.info("Ctrl+C pressed. Exiting...")
    except Exception as e:
        logging.error(f"Error: {e}")
    finally:
        # 清理工作
        if 'test_npc_vehicle' in locals() and test_npc_vehicle is not None:
            logging.info("Destroying hero vehicle.")
            test_npc_vehicle.destroy()
        pygame.quit()
        logging.info("Done.")


if __name__ == '__main__':
    main()