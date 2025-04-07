#!/usr/bin/env python

# Copyright (c) 2025 James Li
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""
脚本功能：在Carla 0.9.15中设置Traffic Manager
"""

import glob
import os
import sys
import time
import argparse
import logging

try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

import carla

def main():
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
        help='Port to communicate with TM (default: 8000)')
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
    
    args = argparser.parse_args()
    
    logging.basicConfig(format='%(levelname)s: %(message)s', level=logging.INFO)
    
    # 连接到CARLA服务器
    client = carla.Client(args.host, args.port)
    client.set_timeout(10.0)
    
    # 获取世界和地图
    world = client.get_world()
    map = world.get_map()
    
    # 获取所有可用的出生点
    spawn_points = map.get_spawn_points()
    
    # 定义出生点和目标点的索引
    spawn_indices = [35, 15, 40, 44, 21]
    target_indices = [33, 5, 19, 8, 17]
    
    # 创建Traffic Manager
    traffic_manager = client.get_trafficmanager(args.tm_port)
    traffic_manager.set_global_distance_to_leading_vehicle(2.0)
    traffic_manager.set_random_device_seed(args.seed)
    
    # # 设置同步/异步模式
    synchronous_master = False
    settings = world.get_settings()
    if not args.asynch:
        print("not args.asynch")
    #     traffic_manager.set_synchronous_mode(False)
    #     if not settings.synchronous_mode:
    #         synchronous_master = True
    #         settings.synchronous_mode = False
    #         settings.fixed_delta_seconds = 0.033
    # world.apply_settings(settings)
    
    # print('Traffic Manager设置完成')
    # time.sleep(5)
    
    # # 恢复原始设置
    # if not args.asynch and synchronous_master:
    #     settings = world.get_settings()
    #     settings.synchronous_mode = False
    #     settings.fixed_delta_seconds = None
    #     world.apply_settings(settings)

if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        pass
    finally:
        print('\n脚本已终止')