#!/usr/bin/env python

# Copyright (c) 2025 James Li
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""
脚本功能：在Carla 0.9.15中生成5个由Traffic Manager管理的NPC车辆，
这5辆车分别在指定出生点生成，并导航到对应的目标点
"""

import glob
import os
import sys
import time
import random
import numpy as np
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
    
    # 设置同步/异步模式
    synchronous_master = False
    settings = world.get_settings()
    # if not args.asynch:
    #     traffic_manager.set_synchronous_mode(False)
    #     if not settings.synchronous_mode:
    #         synchronous_master = True
    #         settings.synchronous_mode = False
    #         settings.fixed_delta_seconds = 0.033
    # world.apply_settings(settings)
    
    # 创建车辆蓝图
    blueprint_library = world.get_blueprint_library()
    # 筛选所有车辆，但排除奥迪车型和自行车
    vehicle_blueprints = [bp for bp in blueprint_library.filter('vehicle.*') 
                         if 'audi' not in bp.id.lower() and 'rider' not in bp.id.lower()]
    
    print(f'可用车辆蓝图数量: {len(vehicle_blueprints)}')
    
    # 存储生成的车辆
    vehicle_list = []
    
    try:
        print('生成NPC车辆...')
        
        # 生成5辆车并设置其目标点
        for i in range(5):
            # 选择一个随机的车辆蓝图（不包含奥迪和自行车）
            bp = random.choice(vehicle_blueprints)
            
            # 输出所选车辆的ID和类型信息
            vehicle_id = bp.id
            vehicle_type = "未知"
            
            # 尝试解析车辆类型（如轿车、SUV等）
            if bp.has_attribute('type'):
                vehicle_type = bp.get_attribute('type')
            elif '.' in vehicle_id:
                # 从ID中提取车辆类型信息
                parts = vehicle_id.split('.')
                if len(parts) >= 2:
                    vehicle_type = parts[1]  # 通常第二部分是制造商
            
            print(f'为车辆 {i+1} 选择蓝图: ID={vehicle_id}, 类型={vehicle_type}')
            
            # 尝试给车辆涂上随机颜色
            if bp.has_attribute('color'):
                color = random.choice(bp.get_attribute('color').recommended_values)
                bp.set_attribute('color', color)
                print(f'  颜色: {color}')
            
            # 设置角色名称为autopilot
            bp.set_attribute('role_name', 'autopilot')
            
            # 在指定的出生点生成车辆
            spawn_point = spawn_points[spawn_indices[i]]
            vehicle = world.try_spawn_actor(bp, spawn_point)
            
            if vehicle is not None:
                print(f'车辆 {i+1} ({vehicle_id}) 已生成在 spawn_point[{spawn_indices[i]}]')
                vehicle_list.append(vehicle)
                
                # 将车辆交给Traffic Manager控制
                vehicle.set_autopilot(True, traffic_manager.get_port())
                
                # 设置目标点
                target_point = spawn_points[target_indices[i]].location
                traffic_manager.set_path(vehicle, [target_point])
                
                # 设置一些车辆行为参数
                traffic_manager.auto_lane_change(vehicle, True)
                traffic_manager.distance_to_leading_vehicle(vehicle, 5.0)
                speed_diff = random.uniform(-20, 10)
                traffic_manager.vehicle_percentage_speed_difference(vehicle, speed_diff)
                
                print(f'  已设置速度差异: {speed_diff:.1f}%')
                print(f'  目标点: spawn_point[{target_indices[i]}]')
            else:
                print(f'无法在 spawn_point[{spawn_indices[i]}] 生成车辆 ({vehicle_id})')
        
        print('所有NPC车辆已生成并设置路径')
        print('按Ctrl+C停止模拟...')
        
        # 主循环
        while True:
            if not args.asynch and synchronous_master:
                world.tick()
            else:
                world.wait_for_tick()
            
    except KeyboardInterrupt:
        print('\n模拟被用户中断')
    finally:
        # 清理并恢复原始设置
        print('正在清理模拟...')
        
        # 恢复原始设置
        if not args.asynch and synchronous_master:
            settings = world.get_settings()
            settings.synchronous_mode = False
            settings.fixed_delta_seconds = None
            world.apply_settings(settings)
        
        # 销毁所有车辆
        print('\n销毁 %d 辆车辆' % len(vehicle_list))
        client.apply_batch([carla.command.DestroyActor(x) for x in vehicle_list])
        
        time.sleep(0.5)
        print('模拟已结束')

if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        pass
    finally:
        print('\n脚本已终止')