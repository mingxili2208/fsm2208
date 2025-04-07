#!/usr/bin/env python

# Copyright (c) 2025 James LI
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""
脚本功能：在Carla中生成NPC车辆并强制它们移动
这个版本针对车辆不移动问题进行了深度优化
"""

import glob
import os
import sys
import time
import random
import math
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
        default=8000,
        type=int,
        help='Port to communicate with TM (default: 8000)')
    argparser.add_argument(
        '--num-vehicles',
        type=int,
        default=5,
        help='Number of NPC vehicles to spawn (default: 5)')
    argparser.add_argument(
        '--debug',
        action='store_true',
        help='Enable debug mode with additional output')
    
    args = argparser.parse_args()
    
    logging.basicConfig(format='%(levelname)s: %(message)s', level=logging.INFO)
    
    # 连接到CARLA服务器
    client = carla.Client(args.host, args.port)
    client.set_timeout(10.0)
    
    # 获取世界和地图
    world = client.get_world()
    map = world.get_map()
    
    # 保存原始设置
    original_settings = world.get_settings()
    
    # 检查当前设置
    settings = world.get_settings()
    if args.debug:
        print(f"当前设置: 同步模式={settings.synchronous_mode}, 固定步长={settings.fixed_delta_seconds}")
    
    # 强制设置同步模式 (重要: 这可能与其他程序冲突，但确保我们能控制车辆运动)
    settings.synchronous_mode = True
    settings.fixed_delta_seconds = 0.05
    world.apply_settings(settings)
    print(f"已设置同步模式: 步长={settings.fixed_delta_seconds}秒")
    
    # 创建或连接到Traffic Manager
    try:
        # 确保关闭所有现有的Traffic Manager
        try:
            previous_tm = client.get_trafficmanager(args.tm_port)
            previous_tm.shut_down()
            print(f"关闭了现有的Traffic Manager (端口 {args.tm_port})")
        except:
            pass

        # 创建新的Traffic Manager
        tm_port = args.tm_port
        traffic_manager = client.get_trafficmanager(tm_port)
        
        # 确保Traffic Manager与世界同步
        traffic_manager.set_synchronous_mode(True)
        print(f"Traffic Manager已设置为同步模式")
        
        # 设置车辆行为 (激进设置以确保车辆移动)
        traffic_manager.set_global_distance_to_leading_vehicle(1.0)  # 较小的距离
        traffic_manager.global_percentage_speed_difference(-50.0)    # 速度比限速快50%
        
        print(f"Traffic Manager (端口={tm_port}) 已初始化，并设置为激进驾驶模式")
    except Exception as e:
        print(f"Traffic Manager初始化失败: {e}")
        # 恢复设置并退出
        world.apply_settings(original_settings)
        return
    
    # 获取所有可用的出生点
    spawn_points = map.get_spawn_points()
    
    # 确保有足够的出生点
    if len(spawn_points) < args.num_vehicles:
        print(f"警告: 请求了{args.num_vehicles}辆车，但只找到{len(spawn_points)}个出生点")
        num_vehicles = len(spawn_points)
    else:
        num_vehicles = args.num_vehicles
    
    # 随机选择出生点
    random.shuffle(spawn_points)
    
    # 获取车辆蓝图
    blueprint_library = world.get_blueprint_library()
    car_blueprints = [bp for bp in blueprint_library.filter('vehicle.tesla.model3')]
    
    if not car_blueprints:
        # 如果没有Tesla Model 3，则使用任何小型车
        car_blueprints = [bp for bp in blueprint_library.filter('vehicle.audi.tt')]
    
    if not car_blueprints:
        # 如果还是没有，使用任何可用车辆
        car_blueprints = [bp for bp in blueprint_library.filter('vehicle.*') 
                         if 'bicycle' not in bp.id.lower()]
    
    if not car_blueprints:
        print("错误: 找不到任何车辆蓝图")
        world.apply_settings(original_settings)
        return
    
    # 创建空的车辆列表
    vehicles = []
    
    # 第一轮: 先仅生成车辆，不设置自动驾驶
    print("\n===== 第一阶段: 生成车辆 =====")
    spawn_count = 0
    spawn_commands = []
    
    for i in range(num_vehicles):
        try:
            # 选择车辆蓝图
            blueprint = random.choice(car_blueprints)
            
            # 设置属性
            if blueprint.has_attribute('color'):
                color = random.choice(blueprint.get_attribute('color').recommended_values)
                blueprint.set_attribute('color', color)
                
            # 设置为不可摧毁
            if blueprint.has_attribute('role_name'):
                blueprint.set_attribute('role_name', 'autopilot')
            
            # 创建生成命令
            spawn_commands.append(carla.command.SpawnActor(blueprint, spawn_points[i]))
            spawn_count += 1
            
        except Exception as e:
            print(f"生成第 {i+1} 辆车辆时发生错误: {e}")
    
    # 批量生成车辆
    print(f"批量生成 {spawn_count} 辆车辆...")
    responses = client.apply_batch_sync(spawn_commands, True)
    
    # 检查响应
    vehicle_ids = []
    for i, response in enumerate(responses):
        if response.error:
            print(f"  车辆 {i+1} 生成失败: {response.error}")
        else:
            vehicle_ids.append(response.actor_id)
            print(f"  车辆 {i+1} 生成成功, ID={response.actor_id}")
    
    # 等待一个 tick，确保车辆已正确生成
    world.tick()
    
    # 获取车辆引用
    vehicles = world.get_actors(vehicle_ids)
    print(f"成功生成 {len(vehicles)} 辆车辆")
    
    # 第二轮: 为每辆车设置自动驾驶
    print("\n===== 第二阶段: 设置自动驾驶 =====")
    for i, vehicle in enumerate(vehicles):
        try:
            # 尝试使用SetAutopilot命令
            client.apply_batch_sync([
                carla.command.SetAutopilot(vehicle.id, True, tm_port)
            ], True)
            
            # 尝试直接修改参数
            traffic_manager.auto_lane_change(vehicle, True)  # 更积极的变道
            traffic_manager.vehicle_percentage_speed_difference(vehicle, -80)  # 更快的速度
            traffic_manager.distance_to_leading_vehicle(vehicle, 0.5)  # 更小的跟车距离
            
            print(f"  车辆 {i+1} (ID={vehicle.id}) 已设置自动驾驶")
        except Exception as e:
            print(f"  设置车辆 {i+1} (ID={vehicle.id}) 自动驾驶时出错: {e}")
    
    # 等待一些tick让Traffic Manager接管
    print("\n执行几个tick以启动车辆...")
    for _ in range(20):
        world.tick()
        time.sleep(0.05)
    
    # 第三轮: 验证车辆是否在移动，如果没有，使用直接控制
    print("\n===== 第三阶段: 验证移动和强制控制 =====")
    moving_count = 0
    for i, vehicle in enumerate(vehicles):
        try:
            if vehicle.is_alive:
                vel = vehicle.get_velocity()
                speed = 3.6 * math.sqrt(vel.x**2 + vel.y**2 + vel.z**2)  # km/h
                
                # 显示速度
                print(f"  车辆 {i+1} (ID={vehicle.id}) 当前速度: {speed:.1f} km/h")
                
                # 如果速度过低，给一个初始推力
                if speed < 5.0:  # 小于5 km/h认为是静止
                    print(f"    车辆静止，应用推力...")
                    # 创建前进控制
                    control = carla.VehicleControl(throttle=1.0, steer=0.0)
                    vehicle.apply_control(control)
                else:
                    moving_count += 1
        except Exception as e:
            print(f"  检查车辆 {i+1} 时出错: {e}")
    
    print(f"\n当前有 {moving_count}/{len(vehicles)} 辆车在移动")
    
    # 再等待一些tick
    print("再执行一些tick...")
    for _ in range(20):
        world.tick()
        time.sleep(0.05)
    
    # 第四轮: 最后检查和手动推动
    print("\n===== 第四阶段: 最终检查和手动控制 =====")
    
    # 重新配置Traffic Manager，最激进的设置
    try:
        traffic_manager.set_global_distance_to_leading_vehicle(0.1)  # 极小距离
        traffic_manager.global_percentage_speed_difference(-100.0)   # 极快速度
        print("Traffic Manager设置为极其激进模式")
    except Exception as e:
        print(f"重新配置Traffic Manager失败: {e}")
    
    # 最终检查
    moving_count = 0
    stuck_vehicles = []
    
    for i, vehicle in enumerate(vehicles):
        try:
            if vehicle.is_alive:
                vel = vehicle.get_velocity()
                speed = 3.6 * math.sqrt(vel.x**2 + vel.y**2 + vel.z**2)  # km/h
                
                print(f"  车辆 {i+1} (ID={vehicle.id}) 最终速度: {speed:.1f} km/h")
                
                if speed < 5.0:
                    stuck_vehicles.append(vehicle)
                else:
                    moving_count += 1
        except Exception as e:
            print(f"  最终检查车辆 {i+1} 时出错: {e}")
    
    print(f"\n最终结果: {moving_count}/{len(vehicles)} 辆车在移动")
    
    # 对仍然卡住的车辆应用持续控制
    if stuck_vehicles:
        print(f"\n===== 第五阶段: 对 {len(stuck_vehicles)} 辆卡住的车辆应用持续控制 =====")
        
        # 创建一个强制移动的闭包函数
        def force_move(vehicles):
            try:
                for v in vehicles:
                    if v.is_alive:
                        # 创建一个waypoint来获取道路方向
                        waypoint = map.get_waypoint(v.get_location())
                        if waypoint:
                            # 获取前进方向
                            forward = waypoint.transform.get_forward_vector()
                            
                            # 创建一个推力
                            impulse = carla.Vector3D(
                                x=forward.x * 50000.0,
                                y=forward.y * 50000.0,
                                z=0.0
                            )
                            
                            # 应用推力
                            v.add_impulse(impulse)
                            print(f"  应用推力到车辆 ID={v.id}")
                            
                            # 直接设置速度
                            v.set_target_velocity(carla.Vector3D(x=forward.x*10, y=forward.y*10, z=0))
                            
                            # 应用控制
                            control = carla.VehicleControl(throttle=1.0, steer=0.0)
                            v.apply_control(control)
            except Exception as e:
                print(f"应用强制移动时出错: {e}")
        
        # 应用强制移动并在主循环中继续
        force_move(stuck_vehicles)
    
    # 主循环
    print("\n===== 进入主循环，按Ctrl+C退出 =====")
    try:
        tick_counter = 0
        while True:
            # 执行tick
            world.tick()
            tick_counter += 1
            
            # 每50个tick检查一次
            if tick_counter % 50 == 0:
                moving_count = 0
                for vehicle in vehicles:
                    if vehicle.is_alive:
                        vel = vehicle.get_velocity()
                        speed = 3.6 * math.sqrt(vel.x**2 + vel.y**2 + vel.z**2)
                        if speed > 5.0:
                            moving_count += 1
                
                print(f"Tick {tick_counter}: {moving_count}/{len(vehicles)} 辆车在移动")
                
                # 对卡住的车再次应用强制
                if stuck_vehicles:
                    force_move(stuck_vehicles)
            
            # 控制循环速度
            time.sleep(0.01)
            
    except KeyboardInterrupt:
        print('\n模拟被用户中断')
    finally:
        # 恢复原始设置
        print('恢复原始设置...')
        world.apply_settings(original_settings)
        
        # 销毁车辆
        print(f'销毁 {len(vehicles)} 辆车辆...')
        client.apply_batch([carla.command.DestroyActor(x) for x in vehicles if x.is_alive])
        
        print('模拟结束')

if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print('\n程序被中断')
    except Exception as e:
        print(f'发生错误: {e}')
    finally:
        print('程序终止')