#!/usr/bin/env python

# Copyright (c) 2025 James LI
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""
脚本功能：在Carla 0.9.15中生成NPC车辆，由Traffic Manager管理
这些车辆在指定出生点生成，并导航到对应的目标点
优化版本：适应Autoware代理环境，减少冲突
"""

import glob
import os
import sys
import time
import random
import numpy as np
import argparse
import logging
import inspect
import datetime
import socket
import json
import setproctitle

setproctitle.setproctitle('scenario_test')
try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

import carla

# 常量定义
EVENT_PORT = 8700  # 用于发送事件通知的端口
EVENT_SERVER = '127.0.0.1'  # 事件服务器地址

def print_tm_info(tm, world):
    """打印Traffic Manager和世界同步模式的详细信息"""
    print("\n=== Traffic Manager 和世界设置信息 ===")
    
    # 打印世界设置
    settings = world.get_settings()

    print(f"世界设置:")
    print(f"  - 同步模式: {settings.synchronous_mode}")
    print(f"  - 固定时间步长: {settings.fixed_delta_seconds}")
    print(f"  - 子步模式: {settings.substepping}")
    print(f"  - 最大子步时间: {settings.max_substep_delta_time}")
    
    # 打印Traffic Manager信息
    print(f"Traffic Manager信息:")
    print(f"  - 端口: {tm.get_port()}")
    
    # 检查Traffic Manager是否处于同步模式
    try:
        is_sync = tm.get_synchronous_mode()
        print(f"  - 同步模式: {is_sync}")
    except:
        print(f"  - 同步模式: 无法获取")
    
    # 获取全局设置
    print("\nTraffic Manager全局设置:")
    try:
        global_distance = None
        # 尝试获取全局跟车距离
        if hasattr(tm, 'get_global_distance_to_leading_vehicle'):
            global_distance = tm.get_global_distance_to_leading_vehicle()
        
        if global_distance is not None:
            print(f"  - 全局跟车距离: {global_distance}米")
        else:
            print(f"  - 全局跟车距离: 无法获取")
    except Exception as e:
        print(f"  - 全局跟车距离: 获取时出错 ({e})")
    
    # 检查和打印当前活动的车辆
    vehicles = world.get_actors().filter('vehicle.*')
    print(f"\n当前世界中的车辆: {len(vehicles)}辆")
    
    for i, vehicle in enumerate(vehicles):
        try:
            autopilot = vehicle.get_autopilot()
            role = vehicle.attributes.get('role_name', 'unknown')
            print(f"  车辆 {i+1}: ID={vehicle.id}, 类型={vehicle.type_id}, 角色={role}, 自动驾驶={autopilot}")
        except:
            print(f"  车辆 {i+1}: ID={vehicle.id}, 类型={vehicle.type_id}")
    
    print("=======================================\n")

def send_event(event_type, event_data):
    """向性能监控服务发送事件通知"""
    try:
        # 创建UDP套接字
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        
        # 构建消息
        message = {
            "type": event_type,
            "time": time.time(),
            "data": event_data
        }
        
        # 发送消息
        sock.sendto(json.dumps(message).encode(), (EVENT_SERVER, EVENT_PORT))
        return True
    except Exception as e:
        print(f"发送事件失败: {e}")
        return False

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
        help='Port to communicate with TM (default: 8200)')
    argparser.add_argument(
        '--alt-tm-port',
        metavar='P',
        default=8200,
        type=int,
        help='Alternative TM port if primary is in use (default: 8100)')
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
    argparser.add_argument(
        '--num-vehicles',
        type=int,
        default=5,
        help='Number of NPC vehicles to spawn (default: 5)')
    argparser.add_argument(
        '--safe-mode',
        action='store_true',
        default=True,
        help='Enable safer driving parameters for NPCs')
    argparser.add_argument(
        '--batch-spawn',
        action='store_true',
        help='Spawn vehicles in batches for better performance')
    argparser.add_argument(
        '--config',
        type=str,
        help='Path to configuration JSON file')
    
    args = argparser.parse_args()
    
    # 如果提供了配置文件，从配置文件加载
    if args.config and os.path.exists(args.config):
        try:
            with open(args.config, 'r') as f:
                config = json.load(f)
                
                # 更新参数
                args.host = config.get('host', args.host)
                args.port = config.get('port', args.port)
                args.num_vehicles = config.get('number-of-vehicles', args.num_vehicles)
                args.seed = config.get('seed', args.seed)
                args.safe_mode = config.get('safe', args.safe_mode)
                args.asynch = config.get('asynch', args.asynch)
                
                print(f"已从配置文件加载设置: {args.config}")
        except Exception as e:
            print(f"加载配置文件时出错: {e}")
    
    logging.basicConfig(format='%(levelname)s: %(message)s', level=logging.INFO)
    
    # 发送启动事件
    send_event("script_start", {
        "script": "carla_npc_manager.py",
        "args": vars(args)
    })
    
    # 连接到CARLA服务器
    client = carla.Client(args.host, args.port)
    client.set_timeout(10.0)
    
    # 获取世界和地图
    world = client.get_world()
    map = world.get_map()
    
    # 发送环境信息事件
    send_event("environment_info", {
        "carla_version": client.get_client_version(),
        "map": map.name,
        "synchronous_mode": world.get_settings().synchronous_mode,
        "fixed_delta_seconds": world.get_settings().fixed_delta_seconds
    })
    
    # 获取当前世界设置
    original_settings = world.get_settings()
    
    # 检测是否有Traffic Manager正在运行
    tm_port = args.tm_port
    tm_connected = False
    
    try:
        # 尝试连接到默认Traffic Manager端口
        traffic_manager = client.get_trafficmanager(tm_port)
        print(f"连接到现有Traffic Manager，端口：{tm_port}")
        tm_connected = True
    except Exception as e:
        print(f"连接到默认Traffic Manager (端口:{tm_port})失败: {e}")
        # 如果失败，尝试使用备用端口
        try:
            tm_port = args.alt_tm_port
            traffic_manager = client.get_trafficmanager(tm_port)
            print(f"创建新的Traffic Manager，端口：{tm_port}")
            tm_connected = True
        except Exception as e:
            print(f"无法创建新的Traffic Manager (端口:{tm_port}): {e}")
            print("退出脚本...")
            send_event("error", {"message": f"无法创建Traffic Manager: {e}"})
            return
    
    # 打印Traffic Manager和世界设置信息
    if tm_connected:
        print_tm_info(traffic_manager, world)
    
    # 设置Traffic Manager参数 - 用于与Autoware共存的安全设置
    traffic_manager.set_global_distance_to_leading_vehicle(5.0)  # 增加跟车距离
    traffic_manager.set_random_device_seed(args.seed)
    
    # 适应当前的同步/异步模式，而不是强制更改
    synchronous_master = False
    
    # 再次打印设置信息，以确认更改已生效
    print("\n=== 更新后的设置 ===")
    print_tm_info(traffic_manager, world)
    
    # 获取所有可用的出生点
    spawn_points = map.get_spawn_points()
    
    # 定义出生点和目标点的索引 - 使用日志中匹配的索引
    spawn_indices = [35, 15, 40,  21]
    target_indices = [33, 5, 19,  17]
    
    # 定义要生成的特定车辆类型，与日志匹配
    vehicle_types = [
        "vehicle.tesla.cybertruck",
        "vehicle.gazelle.omafiets",
        "vehicle.nissan.patrol_2021",
        "vehicle.harley-davidson.low_rider"
    ]
    
    # 定义车辆颜色 - 与日志匹配
    vehicle_colors = [
        None,  # 默认颜色
        "202,88,176",
        "159,0,0",
        "0,38,132"
    ]
    
    # 确保我们有足够的索引
    num_vehicles = min(args.num_vehicles, len(vehicle_types))
    
    # 创建车辆蓝图
    blueprint_library = world.get_blueprint_library()
    
    print(f'准备生成 {num_vehicles} 辆指定车辆')
    
    # 存储生成的车辆
    vehicle_list = []
    
    # 批量生成命令列表
    batch_commands = []
    
    # 记录已有的车辆（在脚本启动时就存在的车辆）
    existing_vehicles = {}
    
    try:
        print('生成NPC车辆...')
        
        # 发送车辆生成开始事件
        send_event("spawn_begin", {"num_vehicles": num_vehicles})
        
        # 检查当前已有的车辆数量并记录它们的信息
        existing_actors = world.get_actors().filter('vehicle.*')
        print(f'当前世界中已有 {len(existing_actors)} 辆车辆')
        
        # 记录现有车辆信息到字典中
        for vehicle in existing_actors:
            existing_vehicles[vehicle.id] = {
                'type': vehicle.type_id,
                'role': vehicle.attributes.get('role_name', 'unknown')
            }
            print(f'记录现有车辆: ID={vehicle.id}, 类型={vehicle.type_id}, 角色={existing_vehicles[vehicle.id]["role"]}')
        
        # 生成指定数量的车辆并设置其目标点
        for i in range(num_vehicles):
            # 获取指定的车辆蓝图
            vehicle_id = vehicle_types[i]
            bp = blueprint_library.find(vehicle_id)
            
            # 输出所选车辆的ID和类型信息
            vehicle_type = vehicle_id.split('.')[1] if '.' in vehicle_id else "未知"
            
            print(f'为车辆 {i+1} 选择蓝图: ID={vehicle_id}, 类型={vehicle_type}')
            
            # 设置指定的颜色（如果有）
            if vehicle_colors[i] and bp.has_attribute('color'):
                bp.set_attribute('color', vehicle_colors[i])
                print(f'  颜色: {vehicle_colors[i]}')
            
            # 设置角色名称为autopilot
            bp.set_attribute('role_name', 'npc_vehicle')
            
            # 在指定的出生点生成车辆
            spawn_idx = spawn_indices[i]
            spawn_point = spawn_points[spawn_idx]
            
            # 修改spawn_point的高度为0.2
            spawn_point.location.z = 0.2
            
            # 如果使用批量生成，添加到命令列表
            if args.batch_spawn:
                batch_commands.append(carla.command.SpawnActor(bp, spawn_point))
                continue
            
            # 否则单独生成
            vehicle = world.try_spawn_actor(bp, spawn_point)
            
            # 如果无法在预定位置生成，尝试随机位置
            spawn_attempts = 0
            while vehicle is None and spawn_attempts < 5:
                spawn_attempts += 1
                print(f"  尝试备用位置 {spawn_attempts}...")
                # 稍微调整位置，而不是随机选择新的出生点
                modified_spawn = carla.Transform(
                    carla.Location(
                        x=spawn_point.location.x + random.uniform(-1.0, 1.0),
                        y=spawn_point.location.y + random.uniform(-1.0, 1.0),
                        z=0.2
                    ),
                    spawn_point.rotation
                )
                vehicle = world.try_spawn_actor(bp, modified_spawn)
            
            if vehicle is not None:
                print(f'车辆 {i+1} ({vehicle_id}) 已生成')
                vehicle_list.append(vehicle)
                
                # 发送车辆生成事件
                send_event("vehicle_spawned", {
                    "id": vehicle.id,
                    "type": vehicle_id,
                    "location": {"x": spawn_point.location.x, "y": spawn_point.location.y, "z": spawn_point.location.z}
                })
                
                # 将车辆交给Traffic Manager控制
                vehicle.set_autopilot(True, tm_port)
                
                # 设置目标点
                target_idx = target_indices[i]
                target_point = spawn_points[target_idx].location
                traffic_manager.set_path(vehicle, [target_point])
                
                # 发送目标点设置事件
                send_event("target_set", {
                    "vehicle_id": vehicle.id,
                    "target": {"x": target_point.x, "y": target_point.y, "z": target_point.z}
                })
                
                # 设置一些车辆行为参数 - 安全驾驶设置
                if args.safe_mode:
                    try:
                        traffic_manager.auto_lane_change(vehicle, True)  # 禁用自动变道
                    except:
                        print("  注意: 无法设置自动变道参数")
                    
                    try:
                        traffic_manager.distance_to_leading_vehicle(vehicle, 5.0)  # 设置较大的跟车距离
                    except:
                        print("  注意: 无法设置跟车距离")
                    
                    try:
                        traffic_manager.vehicle_percentage_speed_difference(vehicle, random.uniform(10, 30))  # 显著降低速度
                    except:
                        print("  注意: 无法设置速度差异")
                    
                    try:
                        if hasattr(traffic_manager, 'set_desired_speed'):
                            traffic_manager.set_desired_speed(vehicle, 20)  # 限制最高速度为20km/h
                    except:
                        print("  注意: 无法设置期望速度")
                    
                    print(f'  已启用安全驾驶模式')
                
                print(f'  目标点: spawn_point[{target_idx}]')
            else:
                print(f'无法生成车辆 ({vehicle_id})，已尝试多个位置')
                # 发送车辆生成失败事件
                send_event("spawn_failed", {
                    "type": vehicle_id
                })
        
        # 如果使用批量生成，执行批处理
        if args.batch_spawn and batch_commands:
            print(f"批量生成 {len(batch_commands)} 辆车辆...")
            results = client.apply_batch_sync(batch_commands, True)
            
            # 处理结果，获取车辆引用
            for i, result in enumerate(results):
                if result.error:
                    print(f"  车辆 {i+1} 生成失败: {result.error}")
                    send_event("spawn_failed", {
                        "index": i,
                        "error": str(result.error)
                    })
                else:
                    # 获取车辆引用
                    vehicle = world.get_actor(result.actor_id)
                    if vehicle:
                        vehicle_list.append(vehicle)
                        print(f"  车辆 {i+1} (ID={result.actor_id}) 生成成功")
                        
                        # 发送车辆生成事件
                        send_event("vehicle_spawned", {
                            "id": result.actor_id,
                            "batch_index": i
                        })
                        
                        # 将车辆交给Traffic Manager控制
                        vehicle.set_autopilot(True, tm_port)
                        
                        # 设置目标点
                        target_idx = target_indices[i]
                        target_point = spawn_points[target_idx].location
                        traffic_manager.set_path(vehicle, [target_point])
                        
                        # 安全驾驶设置
                        if args.safe_mode:
                            try:
                                traffic_manager.auto_lane_change(vehicle, False)
                                traffic_manager.distance_to_leading_vehicle(vehicle, 8.0)
                                traffic_manager.vehicle_percentage_speed_difference(vehicle, random.uniform(30, 50))
                            except:
                                pass
                    else:
                        print(f"  无法获取车辆 {i+1} (ID={result.actor_id}) 的引用")
        
        print(f'成功生成 {len(vehicle_list)} 辆NPC车辆')
        print('按Ctrl+C停止模拟...')
        
        # 发送生成完成事件
        send_event("spawn_complete", {
            "vehicles_spawned": len(vehicle_list),
            "vehicles_requested": num_vehicles
        })
        
        # 主循环
        try:
            while True:
                if synchronous_master:
                    world.tick()
                else:
                    world.wait_for_tick()
                
                # 每10秒发送一次心跳事件
                if int(time.time()) % 10 == 0:
                    send_event("heartbeat", {
                        "active_vehicles": len(vehicle_list),
                        "time": time.time()
                    })
                    time.sleep(0.1)  # 避免在同一秒内多次发送
        except KeyboardInterrupt:
            print('\n模拟被用户中断')
        
    except KeyboardInterrupt:
        print('\n模拟被用户中断')
        # 发送中断事件
        send_event("interrupted", {
            "time": time.time()
        })
    except Exception as e:
        print(f'遇到错误: {e}')
        # 发送错误事件
        send_event("error", {
            "message": str(e),
            "time": time.time()
        })
    finally:
        # 清理并恢复原始设置
        print('正在清理模拟...')
        
        # 恢复原始设置
        if synchronous_master:
            world.apply_settings(original_settings)
            print('已恢复原始世界设置')
        
        # 获取当前所有车辆
        current_vehicles = world.get_actors().filter('vehicle.*')
        print(f'当前世界中有 {len(current_vehicles)} 辆车辆')
        
        # 第一步：识别哪些车辆是本脚本创建的（非先前存在的）
        vehicles_to_destroy = []
        for vehicle in current_vehicles:
            # 如果车辆ID不在先前记录的现有车辆中，则认为是本脚本创建的
            if vehicle.id not in existing_vehicles:
                vehicles_to_destroy.append(vehicle)
                print(f"标记待删除车辆: ID={vehicle.id}, 类型={vehicle.type_id}")
        
        print(f"第1步：需要删除 {len(vehicles_to_destroy)} 辆由本脚本创建的车辆")
        
        # 第2步：关闭所有待删除车辆的自动驾驶
        active_vehicles = []
        for i, vehicle in enumerate(vehicles_to_destroy):
            try:
                if vehicle.is_alive:
                    print(f"  关闭车辆 {i+1} (ID={vehicle.id}) 的自动驾驶")
                    vehicle.set_autopilot(False)
                    active_vehicles.append(vehicle)
                else:
                    print(f"  车辆 {i+1} 已不存在，跳过")
            except Exception as e:
                print(f"  关闭车辆自动驾驶失败: {e}")
        
        # 等待短暂时间，让Traffic Manager反应
        time.sleep(0.5)
        
        # 第3步：逐个销毁车辆
        destroyed_count = 0
        print(f"第3步：逐个销毁 {len(active_vehicles)} 辆活跃车辆")
        for i, vehicle in enumerate(active_vehicles):
            try:
                if vehicle.is_alive:
                    vehicle_id = vehicle.id
                    vehicle.destroy()
                    destroyed_count += 1
                    print(f"  成功销毁车辆 {i+1} (ID={vehicle_id})")
                    # 短暂等待，避免过快销毁造成的问题
                    time.sleep(0.05)
                else:
                    print(f"  车辆 {i+1} 已不存在，跳过")
            except Exception as e:
                print(f"  销毁车辆失败: {e}")
        
        # 第4步：检查是否有任何需要删除的车辆仍然存活，并尝试批量销毁
        print("第4步：检查并批量销毁任何剩余车辆")
        try:
            # 再次获取当前所有车辆
            final_check = world.get_actors().filter('vehicle.*')
            surviving_vehicles = []
            
            # 检查哪些是我们应该删除但仍然存活的车辆
            for vehicle in final_check:
                if vehicle.id not in existing_vehicles and vehicle.is_alive:
                    surviving_vehicles.append(vehicle)
            
            if surviving_vehicles:
                print(f"  尝试批量销毁剩余的 {len(surviving_vehicles)} 辆车辆")
                client.apply_batch([carla.command.DestroyActor(v) for v in surviving_vehicles])
                print("  批量销毁命令已发送")
            else:
                print("  没有剩余的车辆需要销毁")
        except Exception as e:
            print(f"  批量销毁车辆失败: {e}")
        
        # 发送清理完成事件
        send_event("cleanup_complete", {
            "vehicles_destroyed": destroyed_count,
            "vehicles_attempted": len(vehicles_to_destroy),
            "time": time.time()
        })
        
        time.sleep(0.5)
        print('模拟已结束')
        
        # 发送脚本结束事件
        send_event("script_end", {
            "exit_status": "normal",
            "time": time.time()
        })

if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        send_event("script_end", {
            "exit_status": "keyboard_interrupt", 
            "time": time.time()
        })
    except Exception as e:
        print(f'运行时错误: {e}')
        send_event("script_end", {
            "exit_status": "error",
            "error": str(e), 
            "time": time.time()
        })
    finally:
        print('\n脚本已终止')