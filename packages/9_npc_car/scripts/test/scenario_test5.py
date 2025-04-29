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
import setproctitle
from typing import List, Dict, Any, Optional

setproctitle.setproctitle('scenario_test')
try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

import carla

# 尝试导入ROS2相关库
try:
    import rclpy
    from rclpy.node import Node
    from autoware_auto_vehicle_msgs.msg import Engage
    from builtin_interfaces.msg import Time
    from rclpy.time import Time as rclpy_time
    from rclpy.parameter import Parameter
    from rclpy.clock import ClockType
    from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
    from rosgraph_msgs.msg import Clock
    
    ROS2_AVAILABLE = True
except ImportError as e:
    ROS2_AVAILABLE = False
    logging.warning(f"ROS2或Autoware相关库导入失败: {e}")
    logging.warning("Autoware engage功能将被禁用")


class EngagePublisher(Node):
    """
    用于向Autoware发送engage命令的ROS2节点
    """
    def __init__(self):
        super().__init__('carla_engage_publisher')
        
        # 创建QoS配置，确保可靠的消息传递
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # 创建发布者
        self.engage_publisher = self.create_publisher(
            Engage, "/autoware/engage", qos
        )
        
        # 创建订阅者来监听/clock话题
        self.clock_subscription = self.create_subscription(
            Clock, '/clock', self.clock_callback, qos
        )
        
        # 存储最新的时钟时间
        self.latest_clock = Time()
        self.clock_received = False
        
        self.get_logger().info("EngagePublisher节点已初始化")
    
    def clock_callback(self, msg: Clock):
        """处理/clock话题的回调函数"""
        self.latest_clock = msg.clock
        self.clock_received = True
    
    def wait_for_clock(self, timeout_sec: float = 5.0) -> bool:
        """等待接收到至少一条clock消息"""
        start_time = time.time()
        while not self.clock_received and time.time() - start_time < timeout_sec:
            rclpy.spin_once(self, timeout_sec=0.1)
        
        if self.clock_received:
            self.get_logger().info(f"接收到/clock: {self.latest_clock.sec}.{self.latest_clock.nanosec}")
        else:
            self.get_logger().warn(f"等待/clock超时，将使用系统时间")
        
        return self.clock_received
    
    def publish_engage(self, engage_state: bool):
        """发布engage命令到Autoware"""
        msg = Engage()
        
        # 如果收到了/clock消息，使用其作为时间戳；否则使用节点的当前时间
        if self.clock_received:
            msg.stamp = self.latest_clock
        else:
            msg.stamp = self.get_clock().now().to_msg()
        
        msg.engage = engage_state
        
        self.engage_publisher.publish(msg)
        self.get_logger().info(f"发布engage状态: {engage_state}, 时间戳: {msg.stamp.sec}.{msg.stamp.nanosec}")


class Logger:
    """处理日志记录的类"""
    
    @staticmethod
    def setup(log_file=None):
        """设置日志记录器"""
        timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        
        # 如果没有提供日志文件名，则创建带时间戳的默认文件名
        if log_file is None:
            log_dir = "logs"
            # 创建日志目录（如果不存在）
            if not os.path.exists(log_dir):
                os.makedirs(log_dir)
            log_file = os.path.join(log_dir, f"carla_npc_{timestamp}.log")
        
        # 配置根日志记录器
        logger = logging.getLogger()
        logger.setLevel(logging.INFO)
        
        # 清除任何现有的处理程序
        if logger.handlers:
            logger.handlers.clear()
        
        # 创建文件处理程序
        file_handler = logging.FileHandler(log_file)
        file_formatter = logging.Formatter('%(asctime)s - %(levelname)s - %(message)s')
        file_handler.setFormatter(file_formatter)
        logger.addHandler(file_handler)
        
        # 创建控制台处理程序
        console_handler = logging.StreamHandler(sys.stdout)
        console_formatter = logging.Formatter('%(levelname)s: %(message)s')
        console_handler.setFormatter(console_formatter)
        logger.addHandler(console_handler)
        
        # 记录日志文件位置
        logger.info(f"日志记录到: {os.path.abspath(log_file)}")
        
        return logger


class TrafficManagerInfo:
    """Traffic Manager信息处理类"""
    
    @staticmethod
    def print_info(tm, world):
        """打印Traffic Manager和世界同步模式的详细信息"""
        logging.info("\n=== Traffic Manager 和世界设置信息 ===")
        
        # 打印世界设置
        settings = world.get_settings()

        logging.info(f"世界设置:")
        logging.info(f"  - 同步模式: {settings.synchronous_mode}")
        logging.info(f"  - 固定时间步长: {settings.fixed_delta_seconds}")
        logging.info(f"  - 子步模式: {settings.substepping}")
        logging.info(f"  - 最大子步时间: {settings.max_substep_delta_time}")
        
        # 打印Traffic Manager信息
        logging.info(f"Traffic Manager信息:")
        logging.info(f"  - 端口: {tm.get_port()}")
        
        # 检查Traffic Manager是否处于同步模式
        try:
            is_sync = tm.get_synchronous_mode()
            logging.info(f"  - 同步模式: {is_sync}")
        except:
            logging.info(f"  - 同步模式: 无法获取")
        
        # 获取全局设置
        logging.info("\nTraffic Manager全局设置:")
        try:
            global_distance = None
            # 尝试获取全局跟车距离
            if hasattr(tm, 'get_global_distance_to_leading_vehicle'):
                global_distance = tm.get_global_distance_to_leading_vehicle()
            
            if global_distance is not None:
                logging.info(f"  - 全局跟车距离: {global_distance}米")
            else:
                logging.info(f"  - 全局跟车距离: 无法获取")
        except Exception as e:
            logging.info(f"  - 全局跟车距离: 获取时出错 ({e})")
        
        # 检查和打印当前活动的车辆
        vehicles = world.get_actors().filter('vehicle.*')
        logging.info(f"\n当前世界中的车辆: {len(vehicles)}辆")
        
        for i, vehicle in enumerate(vehicles):
            try:
                autopilot = vehicle.get_autopilot()
                role = vehicle.attributes.get('role_name', 'unknown')
                logging.info(f"  车辆 {i+1}: ID={vehicle.id}, 类型={vehicle.type_id}, 角色={role}, 自动驾驶={autopilot}")
            except:
                logging.info(f"  车辆 {i+1}: ID={vehicle.id}, 类型={vehicle.type_id}")
        
        logging.info("=======================================\n")


class CarlaManager:
    """管理CARLA连接和车辆生成的类"""
    
    def __init__(self, args):
        """初始化CarlaManager"""
        self.args = args
        self.client = None
        self.world = None
        self.map = None
        self.traffic_manager = None
        self.tm_port = args.tm_port
        self.blueprint_library = None
        self.spawn_points = []
        self.vehicle_list = []
        self.existing_vehicles = {}
        self.original_settings = None
        self.synchronous_master = False
    
    def connect(self):
        """连接到CARLA服务器"""
        self.client = carla.Client(self.args.host, self.args.port)
        self.client.set_timeout(10.0)
        
        # 获取世界和地图
        self.world = self.client.get_world()
        self.map = self.world.get_map()
        
        # 记录环境信息
        logging.info(f"CARLA版本: {self.client.get_client_version()}")
        logging.info(f"地图: {self.map.name}")
        logging.info(f"同步模式: {self.world.get_settings().synchronous_mode}")
        logging.info(f"固定时间步长: {self.world.get_settings().fixed_delta_seconds}")
        
        # 保存当前设置
        self.original_settings = self.world.get_settings()
        
        # 获取蓝图库
        self.blueprint_library = self.world.get_blueprint_library()
        
        # 获取所有可用的出生点
        self.spawn_points = self.map.get_spawn_points()
        
        return True
    
    def setup_traffic_manager(self):
        """设置并连接到Traffic Manager"""
        # 检测是否有Traffic Manager正在运行
        tm_connected = False
        
        try:
            # 尝试连接到默认Traffic Manager端口
            self.traffic_manager = self.client.get_trafficmanager(self.tm_port)
            logging.info(f"连接到现有Traffic Manager，端口：{self.tm_port}")
            tm_connected = True
        except Exception as e:
            logging.warning(f"连接到默认Traffic Manager (端口:{self.tm_port})失败: {e}")
            # 如果失败，尝试使用备用端口
            try:
                self.tm_port = self.args.alt_tm_port
                self.traffic_manager = self.client.get_trafficmanager(self.tm_port)
                logging.info(f"创建新的Traffic Manager，端口：{self.tm_port}")
                tm_connected = True
            except Exception as e:
                logging.error(f"无法创建新的Traffic Manager (端口:{self.tm_port}): {e}")
                return False
        
        # 打印Traffic Manager和世界设置信息
        if tm_connected:
            TrafficManagerInfo.print_info(self.traffic_manager, self.world)
        
        # 设置Traffic Manager参数 - 用于与Autoware共存的安全设置
        self.traffic_manager.set_global_distance_to_leading_vehicle(5.0)  # 增加跟车距离
        self.traffic_manager.set_random_device_seed(self.args.seed)
        
        # 再次打印设置信息，以确认更改已生效
        logging.info("\n=== 更新后的设置 ===")
        TrafficManagerInfo.print_info(self.traffic_manager, self.world)
        
        return tm_connected
    
    def get_existing_vehicles(self):
        """获取并记录已有的车辆"""
        # 检查当前已有的车辆数量并记录它们的信息
        existing_actors = self.world.get_actors().filter('vehicle.*')
        logging.info(f'当前世界中已有 {len(existing_actors)} 辆车辆')
        
        # 记录现有车辆信息到字典中
        for vehicle in existing_actors:
            self.existing_vehicles[vehicle.id] = {
                'type': vehicle.type_id,
                'role': vehicle.attributes.get('role_name', 'unknown')
            }
            logging.info(f'记录现有车辆: ID={vehicle.id}, 类型={vehicle.type_id}, 角色={self.existing_vehicles[vehicle.id]["role"]}')
    
    def spawn_vehicle(self, vehicle_id, color, spawn_idx, target_idx):
        """生成单个车辆并设置其属性"""
        # 获取指定的车辆蓝图
        bp = self.blueprint_library.find(vehicle_id)
        
        # 输出所选车辆的ID和类型信息
        vehicle_type = vehicle_id.split('.')[1] if '.' in vehicle_id else "未知"
        logging.info(f'选择蓝图: ID={vehicle_id}, 类型={vehicle_type}')
        
        # 设置指定的颜色（如果有）
        if color and bp.has_attribute('color'):
            bp.set_attribute('color', color)
            logging.info(f'  颜色: {color}')
        
        # 设置角色名称为autopilot
        bp.set_attribute('role_name', 'npc_vehicle')
        
        # 在指定的出生点生成车辆
        spawn_point = self.spawn_points[spawn_idx]
        
        # 修改spawn_point的高度为0.2
        spawn_point.location.z = 0.2
        
        # 生成车辆
        vehicle = self.world.try_spawn_actor(bp, spawn_point)
        
        # 如果无法在预定位置生成，尝试随机位置
        spawn_attempts = 0
        while vehicle is None and spawn_attempts < 5:
            spawn_attempts += 1
            logging.info(f"  尝试备用位置 {spawn_attempts}...")
            # 稍微调整位置，而不是随机选择新的出生点
            modified_spawn = carla.Transform(
                carla.Location(
                    x=spawn_point.location.x + random.uniform(-1.0, 1.0),
                    y=spawn_point.location.y + random.uniform(-1.0, 1.0),
                    z=0.2
                ),
                spawn_point.rotation
            )
            vehicle = self.world.try_spawn_actor(bp, modified_spawn)
        
        if vehicle is not None:
            logging.info(f'车辆 ({vehicle_id}) 已生成, ID={vehicle.id}')
            
            # 将车辆交给Traffic Manager控制
            vehicle.set_autopilot(True, self.tm_port)
            
            # 设置目标点
            target_point = self.spawn_points[target_idx].location
            self.traffic_manager.set_path(vehicle, [target_point])
            
            logging.info(f'  设置目标点: spawn_point[{target_idx}]')
            logging.info(f'  目标坐标: x={target_point.x:.2f}, y={target_point.y:.2f}, z={target_point.z:.2f}')
            
            # 设置一些车辆行为参数 - 安全驾驶设置
            if self.args.safe_mode:
                try:
                    self.traffic_manager.auto_lane_change(vehicle, True)  # 禁用自动变道
                except:
                    logging.warning("  注意: 无法设置自动变道参数")
                
                try:
                    self.traffic_manager.distance_to_leading_vehicle(vehicle, 5.0)  # 设置较大的跟车距离
                except:
                    logging.warning("  注意: 无法设置跟车距离")
                
                try:
                    self.traffic_manager.vehicle_percentage_speed_difference(vehicle, random.uniform(0, 10))  # 显著降低速度
                except:
                    logging.warning("  注意: 无法设置速度差异")
                
                try:
                    if hasattr(self.traffic_manager, 'set_desired_speed'):
                        self.traffic_manager.set_desired_speed(vehicle, 25)  # 限制最高速度为25km/h
                except:
                    logging.warning("  注意: 无法设置期望速度")
                
                logging.info(f'  已启用安全驾驶模式')
            
            return vehicle
        else:
            logging.error(f'无法生成车辆 ({vehicle_id})，已尝试多个位置')
            return None
    
    def spawn_vehicles(self):
        """生成所有指定的车辆"""
        # 定义出生点和目标点的索引
        spawn_indices = [15, 40, 21, 44, 29]
        target_indices = [5, 19, 17, 8, 8]
        
        # 定义要生成的特定车辆类型
        vehicle_types = [
            "vehicle.carlamotors.european_hgv",
            "vehicle.mini.cooper_s",
            "vehicle.nissan.patrol_2021",
            "vehicle.tesla.cybertruck",
            "vehicle.mini.cooper_s"
            # "vehicle.tesla.cybertruck"
        ]
        
        # 定义车辆颜色
        vehicle_colors = [
            "202,88,176",
            "159,0,0",
            "0,38,132",
            None,
            "202,88,176"
        ]
        
        # 确保我们有足够的索引
        num_vehicles = min(self.args.num_vehicles, len(vehicle_types))
        
        logging.info(f'准备生成 {num_vehicles} 辆指定车辆')
        
        # 获取并记录现有车辆
        self.get_existing_vehicles()
        
        # 单独生成车辆
        if not self.args.batch_spawn:
            for i in range(num_vehicles):
                vehicle = self.spawn_vehicle(
                    vehicle_types[i], 
                    vehicle_colors[i], 
                    spawn_indices[i], 
                    target_indices[i]
                )
                if vehicle:
                    self.vehicle_list.append(vehicle)
        
        # 批量生成车辆
        else:
            batch_commands = []
            for i in range(num_vehicles):
                # 获取指定的车辆蓝图
                vehicle_id = vehicle_types[i]
                bp = self.blueprint_library.find(vehicle_id)
                
                # 设置指定的颜色（如果有）
                if vehicle_colors[i] and bp.has_attribute('color'):
                    bp.set_attribute('color', vehicle_colors[i])
                
                # 设置角色名称
                bp.set_attribute('role_name', 'npc_vehicle')
                
                # 准备出生点
                spawn_idx = spawn_indices[i]
                spawn_point = self.spawn_points[spawn_idx]
                spawn_point.location.z = 0.2
                
                # 添加到命令列表
                batch_commands.append(carla.command.SpawnActor(bp, spawn_point))
            
            # 执行批量生成
            logging.info(f"批量生成 {len(batch_commands)} 辆车辆...")
            results = self.client.apply_batch_sync(batch_commands, True)
            
            # 处理结果，获取车辆引用
            for i, result in enumerate(results):
                if result.error:
                    logging.error(f"  车辆 {i+1} 生成失败: {result.error}")
                else:
                    # 获取车辆引用
                    vehicle = self.world.get_actor(result.actor_id)
                    if vehicle:
                        self.vehicle_list.append(vehicle)
                        logging.info(f"  车辆 {i+1} (ID={result.actor_id}) 生成成功")
                        
                        # 将车辆交给Traffic Manager控制
                        vehicle.set_autopilot(True, self.tm_port)
                        
                        # 设置目标点
                        target_idx = target_indices[i]
                        target_point = self.spawn_points[target_idx].location
                        self.traffic_manager.set_path(vehicle, [target_point])
                        
                        # 安全驾驶设置
                        if self.args.safe_mode:
                            try:
                                self.traffic_manager.auto_lane_change(vehicle, False)
                                self.traffic_manager.distance_to_leading_vehicle(vehicle, 5)
                                self.traffic_manager.vehicle_percentage_speed_difference(vehicle, random.uniform(0, 10))
                            except:
                                pass
                    else:
                        logging.error(f"  无法获取车辆 {i+1} (ID={result.actor_id}) 的引用")
        
        logging.info(f'成功生成 {len(self.vehicle_list)} 辆NPC车辆')
        return len(self.vehicle_list) > 0
    
    def engage_autoware(self):
        """向Autoware发送engage指令"""
        if not self.args.engage_autoware or not ROS2_AVAILABLE:
            if not ROS2_AVAILABLE:
                logging.warning("ROS2未加载，无法发送engage指令")
            return False
        
        try:
            logging.info('尝试向Autoware发送engage指令...')
            
            # 初始化ROS2
            if not rclpy.ok():
                rclpy.init()
            
            # 创建发布者并发送engage指令
            engage_pub = EngagePublisher()
            
            # 等待/clock话题
            clock_received = engage_pub.wait_for_clock(5.0)
            
            # 发送engage命令
            engage_pub.publish_engage(True)
            
            # 确保消息被发送
            for _ in range(10):  # 自旋几次以确保消息发送
                rclpy.spin_once(engage_pub, timeout_sec=0.1)
            
            logging.info('成功向Autoware发送engage指令')
            
            # 关闭ROS2节点
            engage_pub.destroy_node()
            rclpy.shutdown()
            
            return True
            
        except Exception as e:
            logging.error(f'发送engage指令时出错: {e}')
            return False
    
    def monitor_vehicles(self):
        """监控生成的车辆状态"""
        try:
            while True:
                # 每60秒记录一次状态信息
                if int(time.time()) % 600 == 0:
                    # 检查车辆状态
                    active_count = 0
                    for i, vehicle in enumerate(self.vehicle_list):
                        if vehicle.is_alive:
                            active_count += 1
                            
                            # 获取当前位置和速度
                            loc = vehicle.get_location()
                            vel = vehicle.get_velocity()
                            speed = 3.6 * np.sqrt(vel.x**2 + vel.y**2 + vel.z**2)  # km/h
                            
                            logging.info(f"车辆 {i+1} (ID={vehicle.id}) - "
                                        f"位置: ({loc.x:.1f}, {loc.y:.1f}, {loc.z:.1f}), "
                                        f"速度: {speed:.1f} km/h")
                    
                    logging.info(f"活动车辆: {active_count}/{len(self.vehicle_list)}")
                    time.sleep(0.1)  # 避免在同一秒内多次记录
                else:
                    time.sleep(0.1)  # 减少CPU使用率
        except KeyboardInterrupt:
            logging.info('\n监控被用户中断')
            return
    
    def cleanup(self):
        """清理生成的车辆和资源"""
        # 恢复原始设置
        if self.synchronous_master:
            self.world.apply_settings(self.original_settings)
            logging.info('已恢复原始世界设置')
        
        # 获取当前所有车辆
        current_vehicles = self.world.get_actors().filter('vehicle.*')
        logging.info(f'当前世界中有 {len(current_vehicles)} 辆车辆')
        
        # 第一步：识别哪些车辆是本脚本创建的（非先前存在的）
        vehicles_to_destroy = []
        for vehicle in current_vehicles:
            # 如果车辆ID不在先前记录的现有车辆中，则认为是本脚本创建的
            if vehicle.id not in self.existing_vehicles:
                vehicles_to_destroy.append(vehicle)
                logging.info(f"标记待删除车辆: ID={vehicle.id}, 类型={vehicle.type_id}")
        
        logging.info(f"第1步：需要删除 {len(vehicles_to_destroy)} 辆由本脚本创建的车辆")
        
        # 第2步：关闭所有待删除车辆的自动驾驶
        active_vehicles = []
        for i, vehicle in enumerate(vehicles_to_destroy):
            try:
                if vehicle.is_alive:
                    logging.info(f"  关闭车辆 {i+1} (ID={vehicle.id}) 的自动驾驶")
                    vehicle.set_autopilot(False)
                    active_vehicles.append(vehicle)
                else:
                    logging.info(f"  车辆 {i+1} 已不存在，跳过")
            except Exception as e:
                logging.error(f"  关闭车辆自动驾驶失败: {e}")
        
        # 等待短暂时间，让Traffic Manager反应
        time.sleep(0.5)
        
        # 第3步：逐个销毁车辆
        destroyed_count = 0
        logging.info(f"第3步：逐个销毁 {len(active_vehicles)} 辆活跃车辆")
        for i, vehicle in enumerate(active_vehicles):
            try:
                if vehicle.is_alive:
                    vehicle_id = vehicle.id
                    vehicle.destroy()
                    destroyed_count += 1
                    logging.info(f"  成功销毁车辆 {i+1} (ID={vehicle_id})")
                    # 短暂等待，避免过快销毁造成的问题
                    time.sleep(0.05)
                else:
                    logging.info(f"  车辆 {i+1} 已不存在，跳过")
            except Exception as e:
                logging.error(f"  销毁车辆失败: {e}")
        
        # 第4步：检查是否有任何需要删除的车辆仍然存活，并尝试批量销毁
        logging.info("第4步：检查并批量销毁任何剩余车辆")
        try:
            # 再次获取当前所有车辆
            final_check = self.world.get_actors().filter('vehicle.*')
            surviving_vehicles = []
            
            # 检查哪些是我们应该删除但仍然存活的车辆
            for vehicle in final_check:
                if vehicle.id not in self.existing_vehicles and vehicle.is_alive:
                    surviving_vehicles.append(vehicle)
            
            if surviving_vehicles:
                logging.info(f"  尝试批量销毁剩余的 {len(surviving_vehicles)} 辆车辆")
                self.client.apply_batch([carla.command.DestroyActor(v) for v in surviving_vehicles])
                logging.info("  批量销毁命令已发送")
            else:
                logging.info("  没有剩余的车辆需要销毁")
        except Exception as e:
            logging.error(f"  批量销毁车辆失败: {e}")
        
        time.sleep(0.5)


def parse_arguments():
    """解析命令行参数"""
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
        help='Port to communicate with TM (default: 8100)')
    argparser.add_argument(
        '--alt-tm-port',
        metavar='P',
        default=8200,
        type=int,
        help='Alternative TM port if primary is in use (default: 8200)')
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
    argparser.add_argument(
        '--log-file',
        type=str,
        help='Path to log file')
    argparser.add_argument(
        '--engage-autoware',
        action='store_true',
        default=True,
        help='Send engage command to Autoware after spawning vehicles')
    
    args = argparser.parse_args()
    
    # 如果提供了配置文件，从配置文件加载
    if args.config and os.path.exists(args.config):
        try:
            import json
            with open(args.config, 'r') as f:
                config = json.load(f)
                
                # 更新参数
                args.host = config.get('host', args.host)
                args.port = config.get('port', args.port)
                args.num_vehicles = config.get('number-of-vehicles', args.num_vehicles)
                args.seed = config.get('seed', args.seed)
                args.safe_mode = config.get('safe', args.safe_mode)
                args.asynch = config.get('asynch', args.asynch)
                args.engage_autoware = config.get('engage-autoware', args.engage_autoware)
                
                logging.info(f"已从配置文件加载设置: {args.config}")
        except Exception as e:
            logging.error(f"加载配置文件时出错: {e}")
    
    return args


def main():
    # 设置日志记录器
    logger = Logger.setup()
    
    # 解析命令行参数
    args = parse_arguments()
    
    # 如果提供了日志文件参数，重新设置日志记录器
    if args.log_file:
        logger = Logger.setup(args.log_file)
    
    # 记录脚本启动信息
    logging.info("="*50)
    logging.info(f"脚本启动: carla_npc_manager.py")
    logging.info(f"启动时间: {datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
    logging.info(f"参数: {vars(args)}")
    logging.info("="*50)
    
    # 创建CARLA管理器
    carla_manager = CarlaManager(args)
    
    try:
        # 连接到CARLA服务器
        if not carla_manager.connect():
            logging.error("连接到CARLA服务器失败")
            return
        
        # 设置Traffic Manager
        if not carla_manager.setup_traffic_manager():
            logging.error("设置Traffic Manager失败")
            return
        
        # **关键修改**: 先向Autoware发送engage指令，再生成NPC车辆
        if args.engage_autoware:
            logging.info("首先向Autoware发送engage指令...")
            engage_success = carla_manager.engage_autoware()
            if engage_success:
                # 等待一小段时间让Autoware处理engage命令
                logging.info("等待Autoware处理engage指令 (3秒)...")
                time.sleep(2)
            else:
                logging.warning("Autoware engage失败，将继续生成NPC车辆")
        
        # 生成NPC车辆
        if carla_manager.spawn_vehicles():
            # 监控车辆状态
            logging.info('按Ctrl+C停止模拟...')
            carla_manager.monitor_vehicles()
        else:
            logging.error("生成NPC车辆失败")
    
    except KeyboardInterrupt:
        logging.info('\n模拟被用户中断')
    except Exception as e:
        logging.error(f'遇到错误: {e}')
        # 打印详细的堆栈跟踪
        import traceback
        logging.error(traceback.format_exc())
    finally:
        # 清理资源
        logging.info('正在清理模拟...')
        try:
            carla_manager.cleanup()
        except Exception as e:
            logging.error(f'清理时出错: {e}')
        
        logging.info('模拟已结束')
        logging.info("="*50)
        logging.info(f"脚本结束时间: {datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
        logging.info("="*50)


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        logging.info('\n脚本被键盘中断')
    except Exception as e:
        logging.error(f'运行时错误: {e}')
    finally:
        logging.info('\n脚本已终止')