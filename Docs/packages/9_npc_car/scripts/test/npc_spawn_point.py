#!/usr/bin/env python3

import carla
import time
import random
import math
import os

class BridgeHelpers(object):
    @staticmethod
    def get_agent_actor(world, role_name):
        actors = world.get_actors().filter("vehicle.*")
        for car in actors:
            if car.attributes["role_name"] == role_name:
                return car
        return None

def calculate_route(world, start_location, end_location):
    """
    使用 CARLA Waypoint API 计算从起点到终点的路径
    """
    # 获取最近的起点和终点路点
    start_waypoint = world.get_map().get_waypoint(start_location, project_to_road=True, lane_type=carla.LaneType.Driving)
    end_waypoint = world.get_map().get_waypoint(end_location, project_to_road=True, lane_type=carla.LaneType.Driving)
    
    # 使用 Waypoint API 生成路径
    route = []
    current_waypoint = start_waypoint
    while current_waypoint.transform.location.distance(end_waypoint.transform.location) > 2.0:  # 距离终点小于2米停止生成
        route.append(current_waypoint)
        next_waypoints = current_waypoint.next(2.0)  # 每次前进2米
        if not next_waypoints:
            break
        current_waypoint = next_waypoints[0]
    route.append(end_waypoint)
    return route

def draw_route(world, route):
    """
    在 CARLA 世界中可视化路径
    """
    for waypoint in route:
        world.debug.draw_point(
            waypoint.transform.location + carla.Location(z=1),  # 提高高度以更容易观察
            size=0.2,
            color=carla.Color(0, 255, 0),
            life_time=5.0
        )

def move_npc_along_route(npc_vehicle, route, world):
    """
    控制 NPC 车辆沿着路径移动
    """
    for waypoint in route:
        # 获取导航点的目标位置
        target_location = waypoint.transform.location
        npc_vehicle.set_transform(waypoint.transform)  # 直接将车辆移动到目标位置（简单实现）

        # 可视化当前导航点
        world.debug.draw_string(
            target_location,
            "X",
            draw_shadow=False,
            color=carla.Color(255, 0, 0),
            life_time=1.0,
            persistent_lines=True
        )

        # 停顿一段时间以模拟移动过程
        time.sleep(0.5)

def main():
    try:
        # 连接到 CARLA 服务器
        client = carla.Client('127.0.0.1', 2000)
        client.set_timeout(10.0)
        world = client.get_world()

        # 获取所有 spawn points
        spawn_points = world.get_map().get_spawn_points()
        if len(spawn_points) < 2:
            print("Not enough spawn points to perform navigation.")
            return

        # 使用车辆 ID 查找已生成的车辆
        target_vehicle_id = os.environ["NPC_ROLE_NAME"] # 替换为目标车辆的真实 ID
        npc_vehicle = BridgeHelpers.get_agent_actor(world, target_vehicle_id)

        # 计算 NPC 从 spawn_point 0 到 spawn_point 1 的导航路径
        route = calculate_route(world, spawn_points[0].location, spawn_points[1].location)

        # 可视化路径
        draw_route(world, route)

        # 控制 NPC 车辆沿着路径移动
        move_npc_along_route(npc_vehicle, route, world)

        print("Navigation completed! Vehicle not destroyed.")

    except Exception as e:
        print(f"An error occurred: {e}")

if __name__ == '__main__':
    main()