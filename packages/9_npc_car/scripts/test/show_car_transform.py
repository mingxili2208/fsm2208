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

def main():
    try:
        # 连接到 CARLA 服务器
        client = carla.Client('127.0.0.1', 2000)
        client.set_timeout(10.0)
        world = client.get_world()

        # 获取所有 spawn points
        target_vehicle_id = os.environ["NPC_ROLE_NAME"] # 替换为目标车辆的真实 ID
        npc_vehicle = BridgeHelpers.get_agent_actor(world, target_vehicle_id)
       
        
        # 计算 NPC 从 spawn_point 0 到 spawn_point 1 的导航路径
        while True:
            print( npc_vehicle.get_transform())
            time.sleep(0.02)

    except Exception as e:
        print(f"An error occurred: {e}")

if __name__ == '__main__':
    main()