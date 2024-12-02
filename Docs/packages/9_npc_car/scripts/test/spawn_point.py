#!/usr/bin/env python

import glob
import os
import sys
import time
import carla

def main():
    # 尝试添加 CARLA egg 文件路径到 sys.path
    try:
        # 根据您的 CARLA 版本和操作系统选择正确的 egg 文件
        # 例如，CARLA 0.9.15 在 Linux x86_64 下:
        egg_file = '../carla/PythonAPI/carla/dist/carla-0.9.15-py3.10-linux-x86_64.egg'
        if not os.path.exists(egg_file):
            raise FileNotFoundError(f"CARLA egg file not found at {egg_file}. Please check the path.")
        sys.path.append(egg_file)
    except Exception as e:
        print(f"Error adding CARLA egg file to sys.path: {e}")
        sys.exit(1)

    try:
        # 连接到 CARLA 服务器
        client = carla.Client('127.0.0.1', 2000)
        client.set_timeout(10.0)  # 设置超时

        # 获取当前世界
        world = client.get_world()

        # 获取地图中的所有 spawn points
        spawn_points = world.get_map().get_spawn_points()

        print(f"Found {len(spawn_points)} spawn points.")

        # 在每个 spawn point 位置绘制一个红色的球体和编号，以及方向线
        for i, spawn_point in enumerate(spawn_points):
            location = spawn_point.location
            rotation = spawn_point.rotation

            # 选择颜色
            if i % 2 == 0:
                color = carla.Color(r=255, g=0, b=0)  # 红色
            else:
                color = carla.Color(r=0, g=255, b=0)  # 绿色

            # 绘制球体
            world.debug.draw_point(
                location,
                size=0.5,
                color=color,
                life_time=0.0,
                persistent_lines=True
            )

            # 计算方向线的终点
            direction = rotation.get_forward_vector()
            end_location = location + direction * 2  # 2 米长度

            # 绘制方向线
            world.debug.draw_line(
                location,
                end_location,
                thickness=0.1,
                color=carla.Color(r=0, g=255, b=255),  # 青色
                life_time=0.0,
                persistent_lines=True
            )

            # 绘制文字标签
            world.debug.draw_string(
                location + carla.Location(z=1.0),
                f"SP{i}",
                draw_shadow=False,
                color=carla.Color(r=255, g=255, b=255),  # 白色文字
                life_time=0.0,
                persistent_lines=True
            )

        print("Spawn points have been marked. Press Ctrl+C to exit.")

        # 保持脚本运行，以便观察标记
        try:
            while True:
                time.sleep(1)
        except KeyboardInterrupt:
            print("\nExiting...")

    except Exception as e:
        print(f"An error occurred: {e}")
        sys.exit(1)

if __name__ == '__main__':
    main()