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
        egg_file = '/home/cityu-fsm-lab-carla/Desktop/WorkplaceCarla/carla-0.9.15/PythonAPI/carla/dist/carla-0.9.15-py3.10-linux-x86_64.egg'
        if not os.path.exists(egg_file):
            raise FileNotFoundError(f"CARLA egg file not found at {egg_file}. Please check the path.")
        sys.path.append(egg_file)
    except Exception as e:
        print(f"Error adding CARLA egg file to sys.path: {e}")
        sys.exit(1)

    try:
        import carla  # 确保 CARLA 模块已正确导入

        # 连接到 CARLA 服务器
        client = carla.Client('127.0.0.1', 2000)
        client.set_timeout(10.0)  # 设置超时

        # 获取当前世界
        world = client.get_world()

        # 获取地图中的所有 spawn points
        spawn_points = world.get_map().get_spawn_points()

        print(f"Found {len(spawn_points)} spawn points.")

        # 获取 Debug 绘制器
        debug = world.debug

        # 打开日志文件以写入 spawn point 信息
        log_file_path = './spawn_points_log.log'
        with open(log_file_path, 'w') as log_file:
            log_file.write(f"Spawn Points Log:\n")
            log_file.write(f"Number of spawn points: {len(spawn_points)}\n\n")
            
            # 在每个 spawn point 位置绘制一个红色的球体和编号，以及方向线
            for i, spawn_point in enumerate(spawn_points):
                location = spawn_point.location
                rotation = spawn_point.rotation

                # 将位置和方向信息写入日志文件
                log_file.write(f"Spawn Point {i}:\n")
                log_file.write(f"  Location: x={location.x:.2f}, y={location.y:.2f}, z={location.z:.2f}\n")
                log_file.write(f"  Rotation: pitch={rotation.pitch:.2f}, yaw={rotation.yaw:.2f}, roll={rotation.roll:.2f}\n\n")

                # 选择颜色
                if i % 2 == 0:
                    color = carla.Color(r=255, g=0, b=0)  # 红色
                else:
                    color = carla.Color(r=0, g=255, b=0)  # 绿色

                # 尝试优化点的显示
                debug.draw_point(
                    location,
                    size=0.2,  # 调整尺寸，减少正方形的视觉效果
                    color=color,
                    life_time=10000.0,  # 设置点的存在时间
                    persistent_lines=True
                )

                # 绘制方向线
                direction = rotation.get_forward_vector()
                end_location = location + direction * 2  # 2 米长度
                debug.draw_line(
                    location,
                    end_location,
                    thickness=0.1,
                    color=carla.Color(r=0, g=255, b=255),  # 青色
                    life_time=10000.0,  # 设置线的存在时间
                    persistent_lines=True
                )

                # 绘制文字标签
                debug.draw_string(
                    location + carla.Location(z=0.5),  # 稍微提高 z 坐标
                    f"SP{i}",
                    draw_shadow=False,
                    color=carla.Color(r=255, g=255, b=255),  # 白色文字
                    life_time=10000.0,  # 确保文字标签不会立即消失
                    persistent_lines=True
                )

        print(f"Spawn points information has been logged to {log_file_path}.")
        print("Spawn points have been marked. Press Ctrl+C to exit.")

        # 保持脚本运行，并持续绘制标记
        try:
            while True:
                # 在每个循环中，您可以重新绘制或进行其他更新
                time.sleep(1)
        except KeyboardInterrupt:
            print("\nExiting...")

    except Exception as e:
        print(f"An error occurred: {e}")
        sys.exit(1)

if __name__ == '__main__':
    main()