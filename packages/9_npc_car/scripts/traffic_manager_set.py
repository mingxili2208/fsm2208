#!/usr/bin/env python3

import carla
import time
def print_detailed_physics_settings(world):


    settings = world.get_settings()
    print("\n=== 详细物理引擎设置 ===")
    print(f"子步模拟: {settings.substepping}")
    print(f"最大子步时间: {settings.max_substep_delta_time}")
    print(f"最大子步数: {settings.max_substeps if hasattr(settings, 'max_substeps') else '未知'}")
    print(f"同步模式: {settings.synchronous_mode}")
    print(f"固定时间步长: {settings.fixed_delta_seconds}")
    print(settings)
    # 打印更多物理相关设置
    if hasattr(settings, 'no_rendering_mode'):
        print(f"无渲染模式: {settings.no_rendering_mode}")
    if hasattr(settings, 'deterministic_ragdolls'):
        print(f"deterministic_ragdolls: {settings.deterministic_ragdolls}")
    print("===========================\n")
def main():
    # 连接到 Carla 服务器
    client = carla.Client('localhost', 2000)
    client.set_timeout(10.0)
    
    # 获取世界实例
    world = client.get_world()
    
    # 获取并设置 Traffic Manager
    traffic_manager = client.get_trafficmanager(8000)  # 确保端口匹配你的设置
    traffic_manager.set_synchronous_mode(True)
    
    # # 输出确认信息
    # print("Traffic Manager 同步模式已设置为: True")
    
    # # 在关闭子步模拟前后调用此函数
    # print_detailed_physics_settings(world)
    # # 尝试关闭子步模拟
    # settings = world.get_settings()
    # settings.substepping = False
    # world.apply_settings(settings)
    # # 再次打印确认设置
    print_detailed_physics_settings(world)
    # 可选：关闭子步模拟
    # settings = world.get_settings()
    # if settings.substepping:
    #     settings.substepping = False
    #     world.apply_settings(settings)
    #     print("子步模拟已关闭")
    
    # 保持脚本运行，以防 Carla 重新配置
    try:
        print("脚本运行中，按 Ctrl+C 终止...")
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("脚本已终止")

if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print(f"发生错误: {e}")