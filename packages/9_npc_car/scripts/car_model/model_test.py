import carla  # 确保已安装 CARLA 的 Python API
import random
import signal
import sys
import time

# 手动指定车辆 ID
VEHICLE_ID = "vehicle.audi.etron"  # 修改为你想生成的车辆 ID

# 全局变量，用于保存生成的车辆实例
current_vehicle = None
RUN_TIME = 30  # 程序运行时间限制（单位：秒）

def signal_handler(sig, frame):
    """信号处理器：捕获 Ctrl+C (SIGINT) 信号并删除车辆"""
    global current_vehicle
    if current_vehicle:
        print("\nCtrl+C detected! Removing the spawned vehicle...")
        current_vehicle.destroy()
        print(f"Vehicle '{current_vehicle.id}' has been removed.")
    else:
        print("\nCtrl+C detected! No vehicle to remove.")
    # 正常退出程序
    sys.exit(0)

def main():
    global current_vehicle

    # 连接到 CARLA 服务器
    client = carla.Client('localhost', 2000)  # 根据实际情况调整主机地址和端口
    client.set_timeout(10.0)  # 设置超时时间

    try:
        # 注册信号处理器，捕获 Ctrl+C
        signal.signal(signal.SIGINT, signal_handler)

        # 获取世界和蓝图库
        world = client.get_world()
        blueprint_library = world.get_blueprint_library()

        # 检查车辆 ID 是否存在
        blueprint = blueprint_library.find(VEHICLE_ID)
        if not blueprint:
            print(f"Error: Vehicle ID '{VEHICLE_ID}' not found.")
            return

        # 获取地图中的所有 SpawnPoints
        spawn_points = world.get_map().get_spawn_points()
        if not spawn_points:
            raise RuntimeError("No spawn points available on the map.")

        # 随机选择一个空闲的 SpawnPoint
        spawn_point = random.choice(spawn_points)

        # 在指定的 SpawnPoint 生成车辆
        current_vehicle = world.try_spawn_actor(blueprint, spawn_point)
        if current_vehicle:
            print(f"Vehicle '{VEHICLE_ID}' successfully spawned at location: {spawn_point.location}")
            bounding_box = current_vehicle.bounding_box
            dimensions = bounding_box.extent
        else:
            dimensions = "Spawn failed (collision or occupied spawn point)"
            print(f"Failed to spawn vehicle '{VEHICLE_ID}' (collision or occupied spawn point).")
            return
        dimensions_str = (
            f"L:{dimensions.x*2:.2f}m, "
            f"W:{dimensions.y*2:.2f}m, "
            f"H:{dimensions.z*2:.2f}m"
        ) if dimensions and not isinstance(dimensions, str) else dimensions
        print(dimensions_str)
        # 在主线程中保持运行，直到达到时间限制或用户按下 Ctrl+C
        print("\nPress Ctrl+C to stop the script and remove the vehicle...")
        start_time = time.time()  # 记录程序开始运行的时间
        while True:
            elapsed_time = time.time() - start_time
            if elapsed_time >= RUN_TIME:
                print(f"Time limit of {RUN_TIME} seconds reached. Exiting...")
                break  # 退出循环，结束程序
            time.sleep(1)  # 每秒检查一次时间

    except Exception as e:
        print(f"Error: {e}")

    finally:
        # 如果未通过 Ctrl+C 退出，但需要清理车辆
        if current_vehicle:
            print("Script is exiting. Removing the spawned vehicle...")
            current_vehicle.destroy()
            print(f"Vehicle '{current_vehicle.id}' has been removed.")

if __name__ == "__main__":
    main()