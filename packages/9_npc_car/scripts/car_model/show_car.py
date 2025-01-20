import carla  # 确保已安装 CARLA 的 Python API
import os
import random

# 日志文件路径
log_file_path = "carla_vehicle_models.log"

def save_to_log(log_file, message):
    """保存信息到日志文件"""
    with open(log_file, 'a') as log:
        log.write(message + '\n')

def main():
    # 连接到 CARLA 服务器
    client = carla.Client('localhost', 2000)  # 根据实际情况调整主机地址和端口
    client.set_timeout(10.0)  # 设置超时时间

    try:
        # 获取世界和蓝图库
        world = client.get_world()
        blueprint_library = world.get_blueprint_library()

        # 获取所有车辆模型
        vehicle_blueprints = blueprint_library.filter("vehicle.*")

        # 获取地图中的所有 SpawnPoints
        spawn_points = world.get_map().get_spawn_points()
        if not spawn_points:
            raise RuntimeError("No spawn points available on the map.")

        # 初始化日志文件
        if os.path.exists(log_file_path):
            os.remove(log_file_path)  # 删除旧的日志文件
        save_to_log(log_file_path, "CARLA Vehicle Models Log")
        save_to_log(log_file_path, "===================================")

        # 遍历车辆模型
        for blueprint in vehicle_blueprints:
            vehicle_id = blueprint.id  # 车辆模型 ID

            # 获取蓝图的路径（注意：0.9.15 版本直接使用 `blueprint.id` 提供路径）
            asset_path = blueprint.id  # 因为没有 `get_tag`，直接使用 `id` 提供的完整路径

            # 选择一个随机的空闲 SpawnPoint
            spawn_point = random.choice(spawn_points)

            # 测试生成一次车辆，获取尺寸参数
            dimensions = None
            temporary_vehicle = None
            try:
                # 在随机的 SpawnPoint 生成车辆
                temporary_vehicle = world.try_spawn_actor(blueprint, spawn_point)
                if temporary_vehicle:
                    # 获取车辆的尺寸
                    bounding_box = temporary_vehicle.bounding_box
                    dimensions = bounding_box.extent
                else:
                    # 如果车辆无法生成（例如碰撞），记录无法生成的日志
                    dimensions = "Spawn failed (collision or occupied spawn point)"
            except Exception as e:
                dimensions = f"Error calculating dimensions: {e}"

            # 格式化尺寸参数
            dimensions_str = (
                f"L:{dimensions.x*2:.2f}m, "
                f"W:{dimensions.y*2:.2f}m, "
                f"H:{dimensions.z*2:.2f}m"
            ) if dimensions and not isinstance(dimensions, str) else dimensions

            # 写入日志
            log_message = f"ID: {vehicle_id} | Path: {asset_path} | Dimensions: {dimensions_str}"
            print(log_message)  # 打印到控制台
            save_to_log(log_file_path, log_message)

            # 删除生成的车辆
            if temporary_vehicle:
                temporary_vehicle.destroy()

        print(f"\nVehicle data has been logged to {log_file_path}")

    except Exception as e:
        print(f"Error: {e}")

if __name__ == "__main__":
    main()