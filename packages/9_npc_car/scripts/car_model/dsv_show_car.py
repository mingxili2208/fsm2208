import carla
import logging

# 设置日志文件
logging.basicConfig(
    filename='carla_vehicle_models.log',  # 日志文件名
    level=logging.INFO,                   # 日志级别
    format='%(asctime)s - %(message)s'    # 日志格式
)

def get_vehicle_models(world):
    """
    获取 Carla 中所有车辆模型的信息。
    :param world: Carla 世界对象
    :return: 车辆模型信息列表
    """
    # 获取蓝图库
    blueprint_library = world.get_blueprint_library()

    # 过滤出所有车辆蓝图
    vehicle_blueprints = blueprint_library.filter('vehicle.*')

    # 存储车辆模型信息
    vehicle_models = []

    for blueprint in vehicle_blueprints:
        # 获取车辆 ID
        vehicle_id = blueprint.id

        # 获取车辆路径（蓝图路径）
        vehicle_path = blueprint.get_attribute('path').as_str()

        # 获取车辆尺寸
        vehicle_size = blueprint.get_attribute('size').as_vector3d()

        # 将信息存储到字典中
        vehicle_info = {
            'id': vehicle_id,
            'path': vehicle_path,
            'size': vehicle_size
        }
        vehicle_models.append(vehicle_info)

    return vehicle_models

def log_vehicle_models(vehicle_models):
    """
    将车辆模型信息写入日志文件。
    :param vehicle_models: 车辆模型信息列表
    """
    for vehicle in vehicle_models:
        log_message = (
            f"Vehicle ID: {vehicle['id']}, "
            f"Path: {vehicle['path']}, "
            f"Size: (x={vehicle['size'].x}, y={vehicle['size'].y}, z={vehicle['size'].z})"
        )
        logging.info(log_message)

def main():
    try:
        # 连接到 Carla 服务器
        client = carla.Client('localhost', 2000)
        client.set_timeout(10.0)

        # 获取 Carla 世界
        world = client.get_world()

        # 获取所有车辆模型信息
        vehicle_models = get_vehicle_models(world)

        # 将车辆模型信息写入日志文件
        log_vehicle_models(vehicle_models)

        print(f"Successfully logged {len(vehicle_models)} vehicle models to 'carla_vehicle_models.log'.")

    except Exception as e:
        print(f"An error occurred: {e}")

if __name__ == '__main__':
    main()