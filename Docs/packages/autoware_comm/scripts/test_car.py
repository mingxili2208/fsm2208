import carla

client = carla.Client('localhost', 2000)
client.set_timeout(10.0)

world = client.get_world()

# 获取某个具体车辆的蓝图，例如厢式货车
blueprint = world.get_blueprint_library().find('vehicle.carlamotors.carlacola')

# 选择一个生成点
spawn_point = world.get_map().get_spawn_points()[0]

# 生成车辆
vehicle = world.spawn_actor(blueprint, spawn_point)

# 获取车辆的 Bounding Box
bounding_box = vehicle.bounding_box

# 输出车辆的宽度（Bounding Box 的 x 轴维度）
print(f"Vehicle width: {bounding_box.extent.x * 2} meters")