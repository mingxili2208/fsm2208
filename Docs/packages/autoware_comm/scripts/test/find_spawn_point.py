import carla

# 连接到CARLA服务器
client = carla.Client('localhost', 2000)
client.set_timeout(10.0)

# 获取当前世界（地图）
world = client.get_world()

# 获取地图的生成点
spawn_points = world.get_map().get_spawn_points()

# 检查生成点数量
if len(spawn_points) == 0:
    print("没有可用的生成点！")
else:
    print(f"找到 {len(spawn_points)} 个生成点！")

# 打印第一个生成点的位置
first_spawn_point = spawn_points[0]
print(f"第一个生成点的位置: {first_spawn_point.location}")