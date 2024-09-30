import carla

client = carla.Client('localhost', 2000)
world = client.get_world()
blueprint_library = world.get_blueprint_library()

widest_vehicle = None
max_width = 0

for blueprint in blueprint_library.filter('vehicle.*'):  # 筛选所有车辆蓝图
    bounding_box = blueprint.get_bounding_box()
    width = bounding_box.extent.y * 2  # extent.y 是一半的宽度
    if width > max_width:
        max_width = width
        widest_vehicle = blueprint

if widest_vehicle:
    print(f"最宽的车辆是: {widest_vehicle.id}, 宽度: {max_width}")
else:
    print("未找到车辆。")