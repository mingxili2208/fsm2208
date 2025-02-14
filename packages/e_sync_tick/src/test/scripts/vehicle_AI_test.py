import carla
import math
import time

def check_vehicle_managed_by_traffic_manager(vehicle, traffic_manager):
    """
    检查车辆是否被 Traffic Manager 管理：通过设置目标速度并检查是否生效。
    """
    # 确保车辆被 Traffic Manager 接管
    vehicle.set_autopilot(True, traffic_manager.get_port())

    # 等待一段时间以观察车辆行为
    time.sleep(2)

    # 设置 Traffic Manager 的目标速度为 10 km/h
    traffic_manager.set_desired_speed(vehicle, 10.0)

    # 等待一段时间以观察速度变化
    time.sleep(2)

    # 获取车辆当前速度
    velocity = vehicle.get_velocity()
    speed = math.sqrt(velocity.x**2 + velocity.y**2 + velocity.z**2) * 3.6  # 转换为 km/h

    # 检查速度是否接近 Traffic Manager 设置的目标速度
    if abs(speed - 10.0) < 1.0:  # 容差为 1 km/h
        print(f"Vehicle {vehicle.id} is managed by Traffic Manager.")
    else:
        print(f"Vehicle {vehicle.id} is NOT managed by Traffic Manager.")

def main():
    # 连接到 CARLA 客户端
    client = carla.Client("localhost", 2000)
    client.set_timeout(10.0)

    # 获取世界
    world = client.get_world()

    traffic_manager = client.get_trafficmanager(8000)
    # 获取已生成的车辆
    vehicles = world.get_actors().filter('vehicle.*')
    if not vehicles:
        print("No vehicles found in the simulation.")
        return

    # 假设选择第一辆车辆
    vehicle = vehicles[0]
    print(f"Selected vehicle: {vehicle.id}")

    # 解除 Traffic Manager 的控制
    vehicle.set_autopilot(False)
    print(f"Vehicle {vehicle.id} is no longer managed by Traffic Manager.")

    # 检查车辆是否被 Traffic Manager 管理
    check_vehicle_managed_by_traffic_manager(vehicle, traffic_manager)



if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nSimulation stopped.")


