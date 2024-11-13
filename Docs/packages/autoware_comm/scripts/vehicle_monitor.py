import carla
import time
import math
import logging

# 设置日志文件
logging.basicConfig(filename='../log/ego_vehicle_log.log', level=logging.INFO, format='%(asctime)s - %(message)s')


def get_vehicle_by_id(world, vehicle_id):
    """
    根据车辆ID查找车辆
    """
    actors = world.get_actors().filter('vehicle.*')
    for vehicle in actors:
        if vehicle.attributes["role_name"] == vehicle_id:
            return vehicle
    return None

def log_vehicle_status(vehicle):
    """
    记录车辆的当前状态到日志文件
    """
    if vehicle is None:
        logging.error("Vehicle is None. Cannot log status.")
        return

    # 获取车辆速度（m/s），并转换为 km/h
    velocity = vehicle.get_velocity()
    speed = 3.6 * math.sqrt(velocity.x**2 + velocity.y**2 + velocity.z**2)  # 转换为 km/h

    # 获取车辆的控制状态
    control = vehicle.get_control()
    throttle = control.throttle
    brake = control.brake
    steer = control.steer
    gear = control.gear

    # 记录车辆状态到日志文件
    logging.info(f"Speed: {speed:.2f} km/h, Throttle: {throttle:.2f}, Brake: {brake:.2f}, Steer: {steer:.2f}, Gear: {gear}")

def main(vehicle_id):
    client = carla.Client("localhost", 2000)  # 连接到CARLA服务器
    client.set_timeout(10.0)
    world = client.get_world()

    # 查找目标车辆
    vehicle = get_vehicle_by_id(world, vehicle_id)

    if vehicle is None:
        print(f"Vehicle with ID {vehicle_id} not found.")
        return

    print(f"Found vehicle with ID {vehicle_id}. Logging status every 1 second...")

    try:
        while True:
            # 每隔1秒记录一次车辆状态
            log_vehicle_status(vehicle)
            time.sleep(1)

    except KeyboardInterrupt:
        print("Logging stopped by user.")


if __name__ == "__main__":
    # 在此处设置目标车辆ID
    TARGET_VEHICLE_ID = "pygame_adtruck"  # 替换为实际的车辆ID
    main(TARGET_VEHICLE_ID)