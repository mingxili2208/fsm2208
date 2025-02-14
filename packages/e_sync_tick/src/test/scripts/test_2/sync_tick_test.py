import carla
import time

# 连接到 CARLA
client = carla.Client("localhost", 2000)
client.set_timeout(10.0)

# 获取世界
world = client.get_world()

# 设置同步模式
settings = world.get_settings()
settings.synchronous_mode = True  # 开启同步模式
settings.fixed_delta_seconds = 1.0 / 30.0  # 每秒 30 帧
world.apply_settings(settings)

print("Measuring simulation speed...")

try:
    start_sim_time = world.get_snapshot().timestamp.elapsed_seconds
    start_wall_time = time.time()

    for _ in range(300):  # 运行 300 帧
        world.tick()  # 每次推进一帧
        time.sleep(1.0 / 30.0)  # 每秒调用 30 次 tick

    end_sim_time = world.get_snapshot().timestamp.elapsed_seconds
    end_wall_time = time.time()

    # 计算时间流速
    simulated_time = end_sim_time - start_sim_time
    wall_time = end_wall_time - start_wall_time
    speed_ratio = simulated_time / wall_time

    print(f"Simulated Time: {simulated_time:.2f} s")
    print(f"Wall Time: {wall_time:.2f} s")
    print(f"Speed Ratio: {speed_ratio:.2f}x")

except KeyboardInterrupt:
    print("Stopped.")

finally:
    # 关闭同步模式
    settings.synchronous_mode = False
    world.apply_settings(settings)