import carla
import time

# 连接到模拟器
client = carla.Client('localhost', 2000)
client.set_timeout(10.0)

# 获取世界
world = client.get_world()

# 初始化变量
last_sim_time = None
start_wall_time = time.time()

print("Measuring simulation speed...")

try:
    for _ in range(100):  # 测试 100 帧
        # 获取当前快照
        snapshot = world.get_snapshot()
        sim_time = snapshot.timestamp.elapsed_seconds  # 当前模拟时间（秒）

        if last_sim_time is not None:
            # 计算模拟时间和实际时间的差
            sim_delta = sim_time - last_sim_time
            wall_delta = time.time() - start_wall_time
            speed_ratio = sim_delta / wall_delta  # 模拟速度与实际时间的比率

            print(f"Simulated Time: {sim_delta:.4f} s, Wall Time: {wall_delta:.4f} s, Speed Ratio: {speed_ratio:.2f}x")

        # 更新变量
        last_sim_time = sim_time
        start_wall_time = time.time()

        # 等待一段时间（模拟异步模式下的行为）
        time.sleep(0.1)

except KeyboardInterrupt:
    print("Measurement stopped.")