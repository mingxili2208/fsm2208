import carla
import time

# 连接到 CARLA
client = carla.Client('localhost', 2000)
client.set_timeout(10.0)

# 获取世界
world = client.get_world()

# 开始测量
start_wall_time = time.time()
start_sim_time = world.get_snapshot().timestamp.elapsed_seconds

# 运行一段时间
time.sleep(5)  # 等待 5 秒

# 结束测量
end_wall_time = time.time()
end_sim_time = world.get_snapshot().timestamp.elapsed_seconds

# 计算比率
simulated_time = end_sim_time - start_sim_time
wall_time = end_wall_time - start_wall_time
speed_ratio = simulated_time / wall_time

print(f"Simulated Time: {simulated_time:.2f} s, Wall Time: {wall_time:.2f} s, Speed Ratio: {speed_ratio:.2f}x")