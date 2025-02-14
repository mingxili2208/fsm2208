import carla
import time

# 连接到模拟器
client = carla.Client('localhost', 2000)
client.set_timeout(10.0)

# 获取世界
world = client.get_world()

# 初始化计数
frame_count = 0
last_frame = None
start_time = time.time()

print("Measuring FPS...")

try:
    while True:
        # 获取当前快照
        snapshot = world.get_snapshot()

        # 确保是新帧
        if last_frame is None or snapshot.frame != last_frame:
            last_frame = snapshot.frame  # 更新帧编号
            frame_count += 1            # 增加计数

        # 每秒打印一次 FPS
        elapsed_time = time.time() - start_time
        if elapsed_time >= 1.0:
            fps = frame_count / elapsed_time  # 计算 FPS
            print(f"FPS: {fps:.2f}")
            frame_count = 0                  # 重置帧计数
            start_time = time.time()         # 重置计时器

except KeyboardInterrupt:
    print("Measurement stopped.")