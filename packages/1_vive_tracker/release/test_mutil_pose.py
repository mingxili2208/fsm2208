import sys
import time
import numpy as np
from pynput import keyboard  # 需要安装 pynput: pip install pynput
from vive_tracker import ViveTrackerModule

# 设定追踪器的名称
TRACKER_NAME = "tracker_1"
LOG_FILE = "tracker_log_eular.txt"  # 日志文件名称

# 初始化 ViveTrackerModule
vtm = ViveTrackerModule()
vtm.print_discovered_objects()

# 获取指定名称的追踪器对象
tracker = vtm.devices.get(TRACKER_NAME)

if tracker is None:
    print(f"Error: Tracker '{TRACKER_NAME}' not found!")
    sys.exit(1)

# 用于存储坐标数据
recorded_positions = []
counter = 1  # 记录坐标的编号
running = True  # 控制循环

def on_press(key):
    global counter

    try:
        if key.char == 'r':  # 按下 'R' 记录坐标
            cam_coord = tracker.get_pose_euler()
            recorded_positions.append((counter, cam_coord))
            log_entry = (f"[{counter}] x={cam_coord[0]:.4f}, y={cam_coord[1]:.4f}, z={cam_coord[2]:.4f}, "
                         f"roll={cam_coord[3]:.4f}, yaw={cam_coord[4]:.4f}, pitch={cam_coord[5]:.4f}")
            print("\n" + log_entry)
            counter += 1

    except AttributeError:
        pass  # 处理特殊按键（如 Esc）

def on_release(key):
    global running
    if key == keyboard.Key.esc:  # 按下 'Esc' 退出程序
        print("\nRecorded Coordinates:")
        
        # 打开日志文件，写入所有记录的坐标
        with open(LOG_FILE, "w") as log_file:
            log_file.write("Recorded Coordinates:\n")
            for idx, coord in recorded_positions:
                log_entry = (f"[{idx}] x={coord[0]:.4f}, y={coord[1]:.4f}, z={coord[2]:.4f}, "
                             f"roll={coord[3]:.4f}, yaw={coord[4]:.4f}, pitch={coord[5]:.4f}")
                print(log_entry)   # 输出到终端
                log_file.write(log_entry + "\n")  # 写入日志文件
            print(f"\nCoordinates saved to {LOG_FILE}")

        running = False
        return False  # 停止监听

# 监听键盘输入
listener = keyboard.Listener(on_press=on_press, on_release=on_release)
listener.start()

print("Press 'R' to record coordinates, 'Esc' to exit and print all recorded coordinates to terminal and log file.")

# 主循环
while running:
    time.sleep(0.1)  # 降低 CPU 占用