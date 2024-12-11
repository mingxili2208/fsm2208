#!/usr/bin/env python3

import os
import time
import psutil
import signal
import pynvml

# 阈值设置
MEMORY_WARNING_THRESHOLD = 0.70  # 内存超过 70% 时打印 TOP3
MEMORY_CRITICAL_THRESHOLD = 0.90  # 内存超过 90% 时杀死进程
GPU_WARNING_THRESHOLD = 0.70  # GPU 显存超过 70% 时打印 TOP3
GPU_CRITICAL_THRESHOLD = 0.90  # GPU 显存超过 90% 时杀死进程
CPU_WARNING_THRESHOLD = 0.70  # CPU 超过 70% 时打印 TOP3
CPU_CRITICAL_THRESHOLD = 0.90  # CPU 超过 90% 时杀死进程
INTERVAL = 10  # 检查时间间隔（秒）

# 初始化 NVIDIA 管理库
def initialize_nvml():
    try:
        pynvml.nvmlInit()
        print("[INFO] NVIDIA Management Library (NVML) initialized.")
    except pynvml.NVMLError as e:
        print(f"[ERROR] Could not initialize NVML: {e}")

# 关闭 NVIDIA 管理库
def shutdown_nvml():
    try:
        pynvml.nvmlShutdown()
        print("[INFO] NVML shutdown completed.")
    except pynvml.NVMLError as e:
        print(f"[ERROR] Could not shutdown NVML: {e}")

# 获取系统内存使用率
def get_memory_usage():
    memory_info = psutil.virtual_memory()
    return memory_info.percent / 100.0

# 获取系统 CPU 使用率
def get_cpu_usage():
    return psutil.cpu_percent(interval=1) / 100.0

# 获取 GPU 显存使用率
def get_gpu_memory_usage():
    gpu_memory_usage = []
    try:
        device_count = pynvml.nvmlDeviceGetCount()
        for i in range(device_count):
            handle = pynvml.nvmlDeviceGetHandleByIndex(i)
            memory_info = pynvml.nvmlDeviceGetMemoryInfo(handle)
            usage = memory_info.used / memory_info.total
            gpu_memory_usage.append((i, usage))  # (GPU编号, 使用率)
        return gpu_memory_usage
    except pynvml.NVMLError as e:
        print(f"[ERROR] NVML error while fetching GPU memory info: {e}")
        return []

# 获取占用内存最多的进程
def get_top_memory_processes(top_n=3):
    processes = []
    for proc in psutil.process_iter(['pid', 'name', 'memory_info']):
        try:
            memory_usage = proc.info['memory_info'].rss
            processes.append((proc, memory_usage))
        except (psutil.NoSuchProcess, psutil.AccessDenied, psutil.ZombieProcess):
            pass
    processes.sort(key=lambda p: p[1], reverse=True)
    return processes[:top_n]

# 获取占用 CPU 最多的进程
def get_top_cpu_processes(top_n=3):
    processes = []
    for proc in psutil.process_iter(['pid', 'name', 'cpu_percent']):
        try:
            cpu_usage = proc.info['cpu_percent']
            processes.append((proc, cpu_usage))
        except (psutil.NoSuchProcess, psutil.AccessDenied, psutil.ZombieProcess):
            pass
    processes.sort(key=lambda p: p[1], reverse=True)
    return processes[:top_n]

# 杀死进程
def kill_process(proc):
    try:
        os.kill(proc.pid, signal.SIGKILL)
        print(f"[KILLED] Process {proc.name()} (PID: {proc.pid}) due to high resource usage.")
    except (psutil.NoSuchProcess, psutil.AccessDenied, psutil.ZombieProcess):
        print(f"[ERROR] Failed to kill process {proc.name()} (PID: {proc.pid}).")

# 打印占用内存最多的进程
def print_top_memory_processes(top_n=3):
    top_processes = get_top_memory_processes(top_n)
    print(f"[INFO] Top {top_n} memory-consuming processes:")
    for rank, (proc, mem_usage) in enumerate(top_processes, start=1):
        print(f"  {rank}. {proc.name()} (PID: {proc.pid}) - {mem_usage / (1024 * 1024):.2f} MB")

# 打印占用 CPU 最多的进程
def print_top_cpu_processes(top_n=3):
    top_processes = get_top_cpu_processes(top_n)
    print(f"[INFO] Top {top_n} CPU-consuming processes:")
    for rank, (proc, cpu_usage) in enumerate(top_processes, start=1):
        print(f"  {rank}. {proc.name()} (PID: {proc.pid}) - {cpu_usage:.2f}% CPU")

# 打印 GPU 显存使用情况
def print_gpu_usage(gpu_memory_usage):
    for gpu_id, usage in gpu_memory_usage:
        print(f"[INFO]                GPU {gpu_id}: {usage * 100:.2f}%")

# 主监控逻辑
def monitor_system():
    while True:
        print("------------------------------------------------------------")
        # 内存监控
        memory_usage = get_memory_usage()
        print(f"[INFO] Current memory usage: {memory_usage * 100:.2f}%")
        if memory_usage > MEMORY_WARNING_THRESHOLD:
            print("[WARNING] Memory usage exceeded 70%.")
            print_top_memory_processes(3)
        if memory_usage > MEMORY_CRITICAL_THRESHOLD:
            print("[CRITICAL] Memory usage exceeded 90%. Killing top memory-consuming process.")
            top_process = get_top_memory_processes(1)[0][0] if get_top_memory_processes(1) else None
            if top_process:
                kill_process(top_process)

        # GPU 显存监控
        gpu_memory_usage = get_gpu_memory_usage()
        print(f"[INFO] Current GPU    usage:")
        print_gpu_usage(gpu_memory_usage)
        for gpu_id, usage in gpu_memory_usage:
            if usage > GPU_WARNING_THRESHOLD:
                print(f"[WARNING] GPU {gpu_id} memory usage exceeded 70%.")
            if usage > GPU_CRITICAL_THRESHOLD:
                print(f"[CRITICAL] GPU {gpu_id} memory usage exceeded 90%. Killing top GPU-consuming process.")
                # GPU 进程监控和杀死逻辑可以扩展，但这里需要专门的工具或 API 支持。

        # CPU 使用率监控
        cpu_usage = get_cpu_usage()
        print(f"[INFO] Current CPU    usage: {cpu_usage * 100:.2f}%")
        if cpu_usage > CPU_WARNING_THRESHOLD:
            print("[WARNING] CPU usage exceeded 70%.")
            print_top_cpu_processes(3)
        if cpu_usage > CPU_CRITICAL_THRESHOLD:
            print("[CRITICAL] CPU usage exceeded 90%. Killing top CPU-consuming process.")
            top_process = get_top_cpu_processes(1)[0][0] if get_top_cpu_processes(1) else None
            if top_process:
                kill_process(top_process)

        # 等待指定的时间间隔
        time.sleep(INTERVAL)

if __name__ == '__main__':
    print("Starting system resource monitor...")
    try:
        initialize_nvml()  # 初始化 NVIDIA 管理库
        monitor_system()   # 启动监控
    except KeyboardInterrupt:
        print("Monitoring stopped by user.")
    except Exception as e:
        print(f"An error occurred: {e}")
    finally:
        shutdown_nvml()  # 关闭 NVIDIA 管理库