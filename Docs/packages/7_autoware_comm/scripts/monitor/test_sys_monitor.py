#!/usr/bin/env python3

import os
import time
import psutil
import signal

# 系统内存使用的阈值，超过这个比例（60%）将触发杀死内存最多进程的操作
MEMORY_THRESHOLD = 0.9  # 3/5

# 系统监控间隔时间（秒）
INTERVAL = 10  # 每10秒检查一次

def get_memory_usage():
    """
    获取系统的整体内存使用比例。
    :return: 内存使用比例（0到1之间）。
    """
    memory_info = psutil.virtual_memory()
    return memory_info.percent / 100.0

def get_top_memory_process():
    """
    获取内存占用最多的进程。
    :return: 内存占用最多的进程 (psutil.Process 对象)。
    """
    processes = []
    for proc in psutil.process_iter(['pid', 'name', 'memory_info']):
        try:
            # 获取进程的内存信息
            memory_usage = proc.info['memory_info'].rss  # rss = Resident Set Size
            processes.append((proc, memory_usage))
        except (psutil.NoSuchProcess, psutil.AccessDenied, psutil.ZombieProcess):
            pass

    # 根据内存使用量对进程进行排序
    processes.sort(key=lambda p: p[1], reverse=True)
    
    if processes:
        return processes[0][0]  # 返回内存占用最多的进程
    return None

def kill_process(proc):
    """
    杀死给定的进程。
    :param proc: psutil.Process 对象
    """
    try:
        os.kill(proc.pid, signal.SIGKILL)
        print(f"Killed process {proc.name()} (PID: {proc.pid}) that was using too much memory.")
    except (psutil.NoSuchProcess, psutil.AccessDenied, psutil.ZombieProcess):
        print(f"Failed to kill process {proc.name()} (PID: {proc.pid})")

def monitor_system():
    """
    监控系统内存使用，并在内存占用超过阈值时杀死占用内存最多的进程。
    """
    while True:
        # 获取当前内存使用比例
        memory_usage = get_memory_usage()
        print(f"[INFO] Current memory usage: {memory_usage * 100:.2f}%")

        # 如果内存使用超过了阈值
        if memory_usage > MEMORY_THRESHOLD:
            print("[WARNING] Memory usage exceeded threshold!")

            # 获取内存占用最多的进程
            top_process = get_top_memory_process()

            if top_process:
                print(f"[INFO] Top memory consuming process: {top_process.name()} (PID: {top_process.pid})")
                # 杀死内存占用最多的进程
                kill_process(top_process)
            else:
                print("[ERROR] No process found to kill.")
        
        # 等待指定时间，然后继续下一轮监控
        time.sleep(INTERVAL)

if __name__ == '__main__':
    print("Starting system resource monitor...")
    try:
        monitor_system()
    except KeyboardInterrupt:
        print("Monitoring stopped.")
    except Exception as e:
        print(f"An error occurred: {e}")