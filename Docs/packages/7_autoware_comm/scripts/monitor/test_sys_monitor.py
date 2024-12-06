#!/usr/bin/env python3

import os
import time
import psutil
import signal

# 系统内存使用的阈值，超过这个比例（98%）将触发杀死内存最多进程的操作
MEMORY_THRESHOLD = 0.98  

# 打印Top 3进程的内存使用的阈值
PRINT_TOP_PROCESSES_THRESHOLD = 0.70  

# 系统监控间隔时间（秒）
INTERVAL = 10  # 每10秒检查一次

def get_memory_usage():
    """
    获取系统的整体内存使用比例。
    :return: 内存使用比例（0到1之间）。
    """
    memory_info = psutil.virtual_memory()
    return memory_info.percent / 100.0

def get_top_memory_processes(top_n=3):
    """
    获取内存占用最多的前 N 个进程。
    :param top_n: 返回的进程数量，默认为 3。
    :return: 内存占用最多的前 N 个进程列表 [(psutil.Process, memory_usage)]。
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
    
    return processes[:top_n]  # 返回前 N 个进程

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

        # 如果内存使用超过打印Top进程的阈值
        if memory_usage > PRINT_TOP_PROCESSES_THRESHOLD:
            print("[INFO] Memory usage exceeded 70%. Top memory-consuming processes:")
            top_processes = get_top_memory_processes(3)
            for rank, (proc, mem_usage) in enumerate(top_processes, start=1):
                print(f"  {rank}. {proc.name()} (PID: {proc.pid}) - {mem_usage / (1024 * 1024):.2f} MB")

        # 如果内存使用超过了杀死进程的阈值
        if memory_usage > MEMORY_THRESHOLD:
            print("[WARNING] Memory usage exceeded threshold!")

            # 获取内存占用最多的进程
            top_process = get_top_memory_processes(1)[0][0] if get_top_memory_processes(1) else None

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