#!/usr/bin/env python

# Copyright (c) 2025 James LI
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""
脚本功能：Carla性能监控工具
监控Carla模拟器的性能指标和系统资源使用情况
记录真实FPS、帧时间、CPU使用率、内存使用等信息
接收并记录Carla中发生的事件
"""

import glob
import os
import sys
import time
import socket
import json
import argparse
import csv
import datetime
import threading
import signal
import platform
import subprocess

# 添加Carla库路径
try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

try:
    import carla
    CARLA_AVAILABLE = True
except ImportError:
    CARLA_AVAILABLE = False
    print("警告: Carla Python API未找到，无法获取真实FPS数据")
    print("请确保正确设置了Carla的Python路径")

# 尝试导入可选依赖
try:
    import psutil
    PSUTIL_AVAILABLE = True
except ImportError:
    PSUTIL_AVAILABLE = False
    print("警告: psutil未安装，部分系统监控功能将不可用")
    print("可以通过运行'pip install psutil'安装")

try:
    import GPUtil
    GPUTIL_AVAILABLE = True
except ImportError:
    GPUTIL_AVAILABLE = False
    print("警告: GPUtil未安装，GPU监控功能将不可用")
    print("可以通过运行'pip install GPUtil'安装")

try:
    import matplotlib.pyplot as plt
    MATPLOTLIB_AVAILABLE = True
except ImportError:
    MATPLOTLIB_AVAILABLE = False
    print("警告: matplotlib未安装，实时图表功能将不可用")
    print("可以通过运行'pip install matplotlib'安装")

# 全局变量
running = True  # 控制监控线程
event_socket = None  # 事件接收套接字
performance_data = {
    "fps": [],
    "frame_times": [],
    "cpu_usage": [],
    "ram_usage": [],
    "gpu_usage": [],
    "gpu_memory": [],
    "timestamps": []
}

carla_client = None  # Carla客户端连接
carla_world = None   # Carla世界对象
last_frame_id = 0    # 上一帧ID
last_frame_time = time.time()  # 上一帧时间
frame_delta_acc = 0  # 帧差累计
frame_delta_count = 0  # 帧计数
last_reconnect_time = 0  # 上次重连时间

carla_process = None  # Carla进程信息
carla_pid = None  # Carla进程ID
events = []  # 存储收到的事件
log_file = None  # CSV日志文件
log_writer = None  # CSV写入器
event_log_file = None  # 事件日志文件
event_log_writer = None  # 事件日志写入器

# 配置
EVENT_PORT = 8700  # 事件监听端口
REFRESH_RATE = 1.0  # 性能更新频率（秒）
CARLA_PROCESS_NAME = "CarlaUE4"  # Carla进程名称
LOG_FOLDER = "performance_logs"  # 日志文件夹

def connect_to_carla(host='localhost', port=2000):
    """连接到Carla服务器以获取真实性能数据"""
    global carla_client, carla_world
    
    if not CARLA_AVAILABLE:
        print("Carla Python API不可用，无法连接到Carla服务器")
        return False
    
    try:
        carla_client = carla.Client(host, port)
        carla_client.set_timeout(2.0)
        carla_world = carla_client.get_world()
        print(f"已连接到Carla服务器: {host}:{port}")
        
        # 获取并打印基本信息
        carla_version = carla_client.get_client_version()
        server_version = carla_client.get_server_version()
        map_name = carla_world.get_map().name
        
        print(f"Carla客户端版本: {carla_version}")
        print(f"Carla服务器版本: {server_version}")
        print(f"当前地图: {map_name}")
        
        # 记录到事件日志
        if event_log_writer:
            event_log_writer.writerow([
                datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S'),
                "carla_connected",
                json.dumps({
                    "client_version": carla_version,
                    "server_version": server_version,
                    "map": map_name
                })
            ])
            event_log_file.flush()
        
        return True
    except Exception as e:
        print(f"无法连接到Carla服务器: {e}")
        return False

def get_carla_performance():
    """获取Carla的真实性能数据"""
    global carla_world, last_frame_id, frame_delta_acc, frame_delta_count, last_frame_time
    
    if not carla_world:
        return 0, 0
    
    try:
        # 获取当前快照
        snapshot = carla_world.get_snapshot()
        
        # 获取当前帧ID
        current_frame_id = snapshot.frame
        
        # 获取当前时间
        current_time = time.time()
        
        # 计算帧差
        frame_delta = current_frame_id - last_frame_id
        time_delta = current_time - last_frame_time
        
        # 防止首次调用时的大跳变
        if frame_delta > 1000 or frame_delta <= 0 or time_delta <= 0:
            last_frame_id = current_frame_id
            last_frame_time = current_time
            return 0, 0
        
        # 计算FPS
        instantaneous_fps = frame_delta / time_delta
        
        # 计算帧时间
        frame_time = 1000.0 / instantaneous_fps if instantaneous_fps > 0 else 0  # 毫秒
        
        # 平滑FPS计算
        frame_delta_acc += frame_delta
        frame_delta_count += 1
        
        if frame_delta_count > 10:
            frame_delta_acc = frame_delta
            frame_delta_count = 1
        
        # 计算平均FPS
        avg_fps = 0
        if frame_delta_count > 0 and time_delta > 0:
            avg_fps = frame_delta_acc / (frame_delta_count * time_delta)
        
        # 更新最后的帧ID和时间
        last_frame_id = current_frame_id
        last_frame_time = current_time
        
        return avg_fps, frame_time
    except Exception as e:
        print(f"获取Carla性能数据失败: {e}")
        return 0, 0

def setup_logging(args):
    """设置日志记录"""
    global log_file, log_writer, event_log_file, event_log_writer
    
    if not os.path.exists(LOG_FOLDER):
        os.makedirs(LOG_FOLDER)
    
    timestamp = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
    log_filename = os.path.join(LOG_FOLDER, f"carla_performance_{timestamp}.csv")
    event_log_filename = os.path.join(LOG_FOLDER, f"carla_events_{timestamp}.csv")
    
    try:
        # 设置性能日志
        log_file = open(log_filename, 'w', newline='')
        log_writer = csv.writer(log_file)
        
        # 写入CSV头 - 性能日志
        headers = ['Timestamp', 'FPS', 'Frame_Time_ms', 'CPU_Usage_Percent', 'RAM_Usage_MB']
        
        # 添加GPU相关字段（如果可用）
        if GPUTIL_AVAILABLE:
            headers.extend(['GPU_Usage_Percent', 'GPU_Memory_MB'])
        
        # 添加特定于Carla的字段
        headers.extend(['Carla_CPU_Percent', 'Carla_Memory_MB', 'System_CPU_Count', 'System_RAM_Total_GB'])
        
        log_writer.writerow(headers)
        
        # 设置事件日志
        event_log_file = open(event_log_filename, 'w', newline='')
        event_log_writer = csv.writer(event_log_file)
        
        # 写入CSV头 - 事件日志
        event_log_writer.writerow(['Timestamp', 'Event_Type', 'Event_Data'])
        
        print(f"性能日志将记录到: {log_filename}")
        print(f"事件日志将记录到: {event_log_filename}")
        return True
    except Exception as e:
        print(f"无法创建日志文件: {e}")
        return False

def find_carla_process():
    """查找并获取Carla进程信息"""
    global carla_process, carla_pid
    
    if not PSUTIL_AVAILABLE:
        print("无法获取Carla进程信息: psutil未安装")
        return None
    
    for proc in psutil.process_iter(['pid', 'name', 'cpu_percent', 'memory_info']):
        try:
            process_name = proc.info['name'].lower()
            if CARLA_PROCESS_NAME.lower() in process_name:
                carla_process = proc
                carla_pid = proc.pid
                print(f"找到Carla进程: PID={proc.pid}")
                return proc
        except (psutil.NoSuchProcess, psutil.AccessDenied, psutil.ZombieProcess):
            pass
    
    print("未找到运行中的Carla进程")
    return None

def get_system_info():
    """获取系统信息"""
    system_info = {
        "os": platform.system(),
        "os_version": platform.version(),
        "cpu_model": platform.processor(),
        "cpu_count": os.cpu_count(),
        "python_version": platform.python_version()
    }
    
    if PSUTIL_AVAILABLE:
        memory = psutil.virtual_memory()
        system_info["total_memory_gb"] = memory.total / (1024**3)
        system_info["available_memory_gb"] = memory.available / (1024**3)
    
    if GPUTIL_AVAILABLE:
        try:
            gpus = GPUtil.getGPUs()
            if gpus:
                system_info["gpu_model"] = gpus[0].name
                system_info["gpu_memory_gb"] = gpus[0].memoryTotal / 1024
        except:
            pass
    
    return system_info

def log_system_info():
    """记录系统信息到日志"""
    if log_writer:
        system_info = get_system_info()
        log_writer.writerow(['INFO', 'System Information', json.dumps(system_info)])
        log_file.flush()

def start_event_listener():
    """启动事件监听线程"""
    global event_socket
    
    def event_listener():
        global running, events
        
        try:
            # 创建UDP套接字
            event_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            event_socket.bind(('0.0.0.0', EVENT_PORT))
            event_socket.settimeout(1.0)  # 设置超时，以便能够定期检查running标志
            
            print(f"事件监听器已启动，端口: {EVENT_PORT}")
            
            # 接收事件循环
            while running:
                try:
                    data, addr = event_socket.recvfrom(4096)
                    msg = json.loads(data.decode())
                    
                    # 记录事件
                    timestamp = datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S')
                    events.append({
                        "timestamp": timestamp,
                        "type": msg.get("type", "unknown"),
                        "data": msg.get("data", {}),
                        "source": addr[0]
                    })
                    
                    # 记录到事件日志
                    if event_log_writer:
                        event_log_writer.writerow([
                            timestamp, 
                            msg.get("type", "unknown"), 
                            json.dumps(msg.get("data", {}))
                        ])
                        event_log_file.flush()
                    
                    # 打印事件信息
                    print(f"事件: {msg.get('type', 'unknown')} 来自 {addr[0]}")
                except socket.timeout:
                    # 正常超时，继续循环
                    continue
                except json.JSONDecodeError:
                    print(f"收到无效的JSON数据")
                except Exception as e:
                    print(f"事件监听器错误: {e}")
        except Exception as e:
            print(f"无法启动事件监听器: {e}")
        finally:
            if event_socket:
                event_socket.close()
    
    # 启动监听线程
    listener_thread = threading.Thread(target=event_listener, daemon=True)
    listener_thread.start()
    return listener_thread

def monitor_carla_performance():
    """监控Carla性能的主循环"""
    global running, performance_data, carla_process, carla_client, carla_world, last_reconnect_time
    
    # 记录开始时间
    start_time = time.time()
    last_time = start_time
    last_reconnect_time = start_time
    
    # 如果在终端中运行，清除终端
    clear_console = lambda: os.system('cls' if os.name == 'nt' else 'clear')
    
    print("开始监控Carla性能...")
    
    try:
        while running:
            current_time = time.time()
            elapsed = current_time - last_time
            
            if elapsed < REFRESH_RATE:
                time.sleep(0.1)  # 小的睡眠时间，避免CPU空转
                continue
            
            # 记录时间戳
            timestamp = datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S')
            performance_data["timestamps"].append(timestamp)
            
            # 获取系统CPU和内存使用
            cpu_percent = 0
            ram_usage = 0
            
            if PSUTIL_AVAILABLE:
                cpu_percent = psutil.cpu_percent(interval=0.1)
                ram_usage = psutil.virtual_memory().used / (1024**2)  # MB
            
            performance_data["cpu_usage"].append(cpu_percent)
            performance_data["ram_usage"].append(ram_usage)
            
            # 获取GPU使用情况
            gpu_percent = 0
            gpu_memory = 0
            
            if GPUTIL_AVAILABLE:
                try:
                    gpus = GPUtil.getGPUs()
                    if gpus:
                        gpu_percent = gpus[0].load * 100  # 转换为百分比
                        gpu_memory = gpus[0].memoryUsed  # MB
                except:
                    pass
            
            performance_data["gpu_usage"].append(gpu_percent)
            performance_data["gpu_memory"].append(gpu_memory)
            
            # 获取Carla进程信息
            carla_cpu_percent = 0
            carla_memory_mb = 0
            
            # 如果找不到Carla进程或者需要刷新信息
            if carla_process is None or not carla_process.is_running():
                find_carla_process()
            
            if carla_process:
                try:
                    carla_process.cpu_percent(interval=0.1)  # 第一次调用总是返回0，所以先预热
                    time.sleep(0.1)
                    carla_cpu_percent = carla_process.cpu_percent(interval=0.1)
                    carla_memory_mb = carla_process.memory_info().rss / (1024**2)  # MB
                except:
                    # 如果获取失败，尝试重新查找进程
                    find_carla_process()
            
            # 获取真实的Carla FPS
            real_fps, real_frame_time = 0, 0
            
            if CARLA_AVAILABLE and carla_world:
                real_fps, real_frame_time = get_carla_performance()
            
            # 如果无法获取真实FPS且已经过去5秒，尝试重新连接
            if CARLA_AVAILABLE and (real_fps <= 0) and (current_time - last_reconnect_time) > 5.0:
                print("尝试重新连接到Carla...")
                connect_to_carla()
                last_reconnect_time = current_time
            
            performance_data["fps"].append(real_fps)
            performance_data["frame_times"].append(real_frame_time)
            
            # 记录到CSV
            if log_writer:
                row = [
                    timestamp,
                    f"{real_fps:.1f}",
                    f"{real_frame_time:.2f}",
                    f"{cpu_percent:.1f}",
                    f"{ram_usage:.1f}"
                ]
                
                # 添加GPU数据（如果可用）
                if GPUTIL_AVAILABLE:
                    row.extend([
                        f"{gpu_percent:.1f}",
                        f"{gpu_memory:.1f}"
                    ])
                
                # 添加Carla特定数据
                row.extend([
                    f"{carla_cpu_percent:.1f}",
                    f"{carla_memory_mb:.1f}",
                    os.cpu_count(),
                    f"{psutil.virtual_memory().total / (1024**3):.1f}" if PSUTIL_AVAILABLE else "N/A"
                ])
                
                log_writer.writerow(row)
                log_file.flush()
            
            # 清屏并显示性能信息
            clear_console()
            print("\n===== CARLA 性能监控 =====")
            print(f"时间戳: {timestamp}")
            print(f"运行时间: {current_time - start_time:.1f} 秒")
            print(f"\n--- 模拟性能 ---")
            
            if real_fps > 0:
                print(f"实际FPS: {real_fps:.1f}")
                print(f"实际帧时间: {real_frame_time:.2f} ms")
            else:
                print("实际FPS: 无法获取 (等待Carla数据)")
                print("实际帧时间: 无法获取")
            
            print(f"\n--- 系统资源 ---")
            print(f"系统CPU使用率: {cpu_percent:.1f}%")
            print(f"系统内存使用: {ram_usage:.1f} MB")
            
            if GPUTIL_AVAILABLE:
                print(f"GPU使用率: {gpu_percent:.1f}%")
                print(f"GPU内存使用: {gpu_memory:.1f} MB")
            
            if carla_process:
                print(f"\n--- Carla进程 (PID: {carla_pid}) ---")
                print(f"CPU使用率: {carla_cpu_percent:.1f}%")
                print(f"内存使用: {carla_memory_mb:.1f} MB")
            
            print(f"\n--- 事件 ---")
            print(f"已接收事件数: {len(events)}")
            
            # 显示最近的事件
            recent_events = events[-5:] if len(events) > 5 else events
            for i, event in enumerate(recent_events):
                print(f"  {i+1}. {event['timestamp']} - {event['type']}")
            
            print("\n性能日志文件:")
            if log_file:
                print(f"  {log_file.name}")
            
            if event_log_file:
                print(f"  {event_log_file.name}")
            
            print("===========================")
            
            # 更新时间
            last_time = current_time
    except KeyboardInterrupt:
        print("\n监控被用户中断")
    except Exception as e:
        print(f"监控过程中发生错误: {e}")
    finally:
        if log_file:
            log_file.close()
        if event_log_file:
            event_log_file.close()

def signal_handler(sig, frame):
    """处理信号，安全退出"""
    global running
    print('\n正在停止监控...')
    running = False

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        '--port',
        type=int,
        default=EVENT_PORT,
        help=f'事件监听端口 (默认: {EVENT_PORT})')
    parser.add_argument(
        '--refresh',
        type=float,
        default=REFRESH_RATE,
        help=f'性能刷新频率，单位秒 (默认: {REFRESH_RATE})')
    parser.add_argument(
        '--no-log',
        action='store_true',
        help='禁用日志记录')
    parser.add_argument(
        '--graph',
        action='store_true',
        help='启用实时性能图表 (需要matplotlib)')
    parser.add_argument(
        '--carla-host',
        default='localhost',
        help='Carla服务器地址 (默认: localhost)')
    parser.add_argument(
        '--carla-port',
        type=int,
        default=2000,
        help='Carla服务器端口 (默认: 2000)')
    
    args = parser.parse_args()
    
    # 设置信号处理
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    
    # 打印欢迎信息
    print("\n=== Carla 性能监控工具 ===")
    print(f"版本: 1.0")
    print(f"系统: {platform.system()} {platform.version()}")
    print(f"Python: {platform.python_version()}")
    print(f"Carla Python API可用: {CARLA_AVAILABLE}")
    print(f"psutil可用: {PSUTIL_AVAILABLE}")
    print(f"GPUtil可用: {GPUTIL_AVAILABLE}")
    print(f"matplotlib可用: {MATPLOTLIB_AVAILABLE}")
    
    # 设置日志记录
    if not args.no_log:
        setup_logging(args)
        log_system_info()  # 记录系统信息
    
    # 查找Carla进程
    find_carla_process()
    
    # 连接到Carla服务器
    if CARLA_AVAILABLE:
        connect_to_carla(args.carla_host, args.carla_port)
    
    # 启动事件监听器
    listener_thread = start_event_listener()
    
    # 如果需要图表且matplotlib可用，启动图表线程
    graph_thread = None
    if args.graph and MATPLOTLIB_AVAILABLE:
        # 图表代码在这里...
        pass
    
    try:
        # 开始监控
        monitor_carla_performance()
    finally:
        # 确保正确关闭
        global running
        running = False
        
        # 等待线程结束
        if listener_thread.is_alive():
            listener_thread.join(timeout=2.0)
        
        if graph_thread and graph_thread.is_alive():
            graph_thread.join(timeout=2.0)
        
        # 关闭套接字
        if event_socket:
            event_socket.close()
        
        # 关闭日志文件
        if log_file:
            log_file.close()
        
        if event_log_file:
            event_log_file.close()
        
        print("\n监控已安全结束")

if __name__ == "__main__":
    main()