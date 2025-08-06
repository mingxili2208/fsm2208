import yaml
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import os
import pickle
from datetime import datetime
import time

# --- 数据解析模块 ---

def parse_yaml_log_file_fast(filepath, max_docs=None):
    """快速解析YAML日志文件"""
    messages = []
    doc_count = 0
    
    print(f"Reading file: {filepath}")
    start_time = time.time()
    
    try:
        with open(filepath, 'r') as f:
            docs = yaml.load_all(f, Loader=yaml.CLoader)
            for doc in docs:
                if doc:
                    messages.append(doc)
                    doc_count += 1
                    if max_docs and doc_count >= max_docs:
                        break
                        
                if doc_count % 1000 == 0:
                    print(f"  Processed {doc_count} documents...")
                    
    except Exception as e:
        print(f"Error reading {filepath}: {e}")
        return None
    
    elapsed = time.time() - start_time
    print(f"Successfully loaded {len(messages)} documents in {elapsed:.2f}s")
    return messages

def extract_single_path_points(path_msg):
    """从单个path消息中提取点"""
    points = []
    for point in path_msg.get('points', []):
        if isinstance(point, str):
            continue
        if not isinstance(point, dict):
            continue
        if 'pose' not in point or 'position' not in point['pose']:
            continue
        
        try:
            pos = point['pose']['position']
            points.append([pos['x'], pos['y']])
        except (KeyError, TypeError):
            continue
    
    return np.array(points) if points else np.array([]).reshape(0, 2)

def remove_duplicate_points_from_start(new_path, existing_path, threshold=1.0):
    """
    从新路径的开始部分移除与现有路径重复的点
    threshold: 距离阈值，小于此距离认为是重复点
    """
    if len(existing_path) == 0 or len(new_path) == 0:
        return new_path
    
    # 找到新路径中第一个不重复的点
    start_idx = 0
    for i, new_point in enumerate(new_path):
        # 检查这个点是否与现有路径中的任何点太近
        distances = np.linalg.norm(existing_path - new_point, axis=1)
        min_distance = np.min(distances)
        
        if min_distance > threshold:
            start_idx = i
            break
        start_idx = i + 1
    
    # 返回从第一个不重复点开始的路径
    return new_path[start_idx:] if start_idx < len(new_path) else np.array([]).reshape(0, 2)

def extract_path_points_proper(path_msgs, overlap_threshold=1.0):
    """
    正确拼接path消息，去除重复部分
    """
    print(f"Processing {len(path_msgs)} path messages with proper overlap removal...")
    start_time = time.time()
    
    # 按时间戳排序
    path_msgs_with_time = []
    for msg in path_msgs:
        try:
            ts_sec = msg['header']['stamp']['sec']
            ts_nsec = msg['header']['stamp']['nanosec']
            timestamp = ts_sec + ts_nsec * 1e-9
            path_msgs_with_time.append((timestamp, msg))
        except (KeyError, TypeError):
            continue
    
    path_msgs_with_time.sort(key=lambda x: x[0])
    print(f"Sorted {len(path_msgs_with_time)} path messages by timestamp")
    
    # 拼接路径
    complete_path = np.array([]).reshape(0, 2)
    
    for i, (timestamp, path_msg) in enumerate(path_msgs_with_time):
        # 提取当前路径的点
        current_path_points = extract_single_path_points(path_msg)
        
        if len(current_path_points) == 0:
            continue
        
        if len(complete_path) == 0:
            # 第一条路径，直接添加
            complete_path = current_path_points
        else:
            # 移除与已有路径重复的部分
            non_duplicate_points = remove_duplicate_points_from_start(
                current_path_points, complete_path, overlap_threshold
            )
            
            if len(non_duplicate_points) > 0:
                complete_path = np.vstack([complete_path, non_duplicate_points])
        
        if (i + 1) % 50 == 0:
            print(f"  Processed {i + 1}/{len(path_msgs_with_time)} path messages, total points: {len(complete_path)}")
    
    elapsed = time.time() - start_time
    print(f"Path concatenation completed in {elapsed:.2f}s")
    print(f"Final path has {len(complete_path)} points")
    
    return complete_path

def extract_trajectory_points_proper(traj_msgs, overlap_threshold=0.5, max_points_per_msg=10):
    """
    正确拼接trajectory消息
    """
    print(f"Processing {len(traj_msgs)} trajectory messages with proper overlap removal...")
    start_time = time.time()
    
    # 按时间戳排序
    traj_msgs_with_time = []
    for msg in traj_msgs:
        try:
            ts_sec = msg['header']['stamp']['sec']
            ts_nsec = msg['header']['stamp']['nanosec']
            timestamp = ts_sec + ts_nsec * 1e-9
            traj_msgs_with_time.append((timestamp, msg))
        except (KeyError, TypeError):
            continue
    
    traj_msgs_with_time.sort(key=lambda x: x[0])
    print(f"Sorted {len(traj_msgs_with_time)} trajectory messages by timestamp")
    
    # 拼接轨迹
    complete_traj = np.array([]).reshape(0, 2)
    
    for i, (timestamp, traj_msg) in enumerate(traj_msgs_with_time):
        # 提取当前轨迹的点（只取前几个）
        current_traj_points = []
        for j, point in enumerate(traj_msg.get('points', [])):
            if j >= max_points_per_msg:  # 限制每个消息的点数
                break
                
            if isinstance(point, str):
                continue
            if not isinstance(point, dict):
                continue
            if 'pose' not in point or 'position' not in point['pose']:
                continue
            
            try:
                pos = point['pose']['position']
                current_traj_points.append([pos['x'], pos['y']])
            except (KeyError, TypeError):
                continue
        
        current_traj_points = np.array(current_traj_points) if current_traj_points else np.array([]).reshape(0, 2)
        
        if len(current_traj_points) == 0:
            continue
        
        if len(complete_traj) == 0:
            # 第一条轨迹，直接添加
            complete_traj = current_traj_points
        else:
            # 移除与已有轨迹重复的部分
            non_duplicate_points = remove_duplicate_points_from_start(
                current_traj_points, complete_traj, overlap_threshold
            )
            
            if len(non_duplicate_points) > 0:
                complete_traj = np.vstack([complete_traj, non_duplicate_points])
        
        if (i + 1) % 100 == 0:
            print(f"  Processed {i + 1}/{len(traj_msgs_with_time)} trajectory messages, total points: {len(complete_traj)}")
    
    elapsed = time.time() - start_time
    print(f"Trajectory concatenation completed in {elapsed:.2f}s")
    print(f"Final trajectory has {len(complete_traj)} points")
    
    return complete_traj

def extract_pose_points_batch(pose_msgs):
    """批量提取pose数据"""
    pose_data = []
    
    print(f"Processing {len(pose_msgs)} pose messages...")
    start_time = time.time()
    
    for i, msg in enumerate(pose_msgs):
        try:
            ts_sec = msg['header']['stamp']['sec']
            ts_nsec = msg['header']['stamp']['nanosec']
            timestamp = ts_sec + ts_nsec * 1e-9
            
            pos = msg['pose']['pose']['position']
            pose_data.append({
                'timestamp': timestamp,
                'x': pos['x'],
                'y': pos['y']
            })
        except (KeyError, TypeError):
            continue
        
        if (i + 1) % 500 == 0:
            print(f"  Processed {i + 1}/{len(pose_msgs)} pose messages")
    
    elapsed = time.time() - start_time
    print(f"Extracted {len(pose_data)} pose points in {elapsed:.2f}s")
    return pose_data

# --- 缓存模块 ---

def save_processed_data(data_dict, cache_file):
    """保存处理后的数据"""
    print(f"Saving processed data to {cache_file}")
    with open(cache_file, 'wb') as f:
        pickle.dump(data_dict, f, protocol=pickle.HIGHEST_PROTOCOL)
    file_size = os.path.getsize(cache_file) / (1024 * 1024)
    print(f"Data saved, file size: {file_size:.2f}MB")

def load_processed_data(cache_file):
    """加载处理后的数据"""
    if not os.path.exists(cache_file):
        return None
    
    try:
        with open(cache_file, 'rb') as f:
            data = pickle.load(f)
        print(f"Successfully loaded cached data from {cache_file}")
        return data
    except Exception as e:
        print(f"Error loading cached data: {e}")
        return None

# --- 主分析流程 ---

def main():
    # --- 配置 ---
    timestamp_str = "20250425_163228"
    experiment_name = "autoware_cte_proper_concat"
    
    # 文件路径
    path_file = f'./path_{timestamp_str}.log'
    trajectory_file = f'./trajectory_{timestamp_str}.log'
    actual_pose_file = f'./transformed_pose_{timestamp_str}.log'
    
    # 创建目录
    result_dir = f'./result_{experiment_name}_{timestamp_str}'
    cache_dir = f'./cache_{timestamp_str}'
    os.makedirs(result_dir, exist_ok=True)
    os.makedirs(cache_dir, exist_ok=True)
    
    cache_file = f'{cache_dir}/processed_data_proper.pkl'
    
    print(f"=== Analysis started at {datetime.now()} ===")
    print(f"Experiment: {experiment_name}")
    
    # 检查缓存
    cached_data = load_processed_data(cache_file)
    
    if not cached_data:
        print("\n--- Processing raw data ---")
        
        # 加载原始数据
        print("\n1. Loading actual pose data...")
        actual_pose_msgs = parse_yaml_log_file_fast(actual_pose_file)
        if not actual_pose_msgs:
            return
        
        print("\n2. Loading path data...")
        path_msgs = parse_yaml_log_file_fast(path_file)
        if not path_msgs:
            return
        
        print("\n3. Loading trajectory data...")
        traj_msgs = parse_yaml_log_file_fast(trajectory_file)
        if not traj_msgs:
            return
        
        # 处理数据
        print("\n4. Processing data with proper concatenation...")
        pose_data = extract_pose_points_batch(actual_pose_msgs)
        
        # 使用改进的拼接方法
        path_points = extract_path_points_proper(path_msgs, overlap_threshold=1.0)
        traj_points = extract_trajectory_points_proper(traj_msgs, overlap_threshold=0.5)
        
        # 缓存数据
        processed_data = {
            'pose_data': pose_data,
            'path_points': path_points,
            'traj_points': traj_points,
            'timestamp': datetime.now().isoformat()
        }
        save_processed_data(processed_data, cache_file)
    else:
        pose_data = cached_data['pose_data']
        path_points = cached_data['path_points']
        traj_points = cached_data['traj_points']
        print("Using cached data")
    
    # --- 数据验证 ---
    print(f"\n--- Data Summary ---")
    print(f"Pose records: {len(pose_data)}")
    print(f"Path points: {len(path_points)}")
    print(f"Trajectory points: {len(traj_points)}")
    
    # --- 可视化 ---
    print("\n--- Generating visualizations ---")
    plt.ioff()
    
    # 准备数据
    df_actual = pd.DataFrame(pose_data)
    
    # 图1: 真实轨迹
    plt.figure(figsize=(12, 10))
    plt.plot(df_actual['x'], df_actual['y'], 'b-', linewidth=2, label='Actual Trajectory')
    plt.title('Actual Vehicle Trajectory')
    plt.xlabel('X (meters)')
    plt.ylabel('Y (meters)')
    plt.legend()
    plt.grid(True, alpha=0.3)
    plt.axis('equal')
    plt.tight_layout()
    plt.savefig(f'{result_dir}/01_actual_trajectory.png', dpi=300, bbox_inches='tight')
    plt.close()
    
    # 图2: Path轨迹（修正后）
    if len(path_points) > 0:
        plt.figure(figsize=(12, 10))
        plt.plot(path_points[:, 0], path_points[:, 1], 'r-', linewidth=2, label='Path (Global Planning)')
        plt.title('Path Trajectory (Global Planning) - Properly Concatenated')
        plt.xlabel('X (meters)')
        plt.ylabel('Y (meters)')
        plt.legend()
        plt.grid(True, alpha=0.3)
        plt.axis('equal')
        plt.tight_layout()
        plt.savefig(f'{result_dir}/02_path_trajectory_fixed.png', dpi=300, bbox_inches='tight')
        plt.close()
    
    # 图3: Trajectory轨迹（修正后）
    if len(traj_points) > 0:
        plt.figure(figsize=(12, 10))
        plt.plot(traj_points[:, 0], traj_points[:, 1], 'g-', linewidth=2, label='Trajectory (Local Planning)')
        plt.title('Trajectory (Local Planning) - Properly Concatenated')
        plt.xlabel('X (meters)')
        plt.ylabel('Y (meters)')
        plt.legend()
        plt.grid(True, alpha=0.3)
        plt.axis('equal')
        plt.tight_layout()
        plt.savefig(f'{result_dir}/03_trajectory_trajectory_fixed.png', dpi=300, bbox_inches='tight')
        plt.close()
    
    # 图4: 所有轨迹对比（修正后）
    plt.figure(figsize=(15, 12))
    plt.plot(df_actual['x'], df_actual['y'], 'b-', linewidth=3, label='Actual Trajectory (Truth)')
    
    if len(path_points) > 0:
        plt.plot(path_points[:, 0], path_points[:, 1], 'r-', linewidth=2, alpha=0.8, label='Path (Global Planning)')
    
    if len(traj_points) > 0:
        plt.plot(traj_points[:, 0], traj_points[:, 1], 'g-', linewidth=1, alpha=0.7, label='Trajectory (Local Planning)')
    
    plt.title('Trajectory Comparison - Properly Concatenated')
    plt.xlabel('X (meters)')
    plt.ylabel('Y (meters)')
    plt.legend()
    plt.grid(True, alpha=0.3)
    plt.axis('equal')
    plt.tight_layout()
    plt.savefig(f'{result_dir}/04_trajectory_comparison_fixed.png', dpi=300, bbox_inches='tight')
    plt.close()
    
    # 保存数据
    df_actual.to_csv(f'{result_dir}/pose_data.csv', index=False)
    if len(path_points) > 0:
        np.savetxt(f'{result_dir}/path_points_fixed.csv', path_points, delimiter=',', header='x,y', comments='')
    if len(traj_points) > 0:
        np.savetxt(f'{result_dir}/trajectory_points_fixed.csv', traj_points, delimiter=',', header='x,y', comments='')
    
    print(f"\n=== Analysis completed ===")
    print(f"Results saved to: {result_dir}")

if __name__ == '__main__':
    main()