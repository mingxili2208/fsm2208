import yaml
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import os
import pickle
from datetime import datetime
import time
from scipy.spatial.distance import cdist
from scipy.interpolate import interp1d

# --- CTE计算模块（改进版）---

def calculate_cte_to_single_path(vehicle_pos, path_points, max_distance=50.0):
    """
    计算车辆位置相对于单条路径的CTE
    使用正确的几何方法：点到线段的距离
    """
    vehicle_pos = np.asarray(vehicle_pos)
    path_points = np.asarray(path_points)

    if len(path_points) < 2:
        return np.nan, np.nan, -1

    min_distance = float('inf')
    best_cte = np.nan
    best_segment_idx = -1
    
    # 遍历所有路径线段
    for i in range(len(path_points) - 1):
        p_a = path_points[i]
        p_b = path_points[i + 1]
        
        # 计算点到线段的距离和CTE
        cte, distance = point_to_segment_distance_and_cte(vehicle_pos, p_a, p_b)
        
        if distance < min_distance:
            min_distance = distance
            best_cte = cte
            best_segment_idx = i
    
    # 如果距离太远，认为不在路径上
    if min_distance > max_distance:
        return np.nan, np.nan, -1
        
    return best_cte, min_distance, best_segment_idx

def point_to_segment_distance_and_cte(point, seg_start, seg_end):
    """
    计算点到线段的距离和CTE（带符号）
    
    Returns:
    - cte: 带符号的横向偏差（右正左负）
    - distance: 点到线段的最短距离
    """
    point = np.asarray(point)
    seg_start = np.asarray(seg_start)
    seg_end = np.asarray(seg_end)
    
    # 线段向量
    seg_vec = seg_end - seg_start
    seg_length_sq = np.dot(seg_vec, seg_vec)
    
    if seg_length_sq < 1e-10:  # 线段长度为0
        distance = np.linalg.norm(point - seg_start)
        return distance, distance  # 无方向性
    
    # 计算投影参数t
    point_vec = point - seg_start
    t = np.dot(point_vec, seg_vec) / seg_length_sq
    
    # 将t限制在[0,1]范围内（投影到线段上）
    t = max(0, min(1, t))
    
    # 计算投影点
    projection = seg_start + t * seg_vec
    
    # 计算距离
    distance = np.linalg.norm(point - projection)
    
    # 计算CTE符号（使用叉积）
    # 叉积的z分量表示方向
    cross_z = seg_vec[0] * (point[1] - seg_start[1]) - seg_vec[1] * (point[0] - seg_start[0])
    cte = np.sign(cross_z) * distance
    
    return cte, distance

def calculate_cte_based_on_correspondence(vehicle_positions, path_points, method='nearest_path_point'):
    """
    基于对应关系计算CTE
    
    Parameters:
    - vehicle_positions: 车辆位置数组 (N, 2)
    - path_points: 路径点数组 (M, 2)
    - method: 对应关系方法
        - 'nearest_path_point': 每个vehicle位置找最近的path点
        - 'nearest_vehicle_point': 每个path点找最近的vehicle位置
        - 'bidirectional': 双向对应
    """
    print(f"Calculating CTE using {method} correspondence...")
    
    vehicle_positions = np.asarray(vehicle_positions)
    path_points = np.asarray(path_points)
    
    if method == 'nearest_path_point':
        return _cte_vehicle_to_path_correspondence(vehicle_positions, path_points)
    elif method == 'nearest_vehicle_point':
        return _cte_path_to_vehicle_correspondence(vehicle_positions, path_points)
    elif method == 'bidirectional':
        return _cte_bidirectional_correspondence(vehicle_positions, path_points)
    else:
        raise ValueError(f"Unknown method: {method}")

def _cte_vehicle_to_path_correspondence(vehicle_positions, path_points):
    """每个vehicle位置找最近的path点计算CTE"""
    cte_values = []
    distances = []
    path_indices = []
    
    for i, vehicle_pos in enumerate(vehicle_positions):
        # 找到最近的路径线段
        cte, distance, seg_idx = calculate_cte_to_single_path(vehicle_pos, path_points)
        cte_values.append(cte)
        distances.append(distance)
        path_indices.append(seg_idx)
        
        if (i + 1) % 1000 == 0:
            print(f"  Processed {i + 1}/{len(vehicle_positions)} vehicle positions")
    
    return {
        'cte': np.array(cte_values),
        'distances': np.array(distances),
        'path_indices': np.array(path_indices),
        'method': 'vehicle_to_path'
    }

def _cte_path_to_vehicle_correspondence(vehicle_positions, path_points):
    """每个path点找最近的vehicle位置计算CTE"""
    cte_values = []
    distances = []
    vehicle_indices = []
    
    for i, path_point in enumerate(path_points[:-1]):  # 除了最后一个点
        path_segment = [path_points[i], path_points[i + 1]]
        
        # 找到离这个线段最近的vehicle位置
        min_distance = float('inf')
        best_cte = np.nan
        best_vehicle_idx = -1
        
        for j, vehicle_pos in enumerate(vehicle_positions):
            cte, distance = point_to_segment_distance_and_cte(
                vehicle_pos, path_segment[0], path_segment[1])
            
            if distance < min_distance:
                min_distance = distance
                best_cte = cte
                best_vehicle_idx = j
        
        cte_values.append(best_cte)
        distances.append(min_distance)
        vehicle_indices.append(best_vehicle_idx)
        
        if (i + 1) % 1000 == 0:
            print(f"  Processed {i + 1}/{len(path_points)-1} path segments")
    
    return {
        'cte': np.array(cte_values),
        'distances': np.array(distances),
        'vehicle_indices': np.array(vehicle_indices),
        'method': 'path_to_vehicle'
    }

def _cte_bidirectional_correspondence(vehicle_positions, path_points):
    """双向对应关系"""
    # 计算两个方向的对应关系
    v_to_p = _cte_vehicle_to_path_correspondence(vehicle_positions, path_points)
    p_to_v = _cte_path_to_vehicle_correspondence(vehicle_positions, path_points)
    
    return {
        'vehicle_to_path': v_to_p,
        'path_to_vehicle': p_to_v,
        'method': 'bidirectional'
    }

# --- 之前的辅助函数（保持不变）---

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

def smart_path_concatenation(path_msgs, distance_threshold=2.0, continuity_threshold=10.0):
    """智能路径拼接"""
    print(f"\n=== Smart Path Concatenation ===")
    print(f"Processing {len(path_msgs)} path messages...")
    
    path_segments = []
    
    for i, msg in enumerate(path_msgs):
        try:
            ts_sec = msg['header']['stamp']['sec']
            ts_nsec = msg['header']['stamp']['nanosec']
            timestamp = ts_sec + ts_nsec * 1e-9
            
            points = []
            for point in msg.get('points', []):
                if isinstance(point, dict) and 'pose' in point and 'position' in point['pose']:
                    pos = point['pose']['position']
                    points.append([pos['x'], pos['y']])
            
            if len(points) > 1:
                points = np.array(points)
                path_segments.append({
                    'timestamp': timestamp,
                    'points': points,
                    'msg_id': i
                })
        except Exception as e:
            continue
    
    path_segments.sort(key=lambda x: x['timestamp'])
    print(f"Found {len(path_segments)} valid path segments")
    
    if not path_segments:
        return np.array([]).reshape(0, 2)
    
    final_path = path_segments[0]['points'].copy()
    
    for i in range(1, len(path_segments)):
        current_segment = path_segments[i]['points']
        last_point = final_path[-1]
        first_point = current_segment[0]
        connection_distance = np.linalg.norm(last_point - first_point)
        
        if connection_distance <= continuity_threshold:
            non_duplicate_start = 0
            for j, point in enumerate(current_segment):
                recent_points = final_path[-min(50, len(final_path)):]
                distances = np.linalg.norm(recent_points - point, axis=1)
                min_distance = np.min(distances)
                
                if min_distance > distance_threshold:
                    non_duplicate_start = j
                    break
                non_duplicate_start = j + 1
            
            if non_duplicate_start < len(current_segment):
                new_points = current_segment[non_duplicate_start:]
                final_path = np.vstack([final_path, new_points])
        else:
            if len(final_path) > 0:
                distances_to_existing = cdist(current_segment, final_path)
                min_distances = np.min(distances_to_existing, axis=1)
                far_points_ratio = np.sum(min_distances > distance_threshold * 2) / len(min_distances)
                
                if far_points_ratio > 0.8:
                    final_path = np.vstack([final_path, current_segment])
    
    return final_path

def extract_pose_points_batch(pose_msgs):
    """批量提取pose数据"""
    pose_data = []
    
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
    
    return pose_data

def extract_trajectory_points_proper(traj_msgs, overlap_threshold=0.5, max_points_per_msg=10):
    """拼接trajectory数据"""
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
    
    complete_traj = np.array([]).reshape(0, 2)
    
    for i, (timestamp, traj_msg) in enumerate(traj_msgs_with_time):
        current_traj_points = []
        for j, point in enumerate(traj_msg.get('points', [])):
            if j >= max_points_per_msg:
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
            complete_traj = current_traj_points
        else:
            # 简单的重复检测
            if len(complete_traj) > 0 and len(current_traj_points) > 0:
                last_point = complete_traj[-1]
                first_new_point = current_traj_points[0]
                if np.linalg.norm(last_point - first_new_point) > overlap_threshold:
                    complete_traj = np.vstack([complete_traj, current_traj_points])
                else:
                    # 找到非重复点开始添加
                    for k, point in enumerate(current_traj_points):
                        if np.linalg.norm(last_point - point) > overlap_threshold:
                            complete_traj = np.vstack([complete_traj, current_traj_points[k:]])
                            break
    
    return complete_traj

# --- 主分析流程 ---

def main():
    # --- 配置 ---
    timestamp_str = "20250425_163228"
    experiment_name = "improved_cte_analysis"
    
    # 文件路径
    path_file = f'./path_{timestamp_str}.log'
    trajectory_file = f'./trajectory_{timestamp_str}.log'
    actual_pose_file = f'./transformed_pose_{timestamp_str}.log'
    
    # 创建目录
    result_dir = f'./result_{experiment_name}_{timestamp_str}'
    os.makedirs(result_dir, exist_ok=True)
    
    print(f"=== Improved CTE Analysis ===")
    print(f"Results will be saved to: {result_dir}")
    
    # 加载数据
    print("\n1. Loading data...")
    path_msgs = parse_yaml_log_file_fast(path_file)
    traj_msgs = parse_yaml_log_file_fast(trajectory_file)
    pose_msgs = parse_yaml_log_file_fast(actual_pose_file)
    
    if not all([path_msgs, traj_msgs, pose_msgs]):
        print("Failed to load all required data")
        return
    
    # 处理数据
    print("\n2. Processing path data...")
    clean_path = smart_path_concatenation(path_msgs)  # 使用带gap的原始path
    
    print("\n3. Processing trajectory data...")
    clean_traj = extract_trajectory_points_proper(traj_msgs)
    
    print("\n4. Processing pose data...")
    pose_data = extract_pose_points_batch(pose_msgs)
    
    # 准备分析数据
    df_actual = pd.DataFrame(pose_data)
    vehicle_positions = df_actual[['x', 'y']].values
    
    print(f"Data summary:")
    print(f"- Path points: {len(clean_path)}")
    print(f"- Trajectory points: {len(clean_traj)}")
    print(f"- Vehicle positions: {len(vehicle_positions)}")
    
    # 计算不同方法的CTE
    print("\n5. Calculating CTE using different methods...")
    
    # 方法1: 传统时序方法（每个vehicle位置找最近路径点）
    print("Method 1: Traditional time-based (vehicle to nearest path)")
    cte_traditional = []
    for pos in vehicle_positions:
        cte, _, _ = calculate_cte_to_single_path(pos, clean_path)
        cte_traditional.append(cte)
    
    # 方法2: 基于对应关系 - vehicle到path
    print("Method 2: Vehicle-to-path correspondence")
    result_v2p = calculate_cte_based_on_correspondence(vehicle_positions, clean_path, 'nearest_path_point')
    
    # 方法3: 基于对应关系 - path到vehicle
    print("Method 3: Path-to-vehicle correspondence")
    result_p2v = calculate_cte_based_on_correspondence(vehicle_positions, clean_path, 'nearest_vehicle_point')
    
    # 方法4: 对trajectory计算CTE
    print("Method 4: Traditional to trajectory")
    cte_trajectory = []
    for pos in vehicle_positions:
        cte, _, _ = calculate_cte_to_single_path(pos, clean_traj)
        cte_trajectory.append(cte)
    
    # 整理结果到DataFrame
    df_actual['cte_traditional'] = cte_traditional
    df_actual['cte_v2p'] = result_v2p['cte']
    df_actual['cte_trajectory'] = cte_trajectory
    
    # 为path-to-vehicle结果创建对应的DataFrame
    df_path_based = pd.DataFrame({
        'cte_p2v': result_p2v['cte'],
        'distances': result_p2v['distances'],
        'vehicle_indices': result_p2v['vehicle_indices']
    })
    
    # 移除无效数据
    df_clean = df_actual.dropna(subset=['cte_traditional', 'cte_v2p', 'cte_trajectory'])
    
    print(f"Valid CTE data points: {len(df_clean)}")
    
    # --- 生成可视化 ---
    print("\n6. Generating visualizations...")
    plt.ioff()
    
    # 图1: 清晰的路径、轨迹和实际位置对比图
    plt.figure(figsize=(16, 12))
    
    # 主图
    plt.subplot(2, 2, (1, 2))
    # 使用不同的线宽和透明度来区分
    plt.plot(clean_path[:, 0], clean_path[:, 1], 'r-', linewidth=3, label='Global Path (with gaps)', alpha=0.8)
    plt.plot(clean_traj[:, 0], clean_traj[:, 1], 'g-', linewidth=2, label='Local Trajectory', alpha=0.7)
    plt.plot(df_actual['x'], df_actual['y'], 'b-', linewidth=1.5, label='Actual Vehicle Path', alpha=0.9)
    
    # 添加起始点标记
    if len(clean_path) > 0:
        plt.plot(clean_path[0, 0], clean_path[0, 1], 'ro', markersize=10, label='Path Start')
        plt.plot(clean_path[-1, 0], clean_path[-1, 1], 'rs', markersize=10, label='Path End')
    
    if len(df_actual) > 0:
        plt.plot(df_actual['x'].iloc[0], df_actual['y'].iloc[0], 'bo', markersize=8, label='Vehicle Start')
        plt.plot(df_actual['x'].iloc[-1], df_actual['y'].iloc[-1], 'bs', markersize=8, label='Vehicle End')
    
    plt.title('Path, Trajectory and Actual Vehicle Position Comparison', fontsize=14, fontweight='bold')
    plt.xlabel('X (meters)', fontsize=12)
    plt.ylabel('Y (meters)', fontsize=12)
    plt.legend(fontsize=10)
    plt.grid(True, alpha=0.3)
    plt.axis('equal')
    
    # 局部放大图1
    plt.subplot(2, 2, 3)
    if len(clean_path) > 0:
        # 选择中间部分进行放大
        mid_idx = len(clean_path) // 2
        zoom_range = 100  # 放大范围的点数
        start_idx = max(0, mid_idx - zoom_range)
        end_idx = min(len(clean_path), mid_idx + zoom_range)
        
        path_zoom = clean_path[start_idx:end_idx]
        
        # 找到对应时间范围的vehicle和trajectory数据
        if len(path_zoom) > 0:
            center_x, center_y = path_zoom[len(path_zoom)//2]
            
            # 找到附近的vehicle positions
            vehicle_mask = ((df_actual['x'] - center_x)**2 + (df_actual['y'] - center_y)**2) < 50**2
            vehicle_zoom = df_actual[vehicle_mask]
            
            # 找到附近的trajectory points
            if len(clean_traj) > 0:
                traj_distances = np.sqrt((clean_traj[:, 0] - center_x)**2 + (clean_traj[:, 1] - center_y)**2)
                traj_mask = traj_distances < 50
                traj_zoom = clean_traj[traj_mask]
            else:
                traj_zoom = np.array([]).reshape(0, 2)
            
            plt.plot(path_zoom[:, 0], path_zoom[:, 1], 'r-', linewidth=3, label='Global Path', alpha=0.8)
            if len(traj_zoom) > 0:
                plt.plot(traj_zoom[:, 0], traj_zoom[:, 1], 'g-', linewidth=2, label='Local Trajectory', alpha=0.7)
            if len(vehicle_zoom) > 0:
                plt.plot(vehicle_zoom['x'], vehicle_zoom['y'], 'b-', linewidth=1.5, label='Actual Vehicle', alpha=0.9)
    
    plt.title('Zoom View - Middle Section', fontsize=12, fontweight='bold')
    plt.xlabel('X (meters)')
    plt.ylabel('Y (meters)')
    plt.legend(fontsize=9)
    plt.grid(True, alpha=0.3)
    plt.axis('equal')
    
    # 局部放大图2
    plt.subplot(2, 2, 4)
    if len(clean_path) > 0:
        # 选择开始部分进行放大
        zoom_range = 50
        path_zoom = clean_path[:zoom_range]
        
        if len(path_zoom) > 0:
            center_x, center_y = path_zoom[0]
            
            # 找到附近的vehicle positions
            vehicle_mask = ((df_actual['x'] - center_x)**2 + (df_actual['y'] - center_y)**2) < 30**2
            vehicle_zoom = df_actual[vehicle_mask]
            
            # 找到附近的trajectory points
            if len(clean_traj) > 0:
                traj_distances = np.sqrt((clean_traj[:, 0] - center_x)**2 + (clean_traj[:, 1] - center_y)**2)
                traj_mask = traj_distances < 30
                traj_zoom = clean_traj[traj_mask]
            else:
                traj_zoom = np.array([]).reshape(0, 2)
            
            plt.plot(path_zoom[:, 0], path_zoom[:, 1], 'r-', linewidth=3, label='Global Path', alpha=0.8)
            if len(traj_zoom) > 0:
                plt.plot(traj_zoom[:, 0], traj_zoom[:, 1], 'g-', linewidth=2, label='Local Trajectory', alpha=0.7)
            if len(vehicle_zoom) > 0:
                plt.plot(vehicle_zoom['x'], vehicle_zoom['y'], 'b-', linewidth=1.5, label='Actual Vehicle', alpha=0.9)
    
    plt.title('Zoom View - Starting Section', fontsize=12, fontweight='bold')
    plt.xlabel('X (meters)')
    plt.ylabel('Y (meters)')
    plt.legend(fontsize=9)
    plt.grid(True, alpha=0.3)
    plt.axis('equal')
    
    plt.tight_layout()
    plt.savefig(f'{result_dir}/01_clear_path_comparison.png', dpi=300, bbox_inches='tight')
    plt.close()
    
    # 图2: CTE方法对比柱状图
    fig, axes = plt.subplots(2, 2, figsize=(16, 12))
    
    cte_methods = {
        'Traditional (Time-based)': df_clean['cte_traditional'],
        'Vehicle-to-Path Correspondence': df_clean['cte_v2p'],
        'Trajectory': df_clean['cte_trajectory'],
        'Path-to-Vehicle Correspondence': df_path_based['cte_p2v'].dropna()
    }
    
    colors = ['red', 'blue', 'green', 'orange']
    
    for i, (name, cte_values) in enumerate(cte_methods.items()):
        row, col = i // 2, i % 2
        
        # 移除NaN值
        valid_cte = cte_values.dropna()
        
        if len(valid_cte) > 0:
            axes[row, col].hist(valid_cte, bins=50, alpha=0.7, color=colors[i], edgecolor='black')
            axes[row, col].axvline(valid_cte.mean(), color='darkred', linestyle='--', 
                                 label=f'Mean: {valid_cte.mean():.4f}m')
            axes[row, col].axvline(0, color='black', linestyle='-', alpha=0.5)
            axes[row, col].set_title(f'CTE Distribution - {name}')
            axes[row, col].set_xlabel('CTE (meters)')
            axes[row, col].set_ylabel('Frequency')
            axes[row, col].legend()
            axes[row, col].grid(True, alpha=0.3)
    
    plt.tight_layout()
    plt.savefig(f'{result_dir}/02_cte_method_comparison.png', dpi=300, bbox_inches='tight')
    plt.close()
    
    # 图3: CTE时间序列对比
    plt.figure(figsize=(16, 10))
    
    df_clean['datetime'] = pd.to_datetime(df_clean['timestamp'], unit='s')
    df_clean_sorted = df_clean.set_index('datetime').sort_index()
    
    plt.subplot(2, 1, 1)
    for name, color in zip(['cte_traditional', 'cte_v2p'], ['red', 'blue']):
        valid_data = df_clean_sorted[name].dropna()
        if len(valid_data) > 0:
            plt.plot(valid_data.index, valid_data.values, color=color, linewidth=1.5, 
                    alpha=0.8, label=name.replace('cte_', '').replace('_', ' ').title())
    plt.axhline(0, color='k', linestyle='--', alpha=0.7)
    plt.title('CTE Time Series - Different Correspondence Methods')
    plt.ylabel('CTE (meters)')
    plt.legend()
    plt.grid(True, alpha=0.3)
    
    plt.subplot(2, 1, 2)
    valid_traj = df_clean_sorted['cte_trajectory'].dropna()
    if len(valid_traj) > 0:
        plt.plot(valid_traj.index, valid_traj.values, color='green', linewidth=1.5, 
                alpha=0.8, label='CTE to Trajectory')
    plt.axhline(0, color='k', linestyle='--', alpha=0.7)
    plt.title('CTE Time Series - Trajectory Reference')
    plt.xlabel('Time')
    plt.ylabel('CTE (meters)')
    plt.legend()
    plt.grid(True, alpha=0.3)
    
    plt.tight_layout()
    plt.savefig(f'{result_dir}/03_cte_timeseries_comparison.png', dpi=300, bbox_inches='tight')
    plt.close()
    
    # 图4: 统计对比
    stats_data = {}
    for name, cte_values in cte_methods.items():
        valid_cte = cte_values.dropna()
        if len(valid_cte) > 0:
            stats_data[name] = {
                'Count': len(valid_cte),
                'Mean': valid_cte.mean(),
                'Median': valid_cte.median(),
                'Std': valid_cte.std(),
                'RMSE': np.sqrt((valid_cte**2).mean()),
                'Max Abs': valid_cte.abs().max(),
                '95th Percentile': np.percentile(valid_cte.abs(), 95)
            }
    
    # 创建统计对比图
    metrics = ['Mean', 'Std', 'RMSE', 'Max Abs', '95th Percentile']
    x = np.arange(len(metrics))
    width = 0.2
    
    fig, ax = plt.subplots(figsize=(15, 8))
    
    for i, (name, stats) in enumerate(stats_data.items()):
        values = [abs(stats[metric]) for metric in metrics]  # 使用绝对值便于比较
        ax.bar(x + i * width, values, width, label=name, color=colors[i], alpha=0.8)
    
    ax.set_xlabel('Metrics')
    ax.set_ylabel('Value (meters)')
    ax.set_title('CTE Statistics Comparison - Different Methods')
    ax.set_xticks(x + width * 1.5)
    ax.set_xticklabels(metrics, rotation=45)
    ax.legend()
    ax.grid(True, alpha=0.3)
    
    plt.tight_layout()
    plt.savefig(f'{result_dir}/04_cte_statistics_comparison.png', dpi=300, bbox_inches='tight')
    plt.close()
    
    # --- 生成分析报告 ---
    print("\n7. Generating analysis report...")
    
    report = f"""
=== Improved CTE Analysis Report ===
Generated at: {datetime.now()}

Data Summary:
- Actual pose records: {len(df_actual)}
- Valid CTE calculations: {len(df_clean)}
- Path points: {len(clean_path)}
- Trajectory points: {len(clean_traj)}

CTE Calculation Methods Comparison:

1. Traditional (Time-based): 
   - Each vehicle position finds nearest path segment
   - Most common method in existing systems

2. Vehicle-to-Path Correspondence:
   - Each vehicle position matches to closest path segment
   - Similar to traditional but with improved geometry

3. Path-to-Vehicle Correspondence:
   - Each path segment finds closest vehicle position
   - Ensures every path segment is evaluated

4. Trajectory Reference:
   - CTE calculated relative to local trajectory
   - Represents local planning performance

CTE Statistics:
"""
    
    for name, stats in stats_data.items():
        report += f"\n{name}:\n"
        for metric, value in stats.items():
            if metric == 'Count':
                report += f"  {metric}: {value}\n"
            else:
                report += f"  {metric}: {value:.4f} m\n"
    
    # 分析不同方法的优缺点
    report += f"""

Method Analysis:

Traditional vs Vehicle-to-Path Correspondence:
- Both methods are similar in concept but V2P uses improved geometric calculation
- RMSE comparison shows the difference in accuracy

Path-to-Vehicle Correspondence:
- Provides different perspective: how well the vehicle follows each path segment
- May have different sample size due to different correspondence logic
- Useful for path quality assessment

Trajectory Reference:
- Shows local planning performance
- Generally should have lower CTE as trajectory is adapted to current conditions
- Useful for comparing global path vs local trajectory quality

Recommendations:
1. For path following evaluation: Use Vehicle-to-Path correspondence
2. For path quality assessment: Use Path-to-Vehicle correspondence  
3. For local planning evaluation: Use Trajectory reference
4. The traditional time-based method may not be optimal due to temporal misalignment
"""
    
    # 推荐最佳方法
    vehicle_methods = {k: v for k, v in stats_data.items() if 'Vehicle' in k or 'Traditional' in k}
    if vehicle_methods:
        rmse_values = {name: stats['RMSE'] for name, stats in vehicle_methods.items()}
        best_method = min(rmse_values, key=rmse_values.get)
        
        report += f"\nRecommended Method for Path Following Evaluation:\n"
        report += f"'{best_method}' shows the best performance with RMSE = {rmse_values[best_method]:.4f}m\n"
    
    print(report)
    
    # 保存所有结果
    with open(f'{result_dir}/improved_cte_analysis_report.txt', 'w') as f:
        f.write(report)
    
    # 保存详细数据
    df_clean.to_csv(f'{result_dir}/vehicle_based_cte_analysis.csv')
    df_path_based.to_csv(f'{result_dir}/path_based_cte_analysis.csv')
    
    # 保存原始路径数据
    np.savetxt(f'{result_dir}/original_path_with_gaps.csv', clean_path, delimiter=',', header='x,y', comments='')
    np.savetxt(f'{result_dir}/trajectory.csv', clean_traj, delimiter=',', header='x,y', comments='')
    
    print(f"\n=== Improved CTE Analysis completed ===")
    print(f"All results saved to: {result_dir}")
    if 'best_method' in locals():
        print(f"Recommended method for path following: {best_method}")

if __name__ == '__main__':
    main()