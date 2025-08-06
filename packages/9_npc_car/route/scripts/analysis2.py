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

# --- GAP处理模块 ---

def interpolate_path_gaps(points, max_gap=15.0, interpolation_method='linear'):
    """
    处理路径中的GAP，使用插值填补
    
    Parameters:
    - points: 路径点数组
    - max_gap: 最大允许的GAP距离
    - interpolation_method: 插值方法 ('linear', 'cubic', 'quadratic')
    """
    if len(points) < 2:
        return points, []
    
    # 检测GAP
    distances = np.linalg.norm(np.diff(points, axis=0), axis=1)
    gap_indices = np.where(distances > max_gap)[0]
    
    if len(gap_indices) == 0:
        print("No gaps found in path")
        return points, []
    
    print(f"Found {len(gap_indices)} gaps to interpolate")
    
    interpolated_points = []
    gap_info = []
    last_idx = 0
    
    for gap_idx in gap_indices:
        # 添加GAP前的点
        interpolated_points.extend(points[last_idx:gap_idx+1])
        
        # 插值填补GAP
        p1 = points[gap_idx]
        p2 = points[gap_idx + 1]
        gap_distance = distances[gap_idx]
        
        # 计算需要插入的点数（每米大约1个点）
        num_interpolated = max(2, int(gap_distance / 2.0))
        
        # 线性插值
        for i in range(1, num_interpolated):
            t = i / num_interpolated
            interpolated_point = p1 + t * (p2 - p1)
            interpolated_points.append(interpolated_point)
        
        gap_info.append({
            'original_gap': gap_distance,
            'interpolated_points': num_interpolated - 1,
            'start_point': p1,
            'end_point': p2
        })
        
        last_idx = gap_idx + 1
    
    # 添加剩余的点
    interpolated_points.extend(points[last_idx:])
    
    return np.array(interpolated_points), gap_info

def create_path_segments(points, max_gap=20.0):
    """
    将路径分割成连续的段，不进行插值
    
    Returns:
    - segments: 连续路径段的列表
    - gaps: GAP信息
    """
    if len(points) < 2:
        return [points], []
    
    distances = np.linalg.norm(np.diff(points, axis=0), axis=1)
    gap_indices = np.where(distances > max_gap)[0]
    
    segments = []
    gaps = []
    last_idx = 0
    
    for gap_idx in gap_indices:
        # 添加当前段
        segment = points[last_idx:gap_idx+1]
        if len(segment) > 1:
            segments.append(segment)
        
        # 记录GAP信息
        gaps.append({
            'gap_distance': distances[gap_idx],
            'start_point': points[gap_idx],
            'end_point': points[gap_idx + 1],
            'start_segment': len(segments) - 1,
            'end_segment': len(segments)
        })
        
        last_idx = gap_idx + 1
    
    # 添加最后一段
    if last_idx < len(points):
        segment = points[last_idx:]
        if len(segment) > 1:
            segments.append(segment)
    
    return segments, gaps

# --- CTE计算模块（改进版） ---

def calculate_cte_to_path_segments(vehicle_positions, path_segments, max_distance=50.0):
    """
    计算车辆位置相对于分段路径的CTE
    """
    print("Calculating CTE to segmented path...")
    cte_values = []
    segment_assignments = []
    
    for i, vehicle_pos in enumerate(vehicle_positions):
        min_cte = float('inf')
        assigned_segment = -1
        
        # 检查每个路径段
        for seg_idx, segment in enumerate(path_segments):
            if len(segment) < 2:
                continue
            
            # 计算到这个段的CTE
            cte = calculate_cte_to_single_path(vehicle_pos, segment, max_distance)
            
            if not np.isnan(cte) and abs(cte) < abs(min_cte):
                min_cte = cte
                assigned_segment = seg_idx
        
        if min_cte == float('inf'):
            min_cte = np.nan
        
        cte_values.append(min_cte)
        segment_assignments.append(assigned_segment)
        
        if (i + 1) % 1000 == 0:
            print(f"  Processed {i + 1}/{len(vehicle_positions)} positions")
    
    return cte_values, segment_assignments

def calculate_cte_to_single_path(vehicle_pos, path_points, max_distance=50.0):
    """
    计算车辆位置相对于单条路径的CTE
    """
    vehicle_pos = np.asarray(vehicle_pos)
    path_points = np.asarray(path_points)

    if len(path_points) < 2:
        return np.nan

    # 找到最近的路径点
    distances = np.linalg.norm(path_points - vehicle_pos, axis=1)
    closest_idx = np.argmin(distances)
    
    # 如果距离最近点太远，认为不在这条路径上
    if distances[closest_idx] > max_distance:
        return np.nan

    # 选择合适的线段
    if closest_idx == 0:
        p_a = path_points[0]
        p_b = path_points[1] if len(path_points) > 1 else path_points[0]
    elif closest_idx == len(path_points) - 1:
        p_a = path_points[closest_idx - 1]
        p_b = path_points[closest_idx]
    else:
        # 检查前后线段，选择投影更合理的
        p_prev = path_points[closest_idx - 1]
        p_curr = path_points[closest_idx]
        p_next = path_points[closest_idx + 1]
        
        vec_prev = p_curr - p_prev
        vec_next = p_next - p_curr
        
        t_prev = np.dot(vehicle_pos - p_prev, vec_prev) / np.dot(vec_prev, vec_prev) if np.dot(vec_prev, vec_prev) > 1e-10 else 0
        t_next = np.dot(vehicle_pos - p_curr, vec_next) / np.dot(vec_next, vec_next) if np.dot(vec_next, vec_next) > 1e-10 else 0
        
        if 0 <= t_prev <= 1:
            p_a, p_b = p_prev, p_curr
        elif 0 <= t_next <= 1:
            p_a, p_b = p_curr, p_next
        else:
            if distances[closest_idx - 1] < distances[closest_idx + 1]:
                p_a, p_b = p_prev, p_curr
            else:
                p_a, p_b = p_curr, p_next

    vec_path = p_b - p_a
    vec_vehicle = vehicle_pos - p_a

    path_length_sq = np.dot(vec_path, vec_path)
    if path_length_sq < 1e-10:
        return np.linalg.norm(vec_vehicle)

    cross_product_z = vec_path[0] * vec_vehicle[1] - vec_path[1] * vec_vehicle[0]
    distance = np.abs(cross_product_z) / np.sqrt(path_length_sq)
    cte = np.sign(cross_product_z) * distance
    
    return cte

# --- 之前的辅助函数 ---

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
    """智能路径拼接（从之前的代码复制）"""
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
    experiment_name = "autoware_cte_final_analysis"
    
    # 文件路径
    path_file = f'./path_{timestamp_str}.log'
    trajectory_file = f'./trajectory_{timestamp_str}.log'
    actual_pose_file = f'./transformed_pose_{timestamp_str}.log'
    
    # 创建目录
    result_dir = f'./result_{experiment_name}_{timestamp_str}'
    os.makedirs(result_dir, exist_ok=True)
    
    print(f"=== Final CTE Analysis with Gap Handling ===")
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
    clean_path = smart_path_concatenation(path_msgs)
    
    print("\n3. Processing trajectory data...")
    clean_traj = extract_trajectory_points_proper(traj_msgs)
    
    print("\n4. Processing pose data...")
    pose_data = extract_pose_points_batch(pose_msgs)
    
    # GAP处理选项
    print("\n5. Handling gaps in path...")
    
    # 选项1: 插值填补GAP
    interpolated_path, gap_info = interpolate_path_gaps(clean_path, max_gap=15.0)
    
    # 选项2: 分段处理
    path_segments, gaps = create_path_segments(clean_path, max_gap=20.0)
    
    print(f"Original path: {len(clean_path)} points")
    print(f"Interpolated path: {len(interpolated_path)} points")
    print(f"Path segments: {len(path_segments)} segments with {len(gaps)} gaps")
    
    # 准备分析数据
    df_actual = pd.DataFrame(pose_data)
    vehicle_positions = df_actual[['x', 'y']].values
    
    # 计算CTE
    print("\n6. Calculating CTE...")
    
    # CTE相对于原始path（有GAP）
    cte_original = []
    for pos in vehicle_positions:
        cte = calculate_cte_to_single_path(pos, clean_path)
        cte_original.append(cte)
    
    # CTE相对于插值path
    cte_interpolated = []
    for pos in vehicle_positions:
        cte = calculate_cte_to_single_path(pos, interpolated_path)
        cte_interpolated.append(cte)
    
    # CTE相对于trajectory
    cte_trajectory = []
    for pos in vehicle_positions:
        cte = calculate_cte_to_single_path(pos, clean_traj)
        cte_trajectory.append(cte)
    
    # CTE相对于分段path
    cte_segmented, segment_assignments = calculate_cte_to_path_segments(vehicle_positions, path_segments)
    
    # 整理结果
    df_actual['cte_original_path'] = cte_original
    df_actual['cte_interpolated_path'] = cte_interpolated
    df_actual['cte_trajectory'] = cte_trajectory
    df_actual['cte_segmented_path'] = cte_segmented
    df_actual['assigned_segment'] = segment_assignments
    
    # 移除无效数据
    df_clean = df_actual.dropna(subset=['cte_original_path', 'cte_interpolated_path', 'cte_trajectory'])
    
    print(f"Valid CTE data points: {len(df_clean)}")
    
    # --- 生成可视化 ---
    print("\n7. Generating visualizations...")
    plt.ioff()
    
    # 图1: 路径对比（GAP处理前后）
    plt.figure(figsize=(20, 12))
    
    plt.subplot(2, 2, 1)
    plt.plot(clean_path[:, 0], clean_path[:, 1], 'r-', linewidth=2, label='Original Path (with gaps)')
    plt.plot(df_actual['x'], df_actual['y'], 'b-', linewidth=1, alpha=0.5, label='Actual Trajectory')
    if gaps:
        for gap in gaps:
            plt.plot([gap['start_point'][0], gap['end_point'][0]], 
                    [gap['start_point'][1], gap['end_point'][1]], 
                    'orange', linewidth=3, alpha=0.7)
    plt.title('Original Path with Gaps')
    plt.xlabel('X (meters)')
    plt.ylabel('Y (meters)')
    plt.legend()
    plt.grid(True, alpha=0.3)
    plt.axis('equal')
    
    plt.subplot(2, 2, 2)
    plt.plot(interpolated_path[:, 0], interpolated_path[:, 1], 'g-', linewidth=2, label='Interpolated Path')
    plt.plot(df_actual['x'], df_actual['y'], 'b-', linewidth=1, alpha=0.5, label='Actual Trajectory')
    plt.title('Interpolated Path (gaps filled)')
    plt.xlabel('X (meters)')
    plt.ylabel('Y (meters)')
    plt.legend()
    plt.grid(True, alpha=0.3)
    plt.axis('equal')
    
    plt.subplot(2, 2, 3)
    colors = plt.cm.tab10(np.linspace(0, 1, len(path_segments)))
    for i, segment in enumerate(path_segments):
        plt.plot(segment[:, 0], segment[:, 1], color=colors[i], linewidth=2, label=f'Segment {i+1}')
    plt.plot(df_actual['x'], df_actual['y'], 'b-', linewidth=1, alpha=0.5, label='Actual Trajectory')
    plt.title('Path Segments')
    plt.xlabel('X (meters)')
    plt.ylabel('Y (meters)')
    if len(path_segments) <= 10:  # 只在段数不多时显示图例
        plt.legend()
    plt.grid(True, alpha=0.3)
    plt.axis('equal')
    
    plt.subplot(2, 2, 4)
    plt.plot(clean_traj[:, 0], clean_traj[:, 1], 'm-', linewidth=2, label='Trajectory')
    plt.plot(df_actual['x'], df_actual['y'], 'b-', linewidth=1, alpha=0.5, label='Actual Trajectory')
    plt.title('Trajectory (Local Planning)')
    plt.xlabel('X (meters)')
    plt.ylabel('Y (meters)')
    plt.legend()
    plt.grid(True, alpha=0.3)
    plt.axis('equal')
    
    plt.tight_layout()
    plt.savefig(f'{result_dir}/01_path_comparison.png', dpi=300, bbox_inches='tight')
    plt.close()
    
    # 图2: CTE柱状图对比
    fig, axes = plt.subplots(2, 2, figsize=(16, 12))
    
    cte_data = {
        'Original Path': df_clean['cte_original_path'],
        'Interpolated Path': df_clean['cte_interpolated_path'],
        'Trajectory': df_clean['cte_trajectory'],
        'Segmented Path': df_clean['cte_segmented_path']
    }
    
    colors = ['red', 'green', 'magenta', 'orange']
    
    for i, (name, cte_values) in enumerate(cte_data.items()):
        row, col = i // 2, i % 2
        
        # 移除NaN值
        valid_cte = cte_values.dropna()
        
        if len(valid_cte) > 0:
            axes[row, col].hist(valid_cte, bins=50, alpha=0.7, color=colors[i], edgecolor='black')
            axes[row, col].axvline(valid_cte.mean(), color='darkred', linestyle='--', 
                                 label=f'Mean: {valid_cte.mean():.4f}m')
            axes[row, col].set_title(f'CTE Distribution - {name}')
            axes[row, col].set_xlabel('CTE (meters)')
            axes[row, col].set_ylabel('Frequency')
            axes[row, col].legend()
            axes[row, col].grid(True, alpha=0.3)
    
    plt.tight_layout()
    plt.savefig(f'{result_dir}/02_cte_histograms.png', dpi=300, bbox_inches='tight')
    plt.close()
    
    # 图3: CTE时间序列对比
    plt.figure(figsize=(16, 10))
    
    df_clean['datetime'] = pd.to_datetime(df_clean['timestamp'], unit='s')
    df_clean = df_clean.set_index('datetime').sort_index()
    
    plt.subplot(2, 1, 1)
    for name, color in zip(['cte_original_path', 'cte_interpolated_path'], ['red', 'green']):
        valid_data = df_clean[name].dropna()
        if len(valid_data) > 0:
            plt.plot(valid_data.index, valid_data.values, color=color, linewidth=1.5, 
                    alpha=0.8, label=name.replace('cte_', '').replace('_', ' ').title())
    plt.axhline(0, color='k', linestyle='--', alpha=0.7)
    plt.title('CTE Time Series - Path Comparison')
    plt.ylabel('CTE (meters)')
    plt.legend()
    plt.grid(True, alpha=0.3)
    
    plt.subplot(2, 1, 2)
    for name, color in zip(['cte_trajectory', 'cte_segmented_path'], ['magenta', 'orange']):
        valid_data = df_clean[name].dropna()
        if len(valid_data) > 0:
            plt.plot(valid_data.index, valid_data.values, color=color, linewidth=1.5, 
                    alpha=0.8, label=name.replace('cte_', '').replace('_', ' ').title())
    plt.axhline(0, color='k', linestyle='--', alpha=0.7)
    plt.title('CTE Time Series - Trajectory vs Segmented Path')
    plt.xlabel('Time')
    plt.ylabel('CTE (meters)')
    plt.legend()
    plt.grid(True, alpha=0.3)
    
    plt.tight_layout()
    plt.savefig(f'{result_dir}/03_cte_timeseries.png', dpi=300, bbox_inches='tight')
    plt.close()
    
    # 图4: CTE统计对比
    stats_data = {}
    for name, cte_values in cte_data.items():
        valid_cte = cte_values.dropna()
        if len(valid_cte) > 0:
            stats_data[name] = {
                'Mean': valid_cte.mean(),
                'Median': valid_cte.median(),
                'Std': valid_cte.std(),
                'RMSE': np.sqrt((valid_cte**2).mean()),
                'Max Abs': valid_cte.abs().max(),
                '95th Percentile': np.percentile(valid_cte.abs(), 95)
            }
    
    # 创建统计对比图
    metrics = list(stats_data[list(stats_data.keys())[0]].keys())
    x = np.arange(len(metrics))
    width = 0.2
    
    fig, ax = plt.subplots(figsize=(15, 8))
    
    for i, (name, stats) in enumerate(stats_data.items()):
        values = [abs(stats[metric]) for metric in metrics]  # 使用绝对值便于比较
        ax.bar(x + i * width, values, width, label=name, color=colors[i], alpha=0.8)
    
    ax.set_xlabel('Metrics')
    ax.set_ylabel('Value (meters)')
    ax.set_title('CTE Statistics Comparison')
    ax.set_xticks(x + width * 1.5)
    ax.set_xticklabels(metrics, rotation=45)
    ax.legend()
    ax.grid(True, alpha=0.3)
    
    plt.tight_layout()
    plt.savefig(f'{result_dir}/04_cte_statistics_comparison.png', dpi=300, bbox_inches='tight')
    plt.close()
    
    # --- 生成统计报告 ---
    print("\n8. Generating analysis report...")
    
    report = f"""
=== Final CTE Analysis Report ===
Generated at: {datetime.now()}

Data Summary:
- Actual pose records: {len(df_actual)}
- Valid CTE calculations: {len(df_clean)}
- Original path points: {len(clean_path)}
- Interpolated path points: {len(interpolated_path)}
- Trajectory points: {len(clean_traj)}
- Path segments: {len(path_segments)}
- Detected gaps: {len(gaps)}

Gap Information:
"""
    
    if gaps:
        for i, gap in enumerate(gaps):
            report += f"Gap {i+1}: {gap['gap_distance']:.2f}m\n"
    
    report += "\nCTE Statistics Comparison:\n"
    for name, stats in stats_data.items():
        report += f"\n{name}:\n"
        for metric, value in stats.items():
            report += f"  {metric}: {value:.4f} m\n"
    
    # 推荐最佳方法
    rmse_values = {name: stats['RMSE'] for name, stats in stats_data.items()}
    best_method = min(rmse_values, key=rmse_values.get)
    
    report += f"\nRecommendation:\n"
    report += f"Based on RMSE analysis, '{best_method}' shows the best performance with RMSE = {rmse_values[best_method]:.4f}m\n"
    
    print(report)
    
    # 保存所有结果
    with open(f'{result_dir}/final_analysis_report.txt', 'w') as f:
        f.write(report)
    
    df_clean.to_csv(f'{result_dir}/complete_cte_analysis.csv')
    
    # 保存路径数据
    np.savetxt(f'{result_dir}/original_path.csv', clean_path, delimiter=',', header='x,y', comments='')
    np.savetxt(f'{result_dir}/interpolated_path.csv', interpolated_path, delimiter=',', header='x,y', comments='')
    np.savetxt(f'{result_dir}/trajectory.csv', clean_traj, delimiter=',', header='x,y', comments='')
    
    print(f"\n=== Final Analysis completed ===")
    print(f"All results saved to: {result_dir}")
    print(f"Best performing method: {best_method}")

if __name__ == '__main__':
    main()