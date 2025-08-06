import yaml
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import os
import pickle
from datetime import datetime
import time
from scipy.spatial.distance import cdist

# --- 高级数据分析和清理模块 ---

def analyze_path_structure(path_msgs):
    """深入分析path消息的结构和特征"""
    print("\n=== Deep Path Structure Analysis ===")
    
    path_info = []
    for i, msg in enumerate(path_msgs[:10]):  # 分析前10个消息
        try:
            ts_sec = msg['header']['stamp']['sec']
            ts_nsec = msg['header']['stamp']['nanosec']
            timestamp = ts_sec + ts_nsec * 1e-9
            
            points = []
            for point in msg.get('points', []):
                if isinstance(point, dict) and 'pose' in point and 'position' in point['pose']:
                    pos = point['pose']['position']
                    points.append([pos['x'], pos['y']])
            
            if points:
                points = np.array(points)
                path_length = np.sum(np.linalg.norm(np.diff(points, axis=0), axis=1))
                start_point = points[0]
                end_point = points[-1]
                
                path_info.append({
                    'msg_id': i,
                    'timestamp': timestamp,
                    'num_points': len(points),
                    'path_length': path_length,
                    'start': start_point,
                    'end': end_point
                })
                
                print(f"Message {i}: {len(points)} points, length: {path_length:.2f}m")
                print(f"  Start: ({start_point[0]:.2f}, {start_point[1]:.2f})")
                print(f"  End: ({end_point[0]:.2f}, {end_point[1]:.2f})")
                
        except Exception as e:
            print(f"Error analyzing message {i}: {e}")
    
    return path_info

def detect_path_discontinuities(points, max_gap=10.0):
    """检测路径中的不连续点"""
    if len(points) < 2:
        return []
    
    gaps = []
    distances = np.linalg.norm(np.diff(points, axis=0), axis=1)
    
    for i, dist in enumerate(distances):
        if dist > max_gap:
            gaps.append({
                'index': i,
                'distance': dist,
                'from': points[i],
                'to': points[i+1]
            })
    
    return gaps

def smart_path_concatenation(path_msgs, distance_threshold=2.0, continuity_threshold=10.0):
    """
    智能路径拼接，检测并处理各种异常情况
    """
    print(f"\n=== Smart Path Concatenation ===")
    print(f"Processing {len(path_msgs)} path messages...")
    
    # 按时间戳排序并提取所有路径段
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
            
            if len(points) > 1:  # 至少需要2个点
                points = np.array(points)
                path_segments.append({
                    'timestamp': timestamp,
                    'points': points,
                    'msg_id': i
                })
        except Exception as e:
            continue
    
    # 按时间排序
    path_segments.sort(key=lambda x: x['timestamp'])
    print(f"Found {len(path_segments)} valid path segments")
    
    if not path_segments:
        return np.array([]).reshape(0, 2)
    
    # 智能拼接
    final_path = path_segments[0]['points'].copy()
    
    for i in range(1, len(path_segments)):
        current_segment = path_segments[i]['points']
        
        # 方法1: 检查起始点是否与现有路径的末尾连续
        last_point = final_path[-1]
        first_point = current_segment[0]
        connection_distance = np.linalg.norm(last_point - first_point)
        
        print(f"Segment {i}: Connection distance = {connection_distance:.2f}m")
        
        if connection_distance <= continuity_threshold:
            # 连续路径，找到不重复的起始点
            non_duplicate_start = 0
            for j, point in enumerate(current_segment):
                # 检查与现有路径最后几个点的距离
                recent_points = final_path[-min(50, len(final_path)):]  # 检查最后50个点
                distances = np.linalg.norm(recent_points - point, axis=1)
                min_distance = np.min(distances)
                
                if min_distance > distance_threshold:
                    non_duplicate_start = j
                    break
                non_duplicate_start = j + 1
            
            # 添加非重复部分
            if non_duplicate_start < len(current_segment):
                new_points = current_segment[non_duplicate_start:]
                final_path = np.vstack([final_path, new_points])
                print(f"  Added {len(new_points)} new points (skipped {non_duplicate_start} duplicates)")
            else:
                print(f"  Segment completely overlaps, skipped")
        else:
            print(f"  Large gap detected ({connection_distance:.2f}m), checking if it's a new path section...")
            
            # 大间隔，可能是新的路径段或者错误的跳跃
            # 检查这个段是否在现有路径附近
            if len(final_path) > 0:
                distances_to_existing = cdist(current_segment, final_path)
                min_distances = np.min(distances_to_existing, axis=1)
                
                # 如果大部分点都远离现有路径，可能是合法的新段
                far_points_ratio = np.sum(min_distances > distance_threshold * 2) / len(min_distances)
                
                if far_points_ratio > 0.8:  # 80%的点都远离现有路径
                    print(f"    Adding as new path section (far points ratio: {far_points_ratio:.2f})")
                    final_path = np.vstack([final_path, current_segment])
                else:
                    print(f"    Skipping potential erroneous segment (far points ratio: {far_points_ratio:.2f})")
            else:
                final_path = np.vstack([final_path, current_segment])
        
        if (i + 1) % 50 == 0:
            print(f"  Processed {i + 1}/{len(path_segments)} segments, total points: {len(final_path)}")
    
    print(f"Final concatenated path has {len(final_path)} points")
    
    # 最后清理：移除可能的重复点
    if len(final_path) > 1:
        final_path = remove_consecutive_duplicates(final_path, distance_threshold)
        print(f"After duplicate removal: {len(final_path)} points")
    
    return final_path

def remove_consecutive_duplicates(points, threshold=1.0):
    """移除连续的重复点"""
    if len(points) <= 1:
        return points
    
    cleaned_points = [points[0]]
    
    for i in range(1, len(points)):
        distance = np.linalg.norm(points[i] - cleaned_points[-1])
        if distance > threshold:
            cleaned_points.append(points[i])
    
    return np.array(cleaned_points)

def validate_path_continuity(points, max_gap=15.0):
    """验证路径连续性并报告问题"""
    gaps = detect_path_discontinuities(points, max_gap)
    
    if gaps:
        print(f"\n=== Path Continuity Issues ===")
        print(f"Found {len(gaps)} discontinuities:")
        for gap in gaps:
            print(f"  Gap at index {gap['index']}: {gap['distance']:.2f}m")
            print(f"    From: ({gap['from'][0]:.2f}, {gap['from'][1]:.2f})")
            print(f"    To: ({gap['to'][0]:.2f}, {gap['to'][1]:.2f})")
    else:
        print("Path continuity validation: PASSED")
    
    return gaps

# --- 主分析流程（改进版） ---

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

def main():
    # --- 配置 ---
    timestamp_str = "20250425_163228"
    experiment_name = "autoware_cte_deep_analysis"
    
    # 文件路径
    path_file = f'./path_{timestamp_str}.log'
    trajectory_file = f'./trajectory_{timestamp_str}.log'
    actual_pose_file = f'./transformed_pose_{timestamp_str}.log'
    
    # 创建目录
    result_dir = f'./result_{experiment_name}_{timestamp_str}'
    cache_dir = f'./cache_{timestamp_str}'
    os.makedirs(result_dir, exist_ok=True)
    os.makedirs(cache_dir, exist_ok=True)
    
    print(f"=== Deep Analysis started at {datetime.now()} ===")
    print(f"Experiment: {experiment_name}")
    
    # 加载数据
    print("\n1. Loading path data for deep analysis...")
    path_msgs = parse_yaml_log_file_fast(path_file)
    if not path_msgs:
        return
    
    print("\n2. Loading pose data...")
    actual_pose_msgs = parse_yaml_log_file_fast(actual_pose_file)
    if not actual_pose_msgs:
        return
    
    # 深入分析path结构
    path_info = analyze_path_structure(path_msgs)
    
    # 智能拼接
    print("\n3. Performing smart path concatenation...")
    path_points = smart_path_concatenation(
        path_msgs, 
        distance_threshold=1.5,  # 重复点阈值
        continuity_threshold=8.0  # 连续性阈值
    )
    
    # 验证路径连续性
    gaps = validate_path_continuity(path_points, max_gap=12.0)
    
    # 处理pose数据
    pose_data = extract_pose_points_batch(actual_pose_msgs)
    
    # --- 可视化分析结果 ---
    print("\n4. Generating detailed visualizations...")
    plt.ioff()
    
    df_actual = pd.DataFrame(pose_data)
    
    # 图1: 路径分析图
    plt.figure(figsize=(15, 12))
    
    # 绘制清理后的路径
    if len(path_points) > 0:
        plt.plot(path_points[:, 0], path_points[:, 1], 'r-', linewidth=2, label='Cleaned Path', alpha=0.8)
        
        # 标记不连续点
        if gaps:
            for gap in gaps:
                plt.plot([gap['from'][0], gap['to'][0]], [gap['from'][1], gap['to'][1]], 
                        'orange', linewidth=3, alpha=0.7, label='Detected Gap' if gap == gaps[0] else "")
                plt.scatter(*gap['from'], color='red', s=50, zorder=5)
                plt.scatter(*gap['to'], color='red', s=50, zorder=5)
    
    # 绘制实际轨迹
    plt.plot(df_actual['x'], df_actual['y'], 'b-', linewidth=1, alpha=0.7, label='Actual Trajectory')
    
    plt.title('Deep Path Analysis - Cleaned and Validated')
    plt.xlabel('X (meters)')
    plt.ylabel('Y (meters)')
    plt.legend()
    plt.grid(True, alpha=0.3)
    plt.axis('equal')
    plt.tight_layout()
    plt.savefig(f'{result_dir}/path_deep_analysis.png', dpi=300, bbox_inches='tight')
    plt.close()
    
    # 图2: 路径质量报告图
    if len(path_points) > 1:
        distances = np.linalg.norm(np.diff(path_points, axis=0), axis=1)
        
        plt.figure(figsize=(15, 8))
        
        plt.subplot(2, 1, 1)
        plt.plot(distances, 'b-', linewidth=1)
        plt.axhline(y=8.0, color='r', linestyle='--', alpha=0.7, label='Continuity Threshold')
        plt.title('Point-to-Point Distances in Path')
        plt.ylabel('Distance (meters)')
        plt.legend()
        plt.grid(True, alpha=0.3)
        
        plt.subplot(2, 1, 2)
        plt.hist(distances, bins=50, alpha=0.7, edgecolor='black')
        plt.axvline(x=8.0, color='r', linestyle='--', alpha=0.7, label='Continuity Threshold')
        plt.title('Distribution of Point-to-Point Distances')
        plt.xlabel('Distance (meters)')
        plt.ylabel('Frequency')
        plt.legend()
        plt.grid(True, alpha=0.3)
        
        plt.tight_layout()
        plt.savefig(f'{result_dir}/path_quality_analysis.png', dpi=300, bbox_inches='tight')
        plt.close()
    
    # 保存分析结果
    analysis_report = f"""
=== Deep Path Analysis Report ===
Generated at: {datetime.now()}

Path Statistics:
- Total messages processed: {len(path_msgs)}
- Final path points: {len(path_points)}
- Detected discontinuities: {len(gaps)}

Path Quality Metrics:
"""
    
    if len(path_points) > 1:
        distances = np.linalg.norm(np.diff(path_points, axis=0), axis=1)
        analysis_report += f"""
- Average point spacing: {np.mean(distances):.3f} m
- Max point spacing: {np.max(distances):.3f} m
- Min point spacing: {np.min(distances):.3f} m
- Points with large gaps (>8m): {np.sum(distances > 8.0)}
"""
    
    if gaps:
        analysis_report += f"\nDetected Discontinuities:\n"
        for i, gap in enumerate(gaps):
            analysis_report += f"Gap {i+1}: {gap['distance']:.2f}m at index {gap['index']}\n"
    
    print(analysis_report)
    
    # 保存结果
    with open(f'{result_dir}/deep_analysis_report.txt', 'w') as f:
        f.write(analysis_report)
    
    if len(path_points) > 0:
        np.savetxt(f'{result_dir}/cleaned_path_points.csv', path_points, delimiter=',', header='x,y', comments='')
    
    df_actual.to_csv(f'{result_dir}/pose_data.csv', index=False)
    
    print(f"\n=== Deep Analysis completed ===")
    print(f"Results saved to: {result_dir}")

if __name__ == '__main__':
    main()