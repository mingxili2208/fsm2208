"""
Industrial-Grade Data Processing Script with Physical Accuracy
Philosophy: Robust filtering -> Physical modeling -> Meaningful coordinates
"""
import numpy as np
import matplotlib.pyplot as plt
import plotly.graph_objects as go
import plotly.express as px
from plotly.subplots import make_subplots
from scipy.signal import savgol_filter, medfilt
from scipy.fft import fft, fftfreq
import pandas as pd
import os
import json
import argparse
from datetime import datetime
import warnings
import logging
from enum import Enum
warnings.filterwarnings('ignore')

class MotionState(Enum):
    MOVING = "moving"
    STATIONARY = "stationary"

# 全局色彩规范
COLOR_SCHEME = {
    'world_final_filtered': '#E74C3C',      # 红色 - World/Final/Filtered数据
    'tracker_sync_raw': '#3498DB',          # 蓝色 - Tracker/Synchronized/Raw数据
    'residuals_differences': '#27AE60',     # 绿色 - 残差/差异
    'interpolation_linear': '#F39C12',      # 橙色 - 线性插值
    'interpolation_nearest': '#9B59B6',     # 紫色 - 最近邻插值
    'tolerance_line': '#E67E22',            # 橙红色 - 容差线
    'statistics_background': '#ECF0F1',     # 浅灰色 - 统计背景
    'median_filtered': '#8E44AD'            # 紫色 - 中值滤波结果
}

class IndustrialDataProcessor:
    def __init__(self, raw_data_file, config_file="process_config.json"):
        self.raw_data_file = raw_data_file
        self.config = self.load_config(config_file)
        self.results = {}
        self.processing_stats = {}
        
        # 创建带时间戳的输出目录
        self.output_timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.run_output_dir = os.path.join(
            self.config["output"]["output_dir"], 
            f"run_{self.output_timestamp}"
        )
        os.makedirs(self.run_output_dir, exist_ok=True)
        
        # 设置日志
        self.setup_logging()
        
        # 加载并验证原始数据
        self.load_raw_data()
        
    def setup_logging(self):
        """设置详细的日志记录"""
        log_file = os.path.join(self.run_output_dir, "processing.log")
        logging.basicConfig(
            level=logging.INFO,
            format='%(asctime)s - %(levelname)s - %(message)s',
            handlers=[
                logging.FileHandler(log_file),
                logging.StreamHandler()
            ]
        )
        self.logger = logging.getLogger(__name__)
        self.logger.info(f"Processing session started: {self.output_timestamp}")
        
    def load_config(self, config_file):
        """加载增强的处理配置"""
        default_config = {
            "preprocessing": {
                "enable_stationary_filter": True,
                "stationary_window_size": 10,
                "stationary_std_threshold": 0.001,  # meters
                "stationary_detection_channels": ["distance_2", "distance_3"],
                "enable_median_filter": True,
                "median_filter_kernel_size": 15,  # 增大核大小
                "enable_outlier_detection": True,
                "outlier_detection_method": "iqr",  # 'iqr' or 'zscore'
                "iqr_multiplier": 2.0,
                "zscore_threshold": 3.0
            },
            "synchronization": {
                "interpolation_tolerance_ms": 100,
                "outlier_detection": True,
                "outlier_threshold_sigma": 3.0,
                "disable_angle_interpolation": True
            },
            "pose_filtering": {
                "enable_pose_stability_filter": True,
                "roll_fluctuation_threshold": 2.0,  # degrees
                "pitch_fluctuation_threshold": 2.0,  # degrees
                "yaw_fluctuation_threshold": 5.0,   # degrees
                "histogram_bins": 50
            },
            "segmentation": {
                "max_time_gap_ms": 150,
                "min_segment_length": 10
            },
            "filtering": {
                "sg_window_length": 7,
                "sg_polyorder": 2,
                "auto_optimize_parameters": True
            },
            "physical_correction": {
                "enable_physical_correction": True,
                "physical_offset": 0.05,  # meters
                "world_target_x": 4.250,  # meters
                "world_target_z": 1.660,  # meters
                "apply_angle_projection": True
            },
            "visualization": {
                "generate_interactive_plots": True,
                "generate_static_plots": True,
                "plot_3d_trajectories": True,
                "show_segment_boundaries": True,
                "generate_residual_histogram": True,
                "generate_fft_analysis": True,
                "residual_moving_average_window": 50
            },
            "output": {
                "save_synchronized_csv": True,
                "save_filtered_csv": True,
                "save_final_coordinates_csv": True,
                "output_dir": "processed_data"
            }
        }
        
        try:
            with open(config_file, 'r') as f:
                user_config = json.load(f)
            
            def deep_update(base, update):
                for key, value in update.items():
                    if key in base and isinstance(base[key], dict) and isinstance(value, dict):
                        deep_update(base[key], value)
                    else:
                        base[key] = value
            
            deep_update(default_config, user_config)
        except FileNotFoundError:
            self.logger.warning(f"Config file {config_file} not found, using defaults")
            with open(config_file, 'w') as f:
                json.dump(default_config, f, indent=2)
        
        return default_config
    
    def load_raw_data(self):
        """加载并验证原始数据"""
        self.logger.info(f"Loading raw data from: {self.raw_data_file}")
        
        try:
            data = np.load(self.raw_data_file, allow_pickle=True)
            self.lidar_data = data['lidar_data']
            self.tracker_data = data['tracker_data']
            self.metadata = data['metadata'].item()
            
            self.logger.info(f"Loaded {len(self.lidar_data)} LiDAR samples")
            self.logger.info(f"Loaded {len(self.tracker_data)} Tracker samples")
            self.logger.info(f"Collection duration: {self.metadata.get('duration_seconds', 0):.1f}s")
            
        except Exception as e:
            self.logger.error(f"Error loading raw data: {e}")
            raise
    
    def preprocess_lidar_data(self):
        """步骤一：基于状态机的LiDAR静止数据预过滤"""
        if not self.config["preprocessing"]["enable_stationary_filter"]:
            self.logger.info("Stationary filtering disabled, skipping preprocessing")
            return self.lidar_data
        
        self.logger.info("Starting LiDAR stationary data preprocessing...")
        
        window_size = self.config["preprocessing"]["stationary_window_size"]
        std_threshold = self.config["preprocessing"]["stationary_std_threshold"]
        channels = self.config["preprocessing"]["stationary_detection_channels"]
        
        # 初始化状态机
        state = MotionState.MOVING
        filtered_indices = []
        
        total_points = len(self.lidar_data)
        stationary_segments = 0
        points_removed = 0
        
        for i in range(0, total_points - window_size + 1, window_size):
            window_end = min(i + window_size, total_points)
            window_data = self.lidar_data[i:window_end]
            
            # 计算窗口内距离数据的标准差
            window_std = 0
            for channel in channels:
                if channel in window_data.dtype.names:
                    channel_std = np.std(window_data[channel])
                    window_std = max(window_std, channel_std)
            
            is_stationary = window_std < std_threshold
            
            if state == MotionState.MOVING:
                if is_stationary:
                    # 转换到静止状态，保留第一个点
                    filtered_indices.append(i)
                    state = MotionState.STATIONARY
                    stationary_segments += 1
                    self.logger.debug(f"Stationary segment {stationary_segments} started at index {i}")
                else:
                    # 保持移动状态，保留所有点
                    filtered_indices.extend(range(i, window_end))
            
            elif state == MotionState.STATIONARY:
                if is_stationary:
                    # 保持静止状态，丢弃所有点
                    points_removed += (window_end - i)
                    self.logger.debug(f"Removed {window_end - i} stationary points at index {i}")
                else:
                    # 转换到移动状态，保留所有点
                    filtered_indices.extend(range(i, window_end))
                    state = MotionState.MOVING
                    self.logger.debug(f"Motion resumed at index {i}")
        
        # 处理剩余的点
        if len(filtered_indices) > 0:
            last_processed = max(filtered_indices) + 1
            if last_processed < total_points:
                if state == MotionState.MOVING:
                    filtered_indices.extend(range(last_processed, total_points))
                else:
                    points_removed += (total_points - last_processed)
        
        # 生成过滤后的数据
        filtered_indices = sorted(set(filtered_indices))
        filtered_lidar_data = self.lidar_data[filtered_indices]
        
        reduction_ratio = (1 - len(filtered_lidar_data) / len(self.lidar_data)) * 100
        
        self.logger.info(f"Preprocessing completed:")
        self.logger.info(f"  Original points: {len(self.lidar_data)}")
        self.logger.info(f"  Filtered points: {len(filtered_lidar_data)}")
        self.logger.info(f"  Points removed: {points_removed}")
        self.logger.info(f"  Data reduction: {reduction_ratio:.1f}%")
        self.logger.info(f"  Stationary segments detected: {stationary_segments}")
        
        self.processing_stats['preprocessing'] = {
            'original_points': len(self.lidar_data),
            'filtered_points': len(filtered_lidar_data),
            'points_removed': points_removed,
            'reduction_ratio': reduction_ratio,
            'stationary_segments': stationary_segments
        }
        
        return filtered_lidar_data
    
    def synchronize_data(self, preprocessed_lidar_data):
        """步骤二：改进的数据同步（禁用角度插值）"""
        self.logger.info("Starting improved data synchronization...")
        
        lidar_times = preprocessed_lidar_data['timestamp']
        tracker_times = self.tracker_data['timestamp']
        
        # 找到重叠区域
        start_time = max(lidar_times[0], tracker_times[0])
        end_time = min(lidar_times[-1], tracker_times[-1])
        
        # 过滤LiDAR数据到重叠区域
        lidar_mask = (lidar_times >= start_time) & (lidar_times <= end_time)
        lidar_subset = preprocessed_lidar_data[lidar_mask]
        
        synchronized_data = []
        interpolation_stats = {'success': 0, 'failed': 0, 'outliers_removed': 0}
        tolerance = self.config["synchronization"]["interpolation_tolerance_ms"] / 1000.0
        
        # 保存插值信息用于可视化
        self.interpolation_info = []
        
        for lidar_point in lidar_subset:
            t_lidar = lidar_point['timestamp']
            
            # 找到最近的Tracker数据点（用于角度值）
            time_diffs = np.abs(tracker_times - t_lidar)
            closest_tracker_idx = np.argmin(time_diffs)
            
            # 检查时间容差
            if time_diffs[closest_tracker_idx] > tolerance:
                interpolation_stats['failed'] += 1
                continue
            
            # 对于位置数据，仍然使用插值
            interpolation_type = "nearest"  # 默认为最近邻
            alpha = 0
            if closest_tracker_idx == 0 or closest_tracker_idx == len(tracker_times) - 1:
                # 边界情况，直接使用最近点的所有数据
                closest_tracker = self.tracker_data[closest_tracker_idx]
                interpolated_pose = {
                    'x': closest_tracker['x'],
                    'y': closest_tracker['y'],
                    'z': closest_tracker['z'],
                    'roll': closest_tracker['roll'],
                    'pitch': closest_tracker['pitch'],
                    'yaw': closest_tracker['yaw']
                }
            else:
                # 确定插值点
                if tracker_times[closest_tracker_idx] <= t_lidar:
                    idx_before, idx_after = closest_tracker_idx, closest_tracker_idx + 1
                else:
                    idx_before, idx_after = closest_tracker_idx - 1, closest_tracker_idx
                
                t_before = tracker_times[idx_before]
                t_after = tracker_times[idx_after]
                
                # 线性插值权重
                if t_after != t_before:
                    alpha = (t_lidar - t_before) / (t_after - t_before)
                    interpolation_type = "linear"
                else:
                    alpha = 0
                
                tracker_before = self.tracker_data[idx_before]
                tracker_after = self.tracker_data[idx_after]
                
                # 对位置数据进行插值，角度数据使用最近值
                if self.config["synchronization"]["disable_angle_interpolation"]:
                    # 使用最近的角度值，不进行插值
                    closest_tracker = self.tracker_data[closest_tracker_idx]
                    interpolated_pose = {
                        'x': tracker_before['x'] + alpha * (tracker_after['x'] - tracker_before['x']),
                        'y': tracker_before['y'] + alpha * (tracker_after['y'] - tracker_before['y']),
                        'z': tracker_before['z'] + alpha * (tracker_after['z'] - tracker_before['z']),
                        'roll': closest_tracker['roll'],
                        'pitch': closest_tracker['pitch'],
                        'yaw': closest_tracker['yaw']
                    }
                else:
                    # 传统插值方法
                    interpolated_pose = {}
                    for field in ['x', 'y', 'z', 'roll', 'pitch', 'yaw']:
                        interpolated_pose[field] = tracker_before[field] + alpha * (tracker_after[field] - tracker_before[field])
            
            # 保存插值信息
            self.interpolation_info.append({
                'lidar_timestamp': t_lidar,
                'interpolation_type': interpolation_type,
                'alpha': alpha,
                'time_diff': time_diffs[closest_tracker_idx]
            })
            
            # 异常值检测
            if self.config["synchronization"]["outlier_detection"]:
                distances = [lidar_point['distance_2'], lidar_point['distance_3']]
                if self.is_outlier(distances):
                    interpolation_stats['outliers_removed'] += 1
                    continue
            
            # 存储同步点
            sync_point = {
                'timestamp': t_lidar,
                'distance_2': lidar_point['distance_2'],
                'distance_3': lidar_point['distance_3'],
                'x': interpolated_pose['x'],
                'y': interpolated_pose['y'],
                'z': interpolated_pose['z'],
                'roll': interpolated_pose['roll'],
                'pitch': interpolated_pose['pitch'],
                'yaw': interpolated_pose['yaw']
            }
            synchronized_data.append(sync_point)
            interpolation_stats['success'] += 1
        
        success_rate = interpolation_stats['success'] / (interpolation_stats['success'] + interpolation_stats['failed']) * 100 if (interpolation_stats['success'] + interpolation_stats['failed']) > 0 else 0
        
        self.logger.info(f"Synchronization completed:")
        self.logger.info(f"  Success: {interpolation_stats['success']} points")
        self.logger.info(f"  Failed: {interpolation_stats['failed']} points")
        self.logger.info(f"  Outliers removed: {interpolation_stats['outliers_removed']} points")
        self.logger.info(f"  Success rate: {success_rate:.1f}%")
        
        self.processing_stats['synchronization'] = interpolation_stats
        return synchronized_data
    
    def detect_outliers_iqr(self, data, multiplier=1.5):
        """使用IQR方法检测异常值"""
        Q1 = np.percentile(data, 25)
        Q3 = np.percentile(data, 75)
        IQR = Q3 - Q1
        lower_bound = Q1 - multiplier * IQR
        upper_bound = Q3 + multiplier * IQR
        return (data < lower_bound) | (data > upper_bound)
    
    def detect_outliers_zscore(self, data, threshold=3.0):
        """使用Z-score方法检测异常值"""
        z_scores = np.abs((data - np.mean(data)) / np.std(data))
        return z_scores > threshold
    
    def apply_median_filter(self, data_list):
        """增强的中值滤波器，包含异常值检测和多种策略"""
        if not self.config["preprocessing"]["enable_median_filter"]:
            self.logger.info("Median filtering disabled, skipping.")
            return data_list, None

        self.logger.info("Applying Enhanced Median Filter to remove spikes...")
        
        if len(data_list) == 0:
            return data_list, None

        kernel_size = self.config["preprocessing"]["median_filter_kernel_size"]
        if kernel_size % 2 == 0:
            kernel_size += 1  # 确保核大小为奇数
            self.logger.warning(f"Median filter kernel size must be odd. Adjusted to {kernel_size}.")

        # 转换为DataFrame便于处理
        df_original = pd.DataFrame(data_list)
        df_filtered = df_original.copy()
        
        # 对关键时序数据应用中值滤波和异常值检测
        cols_to_filter = ['distance_2', 'distance_3', 'x', 'y', 'z']
        
        median_stats = {}
        median_comparison_data = {}
        
        for col in cols_to_filter:
            if col in df_filtered.columns:
                original_data = df_filtered[col].values
                original_std = np.std(original_data)
                
                # 第一步：异常值检测
                outliers_removed = 0
                if self.config["preprocessing"]["enable_outlier_detection"]:
                    method = self.config["preprocessing"]["outlier_detection_method"]
                    
                    if method == "iqr":
                        multiplier = self.config["preprocessing"]["iqr_multiplier"]
                        outlier_mask = self.detect_outliers_iqr(original_data, multiplier)
                        self.logger.info(f"  - {col}: IQR method (multiplier={multiplier}) detected {np.sum(outlier_mask)} outliers")
                    else:  # zscore
                        threshold = self.config["preprocessing"]["zscore_threshold"]
                        outlier_mask = self.detect_outliers_zscore(original_data, threshold)
                        self.logger.info(f"  - {col}: Z-score method (threshold={threshold}) detected {np.sum(outlier_mask)} outliers")
                    
                    # 替换异常值为邻近值的中位数
                    if np.any(outlier_mask):
                        outlier_indices = np.where(outlier_mask)[0]
                        for idx in outlier_indices:
                            # 使用周围点的中位数替换异常值
                            start_idx = max(0, idx - kernel_size//2)
                            end_idx = min(len(original_data), idx + kernel_size//2 + 1)
                            window_data = original_data[start_idx:end_idx]
                            # 排除当前异常点
                            window_data = window_data[window_data != original_data[idx]]
                            if len(window_data) > 0:
                                df_filtered.loc[idx, col] = np.median(window_data)
                                outliers_removed += 1
                
                # 第二步：中值滤波
                filtered_data = medfilt(df_filtered[col].values, kernel_size=kernel_size)
                df_filtered[col] = filtered_data
                filtered_std = np.std(filtered_data)
                
                # 计算去除的尖峰数量（基于原始数据和最终滤波数据的差异）
                diff = np.abs(original_data - filtered_data)
                significant_changes = np.sum(diff > 2 * original_std)
                
                median_stats[col] = {
                    'original_std': original_std,
                    'filtered_std': filtered_std,
                    'outliers_detected': np.sum(outlier_mask) if self.config["preprocessing"]["enable_outlier_detection"] else 0,
                    'outliers_replaced': outliers_removed,
                    'significant_changes': significant_changes,
                    'max_change': np.max(diff),
                    'improvement_pct': (original_std - filtered_std) / original_std * 100 if original_std > 0 else 0
                }
                
                # 保存对比数据用于可视化
                median_comparison_data[col] = {
                    'original': original_data,
                    'filtered': filtered_data,
                    'timestamps': df_original['timestamp'].values if 'timestamp' in df_original.columns else np.arange(len(original_data))
                }
                
                self.logger.info(f"  - {col}: Outliers replaced: {outliers_removed}, "
                               f"Std dev: {original_std:.4f} -> {filtered_std:.4f} "
                               f"({median_stats[col]['improvement_pct']:.1f}% improvement), "
                               f"Significant changes: {significant_changes}")

        # 保存中值滤波统计和对比数据
        self.processing_stats['median_filtering'] = median_stats
        self.median_comparison_data = median_comparison_data

        # 将DataFrame转换回字典列表
        return df_filtered.to_dict('records'), median_comparison_data
    
    def generate_median_filter_effect_plot(self):
        """生成中值滤波效果对比图"""
        if not hasattr(self, 'median_comparison_data') or not self.median_comparison_data:
            self.logger.warning("No median filter comparison data available")
            return
        
        # 选择要显示的列
        cols_to_show = ['distance_2', 'distance_3', 'x', 'z']  # 选择最重要的几个
        available_cols = [col for col in cols_to_show if col in self.median_comparison_data]
        
        if not available_cols:
            self.logger.warning("No data available for median filter visualization")
            return
        
        n_cols = len(available_cols)
        fig, axes = plt.subplots(n_cols, 1, figsize=(16, 4*n_cols))
        if n_cols == 1:
            axes = [axes]
        
        fig.suptitle('Median Filter Effects Analysis', fontsize=16, fontweight='bold')
        
        for i, col in enumerate(available_cols):
            data = self.median_comparison_data[col]
            timestamps = data['timestamps']
            original = data['original']
            filtered = data['filtered']
            
            # 时间归一化
            if 'timestamp' in str(type(timestamps[0])):
                time_norm = timestamps - timestamps[0]
            else:
                time_norm = timestamps
            
            # 原始数据
            axes[i].plot(time_norm, original * 1000, color=COLOR_SCHEME['tracker_sync_raw'], 
                        linewidth=1, alpha=0.7, label='Original Data')
            
            # 中值滤波后数据
            axes[i].plot(time_norm, filtered * 1000, color=COLOR_SCHEME['median_filtered'], 
                        linewidth=2, label='Median Filtered')
            
            # 标记显著变化的点
            diff = np.abs(original - filtered)
            if col in self.processing_stats.get('median_filtering', {}):
                stats = self.processing_stats['median_filtering'][col]
                threshold = 2 * stats['original_std']
                significant_mask = diff > threshold
                
                if np.any(significant_mask):
                    axes[i].scatter(time_norm[significant_mask], 
                                   original[significant_mask] * 1000,
                                   color=COLOR_SCHEME['world_final_filtered'], 
                                   s=30, alpha=0.8, label='Corrected Points', zorder=5)
            
            # 添加差异阴影
            axes[i].fill_between(time_norm, original * 1000, filtered * 1000,
                                alpha=0.3, color=COLOR_SCHEME['residuals_differences'],
                                label='Correction Amount')
            
            axes[i].set_ylabel(f'{col.title()} (mm)' if 'distance' in col else f'{col.title()} (mm)')
            axes[i].set_title(f'{col.title()} - Median Filter Effect')
            axes[i].legend()
            axes[i].grid(True, alpha=0.3)
            
            # 添加统计信息
            if col in self.processing_stats.get('median_filtering', {}):
                stats = self.processing_stats['median_filtering'][col]
                stats_text = f'Improvement: {stats["improvement_pct"]:.1f}%\n'
                stats_text += f'Max Change: {stats["max_change"]*1000:.2f}mm\n'
                stats_text += f'Points Corrected: {stats["significant_changes"]}'
                
                axes[i].text(0.02, 0.98, stats_text, transform=axes[i].transAxes,
                           verticalalignment='top', bbox=dict(boxstyle='round', 
                           facecolor=COLOR_SCHEME['statistics_background'], alpha=0.8))
            
            if i == n_cols - 1:
                axes[i].set_xlabel('Time (s)' if 'timestamp' in str(type(timestamps[0])) else 'Sample Index')
        
        plt.tight_layout()
        
        # 保存图片
        plot_file = os.path.join(self.run_output_dir, "median_filter_effects.png")
        plt.savefig(plot_file, dpi=300, bbox_inches='tight')
        plt.close()
        
        self.logger.info(f"Median filter effects plot saved to: {plot_file}")
    
    def filter_by_pose_stability(self, synchronized_data):
        """步骤三：基于Tracker姿态稳定性的全局点对筛选"""
        if not self.config["pose_filtering"]["enable_pose_stability_filter"]:
            self.logger.info("Pose stability filtering disabled, skipping")
            return synchronized_data
        
        self.logger.info("Starting pose stability filtering...")
        
        if len(synchronized_data) == 0:
            self.logger.warning("No synchronized data for pose filtering")
            return synchronized_data
        
        # 提取角度数据
        angles = {
            'roll': np.array([p['roll'] for p in synchronized_data]),
            'pitch': np.array([p['pitch'] for p in synchronized_data]),
            'yaw': np.array([p['yaw'] for p in synchronized_data])
        }
        
        # 转换为度数
        for angle_name in angles:
            angles[angle_name] = np.degrees(angles[angle_name])
        
        # 获取阈值
        thresholds = {
            'roll': self.config["pose_filtering"]["roll_fluctuation_threshold"],
            'pitch': self.config["pose_filtering"]["pitch_fluctuation_threshold"],
            'yaw': self.config["pose_filtering"]["yaw_fluctuation_threshold"]
        }
        
        bins = self.config["pose_filtering"]["histogram_bins"]
        
        # 分析每个角度的分布
        stable_ranges = {}
        filter_masks = {}
        
        for angle_name, angle_data in angles.items():
            # 计算直方图
            hist, bin_edges = np.histogram(angle_data, bins=bins)
            
            # 找到最频繁的箱子
            max_count_idx = np.argmax(hist)
            dominant_range = (bin_edges[max_count_idx], bin_edges[max_count_idx + 1])
            range_width = dominant_range[1] - dominant_range[0]
            
            self.logger.info(f"{angle_name.capitalize()} angle analysis:")
            self.logger.info(f"  Range: {np.min(angle_data):.2f}° to {np.max(angle_data):.2f}°")
            self.logger.info(f"  Dominant range: {dominant_range[0]:.2f}° to {dominant_range[1]:.2f}°")
            self.logger.info(f"  Range width: {range_width:.2f}°")
            self.logger.info(f"  Threshold: {thresholds[angle_name]:.2f}°")
            
            if range_width <= thresholds[angle_name]:
                # 该轴稳定，应用筛选
                mask = (angle_data >= dominant_range[0]) & (angle_data <= dominant_range[1])
                filter_masks[angle_name] = mask
                stable_ranges[angle_name] = dominant_range
                self.logger.info(f"  -> Applying filter (range width within threshold)")
            else:
                # 该轴不稳定，不应用筛选
                filter_masks[angle_name] = np.ones(len(angle_data), dtype=bool)
                stable_ranges[angle_name] = None
                self.logger.info(f"  -> Skipping filter (range width exceeds threshold)")
        
        # 组合所有筛选条件
        combined_mask = np.ones(len(synchronized_data), dtype=bool)
        for angle_name, mask in filter_masks.items():
            combined_mask &= mask
        
        # 应用筛选
        filtered_data = [synchronized_data[i] for i in range(len(synchronized_data)) if combined_mask[i]]
        
        points_removed = len(synchronized_data) - len(filtered_data)
        removal_ratio = points_removed / len(synchronized_data) * 100 if len(synchronized_data) > 0 else 0
        
        self.logger.info(f"Pose stability filtering completed:")
        self.logger.info(f"  Original points: {len(synchronized_data)}")
        self.logger.info(f"  Filtered points: {len(filtered_data)}")
        self.logger.info(f"  Points removed: {points_removed}")
        self.logger.info(f"  Removal ratio: {removal_ratio:.1f}%")
        
        self.processing_stats['pose_filtering'] = {
            'original_points': len(synchronized_data),
            'filtered_points': len(filtered_data),
            'points_removed': points_removed,
            'removal_ratio': removal_ratio,
            'stable_ranges': stable_ranges
        }
        
        return filtered_data
    
    def is_outlier(self, values):
        """简单的异常值检测"""
        threshold = self.config["synchronization"]["outlier_threshold_sigma"]
        if len(values) < 2:
            return False
        
        mean_val = np.mean(values)
        std_val = np.std(values)
        
        if std_val == 0:
            return False
        
        z_scores = [(val - mean_val) / std_val for val in values]
        return any(abs(z) > threshold for z in z_scores)
    
    def segment_data(self, data):
        """数据分段"""
        self.logger.info("Segmenting data...")
        
        if len(data) < 2:
            segments = [data] if data else []
            self.processing_stats['segmentation'] = {
                'segment_count': len(segments),
                'total_points': len(data)
            }
            return segments
        
        segments = []
        current_segment = [data[0]]
        gap_threshold = self.config["segmentation"]["max_time_gap_ms"] / 1000.0
        min_length = self.config["segmentation"]["min_segment_length"]
        
        for i in range(1, len(data)):
            time_gap = data[i]['timestamp'] - data[i-1]['timestamp']
            
            if time_gap > gap_threshold:
                if len(current_segment) >= min_length:
                    segments.append(current_segment)
                    self.logger.info(f"  Segment {len(segments)}: {len(current_segment)} points, duration {current_segment[-1]['timestamp'] - current_segment[0]['timestamp']:.1f}s")
                current_segment = [data[i]]
            else:
                current_segment.append(data[i])
        
        if len(current_segment) >= min_length:
            segments.append(current_segment)
            self.logger.info(f"  Segment {len(segments)}: {len(current_segment)} points, duration {current_segment[-1]['timestamp'] - current_segment[0]['timestamp']:.1f}s")
        
        total_points = sum(len(segment) for segment in segments)
        
        self.logger.info(f"Created {len(segments)} segments with total {total_points} points")
        
        # 确保统计数据是字典格式
        self.processing_stats['segmentation'] = {
            'segment_count': len(segments),
            'total_points': total_points,
            'average_segment_length': total_points / len(segments) if len(segments) > 0 else 0
        }
        
        return segments
    
    def apply_sg_filtering(self, segments):
        """应用Savitzky-Golay滤波器"""
        self.logger.info("Applying Savitzky-Golay filtering...")
        
        filtered_segments = []
        filter_stats = []
        
        # 保存原始数据以便可视化对比
        self.sg_comparison_data = []
        
        for seg_idx, segment in enumerate(segments):
            if len(segment) < 5:
                self.logger.info(f"  Segment {seg_idx}: Too short ({len(segment)} points), skipping")
                continue
            
            # 优化参数
            if self.config["filtering"]["auto_optimize_parameters"]:
                window_length, polyorder = self.optimize_sg_parameters(segment)
                self.logger.info(f"  Segment {seg_idx}: Optimized parameters - window={window_length}, poly={polyorder}")
            else:
                window_length = self.config["filtering"]["sg_window_length"]
                polyorder = self.config["filtering"]["sg_polyorder"]
            
            # 确保参数有效
            if window_length >= len(segment):
                window_length = len(segment) - 1 if len(segment) % 2 == 0 else len(segment) - 2
            if window_length % 2 == 0:
                window_length -= 1
            if polyorder >= window_length:
                polyorder = window_length - 1
            
            if window_length < 3 or polyorder < 1:
                self.logger.info(f"  Segment {seg_idx}: Invalid parameters after adjustment, skipping")
                continue
            
            # 提取数据数组
            timestamps = np.array([point['timestamp'] for point in segment])
            d2_raw = np.array([point['distance_2'] for point in segment])
            d3_raw = np.array([point['distance_3'] for point in segment])
            poses_raw = np.array([[point['x'], point['y'], point['z'], 
                                 point['roll'], point['pitch'], point['yaw']] for point in segment])
            
            try:
                # 应用SG滤波器
                d2_filtered = savgol_filter(d2_raw, window_length, polyorder)
                d3_filtered = savgol_filter(d3_raw, window_length, polyorder)
                
                poses_filtered = np.zeros_like(poses_raw)
                for i in range(poses_raw.shape[1]):
                    poses_filtered[:, i] = savgol_filter(poses_raw[:, i], window_length, polyorder)
                
                # 保存对比数据
                time_offset = timestamps[0]
                self.sg_comparison_data.append({
                    'segment_id': seg_idx,
                    'timestamps': timestamps - time_offset,
                    'd2_raw': d2_raw,
                    'd2_filtered': d2_filtered,
                    'd3_raw': d3_raw,
                    'd3_filtered': d3_filtered,
                    'window_length': window_length,
                    'polyorder': polyorder
                })
                
                # 计算滤波统计
                d2_improvement = (np.std(d2_raw) - np.std(d2_filtered)) / np.std(d2_raw) * 100 if np.std(d2_raw) > 0 else 0
                d3_improvement = (np.std(d3_raw) - np.std(d3_filtered)) / np.std(d3_raw) * 100 if np.std(d3_raw) > 0 else 0
                
                stats = {
                    'segment_id': seg_idx,
                    'points': len(segment),
                    'window_length': window_length,
                    'polyorder': polyorder,
                    'd2_std_improvement': d2_improvement,
                    'd3_std_improvement': d3_improvement
                }
                filter_stats.append(stats)
                
                # 重构滤波段
                filtered_segment = []
                for i in range(len(segment)):
                    filtered_point = {
                        'timestamp': timestamps[i],
                        'distance_2': d2_filtered[i],
                        'distance_3': d3_filtered[i],
                        'x': poses_filtered[i, 0],
                        'y': poses_filtered[i, 1],
                        'z': poses_filtered[i, 2],
                        'roll': poses_filtered[i, 3],
                        'pitch': poses_filtered[i, 4],
                        'yaw': poses_filtered[i, 5],
                        'segment_id': seg_idx
                    }
                    filtered_segment.append(filtered_point)
                
                filtered_segments.append(filtered_segment)
                self.logger.info(f"  Segment {seg_idx}: Filtered {len(filtered_segment)} points, D2 improvement: {d2_improvement:.1f}%, D3 improvement: {d3_improvement:.1f}%")
                
            except Exception as e:
                self.logger.error(f"  Segment {seg_idx}: Filtering failed - {e}")
                continue
        
        self.logger.info(f"Filtering completed on {len(filtered_segments)} segments")
        self.processing_stats['filtering'] = filter_stats
        return filtered_segments
    
    def optimize_sg_parameters(self, segment_data):
        """自动优化SG滤波器参数"""
        if len(segment_data) < 20:
            return self.config["filtering"]["sg_window_length"], self.config["filtering"]["sg_polyorder"]
        
        d2_data = np.array([point['distance_2'] for point in segment_data])
        
        best_params = None
        best_score = float('inf')
        
        window_lengths = [5, 7, 9, 11, 13, 15]
        poly_orders = [2, 3, 4]
        
        for window in window_lengths:
            if window >= len(segment_data):
                continue
            for poly in poly_orders:
                if poly >= window:
                    continue
                
                try:
                    filtered = savgol_filter(d2_data, window, poly)
                    residuals = d2_data - filtered
                    score = np.var(residuals)
                    
                    if score < best_score:
                        best_score = score
                        best_params = (window, poly)
                
                except:
                    continue
        
        if best_params is None:
            return self.config["filtering"]["sg_window_length"], self.config["filtering"]["sg_polyorder"]
        
        return best_params
    
    def apply_physical_correction(self, filtered_segments):
        """基于配置的物理模型校正与坐标变换"""
        if not self.config["physical_correction"]["enable_physical_correction"]:
            self.logger.info("Physical correction disabled, skipping")
            return None
        
        self.logger.info("Applying configurable physical model correction...")
        
        # 展平滤波数据
        filtered_data = []
        for segment in filtered_segments:
            filtered_data.extend(segment)
        
        if not filtered_data:
            self.logger.warning("No filtered data for physical correction")
            return None
        
        # 转换为DataFrame便于处理
        df = pd.DataFrame(filtered_data)
        
        # 获取坐标映射配置
        coord_mapping = self.config.get("coordinate_mapping", {})
        distance_mapping = coord_mapping.get("distance_to_laser_mapping", {
            "distance_2_maps_to": "laser_z",
            "distance_3_maps_to": "laser_x"
        })
        
        # 获取物理校正参数
        physical_offset = self.config["physical_correction"]["physical_offset"]
        world_target_x = self.config["physical_correction"]["world_target_x"]
        world_target_z = self.config["physical_correction"]["world_target_z"]
        apply_projection = self.config["physical_correction"]["apply_angle_projection"]
        
        # 记录坐标映射信息
        if coord_mapping.get("validation", {}).get("log_mapping_details", True):
            self.logger.info("Coordinate mapping configuration:")
            self.logger.info(f"  distance_2 → {distance_mapping['distance_2_maps_to']}")
            self.logger.info(f"  distance_3 → {distance_mapping['distance_3_maps_to']}")
            self.logger.info(f"  Physical offset: {physical_offset:.3f}m")
            self.logger.info(f"  World target X: {world_target_x:.3f}m")
            self.logger.info(f"  World target Z: {world_target_z:.3f}m")
            self.logger.info(f"  Apply angle projection: {apply_projection}")
        
        # 步骤1：根据配置进行距离到激光坐标系的映射
        if distance_mapping["distance_2_maps_to"] == "laser_z":
            df['laser_z_distance'] = df['distance_2'] + physical_offset
            df['laser_x_distance'] = df['distance_3'] + physical_offset
        elif distance_mapping["distance_2_maps_to"] == "laser_x":
            df['laser_x_distance'] = df['distance_2'] + physical_offset
            df['laser_z_distance'] = df['distance_3'] + physical_offset
        else:
            raise ValueError(f"Invalid distance mapping: {distance_mapping}")
        
        # 应用坐标变换符号
        transform_config = coord_mapping.get("coordinate_transform", {})
        laser_x_sign = transform_config.get("laser_x_sign", 1)
        laser_z_sign = transform_config.get("laser_z_sign", 1)
        
        df['laser_x_distance'] *= laser_x_sign
        df['laser_z_distance'] *= laser_z_sign
        
        # 步骤2：角度投影校正
        if apply_projection:
            # 计算平均姿态作为基准
            df['yaw_rad'] = np.radians(df['yaw'])
            df['pitch_rad'] = np.radians(df['pitch'])
            mean_yaw = np.mean(df['yaw_rad'])
            mean_pitch = np.mean(df['pitch_rad'])
            
            self.logger.info(f"Reference attitude:")
            self.logger.info(f"  Mean Yaw: {np.degrees(mean_yaw):.2f}°")
            self.logger.info(f"  Mean Pitch: {np.degrees(mean_pitch):.2f}°")
            
            # 计算每个点的姿态偏差
            df['yaw_deviation'] = df['yaw_rad'] - mean_yaw
            df['pitch_deviation'] = df['pitch_rad'] - mean_pitch
            
            # 应用投影校正
            df['laser_z_corrected'] = df['laser_z_distance'] * np.cos(df['yaw_deviation']) * np.cos(df['pitch_deviation'])
            df['laser_x_corrected'] = df['laser_x_distance'] * np.cos(df['yaw_deviation']) * np.cos(df['pitch_deviation'])
            
            # 计算校正统计
            z_correction_mean = np.mean(np.abs(df['laser_z_corrected'] - df['laser_z_distance']))
            x_correction_mean = np.mean(np.abs(df['laser_x_corrected'] - df['laser_x_distance']))
            
            self.logger.info(f"Angle projection correction applied:")
            self.logger.info(f"  Mean Z correction: {z_correction_mean*1000:.2f}mm")
            self.logger.info(f"  Mean X correction: {x_correction_mean*1000:.2f}mm")
        else:
            df['laser_z_corrected'] = df['laser_z_distance']
            df['laser_x_corrected'] = df['laser_x_distance']
        
        # 步骤3：世界坐标系转换
        transform_method = self.config["physical_correction"].get("coordinate_system", {}).get("world_transform_method", "subtraction")
        
        if transform_method == "subtraction":
            df['world_x'] = world_target_x - df['laser_x_corrected']
            df['world_z'] = world_target_z - df['laser_z_corrected']
        elif transform_method == "addition":
            df['world_x'] = world_target_x + df['laser_x_corrected']
            df['world_z'] = world_target_z + df['laser_z_corrected']
        elif transform_method == "direct":
            df['world_x'] = df['laser_x_corrected']
            df['world_z'] = df['laser_z_corrected']
        else:
            raise ValueError(f"Invalid world transform method: {transform_method}")
        
        # 为SVD准备数据对应关系
        df['laser_x'] = df['world_x']
        df['laser_z'] = df['world_z']
        df['tracker_x'] = df['x']  # 来自tracker的x
        df['tracker_z'] = df['z']  # 来自tracker的z
        
        # 保存完整的处理结果
        final_columns = [
            'timestamp', 'world_x', 'world_z',
            'x', 'y', 'z', 'roll', 'pitch', 'yaw',
            'laser_z_distance', 'laser_x_distance', 
            'laser_z_corrected', 'laser_x_corrected',
            'laser_x', 'laser_z', 'tracker_x', 'tracker_z'
        ]
        
        # 确保所有列都存在
        for col in final_columns:
            if col not in df.columns:
                self.logger.warning(f"Missing column {col} in final coordinates")
        
        final_df = df[final_columns].copy()
        
        # 保存最终坐标文件
        final_coords_file = os.path.join(self.run_output_dir, "final_coordinates.csv")
        final_df.to_csv(final_coords_file, index=False, float_format='%.6f')
        
        self.logger.info(f"Physical correction completed:")
        self.logger.info(f"  World coordinates range X: {final_df['world_x'].min():.3f}m to {final_df['world_x'].max():.3f}m")
        self.logger.info(f"  World coordinates range Z: {final_df['world_z'].min():.3f}m to {final_df['world_z'].max():.3f}m")
        self.logger.info(f"  SVD laser X range: {final_df['laser_x'].min():.3f}m to {final_df['laser_x'].max():.3f}m")
        self.logger.info(f"  SVD laser Z range: {final_df['laser_z'].min():.3f}m to {final_df['laser_z'].max():.3f}m")
        self.logger.info(f"  Final coordinates saved to: {final_coords_file}")
        
        return final_df

    def get_svd_ready_data(self, final_coordinates):
        """获取为SVD准备的数据"""
        if final_coordinates is None:
            return None
        
        svd_data = {
            'laser_points': np.column_stack([
                final_coordinates['laser_x'].values,
                final_coordinates['laser_z'].values
            ]),
            'tracker_points': np.column_stack([
                final_coordinates['tracker_x'].values,
                final_coordinates['tracker_z'].values
            ])
        }
        
        self.logger.info(f"SVD data prepared:")
        self.logger.info(f"  Laser points shape: {svd_data['laser_points'].shape}")
        self.logger.info(f"  Tracker points shape: {svd_data['tracker_points'].shape}")
        
        return svd_data

    def generate_coordinate_comparison_plot(self, final_coordinates):
        """生成世界坐标与追踪器坐标对比图（优化版）"""
        if final_coordinates is None:
            self.logger.warning("No final coordinates for coordinate comparison plot")
            return
        
        fig, axes = plt.subplots(2, 2, figsize=(16, 12))
        fig.suptitle('Coordinate Systems Comparison', fontsize=16, fontweight='bold')
        
        # 提取数据
        world_x = final_coordinates['world_x'].values
        world_z = final_coordinates['world_z'].values
        tracker_x = final_coordinates['tracker_x'].values
        tracker_z = final_coordinates['tracker_z'].values
        
        # 1. 散点图对比
        axes[0, 0].scatter(world_x, world_z, c=COLOR_SCHEME['world_final_filtered'], 
                          alpha=0.6, s=20, label='World Coordinates')
        axes[0, 0].scatter(tracker_x, tracker_z, c=COLOR_SCHEME['tracker_sync_raw'], 
                          alpha=0.6, s=20, label='Tracker Coordinates')
        axes[0, 0].set_xlabel('X (m)')
        axes[0, 0].set_ylabel('Z (m)')
        axes[0, 0].set_title('Scatter Plot Comparison')
        axes[0, 0].legend()
        axes[0, 0].grid(True, alpha=0.3)
        axes[0, 0].axis('equal')
        
        # 2. X坐标时序对比
        timestamps = (final_coordinates['timestamp'].values - final_coordinates['timestamp'].values[0])
        axes[0, 1].plot(timestamps, world_x, color=COLOR_SCHEME['world_final_filtered'], 
                       linewidth=2, label='World X', alpha=0.8)
        axes[0, 1].plot(timestamps, tracker_x, color=COLOR_SCHEME['tracker_sync_raw'], 
                       linewidth=2, label='Tracker X', alpha=0.8)
        axes[0, 1].set_xlabel('Time (s)')
        axes[0, 1].set_ylabel('X Coordinate (m)')
        axes[0, 1].set_title('X Coordinate Time Series')
        axes[0, 1].legend()
        axes[0, 1].grid(True, alpha=0.3)
        
        # 3. Z坐标时序对比
        axes[1, 0].plot(timestamps, world_z, color=COLOR_SCHEME['world_final_filtered'], 
                       linewidth=2, label='World Z', alpha=0.8)
        axes[1, 0].plot(timestamps, tracker_z, color=COLOR_SCHEME['tracker_sync_raw'], 
                       linewidth=2, label='Tracker Z', alpha=0.8)
        axes[1, 0].set_xlabel('Time (s)')
        axes[1, 0].set_ylabel('Z Coordinate (m)')
        axes[1, 0].set_title('Z Coordinate Time Series')
        axes[1, 0].legend()
        axes[1, 0].grid(True, alpha=0.3)
        
        # 4. 优化：使用2D直方图替代散点图展示差值分布
        diff_x = world_x - tracker_x
        diff_z = world_z - tracker_z
        
        # 使用hexbin图显示密度分布
        hb = axes[1, 1].hexbin(diff_x, diff_z, gridsize=30, cmap='Greens', 
                              edgecolors='black', linewidths=0.2)
        axes[1, 1].axhline(y=0, color='k', linestyle='--', alpha=0.5)
        axes[1, 1].axvline(x=0, color='k', linestyle='--', alpha=0.5)
        axes[1, 1].set_xlabel('X Difference (m)')
        axes[1, 1].set_ylabel('Z Difference (m)')
        axes[1, 1].set_title('Coordinate Differences Density')
        
        # 添加颜色条
        cb = plt.colorbar(hb, ax=axes[1, 1])
        cb.set_label('Point Density')
        
        # 添加统计信息
        stats_text = f'X Diff: μ={np.mean(diff_x):.3f}, σ={np.std(diff_x):.3f}\n'
        stats_text += f'Z Diff: μ={np.mean(diff_z):.3f}, σ={np.std(diff_z):.3f}'
        axes[1, 1].text(0.02, 0.98, stats_text, transform=axes[1, 1].transAxes,
                       verticalalignment='top', bbox=dict(boxstyle='round', 
                       facecolor=COLOR_SCHEME['statistics_background'], alpha=0.8))
        
        plt.tight_layout()
        
        # 保存图片
        plot_file = os.path.join(self.run_output_dir, "coordinate_comparison.png")
        plt.savefig(plot_file, dpi=300, bbox_inches='tight')
        plt.close()
        
        self.logger.info(f"Coordinate comparison plot saved to: {plot_file}")

    def generate_sg_filtering_effect_plot(self):
        """生成SG滤波效果图（优化版）"""
        if not hasattr(self, 'sg_comparison_data') or not self.sg_comparison_data:
            self.logger.warning("No SG comparison data available")
            return
        
        # 选择前几个段进行展示
        max_segments = min(4, len(self.sg_comparison_data))
        
        fig, axes = plt.subplots(max_segments, 2, figsize=(16, 4*max_segments))
        if max_segments == 1:
            axes = axes.reshape(1, -1)
        
        fig.suptitle('Savitzky-Golay Filtering Effects', fontsize=16, fontweight='bold')
        
        for i in range(max_segments):
            data = self.sg_comparison_data[i]
            
            # Distance 2 比较 - 使用统一色彩
            axes[i, 0].plot(data['timestamps'], data['d2_raw'] * 1000, 
                           color=COLOR_SCHEME['tracker_sync_raw'], linewidth=1, alpha=0.7, label='Raw Data')
            axes[i, 0].plot(data['timestamps'], data['d2_filtered'] * 1000, 
                           color=COLOR_SCHEME['world_final_filtered'], linewidth=2, label='SG Filtered')
            
            # 添加差异阴影区域
            diff = (data['d2_raw'] - data['d2_filtered']) * 1000
            axes[i, 0].fill_between(data['timestamps'], 
                                   data['d2_raw'] * 1000, 
                                   data['d2_filtered'] * 1000,
                                   alpha=0.3, color=COLOR_SCHEME['residuals_differences'])
            
            axes[i, 0].set_title(f'Segment {data["segment_id"]} - Distance 2 (Win={data["window_length"]}, Poly={data["polyorder"]})')
            axes[i, 0].set_ylabel('Distance (mm)')
            axes[i, 0].legend()
            axes[i, 0].grid(True, alpha=0.3)
            
            # Distance 3 比较
            axes[i, 1].plot(data['timestamps'], data['d3_raw'] * 1000, 
                           color=COLOR_SCHEME['tracker_sync_raw'], linewidth=1, alpha=0.7, label='Raw Data')
            axes[i, 1].plot(data['timestamps'], data['d3_filtered'] * 1000, 
                           color=COLOR_SCHEME['world_final_filtered'], linewidth=2, label='SG Filtered')
            
            # 添加差异阴影区域
            diff = (data['d3_raw'] - data['d3_filtered']) * 1000
            axes[i, 1].fill_between(data['timestamps'], 
                                   data['d3_raw'] * 1000, 
                                   data['d3_filtered'] * 1000,
                                   alpha=0.3, color=COLOR_SCHEME['residuals_differences'])
            
            axes[i, 1].set_title(f'Segment {data["segment_id"]} - Distance 3 (Win={data["window_length"]}, Poly={data["polyorder"]})')
            axes[i, 1].set_ylabel('Distance (mm)')
            axes[i, 1].legend()
            axes[i, 1].grid(True, alpha=0.3)
            
            # 添加噪声减少信息
            d2_noise_reduction = np.std(data['d2_raw'] - data['d2_filtered']) * 1000
            d3_noise_reduction = np.std(data['d3_raw'] - data['d3_filtered']) * 1000
            
            axes[i, 0].text(0.02, 0.98, f'Noise Reduced: {d2_noise_reduction:.2f}mm RMS', 
                           transform=axes[i, 0].transAxes, verticalalignment='top',
                           bbox=dict(boxstyle='round', facecolor=COLOR_SCHEME['statistics_background'], alpha=0.8))
            axes[i, 1].text(0.02, 0.98, f'Noise Reduced: {d3_noise_reduction:.2f}mm RMS', 
                           transform=axes[i, 1].transAxes, verticalalignment='top',
                           bbox=dict(boxstyle='round', facecolor=COLOR_SCHEME['statistics_background'], alpha=0.8))
            
            if i == max_segments - 1:
                axes[i, 0].set_xlabel('Time (s)')
                axes[i, 1].set_xlabel('Time (s)')
        
        plt.tight_layout()
        
        # 保存图片
        plot_file = os.path.join(self.run_output_dir, "sg_filtering_effects.png")
        plt.savefig(plot_file, dpi=300, bbox_inches='tight')
        plt.close()
        
        self.logger.info(f"SG filtering effects plot saved to: {plot_file}")

    def generate_interpolation_effect_plot(self, synchronized_data):
        """生成插值效果图（优化版）"""
        if not hasattr(self, 'interpolation_info') or not self.interpolation_info:
            self.logger.warning("No interpolation info available")
            return
        
        # 创建图表
        fig, axes = plt.subplots(2, 2, figsize=(16, 12))
        fig.suptitle('Data Interpolation Effects', fontsize=16, fontweight='bold')
        
        # 提取数据
        interp_df = pd.DataFrame(self.interpolation_info)
        sync_df = pd.DataFrame(synchronized_data)
        
        # 时间归一化
        time_offset = sync_df['timestamp'].min()
        interp_df['time_norm'] = interp_df['lidar_timestamp'] - time_offset
        sync_df['time_norm'] = sync_df['timestamp'] - time_offset
        
        # 1. 插值类型分布
        interp_types = interp_df['interpolation_type'].value_counts()
        colors = [COLOR_SCHEME['interpolation_linear'], COLOR_SCHEME['interpolation_nearest']]
        
        wedges, texts, autotexts = axes[0, 0].pie(interp_types.values, labels=interp_types.index, 
                                                 autopct='%1.1f%%', colors=colors[:len(interp_types)])
        axes[0, 0].set_title('Interpolation Type Distribution')
        
        # 2. 优化：时间差分布并添加容差线
        tolerance_ms = self.config["synchronization"]["interpolation_tolerance_ms"]
        
        axes[0, 1].hist(interp_df['time_diff'] * 1000, bins=30, alpha=0.7, 
                       color=COLOR_SCHEME['residuals_differences'], edgecolor='black')
        axes[0, 1].axvline(x=tolerance_ms, color=COLOR_SCHEME['tolerance_line'], 
                          linestyle='--', linewidth=3, label=f'Tolerance: {tolerance_ms}ms')
        axes[0, 1].set_xlabel('Time Difference (ms)')
        axes[0, 1].set_ylabel('Count')
        axes[0, 1].set_title('Interpolation Time Differences')
        axes[0, 1].grid(True, alpha=0.3)
        axes[0, 1].legend()
        
        # 统计信息
        mean_diff = np.mean(interp_df['time_diff']) * 1000
        max_diff = np.max(interp_df['time_diff']) * 1000
        within_tolerance = np.sum(interp_df['time_diff'] * 1000 <= tolerance_ms) / len(interp_df) * 100
        
        axes[0, 1].axvline(x=mean_diff, color=COLOR_SCHEME['world_final_filtered'], 
                          linestyle='-', linewidth=2, label=f'Mean: {mean_diff:.1f}ms')
        axes[0, 1].legend()
        
        # 添加统计文本
        stats_text = f'Within Tolerance: {within_tolerance:.1f}%\nMax Diff: {max_diff:.1f}ms'
        axes[0, 1].text(0.98, 0.98, stats_text, transform=axes[0, 1].transAxes,
                       verticalalignment='top', horizontalalignment='right',
                       bbox=dict(boxstyle='round', facecolor=COLOR_SCHEME['statistics_background'], alpha=0.8))
        
        # 3. Alpha值分布（线性插值权重）
        linear_data = interp_df[interp_df['interpolation_type'] == 'linear']
        if len(linear_data) > 0:
            axes[1, 0].hist(linear_data['alpha'], bins=20, alpha=0.7, 
                           color=COLOR_SCHEME['interpolation_linear'], edgecolor='black')
            axes[1, 0].set_xlabel('Alpha (Interpolation Weight)')
            axes[1, 0].set_ylabel('Count')
            axes[1, 0].set_title('Linear Interpolation Weights')
            axes[1, 0].grid(True, alpha=0.3)
            
            # 添加均匀分布参考线
            uniform_height = len(linear_data) / 20  # 20 bins
            axes[1, 0].axhline(y=uniform_height, color=COLOR_SCHEME['tolerance_line'], 
                              linestyle='--', alpha=0.7, label='Uniform Distribution')
            axes[1, 0].legend()
        else:
            axes[1, 0].text(0.5, 0.5, 'No Linear\nInterpolation\nData', 
                           ha='center', va='center', transform=axes[1, 0].transAxes, fontsize=14)
            axes[1, 0].set_title('Linear Interpolation Weights')
        
        # 4. 插值效果时序图
        # 标记不同插值类型的点
        linear_mask = interp_df['interpolation_type'] == 'linear'
        nearest_mask = interp_df['interpolation_type'] == 'nearest'
        
        # 画背景数据
        tracker_times = self.tracker_data['timestamp'] - time_offset
        axes[1, 1].plot(tracker_times, self.tracker_data['x'], 
                       color=COLOR_SCHEME['tracker_sync_raw'], alpha=0.3, linewidth=1, label='Tracker Raw Data')
        
        # 标记插值点
        if np.any(linear_mask):
            linear_times = interp_df[linear_mask]['time_norm']
            linear_indices = sync_df['timestamp'].isin(interp_df[linear_mask]['lidar_timestamp'])
            if np.any(linear_indices):
                linear_x = sync_df[linear_indices]['x']
                axes[1, 1].scatter(linear_times, linear_x, c=COLOR_SCHEME['interpolation_linear'], 
                                 s=30, alpha=0.8, label='Linear Interpolated', zorder=5)
        
        if np.any(nearest_mask):
            nearest_times = interp_df[nearest_mask]['time_norm']
            nearest_indices = sync_df['timestamp'].isin(interp_df[nearest_mask]['lidar_timestamp'])
            if np.any(nearest_indices):
                nearest_x = sync_df[nearest_indices]['x']
                axes[1, 1].scatter(nearest_times, nearest_x, c=COLOR_SCHEME['interpolation_nearest'], 
                                 s=30, alpha=0.8, label='Nearest Neighbor', zorder=5)
        
        axes[1, 1].set_xlabel('Time (s)')
        axes[1, 1].set_ylabel('X Coordinate (m)')
        axes[1, 1].set_title('Interpolation Method Visualization')
        axes[1, 1].legend()
        axes[1, 1].grid(True, alpha=0.3)
        
        plt.tight_layout()
        
        # 保存图片
        plot_file = os.path.join(self.run_output_dir, "interpolation_effects.png")
        plt.savefig(plot_file, dpi=300, bbox_inches='tight')
        plt.close()
        
        self.logger.info(f"Interpolation effects plot saved to: {plot_file}")

    def generate_enhanced_visualizations(self, synchronized_data, filtered_segments, final_coordinates=None):
        """步骤六：生成增强的可视化分析"""
        self.logger.info("Generating enhanced visualizations...")
        
        # 展平滤波数据
        filtered_data = []
        for segment in filtered_segments:
            filtered_data.extend(segment)
        
        if not filtered_data:
            self.logger.warning("No filtered data for visualization")
            return
        
        # 生成新的可视化图表
        self.generate_coordinate_comparison_plot(final_coordinates)
        self.generate_sg_filtering_effect_plot()
        self.generate_interpolation_effect_plot(synchronized_data)
        self.generate_median_filter_effect_plot()  # 新增：中值滤波效果图
        
        # 创建增强的静态图表
        self.generate_enhanced_static_plots(synchronized_data, filtered_data, final_coordinates)
        
        # 创建交互式图表
        if self.config["visualization"]["generate_interactive_plots"]:
            self.generate_interactive_plots(synchronized_data, filtered_data, final_coordinates)
    
    def generate_enhanced_static_plots(self, synchronized_data, filtered_data, final_coordinates=None):
        """生成增强的静态图表（优化版）"""
        fig, axes = plt.subplots(3, 3, figsize=(24, 18))
        fig.suptitle('Enhanced Data Processing Results', fontsize=16, fontweight='bold')
        
        # 提取数据数组
        sync_times = np.array([p['timestamp'] for p in synchronized_data])
        sync_d2 = np.array([p['distance_2'] for p in synchronized_data])
        sync_d3 = np.array([p['distance_3'] for p in synchronized_data])
        
        filt_times = np.array([p['timestamp'] for p in filtered_data])
        filt_d2 = np.array([p['distance_2'] for p in filtered_data])
        filt_d3 = np.array([p['distance_3'] for p in filtered_data])
        
        # 时间归一化
        time_offset = min(sync_times[0], filt_times[0])
        sync_times_norm = sync_times - time_offset
        filt_times_norm = filt_times - time_offset
        
        # 图1：Distance 2 对比（使用统一色彩）
        axes[0, 0].plot(sync_times_norm, sync_d2 * 1000, color=COLOR_SCHEME['tracker_sync_raw'], 
                       alpha=0.6, linewidth=1, label='Synchronized')
        axes[0, 0].plot(filt_times_norm, filt_d2 * 1000, color=COLOR_SCHEME['world_final_filtered'], 
                       linewidth=2, label='SG Filtered')
        axes[0, 0].set_title('Distance Channel 2')
        axes[0, 0].set_ylabel('Distance (mm)')
        axes[0, 0].legend()
        axes[0, 0].grid(True, alpha=0.3)
        
        # 图2：Distance 3 对比
        axes[0, 1].plot(sync_times_norm, sync_d3 * 1000, color=COLOR_SCHEME['tracker_sync_raw'], 
                       alpha=0.6, linewidth=1, label='Synchronized')
        axes[0, 1].plot(filt_times_norm, filt_d3 * 1000, color=COLOR_SCHEME['world_final_filtered'], 
                       linewidth=2, label='SG Filtered')
        axes[0, 1].set_title('Distance Channel 3')
        axes[0, 1].set_ylabel('Distance (mm)')
        axes[0, 1].legend()
        axes[0, 1].grid(True, alpha=0.3)
        
        # 图3：优化的残差分析 - 使用移动平均
        if len(synchronized_data) > 0 and len(filtered_data) > 0:
            residuals_d2, residuals_d3 = self.compute_matched_residuals(synchronized_data, filtered_data)
            
            if len(residuals_d2) > 0:
                # 原始残差
                axes[0, 2].plot(range(len(residuals_d2)), residuals_d2 * 1000, 
                               color=COLOR_SCHEME['residuals_differences'], alpha=0.3, linewidth=1, label='Raw Residuals')
                
                # 移动平均
                window = self.config["visualization"]["residual_moving_average_window"]
                if len(residuals_d2) > window:
                    moving_avg = pd.Series(residuals_d2).rolling(window=window, center=True).mean()
                    axes[0, 2].plot(range(len(residuals_d2)), moving_avg * 1000, 
                                   color=COLOR_SCHEME['world_final_filtered'], linewidth=2, label=f'Moving Average ({window})')
                
                axes[0, 2].axhline(y=0, color='k', linestyle='--', alpha=0.5)
                axes[0, 2].set_title('D2 Filtering Residuals')
                axes[0, 2].set_ylabel('Residual (mm)')
                axes[0, 2].legend()
                axes[0, 2].grid(True, alpha=0.3)
        
        # 图4：残差直方图
        if self.config["visualization"]["generate_residual_histogram"]:
            if len(synchronized_data) > 0 and len(filtered_data) > 0:
                residuals_d2, residuals_d3 = self.compute_matched_residuals(synchronized_data, filtered_data)
                
                if len(residuals_d2) > 0:
                    axes[1, 0].hist(residuals_d2 * 1000, bins=30, alpha=0.7, 
                                   color=COLOR_SCHEME['residuals_differences'], edgecolor='black')
                    axes[1, 0].axvline(x=0, color=COLOR_SCHEME['world_final_filtered'], 
                                      linestyle='--', linewidth=2)
                    axes[1, 0].set_title('D2 Residuals Histogram')
                    axes[1, 0].set_xlabel('Residual (mm)')
                    axes[1, 0].set_ylabel('Count')
                    axes[1, 0].grid(True, alpha=0.3)
        
        # 图5：频域分析 (FFT)
        if self.config["visualization"]["generate_fft_analysis"] and len(filtered_data) > 10:
            # 计算采样率
            dt = np.mean(np.diff(filt_times))
            fs = 1.0 / dt
            
            # FFT分析
            N = len(filt_d3)
            freqs = fftfreq(N, dt)[:N//2]
            
            # 同步数据的FFT（如果长度匹配）
            if len(sync_d3) == len(filt_d3):
                sync_fft = np.abs(fft(sync_d3))[:N//2]
                filt_fft = np.abs(fft(filt_d3))[:N//2]
                
                axes[1, 1].loglog(freqs[1:], sync_fft[1:], color=COLOR_SCHEME['tracker_sync_raw'], 
                                 alpha=0.6, label='Before filtering')
                axes[1, 1].loglog(freqs[1:], filt_fft[1:], color=COLOR_SCHEME['world_final_filtered'], 
                                 linewidth=2, label='After filtering')
                axes[1, 1].set_title('Frequency Domain Analysis (D3)')
                axes[1, 1].set_xlabel('Frequency (Hz)')
                axes[1, 1].set_ylabel('Magnitude')
                axes[1, 1].legend()
                axes[1, 1].grid(True, alpha=0.3)
        
        # 图6：最终坐标（如果可用）
        if final_coordinates is not None:
            axes[1, 2].scatter(final_coordinates['world_x'], final_coordinates['world_z'], 
                             alpha=0.6, s=20, c=COLOR_SCHEME['world_final_filtered'])
            axes[1, 2].set_title('Final Coordinates')
            axes[1, 2].set_xlabel('X (m)')
            axes[1, 2].set_ylabel('Z (m)')
            axes[1, 2].grid(True, alpha=0.3)
            axes[1, 2].axis('equal')
        
        # 图7-9：优化的处理统计 - 使用表格
        self.plot_processing_statistics_table(axes[2, :])
        
        plt.tight_layout()
        
        # 保存图表
        plot_file = os.path.join(self.run_output_dir, "enhanced_processing_results.png")
        plt.savefig(plot_file, dpi=300, bbox_inches='tight')
        plt.close()
        
        self.logger.info(f"Enhanced static plots saved to: {plot_file}")

    def plot_processing_statistics_table(self, axes):
        """优化：绘制处理统计表格而非纯文本"""
        # 清空所有轴
        for ax in axes:
            ax.clear()
            ax.axis('off')
        
        # 准备表格数据
        table_data = []
        headers = ['Stage', 'Metric', 'Value']
        
        # 预处理统计
        if 'preprocessing' in self.processing_stats:
            stats = self.processing_stats['preprocessing']
            if isinstance(stats, dict):
                table_data.extend([
                    ['Preprocessing', 'Original Points', f"{stats.get('original_points', 'N/A')}"],
                    ['', 'Filtered Points', f"{stats.get('filtered_points', 'N/A')}"],
                    ['', 'Reduction %', f"{stats.get('reduction_ratio', 0):.1f}%"],
                    ['', 'Stationary Segments', f"{stats.get('stationary_segments', 'N/A')}"]
                ])
        
        # 中值滤波统计
        if 'median_filtering' in self.processing_stats:
            stats = self.processing_stats['median_filtering']
            if isinstance(stats, dict):
                for col, col_stats in stats.items():
                    table_data.append(['Median Filter', f'{col} Improvement', 
                                     f"{col_stats.get('improvement_pct', 0):.1f}%"])
        
        # 同步统计
        if 'synchronization' in self.processing_stats:
            stats = self.processing_stats['synchronization']
            if isinstance(stats, dict):
                success = stats.get('success', 0)
                failed = stats.get('failed', 0)
                success_rate = success / (success + failed) * 100 if (success + failed) > 0 else 0
                table_data.extend([
                    ['Synchronization', 'Success Points', f"{success}"],
                    ['', 'Failed Points', f"{failed}"],
                    ['', 'Success Rate %', f"{success_rate:.1f}%"]
                ])
        
        # 显示表格（分两列）
        mid_point = len(table_data) // 2
        
        if table_data:
            # 第一个表格
            table1 = axes[0].table(cellText=table_data[:mid_point], 
                                  colLabels=headers,
                                  cellLoc='left',
                                  loc='center',
                                  colWidths=[0.3, 0.4, 0.3])
            table1.auto_set_font_size(False)
            table1.set_fontsize(10)
            table1.scale(1, 2)
            
            # 设置表格样式
            for i in range(len(headers)):
                table1[(0, i)].set_facecolor(COLOR_SCHEME['statistics_background'])
                table1[(0, i)].set_text_props(weight='bold')
            
            axes[0].set_title('Processing Statistics (Part 1)', fontweight='bold')
            
            # 第二个表格
            if mid_point < len(table_data):
                table2 = axes[1].table(cellText=table_data[mid_point:], 
                                      colLabels=headers,
                                      cellLoc='left',
                                      loc='center',
                                      colWidths=[0.3, 0.4, 0.3])
                table2.auto_set_font_size(False)
                table2.set_fontsize(10)
                table2.scale(1, 2)
                
                # 设置表格样式
                for i in range(len(headers)):
                    table2[(0, i)].set_facecolor(COLOR_SCHEME['statistics_background'])
                    table2[(0, i)].set_text_props(weight='bold')
                
                axes[1].set_title('Processing Statistics (Part 2)', fontweight='bold')
        
        # 第三个轴显示配置摘要
        config_summary = [
            f"Median Filter: {'Enabled' if self.config['preprocessing']['enable_median_filter'] else 'Disabled'}",
            f"Kernel Size: {self.config['preprocessing']['median_filter_kernel_size']}",
            f"Outlier Detection: {'Enabled' if self.config['preprocessing']['enable_outlier_detection'] else 'Disabled'}",
            f"SG Window: {self.config['filtering']['sg_window_length']}",
            f"SG Poly Order: {self.config['filtering']['sg_polyorder']}",
            f"Interpolation Tolerance: {self.config['synchronization']['interpolation_tolerance_ms']}ms"
        ]
        
        axes[2].text(0.05, 0.95, '\n'.join(config_summary), transform=axes[2].transAxes, 
                    fontsize=10, verticalalignment='top', fontfamily='monospace',
                    bbox=dict(boxstyle='round', facecolor=COLOR_SCHEME['statistics_background'], alpha=0.8))
        axes[2].set_title('Configuration Summary', fontweight='bold')
    
    def compute_matched_residuals(self, synchronized_data, filtered_data):
        """计算物理对应点的残差"""
        if not synchronized_data or not filtered_data:
            return np.array([]), np.array([])
        
        sync_times = np.array([p['timestamp'] for p in synchronized_data])
        filt_times = np.array([p['timestamp'] for p in filtered_data])
        
        residuals_d2 = []
        residuals_d3 = []
        
        # 为每个滤波点找到对应的同步点
        for filt_point in filtered_data:
            filt_time = filt_point['timestamp']
            
            # 找到最近的同步点
            time_diffs = np.abs(sync_times - filt_time)
            closest_idx = np.argmin(time_diffs)
            
            if time_diffs[closest_idx] < 0.1:  # 100ms容差
                sync_point = synchronized_data[closest_idx]
                residuals_d2.append(sync_point['distance_2'] - filt_point['distance_2'])
                residuals_d3.append(sync_point['distance_3'] - filt_point['distance_3'])
        
        return np.array(residuals_d2), np.array(residuals_d3)
    
    def generate_interactive_plots(self, synchronized_data, filtered_data, final_coordinates=None):
        """生成交互式图表"""
        self.logger.info("Generating interactive plots...")
        
        # 转换为DataFrame
        sync_df = pd.DataFrame(synchronized_data)
        filt_df = pd.DataFrame(filtered_data)
        
        # 时间归一化
        start_time = min(sync_df['timestamp'].min(), filt_df['timestamp'].min())
        sync_df['time_norm'] = sync_df['timestamp'] - start_time
        filt_df['time_norm'] = filt_df['timestamp'] - start_time
        
        # 创建子图
        fig = make_subplots(
            rows=3, cols=2,
            subplot_titles=('Distance 2', 'Distance 3', 'X Coordinate', 'Y Coordinate', 'Z Coordinate', 'Final Coordinates'),
            specs=[[{"secondary_y": False}, {"secondary_y": False}],
                   [{"secondary_y": False}, {"secondary_y": False}],
                   [{"secondary_y": False}, {"secondary_y": False}]]
        )
        
        # 使用统一色彩方案
        # 距离图表
        fig.add_trace(go.Scatter(x=sync_df['time_norm'], y=sync_df['distance_2']*1000,
                                mode='lines', name='D2 Synchronized', 
                                line=dict(color=COLOR_SCHEME['tracker_sync_raw'], width=1), opacity=0.6), row=1, col=1)
        fig.add_trace(go.Scatter(x=filt_df['time_norm'], y=filt_df['distance_2']*1000,
                                mode='lines', name='D2 Filtered',
                                line=dict(color=COLOR_SCHEME['world_final_filtered'], width=2)), row=1, col=1)
        
        fig.add_trace(go.Scatter(x=sync_df['time_norm'], y=sync_df['distance_3']*1000,
                                mode='lines', name='D3 Synchronized',
                                line=dict(color=COLOR_SCHEME['tracker_sync_raw'], width=1), opacity=0.6), row=1, col=2)
        fig.add_trace(go.Scatter(x=filt_df['time_norm'], y=filt_df['distance_3']*1000,
                                mode='lines', name='D3 Filtered',
                                line=dict(color=COLOR_SCHEME['world_final_filtered'], width=2)), row=1, col=2)
        
        # 位置图表
        fig.add_trace(go.Scatter(x=sync_df['time_norm'], y=sync_df['x']*1000,
                                mode='lines', name='X Synchronized',
                                line=dict(color=COLOR_SCHEME['tracker_sync_raw'], width=1), opacity=0.6), row=2, col=1)
        fig.add_trace(go.Scatter(x=filt_df['time_norm'], y=filt_df['x']*1000,
                                mode='lines', name='X Filtered',
                                line=dict(color=COLOR_SCHEME['world_final_filtered'], width=2)), row=2, col=1)
        
        fig.add_trace(go.Scatter(x=sync_df['time_norm'], y=sync_df['y']*1000,
                                mode='lines', name='Y Synchronized',
                                line=dict(color=COLOR_SCHEME['tracker_sync_raw'], width=1), opacity=0.6), row=2, col=2)
        fig.add_trace(go.Scatter(x=filt_df['time_norm'], y=filt_df['y']*1000,
                                mode='lines', name='Y Filtered',
                                line=dict(color=COLOR_SCHEME['world_final_filtered'], width=2)), row=2, col=2)
        
        fig.add_trace(go.Scatter(x=sync_df['time_norm'], y=sync_df['z']*1000,
                                mode='lines', name='Z Synchronized',
                                line=dict(color=COLOR_SCHEME['tracker_sync_raw'], width=1), opacity=0.6), row=3, col=1)
        fig.add_trace(go.Scatter(x=filt_df['time_norm'], y=filt_df['z']*1000,
                                mode='lines', name='Z Filtered',
                                line=dict(color=COLOR_SCHEME['world_final_filtered'], width=2)), row=3, col=1)
        
        # 最终坐标
        if final_coordinates is not None:
            fig.add_trace(go.Scatter(x=final_coordinates['world_x'], y=final_coordinates['world_z'],
                                   mode='markers', name='Final Coordinates',
                                   marker=dict(color=COLOR_SCHEME['world_final_filtered'], size=4)), row=3, col=2)
        
        # 更新布局
        fig.update_layout(height=1200, title_text="Interactive Enhanced Processing Results")
        fig.update_xaxes(title_text="Time (s)")
        
        # 保存交互式图表
        html_file = os.path.join(self.run_output_dir, "interactive_enhanced_results.html")
        fig.write_html(html_file)
        
        self.logger.info(f"Interactive plots saved to: {html_file}")
    
    def save_intermediate_results(self, synchronized_data, filtered_segments):
        """保存中间结果"""
        # 保存同步数据
        if self.config["output"]["save_synchronized_csv"]:
            sync_file = os.path.join(self.run_output_dir, "synchronized_data.csv")
            sync_df = pd.DataFrame(synchronized_data)
            sync_df.to_csv(sync_file, index=False, float_format='%.6f')
            self.logger.info(f"Synchronized data saved to: {sync_file}")
        
        # 保存滤波数据
        if self.config["output"]["save_filtered_csv"]:
            filtered_file = os.path.join(self.run_output_dir, "filtered_data.csv")
            filtered_data = []
            for segment in filtered_segments:
                filtered_data.extend(segment)
            
            if filtered_data:
                filt_df = pd.DataFrame(filtered_data)
                filt_df.to_csv(filtered_file, index=False, float_format='%.6f')
                self.logger.info(f"Filtered data saved to: {filtered_file}")
    
    def generate_comprehensive_report(self):
        """生成综合分析报告"""
        report_file = os.path.join(self.run_output_dir, "comprehensive_analysis_report.txt")
        
        with open(report_file, 'w') as f:
            f.write("=" * 80 + "\n")
            f.write("INDUSTRIAL-GRADE DATA PROCESSING ANALYSIS REPORT\n")
            f.write("=" * 80 + "\n\n")
            
            f.write(f"Processing Date: {datetime.now().isoformat()}\n")
            f.write(f"Session ID: {self.output_timestamp}\n")
            f.write(f"Raw Data File: {self.raw_data_file}\n")
            f.write(f"Output Directory: {self.run_output_dir}\n\n")
            
            # 写入详细统计
            for stage, stats in self.processing_stats.items():
                f.write(f"{stage.upper()} RESULTS:\n")
                f.write("-" * 40 + "\n")
                
                # 处理不同类型的统计数据
                if isinstance(stats, list):
                    # 处理列表类型的统计数据（如 filtering 阶段）
                    if stage == 'filtering':
                        for i, segment_stats in enumerate(stats):
                            f.write(f"Segment {segment_stats.get('segment_id', i)}:\n")
                            for key, value in segment_stats.items():
                                if key != 'segment_id':
                                    f.write(f"  {key}: {value}\n")
                            f.write("\n")
                    else:
                        # 其他列表类型数据
                        for i, item in enumerate(stats):
                            f.write(f"Item {i}: {item}\n")
                elif isinstance(stats, dict):
                    # 处理字典类型的统计数据
                    for key, value in stats.items():
                        if isinstance(value, dict):
                            f.write(f"{key}:\n")
                            for subkey, subvalue in value.items():
                                f.write(f"  {subkey}: {subvalue}\n")
                        elif isinstance(value, (list, tuple)):
                            f.write(f"{key}: {value}\n")
                        else:
                            f.write(f"{key}: {value}\n")
                else:
                    # 其他类型的数据
                    f.write(f"Data: {stats}\n")
                
                f.write("\n")
            
            # 写入配置
            f.write("CONFIGURATION USED:\n")
            f.write("-" * 40 + "\n")
            f.write(json.dumps(self.config, indent=2))
        
        self.logger.info(f"Comprehensive report saved to: {report_file}")
        return report_file
    
    def process(self):
        """主处理流水线"""
        self.logger.info("=" * 80)
        self.logger.info("STARTING INDUSTRIAL-GRADE DATA PROCESSING PIPELINE")
        self.logger.info("=" * 80)
        
        try:
            # 步骤1：预处理
            preprocessed_lidar = self.preprocess_lidar_data()
            
            # 步骤2：同步
            synchronized_data = self.synchronize_data(preprocessed_lidar)
            if not synchronized_data:
                self.logger.error("Synchronization failed, cannot continue")
                return False
            
            # 步骤2.5：增强的中值滤波
            median_filtered_data, median_comparison = self.apply_median_filter(synchronized_data)
            
            # 步骤3：姿态筛选（使用中值滤波后的数据）
            pose_filtered_data = self.filter_by_pose_stability(median_filtered_data)
            if not pose_filtered_data:
                self.logger.error("Pose filtering resulted in no data, cannot continue")
                return False
            
            # 步骤4：分段与滤波
            segments = self.segment_data(pose_filtered_data)
            if not segments:
                self.logger.error("Segmentation failed, cannot continue")
                return False
            
            filtered_segments = self.apply_sg_filtering(segments)
            if not filtered_segments:
                self.logger.error("Filtering failed, cannot continue")
                return False
            
            # 步骤5：保存中间结果
            self.save_intermediate_results(synchronized_data, filtered_segments)
            
            # 步骤6：物理校正
            final_coordinates = self.apply_physical_correction(filtered_segments)
            
            # 步骤7：可视化
            self.generate_enhanced_visualizations(synchronized_data, filtered_segments, final_coordinates)
            
            # 步骤8：生成报告
            self.generate_comprehensive_report()
            
            self.logger.info("=" * 80)
            self.logger.info("PROCESSING COMPLETED SUCCESSFULLY")
            self.logger.info("=" * 80)
            self.logger.info(f"All outputs saved to: {self.run_output_dir}")
            
            return True
            
        except Exception as e:
            self.logger.error(f"Processing failed: {e}")
            import traceback
            traceback.print_exc()
            return False

def main():
    parser = argparse.ArgumentParser(description="Industrial-Grade Data Processing for LiDAR-Tracker Data")
    parser.add_argument("input_file", help="Raw data NPZ file to process")
    parser.add_argument("-c", "--config", default="process_config.json", 
                       help="Processing configuration file")
    
    args = parser.parse_args()
    
    if not os.path.exists(args.input_file):
        print(f"ERROR: Input file not found: {args.input_file}")
        return 1
    
    # 创建处理器并运行
    processor = IndustrialDataProcessor(args.input_file, args.config)
    success = processor.process()
    return 0 if success else 1

if __name__ == "__main__":
    exit(main())