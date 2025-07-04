#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Enhanced Tracker data processing script with Spatial Filtering + SVD + Non-rigid Deformation Learning 
(Fixed NaN handling and enhanced with missing value imputation)
"""
import sys
import os
import datetime
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from sklearn.neighbors import NearestNeighbors
from sklearn.model_selection import KFold, GridSearchCV
from sklearn.preprocessing import StandardScaler
from sklearn.impute import SimpleImputer, KNNImputer  # 添加缺失值处理
from scipy.interpolate import griddata, RBFInterpolator, LSQBivariateSpline
from scipy.spatial.distance import cdist
from scipy import stats
from scipy.optimize import minimize
import itertools
from concurrent.futures import ProcessPoolExecutor
import warnings
warnings.filterwarnings('ignore')

# Create timestamp for output directory
current_time = datetime.datetime.now()
timestamp = current_time.strftime("%Y-%m-%d_%H-%M-%S")

# 1:32缩放后的RMSE目标
SCALE_FACTOR = 32
REAL_WORLD_LATERAL_TARGET = 0.10  # 真实世界本地道路横向精度要求 (10cm)
REAL_WORLD_LONGITUDINAL_TARGET = 0.10  # 真实世界本地道路纵向精度要求 (10cm)

# 缩放后的目标
SCALED_LATERAL_TARGET = REAL_WORLD_LATERAL_TARGET / SCALE_FACTOR  # 3.125mm
SCALED_LONGITUDINAL_TARGET = REAL_WORLD_LONGITUDINAL_TARGET / SCALE_FACTOR  # 3.125mm
SCALED_COMBINED_TARGET = np.sqrt(SCALED_LATERAL_TARGET**2 + SCALED_LONGITUDINAL_TARGET**2)  # ~4.42mm

# 使用更严格的目标
TARGET_RMSE = 0.004  # 4mm

# Create output directory
script_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(script_dir)
result_dir = os.path.join(parent_dir, "results")
# File paths
LASER_TRACKER_CSV = os.path.join(parent_dir, "data/combined_corrected_tracker_data_1.csv")
csv_filename = os.path.basename(LASER_TRACKER_CSV)
output_dir = os.path.join(result_dir, f"SpatialFiltered_SVD_Deformation_{csv_filename}_{timestamp}_1to32_scale_RMSE_{TARGET_RMSE*1000:.1f}mm")
os.makedirs(output_dir, exist_ok=True)

# Create subdirectories
log_dir = os.path.join(output_dir, "log")
data_dir = os.path.join(output_dir, "data")
result_dir = os.path.join(output_dir, "results")
img_dir = os.path.join(output_dir, "img")

for directory in [log_dir, data_dir, result_dir, img_dir]:
    os.makedirs(directory, exist_ok=True)

# Setup logging
import logging
log_file = os.path.join(log_dir, "tracker_process.log")
logging.basicConfig(
    filename=log_file,
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)

console = logging.StreamHandler()
console.setLevel(logging.INFO)
formatter = logging.Formatter('%(levelname)s - %(message)s')
console.setFormatter(formatter)
logger.addHandler(console)

ROW_X = "X"
ROW_Y = "Z"

class MissingValueHandler:
    """
    处理缺失值的专用类，基于scikit-learn的最佳实践
    """
    def __init__(self, strategy='knn', k_neighbors=5):
        """
        Parameters:
        strategy: 'knn', 'mean', 'median', 'most_frequent', 'constant'
        k_neighbors: KNN插补的邻居数量
        """
        self.strategy = strategy
        self.k_neighbors = k_neighbors
        self.imputer = None
        
    def fit_transform_missing_values(self, data, point_type="Unknown"):
        """处理缺失值"""
        data = np.array(data)
        
        # 检查是否有缺失值
        has_nan = np.isnan(data).any()
        if not has_nan:
            logger.info(f"{point_type}: No missing values detected")
            return data
        
        # 统计缺失值
        nan_count = np.isnan(data).sum()
        nan_percentage = (nan_count / data.size) * 100
        
        logger.info(f"{point_type}: Found {nan_count} missing values ({nan_percentage:.2f}%)")
        
        # 选择插补策略
        if self.strategy == 'knn':
            # 使用KNNImputer，它原生支持NaN值
            self.imputer = KNNImputer(n_neighbors=self.k_neighbors, weights="uniform")
        else:
            # 使用SimpleImputer
            self.imputer = SimpleImputer(strategy=self.strategy)
        
        # 只对XZ平面进行插补（Y轴始终为0）
        data_xz = data[:, [0, 2]]
        
        try:
            # 应用插补
            imputed_xz = self.imputer.fit_transform(data_xz)
            
            # 重构3D数据
            result = np.copy(data)
            result[:, [0, 2]] = imputed_xz
            result[:, 1] = 0  # 确保Y=0
            
            # 计算插补效果
            imputed_count = nan_count
            logger.info(f"{point_type}: Successfully imputed {imputed_count} missing values using {self.strategy}")
            
            return result
            
        except Exception as e:
            logger.error(f"Failed to impute missing values for {point_type}: {e}")
            # 回退策略：使用简单的均值填充
            logger.info(f"Falling back to mean imputation for {point_type}")
            fallback_imputer = SimpleImputer(strategy='mean')
            imputed_xz = fallback_imputer.fit_transform(data_xz)
            result = np.copy(data)
            result[:, [0, 2]] = imputed_xz
            result[:, 1] = 0
            return result

class SpatialFilter:
    """
    空间滤波器 - 修复了NaN处理问题
    """
    def __init__(self, filter_type='median', k_neighbors=10, gaussian_sigma=0.001, 
                 adaptive_k=True, preserve_shape=True):
        self.filter_type = filter_type
        self.k_neighbors = k_neighbors
        self.gaussian_sigma = gaussian_sigma
        self.adaptive_k = adaptive_k
        self.preserve_shape = preserve_shape
        self.actual_k_used = None
        self.filtering_stats = {}
        self.missing_handler = MissingValueHandler(strategy='knn', k_neighbors=5)
        
    def apply_spatial_filter(self, positions_A, positions_B):
        """
        对源点云和目标点云应用空间滤波（修复版本）
        """
        logger.info("=== Starting Spatial Filtering Preprocessing ===")
        logger.info(f"Filter type: {self.filter_type}")
        logger.info(f"K neighbors: {self.k_neighbors}")
        logger.info(f"Adaptive K: {self.adaptive_k}")
        logger.info(f"Preserve shape: {self.preserve_shape}")
        
        positions_A = np.array(positions_A)
        positions_B = np.array(positions_B)
        
        logger.info(f"Original point clouds: {len(positions_A)} point pairs")
        
        # Step 1: 处理缺失值
        logger.info("Step 1: Handling missing values...")
        clean_A = self.missing_handler.fit_transform_missing_values(positions_A, "Source")
        clean_B = self.missing_handler.fit_transform_missing_values(positions_B, "Target")
        
        # Step 2: 分析原始数据质量
        self._analyze_original_data(clean_A, clean_B)
        
        # Step 3: 应用滤波器
        logger.info("Step 2: Applying spatial filtering...")
        if self.filter_type == 'median':
            filtered_A = self._apply_median_filter(clean_A, "Source")
            filtered_B = self._apply_median_filter(clean_B, "Target")
        elif self.filter_type == 'gaussian':
            filtered_A = self._apply_gaussian_filter(clean_A, "Source")
            filtered_B = self._apply_gaussian_filter(clean_B, "Target")
        elif self.filter_type == 'bilateral':
            filtered_A = self._apply_bilateral_filter(clean_A, "Source")
            filtered_B = self._apply_bilateral_filter(clean_B, "Target")
        elif self.filter_type == 'adaptive_median':
            filtered_A = self._apply_adaptive_median_filter(clean_A, "Source")
            filtered_B = self._apply_adaptive_median_filter(clean_B, "Target")
        else:
            logger.warning(f"Unknown filter type {self.filter_type}, skipping filtering")
            return clean_A, clean_B
        
        # Step 4: 分析滤波后的数据质量
        self._analyze_filtered_data(clean_A, clean_B, filtered_A, filtered_B)
        
        # Step 5: 可视化滤波效果
        self._visualize_filtering_results(clean_A, clean_B, filtered_A, filtered_B)
        
        logger.info("Spatial filtering preprocessing completed")
        
        return filtered_A, filtered_B
    
    def _apply_median_filter(self, points, point_type):
        """应用中值滤波器（修复版本）"""
        logger.info(f"Applying median filter to {point_type} point cloud...")
        
        points = np.array(points)
        filtered_points = np.copy(points)
        
        # 确保没有NaN值
        if np.isnan(points).any():
            logger.error(f"Found NaN values in {point_type} before median filtering")
            return points
        
        # 只在XZ平面上进行滤波
        points_xz = points[:, [0, 2]]
        
        # 自适应调整k值
        if self.adaptive_k:
            actual_k = self._compute_adaptive_k(points_xz)
        else:
            actual_k = self.k_neighbors
        
        # 确保k值不超过点数
        actual_k = min(actual_k, len(points) - 1)
        if actual_k < 1:
            logger.warning(f"Not enough points for filtering {point_type}, returning original")
            return points
        
        self.actual_k_used = actual_k
        logger.info(f"Using k={actual_k} neighbors for {point_type} cloud")
        
        try:
            # 构建KNN
            nbrs = NearestNeighbors(n_neighbors=actual_k+1, algorithm='auto').fit(points_xz)
            distances, indices = nbrs.kneighbors(points_xz)
            
            # 记录滤波前的噪声水平
            noise_before = self._estimate_noise_level_safe(points_xz, indices)
            
            # 对每个点应用中值滤波
            displacement_magnitudes = []
            
            for i in range(len(points)):
                neighbor_indices = indices[i, 1:]  # 排除自身
                neighbor_points_xz = points_xz[neighbor_indices]
                
                # 计算中位数位置
                median_xz = np.median(neighbor_points_xz, axis=0)
                
                # 记录位移大小
                displacement = np.linalg.norm(points_xz[i] - median_xz)
                displacement_magnitudes.append(displacement)
                
                # 如果preserve_shape为True，限制最大位移
                if self.preserve_shape:
                    max_displacement = np.std(neighbor_points_xz, axis=0) * 2.0
                    displacement_vector = median_xz - points_xz[i]
                    displacement_norm = np.linalg.norm(displacement_vector)
                    
                    if displacement_norm > 0:
                        max_allowed = np.mean(max_displacement)
                        if displacement_norm > max_allowed:
                            displacement_vector = displacement_vector * (max_allowed / displacement_norm)
                            median_xz = points_xz[i] + displacement_vector
                
                # 更新坐标
                filtered_points[i, 0] = median_xz[0]
                filtered_points[i, 2] = median_xz[1]
                filtered_points[i, 1] = 0  # 确保Y=0
            
            # 记录滤波统计信息
            displacement_magnitudes = np.array(displacement_magnitudes)
            noise_after = self._estimate_noise_level_safe(filtered_points[:, [0, 2]], indices)
            
            self.filtering_stats[point_type] = {
                'k_used': actual_k,
                'mean_displacement': np.mean(displacement_magnitudes),
                'max_displacement': np.max(displacement_magnitudes),
                'noise_before': noise_before,
                'noise_after': noise_after,
                'noise_reduction': (noise_before - noise_after) / noise_before * 100 if noise_before > 0 else 0
            }
            
            logger.info(f"{point_type} median filtering completed:")
            logger.info(f"  Mean displacement: {np.mean(displacement_magnitudes)*1000:.3f}mm")
            logger.info(f"  Max displacement: {np.max(displacement_magnitudes)*1000:.3f}mm")
            logger.info(f"  Noise reduction: {self.filtering_stats[point_type]['noise_reduction']:.1f}%")
            
            return filtered_points
            
        except Exception as e:
            logger.error(f"Error in median filtering for {point_type}: {e}")
            return points
    
    def _apply_gaussian_filter(self, points, point_type):
        """应用高斯滤波器（修复版本）"""
        logger.info(f"Applying Gaussian filter to {point_type} point cloud...")
        
        points = np.array(points)
        filtered_points = np.copy(points)
        
        # 确保没有NaN值
        if np.isnan(points).any():
            logger.error(f"Found NaN values in {point_type} before Gaussian filtering")
            return points
        
        points_xz = points[:, [0, 2]]
        
        # 构建KNN
        if self.adaptive_k:
            actual_k = self._compute_adaptive_k(points_xz)
        else:
            actual_k = self.k_neighbors
        
        actual_k = min(actual_k, len(points) - 1)
        if actual_k < 1:
            return points
        
        try:
            nbrs = NearestNeighbors(n_neighbors=actual_k+1, algorithm='auto').fit(points_xz)
            distances, indices = nbrs.kneighbors(points_xz)
            
            # 对每个点应用高斯加权平均
            for i in range(len(points)):
                neighbor_indices = indices[i, 1:]
                neighbor_points_xz = points_xz[neighbor_indices]
                neighbor_distances = distances[i, 1:]
                
                # 计算高斯权重
                weights = np.exp(-(neighbor_distances**2) / (2 * self.gaussian_sigma**2))
                weights = weights / np.sum(weights)  # 归一化
                
                # 加权平均
                weighted_mean_xz = np.average(neighbor_points_xz, axis=0, weights=weights)
                
                filtered_points[i, 0] = weighted_mean_xz[0]
                filtered_points[i, 2] = weighted_mean_xz[1]
                filtered_points[i, 1] = 0
            
            logger.info(f"{point_type} Gaussian filtering completed")
            return filtered_points
            
        except Exception as e:
            logger.error(f"Error in Gaussian filtering for {point_type}: {e}")
            return points
    
    def _apply_bilateral_filter(self, points, point_type):
        """应用双边滤波器（修复版本）"""
        logger.info(f"Applying bilateral filter to {point_type} point cloud...")
        
        points = np.array(points)
        filtered_points = np.copy(points)
        
        if np.isnan(points).any():
            logger.error(f"Found NaN values in {point_type} before bilateral filtering")
            return points
        
        points_xz = points[:, [0, 2]]
        
        # 参数
        spatial_sigma = self.gaussian_sigma
        intensity_sigma = spatial_sigma * 0.5
        
        if self.adaptive_k:
            actual_k = self._compute_adaptive_k(points_xz)
        else:
            actual_k = self.k_neighbors
        
        actual_k = min(actual_k, len(points) - 1)
        if actual_k < 1:
            return points
        
        try:
            nbrs = NearestNeighbors(n_neighbors=actual_k+1, algorithm='auto').fit(points_xz)
            distances, indices = nbrs.kneighbors(points_xz)
            
            for i in range(len(points)):
                neighbor_indices = indices[i, 1:]
                neighbor_points_xz = points_xz[neighbor_indices]
                neighbor_distances = distances[i, 1:]
                
                # 空间权重
                spatial_weights = np.exp(-(neighbor_distances**2) / (2 * spatial_sigma**2))
                
                # 强度权重
                intensity_diffs = np.linalg.norm(neighbor_points_xz - points_xz[i], axis=1)
                intensity_weights = np.exp(-(intensity_diffs**2) / (2 * intensity_sigma**2))
                
                # 组合权重
                combined_weights = spatial_weights * intensity_weights
                combined_weights = combined_weights / np.sum(combined_weights)
                
                # 加权平均
                bilateral_mean_xz = np.average(neighbor_points_xz, axis=0, weights=combined_weights)
                
                filtered_points[i, 0] = bilateral_mean_xz[0]
                filtered_points[i, 2] = bilateral_mean_xz[1]
                filtered_points[i, 1] = 0
            
            logger.info(f"{point_type} bilateral filtering completed")
            return filtered_points
            
        except Exception as e:
            logger.error(f"Error in bilateral filtering for {point_type}: {e}")
            return points
    
    def _apply_adaptive_median_filter(self, points, point_type):
        """应用自适应中值滤波器（修复版本）"""
        logger.info(f"Applying adaptive median filter to {point_type} point cloud...")
        
        points = np.array(points)
        filtered_points = np.copy(points)
        
        if np.isnan(points).any():
            logger.error(f"Found NaN values in {point_type} before adaptive median filtering")
            return points
        
        points_xz = points[:, [0, 2]]
        
        # 初始k值
        base_k = self.k_neighbors if not self.adaptive_k else self._compute_adaptive_k(points_xz)
        base_k = min(base_k, len(points) - 1)
        
        if base_k < 1:
            return points
        
        try:
            max_neighbors = min(base_k*3, len(points)-1)
            nbrs = NearestNeighbors(n_neighbors=max_neighbors+1, algorithm='auto').fit(points_xz)
            distances, indices = nbrs.kneighbors(points_xz)
            
            for i in range(len(points)):
                # 从小到大尝试不同的k值
                for k in [base_k, base_k*2, max_neighbors]:
                    if k >= len(indices[i]) - 1:
                        k = len(indices[i]) - 2
                    
                    if k < 1:
                        break
                    
                    neighbor_indices = indices[i, 1:k+1]
                    neighbor_points_xz = points_xz[neighbor_indices]
                    
                    # 计算中值
                    median_xz = np.median(neighbor_points_xz, axis=0)
                    
                    # 检查当前点是否为离群点
                    current_to_median_dist = np.linalg.norm(points_xz[i] - median_xz)
                    neighbor_to_median_dists = np.linalg.norm(neighbor_points_xz - median_xz, axis=1)
                    
                    # 如果当前点到中值的距离在合理范围内，使用该k值
                    threshold = np.percentile(neighbor_to_median_dists, 75) + 1.5 * np.std(neighbor_to_median_dists)
                    
                    if current_to_median_dist <= threshold or k == max_neighbors:
                        filtered_points[i, 0] = median_xz[0]
                        filtered_points[i, 2] = median_xz[1]
                        filtered_points[i, 1] = 0
                        break
            
            logger.info(f"{point_type} adaptive median filtering completed")
            return filtered_points
            
        except Exception as e:
            logger.error(f"Error in adaptive median filtering for {point_type}: {e}")
            return points
    
    def _compute_adaptive_k(self, points_xz):
        """根据点云密度自适应计算k值"""
        n_points = len(points_xz)
        
        if n_points < 10:
            return max(1, min(3, n_points - 1))
        
        # 计算点云的边界框
        x_range = np.max(points_xz[:, 0]) - np.min(points_xz[:, 0])
        z_range = np.max(points_xz[:, 1]) - np.min(points_xz[:, 1])
        area = x_range * z_range
        
        # 计算点密度
        density = n_points / area if area > 0 else n_points
        
        # 基于密度调整k值
        if density > 1000:  # 高密度
            adaptive_k = max(15, min(25, int(self.k_neighbors * 1.5)))
        elif density > 500:  # 中等密度
            adaptive_k = self.k_neighbors
        else:  # 低密度
            adaptive_k = max(5, int(self.k_neighbors * 0.7))
        
        # 确保k值不超过点数
        adaptive_k = min(adaptive_k, n_points - 1)
        
        logger.info(f"Point density: {density:.1f} points/m², adaptive k: {adaptive_k}")
        
        return adaptive_k
    
    def _estimate_noise_level_safe(self, points_xz, indices):
        """安全的噪声水平估计（处理NaN值）"""
        try:
            # 确保输入数据没有NaN
            if np.isnan(points_xz).any():
                logger.warning("Found NaN values in noise estimation, using fallback method")
                # 计算点云标准差作为噪声估计
                return np.std(points_xz[~np.isnan(points_xz)])
            
            neighbor_distances = []
            
            for i in range(min(len(points_xz), len(indices))):
                if len(indices[i]) > 1:
                    nearest_neighbor_idx = indices[i, 1]  # 最近的邻居
                    if nearest_neighbor_idx < len(points_xz):
                        distance = np.linalg.norm(points_xz[i] - points_xz[nearest_neighbor_idx])
                        if not np.isnan(distance):
                            neighbor_distances.append(distance)
            
            if neighbor_distances:
                noise_estimate = np.std(neighbor_distances)
            else:
                # 回退方案
                noise_estimate = np.std(points_xz)
            
            return noise_estimate if not np.isnan(noise_estimate) else 0.0
            
        except Exception as e:
            logger.warning(f"Error in noise estimation: {e}, using fallback")
            return 0.001  # 默认噪声水平
    
    def _analyze_original_data(self, positions_A, positions_B):
        """分析原始数据质量（安全版本）"""
        logger.info("Analyzing original data quality...")
        
        for points, name in [(positions_A, "Source"), (positions_B, "Target")]:
            points_xz = points[:, [0, 2]]
            
            # 检查NaN值
            nan_count = np.isnan(points_xz).sum()
            if nan_count > 0:
                logger.warning(f"{name} cloud has {nan_count} NaN values")
                continue
            
            # 计算点间距离统计
            try:
                distances = cdist(points_xz, points_xz)
                np.fill_diagonal(distances, np.inf)
                min_distances = np.min(distances, axis=1)
                
                logger.info(f"{name} cloud statistics:")
                logger.info(f"  Points: {len(points)}")
                logger.info(f"  X range: [{np.min(points_xz[:, 0]):.4f}, {np.max(points_xz[:, 0]):.4f}]")
                logger.info(f"  Z range: [{np.min(points_xz[:, 1]):.4f}, {np.max(points_xz[:, 1]):.4f}]")
                logger.info(f"  Min inter-point distance: {np.min(min_distances)*1000:.3f}mm")
                logger.info(f"  Mean inter-point distance: {np.mean(min_distances)*1000:.3f}mm")
            except Exception as e:
                logger.error(f"Error analyzing {name} cloud: {e}")
    
    def _analyze_filtered_data(self, original_A, original_B, filtered_A, filtered_B):
        """分析滤波后的数据质量变化（安全版本）"""
        logger.info("Analyzing filtering effectiveness...")
        
        for (orig, filt, name) in [(original_A, filtered_A, "Source"), (original_B, filtered_B, "Target")]:
            try:
                # 检查NaN值
                if np.isnan(orig).any() or np.isnan(filt).any():
                    logger.warning(f"Found NaN values in {name} data, skipping detailed analysis")
                    continue
                
                # 计算位移统计
                displacements = np.linalg.norm(orig[:, [0, 2]] - filt[:, [0, 2]], axis=1)
                
                # 使用安全的噪声估计
                # 创建简单的临时邻居查找
                try:
                    if len(orig) > 5:
                        nbrs_orig = NearestNeighbors(n_neighbors=min(6, len(orig))).fit(orig[:, [0, 2]])
                        _, indices_orig = nbrs_orig.kneighbors(orig[:, [0, 2]])
                        orig_noise = self._estimate_noise_level_safe(orig[:, [0, 2]], indices_orig)
                        
                        nbrs_filt = NearestNeighbors(n_neighbors=min(6, len(filt))).fit(filt[:, [0, 2]])
                        _, indices_filt = nbrs_filt.kneighbors(filt[:, [0, 2]])
                        filt_noise = self._estimate_noise_level_safe(filt[:, [0, 2]], indices_filt)
                    else:
                        orig_noise = np.std(orig[:, [0, 2]])
                        filt_noise = np.std(filt[:, [0, 2]])
                except:
                    orig_noise = np.std(orig[:, [0, 2]])
                    filt_noise = np.std(filt[:, [0, 2]])
                
                logger.info(f"{name} filtering effectiveness:")
                logger.info(f"  Mean displacement: {np.mean(displacements)*1000:.3f}mm")
                logger.info(f"  Max displacement: {np.max(displacements)*1000:.3f}mm")
                logger.info(f"  Noise before: {orig_noise*1000:.3f}mm")
                logger.info(f"  Noise after: {filt_noise*1000:.3f}mm")
                if orig_noise > 0:
                    logger.info(f"  Noise reduction: {(orig_noise-filt_noise)/orig_noise*100:.1f}%")
                
            except Exception as e:
                logger.error(f"Error analyzing filtering effectiveness for {name}: {e}")
    
    def _visualize_filtering_results(self, original_A, original_B, filtered_A, filtered_B):
        """可视化滤波结果（简化版本）"""
        try:
            fig, axes = plt.subplots(2, 2, figsize=(16, 12))
            
            # 源点云对比
            ax1 = axes[0, 0]
            ax1.scatter(original_A[:, 0], original_A[:, 2], c='blue', alpha=0.6, s=20, label='Original')
            ax1.scatter(filtered_A[:, 0], filtered_A[:, 2], c='red', alpha=0.6, s=20, label='Filtered')
            ax1.set_xlabel('X [m]')
            ax1.set_ylabel('Z [m]')
            ax1.set_title('Source Point Cloud: Before vs After Filtering')
            ax1.legend()
            ax1.grid(True, alpha=0.3)
            
            # 目标点云对比
            ax2 = axes[0, 1]
            ax2.scatter(original_B[:, 0], original_B[:, 2], c='blue', alpha=0.6, s=20, label='Original')
            ax2.scatter(filtered_B[:, 0], filtered_B[:, 2], c='red', alpha=0.6, s=20, label='Filtered')
            ax2.set_xlabel('X [m]')
            ax2.set_ylabel('Z [m]')
            ax2.set_title('Target Point Cloud: Before vs After Filtering')
            ax2.legend()
            ax2.grid(True, alpha=0.3)
            
            # 位移大小分布
            ax3 = axes[1, 0]
            displacements_A = np.linalg.norm((filtered_A - original_A)[:, [0, 2]], axis=1)
            displacements_B = np.linalg.norm((filtered_B - original_B)[:, [0, 2]], axis=1)
            
            ax3.hist(displacements_A*1000, bins=30, alpha=0.7, label='Source', density=True)
            ax3.hist(displacements_B*1000, bins=30, alpha=0.7, label='Target', density=True)
            ax3.set_xlabel('Displacement Magnitude [mm]')
            ax3.set_ylabel('Density')
            ax3.set_title('Filtering Displacement Distribution')
            ax3.legend()
            ax3.grid(True, alpha=0.3)
            
            # 滤波效果统计
            ax4 = axes[1, 1]
            if hasattr(self, 'filtering_stats') and self.filtering_stats:
                stats_data = []
                labels = []
                for point_type, stats in self.filtering_stats.items():
                    stats_data.extend([
                        stats['mean_displacement']*1000,
                        stats['max_displacement']*1000
                    ])
                    labels.extend([f'{point_type}\nMean [mm]', f'{point_type}\nMax [mm]'])
                
                if stats_data:
                    ax4.bar(range(len(stats_data)), stats_data)
                    ax4.set_xticks(range(len(stats_data)))
                    ax4.set_xticklabels(labels, rotation=45, ha='right')
                    ax4.set_ylabel('Displacement [mm]')
                    ax4.set_title('Filtering Statistics')
                    ax4.grid(True, alpha=0.3)
            
            plt.tight_layout()
            plt.savefig(os.path.join(img_dir, f'spatial_filtering_results_{self.filter_type}.png'), 
                       dpi=300, bbox_inches='tight')
            plt.close()
            
            logger.info(f"Spatial filtering visualization saved: spatial_filtering_results_{self.filter_type}.png")
            
        except Exception as e:
            logger.error(f"Error creating filtering visualization: {e}")
    
    def save_filtering_report(self):
        """保存滤波报告"""
        try:
            report_file = os.path.join(result_dir, "spatial_filtering_report.txt")
            with open(report_file, "w") as f:
                f.write("Spatial Filtering Report\n")
                f.write("=" * 50 + "\n")
                f.write(f"Filter Type: {self.filter_type}\n")
                f.write(f"K Neighbors: {self.k_neighbors}\n")
                f.write(f"Adaptive K: {self.adaptive_k}\n")
                f.write(f"Preserve Shape: {self.preserve_shape}\n")
                f.write(f"Actual K Used: {self.actual_k_used}\n")
                
                if hasattr(self, 'filtering_stats') and self.filtering_stats:
                    f.write("\nFiltering Statistics:\n")
                    f.write("-" * 30 + "\n")
                    for point_type, stats in self.filtering_stats.items():
                        f.write(f"\n{point_type} Point Cloud:\n")
                        f.write(f"  K used: {stats['k_used']}\n")
                        f.write(f"  Mean displacement: {stats['mean_displacement']*1000:.3f}mm\n")
                        f.write(f"  Max displacement: {stats['max_displacement']*1000:.3f}mm\n")
                        f.write(f"  Noise before: {stats['noise_before']*1000:.3f}mm\n")
                        f.write(f"  Noise after: {stats['noise_after']*1000:.3f}mm\n")
                        f.write(f"  Noise reduction: {stats['noise_reduction']:.1f}%\n")
            
            logger.info("Spatial filtering report saved")
        except Exception as e:
            logger.error(f"Error saving filtering report: {e}")

# 保留原有的其他类（简化版本）
class DataPreprocessor:
    """简化的数据预处理器"""
    def __init__(self):
        self.duplicate_threshold = 1e-6
        
    def remove_duplicates(self, positions_A, positions_B):
        """移除重复点"""
        positions_A = np.array(positions_A)
        positions_B = np.array(positions_B)
        
        # 检查NaN值
        if np.isnan(positions_A).any() or np.isnan(positions_B).any():
            logger.warning("Found NaN values before duplicate removal")
        
        try:
            distances = cdist(positions_A[:, [0, 2]], positions_A[:, [0, 2]])
            duplicate_pairs = np.where((distances < self.duplicate_threshold) & (distances > 0))
            
            points_to_remove = set()
            for i, j in zip(duplicate_pairs[0], duplicate_pairs[1]):
                if i < j:
                    points_to_remove.add(j)
            
            keep_mask = np.ones(len(positions_A), dtype=bool)
            for idx in points_to_remove:
                keep_mask[idx] = False
            
            logger.info(f"Removed {len(points_to_remove)} duplicate points")
            
            return positions_A[keep_mask], positions_B[keep_mask]
        except Exception as e:
            logger.error(f"Error removing duplicates: {e}")
            return positions_A, positions_B

class OutlierDetector:
    """简化的离群点检测器"""
    def __init__(self, iqr_factor=2.0):
        self.iqr_factor = iqr_factor
        
    def detect_outliers_initial_svd(self, positions_A, positions_B):
        """使用初始SVD检测离群点"""
        try:
            positions_A = np.array(positions_A)
            positions_B = np.array(positions_B)
            
            # 检查NaN值
            if np.isnan(positions_A).any() or np.isnan(positions_B).any():
                logger.error("Found NaN values in outlier detection input")
                return np.ones(len(positions_A), dtype=bool)  # 返回全部保留
            
            # 执行初始SVD
            positions_A_xz = positions_A[:, [0, 2]]
            positions_B_xz = positions_B[:, [0, 2]]

            centroid_A_xz = np.mean(positions_A_xz, axis=0)
            centroid_B_xz = np.mean(positions_B_xz, axis=0)

            H = np.dot((positions_A_xz - centroid_A_xz).T, (positions_B_xz - centroid_B_xz))
            U, S, Vt = np.linalg.svd(H)
            R_2d = np.dot(Vt.T, U.T)
            if np.linalg.det(R_2d) < 0:
                Vt[-1, :] *= -1
                R_2d = np.dot(Vt.T, U.T)
            
            translation_2d = centroid_B_xz.T - np.dot(R_2d, centroid_A_xz.T)
            
            # 构建变换矩阵
            T_initial = np.eye(4)
            T_initial[0, 0] = R_2d[0, 0]
            T_initial[0, 2] = R_2d[0, 1]
            T_initial[2, 0] = R_2d[1, 0]
            T_initial[2, 2] = R_2d[1, 1]
            T_initial[0, 3] = translation_2d[0]
            T_initial[2, 3] = translation_2d[1]
            
            # 应用变换
            positions_A_homo = np.hstack([positions_A, np.ones((positions_A.shape[0], 1))])
            transformed = np.dot(T_initial, positions_A_homo.T).T
            svd_transformed = transformed[:, :3]
            svd_transformed[:, 1] = 0
            
            # 计算残差
            residuals = positions_B - svd_transformed
            residual_magnitudes = np.sqrt(np.sum(residuals**2, axis=1))
            
            # IQR方法检测离群点
            Q1 = np.percentile(residual_magnitudes, 25)
            Q3 = np.percentile(residual_magnitudes, 75)
            IQR = Q3 - Q1
            threshold = Q3 + self.iqr_factor * IQR
            
            outlier_mask = residual_magnitudes > threshold
            
            logger.info(f"Outlier detection: {np.sum(outlier_mask)} outliers removed (threshold: {threshold*1000:.3f}mm)")
            
            return ~outlier_mask
            
        except Exception as e:
            logger.error(f"Error in outlier detection: {e}")
            return np.ones(len(positions_A), dtype=bool)  # 返回全部保留

class RobustRBFModel:
    """简化的RBF模型"""
    def __init__(self, method='thin_plate_spline', smoothing=0.001):
        self.method = method
        self.smoothing = max(smoothing, 1e-6)  # 避免0
        self.rbf_x = None
        self.rbf_z = None
        
    def learn_deformation(self, source_points, residual_vectors):
        """学习RBF形变模型"""
        try:
            source_points = np.array(source_points)
            residual_vectors = np.array(residual_vectors)
            
            # 检查NaN值
            if np.isnan(source_points).any() or np.isnan(residual_vectors).any():
                logger.error("Found NaN values in RBF training data")
                return False
            
            source_xz = source_points[:, [0, 2]]
            residual_xz = residual_vectors[:, [0, 2]]
            
            self.rbf_x = RBFInterpolator(source_xz, residual_xz[:, 0], 
                                       kernel=self.method, smoothing=self.smoothing)
            self.rbf_z = RBFInterpolator(source_xz, residual_xz[:, 1], 
                                       kernel=self.method, smoothing=self.smoothing)
            return True
        except Exception as e:
            logger.error(f"Failed to learn RBF model: {e}")
            return False
    
    def predict_deformation(self, query_points):
        """预测形变"""
        try:
            if self.rbf_x is None or self.rbf_z is None:
                return np.zeros_like(query_points)
            
            query_points = np.array(query_points)
            
            # 检查NaN值
            if np.isnan(query_points).any():
                logger.error("Found NaN values in RBF prediction input")
                return np.zeros_like(query_points)
            
            query_xz = query_points[:, [0, 2]]
            
            deformation_x = self.rbf_x(query_xz)
            deformation_z = self.rbf_z(query_xz)
            
            deformation = np.zeros_like(query_points)
            deformation[:, 0] = deformation_x
            deformation[:, 2] = deformation_z
            
            return deformation
        except Exception as e:
            logger.error(f"Failed to predict deformation: {e}")
            return np.zeros_like(query_points)

class CoordinateTransformer:
    """增强的坐标变换器，集成空间滤波"""
    def __init__(self, positions_A=None, positions_B=None):
        self.T_svd = None
        self.deformation_model = None
        self.__positions_A = positions_A
        self.__positions_B = positions_B
        self.preprocessor = DataPreprocessor()
        self.outlier_detector = OutlierDetector()
        
    def apply_transformation(self, positions, T):
        """应用变换矩阵"""
        try:
            positions = np.array(positions)
            
            # 检查NaN值
            if np.isnan(positions).any():
                logger.error("Found NaN values in transformation input")
                return positions
            
            positions_homo = np.hstack([positions, np.ones((positions.shape[0], 1))])
            transformed = np.dot(T, positions_homo.T).T
            result = transformed[:, :3]
            result[:, 1] = 0
            return result
        except Exception as e:
            logger.error(f"Error in transformation: {e}")
            return positions
    
    def calculate_directional_rmse(self, actual, predicted):
        """计算各方向RMSE"""
        try:
            actual = np.array(actual)
            predicted = np.array(predicted)
            
            # 检查NaN值
            if np.isnan(actual).any() or np.isnan(predicted).any():
                logger.error("Found NaN values in RMSE calculation")
                return {'lateral_rmse': float('inf'), 'longitudinal_rmse': float('inf'), 'overall_rmse': float('inf')}
            
            rmse_lateral = np.sqrt(np.mean((actual[:, 0] - predicted[:, 0])**2))
            rmse_longitudinal = np.sqrt(np.mean((actual[:, 2] - predicted[:, 2])**2))
            overall_rmse = np.sqrt(rmse_lateral**2 + rmse_longitudinal**2)
            
            return {
                'lateral_rmse': rmse_lateral,
                'longitudinal_rmse': rmse_longitudinal,
                'overall_rmse': overall_rmse
            }
        except Exception as e:
            logger.error(f"Error calculating RMSE: {e}")
            return {'lateral_rmse': float('inf'), 'longitudinal_rmse': float('inf'), 'overall_rmse': float('inf')}
    
    def visualize_transformation_stage(self, stage_name, positions_A, positions_B, transformed_positions, rmse_results):
        """可视化变换结果"""
        try:
            fig, axes = plt.subplots(2, 2, figsize=(16, 12))
            
            individual_errors = np.sqrt((positions_B[:, 0] - transformed_positions[:, 0])**2 + 
                                       (positions_B[:, 2] - transformed_positions[:, 2])**2)
            
            # 3D散点图
            ax1 = axes[0, 0]
            ax1.scatter(positions_A[:, 0], positions_A[:, 2], c='blue', marker='o', s=30, 
                       label='Source Points', alpha=0.7)
            ax1.scatter(positions_B[:, 0], positions_B[:, 2], c='red', marker='^', s=30, 
                       label='Target Points', alpha=0.7)
            ax1.scatter(transformed_positions[:, 0], transformed_positions[:, 2], c='green', marker='x', s=30, 
                       label='Transformed Points', alpha=0.7)
            ax1.set_xlabel('X [m]')
            ax1.set_ylabel('Z [m]')
            ax1.set_title(f'{stage_name} - Point Cloud Registration')
            ax1.legend()
            ax1.grid(True, alpha=0.3)
            
            # 误差分布
            ax2 = axes[0, 1]
            ax2.hist(individual_errors * 1000, bins=20, color='skyblue', edgecolor='black', alpha=0.7)
            ax2.axvline(x=TARGET_RMSE*1000, color='r', linestyle='--', linewidth=2, 
                       label=f'Target RMSE ({TARGET_RMSE*1000:.1f}mm)')
            ax2.set_xlabel('Individual Point Error [mm]')
            ax2.set_ylabel('Frequency')
            ax2.set_title(f'{stage_name} - Error Distribution')
            ax2.legend()
            ax2.grid(True, alpha=0.3)
            
            # RMSE条形图
            ax3 = axes[1, 0]
            directions = ['Lateral', 'Longitudinal', 'Overall']
            rmse_values = [
                rmse_results['lateral_rmse'] * 1000,
                rmse_results['longitudinal_rmse'] * 1000,
                rmse_results['overall_rmse'] * 1000
            ]
            targets = [SCALED_LATERAL_TARGET * 1000, SCALED_LONGITUDINAL_TARGET * 1000, TARGET_RMSE * 1000]
            
            bars = ax3.bar(directions, rmse_values, alpha=0.8)
            for i, (bar, target) in enumerate(zip(bars, targets)):
                ax3.axhline(y=target, color='red', linestyle='--', alpha=0.7)
                if rmse_values[i] <= target:
                    bar.set_color('green')
                else:
                    bar.set_color('red')
            
            ax3.set_ylabel('RMSE [mm]')
            ax3.set_title(f'{stage_name} - RMSE by Direction')
            ax3.grid(True, alpha=0.3)
            
            # 误差空间分布
            ax4 = axes[1, 1]
            scatter = ax4.scatter(positions_A[:, 0], positions_A[:, 2], c=individual_errors*1000, 
                                cmap='viridis', s=30, alpha=0.7)
            plt.colorbar(scatter, ax=ax4, label='Error [mm]')
            ax4.set_xlabel('X [m]')
            ax4.set_ylabel('Z [m]')
            ax4.set_title(f'{stage_name} - Spatial Error Distribution')
            ax4.grid(True, alpha=0.3)
            
            plt.tight_layout()
            filename = f'{stage_name.lower().replace(" ", "_")}_results.png'
            plt.savefig(os.path.join(img_dir, filename), dpi=300, bbox_inches='tight')
            plt.close()
        except Exception as e:
            logger.error(f"Error creating visualization: {e}")
    
    def spatial_filtered_registration(self, filter_type='median', k_neighbors=10):
        """空间滤波增强的两阶段配准"""
        if self.__positions_A is None or self.__positions_B is None:
            logger.error("Please provide position data first!")
            return None, None
        
        logger.info("=== Starting Spatial Filtering Enhanced Registration ===")
        
        positions_A = np.array(self.__positions_A)
        positions_B = np.array(self.__positions_B)
        
        logger.info(f"Original dataset: {len(positions_A)} point pairs")
        
        # Step 1: 空间滤波预处理（包含缺失值处理）
        logger.info("Step 1: Spatial Filtering Preprocessing")
        spatial_filter = SpatialFilter(
            filter_type=filter_type,
            k_neighbors=k_neighbors,
            adaptive_k=True,
            preserve_shape=True
        )
        
        filtered_A, filtered_B = spatial_filter.apply_spatial_filter(positions_A, positions_B)
        spatial_filter.save_filtering_report()
        
        # Step 2: 数据预处理（去重）
        logger.info("Step 2: Data Preprocessing")
        clean_A, clean_B = self.preprocessor.remove_duplicates(filtered_A, filtered_B)
        
        # Step 3: 离群点检测
        logger.info("Step 3: Outlier Detection")
        clean_mask = self.outlier_detector.detect_outliers_initial_svd(clean_A, clean_B)
        final_A = clean_A[clean_mask]
        final_B = clean_B[clean_mask]
        
        logger.info(f"Final clean dataset: {len(final_A)} point pairs")
        
        # 检查最终数据集是否有效
        if len(final_A) < 3:
            logger.error("Not enough valid points for registration")
            return None, None
        
        # Step 4: SVD粗配准
        logger.info("Step 4: SVD Coarse Registration")
        try:
            positions_A_xz = final_A[:, [0, 2]]
            positions_B_xz = final_B[:, [0, 2]]

            centroid_A_xz = np.mean(positions_A_xz, axis=0)
            centroid_B_xz = np.mean(positions_B_xz, axis=0)

            H = np.dot((positions_A_xz - centroid_A_xz).T, (positions_B_xz - centroid_B_xz))
            U, S, Vt = np.linalg.svd(H)
            R_2d = np.dot(Vt.T, U.T)
            if np.linalg.det(R_2d) < 0:
                Vt[-1, :] *= -1
                R_2d = np.dot(Vt.T, U.T)
            
            translation_2d = centroid_B_xz.T - np.dot(R_2d, centroid_A_xz.T)
            
            self.T_svd = np.eye(4)
            self.T_svd[0, 0] = R_2d[0, 0]
            self.T_svd[0, 2] = R_2d[0, 1]
            self.T_svd[2, 0] = R_2d[1, 0]
            self.T_svd[2, 2] = R_2d[1, 1]
            self.T_svd[0, 3] = translation_2d[0]
            self.T_svd[2, 3] = translation_2d[1]
            
            svd_transformed = self.apply_transformation(final_A, self.T_svd)
            svd_rmse_results = self.calculate_directional_rmse(final_B, svd_transformed)
            
            logger.info(f"SVD Overall RMSE: {svd_rmse_results['overall_rmse']*1000:.3f}mm")
            
            self.visualize_transformation_stage("SVD Coarse Registration", 
                                               final_A, final_B, svd_transformed, svd_rmse_results)
            
            # Step 5: 非刚性形变学习
            logger.info("Step 5: Non-rigid Deformation Learning")
            residual_vectors = final_B - svd_transformed
            
            self.deformation_model = RobustRBFModel(method='thin_plate_spline', smoothing=0.001)
            success = self.deformation_model.learn_deformation(svd_transformed, residual_vectors)
            
            if not success:
                logger.error("Failed to learn deformation model")
                return self.T_svd, svd_rmse_results
            
            # 应用非刚性形变
            predicted_deformation = self.deformation_model.predict_deformation(svd_transformed)
            final_transformed = svd_transformed + predicted_deformation
            final_transformed[:, 1] = 0
            
            final_rmse_results = self.calculate_directional_rmse(final_B, final_transformed)
            
            logger.info(f"Final Overall RMSE: {final_rmse_results['overall_rmse']*1000:.3f}mm")
            
            improvement = (svd_rmse_results['overall_rmse'] - final_rmse_results['overall_rmse']) * 1000
            logger.info(f"Improvement: {improvement:.3f}mm")
            
            self.visualize_transformation_stage("Final Spatial Filtered Registration", 
                                               final_A, final_B, final_transformed, final_rmse_results)
            
            return self.T_svd, final_rmse_results
            
        except Exception as e:
            logger.error(f"Error in SVD registration: {e}")
            return None, None

def process_laser_tracker_data(input_file):
    """处理激光跟踪器数据（增强调试版本）"""
    logger.info(f"Processing laser tracker data from {input_file}")
    
    try:
        # 添加详细的数据加载分析
        logger.info("Analyzing CSV file structure...")
        
        # 1. 检查文件基本信息
        with open(input_file, 'r') as f:
            lines = f.readlines()
            logger.info(f"Total lines in file: {len(lines)}")
            logger.info(f"First few lines:")
            for i, line in enumerate(lines[:5]):
                logger.info(f"  Line {i}: {line.strip()}")
        
        # 2. 读取CSV并分析
        df = pd.read_csv(input_file)
        logger.info(f"DataFrame shape: {df.shape}")
        logger.info(f"DataFrame columns: {list(df.columns)}")
        logger.info(f"DataFrame index range: {df.index.min()} to {df.index.max()}")
        
        # 3. 检查是否有重复行
        duplicates = df.duplicated()
        num_duplicates = duplicates.sum()
        logger.info(f"Number of duplicate rows: {num_duplicates}")
        
        # 4. 检查必需的列是否存在
        required_columns = [ROW_X, ROW_Y, "laser_x", "laser_z"]
        missing_columns = [col for col in required_columns if col not in df.columns]
        if missing_columns:
            logger.error(f"Missing required columns: {missing_columns}")
            logger.info(f"Available columns: {list(df.columns)}")
            return None
        
        # 5. 检查空值
        null_counts = df[required_columns].isnull().sum()
        logger.info(f"Null values in required columns:")
        for col, count in null_counts.items():
            logger.info(f"  {col}: {count} nulls")
        
        # 6. 检查数据范围
        logger.info(f"Data ranges:")
        for col in required_columns:
            if col in df.columns:
                logger.info(f"  {col}: [{df[col].min():.6f}, {df[col].max():.6f}]")
        
        # 7. 检查数据类型
        logger.info(f"Data types:")
        for col in required_columns:
            if col in df.columns:
                logger.info(f"  {col}: {df[col].dtype}")
        
        logger.info(f"Successfully processed {len(df)} data points")
        
        return df
        
    except Exception as e:
        logger.error(f"Error processing laser tracker data: {e}")
        import traceback
        logger.error(f"Traceback: {traceback.format_exc()}")
        return None

def main():
    """主函数"""
    import os
    logger.info("Starting SPATIAL FILTERING ENHANCED (NaN-Safe) 1:32 scale model tracker data processing")
    logger.info(f"Scale factor: {SCALE_FACTOR}")
    logger.info(f"Target RMSE: {TARGET_RMSE*1000:.2f}mm")
    logger.info("Enhanced Features:")
    logger.info("  - Spatial filtering preprocessing with NaN handling")
    logger.info("  - KNN and Simple imputation for missing values")
    logger.info("  - Robust error handling throughout pipeline")
    logger.info("  - Safe nearest neighbor operations")
    
    global LASER_TRACKER_CSV
    if len(sys.argv) > 1:
        LASER_TRACKER_CSV = sys.argv[1]
    
    if not os.path.exists(LASER_TRACKER_CSV):
        logger.error(f"Laser tracker data file {LASER_TRACKER_CSV} not found.")
        logger.info("Usage: python script.py <path_to_laser_tracker.csv>")
        return
    
    # 检查文件大小和基本信息
    import os
    file_size = os.path.getsize(LASER_TRACKER_CSV)
    logger.info(f"Input file size: {file_size} bytes")
    
    # 处理激光跟踪器数据
    logger.info("Processing laser tracker data...")
    df = process_laser_tracker_data(LASER_TRACKER_CSV)
    
    if df is None:
        logger.error("Failed to process laser tracker data")
        return
    
    # 准备数据 - 添加详细检查
    logger.info("Preparing position data...")
    
    try:
        # 检查数据完整性
        required_columns = [ROW_X, ROW_Y, "laser_x", "laser_z"]
        valid_rows = df[required_columns].notna().all(axis=1)
        num_valid = valid_rows.sum()
        num_invalid = len(df) - num_valid
        
        logger.info(f"Data validation:")
        logger.info(f"  Total rows in DataFrame: {len(df)}")
        logger.info(f"  Valid rows (no NaN): {num_valid}")
        logger.info(f"  Invalid rows (with NaN): {num_invalid}")
        
        if num_invalid > 0:
            logger.warning(f"Found {num_invalid} rows with missing data - will be handled by imputation")
        
        # 创建位置数组（允许NaN值，后续由MissingValueHandler处理）
        positions_A = []
        positions_B = []
        
        for idx, row in df.iterrows():
            try:
                pos_A = [float(row[ROW_X]) if not pd.isna(row[ROW_X]) else np.nan, 
                        0.0, 
                        float(row[ROW_Y]) if not pd.isna(row[ROW_Y]) else np.nan]
                pos_B = [float(row["laser_x"]) if not pd.isna(row["laser_x"]) else np.nan, 
                        0.0, 
                        float(row["laser_z"]) if not pd.isna(row["laser_z"]) else np.nan]
                positions_A.append(pos_A)
                positions_B.append(pos_B)
            except (ValueError, KeyError) as e:
                logger.warning(f"Skipping row {idx} due to data conversion error: {e}")
                continue
        
        logger.info(f"Successfully created position arrays:")
        logger.info(f"  positions_A: {len(positions_A)} points")
        logger.info(f"  positions_B: {len(positions_B)} points")
        
        # 验证数据一致性
        if len(positions_A) != len(positions_B):
            logger.error(f"Mismatch: positions_A has {len(positions_A)} points, positions_B has {len(positions_B)} points")
            return
        
        # 显示数据范围（忽略NaN）
        positions_A_np = np.array(positions_A)
        positions_B_np = np.array(positions_B)
        
        logger.info(f"Position data ranges (excluding NaN):")
        for i, label in enumerate(['X', 'Y', 'Z']):
            valid_A = positions_A_np[:, i][~np.isnan(positions_A_np[:, i])]
            valid_B = positions_B_np[:, i][~np.isnan(positions_B_np[:, i])]
            if len(valid_A) > 0:
                logger.info(f"  Source {label}: [{valid_A.min():.6f}, {valid_A.max():.6f}]")
            if len(valid_B) > 0:
                logger.info(f"  Target {label}: [{valid_B.min():.6f}, {valid_B.max():.6f}]")
        
    except Exception as e:
        logger.error(f"Error preparing position data: {e}")
        import traceback
        logger.error(f"Traceback: {traceback.format_exc()}")
        return
    
    logger.info(f"Final dataset ready: {len(positions_A)} point pairs for registration")
    
    # 空间滤波增强的配准
    transformer = CoordinateTransformer(positions_A, positions_B)
    
    # 可以尝试不同的滤波类型和参数
    filter_configs = [
        {'filter_type': 'median', 'k_neighbors': 10},
        {'filter_type': 'adaptive_median', 'k_neighbors': 8},
        {'filter_type': 'gaussian', 'k_neighbors': 12},
        {'filter_type': 'bilateral', 'k_neighbors': 10}
    ]
    
    best_rmse = float('inf')
    best_config = None
    best_results = None
    
    for config in filter_configs:
        logger.info(f"\nTesting spatial filter configuration: {config}")
        
        T_svd, final_rmse_results = transformer.spatial_filtered_registration(
            filter_type=config['filter_type'],
            k_neighbors=config['k_neighbors']
        )
        
        if final_rmse_results and final_rmse_results['overall_rmse'] < best_rmse:
            best_rmse = final_rmse_results['overall_rmse']
            best_config = config
            best_results = final_rmse_results
    
    # 输出最佳结果
    if best_results is not None:
        success = best_results['overall_rmse'] <= TARGET_RMSE
        if success:
            logger.info(f"\nSUCCESS: Spatial filtering enhanced registration completed!")
            logger.info(f"Best configuration: {best_config}")
            logger.info(f"Best RMSE: {best_results['overall_rmse']*1000:.3f}mm")
        else:
            logger.info(f"\nPARTIAL: Registration completed, but RMSE {best_results['overall_rmse']*1000:.3f}mm exceeds target {TARGET_RMSE*1000:.2f}mm")
            logger.info(f"Best configuration: {best_config}")
        
        # 保存最终总结
        summary_file = os.path.join(output_dir, "spatial_filtering_registration_summary.txt")
        with open(summary_file, "w") as f:
            f.write("Spatial Filtering Enhanced Registration Summary (NaN-Safe)\n")
            f.write("=" * 80 + "\n")
            f.write(f"Scale Factor: {SCALE_FACTOR}\n")
            f.write(f"Target RMSE: {TARGET_RMSE*1000:.2f}mm\n")
            f.write(f"Best RMSE: {best_results['overall_rmse']*1000:.3f}mm\n")
            f.write(f"Success: {'Yes' if success else 'No'}\n")
            f.write(f"Best filter configuration: {best_config}\n")
            f.write(f"Original points: {len(positions_A)}\n")
            f.write(f"Real-world equivalent: {best_results['overall_rmse']*SCALE_FACTOR*100:.2f}cm\n")
            f.write("\nNaN-Safe spatial filtering features:\n")
            f.write("- KNNImputer and SimpleImputer for missing value handling\n")
            f.write("- Safe nearest neighbor operations\n")
            f.write("- Robust error handling throughout pipeline\n")
            f.write("- Multiple filter types with adaptive parameters\n")
            f.write("- Comprehensive data validation and debugging\n")
    else:
        logger.error("All spatial filtering configurations failed")
    
    logger.info("NaN-safe spatial filtering enhanced processing complete")

if __name__ == "__main__":
    main()