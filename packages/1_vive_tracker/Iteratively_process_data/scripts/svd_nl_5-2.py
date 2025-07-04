#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
FFD-based (Free-Form Deformation) Tracker data processing script with SVD + Control Grid Deformation
Enhanced with robust parameter selection and FFD optimization strategies
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
from scipy.interpolate import griddata, RBFInterpolator, LSQBivariateSpline
from scipy.spatial.distance import cdist
from scipy import stats
from scipy.optimize import minimize, differential_evolution
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
output_dir = os.path.join(result_dir, f"FFD_Deformation_{csv_filename}_{timestamp}_1to32_scale_RMSE_{TARGET_RMSE*1000:.1f}mm")
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
log_file = os.path.join(log_dir, "ffd_tracker_process.log")
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

ROW_X="X"
ROW_Y="Z"

class DataPreprocessor:
    """
    数据预处理器：标准化、去重、质量检查
    """
    def __init__(self):
        self.scaler_A = StandardScaler()
        self.scaler_B = StandardScaler()
        self.duplicate_threshold = 1e-6  # 重复点阈值
        
    def preprocess_data(self, positions_A, positions_B):
        """
        预处理数据：去重、标准化、质量检查
        """
        positions_A = np.array(positions_A)
        positions_B = np.array(positions_B)
        
        logger.info("Starting data preprocessing...")
        
        # 1. 检查数据基本统计
        self._analyze_data_quality(positions_A, positions_B)
        
        # 2. 去除重复点
        clean_A, clean_B, duplicate_mask = self._remove_duplicates(positions_A, positions_B)
        
        # 3. 检查点间最小距离
        min_distances = self._check_minimum_distances(clean_A)
        
        # 4. 数据标准化（可选，用于某些算法）
        normalized_A, normalized_B = self._normalize_data(clean_A, clean_B)
        
        preprocessing_info = {
            'original_points': len(positions_A),
            'after_deduplication': len(clean_A),
            'duplicates_removed': np.sum(duplicate_mask),
            'min_distance_between_points': np.min(min_distances),
            'mean_distance_between_points': np.mean(min_distances)
        }
        
        logger.info(f"Preprocessing results:")
        logger.info(f"  Original points: {preprocessing_info['original_points']}")
        logger.info(f"  After deduplication: {preprocessing_info['after_deduplication']}")
        logger.info(f"  Duplicates removed: {preprocessing_info['duplicates_removed']}")
        logger.info(f"  Min distance between points: {preprocessing_info['min_distance_between_points']*1000:.3f}mm")
        logger.info(f"  Mean distance between points: {preprocessing_info['mean_distance_between_points']*1000:.3f}mm")
        
        return clean_A, clean_B, preprocessing_info, normalized_A, normalized_B
    
    def _analyze_data_quality(self, positions_A, positions_B):
        """分析数据质量"""
        # 检查XZ平面的分布
        A_xz = positions_A[:, [0, 2]]
        B_xz = positions_B[:, [0, 2]]
        
        logger.info("Data quality analysis:")
        logger.info(f"  Source points range: X[{np.min(A_xz[:, 0]):.3f}, {np.max(A_xz[:, 0]):.3f}], "
                   f"Z[{np.min(A_xz[:, 1]):.3f}, {np.max(A_xz[:, 1]):.3f}]")
        logger.info(f"  Target points range: X[{np.min(B_xz[:, 0]):.3f}, {np.max(B_xz[:, 0]):.3f}], "
                   f"Z[{np.min(B_xz[:, 1]):.3f}, {np.max(B_xz[:, 1]):.3f}]")
        
        # 检查数据的条件数
        try:
            A_centered = A_xz - np.mean(A_xz, axis=0)
            cond_A = np.linalg.cond(A_centered.T @ A_centered)
            logger.info(f"  Source data condition number: {cond_A:.2e}")
        except:
            logger.warning("  Could not compute condition number for source data")
    
    def _remove_duplicates(self, positions_A, positions_B):
        """移除重复点"""
        # 计算所有点对之间的距离
        distances = cdist(positions_A[:, [0, 2]], positions_A[:, [0, 2]])
        
        # 找到重复点（距离小于阈值且不是自身）
        duplicate_pairs = np.where((distances < self.duplicate_threshold) & (distances > 0))
        
        # 标记要删除的点
        points_to_remove = set()
        for i, j in zip(duplicate_pairs[0], duplicate_pairs[1]):
            if i < j:  # 只保留索引较小的点
                points_to_remove.add(j)
        
        # 创建保留点的mask
        keep_mask = np.ones(len(positions_A), dtype=bool)
        for idx in points_to_remove:
            keep_mask[idx] = False
        
        duplicate_mask = ~keep_mask
        
        return positions_A[keep_mask], positions_B[keep_mask], duplicate_mask
    
    def _check_minimum_distances(self, positions):
        """检查点间最小距离"""
        positions_xz = positions[:, [0, 2]]
        distances = cdist(positions_xz, positions_xz)
        
        # 将对角线设为无穷大（排除自身距离）
        np.fill_diagonal(distances, np.inf)
        
        # 每个点到其他点的最小距离
        min_distances = np.min(distances, axis=1)
        
        return min_distances
    
    def _normalize_data(self, positions_A, positions_B):
        """标准化数据（仅用于某些算法）"""
        A_xz = positions_A[:, [0, 2]]
        B_xz = positions_B[:, [0, 2]]
        
        # 拟合标准化器
        self.scaler_A.fit(A_xz)
        self.scaler_B.fit(B_xz)
        
        # 应用标准化
        normalized_A_xz = self.scaler_A.transform(A_xz)
        normalized_B_xz = self.scaler_B.transform(B_xz)
        
        # 重构3D点
        normalized_A = positions_A.copy()
        normalized_A[:, [0, 2]] = normalized_A_xz
        
        normalized_B = positions_B.copy()
        normalized_B[:, [0, 2]] = normalized_B_xz
        
        return normalized_A, normalized_B

class OutlierDetector:
    """
    增强的离群点检测器
    """
    def __init__(self, method='robust_iqr', iqr_factor=2.0, z_score_threshold=2.5, 
                 mad_threshold=2.5, isolation_contamination=0.1):
        """
        Parameters:
        method: 'robust_iqr', 'z_score', 'modified_z_score', 'isolation_forest', 'ensemble'
        """
        self.method = method
        self.iqr_factor = iqr_factor
        self.z_score_threshold = z_score_threshold
        self.mad_threshold = mad_threshold
        self.isolation_contamination = isolation_contamination
        self.outlier_indices = []
        self.outlier_stats = {}
        
    def detect_outliers_initial_svd(self, positions_A, positions_B):
        """
        使用初始SVD检测离群点（更稳健的方法）
        """
        logger.info(f"Starting robust outlier detection using {self.method} method...")
        
        positions_A = np.array(positions_A)
        positions_B = np.array(positions_B)
        
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
        
        logger.info(f"Initial SVD residual statistics:")
        logger.info(f"  Mean: {np.mean(residual_magnitudes)*1000:.3f}mm")
        logger.info(f"  Median: {np.median(residual_magnitudes)*1000:.3f}mm")
        logger.info(f"  Std: {np.std(residual_magnitudes)*1000:.3f}mm")
        logger.info(f"  Max: {np.max(residual_magnitudes)*1000:.3f}mm")
        logger.info(f"  95th percentile: {np.percentile(residual_magnitudes, 95)*1000:.3f}mm")
        
        # 检测离群点
        outlier_mask = self._detect_outliers_robust(residual_magnitudes)
        self.outlier_indices = np.where(outlier_mask)[0]
        
        # 统计信息
        self.outlier_stats = {
            'total_points': len(positions_A),
            'outliers_detected': len(self.outlier_indices),
            'outlier_ratio': len(self.outlier_indices) / len(positions_A),
            'outlier_residuals': residual_magnitudes[self.outlier_indices],
            'clean_residuals': residual_magnitudes[~outlier_mask],
            'threshold_used': self._get_threshold_robust(residual_magnitudes)
        }
        
        logger.info(f"Robust outlier detection results:")
        logger.info(f"  Method: {self.method}")
        logger.info(f"  Total points: {self.outlier_stats['total_points']}")
        logger.info(f"  Outliers detected: {self.outlier_stats['outliers_detected']}")
        logger.info(f"  Outlier ratio: {self.outlier_stats['outlier_ratio']*100:.2f}%")
        logger.info(f"  Threshold: {self.outlier_stats['threshold_used']*1000:.3f}mm")
        
        if len(self.outlier_indices) > 0:
            logger.info(f"  Max outlier residual: {np.max(self.outlier_stats['outlier_residuals'])*1000:.3f}mm")
            logger.info(f"  Mean clean residual: {np.mean(self.outlier_stats['clean_residuals'])*1000:.3f}mm")
        
        # 可视化离群点检测结果
        self._visualize_outlier_detection(positions_A, positions_B, svd_transformed, 
                                         residual_magnitudes, outlier_mask)
        
        return ~outlier_mask  # 返回非离群点的mask
    
    def _detect_outliers_robust(self, residual_magnitudes):
        """稳健的离群点检测"""
        if self.method == 'robust_iqr':
            return self._robust_iqr_method(residual_magnitudes)
        elif self.method == 'modified_z_score':
            return self._modified_z_score_method(residual_magnitudes)
        elif self.method == 'ensemble':
            return self._ensemble_method(residual_magnitudes)
        elif self.method == 'percentile':
            return self._percentile_method(residual_magnitudes)
        else:
            return self._robust_iqr_method(residual_magnitudes)  # 默认方法
    
    def _robust_iqr_method(self, data):
        """稳健的IQR方法"""
        Q1 = np.percentile(data, 25)
        Q3 = np.percentile(data, 75)
        IQR = Q3 - Q1
        
        # 使用更保守的阈值，并考虑数据分布
        median = np.median(data)
        
        # 如果IQR很小，说明数据很集中，使用绝对阈值
        if IQR < 0.001:  # 1mm
            threshold = median + 0.005  # 5mm阈值
        else:
            threshold = Q3 + self.iqr_factor * IQR
        
        return data > threshold
    
    def _modified_z_score_method(self, data):
        """修正Z-score方法（基于中位数绝对偏差）"""
        median = np.median(data)
        mad = np.median(np.abs(data - median))
        
        if mad == 0:
            mad = np.std(data)  # 备用方案
        
        modified_z_scores = 0.6745 * (data - median) / mad
        return np.abs(modified_z_scores) > self.mad_threshold
    
    def _ensemble_method(self, data):
        """集成方法：结合多种检测方法"""
        outliers_iqr = self._robust_iqr_method(data)
        outliers_mad = self._modified_z_score_method(data)
        outliers_percentile = self._percentile_method(data)
        
        # 至少两种方法认为是离群点才标记为离群点
        outlier_votes = outliers_iqr.astype(int) + outliers_mad.astype(int) + outliers_percentile.astype(int)
        return outlier_votes >= 2
    
    def _percentile_method(self, data):
        """基于百分位数的方法"""
        threshold = np.percentile(data, 95)  # 95th percentile
        return data > threshold
    
    def _get_threshold_robust(self, data):
        """获取稳健阈值"""
        if self.method == 'robust_iqr':
            Q1 = np.percentile(data, 25)
            Q3 = np.percentile(data, 75)
            IQR = Q3 - Q1
            median = np.median(data)
            if IQR < 0.001:
                return median + 0.005
            else:
                return Q3 + self.iqr_factor * IQR
        elif self.method == 'modified_z_score':
            median = np.median(data)
            mad = np.median(np.abs(data - median))
            if mad == 0:
                mad = np.std(data)
            return median + self.mad_threshold * mad / 0.6745
        elif self.method == 'percentile':
            return np.percentile(data, 95)
        else:
            return self._robust_iqr_method(data)
    
    def _visualize_outlier_detection(self, positions_A, positions_B, svd_transformed, 
                                   residual_magnitudes, outlier_mask):
        """可视化离群点检测结果"""
        fig, axes = plt.subplots(2, 2, figsize=(16, 12))
        
        # 1. 3D散点图显示离群点
        ax1 = axes[0, 0]
        colors = ['red' if outlier else 'blue' for outlier in outlier_mask]
        scatter = ax1.scatter(positions_A[:, 0], positions_A[:, 2], c=colors, alpha=0.7)
        ax1.set_xlabel('X [m]')
        ax1.set_ylabel('Z [m]')
        ax1.set_title(f'Outlier Detection - {self.method.upper()}')
        ax1.grid(True, alpha=0.3)
        
        # 添加图例
        from matplotlib.patches import Patch
        legend_elements = [Patch(facecolor='blue', label='Normal Points'),
                          Patch(facecolor='red', label='Outliers')]
        ax1.legend(handles=legend_elements)
        
        # 2. 残差分布直方图
        ax2 = axes[0, 1]
        ax2.hist(residual_magnitudes[~outlier_mask]*1000, bins=30, alpha=0.7, 
                color='blue', label='Normal Points', density=True)
        if np.any(outlier_mask):
            ax2.hist(residual_magnitudes[outlier_mask]*1000, bins=10, alpha=0.7, 
                    color='red', label='Outliers', density=True)
        
        threshold = self._get_threshold_robust(residual_magnitudes)
        ax2.axvline(x=threshold*1000, color='red', linestyle='--', linewidth=2,
                   label=f'Threshold ({threshold*1000:.1f}mm)')
        
        ax2.set_xlabel('Residual Magnitude [mm]')
        ax2.set_ylabel('Density')
        ax2.set_title(f'Residual Distribution ({self.method.upper()})')
        ax2.legend()
        ax2.grid(True, alpha=0.3)
        
        # 3. 箱线图
        ax3 = axes[1, 0]
        box_data = [residual_magnitudes[~outlier_mask]*1000]
        if np.any(outlier_mask):
            box_data.append(residual_magnitudes[outlier_mask]*1000)
            labels = ['Normal', 'Outliers']
        else:
            labels = ['Normal']
        
        bp = ax3.boxplot(box_data, labels=labels, patch_artist=True)
        bp['boxes'][0].set_facecolor('lightblue')
        if len(bp['boxes']) > 1:
            bp['boxes'][1].set_facecolor('lightcoral')
        
        ax3.set_ylabel('Residual Magnitude [mm]')
        ax3.set_title('Residual Distribution Boxplot')
        ax3.grid(True, alpha=0.3)
        
        # 4. 累积分布函数
        ax4 = axes[1, 1]
        sorted_normal = np.sort(residual_magnitudes[~outlier_mask]*1000)
        y_normal = np.arange(1, len(sorted_normal)+1) / len(sorted_normal)
        ax4.plot(sorted_normal, y_normal, 'b-', label='Normal Points', linewidth=2)
        
        if np.any(outlier_mask):
            sorted_outliers = np.sort(residual_magnitudes[outlier_mask]*1000)
            y_outliers = np.arange(1, len(sorted_outliers)+1) / len(sorted_outliers)
            ax4.plot(sorted_outliers, y_outliers, 'r-', label='Outliers', linewidth=2)
        
        ax4.axvline(x=threshold*1000, color='red', linestyle='--', alpha=0.7,
                   label=f'Threshold ({threshold*1000:.1f}mm)')
        ax4.set_xlabel('Residual Magnitude [mm]')
        ax4.set_ylabel('Cumulative Probability')
        ax4.set_title('Cumulative Distribution Function')
        ax4.legend()
        ax4.grid(True, alpha=0.3)
        
        plt.tight_layout()
        plt.savefig(os.path.join(img_dir, f'outlier_detection_{self.method}.png'), 
                   dpi=300, bbox_inches='tight')
        plt.close()
        
        logger.info(f"Outlier detection visualization saved: outlier_detection_{self.method}.png")

class FFDControlGrid:
    """
    FFD控制网格：基于B-Spline思想的自由形态形变控制网格
    """
    def __init__(self, grid_resolution=(5, 5), constrain_y=True):
        """
        Parameters:
        grid_resolution: (nx, nz) 控制网格的分辨率
        constrain_y: 是否约束Y轴形变为0
        """
        self.grid_resolution = grid_resolution
        self.constrain_y = constrain_y
        self.control_points = None
        self.bounds = None  # [x_min, x_max, z_min, z_max]
        self.grid_spacing = None
        self.initial_control_positions = None
        
    def initialize_control_grid(self, source_points):
        """
        根据源点云初始化控制网格
        """
        source_points = np.array(source_points)
        source_xz = source_points[:, [0, 2]]
        
        # 计算边界，并稍微扩展以包含所有点
        x_min, x_max = np.min(source_xz[:, 0]), np.max(source_xz[:, 0])
        z_min, z_max = np.min(source_xz[:, 1]), np.max(source_xz[:, 1])
        
        # 扩展边界5%
        x_margin = (x_max - x_min) * 0.05
        z_margin = (z_max - z_min) * 0.05
        
        x_min -= x_margin
        x_max += x_margin
        z_min -= z_margin
        z_max += z_margin
        
        self.bounds = [x_min, x_max, z_min, z_max]
        
        # 创建均匀分布的控制点网格
        nx, nz = self.grid_resolution
        x_coords = np.linspace(x_min, x_max, nx)
        z_coords = np.linspace(z_min, z_max, nz)
        
        # 生成网格点
        X, Z = np.meshgrid(x_coords, z_coords)
        
        # 控制点初始位置 (nz*nx, 3)
        self.initial_control_positions = np.zeros((nx * nz, 3))
        self.initial_control_positions[:, 0] = X.ravel()
        self.initial_control_positions[:, 2] = Z.ravel()
        
        # 控制点位移初始化为0
        self.control_points = self.initial_control_positions.copy()
        
        self.grid_spacing = [(x_max - x_min) / (nx - 1), (z_max - z_min) / (nz - 1)]
        
        logger.info(f"FFD control grid initialized: {nx}x{nz} = {nx*nz} control points")
        logger.info(f"Grid bounds: X[{x_min:.3f}, {x_max:.3f}], Z[{z_min:.3f}, {z_max:.3f}]")
        logger.info(f"Grid spacing: dX={self.grid_spacing[0]:.3f}, dZ={self.grid_spacing[1]:.3f}")
        
        return self.control_points
    
    def set_control_displacements(self, displacements):
        """
        设置控制点位移
        Parameters:
        displacements: (n_control_points * 2,) 或 (n_control_points, 2) 控制点在XZ平面的位移
        """
        displacements = np.array(displacements)
        
        if displacements.ndim == 1:
            # 将1D向量重新整形为(n_points, 2)
            n_control_points = len(self.initial_control_positions)
            displacements = displacements.reshape(n_control_points, 2)
        
        # 更新控制点位置
        self.control_points = self.initial_control_positions.copy()
        self.control_points[:, 0] += displacements[:, 0]  # X方向位移
        self.control_points[:, 2] += displacements[:, 1]  # Z方向位移
        
        return self.control_points
    
    def compute_bspline_weights(self, query_points):
        """
        计算查询点相对于控制网格的B-Spline权重
        使用三次B-Spline基函数
        """
        query_points = np.array(query_points)
        query_xz = query_points[:, [0, 2]]
        
        n_query = len(query_xz)
        n_control = len(self.control_points)
        weights = np.zeros((n_query, n_control))
        
        nx, nz = self.grid_resolution
        x_min, x_max, z_min, z_max = self.bounds
        
        # 将查询点映射到网格坐标
        u = (query_xz[:, 0] - x_min) / (x_max - x_min) * (nx - 1)
        v = (query_xz[:, 1] - z_min) / (z_max - z_min) * (nz - 1)
        
        # 对每个查询点计算权重
        for i in range(n_query):
            ui, vi = u[i], v[i]
            
            # 找到影响区域内的控制点（4x4邻域）
            i_start = max(0, int(np.floor(ui)) - 1)
            i_end = min(nx, int(np.floor(ui)) + 3)
            j_start = max(0, int(np.floor(vi)) - 1)
            j_end = min(nz, int(np.floor(vi)) + 3)
            
            for ci in range(i_start, i_end):
                for cj in range(j_start, j_end):
                    # 计算B-Spline基函数值
                    weight_u = self._cubic_bspline_basis(ui - ci)
                    weight_v = self._cubic_bspline_basis(vi - cj)
                    weight = weight_u * weight_v
                    
                    # 控制点的线性索引
                    control_idx = cj * nx + ci
                    weights[i, control_idx] = weight
        
        return weights
    
    def _cubic_bspline_basis(self, t):
        """
        三次B-Spline基函数
        """
        t = abs(t)
        if t < 1:
            return (2/3) - t*t + 0.5*t*t*t
        elif t < 2:
            return (2 - t) * (2 - t) * (2 - t) / 6
        else:
            return 0.0
    
    def apply_deformation(self, query_points):
        """
        对查询点应用FFD形变
        """
        query_points = np.array(query_points)
        
        # 计算B-Spline权重
        weights = self.compute_bspline_weights(query_points)
        
        # 计算控制点位移
        control_displacements = self.control_points - self.initial_control_positions
        
        # 应用形变：只在XZ平面
        deformation = np.zeros_like(query_points)
        deformation[:, 0] = np.sum(weights * control_displacements[:, 0], axis=1)
        deformation[:, 2] = np.sum(weights * control_displacements[:, 2], axis=1)
        
        return deformation
    
    def get_control_displacement_vector(self):
        """
        获取控制点位移向量（用于优化）
        """
        control_displacements = self.control_points - self.initial_control_positions
        # 只返回XZ方向的位移
        return control_displacements[:, [0, 2]].ravel()
    
    def visualize_control_grid(self, title="FFD Control Grid"):
        """
        可视化控制网格
        """
        fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(16, 6))
        
        # 初始网格
        ax1.scatter(self.initial_control_positions[:, 0], 
                   self.initial_control_positions[:, 2], 
                   c='blue', s=100, alpha=0.7, marker='o', label='Initial Control Points')
        
        # 绘制网格线
        nx, nz = self.grid_resolution
        for i in range(nx):
            for j in range(nz):
                idx = j * nx + i
                if i < nx - 1:  # 水平线
                    next_idx = j * nx + (i + 1)
                    ax1.plot([self.initial_control_positions[idx, 0], 
                             self.initial_control_positions[next_idx, 0]],
                            [self.initial_control_positions[idx, 2], 
                             self.initial_control_positions[next_idx, 2]], 
                            'b-', alpha=0.3)
                if j < nz - 1:  # 垂直线
                    next_idx = (j + 1) * nx + i
                    ax1.plot([self.initial_control_positions[idx, 0], 
                             self.initial_control_positions[next_idx, 0]],
                            [self.initial_control_positions[idx, 2], 
                             self.initial_control_positions[next_idx, 2]], 
                            'b-', alpha=0.3)
        
        ax1.set_xlabel('X [m]')
        ax1.set_ylabel('Z [m]')
        ax1.set_title('Initial Control Grid')
        ax1.grid(True, alpha=0.3)
        ax1.axis('equal')
        ax1.legend()
        
        # 变形后的网格
        ax2.scatter(self.control_points[:, 0], self.control_points[:, 2], 
                   c='red', s=100, alpha=0.7, marker='s', label='Deformed Control Points')
        
        # 绘制变形后的网格线
        for i in range(nx):
            for j in range(nz):
                idx = j * nx + i
                if i < nx - 1:
                    next_idx = j * nx + (i + 1)
                    ax2.plot([self.control_points[idx, 0], self.control_points[next_idx, 0]],
                            [self.control_points[idx, 2], self.control_points[next_idx, 2]], 
                            'r-', alpha=0.3)
                if j < nz - 1:
                    next_idx = (j + 1) * nx + i
                    ax2.plot([self.control_points[idx, 0], self.control_points[next_idx, 0]],
                            [self.control_points[idx, 2], self.control_points[next_idx, 2]], 
                            'r-', alpha=0.3)
        
        # 绘制位移向量
        control_displacements = self.control_points - self.initial_control_positions
        scale_factor = 20  # 放大因子
        ax2.quiver(self.initial_control_positions[:, 0], 
                  self.initial_control_positions[:, 2],
                  control_displacements[:, 0] * scale_factor,
                  control_displacements[:, 2] * scale_factor,
                  angles='xy', scale_units='xy', scale=1, 
                  color='purple', alpha=0.7, width=0.003,
                  label=f'Displacement (×{scale_factor})')
        
        ax2.set_xlabel('X [m]')
        ax2.set_ylabel('Z [m]')
        ax2.set_title('Deformed Control Grid')
        ax2.grid(True, alpha=0.3)
        ax2.axis('equal')
        ax2.legend()
        
        plt.tight_layout()
        plt.savefig(os.path.join(img_dir, f'ffd_control_grid_{title.lower().replace(" ", "_")}.png'), 
                   dpi=300, bbox_inches='tight')
        plt.close()

class FFDDeformationModel:
    """
    基于FFD控制网格的形变模型
    """
    def __init__(self, grid_resolution=(5, 5), constrain_y=True, optimization_method='L-BFGS-B'):
        """
        Parameters:
        grid_resolution: 控制网格分辨率
        constrain_y: 是否约束Y轴
        optimization_method: 优化方法 ('L-BFGS-B', 'differential_evolution', 'SLSQP')
        """
        self.grid_resolution = grid_resolution
        self.constrain_y = constrain_y
        self.optimization_method = optimization_method
        self.control_grid = FFDControlGrid(grid_resolution, constrain_y)
        self.source_points = None
        self.target_residuals = None
        self.optimization_result = None
        
    def learn_deformation(self, source_points, residual_vectors):
        """
        学习FFD形变模型
        """
        self.source_points = np.array(source_points)
        residual_vectors = np.array(residual_vectors)
        
        if self.constrain_y:
            self.target_residuals = residual_vectors[:, [0, 2]]  # 只使用XZ方向
        else:
            self.target_residuals = residual_vectors
        
        logger.info(f"Learning FFD deformation with {self.grid_resolution[0]}x{self.grid_resolution[1]} control grid")
        
        # 初始化控制网格
        self.control_grid.initialize_control_grid(self.source_points)
        
        # 定义优化目标函数
        def objective_function(control_displacements):
            # 设置控制点位移
            self.control_grid.set_control_displacements(control_displacements.reshape(-1, 2))
            
            # 计算预测的形变
            predicted_deformation = self.control_grid.apply_deformation(self.source_points)
            predicted_deformation_xz = predicted_deformation[:, [0, 2]]
            
            # 计算RMSE
            residual_diff = self.target_residuals - predicted_deformation_xz
            rmse = np.sqrt(np.mean(np.sum(residual_diff**2, axis=1)))
            
            return rmse
        
        # 初始控制点位移（全部为0）
        n_control_points = self.grid_resolution[0] * self.grid_resolution[1]
        initial_displacements = np.zeros(n_control_points * 2)
        
        # 设置优化边界（限制控制点位移的范围）
        displacement_bound = 0.05  # 5cm的最大位移
        bounds = [(-displacement_bound, displacement_bound)] * (n_control_points * 2)
        
        logger.info(f"Starting FFD optimization using {self.optimization_method}")
        logger.info(f"Control points: {n_control_points}")
        logger.info(f"Optimization parameters: {n_control_points * 2}")
        logger.info(f"Displacement bounds: ±{displacement_bound*1000:.1f}mm")
        
        try:
            if self.optimization_method == 'differential_evolution':
                # 全局优化方法
                self.optimization_result = differential_evolution(
                    objective_function,
                    bounds=bounds,
                    seed=42,
                    maxiter=300,
                    popsize=15,
                    atol=1e-6,
                    tol=1e-6,
                    workers=1
                )
            else:
                # 局部优化方法
                self.optimization_result = minimize(
                    objective_function,
                    initial_displacements,
                    method=self.optimization_method,
                    bounds=bounds,
                    options={'maxiter': 1000, 'ftol': 1e-9}
                )
            
            if self.optimization_result.success:
                # 设置最优控制点位移
                optimal_displacements = self.optimization_result.x
                self.control_grid.set_control_displacements(optimal_displacements.reshape(-1, 2))
                
                final_rmse = self.optimization_result.fun
                logger.info(f"FFD optimization successful!")
                logger.info(f"Final RMSE: {final_rmse*1000:.3f}mm")
                logger.info(f"Optimization iterations: {self.optimization_result.nit}")
                
                # 可视化控制网格
                self.control_grid.visualize_control_grid("Optimized FFD Grid")
                
                return True
            else:
                logger.error(f"FFD optimization failed: {self.optimization_result.message}")
                return False
                
        except Exception as e:
            logger.error(f"FFD optimization error: {e}")
            return False
    
    def predict_deformation(self, query_points):
        """
        预测查询点的形变
        """
        if self.control_grid.control_points is None:
            logger.error("FFD model not trained yet")
            return np.zeros_like(query_points)
        
        return self.control_grid.apply_deformation(query_points)

class FFDHyperparameterOptimizer:
    """
    FFD模型的超参数优化器
    """
    def __init__(self, cv_folds=3, random_state=42):
        """
        Parameters:
        cv_folds: 交叉验证折数（FFD优化较慢，使用较少折数）
        """
        self.cv_folds = cv_folds
        self.random_state = random_state
        self.best_params = None
        self.best_score = float('inf')
        self.cv_results = []
        
    def optimize_ffd_params(self, positions_A, positions_B, T_svd):
        """
        优化FFD模型参数
        """
        logger.info("Starting FFD hyperparameter optimization with cross-validation...")
        
        # 定义参数网格
        grid_resolutions = [(3, 3), (4, 4), (5, 5), (6, 6), (4, 6), (6, 4)]
        optimization_methods = ['L-BFGS-B', 'differential_evolution']
        
        param_combinations = list(itertools.product(grid_resolutions, optimization_methods))
        
        logger.info(f"Testing {len(param_combinations)} FFD parameter combinations with {self.cv_folds}-fold CV")
        
        # 设置交叉验证
        kf = KFold(n_splits=self.cv_folds, shuffle=True, random_state=self.random_state)
        
        positions_A = np.array(positions_A)
        positions_B = np.array(positions_B)
        indices = np.arange(len(positions_A))
        
        best_params = None
        best_mean_rmse = float('inf')
        
        for grid_resolution, opt_method in param_combinations:
            fold_rmses = []
            fold_times = []
            
            logger.info(f"Testing grid {grid_resolution[0]}x{grid_resolution[1]} with {opt_method}")
            
            try:
                for fold, (train_idx, test_idx) in enumerate(kf.split(indices)):
                    start_time = datetime.datetime.now()
                    
                    # 分割训练和测试数据
                    train_A = positions_A[train_idx]
                    train_B = positions_B[train_idx]
                    test_A = positions_A[test_idx]
                    test_B = positions_B[test_idx]
                    
                    # 在训练集上执行SVD变换
                    svd_train_transformed = self._apply_transformation(train_A, T_svd)
                    residual_vectors = train_B - svd_train_transformed
                    
                    # 训练FFD模型
                    model = FFDDeformationModel(
                        grid_resolution=grid_resolution,
                        constrain_y=True,
                        optimization_method=opt_method
                    )
                    success = model.learn_deformation(svd_train_transformed, residual_vectors)
                    
                    if not success:
                        fold_rmses.append(float('inf'))
                        continue
                    
                    # 在测试集上评估
                    svd_test_transformed = self._apply_transformation(test_A, T_svd)
                    predicted_deformation = model.predict_deformation(svd_test_transformed)
                    final_test_transformed = svd_test_transformed + predicted_deformation
                    final_test_transformed[:, 1] = 0
                    
                    # 计算测试集RMSE
                    test_rmse = self._calculate_rmse(test_B, final_test_transformed)
                    fold_rmses.append(test_rmse)
                    
                    fold_time = (datetime.datetime.now() - start_time).total_seconds()
                    fold_times.append(fold_time)
                    
                    logger.info(f"    Fold {fold+1}: RMSE={test_rmse*1000:.3f}mm, Time={fold_time:.1f}s")
                
                # 计算平均RMSE
                if len(fold_rmses) > 0 and not any(np.isinf(fold_rmses)):
                    mean_rmse = np.mean(fold_rmses)
                    std_rmse = np.std(fold_rmses)
                    mean_time = np.mean(fold_times)
                    
                    self.cv_results.append({
                        'grid_resolution': grid_resolution,
                        'optimization_method': opt_method,
                        'mean_rmse': mean_rmse,
                        'std_rmse': std_rmse,
                        'mean_time': mean_time,
                        'fold_rmses': fold_rmses
                    })
                    
                    logger.info(f"  Grid {grid_resolution[0]}x{grid_resolution[1]} + {opt_method}: "
                              f"RMSE={mean_rmse*1000:.3f}±{std_rmse*1000:.3f}mm, "
                              f"Time={mean_time:.1f}s")
                    
                    if mean_rmse < best_mean_rmse:
                        best_mean_rmse = mean_rmse
                        best_params = {
                            'grid_resolution': grid_resolution,
                            'optimization_method': opt_method
                        }
                
            except Exception as e:
                logger.warning(f"Failed to evaluate grid {grid_resolution[0]}x{grid_resolution[1]} + {opt_method}: {e}")
                continue
        
        self.best_params = best_params
        self.best_score = best_mean_rmse
        
        if best_params:
            logger.info(f"Best FFD parameters: {best_params}")
            logger.info(f"Best CV RMSE: {best_mean_rmse*1000:.3f}mm")
        else:
            logger.error("No valid FFD parameter combination found")
        
        return best_params
    
    def _apply_transformation(self, positions, T):
        """应用变换矩阵"""
        positions = np.array(positions)
        positions_homo = np.hstack([positions, np.ones((positions.shape[0], 1))])
        transformed = np.dot(T, positions_homo.T).T
        result = transformed[:, :3]
        result[:, 1] = 0
        return result
    
    def _calculate_rmse(self, actual, predicted):
        """计算RMSE"""
        actual = np.array(actual)
        predicted = np.array(predicted)
        diff_xz = actual[:, [0, 2]] - predicted[:, [0, 2]]
        return np.sqrt(np.mean(np.sum(diff_xz**2, axis=1)))
    
    def save_cv_results(self):
        """保存交叉验证结果"""
        if self.cv_results:
            cv_df = pd.DataFrame(self.cv_results)
            cv_df.to_csv(os.path.join(result_dir, "cv_results_ffd.csv"), index=False)
            
            # 可视化CV结果
            self._plot_cv_results()
    
    def _plot_cv_results(self):
        """可视化交叉验证结果"""
        if not self.cv_results:
            return
        
        df = pd.DataFrame(self.cv_results)
        
        fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(16, 6))
        
        # 1. RMSE比较
        methods = df['optimization_method'].unique()
        x_pos = np.arange(len(df))
        
        colors = ['blue' if method == 'L-BFGS-B' else 'red' for method in df['optimization_method']]
        bars = ax1.bar(x_pos, df['mean_rmse']*1000, yerr=df['std_rmse']*1000, 
                      color=colors, alpha=0.7, capsize=5)
        
        # 添加网格分辨率标签
        labels = [f"{res[0]}x{res[1]}\n{method}" for res, method in 
                 zip(df['grid_resolution'], df['optimization_method'])]
        ax1.set_xticks(x_pos)
        ax1.set_xticklabels(labels, rotation=45, ha='right')
        ax1.set_ylabel('RMSE [mm]')
        ax1.set_title('FFD Cross-Validation Results')
        ax1.grid(True, alpha=0.3)
        ax1.axhline(y=TARGET_RMSE*1000, color='green', linestyle='--', alpha=0.7, label='Target')
        ax1.legend()
        
        # 2. 计算时间比较
        bars2 = ax2.bar(x_pos, df['mean_time'], color=colors, alpha=0.7)
        ax2.set_xticks(x_pos)
        ax2.set_xticklabels(labels, rotation=45, ha='right')
        ax2.set_ylabel('Training Time [s]')
        ax2.set_title('FFD Training Time Comparison')
        ax2.grid(True, alpha=0.3)
        
        # 添加方法图例
        from matplotlib.patches import Patch
        legend_elements = [Patch(facecolor='blue', label='L-BFGS-B'),
                          Patch(facecolor='red', label='Differential Evolution')]
        ax2.legend(handles=legend_elements)
        
        plt.tight_layout()
        plt.savefig(os.path.join(img_dir, 'cv_results_ffd.png'), dpi=300, bbox_inches='tight')
        plt.close()

class ResidualAnalyzer:
    """
    残差分析器，用于分析最终残差的空间和时间分布
    """
    def __init__(self):
        self.residual_vectors = None
        self.residual_magnitudes = None
        self.positions = None
        
    def analyze_final_residuals(self, positions, actual, predicted, stage_name="Final"):
        """分析最终残差"""
        self.positions = np.array(positions)
        actual = np.array(actual)
        predicted = np.array(predicted)
        
        self.residual_vectors = actual - predicted
        self.residual_magnitudes = np.sqrt(np.sum(self.residual_vectors**2, axis=1))
        
        logger.info(f"\n=== {stage_name} Residual Analysis ===")
        logger.info(f"Residual statistics:")
        logger.info(f"  Mean magnitude: {np.mean(self.residual_magnitudes)*1000:.3f}mm")
        logger.info(f"  Median magnitude: {np.median(self.residual_magnitudes)*1000:.3f}mm")
        logger.info(f"  Std magnitude: {np.std(self.residual_magnitudes)*1000:.3f}mm")
        logger.info(f"  Max magnitude: {np.max(self.residual_magnitudes)*1000:.3f}mm")
        logger.info(f"  Min magnitude: {np.min(self.residual_magnitudes)*1000:.3f}mm")
        
        # 分析高误差点
        high_error_threshold = np.percentile(self.residual_magnitudes, 90)
        high_error_mask = self.residual_magnitudes > high_error_threshold
        num_high_error = np.sum(high_error_mask)
        
        logger.info(f"High error points (>90th percentile):")
        logger.info(f"  Threshold: {high_error_threshold*1000:.3f}mm")
        logger.info(f"  Count: {num_high_error} ({num_high_error/len(self.residual_magnitudes)*100:.1f}%)")
        
        if num_high_error > 0:
            logger.info(f"  Mean high error: {np.mean(self.residual_magnitudes[high_error_mask])*1000:.3f}mm")
        
        # 可视化残差分析
        self._visualize_residual_analysis(stage_name, high_error_mask, high_error_threshold)
        
        return {
            'residual_vectors': self.residual_vectors,
            'residual_magnitudes': self.residual_magnitudes,
            'high_error_mask': high_error_mask,
            'high_error_threshold': high_error_threshold
        }
    
    def _visualize_residual_analysis(self, stage_name, high_error_mask, high_error_threshold):
        """可视化残差分析结果"""
        fig, axes = plt.subplots(2, 3, figsize=(20, 12))
        
        # 1. 空间分布 - 按误差大小着色
        ax1 = axes[0, 0]
        scatter = ax1.scatter(self.positions[:, 0], self.positions[:, 2], 
                            c=self.residual_magnitudes*1000, cmap='viridis', 
                            s=50, alpha=0.7)
        plt.colorbar(scatter, ax=ax1, label='Residual Magnitude [mm]')
        ax1.set_xlabel('X [m]')
        ax1.set_ylabel('Z [m]')
        ax1.set_title('Spatial Distribution of Residuals')
        ax1.grid(True, alpha=0.3)
        
        # 2. 高误差点突出显示
        ax2 = axes[0, 1]
        ax2.scatter(self.positions[~high_error_mask, 0], self.positions[~high_error_mask, 2], 
                   c='blue', alpha=0.6, s=20, label='Normal Points')
        ax2.scatter(self.positions[high_error_mask, 0], self.positions[high_error_mask, 2], 
                   c='red', alpha=0.8, s=80, marker='x', label='High Error Points')
        ax2.set_xlabel('X [m]')
        ax2.set_ylabel('Z [m]')
        ax2.set_title(f'High Error Points (>{high_error_threshold*1000:.1f}mm)')
        ax2.legend()
        ax2.grid(True, alpha=0.3)
        
        # 3. 残差向量场
        ax3 = axes[0, 2]
        # 抽样显示残差向量，避免过于密集
        step = max(1, len(self.positions) // 50)
        scale_factor = 100  # 放大因子
        ax3.quiver(self.positions[::step, 0], self.positions[::step, 2], 
                  self.residual_vectors[::step, 0]*scale_factor, 
                  self.residual_vectors[::step, 2]*scale_factor,
                  angles='xy', scale_units='xy', scale=1, alpha=0.7)
        ax3.set_xlabel('X [m]')
        ax3.set_ylabel('Z [m]')
        ax3.set_title(f'Residual Vector Field (×{scale_factor})')
        ax3.grid(True, alpha=0.3)
        ax3.axis('equal')
        
        # 4. 时间序列
        ax4 = axes[1, 0]
        indices = np.arange(len(self.residual_magnitudes))
        ax4.plot(indices, self.residual_magnitudes*1000, 'b-', alpha=0.7, linewidth=1)
        ax4.scatter(indices[high_error_mask], self.residual_magnitudes[high_error_mask]*1000, 
                   c='red', s=50, alpha=0.8, label='High Error')
        ax4.axhline(y=TARGET_RMSE*1000, color='red', linestyle='--', alpha=0.7, 
                   label=f'Target ({TARGET_RMSE*1000:.1f}mm)')
        ax4.set_xlabel('Data Point Index')
        ax4.set_ylabel('Residual Magnitude [mm]')
        ax4.set_title('Residual Magnitude Time Series')
        ax4.legend()
        ax4.grid(True, alpha=0.3)
        
        # 5. 残差分布直方图
        ax5 = axes[1, 1]
        ax5.hist(self.residual_magnitudes*1000, bins=30, alpha=0.7, color='skyblue', 
                edgecolor='black', density=True)
        ax5.axvline(x=np.mean(self.residual_magnitudes)*1000, color='red', linestyle='-', 
                   label=f'Mean ({np.mean(self.residual_magnitudes)*1000:.2f}mm)')
        ax5.axvline(x=np.median(self.residual_magnitudes)*1000, color='green', linestyle='-', 
                   label=f'Median ({np.median(self.residual_magnitudes)*1000:.2f}mm)')
        ax5.axvline(x=TARGET_RMSE*1000, color='orange', linestyle='--', 
                   label=f'Target ({TARGET_RMSE*1000:.1f}mm)')
        ax5.set_xlabel('Residual Magnitude [mm]')
        ax5.set_ylabel('Density')
        ax5.set_title('Residual Distribution')
        ax5.legend()
        ax5.grid(True, alpha=0.3)
        
        # 6. 方向性分析
        ax6 = axes[1, 2]
        # 计算X和Z方向的残差
        x_residuals = np.abs(self.residual_vectors[:, 0]) * 1000
        z_residuals = np.abs(self.residual_vectors[:, 2]) * 1000
        
        ax6.scatter(x_residuals, z_residuals, alpha=0.6, s=20)
        ax6.set_xlabel('X-direction Residual [mm]')
        ax6.set_ylabel('Z-direction Residual [mm]')
        ax6.set_title('Directional Residual Analysis')
        ax6.grid(True, alpha=0.3)
        
        # 添加对角线
        max_val = max(np.max(x_residuals), np.max(z_residuals))
        ax6.plot([0, max_val], [0, max_val], 'r--', alpha=0.5, label='X=Z line')
        ax6.legend()
        
        plt.tight_layout()
        filename = f'{stage_name.lower().replace(" ", "_")}_residual_analysis.png'
        plt.savefig(os.path.join(img_dir, filename), dpi=300, bbox_inches='tight')
        plt.close()
        
        logger.info(f"Residual analysis visualization saved: {filename}")

# 继续原有的CoordinateTransformer类，但使用FFD模型
class CoordinateTransformer:
    """
    Enhanced two-stage coordinate transformation with FFD deformation
    """
    def __init__(self, positions_A=None, positions_B=None):
        self.T_svd = None
        self.deformation_model = None
        self.best_model_type = "FFD"
        self.__positions_A = positions_A
        self.__positions_B = positions_B
        self.optimizer = FFDHyperparameterOptimizer()
        self.outlier_detector = OutlierDetector(method='robust_iqr', iqr_factor=2.0)
        self.residual_analyzer = ResidualAnalyzer()
        self.preprocessor = DataPreprocessor()
        self.clean_indices = None
        self.preprocessing_info = None
        
    @staticmethod
    def construct_transformation_matrix(rotation_matrix, translation):
        T = np.eye(4)
        T[:3, :3] = rotation_matrix
        T[:3, 3] = translation
        return T

    def calculate_svd_transformation(self, positions_A=None, positions_B=None):
        """Stage 1: SVD-based coarse registration with Y-axis constraint"""
        if positions_A is not None and positions_B is not None:
            self.__positions_A = positions_A
            self.__positions_B = positions_B
        elif self.__positions_A is None or self.__positions_B is None:
            logger.error("Please provide position data first!")
            return

        positions_A = np.array(self.__positions_A)
        positions_B = np.array(self.__positions_B)

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
        
        self.T_svd = np.eye(4)
        self.T_svd[0, 0] = R_2d[0, 0]
        self.T_svd[0, 2] = R_2d[0, 1]
        self.T_svd[2, 0] = R_2d[1, 0]
        self.T_svd[2, 2] = R_2d[1, 1]
        self.T_svd[0, 3] = translation_2d[0]
        self.T_svd[2, 3] = translation_2d[1]
        
        logger.info("SVD transformation matrix calculated (Y-axis constrained)")
        return R_2d, translation_2d

    def apply_transformation(self, positions, T):
        """Apply transformation matrix to positions"""
        positions = np.array(positions)
        positions_homo = np.hstack([positions, np.ones((positions.shape[0], 1))])
        transformed = np.dot(T, positions_homo.T).T
        result = transformed[:, :3]
        result[:, 1] = 0
        return result

    def calculate_directional_rmse(self, actual, predicted):
        """Calculate RMSE for each direction separately"""
        actual = np.array(actual)
        predicted = np.array(predicted)
        
        rmse_lateral = np.sqrt(np.mean((actual[:, 0] - predicted[:, 0])**2))
        rmse_vertical = 0.0
        rmse_longitudinal = np.sqrt(np.mean((actual[:, 2] - predicted[:, 2])**2))
        
        overall_rmse = np.sqrt(rmse_lateral**2 + rmse_longitudinal**2)
        horizontal_rmse = overall_rmse
        
        return {
            'lateral_rmse': rmse_lateral,
            'vertical_rmse': rmse_vertical,
            'longitudinal_rmse': rmse_longitudinal,
            'horizontal_rmse': horizontal_rmse,
            'overall_rmse': overall_rmse
        }

    def evaluate_performance(self, actual, predicted):
        """Evaluate performance against targets"""
        rmse_results = self.calculate_directional_rmse(actual, predicted)
        
        lateral_pass = rmse_results['lateral_rmse'] <= SCALED_LATERAL_TARGET
        longitudinal_pass = rmse_results['longitudinal_rmse'] <= SCALED_LONGITUDINAL_TARGET
        horizontal_pass = rmse_results['horizontal_rmse'] <= SCALED_COMBINED_TARGET
        overall_pass = rmse_results['overall_rmse'] <= TARGET_RMSE
        
        real_world_equivalent = {
            'lateral_real_world_eq': rmse_results['lateral_rmse'] * SCALE_FACTOR,
            'longitudinal_real_world_eq': rmse_results['longitudinal_rmse'] * SCALE_FACTOR,
            'horizontal_real_world_eq': rmse_results['horizontal_rmse'] * SCALE_FACTOR,
            'overall_real_world_eq': rmse_results['overall_rmse'] * SCALE_FACTOR
        }
        
        return {
            'rmse_results': rmse_results,
            'pass_criteria': {
                'lateral_pass': lateral_pass,
                'longitudinal_pass': longitudinal_pass,
                'horizontal_pass': horizontal_pass,
                'overall_pass': overall_pass
            },
            'real_world_equivalent': real_world_equivalent
        }

    def visualize_transformation_stage(self, stage_name, positions_A, positions_B, transformed_positions, rmse_results, residual_vectors=None):
        """Visualize transformation results for a specific stage"""
        positions_A = np.array(positions_A)
        positions_B = np.array(positions_B)
        transformed_positions = np.array(transformed_positions)
        
        positions_A[:, 1] = 0
        positions_B[:, 1] = 0
        transformed_positions[:, 1] = 0
        
        individual_errors = np.sqrt((positions_B[:, 0] - transformed_positions[:, 0])**2 + 
                                   (positions_B[:, 2] - transformed_positions[:, 2])**2)
        
        fig = plt.figure(figsize=(20, 12))
        
        # 3D散点图
        ax1 = fig.add_subplot(221, projection='3d')
        
        ax1.scatter(positions_A[:, 0], positions_A[:, 2], positions_A[:, 1], 
                   c='blue', marker='o', s=50, label='Source Points', alpha=0.7)
        ax1.scatter(positions_B[:, 0], positions_B[:, 2], positions_B[:, 1], 
                   c='red', marker='^', s=50, label='Target Points', alpha=0.7)
        ax1.scatter(transformed_positions[:, 0], transformed_positions[:, 2], transformed_positions[:, 1], 
                   c='green', marker='x', s=50, label='Transformed Points', alpha=0.7)
        
        ax1.set_xlabel('X (Lateral) [m]')
        ax1.set_ylabel('Z (Longitudinal) [m]')
        ax1.set_zlabel('Y (Vertical) [m]')
        ax1.set_title(f'{stage_name} - 3D Point Cloud Registration')
        ax1.legend()
        ax1.set_zlim(-0.1, 0.1)
        
        # 误差分布
        ax2 = fig.add_subplot(222)
        ax2.hist(individual_errors * 1000, bins=20, color='skyblue', edgecolor='black', alpha=0.7)
        ax2.axvline(x=TARGET_RMSE*1000, color='r', linestyle='--', linewidth=2, 
                   label=f'Target RMSE ({TARGET_RMSE*1000:.1f}mm)')
        ax2.set_xlabel('Individual Point Error [mm]')
        ax2.set_ylabel('Frequency')
        ax2.set_title(f'{stage_name} - Error Distribution')
        ax2.legend()
        ax2.grid(True, alpha=0.3)
        
        # RMSE by direction
        ax3 = fig.add_subplot(223)
        directions = ['Lateral', 'Longitudinal', 'Horizontal', 'Overall']
        rmse_values = [
            rmse_results['lateral_rmse'] * 1000,
            rmse_results['longitudinal_rmse'] * 1000,
            rmse_results['horizontal_rmse'] * 1000,
            rmse_results['overall_rmse'] * 1000
        ]
        targets = [
            SCALED_LATERAL_TARGET * 1000,
            SCALED_LONGITUDINAL_TARGET * 1000,
            SCALED_COMBINED_TARGET * 1000,
            TARGET_RMSE * 1000
        ]
        
        bars = ax3.bar(directions, rmse_values, color=['lightblue', 'lightcoral', 'lightyellow', 'lightpink'])
        for i, (bar, target) in enumerate(zip(bars, targets)):
            ax3.axhline(y=target, color='red', linestyle='--', alpha=0.7)
            if rmse_values[i] <= target:
                bar.set_color('green')
                bar.set_alpha(0.7)
            else:
                bar.set_color('red')
                bar.set_alpha(0.7)
        
        ax3.set_ylabel('RMSE [mm]')
        ax3.set_title(f'{stage_name} - RMSE by Direction')
        ax3.tick_params(axis='x', rotation=45)
        ax3.grid(True, alpha=0.3)
        
        # XZ平面视图
        ax4 = fig.add_subplot(224)
        ax4.scatter(positions_A[:, 0], positions_A[:, 2], c='blue', marker='o', s=50, 
                   label='Source Points', alpha=0.7)
        ax4.scatter(positions_B[:, 0], positions_B[:, 2], c='red', marker='^', s=50, 
                   label='Target Points', alpha=0.7)
        ax4.scatter(transformed_positions[:, 0], transformed_positions[:, 2], c='green', marker='x', s=50, 
                   label='Transformed Points', alpha=0.7)
        
        # 绘制误差向量
        for i in range(len(positions_B)):
            if individual_errors[i] > TARGET_RMSE:
                ax4.plot([positions_B[i, 0], transformed_positions[i, 0]], 
                        [positions_B[i, 2], transformed_positions[i, 2]], 
                        'k--', alpha=0.3)
        
        if residual_vectors is not None:
            residual_vectors = np.array(residual_vectors)
            scale_factor = 20
            ax4.quiver(transformed_positions[:, 0], transformed_positions[:, 2], 
                      residual_vectors[:, 0] * scale_factor, residual_vectors[:, 2] * scale_factor, 
                      angles='xy', scale_units='xy', scale=1, color='purple', alpha=0.6, 
                      label='Residual Vectors (×20)')
        
        ax4.set_xlabel('X (Lateral) [m]')
        ax4.set_ylabel('Z (Longitudinal) [m]')
        ax4.set_title(f'{stage_name} - XZ Plane View')
        ax4.legend()
        ax4.grid(True, alpha=0.3)
        ax4.axis('equal')
        
        plt.tight_layout()
        
        filename = f'{stage_name.lower().replace(" ", "_").replace("(", "").replace(")", "")}_transformation_results.png'
        plt.savefig(os.path.join(img_dir, filename), dpi=300, bbox_inches='tight')
        plt.close()
        
        self._create_performance_summary(stage_name, rmse_results)

    def _create_performance_summary(self, stage_name, rmse_results):
        """Create detailed performance summary visualization"""
        fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(16, 6))
        
        categories = ['Lateral', 'Longitudinal', 'Horizontal', 'Overall']
        actual_values = [
            rmse_results['lateral_rmse'] * 1000,
            rmse_results['longitudinal_rmse'] * 1000,
            rmse_results['horizontal_rmse'] * 1000,
            rmse_results['overall_rmse'] * 1000
        ]
        target_values = [
            SCALED_LATERAL_TARGET * 1000,
            SCALED_LONGITUDINAL_TARGET * 1000,
            SCALED_COMBINED_TARGET * 1000,
            TARGET_RMSE * 1000
        ]
        
        x = np.arange(len(categories))
        width = 0.35
        
        bars1 = ax1.bar(x - width/2, actual_values, width, label='Actual RMSE', alpha=0.8)
        bars2 = ax1.bar(x + width/2, target_values, width, label='Target RMSE', alpha=0.8)
        
        for i, (actual, target) in enumerate(zip(actual_values, target_values)):
            if actual <= target:
                bars1[i].set_color('green')
            else:
                bars1[i].set_color('red')
        
        ax1.set_xlabel('Direction')
        ax1.set_ylabel('RMSE [mm]')
        ax1.set_title(f'{stage_name} - Performance vs Targets')
        ax1.set_xticks(x)
        ax1.set_xticklabels(categories)
        ax1.legend()
        ax1.grid(True, alpha=0.3)
        
        real_world_eq = [val * SCALE_FACTOR / 10 for val in actual_values]
        
        bars3 = ax2.bar(categories, real_world_eq, color='lightblue', alpha=0.8)
        ax2.axhline(y=REAL_WORLD_LATERAL_TARGET * 100, color='red', linestyle='--', 
                   label=f'Real-world Target ({REAL_WORLD_LATERAL_TARGET * 100:.0f}cm)')
        
        ax2.set_xlabel('Direction')
        ax2.set_ylabel('Real-world Equivalent [cm]')
        ax2.set_title(f'{stage_name} - Real-world Equivalent Performance')
        ax2.legend()
        ax2.grid(True, alpha=0.3)
        
        plt.tight_layout()
        
        filename = f'{stage_name.lower().replace(" ", "_").replace("(", "").replace(")", "")}_performance_summary.png'
        plt.savefig(os.path.join(img_dir, filename), dpi=300, bbox_inches='tight')
        plt.close()

    def two_stage_registration_ffd(self):
        """Perform enhanced two-stage registration with FFD deformation"""
        if self.__positions_A is None or self.__positions_B is None:
            logger.error("Please provide position data first!")
            return None, None
        
        logger.info("=== Starting FFD-based Two-Stage Registration ===")
        logger.info("FFD Features:")
        logger.info("  - Free-Form Deformation with B-Spline control grid")
        logger.info("  - Low parameter count for stable optimization")
        logger.info("  - Smooth and natural deformation field")
        logger.info("  - Intuitive control over deformation smoothness")
        
        positions_A = np.array(self.__positions_A)
        positions_B = np.array(self.__positions_B)
        
        logger.info(f"Original dataset: {len(positions_A)} point pairs")
        
        # Step 0: Data Preprocessing
        logger.info("Step 0: Data Preprocessing")
        clean_positions_A, clean_positions_B, self.preprocessing_info, normalized_A, normalized_B = \
            self.preprocessor.preprocess_data(positions_A, positions_B)
        
        # Step 1: Outlier Detection
        logger.info("Step 1: Robust Outlier Detection")
        clean_mask = self.outlier_detector.detect_outliers_initial_svd(clean_positions_A, clean_positions_B)
        self.clean_indices = np.where(clean_mask)[0]
        
        # 使用最终清洁的数据集
        final_clean_A = clean_positions_A[clean_mask]
        final_clean_B = clean_positions_B[clean_mask]
        
        logger.info(f"Final clean dataset: {len(final_clean_A)} point pairs")
        logger.info(f"Total removed: {len(positions_A) - len(final_clean_A)} points")
        
        # 保存清洁数据集信息
        clean_data_df = pd.DataFrame({
            'clean_index': np.arange(len(final_clean_A)),
            'source_x': final_clean_A[:, 0],
            'source_z': final_clean_A[:, 2],
            'target_x': final_clean_B[:, 0],
            'target_z': final_clean_B[:, 2]
        })
        clean_data_df.to_csv(os.path.join(result_dir, "final_clean_dataset_ffd.csv"), index=False)
        
        # Stage 2: SVD coarse registration on final clean data
        logger.info("Stage 2: SVD Coarse Registration (Final Clean Data)")
        
        # 重新计算SVD在最终清洁数据上
        positions_A_xz = final_clean_A[:, [0, 2]]
        positions_B_xz = final_clean_B[:, [0, 2]]

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
        
        svd_transformed = self.apply_transformation(final_clean_A, self.T_svd)
        svd_rmse_results = self.calculate_directional_rmse(final_clean_B, svd_transformed)
        svd_performance = self.evaluate_performance(final_clean_B, svd_transformed)
        
        logger.info(f"SVD Overall RMSE (final clean data): {svd_rmse_results['overall_rmse']*1000:.3f}mm")
        
        residual_vectors = final_clean_B - svd_transformed
        residual_magnitudes = np.sqrt(np.sum(residual_vectors**2, axis=1))
        logger.info(f"Residual vector statistics (final clean data):")
        logger.info(f"  Mean magnitude: {np.mean(residual_magnitudes)*1000:.3f}mm")
        logger.info(f"  Max magnitude: {np.max(residual_magnitudes)*1000:.3f}mm")
        logger.info(f"  Std magnitude: {np.std(residual_magnitudes)*1000:.3f}mm")
        
        self.visualize_transformation_stage("SVD Coarse Registration (FFD)", 
                                           final_clean_A, final_clean_B, svd_transformed, 
                                           svd_rmse_results, residual_vectors)
        
        # Stage 3: FFD hyperparameter optimization and model training
        logger.info("Stage 3: FFD Hyperparameter Optimization and Model Training")
        
        # 优化FFD参数
        best_ffd_params = self.optimizer.optimize_ffd_params(final_clean_A, final_clean_B, self.T_svd)
        
        # 保存CV结果
        self.optimizer.save_cv_results()
        
        if best_ffd_params is None:
            logger.error("No suitable FFD model found")
            return None, None
        
        # 使用最佳参数训练最终FFD模型
        logger.info(f"Training final FFD model: {best_ffd_params}")
        
        self.deformation_model = FFDDeformationModel(
            grid_resolution=best_ffd_params['grid_resolution'],
            constrain_y=True,
            optimization_method=best_ffd_params['optimization_method']
        )
        
        # 在最终清洁数据上训练最终模型
        success = self.deformation_model.learn_deformation(svd_transformed, residual_vectors)
        
        if not success:
            logger.error("Failed to train final FFD deformation model")
            return None, None
        
        # 应用最终FFD模型
        predicted_deformation = self.deformation_model.predict_deformation(svd_transformed)
        final_transformed = svd_transformed + predicted_deformation
        final_transformed[:, 1] = 0
        
        # 评估最终结果
        final_rmse_results = self.calculate_directional_rmse(final_clean_B, final_transformed)
        final_performance = self.evaluate_performance(final_clean_B, final_transformed)
        
        logger.info(f"Final Overall RMSE (FFD enhanced): {final_rmse_results['overall_rmse']*1000:.3f}mm")
        
        final_residuals = final_clean_B - final_transformed
        final_residual_magnitudes = np.sqrt(np.sum(final_residuals**2, axis=1))
        improvement = (np.mean(residual_magnitudes) - np.mean(final_residual_magnitudes))*1000
        logger.info(f"Final residual statistics (FFD enhanced):")
        logger.info(f"  Mean magnitude: {np.mean(final_residual_magnitudes)*1000:.3f}mm")
        logger.info(f"  Max magnitude: {np.max(final_residual_magnitudes)*1000:.3f}mm")
        logger.info(f"  Improvement: {improvement:.3f}mm")
        
        # 检查交叉验证预测与最终结果的一致性
        cv_score = self.optimizer.best_score*1000
        final_score = final_rmse_results['overall_rmse']*1000
        logger.info(f"FFD: CV predicted RMSE: {cv_score:.3f}mm")
        logger.info(f"FFD: Final actual RMSE: {final_score:.3f}mm")
        logger.info(f"FFD: Consistency (CV vs Final): {abs(cv_score - final_score):.3f}mm difference")
        
        # 可视化最终结果
        grid_str = f"{best_ffd_params['grid_resolution'][0]}x{best_ffd_params['grid_resolution'][1]}"
        self.visualize_transformation_stage(f"Final FFD Enhanced ({grid_str})", 
                                           final_clean_A, final_clean_B, final_transformed, 
                                           final_rmse_results, final_residuals)
        
        # 残差分析
        self.residual_analyzer.analyze_final_residuals(
            final_clean_A, final_clean_B, final_transformed, 
            f"Final FFD Enhanced ({grid_str})"
        )
        
        # 打印详细结果
        self._print_detailed_results(svd_performance, final_performance, best_ffd_params)
        
        # 保存数据
        self._save_transformation_data(best_ffd_params)
        
        return self.T_svd, final_rmse_results

    def _print_detailed_results(self, svd_performance, final_performance, best_ffd_params):
        """Print detailed performance analysis"""
        logger.info("\n=== Detailed FFD Enhanced Performance Analysis ===")
        
        logger.info("Data Preprocessing Summary:")
        logger.info(f"  Original points: {self.preprocessing_info['original_points']}")
        logger.info(f"  After deduplication: {self.preprocessing_info['after_deduplication']}")
        logger.info(f"  Duplicates removed: {self.preprocessing_info['duplicates_removed']}")
        logger.info(f"  Min distance between points: {self.preprocessing_info['min_distance_between_points']*1000:.3f}mm")
        
        logger.info("SVD Stage Results:")
        svd_rmse = svd_performance['rmse_results']
        logger.info(f"  Lateral RMSE: {svd_rmse['lateral_rmse']*1000:.3f}mm")
        logger.info(f"  Longitudinal RMSE: {svd_rmse['longitudinal_rmse']*1000:.3f}mm")
        logger.info(f"  Overall RMSE: {svd_rmse['overall_rmse']*1000:.3f}mm")
        
        grid_str = f"{best_ffd_params['grid_resolution'][0]}x{best_ffd_params['grid_resolution'][1]}"
        logger.info(f"Final (SVD + FFD Enhanced {grid_str}) Results:")
        final_rmse = final_performance['rmse_results']
        logger.info(f"  Lateral RMSE: {final_rmse['lateral_rmse']*1000:.3f}mm")
        logger.info(f"  Longitudinal RMSE: {final_rmse['longitudinal_rmse']*1000:.3f}mm")
        logger.info(f"  Overall RMSE: {final_rmse['overall_rmse']*1000:.3f}mm")
        
        improvement = (svd_rmse['overall_rmse'] - final_rmse['overall_rmse']) * 1000
        improvement_pct = improvement / (svd_rmse['overall_rmse'] * 1000) * 100
        logger.info(f"  Improvement: {improvement:.3f}mm ({improvement_pct:.1f}%)")
        
        logger.info("Target Achievement:")
        pass_criteria = final_performance['pass_criteria']
        logger.info(f"  Overall target achieved: {'✓' if pass_criteria['overall_pass'] else '✗'}")
        logger.info(f"  Target RMSE: {TARGET_RMSE*1000:.2f}mm")
        
        rw_eq = final_performance['real_world_equivalent']
        logger.info("Real-world Equivalent Performance:")
        logger.info(f"  Overall equivalent: {rw_eq['overall_real_world_eq']*100:.2f}cm")
        
        logger.info(f"Best FFD Model: {grid_str} grid")
        logger.info(f"Best Parameters: {best_ffd_params}")
        
        # 离群点统计
        logger.info("Outlier Detection Summary:")
        logger.info(f"  Method: {self.outlier_detector.method}")
        logger.info(f"  Outliers removed: {self.outlier_detector.outlier_stats['outliers_detected']}")
        logger.info(f"  Outlier ratio: {self.outlier_detector.outlier_stats['outlier_ratio']*100:.2f}%")
        
        # FFD特性汇总
        logger.info("FFD Features Summary:")
        logger.info("  ✓ B-Spline-based control grid deformation")
        logger.info(f"  ✓ Low parameter count: {best_ffd_params['grid_resolution'][0]*best_ffd_params['grid_resolution'][1]*2} parameters")
        logger.info("  ✓ Smooth and natural deformation field")
        logger.info("  ✓ Intuitive control over deformation resolution")

    def _save_transformation_data(self, best_ffd_params):
        """Save transformation matrices and model data"""
        if self.T_svd is not None:
            np.savetxt(os.path.join(result_dir, "T_svd_ffd.txt"), self.T_svd)
        
        # 保存FFD模型信息
        model_info_file = os.path.join(result_dir, "ffd_model_info.txt")
        with open(model_info_file, "w") as f:
            f.write(f"FFD Enhanced Model Type: Free-Form Deformation\n")
            f.write(f"Grid Resolution: {best_ffd_params['grid_resolution'][0]}x{best_ffd_params['grid_resolution'][1]}\n")
            f.write(f"Optimization Method: {best_ffd_params['optimization_method']}\n")
            f.write(f"Control Points: {best_ffd_params['grid_resolution'][0]*best_ffd_params['grid_resolution'][1]}\n")
            f.write(f"Optimization Parameters: {best_ffd_params['grid_resolution'][0]*best_ffd_params['grid_resolution'][1]*2}\n")
            f.write("\nFFD Features:\n")
            f.write("- B-Spline-based control grid deformation\n")
            f.write("- Low parameter count for stable optimization\n") 
            f.write("- Smooth and natural deformation field\n")
            f.write("- Intuitive control over deformation smoothness\n")
        
        # 保存控制网格信息
        if self.deformation_model and self.deformation_model.control_grid:
            control_grid_file = os.path.join(result_dir, "ffd_control_grid.csv")
            control_points = self.deformation_model.control_grid.control_points
            initial_points = self.deformation_model.control_grid.initial_control_positions
            displacements = control_points - initial_points
            
            grid_df = pd.DataFrame({
                'control_idx': np.arange(len(control_points)),
                'initial_x': initial_points[:, 0],
                'initial_z': initial_points[:, 2],
                'final_x': control_points[:, 0],
                'final_z': control_points[:, 2],
                'displacement_x': displacements[:, 0],
                'displacement_z': displacements[:, 2],
                'displacement_magnitude': np.sqrt(displacements[:, 0]**2 + displacements[:, 2]**2)
            })
            grid_df.to_csv(control_grid_file, index=False)
        
        # 保存数据预处理信息
        preprocessing_info_file = os.path.join(result_dir, "preprocessing_info_ffd.txt")
        with open(preprocessing_info_file, "w") as f:
            f.write("FFD Data Preprocessing Summary\n")
            f.write("=" * 40 + "\n")
            for key, value in self.preprocessing_info.items():
                if 'distance' in key:
                    f.write(f"{key}: {value*1000:.3f}mm\n")
                else:
                    f.write(f"{key}: {value}\n")
        
        # 保存离群点检测信息
        outlier_info_file = os.path.join(result_dir, "ffd_outlier_detection_info.txt")
        with open(outlier_info_file, "w") as f:
            f.write("FFD Enhanced Outlier Detection Summary\n")
            f.write("=" * 40 + "\n")
            f.write(f"Method: {self.outlier_detector.method}\n")
            f.write(f"Total points: {self.outlier_detector.outlier_stats['total_points']}\n")
            f.write(f"Outliers detected: {self.outlier_detector.outlier_stats['outliers_detected']}\n")
            f.write(f"Outlier ratio: {self.outlier_detector.outlier_stats['outlier_ratio']*100:.2f}%\n")
            f.write(f"Threshold used: {self.outlier_detector.outlier_stats['threshold_used']*1000:.3f}mm\n")
            if len(self.outlier_detector.outlier_indices) > 0:
                f.write(f"Max outlier residual: {np.max(self.outlier_detector.outlier_stats['outlier_residuals'])*1000:.3f}mm\n")
        
        logger.info("FFD Enhanced transformation data and all analysis info saved")

def process_laser_tracker_data(input_file):
    """Process laser tracker data"""
    logger.info(f"Processing laser tracker data from {input_file}")
    
    try:
        df = pd.read_csv(input_file)
        logger.info(f"Processed {len(df)} data points")
        
        return df
        
    except Exception as e:
        logger.error(f"Error processing laser tracker data: {e}")
        return None

def main():
    """Main function"""
    logger.info("Starting FFD-based 1:32 scale model tracker data processing")
    logger.info(f"Scale factor: {SCALE_FACTOR}")
    logger.info(f"Target RMSE: {TARGET_RMSE*1000:.2f}mm")
    logger.info("FFD Enhanced Features:")
    logger.info("  - Free-Form Deformation with B-Spline control grid")
    logger.info("  - Low parameter count for stable optimization")
    logger.info("  - Smooth and natural deformation field")
    logger.info("  - Intuitive control over deformation smoothness")
    logger.info("  - Data preprocessing with deduplication")
    logger.info("  - Robust outlier detection")
    logger.info("  - Comprehensive residual analysis")
    
    global LASER_TRACKER_CSV
    if len(sys.argv) > 1:
        LASER_TRACKER_CSV = sys.argv[1]
    
    if not os.path.exists(LASER_TRACKER_CSV):
        logger.error(f"Laser tracker data file {LASER_TRACKER_CSV} not found.")
        logger.info("Usage: python script.py <path_to_laser_tracker.csv>")
        return
    
    # Process laser tracker data
    logger.info("Processing laser tracker data...")
    df = process_laser_tracker_data(LASER_TRACKER_CSV)
    
    if df is None:
        logger.error("Failed to process laser tracker data")
        return
    
    # Prepare data for transformation
    positions_A = [[row[ROW_X], 0, row[ROW_Y]] for _, row in df.iterrows()]
    positions_B = [[row["laser_x"], 0, row["laser_z"]] for _, row in df.iterrows()]
    
    logger.info(f"Loaded {len(positions_A)} point pairs for registration")
    
    # Perform FFD-based two-stage registration
    transformer = CoordinateTransformer(positions_A, positions_B)
    T_svd, final_rmse_results = transformer.two_stage_registration_ffd()
    
    if T_svd is not None and final_rmse_results is not None:
        success = final_rmse_results['overall_rmse'] <= TARGET_RMSE
        if success:
            logger.info(f"SUCCESS: FFD Enhanced registration completed with RMSE {final_rmse_results['overall_rmse']*1000:.3f}mm")
        else:
            logger.info(f"PARTIAL: FFD Enhanced registration completed, but RMSE {final_rmse_results['overall_rmse']*1000:.3f}mm exceeds target {TARGET_RMSE*1000:.2f}mm")
        
        # Save final summary
        summary_file = os.path.join(output_dir, "ffd_registration_summary.txt")
        with open(summary_file, "w") as f:
            f.write("FFD Enhanced Two-Stage Registration Summary\n")
            f.write("=" * 80 + "\n")
            f.write(f"Scale Factor: {SCALE_FACTOR}\n")
            f.write(f"Target RMSE: {TARGET_RMSE*1000:.2f}mm\n")
            f.write(f"Final RMSE: {final_rmse_results['overall_rmse']*1000:.3f}mm\n")
            f.write(f"Success: {'Yes' if success else 'No'}\n")
            f.write(f"Original points: {len(positions_A)}\n")
            if hasattr(transformer, 'preprocessing_info'):
                f.write(f"After preprocessing: {transformer.preprocessing_info['after_deduplication']}\n")
                f.write(f"Duplicates removed: {transformer.preprocessing_info['duplicates_removed']}\n")
            f.write(f"Clean points used: {len(transformer.clean_indices) if transformer.clean_indices is not None else 'N/A'}\n")
            f.write(f"Real-world equivalent: {final_rmse_results['overall_rmse']*SCALE_FACTOR*100:.2f}cm\n")
            f.write(f"Best model type: FFD (Free-Form Deformation)\n")
            if hasattr(transformer, 'deformation_model') and transformer.deformation_model:
                grid_res = transformer.deformation_model.grid_resolution
                f.write(f"Control grid resolution: {grid_res[0]}x{grid_res[1]}\n")
                f.write(f"Control points: {grid_res[0]*grid_res[1]}\n")
                f.write(f"Optimization parameters: {grid_res[0]*grid_res[1]*2}\n")
            f.write(f"Outlier detection method: {transformer.outlier_detector.method}\n")
            f.write("\nFFD Features:\n")
            f.write("- B-Spline-based control grid deformation\n")
            f.write("- Low parameter count for stable optimization\n")
            f.write("- Smooth and natural deformation field\n")
            f.write("- Intuitive control over deformation smoothness\n")
            f.write("\nEnhanced features:\n")
            f.write("- Data preprocessing with deduplication and quality check\n")
            f.write("- Robust statistical outlier detection\n")
            f.write("- Cross-validation with proper parameter handling\n")
            f.write("- Automatic model selection\n")
            f.write("- Comprehensive residual analysis\n")
    else:
        logger.error("FFD Enhanced registration failed")
    
    logger.info("FFD Enhanced processing complete")

if __name__ == "__main__":
    main()