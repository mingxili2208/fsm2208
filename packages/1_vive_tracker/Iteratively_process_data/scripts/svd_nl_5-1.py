#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Tracker data processing script with SVD + GPR Deformation Learning 
(Fixed: Improved numerical stability and performance optimization)
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
from sklearn.gaussian_process import GaussianProcessRegressor
from sklearn.gaussian_process.kernels import RBF, Matern, WhiteKernel, ConstantKernel as C
from scipy.interpolate import griddata, RBFInterpolator, LSQBivariateSpline
from scipy.spatial.distance import cdist
from scipy import stats
from scipy.optimize import minimize
import itertools
from concurrent.futures import ProcessPoolExecutor
import warnings
import time
import signal
from contextlib import contextmanager
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
output_dir = os.path.join(result_dir, f"SVD_GPR_Enhanced_{csv_filename}_{timestamp}_1to32_scale_RMSE_{TARGET_RMSE*1000:.1f}mm")
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

ROW_X="X"
ROW_Y="Z"

# 添加超时装饰器
@contextmanager
def timeout(duration):
    """超时上下文管理器"""
    def timeout_handler(signum, frame):
        raise TimeoutError(f"Operation timed out after {duration} seconds")
    
    # 设置信号处理器
    old_handler = signal.signal(signal.SIGALRM, timeout_handler)
    signal.alarm(duration)
    
    try:
        yield
    finally:
        # 恢复原来的处理器
        signal.alarm(0)
        signal.signal(signal.SIGALRM, old_handler)

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

class GPRDeformationModel:
    """
    基于高斯过程回归的非刚性形变模型 (GPR - 改进数值稳定性和性能)
    """
    def __init__(self, kernel_type='RBF+White', length_scale=1.0, noise_level=1e-5, 
                 alpha=1e-10, normalize_y=True, constrain_y=True, n_restarts_optimizer=3,
                 max_iter=1000, optimizer='fmin_l_bfgs_b'):
        """
        参数:
        kernel_type: 'RBF', 'Matern', 'RBF+White', 'Matern+White', 'composite'
        length_scale: 核函数的长度尺度
        noise_level: 噪声水平（用于WhiteKernel）
        alpha: 对角线正则化项 (增加以提高数值稳定性)
        normalize_y: 是否标准化目标值
        n_restarts_optimizer: 超参数优化重启次数 (减少以提高速度)
        max_iter: 最大迭代次数
        optimizer: 优化器选择
        """
        self.kernel_type = kernel_type
        self.length_scale = length_scale
        self.noise_level = max(noise_level, 1e-8)  # 确保最小噪声水平
        self.alpha = max(alpha, 1e-10)  # 确保最小正则化
        self.normalize_y = normalize_y
        self.constrain_y = constrain_y
        self.n_restarts_optimizer = n_restarts_optimizer
        self.max_iter = max_iter
        self.optimizer = optimizer
        
        # GPR模型（X和Z方向各一个）
        self.gpr_x = None
        self.gpr_z = None
        self.source_points = None
        self.training_residuals = None
        
        # 不确定性分析相关
        self.uncertainty_threshold = None
        self.high_uncertainty_regions = None
        
        logger.info(f"GPR Deformation Model initialized (Optimized):")
        logger.info(f"  Kernel: {kernel_type}")
        logger.info(f"  Length scale: {length_scale}")
        logger.info(f"  Noise level: {self.noise_level}")
        logger.info(f"  Alpha: {self.alpha}")
        logger.info(f"  Normalize Y: {normalize_y}")
        logger.info(f"  Optimizer restarts: {n_restarts_optimizer}")
        logger.info(f"  Max iterations: {max_iter}")
        
    def _create_kernel(self):
        """创建GPR核函数（优化版本）"""
        # 使用更保守的边界以提高数值稳定性
        if self.kernel_type == 'RBF':
            kernel = C(1.0, (1e-2, 1e2)) * RBF(
                length_scale=self.length_scale, 
                length_scale_bounds=(1e-1, 1e1)
            )
        elif self.kernel_type == 'Matern':
            kernel = C(1.0, (1e-2, 1e2)) * Matern(
                length_scale=self.length_scale, 
                length_scale_bounds=(1e-1, 1e1), 
                nu=1.5  # 使用更简单的nu值
            )
        elif self.kernel_type == 'RBF+White':
            kernel = (C(1.0, (1e-2, 1e2)) * RBF(
                length_scale=self.length_scale, 
                length_scale_bounds=(1e-1, 1e1)
            ) + WhiteKernel(
                noise_level=self.noise_level, 
                noise_level_bounds=(1e-8, 1e-2)
            ))
        elif self.kernel_type == 'Matern+White':
            kernel = (C(1.0, (1e-2, 1e2)) * Matern(
                length_scale=self.length_scale, 
                length_scale_bounds=(1e-1, 1e1), 
                nu=1.5
            ) + WhiteKernel(
                noise_level=self.noise_level, 
                noise_level_bounds=(1e-8, 1e-2)
            ))
        elif self.kernel_type == 'composite':
            # 简化的组合核
            global_kernel = C(0.5, (1e-2, 1e1)) * RBF(
                length_scale=self.length_scale*2, 
                length_scale_bounds=(1e-1, 1e1)
            )
            local_kernel = C(0.5, (1e-2, 1e1)) * RBF(
                length_scale=self.length_scale*0.5, 
                length_scale_bounds=(1e-2, 1e0)
            )
            noise_kernel = WhiteKernel(
                noise_level=self.noise_level, 
                noise_level_bounds=(1e-8, 1e-2)
            )
            kernel = global_kernel + local_kernel + noise_kernel
        else:
            # 默认使用简单的RBF
            kernel = C(1.0, (1e-2, 1e2)) * RBF(
                length_scale=self.length_scale, 
                length_scale_bounds=(1e-1, 1e1)
            )
        
        return kernel
    
    def learn_deformation(self, source_points, residual_vectors, timeout_seconds=300):
        """学习GPR形变模型（添加超时和数值稳定性改进）"""
        source_points = np.array(source_points)
        residual_vectors = np.array(residual_vectors)
        
        if self.constrain_y:
            source_xz = source_points[:, [0, 2]]
            residual_xz = residual_vectors[:, [0, 2]]
            
            # 数据预处理：检查和处理数值问题
            if np.any(np.isnan(source_xz)) or np.any(np.isnan(residual_xz)):
                logger.error("NaN values detected in input data")
                return False
            
            if np.any(np.isinf(source_xz)) or np.any(np.isinf(residual_xz)):
                logger.error("Infinite values detected in input data")
                return False
            
            # 检查数据规模
            n_samples = len(source_xz)
            if n_samples > 1000:
                logger.warning(f"Large dataset ({n_samples} points) may cause slow training")
                # 可以考虑子采样
                subsample_idx = np.random.choice(n_samples, 1000, replace=False)
                source_xz = source_xz[subsample_idx]
                residual_xz = residual_xz[subsample_idx]
                logger.info(f"Subsampled to {len(source_xz)} points for faster training")
            
            # 保存训练数据
            self.source_points = source_xz
            self.training_residuals = residual_xz
            
            try:
                # 创建核函数
                kernel_x = self._create_kernel()
                kernel_z = self._create_kernel()
                
                # 创建GPR模型（使用更保守的参数）
                gpr_params = {
                    'alpha': self.alpha,
                    'normalize_y': self.normalize_y,
                    'n_restarts_optimizer': self.n_restarts_optimizer,
                    'random_state': 42,
                    'optimizer': self.optimizer
                }
                
                self.gpr_x = GaussianProcessRegressor(kernel=kernel_x, **gpr_params)
                self.gpr_z = GaussianProcessRegressor(kernel=kernel_z, **gpr_params)
                
                # 训练X方向模型（带超时）
                logger.info("Training GPR model for X-direction deformation...")
                start_time = time.time()
                
                try:
                    with timeout(timeout_seconds):
                        self.gpr_x.fit(source_xz, residual_xz[:, 0])
                    
                    x_train_time = time.time() - start_time
                    logger.info(f"X-direction training completed in {x_train_time:.2f}s")
                    
                except TimeoutError:
                    logger.error(f"X-direction training timed out after {timeout_seconds}s")
                    return False
                except Exception as e:
                    logger.error(f"X-direction training failed: {e}")
                    return False
                
                # 训练Z方向模型（带超时）
                logger.info("Training GPR model for Z-direction deformation...")
                start_time = time.time()
                
                try:
                    with timeout(timeout_seconds):
                        self.gpr_z.fit(source_xz, residual_xz[:, 1])
                    
                    z_train_time = time.time() - start_time
                    logger.info(f"Z-direction training completed in {z_train_time:.2f}s")
                    
                except TimeoutError:
                    logger.error(f"Z-direction training timed out after {timeout_seconds}s")
                    return False
                except Exception as e:
                    logger.error(f"Z-direction training failed: {e}")
                    return False
                
                # 记录优化后的超参数
                logger.info("GPR model training completed successfully:")
                logger.info(f"  X-direction kernel: {self.gpr_x.kernel_}")
                logger.info(f"  Z-direction kernel: {self.gpr_z.kernel_}")
                
                try:
                    x_log_likelihood = self.gpr_x.log_marginal_likelihood()
                    z_log_likelihood = self.gpr_z.log_marginal_likelihood()
                    logger.info(f"  X-direction log marginal likelihood: {x_log_likelihood:.3f}")
                    logger.info(f"  Z-direction log marginal likelihood: {z_log_likelihood:.3f}")
                except Exception as e:
                    logger.warning(f"Could not compute log marginal likelihood: {e}")
                
                # 计算训练数据的拟合质量
                try:
                    train_pred_x, train_std_x = self.gpr_x.predict(source_xz, return_std=True)
                    train_pred_z, train_std_z = self.gpr_z.predict(source_xz, return_std=True)
                    
                    train_rmse_x = np.sqrt(np.mean((residual_xz[:, 0] - train_pred_x)**2))
                    train_rmse_z = np.sqrt(np.mean((residual_xz[:, 1] - train_pred_z)**2))
                    
                    logger.info(f"  Training RMSE - X: {train_rmse_x*1000:.3f}mm, Z: {train_rmse_z*1000:.3f}mm")
                    logger.info(f"  Mean prediction std - X: {np.mean(train_std_x)*1000:.3f}mm, Z: {np.mean(train_std_z)*1000:.3f}mm")
                except Exception as e:
                    logger.warning(f"Could not compute training statistics: {e}")
                
                return True
                
            except Exception as e:
                logger.error(f"Failed to learn GPR model: {e}")
                logger.error(f"Error details: {type(e).__name__}: {str(e)}")
                return False
        else:
            logger.warning("3D GPR deformation not implemented with Y-axis constraint")
            return False
    
    def predict_deformation(self, query_points, return_uncertainty=True):
        """
        预测形变（包含不确定性估计，优化版本）
        """
        if self.gpr_x is None or self.gpr_z is None:
            logger.error("GPR model not trained yet")
            if return_uncertainty:
                return np.zeros_like(query_points), np.ones(len(query_points)) * np.inf
            else:
                return np.zeros_like(query_points)
        
        query_points = np.array(query_points)
        
        if self.constrain_y:
            query_xz = query_points[:, [0, 2]]
            
            try:
                # 预测形变和不确定性
                if return_uncertainty:
                    deformation_x, std_x = self.gpr_x.predict(query_xz, return_std=True)
                    deformation_z, std_z = self.gpr_z.predict(query_xz, return_std=True)
                    
                    # 计算总不确定性（X和Z方向的组合）
                    total_uncertainty = np.sqrt(std_x**2 + std_z**2)
                else:
                    deformation_x = self.gpr_x.predict(query_xz, return_std=False)
                    deformation_z = self.gpr_z.predict(query_xz, return_std=False)
                
                # 构建形变向量
                deformation = np.zeros_like(query_points)
                deformation[:, 0] = deformation_x
                deformation[:, 2] = deformation_z
                
                if return_uncertainty:
                    return deformation, total_uncertainty
                else:
                    return deformation
                
            except Exception as e:
                logger.error(f"Failed to predict GPR deformation: {e}")
                if return_uncertainty:
                    return np.zeros_like(query_points), np.ones(len(query_points)) * np.inf
                else:
                    return np.zeros_like(query_points)
        else:
            if return_uncertainty:
                return np.zeros_like(query_points), np.ones(len(query_points)) * np.inf
            else:
                return np.zeros_like(query_points)
    
    def analyze_uncertainty(self, query_points=None, uncertainty_threshold=None):
        """
        分析预测不确定性
        """
        if self.gpr_x is None or self.gpr_z is None:
            logger.warning("GPR model not trained, cannot analyze uncertainty")
            return None
        
        if query_points is None:
            # 使用训练数据点进行分析
            query_points = self.source_points
        else:
            query_points = np.array(query_points)
            if self.constrain_y:
                query_points = query_points[:, [0, 2]]
        
        try:
            # 预测不确定性
            _, uncertainty = self.predict_deformation(
                np.column_stack([query_points, np.zeros(len(query_points))]), 
                return_uncertainty=True
            )
            
            # 设置不确定性阈值
            if uncertainty_threshold is None:
                self.uncertainty_threshold = np.percentile(uncertainty, 75)  # 75th percentile
            else:
                self.uncertainty_threshold = uncertainty_threshold
            
            # 识别高不确定性区域
            high_uncertainty_mask = uncertainty > self.uncertainty_threshold
            self.high_uncertainty_regions = query_points[high_uncertainty_mask]
            
            uncertainty_stats = {
                'mean_uncertainty': np.mean(uncertainty),
                'std_uncertainty': np.std(uncertainty),
                'max_uncertainty': np.max(uncertainty),
                'min_uncertainty': np.min(uncertainty),
                'uncertainty_threshold': self.uncertainty_threshold,
                'high_uncertainty_ratio': np.sum(high_uncertainty_mask) / len(uncertainty),
                'high_uncertainty_points': self.high_uncertainty_regions
            }
            
            logger.info("GPR Uncertainty Analysis:")
            logger.info(f"  Mean uncertainty: {uncertainty_stats['mean_uncertainty']*1000:.3f}mm")
            logger.info(f"  Std uncertainty: {uncertainty_stats['std_uncertainty']*1000:.3f}mm")
            logger.info(f"  Max uncertainty: {uncertainty_stats['max_uncertainty']*1000:.3f}mm")
            logger.info(f"  Uncertainty threshold: {uncertainty_stats['uncertainty_threshold']*1000:.3f}mm")
            logger.info(f"  High uncertainty ratio: {uncertainty_stats['high_uncertainty_ratio']*100:.1f}%")
            
            return uncertainty_stats
            
        except Exception as e:
            logger.error(f"Failed to analyze uncertainty: {e}")
            return None
    
    def visualize_uncertainty_map(self, bounds=None, resolution=30):
        """
        绘制不确定性地图（降低分辨率以提高性能）
        """
        if self.gpr_x is None or self.gpr_z is None:
            logger.warning("GPR model not trained, cannot create uncertainty map")
            return
        
        try:
            if bounds is None and self.source_points is not None:
                # 基于训练数据确定边界
                x_min, x_max = np.min(self.source_points[:, 0]), np.max(self.source_points[:, 0])
                z_min, z_max = np.min(self.source_points[:, 1]), np.max(self.source_points[:, 1])
                
                # 扩展边界
                x_range = x_max - x_min
                z_range = z_max - z_min
                x_min -= 0.1 * x_range
                x_max += 0.1 * x_range
                z_min -= 0.1 * z_range
                z_max += 0.1 * z_range
            else:
                x_min, x_max, z_min, z_max = bounds
            
            # 创建网格（降低分辨率）
            x_grid = np.linspace(x_min, x_max, resolution)
            z_grid = np.linspace(z_min, z_max, resolution)
            X_grid, Z_grid = np.meshgrid(x_grid, z_grid)
            grid_points = np.column_stack([X_grid.ravel(), Z_grid.ravel()])
            
            # 预测网格点的不确定性
            query_3d = np.column_stack([grid_points, np.zeros(len(grid_points))])
            _, uncertainty_grid = self.predict_deformation(query_3d, return_uncertainty=True)
            uncertainty_grid = uncertainty_grid.reshape(X_grid.shape)
            
            # 创建可视化
            fig, axes = plt.subplots(1, 3, figsize=(20, 6))
            
            # 1. 不确定性热力图
            ax1 = axes[0]
            im1 = ax1.contourf(X_grid, Z_grid, uncertainty_grid*1000, levels=20, cmap='viridis')
            plt.colorbar(im1, ax=ax1, label='Uncertainty [mm]')
            
            # 叠加训练点
            if self.source_points is not None:
                ax1.scatter(self.source_points[:, 0], self.source_points[:, 1], 
                           c='red', s=20, alpha=0.7, label='Training Points')
            
            # 叠加高不确定性区域
            if self.high_uncertainty_regions is not None and len(self.high_uncertainty_regions) > 0:
                ax1.scatter(self.high_uncertainty_regions[:, 0], self.high_uncertainty_regions[:, 1], 
                           c='white', s=50, marker='x', alpha=0.8, label='High Uncertainty')
            
            ax1.set_xlabel('X [m]')
            ax1.set_ylabel('Z [m]')
            ax1.set_title('GPR Prediction Uncertainty Map')
            ax1.legend()
            ax1.grid(True, alpha=0.3)
            
            # 2. 不确定性等高线图
            ax2 = axes[1]
            contour = ax2.contour(X_grid, Z_grid, uncertainty_grid*1000, levels=10, colors='black', alpha=0.6)
            ax2.clabel(contour, inline=True, fontsize=8, fmt='%.1f mm')
            im2 = ax2.contourf(X_grid, Z_grid, uncertainty_grid*1000, levels=20, cmap='plasma', alpha=0.7)
            plt.colorbar(im2, ax=ax2, label='Uncertainty [mm]')
            
            if self.source_points is not None:
                ax2.scatter(self.source_points[:, 0], self.source_points[:, 1], 
                           c='blue', s=15, alpha=0.8, label='Training Points')
            
            ax2.set_xlabel('X [m]')
            ax2.set_ylabel('Z [m]')
            ax2.set_title('GPR Uncertainty Contours')
            ax2.legend()
            ax2.grid(True, alpha=0.3)
            
            # 3. 不确定性分布直方图
            ax3 = axes[2]
            ax3.hist(uncertainty_grid.ravel()*1000, bins=30, alpha=0.7, color='skyblue', 
                    edgecolor='black', density=True)
            ax3.axvline(x=np.mean(uncertainty_grid)*1000, color='red', linestyle='-', linewidth=2,
                       label=f'Mean ({np.mean(uncertainty_grid)*1000:.2f}mm)')
            ax3.axvline(x=np.median(uncertainty_grid)*1000, color='green', linestyle='-', linewidth=2,
                       label=f'Median ({np.median(uncertainty_grid)*1000:.2f}mm)')
            if self.uncertainty_threshold is not None:
                ax3.axvline(x=self.uncertainty_threshold*1000, color='orange', linestyle='--', linewidth=2,
                           label=f'Threshold ({self.uncertainty_threshold*1000:.2f}mm)')
            
            ax3.set_xlabel('Uncertainty [mm]')
            ax3.set_ylabel('Density')
            ax3.set_title('Uncertainty Distribution')
            ax3.legend()
            ax3.grid(True, alpha=0.3)
            
            plt.tight_layout()
            plt.savefig(os.path.join(img_dir, 'gpr_uncertainty_map.png'), dpi=300, bbox_inches='tight')
            plt.close()
            
            logger.info("GPR uncertainty map saved: gpr_uncertainty_map.png")
            
        except Exception as e:
            logger.error(f"Failed to create uncertainty map: {e}")

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

class HyperparameterOptimizer:
    """
    GPR超参数优化器，使用交叉验证（优化性能版本）
    """
    def __init__(self, cv_folds=3, random_state=42):  # 减少CV折数
        self.cv_folds = cv_folds
        self.random_state = random_state
        self.best_params = None
        self.best_score = float('inf')
        self.cv_results = []
        
    def optimize_gpr_params(self, positions_A, positions_B, T_svd):
        """优化GPR模型参数（性能优化版本）"""
        logger.info("Starting GPR hyperparameter optimization with cross-validation (Optimized)...")
        
        # 减少参数网格以提高速度
        param_grid = {
            'kernel_type': ['RBF+White', 'Matern+White'],  # 减少核函数类型
            'length_scale': [0.5, 1.0, 2.0],  # 减少长度尺度选项
            'noise_level': [1e-5, 1e-4],  # 减少噪声水平选项
            'alpha': [1e-10, 1e-8],  # 减少alpha选项
            'normalize_y': [True]  # 只使用标准化
        }
        
        # 生成参数组合
        param_combinations = []
        for kernel_type in param_grid['kernel_type']:
            for length_scale in param_grid['length_scale']:
                for noise_level in param_grid['noise_level']:
                    for alpha in param_grid['alpha']:
                        for normalize_y in param_grid['normalize_y']:
                            param_combinations.append({
                                'kernel_type': kernel_type,
                                'length_scale': length_scale,
                                'noise_level': noise_level,
                                'alpha': alpha,
                                'normalize_y': normalize_y
                            })
        
        logger.info(f"Testing {len(param_combinations)} parameter combinations (reduced for speed)")
        
        # 设置交叉验证
        kf = KFold(n_splits=self.cv_folds, shuffle=True, random_state=self.random_state)
        
        positions_A = np.array(positions_A)
        positions_B = np.array(positions_B)
        indices = np.arange(len(positions_A))
        
        best_params = None
        best_mean_rmse = float('inf')
        
        for i, params in enumerate(param_combinations):
            fold_rmses = []
            
            try:
                logger.info(f"Testing combination {i+1}/{len(param_combinations)}: {params}")
                
                for fold, (train_idx, test_idx) in enumerate(kf.split(indices)):
                    # 分割训练和测试数据
                    train_A = positions_A[train_idx]
                    train_B = positions_B[train_idx]
                    test_A = positions_A[test_idx]
                    test_B = positions_B[test_idx]
                    
                    # 在训练集上执行SVD变换
                    svd_train_transformed = self._apply_transformation(train_A, T_svd)
                    residual_vectors = train_B - svd_train_transformed
                    
                    # 训练GPR形变模型（使用更短的超时和更少的重启）
                    model = GPRDeformationModel(
                        kernel_type=params['kernel_type'],
                        length_scale=params['length_scale'],
                        noise_level=params['noise_level'],
                        alpha=params['alpha'],
                        normalize_y=params['normalize_y'],
                        constrain_y=True,
                        n_restarts_optimizer=1,  # 减少重启次数
                        max_iter=500  # 减少最大迭代次数
                    )
                    
                    # 使用较短的超时时间
                    success = model.learn_deformation(svd_train_transformed, residual_vectors, timeout_seconds=120)
                    
                    if not success:
                        logger.warning(f"  Fold {fold+1} failed, skipping...")
                        fold_rmses.append(float('inf'))
                        continue
                    
                    # 在测试集上评估
                    svd_test_transformed = self._apply_transformation(test_A, T_svd)
                    predicted_deformation = model.predict_deformation(svd_test_transformed, return_uncertainty=False)
                    final_test_transformed = svd_test_transformed + predicted_deformation
                    final_test_transformed[:, 1] = 0  # 确保Y=0
                    
                    # 计算测试集RMSE
                    test_rmse = self._calculate_rmse(test_B, final_test_transformed)
                    fold_rmses.append(test_rmse)
                    logger.info(f"    Fold {fold+1}: RMSE={test_rmse*1000:.3f}mm")
                
                # 计算平均RMSE（忽略失败的fold）
                valid_rmses = [rmse for rmse in fold_rmses if not np.isinf(rmse)]
                if len(valid_rmses) >= self.cv_folds // 2:  # 至少一半的fold成功
                    mean_rmse = np.mean(valid_rmses)
                    std_rmse = np.std(valid_rmses)
                    
                    self.cv_results.append({
                        'kernel_type': params['kernel_type'],
                        'length_scale': params['length_scale'],
                        'noise_level': params['noise_level'],
                        'alpha': params['alpha'],
                        'normalize_y': params['normalize_y'],
                        'mean_rmse': mean_rmse,
                        'std_rmse': std_rmse,
                        'fold_rmses': valid_rmses,
                        'valid_folds': len(valid_rmses)
                    })
                    
                    logger.info(f"  Result: RMSE={mean_rmse*1000:.3f}±{std_rmse*1000:.3f}mm ({len(valid_rmses)}/{self.cv_folds} folds)")
                    
                    if mean_rmse < best_mean_rmse:
                        best_mean_rmse = mean_rmse
                        best_params = params.copy()
                else:
                    logger.warning(f"  Too many failed folds ({len(valid_rmses)}/{self.cv_folds}), skipping combination")
                
            except Exception as e:
                logger.warning(f"Failed to evaluate GPR parameters {params}: {e}")
                continue
        
        self.best_params = best_params
        self.best_score = best_mean_rmse
        
        if best_params:
            logger.info(f"Best GPR parameters: {best_params}")
            logger.info(f"Best CV RMSE: {best_mean_rmse*1000:.3f}mm")
        else:
            logger.error("No valid parameter combination found")
        
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
            cv_df.to_csv(os.path.join(result_dir, "cv_results_gpr.csv"), index=False)
            
            # 可视化CV结果
            self._plot_cv_results()
    
    def _plot_cv_results(self):
        """可视化交叉验证结果"""
        if not self.cv_results:
            return
        
        try:
            df = pd.DataFrame(self.cv_results)
            
            # 创建简化的可视化
            fig, axes = plt.subplots(2, 2, figsize=(16, 12))
            axes = axes.ravel()
            
            # 1. 按核函数类型分组
            ax = axes[0]
            kernel_groups = df.groupby('kernel_type')
            for kernel, group in kernel_groups:
                ax.errorbar(range(len(group)), group['mean_rmse']*1000, 
                           yerr=group['std_rmse']*1000, 
                           marker='o', capsize=5, label=kernel)
            ax.set_xlabel('Configuration Index')
            ax.set_ylabel('RMSE [mm]')
            ax.set_title('GPR CV Results by Kernel Type')
            ax.axhline(y=TARGET_RMSE*1000, color='r', linestyle='--', alpha=0.7, label='Target')
            ax.legend()
            ax.grid(True, alpha=0.3)
            
            # 2. 长度尺度影响
            ax = axes[1]
            length_groups = df.groupby('length_scale')
            length_scales = sorted(length_groups.groups.keys())
            mean_rmses = [length_groups.get_group(ls)['mean_rmse'].mean()*1000 for ls in length_scales]
            std_rmses = [length_groups.get_group(ls)['mean_rmse'].std()*1000 for ls in length_scales]
            ax.errorbar(length_scales, mean_rmses, yerr=std_rmses, marker='o', capsize=5)
            ax.set_xlabel('Length Scale')
            ax.set_ylabel('Mean RMSE [mm]')
            ax.set_title('Effect of Length Scale')
            ax.axhline(y=TARGET_RMSE*1000, color='r', linestyle='--', alpha=0.7, label='Target')
            ax.grid(True, alpha=0.3)
            ax.legend()
            
            # 3. 噪声水平影响
            ax = axes[2]
            noise_groups = df.groupby('noise_level')
            noise_levels = sorted(noise_groups.groups.keys())
            mean_rmses = [noise_groups.get_group(nl)['mean_rmse'].mean()*1000 for nl in noise_levels]
            std_rmses = [noise_groups.get_group(nl)['mean_rmse'].std()*1000 for nl in noise_levels]
            ax.errorbar(noise_levels, mean_rmses, yerr=std_rmses, marker='o', capsize=5)
            ax.set_xscale('log')
            ax.set_xlabel('Noise Level')
            ax.set_ylabel('Mean RMSE [mm]')
            ax.set_title('Effect of Noise Level')
            ax.axhline(y=TARGET_RMSE*1000, color='r', linestyle='--', alpha=0.7, label='Target')
            ax.grid(True, alpha=0.3)
            ax.legend()
            
            # 4. 最佳结果汇总
            ax = axes[3]
            best_results = df.nsmallest(min(5, len(df)), 'mean_rmse')  # 显示最多5个
            ax.barh(range(len(best_results)), best_results['mean_rmse']*1000, 
                   xerr=best_results['std_rmse']*1000, capsize=5)
            ax.set_yticks(range(len(best_results)))
            ax.set_yticklabels([f"{row['kernel_type'][:3]}_ls{row['length_scale']}" 
                               for _, row in best_results.iterrows()])
            ax.set_xlabel('RMSE [mm]')
            ax.set_title('Top GPR Configurations')
            ax.axvline(x=TARGET_RMSE*1000, color='r', linestyle='--', alpha=0.7, label='Target')
            ax.grid(True, alpha=0.3)
            ax.legend()
            
            plt.tight_layout()
            plt.savefig(os.path.join(img_dir, 'gpr_cv_results_analysis.png'), dpi=300, bbox_inches='tight')
            plt.close()
            
            logger.info("GPR CV results visualization saved: gpr_cv_results_analysis.png")
            
        except Exception as e:
            logger.error(f"Failed to create CV results visualization: {e}")

# 继续原有的CoordinateTransformer类，但使用优化的GPR模型
class CoordinateTransformer:
    """
    Enhanced two-stage coordinate transformation with optimized GPR-based uncertainty-aware deformation learning
    """
    def __init__(self, positions_A=None, positions_B=None):
        self.T_svd = None
        self.deformation_model = None
        self.best_model_type = "GPR"
        self.__positions_A = positions_A
        self.__positions_B = positions_B
        self.optimizer = HyperparameterOptimizer()
        self.outlier_detector = OutlierDetector(method='robust_iqr', iqr_factor=2.0)
        self.residual_analyzer = ResidualAnalyzer()
        self.preprocessor = DataPreprocessor()
        self.clean_indices = None
        self.preprocessing_info = None
        self.uncertainty_stats = None
        
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

    def visualize_transformation_stage(self, stage_name, positions_A, positions_B, transformed_positions, rmse_results, residual_vectors=None, uncertainty=None):
        """Visualize transformation results for a specific stage with uncertainty visualization"""
        positions_A = np.array(positions_A)
        positions_B = np.array(positions_B)
        transformed_positions = np.array(transformed_positions)
        
        positions_A[:, 1] = 0
        positions_B[:, 1] = 0
        transformed_positions[:, 1] = 0
        
        individual_errors = np.sqrt((positions_B[:, 0] - transformed_positions[:, 0])**2 + 
                                   (positions_B[:, 2] - transformed_positions[:, 2])**2)
        
        # 创建基本的4子图布局
        fig = plt.figure(figsize=(20, 12))
        ax1 = fig.add_subplot(221, projection='3d')
        ax2 = fig.add_subplot(222)
        ax3 = fig.add_subplot(223)
        ax4 = fig.add_subplot(224)
        
        # 3D散点图
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
        ax2.hist(individual_errors * 1000, bins=20, color='skyblue', edgecolor='black', alpha=0.7)
        ax2.axvline(x=TARGET_RMSE*1000, color='r', linestyle='--', linewidth=2, 
                   label=f'Target RMSE ({TARGET_RMSE*1000:.1f}mm)')
        ax2.set_xlabel('Individual Point Error [mm]')
        ax2.set_ylabel('Frequency')
        ax2.set_title(f'{stage_name} - Error Distribution')
        ax2.legend()
        ax2.grid(True, alpha=0.3)
        
        # RMSE by direction
        ax3.bar(['Lateral', 'Longitudinal', 'Horizontal', 'Overall'],
               [rmse_results['lateral_rmse'] * 1000,
                rmse_results['longitudinal_rmse'] * 1000,
                rmse_results['horizontal_rmse'] * 1000,
                rmse_results['overall_rmse'] * 1000],
               color=['lightblue', 'lightcoral', 'lightyellow', 'lightpink'])
        ax3.axhline(y=TARGET_RMSE*1000, color='red', linestyle='--', alpha=0.7, label='Target')
        ax3.set_ylabel('RMSE [mm]')
        ax3.set_title(f'{stage_name} - RMSE by Direction')
        ax3.tick_params(axis='x', rotation=45)
        ax3.grid(True, alpha=0.3)
        ax3.legend()
        
        # XZ平面视图（如果有不确定性，添加不确定性着色）
        if uncertainty is not None:
            scatter = ax4.scatter(positions_A[:, 0], positions_A[:, 2], 
                                c=uncertainty*1000, cmap='viridis', s=50, alpha=0.7,
                                label='Source Points (colored by uncertainty)')
            plt.colorbar(scatter, ax=ax4, label='Uncertainty [mm]')
        else:
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

    def two_stage_registration_gpr_enhanced(self):
        """Perform enhanced two-stage registration with optimized GPR-based uncertainty-aware deformation learning"""
        if self.__positions_A is None or self.__positions_B is None:
            logger.error("Please provide position data first!")
            return None, None
        
        logger.info("=== Starting Optimized GPR-Enhanced Two-Stage Registration ===")
        logger.info("Optimized GPR Features:")
        logger.info("  - Improved numerical stability and performance")
        logger.info("  - Timeout protection for training")
        logger.info("  - Reduced parameter grid for faster optimization")
        logger.info("  - Data subsampling for large datasets")
        logger.info("  - Error handling and graceful degradation")
        
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
        clean_data_df.to_csv(os.path.join(result_dir, "final_clean_dataset.csv"), index=False)
        
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
        
        self.visualize_transformation_stage("SVD Coarse Registration (Optimized GPR)", 
                                           final_clean_A, final_clean_B, svd_transformed, 
                                           svd_rmse_results, residual_vectors)
        
        # Stage 3: Optimized GPR hyperparameter optimization
        logger.info("Stage 3: Optimized GPR Hyperparameter Optimization and Model Training")
        
        # 优化GPR参数
        best_gpr_params = self.optimizer.optimize_gpr_params(final_clean_A, final_clean_B, self.T_svd)
        
        # 保存CV结果
        self.optimizer.save_cv_results()
        
        if best_gpr_params is None:
            logger.error("No suitable GPR parameters found")
            return None, None
        
        # 使用最佳参数训练最终GPR模型
        logger.info(f"Training final optimized GPR model with parameters: {best_gpr_params}")
        
        self.deformation_model = GPRDeformationModel(
            kernel_type=best_gpr_params['kernel_type'],
            length_scale=best_gpr_params['length_scale'],
            noise_level=best_gpr_params['noise_level'],
            alpha=best_gpr_params['alpha'],
            normalize_y=best_gpr_params['normalize_y'],
            constrain_y=True,
            n_restarts_optimizer=5,  # 最终模型使用适中的重启次数
            max_iter=1000
        )
        
        # 在最终清洁数据上训练最终模型（使用更长的超时）
        success = self.deformation_model.learn_deformation(svd_transformed, residual_vectors, timeout_seconds=600)
        
        if not success:
            logger.error("Failed to train final GPR deformation model")
            return None, None
        
        # 应用最终GPR模型（包含不确定性预测）
        predicted_deformation, prediction_uncertainty = self.deformation_model.predict_deformation(
            svd_transformed, return_uncertainty=True
        )
        final_transformed = svd_transformed + predicted_deformation
        final_transformed[:, 1] = 0
        
        # 分析预测不确定性
        self.uncertainty_stats = self.deformation_model.analyze_uncertainty(svd_transformed)
        
        # 创建不确定性地图
        self.deformation_model.visualize_uncertainty_map()
        
        # 评估最终结果
        final_rmse_results = self.calculate_directional_rmse(final_clean_B, final_transformed)
        final_performance = self.evaluate_performance(final_clean_B, final_transformed)
        
        logger.info(f"Final Overall RMSE (Optimized GPR enhanced): {final_rmse_results['overall_rmse']*1000:.3f}mm")
        
        final_residuals = final_clean_B - final_transformed
        final_residual_magnitudes = np.sqrt(np.sum(final_residuals**2, axis=1))
        improvement = (np.mean(residual_magnitudes) - np.mean(final_residual_magnitudes))*1000
        logger.info(f"Final residual statistics (Optimized GPR enhanced):")
        logger.info(f"  Mean magnitude: {np.mean(final_residual_magnitudes)*1000:.3f}mm")
        logger.info(f"  Max magnitude: {np.max(final_residual_magnitudes)*1000:.3f}mm")
        logger.info(f"  Improvement: {improvement:.3f}mm")
        
        # 检查交叉验证预测与最终结果的一致性
        cv_score = self.optimizer.best_score*1000
        final_score = final_rmse_results['overall_rmse']*1000
        logger.info(f"Optimized GPR: CV predicted RMSE: {cv_score:.3f}mm")
        logger.info(f"Optimized GPR: Final actual RMSE: {final_score:.3f}mm")
        logger.info(f"Optimized GPR: Consistency (CV vs Final): {abs(cv_score - final_score):.3f}mm difference")
        
        # 不确定性分析报告
        if self.uncertainty_stats:
            logger.info("Optimized GPR Uncertainty Analysis Summary:")
            logger.info(f"  Mean prediction uncertainty: {self.uncertainty_stats['mean_uncertainty']*1000:.3f}mm")
            logger.info(f"  Max prediction uncertainty: {self.uncertainty_stats['max_uncertainty']*1000:.3f}mm")
            logger.info(f"  High uncertainty regions: {self.uncertainty_stats['high_uncertainty_ratio']*100:.1f}%")
        
        # 可视化最终结果（包含不确定性）
        self.visualize_transformation_stage("Final Optimized GPR Enhanced", 
                                           final_clean_A, final_clean_B, final_transformed, 
                                           final_rmse_results, final_residuals, prediction_uncertainty)
        
        # 残差分析
        self.residual_analyzer.analyze_final_residuals(
            final_clean_A, final_clean_B, final_transformed, 
            "Final Optimized GPR Enhanced"
        )
        
        # 打印详细结果
        self._print_detailed_results(svd_performance, final_performance, best_gpr_params)
        
        # 保存数据
        self._save_transformation_data()
        
        return self.T_svd, final_rmse_results
    
    def _print_detailed_results(self, svd_performance, final_performance, best_gpr_params):
        """Print detailed performance analysis"""
        logger.info("\n=== Detailed Optimized GPR Enhanced Performance Analysis ===")
        
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
        
        logger.info("Final (SVD + Optimized GPR Enhanced) Results:")
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
        
        logger.info("Best Optimized GPR Model:")
        logger.info(f"  Kernel type: {best_gpr_params['kernel_type']}")
        logger.info(f"  Length scale: {best_gpr_params['length_scale']}")
        logger.info(f"  Noise level: {best_gpr_params['noise_level']}")
        logger.info(f"  Alpha: {best_gpr_params['alpha']}")
        logger.info(f"  Normalize Y: {best_gpr_params['normalize_y']}")
        
        # 离群点统计
        logger.info("Outlier Detection Summary:")
        logger.info(f"  Method: {self.outlier_detector.method}")
        logger.info(f"  Outliers removed: {self.outlier_detector.outlier_stats['outliers_detected']}")
        logger.info(f"  Outlier ratio: {self.outlier_detector.outlier_stats['outlier_ratio']*100:.2f}%")
        
        # 不确定性统计
        if self.uncertainty_stats:
            logger.info("Optimized GPR Uncertainty Statistics:")
            logger.info(f"  Mean uncertainty: {self.uncertainty_stats['mean_uncertainty']*1000:.3f}mm")
            logger.info(f"  Max uncertainty: {self.uncertainty_stats['max_uncertainty']*1000:.3f}mm")
            logger.info(f"  High uncertainty ratio: {self.uncertainty_stats['high_uncertainty_ratio']*100:.1f}%")
            logger.info(f"  Uncertainty threshold: {self.uncertainty_stats['uncertainty_threshold']*1000:.3f}mm")
        
        # 优化特性汇总
        logger.info("Optimized GPR Enhanced Features Summary:")
        logger.info("  ✓ Improved numerical stability and performance")
        logger.info("  ✓ Timeout protection for training")
        logger.info("  ✓ Reduced parameter grid for faster optimization")
        logger.info("  ✓ Data subsampling for large datasets")
        logger.info("  ✓ Error handling and graceful degradation")
        logger.info("  ✓ Uncertainty quantification and mapping")

    def _save_transformation_data(self):
        """Save transformation matrices and optimized GPR model data"""
        if self.T_svd is not None:
            np.savetxt(os.path.join(result_dir, "T_svd_optimized_gpr_enhanced.txt"), self.T_svd)
        
        # 保存优化GPR模型信息
        model_info_file = os.path.join(result_dir, "optimized_gpr_enhanced_model_info.txt")
        with open(model_info_file, "w") as f:
            f.write("Optimized GPR Enhanced Model Information\n")
            f.write("=" * 40 + "\n")
            f.write(f"Model Type: {self.best_model_type}\n")
            if hasattr(self.deformation_model, 'kernel_type'):
                f.write(f"Kernel Type: {self.deformation_model.kernel_type}\n")
                f.write(f"Length Scale: {self.deformation_model.length_scale}\n")
                f.write(f"Noise Level: {self.deformation_model.noise_level}\n")
                f.write(f"Alpha: {self.deformation_model.alpha}\n")
                f.write(f"Normalize Y: {self.deformation_model.normalize_y}\n")
                f.write(f"N Restarts Optimizer: {self.deformation_model.n_restarts_optimizer}\n")
                f.write(f"Max Iterations: {self.deformation_model.max_iter}\n")
                f.write(f"Optimizer: {self.deformation_model.optimizer}\n")
                
                if hasattr(self.deformation_model, 'gpr_x') and self.deformation_model.gpr_x is not None:
                    f.write(f"Trained X Kernel: {self.deformation_model.gpr_x.kernel_}\n")
                    try:
                        f.write(f"X Log Marginal Likelihood: {self.deformation_model.gpr_x.log_marginal_likelihood():.6f}\n")
                    except:
                        f.write("X Log Marginal Likelihood: Could not compute\n")
                
                if hasattr(self.deformation_model, 'gpr_z') and self.deformation_model.gpr_z is not None:
                    f.write(f"Trained Z Kernel: {self.deformation_model.gpr_z.kernel_}\n")
                    try:
                        f.write(f"Z Log Marginal Likelihood: {self.deformation_model.gpr_z.log_marginal_likelihood():.6f}\n")
                    except:
                        f.write("Z Log Marginal Likelihood: Could not compute\n")
            
            f.write("\nOptimized GPR Enhanced Features:\n")
            f.write("- Improved numerical stability and performance\n")
            f.write("- Timeout protection for training\n")
            f.write("- Reduced parameter grid for faster optimization\n")
            f.write("- Data subsampling for large datasets\n")
            f.write("- Error handling and graceful degradation\n")
            f.write("- Uncertainty quantification and mapping\n")
        
        # 保存数据预处理信息
        preprocessing_info_file = os.path.join(result_dir, "preprocessing_info.txt")
        with open(preprocessing_info_file, "w") as f:
            f.write("Data Preprocessing Summary\n")
            f.write("=" * 40 + "\n")
            for key, value in self.preprocessing_info.items():
                if 'distance' in key:
                    f.write(f"{key}: {value*1000:.3f}mm\n")
                else:
                    f.write(f"{key}: {value}\n")
        
        # 保存离群点检测信息
        outlier_info_file = os.path.join(result_dir, "optimized_gpr_enhanced_outlier_detection_info.txt")
        with open(outlier_info_file, "w") as f:
            f.write("Optimized GPR Enhanced Outlier Detection Summary\n")
            f.write("=" * 40 + "\n")
            f.write(f"Method: {self.outlier_detector.method}\n")
            f.write(f"Total points: {self.outlier_detector.outlier_stats['total_points']}\n")
            f.write(f"Outliers detected: {self.outlier_detector.outlier_stats['outliers_detected']}\n")
            f.write(f"Outlier ratio: {self.outlier_detector.outlier_stats['outlier_ratio']*100:.2f}%\n")
            f.write(f"Threshold used: {self.outlier_detector.outlier_stats['threshold_used']*1000:.3f}mm\n")
            if len(self.outlier_detector.outlier_indices) > 0:
                f.write(f"Max outlier residual: {np.max(self.outlier_detector.outlier_stats['outlier_residuals'])*1000:.3f}mm\n")
        
        # 保存不确定性分析信息
        if self.uncertainty_stats:
            uncertainty_info_file = os.path.join(result_dir, "optimized_gpr_uncertainty_analysis.txt")
            with open(uncertainty_info_file, "w") as f:
                f.write("Optimized GPR Uncertainty Analysis Summary\n")
                f.write("=" * 40 + "\n")
                for key, value in self.uncertainty_stats.items():
                    if key == 'high_uncertainty_points':
                        f.write(f"{key}: {len(value)} points\n")
                    elif 'uncertainty' in key and isinstance(value, (int, float)):
                        f.write(f"{key}: {value*1000:.3f}mm\n")
                    else:
                        f.write(f"{key}: {value}\n")
        
        logger.info("Optimized GPR Enhanced transformation data and all analysis info saved")

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
    logger.info("Starting OPTIMIZED GPR ENHANCED 1:32 scale model tracker data processing")
    logger.info(f"Scale factor: {SCALE_FACTOR}")
    logger.info(f"Target RMSE: {TARGET_RMSE*1000:.2f}mm")
    logger.info("Optimized GPR Enhanced Features:")
    logger.info("  - Improved numerical stability and performance")
    logger.info("  - Timeout protection for training")
    logger.info("  - Reduced parameter grid for faster optimization")
    logger.info("  - Data subsampling for large datasets")
    logger.info("  - Error handling and graceful degradation")
    logger.info("  - Uncertainty quantification and mapping")
    
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
    
    # Perform optimized GPR enhanced two-stage registration
    transformer = CoordinateTransformer(positions_A, positions_B)
    T_svd, final_rmse_results = transformer.two_stage_registration_gpr_enhanced()
    
    if T_svd is not None and final_rmse_results is not None:
        success = final_rmse_results['overall_rmse'] <= TARGET_RMSE
        if success:
            logger.info(f"SUCCESS: Optimized GPR Enhanced registration completed with RMSE {final_rmse_results['overall_rmse']*1000:.3f}mm")
        else:
            logger.info(f"PARTIAL: Optimized GPR Enhanced registration completed, but RMSE {final_rmse_results['overall_rmse']*1000:.3f}mm exceeds target {TARGET_RMSE*1000:.2f}mm")
        
        # Save final summary
        summary_file = os.path.join(output_dir, "optimized_gpr_enhanced_registration_summary.txt")
        with open(summary_file, "w") as f:
            f.write("Optimized GPR Enhanced Two-Stage Registration Summary\n")
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
            f.write(f"Best model type: {transformer.best_model_type}\n")
            f.write(f"Outlier detection method: {transformer.outlier_detector.method}\n")
            
            if hasattr(transformer, 'uncertainty_stats') and transformer.uncertainty_stats:
                f.write(f"Mean prediction uncertainty: {transformer.uncertainty_stats['mean_uncertainty']*1000:.3f}mm\n")
                f.write(f"High uncertainty regions: {transformer.uncertainty_stats['high_uncertainty_ratio']*100:.1f}%\n")
            
            f.write("\nOptimized GPR Enhanced features:\n")
            f.write("- Improved numerical stability and performance\n")
            f.write("- Timeout protection for training\n")
            f.write("- Reduced parameter grid for faster optimization\n")
            f.write("- Data subsampling for large datasets\n")
            f.write("- Error handling and graceful degradation\n")
            f.write("- Uncertainty quantification and mapping\n")
    else:
        logger.error("Optimized GPR Enhanced registration failed")
    
    logger.info("Optimized GPR Enhanced processing complete")

if __name__ == "__main__":
    main()