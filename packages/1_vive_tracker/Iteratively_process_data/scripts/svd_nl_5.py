#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Tracker data processing script with SVD + Non-rigid Deformation Learning 
(Enhanced with robust parameter selection and advanced optimization strategies)
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
output_dir = os.path.join(result_dir, f"SVD_Deformation_Enhanced_{csv_filename}_{timestamp}_1to32_scale_RMSE_{TARGET_RMSE*1000:.1f}mm")
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

class RobustRBFModel:
    """
    稳健的RBF模型，避免数值不稳定
    """
    def __init__(self, method='thin_plate_spline', smoothing=0.001, epsilon=None, 
                 constrain_y=True, adaptive_smoothing=False, min_smoothing=1e-6):
        self.method = method
        self.smoothing = smoothing
        self.epsilon = epsilon  # 新增epsilon参数
        self.constrain_y = constrain_y
        self.adaptive_smoothing = adaptive_smoothing
        self.min_smoothing = min_smoothing
        self.rbf_x = None
        self.rbf_z = None
        self.source_points = None
        self.actual_smoothing = smoothing
        
    def learn_deformation(self, source_points, residual_vectors):
        """学习RBF形变模型（稳健版本）"""
        source_points = np.array(source_points)
        residual_vectors = np.array(residual_vectors)
        
        if self.constrain_y:
            source_xz = source_points[:, [0, 2]]
            residual_xz = residual_vectors[:, [0, 2]]
            
            # 修复：直接使用交叉验证找到的最佳平滑参数，禁用自适应平滑
            self.actual_smoothing = max(self.smoothing, self.min_smoothing)
            
            # 准备RBF参数
            rbf_kwargs = {
                'kernel': self.method,
                'smoothing': self.actual_smoothing
            }
            
            # 修复：为需要epsilon的核函数添加epsilon参数
            if self.method in ['multiquadric', 'inverse_multiquadric', 'gaussian'] and self.epsilon is not None:
                rbf_kwargs['epsilon'] = self.epsilon
            
            try:
                # 学习X方向的形变
                self.rbf_x = RBFInterpolator(
                    source_xz, 
                    residual_xz[:, 0], 
                    **rbf_kwargs
                )
                
                # 学习Z方向的形变
                self.rbf_z = RBFInterpolator(
                    source_xz, 
                    residual_xz[:, 1], 
                    **rbf_kwargs
                )
                
                self.source_points = source_xz
                logger.info(f"Robust RBF model learned: {self.method}, smoothing={self.actual_smoothing:.6f}")
                if self.epsilon is not None:
                    logger.info(f"  epsilon={self.epsilon}")
                return True
                
            except Exception as e:
                logger.error(f"Failed to learn robust RBF model: {e}")
                return False
        else:
            logger.warning("3D RBF deformation not implemented with Y-axis constraint")
            return False
    
    def predict_deformation(self, query_points):
        """预测形变"""
        if self.rbf_x is None or self.rbf_z is None:
            logger.error("RBF model not trained yet")
            return np.zeros_like(query_points)
        
        query_points = np.array(query_points)
        
        if self.constrain_y:
            query_xz = query_points[:, [0, 2]]
            
            try:
                deformation_x = self.rbf_x(query_xz)
                deformation_z = self.rbf_z(query_xz)
                
                deformation = np.zeros_like(query_points)
                deformation[:, 0] = deformation_x
                deformation[:, 2] = deformation_z
                
                return deformation
                
            except Exception as e:
                logger.error(f"Failed to predict RBF deformation: {e}")
                return np.zeros_like(query_points)
        else:
            return np.zeros_like(query_points)

class BSplineDeformationModel:
    """
    B-Spline based non-rigid deformation model using LSQBivariateSpline (Fixed)
    """
    def __init__(self, knot_density=10, smoothing_factor=0.1, constrain_y=True):
        self.knot_density = knot_density
        self.smoothing_factor = smoothing_factor
        self.constrain_y = constrain_y
        self.spline_x = None
        self.spline_z = None
        self.bounds = None
        
    def learn_deformation(self, source_points, residual_vectors):
        """
        学习B样条形变模型 (修正了平滑参数传递)
        """
        source_points = np.array(source_points)
        residual_vectors = np.array(residual_vectors)
        
        if self.constrain_y:
            source_xz = source_points[:, [0, 2]]
            residual_xz = residual_vectors[:, [0, 2]]
            
            # 计算边界
            x_min, x_max = np.min(source_xz[:, 0]), np.max(source_xz[:, 0])
            z_min, z_max = np.min(source_xz[:, 1]), np.max(source_xz[:, 1])
            self.bounds = [x_min, x_max, z_min, z_max]
            
            # 生成节点
            x_knots = np.linspace(x_min, x_max, self.knot_density)[1:-1]  # 内部节点
            z_knots = np.linspace(z_min, z_max, self.knot_density)[1:-1]
            
            try:
                # 关键修正：正确计算平滑参数值
                m = len(source_points)
                smoothing_value = m * self.smoothing_factor
                
                # 学习X方向形变 - 修正：直接传递平滑参数值
                self.spline_x = LSQBivariateSpline(
                    source_xz[:, 0], source_xz[:, 1], residual_xz[:, 0],
                    x_knots, z_knots, 
                    kx=3, ky=3,  # 3次样条
                    s=smoothing_value  # 修正：直接传递计算后的值，而不是s=s
                )
                
                # 学习Z方向形变
                self.spline_z = LSQBivariateSpline(
                    source_xz[:, 0], source_xz[:, 1], residual_xz[:, 1],
                    x_knots, z_knots,
                    kx=3, ky=3,
                    s=smoothing_value  # 修正：直接传递计算后的值
                )
                
                logger.info(f"B-Spline deformation model learned: {self.knot_density}x{self.knot_density} grid, s={smoothing_value:.2f}")
                return True
                
            except Exception as e:
                logger.error(f"Failed to learn B-Spline model: {e}")
                return False
        else:
            logger.warning("3D B-Spline deformation not implemented with Y-axis constraint")
            return False
    
    def predict_deformation(self, query_points):
        """预测形变"""
        if self.spline_x is None or self.spline_z is None:
            logger.error("B-Spline model not trained yet")
            return np.zeros_like(query_points)
        
        query_points = np.array(query_points)
        
        if self.constrain_y:
            query_xz = query_points[:, [0, 2]]
            
            try:
                # 确保查询点在边界内
                x_min, x_max, z_min, z_max = self.bounds
                query_xz[:, 0] = np.clip(query_xz[:, 0], x_min, x_max)
                query_xz[:, 1] = np.clip(query_xz[:, 1], z_min, z_max)
                
                deformation_x = self.spline_x.ev(query_xz[:, 0], query_xz[:, 1])
                deformation_z = self.spline_z.ev(query_xz[:, 0], query_xz[:, 1])
                
                deformation = np.zeros_like(query_points)
                deformation[:, 0] = deformation_x
                deformation[:, 2] = deformation_z
                
                return deformation
                
            except Exception as e:
                logger.error(f"Failed to predict B-Spline deformation: {e}")
                return np.zeros_like(query_points)
        else:
            return np.zeros_like(query_points)

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
    超参数优化器，使用交叉验证（修复后的稳健版本）
    """
    def __init__(self, cv_folds=5, random_state=42):
        self.cv_folds = cv_folds
        self.random_state = random_state
        self.best_params = None
        self.best_score = float('inf')
        self.cv_results = []
        
    def optimize_rbf_params(self, positions_A, positions_B, T_svd):
        """优化RBF模型参数（修复epsilon问题和参数组合逻辑）"""
        logger.info("Starting robust RBF hyperparameter optimization with cross-validation...")
        
        # 修复：为不同核函数定义不同的参数网格
        kernels_no_epsilon = ['thin_plate_spline']
        kernels_with_epsilon = ['multiquadric', 'inverse_multiquadric', 'gaussian']
        smoothing_values = [1e-6, 1e-5, 1e-4, 1e-3, 1e-2, 1e-1, 1.0]
        epsilon_values = [0.1, 1.0, 10.0]
        
        # 生成参数组合
        param_combinations = []
        
        # 不需要epsilon的核函数
        for kernel in kernels_no_epsilon:
            for smoothing in smoothing_values:
                param_combinations.append({
                    'kernel': kernel,
                    'smoothing': smoothing,
                    'epsilon': None
                })
        
        # 需要epsilon的核函数
        for kernel in kernels_with_epsilon:
            for smoothing in smoothing_values:
                for epsilon in epsilon_values:
                    param_combinations.append({
                        'kernel': kernel,
                        'smoothing': smoothing,
                        'epsilon': epsilon
                    })
        
        logger.info(f"Testing {len(param_combinations)} parameter combinations with {self.cv_folds}-fold CV")
        logger.info("Fixed: Added epsilon parameter for kernels that require it")
        
        # 设置交叉验证
        kf = KFold(n_splits=self.cv_folds, shuffle=True, random_state=self.random_state)
        
        positions_A = np.array(positions_A)
        positions_B = np.array(positions_B)
        indices = np.arange(len(positions_A))
        
        best_params = None
        best_mean_rmse = float('inf')
        
        for params in param_combinations:
            fold_rmses = []
            
            try:
                for fold, (train_idx, test_idx) in enumerate(kf.split(indices)):
                    # 分割训练和测试数据
                    train_A = positions_A[train_idx]
                    train_B = positions_B[train_idx]
                    test_A = positions_A[test_idx]
                    test_B = positions_B[test_idx]
                    
                    # 在训练集上执行SVD变换
                    svd_train_transformed = self._apply_transformation(train_A, T_svd)
                    residual_vectors = train_B - svd_train_transformed
                    
                    # 训练稳健形变模型 - 修复：传递epsilon参数
                    model = RobustRBFModel(
                        method=params['kernel'], 
                        smoothing=params['smoothing'],
                        epsilon=params['epsilon'],  # 修复：传递epsilon参数
                        constrain_y=True, 
                        adaptive_smoothing=False
                    )
                    success = model.learn_deformation(svd_train_transformed, residual_vectors)
                    
                    if not success:
                        fold_rmses.append(float('inf'))
                        continue
                    
                    # 在测试集上评估
                    svd_test_transformed = self._apply_transformation(test_A, T_svd)
                    predicted_deformation = model.predict_deformation(svd_test_transformed)
                    final_test_transformed = svd_test_transformed + predicted_deformation
                    final_test_transformed[:, 1] = 0  # 确保Y=0
                    
                    # 计算测试集RMSE
                    test_rmse = self._calculate_rmse(test_B, final_test_transformed)
                    fold_rmses.append(test_rmse)
                
                # 计算平均RMSE
                if len(fold_rmses) > 0 and not any(np.isinf(fold_rmses)):
                    mean_rmse = np.mean(fold_rmses)
                    std_rmse = np.std(fold_rmses)
                    
                    self.cv_results.append({
                        'kernel': params['kernel'],
                        'smoothing': params['smoothing'],
                        'epsilon': params['epsilon'],
                        'mean_rmse': mean_rmse,
                        'std_rmse': std_rmse,
                        'fold_rmses': fold_rmses
                    })
                    
                    # 构建参数显示字符串
                    param_str = f"{params['kernel']}, smoothing={params['smoothing']:.6f}"
                    if params['epsilon'] is not None:
                        param_str += f", epsilon={params['epsilon']}"
                    
                    logger.info(f"  {param_str}: RMSE={mean_rmse*1000:.3f}±{std_rmse*1000:.3f}mm")
                    
                    if mean_rmse < best_mean_rmse:
                        best_mean_rmse = mean_rmse
                        best_params = params.copy()
                
            except Exception as e:
                param_str = f"{params['kernel']}, smoothing={params['smoothing']:.6f}"
                if params['epsilon'] is not None:
                    param_str += f", epsilon={params['epsilon']}"
                logger.warning(f"Failed to evaluate {param_str}: {e}")
                continue
        
        self.best_params = best_params
        self.best_score = best_mean_rmse
        
        if best_params:
            logger.info(f"Best RBF parameters: {best_params}")
            logger.info(f"Best CV RMSE: {best_mean_rmse*1000:.3f}mm")
        else:
            logger.error("No valid parameter combination found")
        
        return best_params
    
    def optimize_bspline_params(self, positions_A, positions_B, T_svd):
        """优化B样条模型参数"""
        logger.info("Starting B-Spline hyperparameter optimization with cross-validation...")
        
        # 定义参数网格
        param_grid = {
            'knot_density': [5, 8, 10, 12, 15],
            'smoothing_factor': [0.01, 0.1, 0.5, 1.0, 2.0]  # 增加更多平滑选项
        }
        
        param_combinations = list(itertools.product(param_grid['knot_density'], param_grid['smoothing_factor']))
        
        logger.info(f"Testing {len(param_combinations)} B-Spline parameter combinations")
        
        kf = KFold(n_splits=self.cv_folds, shuffle=True, random_state=self.random_state)
        
        positions_A = np.array(positions_A)
        positions_B = np.array(positions_B)
        indices = np.arange(len(positions_A))
        
        best_params = None
        best_mean_rmse = float('inf')
        bspline_results = []
        
        for knot_density, smoothing_factor in param_combinations:
            fold_rmses = []
            
            try:
                for fold, (train_idx, test_idx) in enumerate(kf.split(indices)):
                    train_A = positions_A[train_idx]
                    train_B = positions_B[train_idx]
                    test_A = positions_A[test_idx]
                    test_B = positions_B[test_idx]
                    
                    svd_train_transformed = self._apply_transformation(train_A, T_svd)
                    residual_vectors = train_B - svd_train_transformed
                    
                    model = BSplineDeformationModel(
                        knot_density=knot_density, 
                        smoothing_factor=smoothing_factor, 
                        constrain_y=True
                    )
                    success = model.learn_deformation(svd_train_transformed, residual_vectors)
                    
                    if not success:
                        fold_rmses.append(float('inf'))
                        continue
                    
                    svd_test_transformed = self._apply_transformation(test_A, T_svd)
                    predicted_deformation = model.predict_deformation(svd_test_transformed)
                    final_test_transformed = svd_test_transformed + predicted_deformation
                    final_test_transformed[:, 1] = 0
                    
                    test_rmse = self._calculate_rmse(test_B, final_test_transformed)
                    fold_rmses.append(test_rmse)
                
                if len(fold_rmses) > 0 and not any(np.isinf(fold_rmses)):
                    mean_rmse = np.mean(fold_rmses)
                    std_rmse = np.std(fold_rmses)
                    
                    bspline_results.append({
                        'knot_density': knot_density,
                        'smoothing_factor': smoothing_factor,
                        'mean_rmse': mean_rmse,
                        'std_rmse': std_rmse,
                        'fold_rmses': fold_rmses
                    })
                    
                    logger.info(f"  knots={knot_density}, smoothing={smoothing_factor}: "
                              f"RMSE={mean_rmse*1000:.3f}±{std_rmse*1000:.3f}mm")
                    
                    if mean_rmse < best_mean_rmse:
                        best_mean_rmse = mean_rmse
                        best_params = {'knot_density': knot_density, 'smoothing_factor': smoothing_factor}
                
            except Exception as e:
                logger.warning(f"Failed to evaluate B-Spline knots={knot_density}, smoothing={smoothing_factor}: {e}")
                continue
        
        if best_params:
            logger.info(f"Best B-Spline parameters: {best_params}")
            logger.info(f"Best B-Spline CV RMSE: {best_mean_rmse*1000:.3f}mm")
        
        return best_params, bspline_results
    
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
            cv_df.to_csv(os.path.join(result_dir, "cv_results_rbf.csv"), index=False)
            
            # 可视化CV结果
            self._plot_cv_results()
    
    def _plot_cv_results(self):
        """可视化交叉验证结果"""
        if not self.cv_results:
            return
        
        df = pd.DataFrame(self.cv_results)
        
        # 按kernel分组绘制
        fig, axes = plt.subplots(2, 2, figsize=(16, 12))
        axes = axes.ravel()
        
        kernels = df['kernel'].unique()
        
        for i, kernel in enumerate(kernels):
            if i >= 4:
                break
                
            kernel_df = df[df['kernel'] == kernel]
            
            ax = axes[i]
            
            # 根据是否有epsilon参数分组
            if kernel in ['multiquadric', 'inverse_multiquadric', 'gaussian']:
                # 有epsilon的核函数 - 按epsilon值分组
                epsilons = kernel_df['epsilon'].unique()
                for eps in epsilons:
                    eps_df = kernel_df[kernel_df['epsilon'] == eps]
                    ax.errorbar(eps_df['smoothing'], eps_df['mean_rmse']*1000, 
                               yerr=eps_df['std_rmse']*1000, 
                               marker='o', capsize=5, capthick=2, label=f'ε={eps}')
            else:
                # 没有epsilon的核函数
                ax.errorbar(kernel_df['smoothing'], kernel_df['mean_rmse']*1000, 
                           yerr=kernel_df['std_rmse']*1000, 
                           marker='o', capsize=5, capthick=2)
            
            ax.set_xscale('log')
            ax.set_xlabel('Smoothing Parameter')
            ax.set_ylabel('RMSE [mm]')
            ax.set_title(f'CV Results: {kernel}')
            ax.grid(True, alpha=0.3)
            ax.axhline(y=TARGET_RMSE*1000, color='r', linestyle='--', alpha=0.7, label='Target')
            ax.legend()
        
        plt.tight_layout()
        plt.savefig(os.path.join(img_dir, 'cv_results_comparison.png'), dpi=300, bbox_inches='tight')
        plt.close()

# 继续原有的CoordinateTransformer类，但使用修复后的模型
class CoordinateTransformer:
    """
    Enhanced two-stage coordinate transformation with robust optimization
    """
    def __init__(self, positions_A=None, positions_B=None):
        self.T_svd = None
        self.deformation_model = None
        self.best_model_type = None
        self.__positions_A = positions_A
        self.__positions_B = positions_B
        self.optimizer = HyperparameterOptimizer()
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

    def two_stage_registration_enhanced(self):
        """Perform enhanced two-stage registration with all optimizations"""
        if self.__positions_A is None or self.__positions_B is None:
            logger.error("Please provide position data first!")
            return None, None
        
        logger.info("=== Starting Enhanced Two-Stage Registration (FIXED) ===")
        logger.info("Fixed Features:")
        logger.info("  - B-Spline LSQBivariateSpline parameter passing")
        logger.info("  - RBF epsilon parameter handling for multiquadric/gaussian kernels")
        logger.info("  - Direct use of CV-optimized parameters (no adaptive override)")
        
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
        
        self.visualize_transformation_stage("SVD Coarse Registration (Enhanced Fixed)", 
                                           final_clean_A, final_clean_B, svd_transformed, 
                                           svd_rmse_results, residual_vectors)
        
        # Stage 3: Fixed hyperparameter optimization and model selection
        logger.info("Stage 3: FIXED Hyperparameter Optimization and Model Selection")
        
        # 修复后的优化RBF参数
        best_rbf_params = self.optimizer.optimize_rbf_params(final_clean_A, final_clean_B, self.T_svd)
        
        # 优化B样条参数
        best_bspline_params, bspline_results = self.optimizer.optimize_bspline_params(final_clean_A, final_clean_B, self.T_svd)
        
        # 保存CV结果
        self.optimizer.save_cv_results()
        
        # 选择最佳模型
        best_model_info = self._select_best_model(best_rbf_params, best_bspline_params, bspline_results)
        
        if best_model_info is None:
            logger.error("No suitable model found")
            return None, None
        
        # 使用最佳模型训练最终模型
        logger.info(f"Training final FIXED model: {best_model_info}")
        
        if best_model_info['type'] == 'rbf':
            # 修复：直接使用CV找到的最佳参数，不启用自适应平滑
            self.deformation_model = RobustRBFModel(
                method=best_model_info['params']['kernel'],
                smoothing=best_model_info['params']['smoothing'],
                epsilon=best_model_info['params'].get('epsilon'),
                constrain_y=True,
                adaptive_smoothing=False  # 修复：禁用自适应平滑，直接使用CV最优参数
            )
        else:  # bspline
            self.deformation_model = BSplineDeformationModel(
                knot_density=best_model_info['params']['knot_density'],
                smoothing_factor=best_model_info['params']['smoothing_factor'],
                constrain_y=True
            )
        
        self.best_model_type = best_model_info['type']
        
        # 在最终清洁数据上训练最终模型
        success = self.deformation_model.learn_deformation(svd_transformed, residual_vectors)
        
        if not success:
            logger.error("Failed to train final deformation model")
            return None, None
        
        # 应用最终模型
        predicted_deformation = self.deformation_model.predict_deformation(svd_transformed)
        final_transformed = svd_transformed + predicted_deformation
        final_transformed[:, 1] = 0
        
        # 评估最终结果
        final_rmse_results = self.calculate_directional_rmse(final_clean_B, final_transformed)
        final_performance = self.evaluate_performance(final_clean_B, final_transformed)
        
        logger.info(f"Final Overall RMSE (FIXED enhanced): {final_rmse_results['overall_rmse']*1000:.3f}mm")
        
        final_residuals = final_clean_B - final_transformed
        final_residual_magnitudes = np.sqrt(np.sum(final_residuals**2, axis=1))
        improvement = (np.mean(residual_magnitudes) - np.mean(final_residual_magnitudes))*1000
        logger.info(f"Final residual statistics (FIXED enhanced):")
        logger.info(f"  Mean magnitude: {np.mean(final_residual_magnitudes)*1000:.3f}mm")
        logger.info(f"  Max magnitude: {np.max(final_residual_magnitudes)*1000:.3f}mm")
        logger.info(f"  Improvement: {improvement:.3f}mm")
        
        # 检查交叉验证预测与最终结果的一致性
        cv_score = best_model_info['score']*1000
        final_score = final_rmse_results['overall_rmse']*1000
        logger.info(f"FIXED: CV predicted RMSE: {cv_score:.3f}mm")
        logger.info(f"FIXED: Final actual RMSE: {final_score:.3f}mm")
        logger.info(f"FIXED: Consistency (CV vs Final): {abs(cv_score - final_score):.3f}mm difference")
        
        # 可视化最终结果
        self.visualize_transformation_stage(f"Final FIXED Enhanced {best_model_info['type'].upper()}", 
                                           final_clean_A, final_clean_B, final_transformed, 
                                           final_rmse_results, final_residuals)
        
        # 残差分析
        self.residual_analyzer.analyze_final_residuals(
            final_clean_A, final_clean_B, final_transformed, 
            f"Final FIXED Enhanced {best_model_info['type'].upper()}"
        )
        
        # 打印详细结果
        self._print_detailed_results(svd_performance, final_performance, best_model_info)
        
        # 保存数据
        self._save_transformation_data()
        
        return self.T_svd, final_rmse_results
    
    def _select_best_model(self, best_rbf_params, best_bspline_params, bspline_results):
        """选择最佳模型"""
        candidates = []
        
        # RBF候选
        if best_rbf_params and self.optimizer.best_score < float('inf'):
            candidates.append({
                'type': 'rbf',
                'params': best_rbf_params,
                'score': self.optimizer.best_score
            })
        
        # B样条候选
        if best_bspline_params and bspline_results:
            best_bspline_score = min([r['mean_rmse'] for r in bspline_results if not np.isinf(r['mean_rmse'])])
            candidates.append({
                'type': 'bspline',
                'params': best_bspline_params,
                'score': best_bspline_score
            })
        
        if not candidates:
            return None
        
        # 选择得分最低的模型
        best_candidate = min(candidates, key=lambda x: x['score'])
        
        logger.info("Model comparison:")
        for candidate in candidates:
            logger.info(f"  {candidate['type'].upper()}: RMSE={candidate['score']*1000:.3f}mm")
        
        logger.info(f"Selected best model: {best_candidate['type'].upper()}")
        
        return best_candidate

    def _print_detailed_results(self, svd_performance, final_performance, best_model_info):
        """Print detailed performance analysis"""
        logger.info("\n=== Detailed FIXED Enhanced Performance Analysis ===")
        
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
        
        logger.info(f"Final (SVD + FIXED Enhanced {best_model_info['type'].upper()}) Results:")
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
        
        logger.info(f"Best Model: FIXED Enhanced {best_model_info['type'].upper()}")
        logger.info(f"Best Parameters: {best_model_info['params']}")
        
        # 离群点统计
        logger.info("Outlier Detection Summary:")
        logger.info(f"  Method: {self.outlier_detector.method}")
        logger.info(f"  Outliers removed: {self.outlier_detector.outlier_stats['outliers_detected']}")
        logger.info(f"  Outlier ratio: {self.outlier_detector.outlier_stats['outlier_ratio']*100:.2f}%")
        
        # 修复状态汇总
        logger.info("FIXED Issues Summary:")
        logger.info("  ✓ B-Spline LSQBivariateSpline parameter passing fixed")
        logger.info("  ✓ RBF epsilon parameter handling for multiquadric/gaussian kernels fixed") 
        logger.info("  ✓ Direct use of CV-optimized parameters (adaptive smoothing disabled)")

    def _save_transformation_data(self):
        """Save transformation matrices and model data"""
        if self.T_svd is not None:
            np.savetxt(os.path.join(result_dir, "T_svd_enhanced_fixed.txt"), self.T_svd)
        
        # 保存最佳模型信息
        model_info_file = os.path.join(result_dir, "enhanced_fixed_model_info.txt")
        with open(model_info_file, "w") as f:
            f.write(f"FIXED Enhanced Model Type: {self.best_model_type}\n")
            if hasattr(self.deformation_model, 'method'):
                f.write(f"RBF Kernel: {self.deformation_model.method}\n")
                f.write(f"RBF Smoothing: {self.deformation_model.smoothing}\n")
                f.write(f"RBF Actual Smoothing: {self.deformation_model.actual_smoothing}\n")
                f.write(f"RBF Epsilon: {self.deformation_model.epsilon}\n")
                f.write(f"RBF Adaptive Smoothing: {self.deformation_model.adaptive_smoothing}\n")
            elif hasattr(self.deformation_model, 'knot_density'):
                f.write(f"B-Spline Knot Density: {self.deformation_model.knot_density}\n")
                f.write(f"B-Spline Smoothing Factor: {self.deformation_model.smoothing_factor}\n")
            f.write("\nFixed Issues:\n")
            f.write("- B-Spline LSQBivariateSpline parameter passing\n")
            f.write("- RBF epsilon parameter for multiquadric/gaussian kernels\n") 
            f.write("- Direct use of CV-optimized parameters\n")
        
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
        outlier_info_file = os.path.join(result_dir, "enhanced_fixed_outlier_detection_info.txt")
        with open(outlier_info_file, "w") as f:
            f.write("Enhanced FIXED Outlier Detection Summary\n")
            f.write("=" * 40 + "\n")
            f.write(f"Method: {self.outlier_detector.method}\n")
            f.write(f"Total points: {self.outlier_detector.outlier_stats['total_points']}\n")
            f.write(f"Outliers detected: {self.outlier_detector.outlier_stats['outliers_detected']}\n")
            f.write(f"Outlier ratio: {self.outlier_detector.outlier_stats['outlier_ratio']*100:.2f}%\n")
            f.write(f"Threshold used: {self.outlier_detector.outlier_stats['threshold_used']*1000:.3f}mm\n")
            if len(self.outlier_detector.outlier_indices) > 0:
                f.write(f"Max outlier residual: {np.max(self.outlier_detector.outlier_stats['outlier_residuals'])*1000:.3f}mm\n")
        
        logger.info("FIXED Enhanced transformation data and all analysis info saved")

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
    logger.info("Starting FIXED ENHANCED 1:32 scale model tracker data processing")
    logger.info(f"Scale factor: {SCALE_FACTOR}")
    logger.info(f"Target RMSE: {TARGET_RMSE*1000:.2f}mm")
    logger.info("FIXED Enhanced Features:")
    logger.info("  - FIXED: B-Spline LSQBivariateSpline parameter passing")
    logger.info("  - FIXED: RBF epsilon parameter for multiquadric/gaussian kernels")
    logger.info("  - FIXED: Direct use of CV-optimized parameters (no adaptive override)")
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
    
    # Perform FIXED enhanced two-stage registration
    transformer = CoordinateTransformer(positions_A, positions_B)
    T_svd, final_rmse_results = transformer.two_stage_registration_enhanced()
    
    if T_svd is not None and final_rmse_results is not None:
        success = final_rmse_results['overall_rmse'] <= TARGET_RMSE
        if success:
            logger.info(f"SUCCESS: FIXED Enhanced registration completed with RMSE {final_rmse_results['overall_rmse']*1000:.3f}mm")
        else:
            logger.info(f"PARTIAL: FIXED Enhanced registration completed, but RMSE {final_rmse_results['overall_rmse']*1000:.3f}mm exceeds target {TARGET_RMSE*1000:.2f}mm")
        
        # Save final summary
        summary_file = os.path.join(output_dir, "fixed_enhanced_registration_summary.txt")
        with open(summary_file, "w") as f:
            f.write("FIXED Enhanced Two-Stage Registration Summary\n")
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
            f.write("\nFIXED Issues:\n")
            f.write("- B-Spline LSQBivariateSpline parameter passing (s=smoothing_value instead of s=s)\n")
            f.write("- RBF epsilon parameter for multiquadric/gaussian kernels properly handled\n")
            f.write("- Direct use of CV-optimized parameters (adaptive smoothing disabled)\n")
            f.write("\nEnhanced features:\n")
            f.write("- Data preprocessing with deduplication and quality check\n")
            f.write("- Robust statistical outlier detection\n")
            f.write("- 5-fold cross-validation with proper parameter handling\n")
            f.write("- Automatic model selection\n")
            f.write("- Comprehensive residual analysis\n")
    else:
        logger.error("FIXED Enhanced registration failed")
    
    logger.info("FIXED Enhanced processing complete")

if __name__ == "__main__":
    main()