#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Enhanced Tracker data processing script with SVD → RANSAC → Point Selection → GPR Pipeline
Four-Stage Registration: SVD (Coarse Alignment) → RANSAC (Inlier Selection) → Point Selection (Downsampling) → GPR (Fine Registration)
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
from sklearn.gaussian_process.kernels import RBF, WhiteKernel, Matern, RationalQuadratic
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
LASER_TRACKER_CSV = os.path.join(parent_dir, "data/corrected_tracker_data_0703_162626.csv")
csv_filename = os.path.basename(LASER_TRACKER_CSV)
output_dir = os.path.join(result_dir, f"SVD_RANSAC_GPR_{csv_filename}_{timestamp}_1to32_scale_RMSE_{TARGET_RMSE*1000:.1f}mm")
os.makedirs(output_dir, exist_ok=True)

# Create subdirectories
log_dir = os.path.join(output_dir, "log")
data_dir = os.path.join(output_dir, "data")
result_dir_final = os.path.join(output_dir, "results")
img_dir = os.path.join(output_dir, "img")

for directory in [log_dir, data_dir, result_dir_final, img_dir]:
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

class Stage1_SVDCoarseAlignment:
    """
    Stage 1: SVD粗配准
    计算全局的最佳刚性变换（旋转+平移），将源点云整体对齐到目标点云
    """
    def __init__(self):
        self.T_svd = None
        self.svd_stats = {}
        
    def perform_svd_alignment(self, positions_A, positions_B):
        """
        执行SVD粗配准
        
        Returns:
        - svd_transformed_A: 经过SVD变换后的源点云
        - residual_vectors: SVD之后的残差 (positions_B - svd_transformed_A)
        - T_svd: SVD变换矩阵
        """
        logger.info("=== Stage 1: SVD Coarse Alignment ===")
        
        positions_A = np.array(positions_A)
        positions_B = np.array(positions_B)
        
        logger.info(f"Input: {len(positions_A)} point pairs")
        
        # 只在XZ平面上进行SVD配准（Y轴始终为0）
        positions_A_xz = positions_A[:, [0, 2]]
        positions_B_xz = positions_B[:, [0, 2]]
        
        # 计算质心
        centroid_A_xz = np.mean(positions_A_xz, axis=0)
        centroid_B_xz = np.mean(positions_B_xz, axis=0)
        
        logger.info(f"Source centroid (XZ): [{centroid_A_xz[0]:.6f}, {centroid_A_xz[1]:.6f}]")
        logger.info(f"Target centroid (XZ): [{centroid_B_xz[0]:.6f}, {centroid_B_xz[1]:.6f}]")
        
        # 中心化
        centered_A_xz = positions_A_xz - centroid_A_xz
        centered_B_xz = positions_B_xz - centroid_B_xz
        
        # 计算协方差矩阵H
        H = np.dot(centered_A_xz.T, centered_B_xz)
        logger.info(f"Covariance matrix H:\n{H}")
        
        # SVD分解
        U, S, Vt = np.linalg.svd(H)
        logger.info(f"SVD singular values: {S}")
        
        # 计算旋转矩阵
        R_2d = np.dot(Vt.T, U.T)
        
        # 确保是右手坐标系（行列式为正）
        if np.linalg.det(R_2d) < 0:
            logger.info("Correcting reflection in rotation matrix")
            Vt[-1, :] *= -1
            R_2d = np.dot(Vt.T, U.T)
        
        # 计算平移向量
        translation_2d = centroid_B_xz.T - np.dot(R_2d, centroid_A_xz.T)
        
        # 构建4x4变换矩阵
        self.T_svd = np.eye(4)
        self.T_svd[0, 0] = R_2d[0, 0]  # R11
        self.T_svd[0, 2] = R_2d[0, 1]  # R13
        self.T_svd[2, 0] = R_2d[1, 0]  # R31
        self.T_svd[2, 2] = R_2d[1, 1]  # R33
        self.T_svd[0, 3] = translation_2d[0]  # tx
        self.T_svd[2, 3] = translation_2d[1]  # tz
        
        logger.info(f"SVD Transformation Matrix:\n{self.T_svd}")
        
        # 应用变换
        svd_transformed_A = self._apply_transformation(positions_A, self.T_svd)
        
        # 计算残差向量
        residual_vectors = positions_B - svd_transformed_A
        
        # 分析SVD结果
        self._analyze_svd_results(positions_A, positions_B, svd_transformed_A, residual_vectors)
        
        logger.info("Stage 1 (SVD) completed successfully")
        logger.info(f"Output: svd_transformed_A shape: {svd_transformed_A.shape}")
        logger.info(f"Output: residual_vectors shape: {residual_vectors.shape}")
        
        return svd_transformed_A, residual_vectors, self.T_svd
    
    def _apply_transformation(self, positions, T):
        """应用4x4变换矩阵"""
        positions = np.array(positions)
        positions_homo = np.hstack([positions, np.ones((positions.shape[0], 1))])
        transformed = np.dot(T, positions_homo.T).T
        result = transformed[:, :3]
        result[:, 1] = 0  # 确保Y=0
        return result
    
    def _analyze_svd_results(self, original_A, target_B, transformed_A, residuals):
        """分析SVD配准结果"""
        # 计算各种误差指标
        residual_magnitudes = np.sqrt(np.sum(residuals**2, axis=1))
        
        rmse_overall = np.sqrt(np.mean(residual_magnitudes**2))
        rmse_x = np.sqrt(np.mean(residuals[:, 0]**2))
        rmse_z = np.sqrt(np.mean(residuals[:, 2]**2))
        
        max_error = np.max(residual_magnitudes)
        mean_error = np.mean(residual_magnitudes)
        std_error = np.std(residual_magnitudes)
        
        # 保存统计信息
        self.svd_stats = {
            'rmse_overall': rmse_overall,
            'rmse_x': rmse_x,
            'rmse_z': rmse_z,
            'max_error': max_error,
            'mean_error': mean_error,
            'std_error': std_error,
            'num_points': len(original_A)
        }
        
        logger.info("SVD Alignment Results:")
        logger.info(f"  Overall RMSE: {rmse_overall*1000:.3f}mm")
        logger.info(f"  X-direction RMSE: {rmse_x*1000:.3f}mm")
        logger.info(f"  Z-direction RMSE: {rmse_z*1000:.3f}mm")
        logger.info(f"  Maximum error: {max_error*1000:.3f}mm")
        logger.info(f"  Mean error: {mean_error*1000:.3f}mm")
        logger.info(f"  Std deviation: {std_error*1000:.3f}mm")
        
        # 可视化SVD结果
        self._visualize_svd_results(original_A, target_B, transformed_A, residuals)
    
    def _visualize_svd_results(self, original_A, target_B, transformed_A, residuals):
        """可视化SVD配准结果"""
        fig, axes = plt.subplots(2, 3, figsize=(18, 12))
        
        # 原始配准结果
        ax1 = axes[0, 0]
        ax1.scatter(original_A[:, 0], original_A[:, 2], c='blue', alpha=0.6, s=30, label='Original Source')
        ax1.scatter(transformed_A[:, 0], transformed_A[:, 2], c='green', alpha=0.6, s=30, label='SVD Transformed')
        ax1.scatter(target_B[:, 0], target_B[:, 2], c='red', alpha=0.6, s=30, label='Target')
        ax1.set_xlabel('X [m]')
        ax1.set_ylabel('Z [m]')
        ax1.set_title('SVD Coarse Alignment Results')
        ax1.legend()
        ax1.grid(True, alpha=0.3)
        ax1.axis('equal')
        
        # 残差向量场
        ax2 = axes[0, 1]
        step = max(1, len(original_A) // 50)  # 抽样显示
        scale_factor = 20
        ax2.quiver(transformed_A[::step, 0], transformed_A[::step, 2], 
                  residuals[::step, 0]*scale_factor, residuals[::step, 2]*scale_factor,
                  angles='xy', scale_units='xy', scale=1, alpha=0.7, color='red')
        ax2.scatter(transformed_A[::step, 0], transformed_A[::step, 2], c='blue', alpha=0.5, s=20)
        ax2.set_xlabel('X [m]')
        ax2.set_ylabel('Z [m]')
        ax2.set_title(f'SVD Residual Vectors (×{scale_factor})')
        ax2.grid(True, alpha=0.3)
        ax2.axis('equal')
        
        # 残差大小分布
        ax3 = axes[0, 2]
        residual_magnitudes = np.sqrt(np.sum(residuals**2, axis=1))
        ax3.hist(residual_magnitudes*1000, bins=30, alpha=0.7, color='skyblue', edgecolor='black')
        ax3.axvline(x=self.svd_stats['rmse_overall']*1000, color='red', linestyle='--', 
                   label=f"RMSE: {self.svd_stats['rmse_overall']*1000:.1f}mm")
        ax3.set_xlabel('Residual Magnitude [mm]')
        ax3.set_ylabel('Frequency')
        ax3.set_title('SVD Residual Distribution')
        ax3.legend()
        ax3.grid(True, alpha=0.3)
        
        # X方向残差
        ax4 = axes[1, 0]
        ax4.scatter(transformed_A[:, 0], residuals[:, 0]*1000, alpha=0.6, s=20)
        ax4.set_xlabel('X Position [m]')
        ax4.set_ylabel('X Residual [mm]')
        ax4.set_title('X-Direction Residuals')
        ax4.grid(True, alpha=0.3)
        ax4.axhline(y=0, color='red', linestyle='--', alpha=0.7)
        
        # Z方向残差
        ax5 = axes[1, 1]
        ax5.scatter(transformed_A[:, 2], residuals[:, 2]*1000, alpha=0.6, s=20)
        ax5.set_xlabel('Z Position [m]')
        ax5.set_ylabel('Z Residual [mm]')
        ax5.set_title('Z-Direction Residuals')
        ax5.grid(True, alpha=0.3)
        ax5.axhline(y=0, color='red', linestyle='--', alpha=0.7)
        
        # 残差空间分布
        ax6 = axes[1, 2]
        scatter = ax6.scatter(transformed_A[:, 0], transformed_A[:, 2], c=residual_magnitudes*1000, 
                            cmap='viridis', s=30, alpha=0.7)
        plt.colorbar(scatter, ax=ax6, label='Residual Magnitude [mm]')
        ax6.set_xlabel('X [m]')
        ax6.set_ylabel('Z [m]')
        ax6.set_title('Spatial Distribution of SVD Residuals')
        ax6.grid(True, alpha=0.3)
        ax6.axis('equal')
        
        plt.tight_layout()
        plt.savefig(os.path.join(img_dir, 'stage1_svd_results.png'), dpi=300, bbox_inches='tight')
        plt.close()
        
        logger.info("SVD visualization saved: stage1_svd_results.png")

class Stage2_RANSACInlierSelection:
    """
    Stage 2: RANSAC内点筛选 (增强版)
    RANSAC充当"聪明的门卫"，为Stage 3筛选出可靠的训练数据
    
    核心改进：
    1. 动态调整距离阈值（基于SVD残差的统计特性）
    2. 增加样本量提高稳健性
    3. 使用轻量级RBF作为临时模型（可选）
    4. 增加早停机制和自适应策略
    """
    def __init__(self, max_iterations=500, sample_size=None, min_inliers_ratio=0.5, 
                 use_rbf_model=False, adaptive_threshold=True):
        self.max_iterations = max_iterations
        self.sample_size = sample_size  # 如果为None，将自适应计算
        self.min_inliers_ratio = min_inliers_ratio
        self.use_rbf_model = use_rbf_model  # 是否使用RBF作为临时模型
        self.adaptive_threshold = adaptive_threshold
        self.distance_threshold = None  # 将动态计算
        self.ransac_stats = {}
        
    def select_inliers(self, svd_transformed_A, residual_vectors):
        """
        使用RANSAC筛选内点
        
        Args:
        - svd_transformed_A: SVD变换后的源点云
        - residual_vectors: SVD残差向量
        
        Returns:
        - inlier_indices: 内点索引
        - best_model_params: 最佳简单模型参数
        """
        logger.info("=== Stage 2: RANSAC Inlier Selection (Enhanced) ===")
        
        svd_transformed_A = np.array(svd_transformed_A)
        residual_vectors = np.array(residual_vectors)
        
        # 只在XZ平面进行RANSAC
        source_xz = svd_transformed_A[:, [0, 2]]
        residual_xz = residual_vectors[:, [0, 2]]
        
        n_points = len(source_xz)
        
        # --- 核心修改 1: 智能计算距离阈值 ---
        if self.adaptive_threshold:
            self.distance_threshold = self._compute_adaptive_threshold(residual_xz)
        else:
            # 备用：使用固定阈值
            residual_magnitudes = np.linalg.norm(residual_xz, axis=1)
            self.distance_threshold = np.median(residual_magnitudes) * 2.5
        
        # --- 核心修改 2: 自适应样本量 ---
        if self.sample_size is None:
            self.sample_size = self._compute_adaptive_sample_size(n_points)
        else:
            self.sample_size = min(self.sample_size, n_points // 2)
        
        min_inliers_count = int(n_points * self.min_inliers_ratio)
        
        logger.info(f"Input: {n_points} points with residuals")
        logger.info(f"RANSAC Enhanced Parameters:")
        logger.info(f"  Max iterations: {self.max_iterations}")
        logger.info(f"  Sample size: {self.sample_size}")
        logger.info(f"  Distance threshold: {self.distance_threshold*1000:.2f}mm")
        logger.info(f"  Min inliers ratio: {self.min_inliers_ratio}")
        logger.info(f"  Model type: {'Lightweight RBF' if self.use_rbf_model else 'Polynomial'}")
        
        best_inlier_indices = np.arange(n_points)  # 默认所有点都是内点
        best_inlier_count = 0
        best_model = None
        best_score = -np.inf
        
        # 早停机制
        no_improvement_count = 0
        patience = 50  # 连续50次迭代无改进则提前停止
        
        logger.info("Starting Enhanced RANSAC iterations...")
        
        iteration = 0
        for iteration in range(self.max_iterations):
            try:
                # 随机选择样本点（避免聚集采样）
                sample_indices = self._smart_sampling(source_xz, self.sample_size)
                
                sample_source = source_xz[sample_indices]
                sample_residual = residual_xz[sample_indices]
                
                # 训练临时模型
                if self.use_rbf_model:
                    model_params = self._fit_lightweight_rbf_model(sample_source, sample_residual)
                else:
                    model_params = self._fit_enhanced_polynomial_model(sample_source, sample_residual)
                
                if model_params is None:
                    continue
                
                # 用模型预测所有点的残差
                if self.use_rbf_model:
                    predicted_residuals = self._predict_rbf_model(source_xz, model_params)
                else:
                    predicted_residuals = self._predict_polynomial_model(source_xz, model_params)
                
                # 计算预测误差
                prediction_errors = np.linalg.norm(residual_xz - predicted_residuals, axis=1)
                
                # 统计内点
                current_inlier_mask = prediction_errors < self.distance_threshold
                current_inlier_count = np.sum(current_inlier_mask)
                
                # 计算模型质量分数（结合内点数量和拟合质量）
                current_score = self._compute_model_score(current_inlier_count, prediction_errors, 
                                                        current_inlier_mask, n_points)
                
                # 评估当前模型
                if current_score > best_score and current_inlier_count >= min_inliers_count:
                    best_score = current_score
                    best_inlier_count = current_inlier_count
                    best_inlier_indices = np.where(current_inlier_mask)[0]
                    best_model = model_params.copy()
                    no_improvement_count = 0
                    
                    logger.info(f"  Iteration {iteration+1}: New best model - {current_inlier_count} inliers "
                              f"({current_inlier_count/n_points*100:.1f}%), score: {current_score:.3f}")
                    
                    # 优化：如果已经找到了一个很好的模型，可以提前退出
                    if current_inlier_count > n_points * 0.95:
                        logger.info("  Found a model with >95% inliers, stopping early.")
                        break
                else:
                    no_improvement_count += 1
                
                # 早停检查
                if no_improvement_count >= patience:
                    logger.info(f"  Early stopping at iteration {iteration+1} (no improvement for {patience} iterations)")
                    break
                
            except (np.linalg.LinAlgError, ValueError) as e:
                # 样本可能共线等问题，跳过本次迭代
                continue
        
        # 结果验证和后处理
        if best_inlier_count < min_inliers_count:
            logger.warning(f"RANSAC failed to find a consensus model with at least {min_inliers_count} inliers.")
            logger.warning(f"Best model found only had {best_inlier_count} inliers. Using fallback strategy.")
            # 使用更宽松的阈值重试
            best_inlier_indices = self._fallback_strategy(source_xz, residual_xz)
        else:
            logger.info(f"RANSAC successful. Final inlier set size: {best_inlier_count}")
        
        # 可选：对最终内点集进行精炼
        best_inlier_indices = self._refine_inlier_set(source_xz, residual_xz, best_inlier_indices)
        
        inlier_ratio = len(best_inlier_indices) / n_points
        
        # 保存RANSAC统计信息
        self.ransac_stats = {
            'total_points': n_points,
            'inlier_count': len(best_inlier_indices),
            'inlier_ratio': inlier_ratio,
            'outlier_count': n_points - len(best_inlier_indices),
            'iterations_used': iteration + 1,
            'distance_threshold': self.distance_threshold,
            'best_model': best_model,
            'best_score': best_score,
            'sample_size_used': self.sample_size,
            'early_stopping': no_improvement_count >= patience
        }
        
        logger.info("Enhanced RANSAC Results:")
        logger.info(f"  Total points: {n_points}")
        logger.info(f"  Inliers found: {len(best_inlier_indices)} ({inlier_ratio*100:.1f}%)")
        logger.info(f"  Outliers removed: {n_points - len(best_inlier_indices)} ({(1-inlier_ratio)*100:.1f}%)")
        logger.info(f"  Iterations used: {self.ransac_stats['iterations_used']}/{self.max_iterations}")
        logger.info(f"  Best model score: {best_score:.3f}")
        
        # 可视化RANSAC结果
        self._visualize_enhanced_ransac_results(svd_transformed_A, residual_vectors, best_inlier_indices)
        
        logger.info("Stage 2 (Enhanced RANSAC) completed")
        
        return best_inlier_indices, best_model
    
    def _compute_adaptive_threshold(self, residual_xz):
        """智能计算距离阈值"""
        residual_magnitudes = np.linalg.norm(residual_xz, axis=1)
        
        # 使用多种统计量来估计噪声水平
        median_residual = np.median(residual_magnitudes)
        mad = np.median(np.abs(residual_magnitudes - median_residual))  # 中位数绝对偏差
        q75 = np.percentile(residual_magnitudes, 75)
        q25 = np.percentile(residual_magnitudes, 25)
        iqr = q75 - q25
        
        # 使用稳健的噪声估计
        # MAD是一个非常稳健的噪声估计量
        robust_noise_estimate = mad * 1.4826  # 1.4826是使MAD与正态分布标准差一致的因子
        
        # 选择阈值策略
        threshold_candidates = [
            median_residual * 2.5,  # 基于中位数
            robust_noise_estimate * 3.0,  # 基于MAD的3σ规则
            q75 + 1.5 * iqr,  # 基于IQR的离群点定义
        ]
        
        # 选择中等严格程度的阈值
        threshold = np.median(threshold_candidates)
        
        logger.info(f"Adaptive threshold computation:")
        logger.info(f"  Median residual: {median_residual*1000:.2f}mm")
        logger.info(f"  MAD-based noise estimate: {robust_noise_estimate*1000:.2f}mm")
        logger.info(f"  IQR: {iqr*1000:.2f}mm")
        logger.info(f"  Selected threshold: {threshold*1000:.2f}mm")
        
        return threshold
    
    def _compute_adaptive_sample_size(self, n_points):
        """自适应计算样本大小"""
        # 基于数据规模和预期内点比例计算合适的样本大小
        base_size = min(100, n_points // 20)  # 基础大小：不超过总数的5%
        
        # 根据数据规模调整
        if n_points < 1000:
            sample_size = min(50, n_points // 10)
        elif n_points < 5000:
            sample_size = base_size
        else:
            sample_size = min(150, n_points // 30)  # 大数据集使用更大样本
        
        # 确保样本大小合理
        sample_size = max(20, min(sample_size, n_points // 2))
        
        logger.info(f"Adaptive sample size: {sample_size} (for {n_points} points)")
        return sample_size
    
    def _smart_sampling(self, source_xz, sample_size):
        """智能采样：避免聚集，确保样本分布均匀"""
        n_points = len(source_xz)
        
        # 方法1：简单随机采样（快速）
        if n_points < 1000:
            return np.random.choice(n_points, sample_size, replace=False)
        
        # 方法2：分层采样（确保空间分布均匀）
        try:
            # 将空间划分为网格
            x_min, x_max = np.min(source_xz[:, 0]), np.max(source_xz[:, 0])
            z_min, z_max = np.min(source_xz[:, 1]), np.max(source_xz[:, 1])
            
            grid_size = int(np.sqrt(sample_size))  # 大约形成正方形网格
            grid_size = max(2, grid_size)  # 确保至少有2x2网格
            
            x_bins = np.linspace(x_min, x_max, grid_size + 1)
            z_bins = np.linspace(z_min, z_max, grid_size + 1)
            
            # 从每个网格单元中随机选择点
            selected_indices = []
            points_per_cell = max(1, sample_size // (grid_size * grid_size))
            
            for i in range(grid_size):
                for j in range(grid_size):
                    # 找到当前网格单元中的点
                    mask = ((source_xz[:, 0] >= x_bins[i]) & (source_xz[:, 0] < x_bins[i+1]) &
                           (source_xz[:, 1] >= z_bins[j]) & (source_xz[:, 1] < z_bins[j+1]))
                    
                    cell_indices = np.where(mask)[0]
                    
                    if len(cell_indices) > 0:
                        # 从当前单元随机选择
                        n_select = min(points_per_cell, len(cell_indices))
                        selected_indices.extend(
                            np.random.choice(cell_indices, n_select, replace=False)
                        )
            
            # 如果选择的点不够，补充随机点
            while len(selected_indices) < sample_size:
                remaining = sample_size - len(selected_indices)
                all_indices = set(range(n_points))
                available_indices = list(all_indices - set(selected_indices))
                
                if available_indices:
                    additional = np.random.choice(available_indices, 
                                                min(remaining, len(available_indices)), 
                                                replace=False)
                    selected_indices.extend(additional)
                else:
                    break
            
            return np.array(selected_indices[:sample_size])
            
        except Exception as e:
            # 如果分层采样失败，退回到简单随机采样
            return np.random.choice(n_points, sample_size, replace=False)
    
    def _fit_enhanced_polynomial_model(self, sample_source, sample_residual):
        """拟合增强的多项式模型"""
        try:
            x = sample_source[:, 0]
            z = sample_source[:, 1]
            
            # 使用二次多项式，但添加正则化
            # [1, x, z, x^2, z^2, xz]
            features = np.column_stack([
                np.ones(len(x)),
                x, z,
                x**2, z**2, x*z
            ])
            
            # 添加L2正则化（岭回归）
            alpha = 1e-6  # 正则化参数
            
            # X方向
            A_x = features.T @ features + alpha * np.eye(features.shape[1])
            b_x = features.T @ sample_residual[:, 0]
            coeffs_x = np.linalg.solve(A_x, b_x)
            
            # Z方向
            A_z = features.T @ features + alpha * np.eye(features.shape[1])
            b_z = features.T @ sample_residual[:, 1]
            coeffs_z = np.linalg.solve(A_z, b_z)
            
            return {
                'coeffs_x': coeffs_x,
                'coeffs_z': coeffs_z,
                'model_type': 'enhanced_polynomial'
            }
        except Exception as e:
            return None
    
    def _fit_lightweight_rbf_model(self, sample_source, sample_residual):
        """拟合轻量级RBF模型作为临时模型"""
        try:
            # 使用很少的RBF中心来保持计算效率
            n_centers = min(10, len(sample_source) // 2)
            n_centers = max(1, n_centers)  # 确保至少有1个中心
            
            # 选择RBF中心（k-means++ 初始化的简化版本）
            center_indices = [np.random.randint(len(sample_source))]
            
            for _ in range(n_centers - 1):
                distances = []
                for i in range(len(sample_source)):
                    min_dist = min([np.linalg.norm(sample_source[i] - sample_source[j]) 
                                  for j in center_indices])
                    distances.append(min_dist)
                
                # 选择距离现有中心最远的点
                center_indices.append(np.argmax(distances))
            
            rbf_centers = sample_source[center_indices]
            
            # 计算RBF特征矩阵
            if len(sample_source) > 1:
                sigma = np.median([np.linalg.norm(sample_source[i] - sample_source[j]) 
                                 for i in range(len(sample_source)) 
                                 for j in range(i+1, len(sample_source))]) / 2
            else:
                sigma = 1.0
            
            sigma = max(sigma, 1e-6)  # 防止sigma为0
            
            features = np.zeros((len(sample_source), n_centers))
            for i, center in enumerate(rbf_centers):
                distances = np.linalg.norm(sample_source - center, axis=1)
                features[:, i] = np.exp(-(distances**2) / (2 * sigma**2))
            
            # 拟合权重
            weights_x = np.linalg.lstsq(features, sample_residual[:, 0], rcond=None)[0]
            weights_z = np.linalg.lstsq(features, sample_residual[:, 1], rcond=None)[0]
            
            return {
                'rbf_centers': rbf_centers,
                'weights_x': weights_x,
                'weights_z': weights_z,
                'sigma': sigma,
                'model_type': 'lightweight_rbf'
            }
        except Exception as e:
            return None
    
    def _predict_polynomial_model(self, source_points, model_params):
        """使用多项式模型预测残差"""
        x = source_points[:, 0]
        z = source_points[:, 1]
        
        # 构建特征矩阵
        features = np.column_stack([
            np.ones(len(x)),
            x, z,
            x**2, z**2, x*z
        ])
        
        # 预测残差
        pred_x = np.dot(features, model_params['coeffs_x'])
        pred_z = np.dot(features, model_params['coeffs_z'])
        
        return np.column_stack([pred_x, pred_z])
    
    def _predict_rbf_model(self, source_points, model_params):
        """使用RBF模型预测残差"""
        rbf_centers = model_params['rbf_centers']
        weights_x = model_params['weights_x']
        weights_z = model_params['weights_z']
        sigma = model_params['sigma']
        
        # 计算RBF特征
        features = np.zeros((len(source_points), len(rbf_centers)))
        for i, center in enumerate(rbf_centers):
            distances = np.linalg.norm(source_points - center, axis=1)
            features[:, i] = np.exp(-(distances**2) / (2 * sigma**2))
        
        # 预测
        pred_x = np.dot(features, weights_x)
        pred_z = np.dot(features, weights_z)
        
        return np.column_stack([pred_x, pred_z])
    
    def _compute_model_score(self, inlier_count, prediction_errors, inlier_mask, total_points):
        """计算模型质量分数"""
        if inlier_count == 0:
            return -np.inf
        
        # 内点比例
        inlier_ratio = inlier_count / total_points
        
        # 内点的平均拟合误差
        inlier_errors = prediction_errors[inlier_mask]
        mean_inlier_error = np.mean(inlier_errors)
        
        # 综合分数：内点比例 - 惩罚拟合误差
        score = inlier_ratio - 0.1 * mean_inlier_error / self.distance_threshold
        
        return score
    
    def _fallback_strategy(self, source_xz, residual_xz):
        """备用策略：当RANSAC失败时的处理"""
        logger.info("Applying fallback strategy...")
        
        # 使用更宽松的阈值
        residual_magnitudes = np.linalg.norm(residual_xz, axis=1)
        relaxed_threshold = np.percentile(residual_magnitudes, 80)  # 保留80%的点
        
        inlier_mask = residual_magnitudes < relaxed_threshold
        fallback_indices = np.where(inlier_mask)[0]
        
        logger.info(f"Fallback strategy: using {len(fallback_indices)} points with relaxed threshold {relaxed_threshold*1000:.2f}mm")
        
        return fallback_indices
    
    def _refine_inlier_set(self, source_xz, residual_xz, initial_inliers):
        """精炼内点集：移除可能的噪声点"""
        if len(initial_inliers) < 10:  # 如果内点太少，不进行精炼
            return initial_inliers
        
        # 计算内点的残差统计
        inlier_residuals = residual_xz[initial_inliers]
        inlier_magnitudes = np.linalg.norm(inlier_residuals, axis=1)
        
        # 使用IQR方法进一步过滤
        q25 = np.percentile(inlier_magnitudes, 25)
        q75 = np.percentile(inlier_magnitudes, 75)
        iqr = q75 - q25
        
        refined_threshold = q75 + 1.5 * iqr
        refined_mask = inlier_magnitudes < refined_threshold
        
        refined_inliers = initial_inliers[refined_mask]
        
        removed_count = len(initial_inliers) - len(refined_inliers)
        if removed_count > 0:
            logger.info(f"Inlier refinement: removed {removed_count} additional outliers")
        
        return refined_inliers
    
    def _visualize_enhanced_ransac_results(self, svd_transformed_A, residual_vectors, inlier_indices):
        """可视化增强的RANSAC结果"""
        fig, axes = plt.subplots(2, 4, figsize=(24, 12))
        
        # 创建内点/外点掩码
        n_points = len(svd_transformed_A)
        inlier_mask = np.zeros(n_points, dtype=bool)
        inlier_mask[inlier_indices] = True
        outlier_mask = ~inlier_mask
        
        # 1. 内点外点分布
        ax1 = axes[0, 0]
        if np.sum(inlier_mask) > 0:
            ax1.scatter(svd_transformed_A[inlier_mask, 0], svd_transformed_A[inlier_mask, 2], 
                       c='green', alpha=0.7, s=25, label=f'Inliers ({np.sum(inlier_mask)})')
        if np.sum(outlier_mask) > 0:
            ax1.scatter(svd_transformed_A[outlier_mask, 0], svd_transformed_A[outlier_mask, 2], 
                       c='red', alpha=0.7, s=25, label=f'Outliers ({np.sum(outlier_mask)})')
        ax1.set_xlabel('X [m]')
        ax1.set_ylabel('Z [m]')
        ax1.set_title('Enhanced RANSAC Classification')
        ax1.legend()
        ax1.grid(True, alpha=0.3)
        ax1.axis('equal')
        
        # 2. 残差大小分布
        ax2 = axes[0, 1]
        residual_magnitudes = np.sqrt(np.sum(residual_vectors[:, [0, 2]]**2, axis=1))
        if np.sum(inlier_mask) > 0:
            ax2.hist(residual_magnitudes[inlier_mask]*1000, bins=25, alpha=0.7, 
                    color='green', label='Inliers', density=True)
        if np.sum(outlier_mask) > 0:
            ax2.hist(residual_magnitudes[outlier_mask]*1000, bins=25, alpha=0.7, 
                    color='red', label='Outliers', density=True)
        ax2.axvline(x=self.distance_threshold*1000, color='black', linestyle='--', 
                   label=f'Threshold: {self.distance_threshold*1000:.1f}mm')
        ax2.set_xlabel('Residual Magnitude [mm]')
        ax2.set_ylabel('Density')
        ax2.set_title('Residual Distribution with Adaptive Threshold')
        ax2.legend()
        ax2.grid(True, alpha=0.3)
        
        # 3. 阈值分析
        ax3 = axes[0, 2]
        residual_mags = residual_magnitudes * 1000
        percentiles = [25, 50, 75, 90, 95, 99]
        values = [np.percentile(residual_mags, p) for p in percentiles]
        
        bars = ax3.bar([f'P{p}' for p in percentiles], values, alpha=0.7, color='skyblue')
        ax3.axhline(y=self.distance_threshold*1000, color='red', linestyle='--', 
                   label=f'RANSAC Threshold')
        ax3.set_ylabel('Residual [mm]')
        ax3.set_title('Residual Percentiles vs Threshold')
        ax3.legend()
        ax3.grid(True, alpha=0.3)
        
        # 4. 内点比例饼图
        ax4 = axes[0, 3]
        sizes = [np.sum(inlier_mask), np.sum(outlier_mask)]
        labels = ['Inliers', 'Outliers']
        colors = ['green', 'red']
        if sizes[1] > 0:  # 只有当有离群点时才画饼图
            ax4.pie(sizes, labels=labels, colors=colors, autopct='%1.1f%%', startangle=90)
        else:
            ax4.pie([100], labels=['All Inliers'], colors=['green'], autopct='%1.1f%%', startangle=90)
        ax4.set_title(f'Classification Results\n(Threshold: {self.distance_threshold*1000:.1f}mm)')
        
        # 5. X方向残差分类
        ax5 = axes[1, 0]
        if np.sum(inlier_mask) > 0:
            ax5.scatter(svd_transformed_A[inlier_mask, 0], residual_vectors[inlier_mask, 0]*1000, 
                       c='green', alpha=0.6, s=15, label='Inliers')
        if np.sum(outlier_mask) > 0:
            ax5.scatter(svd_transformed_A[outlier_mask, 0], residual_vectors[outlier_mask, 0]*1000, 
                       c='red', alpha=0.6, s=15, label='Outliers')
        ax5.set_xlabel('X Position [m]')
        ax5.set_ylabel('X Residual [mm]')
        ax5.set_title('X-Direction Classification')
        ax5.legend()
        ax5.grid(True, alpha=0.3)
        ax5.axhline(y=0, color='black', linestyle='-', alpha=0.3)
        
        # 6. Z方向残差分类
        ax6 = axes[1, 1]
        if np.sum(inlier_mask) > 0:
            ax6.scatter(svd_transformed_A[inlier_mask, 2], residual_vectors[inlier_mask, 2]*1000, 
                       c='green', alpha=0.6, s=15, label='Inliers')
        if np.sum(outlier_mask) > 0:
            ax6.scatter(svd_transformed_A[outlier_mask, 2], residual_vectors[outlier_mask, 2]*1000, 
                       c='red', alpha=0.6, s=15, label='Outliers')
        ax6.set_xlabel('Z Position [m]')
        ax6.set_ylabel('Z Residual [mm]')
        ax6.set_title('Z-Direction Classification')
        ax6.legend()
        ax6.grid(True, alpha=0.3)
        ax6.axhline(y=0, color='black', linestyle='-', alpha=0.3)
        
        # 7. 统计信息
        ax7 = axes[1, 2]
        stats_text = f"""Enhanced RANSAC Statistics:

Total Points: {self.ransac_stats['total_points']}
Inliers: {self.ransac_stats['inlier_count']} ({self.ransac_stats['inlier_ratio']*100:.1f}%)
Outliers: {self.ransac_stats['outlier_count']} ({(1-self.ransac_stats['inlier_ratio'])*100:.1f}%)

Iterations Used: {self.ransac_stats['iterations_used']}/{self.max_iterations}
Sample Size: {self.ransac_stats['sample_size_used']}
Distance Threshold: {self.distance_threshold*1000:.2f}mm

Best Model Score: {self.ransac_stats.get('best_score', 'N/A'):.3f}
Early Stopping: {'Yes' if self.ransac_stats.get('early_stopping', False) else 'No'}

Model Type: {'RBF' if self.use_rbf_model else 'Polynomial'}"""
        
        ax7.text(0.05, 0.95, stats_text, transform=ax7.transAxes, fontsize=10,
                verticalalignment='top', fontfamily='monospace')
        ax7.set_xlim(0, 1)
        ax7.set_ylim(0, 1)
        ax7.set_title('Enhanced RANSAC Statistics')
        ax7.axis('off')
        
        # 8. 空间分布的分类结果
        ax8 = axes[1, 3]
        classification = np.zeros(n_points)
        classification[outlier_mask] = 1  # 外点标记为1
        
        scatter = ax8.scatter(svd_transformed_A[:, 0], svd_transformed_A[:, 2], 
                            c=classification, cmap='RdYlGn_r', s=25, alpha=0.7)
        ax8.set_xlabel('X [m]')
        ax8.set_ylabel('Z [m]')
        ax8.set_title('Spatial Classification Pattern')
        cbar = plt.colorbar(scatter, ax=ax8)
        cbar.set_label('0=Inlier, 1=Outlier')
        ax8.grid(True, alpha=0.3)
        ax8.axis('equal')
        
        plt.tight_layout()
        plt.savefig(os.path.join(img_dir, 'stage2_enhanced_ransac_results.png'), 
                   dpi=300, bbox_inches='tight')
        plt.close()
        
        logger.info("Enhanced RANSAC visualization saved: stage2_enhanced_ransac_results.png")

class GPRPointSelector:
    """
    智能自适应体素格下采样器
    
    这个类实现了一个动态调整格子大小的智能下采样算法：
    1. 根据目标点数和数据分布计算初始格子大小
    2. 通过迭代调整格子大小，直到得到接近目标数量的点
    3. 确保采样点在空间上分布均匀
    """
    
    def __init__(self, target_point_count=2000, tolerance_ratio=0.05, max_iterations=20):
        """
        初始化自适应体素格下采样器
        
        Args:
            target_point_count (int): 目标点数
            tolerance_ratio (float): 可接受的误差比例 (默认5%)
            max_iterations (int): 最大迭代次数
        """
        self.target_point_count = target_point_count
        self.tolerance_ratio = tolerance_ratio
        self.max_iterations = max_iterations
        
        # 计算可接受的点数范围
        self.min_acceptable = int(target_point_count * (1 - tolerance_ratio))
        self.max_acceptable = int(target_point_count * (1 + tolerance_ratio))
        
        # 统计信息
        self.selection_stats = {}
        
        logger.info(f"Adaptive Voxel Grid Downsampling initialized:")
        logger.info(f"  Target points: {target_point_count}")
        logger.info(f"  Acceptable range: [{self.min_acceptable}, {self.max_acceptable}]")
        logger.info(f"  Tolerance: {tolerance_ratio*100:.1f}%")
        logger.info(f"  Max iterations: {max_iterations}")
    
    def select_points(self, source_points, inlier_indices):
        """
        主要入口函数：从内点中智能选择训练点
        
        Args:
            source_points (np.array): SVD变换后的完整源点云
            inlier_indices (np.array): RANSAC筛选的内点索引
        
        Returns:
            np.array: 最终选择的点的索引（inlier_indices的子集）
        """
        logger.info("=== Stage 2.5: Adaptive Voxel Grid Point Selection ===")
        
        # 边界情况：内点数量已经小于等于目标
        if len(inlier_indices) <= self.target_point_count:
            logger.warning(f"Inlier count ({len(inlier_indices)}) <= target ({self.target_point_count})")
            logger.warning("Using all inliers for GPR training")
            return inlier_indices
        
        # 提取内点的XZ坐标
        inlier_points = source_points[inlier_indices]
        inlier_points_xz = inlier_points[:, [0, 2]]  # 只关心XZ平面
        
        logger.info(f"Starting adaptive downsampling: {len(inlier_indices)} -> ~{self.target_point_count} points")
        
        # 执行自适应体素格下采样
        selected_sub_indices = self._adaptive_voxel_grid_downsample(inlier_points_xz)
        
        # 将子索引映射回原始索引
        final_selected_indices = inlier_indices[selected_sub_indices]
        
        # 生成详细的选择报告
        self._generate_selection_report(inlier_points_xz, selected_sub_indices)
        
        # 可视化选择结果
        self._visualize_selection_results(inlier_points_xz, selected_sub_indices)
        
        logger.info(f"Adaptive point selection completed: {len(final_selected_indices)} points selected")
        logger.info(f"Selection efficiency: {len(final_selected_indices)/len(inlier_indices)*100:.1f}%")
        
        return final_selected_indices
    
    def _adaptive_voxel_grid_downsample(self, points_xz):
        """
        自适应体素格下采样的核心算法
        
        这个方法会迭代调整格子大小，直到得到理想数量的采样点
        """
        logger.info("Starting adaptive voxel grid downsampling iterations...")
        
        # 第1步：计算数据边界和初始格子大小
        x_min, z_min = points_xz.min(axis=0)
        x_max, z_max = points_xz.max(axis=0)
        total_area = (x_max - x_min) * (z_max - z_min)
        
        # 防止面积为0的边界情况
        if total_area <= 1e-12:
            logger.warning("Data area is too small, using random sampling")
            return np.random.choice(len(points_xz), 
                                  min(self.target_point_count, len(points_xz)), 
                                  replace=False)
        
        # 初始格子大小估算
        estimated_area_per_point = total_area / self.target_point_count
        initial_grid_size = np.sqrt(estimated_area_per_point)
        
        logger.info(f"Data boundary: X=[{x_min:.4f}, {x_max:.4f}], Z=[{z_min:.4f}, {z_max:.4f}]")
        logger.info(f"Total area: {total_area:.6f} m²")
        logger.info(f"Initial grid size estimate: {initial_grid_size:.6f} m")
        
        # 第2步：迭代调整格子大小
        current_grid_size = initial_grid_size
        iteration = 0
        best_result = None
        best_score = float('inf')  # 目标是最小化与target的差距
        
        # 记录迭代历史
        iteration_history = []
        
        for iteration in range(self.max_iterations):
            # 执行一次试采样
            selected_indices = self._perform_voxel_sampling(points_xz, current_grid_size, 
                                                          x_min, x_max, z_min, z_max)
            current_count = len(selected_indices)
            
            # 计算与目标的差距
            deviation = abs(current_count - self.target_point_count)
            deviation_ratio = deviation / self.target_point_count
            
            # 记录本次迭代
            iteration_info = {
                'iteration': iteration + 1,
                'grid_size': current_grid_size,
                'point_count': current_count,
                'deviation': deviation,
                'deviation_ratio': deviation_ratio
            }
            iteration_history.append(iteration_info)
            
            logger.info(f"  Iteration {iteration+1}: grid_size={current_grid_size:.6f}m, "
                       f"points={current_count}, target={self.target_point_count}, "
                       f"deviation={deviation} ({deviation_ratio*100:.1f}%)")
            
            # 检查是否找到可接受的结果
            if self.min_acceptable <= current_count <= self.max_acceptable:
                logger.info(f"  Found acceptable result in iteration {iteration+1}")
                best_result = selected_indices
                best_score = deviation
                break
            
            # 更新最佳结果
            if deviation < best_score:
                best_result = selected_indices
                best_score = deviation
                logger.info(f"  New best result: {current_count} points (deviation: {deviation})")
            
            # 第3步：智能调整格子大小
            current_grid_size = self._compute_next_grid_size(
                current_grid_size, current_count, self.target_point_count
            )
            
            # 防止格子大小变得不合理
            min_grid_size = np.sqrt(total_area / (len(points_xz) * 2))  # 最小：最多一半的点
            max_grid_size = np.sqrt(total_area / 10)  # 最大：至少10个点
            current_grid_size = np.clip(current_grid_size, min_grid_size, max_grid_size)
        
        # 保存迭代统计
        self.selection_stats = {
            'iterations_used': iteration + 1,
            'max_iterations': self.max_iterations,
            'initial_grid_size': initial_grid_size,
            'final_grid_size': current_grid_size,
            'target_points': self.target_point_count,
            'final_points': len(best_result) if best_result is not None else 0,
            'final_deviation': best_score,
            'success': best_score <= self.target_point_count * self.tolerance_ratio,
            'data_area': total_area,
            'input_points': len(points_xz),
            'iteration_history': iteration_history
        }
        
        if best_result is None:
            logger.warning("Adaptive downsampling failed to find good result, using fallback")
            # 备用策略：使用最后一次的结果或简单随机采样
            best_result = self._fallback_selection(points_xz)
        
        logger.info(f"Adaptive downsampling completed:")
        logger.info(f"  Final result: {len(best_result)} points")
        logger.info(f"  Target: {self.target_point_count} points")
        logger.info(f"  Deviation: {abs(len(best_result) - self.target_point_count)} ({abs(len(best_result) - self.target_point_count)/self.target_point_count*100:.1f}%)")
        logger.info(f"  Iterations used: {iteration + 1}/{self.max_iterations}")
        logger.info(f"  Success: {'Yes' if self.selection_stats['success'] else 'No'}")
        
        return best_result
    
    def _perform_voxel_sampling(self, points_xz, grid_size, x_min, x_max, z_min, z_max):
        """
        执行一次体素格采样
        
        Args:
            points_xz: 点的XZ坐标
            grid_size: 当前格子大小
            x_min, x_max, z_min, z_max: 数据边界
        
        Returns:
            选中的点的索引数组
        """
        # 创建格子索引映射
        grid_to_point_map = {}
        
        for i, point in enumerate(points_xz):
            # 计算点所属的格子坐标
            grid_x = int((point[0] - x_min) / grid_size)
            grid_z = int((point[1] - z_min) / grid_size)
            grid_key = (grid_x, grid_z)
            
            # 每个格子只保留第一个遇到的点
            if grid_key not in grid_to_point_map:
                grid_to_point_map[grid_key] = i
        
        # 返回所有被选中的点的索引
        selected_indices = np.array(list(grid_to_point_map.values()))
        return selected_indices
    
    def _compute_next_grid_size(self, current_grid_size, current_count, target_count):
        """
        智能计算下一次迭代的格子大小
        
        使用比例控制策略：格子大小与点数的平方根成反比
        """
        if current_count == 0:
            # 特殊情况：当前没有选中任何点，减小格子
            return current_grid_size * 0.8
        
        # 计算缩放因子
        # 理论上：格子大小 ∝ 1/sqrt(点数)
        # 所以：new_grid_size = current_grid_size * sqrt(current_count / target_count)
        ratio = current_count / target_count
        scale_factor = np.sqrt(ratio)
        
        # 为了避免震荡，限制调整幅度
        scale_factor = np.clip(scale_factor, 0.7, 1.5)  # 最多调整50%
        
        next_grid_size = current_grid_size * scale_factor
        
        return next_grid_size
    
    def _fallback_selection(self, points_xz):
        """备用选择策略：当自适应算法失败时使用"""
        logger.info("Using fallback selection strategy: stratified random sampling")
        
        target_count = min(self.target_point_count, len(points_xz))
        
        try:
            # 尝试分层随机采样
            x_min, z_min = points_xz.min(axis=0)
            x_max, z_max = points_xz.max(axis=0)
            
            # 创建粗糙的网格
            grid_size = max(8, int(np.sqrt(target_count)))  # 至少8x8网格
            x_bins = np.linspace(x_min, x_max, grid_size + 1)
            z_bins = np.linspace(z_min, z_max, grid_size + 1)
            
            selected_indices = []
            points_per_cell = max(1, target_count // (grid_size * grid_size))
            
            for i in range(grid_size):
                for j in range(grid_size):
                    # 找到当前网格中的点
                    mask = ((points_xz[:, 0] >= x_bins[i]) & (points_xz[:, 0] < x_bins[i+1]) &
                           (points_xz[:, 1] >= z_bins[j]) & (points_xz[:, 1] < z_bins[j+1]))
                    cell_indices = np.where(mask)[0]
                    
                    if len(cell_indices) > 0:
                        n_select = min(points_per_cell, len(cell_indices))
                        selected = np.random.choice(cell_indices, n_select, replace=False)
                        selected_indices.extend(selected)
            
            # 如果还不够，随机补充
            while len(selected_indices) < target_count:
                remaining = target_count - len(selected_indices)
                available = list(set(range(len(points_xz))) - set(selected_indices))
                if not available:
                    break
                additional = np.random.choice(available, min(remaining, len(available)), replace=False)
                selected_indices.extend(additional)
            
            return np.array(selected_indices[:target_count])
        
        except Exception as e:
            logger.warning(f"Fallback strategy failed: {e}, using simple random sampling")
            return np.random.choice(len(points_xz), target_count, replace=False)
    
    def _generate_selection_report(self, inlier_points_xz, selected_indices):
        """生成详细的选择报告"""
        selected_points = inlier_points_xz[selected_indices]
        
        # 空间分布分析
        original_span_x = np.max(inlier_points_xz[:, 0]) - np.min(inlier_points_xz[:, 0])
        original_span_z = np.max(inlier_points_xz[:, 1]) - np.min(inlier_points_xz[:, 1])
        
        selected_span_x = np.max(selected_points[:, 0]) - np.min(selected_points[:, 0])
        selected_span_z = np.max(selected_points[:, 1]) - np.min(selected_points[:, 1])
        
        coverage_x = selected_span_x / original_span_x if original_span_x > 0 else 0
        coverage_z = selected_span_z / original_span_z if original_span_z > 0 else 0
        
        # 密度分析
        original_density = len(inlier_points_xz) / self.selection_stats['data_area']
        selected_density = len(selected_points) / self.selection_stats['data_area']
        
        logger.info("Point Selection Analysis:")
        logger.info(f"  Spatial coverage X: {coverage_x*100:.1f}%")
        logger.info(f"  Spatial coverage Z: {coverage_z*100:.1f}%")
        logger.info(f"  Original density: {original_density:.2f} points/m²")
        logger.info(f"  Selected density: {selected_density:.2f} points/m²")
        logger.info(f"  Density ratio: {selected_density/original_density*100:.1f}%")
    
    def _visualize_selection_results(self, inlier_points_xz, selected_indices):
        """可视化选择结果"""
        fig, axes = plt.subplots(2, 3, figsize=(18, 12))
        
        # 准备数据
        all_indices = np.arange(len(inlier_points_xz))
        unselected_indices = np.setdiff1d(all_indices, selected_indices)
        
        selected_points = inlier_points_xz[selected_indices]
        unselected_points = inlier_points_xz[unselected_indices] if len(unselected_indices) > 0 else np.empty((0, 2))
        
        # 1. 选择结果总览
        ax1 = axes[0, 0]
        if len(unselected_points) > 0:
            ax1.scatter(unselected_points[:, 0], unselected_points[:, 1], 
                       c='lightgray', alpha=0.5, s=10, label=f'Unselected ({len(unselected_indices)})')
        ax1.scatter(selected_points[:, 0], selected_points[:, 1], 
                   c='red', alpha=0.8, s=25, label=f'Selected ({len(selected_indices)})')
        ax1.set_xlabel('X [m]')
        ax1.set_ylabel('Z [m]')
        ax1.set_title('Adaptive Voxel Grid Selection Results')
        ax1.legend()
        ax1.grid(True, alpha=0.3)
        ax1.axis('equal')
        
        # 2. 迭代收敛过程
        ax2 = axes[0, 1]
        if 'iteration_history' in self.selection_stats:
            history = self.selection_stats['iteration_history']
            iterations = [h['iteration'] for h in history]
            point_counts = [h['point_count'] for h in history]
            
            ax2.plot(iterations, point_counts, 'b-o', markersize=6, linewidth=2, label='Point Count')
            ax2.axhline(y=self.target_point_count, color='red', linestyle='--', 
                       label=f'Target: {self.target_point_count}')
            ax2.axhline(y=self.min_acceptable, color='green', linestyle=':', alpha=0.7,
                       label=f'Acceptable Range')
            ax2.axhline(y=self.max_acceptable, color='green', linestyle=':', alpha=0.7)
            ax2.fill_between([min(iterations), max(iterations)], 
                           self.min_acceptable, self.max_acceptable, 
                           alpha=0.2, color='green')
            ax2.set_xlabel('Iteration')
            ax2.set_ylabel('Point Count')
            ax2.set_title('Adaptive Algorithm Convergence')
            ax2.legend()
            ax2.grid(True, alpha=0.3)
        
        # 3. 格子大小变化
        ax3 = axes[0, 2]
        if 'iteration_history' in self.selection_stats:
            grid_sizes = [h['grid_size'] for h in history]
            ax3.plot(iterations, grid_sizes, 'g-s', markersize=6, linewidth=2)
            ax3.set_xlabel('Iteration')
            ax3.set_ylabel('Grid Size [m]')
            ax3.set_title('Grid Size Adaptation')
            ax3.grid(True, alpha=0.3)
        
        # 4. 目标偏差变化
        ax4 = axes[1, 0]
        if 'iteration_history' in self.selection_stats:
            deviations = [h['deviation'] for h in history]
            deviation_ratios = [h['deviation_ratio']*100 for h in history]
            
            ax4_twin = ax4.twinx()
            line1 = ax4.plot(iterations, deviations, 'b-o', markersize=6, label='Absolute Deviation')
            line2 = ax4_twin.plot(iterations, deviation_ratios, 'r-s', markersize=6, label='Relative Deviation (%)')
            
            ax4.set_xlabel('Iteration')
            ax4.set_ylabel('Absolute Deviation', color='b')
            ax4_twin.set_ylabel('Relative Deviation (%)', color='r')
            ax4.set_title('Target Deviation Over Iterations')
            
            # 合并图例
            lines = line1 + line2
            labels = [l.get_label() for l in lines]
            ax4.legend(lines, labels, loc='upper right')
            ax4.grid(True, alpha=0.3)
        
        # 5. 选择密度分布
        ax5 = axes[1, 1]
        # 创建密度热图
        from scipy.stats import gaussian_kde
        if len(selected_points) > 10:  # 确保有足够的点
            try:
                kde = gaussian_kde(selected_points.T)
                
                # 创建网格
                x_range = np.linspace(selected_points[:, 0].min(), selected_points[:, 0].max(), 50)
                z_range = np.linspace(selected_points[:, 1].min(), selected_points[:, 1].max(), 50)
                X, Z = np.meshgrid(x_range, z_range)
                positions = np.vstack([X.ravel(), Z.ravel()])
                density = kde(positions).reshape(X.shape)
                
                im = ax5.contourf(X, Z, density, levels=10, cmap='viridis', alpha=0.7)
                ax5.scatter(selected_points[:, 0], selected_points[:, 1], c='red', s=10, alpha=0.8)
                plt.colorbar(im, ax=ax5, label='Density')
                ax5.set_xlabel('X [m]')
                ax5.set_ylabel('Z [m]')
                ax5.set_title('Selected Points Density Distribution')
                ax5.axis('equal')
            except Exception as e:
                ax5.text(0.5, 0.5, f'Density plot\nunavailable\n({str(e)[:30]}...)', 
                        ha='center', va='center', transform=ax5.transAxes)
        else:
            ax5.scatter(selected_points[:, 0], selected_points[:, 1], c='red', s=20)
            ax5.set_title('Selected Points (Too few for density)')
            ax5.axis('equal')
        
        # 6. 统计信息摘要
        ax6 = axes[1, 2]
        stats_text = f"""Adaptive Selection Statistics:

Input Points: {self.selection_stats['input_points']}
Target Points: {self.selection_stats['target_points']}
Final Points: {self.selection_stats['final_points']}
Selection Ratio: {self.selection_stats['final_points']/self.selection_stats['input_points']*100:.1f}%

Iterations Used: {self.selection_stats['iterations_used']}/{self.selection_stats['max_iterations']}
Success: {'Yes' if self.selection_stats['success'] else 'No'}

Initial Grid Size: {self.selection_stats['initial_grid_size']:.6f}m
Final Grid Size: {self.selection_stats['final_grid_size']:.6f}m

Data Area: {self.selection_stats['data_area']:.6f}m²
Final Deviation: {self.selection_stats['final_deviation']}
Deviation Ratio: {self.selection_stats['final_deviation']/self.selection_stats['target_points']*100:.1f}%

Algorithm: Adaptive Voxel Grid
Strategy: Iterative Grid Size Adjustment"""
        
        ax6.text(0.05, 0.95, stats_text, transform=ax6.transAxes, fontsize=10,
                verticalalignment='top', fontfamily='monospace')
        ax6.set_xlim(0, 1)
        ax6.set_ylim(0, 1)
        ax6.set_title('Selection Statistics')
        ax6.axis('off')
        
        plt.tight_layout()
        plt.savefig(os.path.join(img_dir, 'stage2_5_adaptive_point_selection.png'), 
                   dpi=300, bbox_inches='tight')
        plt.close()
        
        logger.info("Adaptive point selection visualization saved: stage2_5_adaptive_point_selection.png")

class Stage3_GPRFineRegistration:
    """
    Stage 3: GPR精配准
    使用高斯过程回归学习精确且稳健的非刚性形变场
    """
    def __init__(self):
        self.gpr_x = None
        self.gpr_z = None
        self.gpr_stats = {}
        self.scaler_input = StandardScaler()
        self.scaler_output_x = StandardScaler()
        self.scaler_output_z = StandardScaler()
        
    def learn_and_apply_deformation(self, svd_transformed_A, residual_vectors, training_indices, target_B):
        """
        学习非刚性形变并应用到所有点
        
        Args:
        - svd_transformed_A: SVD变换后的源点云
        - residual_vectors: SVD残差向量
        - training_indices: 用于GPR训练的点的索引
        - target_B: 目标点云（用于最终评估）
        
        Returns:
        - final_transformed_A: 最终变换后的源点云
        - gpr_stats: GPR统计信息
        """
        logger.info("=== Stage 3: GPR Fine Registration ===")
        
        svd_transformed_A = np.array(svd_transformed_A)
        residual_vectors = np.array(residual_vectors)
        target_B = np.array(target_B)
        
        # 使用选择的训练点训练GPR
        training_source = svd_transformed_A[training_indices]
        training_residuals = residual_vectors[training_indices]
        
        logger.info(f"Training GPR with {len(training_indices)} carefully selected points")
        logger.info(f"Total points for final prediction: {len(svd_transformed_A)}")
        
        # 训练GPR模型
        success = self._train_gpr_models(training_source, training_residuals)
        
        if not success:
            logger.error("GPR training failed")
            return svd_transformed_A, None
        
        # 预测所有点的形变
        predicted_deformation = self._predict_deformation(svd_transformed_A)
        
        # 应用形变得到最终结果
        final_transformed_A = svd_transformed_A + predicted_deformation
        final_transformed_A[:, 1] = 0  # 确保Y=0
        
        # 评估最终结果
        final_rmse_results = self._evaluate_final_results(target_B, final_transformed_A, svd_transformed_A)
        
        logger.info("Stage 3 (GPR) completed successfully")
        
        return final_transformed_A, final_rmse_results
    
    def _train_gpr_models(self, training_source, training_residuals):
        """训练GPR模型"""
        logger.info("Training Gaussian Process Regression models...")
        
        # 准备训练数据（只使用XZ坐标）
        X_train = training_source[:, [0, 2]]
        y_train_x = training_residuals[:, 0]
        y_train_z = training_residuals[:, 2]
        
        # 数据标准化
        X_train_scaled = self.scaler_input.fit_transform(X_train)
        y_train_x_scaled = self.scaler_output_x.fit_transform(y_train_x.reshape(-1, 1)).ravel()
        y_train_z_scaled = self.scaler_output_z.fit_transform(y_train_z.reshape(-1, 1)).ravel()
        
        try:
            # 定义多个候选核函数
            kernels = [
                RBF(length_scale=1.0, length_scale_bounds=(1e-3, 1e3)) + WhiteKernel(noise_level=1e-5),
                Matern(length_scale=1.0, nu=1.5) + WhiteKernel(noise_level=1e-5),
                Matern(length_scale=1.0, nu=2.5) + WhiteKernel(noise_level=1e-5),
                RationalQuadratic(length_scale=1.0, alpha=1.0) + WhiteKernel(noise_level=1e-5)
            ]
            
            best_score_x = -np.inf
            best_score_z = -np.inf
            best_gpr_x = None
            best_gpr_z = None
            
            # 通过交叉验证选择最佳核函数
            for i, kernel in enumerate(kernels):
                logger.info(f"  Testing kernel {i+1}/{len(kernels)}: {kernel}")
                
                # X方向GPR
                gpr_x = GaussianProcessRegressor(
                    kernel=kernel, 
                    alpha=1e-6,
                    normalize_y=False,
                    n_restarts_optimizer=3,
                    random_state=42
                )
                
                # Z方向GPR
                gpr_z = GaussianProcessRegressor(
                    kernel=kernel, 
                    alpha=1e-6,
                    normalize_y=False,
                    n_restarts_optimizer=3,
                    random_state=42
                )
                
                try:
                    # 训练模型
                    gpr_x.fit(X_train_scaled, y_train_x_scaled)
                    gpr_z.fit(X_train_scaled, y_train_z_scaled)
                    
                    # 计算训练分数（对数边际似然）
                    score_x = gpr_x.log_marginal_likelihood()
                    score_z = gpr_z.log_marginal_likelihood()
                    
                    logger.info(f"    X-direction log marginal likelihood: {score_x:.3f}")
                    logger.info(f"    Z-direction log marginal likelihood: {score_z:.3f}")
                    
                    # 选择最佳模型
                    if score_x > best_score_x:
                        best_score_x = score_x
                        best_gpr_x = gpr_x
                    
                    if score_z > best_score_z:
                        best_score_z = score_z
                        best_gpr_z = gpr_z
                        
                except Exception as e:
                    logger.warning(f"    Kernel {i+1} failed: {e}")
                    continue
            
            if best_gpr_x is None or best_gpr_z is None:
                logger.error("All GPR kernels failed")
                return False
            
            self.gpr_x = best_gpr_x
            self.gpr_z = best_gpr_z
            
            # 保存GPR统计信息
            self.gpr_stats = {
                'training_points': len(X_train),
                'best_score_x': best_score_x,
                'best_score_z': best_score_z,
                'kernel_x': str(self.gpr_x.kernel_),
                'kernel_z': str(self.gpr_z.kernel_),
                'kernel_params_x': self.gpr_x.kernel_.get_params(),
                'kernel_params_z': self.gpr_z.kernel_.get_params()
            }
            
            logger.info("GPR training completed successfully:")
            logger.info(f"  Best X-direction score: {best_score_x:.3f}")
            logger.info(f"  Best Z-direction score: {best_score_z:.3f}")
            logger.info(f"  Final X kernel: {self.gpr_x.kernel_}")
            logger.info(f"  Final Z kernel: {self.gpr_z.kernel_}")
            
            return True
            
        except Exception as e:
            logger.error(f"GPR training failed: {e}")
            return False
    
    def _predict_deformation(self, query_points):
        """预测形变"""
        if self.gpr_x is None or self.gpr_z is None:
            logger.error("GPR models not trained")
            return np.zeros_like(query_points)
        
        # 准备查询数据
        X_query = query_points[:, [0, 2]]
        X_query_scaled = self.scaler_input.transform(X_query)
        
        try:
            # 预测（包含不确定性）
            pred_x_scaled, std_x = self.gpr_x.predict(X_query_scaled, return_std=True)
            pred_z_scaled, std_z = self.gpr_z.predict(X_query_scaled, return_std=True)
            
            # 反标准化
            pred_x = self.scaler_output_x.inverse_transform(pred_x_scaled.reshape(-1, 1)).ravel()
            pred_z = self.scaler_output_z.inverse_transform(pred_z_scaled.reshape(-1, 1)).ravel()
            
            # 构建形变向量
            deformation = np.zeros_like(query_points)
            deformation[:, 0] = pred_x
            deformation[:, 2] = pred_z
            
            # 保存不确定性信息
            self.prediction_uncertainty = {
                'std_x': std_x,
                'std_z': std_z,
                'mean_std_x': np.mean(std_x),
                'mean_std_z': np.mean(std_z),
                'max_std_x': np.max(std_x),
                'max_std_z': np.max(std_z)
            }
            
            logger.info("GPR prediction completed:")
            logger.info(f"  Mean X uncertainty: {np.mean(std_x):.6f}")
            logger.info(f"  Mean Z uncertainty: {np.mean(std_z):.6f}")
            logger.info(f"  Max X uncertainty: {np.max(std_x):.6f}")
            logger.info(f"  Max Z uncertainty: {np.max(std_z):.6f}")
            
            return deformation
            
        except Exception as e:
            logger.error(f"GPR prediction failed: {e}")
            return np.zeros_like(query_points)
    
    def _evaluate_final_results(self, target_B, final_transformed_A, svd_transformed_A):
        """评估最终配准结果"""
        # 计算最终误差
        final_errors = target_B - final_transformed_A
        final_error_magnitudes = np.sqrt(np.sum(final_errors**2, axis=1))
        
        # 计算SVD误差（用于对比）
        svd_errors = target_B - svd_transformed_A
        svd_error_magnitudes = np.sqrt(np.sum(svd_errors**2, axis=1))
        
        # 计算各种RMSE指标
        final_rmse_overall = np.sqrt(np.mean(final_error_magnitudes**2))
        final_rmse_x = np.sqrt(np.mean(final_errors[:, 0]**2))
        final_rmse_z = np.sqrt(np.mean(final_errors[:, 2]**2))
        
        svd_rmse_overall = np.sqrt(np.mean(svd_error_magnitudes**2))
        svd_rmse_x = np.sqrt(np.mean(svd_errors[:, 0]**2))
        svd_rmse_z = np.sqrt(np.mean(svd_errors[:, 2]**2))
        
        # 计算改进
        improvement_overall = (svd_rmse_overall - final_rmse_overall)
        improvement_x = (svd_rmse_x - final_rmse_x)
        improvement_z = (svd_rmse_z - final_rmse_z)
        
        # 更新统计信息
        self.gpr_stats.update({
            'final_rmse_overall': final_rmse_overall,
            'final_rmse_x': final_rmse_x,
            'final_rmse_z': final_rmse_z,
            'svd_rmse_overall': svd_rmse_overall,
            'svd_rmse_x': svd_rmse_x,
            'svd_rmse_z': svd_rmse_z,
            'improvement_overall': improvement_overall,
            'improvement_x': improvement_x,
            'improvement_z': improvement_z,
            'improvement_ratio': improvement_overall / svd_rmse_overall if svd_rmse_overall > 0 else 0,
            'final_max_error': np.max(final_error_magnitudes),
            'final_mean_error': np.mean(final_error_magnitudes),
            'final_std_error': np.std(final_error_magnitudes)
        })
        
        logger.info("Final GPR Results:")
        logger.info(f"  Final overall RMSE: {final_rmse_overall*1000:.3f}mm")
        logger.info(f"  Final X RMSE: {final_rmse_x*1000:.3f}mm")
        logger.info(f"  Final Z RMSE: {final_rmse_z*1000:.3f}mm")
        logger.info(f"  SVD overall RMSE: {svd_rmse_overall*1000:.3f}mm")
        logger.info(f"  Overall improvement: {improvement_overall*1000:.3f}mm ({improvement_overall/svd_rmse_overall*100:.1f}%)")
        logger.info(f"  X improvement: {improvement_x*1000:.3f}mm")
        logger.info(f"  Z improvement: {improvement_z*1000:.3f}mm")
        
        # 可视化最终结果
        self._visualize_final_results(target_B, final_transformed_A, svd_transformed_A)
        
        return {
            'lateral_rmse': final_rmse_x,
            'longitudinal_rmse': final_rmse_z,
            'overall_rmse': final_rmse_overall
        }
    
    def _visualize_final_results(self, target_B, final_transformed_A, svd_transformed_A):
        """可视化最终GPR结果"""
        fig, axes = plt.subplots(2, 3, figsize=(18, 12))
        
        final_errors = np.sqrt(np.sum((target_B - final_transformed_A)**2, axis=1))
        
        # 最终配准结果对比
        ax1 = axes[0, 0]
        ax1.scatter(svd_transformed_A[:, 0], svd_transformed_A[:, 2], c='blue', alpha=0.6, s=30, label='SVD Result')
        ax1.scatter(final_transformed_A[:, 0], final_transformed_A[:, 2], c='green', alpha=0.6, s=30, label='Final GPR Result')
        ax1.scatter(target_B[:, 0], target_B[:, 2], c='red', alpha=0.6, s=30, label='Target')
        ax1.set_xlabel('X [m]')
        ax1.set_ylabel('Z [m]')
        ax1.set_title('Final Registration Results')
        ax1.legend()
        ax1.grid(True, alpha=0.3)
        ax1.axis('equal')
        
        # 误差对比分布
        ax2 = axes[0, 1]
        svd_errors = np.sqrt(np.sum((target_B - svd_transformed_A)**2, axis=1))
        
        ax2.hist(svd_errors*1000, bins=30, alpha=0.7, color='blue', label='SVD Errors', density=True)
        ax2.hist(final_errors*1000, bins=30, alpha=0.7, color='green', label='Final Errors', density=True)
        ax2.axvline(x=TARGET_RMSE*1000, color='red', linestyle='--', label=f'Target: {TARGET_RMSE*1000:.1f}mm')
        ax2.set_xlabel('Error Magnitude [mm]')
        ax2.set_ylabel('Density')
        ax2.set_title('Error Distribution Comparison')
        ax2.legend()
        ax2.grid(True, alpha=0.3)
        
        # RMSE改进条形图
        ax3 = axes[0, 2]
        categories = ['Overall', 'X-Direction', 'Z-Direction']
        svd_rmse = [self.gpr_stats['svd_rmse_overall']*1000, 
                   self.gpr_stats['svd_rmse_x']*1000, 
                   self.gpr_stats['svd_rmse_z']*1000]
        final_rmse = [self.gpr_stats['final_rmse_overall']*1000, 
                     self.gpr_stats['final_rmse_x']*1000, 
                     self.gpr_stats['final_rmse_z']*1000]
        
        x = np.arange(len(categories))
        width = 0.35
        
        bars1 = ax3.bar(x - width/2, svd_rmse, width, label='SVD Only', alpha=0.8, color='blue')
        bars2 = ax3.bar(x + width/2, final_rmse, width, label='SVD + GPR', alpha=0.8, color='green')
        
        ax3.axhline(y=TARGET_RMSE*1000, color='red', linestyle='--', alpha=0.7, label=f'Target: {TARGET_RMSE*1000:.1f}mm')
        ax3.set_xlabel('Error Type')
        ax3.set_ylabel('RMSE [mm]')
        ax3.set_title('RMSE Improvement Comparison')
        ax3.set_xticks(x)
        ax3.set_xticklabels(categories)
        ax3.legend()
        ax3.grid(True, alpha=0.3)
        
        # 形变场可视化
        ax4 = axes[1, 0]
        deformation = final_transformed_A - svd_transformed_A
        step = max(1, len(svd_transformed_A) // 50)
        scale_factor = 20
        ax4.quiver(svd_transformed_A[::step, 0], svd_transformed_A[::step, 2], 
                  deformation[::step, 0]*scale_factor, deformation[::step, 2]*scale_factor,
                  angles='xy', scale_units='xy', scale=1, alpha=0.7, color='green')
        ax4.scatter(svd_transformed_A[::step, 0], svd_transformed_A[::step, 2], c='blue', alpha=0.5, s=20)
        ax4.set_xlabel('X [m]')
        ax4.set_ylabel('Z [m]')
        ax4.set_title(f'GPR Deformation Field (×{scale_factor})')
        ax4.grid(True, alpha=0.3)
        ax4.axis('equal')
        
        # 不确定性可视化（如果有的话）
        ax5 = axes[1, 1]
        if hasattr(self, 'prediction_uncertainty'):
            uncertainty_combined = np.sqrt(self.prediction_uncertainty['std_x']**2 + 
                                         self.prediction_uncertainty['std_z']**2)
            scatter = ax5.scatter(svd_transformed_A[:, 0], svd_transformed_A[:, 2], 
                                c=uncertainty_combined, cmap='viridis', s=30, alpha=0.7)
            plt.colorbar(scatter, ax=ax5, label='Prediction Uncertainty')
            ax5.set_xlabel('X [m]')
            ax5.set_ylabel('Z [m]')
            ax5.set_title('GPR Prediction Uncertainty')
            ax5.grid(True, alpha=0.3)
            ax5.axis('equal')
        else:
            ax5.text(0.5, 0.5, 'Uncertainty data\nnot available', 
                    ha='center', va='center', transform=ax5.transAxes, fontsize=12)
            ax5.set_title('GPR Prediction Uncertainty')
        
        # 最终误差空间分布
        ax6 = axes[1, 2]
        scatter = ax6.scatter(final_transformed_A[:, 0], final_transformed_A[:, 2], 
                            c=final_errors*1000, cmap='viridis', s=30, alpha=0.7)
        plt.colorbar(scatter, ax=ax6, label='Final Error [mm]')
        ax6.set_xlabel('X [m]')
        ax6.set_ylabel('Z [m]')
        ax6.set_title('Spatial Distribution of Final Errors')
        ax6.grid(True, alpha=0.3)
        ax6.axis('equal')
        
        plt.tight_layout()
        plt.savefig(os.path.join(img_dir, 'stage3_gpr_results.png'), dpi=300, bbox_inches='tight')
        plt.close()
        
        logger.info("GPR visualization saved: stage3_gpr_results.png")

class ThreeStageRegistrationPipeline:
    """
    四阶段配准流水线：SVD → RANSAC → Point Selection → GPR
    """
    def __init__(self, positions_A, positions_B):
        self.positions_A = np.array(positions_A)
        self.positions_B = np.array(positions_B)
        self.stage1 = Stage1_SVDCoarseAlignment()
        self.stage2 = Stage2_RANSACInlierSelection()
        self.point_selector = GPRPointSelector(target_point_count=2000, tolerance_ratio=0.05, max_iterations=20)
        self.stage3 = Stage3_GPRFineRegistration()
        self.pipeline_stats = {}
        
    def run_complete_pipeline(self):
        """运行完整的四阶段配准流水线"""
        logger.info("=" * 80)
        logger.info("STARTING FOUR-STAGE REGISTRATION PIPELINE")
        logger.info("SVD (Coarse) → RANSAC (Filter) → Adaptive Point Selection (Downsample) → GPR (Fine)")
        logger.info("=" * 80)
        
        start_time = datetime.datetime.now()
        
        # 数据预处理
        logger.info("Data Preprocessing...")
        clean_A, clean_B = self._preprocess_data()
        
        # Stage 1: SVD粗配准
        logger.info("\n" + "="*50)
        svd_transformed_A, residual_vectors, T_svd = self.stage1.perform_svd_alignment(clean_A, clean_B)
        
        # Stage 2: RANSAC内点筛选
        logger.info("\n" + "="*50)
        inlier_indices, ransac_model = self.stage2.select_inliers(svd_transformed_A, residual_vectors)
        
        # Stage 2.5: 自适应点选择
        logger.info("\n" + "="*50)
        gpr_training_indices = self.point_selector.select_points(svd_transformed_A, inlier_indices)
        
        # Stage 3: GPR精配准
        logger.info("\n" + "="*50)
        final_transformed_A, final_rmse_results = self.stage3.learn_and_apply_deformation(
            svd_transformed_A, 
            residual_vectors, 
            gpr_training_indices,
            clean_B
        )
        
        end_time = datetime.datetime.now()
        processing_time = (end_time - start_time).total_seconds()
        
        # 汇总结果
        self._summarize_pipeline_results(clean_A, clean_B, final_transformed_A, final_rmse_results, processing_time)
        
        # 生成综合报告
        self._generate_comprehensive_report()
        
        # 创建综合可视化
        self._create_comprehensive_visualization(clean_A, clean_B, svd_transformed_A, final_transformed_A)
        
        logger.info("=" * 80)
        logger.info("FOUR-STAGE REGISTRATION PIPELINE COMPLETED")
        logger.info("=" * 80)
        
        return final_rmse_results
    
    def _preprocess_data(self):
        """数据预处理：去重和基本清理"""
        # 去除重复点
        positions_A = self.positions_A.copy()
        positions_B = self.positions_B.copy()
        
        # 简单的重复点检测
        unique_indices = []
        seen = set()
        
        for i, (point_a, point_b) in enumerate(zip(positions_A, positions_B)):
            key = (round(point_a[0], 6), round(point_a[2], 6), round(point_b[0], 6), round(point_b[2], 6))
            if key not in seen:
                seen.add(key)
                unique_indices.append(i)
        
        clean_A = positions_A[unique_indices]
        clean_B = positions_B[unique_indices]
        
        logger.info(f"Data preprocessing: {len(positions_A)} → {len(clean_A)} points after duplicate removal")
        
        return clean_A, clean_B
    
    def _summarize_pipeline_results(self, original_A, target_B, final_A, final_rmse_results, processing_time):
        """汇总流水线结果"""
        # 检查是否达到目标精度
        success = final_rmse_results['overall_rmse'] <= TARGET_RMSE
        
        # 计算改进程度
        svd_rmse = self.stage1.svd_stats['rmse_overall']
        improvement = (svd_rmse - final_rmse_results['overall_rmse']) * 1000  # mm
        improvement_ratio = improvement / (svd_rmse * 1000) * 100  # %
        
        # 保存流水线统计信息
        self.pipeline_stats = {
            'total_points': len(original_A),
            'processing_time': processing_time,
            'success': success,
            'target_rmse': TARGET_RMSE,
            'final_rmse': final_rmse_results['overall_rmse'],
            'svd_rmse': svd_rmse,
            'improvement_mm': improvement,
            'improvement_ratio': improvement_ratio,
            'ransac_inlier_ratio': self.stage2.ransac_stats['inlier_ratio'],
            'gpr_training_points': self.stage3.gpr_stats['training_points'],
            'stage1_stats': self.stage1.svd_stats,
            'stage2_stats': self.stage2.ransac_stats,
            'stage2_5_stats': self.point_selector.selection_stats,
            'stage3_stats': self.stage3.gpr_stats
        }
        
        logger.info("\n" + "="*60)
        logger.info("PIPELINE SUMMARY")
        logger.info("="*60)
        logger.info(f"Processing time: {processing_time:.2f} seconds")
        logger.info(f"Total points processed: {len(original_A)}")
        logger.info(f"RANSAC inlier ratio: {self.stage2.ransac_stats['inlier_ratio']*100:.1f}%")
        logger.info(f"Adaptive selection success: {'Yes' if self.point_selector.selection_stats['success'] else 'No'}")
        logger.info(f"GPR training points: {self.stage3.gpr_stats['training_points']}")
        logger.info("")
        logger.info("ACCURACY RESULTS:")
        logger.info(f"  Target RMSE: {TARGET_RMSE*1000:.2f}mm")
        logger.info(f"  SVD RMSE: {svd_rmse*1000:.3f}mm")
        logger.info(f"  Final RMSE: {final_rmse_results['overall_rmse']*1000:.3f}mm")
        logger.info(f"  Improvement: {improvement:.3f}mm ({improvement_ratio:.1f}%)")
        logger.info(f"  SUCCESS: {'YES' if success else 'NO'}")
        
        if success:
            logger.info("Target accuracy achieved!")
        else:
            logger.info(f"Target accuracy not achieved (deficit: {(final_rmse_results['overall_rmse'] - TARGET_RMSE)*1000:.3f}mm)")
    
    def _generate_comprehensive_report(self):
        """生成综合报告"""
        report_file = os.path.join(result_dir_final, "four_stage_registration_report.txt")
        
        with open(report_file, "w") as f:
            f.write("FOUR-STAGE REGISTRATION PIPELINE REPORT\n")
            f.write("=" * 80 + "\n")
            f.write(f"Generated: {datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n")
            f.write(f"Scale Factor: 1:{SCALE_FACTOR}\n")
            f.write(f"Target RMSE: {TARGET_RMSE*1000:.2f}mm\n")
            f.write("\n")
            
            f.write("PIPELINE OVERVIEW\n")
            f.write("-" * 40 + "\n")
            f.write("Stage 1: SVD Coarse Alignment\n")
            f.write("  Purpose: Compute global rigid transformation (rotation + translation)\n")
            f.write("  Output: Transformed source points + residual vectors\n")
            f.write("\n")
            f.write("Stage 2: RANSAC Inlier Selection\n")
            f.write("  Purpose: Select reliable data points for final model training\n")
            f.write("  Output: Inlier indices (noise-free training data)\n")
            f.write("\n")
            f.write("Stage 2.5: Adaptive Voxel Grid Point Selection\n")
            f.write("  Purpose: Intelligently downsample inliers for efficient GPR training\n")
            f.write("  Strategy: Iterative grid size adjustment to reach target point count\n")
            f.write("  Algorithm: Adaptive voxel grid downsampling with automatic convergence\n")
            f.write("  Output: Optimally selected training point indices\n")
            f.write("\n")
            f.write("Stage 3: GPR Fine Registration\n")
            f.write("  Purpose: Learn precise non-rigid deformation field\n")
            f.write("  Output: Final transformed source points\n")
            f.write("\n")
            
            f.write("PROCESSING RESULTS\n")
            f.write("-" * 40 + "\n")
            f.write(f"Total processing time: {self.pipeline_stats['processing_time']:.2f} seconds\n")
            f.write(f"Total points: {self.pipeline_stats['total_points']}\n")
            f.write(f"Final RMSE: {self.pipeline_stats['final_rmse']*1000:.3f}mm\n")
            f.write(f"Target achieved: {'YES' if self.pipeline_stats['success'] else 'NO'}\n")
            f.write(f"Improvement: {self.pipeline_stats['improvement_mm']:.3f}mm ({self.pipeline_stats['improvement_ratio']:.1f}%)\n")
            f.write("\n")
            
            f.write("STAGE 1 RESULTS (SVD)\n")
            f.write("-" * 40 + "\n")
            stage1_stats = self.pipeline_stats['stage1_stats']
            f.write(f"Overall RMSE: {stage1_stats['rmse_overall']*1000:.3f}mm\n")
            f.write(f"X-direction RMSE: {stage1_stats['rmse_x']*1000:.3f}mm\n")
            f.write(f"Z-direction RMSE: {stage1_stats['rmse_z']*1000:.3f}mm\n")
            f.write(f"Maximum error: {stage1_stats['max_error']*1000:.3f}mm\n")
            f.write(f"Mean error: {stage1_stats['mean_error']*1000:.3f}mm\n")
            f.write("\n")
            
            f.write("STAGE 2 RESULTS (RANSAC)\n")
            f.write("-" * 40 + "\n")
            stage2_stats = self.pipeline_stats['stage2_stats']
            f.write(f"Total points: {stage2_stats['total_points']}\n")
            f.write(f"Inliers found: {stage2_stats['inlier_count']} ({stage2_stats['inlier_ratio']*100:.1f}%)\n")
            f.write(f"Outliers removed: {stage2_stats['outlier_count']} ({(1-stage2_stats['inlier_ratio'])*100:.1f}%)\n")
            f.write(f"Distance threshold: {stage2_stats['distance_threshold']*1000:.1f}mm\n")
            f.write(f"Iterations used: {stage2_stats['iterations_used']}\n")
            f.write("\n")
            
            f.write("STAGE 2.5 RESULTS (ADAPTIVE POINT SELECTION)\n")
            f.write("-" * 40 + "\n")
            stage2_5_stats = self.pipeline_stats['stage2_5_stats']
            f.write(f"Input points (inliers): {stage2_5_stats['input_points']}\n")
            f.write(f"Target points: {stage2_5_stats['target_points']}\n")
            f.write(f"Final selected points: {stage2_5_stats['final_points']}\n")
            f.write(f"Selection success: {'Yes' if stage2_5_stats['success'] else 'No'}\n")
            f.write(f"Algorithm: Adaptive Voxel Grid Downsampling\n")
            f.write(f"Iterations used: {stage2_5_stats['iterations_used']}/{stage2_5_stats['max_iterations']}\n")
            f.write(f"Initial grid size: {stage2_5_stats['initial_grid_size']:.6f}m\n")
            f.write(f"Final grid size: {stage2_5_stats['final_grid_size']:.6f}m\n")
            f.write(f"Final deviation: {stage2_5_stats['final_deviation']} points\n")
            f.write(f"Selection ratio: {stage2_5_stats['final_points']/stage2_5_stats['input_points']*100:.1f}%\n")
            f.write("\n")
            
            f.write("STAGE 3 RESULTS (GPR)\n")
            f.write("-" * 40 + "\n")
            stage3_stats = self.pipeline_stats['stage3_stats']
            f.write(f"Training points: {stage3_stats['training_points']}\n")
            f.write(f"Final overall RMSE: {stage3_stats['final_rmse_overall']*1000:.3f}mm\n")
            f.write(f"Final X RMSE: {stage3_stats['final_rmse_x']*1000:.3f}mm\n")
            f.write(f"Final Z RMSE: {stage3_stats['final_rmse_z']*1000:.3f}mm\n")
            f.write(f"Best X kernel score: {stage3_stats['best_score_x']:.3f}\n")
            f.write(f"Best Z kernel score: {stage3_stats['best_score_z']:.3f}\n")
            f.write(f"Overall improvement: {stage3_stats['improvement_overall']*1000:.3f}mm\n")
            f.write(f"Improvement ratio: {stage3_stats['improvement_ratio']*100:.1f}%\n")
            f.write("\n")
            
            f.write("REAL-WORLD SCALING\n")
            f.write("-" * 40 + "\n")
            f.write(f"Model scale: 1:{SCALE_FACTOR}\n")
            f.write(f"Model RMSE: {self.pipeline_stats['final_rmse']*1000:.3f}mm\n")
            f.write(f"Real-world equivalent: {self.pipeline_stats['final_rmse']*SCALE_FACTOR*100:.2f}cm\n")
            f.write(f"Real-world target: {REAL_WORLD_LATERAL_TARGET*100:.0f}cm\n")
            f.write("\n")
        
        logger.info(f"Comprehensive report saved: {report_file}")
    
    def _create_comprehensive_visualization(self, original_A, target_B, svd_A, final_A):
        """创建综合可视化图表"""
        fig = plt.figure(figsize=(20, 16))
        
        # 创建网格布局
        gs = fig.add_gridspec(3, 4, hspace=0.3, wspace=0.3)
        
        # 1. 流水线概览
        ax1 = fig.add_subplot(gs[0, :2])
        ax1.scatter(original_A[:, 0], original_A[:, 2], c='blue', alpha=0.6, s=20, label='Original Source')
        ax1.scatter(svd_A[:, 0], svd_A[:, 2], c='orange', alpha=0.6, s=20, label='After SVD')
        ax1.scatter(final_A[:, 0], final_A[:, 2], c='green', alpha=0.6, s=20, label='Final Result')
        ax1.scatter(target_B[:, 0], target_B[:, 2], c='red', alpha=0.6, s=20, label='Target')
        ax1.set_xlabel('X [m]')
        ax1.set_ylabel('Z [m]')
        ax1.set_title('Four-Stage Registration Pipeline Overview')
        ax1.legend()
        ax1.grid(True, alpha=0.3)
        ax1.axis('equal')
        
        # 2. RMSE进展
        ax2 = fig.add_subplot(gs[0, 2:])
        stages = ['SVD Only', 'Final (SVD+RANSAC+AdaptiveSelection+GPR)']
        rmse_values = [self.pipeline_stats['svd_rmse']*1000, self.pipeline_stats['final_rmse']*1000]
        colors = ['orange', 'green']
        
        bars = ax2.bar(stages, rmse_values, color=colors, alpha=0.8)
        ax2.axhline(y=TARGET_RMSE*1000, color='red', linestyle='--', alpha=0.7, 
                   label=f'Target: {TARGET_RMSE*1000:.1f}mm')
        ax2.set_ylabel('RMSE [mm]')
        ax2.set_title('RMSE Improvement Through Pipeline')
        ax2.legend()
        ax2.grid(True, alpha=0.3)
        
        # 在柱子上添加数值标签
        for bar, value in zip(bars, rmse_values):
            ax2.text(bar.get_x() + bar.get_width()/2, bar.get_height() + 0.1,
                    f'{value:.2f}mm', ha='center', va='bottom', fontweight='bold')
        
        # 3. 误差分布对比
        ax3 = fig.add_subplot(gs[1, 0])
        svd_errors = np.sqrt(np.sum((target_B - svd_A)**2, axis=1))
        final_errors = np.sqrt(np.sum((target_B - final_A)**2, axis=1))
        
        ax3.hist(svd_errors*1000, bins=25, alpha=0.7, color='orange', label='SVD Only', density=True)
        ax3.hist(final_errors*1000, bins=25, alpha=0.7, color='green', label='Final Result', density=True)
        ax3.axvline(x=TARGET_RMSE*1000, color='red', linestyle='--', alpha=0.7)
        ax3.set_xlabel('Error [mm]')
        ax3.set_ylabel('Density')
        ax3.set_title('Error Distribution Comparison')
        ax3.legend()
        ax3.grid(True, alpha=0.3)
        
        # 4. RANSAC结果
        ax4 = fig.add_subplot(gs[1, 1])
        inlier_ratio = self.pipeline_stats['ransac_inlier_ratio']
        outlier_ratio = 1 - inlier_ratio
        if outlier_ratio > 0:
            ax4.pie([inlier_ratio, outlier_ratio], labels=['Inliers', 'Outliers'], 
                   colors=['green', 'red'], autopct='%1.1f%%', startangle=90)
        else:
            ax4.pie([100], labels=['All Inliers'], colors=['green'], autopct='%1.1f%%', startangle=90)
        ax4.set_title(f'RANSAC Classification\n({self.stage2.ransac_stats["inlier_count"]} inliers)')
        
        # 5. 空间误差分布
        ax5 = fig.add_subplot(gs[1, 2])
        scatter = ax5.scatter(final_A[:, 0], final_A[:, 2], c=final_errors*1000, 
                            cmap='viridis', s=25, alpha=0.7)
        plt.colorbar(scatter, ax=ax5, label='Final Error [mm]')
        ax5.set_xlabel('X [m]')
        ax5.set_ylabel('Z [m]')
        ax5.set_title('Spatial Error Distribution')
        ax5.grid(True, alpha=0.3)
        ax5.axis('equal')
        
        # 6. 自适应选择统计
        ax6 = fig.add_subplot(gs[1, 3])
        selection_stats = self.pipeline_stats['stage2_5_stats']
        ax6.text(0.1, 0.9, f"Processing Time: {self.pipeline_stats['processing_time']:.2f}s", 
                transform=ax6.transAxes, fontsize=12, fontweight='bold')
        ax6.text(0.1, 0.8, f"Total Points: {self.pipeline_stats['total_points']}", 
                transform=ax6.transAxes, fontsize=11)
        ax6.text(0.1, 0.7, f"Inlier Ratio: {inlier_ratio*100:.1f}%", 
                transform=ax6.transAxes, fontsize=11)
        ax6.text(0.1, 0.6, f"Adaptive Selection: {'Success' if selection_stats['success'] else 'Partial'}", 
                transform=ax6.transAxes, fontsize=11, 
                color='green' if selection_stats['success'] else 'orange')
        ax6.text(0.1, 0.5, f"GPR Training Points: {self.stage3.gpr_stats['training_points']}", 
                transform=ax6.transAxes, fontsize=11)
        ax6.text(0.1, 0.4, f"Selection Iterations: {selection_stats['iterations_used']}/{selection_stats['max_iterations']}", 
                transform=ax6.transAxes, fontsize=10)
        ax6.text(0.1, 0.3, f"Final RMSE: {self.pipeline_stats['final_rmse']*1000:.3f}mm", 
                transform=ax6.transAxes, fontsize=11, color='green', fontweight='bold')
        ax6.text(0.1, 0.2, f"Target RMSE: {TARGET_RMSE*1000:.2f}mm", 
                transform=ax6.transAxes, fontsize=11, color='red')
        ax6.text(0.1, 0.1, f"Success: {'YES' if self.pipeline_stats['success'] else 'NO'}", 
                transform=ax6.transAxes, fontsize=12, 
                color='green' if self.pipeline_stats['success'] else 'red', fontweight='bold')
        ax6.set_xlim(0, 1)
        ax6.set_ylim(0, 1)
        ax6.set_title('Pipeline Statistics')
        ax6.axis('off')
        
        # 7. 形变场可视化
        ax7 = fig.add_subplot(gs[2, :2])
        deformation = final_A - svd_A
        step = max(1, len(svd_A) // 50)
        scale_factor = 15
        ax7.quiver(svd_A[::step, 0], svd_A[::step, 2], 
                  deformation[::step, 0]*scale_factor, deformation[::step, 2]*scale_factor,
                  angles='xy', scale_units='xy', scale=1, alpha=0.7, color='purple', width=0.003)
        ax7.scatter(svd_A[::step, 0], svd_A[::step, 2], c='blue', alpha=0.5, s=15)
        ax7.set_xlabel('X [m]')
        ax7.set_ylabel('Z [m]')
        ax7.set_title(f'GPR Learned Deformation Field (×{scale_factor})')
        ax7.grid(True, alpha=0.3)
        ax7.axis('equal')
        
        # 8. 阶段性改进
        ax8 = fig.add_subplot(gs[2, 2:])
        improvement_stages = ['SVD→Final']
        improvement_values = [self.pipeline_stats['improvement_mm']]
        
        bars = ax8.bar(improvement_stages, improvement_values, color='purple', alpha=0.8)
        ax8.set_ylabel('Improvement [mm]')
        ax8.set_title('Registration Improvement')
        ax8.grid(True, alpha=0.3)
        
        for bar, value in zip(bars, improvement_values):
            ax8.text(bar.get_x() + bar.get_width()/2, bar.get_height() + 0.01,
                    f'{value:.3f}mm\n({self.pipeline_stats["improvement_ratio"]:.1f}%)', 
                    ha='center', va='bottom', fontweight='bold')
        
        # 添加总标题
        fig.suptitle('Four-Stage Registration Pipeline: SVD → RANSAC → Adaptive Point Selection → GPR\n' + 
                    f'Final RMSE: {self.pipeline_stats["final_rmse"]*1000:.3f}mm | ' +
                    f'Target: {TARGET_RMSE*1000:.2f}mm | ' +
                    f'Success: {"YES" if self.pipeline_stats["success"] else "NO"}', 
                    fontsize=16, fontweight='bold')
        
        plt.savefig(os.path.join(img_dir, 'comprehensive_pipeline_results.png'), 
                   dpi=300, bbox_inches='tight')
        plt.close()
        
        logger.info("Comprehensive visualization saved: comprehensive_pipeline_results.png")

def process_laser_tracker_data(input_file):
    """处理激光跟踪器数据"""
    logger.info(f"Processing laser tracker data from {input_file}")
    
    try:
        df = pd.read_csv(input_file)
        logger.info(f"Loaded {len(df)} data points")
        return df
    except Exception as e:
        logger.error(f"Error processing laser tracker data: {e}")
        return None

def main():
    """主函数"""
    logger.info("Starting SVD → RANSAC → Adaptive Point Selection → GPR Four-Stage Registration Pipeline")
    logger.info(f"Scale factor: 1:{SCALE_FACTOR}")
    logger.info(f"Target RMSE: {TARGET_RMSE*1000:.2f}mm")
    logger.info("Pipeline Features:")
    logger.info("  - Stage 1: SVD coarse alignment (global rigid transformation)")
    logger.info("  - Stage 2: RANSAC inlier selection (reliable data filtering)")  
    logger.info("  - Stage 2.5: Adaptive voxel grid point selection (intelligent downsampling)")
    logger.info("  - Stage 3: GPR fine registration (precise non-rigid deformation)")
    
    global LASER_TRACKER_CSV
    if len(sys.argv) > 1:
        LASER_TRACKER_CSV = sys.argv[1]
    
    if not os.path.exists(LASER_TRACKER_CSV):
        logger.error(f"Laser tracker data file {LASER_TRACKER_CSV} not found.")
        logger.info("Usage: python script.py <path_to_laser_tracker.csv>")
        return
    
    # 处理激光跟踪器数据
    logger.info("Loading and processing laser tracker data...")
    df = process_laser_tracker_data(LASER_TRACKER_CSV)
    
    if df is None:
        logger.error("Failed to process laser tracker data")
        return
    
    # 准备数据
    positions_A = [[row[ROW_X], 0, row[ROW_Y]] for _, row in df.iterrows()]
    positions_B = [[row["laser_x"], 0, row["laser_z"]] for _, row in df.iterrows()]
    
    logger.info(f"Loaded {len(positions_A)} point pairs for registration")
    
    # 运行四阶段配准流水线
    pipeline = ThreeStageRegistrationPipeline(positions_A, positions_B)
    final_rmse_results = pipeline.run_complete_pipeline()
    
    # 输出最终结果
    if final_rmse_results is not None:
        success = final_rmse_results['overall_rmse'] <= TARGET_RMSE
        
        logger.info("\n" + "="*80)
        logger.info("FINAL RESULTS SUMMARY")
        logger.info("="*80)
        
        if success:
            logger.info("SUCCESS: Four-stage registration completed successfully!")
            logger.info(f"Final RMSE: {final_rmse_results['overall_rmse']*1000:.3f}mm <= Target: {TARGET_RMSE*1000:.2f}mm")
        else:
            logger.info("PARTIAL SUCCESS: Registration completed but target not achieved")
            logger.info(f"Final RMSE: {final_rmse_results['overall_rmse']*1000:.3f}mm > Target: {TARGET_RMSE*1000:.2f}mm")
            logger.info(f"Deficit: {(final_rmse_results['overall_rmse'] - TARGET_RMSE)*1000:.3f}mm")
        
        logger.info(f"Lateral RMSE: {final_rmse_results['lateral_rmse']*1000:.3f}mm")
        logger.info(f"Longitudinal RMSE: {final_rmse_results['longitudinal_rmse']*1000:.3f}mm")
        logger.info(f"Processing time: {pipeline.pipeline_stats['processing_time']:.2f}s")
        logger.info(f"Real-world equivalent: {final_rmse_results['overall_rmse']*SCALE_FACTOR*100:.2f}cm")
        
        # 保存最终结果摘要
        summary_file = os.path.join(output_dir, "FINAL_RESULTS_SUMMARY.txt")
        with open(summary_file, "w") as f:
            f.write("FOUR-STAGE REGISTRATION FINAL RESULTS\n")
            f.write("=" * 60 + "\n")
            f.write(f"Timestamp: {datetime.datetime.now()}\n")
            f.write(f"Pipeline: SVD → RANSAC → Adaptive Point Selection → GPR\n")
            f.write(f"Success: {'YES' if success else 'NO'}\n")
            f.write(f"Final RMSE: {final_rmse_results['overall_rmse']*1000:.3f}mm\n")
            f.write(f"Target RMSE: {TARGET_RMSE*1000:.2f}mm\n")
            f.write(f"Scale Factor: 1:{SCALE_FACTOR}\n")
            f.write(f"Real-world Equivalent: {final_rmse_results['overall_rmse']*SCALE_FACTOR*100:.2f}cm\n")
            f.write(f"Processing Time: {pipeline.pipeline_stats['processing_time']:.2f}s\n")
            f.write(f"Points Processed: {pipeline.pipeline_stats['total_points']}\n")
            f.write(f"Improvement: {pipeline.pipeline_stats['improvement_mm']:.3f}mm\n")
            f.write(f"GPR Training Points: {pipeline.pipeline_stats['gpr_training_points']}\n")
            
            # 添加自适应选择的详细信息
            adaptive_stats = pipeline.pipeline_stats['stage2_5_stats']
            f.write(f"Adaptive Selection Success: {'Yes' if adaptive_stats['success'] else 'No'}\n")
            f.write(f"Selection Iterations: {adaptive_stats['iterations_used']}/{adaptive_stats['max_iterations']}\n")
            f.write(f"Initial Grid Size: {adaptive_stats['initial_grid_size']:.6f}m\n")
            f.write(f"Final Grid Size: {adaptive_stats['final_grid_size']:.6f}m\n")
            f.write(f"Target vs Actual Points: {adaptive_stats['target_points']} vs {adaptive_stats['final_points']}\n")
            f.write(f"Selection Deviation: {adaptive_stats['final_deviation']} points\n")
            selection_ratio = adaptive_stats['final_points'] / adaptive_stats['input_points']
            f.write(f"Point Selection Ratio: {selection_ratio*100:.1f}%\n")
        
        logger.info(f"Final summary saved to: {summary_file}")
        
    else:
        logger.error("Four-stage registration pipeline failed")
    
    logger.info("Four-stage registration processing complete")

if __name__ == "__main__":
    main()