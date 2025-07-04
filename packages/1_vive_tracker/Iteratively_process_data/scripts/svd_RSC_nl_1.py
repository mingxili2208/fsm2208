#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Enhanced Tracker data processing script with SVD → RANSAC → GPR Pipeline
Three-Stage Registration: SVD (Coarse Alignment) → RANSAC (Inlier Selection) → GPR (Fine Registration)
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
LASER_TRACKER_CSV = os.path.join(parent_dir, "data/combined_corrected_tracker_data_1.csv")
csv_filename = os.path.basename(LASER_TRACKER_CSV)
output_dir = os.path.join(result_dir, f"SVD_RANSAC_GPR_{csv_filename}_{timestamp}_1to32_scale_RMSE_{TARGET_RMSE*1000:.1f}mm")
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
    Stage 2: RANSAC内点筛选
    RANSAC充当"聪明的门卫"，为Stage 3筛选出可靠的训练数据
    """
    def __init__(self, max_iterations=1000, distance_threshold=0.005, min_inliers_ratio=0.6):
        self.max_iterations = max_iterations
        self.distance_threshold = distance_threshold  # 5mm阈值
        self.min_inliers_ratio = min_inliers_ratio
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
        logger.info("=== Stage 2: RANSAC Inlier Selection ===")
        
        svd_transformed_A = np.array(svd_transformed_A)
        residual_vectors = np.array(residual_vectors)
        
        logger.info(f"Input: {len(svd_transformed_A)} points with residuals")
        logger.info(f"RANSAC parameters:")
        logger.info(f"  Max iterations: {self.max_iterations}")
        logger.info(f"  Distance threshold: {self.distance_threshold*1000:.1f}mm")
        logger.info(f"  Min inliers ratio: {self.min_inliers_ratio}")
        
        # 只在XZ平面进行RANSAC
        source_xz = svd_transformed_A[:, [0, 2]]
        residual_xz = residual_vectors[:, [0, 2]]
        
        n_points = len(source_xz)
        min_inliers = int(n_points * self.min_inliers_ratio)
        
        best_inliers = []
        best_model = None
        best_score = 0
        
        logger.info("Starting RANSAC iterations...")
        
        for iteration in range(self.max_iterations):
            # 随机选择少量样本点用于模型拟合
            sample_size = min(10, n_points // 10)  # 自适应样本大小
            sample_indices = np.random.choice(n_points, sample_size, replace=False)
            
            sample_source = source_xz[sample_indices]
            sample_residual = residual_xz[sample_indices]
            
            # 用样本训练一个简单的非刚性模型（多项式模型）
            try:
                model_params = self._fit_simple_polynomial_model(sample_source, sample_residual)
                if model_params is None:
                    continue
                
                # 用模型预测所有点的残差
                predicted_residuals = self._predict_polynomial_model(source_xz, model_params)
                
                # 计算预测误差
                prediction_errors = np.linalg.norm(residual_xz - predicted_residuals, axis=1)
                
                # 统计内点
                inlier_mask = prediction_errors < self.distance_threshold
                current_inliers = np.where(inlier_mask)[0]
                
                # 评估当前模型
                if len(current_inliers) > best_score and len(current_inliers) >= min_inliers:
                    best_score = len(current_inliers)
                    best_inliers = current_inliers.copy()
                    best_model = model_params.copy()
                    
                    logger.info(f"  Iteration {iteration+1}: Found {len(current_inliers)} inliers ({len(current_inliers)/n_points*100:.1f}%)")
                
            except Exception as e:
                continue
        
        if len(best_inliers) == 0:
            logger.warning("RANSAC failed to find sufficient inliers, using all points")
            best_inliers = np.arange(n_points)
        
        inlier_ratio = len(best_inliers) / n_points
        
        # 保存RANSAC统计信息
        self.ransac_stats = {
            'total_points': n_points,
            'inlier_count': len(best_inliers),
            'inlier_ratio': inlier_ratio,
            'outlier_count': n_points - len(best_inliers),
            'iterations_used': self.max_iterations,
            'distance_threshold': self.distance_threshold,
            'best_model': best_model
        }
        
        logger.info("RANSAC Results:")
        logger.info(f"  Total points: {n_points}")
        logger.info(f"  Inliers found: {len(best_inliers)} ({inlier_ratio*100:.1f}%)")
        logger.info(f"  Outliers removed: {n_points - len(best_inliers)} ({(1-inlier_ratio)*100:.1f}%)")
        
        # 可视化RANSAC结果
        self._visualize_ransac_results(svd_transformed_A, residual_vectors, best_inliers)
        
        logger.info("Stage 2 (RANSAC) completed successfully")
        
        return best_inliers, best_model
    
    def _fit_simple_polynomial_model(self, source_points, residual_vectors):
        """拟合简单的多项式模型作为RANSAC的临时模型"""
        try:
            x = source_points[:, 0]
            z = source_points[:, 1]
            
            # 构建特征矩阵（二次多项式）
            # [1, x, z, x^2, z^2, xz]
            features = np.column_stack([
                np.ones(len(x)),
                x, z,
                x**2, z**2, x*z
            ])
            
            # 分别拟合X和Z方向的残差
            coeffs_x = np.linalg.lstsq(features, residual_vectors[:, 0], rcond=None)[0]
            coeffs_z = np.linalg.lstsq(features, residual_vectors[:, 1], rcond=None)[0]
            
            return {
                'coeffs_x': coeffs_x,
                'coeffs_z': coeffs_z,
                'model_type': 'polynomial_2nd_order'
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
    
    def _visualize_ransac_results(self, svd_transformed_A, residual_vectors, inlier_indices):
        """可视化RANSAC结果"""
        fig, axes = plt.subplots(2, 3, figsize=(18, 12))
        
        # 创建内点/外点掩码
        n_points = len(svd_transformed_A)
        inlier_mask = np.zeros(n_points, dtype=bool)
        inlier_mask[inlier_indices] = True
        outlier_mask = ~inlier_mask
        
        # 内点外点分布
        ax1 = axes[0, 0]
        if np.sum(inlier_mask) > 0:
            ax1.scatter(svd_transformed_A[inlier_mask, 0], svd_transformed_A[inlier_mask, 2], 
                       c='green', alpha=0.7, s=30, label=f'Inliers ({np.sum(inlier_mask)})')
        if np.sum(outlier_mask) > 0:
            ax1.scatter(svd_transformed_A[outlier_mask, 0], svd_transformed_A[outlier_mask, 2], 
                       c='red', alpha=0.7, s=30, label=f'Outliers ({np.sum(outlier_mask)})')
        ax1.set_xlabel('X [m]')
        ax1.set_ylabel('Z [m]')
        ax1.set_title('RANSAC Inlier/Outlier Classification')
        ax1.legend()
        ax1.grid(True, alpha=0.3)
        ax1.axis('equal')
        
        # 残差大小对比
        ax2 = axes[0, 1]
        residual_magnitudes = np.sqrt(np.sum(residual_vectors[:, [0, 2]]**2, axis=1))
        if np.sum(inlier_mask) > 0:
            ax2.hist(residual_magnitudes[inlier_mask]*1000, bins=20, alpha=0.7, 
                    color='green', label='Inliers', density=True)
        if np.sum(outlier_mask) > 0:
            ax2.hist(residual_magnitudes[outlier_mask]*1000, bins=20, alpha=0.7, 
                    color='red', label='Outliers', density=True)
        ax2.axvline(x=self.distance_threshold*1000, color='black', linestyle='--', 
                   label=f'Threshold: {self.distance_threshold*1000:.1f}mm')
        ax2.set_xlabel('Residual Magnitude [mm]')
        ax2.set_ylabel('Density')
        ax2.set_title('Residual Distribution: Inliers vs Outliers')
        ax2.legend()
        ax2.grid(True, alpha=0.3)
        
        # RANSAC统计饼图
        ax3 = axes[0, 2]
        sizes = [np.sum(inlier_mask), np.sum(outlier_mask)]
        labels = ['Inliers', 'Outliers']
        colors = ['green', 'red']
        ax3.pie(sizes, labels=labels, colors=colors, autopct='%1.1f%%', startangle=90)
        ax3.set_title('RANSAC Classification Results')
        
        # X方向残差分类
        ax4 = axes[1, 0]
        if np.sum(inlier_mask) > 0:
            ax4.scatter(svd_transformed_A[inlier_mask, 0], residual_vectors[inlier_mask, 0]*1000, 
                       c='green', alpha=0.6, s=20, label='Inliers')
        if np.sum(outlier_mask) > 0:
            ax4.scatter(svd_transformed_A[outlier_mask, 0], residual_vectors[outlier_mask, 0]*1000, 
                       c='red', alpha=0.6, s=20, label='Outliers')
        ax4.set_xlabel('X Position [m]')
        ax4.set_ylabel('X Residual [mm]')
        ax4.set_title('X-Direction: Inliers vs Outliers')
        ax4.legend()
        ax4.grid(True, alpha=0.3)
        ax4.axhline(y=0, color='black', linestyle='-', alpha=0.3)
        
        # Z方向残差分类
        ax5 = axes[1, 1]
        if np.sum(inlier_mask) > 0:
            ax5.scatter(svd_transformed_A[inlier_mask, 2], residual_vectors[inlier_mask, 2]*1000, 
                       c='green', alpha=0.6, s=20, label='Inliers')
        if np.sum(outlier_mask) > 0:
            ax5.scatter(svd_transformed_A[outlier_mask, 2], residual_vectors[outlier_mask, 2]*1000, 
                       c='red', alpha=0.6, s=20, label='Outliers')
        ax5.set_xlabel('Z Position [m]')
        ax5.set_ylabel('Z Residual [mm]')
        ax5.set_title('Z-Direction: Inliers vs Outliers')
        ax5.legend()
        ax5.grid(True, alpha=0.3)
        ax5.axhline(y=0, color='black', linestyle='-', alpha=0.3)
        
        # 空间分布的分类结果
        ax6 = axes[1, 2]
        # 创建分类标签用于着色
        classification = np.zeros(n_points)
        classification[outlier_mask] = 1  # 外点标记为1
        
        scatter = ax6.scatter(svd_transformed_A[:, 0], svd_transformed_A[:, 2], 
                            c=classification, cmap='RdYlGn_r', s=30, alpha=0.7)
        ax6.set_xlabel('X [m]')
        ax6.set_ylabel('Z [m]')
        ax6.set_title('Spatial Distribution of Classification')
        cbar = plt.colorbar(scatter, ax=ax6)
        cbar.set_label('0=Inlier, 1=Outlier')
        ax6.grid(True, alpha=0.3)
        ax6.axis('equal')
        
        plt.tight_layout()
        plt.savefig(os.path.join(img_dir, 'stage2_ransac_results.png'), dpi=300, bbox_inches='tight')
        plt.close()
        
        logger.info("RANSAC visualization saved: stage2_ransac_results.png")

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
        
    def learn_and_apply_deformation(self, svd_transformed_A, residual_vectors, inlier_indices, target_B):
        """
        学习非刚性形变并应用到所有点
        
        Args:
        - svd_transformed_A: SVD变换后的源点云
        - residual_vectors: SVD残差向量
        - inlier_indices: RANSAC筛选的内点索引
        - target_B: 目标点云（用于最终评估）
        
        Returns:
        - final_transformed_A: 最终变换后的源点云
        - gpr_stats: GPR统计信息
        """
        logger.info("=== Stage 3: GPR Fine Registration ===")
        
        svd_transformed_A = np.array(svd_transformed_A)
        residual_vectors = np.array(residual_vectors)
        target_B = np.array(target_B)
        
        # 使用内点数据训练GPR
        inlier_source = svd_transformed_A[inlier_indices]
        inlier_residuals = residual_vectors[inlier_indices]
        
        logger.info(f"Training GPR with {len(inlier_indices)} inlier points")
        logger.info(f"Total points for final prediction: {len(svd_transformed_A)}")
        
        # 训练GPR模型
        success = self._train_gpr_models(inlier_source, inlier_residuals)
        
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
    
    def _train_gpr_models(self, inlier_source, inlier_residuals):
        """训练GPR模型"""
        logger.info("Training Gaussian Process Regression models...")
        
        # 准备训练数据（只使用XZ坐标）
        X_train = inlier_source[:, [0, 2]]
        y_train_x = inlier_residuals[:, 0]
        y_train_z = inlier_residuals[:, 2]
        
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
        final_errors = np.sqrt(np.sum((target_B - final_transformed_A)**2, axis=1))
        
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
    三阶段配准流水线：SVD → RANSAC → GPR
    """
    def __init__(self, positions_A, positions_B):
        self.positions_A = np.array(positions_A)
        self.positions_B = np.array(positions_B)
        self.stage1 = Stage1_SVDCoarseAlignment()
        self.stage2 = Stage2_RANSACInlierSelection()
        self.stage3 = Stage3_GPRFineRegistration()
        self.pipeline_stats = {}
        
    def run_complete_pipeline(self):
        """运行完整的三阶段配准流水线"""
        logger.info("=" * 80)
        logger.info("STARTING THREE-STAGE REGISTRATION PIPELINE")
        logger.info("SVD (Coarse Alignment) → RANSAC (Inlier Selection) → GPR (Fine Registration)")
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
        
        # Stage 3: GPR精配准
        logger.info("\n" + "="*50)
        final_transformed_A, final_rmse_results = self.stage3.learn_and_apply_deformation(
            svd_transformed_A, residual_vectors, inlier_indices, clean_B
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
        logger.info("THREE-STAGE REGISTRATION PIPELINE COMPLETED")
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
            'stage3_stats': self.stage3.gpr_stats
        }
        
        logger.info("\n" + "="*60)
        logger.info("PIPELINE SUMMARY")
        logger.info("="*60)
        logger.info(f"Processing time: {processing_time:.2f} seconds")
        logger.info(f"Total points processed: {len(original_A)}")
        logger.info(f"RANSAC inlier ratio: {self.stage2.ransac_stats['inlier_ratio']*100:.1f}%")
        logger.info(f"GPR training points: {self.stage3.gpr_stats['training_points']}")
        logger.info("")
        logger.info("ACCURACY RESULTS:")
        logger.info(f"  Target RMSE: {TARGET_RMSE*1000:.2f}mm")
        logger.info(f"  SVD RMSE: {svd_rmse*1000:.3f}mm")
        logger.info(f"  Final RMSE: {final_rmse_results['overall_rmse']*1000:.3f}mm")
        logger.info(f"  Improvement: {improvement:.3f}mm ({improvement_ratio:.1f}%)")
        logger.info(f"  SUCCESS: {'YES' if success else 'NO'}")
        
        if success:
            logger.info(f"✓ Target accuracy achieved!")
        else:
            logger.info(f"✗ Target accuracy not achieved (deficit: {(final_rmse_results['overall_rmse'] - TARGET_RMSE)*1000:.3f}mm)")
    
    def _generate_comprehensive_report(self):
        """生成综合报告"""
        report_file = os.path.join(result_dir, "three_stage_registration_report.txt")
        
        with open(report_file, "w") as f:
            f.write("THREE-STAGE REGISTRATION PIPELINE REPORT\n")
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
        ax1.set_title('Three-Stage Registration Pipeline Overview')
        ax1.legend()
        ax1.grid(True, alpha=0.3)
        ax1.axis('equal')
        
        # 2. RMSE进展
        ax2 = fig.add_subplot(gs[0, 2:])
        stages = ['SVD Only', 'Final (SVD+RANSAC+GPR)']
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
        ax4.pie([inlier_ratio, outlier_ratio], labels=['Inliers', 'Outliers'], 
               colors=['green', 'red'], autopct='%1.1f%%', startangle=90)
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
        
        # 6. 处理统计
        ax6 = fig.add_subplot(gs[1, 3])
        ax6.text(0.1, 0.9, f"Processing Time: {self.pipeline_stats['processing_time']:.2f}s", 
                transform=ax6.transAxes, fontsize=12, fontweight='bold')
        ax6.text(0.1, 0.8, f"Total Points: {self.pipeline_stats['total_points']}", 
                transform=ax6.transAxes, fontsize=11)
        ax6.text(0.1, 0.7, f"Inlier Ratio: {inlier_ratio*100:.1f}%", 
                transform=ax6.transAxes, fontsize=11)
        ax6.text(0.1, 0.6, f"GPR Training Points: {self.stage3.gpr_stats['training_points']}", 
                transform=ax6.transAxes, fontsize=11)
        ax6.text(0.1, 0.5, f"Final RMSE: {self.pipeline_stats['final_rmse']*1000:.3f}mm", 
                transform=ax6.transAxes, fontsize=11, color='green', fontweight='bold')
        ax6.text(0.1, 0.4, f"Target RMSE: {TARGET_RMSE*1000:.2f}mm", 
                transform=ax6.transAxes, fontsize=11, color='red')
        ax6.text(0.1, 0.3, f"Success: {'YES' if self.pipeline_stats['success'] else 'NO'}", 
                transform=ax6.transAxes, fontsize=12, 
                color='green' if self.pipeline_stats['success'] else 'red', fontweight='bold')
        ax6.text(0.1, 0.2, f"Improvement: {self.pipeline_stats['improvement_mm']:.3f}mm", 
                transform=ax6.transAxes, fontsize=11)
        ax6.text(0.1, 0.1, f"Real-world Equiv: {self.pipeline_stats['final_rmse']*SCALE_FACTOR*100:.2f}cm", 
                transform=ax6.transAxes, fontsize=11)
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
        fig.suptitle('Three-Stage Registration Pipeline: SVD → RANSAC → GPR\n' + 
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
    logger.info("Starting SVD → RANSAC → GPR Three-Stage Registration Pipeline")
    logger.info(f"Scale factor: 1:{SCALE_FACTOR}")
    logger.info(f"Target RMSE: {TARGET_RMSE*1000:.2f}mm")
    logger.info("Pipeline Features:")
    logger.info("  - Stage 1: SVD coarse alignment (global rigid transformation)")
    logger.info("  - Stage 2: RANSAC inlier selection (reliable data filtering)")  
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
    
    # 运行三阶段配准流水线
    pipeline = ThreeStageRegistrationPipeline(positions_A, positions_B)
    final_rmse_results = pipeline.run_complete_pipeline()
    
    # 输出最终结果
    if final_rmse_results is not None:
        success = final_rmse_results['overall_rmse'] <= TARGET_RMSE
        
        logger.info("\n" + "="*80)
        logger.info("FINAL RESULTS SUMMARY")
        logger.info("="*80)
        
        if success:
            logger.info("🎉 SUCCESS: Three-stage registration completed successfully!")
            logger.info(f"✓ Final RMSE: {final_rmse_results['overall_rmse']*1000:.3f}mm ≤ Target: {TARGET_RMSE*1000:.2f}mm")
        else:
            logger.info("⚠️  PARTIAL SUCCESS: Registration completed but target not achieved")
            logger.info(f"✗ Final RMSE: {final_rmse_results['overall_rmse']*1000:.3f}mm > Target: {TARGET_RMSE*1000:.2f}mm")
            logger.info(f"  Deficit: {(final_rmse_results['overall_rmse'] - TARGET_RMSE)*1000:.3f}mm")
        
        logger.info(f"Lateral RMSE: {final_rmse_results['lateral_rmse']*1000:.3f}mm")
        logger.info(f"Longitudinal RMSE: {final_rmse_results['longitudinal_rmse']*1000:.3f}mm")
        logger.info(f"Processing time: {pipeline.pipeline_stats['processing_time']:.2f}s")
        logger.info(f"Real-world equivalent: {final_rmse_results['overall_rmse']*SCALE_FACTOR*100:.2f}cm")
        
        # 保存最终结果摘要
        summary_file = os.path.join(output_dir, "FINAL_RESULTS_SUMMARY.txt")
        with open(summary_file, "w") as f:
            f.write("THREE-STAGE REGISTRATION FINAL RESULTS\n")
            f.write("=" * 60 + "\n")
            f.write(f"Timestamp: {datetime.datetime.now()}\n")
            f.write(f"Pipeline: SVD → RANSAC → GPR\n")
            f.write(f"Success: {'YES' if success else 'NO'}\n")
            f.write(f"Final RMSE: {final_rmse_results['overall_rmse']*1000:.3f}mm\n")
            f.write(f"Target RMSE: {TARGET_RMSE*1000:.2f}mm\n")
            f.write(f"Scale Factor: 1:{SCALE_FACTOR}\n")
            f.write(f"Real-world Equivalent: {final_rmse_results['overall_rmse']*SCALE_FACTOR*100:.2f}cm\n")
            f.write(f"Processing Time: {pipeline.pipeline_stats['processing_time']:.2f}s\n")
            f.write(f"Points Processed: {pipeline.pipeline_stats['total_points']}\n")
            f.write(f"Improvement: {pipeline.pipeline_stats['improvement_mm']:.3f}mm\n")
        
        logger.info(f"Final summary saved to: {summary_file}")
        
    else:
        logger.error("Three-stage registration pipeline failed")
    
    logger.info("Three-stage registration processing complete")

if __name__ == "__main__":
    main()