#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Enhanced Tracker data processing script with Adaptive Voxel Sampling + SVD → RANSAC → GPR Pipeline
Features:
1. Adaptive voxel-based sampling for train/test split
2. Three-Stage Registration: SVD (Coarse Alignment) → RANSAC (Inlier Selection) → GPR (Fine Registration)
3. Comprehensive evaluation on test set with multiple model comparisons
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

# 训练测试集分割比例
TRAIN_RATIO = 0.8
TEST_RATIO = 0.2

# Create output directory
script_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(script_dir)
result_dir = os.path.join(parent_dir, "results")
# File paths
LASER_TRACKER_CSV = os.path.join(parent_dir, "data/combined_corrected_tracker_data_1.csv")
csv_filename = os.path.basename(LASER_TRACKER_CSV)
output_dir = os.path.join(result_dir, f"Enhanced_VoxelSampling_SVD_RANSAC_GPR_{csv_filename}_{timestamp}_1to32_scale_RMSE_{TARGET_RMSE*1000:.1f}mm")
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
log_file = os.path.join(log_dir, "enhanced_tracker_process.log")
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

class AdaptiveVoxelSampler:
    """
    自适应体素网格采样器
    根据数据分布自动确定最优网格大小，并按比例分配训练测试集
    """
    def __init__(self, train_ratio=0.8, min_points_per_voxel=5):
        self.train_ratio = train_ratio
        self.min_points_per_voxel = min_points_per_voxel
        self.optimal_voxel_size = None
        self.voxel_stats = {}
        
    def adaptive_train_test_split(self, positions_A, positions_B):
        """
        基于自适应体素网格的训练测试集划分
        
        Args:
        - positions_A: 源点云
        - positions_B: 目标点云
        
        Returns:
        - train_indices, test_indices: 训练集和测试集的索引
        - voxel_info: 体素网格信息
        """
        logger.info("=== Adaptive Voxel-Based Train/Test Split ===")
        
        positions_A = np.array(positions_A)
        positions_B = np.array(positions_B)
        
        # 只考虑XZ平面的分布
        points_xz = positions_A[:, [0, 2]]
        
        # 计算空间范围
        x_min, x_max = np.min(points_xz[:, 0]), np.max(points_xz[:, 0])
        z_min, z_max = np.min(points_xz[:, 1]), np.max(points_xz[:, 1])
        
        logger.info(f"Data spatial range: X[{x_min:.3f}, {x_max:.3f}], Z[{z_min:.3f}, {z_max:.3f}]")
        logger.info(f"Total points: {len(positions_A)}")
        
        # 寻找最优体素大小
        optimal_voxel_size = self._find_optimal_voxel_size(points_xz, x_min, x_max, z_min, z_max)
        self.optimal_voxel_size = optimal_voxel_size
        
        # 使用最优体素大小进行采样
        train_indices, test_indices, voxel_info = self._perform_voxel_sampling(
            points_xz, x_min, x_max, z_min, z_max, optimal_voxel_size
        )
        
        # 验证分割结果
        self._validate_split(train_indices, test_indices, points_xz)
        
        # 可视化体素网格和分割结果
        self._visualize_voxel_sampling(points_xz, train_indices, test_indices, voxel_info)
        
        logger.info(f"Adaptive voxel sampling completed:")
        logger.info(f"  Optimal voxel size: {optimal_voxel_size:.6f}")
        logger.info(f"  Training set: {len(train_indices)} points ({len(train_indices)/len(positions_A)*100:.1f}%)")
        logger.info(f"  Test set: {len(test_indices)} points ({len(test_indices)/len(positions_A)*100:.1f}%)")
        
        return train_indices, test_indices, voxel_info
    
    def _find_optimal_voxel_size(self, points_xz, x_min, x_max, z_min, z_max):
        """寻找最优体素大小"""
        logger.info("Searching for optimal voxel size...")
        
        # 计算候选体素大小
        x_range = x_max - x_min
        z_range = z_max - z_min
        total_area = x_range * z_range
        n_points = len(points_xz)
        
        # 基于点密度计算基础体素大小
        base_voxel_area = total_area / (n_points / 20)  # 平均每个体素20个点
        base_voxel_size = np.sqrt(base_voxel_area)
        
        # 生成候选体素大小序列
        size_factors = np.linspace(0.5, 3.0, 15)  # 从0.5倍到3倍
        candidate_sizes = base_voxel_size * size_factors
        
        best_score = -np.inf
        best_size = base_voxel_size
        size_scores = []
        
        for voxel_size in candidate_sizes:
            score = self._evaluate_voxel_size(points_xz, x_min, x_max, z_min, z_max, voxel_size)
            size_scores.append(score)
            
            if score > best_score:
                best_score = score
                best_size = voxel_size
        
        # 保存评估结果
        self.voxel_stats = {
            'candidate_sizes': candidate_sizes,
            'scores': size_scores,
            'best_size': best_size,
            'best_score': best_score,
            'base_size': base_voxel_size
        }
        
        logger.info(f"  Base voxel size: {base_voxel_size:.6f}")
        logger.info(f"  Optimal voxel size: {best_size:.6f}")
        logger.info(f"  Best score: {best_score:.3f}")
        
        return best_size
    
    def _evaluate_voxel_size(self, points_xz, x_min, x_max, z_min, z_max, voxel_size):
        """评估体素大小的质量"""
        # 创建体素网格
        n_x = max(1, int(np.ceil((x_max - x_min) / voxel_size)))
        n_z = max(1, int(np.ceil((z_max - z_min) / voxel_size)))
        
        # 计算每个点的体素索引
        voxel_indices_x = np.floor((points_xz[:, 0] - x_min) / voxel_size).astype(int)
        voxel_indices_z = np.floor((points_xz[:, 1] - z_min) / voxel_size).astype(int)
        
        # 确保索引在有效范围内
        voxel_indices_x = np.clip(voxel_indices_x, 0, n_x - 1)
        voxel_indices_z = np.clip(voxel_indices_z, 0, n_z - 1)
        
        # 计算每个体素的点数
        voxel_counts = {}
        for i, (vx, vz) in enumerate(zip(voxel_indices_x, voxel_indices_z)):
            key = (vx, vz)
            if key not in voxel_counts:
                voxel_counts[key] = 0
            voxel_counts[key] += 1
        
        if len(voxel_counts) == 0:
            return -np.inf
        
        counts = list(voxel_counts.values())
        
        # 评估指标
        # 1. 分布均匀性（方差越小越好）
        uniformity_score = -np.var(counts) / (np.mean(counts) + 1e-6)
        
        # 2. 覆盖率（有效体素比例）
        total_possible_voxels = n_x * n_z
        coverage_score = len(voxel_counts) / total_possible_voxels
        
        # 3. 最小点数惩罚（体素中点数太少的惩罚）
        min_points_penalty = -sum([1 for c in counts if c < self.min_points_per_voxel])
        
        # 4. 体素数量合理性（太多或太少都不好）
        ideal_voxel_count = len(points_xz) / 15  # 理想情况下每个体素15个点
        voxel_count_score = -abs(len(voxel_counts) - ideal_voxel_count) / ideal_voxel_count
        
        # 综合评分
        total_score = (0.4 * uniformity_score + 
                      0.3 * coverage_score + 
                      0.2 * min_points_penalty / len(points_xz) +
                      0.1 * voxel_count_score)
        
        return total_score
    
    def _perform_voxel_sampling(self, points_xz, x_min, x_max, z_min, z_max, voxel_size):
        """执行体素网格采样"""
        # 创建体素网格
        n_x = max(1, int(np.ceil((x_max - x_min) / voxel_size)))
        n_z = max(1, int(np.ceil((z_max - z_min) / voxel_size)))
        
        # 计算每个点的体素索引
        voxel_indices_x = np.floor((points_xz[:, 0] - x_min) / voxel_size).astype(int)
        voxel_indices_z = np.floor((points_xz[:, 1] - z_min) / voxel_size).astype(int)
        
        # 确保索引在有效范围内
        voxel_indices_x = np.clip(voxel_indices_x, 0, n_x - 1)
        voxel_indices_z = np.clip(voxel_indices_z, 0, n_z - 1)
        
        # 按体素分组点
        voxel_groups = {}
        for i, (vx, vz) in enumerate(zip(voxel_indices_x, voxel_indices_z)):
            key = (vx, vz)
            if key not in voxel_groups:
                voxel_groups[key] = []
            voxel_groups[key].append(i)
        
        # 在每个体素内按比例分配训练测试集
        train_indices = []
        test_indices = []
        
        for voxel_key, point_indices in voxel_groups.items():
            n_points_in_voxel = len(point_indices)
            n_train = int(n_points_in_voxel * self.train_ratio)
            
            # 随机打乱体素内的点
            np.random.shuffle(point_indices)
            
            # 分配训练测试集
            train_indices.extend(point_indices[:n_train])
            test_indices.extend(point_indices[n_train:])
        
        # 准备体素信息用于可视化
        voxel_info = {
            'grid_size': (n_x, n_z),
            'voxel_size': voxel_size,
            'bounds': (x_min, x_max, z_min, z_max),
            'voxel_groups': voxel_groups,
            'voxel_indices_x': voxel_indices_x,
            'voxel_indices_z': voxel_indices_z
        }
        
        return np.array(train_indices), np.array(test_indices), voxel_info
    
    def _validate_split(self, train_indices, test_indices, points_xz):
        """验证训练测试集分割的质量"""
        # 检查是否有重叠
        overlap = set(train_indices) & set(test_indices)
        if len(overlap) > 0:
            logger.warning(f"Found {len(overlap)} overlapping indices between train and test sets!")
        
        # 检查覆盖率
        total_indices = set(train_indices) | set(test_indices)
        expected_total = len(points_xz)
        if len(total_indices) != expected_total:
            logger.warning(f"Split coverage issue: {len(total_indices)} vs expected {expected_total}")
        
        # 分析空间分布
        train_points = points_xz[train_indices]
        test_points = points_xz[test_indices]
        
        train_x_range = [np.min(train_points[:, 0]), np.max(train_points[:, 0])]
        train_z_range = [np.min(train_points[:, 1]), np.max(train_points[:, 1])]
        test_x_range = [np.min(test_points[:, 0]), np.max(test_points[:, 0])]
        test_z_range = [np.min(test_points[:, 1]), np.max(test_points[:, 1])]
        
        logger.info("Split quality validation:")
        logger.info(f"  No overlap: {'✓' if len(overlap) == 0 else '✗'}")
        logger.info(f"  Complete coverage: {'✓' if len(total_indices) == expected_total else '✗'}")
        logger.info(f"  Train X range: [{train_x_range[0]:.3f}, {train_x_range[1]:.3f}]")
        logger.info(f"  Train Z range: [{train_z_range[0]:.3f}, {train_z_range[1]:.3f}]")
        logger.info(f"  Test X range: [{test_x_range[0]:.3f}, {test_z_range[1]:.3f}]")
        logger.info(f"  Test Z range: [{test_z_range[0]:.3f}, {test_z_range[1]:.3f}]")
    
    def _visualize_voxel_sampling(self, points_xz, train_indices, test_indices, voxel_info):
        """可视化体素网格采样结果"""
        fig, axes = plt.subplots(2, 3, figsize=(18, 12))
        
        # 1. 原始数据分布
        ax1 = axes[0, 0]
        ax1.scatter(points_xz[:, 0], points_xz[:, 1], c='blue', alpha=0.6, s=20)
        ax1.set_xlabel('X [m]')
        ax1.set_ylabel('Z [m]')
        ax1.set_title(f'Original Data Distribution\n({len(points_xz)} points)')
        ax1.grid(True, alpha=0.3)
        ax1.axis('equal')
        
        # 2. 体素网格划分
        ax2 = axes[0, 1]
        x_min, x_max, z_min, z_max = voxel_info['bounds']
        n_x, n_z = voxel_info['grid_size']
        voxel_size = voxel_info['voxel_size']
        
        # 绘制网格线
        for i in range(n_x + 1):
            x = x_min + i * voxel_size
            ax2.axvline(x=x, color='gray', alpha=0.5, linewidth=0.5)
        for j in range(n_z + 1):
            z = z_min + j * voxel_size
            ax2.axhline(y=z, color='gray', alpha=0.5, linewidth=0.5)
        
        ax2.scatter(points_xz[:, 0], points_xz[:, 1], c='blue', alpha=0.6, s=15)
        ax2.set_xlabel('X [m]')
        ax2.set_ylabel('Z [m]')
        ax2.set_title(f'Voxel Grid ({n_x}×{n_z})\nVoxel size: {voxel_size:.6f}')
        ax2.grid(True, alpha=0.3)
        ax2.axis('equal')
        
        # 3. 体素大小优化过程
        ax3 = axes[0, 2]
        if hasattr(self, 'voxel_stats'):
            ax3.plot(self.voxel_stats['candidate_sizes'], self.voxel_stats['scores'], 'b-o', alpha=0.7)
            best_idx = np.argmax(self.voxel_stats['scores'])
            ax3.plot(self.voxel_stats['candidate_sizes'][best_idx], self.voxel_stats['scores'][best_idx], 
                    'ro', markersize=10, label='Optimal')
            ax3.axvline(x=self.voxel_stats['base_size'], color='green', linestyle='--', 
                       alpha=0.7, label='Base size')
        ax3.set_xlabel('Voxel Size')
        ax3.set_ylabel('Quality Score')
        ax3.set_title('Voxel Size Optimization')
        ax3.legend()
        ax3.grid(True, alpha=0.3)
        
        # 4. 训练测试集分布
        ax4 = axes[1, 0]
        train_points = points_xz[train_indices]
        test_points = points_xz[test_indices]
        
        ax4.scatter(train_points[:, 0], train_points[:, 1], c='green', alpha=0.6, s=15, label='Training')
        ax4.scatter(test_points[:, 0], test_points[:, 1], c='red', alpha=0.6, s=15, label='Test')
        ax4.set_xlabel('X [m]')
        ax4.set_ylabel('Z [m]')
        ax4.set_title(f'Train/Test Split\nTrain: {len(train_indices)}, Test: {len(test_indices)}')
        ax4.legend()
        ax4.grid(True, alpha=0.3)
        ax4.axis('equal')
        
        # 5. 每个体素的点数分布
        ax5 = axes[1, 1]
        voxel_groups = voxel_info['voxel_groups']
        voxel_point_counts = [len(points) for points in voxel_groups.values()]
        
        ax5.hist(voxel_point_counts, bins=min(20, len(voxel_groups)), alpha=0.7, color='skyblue')
        ax5.axvline(x=np.mean(voxel_point_counts), color='red', linestyle='--', 
                   label=f'Mean: {np.mean(voxel_point_counts):.1f}')
        ax5.set_xlabel('Points per Voxel')
        ax5.set_ylabel('Frequency')
        ax5.set_title('Voxel Point Distribution')
        ax5.legend()
        ax5.grid(True, alpha=0.3)
        
        # 6. 体素密度热图
        ax6 = axes[1, 2]
        density_grid = np.zeros((n_z, n_x))
        for (vx, vz), points in voxel_groups.items():
            if 0 <= vz < n_z and 0 <= vx < n_x:
                density_grid[vz, vx] = len(points)
        
        im = ax6.imshow(density_grid, cmap='viridis', origin='lower', aspect='auto')
        plt.colorbar(im, ax=ax6, label='Points per Voxel')
        ax6.set_xlabel('Voxel X Index')
        ax6.set_ylabel('Voxel Z Index')
        ax6.set_title('Voxel Density Heatmap')
        
        plt.tight_layout()
        plt.savefig(os.path.join(img_dir, 'adaptive_voxel_sampling.png'), dpi=300, bbox_inches='tight')
        plt.close()
        
        logger.info("Voxel sampling visualization saved: adaptive_voxel_sampling.png")

class Stage1_SVDCoarseAlignment:
    """
    Stage 1: SVD粗配准 (仅在训练集上进行)
    计算全局的最佳刚性变换（旋转+平移），将源点云整体对齐到目标点云
    """
    def __init__(self):
        self.T_svd = None
        self.svd_stats = {}
        
    def perform_svd_alignment(self, train_positions_A, train_positions_B):
        """
        在训练集上执行SVD粗配准
        
        Returns:
        - svd_transformed_A: 经过SVD变换后的训练集源点云
        - residual_vectors: SVD之后的训练集残差
        - T_svd: SVD变换矩阵（将用于测试集）
        """
        logger.info("=== Stage 1: SVD Coarse Alignment (Training Set Only) ===")
        
        train_positions_A = np.array(train_positions_A)
        train_positions_B = np.array(train_positions_B)
        
        logger.info(f"Training on {len(train_positions_A)} point pairs")
        
        # 只在XZ平面上进行SVD配准（Y轴始终为0）
        positions_A_xz = train_positions_A[:, [0, 2]]
        positions_B_xz = train_positions_B[:, [0, 2]]
        
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
        
        # 应用变换到训练集
        svd_transformed_A = self._apply_transformation(train_positions_A, self.T_svd)
        
        # 计算训练集残差向量
        residual_vectors = train_positions_B - svd_transformed_A
        
        # 分析SVD结果
        self._analyze_svd_results(train_positions_A, train_positions_B, svd_transformed_A, residual_vectors)
        
        logger.info("Stage 1 (SVD) completed successfully")
        logger.info(f"Output: svd_transformed_A shape: {svd_transformed_A.shape}")
        logger.info(f"Output: residual_vectors shape: {residual_vectors.shape}")
        
        return svd_transformed_A, residual_vectors, self.T_svd
    
    def apply_transformation_to_test(self, test_positions_A):
        """将SVD变换应用到测试集"""
        if self.T_svd is None:
            raise ValueError("SVD transformation not trained yet")
        
        return self._apply_transformation(test_positions_A, self.T_svd)
    
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
        
        logger.info("SVD Alignment Results (Training Set):")
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
        ax1.scatter(original_A[:, 0], original_A[:, 2], c='blue', alpha=0.6, s=30, label='Original Source (Train)')
        ax1.scatter(transformed_A[:, 0], transformed_A[:, 2], c='green', alpha=0.6, s=30, label='SVD Transformed (Train)')
        ax1.scatter(target_B[:, 0], target_B[:, 2], c='red', alpha=0.6, s=30, label='Target (Train)')
        ax1.set_xlabel('X [m]')
        ax1.set_ylabel('Z [m]')
        ax1.set_title('SVD Coarse Alignment Results (Training Set)')
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
        ax2.set_title(f'SVD Residual Vectors (×{scale_factor}) - Training')
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
        ax3.set_title('SVD Residual Distribution (Training)')
        ax3.legend()
        ax3.grid(True, alpha=0.3)
        
        # X方向残差
        ax4 = axes[1, 0]
        ax4.scatter(transformed_A[:, 0], residuals[:, 0]*1000, alpha=0.6, s=20)
        ax4.set_xlabel('X Position [m]')
        ax4.set_ylabel('X Residual [mm]')
        ax4.set_title('X-Direction Residuals (Training)')
        ax4.grid(True, alpha=0.3)
        ax4.axhline(y=0, color='red', linestyle='--', alpha=0.7)
        
        # Z方向残差
        ax5 = axes[1, 1]
        ax5.scatter(transformed_A[:, 2], residuals[:, 2]*1000, alpha=0.6, s=20)
        ax5.set_xlabel('Z Position [m]')
        ax5.set_ylabel('Z Residual [mm]')
        ax5.set_title('Z-Direction Residuals (Training)')
        ax5.grid(True, alpha=0.3)
        ax5.axhline(y=0, color='red', linestyle='--', alpha=0.7)
        
        # 残差空间分布
        ax6 = axes[1, 2]
        scatter = ax6.scatter(transformed_A[:, 0], transformed_A[:, 2], c=residual_magnitudes*1000, 
                            cmap='viridis', s=30, alpha=0.7)
        plt.colorbar(scatter, ax=ax6, label='Residual Magnitude [mm]')
        ax6.set_xlabel('X [m]')
        ax6.set_ylabel('Z [m]')
        ax6.set_title('Spatial Distribution of SVD Residuals (Training)')
        ax6.grid(True, alpha=0.3)
        ax6.axis('equal')
        
        plt.tight_layout()
        plt.savefig(os.path.join(img_dir, 'stage1_svd_results_training.png'), dpi=300, bbox_inches='tight')
        plt.close()
        
        logger.info("SVD visualization saved: stage1_svd_results_training.png")

class Stage2_RANSACInlierSelection:
    """
    Stage 2: RANSAC内点筛选 (仅在训练集上进行)
    """
    def __init__(self, max_iterations=500, sample_size=None, min_inliers_ratio=0.5, 
                 use_rbf_model=False, adaptive_threshold=True):
        self.max_iterations = max_iterations
        self.sample_size = sample_size
        self.min_inliers_ratio = min_inliers_ratio
        self.use_rbf_model = use_rbf_model
        self.adaptive_threshold = adaptive_threshold
        self.distance_threshold = None
        self.ransac_stats = {}
        
    def select_inliers(self, train_svd_transformed_A, train_residual_vectors):
        """
        在训练集上使用RANSAC筛选内点
        
        Args:
        - train_svd_transformed_A: SVD变换后的训练集源点云
        - train_residual_vectors: 训练集SVD残差向量
        
        Returns:
        - inlier_indices: 内点在训练集中的索引
        - best_model_params: 最佳简单模型参数
        """
        logger.info("=== Stage 2: RANSAC Inlier Selection (Training Set Only) ===")
        
        train_svd_transformed_A = np.array(train_svd_transformed_A)
        train_residual_vectors = np.array(train_residual_vectors)
        
        # 只在XZ平面进行RANSAC
        source_xz = train_svd_transformed_A[:, [0, 2]]
        residual_xz = train_residual_vectors[:, [0, 2]]
        
        n_points = len(source_xz)
        
        # 智能计算距离阈值
        if self.adaptive_threshold:
            self.distance_threshold = self._compute_adaptive_threshold(residual_xz)
        else:
            residual_magnitudes = np.linalg.norm(residual_xz, axis=1)
            self.distance_threshold = np.median(residual_magnitudes) * 2.5
        
        # 自适应样本量
        if self.sample_size is None:
            self.sample_size = self._compute_adaptive_sample_size(n_points)
        else:
            self.sample_size = min(self.sample_size, n_points // 2)
        
        min_inliers_count = int(n_points * self.min_inliers_ratio)
        
        logger.info(f"Training on {n_points} points with residuals")
        logger.info(f"RANSAC Parameters:")
        logger.info(f"  Max iterations: {self.max_iterations}")
        logger.info(f"  Sample size: {self.sample_size}")
        logger.info(f"  Distance threshold: {self.distance_threshold*1000:.2f}mm")
        logger.info(f"  Min inliers ratio: {self.min_inliers_ratio}")
        
        best_inlier_indices = np.arange(n_points)
        best_inlier_count = 0
        best_model = None
        best_score = -np.inf
        
        # 早停机制
        no_improvement_count = 0
        patience = 50
        
        logger.info("Starting RANSAC iterations...")
        
        iteration = 0
        for iteration in range(self.max_iterations):
            try:
                # 随机选择样本点
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
                
                # 计算模型质量分数
                current_score = self._compute_model_score(current_inlier_count, prediction_errors, 
                                                        current_inlier_mask, n_points)
                
                # 评估当前模型
                if current_score > best_score and current_inlier_count >= min_inliers_count:
                    best_score = current_score
                    best_inlier_count = current_inlier_count
                    best_inlier_indices = np.where(current_inlier_mask)[0]
                    best_model = model_params.copy()
                    no_improvement_count = 0
                    
                    if iteration % 50 == 0 or current_inlier_count > n_points * 0.95:
                        logger.info(f"  Iteration {iteration+1}: New best model - {current_inlier_count} inliers "
                                  f"({current_inlier_count/n_points*100:.1f}%), score: {current_score:.3f}")
                    
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
                continue
        
        # 结果验证和后处理
        if best_inlier_count < min_inliers_count:
            logger.warning(f"RANSAC failed to find a consensus model with at least {min_inliers_count} inliers.")
            logger.warning(f"Best model found only had {best_inlier_count} inliers. Using fallback strategy.")
            best_inlier_indices = self._fallback_strategy(source_xz, residual_xz)
        else:
            logger.info(f"RANSAC successful. Final inlier set size: {best_inlier_count}")
        
        # 精炼内点集
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
        
        logger.info("RANSAC Results (Training Set):")
        logger.info(f"  Total points: {n_points}")
        logger.info(f"  Inliers found: {len(best_inlier_indices)} ({inlier_ratio*100:.1f}%)")
        logger.info(f"  Outliers removed: {n_points - len(best_inlier_indices)} ({(1-inlier_ratio)*100:.1f}%)")
        logger.info(f"  Iterations used: {self.ransac_stats['iterations_used']}/{self.max_iterations}")
        logger.info(f"  Best model score: {best_score:.3f}")
        
        # 可视化RANSAC结果
        self._visualize_ransac_results(train_svd_transformed_A, train_residual_vectors, best_inlier_indices)
        
        logger.info("Stage 2 (RANSAC) completed")
        
        return best_inlier_indices, best_model
    
    def _compute_adaptive_threshold(self, residual_xz):
        """智能计算距离阈值"""
        residual_magnitudes = np.linalg.norm(residual_xz, axis=1)
        
        median_residual = np.median(residual_magnitudes)
        mad = np.median(np.abs(residual_magnitudes - median_residual))
        q75 = np.percentile(residual_magnitudes, 75)
        q25 = np.percentile(residual_magnitudes, 25)
        iqr = q75 - q25
        
        robust_noise_estimate = mad * 1.4826
        
        threshold_candidates = [
            median_residual * 2.5,
            robust_noise_estimate * 3.0,
            q75 + 1.5 * iqr,
        ]
        
        threshold = np.median(threshold_candidates)
        
        logger.info(f"Adaptive threshold computation:")
        logger.info(f"  Median residual: {median_residual*1000:.2f}mm")
        logger.info(f"  MAD-based noise estimate: {robust_noise_estimate*1000:.2f}mm")
        logger.info(f"  IQR: {iqr*1000:.2f}mm")
        logger.info(f"  Selected threshold: {threshold*1000:.2f}mm")
        
        return threshold
    
    def _compute_adaptive_sample_size(self, n_points):
        """自适应计算样本大小"""
        base_size = min(100, n_points // 20)
        
        if n_points < 1000:
            sample_size = min(50, n_points // 10)
        elif n_points < 5000:
            sample_size = base_size
        else:
            sample_size = min(150, n_points // 30)
        
        sample_size = max(20, min(sample_size, n_points // 2))
        
        logger.info(f"Adaptive sample size: {sample_size} (for {n_points} points)")
        return sample_size
    
    def _smart_sampling(self, source_xz, sample_size):
        """智能采样：避免聚集，确保样本分布均匀"""
        n_points = len(source_xz)
        
        if n_points < 1000:
            return np.random.choice(n_points, sample_size, replace=False)
        
        try:
            x_min, x_max = np.min(source_xz[:, 0]), np.max(source_xz[:, 0])
            z_min, z_max = np.min(source_xz[:, 1]), np.max(source_xz[:, 1])
            
            grid_size = int(np.sqrt(sample_size))
            grid_size = max(2, grid_size)
            
            x_bins = np.linspace(x_min, x_max, grid_size + 1)
            z_bins = np.linspace(z_min, z_max, grid_size + 1)
            
            selected_indices = []
            points_per_cell = max(1, sample_size // (grid_size * grid_size))
            
            for i in range(grid_size):
                for j in range(grid_size):
                    mask = ((source_xz[:, 0] >= x_bins[i]) & (source_xz[:, 0] < x_bins[i+1]) &
                           (source_xz[:, 1] >= z_bins[j]) & (source_xz[:, 1] < z_bins[j+1]))
                    
                    cell_indices = np.where(mask)[0]
                    
                    if len(cell_indices) > 0:
                        n_select = min(points_per_cell, len(cell_indices))
                        selected_indices.extend(
                            np.random.choice(cell_indices, n_select, replace=False)
                        )
            
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
            return np.random.choice(n_points, sample_size, replace=False)
    
    def _fit_enhanced_polynomial_model(self, sample_source, sample_residual):
        """拟合增强的多项式模型"""
        try:
            x = sample_source[:, 0]
            z = sample_source[:, 1]
            
            # 使用二次多项式，但添加正则化
            features = np.column_stack([
                np.ones(len(x)),
                x, z,
                x**2, z**2, x*z
            ])
            
            alpha = 1e-6
            
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
            n_centers = min(10, len(sample_source) // 2)
            n_centers = max(1, n_centers)
            
            center_indices = [np.random.randint(len(sample_source))]
            
            for _ in range(n_centers - 1):
                distances = []
                for i in range(len(sample_source)):
                    min_dist = min([np.linalg.norm(sample_source[i] - sample_source[j]) 
                                  for j in center_indices])
                    distances.append(min_dist)
                
                center_indices.append(np.argmax(distances))
            
            rbf_centers = sample_source[center_indices]
            
            if len(sample_source) > 1:
                sigma = np.median([np.linalg.norm(sample_source[i] - sample_source[j]) 
                                 for i in range(len(sample_source)) 
                                 for j in range(i+1, len(sample_source))]) / 2
            else:
                sigma = 1.0
            
            sigma = max(sigma, 1e-6)
            
            features = np.zeros((len(sample_source), n_centers))
            for i, center in enumerate(rbf_centers):
                distances = np.linalg.norm(sample_source - center, axis=1)
                features[:, i] = np.exp(-(distances**2) / (2 * sigma**2))
            
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
        
        features = np.column_stack([
            np.ones(len(x)),
            x, z,
            x**2, z**2, x*z
        ])
        
        pred_x = np.dot(features, model_params['coeffs_x'])
        pred_z = np.dot(features, model_params['coeffs_z'])
        
        return np.column_stack([pred_x, pred_z])
    
    def _predict_rbf_model(self, source_points, model_params):
        """使用RBF模型预测残差"""
        rbf_centers = model_params['rbf_centers']
        weights_x = model_params['weights_x']
        weights_z = model_params['weights_z']
        sigma = model_params['sigma']
        
        features = np.zeros((len(source_points), len(rbf_centers)))
        for i, center in enumerate(rbf_centers):
            distances = np.linalg.norm(source_points - center, axis=1)
            features[:, i] = np.exp(-(distances**2) / (2 * sigma**2))
        
        pred_x = np.dot(features, weights_x)
        pred_z = np.dot(features, weights_z)
        
        return np.column_stack([pred_x, pred_z])
    
    def _compute_model_score(self, inlier_count, prediction_errors, inlier_mask, total_points):
        """计算模型质量分数"""
        if inlier_count == 0:
            return -np.inf
        
        inlier_ratio = inlier_count / total_points
        inlier_errors = prediction_errors[inlier_mask]
        mean_inlier_error = np.mean(inlier_errors)
        
        score = inlier_ratio - 0.1 * mean_inlier_error / self.distance_threshold
        
        return score
    
    def _fallback_strategy(self, source_xz, residual_xz):
        """备用策略：当RANSAC失败时的处理"""
        logger.info("Applying fallback strategy...")
        
        residual_magnitudes = np.linalg.norm(residual_xz, axis=1)
        relaxed_threshold = np.percentile(residual_magnitudes, 80)
        
        inlier_mask = residual_magnitudes < relaxed_threshold
        fallback_indices = np.where(inlier_mask)[0]
        
        logger.info(f"Fallback strategy: using {len(fallback_indices)} points with relaxed threshold {relaxed_threshold*1000:.2f}mm")
        
        return fallback_indices
    
    def _refine_inlier_set(self, source_xz, residual_xz, initial_inliers):
        """精炼内点集：移除可能的噪声点"""
        if len(initial_inliers) < 10:
            return initial_inliers
        
        inlier_residuals = residual_xz[initial_inliers]
        inlier_magnitudes = np.linalg.norm(inlier_residuals, axis=1)
        
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
    
    def _visualize_ransac_results(self, svd_transformed_A, residual_vectors, inlier_indices):
        """可视化RANSAC结果"""
        fig, axes = plt.subplots(2, 3, figsize=(18, 12))
        
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
        ax1.set_title('RANSAC Classification (Training Set)')
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
        ax2.set_title('Residual Distribution (Training)')
        ax2.legend()
        ax2.grid(True, alpha=0.3)
        
        # 3. 统计信息
        ax3 = axes[0, 2]
        stats_text = f"""RANSAC Statistics (Training):

Total Points: {self.ransac_stats['total_points']}
Inliers: {self.ransac_stats['inlier_count']} ({self.ransac_stats['inlier_ratio']*100:.1f}%)
Outliers: {self.ransac_stats['outlier_count']}

Iterations: {self.ransac_stats['iterations_used']}/{self.max_iterations}
Threshold: {self.distance_threshold*1000:.2f}mm
Sample Size: {self.ransac_stats['sample_size_used']}

Best Score: {self.ransac_stats.get('best_score', 'N/A'):.3f}
Early Stop: {'Yes' if self.ransac_stats.get('early_stopping', False) else 'No'}"""
        
        ax3.text(0.05, 0.95, stats_text, transform=ax3.transAxes, fontsize=10,
                verticalalignment='top', fontfamily='monospace')
        ax3.set_xlim(0, 1)
        ax3.set_ylim(0, 1)
        ax3.set_title('RANSAC Statistics')
        ax3.axis('off')
        
        # 4-6. 其他图表...
        ax4 = axes[1, 0]
        if np.sum(inlier_mask) > 0:
            ax4.scatter(svd_transformed_A[inlier_mask, 0], residual_vectors[inlier_mask, 0]*1000, 
                       c='green', alpha=0.6, s=15, label='Inliers')
        if np.sum(outlier_mask) > 0:
            ax4.scatter(svd_transformed_A[outlier_mask, 0], residual_vectors[outlier_mask, 0]*1000, 
                       c='red', alpha=0.6, s=15, label='Outliers')
        ax4.set_xlabel('X Position [m]')
        ax4.set_ylabel('X Residual [mm]')
        ax4.set_title('X-Direction Classification (Training)')
        ax4.legend()
        ax4.grid(True, alpha=0.3)
        ax4.axhline(y=0, color='black', linestyle='-', alpha=0.3)
        
        ax5 = axes[1, 1]
        if np.sum(inlier_mask) > 0:
            ax5.scatter(svd_transformed_A[inlier_mask, 2], residual_vectors[inlier_mask, 2]*1000, 
                       c='green', alpha=0.6, s=15, label='Inliers')
        if np.sum(outlier_mask) > 0:
            ax5.scatter(svd_transformed_A[outlier_mask, 2], residual_vectors[outlier_mask, 2]*1000, 
                       c='red', alpha=0.6, s=15, label='Outliers')
        ax5.set_xlabel('Z Position [m]')
        ax5.set_ylabel('Z Residual [mm]')
        ax5.set_title('Z-Direction Classification (Training)')
        ax5.legend()
        ax5.grid(True, alpha=0.3)
        ax5.axhline(y=0, color='black', linestyle='-', alpha=0.3)
        
        ax6 = axes[1, 2]
        classification = np.zeros(n_points)
        classification[outlier_mask] = 1
        
        scatter = ax6.scatter(svd_transformed_A[:, 0], svd_transformed_A[:, 2], 
                            c=classification, cmap='RdYlGn_r', s=25, alpha=0.7)
        ax6.set_xlabel('X [m]')
        ax6.set_ylabel('Z [m]')
        ax6.set_title('Spatial Classification (Training)')
        cbar = plt.colorbar(scatter, ax=ax6)
        cbar.set_label('0=Inlier, 1=Outlier')
        ax6.grid(True, alpha=0.3)
        ax6.axis('equal')
        
        plt.tight_layout()
        plt.savefig(os.path.join(img_dir, 'stage2_ransac_results_training.png'), 
                   dpi=300, bbox_inches='tight')
        plt.close()
        
        logger.info("RANSAC visualization saved: stage2_ransac_results_training.png")

class Stage3_GPRFineRegistration:
    """
    Stage 3: GPR精配准
    使用高斯过程回归学习精确且稳健的非刚性形变场
    支持在全数据集和内点集上分别训练，并在测试集上评估
    """
    def __init__(self):
        self.gpr_x_all = None
        self.gpr_z_all = None
        self.gpr_x_inliers = None
        self.gpr_z_inliers = None
        self.scaler_input_all = StandardScaler()
        self.scaler_output_x_all = StandardScaler()
        self.scaler_output_z_all = StandardScaler()
        self.scaler_input_inliers = StandardScaler()
        self.scaler_output_x_inliers = StandardScaler()
        self.scaler_output_z_inliers = StandardScaler()
        self.gpr_stats = {}
        
    def train_and_evaluate_models(self, train_svd_transformed_A, train_residual_vectors, 
                                 train_inlier_indices, 
                                 test_svd_transformed_A, test_positions_B):
        """
        训练两个GPR模型（全数据集和内点集）并在测试集上评估
        
        Args:
        - train_svd_transformed_A: 训练集SVD变换后的源点云
        - train_residual_vectors: 训练集SVD残差向量
        - train_inlier_indices: 训练集内点索引
        - test_svd_transformed_A: 测试集SVD变换后的源点云
        - test_positions_B: 测试集目标点云
        
        Returns:
        - evaluation_results: 详细的评估结果
        """
        logger.info("=== Stage 3: GPR Fine Registration (Training and Evaluation) ===")
        
        train_svd_transformed_A = np.array(train_svd_transformed_A)
        train_residual_vectors = np.array(train_residual_vectors)
        test_svd_transformed_A = np.array(test_svd_transformed_A)
        test_positions_B = np.array(test_positions_B)
        
        # 准备训练数据
        all_train_source = train_svd_transformed_A
        all_train_residuals = train_residual_vectors
        
        inlier_train_source = train_svd_transformed_A[train_inlier_indices]
        inlier_train_residuals = train_residual_vectors[train_inlier_indices]
        
        logger.info(f"Training GPR models:")
        logger.info(f"  All data model: {len(all_train_source)} points")
        logger.info(f"  Inliers model: {len(inlier_train_source)} points")
        logger.info(f"  Test set: {len(test_svd_transformed_A)} points")
        
        # 训练两个GPR模型
        success_all = self._train_gpr_model(all_train_source, all_train_residuals, model_type='all')
        success_inliers = self._train_gpr_model(inlier_train_source, inlier_train_residuals, model_type='inliers')
        
        if not success_all or not success_inliers:
            logger.error("GPR model training failed")
            return None
        
        # 在测试集上评估两个模型
        evaluation_results = self._evaluate_models_on_test(test_svd_transformed_A, test_positions_B)
        
        # 可视化评估结果
        self._visualize_evaluation_results(test_svd_transformed_A, test_positions_B, evaluation_results)
        
        logger.info("Stage 3 (GPR) completed successfully")
        
        return evaluation_results
    
    def _train_gpr_model(self, train_source, train_residuals, model_type='all'):
        """训练单个GPR模型"""
        logger.info(f"Training GPR model for {model_type} data...")
        
        # 准备训练数据（只使用XZ坐标）
        X_train = train_source[:, [0, 2]]
        y_train_x = train_residuals[:, 0]
        y_train_z = train_residuals[:, 2]
        
        # 选择对应的标准化器
        if model_type == 'all':
            scaler_input = self.scaler_input_all
            scaler_output_x = self.scaler_output_x_all
            scaler_output_z = self.scaler_output_z_all
        else:  # inliers
            scaler_input = self.scaler_input_inliers
            scaler_output_x = self.scaler_output_x_inliers
            scaler_output_z = self.scaler_output_z_inliers
        
        # 数据标准化
        X_train_scaled = scaler_input.fit_transform(X_train)
        y_train_x_scaled = scaler_output_x.fit_transform(y_train_x.reshape(-1, 1)).ravel()
        y_train_z_scaled = scaler_output_z.fit_transform(y_train_z.reshape(-1, 1)).ravel()
        
        try:
            # 定义候选核函数
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
                logger.info(f"  Testing kernel {i+1}/{len(kernels)} for {model_type}: {kernel}")
                
                gpr_x = GaussianProcessRegressor(
                    kernel=kernel, 
                    alpha=1e-6,
                    normalize_y=False,
                    n_restarts_optimizer=3,
                    random_state=42
                )
                
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
                    
                    # 计算训练分数
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
                    logger.warning(f"    Kernel {i+1} failed for {model_type}: {e}")
                    continue
            
            if best_gpr_x is None or best_gpr_z is None:
                logger.error(f"All GPR kernels failed for {model_type}")
                return False
            
            # 保存模型
            if model_type == 'all':
                self.gpr_x_all = best_gpr_x
                self.gpr_z_all = best_gpr_z
            else:  # inliers
                self.gpr_x_inliers = best_gpr_x
                self.gpr_z_inliers = best_gpr_z
            
            # 保存统计信息
            if f'{model_type}_stats' not in self.gpr_stats:
                self.gpr_stats[f'{model_type}_stats'] = {}
                
            self.gpr_stats[f'{model_type}_stats'] = {
                'training_points': len(X_train),
                'best_score_x': best_score_x,
                'best_score_z': best_score_z,
                'kernel_x': str(best_gpr_x.kernel_),
                'kernel_z': str(best_gpr_z.kernel_),
                'kernel_params_x': best_gpr_x.kernel_.get_params(),
                'kernel_params_z': best_gpr_z.kernel_.get_params()
            }
            
            logger.info(f"GPR training completed for {model_type}:")
            logger.info(f"  Best X-direction score: {best_score_x:.3f}")
            logger.info(f"  Best Z-direction score: {best_score_z:.3f}")
            logger.info(f"  Final X kernel: {best_gpr_x.kernel_}")
            logger.info(f"  Final Z kernel: {best_gpr_z.kernel_}")
            
            return True
            
        except Exception as e:
            logger.error(f"GPR training failed for {model_type}: {e}")
            return False
    
    def _predict_deformation(self, query_points, model_type='all'):
        """预测形变"""
        if model_type == 'all':
            gpr_x, gpr_z = self.gpr_x_all, self.gpr_z_all
            scaler_input = self.scaler_input_all
            scaler_output_x = self.scaler_output_x_all
            scaler_output_z = self.scaler_output_z_all
        else:  # inliers
            gpr_x, gpr_z = self.gpr_x_inliers, self.gpr_z_inliers
            scaler_input = self.scaler_input_inliers
            scaler_output_x = self.scaler_output_x_inliers
            scaler_output_z = self.scaler_output_z_inliers
        
        if gpr_x is None or gpr_z is None:
            logger.error(f"GPR models not trained for {model_type}")
            return np.zeros_like(query_points), None
        
        # 准备查询数据
        X_query = query_points[:, [0, 2]]
        X_query_scaled = scaler_input.transform(X_query)
        
        try:
            # 预测（包含不确定性）
            pred_x_scaled, std_x = gpr_x.predict(X_query_scaled, return_std=True)
            pred_z_scaled, std_z = gpr_z.predict(X_query_scaled, return_std=True)
            
            # 反标准化
            pred_x = scaler_output_x.inverse_transform(pred_x_scaled.reshape(-1, 1)).ravel()
            pred_z = scaler_output_z.inverse_transform(pred_z_scaled.reshape(-1, 1)).ravel()
            
            # 构建形变向量
            deformation = np.zeros_like(query_points)
            deformation[:, 0] = pred_x
            deformation[:, 2] = pred_z
            
            # 不确定性信息
            uncertainty = {
                'std_x': std_x,
                'std_z': std_z,
                'mean_std_x': np.mean(std_x),
                'mean_std_z': np.mean(std_z),
                'max_std_x': np.max(std_x),
                'max_std_z': np.max(std_z)
            }
            
            return deformation, uncertainty
            
        except Exception as e:
            logger.error(f"GPR prediction failed for {model_type}: {e}")
            return np.zeros_like(query_points), None
    
    def _evaluate_models_on_test(self, test_svd_transformed_A, test_positions_B):
        """在测试集上评估两个GPR模型"""
        logger.info("Evaluating both GPR models on test set...")
        
        # 基准：仅SVD的性能
        svd_errors = test_positions_B - test_svd_transformed_A
        svd_error_magnitudes = np.sqrt(np.sum(svd_errors**2, axis=1))
        svd_rmse_overall = np.sqrt(np.mean(svd_error_magnitudes**2))
        svd_rmse_x = np.sqrt(np.mean(svd_errors[:, 0]**2))
        svd_rmse_z = np.sqrt(np.mean(svd_errors[:, 2]**2))
        
        # 全数据集GPR模型预测
        deformation_all, uncertainty_all = self._predict_deformation(test_svd_transformed_A, 'all')
        final_transformed_all = test_svd_transformed_A + deformation_all
        final_transformed_all[:, 1] = 0
        
        all_errors = test_positions_B - final_transformed_all
        all_error_magnitudes = np.sqrt(np.sum(all_errors**2, axis=1))
        all_rmse_overall = np.sqrt(np.mean(all_error_magnitudes**2))
        all_rmse_x = np.sqrt(np.mean(all_errors[:, 0]**2))
        all_rmse_z = np.sqrt(np.mean(all_errors[:, 2]**2))
        
        # 内点集GPR模型预测
        deformation_inliers, uncertainty_inliers = self._predict_deformation(test_svd_transformed_A, 'inliers')
        final_transformed_inliers = test_svd_transformed_A + deformation_inliers
        final_transformed_inliers[:, 1] = 0
        
        inliers_errors = test_positions_B - final_transformed_inliers
        inliers_error_magnitudes = np.sqrt(np.sum(inliers_errors**2, axis=1))
        inliers_rmse_overall = np.sqrt(np.mean(inliers_error_magnitudes**2))
        inliers_rmse_x = np.sqrt(np.mean(inliers_errors[:, 0]**2))
        inliers_rmse_z = np.sqrt(np.mean(inliers_errors[:, 2]**2))
        
        # 计算改进
        improvement_all = (svd_rmse_overall - all_rmse_overall) * 1000  # mm
        improvement_inliers = (svd_rmse_overall - inliers_rmse_overall) * 1000  # mm
        
        # 保存评估结果
        evaluation_results = {
            'test_points': len(test_svd_transformed_A),
            'svd_only': {
                'rmse_overall': svd_rmse_overall,
                'rmse_x': svd_rmse_x,
                'rmse_z': svd_rmse_z,
                'max_error': np.max(svd_error_magnitudes),
                'mean_error': np.mean(svd_error_magnitudes),
                'errors': svd_errors,
                'error_magnitudes': svd_error_magnitudes
            },
            'gpr_all_data': {
                'rmse_overall': all_rmse_overall,
                'rmse_x': all_rmse_x,
                'rmse_z': all_rmse_z,
                'max_error': np.max(all_error_magnitudes),
                'mean_error': np.mean(all_error_magnitudes),
                'improvement': improvement_all,
                'improvement_ratio': improvement_all / (svd_rmse_overall * 1000) * 100,
                'errors': all_errors,
                'error_magnitudes': all_error_magnitudes,
                'transformed_points': final_transformed_all,
                'deformation': deformation_all,
                'uncertainty': uncertainty_all,
                'success': all_rmse_overall <= TARGET_RMSE
            },
            'gpr_inliers': {
                'rmse_overall': inliers_rmse_overall,
                'rmse_x': inliers_rmse_x,
                'rmse_z': inliers_rmse_z,
                'max_error': np.max(inliers_error_magnitudes),
                'mean_error': np.mean(inliers_error_magnitudes),
                'improvement': improvement_inliers,
                'improvement_ratio': improvement_inliers / (svd_rmse_overall * 1000) * 100,
                'errors': inliers_errors,
                'error_magnitudes': inliers_error_magnitudes,
                'transformed_points': final_transformed_inliers,
                'deformation': deformation_inliers,
                'uncertainty': uncertainty_inliers,
                'success': inliers_rmse_overall <= TARGET_RMSE
            }
        }
        
        logger.info("Test Set Evaluation Results:")
        logger.info(f"  SVD Only RMSE: {svd_rmse_overall*1000:.3f}mm")
        logger.info(f"  GPR (All Data) RMSE: {all_rmse_overall*1000:.3f}mm (Improvement: {improvement_all:.3f}mm)")
        logger.info(f"  GPR (Inliers) RMSE: {inliers_rmse_overall*1000:.3f}mm (Improvement: {improvement_inliers:.3f}mm)")
        logger.info(f"  Target RMSE: {TARGET_RMSE*1000:.2f}mm")
        logger.info(f"  GPR (All Data) Success: {'YES' if evaluation_results['gpr_all_data']['success'] else 'NO'}")
        logger.info(f"  GPR (Inliers) Success: {'YES' if evaluation_results['gpr_inliers']['success'] else 'NO'}")
        
        # 更新统计信息
        self.gpr_stats['evaluation_results'] = evaluation_results
        
        return evaluation_results
    
    def _visualize_evaluation_results(self, test_svd_transformed_A, test_positions_B, evaluation_results):
        """可视化测试集评估结果"""
        fig, axes = plt.subplots(3, 4, figsize=(24, 18))
        
        # 数据准备
        svd_result = evaluation_results['svd_only']
        all_result = evaluation_results['gpr_all_data']
        inliers_result = evaluation_results['gpr_inliers']
        
        # 1. 点云对比
        ax1 = axes[0, 0]
        ax1.scatter(test_svd_transformed_A[:, 0], test_svd_transformed_A[:, 2], 
                   c='blue', alpha=0.6, s=20, label='SVD Only')
        ax1.scatter(all_result['transformed_points'][:, 0], all_result['transformed_points'][:, 2], 
                   c='green', alpha=0.6, s=20, label='GPR (All Data)')
        ax1.scatter(inliers_result['transformed_points'][:, 0], inliers_result['transformed_points'][:, 2], 
                   c='orange', alpha=0.6, s=20, label='GPR (Inliers)')
        ax1.scatter(test_positions_B[:, 0], test_positions_B[:, 2], 
                   c='red', alpha=0.6, s=20, label='Target (Test)')
        ax1.set_xlabel('X [m]')
        ax1.set_ylabel('Z [m]')
        ax1.set_title('Test Set Registration Results Comparison')
        ax1.legend()
        ax1.grid(True, alpha=0.3)
        ax1.axis('equal')
        
        # 2. RMSE对比条形图
        ax2 = axes[0, 1]
        categories = ['Overall', 'X-Direction', 'Z-Direction']
        svd_rmse = [svd_result['rmse_overall']*1000, svd_result['rmse_x']*1000, svd_result['rmse_z']*1000]
        all_rmse = [all_result['rmse_overall']*1000, all_result['rmse_x']*1000, all_result['rmse_z']*1000]
        inliers_rmse = [inliers_result['rmse_overall']*1000, inliers_result['rmse_x']*1000, inliers_result['rmse_z']*1000]
        
        x = np.arange(len(categories))
        width = 0.25
        
        ax2.bar(x - width, svd_rmse, width, label='SVD Only', alpha=0.8, color='blue')
        ax2.bar(x, all_rmse, width, label='GPR (All Data)', alpha=0.8, color='green')
        ax2.bar(x + width, inliers_rmse, width, label='GPR (Inliers)', alpha=0.8, color='orange')
        
        ax2.axhline(y=TARGET_RMSE*1000, color='red', linestyle='--', alpha=0.7, 
                   label=f'Target: {TARGET_RMSE*1000:.1f}mm')
        ax2.set_xlabel('Error Type')
        ax2.set_ylabel('RMSE [mm]')
        ax2.set_title('Test Set RMSE Comparison')
        ax2.set_xticks(x)
        ax2.set_xticklabels(categories)
        ax2.legend()
        ax2.grid(True, alpha=0.3)
        
        # 3. 误差分布对比
        ax3 = axes[0, 2]
        ax3.hist(svd_result['error_magnitudes']*1000, bins=30, alpha=0.7, color='blue', 
                label='SVD Only', density=True)
        ax3.hist(all_result['error_magnitudes']*1000, bins=30, alpha=0.7, color='green', 
                label='GPR (All Data)', density=True)
        ax3.hist(inliers_result['error_magnitudes']*1000, bins=30, alpha=0.7, color='orange', 
                label='GPR (Inliers)', density=True)
        ax3.axvline(x=TARGET_RMSE*1000, color='red', linestyle='--', alpha=0.7)
        ax3.set_xlabel('Error Magnitude [mm]')
        ax3.set_ylabel('Density')
        ax3.set_title('Test Set Error Distribution')
        ax3.legend()
        ax3.grid(True, alpha=0.3)
        
        # 4. 改进量对比
        ax4 = axes[0, 3]
        models = ['GPR (All Data)', 'GPR (Inliers)']
        improvements = [all_result['improvement'], inliers_result['improvement']]
        colors = ['green', 'orange']
        
        bars = ax4.bar(models, improvements, color=colors, alpha=0.8)
        ax4.set_ylabel('Improvement [mm]')
        ax4.set_title('Improvement Over SVD-Only')
        ax4.grid(True, alpha=0.3)
        
        for bar, improvement in zip(bars, improvements):
            ax4.text(bar.get_x() + bar.get_width()/2, bar.get_height() + 0.01,
                    f'{improvement:.3f}mm', ha='center', va='bottom', fontweight='bold')
        
        # 5. 空间误差分布 - SVD Only
        ax5 = axes[1, 0]
        scatter = ax5.scatter(test_svd_transformed_A[:, 0], test_svd_transformed_A[:, 2], 
                            c=svd_result['error_magnitudes']*1000, cmap='viridis', s=25, alpha=0.7)
        plt.colorbar(scatter, ax=ax5, label='Error [mm]')
        ax5.set_xlabel('X [m]')
        ax5.set_ylabel('Z [m]')
        ax5.set_title('Spatial Error Distribution - SVD Only')
        ax5.grid(True, alpha=0.3)
        ax5.axis('equal')
        
        # 6. 空间误差分布 - GPR (All Data)
        ax6 = axes[1, 1]
        scatter = ax6.scatter(all_result['transformed_points'][:, 0], all_result['transformed_points'][:, 2], 
                            c=all_result['error_magnitudes']*1000, cmap='viridis', s=25, alpha=0.7)
        plt.colorbar(scatter, ax=ax6, label='Error [mm]')
        ax6.set_xlabel('X [m]')
        ax6.set_ylabel('Z [m]')
        ax6.set_title('Spatial Error Distribution - GPR (All Data)')
        ax6.grid(True, alpha=0.3)
        ax6.axis('equal')
        
        # 7. 空间误差分布 - GPR (Inliers)
        ax7 = axes[1, 2]
        scatter = ax7.scatter(inliers_result['transformed_points'][:, 0], inliers_result['transformed_points'][:, 2], 
                            c=inliers_result['error_magnitudes']*1000, cmap='viridis', s=25, alpha=0.7)
        plt.colorbar(scatter, ax=ax7, label='Error [mm]')
        ax7.set_xlabel('X [m]')
        ax7.set_ylabel('Z [m]')
        ax7.set_title('Spatial Error Distribution - GPR (Inliers)')
        ax7.grid(True, alpha=0.3)
        ax7.axis('equal')
        
        # 8. 形变场对比
        ax8 = axes[1, 3]
        step = max(1, len(test_svd_transformed_A) // 30)
        scale_factor = 15
        
        # GPR (All Data) 形变场
        ax8.quiver(test_svd_transformed_A[::step, 0], test_svd_transformed_A[::step, 2], 
                  all_result['deformation'][::step, 0]*scale_factor, 
                  all_result['deformation'][::step, 2]*scale_factor,
                  angles='xy', scale_units='xy', scale=1, alpha=0.7, color='green', 
                  width=0.003, label='GPR (All Data)')
        
        # GPR (Inliers) 形变场
        ax8.quiver(test_svd_transformed_A[::step, 0], test_svd_transformed_A[::step, 2], 
                  inliers_result['deformation'][::step, 0]*scale_factor, 
                  inliers_result['deformation'][::step, 2]*scale_factor,
                  angles='xy', scale_units='xy', scale=1, alpha=0.5, color='orange', 
                  width=0.002, label='GPR (Inliers)')
        
        ax8.scatter(test_svd_transformed_A[::step, 0], test_svd_transformed_A[::step, 2], 
                   c='blue', alpha=0.5, s=15)
        ax8.set_xlabel('X [m]')
        ax8.set_ylabel('Z [m]')
        ax8.set_title(f'Deformation Fields Comparison (×{scale_factor})')
        ax8.legend()
        ax8.grid(True, alpha=0.3)
        ax8.axis('equal')
        
        # 9. 不确定性对比 - GPR (All Data)
        ax9 = axes[2, 0]
        if all_result['uncertainty'] is not None:
            uncertainty_all_combined = np.sqrt(all_result['uncertainty']['std_x']**2 + 
                                             all_result['uncertainty']['std_z']**2)
            scatter = ax9.scatter(all_result['transformed_points'][:, 0], all_result['transformed_points'][:, 2], 
                                c=uncertainty_all_combined, cmap='plasma', s=25, alpha=0.7)
            plt.colorbar(scatter, ax=ax9, label='Uncertainty')
            ax9.set_xlabel('X [m]')
            ax9.set_ylabel('Z [m]')
            ax9.set_title('Prediction Uncertainty - GPR (All Data)')
            ax9.grid(True, alpha=0.3)
            ax9.axis('equal')
        else:
            ax9.text(0.5, 0.5, 'Uncertainty data\nnot available', 
                    ha='center', va='center', transform=ax9.transAxes, fontsize=12)
            ax9.set_title('Prediction Uncertainty - GPR (All Data)')
        
        # 10. 不确定性对比 - GPR (Inliers)
        ax10 = axes[2, 1]
        if inliers_result['uncertainty'] is not None:
            uncertainty_inliers_combined = np.sqrt(inliers_result['uncertainty']['std_x']**2 + 
                                                  inliers_result['uncertainty']['std_z']**2)
            scatter = ax10.scatter(inliers_result['transformed_points'][:, 0], inliers_result['transformed_points'][:, 2], 
                                 c=uncertainty_inliers_combined, cmap='plasma', s=25, alpha=0.7)
            plt.colorbar(scatter, ax=ax10, label='Uncertainty')
            ax10.set_xlabel('X [m]')
            ax10.set_ylabel('Z [m]')
            ax10.set_title('Prediction Uncertainty - GPR (Inliers)')
            ax10.grid(True, alpha=0.3)
            ax10.axis('equal')
        else:
            ax10.text(0.5, 0.5, 'Uncertainty data\nnot available', 
                     ha='center', va='center', transform=ax10.transAxes, fontsize=12)
            ax10.set_title('Prediction Uncertainty - GPR (Inliers)')
        
        # 11. 模型性能统计
        ax11 = axes[2, 2]
        stats_text = f"""Test Set Evaluation Summary:

Test Points: {evaluation_results['test_points']}
Target RMSE: {TARGET_RMSE*1000:.2f}mm

SVD Only:
  RMSE: {svd_result['rmse_overall']*1000:.3f}mm
  
GPR (All Data):
  RMSE: {all_result['rmse_overall']*1000:.3f}mm
  Improvement: {all_result['improvement']:.3f}mm
  Success: {'YES' if all_result['success'] else 'NO'}
  
GPR (Inliers):
  RMSE: {inliers_result['rmse_overall']*1000:.3f}mm
  Improvement: {inliers_result['improvement']:.3f}mm
  Success: {'YES' if inliers_result['success'] else 'NO'}

Best Model: {'GPR (All Data)' if all_result['rmse_overall'] < inliers_result['rmse_overall'] else 'GPR (Inliers)'}"""
        
        ax11.text(0.05, 0.95, stats_text, transform=ax11.transAxes, fontsize=10,
                 verticalalignment='top', fontfamily='monospace')
        ax11.set_xlim(0, 1)
        ax11.set_ylim(0, 1)
        ax11.set_title('Evaluation Statistics')
        ax11.axis('off')
        
        # 12. 成功率对比
        ax12 = axes[2, 3]
        models = ['SVD Only', 'GPR (All Data)', 'GPR (Inliers)']
        success_rates = [
            1 if svd_result['rmse_overall'] <= TARGET_RMSE else 0,
            1 if all_result['success'] else 0,
            1 if inliers_result['success'] else 0
        ]
        colors = ['blue', 'green', 'orange']
        
        bars = ax12.bar(models, success_rates, color=colors, alpha=0.8)
        ax12.set_ylabel('Success (Target Achieved)')
        ax12.set_title('Target Achievement Comparison')
        ax12.set_ylim(0, 1.2)
        ax12.grid(True, alpha=0.3)
        
        for bar, success in zip(bars, success_rates):
            ax12.text(bar.get_x() + bar.get_width()/2, bar.get_height() + 0.05,
                     'YES' if success else 'NO', ha='center', va='bottom', fontweight='bold')
        
        plt.tight_layout()
        plt.savefig(os.path.join(img_dir, 'stage3_gpr_evaluation_results.png'), 
                   dpi=300, bbox_inches='tight')
        plt.close()
        
        logger.info("GPR evaluation visualization saved: stage3_gpr_evaluation_results.png")

class EnhancedThreeStageRegistrationPipeline:
    """
    增强版三阶段配准流水线：体素采样 + SVD → RANSAC → GPR + 测试评估
    """
    def __init__(self, positions_A, positions_B):
        self.positions_A = np.array(positions_A)
        self.positions_B = np.array(positions_B)
        self.voxel_sampler = AdaptiveVoxelSampler(train_ratio=TRAIN_RATIO)
        self.stage1 = Stage1_SVDCoarseAlignment()
        self.stage2 = Stage2_RANSACInlierSelection()
        self.stage3 = Stage3_GPRFineRegistration()
        self.pipeline_stats = {}
        
    def run_complete_pipeline(self):
        """运行完整的增强版三阶段配准流水线"""
        logger.info("=" * 100)
        logger.info("STARTING ENHANCED THREE-STAGE REGISTRATION PIPELINE")
        logger.info("Voxel Sampling → SVD (Coarse) → RANSAC (Inlier Selection) → GPR (Fine Registration)")
        logger.info("=" * 100)
        
        start_time = datetime.datetime.now()
        
        # 数据预处理
        logger.info("Data Preprocessing...")
        clean_A, clean_B = self._preprocess_data()
        
        # 步骤1：自适应体素网格采样，划分训练测试集
        logger.info("\n" + "="*60)
        logger.info("STEP 1: ADAPTIVE VOXEL SAMPLING FOR TRAIN/TEST SPLIT")
        logger.info("="*60)
        train_indices, test_indices, voxel_info = self.voxel_sampler.adaptive_train_test_split(clean_A, clean_B)
        
        # 准备训练测试集数据
        train_A = clean_A[train_indices]
        train_B = clean_B[train_indices]
        test_A = clean_A[test_indices]
        test_B = clean_B[test_indices]
        
        # 步骤2：在训练集上进行SVD粗配准
        logger.info("\n" + "="*60)
        logger.info("STEP 2: SVD COARSE ALIGNMENT (TRAINING SET)")
        logger.info("="*60)
        train_svd_transformed_A, train_residual_vectors, T_svd = self.stage1.perform_svd_alignment(train_A, train_B)
        
        # 步骤3：在训练集上进行RANSAC内点筛选
        logger.info("\n" + "="*60)
        logger.info("STEP 3: RANSAC INLIER SELECTION (TRAINING SET)")
        logger.info("="*60)
        train_inlier_indices, ransac_model = self.stage2.select_inliers(train_svd_transformed_A, train_residual_vectors)
        
        # 步骤4：将SVD变换应用到测试集
        logger.info("\n" + "="*60)
        logger.info("STEP 4: APPLY SVD TRANSFORMATION TO TEST SET")
        logger.info("="*60)
        test_svd_transformed_A = self.stage1.apply_transformation_to_test(test_A)
        logger.info(f"Applied SVD transformation to {len(test_svd_transformed_A)} test points")
        
        # 步骤5：训练两个GPR模型并在测试集上评估
        logger.info("\n" + "="*60)
        logger.info("STEP 5: GPR TRAINING AND TEST SET EVALUATION")
        logger.info("="*60)
        evaluation_results = self.stage3.train_and_evaluate_models(
            train_svd_transformed_A, train_residual_vectors, train_inlier_indices,
            test_svd_transformed_A, test_B
        )
        
        end_time = datetime.datetime.now()
        processing_time = (end_time - start_time).total_seconds()
        
        # 汇总结果
        self._summarize_enhanced_pipeline_results(clean_A, clean_B, train_indices, test_indices, 
                                                evaluation_results, processing_time)
        
        # 生成综合报告
        self._generate_comprehensive_report()
        
        # 创建综合可视化
        self._create_comprehensive_visualization(clean_A, clean_B, train_indices, test_indices, 
                                               train_svd_transformed_A, test_svd_transformed_A, evaluation_results)
        
        logger.info("=" * 100)
        logger.info("ENHANCED THREE-STAGE REGISTRATION PIPELINE COMPLETED")
        logger.info("=" * 100)
        
        return evaluation_results
    
    def _preprocess_data(self):
        """数据预处理：去重和基本清理"""
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
    
    def _summarize_enhanced_pipeline_results(self, original_A, original_B, train_indices, test_indices, 
                                           evaluation_results, processing_time):
        """汇总增强流水线结果"""
        if evaluation_results is None:
            logger.error("No evaluation results to summarize")
            return
        
        # 获取最佳模型结果
        all_data_result = evaluation_results['gpr_all_data']
        inliers_result = evaluation_results['gpr_inliers']
        svd_result = evaluation_results['svd_only']
        
        # 确定最佳模型
        best_model_name = 'GPR (All Data)' if all_data_result['rmse_overall'] < inliers_result['rmse_overall'] else 'GPR (Inliers)'
        best_result = all_data_result if all_data_result['rmse_overall'] < inliers_result['rmse_overall'] else inliers_result
        
        # 保存流水线统计信息
        self.pipeline_stats = {
            'total_points': len(original_A),
            'train_points': len(train_indices),
            'test_points': len(test_indices),
            'processing_time': processing_time,
            'target_rmse': TARGET_RMSE,
            'voxel_sampler_stats': self.voxel_sampler.voxel_stats,
            'stage1_stats': self.stage1.svd_stats,
            'stage2_stats': self.stage2.ransac_stats,
            'stage3_stats': self.stage3.gpr_stats,
            'evaluation_results': evaluation_results,
            'best_model_name': best_model_name,
            'best_result': best_result,
            'svd_baseline': svd_result
        }
        
        logger.info("\n" + "="*80)
        logger.info("ENHANCED PIPELINE SUMMARY")
        logger.info("="*80)
        logger.info(f"Processing time: {processing_time:.2f} seconds")
        logger.info(f"Total points: {len(original_A)}")
        logger.info(f"Training points: {len(train_indices)} ({len(train_indices)/len(original_A)*100:.1f}%)")
        logger.info(f"Test points: {len(test_indices)} ({len(test_indices)/len(original_A)*100:.1f}%)")
        logger.info(f"RANSAC inlier ratio: {self.stage2.ransac_stats['inlier_ratio']*100:.1f}%")
        logger.info("")
        logger.info("TEST SET ACCURACY RESULTS:")
        logger.info(f"  Target RMSE: {TARGET_RMSE*1000:.2f}mm")
        logger.info(f"  SVD Baseline: {svd_result['rmse_overall']*1000:.3f}mm")
        logger.info(f"  GPR (All Data): {all_data_result['rmse_overall']*1000:.3f}mm (Improvement: {all_data_result['improvement']:.3f}mm)")
        logger.info(f"  GPR (Inliers): {inliers_result['rmse_overall']*1000:.3f}mm (Improvement: {inliers_result['improvement']:.3f}mm)")
        logger.info(f"  Best Model: {best_model_name}")
        logger.info(f"  Best RMSE: {best_result['rmse_overall']*1000:.3f}mm")
        logger.info("")
        logger.info("SUCCESS EVALUATION:")
        logger.info(f"  SVD Only Success: {'YES' if svd_result['rmse_overall'] <= TARGET_RMSE else 'NO'}")
        logger.info(f"  GPR (All Data) Success: {'YES' if all_data_result['success'] else 'NO'}")
        logger.info(f"  GPR (Inliers) Success: {'YES' if inliers_result['success'] else 'NO'}")
        logger.info(f"  Overall Pipeline Success: {'YES' if best_result['success'] else 'NO'}")
        
        if best_result['success']:
            logger.info(f"✓ Target accuracy achieved with {best_model_name}!")
        else:
            deficit = (best_result['rmse_overall'] - TARGET_RMSE) * 1000
            logger.info(f"✗ Target accuracy not achieved (deficit: {deficit:.3f}mm)")
    
    def _generate_comprehensive_report(self):
        """生成综合报告"""
        report_file = os.path.join(result_dir_final, "enhanced_pipeline_comprehensive_report.txt")
        
        with open(report_file, "w") as f:
            f.write("ENHANCED THREE-STAGE REGISTRATION PIPELINE REPORT\n")
            f.write("=" * 100 + "\n")
            f.write(f"Generated: {datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n")
            f.write(f"Scale Factor: 1:{SCALE_FACTOR}\n")
            f.write(f"Target RMSE: {TARGET_RMSE*1000:.2f}mm\n")
            f.write(f"Train/Test Ratio: {TRAIN_RATIO:.1f}/{TEST_RATIO:.1f}\n")
            f.write("\n")
            
            f.write("PIPELINE OVERVIEW\n")
            f.write("-" * 50 + "\n")
            f.write("Step 1: Adaptive Voxel Sampling for Train/Test Split\n")
            f.write("  Purpose: Spatially uniform data splitting based on data distribution\n")
            f.write("  Output: Training and test sets with balanced spatial coverage\n")
            f.write("\n")
            f.write("Step 2: SVD Coarse Alignment (Training Set)\n")
            f.write("  Purpose: Compute global rigid transformation (rotation + translation)\n")
            f.write("  Output: Transformed training points + residual vectors\n")
            f.write("\n")
            f.write("Step 3: RANSAC Inlier Selection (Training Set)\n")
            f.write("  Purpose: Select reliable data points for final model training\n")
            f.write("  Output: Inlier indices (noise-free training data)\n")
            f.write("\n")
            f.write("Step 4: Apply SVD to Test Set\n")
            f.write("  Purpose: Transform test set using learned SVD parameters\n")
            f.write("  Output: SVD-transformed test points\n")
            f.write("\n")
            f.write("Step 5: GPR Training and Test Evaluation\n")
            f.write("  Purpose: Train two GPR models and evaluate on test set\n")
            f.write("  Models: GPR(All Training Data) vs GPR(Inliers Only)\n")
            f.write("  Output: Comprehensive test set evaluation\n")
            f.write("\n")
            
            f.write("PROCESSING RESULTS\n")
            f.write("-" * 50 + "\n")
            f.write(f"Total processing time: {self.pipeline_stats['processing_time']:.2f} seconds\n")
            f.write(f"Total points: {self.pipeline_stats['total_points']}\n")
            f.write(f"Training points: {self.pipeline_stats['train_points']} ({self.pipeline_stats['train_points']/self.pipeline_stats['total_points']*100:.1f}%)\n")
            f.write(f"Test points: {self.pipeline_stats['test_points']} ({self.pipeline_stats['test_points']/self.pipeline_stats['total_points']*100:.1f}%)\n")
            f.write(f"Best model: {self.pipeline_stats['best_model_name']}\n")
            f.write(f"Best test RMSE: {self.pipeline_stats['best_result']['rmse_overall']*1000:.3f}mm\n")
            f.write(f"Target achieved: {'YES' if self.pipeline_stats['best_result']['success'] else 'NO'}\n")
            f.write("\n")
            
            f.write("VOXEL SAMPLING RESULTS\n")
            f.write("-" * 50 + "\n")
            if hasattr(self.voxel_sampler, 'voxel_stats') and self.voxel_sampler.voxel_stats:
                voxel_stats = self.voxel_sampler.voxel_stats
                f.write(f"Optimal voxel size: {voxel_stats['best_size']:.6f}\n")
                f.write(f"Base voxel size: {voxel_stats['base_size']:.6f}\n")
                f.write(f"Best quality score: {voxel_stats['best_score']:.3f}\n")
            f.write("\n")
            
            f.write("SVD RESULTS (TRAINING SET)\n")
            f.write("-" * 50 + "\n")
            stage1_stats = self.pipeline_stats['stage1_stats']
            f.write(f"Training points: {stage1_stats['num_points']}\n")
            f.write(f"Overall RMSE: {stage1_stats['rmse_overall']*1000:.3f}mm\n")
            f.write(f"X-direction RMSE: {stage1_stats['rmse_x']*1000:.3f}mm\n")
            f.write(f"Z-direction RMSE: {stage1_stats['rmse_z']*1000:.3f}mm\n")
            f.write(f"Maximum error: {stage1_stats['max_error']*1000:.3f}mm\n")
            f.write(f"Mean error: {stage1_stats['mean_error']*1000:.3f}mm\n")
            f.write("\n")
            
            f.write("RANSAC RESULTS (TRAINING SET)\n")
            f.write("-" * 50 + "\n")
            stage2_stats = self.pipeline_stats['stage2_stats']
            f.write(f"Training points: {stage2_stats['total_points']}\n")
            f.write(f"Inliers found: {stage2_stats['inlier_count']} ({stage2_stats['inlier_ratio']*100:.1f}%)\n")
            f.write(f"Outliers removed: {stage2_stats['outlier_count']} ({(1-stage2_stats['inlier_ratio'])*100:.1f}%)\n")
            f.write(f"Distance threshold: {stage2_stats['distance_threshold']*1000:.1f}mm\n")
            f.write(f"Iterations used: {stage2_stats['iterations_used']}\n")
            f.write("\n")
            
            f.write("GPR RESULTS (TEST SET EVALUATION)\n")
            f.write("-" * 50 + "\n")
            eval_results = self.pipeline_stats['evaluation_results']
            f.write(f"Test points: {eval_results['test_points']}\n")
            f.write("\n")
            f.write("SVD Baseline:\n")
            f.write(f"  Overall RMSE: {eval_results['svd_only']['rmse_overall']*1000:.3f}mm\n")
            f.write(f"  X RMSE: {eval_results['svd_only']['rmse_x']*1000:.3f}mm\n")
            f.write(f"  Z RMSE: {eval_results['svd_only']['rmse_z']*1000:.3f}mm\n")
            f.write("\n")
            f.write("GPR (All Training Data):\n")
            f.write(f"  Overall RMSE: {eval_results['gpr_all_data']['rmse_overall']*1000:.3f}mm\n")
            f.write(f"  X RMSE: {eval_results['gpr_all_data']['rmse_x']*1000:.3f}mm\n")
            f.write(f"  Z RMSE: {eval_results['gpr_all_data']['rmse_z']*1000:.3f}mm\n")
            f.write(f"  Improvement: {eval_results['gpr_all_data']['improvement']:.3f}mm\n")
            f.write(f"  Success: {'YES' if eval_results['gpr_all_data']['success'] else 'NO'}\n")
            f.write("\n")
            f.write("GPR (Inliers Only):\n")
            f.write(f"  Overall RMSE: {eval_results['gpr_inliers']['rmse_overall']*1000:.3f}mm\n")
            f.write(f"  X RMSE: {eval_results['gpr_inliers']['rmse_x']*1000:.3f}mm\n")
            f.write(f"  Z RMSE: {eval_results['gpr_inliers']['rmse_z']*1000:.3f}mm\n")
            f.write(f"  Improvement: {eval_results['gpr_inliers']['improvement']:.3f}mm\n")
            f.write(f"  Success: {'YES' if eval_results['gpr_inliers']['success'] else 'NO'}\n")
            f.write("\n")
            
            f.write("REAL-WORLD SCALING\n")
            f.write("-" * 50 + "\n")
            f.write(f"Model scale: 1:{SCALE_FACTOR}\n")
            f.write(f"Best model RMSE: {self.pipeline_stats['best_result']['rmse_overall']*1000:.3f}mm\n")
            f.write(f"Real-world equivalent: {self.pipeline_stats['best_result']['rmse_overall']*SCALE_FACTOR*100:.2f}cm\n")
            f.write(f"Real-world target: {REAL_WORLD_LATERAL_TARGET*100:.0f}cm\n")
            f.write("\n")
        
        logger.info(f"Comprehensive report saved: {report_file}")
    
    def _create_comprehensive_visualization(self, original_A, original_B, train_indices, test_indices,
                                          train_svd_A, test_svd_A, evaluation_results):
        """创建综合可视化图表"""
        fig = plt.figure(figsize=(24, 20))
        
        # 创建网格布局
        gs = fig.add_gridspec(4, 4, hspace=0.3, wspace=0.3)
        
        # 准备数据
        train_A = original_A[train_indices]
        train_B = original_B[train_indices]
        test_A = original_A[test_indices]
        test_B = original_B[test_indices]
        
        eval_results = evaluation_results
        all_result = eval_results['gpr_all_data']
        inliers_result = eval_results['gpr_inliers']
        svd_result = eval_results['svd_only']
        
        # 1. 数据集分割可视化
        ax1 = fig.add_subplot(gs[0, 0])
        ax1.scatter(train_A[:, 0], train_A[:, 2], c='green', alpha=0.6, s=15, label=f'Training ({len(train_A)})')
        ax1.scatter(test_A[:, 0], test_A[:, 2], c='red', alpha=0.6, s=15, label=f'Test ({len(test_A)})')
        ax1.set_xlabel('X [m]')
        ax1.set_ylabel('Z [m]')
        ax1.set_title('Adaptive Voxel-Based Train/Test Split')
        ax1.legend()
        ax1.grid(True, alpha=0.3)
        ax1.axis('equal')
        
        # 2. 测试集配准结果对比
        ax2 = fig.add_subplot(gs[0, 1])
        ax2.scatter(test_svd_A[:, 0], test_svd_A[:, 2], c='blue', alpha=0.6, s=15, label='SVD Only')
        ax2.scatter(all_result['transformed_points'][:, 0], all_result['transformed_points'][:, 2], 
                   c='green', alpha=0.6, s=15, label='GPR (All Data)')
        ax2.scatter(inliers_result['transformed_points'][:, 0], inliers_result['transformed_points'][:, 2], 
                   c='orange', alpha=0.6, s=15, label='GPR (Inliers)')
        ax2.scatter(test_B[:, 0], test_B[:, 2], c='red', alpha=0.6, s=15, label='Target')
        ax2.set_xlabel('X [m]')
        ax2.set_ylabel('Z [m]')
        ax2.set_title('Test Set Registration Results')
        ax2.legend()
        ax2.grid(True, alpha=0.3)
        ax2.axis('equal')
        
        # 3. RMSE对比
        ax3 = fig.add_subplot(gs[0, 2])
        categories = ['Overall', 'X-Direction', 'Z-Direction']
        svd_rmse = [svd_result['rmse_overall']*1000, svd_result['rmse_x']*1000, svd_result['rmse_z']*1000]
        all_rmse = [all_result['rmse_overall']*1000, all_result['rmse_x']*1000, all_result['rmse_z']*1000]
        inliers_rmse = [inliers_result['rmse_overall']*1000, inliers_result['rmse_x']*1000, inliers_result['rmse_z']*1000]
        
        x = np.arange(len(categories))
        width = 0.25
        
        ax3.bar(x - width, svd_rmse, width, label='SVD Only', alpha=0.8, color='blue')
        ax3.bar(x, all_rmse, width, label='GPR (All Data)', alpha=0.8, color='green')
        ax3.bar(x + width, inliers_rmse, width, label='GPR (Inliers)', alpha=0.8, color='orange')
        
        ax3.axhline(y=TARGET_RMSE*1000, color='red', linestyle='--', alpha=0.7, 
                   label=f'Target: {TARGET_RMSE*1000:.1f}mm')
        ax3.set_xlabel('Error Type')
        ax3.set_ylabel('RMSE [mm]')
        ax3.set_title('Test Set RMSE Comparison')
        ax3.set_xticks(x)
        ax3.set_xticklabels(categories)
        ax3.legend()
        ax3.grid(True, alpha=0.3)
        
        # 4. 成功率对比
        ax4 = fig.add_subplot(gs[0, 3])
        models = ['SVD Only', 'GPR (All Data)', 'GPR (Inliers)']
        success_rates = [
            1 if svd_result['rmse_overall'] <= TARGET_RMSE else 0,
            1 if all_result['success'] else 0,
            1 if inliers_result['success'] else 0
        ]
        colors = ['blue', 'green', 'orange']
        
        bars = ax4.bar(models, success_rates, color=colors, alpha=0.8)
        ax4.set_ylabel('Success (Target Achieved)')
        ax4.set_title('Target Achievement Comparison')
        ax4.set_ylim(0, 1.2)
        ax4.grid(True, alpha=0.3)
        
        for bar, success in zip(bars, success_rates):
            ax4.text(bar.get_x() + bar.get_width()/2, bar.get_height() + 0.05,
                     'YES' if success else 'NO', ha='center', va='bottom', fontweight='bold')
        
        # 5. 误差分布对比
        ax5 = fig.add_subplot(gs[1, 0])
        ax5.hist(svd_result['error_magnitudes']*1000, bins=25, alpha=0.7, color='blue', 
                label='SVD Only', density=True)
        ax5.hist(all_result['error_magnitudes']*1000, bins=25, alpha=0.7, color='green', 
                label='GPR (All Data)', density=True)
        ax5.hist(inliers_result['error_magnitudes']*1000, bins=25, alpha=0.7, color='orange', 
                label='GPR (Inliers)', density=True)
        ax5.axvline(x=TARGET_RMSE*1000, color='red', linestyle='--', alpha=0.7)
        ax5.set_xlabel('Error Magnitude [mm]')
        ax5.set_ylabel('Density')
        ax5.set_title('Test Set Error Distribution')
        ax5.legend()
        ax5.grid(True, alpha=0.3)
        
        # 6. 空间误差分布 - SVD Only
        ax6 = fig.add_subplot(gs[1, 1])
        scatter = ax6.scatter(test_svd_A[:, 0], test_svd_A[:, 2], 
                            c=svd_result['error_magnitudes']*1000, cmap='viridis', s=20, alpha=0.7)
        plt.colorbar(scatter, ax=ax6, label='Error [mm]')
        ax6.set_xlabel('X [m]')
        ax6.set_ylabel('Z [m]')
        ax6.set_title('SVD Only - Spatial Error Distribution')
        ax6.grid(True, alpha=0.3)
        ax6.axis('equal')
        
        # 7. 空间误差分布 - GPR (All Data)
        ax7 = fig.add_subplot(gs[1, 2])
        scatter = ax7.scatter(all_result['transformed_points'][:, 0], all_result['transformed_points'][:, 2], 
                            c=all_result['error_magnitudes']*1000, cmap='viridis', s=20, alpha=0.7)
        plt.colorbar(scatter, ax=ax7, label='Error [mm]')
        ax7.set_xlabel('X [m]')
        ax7.set_ylabel('Z [m]')
        ax7.set_title('GPR (All Data) - Spatial Error Distribution')
        ax7.grid(True, alpha=0.3)
        ax7.axis('equal')
        
        # 8. 空间误差分布 - GPR (Inliers)
        ax8 = fig.add_subplot(gs[1, 3])
        scatter = ax8.scatter(inliers_result['transformed_points'][:, 0], inliers_result['transformed_points'][:, 2], 
                            c=inliers_result['error_magnitudes']*1000, cmap='viridis', s=20, alpha=0.7)
        plt.colorbar(scatter, ax=ax8, label='Error [mm]')
        ax8.set_xlabel('X [m]')
        ax8.set_ylabel('Z [m]')
        ax8.set_title('GPR (Inliers) - Spatial Error Distribution')
        ax8.grid(True, alpha=0.3)
        ax8.axis('equal')
        
        # 9. 形变场对比
        ax9 = fig.add_subplot(gs[2, 0])
        step = max(1, len(test_svd_A) // 40)
        scale_factor = 15
        
        # GPR (All Data) 形变场
        ax9.quiver(test_svd_A[::step, 0], test_svd_A[::step, 2], 
                  all_result['deformation'][::step, 0]*scale_factor, 
                  all_result['deformation'][::step, 2]*scale_factor,
                  angles='xy', scale_units='xy', scale=1, alpha=0.8, color='green', 
                  width=0.003, label='GPR (All Data)')
        
        ax9.scatter(test_svd_A[::step, 0], test_svd_A[::step, 2], c='blue', alpha=0.5, s=15)
        ax9.set_xlabel('X [m]')
        ax9.set_ylabel('Z [m]')
        ax9.set_title(f'GPR (All Data) Deformation (×{scale_factor})')
        ax9.grid(True, alpha=0.3)
        ax9.axis('equal')
        
        # 10. 形变场对比 - Inliers
        ax10 = fig.add_subplot(gs[2, 1])
        ax10.quiver(test_svd_A[::step, 0], test_svd_A[::step, 2], 
                   inliers_result['deformation'][::step, 0]*scale_factor, 
                   inliers_result['deformation'][::step, 2]*scale_factor,
                   angles='xy', scale_units='xy', scale=1, alpha=0.8, color='orange', 
                   width=0.003, label='GPR (Inliers)')
        
        ax10.scatter(test_svd_A[::step, 0], test_svd_A[::step, 2], c='blue', alpha=0.5, s=15)
        ax10.set_xlabel('X [m]')
        ax10.set_ylabel('Z [m]')
        ax10.set_title(f'GPR (Inliers) Deformation (×{scale_factor})')
        ax10.grid(True, alpha=0.3)
        ax10.axis('equal')
        
        # 11. 改进量对比
        ax11 = fig.add_subplot(gs[2, 2])
        models = ['GPR (All Data)', 'GPR (Inliers)']
        improvements = [all_result['improvement'], inliers_result['improvement']]
        colors = ['green', 'orange']
        
        bars = ax11.bar(models, improvements, color=colors, alpha=0.8)
        ax11.set_ylabel('Improvement [mm]')
        ax11.set_title('Improvement Over SVD Baseline')
        ax11.grid(True, alpha=0.3)
        
        for bar, improvement in zip(bars, improvements):
            ax11.text(bar.get_x() + bar.get_width()/2, bar.get_height() + 0.01,
                     f'{improvement:.3f}mm\n({improvement/(svd_result["rmse_overall"]*1000)*100:.1f}%)', 
                     ha='center', va='bottom', fontweight='bold')
        
        # 12. 处理时间和统计信息
        ax12 = fig.add_subplot(gs[2, 3])
        processing_stats_text = f"""Processing Statistics:

Total Time: {self.pipeline_stats['processing_time']:.2f}s
Total Points: {self.pipeline_stats['total_points']}
Train Points: {self.pipeline_stats['train_points']}
Test Points: {self.pipeline_stats['test_points']}

Voxel Size: {self.voxel_sampler.optimal_voxel_size:.6f}
RANSAC Inliers: {self.stage2.ransac_stats['inlier_ratio']*100:.1f}%

Best Model: {self.pipeline_stats['best_model_name']}
Best RMSE: {self.pipeline_stats['best_result']['rmse_overall']*1000:.3f}mm
Target: {TARGET_RMSE*1000:.2f}mm"""
        
        ax12.text(0.05, 0.95, processing_stats_text, transform=ax12.transAxes, fontsize=10,
                 verticalalignment='top', fontfamily='monospace')
        ax12.set_xlim(0, 1)
        ax12.set_ylim(0, 1)
        ax12.set_title('Processing Statistics')
        ax12.axis('off')
        
        # 13. 不确定性分析 - GPR (All Data)
        ax13 = fig.add_subplot(gs[3, 0])
        if all_result['uncertainty'] is not None:
            uncertainty_all = np.sqrt(all_result['uncertainty']['std_x']**2 + 
                                    all_result['uncertainty']['std_z']**2)
            scatter = ax13.scatter(all_result['transformed_points'][:, 0], all_result['transformed_points'][:, 2], 
                                 c=uncertainty_all, cmap='plasma', s=20, alpha=0.7)
            plt.colorbar(scatter, ax=ax13, label='Uncertainty')
            ax13.set_xlabel('X [m]')
            ax13.set_ylabel('Z [m]')
            ax13.set_title('GPR (All Data) Uncertainty')
            ax13.grid(True, alpha=0.3)
            ax13.axis('equal')
        else:
            ax13.text(0.5, 0.5, 'Uncertainty data\nnot available', 
                     ha='center', va='center', transform=ax13.transAxes, fontsize=12)
            ax13.set_title('GPR (All Data) Uncertainty')
        
        # 14. 不确定性分析 - GPR (Inliers)
        ax14 = fig.add_subplot(gs[3, 1])
        if inliers_result['uncertainty'] is not None:
            uncertainty_inliers = np.sqrt(inliers_result['uncertainty']['std_x']**2 + 
                                        inliers_result['uncertainty']['std_z']**2)
            scatter = ax14.scatter(inliers_result['transformed_points'][:, 0], inliers_result['transformed_points'][:, 2], 
                                 c=uncertainty_inliers, cmap='plasma', s=20, alpha=0.7)
            plt.colorbar(scatter, ax=ax14, label='Uncertainty')
            ax14.set_xlabel('X [m]')
            ax14.set_ylabel('Z [m]')
            ax14.set_title('GPR (Inliers) Uncertainty')
            ax14.grid(True, alpha=0.3)
            ax14.axis('equal')
        else:
            ax14.text(0.5, 0.5, 'Uncertainty data\nnot available', 
                     ha='center', va='center', transform=ax14.transAxes, fontsize=12)
            ax14.set_title('GPR (Inliers) Uncertainty')
        
        # 15. 流水线阶段性RMSE进展
        ax15 = fig.add_subplot(gs[3, 2])
        stages = ['SVD\nBaseline', 'GPR\n(All Data)', 'GPR\n(Inliers)']
        rmse_values = [
            svd_result['rmse_overall']*1000,
            all_result['rmse_overall']*1000,
            inliers_result['rmse_overall']*1000
        ]
        colors = ['blue', 'green', 'orange']
        
        bars = ax15.bar(stages, rmse_values, color=colors, alpha=0.8)
        ax15.axhline(y=TARGET_RMSE*1000, color='red', linestyle='--', alpha=0.7, 
                    label=f'Target: {TARGET_RMSE*1000:.1f}mm')
        ax15.set_ylabel('RMSE [mm]')
        ax15.set_title('Pipeline RMSE Progress (Test Set)')
        ax15.legend()
        ax15.grid(True, alpha=0.3)
        
        # 在柱子上添加数值标签
        for bar, value in zip(bars, rmse_values):
            ax15.text(bar.get_x() + bar.get_width()/2, bar.get_height() + 0.05,
                     f'{value:.2f}mm', ha='center', va='bottom', fontweight='bold')
        
        # 16. 最终评估总结
        ax16 = fig.add_subplot(gs[3, 3])
        best_model_name = self.pipeline_stats['best_model_name']
        best_rmse = self.pipeline_stats['best_result']['rmse_overall']*1000
        target_achieved = self.pipeline_stats['best_result']['success']
        
        final_summary_text = f"""FINAL EVALUATION SUMMARY

Dataset Split:
• Training: {len(train_indices)} points ({len(train_indices)/len(original_A)*100:.1f}%)
• Test: {len(test_indices)} points ({len(test_indices)/len(original_A)*100:.1f}%)

Test Set Results:
• SVD Baseline: {svd_result['rmse_overall']*1000:.3f}mm
• GPR (All): {all_result['rmse_overall']*1000:.3f}mm
• GPR (Inliers): {inliers_result['rmse_overall']*1000:.3f}mm

Best Performance:
• Model: {best_model_name}
• RMSE: {best_rmse:.3f}mm
• Target: {TARGET_RMSE*1000:.2f}mm
• Success: {'✓ YES' if target_achieved else '✗ NO'}

Real-world Equivalent:
• Model: {best_rmse/1000*SCALE_FACTOR*100:.2f}cm
• Target: {REAL_WORLD_LATERAL_TARGET*100:.0f}cm"""
        
        ax16.text(0.05, 0.95, final_summary_text, transform=ax16.transAxes, fontsize=9,
                 verticalalignment='top', fontfamily='monospace',
                 bbox=dict(boxstyle='round,pad=0.5', facecolor='lightgray', alpha=0.8))
        ax16.set_xlim(0, 1)
        ax16.set_ylim(0, 1)
        ax16.set_title('Final Evaluation Summary')
        ax16.axis('off')
        
        # 添加总标题
        success_indicator = "✓ SUCCESS" if target_achieved else "⚠ PARTIAL SUCCESS"
        fig.suptitle(f'Enhanced Three-Stage Registration Pipeline: Voxel Sampling → SVD → RANSAC → GPR\n'
                    f'Best Model: {best_model_name} | Test RMSE: {best_rmse:.3f}mm | Target: {TARGET_RMSE*1000:.2f}mm | {success_indicator}', 
                    fontsize=16, fontweight='bold')
        
        plt.savefig(os.path.join(img_dir, 'comprehensive_enhanced_pipeline_results.png'), 
                   dpi=300, bbox_inches='tight')
        plt.close()
        
        logger.info("Comprehensive enhanced pipeline visualization saved: comprehensive_enhanced_pipeline_results.png")

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
    logger.info("Starting Enhanced Three-Stage Registration Pipeline with Adaptive Voxel Sampling")
    logger.info(f"Scale factor: 1:{SCALE_FACTOR}")
    logger.info(f"Target RMSE: {TARGET_RMSE*1000:.2f}mm")
    logger.info(f"Train/Test ratio: {TRAIN_RATIO:.1f}/{TEST_RATIO:.1f}")
    logger.info("Enhanced Pipeline Features:")
    logger.info("  - Adaptive voxel-based spatial sampling for train/test split")
    logger.info("  - Stage 1: SVD coarse alignment (trained on training set)")
    logger.info("  - Stage 2: RANSAC inlier selection (trained on training set)")  
    logger.info("  - Stage 3: Dual GPR models (All Data vs Inliers, evaluated on test set)")
    logger.info("  - Comprehensive test set evaluation and comparison")
    
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
    
    # 设置随机种子以保证结果可重现
    np.random.seed(42)
    
    # 运行增强版三阶段配准流水线
    pipeline = EnhancedThreeStageRegistrationPipeline(positions_A, positions_B)
    evaluation_results = pipeline.run_complete_pipeline()
    
    # 输出最终结果
    if evaluation_results is not None:
        # 获取最佳模型结果
        all_data_result = evaluation_results['gpr_all_data']
        inliers_result = evaluation_results['gpr_inliers']
        svd_result = evaluation_results['svd_only']
        
        # 确定最佳模型
        best_model_name = 'GPR (All Data)' if all_data_result['rmse_overall'] < inliers_result['rmse_overall'] else 'GPR (Inliers)'
        best_result = all_data_result if all_data_result['rmse_overall'] < inliers_result['rmse_overall'] else inliers_result
        
        overall_success = best_result['success']
        
        logger.info("\n" + "="*100)
        logger.info("FINAL ENHANCED PIPELINE RESULTS SUMMARY")
        logger.info("="*100)
        
        if overall_success:
            logger.info("🎉 SUCCESS: Enhanced three-stage registration completed successfully!")
            logger.info(f"✓ Best Model: {best_model_name}")
            logger.info(f"✓ Test RMSE: {best_result['rmse_overall']*1000:.3f}mm ≤ Target: {TARGET_RMSE*1000:.2f}mm")
        else:
            logger.info("⚠️  PARTIAL SUCCESS: Registration completed but target not achieved")
            logger.info(f"Best Model: {best_model_name}")
            logger.info(f"✗ Test RMSE: {best_result['rmse_overall']*1000:.3f}mm > Target: {TARGET_RMSE*1000:.2f}mm")
            logger.info(f"  Deficit: {(best_result['rmse_overall'] - TARGET_RMSE)*1000:.3f}mm")
        
        logger.info("\nDETAILED RESULTS:")
        logger.info(f"Dataset: {evaluation_results['test_points']} test points")
        logger.info(f"SVD Baseline RMSE: {svd_result['rmse_overall']*1000:.3f}mm")
        logger.info(f"GPR (All Data) RMSE: {all_data_result['rmse_overall']*1000:.3f}mm (Improvement: {all_data_result['improvement']:.3f}mm)")
        logger.info(f"GPR (Inliers) RMSE: {inliers_result['rmse_overall']*1000:.3f}mm (Improvement: {inliers_result['improvement']:.3f}mm)")
        logger.info(f"Processing time: {pipeline.pipeline_stats['processing_time']:.2f}s")
        logger.info(f"Real-world equivalent: {best_result['rmse_overall']*SCALE_FACTOR*100:.2f}cm")
        
        logger.info("\nMODEL COMPARISON:")
        logger.info(f"GPR (All Data) Success: {'YES' if all_data_result['success'] else 'NO'}")
        logger.info(f"GPR (Inliers) Success: {'YES' if inliers_result['success'] else 'NO'}")
        
        # 保存最终结果摘要
        summary_file = os.path.join(output_dir, "ENHANCED_FINAL_RESULTS_SUMMARY.txt")
        with open(summary_file, "w") as f:
            f.write("ENHANCED THREE-STAGE REGISTRATION FINAL RESULTS\n")
            f.write("=" * 80 + "\n")
            f.write(f"Timestamp: {datetime.datetime.now()}\n")
            f.write(f"Pipeline: Voxel Sampling → SVD → RANSAC → GPR (Dual Models)\n")
            f.write(f"Overall Success: {'YES' if overall_success else 'NO'}\n")
            f.write(f"Best Model: {best_model_name}\n")
            f.write(f"Best Test RMSE: {best_result['rmse_overall']*1000:.3f}mm\n")
            f.write(f"Target RMSE: {TARGET_RMSE*1000:.2f}mm\n")
            f.write(f"Scale Factor: 1:{SCALE_FACTOR}\n")
            f.write(f"Real-world Equivalent: {best_result['rmse_overall']*SCALE_FACTOR*100:.2f}cm\n")
            f.write(f"Processing Time: {pipeline.pipeline_stats['processing_time']:.2f}s\n")
            f.write(f"Total Points: {pipeline.pipeline_stats['total_points']}\n")
            f.write(f"Test Points: {evaluation_results['test_points']}\n")
            f.write(f"Train Points: {pipeline.pipeline_stats['train_points']}\n")
            f.write(f"RANSAC Inlier Ratio: {pipeline.pipeline_stats['stage2_stats']['inlier_ratio']*100:.1f}%\n")
            f.write("\n")
            f.write("DETAILED RESULTS:\n")
            f.write(f"SVD Baseline: {svd_result['rmse_overall']*1000:.3f}mm\n")
            f.write(f"GPR (All Data): {all_data_result['rmse_overall']*1000:.3f}mm (Improvement: {all_data_result['improvement']:.3f}mm)\n")
            f.write(f"GPR (Inliers): {inliers_result['rmse_overall']*1000:.3f}mm (Improvement: {inliers_result['improvement']:.3f}mm)\n")
            f.write(f"GPR (All Data) Success: {'YES' if all_data_result['success'] else 'NO'}\n")
            f.write(f"GPR (Inliers) Success: {'YES' if inliers_result['success'] else 'NO'}\n")
        
        logger.info(f"Final summary saved to: {summary_file}")
        
        # 保存详细的测试集结果数据
        test_results_file = os.path.join(data_dir, "test_set_evaluation_results.npz")
        np.savez(test_results_file,
                test_points=evaluation_results['test_points'],
                test_positions_B=evaluation_results['svd_only']['errors'] + evaluation_results['gpr_all_data']['transformed_points'],  # 重构目标点
                svd_transformed=evaluation_results['gpr_all_data']['transformed_points'] - evaluation_results['gpr_all_data']['deformation'],  # 重构SVD变换点
                gpr_all_transformed=evaluation_results['gpr_all_data']['transformed_points'],
                gpr_inliers_transformed=evaluation_results['gpr_inliers']['transformed_points'],
                svd_errors=evaluation_results['svd_only']['errors'],
                gpr_all_errors=evaluation_results['gpr_all_data']['errors'],
                gpr_inliers_errors=evaluation_results['gpr_inliers']['errors'],
                gpr_all_deformation=evaluation_results['gpr_all_data']['deformation'],
                gpr_inliers_deformation=evaluation_results['gpr_inliers']['deformation'],
                target_rmse=TARGET_RMSE,
                svd_rmse=evaluation_results['svd_only']['rmse_overall'],
                gpr_all_rmse=evaluation_results['gpr_all_data']['rmse_overall'],
                gpr_inliers_rmse=evaluation_results['gpr_inliers']['rmse_overall'],
                gpr_all_success=evaluation_results['gpr_all_data']['success'],
                gpr_inliers_success=evaluation_results['gpr_inliers']['success'],
                best_model_name=best_model_name
                )
        
        logger.info(f"Detailed test results saved to: {test_results_file}")
        
    else:
        logger.error("Enhanced three-stage registration pipeline failed")
    
    logger.info("Enhanced three-stage registration processing complete")

if __name__ == "__main__":
    main()