#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Tracker data processing script with RANSAC + SVD + Non-rigid Deformation Learning (Y-axis constrained)
Enhanced with spatial voxel-based train/test split and comprehensive evaluation
"""
import sys
import os
import datetime
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from sklearn.neighbors import NearestNeighbors
from scipy.interpolate import griddata, RBFInterpolator
from scipy.spatial.distance import cdist
import random

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

# RANSAC 参数
RANSAC_MIN_SAMPLES = 32  # 最少点数
RANSAC_ITERATIONS = 2000  # 迭代次数
RANSAC_INLIER_THRESHOLD = 0.015  # 内点阈值 30mm (0.030m)

# 训练/测试集划分参数
TRAIN_TEST_RATIO = 0.8  # 80%用于训练，20%用于测试

# Create output directory
script_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(script_dir)
result_dir = os.path.join(parent_dir, "results")
LASER_TRACKER_CSV = os.path.join(parent_dir, "data/combined_corrected_tracker_data_2.csv")
csv_filename = os.path.basename(LASER_TRACKER_CSV)
output_dir = os.path.join(result_dir, f"Enhanced_RANSAC_SVD_Deformation_{timestamp}_1to32_scale_RMSE_{TARGET_RMSE*1000:.1f}mm_{csv_filename}")
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

class SpatialDataSplitter:
    """
    基于空间分布的数据集划分器（类似体素格采样）
    """
    def __init__(self, train_ratio=0.8, min_points_per_voxel=2):
        self.train_ratio = train_ratio
        self.min_points_per_voxel = min_points_per_voxel
        self.optimal_voxel_size = None
        self.voxel_grid = None
        
    def find_optimal_voxel_size(self, points, min_size=0.01, max_size=0.5, num_trials=20):
        """
        迭代找出能够分出格内数据分布最均匀的格子边长
        """
        logger.info("Finding optimal voxel size for spatial sampling...")
        
        points = np.array(points)
        # 只使用XZ坐标（因为Y被约束为0）
        points_xz = points[:, [0, 2]]
        
        x_min, x_max = np.min(points_xz[:, 0]), np.max(points_xz[:, 0])
        z_min, z_max = np.min(points_xz[:, 1]), np.max(points_xz[:, 1])
        
        best_size = None
        best_uniformity = float('inf')
        size_results = []
        
        # 尝试不同的体素大小
        for voxel_size in np.linspace(min_size, max_size, num_trials):
            # 计算网格数量
            nx = int(np.ceil((x_max - x_min) / voxel_size))
            nz = int(np.ceil((z_max - z_min) / voxel_size))
            
            if nx == 0 or nz == 0:
                continue
                
            # 为每个点分配体素索引
            voxel_indices = self._assign_voxel_indices(points_xz, x_min, z_min, voxel_size)
            
            # 计算每个体素中的点数
            unique_voxels, voxel_counts = np.unique(voxel_indices, return_counts=True)
            
            # 过滤掉点数过少的体素
            valid_voxels = voxel_counts >= self.min_points_per_voxel
            if np.sum(valid_voxels) < 2:  # 至少需要2个有效体素
                continue
                
            valid_counts = voxel_counts[valid_voxels]
            
            # 计算分布均匀性（使用标准差作为衡量标准）
            uniformity = np.std(valid_counts) / np.mean(valid_counts) if len(valid_counts) > 1 else float('inf')
            
            size_results.append({
                'voxel_size': voxel_size,
                'num_voxels': len(unique_voxels),
                'valid_voxels': np.sum(valid_voxels),
                'uniformity': uniformity,
                'mean_points': np.mean(valid_counts),
                'std_points': np.std(valid_counts)
            })
            
            if uniformity < best_uniformity:
                best_uniformity = uniformity
                best_size = voxel_size
        
        if best_size is None:
            # 如果没有找到合适的尺寸，使用默认值
            best_size = (x_max - x_min + z_max - z_min) / 20
            logger.warning(f"No optimal voxel size found, using default: {best_size:.4f}m")
        else:
            logger.info(f"Optimal voxel size found: {best_size:.4f}m (uniformity: {best_uniformity:.4f})")
        
        self.optimal_voxel_size = best_size
        
        # 保存分析结果
        self._save_voxel_analysis(size_results, points_xz)
        
        return best_size
    
    def _assign_voxel_indices(self, points_xz, x_min, z_min, voxel_size):
        """为点分配体素索引"""
        voxel_x = np.floor((points_xz[:, 0] - x_min) / voxel_size).astype(int)
        voxel_z = np.floor((points_xz[:, 1] - z_min) / voxel_size).astype(int)
        
        # 创建唯一的体素索引
        max_z = int(np.max(voxel_z)) + 1
        voxel_indices = voxel_x * max_z + voxel_z
        
        return voxel_indices
    
    def split_data_spatially(self, positions_A, positions_B, df=None):
        """
        基于空间分布划分训练集和测试集
        """
        logger.info("=== Starting Spatial Data Splitting ===")
        
        positions_A = np.array(positions_A)
        positions_B = np.array(positions_B)
        
        # 找到最优体素大小
        if self.optimal_voxel_size is None:
            self.find_optimal_voxel_size(positions_A)
        
        # 只使用XZ坐标进行体素分配
        points_xz = positions_A[:, [0, 2]]
        x_min, x_max = np.min(points_xz[:, 0]), np.max(points_xz[:, 0])
        z_min, z_max = np.min(points_xz[:, 1]), np.max(points_xz[:, 1])
        
        # 分配体素索引
        voxel_indices = self._assign_voxel_indices(points_xz, x_min, z_min, self.optimal_voxel_size)
        
        # 统计每个体素中的点数
        unique_voxels, voxel_counts = np.unique(voxel_indices, return_counts=True)
        
        # 过滤有效体素
        valid_voxel_mask = voxel_counts >= self.min_points_per_voxel
        valid_voxels = unique_voxels[valid_voxel_mask]
        valid_counts = voxel_counts[valid_voxel_mask]
        
        logger.info(f"Total voxels: {len(unique_voxels)}, Valid voxels: {len(valid_voxels)}")
        logger.info(f"Points per voxel - Mean: {np.mean(valid_counts):.1f}, Std: {np.std(valid_counts):.1f}")
        
        # 为每个有效体素按比例选择训练点
        train_indices = []
        test_indices = []
        
        for voxel_id in valid_voxels:
            # 找到属于当前体素的所有点
            voxel_point_indices = np.where(voxel_indices == voxel_id)[0]
            
            # 计算该体素中应选择的训练点数量
            total_points_in_voxel = len(voxel_point_indices)
            train_points_needed = max(1, int(total_points_in_voxel * self.train_ratio))
            
            # 随机选择训练点
            np.random.shuffle(voxel_point_indices)
            voxel_train_indices = voxel_point_indices[:train_points_needed]
            voxel_test_indices = voxel_point_indices[train_points_needed:]
            
            train_indices.extend(voxel_train_indices)
            test_indices.extend(voxel_test_indices)
        
        # 转换为numpy数组
        train_indices = np.array(train_indices)
        test_indices = np.array(test_indices)
        
        # 提取训练集和测试集
        train_A = positions_A[train_indices]
        train_B = positions_B[train_indices]
        test_A = positions_A[test_indices]
        test_B = positions_B[test_indices]
        
        train_df = df.iloc[train_indices] if df is not None else None
        test_df = df.iloc[test_indices] if df is not None else None
        
        logger.info(f"Data split completed:")
        logger.info(f"  Total points: {len(positions_A)}")
        logger.info(f"  Training points: {len(train_indices)} ({len(train_indices)/len(positions_A)*100:.1f}%)")
        logger.info(f"  Testing points: {len(test_indices)} ({len(test_indices)/len(positions_A)*100:.1f}%)")
        
        # 可视化数据划分
        self._visualize_data_split(positions_A, train_indices, test_indices, voxel_indices, valid_voxels)
        
        # 保存划分结果
        self._save_split_data(train_A, train_B, test_A, test_B, train_df, test_df, train_indices, test_indices)
        
        return {
            'train_A': train_A,
            'train_B': train_B, 
            'test_A': test_A,
            'test_B': test_B,
            'train_df': train_df,
            'test_df': test_df,
            'train_indices': train_indices,
            'test_indices': test_indices
        }
    
    def _visualize_data_split(self, positions_A, train_indices, test_indices, voxel_indices, valid_voxels):
        """可视化数据划分结果"""
        logger.info("Creating data split visualization...")
        
        fig, ((ax1, ax2), (ax3, ax4)) = plt.subplots(2, 2, figsize=(16, 12))
        
        # 原始数据分布
        ax1.scatter(positions_A[:, 0], positions_A[:, 2], c='blue', s=20, alpha=0.6, label='All Points')
        ax1.set_xlabel('X [m]')
        ax1.set_ylabel('Z [m]')
        ax1.set_title(f'Original Data Distribution ({len(positions_A)} points)')
        ax1.legend()
        ax1.grid(True, alpha=0.3)
        ax1.axis('equal')
        
        # 训练集分布
        train_positions = positions_A[train_indices]
        ax2.scatter(train_positions[:, 0], train_positions[:, 2], c='green', s=20, alpha=0.7, label='Training Points')
        ax2.set_xlabel('X [m]')
        ax2.set_ylabel('Z [m]')
        ax2.set_title(f'Training Set ({len(train_indices)} points)')
        ax2.legend()
        ax2.grid(True, alpha=0.3)
        ax2.axis('equal')
        
        # 测试集分布
        test_positions = positions_A[test_indices]
        ax3.scatter(test_positions[:, 0], test_positions[:, 2], c='red', s=20, alpha=0.7, label='Testing Points')
        ax3.set_xlabel('X [m]')
        ax3.set_ylabel('Z [m]')
        ax3.set_title(f'Test Set ({len(test_indices)} points)')
        ax3.legend()
        ax3.grid(True, alpha=0.3)
        ax3.axis('equal')
        
        # 体素网格可视化
        colors = plt.cm.tab20(np.linspace(0, 1, len(valid_voxels)))
        for i, voxel_id in enumerate(valid_voxels):
            voxel_points = positions_A[voxel_indices == voxel_id]
            ax4.scatter(voxel_points[:, 0], voxel_points[:, 2], c=[colors[i]], s=25, 
                       alpha=0.7, label=f'Voxel {i}' if i < 10 else "")
        
        ax4.set_xlabel('X [m]')
        ax4.set_ylabel('Z [m]')
        ax4.set_title(f'Voxel Grid (size: {self.optimal_voxel_size:.4f}m)')
        if len(valid_voxels) <= 10:
            ax4.legend()
        ax4.grid(True, alpha=0.3)
        ax4.axis('equal')
        
        plt.tight_layout()
        plt.savefig(os.path.join(img_dir, 'spatial_data_split.png'), dpi=300, bbox_inches='tight')
        plt.close()
        
        # 创建训练/测试集覆盖图
        fig, ax = plt.subplots(1, 1, figsize=(12, 8))
        ax.scatter(train_positions[:, 0], train_positions[:, 2], c='green', s=30, alpha=0.6, 
                  label=f'Training ({len(train_indices)} points)')
        ax.scatter(test_positions[:, 0], test_positions[:, 2], c='red', s=30, alpha=0.6, 
                  label=f'Testing ({len(test_indices)} points)')
        ax.set_xlabel('X [m]')
        ax.set_ylabel('Z [m]')
        ax.set_title('Training/Test Split Spatial Distribution')
        ax.legend()
        ax.grid(True, alpha=0.3)
        ax.axis('equal')
        
        plt.tight_layout()
        plt.savefig(os.path.join(img_dir, 'train_test_distribution.png'), dpi=300, bbox_inches='tight')
        plt.close()
        
        logger.info("Data split visualization saved")
    
    def _save_voxel_analysis(self, size_results, points_xz):
        """保存体素分析结果"""
        if not size_results:
            return
            
        # 保存体素分析数据
        analysis_df = pd.DataFrame(size_results)
        analysis_df.to_csv(os.path.join(result_dir, 'voxel_size_analysis.csv'), index=False)
        
        # 创建体素分析图
        fig, ((ax1, ax2), (ax3, ax4)) = plt.subplots(2, 2, figsize=(16, 12))
        
        sizes = analysis_df['voxel_size']
        
        # 均匀性随体素大小变化
        ax1.plot(sizes, analysis_df['uniformity'], 'b-o', markersize=4)
        ax1.set_xlabel('Voxel Size [m]')
        ax1.set_ylabel('Uniformity (lower is better)')
        ax1.set_title('Data Distribution Uniformity vs Voxel Size')
        ax1.grid(True, alpha=0.3)
        if self.optimal_voxel_size is not None:
            ax1.axvline(x=self.optimal_voxel_size, color='red', linestyle='--', 
                       label=f'Optimal: {self.optimal_voxel_size:.4f}m')
            ax1.legend()
        
        # 有效体素数量
        ax2.plot(sizes, analysis_df['valid_voxels'], 'g-o', markersize=4)
        ax2.set_xlabel('Voxel Size [m]')
        ax2.set_ylabel('Number of Valid Voxels')
        ax2.set_title('Valid Voxels vs Voxel Size')
        ax2.grid(True, alpha=0.3)
        
        # 平均点数
        ax3.plot(sizes, analysis_df['mean_points'], 'r-o', markersize=4)
        ax3.set_xlabel('Voxel Size [m]')
        ax3.set_ylabel('Mean Points per Voxel')
        ax3.set_title('Mean Points per Voxel vs Voxel Size')
        ax3.grid(True, alpha=0.3)
        
        # 点数标准差
        ax4.plot(sizes, analysis_df['std_points'], 'm-o', markersize=4)
        ax4.set_xlabel('Voxel Size [m]')
        ax4.set_ylabel('Std Points per Voxel')
        ax4.set_title('Standard Deviation of Points per Voxel')
        ax4.grid(True, alpha=0.3)
        
        plt.tight_layout()
        plt.savefig(os.path.join(img_dir, 'voxel_analysis.png'), dpi=300, bbox_inches='tight')
        plt.close()
    
    def _save_split_data(self, train_A, train_B, test_A, test_B, train_df, test_df, train_indices, test_indices):
        """保存划分后的数据"""
        # 保存训练集
        train_data = np.hstack([train_A, train_B])
        np.savetxt(os.path.join(result_dir, "spatial_train_data.txt"), train_data, 
                  header="train_source_x train_source_y train_source_z train_target_x train_target_y train_target_z")
        
        # 保存测试集
        test_data = np.hstack([test_A, test_B])
        np.savetxt(os.path.join(result_dir, "spatial_test_data.txt"), test_data, 
                  header="test_source_x test_source_y test_source_z test_target_x test_target_y test_target_z")
        
        # 保存CSV文件
        if train_df is not None:
            train_df_copy = train_df.copy()
            train_df_copy['split_type'] = 'train'
            train_df_copy.to_csv(os.path.join(result_dir, "spatial_train_data.csv"), index=False)
        
        if test_df is not None:
            test_df_copy = test_df.copy()
            test_df_copy['split_type'] = 'test'
            test_df_copy.to_csv(os.path.join(result_dir, "spatial_test_data.csv"), index=False)
        
        # 保存索引
        np.savetxt(os.path.join(result_dir, "train_indices.txt"), train_indices, fmt='%d')
        np.savetxt(os.path.join(result_dir, "test_indices.txt"), test_indices, fmt='%d')
        
        # 保存划分摘要
        with open(os.path.join(result_dir, "spatial_split_summary.txt"), "w") as f:
            f.write("Spatial Data Split Summary\n")
            f.write("=" * 40 + "\n")
            f.write(f"Optimal voxel size: {self.optimal_voxel_size:.6f}m\n")
            f.write(f"Train ratio target: {self.train_ratio:.2f}\n")
            f.write(f"Actual train ratio: {len(train_indices)/(len(train_indices)+len(test_indices)):.3f}\n")
            f.write(f"Training points: {len(train_indices)}\n")
            f.write(f"Testing points: {len(test_indices)}\n")
            f.write(f"Total points: {len(train_indices) + len(test_indices)}\n")
            f.write(f"Min points per voxel: {self.min_points_per_voxel}\n")
        
        logger.info("Spatial split data saved")

class RANSACFilter:
    """
    RANSAC-based data filtering for point cloud registration
    """
    def __init__(self, min_samples=12, max_iterations=100, inlier_threshold=0.030):
        self.min_samples = min_samples
        self.max_iterations = max_iterations
        self.inlier_threshold = inlier_threshold
        self.best_inliers = None
        self.best_transformation = None
        self.iteration_stats = []
        
    def compute_2d_svd_transformation(self, points_A, points_B):
        """计算2D SVD变换（只考虑XZ平面）"""
        points_A = np.array(points_A)
        points_B = np.array(points_B)
        
        # 只使用XZ坐标
        points_A_xz = points_A[:, [0, 2]]
        points_B_xz = points_B[:, [0, 2]]
        
        # 计算质心
        centroid_A = np.mean(points_A_xz, axis=0)
        centroid_B = np.mean(points_B_xz, axis=0)
        
        # 去中心化
        A_centered = points_A_xz - centroid_A
        B_centered = points_B_xz - centroid_B
        
        # SVD计算旋转矩阵
        H = np.dot(A_centered.T, B_centered)
        U, S, Vt = np.linalg.svd(H)
        R = np.dot(Vt.T, U.T)
        
        # 确保旋转矩阵的行列式为正
        if np.linalg.det(R) < 0:
            Vt[-1, :] *= -1
            R = np.dot(Vt.T, U.T)
        
        # 计算平移向量
        t = centroid_B - np.dot(R, centroid_A)
        
        return R, t
    
    def apply_2d_transformation(self, points, R, t):
        """应用2D变换到点集"""
        points = np.array(points)
        points_xz = points[:, [0, 2]]
        
        # 应用变换
        transformed_xz = np.dot(points_xz, R.T) + t
        
        # 重构3D点（保持Y=0）
        transformed = np.zeros_like(points)
        transformed[:, [0, 2]] = transformed_xz
        transformed[:, 1] = 0  # Y轴约束
        
        return transformed
    
    def count_inliers(self, positions_A, positions_B, R, t):
        """计算内点数量"""
        # 应用变换
        transformed_A = self.apply_2d_transformation(positions_A, R, t)
        
        # 计算变换后的距离（只考虑XZ平面）
        distances = np.sqrt((transformed_A[:, 0] - positions_B[:, 0])**2 + 
                           (transformed_A[:, 2] - positions_B[:, 2])**2)
        
        # 统计内点
        inliers = distances < self.inlier_threshold
        
        return inliers, distances
    
    def ransac_filter(self, positions_A, positions_B):
        """执行RANSAC滤波"""
        logger.info("=== Starting RANSAC Data Filtering ===")
        
        positions_A = np.array(positions_A)
        positions_B = np.array(positions_B)
        
        if len(positions_A) < self.min_samples:
            logger.error(f"Only {len(positions_A)} points available, less than minimum required {self.min_samples}")
            return None, None, None, None
        
        best_inlier_count = 0
        best_inliers = None
        best_R = None
        best_t = None
        
        logger.info(f"RANSAC parameters:")
        logger.info(f"  - Min samples: {self.min_samples}")
        logger.info(f"  - Max iterations: {self.max_iterations}")
        logger.info(f"  - Inlier threshold: {self.inlier_threshold*1000:.1f}mm")
        logger.info(f"  - Total points: {len(positions_A)}")
        
        # RANSAC迭代
        for iteration in range(self.max_iterations):
            # 随机采样
            if len(positions_A) <= self.min_samples:
                sample_indices = np.arange(len(positions_A))
            else:
                sample_indices = np.random.choice(len(positions_A), self.min_samples, replace=False)
            
            sample_A = positions_A[sample_indices]
            sample_B = positions_B[sample_indices]
            
            try:
                # 计算变换
                R, t = self.compute_2d_svd_transformation(sample_A, sample_B)
                
                # 验证变换
                inliers, distances = self.count_inliers(positions_A, positions_B, R, t)
                inlier_count = np.sum(inliers)
                
                # 记录统计信息
                self.iteration_stats.append({
                    'iteration': iteration,
                    'inlier_count': inlier_count,
                    'inlier_ratio': inlier_count / len(positions_A),
                    'mean_distance': np.mean(distances[inliers]) if inlier_count > 0 else float('inf')
                })
                
                # 更新最佳模型
                if inlier_count > best_inlier_count:
                    best_inlier_count = inlier_count
                    best_inliers = inliers.copy()
                    best_R = R.copy()
                    best_t = t.copy()
                    
                    logger.info(f"  Iteration {iteration}: New best model with {inlier_count}/{len(positions_A)} inliers ({inlier_count/len(positions_A)*100:.1f}%)")
                
            except Exception as e:
                logger.warning(f"  Iteration {iteration}: Failed to compute transformation: {e}")
                continue
        
        if best_inliers is None:
            logger.error("RANSAC failed to find any valid transformation")
            return None, None, None, None
        
        # 提取最佳内点
        inlier_A = positions_A[best_inliers]
        inlier_B = positions_B[best_inliers]
        outlier_A = positions_A[~best_inliers]
        outlier_B = positions_B[~best_inliers]
        
        logger.info(f"RANSAC completed:")
        logger.info(f"  - Best inlier count: {best_inlier_count}/{len(positions_A)} ({best_inlier_count/len(positions_A)*100:.1f}%)")
        logger.info(f"  - Outliers removed: {len(positions_A) - best_inlier_count}")
        
        # 存储结果
        self.best_inliers = best_inliers
        self.best_transformation = (best_R, best_t)
        
        return inlier_A, inlier_B, outlier_A, outlier_B
    
    def visualize_ransac_results(self, original_A, original_B, inlier_A, inlier_B, outlier_A, outlier_B, stage_name=""):
        """可视化RANSAC结果"""
        logger.info("Creating RANSAC visualization...")
        
        # 确保所有数组都是numpy数组
        original_A = np.array(original_A)
        original_B = np.array(original_B)
        
        fig = plt.figure(figsize=(20, 12))
        
        # 3D视图 - 剔除前
        ax1 = fig.add_subplot(231, projection='3d')
        ax1.scatter(original_A[:, 0], original_A[:, 2], original_A[:, 1], 
                   c='blue', marker='o', s=30, label='Source Points', alpha=0.7)
        ax1.scatter(original_B[:, 0], original_B[:, 2], original_B[:, 1], 
                   c='red', marker='^', s=30, label='Target Points', alpha=0.7)
        ax1.set_xlabel('X [m]')
        ax1.set_ylabel('Z [m]')
        ax1.set_zlabel('Y [m]')
        ax1.set_title(f'{stage_name} Before RANSAC (Total: {len(original_A)} points)')
        ax1.legend()
        ax1.set_zlim(-0.1, 0.1)
        
        # 3D视图 - 剔除后（内点）
        ax2 = fig.add_subplot(232, projection='3d')
        if len(inlier_A) > 0:
            ax2.scatter(inlier_A[:, 0], inlier_A[:, 2], inlier_A[:, 1], 
                       c='green', marker='o', s=30, label='Inlier Source', alpha=0.7)
            ax2.scatter(inlier_B[:, 0], inlier_B[:, 2], inlier_B[:, 1], 
                       c='darkgreen', marker='^', s=30, label='Inlier Target', alpha=0.7)
        ax2.set_xlabel('X [m]')
        ax2.set_ylabel('Z [m]')
        ax2.set_zlabel('Y [m]')
        ax2.set_title(f'{stage_name} After RANSAC - Inliers ({len(inlier_A)} points)')
        ax2.legend()
        ax2.set_zlim(-0.1, 0.1)
        
        # 3D视图 - 被剔除的点（外点）
        ax3 = fig.add_subplot(233, projection='3d')
        if len(outlier_A) > 0:
            ax3.scatter(outlier_A[:, 0], outlier_A[:, 2], outlier_A[:, 1], 
                       c='orange', marker='o', s=30, label='Outlier Source', alpha=0.7)
            ax3.scatter(outlier_B[:, 0], outlier_B[:, 2], outlier_B[:, 1], 
                       c='red', marker='^', s=30, label='Outlier Target', alpha=0.7)
        ax3.set_xlabel('X [m]')
        ax3.set_ylabel('Z [m]')
        ax3.set_zlabel('Y [m]')
        ax3.set_title(f'{stage_name} Removed Outliers ({len(outlier_A)} points)')
        ax3.legend()
        ax3.set_zlim(-0.1, 0.1)
        
        # XZ平面视图 - 剔除前
        ax4 = fig.add_subplot(234)
        ax4.scatter(original_A[:, 0], original_A[:, 2], c='blue', marker='o', s=30, 
                   label='Source Points', alpha=0.7)
        ax4.scatter(original_B[:, 0], original_B[:, 2], c='red', marker='^', s=30, 
                   label='Target Points', alpha=0.7)
        ax4.set_xlabel('X (Lateral) [m]')
        ax4.set_ylabel('Z (Longitudinal) [m]')
        ax4.set_title(f'{stage_name} XZ Plane - Before RANSAC ({len(original_A)} points)')
        ax4.legend()
        ax4.grid(True, alpha=0.3)
        ax4.axis('equal')
        
        # XZ平面视图 - 剔除后（内点）
        ax5 = fig.add_subplot(235)
        if len(inlier_A) > 0:
            ax5.scatter(inlier_A[:, 0], inlier_A[:, 2], c='green', marker='o', s=30, 
                       label='Inlier Source', alpha=0.7)
            ax5.scatter(inlier_B[:, 0], inlier_B[:, 2], c='darkgreen', marker='^', s=30, 
                       label='Inlier Target', alpha=0.7)
        ax5.set_xlabel('X (Lateral) [m]')
        ax5.set_ylabel('Z (Longitudinal) [m]')
        ax5.set_title(f'{stage_name} XZ Plane - Inliers ({len(inlier_A)} points)')
        ax5.legend()
        ax5.grid(True, alpha=0.3)
        ax5.axis('equal')
        
        # XZ平面视图 - 被剔除的点
        ax6 = fig.add_subplot(236)
        if len(outlier_A) > 0:
            ax6.scatter(outlier_A[:, 0], outlier_A[:, 2], c='orange', marker='o', s=30, 
                       label='Outlier Source', alpha=0.7)
            ax6.scatter(outlier_B[:, 0], outlier_B[:, 2], c='red', marker='^', s=30, 
                       label='Outlier Target', alpha=0.7)
        ax6.set_xlabel('X (Lateral) [m]')
        ax6.set_ylabel('Z (Longitudinal) [m]')
        ax6.set_title(f'{stage_name} XZ Plane - Outliers ({len(outlier_A)} points)')
        ax6.legend()
        ax6.grid(True, alpha=0.3)
        ax6.axis('equal')
        
        plt.tight_layout()
        
        filename = f'ransac_filtering_results_{stage_name.lower().replace(" ", "_")}.png'
        plt.savefig(os.path.join(img_dir, filename), dpi=300, bbox_inches='tight')
        plt.close()
        
        # 创建RANSAC迭代统计图
        self.plot_ransac_statistics(stage_name)
        
        logger.info("RANSAC visualization saved")
    
    def plot_ransac_statistics(self, stage_name=""):
        """绘制RANSAC迭代统计"""
        if not self.iteration_stats:
            return
        
        iterations = [stat['iteration'] for stat in self.iteration_stats]
        inlier_counts = [stat['inlier_count'] for stat in self.iteration_stats]
        inlier_ratios = [stat['inlier_ratio'] for stat in self.iteration_stats]
        mean_distances = [stat['mean_distance'] for stat in self.iteration_stats if stat['mean_distance'] != float('inf')]
        
        fig, (ax1, ax2, ax3) = plt.subplots(1, 3, figsize=(18, 6))
        
        # 内点数量
        ax1.plot(iterations, inlier_counts, 'b-', alpha=0.7)
        ax1.scatter(iterations, inlier_counts, c='blue', s=20, alpha=0.7)
        if inlier_counts:
            best_iter = iterations[np.argmax(inlier_counts)]
            best_count = max(inlier_counts)
            ax1.axhline(y=best_count, color='red', linestyle='--', alpha=0.7, label=f'Best: {best_count}')
            ax1.scatter([best_iter], [best_count], c='red', s=100, marker='*', label=f'Best iteration: {best_iter}')
        ax1.set_xlabel('Iteration')
        ax1.set_ylabel('Inlier Count')
        ax1.set_title(f'{stage_name} RANSAC Inlier Count per Iteration')
        ax1.legend()
        ax1.grid(True, alpha=0.3)
        
        # 内点比例
        ax2.plot(iterations, [ratio*100 for ratio in inlier_ratios], 'g-', alpha=0.7)
        ax2.scatter(iterations, [ratio*100 for ratio in inlier_ratios], c='green', s=20, alpha=0.7)
        if inlier_ratios:
            best_ratio = max(inlier_ratios) * 100
            ax2.axhline(y=best_ratio, color='red', linestyle='--', alpha=0.7, label=f'Best: {best_ratio:.1f}%')
        ax2.set_xlabel('Iteration')
        ax2.set_ylabel('Inlier Ratio [%]')
        ax2.set_title(f'{stage_name} RANSAC Inlier Ratio per Iteration')
        ax2.legend()
        ax2.grid(True, alpha=0.3)
        
        # 平均距离
        if mean_distances:
            valid_iters = [iterations[i] for i, stat in enumerate(self.iteration_stats) if stat['mean_distance'] != float('inf')]
            ax3.plot(valid_iters, [d*1000 for d in mean_distances], 'orange', alpha=0.7)
            ax3.scatter(valid_iters, [d*1000 for d in mean_distances], c='orange', s=20, alpha=0.7)
            ax3.axhline(y=self.inlier_threshold*1000, color='red', linestyle='--', alpha=0.7, 
                       label=f'Threshold: {self.inlier_threshold*1000:.1f}mm')
        ax3.set_xlabel('Iteration')
        ax3.set_ylabel('Mean Inlier Distance [mm]')
        ax3.set_title(f'{stage_name} RANSAC Mean Inlier Distance per Iteration')
        ax3.legend()
        ax3.grid(True, alpha=0.3)
        
        plt.tight_layout()
        
        filename = f'ransac_statistics_{stage_name.lower().replace(" ", "_")}.png'
        plt.savefig(os.path.join(img_dir, filename), dpi=300, bbox_inches='tight')
        plt.close()

class NonRigidDeformationModel:
    """
    Non-rigid deformation model using Radial Basis Function (RBF) interpolation
    """
    def __init__(self, method='thin_plate_spline', smoothing=0.001, constrain_y=True):
        self.method = method
        self.smoothing = smoothing
        self.constrain_y = constrain_y
        self.rbf_x = None
        self.rbf_z = None
        self.source_points = None
        
    def learn_deformation(self, source_points, residual_vectors):
        """
        学习非刚性形变模型
        source_points: SVD变换后的源点 (N, 3)
        residual_vectors: 残差向量 (N, 3)
        """
        source_points = np.array(source_points)
        residual_vectors = np.array(residual_vectors)
        
        # 只在XZ平面上学习形变
        if self.constrain_y:
            source_xz = source_points[:, [0, 2]]
            residual_xz = residual_vectors[:, [0, 2]]
            
            # 学习X方向的形变
            try:
                self.rbf_x = RBFInterpolator(
                    source_xz, 
                    residual_xz[:, 0], 
                    kernel=self.method,
                    smoothing=self.smoothing
                )
                
                # 学习Z方向的形变
                self.rbf_z = RBFInterpolator(
                    source_xz, 
                    residual_xz[:, 1], 
                    kernel=self.method,
                    smoothing=self.smoothing
                )
                
                self.source_points = source_xz
                logger.info(f"Non-rigid deformation model learned with {len(source_points)} control points")
                return True
                
            except Exception as e:
                logger.error(f"Failed to learn deformation model: {e}")
                return False
        else:
            # 3D形变学习（暂时不实现，因为约束Y轴）
            logger.warning("3D deformation learning not implemented with Y-axis constraint")
            return False
    
    def predict_deformation(self, query_points):
        """
        预测给定点的形变
        query_points: 查询点 (M, 3)
        返回: 形变向量 (M, 3)
        """
        if self.rbf_x is None or self.rbf_z is None:
            logger.error("Deformation model not trained yet")
            return np.zeros_like(query_points)
        
        query_points = np.array(query_points)
        
        if self.constrain_y:
            query_xz = query_points[:, [0, 2]]
            
            try:
                # 预测X和Z方向的形变
                deformation_x = self.rbf_x(query_xz)
                deformation_z = self.rbf_z(query_xz)
                
                # 构造完整的形变向量
                deformation = np.zeros_like(query_points)
                deformation[:, 0] = deformation_x
                deformation[:, 2] = deformation_z
                # Y方向形变保持为0
                
                return deformation
                
            except Exception as e:
                logger.error(f"Failed to predict deformation: {e}")
                return np.zeros_like(query_points)
        else:
            return np.zeros_like(query_points)
    
    def visualize_deformation_field(self, bounds, resolution=50, stage_name=""):
        """
        可视化形变场
        bounds: [x_min, x_max, z_min, z_max]
        resolution: 网格分辨率
        """
        if self.rbf_x is None or self.rbf_z is None:
            logger.warning("Cannot visualize: deformation model not trained")
            return
        
        x_min, x_max, z_min, z_max = bounds
        x_grid = np.linspace(x_min, x_max, resolution)
        z_grid = np.linspace(z_min, z_max, resolution)
        X, Z = np.meshgrid(x_grid, z_grid)
        
        # 构造查询点
        query_points_2d = np.column_stack([X.ravel(), Z.ravel()])
        
        try:
            # 预测形变
            deformation_x = self.rbf_x(query_points_2d)
            deformation_z = self.rbf_z(query_points_2d)
            
            # 重塑为网格
            Dx = deformation_x.reshape(X.shape)
            Dz = deformation_z.reshape(X.shape)
            
            # 可视化
            fig, (ax1, ax2, ax3) = plt.subplots(1, 3, figsize=(18, 6))
            
            # X方向形变
            im1 = ax1.contourf(X, Z, Dx*1000, levels=20, cmap='RdBu_r')
            ax1.quiver(X[::5, ::5], Z[::5, ::5], Dx[::5, ::5]*1000, np.zeros_like(Dx[::5, ::5]), 
                      scale=10, alpha=0.7)
            plt.colorbar(im1, ax=ax1)
            ax1.set_title(f'{stage_name} X-direction Deformation [mm]')
            ax1.set_xlabel('X [m]')
            ax1.set_ylabel('Z [m]')
            
            # Z方向形变
            im2 = ax2.contourf(X, Z, Dz*1000, levels=20, cmap='RdBu_r')
            ax2.quiver(X[::5, ::5], Z[::5, ::5], np.zeros_like(Dz[::5, ::5]), Dz[::5, ::5]*1000, 
                      scale=10, alpha=0.7)
            plt.colorbar(im2, ax=ax2)
            ax2.set_title(f'{stage_name} Z-direction Deformation [mm]')
            ax2.set_xlabel('X [m]')
            ax2.set_ylabel('Z [m]')
            
            # 形变大小
            magnitude = np.sqrt(Dx**2 + Dz**2) * 1000
            im3 = ax3.contourf(X, Z, magnitude, levels=20, cmap='viridis')
            ax3.quiver(X[::5, ::5], Z[::5, ::5], Dx[::5, ::5]*1000, Dz[::5, ::5]*1000, 
                      scale=10, alpha=0.7)
            plt.colorbar(im3, ax=ax3)
            ax3.set_title(f'{stage_name} Deformation Magnitude [mm]')
            ax3.set_xlabel('X [m]')
            ax3.set_ylabel('Z [m]')
            
            # 叠加控制点
            if self.source_points is not None:
                for ax in [ax1, ax2, ax3]:
                    ax.scatter(self.source_points[:, 0], self.source_points[:, 1], 
                             c='black', s=20, marker='o', alpha=0.8, label='Control Points')
                    ax.legend()
            
            plt.tight_layout()
            
            filename = f'deformation_field_{stage_name.lower().replace(" ", "_")}.png'
            plt.savefig(os.path.join(img_dir, filename), dpi=300, bbox_inches='tight')
            plt.close()
            
            logger.info("Deformation field visualization saved")
            
        except Exception as e:
            logger.error(f"Failed to visualize deformation field: {e}")

class CoordinateTransformer:
    """
    Enhanced coordinate transformation with spatial data splitting and comprehensive evaluation
    """
    def __init__(self, positions_A=None, positions_B=None, original_df=None):
        self.T_svd = None
        self.deformation_model_all = None
        self.deformation_model_inliers = None
        self.ransac_filter = None
        self.__positions_A = positions_A
        self.__positions_B = positions_B
        self.__original_df = original_df
        
        # Training data (filtered)
        self.__train_positions_A = None
        self.__train_positions_B = None
        self.__train_inlier_A = None
        self.__train_inlier_B = None
        
        # Testing data
        self.__test_positions_A = None
        self.__test_positions_B = None
        
        # Data splitter
        self.data_splitter = SpatialDataSplitter(train_ratio=TRAIN_TEST_RATIO)
        
    def calculate_svd_transformation(self, positions_A=None, positions_B=None):
        """Calculate SVD transformation with Y-axis constraint"""
        if positions_A is not None and positions_B is not None:
            self.__train_positions_A = positions_A
            self.__train_positions_B = positions_B
        elif self.__train_positions_A is None or self.__train_positions_B is None:
            logger.error("Please provide training position data first!")
            return

        positions_A = np.array(self.__train_positions_A)
        positions_B = np.array(self.__train_positions_B)

        # 只使用XZ坐标进行SVD变换
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
        
        # 构造3D变换矩阵
        self.T_svd = np.eye(4)
        self.T_svd[0, 0] = R_2d[0, 0]
        self.T_svd[0, 2] = R_2d[0, 1]
        self.T_svd[2, 0] = R_2d[1, 0]
        self.T_svd[2, 2] = R_2d[1, 1]
        self.T_svd[0, 3] = translation_2d[0]
        self.T_svd[2, 3] = translation_2d[1]
        
        logger.info("SVD transformation matrix calculated (Y-axis constrained, on training data)")
        return R_2d, translation_2d

    def apply_transformation(self, positions, T):
        """Apply transformation matrix to positions"""
        positions = np.array(positions)
        positions_homo = np.hstack([positions, np.ones((positions.shape[0], 1))])
        transformed = np.dot(T, positions_homo.T).T
        result = transformed[:, :3]
        # 确保Y坐标保持为0
        result[:, 1] = 0
        return result

    def calculate_directional_rmse(self, actual, predicted):
        """Calculate RMSE for each direction separately"""
        actual = np.array(actual)
        predicted = np.array(predicted)
        
        # 只考虑XZ平面的误差，因为Y轴被约束为0
        rmse_lateral = np.sqrt(np.mean((actual[:, 0] - predicted[:, 0])**2))
        rmse_vertical = 0.0  # Y轴被约束，误差为0
        rmse_longitudinal = np.sqrt(np.mean((actual[:, 2] - predicted[:, 2])**2))
        
        overall_rmse = np.sqrt(rmse_lateral**2 + rmse_longitudinal**2)
        horizontal_rmse = overall_rmse  # 等同于overall_rmse，因为Y=0
        
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
        
        # 确保所有Y坐标都为0（用于显示）
        positions_A[:, 1] = 0
        positions_B[:, 1] = 0
        transformed_positions[:, 1] = 0
        
        # 计算个体误差（只考虑XZ平面）
        individual_errors = np.sqrt((positions_B[:, 0] - transformed_positions[:, 0])**2 + 
                                   (positions_B[:, 2] - transformed_positions[:, 2])**2)
        
        # 3D可视化
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
        
        # 设置Z轴范围以便更好地观察
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
        
        # XZ平面视图（最重要的导航视图）
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
        
        # 如果有残差向量，绘制残差向量场
        if residual_vectors is not None:
            residual_vectors = np.array(residual_vectors)
            scale_factor = 20  # 放大向量以便可视化
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
        
        # 保存阶段特定的文件名
        filename = f'{stage_name.lower().replace(" ", "_").replace("(", "").replace(")", "")}_transformation_results.png'
        plt.savefig(os.path.join(img_dir, filename), dpi=300, bbox_inches='tight')
        plt.close()
        
        # 创建详细的性能摘要图
        self._create_performance_summary(stage_name, rmse_results)

    def _create_performance_summary(self, stage_name, rmse_results):
        """Create detailed performance summary visualization"""
        fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(16, 6))
        
        # Performance vs targets
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
        
        # Color bars based on performance
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
        
        # Real-world equivalent
        real_world_eq = [val * SCALE_FACTOR / 10 for val in actual_values]  # Convert to cm
        
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

    def enhanced_registration(self):
        """Enhanced registration with spatial data splitting and comprehensive evaluation"""
        if self.__positions_A is None or self.__positions_B is None:
            logger.error("Please provide position data first!")
            return None, None
        
        logger.info("=== Starting Enhanced Registration with Spatial Data Splitting ===")
        
        original_positions_A = np.array(self.__positions_A)
        original_positions_B = np.array(self.__positions_B)
        
        # Step 1: Spatial data splitting
        logger.info("Step 1: Spatial Data Splitting")
        split_data = self.data_splitter.split_data_spatially(
            original_positions_A, original_positions_B, self.__original_df
        )
        
        train_A = split_data['train_A']
        train_B = split_data['train_B']
        test_A = split_data['test_A']
        test_B = split_data['test_B']
        
        # 保存为类属性
        self.__train_positions_A = train_A
        self.__train_positions_B = train_B
        self.__test_positions_A = test_A
        self.__test_positions_B = test_B
        
        logger.info(f"Training set: {len(train_A)} points")
        logger.info(f"Test set: {len(test_A)} points")
        
        # Step 2: RANSAC filtering on training data
        logger.info("Step 2: RANSAC Filtering on Training Data")
        self.ransac_filter = RANSACFilter(
            min_samples=RANSAC_MIN_SAMPLES,
            max_iterations=RANSAC_ITERATIONS,
            inlier_threshold=RANSAC_INLIER_THRESHOLD
        )
        
        train_inlier_A, train_inlier_B, train_outlier_A, train_outlier_B = self.ransac_filter.ransac_filter(
            train_A, train_B
        )
        
        if train_inlier_A is None or len(train_inlier_A) == 0:
            logger.error("RANSAC filtering failed - no inliers found")
            return None, None
        
        # 保存训练集内点
        self.__train_inlier_A = train_inlier_A
        self.__train_inlier_B = train_inlier_B
        
        # 可视化RANSAC结果
        self.ransac_filter.visualize_ransac_results(
            train_A, train_B, 
            train_inlier_A, train_inlier_B, train_outlier_A, train_outlier_B,
            "Training Set"
        )
        
        # Step 3: SVD transformation on training data
        logger.info("Step 3: SVD Transformation on Training Data")
        self.calculate_svd_transformation(train_inlier_A, train_inlier_B)
        
        # Apply SVD to training inliers
        train_svd_transformed = self.apply_transformation(train_inlier_A, self.T_svd)
        train_svd_rmse = self.calculate_directional_rmse(train_inlier_B, train_svd_transformed)
        
        logger.info(f"Training SVD RMSE: {train_svd_rmse['overall_rmse']*1000:.3f}mm")
        
        # 计算训练集残差向量
        train_residual_vectors = train_inlier_B - train_svd_transformed
        
        # Step 4: Learn deformation models
        logger.info("Step 4: Learning Deformation Models")
        
        # 4a: Learn on all training data (post-SVD)
        logger.info("  4a: Learning deformation on ALL training data")
        self.deformation_model_all = NonRigidDeformationModel(
            method='thin_plate_spline',
            smoothing=0.001,
            constrain_y=True
        )
        
        # Apply SVD to all training data for deformation learning
        train_all_svd_transformed = self.apply_transformation(train_A, self.T_svd)
        train_all_residual_vectors = train_B - train_all_svd_transformed
        
        success_all = self.deformation_model_all.learn_deformation(
            train_all_svd_transformed, train_all_residual_vectors
        )
        
        # 4b: Learn on training inliers only
        logger.info("  4b: Learning deformation on INLIER training data only")
        self.deformation_model_inliers = NonRigidDeformationModel(
            method='thin_plate_spline',
            smoothing=0.001,
            constrain_y=True
        )
        
        success_inliers = self.deformation_model_inliers.learn_deformation(
            train_svd_transformed, train_residual_vectors
        )
        
        if not success_all and not success_inliers:
            logger.error("Failed to learn any deformation models")
            return None, None
        
        # Step 5: Evaluate on test set
        logger.info("Step 5: Evaluation on Test Set")
        
        # Apply SVD to test data
        test_svd_transformed = self.apply_transformation(test_A, self.T_svd)
        test_svd_rmse = self.calculate_directional_rmse(test_B, test_svd_transformed)
        
        logger.info(f"Test SVD RMSE: {test_svd_rmse['overall_rmse']*1000:.3f}mm")
        
        # Evaluate deformation model learned on all training data
        if success_all:
            test_deformation_all = self.deformation_model_all.predict_deformation(test_svd_transformed)
            test_final_all = test_svd_transformed + test_deformation_all
            test_final_all[:, 1] = 0  # Ensure Y=0
            test_final_rmse_all = self.calculate_directional_rmse(test_B, test_final_all)
            
            logger.info(f"Test Final RMSE (All Data Model): {test_final_rmse_all['overall_rmse']*1000:.3f}mm")
        else:
            test_final_rmse_all = None
            test_final_all = None
        
        # Evaluate deformation model learned on inliers only
        if success_inliers:
            test_deformation_inliers = self.deformation_model_inliers.predict_deformation(test_svd_transformed)
            test_final_inliers = test_svd_transformed + test_deformation_inliers
            test_final_inliers[:, 1] = 0  # Ensure Y=0
            test_final_rmse_inliers = self.calculate_directional_rmse(test_B, test_final_inliers)
            
            logger.info(f"Test Final RMSE (Inliers Model): {test_final_rmse_inliers['overall_rmse']*1000:.3f}mm")
        else:
            test_final_rmse_inliers = None
            test_final_inliers = None
        
        # Step 6: Visualizations and comparisons
        logger.info("Step 6: Creating Visualizations and Comparisons")
        
        # Visualize SVD results on test set
        self.visualize_transformation_stage("Test SVD", test_A, test_B, test_svd_transformed, test_svd_rmse)
        
        # Visualize final results
        if test_final_rmse_all is not None:
            test_residual_all = test_B - test_final_all
            self.visualize_transformation_stage("Test Final (All Data Model)", test_A, test_B, test_final_all, 
                                               test_final_rmse_all, test_residual_all)
        
        if test_final_rmse_inliers is not None:
            test_residual_inliers = test_B - test_final_inliers
            self.visualize_transformation_stage("Test Final (Inliers Model)", test_A, test_B, test_final_inliers, 
                                               test_final_rmse_inliers, test_residual_inliers)
        
        # Create comparison visualization
        self._create_comparison_visualization(test_A, test_B, test_svd_transformed, test_final_all, test_final_inliers,
                                            test_svd_rmse, test_final_rmse_all, test_final_rmse_inliers)
        
        # Visualize deformation fields
        if len(test_svd_transformed) > 0:
            x_min, x_max = np.min(test_svd_transformed[:, 0]), np.max(test_svd_transformed[:, 0])
            z_min, z_max = np.min(test_svd_transformed[:, 2]), np.max(test_svd_transformed[:, 2])
            
            x_range = x_max - x_min
            z_range = z_max - z_min
            bounds = [
                x_min - 0.1 * x_range, x_max + 0.1 * x_range,
                z_min - 0.1 * z_range, z_max + 0.1 * z_range
            ]
            
            if success_all:
                self.deformation_model_all.visualize_deformation_field(bounds, resolution=30, stage_name="All Data Model")
            
            if success_inliers:
                self.deformation_model_inliers.visualize_deformation_field(bounds, resolution=30, stage_name="Inliers Model")
        
        # Save comprehensive results
        self._save_comprehensive_results(split_data, test_svd_rmse, test_final_rmse_all, test_final_rmse_inliers)
        
        # Print detailed analysis
        self._print_comprehensive_analysis(len(original_positions_A), len(train_A), len(train_inlier_A), len(test_A),
                                         test_svd_rmse, test_final_rmse_all, test_final_rmse_inliers)
        
        return self.T_svd, (test_final_rmse_all, test_final_rmse_inliers)

    def _create_comparison_visualization(self, test_A, test_B, svd_result, final_all, final_inliers, 
                                       svd_rmse, rmse_all, rmse_inliers):
        """Create comprehensive comparison visualization"""
        logger.info("Creating comparison visualization...")
        
        fig = plt.figure(figsize=(24, 16))
        
        # XZ plane comparisons
        ax1 = fig.add_subplot(231)
        ax1.scatter(test_A[:, 0], test_A[:, 2], c='blue', s=30, alpha=0.7, label='Source')
        ax1.scatter(test_B[:, 0], test_B[:, 2], c='red', s=30, alpha=0.7, label='Target')
        ax1.scatter(svd_result[:, 0], svd_result[:, 2], c='green', s=30, alpha=0.7, label='SVD Result')
        ax1.set_title(f'SVD Only\nRMSE: {svd_rmse["overall_rmse"]*1000:.2f}mm')
        ax1.set_xlabel('X [m]')
        ax1.set_ylabel('Z [m]')
        ax1.legend()
        ax1.grid(True, alpha=0.3)
        ax1.axis('equal')
        
        if final_all is not None and rmse_all is not None:
            ax2 = fig.add_subplot(232)
            ax2.scatter(test_A[:, 0], test_A[:, 2], c='blue', s=30, alpha=0.7, label='Source')
            ax2.scatter(test_B[:, 0], test_B[:, 2], c='red', s=30, alpha=0.7, label='Target')
            ax2.scatter(final_all[:, 0], final_all[:, 2], c='purple', s=30, alpha=0.7, label='Final (All Data)')
            ax2.set_title(f'SVD + Deformation (All Data)\nRMSE: {rmse_all["overall_rmse"]*1000:.2f}mm')
            ax2.set_xlabel('X [m]')
            ax2.set_ylabel('Z [m]')
            ax2.legend()
            ax2.grid(True, alpha=0.3)
            ax2.axis('equal')
        
        if final_inliers is not None and rmse_inliers is not None:
            ax3 = fig.add_subplot(233)
            ax3.scatter(test_A[:, 0], test_A[:, 2], c='blue', s=30, alpha=0.7, label='Source')
            ax3.scatter(test_B[:, 0], test_B[:, 2], c='red', s=30, alpha=0.7, label='Target')
            ax3.scatter(final_inliers[:, 0], final_inliers[:, 2], c='orange', s=30, alpha=0.7, label='Final (Inliers)')
            ax3.set_title(f'SVD + Deformation (Inliers)\nRMSE: {rmse_inliers["overall_rmse"]*1000:.2f}mm')
            ax3.set_xlabel('X [m]')
            ax3.set_ylabel('Z [m]')
            ax3.legend()
            ax3.grid(True, alpha=0.3)
            ax3.axis('equal')
        
        # RMSE comparison bar chart
        ax4 = fig.add_subplot(234)
        methods = ['SVD Only']
        rmse_values = [svd_rmse['overall_rmse'] * 1000]
        colors = ['green']
        
        if rmse_all is not None:
            methods.append('SVD + Deform (All)')
            rmse_values.append(rmse_all['overall_rmse'] * 1000)
            colors.append('purple')
        
        if rmse_inliers is not None:
            methods.append('SVD + Deform (Inliers)')
            rmse_values.append(rmse_inliers['overall_rmse'] * 1000)
            colors.append('orange')
        
        bars = ax4.bar(methods, rmse_values, color=colors, alpha=0.7)
        ax4.axhline(y=TARGET_RMSE*1000, color='red', linestyle='--', label=f'Target: {TARGET_RMSE*1000:.1f}mm')
        ax4.set_ylabel('RMSE [mm]')
        ax4.set_title('Method Comparison on Test Set')
        ax4.legend()
        ax4.grid(True, alpha=0.3)
        
        # Error distribution comparison
        ax5 = fig.add_subplot(235)
        svd_errors = np.sqrt((test_B[:, 0] - svd_result[:, 0])**2 + (test_B[:, 2] - svd_result[:, 2])**2) * 1000
        ax5.hist(svd_errors, bins=20, alpha=0.7, label='SVD Only', color='green')
        
        if final_all is not None:
            all_errors = np.sqrt((test_B[:, 0] - final_all[:, 0])**2 + (test_B[:, 2] - final_all[:, 2])**2) * 1000
            ax5.hist(all_errors, bins=20, alpha=0.7, label='SVD + Deform (All)', color='purple')
        
        if final_inliers is not None:
            inlier_errors = np.sqrt((test_B[:, 0] - final_inliers[:, 0])**2 + (test_B[:, 2] - final_inliers[:, 2])**2) * 1000
            ax5.hist(inlier_errors, bins=20, alpha=0.7, label='SVD + Deform (Inliers)', color='orange')
        
        ax5.axvline(x=TARGET_RMSE*1000, color='red', linestyle='--', label=f'Target: {TARGET_RMSE*1000:.1f}mm')
        ax5.set_xlabel('Error [mm]')
        ax5.set_ylabel('Frequency')
        ax5.set_title('Error Distribution Comparison')
        ax5.legend()
        ax5.grid(True, alpha=0.3)
        
        # Improvement analysis
        ax6 = fig.add_subplot(236)
        if rmse_all is not None and rmse_inliers is not None:
            improvement_all = (svd_rmse['overall_rmse'] - rmse_all['overall_rmse']) * 1000
            improvement_inliers = (svd_rmse['overall_rmse'] - rmse_inliers['overall_rmse']) * 1000
            
            improvements = [improvement_all, improvement_inliers]
            labels = ['All Data Model', 'Inliers Model']
            colors = ['purple', 'orange']
            
            bars = ax6.bar(labels, improvements, color=colors, alpha=0.7)
            ax6.set_ylabel('RMSE Improvement [mm]')
            ax6.set_title('Deformation Model Improvement')
            ax6.grid(True, alpha=0.3)
            
            # Add value labels on bars
            for bar, improvement in zip(bars, improvements):
                height = bar.get_height()
                ax6.text(bar.get_x() + bar.get_width()/2., height,
                        f'{improvement:.2f}mm',
                        ha='center', va='bottom' if height >= 0 else 'top')
        
        plt.tight_layout()
        plt.savefig(os.path.join(img_dir, 'comprehensive_comparison.png'), dpi=300, bbox_inches='tight')
        plt.close()
        
        logger.info("Comparison visualization saved")

    def _save_comprehensive_results(self, split_data, test_svd_rmse, test_final_rmse_all, test_final_rmse_inliers):
        """Save comprehensive results"""
        # Save test results
        test_data = np.hstack([self.__test_positions_A, self.__test_positions_B])
        np.savetxt(os.path.join(result_dir, "test_evaluation_data.txt"), test_data, 
                  header="test_source_x test_source_y test_source_z test_target_x test_target_y test_target_z")
        
        # Save transformation matrix
        if self.T_svd is not None:
            np.savetxt(os.path.join(result_dir, "T_svd_enhanced.txt"), self.T_svd)
        
        # Save comprehensive summary
        with open(os.path.join(result_dir, "comprehensive_summary.txt"), "w") as f:
            f.write("Enhanced Registration Summary\n")
            f.write("=" * 50 + "\n")
            f.write(f"Original dataset size: {len(self.__positions_A)}\n")
            f.write(f"Training set size: {len(self.__train_positions_A)}\n")
            f.write(f"Training inliers: {len(self.__train_inlier_A) if self.__train_inlier_A is not None else 'N/A'}\n")
            f.write(f"Test set size: {len(self.__test_positions_A)}\n")
            f.write(f"Spatial voxel size: {self.data_splitter.optimal_voxel_size:.6f}m\n")
            f.write("\n")
            
            f.write("Test Set Results:\n")
            f.write(f"SVD RMSE: {test_svd_rmse['overall_rmse']*1000:.3f}mm\n")
            
            if test_final_rmse_all is not None:
                f.write(f"Final RMSE (All Data Model): {test_final_rmse_all['overall_rmse']*1000:.3f}mm\n")
                improvement_all = (test_svd_rmse['overall_rmse'] - test_final_rmse_all['overall_rmse']) * 1000
                f.write(f"Improvement (All Data): {improvement_all:.3f}mm\n")
            
            if test_final_rmse_inliers is not None:
                f.write(f"Final RMSE (Inliers Model): {test_final_rmse_inliers['overall_rmse']*1000:.3f}mm\n")
                improvement_inliers = (test_svd_rmse['overall_rmse'] - test_final_rmse_inliers['overall_rmse']) * 1000
                f.write(f"Improvement (Inliers): {improvement_inliers:.3f}mm\n")
            
            f.write(f"\nTarget RMSE: {TARGET_RMSE*1000:.2f}mm\n")
            
            success_svd = test_svd_rmse['overall_rmse'] <= TARGET_RMSE
            f.write(f"SVD meets target: {'Yes' if success_svd else 'No'}\n")
            
            if test_final_rmse_all is not None:
                success_all = test_final_rmse_all['overall_rmse'] <= TARGET_RMSE
                f.write(f"All Data Model meets target: {'Yes' if success_all else 'No'}\n")
            
            if test_final_rmse_inliers is not None:
                success_inliers = test_final_rmse_inliers['overall_rmse'] <= TARGET_RMSE
                f.write(f"Inliers Model meets target: {'Yes' if success_inliers else 'No'}\n")
        
        logger.info("Comprehensive results saved")

    def _print_comprehensive_analysis(self, total_points, train_points, train_inlier_points, test_points,
                                    test_svd_rmse, test_final_rmse_all, test_final_rmse_inliers):
        """Print comprehensive analysis"""
        logger.info("\n=== Comprehensive Performance Analysis ===")
        
        logger.info("Data Distribution:")
        logger.info(f"  Total original points: {total_points}")
        logger.info(f"  Training points: {train_points} ({train_points/total_points*100:.1f}%)")
        logger.info(f"  Training inliers: {train_inlier_points} ({train_inlier_points/train_points*100:.1f}% of training)")
        logger.info(f"  Test points: {test_points} ({test_points/total_points*100:.1f}%)")
        
        logger.info("Test Set Performance:")
        logger.info(f"  SVD RMSE: {test_svd_rmse['overall_rmse']*1000:.3f}mm")
        
        if test_final_rmse_all is not None:
            improvement_all = (test_svd_rmse['overall_rmse'] - test_final_rmse_all['overall_rmse']) * 1000
            improvement_pct_all = improvement_all / (test_svd_rmse['overall_rmse'] * 1000) * 100
            logger.info(f"  Final RMSE (All Data Model): {test_final_rmse_all['overall_rmse']*1000:.3f}mm")
            logger.info(f"  Improvement (All Data): {improvement_all:.3f}mm ({improvement_pct_all:.1f}%)")
        
        if test_final_rmse_inliers is not None:
            improvement_inliers = (test_svd_rmse['overall_rmse'] - test_final_rmse_inliers['overall_rmse']) * 1000
            improvement_pct_inliers = improvement_inliers / (test_svd_rmse['overall_rmse'] * 1000) * 100
            logger.info(f"  Final RMSE (Inliers Model): {test_final_rmse_inliers['overall_rmse']*1000:.3f}mm")
            logger.info(f"  Improvement (Inliers): {improvement_inliers:.3f}mm ({improvement_pct_inliers:.1f}%)")
        
        logger.info("Target Achievement:")
        logger.info(f"  Target RMSE: {TARGET_RMSE*1000:.2f}mm")
        
        success_svd = test_svd_rmse['overall_rmse'] <= TARGET_RMSE
        logger.info(f"  SVD meets target: {'✓' if success_svd else '✗'}")
        
        if test_final_rmse_all is not None:
            success_all = test_final_rmse_all['overall_rmse'] <= TARGET_RMSE
            logger.info(f"  All Data Model meets target: {'✓' if success_all else '✗'}")
        
        if test_final_rmse_inliers is not None:
            success_inliers = test_final_rmse_inliers['overall_rmse'] <= TARGET_RMSE
            logger.info(f"  Inliers Model meets target: {'✓' if success_inliers else '✗'}")
        
        # Model comparison
        if test_final_rmse_all is not None and test_final_rmse_inliers is not None:
            if test_final_rmse_all['overall_rmse'] < test_final_rmse_inliers['overall_rmse']:
                logger.info("  Best model: All Data Model")
            elif test_final_rmse_inliers['overall_rmse'] < test_final_rmse_all['overall_rmse']:
                logger.info("  Best model: Inliers Model")
            else:
                logger.info("  Both models perform equally")

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
    logger.info("Starting Enhanced Tracker Data Processing")
    logger.info("Features: Spatial data splitting + RANSAC + SVD + Dual deformation learning + Test evaluation")
    logger.info(f"Scale factor: {SCALE_FACTOR}")
    logger.info(f"Target RMSE: {TARGET_RMSE*1000:.2f}mm")
    logger.info(f"Train/Test ratio: {TRAIN_TEST_RATIO:.1f}/{1-TRAIN_TEST_RATIO:.1f}")
    
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
    
    # Perform enhanced registration
    transformer = CoordinateTransformer(positions_A, positions_B, df)
    T_svd, final_rmse_results = transformer.enhanced_registration()
    
    if T_svd is not None and final_rmse_results is not None:
        rmse_all, rmse_inliers = final_rmse_results
        
        # Determine best performance
        best_rmse = None
        best_model = "SVD Only"
        
        if rmse_all is not None:
            best_rmse = rmse_all['overall_rmse']
            best_model = "All Data Model"
        
        if rmse_inliers is not None:
            if best_rmse is None or rmse_inliers['overall_rmse'] < best_rmse:
                best_rmse = rmse_inliers['overall_rmse']
                best_model = "Inliers Model"
        
        success = best_rmse is not None and best_rmse <= TARGET_RMSE
        
        if success:
            logger.info(f"SUCCESS: Registration completed with best RMSE {best_rmse*1000:.3f}mm using {best_model}")
        else:
            logger.info(f"PARTIAL: Registration completed, but best RMSE {best_rmse*1000:.3f}mm exceeds target {TARGET_RMSE*1000:.2f}mm")
        
        # Save final summary
        summary_file = os.path.join(output_dir, "enhanced_registration_summary.txt")
        with open(summary_file, "w") as f:
            f.write("Enhanced Registration Summary\n")
            f.write("=" * 80 + "\n")
            f.write("Features: Spatial data splitting + RANSAC + SVD + Dual deformation learning + Test evaluation\n")
            f.write(f"Scale Factor: {SCALE_FACTOR}\n")
            f.write(f"Target RMSE: {TARGET_RMSE*1000:.2f}mm\n")
            f.write(f"Train/Test ratio: {TRAIN_TEST_RATIO:.1f}/{1-TRAIN_TEST_RATIO:.1f}\n")
            f.write(f"Best RMSE: {best_rmse*1000:.3f}mm\n")
            f.write(f"Best Model: {best_model}\n")
            f.write(f"Success: {'Yes' if success else 'No'}\n")
            f.write(f"Total points: {len(positions_A)}\n")
            f.write(f"Real-world equivalent: {best_rmse*SCALE_FACTOR*100:.2f}cm\n")
            f.write(f"\nMethod: Spatial voxel-based data splitting + RANSAC outlier removal + SVD registration + Dual RBF deformation learning\n")
            f.write("Evaluation: All operations performed on training set, final evaluation on independent test set\n")
            f.write("Deformation models: (1) Learned on all training data, (2) Learned on RANSAC inliers only\n")
    else:
        logger.error("Enhanced registration failed")
    
    logger.info("Enhanced processing complete")

if __name__ == "__main__":
    main()