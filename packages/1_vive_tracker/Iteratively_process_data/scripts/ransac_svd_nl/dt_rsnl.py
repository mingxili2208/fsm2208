#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Tracker data processing script with Enhanced RANSAC + SVD + Non-rigid Deformation Learning (Y-axis constrained)
"""
import sys
import os
import datetime
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from sklearn.neighbors import NearestNeighbors
from sklearn.cluster import KMeans
from sklearn.model_selection import train_test_split
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

# Enhanced RANSAC 参数
RANSAC_MIN_SAMPLES = 32  # 最少点数
RANSAC_ITERATIONS = 2000  # 迭代次数
RANSAC_INLIER_THRESHOLD = 0.015  # 内点阈值 15mm
RANSAC_DELETE_THRESHOLD = 0.050  # 删除判断阈值 50mm (大于此值的点会被从数据集中删除)

script_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(script_dir)
result_dir = os.path.join(parent_dir, "results")
parent_dir = os.path.dirname(parent_dir)
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

# File paths
ROW_X="X"
ROW_Y="Z"

class EnhancedRANSACFilter:
    """
    Enhanced RANSAC-based data filtering with deletion threshold and point classification
    """
    def __init__(self, min_samples=12, max_iterations=100, inlier_threshold=0.015, delete_threshold=0.050):
        self.min_samples = min_samples
        self.max_iterations = max_iterations
        self.inlier_threshold = inlier_threshold
        self.delete_threshold = delete_threshold
        self.best_inliers = None
        self.best_transformation = None
        self.iteration_stats = []
        self.deleted_points_mask = None
        self.inlier_points_mask = None
        self.remaining_points_mask = None
        
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
    
    def count_inliers_and_classify(self, positions_A, positions_B, R, t):
        """计算内点数量并分类点"""
        # 应用变换
        transformed_A = self.apply_2d_transformation(positions_A, R, t)
        
        # 计算变换后的距离（只考虑XZ平面）
        distances = np.sqrt((transformed_A[:, 0] - positions_B[:, 0])**2 + 
                           (transformed_A[:, 2] - positions_B[:, 2])**2)
        
        # 分类点
        inliers = distances < self.inlier_threshold
        to_delete = distances > self.delete_threshold
        remaining = (~inliers) & (~to_delete)  # 大于内点阈值但小于删除阈值的点
        
        return inliers, remaining, to_delete, distances
    
    def enhanced_ransac_filter(self, positions_A, positions_B):
        """执行增强的RANSAC滤波，包含点分类"""
        logger.info("=== Starting Enhanced RANSAC Data Filtering ===")
        
        positions_A = np.array(positions_A)
        positions_B = np.array(positions_B)
        
        if len(positions_A) < self.min_samples:
            logger.error(f"Only {len(positions_A)} points available, less than minimum required {self.min_samples}")
            return None, None, None, None, None, None
        
        best_inlier_count = 0
        best_inliers = None
        best_remaining = None
        best_to_delete = None
        best_R = None
        best_t = None
        
        logger.info(f"Enhanced RANSAC parameters:")
        logger.info(f"  - Min samples: {self.min_samples}")
        logger.info(f"  - Max iterations: {self.max_iterations}")
        logger.info(f"  - Inlier threshold: {self.inlier_threshold*1000:.1f}mm")
        logger.info(f"  - Delete threshold: {self.delete_threshold*1000:.1f}mm")
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
                
                # 分类点
                inliers, remaining, to_delete, distances = self.count_inliers_and_classify(
                    positions_A, positions_B, R, t)
                
                inlier_count = np.sum(inliers)
                remaining_count = np.sum(remaining)
                delete_count = np.sum(to_delete)
                
                # 记录统计信息
                self.iteration_stats.append({
                    'iteration': iteration,
                    'inlier_count': inlier_count,
                    'remaining_count': remaining_count,
                    'delete_count': delete_count,
                    'inlier_ratio': inlier_count / len(positions_A),
                    'mean_distance': np.mean(distances[inliers]) if inlier_count > 0 else float('inf')
                })
                
                # 更新最佳模型（基于内点数量）
                if inlier_count > best_inlier_count:
                    best_inlier_count = inlier_count
                    best_inliers = inliers.copy()
                    best_remaining = remaining.copy()
                    best_to_delete = to_delete.copy()
                    best_R = R.copy()
                    best_t = t.copy()
                    
                    logger.info(f"  Iteration {iteration}: New best model - Inliers: {inlier_count}, Remaining: {remaining_count}, Delete: {delete_count}")
                
            except Exception as e:
                logger.warning(f"  Iteration {iteration}: Failed to compute transformation: {e}")
                continue
        
        if best_inliers is None:
            logger.error("Enhanced RANSAC failed to find any valid transformation")
            return None, None, None, None, None, None
        
        # 存储分类结果
        self.inlier_points_mask = best_inliers
        self.remaining_points_mask = best_remaining
        self.deleted_points_mask = best_to_delete
        
        # 提取分类后的点
        inlier_A = positions_A[best_inliers]
        inlier_B = positions_B[best_inliers]
        remaining_A = positions_A[best_remaining]
        remaining_B = positions_B[best_remaining]
        deleted_A = positions_A[best_to_delete]
        deleted_B = positions_B[best_to_delete]
        
        logger.info(f"Enhanced RANSAC completed:")
        logger.info(f"  - Inliers: {len(inlier_A)}/{len(positions_A)} ({len(inlier_A)/len(positions_A)*100:.1f}%)")
        logger.info(f"  - Remaining: {len(remaining_A)}/{len(positions_A)} ({len(remaining_A)/len(positions_A)*100:.1f}%)")
        logger.info(f"  - Deleted: {len(deleted_A)}/{len(positions_A)} ({len(deleted_A)/len(positions_A)*100:.1f}%)")
        
        # 存储结果
        self.best_inliers = best_inliers
        self.best_transformation = (best_R, best_t)
        
        return inlier_A, inlier_B, remaining_A, remaining_B, deleted_A, deleted_B
    
    def visualize_enhanced_ransac_results(self, original_A, original_B, inlier_A, inlier_B, 
                                        remaining_A, remaining_B, deleted_A, deleted_B):
        """可视化增强RANSAC结果"""
        logger.info("Creating Enhanced RANSAC visualization...")
        
        # 确保所有数组都是numpy数组
        original_A = np.array(original_A)
        original_B = np.array(original_B)
        
        fig = plt.figure(figsize=(24, 16))
        
        # 3D视图 - 原始数据
        ax1 = fig.add_subplot(241, projection='3d')
        ax1.scatter(original_A[:, 0], original_A[:, 2], original_A[:, 1], 
                   c='blue', marker='o', s=30, label='Source Points', alpha=0.7)
        ax1.scatter(original_B[:, 0], original_B[:, 2], original_B[:, 1], 
                   c='red', marker='^', s=30, label='Target Points', alpha=0.7)
        ax1.set_xlabel('X [m]')
        ax1.set_ylabel('Z [m]')
        ax1.set_zlabel('Y [m]')
        ax1.set_title(f'Original Data (Total: {len(original_A)} points)')
        ax1.legend()
        ax1.set_zlim(-0.1, 0.1)
        
        # 3D视图 - 内点
        ax2 = fig.add_subplot(242, projection='3d')
        if len(inlier_A) > 0:
            ax2.scatter(inlier_A[:, 0], inlier_A[:, 2], inlier_A[:, 1], 
                       c='green', marker='o', s=30, label='Inlier Source', alpha=0.7)
            ax2.scatter(inlier_B[:, 0], inlier_B[:, 2], inlier_B[:, 1], 
                       c='darkgreen', marker='^', s=30, label='Inlier Target', alpha=0.7)
        ax2.set_xlabel('X [m]')
        ax2.set_ylabel('Z [m]')
        ax2.set_zlabel('Y [m]')
        ax2.set_title(f'Inliers ({len(inlier_A)} points)')
        ax2.legend()
        ax2.set_zlim(-0.1, 0.1)
        
        # 3D视图 - 剩余点
        ax3 = fig.add_subplot(243, projection='3d')
        if len(remaining_A) > 0:
            ax3.scatter(remaining_A[:, 0], remaining_A[:, 2], remaining_A[:, 1], 
                       c='orange', marker='o', s=30, label='Remaining Source', alpha=0.7)
            ax3.scatter(remaining_B[:, 0], remaining_B[:, 2], remaining_B[:, 1], 
                       c='darkorange', marker='^', s=30, label='Remaining Target', alpha=0.7)
        ax3.set_xlabel('X [m]')
        ax3.set_ylabel('Z [m]')
        ax3.set_zlabel('Y [m]')
        ax3.set_title(f'Remaining Points ({len(remaining_A)} points)')
        ax3.legend()
        ax3.set_zlim(-0.1, 0.1)
        
        # 3D视图 - 删除点
        ax4 = fig.add_subplot(244, projection='3d')
        if len(deleted_A) > 0:
            ax4.scatter(deleted_A[:, 0], deleted_A[:, 2], deleted_A[:, 1], 
                       c='red', marker='o', s=30, label='Deleted Source', alpha=0.7)
            ax4.scatter(deleted_B[:, 0], deleted_B[:, 2], deleted_B[:, 1], 
                       c='darkred', marker='^', s=30, label='Deleted Target', alpha=0.7)
        ax4.set_xlabel('X [m]')
        ax4.set_ylabel('Z [m]')
        ax4.set_zlabel('Y [m]')
        ax4.set_title(f'Deleted Points ({len(deleted_A)} points)')
        ax4.legend()
        ax4.set_zlim(-0.1, 0.1)
        
        # XZ平面视图 - 原始数据
        ax5 = fig.add_subplot(245)
        ax5.scatter(original_A[:, 0], original_A[:, 2], c='blue', marker='o', s=30, 
                   label='Source Points', alpha=0.7)
        ax5.scatter(original_B[:, 0], original_B[:, 2], c='red', marker='^', s=30, 
                   label='Target Points', alpha=0.7)
        ax5.set_xlabel('X (Lateral) [m]')
        ax5.set_ylabel('Z (Longitudinal) [m]')
        ax5.set_title(f'XZ Plane - Original ({len(original_A)} points)')
        ax5.legend()
        ax5.grid(True, alpha=0.3)
        ax5.axis('equal')
        
        # XZ平面视图 - 内点
        ax6 = fig.add_subplot(246)
        if len(inlier_A) > 0:
            ax6.scatter(inlier_A[:, 0], inlier_A[:, 2], c='green', marker='o', s=30, 
                       label='Inlier Source', alpha=0.7)
            ax6.scatter(inlier_B[:, 0], inlier_B[:, 2], c='darkgreen', marker='^', s=30, 
                       label='Inlier Target', alpha=0.7)
        ax6.set_xlabel('X (Lateral) [m]')
        ax6.set_ylabel('Z (Longitudinal) [m]')
        ax6.set_title(f'XZ Plane - Inliers ({len(inlier_A)} points)')
        ax6.legend()
        ax6.grid(True, alpha=0.3)
        ax6.axis('equal')
        
        # XZ平面视图 - 剩余点
        ax7 = fig.add_subplot(247)
        if len(remaining_A) > 0:
            ax7.scatter(remaining_A[:, 0], remaining_A[:, 2], c='orange', marker='o', s=30, 
                       label='Remaining Source', alpha=0.7)
            ax7.scatter(remaining_B[:, 0], remaining_B[:, 2], c='darkorange', marker='^', s=30, 
                       label='Remaining Target', alpha=0.7)
        ax7.set_xlabel('X (Lateral) [m]')
        ax7.set_ylabel('Z (Longitudinal) [m]')
        ax7.set_title(f'XZ Plane - Remaining ({len(remaining_A)} points)')
        ax7.legend()
        ax7.grid(True, alpha=0.3)
        ax7.axis('equal')
        
        # XZ平面视图 - 删除点
        ax8 = fig.add_subplot(248)
        if len(deleted_A) > 0:
            ax8.scatter(deleted_A[:, 0], deleted_A[:, 2], c='red', marker='o', s=30, 
                       label='Deleted Source', alpha=0.7)
            ax8.scatter(deleted_B[:, 0], deleted_B[:, 2], c='darkred', marker='^', s=30, 
                       label='Deleted Target', alpha=0.7)
        ax8.set_xlabel('X (Lateral) [m]')
        ax8.set_ylabel('Z (Longitudinal) [m]')
        ax8.set_title(f'XZ Plane - Deleted ({len(deleted_A)} points)')
        ax8.legend()
        ax8.grid(True, alpha=0.3)
        ax8.axis('equal')
        
        plt.tight_layout()
        plt.savefig(os.path.join(img_dir, 'enhanced_ransac_filtering_results.png'), dpi=300, bbox_inches='tight')
        plt.close()
        
        # 创建RANSAC迭代统计图
        self.plot_enhanced_ransac_statistics()
        
        logger.info("Enhanced RANSAC visualization saved")
    
    def plot_enhanced_ransac_statistics(self):
        """绘制增强RANSAC迭代统计"""
        if not self.iteration_stats:
            return
        
        iterations = [stat['iteration'] for stat in self.iteration_stats]
        inlier_counts = [stat['inlier_count'] for stat in self.iteration_stats]
        remaining_counts = [stat['remaining_count'] for stat in self.iteration_stats]
        delete_counts = [stat['delete_count'] for stat in self.iteration_stats]
        inlier_ratios = [stat['inlier_ratio'] for stat in self.iteration_stats]
        
        fig, ((ax1, ax2), (ax3, ax4)) = plt.subplots(2, 2, figsize=(16, 12))
        
        # 点分类计数
        ax1.plot(iterations, inlier_counts, 'g-', alpha=0.7, label='Inliers')
        ax1.plot(iterations, remaining_counts, 'orange', alpha=0.7, label='Remaining')
        ax1.plot(iterations, delete_counts, 'r-', alpha=0.7, label='Deleted')
        ax1.scatter(iterations, inlier_counts, c='green', s=20, alpha=0.7)
        ax1.scatter(iterations, remaining_counts, c='orange', s=20, alpha=0.7)
        ax1.scatter(iterations, delete_counts, c='red', s=20, alpha=0.7)
        
        if inlier_counts:
            best_iter = iterations[np.argmax(inlier_counts)]
            best_count = max(inlier_counts)
            ax1.axhline(y=best_count, color='red', linestyle='--', alpha=0.7)
            ax1.scatter([best_iter], [best_count], c='red', s=100, marker='*', label=f'Best: {best_count}')
        
        ax1.set_xlabel('Iteration')
        ax1.set_ylabel('Point Count')
        ax1.set_title('Enhanced RANSAC Point Classification per Iteration')
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
        ax2.set_title('Enhanced RANSAC Inlier Ratio per Iteration')
        ax2.legend()
        ax2.grid(True, alpha=0.3)
        
        # 累积点分类
        ax3.bar(range(len(iterations)), inlier_counts, alpha=0.7, label='Inliers', color='green')
        ax3.bar(range(len(iterations)), remaining_counts, bottom=inlier_counts, alpha=0.7, label='Remaining', color='orange')
        ax3.bar(range(len(iterations)), delete_counts, 
               bottom=np.array(inlier_counts) + np.array(remaining_counts), 
               alpha=0.7, label='Deleted', color='red')
        ax3.set_xlabel('Iteration')
        ax3.set_ylabel('Point Count')
        ax3.set_title('Enhanced RANSAC Stacked Point Classification')
        ax3.legend()
        
        # 阈值线
        ax4.axhline(y=self.inlier_threshold*1000, color='green', linestyle='-', alpha=0.7, 
                   label=f'Inlier Threshold: {self.inlier_threshold*1000:.1f}mm')
        ax4.axhline(y=self.delete_threshold*1000, color='red', linestyle='-', alpha=0.7, 
                   label=f'Delete Threshold: {self.delete_threshold*1000:.1f}mm')
        ax4.fill_between([0, len(iterations)], [0, 0], [self.inlier_threshold*1000, self.inlier_threshold*1000], 
                        alpha=0.3, color='green', label='Inlier Zone')
        ax4.fill_between([0, len(iterations)], [self.inlier_threshold*1000, self.inlier_threshold*1000], 
                        [self.delete_threshold*1000, self.delete_threshold*1000], 
                        alpha=0.3, color='orange', label='Remaining Zone')
        ax4.fill_between([0, len(iterations)], [self.delete_threshold*1000, self.delete_threshold*1000], 
                        [100, 100], alpha=0.3, color='red', label='Delete Zone')
        ax4.set_xlabel('Iteration')
        ax4.set_ylabel('Distance [mm]')
        ax4.set_title('Enhanced RANSAC Threshold Zones')
        ax4.legend()
        ax4.grid(True, alpha=0.3)
        ax4.set_ylim(0, min(100, self.delete_threshold*1000 * 1.5))
        
        plt.tight_layout()
        plt.savefig(os.path.join(img_dir, 'enhanced_ransac_statistics.png'), dpi=300, bbox_inches='tight')
        plt.close()

class KMeansRBFDeformationModel:
    """
    K-means clustered RBF deformation model for non-rigid learning
    """
    def __init__(self, n_clusters=5, method='thin_plate_spline', smoothing=0.001, constrain_y=True):
        self.n_clusters = n_clusters
        self.method = method
        self.smoothing = smoothing
        self.constrain_y = constrain_y
        self.kmeans = None
        self.rbf_models_x = {}
        self.rbf_models_z = {}
        self.cluster_centers = None
        self.source_points = None
        
    def learn_deformation(self, source_points, residual_vectors):
        """
        使用K-means聚类学习局部RBF形变模型
        """
        source_points = np.array(source_points)
        residual_vectors = np.array(residual_vectors)
        
        if len(source_points) < self.n_clusters:
            logger.warning(f"Not enough points ({len(source_points)}) for {self.n_clusters} clusters. Using {len(source_points)} clusters.")
            self.n_clusters = len(source_points)
        
        # 只在XZ平面上学习形变
        if self.constrain_y:
            source_xz = source_points[:, [0, 2]]
            residual_xz = residual_vectors[:, [0, 2]]
            
            try:
                # K-means聚类
                self.kmeans = KMeans(n_clusters=self.n_clusters, random_state=42, n_init=10)
                cluster_labels = self.kmeans.fit_predict(source_xz)
                self.cluster_centers = self.kmeans.cluster_centers_
                
                logger.info(f"K-means clustering completed with {self.n_clusters} clusters")
                
                # 为每个聚类学习RBF模型
                for cluster_id in range(self.n_clusters):
                    cluster_mask = cluster_labels == cluster_id
                    cluster_points = source_xz[cluster_mask]
                    cluster_residuals_x = residual_xz[cluster_mask, 0]
                    cluster_residuals_z = residual_xz[cluster_mask, 1]
                    
                    if len(cluster_points) < 3:  # RBF需要至少3个点
                        logger.warning(f"Cluster {cluster_id} has only {len(cluster_points)} points, skipping RBF learning")
                        continue
                    
                    # 学习X方向的形变
                    try:
                        self.rbf_models_x[cluster_id] = RBFInterpolator(
                            cluster_points, 
                            cluster_residuals_x, 
                            kernel=self.method,
                            smoothing=self.smoothing
                        )
                        
                        # 学习Z方向的形变
                        self.rbf_models_z[cluster_id] = RBFInterpolator(
                            cluster_points, 
                            cluster_residuals_z, 
                            kernel=self.method,
                            smoothing=self.smoothing
                        )
                        
                        logger.info(f"RBF model learned for cluster {cluster_id} with {len(cluster_points)} points")
                        
                    except Exception as e:
                        logger.warning(f"Failed to learn RBF for cluster {cluster_id}: {e}")
                        continue
                
                self.source_points = source_xz
                logger.info(f"K-means RBF deformation model learned with {len(source_points)} control points in {len(self.rbf_models_x)} clusters")
                return True
                
            except Exception as e:
                logger.error(f"Failed to learn K-means RBF deformation model: {e}")
                return False
        else:
            logger.warning("3D deformation learning not implemented with Y-axis constraint")
            return False
    
    def predict_deformation(self, query_points):
        """
        预测给定点的形变
        """
        if self.kmeans is None or len(self.rbf_models_x) == 0:
            logger.error("K-means RBF deformation model not trained yet")
            return np.zeros_like(query_points)
        
        query_points = np.array(query_points)
        
        if self.constrain_y:
            query_xz = query_points[:, [0, 2]]
            
            try:
                # 为查询点分配到最近的聚类
                query_clusters = self.kmeans.predict(query_xz)
                
                # 预测形变
                deformation_x = np.zeros(len(query_points))
                deformation_z = np.zeros(len(query_points))
                
                for cluster_id in range(self.n_clusters):
                    cluster_mask = query_clusters == cluster_id
                    if not np.any(cluster_mask):
                        continue
                    
                    cluster_query_points = query_xz[cluster_mask]
                    
                    if cluster_id in self.rbf_models_x and cluster_id in self.rbf_models_z:
                        try:
                            deformation_x[cluster_mask] = self.rbf_models_x[cluster_id](cluster_query_points)
                            deformation_z[cluster_mask] = self.rbf_models_z[cluster_id](cluster_query_points)
                        except Exception as e:
                            logger.warning(f"Failed to predict deformation for cluster {cluster_id}: {e}")
                            # 如果RBF预测失败，使用零形变
                            deformation_x[cluster_mask] = 0
                            deformation_z[cluster_mask] = 0
                    else:
                        # 如果聚类没有RBF模型，使用零形变
                        deformation_x[cluster_mask] = 0
                        deformation_z[cluster_mask] = 0
                
                # 构造完整的形变向量
                deformation = np.zeros_like(query_points)
                deformation[:, 0] = deformation_x
                deformation[:, 2] = deformation_z
                # Y方向形变保持为0
                
                return deformation
                
            except Exception as e:
                logger.error(f"Failed to predict K-means RBF deformation: {e}")
                return np.zeros_like(query_points)
        else:
            return np.zeros_like(query_points)
    
    def visualize_clustered_deformation_field(self, bounds, resolution=50):
        """
        可视化聚类形变场
        """
        if self.kmeans is None or len(self.rbf_models_x) == 0:
            logger.warning("Cannot visualize: K-means RBF deformation model not trained")
            return
        
        x_min, x_max, z_min, z_max = bounds
        x_grid = np.linspace(x_min, x_max, resolution)
        z_grid = np.linspace(z_min, z_max, resolution)
        X, Z = np.meshgrid(x_grid, z_grid)
        
        # 构造查询点
        query_points_2d = np.column_stack([X.ravel(), Z.ravel()])
        
        try:
            # 预测聚类
            cluster_assignments = self.kmeans.predict(query_points_2d)
            
            # 预测形变
            deformation_x = np.zeros(len(query_points_2d))
            deformation_z = np.zeros(len(query_points_2d))
            
            for cluster_id in range(self.n_clusters):
                cluster_mask = cluster_assignments == cluster_id
                if not np.any(cluster_mask):
                    continue
                
                cluster_points = query_points_2d[cluster_mask]
                
                if cluster_id in self.rbf_models_x and cluster_id in self.rbf_models_z:
                    try:
                        deformation_x[cluster_mask] = self.rbf_models_x[cluster_id](cluster_points)
                        deformation_z[cluster_mask] = self.rbf_models_z[cluster_id](cluster_points)
                    except:
                        pass  # 保持零形变
            
            # 重塑为网格
            Dx = deformation_x.reshape(X.shape)
            Dz = deformation_z.reshape(X.shape)
            Clusters = cluster_assignments.reshape(X.shape)
            
            # 可视化
            fig, ((ax1, ax2), (ax3, ax4)) = plt.subplots(2, 2, figsize=(16, 12))
            
            # 聚类分布
            im0 = ax1.contourf(X, Z, Clusters, levels=self.n_clusters, cmap='tab10', alpha=0.7)
            ax1.scatter(self.cluster_centers[:, 0], self.cluster_centers[:, 1], 
                       c='red', s=100, marker='X', label='Cluster Centers')
            if self.source_points is not None:
                ax1.scatter(self.source_points[:, 0], self.source_points[:, 1], 
                           c='black', s=20, marker='o', alpha=0.8, label='Control Points')
            plt.colorbar(im0, ax=ax1)
            ax1.set_title('K-means Cluster Assignment')
            ax1.set_xlabel('X [m]')
            ax1.set_ylabel('Z [m]')
            ax1.legend()
            
            # X方向形变
            im1 = ax2.contourf(X, Z, Dx*1000, levels=20, cmap='RdBu_r')
            ax2.quiver(X[::5, ::5], Z[::5, ::5], Dx[::5, ::5]*1000, np.zeros_like(Dx[::5, ::5]), 
                      scale=10, alpha=0.7)
            plt.colorbar(im1, ax=ax2)
            ax2.set_title('X-direction Deformation [mm]')
            ax2.set_xlabel('X [m]')
            ax2.set_ylabel('Z [m]')
            
            # Z方向形变
            im2 = ax3.contourf(X, Z, Dz*1000, levels=20, cmap='RdBu_r')
            ax3.quiver(X[::5, ::5], Z[::5, ::5], np.zeros_like(Dz[::5, ::5]), Dz[::5, ::5]*1000, 
                      scale=10, alpha=0.7)
            plt.colorbar(im2, ax=ax3)
            ax3.set_title('Z-direction Deformation [mm]')
            ax3.set_xlabel('X [m]')
            ax3.set_ylabel('Z [m]')
            
            # 形变大小
            magnitude = np.sqrt(Dx**2 + Dz**2) * 1000
            im3 = ax4.contourf(X, Z, magnitude, levels=20, cmap='viridis')
            ax4.quiver(X[::5, ::5], Z[::5, ::5], Dx[::5, ::5]*1000, Dz[::5, ::5]*1000, 
                      scale=10, alpha=0.7)
            plt.colorbar(im3, ax=ax4)
            ax4.set_title('Deformation Magnitude [mm]')
            ax4.set_xlabel('X [m]')
            ax4.set_ylabel('Z [m]')
            
            plt.tight_layout()
            plt.savefig(os.path.join(img_dir, 'kmeans_rbf_deformation_field.png'), dpi=300, bbox_inches='tight')
            plt.close()
            
            logger.info("K-means RBF deformation field visualization saved")
            
        except Exception as e:
            logger.error(f"Failed to visualize K-means RBF deformation field: {e}")

class NonRigidDeformationModel:
    """
    Standard non-rigid deformation model using Radial Basis Function (RBF) interpolation
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
                logger.info(f"Standard RBF deformation model learned with {len(source_points)} control points")
                return True
                
            except Exception as e:
                logger.error(f"Failed to learn standard RBF deformation model: {e}")
                return False
        else:
            logger.warning("3D deformation learning not implemented with Y-axis constraint")
            return False
    
    def predict_deformation(self, query_points):
        """
        预测给定点的形变
        """
        if self.rbf_x is None or self.rbf_z is None:
            logger.error("Standard RBF deformation model not trained yet")
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
                logger.error(f"Failed to predict standard RBF deformation: {e}")
                return np.zeros_like(query_points)
        else:
            return np.zeros_like(query_points)

class EnhancedCoordinateTransformer:
    """
    Enhanced three-stage coordinate transformation with K-means RBF deformation learning
    """
    def __init__(self, positions_A=None, positions_B=None, original_df=None):
        self.T_svd = None
        self.kmeans_deformation_model = None
        self.standard_deformation_model = None
        self.ransac_filter = None
        self.__positions_A = positions_A
        self.__positions_B = positions_B
        self.__original_df = original_df
        self.__inlier_positions_A = None
        self.__inlier_positions_B = None
        self.__remaining_positions_A = None
        self.__remaining_positions_B = None
        self.__deleted_positions_A = None
        self.__deleted_positions_B = None
        
    @staticmethod
    def construct_transformation_matrix(rotation_matrix, translation):
        T = np.eye(4)
        T[:3, :3] = rotation_matrix
        T[:3, 3] = translation
        return T

    def calculate_svd_transformation(self, positions_A=None, positions_B=None):
        """Stage 2: SVD-based coarse registration with Y-axis constraint (使用内点)"""
        if positions_A is not None and positions_B is not None:
            self.__inlier_positions_A = positions_A
            self.__inlier_positions_B = positions_B
        elif self.__inlier_positions_A is None or self.__inlier_positions_B is None:
            logger.error("Please provide inlier position data first!")
            return

        positions_A = np.array(self.__inlier_positions_A)
        positions_B = np.array(self.__inlier_positions_B)

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
        
        logger.info("SVD transformation matrix calculated using inlier points only")
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
        
        rmse_lateral = np.sqrt(np.mean((actual[:, 0] - predicted[:, 0])**2))
        rmse_vertical = 0.0  # Y轴被约束，误差为0
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
        
        # 如果有残差向量，绘制残差向量场
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

    def enhanced_three_stage_registration(self):
        """执行增强的三阶段配准：Enhanced RANSAC + SVD + K-means RBF 形变学习"""
        if self.__positions_A is None or self.__positions_B is None:
            logger.error("Please provide position data first!")
            return None, None
        
        logger.info("=== Starting Enhanced Three-Stage Registration ===")
        
        original_positions_A = np.array(self.__positions_A)
        original_positions_B = np.array(self.__positions_B)
        
        # Stage 1: Enhanced RANSAC data filtering with point classification
        logger.info("Stage 1: Enhanced RANSAC Data Filtering with Point Classification")
        self.ransac_filter = EnhancedRANSACFilter(
            min_samples=RANSAC_MIN_SAMPLES,
            max_iterations=RANSAC_ITERATIONS,
            inlier_threshold=RANSAC_INLIER_THRESHOLD,
            delete_threshold=RANSAC_DELETE_THRESHOLD
        )
        
        inlier_A, inlier_B, remaining_A, remaining_B, deleted_A, deleted_B = self.ransac_filter.enhanced_ransac_filter(
            original_positions_A, original_positions_B
        )
        
        if inlier_A is None or len(inlier_A) == 0:
            logger.error("Enhanced RANSAC filtering failed - no inliers found")
            return None, None
        
        # 存储分类结果
        self.__inlier_positions_A = inlier_A
        self.__inlier_positions_B = inlier_B
        self.__remaining_positions_A = remaining_A
        self.__remaining_positions_B = remaining_B
        self.__deleted_positions_A = deleted_A
        self.__deleted_positions_B = deleted_B
        
        # 可视化Enhanced RANSAC结果
        self.ransac_filter.visualize_enhanced_ransac_results(
            original_positions_A, original_positions_B, 
            inlier_A, inlier_B, remaining_A, remaining_B, deleted_A, deleted_B
        )
        
        # 保存分类数据
        self.save_enhanced_ransac_results()
        
        # Stage 2: SVD coarse registration (使用内点)
        logger.info("Stage 2: SVD Coarse Registration (Using Inliers Only)")
        self.calculate_svd_transformation()
        
        # Apply SVD transformation to inliers
        svd_transformed_inliers = self.apply_transformation(self.__inlier_positions_A, self.T_svd)
        
        # Evaluate SVD results on inliers
        svd_rmse_results = self.calculate_directional_rmse(self.__inlier_positions_B, svd_transformed_inliers)
        
        logger.info(f"SVD Overall RMSE (inliers only): {svd_rmse_results['overall_rmse']*1000:.3f}mm")
        
        # 计算残差向量（仅内点）
        residual_vectors_inliers = self.__inlier_positions_B - svd_transformed_inliers
        logger.info(f"Calculated residual vectors for {len(residual_vectors_inliers)} inlier points")
        
        # Visualize SVD results
        self.visualize_transformation_stage("SVD Coarse Registration (Inliers)", 
                                           self.__inlier_positions_A, self.__inlier_positions_B, 
                                           svd_transformed_inliers, svd_rmse_results, residual_vectors_inliers)
        
        # Stage 3: Train-Test Split and Deformation Learning
        logger.info("Stage 3: Train-Test Split and Deformation Learning")
        
        # 3.1: 将剩余点（内点+剩余点）划分为训练集和测试集
        all_remaining_A = np.vstack([self.__inlier_positions_A, self.__remaining_positions_A]) if len(self.__remaining_positions_A) > 0 else self.__inlier_positions_A
        all_remaining_B = np.vstack([self.__inlier_positions_B, self.__remaining_positions_B]) if len(self.__remaining_positions_B) > 0 else self.__inlier_positions_B
        
        # 为所有剩余点应用SVD变换
        all_remaining_transformed = self.apply_transformation(all_remaining_A, self.T_svd)
        all_remaining_residuals = all_remaining_B - all_remaining_transformed
        
        # 划分训练集和测试集
        if len(all_remaining_A) > 10:  # 确保有足够的点进行划分
            train_A, test_A, train_B, test_B, train_transformed, test_transformed, train_residuals, test_residuals = train_test_split(
                all_remaining_A, all_remaining_B, all_remaining_transformed, all_remaining_residuals,
                test_size=0.3, random_state=42
            )
        else:
            # 如果点不够，使用所有点作为训练集
            train_A, train_B, train_transformed, train_residuals = all_remaining_A, all_remaining_B, all_remaining_transformed, all_remaining_residuals
            test_A, test_B, test_transformed, test_residuals = all_remaining_A, all_remaining_B, all_remaining_transformed, all_remaining_residuals
        
        logger.info(f"Train-Test Split completed:")
        logger.info(f"  - Training set: {len(train_A)} points")
        logger.info(f"  - Test set: {len(test_A)} points")
        
        # 3.2: K-means RBF deformation learning (仅使用内点)
        logger.info("Learning K-means RBF deformation model (using inliers only)...")
        
        # 确定合适的聚类数
        n_clusters = min(5, max(2, len(self.__inlier_positions_A) // 10))
        
        self.kmeans_deformation_model = KMeansRBFDeformationModel(
            n_clusters=n_clusters,
            method='thin_plate_spline',
            smoothing=0.001,
            constrain_y=True
        )
        
        kmeans_success = self.kmeans_deformation_model.learn_deformation(
            svd_transformed_inliers, residual_vectors_inliers
        )
        
        # 3.3: Standard RBF deformation learning (使用所有剩余点)
        logger.info("Learning standard RBF deformation model (using all remaining points)...")
        
        self.standard_deformation_model = NonRigidDeformationModel(
            method='thin_plate_spline',
            smoothing=0.001,
            constrain_y=True
        )
        
        standard_success = self.standard_deformation_model.learn_deformation(
            train_transformed, train_residuals
        )
        
        # 3.4: 应用形变模型并对比结果
        results = {}
        
        if kmeans_success:
            # K-means RBF 预测（在测试集上）
            kmeans_predicted_deformation = self.kmeans_deformation_model.predict_deformation(test_transformed)
            kmeans_final_transformed = test_transformed + kmeans_predicted_deformation
            kmeans_final_transformed[:, 1] = 0
            
            kmeans_rmse_results = self.calculate_directional_rmse(test_B, kmeans_final_transformed)
            results['kmeans_rbf'] = {
                'rmse_results': kmeans_rmse_results,
                'performance': self.evaluate_performance(test_B, kmeans_final_transformed),
                'transformed_points': kmeans_final_transformed,
                'residuals': test_B - kmeans_final_transformed
            }
            
            logger.info(f"K-means RBF Overall RMSE (test set): {kmeans_rmse_results['overall_rmse']*1000:.3f}mm")
            
            # 可视化K-means RBF结果
            self.visualize_transformation_stage("K-means RBF Deformation (Test Set)", 
                                               test_A, test_B, kmeans_final_transformed, 
                                               kmeans_rmse_results, test_B - kmeans_final_transformed)
            
            # 可视化聚类形变场
            if len(all_remaining_transformed) > 0:
                x_min, x_max = np.min(all_remaining_transformed[:, 0]), np.max(all_remaining_transformed[:, 0])
                z_min, z_max = np.min(all_remaining_transformed[:, 2]), np.max(all_remaining_transformed[:, 2])
                x_range = x_max - x_min
                z_range = z_max - z_min
                bounds = [
                    x_min - 0.1 * x_range, x_max + 0.1 * x_range,
                    z_min - 0.1 * z_range, z_max + 0.1 * z_range
                ]
                self.kmeans_deformation_model.visualize_clustered_deformation_field(bounds, resolution=30)
        
        if standard_success:
            # Standard RBF 预测（在测试集上）
            standard_predicted_deformation = self.standard_deformation_model.predict_deformation(test_transformed)
            standard_final_transformed = test_transformed + standard_predicted_deformation
            standard_final_transformed[:, 1] = 0
            
            standard_rmse_results = self.calculate_directional_rmse(test_B, standard_final_transformed)
            results['standard_rbf'] = {
                'rmse_results': standard_rmse_results,
                'performance': self.evaluate_performance(test_B, standard_final_transformed),
                'transformed_points': standard_final_transformed,
                'residuals': test_B - standard_final_transformed
            }
            
            logger.info(f"Standard RBF Overall RMSE (test set): {standard_rmse_results['overall_rmse']*1000:.3f}mm")
            
            # 可视化Standard RBF结果
            self.visualize_transformation_stage("Standard RBF Deformation (Test Set)", 
                                               test_A, test_B, standard_final_transformed, 
                                               standard_rmse_results, test_B - standard_final_transformed)
        
        # 3.5: 对比两种方法的结果
        self.compare_deformation_methods(results, test_A, test_B)
        
        # 打印详细结果
        self._print_enhanced_detailed_results(svd_rmse_results, results, len(original_positions_A), 
                                            len(inlier_A), len(remaining_A), len(deleted_A))
        
        # 保存变换数据
        self._save_enhanced_transformation_data(results)
        
        # 返回最佳结果
        best_result = None
        best_rmse = float('inf')
        
        for method_name, result in results.items():
            if result['rmse_results']['overall_rmse'] < best_rmse:
                best_rmse = result['rmse_results']['overall_rmse']
                best_result = result['rmse_results']
        
        return self.T_svd, best_result

    def compare_deformation_methods(self, results, test_A, test_B):
        """对比不同形变方法的结果"""
        if len(results) < 2:
            logger.warning("Not enough deformation methods to compare")
            return
        
        logger.info("Creating deformation methods comparison...")
        
        fig, axes = plt.subplots(2, 3, figsize=(18, 12))
        
        method_names = list(results.keys())
        colors = ['green', 'blue', 'red', 'purple']
        
        # 3D比较视图
        ax1 = fig.add_subplot(231, projection='3d')
        ax1.scatter(test_A[:, 0], test_A[:, 2], test_A[:, 1], 
                   c='gray', marker='o', s=30, label='Source Points', alpha=0.7)
        ax1.scatter(test_B[:, 0], test_B[:, 2], test_B[:, 1], 
                   c='red', marker='^', s=30, label='Target Points', alpha=0.7)
        
        for i, (method_name, result) in enumerate(results.items()):
            transformed = result['transformed_points']
            ax1.scatter(transformed[:, 0], transformed[:, 2], transformed[:, 1], 
                       c=colors[i], marker='x', s=30, label=f'{method_name.replace("_", " ").title()}', alpha=0.7)
        
        ax1.set_xlabel('X [m]')
        ax1.set_ylabel('Z [m]')
        ax1.set_zlabel('Y [m]')
        ax1.set_title('3D Comparison of Deformation Methods')
        ax1.legend()
        ax1.set_zlim(-0.1, 0.1)
        
        # RMSE比较
        ax2 = fig.add_subplot(232)
        rmse_values = [results[method]['rmse_results']['overall_rmse'] * 1000 for method in method_names]
        bars = ax2.bar([method.replace('_', '\n').title() for method in method_names], rmse_values, 
                      color=['lightgreen', 'lightblue'])
        ax2.axhline(y=TARGET_RMSE*1000, color='red', linestyle='--', alpha=0.7, 
                   label=f'Target: {TARGET_RMSE*1000:.1f}mm')
        
        for bar, rmse in zip(bars, rmse_values):
            if rmse <= TARGET_RMSE*1000:
                bar.set_color('green')
            else:
                bar.set_color('red')
        
        ax2.set_ylabel('Overall RMSE [mm]')
        ax2.set_title('RMSE Comparison')
        ax2.legend()
        ax2.grid(True, alpha=0.3)
        
        # 误差分布比较
        ax3 = fig.add_subplot(233)
        for i, (method_name, result) in enumerate(results.items()):
            residuals = result['residuals']
            individual_errors = np.sqrt(np.sum(residuals**2, axis=1)) * 1000
            ax3.hist(individual_errors, bins=15, alpha=0.6, label=f'{method_name.replace("_", " ").title()}', 
                    color=colors[i])
        
        ax3.axvline(x=TARGET_RMSE*1000, color='red', linestyle='--', alpha=0.7, 
                   label=f'Target: {TARGET_RMSE*1000:.1f}mm')
        ax3.set_xlabel('Individual Point Error [mm]')
        ax3.set_ylabel('Frequency')
        ax3.set_title('Error Distribution Comparison')
        ax3.legend()
        ax3.grid(True, alpha=0.3)
        
        # XZ平面比较
        ax4 = fig.add_subplot(234)
        ax4.scatter(test_A[:, 0], test_A[:, 2], c='gray', marker='o', s=30, 
                   label='Source Points', alpha=0.7)
        ax4.scatter(test_B[:, 0], test_B[:, 2], c='red', marker='^', s=30, 
                   label='Target Points', alpha=0.7)
        
        for i, (method_name, result) in enumerate(results.items()):
            transformed = result['transformed_points']
            ax4.scatter(transformed[:, 0], transformed[:, 2], c=colors[i], marker='x', s=30, 
                       label=f'{method_name.replace("_", " ").title()}', alpha=0.7)
        
        ax4.set_xlabel('X (Lateral) [m]')
        ax4.set_ylabel('Z (Longitudinal) [m]')
        ax4.set_title('XZ Plane Comparison')
        ax4.legend()
        ax4.grid(True, alpha=0.3)
        ax4.axis('equal')
        
        # 方向性RMSE比较
        ax5 = fig.add_subplot(235)
        directions = ['Lateral', 'Longitudinal', 'Overall']
        x = np.arange(len(directions))
        width = 0.35
        
        for i, (method_name, result) in enumerate(results.items()):
            rmse_vals = [
                result['rmse_results']['lateral_rmse'] * 1000,
                result['rmse_results']['longitudinal_rmse'] * 1000,
                result['rmse_results']['overall_rmse'] * 1000
            ]
            ax5.bar(x + i*width, rmse_vals, width, label=f'{method_name.replace("_", " ").title()}', 
                   color=colors[i], alpha=0.7)
        
        targets = [SCALED_LATERAL_TARGET * 1000, SCALED_LONGITUDINAL_TARGET * 1000, TARGET_RMSE * 1000]
        ax5.plot(x, targets, 'r--', marker='o', label='Targets')
        
        ax5.set_xlabel('Direction')
        ax5.set_ylabel('RMSE [mm]')
        ax5.set_title('Directional RMSE Comparison')
        ax5.set_xticks(x + width/2)
        ax5.set_xticklabels(directions)
        ax5.legend()
        ax5.grid(True, alpha=0.3)
        
        # 改进百分比
        ax6 = fig.add_subplot(236)
        if len(results) >= 2:
            method_list = list(results.keys())
            baseline_rmse = results[method_list[0]]['rmse_results']['overall_rmse']
            improvements = []
            method_labels = []
            
            for method_name, result in results.items():
                current_rmse = result['rmse_results']['overall_rmse']
                improvement = (baseline_rmse - current_rmse) / baseline_rmse * 100
                improvements.append(improvement)
                method_labels.append(method_name.replace('_', '\n').title())
            
            bars = ax6.bar(method_labels, improvements, color=['lightgreen', 'lightblue'])
            for bar, imp in zip(bars, improvements):
                if imp > 0:
                    bar.set_color('green')
                else:
                    bar.set_color('red')
            
            ax6.axhline(y=0, color='black', linestyle='-', alpha=0.7)
            ax6.set_ylabel('Improvement [%]')
            ax6.set_title(f'Improvement vs {method_list[0].replace("_", " ").title()}')
            ax6.grid(True, alpha=0.3)
        
        plt.tight_layout()
        plt.savefig(os.path.join(img_dir, 'deformation_methods_comparison.png'), dpi=300, bbox_inches='tight')
        plt.close()
        
        logger.info("Deformation methods comparison visualization saved")

    def save_enhanced_ransac_results(self):
        """保存增强RANSAC结果"""
        # 保存内点
        if len(self.__inlier_positions_A) > 0:
            inlier_data = np.hstack([self.__inlier_positions_A, self.__inlier_positions_B])
            np.savetxt(os.path.join(result_dir, "enhanced_ransac_inliers.txt"), inlier_data, 
                      header="source_x source_y source_z target_x target_y target_z")
        
        # 保存剩余点
        if len(self.__remaining_positions_A) > 0:
            remaining_data = np.hstack([self.__remaining_positions_A, self.__remaining_positions_B])
            np.savetxt(os.path.join(result_dir, "enhanced_ransac_remaining.txt"), remaining_data, 
                      header="source_x source_y source_z target_x target_y target_z")
        
        # 保存删除点
        if len(self.__deleted_positions_A) > 0:
            deleted_data = np.hstack([self.__deleted_positions_A, self.__deleted_positions_B])
            np.savetxt(os.path.join(result_dir, "enhanced_ransac_deleted.txt"), deleted_data, 
                      header="source_x source_y source_z target_x target_y target_z")
        
        # 保存增强RANSAC参数和结果摘要
        with open(os.path.join(result_dir, "enhanced_ransac_summary.txt"), "w") as f:
            f.write("Enhanced RANSAC Filtering Summary\n")
            f.write("=" * 40 + "\n")
            f.write(f"Parameters:\n")
            f.write(f"  - Min samples: {self.ransac_filter.min_samples}\n")
            f.write(f"  - Max iterations: {self.ransac_filter.max_iterations}\n")
            f.write(f"  - Inlier threshold: {self.ransac_filter.inlier_threshold*1000:.1f}mm\n")
            f.write(f"  - Delete threshold: {self.ransac_filter.delete_threshold*1000:.1f}mm\n")
            f.write(f"\nResults:\n")
            f.write(f"  - Original points: {len(self.__positions_A)}\n")
            f.write(f"  - Inliers: {len(self.__inlier_positions_A)} ({len(self.__inlier_positions_A)/len(self.__positions_A)*100:.1f}%)\n")
            f.write(f"  - Remaining: {len(self.__remaining_positions_A)} ({len(self.__remaining_positions_A)/len(self.__positions_A)*100:.1f}%)\n")
            f.write(f"  - Deleted: {len(self.__deleted_positions_A)} ({len(self.__deleted_positions_A)/len(self.__positions_A)*100:.1f}%)\n")

    def _print_enhanced_detailed_results(self, svd_performance, deformation_results, 
                                       original_count, inlier_count, remaining_count, deleted_count):
        """打印增强详细性能分析"""
        logger.info("\n=== Enhanced Detailed Performance Analysis ===")
        
        logger.info("Enhanced RANSAC Filtering Results:")
        logger.info(f"  Original points: {original_count}")
        logger.info(f"  Inliers: {inlier_count} ({inlier_count/original_count*100:.1f}%)")
        logger.info(f"  Remaining: {remaining_count} ({remaining_count/original_count*100:.1f}%)")
        logger.info(f"  Deleted: {deleted_count} ({deleted_count/original_count*100:.1f}%)")
        
        logger.info("SVD Stage Results (Using Inliers Only):")
        logger.info(f"  Lateral RMSE: {svd_performance['lateral_rmse']*1000:.3f}mm")
        logger.info(f"  Longitudinal RMSE: {svd_performance['longitudinal_rmse']*1000:.3f}mm")
        logger.info(f"  Overall RMSE: {svd_performance['overall_rmse']*1000:.3f}mm")
        
        logger.info("Deformation Learning Results:")
        for method_name, result in deformation_results.items():
            rmse = result['rmse_results']
            logger.info(f"  {method_name.replace('_', ' ').title()}:")
            logger.info(f"    Lateral RMSE: {rmse['lateral_rmse']*1000:.3f}mm")
            logger.info(f"    Longitudinal RMSE: {rmse['longitudinal_rmse']*1000:.3f}mm")
            logger.info(f"    Overall RMSE: {rmse['overall_rmse']*1000:.3f}mm")
            
            # 计算相对于SVD的改进
            improvement = (svd_performance['overall_rmse'] - rmse['overall_rmse']) * 1000
            improvement_pct = improvement / (svd_performance['overall_rmse'] * 1000) * 100
            logger.info(f"    Improvement vs SVD: {improvement:.3f}mm ({improvement_pct:.1f}%)")
        
        # 方法比较
        if len(deformation_results) >= 2:
            methods = list(deformation_results.keys())
            rmse1 = deformation_results[methods[0]]['rmse_results']['overall_rmse']
            rmse2 = deformation_results[methods[1]]['rmse_results']['overall_rmse']
            
            if rmse1 < rmse2:
                better_method = methods[0]
                improvement = (rmse2 - rmse1) * 1000
                improvement_pct = improvement / (rmse2 * 1000) * 100
            else:
                better_method = methods[1]
                improvement = (rmse1 - rmse2) * 1000
                improvement_pct = improvement / (rmse1 * 1000) * 100
            
            logger.info(f"Best Method: {better_method.replace('_', ' ').title()}")
            logger.info(f"Advantage: {improvement:.3f}mm ({improvement_pct:.1f}%)")

    def _save_enhanced_transformation_data(self, results):
        """保存增强变换数据"""
        # 保存SVD变换
        if self.T_svd is not None:
            np.savetxt(os.path.join(result_dir, "T_svd_enhanced.txt"), self.T_svd)
        
        # 保存K-means形变模型信息
        if self.kmeans_deformation_model is not None and self.kmeans_deformation_model.source_points is not None:
            np.savetxt(os.path.join(result_dir, "kmeans_deformation_control_points.txt"), 
                      self.kmeans_deformation_model.source_points)
            np.savetxt(os.path.join(result_dir, "kmeans_cluster_centers.txt"), 
                      self.kmeans_deformation_model.cluster_centers)
            
            with open(os.path.join(result_dir, "kmeans_deformation_model_info.txt"), "w") as f:
                f.write(f"K-means RBF Deformation Model Info\n")
                f.write(f"Method: {self.kmeans_deformation_model.method}\n")
                f.write(f"Smoothing: {self.kmeans_deformation_model.smoothing}\n")
                f.write(f"Y-axis constrained: {self.kmeans_deformation_model.constrain_y}\n")
                f.write(f"Number of clusters: {self.kmeans_deformation_model.n_clusters}\n")
                f.write(f"Control points: {len(self.kmeans_deformation_model.source_points)}\n")
                f.write(f"Active RBF models: {len(self.kmeans_deformation_model.rbf_models_x)}\n")
        
        # 保存标准RBF形变模型信息
        if self.standard_deformation_model is not None and self.standard_deformation_model.source_points is not None:
            np.savetxt(os.path.join(result_dir, "standard_deformation_control_points.txt"), 
                      self.standard_deformation_model.source_points)
            
            with open(os.path.join(result_dir, "standard_deformation_model_info.txt"), "w") as f:
                f.write(f"Standard RBF Deformation Model Info\n")
                f.write(f"Method: {self.standard_deformation_model.method}\n")
                f.write(f"Smoothing: {self.standard_deformation_model.smoothing}\n")
                f.write(f"Y-axis constrained: {self.standard_deformation_model.constrain_y}\n")
                f.write(f"Control points: {len(self.standard_deformation_model.source_points)}\n")
        
        # 保存结果摘要
        with open(os.path.join(result_dir, "deformation_results_summary.txt"), "w") as f:
            f.write("Deformation Learning Results Summary\n")
            f.write("=" * 50 + "\n")
            for method_name, result in results.items():
                rmse = result['rmse_results']
                f.write(f"\n{method_name.replace('_', ' ').title()}:\n")
                f.write(f"  Overall RMSE: {rmse['overall_rmse']*1000:.3f}mm\n")
                f.write(f"  Lateral RMSE: {rmse['lateral_rmse']*1000:.3f}mm\n")
                f.write(f"  Longitudinal RMSE: {rmse['longitudinal_rmse']*1000:.3f}mm\n")
                f.write(f"  Target achieved: {'Yes' if rmse['overall_rmse'] <= TARGET_RMSE else 'No'}\n")
        
        logger.info("Enhanced transformation data saved to results directory")

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
    logger.info("Starting Enhanced 1:32 scale model tracker data processing")
    logger.info(f"Scale factor: {SCALE_FACTOR}")
    logger.info(f"Target RMSE: {TARGET_RMSE*1000:.2f}mm")
    logger.info(f"Enhanced RANSAC parameters:")
    logger.info(f"  - Min samples: {RANSAC_MIN_SAMPLES}")
    logger.info(f"  - Iterations: {RANSAC_ITERATIONS}")
    logger.info(f"  - Inlier threshold: {RANSAC_INLIER_THRESHOLD*1000:.1f}mm")
    logger.info(f"  - Delete threshold: {RANSAC_DELETE_THRESHOLD*1000:.1f}mm")
    
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
    
    # Perform enhanced three-stage registration
    transformer = EnhancedCoordinateTransformer(positions_A, positions_B, df)
    T_svd, final_rmse_results = transformer.enhanced_three_stage_registration()
    
    if T_svd is not None and final_rmse_results is not None:
        success = final_rmse_results['overall_rmse'] <= TARGET_RMSE
        if success:
            logger.info(f"SUCCESS: Enhanced registration completed with RMSE {final_rmse_results['overall_rmse']*1000:.3f}mm")
        else:
            logger.info(f"PARTIAL: Enhanced registration completed, but RMSE {final_rmse_results['overall_rmse']*1000:.3f}mm exceeds target {TARGET_RMSE*1000:.2f}mm")
        
        # Save final summary
        summary_file = os.path.join(output_dir, "enhanced_registration_summary.txt")
        with open(summary_file, "w") as f:
            f.write("Enhanced Three-Stage Registration Summary\n")
            f.write("(Enhanced RANSAC + SVD + K-means RBF + Standard RBF)\n")
            f.write("=" * 80 + "\n")
            f.write(f"Scale Factor: {SCALE_FACTOR}\n")
            f.write(f"Target RMSE: {TARGET_RMSE*1000:.2f}mm\n")
            f.write(f"Final RMSE: {final_rmse_results['overall_rmse']*1000:.3f}mm\n")
            f.write(f"Success: {'Yes' if success else 'No'}\n")
            f.write(f"Points processed: {len(positions_A)}\n")
            f.write(f"Points after Enhanced RANSAC: {len(transformer._EnhancedCoordinateTransformer__inlier_positions_A) if hasattr(transformer, '_EnhancedCoordinateTransformer__inlier_positions_A') and transformer._EnhancedCoordinateTransformer__inlier_positions_A is not None else 'N/A'}\n")
            f.write(f"Real-world equivalent: {final_rmse_results['overall_rmse']*SCALE_FACTOR*100:.2f}cm\n")
            f.write(f"\nEnhanced RANSAC Parameters:\n")
            f.write(f"  Min samples: {RANSAC_MIN_SAMPLES}\n")
            f.write(f"  Iterations: {RANSAC_ITERATIONS}\n")
            f.write(f"  Inlier threshold: {RANSAC_INLIER_THRESHOLD*1000:.1f}mm\n")
            f.write(f"  Delete threshold: {RANSAC_DELETE_THRESHOLD*1000:.1f}mm\n")
            f.write("\nMethods:\n")
            f.write("  1. Enhanced RANSAC with point classification\n")
            f.write("  2. SVD coarse registration using inliers only\n")
            f.write("  3. K-means clustered RBF deformation learning\n")
            f.write("  4. Standard RBF deformation learning\n")
            f.write("  5. Train-test split for evaluation\n")
            f.write("  6. Method comparison and analysis\n")
    else:
        logger.error("Enhanced registration failed")
    
    logger.info("Enhanced processing complete")

if __name__ == "__main__":
    main()