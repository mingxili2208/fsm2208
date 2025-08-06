#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Tracker data processing script with Train/Test Split First, then RANSAC + SVD + Non-rigid Deformation Learning (Y-axis constrained)
Train on train set, evaluate on test set
"""
import sys
import os
import datetime
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from sklearn.neighbors import NearestNeighbors
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

# RANSAC 参数
RANSAC_MIN_SAMPLES = 32  # 最少点数
RANSAC_ITERATIONS = 2000  # 迭代次数
RANSAC_INLIER_THRESHOLD = 0.03  # 内点阈值 30mm (0.030m)

# Train/Test split parameters
TRAIN_TEST_SPLIT_RATIO = 0.7  # 70% for training, 30% for testing
RANDOM_STATE = 42  # For reproducible results

# Create output directory
script_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(script_dir)
result_dir = os.path.join(parent_dir, "results")
##########################################################
#corrected_tracker_data_0703_162626
#crossed_corrected_tracker_data_0707_125849
#combined_corrected_tracker_data_2
#########################################################
LASER_TRACKER_CSV = os.path.join(parent_dir, "data/combined_corrected_tracker_data_2.csv")
csv_filename = os.path.basename(LASER_TRACKER_CSV)
output_dir = os.path.join(result_dir, f"SplitFirst_RANSAC_SVD_Deformation_{timestamp}_1to32_scale_RMSE_{TARGET_RMSE*1000:.1f}mm_{csv_filename}")
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

class RANSACFilter:
    """
    RANSAC-based data filtering for point cloud registration (without distance filtering)
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
        """执行RANSAC滤波（不进行距离预过滤）"""
        logger.info("=== Starting RANSAC Data Filtering on Training Set ===")
        
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
        logger.info(f"  - Total training points: {len(positions_A)}")
        
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
        
        logger.info(f"RANSAC completed on training set:")
        logger.info(f"  - Best inlier count: {best_inlier_count}/{len(positions_A)} ({best_inlier_count/len(positions_A)*100:.1f}%)")
        logger.info(f"  - Outliers removed: {len(positions_A) - best_inlier_count}")
        
        # 存储结果
        self.best_inliers = best_inliers
        self.best_transformation = (best_R, best_t)
        
        return inlier_A, inlier_B, outlier_A, outlier_B
    
    def apply_ransac_model_to_test(self, test_positions_A, test_positions_B):
        """Apply the learned RANSAC model to test set"""
        if self.best_transformation is None:
            logger.error("No RANSAC model available. Must train first.")
            return None, None, None, None
        
        logger.info("=== Applying RANSAC Model to Test Set ===")
        
        test_positions_A = np.array(test_positions_A)
        test_positions_B = np.array(test_positions_B)
        
        best_R, best_t = self.best_transformation
        
        # 计算测试集内点
        inliers, distances = self.count_inliers(test_positions_A, test_positions_B, best_R, best_t)
        
        inlier_count = np.sum(inliers)
        
        # 提取内点和外点
        test_inlier_A = test_positions_A[inliers]
        test_inlier_B = test_positions_B[inliers]
        test_outlier_A = test_positions_A[~inliers]
        test_outlier_B = test_positions_B[~inliers]
        
        logger.info(f"RANSAC model applied to test set:")
        logger.info(f"  - Test inliers: {inlier_count}/{len(test_positions_A)} ({inlier_count/len(test_positions_A)*100:.1f}%)")
        logger.info(f"  - Test outliers: {len(test_positions_A) - inlier_count}")
        
        return test_inlier_A, test_inlier_B, test_outlier_A, test_outlier_B
    
    def visualize_ransac_results(self, train_original_A, train_original_B, train_inlier_A, train_inlier_B, 
                                train_outlier_A, train_outlier_B, test_original_A=None, test_original_B=None,
                                test_inlier_A=None, test_inlier_B=None, test_outlier_A=None, test_outlier_B=None):
        """可视化RANSAC结果（包括训练集和测试集）"""
        logger.info("Creating RANSAC visualization...")
        
        # 确保所有数组都是numpy数组
        train_original_A = np.array(train_original_A)
        train_original_B = np.array(train_original_B)
        
        if test_original_A is not None:
            test_original_A = np.array(test_original_A)
            test_original_B = np.array(test_original_B)
            
        fig = plt.figure(figsize=(24, 16))
        
        # 训练集 - 剔除前
        ax1 = fig.add_subplot(331, projection='3d')
        ax1.scatter(train_original_A[:, 0], train_original_A[:, 2], train_original_A[:, 1], 
                   c='blue', marker='o', s=30, label='Train Source', alpha=0.7)
        ax1.scatter(train_original_B[:, 0], train_original_B[:, 2], train_original_B[:, 1], 
                   c='red', marker='^', s=30, label='Train Target', alpha=0.7)
        ax1.set_xlabel('X [m]')
        ax1.set_ylabel('Z [m]')
        ax1.set_zlabel('Y [m]')
        ax1.set_title(f'Train Set Before RANSAC ({len(train_original_A)} points)')
        ax1.legend()
        ax1.set_zlim(-0.1, 0.1)
        
        # 训练集 - 内点
        ax2 = fig.add_subplot(332, projection='3d')
        if len(train_inlier_A) > 0:
            ax2.scatter(train_inlier_A[:, 0], train_inlier_A[:, 2], train_inlier_A[:, 1], 
                       c='green', marker='o', s=30, label='Train Inliers', alpha=0.7)
            ax2.scatter(train_inlier_B[:, 0], train_inlier_B[:, 2], train_inlier_B[:, 1], 
                       c='darkgreen', marker='^', s=30, label='Train Targets', alpha=0.7)
        ax2.set_xlabel('X [m]')
        ax2.set_ylabel('Z [m]')
        ax2.set_zlabel('Y [m]')
        ax2.set_title(f'Train Set Inliers ({len(train_inlier_A)} points)')
        ax2.legend()
        ax2.set_zlim(-0.1, 0.1)
        
        # 训练集 - 外点
        ax3 = fig.add_subplot(333, projection='3d')
        if len(train_outlier_A) > 0:
            ax3.scatter(train_outlier_A[:, 0], train_outlier_A[:, 2], train_outlier_A[:, 1], 
                       c='orange', marker='o', s=30, label='Train Outliers', alpha=0.7)
            ax3.scatter(train_outlier_B[:, 0], train_outlier_B[:, 2], train_outlier_B[:, 1], 
                       c='red', marker='^', s=30, label='Train Targets', alpha=0.7)
        ax3.set_xlabel('X [m]')
        ax3.set_ylabel('Z [m]')
        ax3.set_zlabel('Y [m]')
        ax3.set_title(f'Train Set Outliers ({len(train_outlier_A)} points)')
        ax3.legend()
        ax3.set_zlim(-0.1, 0.1)
        
        # 测试集结果（如果有）
        if test_original_A is not None:
            # 测试集 - 剔除前
            ax4 = fig.add_subplot(334, projection='3d')
            ax4.scatter(test_original_A[:, 0], test_original_A[:, 2], test_original_A[:, 1], 
                       c='lightblue', marker='s', s=30, label='Test Source', alpha=0.7)
            ax4.scatter(test_original_B[:, 0], test_original_B[:, 2], test_original_B[:, 1], 
                       c='pink', marker='d', s=30, label='Test Target', alpha=0.7)
            ax4.set_xlabel('X [m]')
            ax4.set_ylabel('Z [m]')
            ax4.set_zlabel('Y [m]')
            ax4.set_title(f'Test Set Before RANSAC ({len(test_original_A)} points)')
            ax4.legend()
            ax4.set_zlim(-0.1, 0.1)
            
            # 测试集 - 内点
            ax5 = fig.add_subplot(335, projection='3d')
            if test_inlier_A is not None and len(test_inlier_A) > 0:
                ax5.scatter(test_inlier_A[:, 0], test_inlier_A[:, 2], test_inlier_A[:, 1], 
                           c='cyan', marker='s', s=30, label='Test Inliers', alpha=0.7)
                ax5.scatter(test_inlier_B[:, 0], test_inlier_B[:, 2], test_inlier_B[:, 1], 
                           c='blue', marker='d', s=30, label='Test Targets', alpha=0.7)
            ax5.set_xlabel('X [m]')
            ax5.set_ylabel('Z [m]')
            ax5.set_zlabel('Y [m]')
            ax5.set_title(f'Test Set Inliers ({len(test_inlier_A) if test_inlier_A is not None else 0} points)')
            ax5.legend()
            ax5.set_zlim(-0.1, 0.1)
            
            # 测试集 - 外点
            ax6 = fig.add_subplot(336, projection='3d')
            if test_outlier_A is not None and len(test_outlier_A) > 0:
                ax6.scatter(test_outlier_A[:, 0], test_outlier_A[:, 2], test_outlier_A[:, 1], 
                           c='yellow', marker='s', s=30, label='Test Outliers', alpha=0.7)
                ax6.scatter(test_outlier_B[:, 0], test_outlier_B[:, 2], test_outlier_B[:, 1], 
                           c='red', marker='d', s=30, label='Test Targets', alpha=0.7)
            ax6.set_xlabel('X [m]')
            ax6.set_ylabel('Z [m]')
            ax6.set_zlabel('Y [m]')
            ax6.set_title(f'Test Set Outliers ({len(test_outlier_A) if test_outlier_A is not None else 0} points)')
            ax6.legend()
            ax6.set_zlim(-0.1, 0.1)
        
        # XZ平面视图 - 训练集对比
        ax7 = fig.add_subplot(337)
        ax7.scatter(train_original_A[:, 0], train_original_A[:, 2], c='blue', marker='o', s=30, 
                   label='Train Source', alpha=0.5)
        ax7.scatter(train_original_B[:, 0], train_original_B[:, 2], c='red', marker='^', s=30, 
                   label='Train Target', alpha=0.5)
        if len(train_inlier_A) > 0:
            ax7.scatter(train_inlier_A[:, 0], train_inlier_A[:, 2], c='green', marker='o', s=50, 
                       label='Train Inliers', alpha=0.8, edgecolor='black')
        ax7.set_xlabel('X (Lateral) [m]')
        ax7.set_ylabel('Z (Longitudinal) [m]')
        ax7.set_title('Train Set - XZ Plane View')
        ax7.legend()
        ax7.grid(True, alpha=0.3)
        ax7.axis('equal')
        
        # XZ平面视图 - 测试集对比（如果有）
        ax8 = fig.add_subplot(338)
        if test_original_A is not None:
            ax8.scatter(test_original_A[:, 0], test_original_A[:, 2], c='lightblue', marker='s', s=30, 
                       label='Test Source', alpha=0.5)
            ax8.scatter(test_original_B[:, 0], test_original_B[:, 2], c='pink', marker='d', s=30, 
                       label='Test Target', alpha=0.5)
            if test_inlier_A is not None and len(test_inlier_A) > 0:
                ax8.scatter(test_inlier_A[:, 0], test_inlier_A[:, 2], c='cyan', marker='s', s=50, 
                           label='Test Inliers', alpha=0.8, edgecolor='black')
        ax8.set_xlabel('X (Lateral) [m]')
        ax8.set_ylabel('Z (Longitudinal) [m]')
        ax8.set_title('Test Set - XZ Plane View')
        ax8.legend()
        ax8.grid(True, alpha=0.3)
        ax8.axis('equal')
        
        # 统计对比
        ax9 = fig.add_subplot(339)
        categories = ['Original', 'Inliers', 'Outliers']
        train_counts = [len(train_original_A), len(train_inlier_A), len(train_outlier_A)]
        test_counts = [len(test_original_A) if test_original_A is not None else 0,
                      len(test_inlier_A) if test_inlier_A is not None else 0,
                      len(test_outlier_A) if test_outlier_A is not None else 0]
        
        x = np.arange(len(categories))
        width = 0.35
        
        ax9.bar(x - width/2, train_counts, width, label='Train Set', alpha=0.8)
        ax9.bar(x + width/2, test_counts, width, label='Test Set', alpha=0.8)
        
        ax9.set_xlabel('Category')
        ax9.set_ylabel('Point Count')
        ax9.set_title('RANSAC Results Comparison')
        ax9.set_xticks(x)
        ax9.set_xticklabels(categories)
        ax9.legend()
        ax9.grid(True, alpha=0.3)
        
        plt.tight_layout()
        plt.savefig(os.path.join(img_dir, 'ransac_filtering_results_train_test.png'), dpi=300, bbox_inches='tight')
        plt.close()
        
        # 创建RANSAC迭代统计图
        self.plot_ransac_statistics()
        
        logger.info("RANSAC visualization saved")
    
    def plot_ransac_statistics(self):
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
        ax1.set_title('RANSAC Inlier Count per Iteration (Train Set)')
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
        ax2.set_title('RANSAC Inlier Ratio per Iteration (Train Set)')
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
        ax3.set_title('RANSAC Mean Inlier Distance per Iteration (Train Set)')
        ax3.legend()
        ax3.grid(True, alpha=0.3)
        
        plt.tight_layout()
        plt.savefig(os.path.join(img_dir, 'ransac_statistics.png'), dpi=300, bbox_inches='tight')
        plt.close()
    
    def save_ransac_results(self, train_original_A, train_original_B, train_inlier_A, train_inlier_B, 
                           train_outlier_A, train_outlier_B, test_original_A=None, test_original_B=None,
                           test_inlier_A=None, test_inlier_B=None, test_outlier_A=None, test_outlier_B=None,
                           train_original_df=None, test_original_df=None):
        """保存RANSAC结果"""
        # 保存训练集内点
        if len(train_inlier_A) > 0:
            train_inlier_data = np.hstack([train_inlier_A, train_inlier_B])
            np.savetxt(os.path.join(result_dir, "ransac_train_inliers.txt"), train_inlier_data, 
                      header="source_x source_y source_z target_x target_y target_z")
        
        # 保存训练集外点
        if len(train_outlier_A) > 0:
            train_outlier_data = np.hstack([train_outlier_A, train_outlier_B])
            np.savetxt(os.path.join(result_dir, "ransac_train_outliers.txt"), train_outlier_data, 
                      header="source_x source_y source_z target_x target_y target_z")
        
        # 保存测试集结果（如果有）
        if test_inlier_A is not None and len(test_inlier_A) > 0:
            test_inlier_data = np.hstack([test_inlier_A, test_inlier_B])
            np.savetxt(os.path.join(result_dir, "ransac_test_inliers.txt"), test_inlier_data, 
                      header="source_x source_y source_z target_x target_y target_z")
        
        if test_outlier_A is not None and len(test_outlier_A) > 0:
            test_outlier_data = np.hstack([test_outlier_A, test_outlier_B])
            np.savetxt(os.path.join(result_dir, "ransac_test_outliers.txt"), test_outlier_data, 
                      header="source_x source_y source_z target_x target_y target_z")
        
        # 保存RANSAC参数和结果摘要
        with open(os.path.join(result_dir, "ransac_summary.txt"), "w") as f:
            f.write("RANSAC Filtering Summary (Split First Approach)\n")
            f.write("=" * 50 + "\n")
            f.write(f"Parameters:\n")
            f.write(f"  - Min samples: {self.min_samples}\n")
            f.write(f"  - Max iterations: {self.max_iterations}\n")
            f.write(f"  - Inlier threshold: {self.inlier_threshold*1000:.1f}mm\n")
            f.write(f"\nTraining Set Results:\n")
            f.write(f"  - Original points: {len(train_original_A)}\n")
            f.write(f"  - Inliers: {len(train_inlier_A)} ({len(train_inlier_A)/len(train_original_A)*100:.1f}%)\n")
            f.write(f"  - Outliers: {len(train_outlier_A)} ({len(train_outlier_A)/len(train_original_A)*100:.1f}%)\n")
            
            if test_original_A is not None:
                f.write(f"\nTest Set Results (Model Applied):\n")
                f.write(f"  - Original points: {len(test_original_A)}\n")
                f.write(f"  - Inliers: {len(test_inlier_A) if test_inlier_A is not None else 0} ({len(test_inlier_A)/len(test_original_A)*100:.1f}% if test_inlier_A is not None else 0)\n")
                f.write(f"  - Outliers: {len(test_outlier_A) if test_outlier_A is not None else 0} ({len(test_outlier_A)/len(test_original_A)*100:.1f}% if test_outlier_A is not None else 0)\n")
            
            if self.best_transformation:
                f.write(f"  - Best transformation found: Yes\n")
            else:
                f.write(f"  - Best transformation found: No\n")

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
    
    def visualize_deformation_field(self, bounds, resolution=50):
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
            ax1.set_title('X-direction Deformation [mm]')
            ax1.set_xlabel('X [m]')
            ax1.set_ylabel('Z [m]')
            
            # Z方向形变
            im2 = ax2.contourf(X, Z, Dz*1000, levels=20, cmap='RdBu_r')
            ax2.quiver(X[::5, ::5], Z[::5, ::5], np.zeros_like(Dz[::5, ::5]), Dz[::5, ::5]*1000, 
                      scale=10, alpha=0.7)
            plt.colorbar(im2, ax=ax2)
            ax2.set_title('Z-direction Deformation [mm]')
            ax2.set_xlabel('X [m]')
            ax2.set_ylabel('Z [m]')
            
            # 形变大小
            magnitude = np.sqrt(Dx**2 + Dz**2) * 1000
            im3 = ax3.contourf(X, Z, magnitude, levels=20, cmap='viridis')
            ax3.quiver(X[::5, ::5], Z[::5, ::5], Dx[::5, ::5]*1000, Dz[::5, ::5]*1000, 
                      scale=10, alpha=0.7)
            plt.colorbar(im3, ax=ax3)
            ax3.set_title('Deformation Magnitude [mm]')
            ax3.set_xlabel('X [m]')
            ax3.set_ylabel('Z [m]')
            
            # 叠加控制点
            if self.source_points is not None:
                for ax in [ax1, ax2, ax3]:
                    ax.scatter(self.source_points[:, 0], self.source_points[:, 1], 
                             c='black', s=20, marker='o', alpha=0.8, label='Control Points')
                    ax.legend()
            
            plt.tight_layout()
            plt.savefig(os.path.join(img_dir, 'deformation_field_trained_on_train.png'), dpi=300, bbox_inches='tight')
            plt.close()
            
            logger.info("Deformation field visualization saved")
            
        except Exception as e:
            logger.error(f"Failed to visualize deformation field: {e}")

class CoordinateTransformer:
    """
    Three-stage coordinate transformation with Split First approach:
    1. Split data into train/test first
    2. Train on train set: RANSAC filtering + SVD coarse + Non-rigid deformation learning
    3. Evaluate on test set
    """
    def __init__(self, positions_A=None, positions_B=None, original_df=None):
        self.T_svd = None
        self.deformation_model = None
        self.ransac_filter = None
        self.__positions_A = positions_A
        self.__positions_B = positions_B
        self.__original_df = original_df
        
        # Train/Test split data (raw split before any processing)
        self.__train_A_raw = None
        self.__train_B_raw = None
        self.__test_A_raw = None
        self.__test_B_raw = None
        self.__train_indices = None
        self.__test_indices = None
        self.__train_df = None
        self.__test_df = None
        
        # Processed data after RANSAC
        self.__train_inlier_A = None
        self.__train_inlier_B = None
        self.__test_inlier_A = None
        self.__test_inlier_B = None
        
    @staticmethod
    def construct_transformation_matrix(rotation_matrix, translation):
        T = np.eye(4)
        T[:3, :3] = rotation_matrix
        T[:3, 3] = translation
        return T

    def split_data_first(self):
        """
        Split original data into training and testing sets BEFORE any processing
        """
        logger.info("=== Step 1: Splitting Original Data into Train/Test Sets ===")
        
        if self.__positions_A is None or self.__positions_B is None:
            logger.error("No position data available for splitting")
            return False
        
        positions_A = np.array(self.__positions_A)
        positions_B = np.array(self.__positions_B)
        
        # Perform train/test split on raw data
        train_A, test_A, train_B, test_B, train_indices, test_indices = train_test_split(
            positions_A, positions_B, np.arange(len(positions_A)),
            test_size=(1 - TRAIN_TEST_SPLIT_RATIO),
            random_state=RANDOM_STATE,
            shuffle=True
        )
        
        self.__train_A_raw = train_A
        self.__train_B_raw = train_B
        self.__test_A_raw = test_A
        self.__test_B_raw = test_B
        self.__train_indices = train_indices
        self.__test_indices = test_indices
        
        # Split the original dataframe as well
        if self.__original_df is not None:
            self.__train_df = self.__original_df.iloc[train_indices].copy().reset_index(drop=True)
            self.__test_df = self.__original_df.iloc[test_indices].copy().reset_index(drop=True)
        
        logger.info(f"Data split completed:")
        logger.info(f"  - Total original points: {len(positions_A)}")
        logger.info(f"  - Training set: {len(train_A)} points ({len(train_A)/len(positions_A)*100:.1f}%)")
        logger.info(f"  - Testing set: {len(test_A)} points ({len(test_A)/len(positions_A)*100:.1f}%)")
        logger.info(f"  - Random state: {RANDOM_STATE}")
        
        # Save split information
        self._save_initial_split()
        
        # Visualize initial split
        self._visualize_initial_split()
        
        return True

    def _save_initial_split(self):
        """Save initial train/test split data"""
        # Save raw training set
        if len(self.__train_A_raw) > 0:
            train_data = np.hstack([self.__train_A_raw, self.__train_B_raw])
            np.savetxt(os.path.join(result_dir, "raw_train_data.txt"), train_data, 
                      header="source_x source_y source_z target_x target_y target_z")
            
        # Save raw testing set
        if len(self.__test_A_raw) > 0:
            test_data = np.hstack([self.__test_A_raw, self.__test_B_raw])
            np.savetxt(os.path.join(result_dir, "raw_test_data.txt"), test_data, 
                      header="source_x source_y source_z target_x target_y target_z")
        
        # Save indices
        np.savetxt(os.path.join(result_dir, "train_indices.txt"), self.__train_indices, fmt='%d')
        np.savetxt(os.path.join(result_dir, "test_indices.txt"), self.__test_indices, fmt='%d')
        
        # Save split summary
        with open(os.path.join(result_dir, "initial_split_summary.txt"), "w") as f:
            f.write("Initial Train/Test Split Summary (Split First Approach)\n")
            f.write("=" * 60 + "\n")
            f.write(f"Approach: Split original data first, then process train set\n")
            f.write(f"Split ratio: {TRAIN_TEST_SPLIT_RATIO:.1f} train / {1-TRAIN_TEST_SPLIT_RATIO:.1f} test\n")
            f.write(f"Random state: {RANDOM_STATE}\n")
            f.write(f"Total original points: {len(self.__train_A_raw) + len(self.__test_A_raw)}\n")
            f.write(f"Training points: {len(self.__train_A_raw)}\n")
            f.write(f"Testing points: {len(self.__test_A_raw)}\n")
            f.write(f"Training percentage: {len(self.__train_A_raw)/(len(self.__train_A_raw) + len(self.__test_A_raw))*100:.2f}%\n")
            f.write(f"Testing percentage: {len(self.__test_A_raw)/(len(self.__train_A_raw) + len(self.__test_A_raw))*100:.2f}%\n")
            f.write(f"\nNext steps:\n")
            f.write(f"1. Apply RANSAC filtering to training set only\n")
            f.write(f"2. Learn SVD transformation on filtered training data\n")
            f.write(f"3. Learn deformation model on training data\n")
            f.write(f"4. Apply learned models to test set for evaluation\n")

    def _visualize_initial_split(self):
        """Visualize initial train/test split"""
        logger.info("Creating initial train/test split visualization...")
        
        fig = plt.figure(figsize=(20, 12))
        
        # 3D visualization
        ax1 = fig.add_subplot(221, projection='3d')
        
        # Plot training points
        ax1.scatter(self.__train_A_raw[:, 0], self.__train_A_raw[:, 2], self.__train_A_raw[:, 1], 
                   c='blue', marker='o', s=40, label=f'Train Source ({len(self.__train_A_raw)})', alpha=0.8)
        ax1.scatter(self.__train_B_raw[:, 0], self.__train_B_raw[:, 2], self.__train_B_raw[:, 1], 
                   c='red', marker='^', s=40, label=f'Train Target ({len(self.__train_B_raw)})', alpha=0.8)
        
        # Plot testing points
        ax1.scatter(self.__test_A_raw[:, 0], self.__test_A_raw[:, 2], self.__test_A_raw[:, 1], 
                   c='lightblue', marker='s', s=40, label=f'Test Source ({len(self.__test_A_raw)})', alpha=0.8)
        ax1.scatter(self.__test_B_raw[:, 0], self.__test_B_raw[:, 2], self.__test_B_raw[:, 1], 
                   c='orange', marker='d', s=40, label=f'Test Target ({len(self.__test_B_raw)})', alpha=0.8)
        
        ax1.set_xlabel('X [m]')
        ax1.set_ylabel('Z [m]')
        ax1.set_zlabel('Y [m]')
        ax1.set_title('3D View - Initial Train/Test Split')
        ax1.legend()
        ax1.set_zlim(-0.1, 0.1)
        
        # XZ plane view
        ax2 = fig.add_subplot(222)
        
        # Plot training points
        ax2.scatter(self.__train_A_raw[:, 0], self.__train_A_raw[:, 2], 
                   c='blue', marker='o', s=40, label=f'Train Source ({len(self.__train_A_raw)})', alpha=0.8)
        ax2.scatter(self.__train_B_raw[:, 0], self.__train_B_raw[:, 2], 
                   c='red', marker='^', s=40, label=f'Train Target ({len(self.__train_B_raw)})', alpha=0.8)
        
        # Plot testing points
        ax2.scatter(self.__test_A_raw[:, 0], self.__test_A_raw[:, 2], 
                   c='lightblue', marker='s', s=40, label=f'Test Source ({len(self.__test_A_raw)})', alpha=0.8)
        ax2.scatter(self.__test_B_raw[:, 0], self.__test_B_raw[:, 2], 
                   c='orange', marker='d', s=40, label=f'Test Target ({len(self.__test_B_raw)})', alpha=0.8)
        
        ax2.set_xlabel('X (Lateral) [m]')
        ax2.set_ylabel('Z (Longitudinal) [m]')
        ax2.set_title('XZ Plane - Initial Train/Test Split')
        ax2.legend()
        ax2.grid(True, alpha=0.3)
        ax2.axis('equal')
        
        # Distribution comparison
        ax3 = fig.add_subplot(223)
        
        # X coordinate distribution
        ax3.hist(self.__train_A_raw[:, 0], bins=20, alpha=0.5, label='Train X', color='blue')
        ax3.hist(self.__test_A_raw[:, 0], bins=20, alpha=0.5, label='Test X', color='orange')
        ax3.set_xlabel('X Coordinate [m]')
        ax3.set_ylabel('Frequency')
        ax3.set_title('X Coordinate Distribution')
        ax3.legend()
        ax3.grid(True, alpha=0.3)
        
        # Z coordinate distribution
        ax4 = fig.add_subplot(224)
        
        ax4.hist(self.__train_A_raw[:, 2], bins=20, alpha=0.5, label='Train Z', color='blue')
        ax4.hist(self.__test_A_raw[:, 2], bins=20, alpha=0.5, label='Test Z', color='orange')
        ax4.set_xlabel('Z Coordinate [m]')
        ax4.set_ylabel('Frequency')
        ax4.set_title('Z Coordinate Distribution')
        ax4.legend()
        ax4.grid(True, alpha=0.3)
        
        plt.tight_layout()
        plt.savefig(os.path.join(img_dir, 'initial_train_test_split_visualization.png'), dpi=300, bbox_inches='tight')
        plt.close()
        
        logger.info("Initial train/test split visualization saved")

    def calculate_svd_transformation(self, positions_A=None, positions_B=None):
        """Stage 2: SVD-based coarse registration with Y-axis constraint (using training data)"""
        if positions_A is not None and positions_B is not None:
            train_A = positions_A
            train_B = positions_B
        else:
            train_A = self.__train_inlier_A
            train_B = self.__train_inlier_B
        
        if train_A is None or train_B is None:
            logger.error("Please provide training data first!")
            return

        positions_A = np.array(train_A)
        positions_B = np.array(train_B)

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
        
        logger.info(f"SVD transformation matrix calculated using {len(train_A)} training inlier points")
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

    def visualize_transformation_stage(self, stage_name, positions_A, positions_B, transformed_positions, rmse_results, residual_vectors=None, is_test=False):
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
        
        dataset_suffix = "Test Set" if is_test else "Train Set"
        
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
        ax1.set_title(f'{stage_name} - 3D Registration ({dataset_suffix})')
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
        ax2.set_title(f'{stage_name} - Error Distribution ({dataset_suffix})')
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
        ax3.set_title(f'{stage_name} - RMSE by Direction ({dataset_suffix})')
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
        ax4.set_title(f'{stage_name} - XZ Plane View ({dataset_suffix})')
        ax4.legend()
        ax4.grid(True, alpha=0.3)
        ax4.axis('equal')
        
        plt.tight_layout()
        
        # 保存阶段特定的文件名
        test_suffix = "_test" if is_test else "_train"
        filename = f'{stage_name.lower().replace(" ", "_").replace("(", "").replace(")", "")}{test_suffix}_results.png'
        plt.savefig(os.path.join(img_dir, filename), dpi=300, bbox_inches='tight')
        plt.close()
        
        # 创建详细的性能摘要图
        self._create_performance_summary(stage_name, rmse_results, is_test)

    def _create_performance_summary(self, stage_name, rmse_results, is_test=False):
        """Create detailed performance summary visualization"""
        fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(16, 6))
        
        dataset_suffix = "Test Set" if is_test else "Train Set"
        
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
        ax1.set_title(f'{stage_name} - Performance vs Targets ({dataset_suffix})')
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
        ax2.set_title(f'{stage_name} - Real-world Equivalent ({dataset_suffix})')
        ax2.legend()
        ax2.grid(True, alpha=0.3)
        
        plt.tight_layout()
        
        test_suffix = "_test" if is_test else "_train"
        filename = f'{stage_name.lower().replace(" ", "_").replace("(", "").replace(")", "")}{test_suffix}_performance.png'
        plt.savefig(os.path.join(img_dir, filename), dpi=300, bbox_inches='tight')
        plt.close()

    def three_stage_registration_split_first(self):
        """Perform three-stage registration with Split First approach"""
        if self.__positions_A is None or self.__positions_B is None:
            logger.error("Please provide position data first!")
            return None, None
        
        logger.info("=== Starting Three-Stage Registration with Split First Approach ===")
        
        # Step 1: Split data first
        if not self.split_data_first():
            logger.error("Failed to split data")
            return None, None
        
        # Step 2: Train RANSAC model on training set only
        logger.info("=== Step 2: RANSAC Filtering on Training Set ===")
        self.ransac_filter = RANSACFilter(
            min_samples=RANSAC_MIN_SAMPLES,
            max_iterations=RANSAC_ITERATIONS,
            inlier_threshold=RANSAC_INLIER_THRESHOLD
        )
        
        train_inlier_A, train_inlier_B, train_outlier_A, train_outlier_B = self.ransac_filter.ransac_filter(
            self.__train_A_raw, self.__train_B_raw
        )
        
        if train_inlier_A is None or len(train_inlier_A) == 0:
            logger.error("RANSAC filtering failed on training set - no inliers found")
            return None, None
        
        self.__train_inlier_A = train_inlier_A
        self.__train_inlier_B = train_inlier_B
        
        # Apply RANSAC model to test set
        test_inlier_A, test_inlier_B, test_outlier_A, test_outlier_B = self.ransac_filter.apply_ransac_model_to_test(
            self.__test_A_raw, self.__test_B_raw
        )
        
        self.__test_inlier_A = test_inlier_A
        self.__test_inlier_B = test_inlier_B
        
        # 可视化RANSAC结果
        self.ransac_filter.visualize_ransac_results(
            self.__train_A_raw, self.__train_B_raw, train_inlier_A, train_inlier_B, 
            train_outlier_A, train_outlier_B, self.__test_A_raw, self.__test_B_raw,
            test_inlier_A, test_inlier_B, test_outlier_A, test_outlier_B
        )
        
        # 保存RANSAC结果
        self.ransac_filter.save_ransac_results(
            self.__train_A_raw, self.__train_B_raw, train_inlier_A, train_inlier_B,
            train_outlier_A, train_outlier_B, self.__test_A_raw, self.__test_B_raw,
            test_inlier_A, test_inlier_B, test_outlier_A, test_outlier_B,
            self.__train_df, self.__test_df
        )
        
        # Step 3: SVD transformation learning (on training inliers only)
        logger.info("=== Step 3: SVD Transformation Learning on Training Inliers ===")
        self.calculate_svd_transformation(train_inlier_A, train_inlier_B)
        
        # Apply SVD transformation to training set
        svd_transformed_train = self.apply_transformation(train_inlier_A, self.T_svd)
        svd_rmse_train = self.calculate_directional_rmse(train_inlier_B, svd_transformed_train)
        
        logger.info(f"SVD Training RMSE: {svd_rmse_train['overall_rmse']*1000:.3f}mm")
        
        # Apply SVD transformation to test set
        if test_inlier_A is not None and len(test_inlier_A) > 0:
            svd_transformed_test = self.apply_transformation(test_inlier_A, self.T_svd)
            svd_rmse_test = self.calculate_directional_rmse(test_inlier_B, svd_transformed_test)
            logger.info(f"SVD Testing RMSE: {svd_rmse_test['overall_rmse']*1000:.3f}mm")
        else:
            logger.warning("No test inliers available for SVD evaluation")
            svd_transformed_test = None
            svd_rmse_test = None
        
        # Calculate residual vectors for training data
        residual_vectors_train = train_inlier_B - svd_transformed_train
        logger.info(f"Calculated training residual vectors for {len(residual_vectors_train)} points")
        
        # Visualize SVD results
        self.visualize_transformation_stage("SVD Coarse Registration", 
                                           train_inlier_A, train_inlier_B, svd_transformed_train, 
                                           svd_rmse_train, residual_vectors_train, is_test=False)
        
        if svd_transformed_test is not None:
            self.visualize_transformation_stage("SVD Coarse Registration", 
                                               test_inlier_A, test_inlier_B, svd_transformed_test, 
                                               svd_rmse_test, None, is_test=True)
        
        # Step 4: Non-rigid deformation learning (on training data only)
        logger.info("=== Step 4: Non-rigid Deformation Learning on Training Data ===")
        
        self.deformation_model = NonRigidDeformationModel(
            method='thin_plate_spline',
            smoothing=0.001,
            constrain_y=True
        )
        
        # 学习形变模型（仅使用训练数据）
        success = self.deformation_model.learn_deformation(svd_transformed_train, residual_vectors_train)
        
        if not success:
            logger.error("Failed to learn deformation model")
            return None, None
        
        # Apply deformation to training set
        predicted_deformation_train = self.deformation_model.predict_deformation(svd_transformed_train)
        final_transformed_train = svd_transformed_train + predicted_deformation_train
        final_transformed_train[:, 1] = 0  # Ensure Y=0
        
        final_rmse_train = self.calculate_directional_rmse(train_inlier_B, final_transformed_train)
        logger.info(f"Final Training RMSE: {final_rmse_train['overall_rmse']*1000:.3f}mm")
        
        # Apply deformation to test set (this is the real evaluation)
        if svd_transformed_test is not None:
            predicted_deformation_test = self.deformation_model.predict_deformation(svd_transformed_test)
            final_transformed_test = svd_transformed_test + predicted_deformation_test
            final_transformed_test[:, 1] = 0  # Ensure Y=0
            
            final_rmse_test = self.calculate_directional_rmse(test_inlier_B, final_transformed_test)
            logger.info(f"Final Testing RMSE: {final_rmse_test['overall_rmse']*1000:.3f}mm")
        else:
            final_transformed_test = None
            final_rmse_test = None
        
        # Calculate final residuals
        final_residuals_train = train_inlier_B - final_transformed_train
        if final_transformed_test is not None:
            final_residuals_test = test_inlier_B - final_transformed_test
        else:
            final_residuals_test = None
        
        # Visualize final results
        self.visualize_transformation_stage("Non-rigid Deformation Final", 
                                           train_inlier_A, train_inlier_B, final_transformed_train, 
                                           final_rmse_train, final_residuals_train, is_test=False)
        
        if final_transformed_test is not None:
            self.visualize_transformation_stage("Non-rigid Deformation Final", 
                                               test_inlier_A, test_inlier_B, final_transformed_test, 
                                               final_rmse_test, final_residuals_test, is_test=True)
        
        # 可视化形变场
        if len(svd_transformed_train) > 0:
            # 计算点云边界
            x_min, x_max = np.min(svd_transformed_train[:, 0]), np.max(svd_transformed_train[:, 0])
            z_min, z_max = np.min(svd_transformed_train[:, 2]), np.max(svd_transformed_train[:, 2])
            
            # 扩展边界
            x_range = x_max - x_min
            z_range = z_max - z_min
            bounds = [
                x_min - 0.1 * x_range, x_max + 0.1 * x_range,
                z_min - 0.1 * z_range, z_max + 0.1 * z_range
            ]
            
            self.deformation_model.visualize_deformation_field(bounds, resolution=30)
        
        # Create comparison visualization
        if svd_rmse_test is not None and final_rmse_test is not None:
            self._create_train_test_comparison(
                svd_rmse_train, svd_rmse_test, 
                final_rmse_train, final_rmse_test
            )
        
        # Print detailed results
        self._print_detailed_results_split_first(
            len(self.__positions_A), len(self.__train_A_raw), len(self.__test_A_raw),
            len(train_inlier_A), len(test_inlier_A) if test_inlier_A is not None else 0,
            svd_rmse_train, svd_rmse_test, final_rmse_train, final_rmse_test
        )
        
        # Save transformation matrices and deformation model
        self._save_transformation_data()
        
        # Return test set results (the true performance indicator)
        return self.T_svd, final_rmse_test

    def _create_train_test_comparison(self, svd_train, svd_test, final_train, final_test):
        """Create comparison visualization between train and test performance"""
        logger.info("Creating train/test performance comparison...")
        
        fig, ((ax1, ax2), (ax3, ax4)) = plt.subplots(2, 2, figsize=(16, 12))
        
        # SVD stage comparison
        categories = ['Lateral', 'Longitudinal', 'Horizontal', 'Overall']
        
        train_svd_values = [
            svd_train['lateral_rmse'] * 1000,
            svd_train['longitudinal_rmse'] * 1000,
            svd_train['horizontal_rmse'] * 1000,
            svd_train['overall_rmse'] * 1000
        ]
        
        test_svd_values = [
            svd_test['lateral_rmse'] * 1000,
            svd_test['longitudinal_rmse'] * 1000,
            svd_test['horizontal_rmse'] * 1000,
            svd_test['overall_rmse'] * 1000
        ]
        
        x = np.arange(len(categories))
        width = 0.35
        
        ax1.bar(x - width/2, train_svd_values, width, label='Train', alpha=0.8, color='blue')
        ax1.bar(x + width/2, test_svd_values, width, label='Test', alpha=0.8, color='orange')
        ax1.axhline(y=TARGET_RMSE*1000, color='red', linestyle='--', alpha=0.7, label=f'Target ({TARGET_RMSE*1000:.1f}mm)')
        
        ax1.set_xlabel('Direction')
        ax1.set_ylabel('RMSE [mm]')
        ax1.set_title('SVD Stage - Train vs Test Performance')
        ax1.set_xticks(x)
        ax1.set_xticklabels(categories)
        ax1.legend()
        ax1.grid(True, alpha=0.3)
        
        # Final stage comparison
        train_final_values = [
            final_train['lateral_rmse'] * 1000,
            final_train['longitudinal_rmse'] * 1000,
            final_train['horizontal_rmse'] * 1000,
            final_train['overall_rmse'] * 1000
        ]
        
        test_final_values = [
            final_test['lateral_rmse'] * 1000,
            final_test['longitudinal_rmse'] * 1000,
            final_test['horizontal_rmse'] * 1000,
            final_test['overall_rmse'] * 1000
        ]
        
        ax2.bar(x - width/2, train_final_values, width, label='Train', alpha=0.8, color='blue')
        ax2.bar(x + width/2, test_final_values, width, label='Test', alpha=0.8, color='orange')
        ax2.axhline(y=TARGET_RMSE*1000, color='red', linestyle='--', alpha=0.7, label=f'Target ({TARGET_RMSE*1000:.1f}mm)')
        
        ax2.set_xlabel('Direction')
        ax2.set_ylabel('RMSE [mm]')
        ax2.set_title('Final Stage - Train vs Test Performance')
        ax2.set_xticks(x)
        ax2.set_xticklabels(categories)
        ax2.legend()
        ax2.grid(True, alpha=0.3)
        
        # Overall RMSE progression
        stages = ['SVD', 'Final']
        train_overall = [svd_train['overall_rmse'] * 1000, final_train['overall_rmse'] * 1000]
        test_overall = [svd_test['overall_rmse'] * 1000, final_test['overall_rmse'] * 1000]
        
        x_stages = np.arange(len(stages))
        ax3.plot(x_stages, train_overall, 'bo-', linewidth=2, markersize=8, label='Train', alpha=0.8)
        ax3.plot(x_stages, test_overall, 'ro-', linewidth=2, markersize=8, label='Test', alpha=0.8)
        ax3.axhline(y=TARGET_RMSE*1000, color='red', linestyle='--', alpha=0.7, label=f'Target ({TARGET_RMSE*1000:.1f}mm)')
        
        ax3.set_xlabel('Stage')
        ax3.set_ylabel('Overall RMSE [mm]')
        ax3.set_title('RMSE Progression - Train vs Test')
        ax3.set_xticks(x_stages)
        ax3.set_xticklabels(stages)
        ax3.legend()
        ax3.grid(True, alpha=0.3)
        
        # Improvement analysis
        train_improvement = (svd_train['overall_rmse'] - final_train['overall_rmse']) * 1000
        test_improvement = (svd_test['overall_rmse'] - final_test['overall_rmse']) * 1000
        
        improvement_data = [train_improvement, test_improvement]
        improvement_labels = ['Train', 'Test']
        colors = ['blue', 'orange']
        
        bars = ax4.bar(improvement_labels, improvement_data, color=colors, alpha=0.8)
        for i, (bar, val) in enumerate(zip(bars, improvement_data)):
            ax4.text(bar.get_x() + bar.get_width()/2, bar.get_height() + 0.01,
                    f'{val:.2f}mm', ha='center', va='bottom')
        
        ax4.set_ylabel('RMSE Improvement [mm]')
        ax4.set_title('Deformation Model Improvement')
        ax4.grid(True, alpha=0.3)
        
        plt.tight_layout()
        plt.savefig(os.path.join(img_dir, 'split_first_train_test_comparison.png'), dpi=300, bbox_inches='tight')
        plt.close()
        
        logger.info("Split first train/test performance comparison saved")

    def _print_detailed_results_split_first(self, original_count, train_count, test_count,
                                          train_inlier_count, test_inlier_count,
                                          svd_train, svd_test, final_train, final_test):
        """Print detailed performance analysis with split first approach"""
        logger.info("\n=== Detailed Performance Analysis (Split First Approach) ===")
        
        logger.info("Data Processing Summary:")
        logger.info(f"  Original points: {original_count}")
        logger.info(f"  Split into - Train: {train_count}, Test: {test_count}")
        logger.info(f"  After RANSAC - Train inliers: {train_inlier_count}, Test inliers: {test_inlier_count}")
        logger.info(f"  Train inlier ratio: {train_inlier_count/train_count*100:.1f}%")
        logger.info(f"  Test inlier ratio: {test_inlier_count/test_count*100:.1f}%" if test_count > 0 else "  Test inlier ratio: N/A")
        
        logger.info("SVD Stage Results:")
        logger.info(f"  Training RMSE: {svd_train['overall_rmse']*1000:.3f}mm")
        if svd_test is not None:
            logger.info(f"  Testing RMSE: {svd_test['overall_rmse']*1000:.3f}mm")
            logger.info(f"  Generalization gap: {(svd_test['overall_rmse'] - svd_train['overall_rmse'])*1000:.3f}mm")
        else:
            logger.info(f"  Testing RMSE: N/A (no test inliers)")
        
        logger.info("Final (Deformation Corrected) Results:")
        logger.info(f"  Training RMSE: {final_train['overall_rmse']*1000:.3f}mm")
        if final_test is not None:
            logger.info(f"  Testing RMSE: {final_test['overall_rmse']*1000:.3f}mm")
            logger.info(f"  Generalization gap: {(final_test['overall_rmse'] - final_train['overall_rmse'])*1000:.3f}mm")
        else:
            logger.info(f"  Testing RMSE: N/A (no test inliers)")
        
        # Calculate improvements
        train_improvement = (svd_train['overall_rmse'] - final_train['overall_rmse']) * 1000
        if svd_test is not None and final_test is not None:
            test_improvement = (svd_test['overall_rmse'] - final_test['overall_rmse']) * 1000
        else:
            test_improvement = None
        
        logger.info("Deformation Model Improvement:")
        logger.info(f"  Training improvement: {train_improvement:.3f}mm ({train_improvement/(svd_train['overall_rmse']*1000)*100:.1f}%)")
        if test_improvement is not None:
            logger.info(f"  Testing improvement: {test_improvement:.3f}mm ({test_improvement/(svd_test['overall_rmse']*1000)*100:.1f}%)")
        else:
            logger.info(f"  Testing improvement: N/A")
        
        # Overfitting analysis
        if final_test is not None:
            if final_test['overall_rmse'] > final_train['overall_rmse']:
                overfitting_severity = (final_test['overall_rmse'] - final_train['overall_rmse']) / final_train['overall_rmse'] * 100
                logger.info(f"Model Analysis: Potential overfitting detected ({overfitting_severity:.1f}% performance degradation)")
            else:
                logger.info("Model Analysis: Good generalization - test performance matches or exceeds training")
        else:
            logger.info("Model Analysis: Cannot assess - no test results available")
        
        # Target achievement
        train_pass = final_train['overall_rmse'] <= TARGET_RMSE
        if final_test is not None:
            test_pass = final_test['overall_rmse'] <= TARGET_RMSE
        else:
            test_pass = False
        
        logger.info("Target Achievement:")
        logger.info(f"  Training target achieved: {'✓' if train_pass else '✗'}")
        logger.info(f"  Testing target achieved: {'✓' if test_pass else '✗' if final_test is not None else 'N/A'}")
        logger.info(f"  Target RMSE: {TARGET_RMSE*1000:.2f}mm")
        
        # Real-world equivalent for test set (most important)
        if final_test is not None:
            test_real_world = final_test['overall_rmse'] * SCALE_FACTOR * 100
            logger.info(f"Real-world Test Performance: {test_real_world:.2f}cm")
        else:
            logger.info(f"Real-world Test Performance: N/A")

    def _save_transformation_data(self):
        """Save transformation matrices and deformation model data"""
        # Save SVD transformation
        if self.T_svd is not None:
            np.savetxt(os.path.join(result_dir, "T_svd.txt"), self.T_svd)
        
        # Save deformation model parameters if available
        if self.deformation_model is not None and self.deformation_model.source_points is not None:
            # Save control points
            np.savetxt(os.path.join(result_dir, "deformation_control_points.txt"), 
                      self.deformation_model.source_points)
            
            # Save model metadata
            with open(os.path.join(result_dir, "deformation_model_info.txt"), "w") as f:
                f.write(f"Approach: Split First - train on train set, evaluate on test set\n")
                f.write(f"Method: {self.deformation_model.method}\n")
                f.write(f"Smoothing: {self.deformation_model.smoothing}\n")
                f.write(f"Y-axis constrained: {self.deformation_model.constrain_y}\n")
                f.write(f"Control points: {len(self.deformation_model.source_points)}\n")
                f.write(f"Training set size: {len(self.__train_A_raw) if self.__train_A_raw is not None else 'N/A'}\n")
                f.write(f"Testing set size: {len(self.__test_A_raw) if self.__test_A_raw is not None else 'N/A'}\n")
                f.write(f"Train inliers: {len(self.__train_inlier_A) if self.__train_inlier_A is not None else 'N/A'}\n")
                f.write(f"Test inliers: {len(self.__test_inlier_A) if self.__test_inlier_A is not None else 'N/A'}\n")
                f.write(f"Split ratio: {TRAIN_TEST_SPLIT_RATIO}\n")
                f.write(f"Random state: {RANDOM_STATE}\n")
            
        logger.info("Transformation data saved to results directory")

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
    logger.info("Starting 1:32 scale model tracker data processing with Split First approach")
    logger.info("Approach: Split data first, then train RANSAC + SVD + Non-rigid Deformation on train set, evaluate on test set")
    logger.info(f"Scale factor: {SCALE_FACTOR}")
    logger.info(f"Target RMSE: {TARGET_RMSE*1000:.2f}mm")
    logger.info(f"Train/Test split ratio: {TRAIN_TEST_SPLIT_RATIO:.1f}/{1-TRAIN_TEST_SPLIT_RATIO:.1f}")
    logger.info(f"Random state: {RANDOM_STATE}")
    logger.info(f"RANSAC parameters: min_samples={RANSAC_MIN_SAMPLES}, iterations={RANSAC_ITERATIONS}, threshold={RANSAC_INLIER_THRESHOLD*1000:.1f}mm")
    
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
    
    # Perform three-stage registration with split first approach
    transformer = CoordinateTransformer(positions_A, positions_B, df)
    T_svd, final_rmse_test = transformer.three_stage_registration_split_first()
    
    if T_svd is not None:
        # Success is determined by test set performance (if available)
        if final_rmse_test is not None:
            success = final_rmse_test['overall_rmse'] <= TARGET_RMSE
            test_rmse_msg = f"test RMSE {final_rmse_test['overall_rmse']*1000:.3f}mm"
            test_real_world = final_rmse_test['overall_rmse'] * SCALE_FACTOR * 100
        else:
            success = False
            test_rmse_msg = "no test results (insufficient test inliers)"
            test_real_world = None
        
        if success:
            logger.info(f"SUCCESS: Registration completed with {test_rmse_msg}")
        else:
            logger.info(f"PARTIAL: Registration completed, but {test_rmse_msg}")
        
        # Save final summary
        summary_file = os.path.join(output_dir, "registration_summary_split_first.txt")
        with open(summary_file, "w") as f:
            f.write("Three-Stage Registration Summary with Split First Approach\n")
            f.write("=" * 70 + "\n")
            f.write(f"Approach: Split data first, then train on train set, evaluate on test set\n")
            f.write(f"Method: RANSAC + SVD + Non-rigid Deformation Learning\n")
            f.write(f"Scale Factor: {SCALE_FACTOR}\n")
            f.write(f"Target RMSE: {TARGET_RMSE*1000:.2f}mm\n")
            f.write(f"Test Set RMSE: {final_rmse_test['overall_rmse']*1000:.3f}mm" if final_rmse_test else "Test Set RMSE: N/A (no test inliers)\n")
            f.write(f"Success: {'Yes' if success else 'No'}\n")
            f.write(f"\nApproach Details:\n")
            f.write(f"  1. Split original data into train/test ({TRAIN_TEST_SPLIT_RATIO:.1f}/{1-TRAIN_TEST_SPLIT_RATIO:.1f})\n")
            f.write(f"  2. Apply RANSAC filtering to training set only\n")
            f.write(f"  3. Learn SVD transformation on training inliers\n")
            f.write(f"  4. Learn deformation model on training data\n")
            f.write(f"  5. Apply learned models to test set for evaluation\n")
            f.write(f"\nData Processing:\n")
            f.write(f"  Random state: {RANDOM_STATE}\n")
            f.write(f"  Original points: {len(positions_A)}\n")
            if hasattr(transformer, '_CoordinateTransformer__train_A_raw') and transformer._CoordinateTransformer__train_A_raw is not None:
                f.write(f"  Training points: {len(transformer._CoordinateTransformer__train_A_raw)}\n")
                f.write(f"  Testing points: {len(transformer._CoordinateTransformer__test_A_raw)}\n")
                f.write(f"  Training inliers: {len(transformer._CoordinateTransformer__train_inlier_A) if transformer._CoordinateTransformer__train_inlier_A is not None else 'N/A'}\n")
                f.write(f"  Testing inliers: {len(transformer._CoordinateTransformer__test_inlier_A) if transformer._CoordinateTransformer__test_inlier_A is not None else 'N/A'}\n")
            
            if final_rmse_test is not None:
                f.write(f"\nTest Set Performance (Final Evaluation):\n")
                f.write(f"  Real-world equivalent: {test_real_world:.2f}cm\n")
                f.write(f"  Lateral RMSE: {final_rmse_test['lateral_rmse']*1000:.3f}mm\n")
                f.write(f"  Longitudinal RMSE: {final_rmse_test['longitudinal_rmse']*1000:.3f}mm\n")
            else:
                f.write(f"\nTest Set Performance: N/A (insufficient test inliers after RANSAC)\n")
                
            f.write(f"\nRANSAC Parameters:\n")
            f.write(f"  Min samples: {RANSAC_MIN_SAMPLES}\n")
            f.write(f"  Iterations: {RANSAC_ITERATIONS}\n")
            f.write(f"  Inlier threshold: {RANSAC_INLIER_THRESHOLD*1000:.1f}mm\n")
            f.write(f"\nDeformation Model:\n")
            f.write(f"  Method: Thin-plate spline interpolation\n")
            f.write(f"  Y-axis constraint: Enabled\n")
            f.write(f"  Training: Train set only\n")
            f.write(f"  Evaluation: Test set (completely unseen data)\n")
            f.write(f"\nKey Advantage:\n")
            f.write(f"  - More realistic evaluation: test set never seen during training\n")
            f.write(f"  - Better assessment of model generalization capability\n")
            f.write(f"  - Reduced risk of overfitting evaluation\n")
    else:
        logger.error("Registration failed")
    
    logger.info("Processing complete")

if __name__ == "__main__":
    main()