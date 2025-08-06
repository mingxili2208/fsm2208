#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Tracker data processing script with Train/Test Split First, then RANSAC + SVD + Non-rigid Deformation Learning (Y-axis constrained)
Train on train set, evaluate on test set
Enhanced with baseline SVD and detailed visualizations (English labels)
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

# Set matplotlib to use English labels
plt.rcParams['font.family'] = 'sans-serif'
plt.rcParams['font.sans-serif'] = ['Arial', 'DejaVu Sans', 'Liberation Sans']

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
output_dir = os.path.join(result_dir, f"Enhanced_SplitFirst_RANSAC_SVD_Deformation_{timestamp}_1to32_scale_RMSE_{TARGET_RMSE*1000:.1f}mm_{csv_filename}")
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

class CoordinateTransformer:
    """
    Three-stage coordinate transformation with Split First approach:
    1. Split data into train/test first
    2. Train on train set: RANSAC filtering + SVD coarse + Non-rigid deformation learning
    3. Evaluate on test set
    """
    def __init__(self, positions_A=None, positions_B=None, original_df=None):
        self.T_svd = None
        self.T_baseline = None  # 原初baseline SVD变换
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
        
        # Baseline results storage
        self.__baseline_rmse_train = None
        self.__baseline_rmse_test = None
        self.__ransac_svd_rmse_train = None
        self.__ransac_svd_rmse_test = None
        self.__final_rmse_train = None
        self.__final_rmse_test = None
        
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
        
        # Generate Figure 1: Training and Testing Data Distribution
        self.create_train_test_distribution_visualization()
        
        return True

    def create_train_test_distribution_visualization(self):
        """
        Figure 1: Training and Testing Data Distribution Visualization
        """
        logger.info("Creating Figure 1: Train/Test Data Distribution Visualization...")
        
        fig = plt.figure(figsize=(20, 12))
        
        # Main title
        fig.suptitle('Figure 1: Spatial Distribution of Training and Testing Data Points', 
                     fontsize=16, fontweight='bold')
        
        # 3D spatial distribution
        ax1 = fig.add_subplot(221, projection='3d')
        
        # Plot training set
        ax1.scatter(self.__train_A_raw[:, 0], self.__train_A_raw[:, 2], self.__train_A_raw[:, 1], 
                   c='blue', marker='o', s=30, label=f'Train Source ({len(self.__train_A_raw)})', alpha=0.7)
        ax1.scatter(self.__train_B_raw[:, 0], self.__train_B_raw[:, 2], self.__train_B_raw[:, 1], 
                   c='red', marker='^', s=30, label=f'Train Target ({len(self.__train_B_raw)})', alpha=0.7)
        
        # Plot testing set
        ax1.scatter(self.__test_A_raw[:, 0], self.__test_A_raw[:, 2], self.__test_A_raw[:, 1], 
                   c='lightblue', marker='s', s=30, label=f'Test Source ({len(self.__test_A_raw)})', alpha=0.7)
        ax1.scatter(self.__test_B_raw[:, 0], self.__test_B_raw[:, 2], self.__test_B_raw[:, 1], 
                   c='orange', marker='d', s=30, label=f'Test Target ({len(self.__test_B_raw)})', alpha=0.7)
        
        ax1.set_xlabel('X (Lateral) [m]')
        ax1.set_ylabel('Z (Longitudinal) [m]')
        ax1.set_zlabel('Y (Vertical) [m]')
        ax1.set_title('(a) 3D Spatial Distribution')
        ax1.legend()
        ax1.set_zlim(-0.1, 0.1)
        
        # XZ plane projection
        ax2 = fig.add_subplot(222)
        
        ax2.scatter(self.__train_A_raw[:, 0], self.__train_A_raw[:, 2], 
                   c='blue', marker='o', s=30, label='Train Source', alpha=0.8)
        ax2.scatter(self.__train_B_raw[:, 0], self.__train_B_raw[:, 2], 
                   c='red', marker='^', s=30, label='Train Target', alpha=0.8)
        ax2.scatter(self.__test_A_raw[:, 0], self.__test_A_raw[:, 2], 
                   c='lightblue', marker='s', s=30, label='Test Source', alpha=0.8)
        ax2.scatter(self.__test_B_raw[:, 0], self.__test_B_raw[:, 2], 
                   c='orange', marker='d', s=30, label='Test Target', alpha=0.8)
        
        ax2.set_xlabel('X (Lateral) [m]')
        ax2.set_ylabel('Z (Longitudinal) [m]')
        ax2.set_title('(b) XZ Plane Projection (Navigation Plane)')
        ax2.legend()
        ax2.grid(True, alpha=0.3)
        ax2.axis('equal')
        
        # X coordinate distribution
        ax3 = fig.add_subplot(223)
        
        bins_x = np.linspace(min(np.min(self.__train_A_raw[:, 0]), np.min(self.__test_A_raw[:, 0])),
                            max(np.max(self.__train_A_raw[:, 0]), np.max(self.__test_A_raw[:, 0])), 20)
        
        ax3.hist(self.__train_A_raw[:, 0], bins=bins_x, alpha=0.6, label='Training Set', color='blue', density=True)
        ax3.hist(self.__test_A_raw[:, 0], bins=bins_x, alpha=0.6, label='Testing Set', color='orange', density=True)
        ax3.set_xlabel('X Coordinate [m]')
        ax3.set_ylabel('Density')
        ax3.set_title('(c) X Coordinate Distribution')
        ax3.legend()
        ax3.grid(True, alpha=0.3)
        
        # Z coordinate distribution
        ax4 = fig.add_subplot(224)
        
        bins_z = np.linspace(min(np.min(self.__train_A_raw[:, 2]), np.min(self.__test_A_raw[:, 2])),
                            max(np.max(self.__train_A_raw[:, 2]), np.max(self.__test_A_raw[:, 2])), 20)
        
        ax4.hist(self.__train_A_raw[:, 2], bins=bins_z, alpha=0.6, label='Training Set', color='blue', density=True)
        ax4.hist(self.__test_A_raw[:, 2], bins=bins_z, alpha=0.6, label='Testing Set', color='orange', density=True)
        ax4.set_xlabel('Z Coordinate [m]')
        ax4.set_ylabel('Density')
        ax4.set_title('(d) Z Coordinate Distribution')
        ax4.legend()
        ax4.grid(True, alpha=0.3)
        
        plt.tight_layout()
        plt.savefig(os.path.join(img_dir, 'Figure1_train_test_distribution.png'), dpi=300, bbox_inches='tight')
        plt.close()
        
        logger.info("Figure 1: Train/Test Distribution visualization saved")

    def calculate_baseline_svd(self):
        """
        Calculate initial baseline SVD transformation (after train/test split, before RANSAC)
        """
        logger.info("=== Calculating Initial Baseline SVD Transformation ===")
        
        # Calculate baseline SVD on training set
        R_baseline, t_baseline = self.compute_2d_svd_transformation(self.__train_A_raw, self.__train_B_raw)
        
        # Construct 3D transformation matrix
        self.T_baseline = np.eye(4)
        self.T_baseline[0, 0] = R_baseline[0, 0]
        self.T_baseline[0, 2] = R_baseline[0, 1]
        self.T_baseline[2, 0] = R_baseline[1, 0]
        self.T_baseline[2, 2] = R_baseline[1, 1]
        self.T_baseline[0, 3] = t_baseline[0]
        self.T_baseline[2, 3] = t_baseline[1]
        
        # Apply baseline transformation to training and testing sets
        baseline_transformed_train = self.apply_transformation(self.__train_A_raw, self.T_baseline)
        baseline_transformed_test = self.apply_transformation(self.__test_A_raw, self.T_baseline)
        
        # Calculate baseline RMSE
        self.__baseline_rmse_train = self.calculate_directional_rmse(self.__train_B_raw, baseline_transformed_train)
        self.__baseline_rmse_test = self.calculate_directional_rmse(self.__test_B_raw, baseline_transformed_test)
        
        logger.info(f"Baseline SVD Results:")
        logger.info(f"  - Training RMSE: {self.__baseline_rmse_train['overall_rmse']*1000:.3f}mm")
        logger.info(f"  - Testing RMSE: {self.__baseline_rmse_test['overall_rmse']*1000:.3f}mm")
        
        return R_baseline, t_baseline
    
    def compute_2d_svd_transformation(self, points_A, points_B):
        """Calculate 2D SVD transformation (XZ plane only)"""
        points_A = np.array(points_A)
        points_B = np.array(points_B)
        
        # Use XZ coordinates only
        points_A_xz = points_A[:, [0, 2]]
        points_B_xz = points_B[:, [0, 2]]
        
        # Calculate centroids
        centroid_A = np.mean(points_A_xz, axis=0)
        centroid_B = np.mean(points_B_xz, axis=0)
        
        # Center the points
        A_centered = points_A_xz - centroid_A
        B_centered = points_B_xz - centroid_B
        
        # SVD to calculate rotation matrix
        H = np.dot(A_centered.T, B_centered)
        U, S, Vt = np.linalg.svd(H)
        R = np.dot(Vt.T, U.T)
        
        # Ensure proper rotation matrix
        if np.linalg.det(R) < 0:
            Vt[-1, :] *= -1
            R = np.dot(Vt.T, U.T)
        
        # Calculate translation vector
        t = centroid_B - np.dot(R, centroid_A)
        
        return R, t

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

        # Use XZ coordinates only for SVD transformation
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
        
        # Construct 3D transformation matrix
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
        # Ensure Y coordinate remains 0
        result[:, 1] = 0
        return result

    def calculate_directional_rmse(self, actual, predicted):
        """Calculate RMSE for each direction separately"""
        actual = np.array(actual)
        predicted = np.array(predicted)
        
        # Only consider XZ plane errors since Y-axis is constrained to 0
        rmse_lateral = np.sqrt(np.mean((actual[:, 0] - predicted[:, 0])**2))
        rmse_vertical = 0.0  # Y-axis is constrained, error is 0
        rmse_longitudinal = np.sqrt(np.mean((actual[:, 2] - predicted[:, 2])**2))
        
        overall_rmse = np.sqrt(rmse_lateral**2 + rmse_longitudinal**2)
        horizontal_rmse = overall_rmse  # Same as overall_rmse since Y=0
        
        return {
            'lateral_rmse': rmse_lateral,
            'vertical_rmse': rmse_vertical,
            'longitudinal_rmse': rmse_longitudinal,
            'horizontal_rmse': horizontal_rmse,
            'overall_rmse': overall_rmse
        }

    def create_ransac_svd_visualization(self, train_inlier_A, train_inlier_B, train_outlier_A, train_outlier_B,
                                       test_inlier_A, test_inlier_B, svd_transformed_train, svd_transformed_test):
        """
        Figure 2: RANSAC Outlier Removal and SVD Coarse Registration
        """
        logger.info("Creating Figure 2: RANSAC Outlier Removal and SVD Coarse Registration...")
        
        fig = plt.figure(figsize=(20, 12))
        fig.suptitle('Figure 2: RANSAC Outlier Removal and SVD Coarse Registration Results', 
                     fontsize=16, fontweight='bold')
        
        # (a) RANSAC outlier removal results - training set
        ax1 = fig.add_subplot(221, projection='3d')
        
        # Plot inliers
        if len(train_inlier_A) > 0:
            ax1.scatter(train_inlier_A[:, 0], train_inlier_A[:, 2], train_inlier_A[:, 1], 
                       c='green', marker='o', s=40, label=f'Train Inliers ({len(train_inlier_A)})', alpha=0.8)
            ax1.scatter(train_inlier_B[:, 0], train_inlier_B[:, 2], train_inlier_B[:, 1], 
                       c='darkgreen', marker='^', s=40, label=f'Target Points ({len(train_inlier_B)})', alpha=0.8)
        
        # Plot outliers
        if len(train_outlier_A) > 0:
            ax1.scatter(train_outlier_A[:, 0], train_outlier_A[:, 2], train_outlier_A[:, 1], 
                       c='red', marker='x', s=60, label=f'Outliers ({len(train_outlier_A)})', alpha=0.8)
            ax1.scatter(train_outlier_B[:, 0], train_outlier_B[:, 2], train_outlier_B[:, 1], 
                       c='darkred', marker='x', s=60, alpha=0.8)
        
        ax1.set_xlabel('X (Lateral) [m]')
        ax1.set_ylabel('Z (Longitudinal) [m]')
        ax1.set_zlabel('Y (Vertical) [m]')
        ax1.set_title('(a) RANSAC Outlier Removal (Training Set)')
        ax1.legend()
        ax1.set_zlim(-0.1, 0.1)
        
        # (b) SVD coarse registration effect - training set
        ax2 = fig.add_subplot(222)
        
        # Before and after transformation comparison
        ax2.scatter(train_inlier_A[:, 0], train_inlier_A[:, 2], 
                   c='blue', marker='o', s=40, label='Before Transform', alpha=0.6)
        ax2.scatter(train_inlier_B[:, 0], train_inlier_B[:, 2], 
                   c='red', marker='^', s=40, label='Target Points', alpha=0.8)
        ax2.scatter(svd_transformed_train[:, 0], svd_transformed_train[:, 2], 
                   c='green', marker='s', s=40, label='After SVD Transform', alpha=0.8)
        
        # Draw some registration vectors
        for i in range(0, len(train_inlier_A), max(1, len(train_inlier_A)//20)):
            ax2.arrow(train_inlier_A[i, 0], train_inlier_A[i, 2], 
                     svd_transformed_train[i, 0] - train_inlier_A[i, 0],
                     svd_transformed_train[i, 2] - train_inlier_A[i, 2],
                     head_width=0.002, head_length=0.003, fc='purple', ec='purple', alpha=0.6)
        
        ax2.set_xlabel('X (Lateral) [m]')
        ax2.set_ylabel('Z (Longitudinal) [m]')
        ax2.set_title(f'(b) SVD Coarse Registration (Training Set)\nRMSE: {self.__ransac_svd_rmse_train["overall_rmse"]*1000:.2f}mm')
        ax2.legend()
        ax2.grid(True, alpha=0.3)
        ax2.axis('equal')
        
        # (c) Test set SVD application
        ax3 = fig.add_subplot(223)
        
        if test_inlier_A is not None and len(test_inlier_A) > 0:
            ax3.scatter(test_inlier_A[:, 0], test_inlier_A[:, 2], 
                       c='lightblue', marker='o', s=40, label='Test Source Points', alpha=0.6)
            ax3.scatter(test_inlier_B[:, 0], test_inlier_B[:, 2], 
                       c='orange', marker='^', s=40, label='Test Target Points', alpha=0.8)
            ax3.scatter(svd_transformed_test[:, 0], svd_transformed_test[:, 2], 
                       c='cyan', marker='s', s=40, label='After SVD Transform', alpha=0.8)
            
            # Draw some registration vectors
            for i in range(0, len(test_inlier_A), max(1, len(test_inlier_A)//15)):
                ax3.arrow(test_inlier_A[i, 0], test_inlier_A[i, 2], 
                         svd_transformed_test[i, 0] - test_inlier_A[i, 0],
                         svd_transformed_test[i, 2] - test_inlier_A[i, 2],
                         head_width=0.002, head_length=0.003, fc='purple', ec='purple', alpha=0.6)
        
        ax3.set_xlabel('X (Lateral) [m]')
        ax3.set_ylabel('Z (Longitudinal) [m]')
        ax3.set_title(f'(c) Test Set SVD Registration\nRMSE: {self.__ransac_svd_rmse_test["overall_rmse"]*1000:.2f}mm' if self.__ransac_svd_rmse_test else '(c) Test Set SVD Registration')
        ax3.legend()
        ax3.grid(True, alpha=0.3)
        ax3.axis('equal')
        
        # (d) Residual distribution analysis
        ax4 = fig.add_subplot(224)
        
        # Calculate residuals
        residuals_train = train_inlier_B - svd_transformed_train
        residual_magnitudes_train = np.sqrt(residuals_train[:, 0]**2 + residuals_train[:, 2]**2) * 1000
        
        if svd_transformed_test is not None and len(svd_transformed_test) > 0:
            residuals_test = test_inlier_B - svd_transformed_test
            residual_magnitudes_test = np.sqrt(residuals_test[:, 0]**2 + residuals_test[:, 2]**2) * 1000
        else:
            residual_magnitudes_test = []
        
        # Plot residual distribution
        bins = np.linspace(0, max(np.max(residual_magnitudes_train), 
                                np.max(residual_magnitudes_test) if len(residual_magnitudes_test) > 0 else np.max(residual_magnitudes_train)), 20)
        
        ax4.hist(residual_magnitudes_train, bins=bins, alpha=0.6, label='Training Residuals', color='blue', density=True)
        if len(residual_magnitudes_test) > 0:
            ax4.hist(residual_magnitudes_test, bins=bins, alpha=0.6, label='Testing Residuals', color='orange', density=True)
        
        ax4.axvline(x=TARGET_RMSE*1000, color='red', linestyle='--', linewidth=2, 
                   label=f'Target Accuracy ({TARGET_RMSE*1000:.1f}mm)')
        
        ax4.set_xlabel('Residual Magnitude [mm]')
        ax4.set_ylabel('Density')
        ax4.set_title('(d) SVD Registration Residual Distribution')
        ax4.legend()
        ax4.grid(True, alpha=0.3)
        
        plt.tight_layout()
        plt.savefig(os.path.join(img_dir, 'Figure2_ransac_svd_registration.png'), dpi=300, bbox_inches='tight')
        plt.close()
        
        logger.info("Figure 2: RANSAC and SVD Registration visualization saved")

    def create_deformation_field_and_final_results_visualization(self, svd_transformed_train, train_inlier_B,
                                                                final_transformed_train, svd_transformed_test,
                                                                test_inlier_B, final_transformed_test):
        """
        Figure 3: Non-rigid Deformation Field Visualization and Final Registration Results
        """
        logger.info("Creating Figure 3: Deformation Field and Final Registration Results...")
        
        fig = plt.figure(figsize=(20, 12))
        fig.suptitle('Figure 3: Non-rigid Deformation Field and Final Registration Results', 
                     fontsize=16, fontweight='bold')
        
        # (a) Deformation field visualization
        ax1 = fig.add_subplot(221)
        
        # Calculate residual vectors
        residual_vectors = train_inlier_B - svd_transformed_train
        
        # Create deformation field grid
        x_min, x_max = np.min(svd_transformed_train[:, 0]), np.max(svd_transformed_train[:, 0])
        z_min, z_max = np.min(svd_transformed_train[:, 2]), np.max(svd_transformed_train[:, 2])
        x_range = x_max - x_min
        z_range = z_max - z_min
        
        # Extend boundaries
        x_min_ext = x_min - 0.1 * x_range
        x_max_ext = x_max + 0.1 * x_range
        z_min_ext = z_min - 0.1 * z_range
        z_max_ext = z_max + 0.1 * z_range
        
        # Create grid
        resolution = 20
        x_grid = np.linspace(x_min_ext, x_max_ext, resolution)
        z_grid = np.linspace(z_min_ext, z_max_ext, resolution)
        X, Z = np.meshgrid(x_grid, z_grid)
        
        # Use learned deformation model to predict grid deformation
        if self.deformation_model and self.deformation_model.rbf_x and self.deformation_model.rbf_z:
            query_points_2d = np.column_stack([X.ravel(), Z.ravel()])
            try:
                deformation_x = self.deformation_model.rbf_x(query_points_2d)
                deformation_z = self.deformation_model.rbf_z(query_points_2d)
                
                Dx = deformation_x.reshape(X.shape)
                Dz = deformation_z.reshape(X.shape)
                
                # Plot deformation magnitude contour
                magnitude = np.sqrt(Dx**2 + Dz**2) * 1000
                contour = ax1.contourf(X, Z, magnitude, levels=15, cmap='viridis', alpha=0.7)
                plt.colorbar(contour, ax=ax1, label='Deformation Magnitude [mm]')
                
                # Plot deformation vector field (downsampled)
                step = 2
                ax1.quiver(X[::step, ::step], Z[::step, ::step], 
                          Dx[::step, ::step]*500, Dz[::step, ::step]*500,  # Scale arrows for visibility
                          scale=1, scale_units='xy', angles='xy', alpha=0.8, color='white')
                
            except:
                logger.warning("Could not visualize interpolated deformation field")
        
        # Overlay training point deformation vectors
        scale_factor = 200  # Scale vectors for visibility
        ax1.quiver(svd_transformed_train[:, 0], svd_transformed_train[:, 2], 
                  residual_vectors[:, 0] * scale_factor, residual_vectors[:, 2] * scale_factor,
                  angles='xy', scale_units='xy', scale=1, color='red', alpha=0.8, width=0.003)
        
        # Plot control points
        ax1.scatter(svd_transformed_train[:, 0], svd_transformed_train[:, 2], 
                   c='black', s=20, marker='o', alpha=0.8, label='Control Points')
        
        ax1.set_xlabel('X (Lateral) [m]')
        ax1.set_ylabel('Z (Longitudinal) [m]')
        ax1.set_title('(a) Learned Non-rigid Deformation Field')
        ax1.legend()
        ax1.grid(True, alpha=0.3)
        ax1.axis('equal')
        
        # (b) Final registration results - training set
        ax2 = fig.add_subplot(222)
        
        ax2.scatter(train_inlier_B[:, 0], train_inlier_B[:, 2], 
                   c='red', marker='^', s=50, label='Target Points', alpha=0.8)
        ax2.scatter(final_transformed_train[:, 0], final_transformed_train[:, 2], 
                   c='green', marker='o', s=30, label='Final Registration', alpha=0.8)
        
        # Calculate and show final residuals
        final_residuals_train = train_inlier_B - final_transformed_train
        for i in range(0, len(train_inlier_B), max(1, len(train_inlier_B)//30)):
            ax2.plot([train_inlier_B[i, 0], final_transformed_train[i, 0]], 
                    [train_inlier_B[i, 2], final_transformed_train[i, 2]], 
                    'k--', alpha=0.3, linewidth=0.5)
        
        ax2.set_xlabel('X (Lateral) [m]')
        ax2.set_ylabel('Z (Longitudinal) [m]')
        ax2.set_title(f'(b) Final Registration (Training Set)\nRMSE: {self.__final_rmse_train["overall_rmse"]*1000:.2f}mm')
        ax2.legend()
        ax2.grid(True, alpha=0.3)
        ax2.axis('equal')
        
        # (c) Final registration results - test set
        ax3 = fig.add_subplot(223)
        
        if final_transformed_test is not None and len(final_transformed_test) > 0:
            ax3.scatter(test_inlier_B[:, 0], test_inlier_B[:, 2], 
                       c='orange', marker='^', s=50, label='Test Target Points', alpha=0.8)
            ax3.scatter(final_transformed_test[:, 0], final_transformed_test[:, 2], 
                       c='cyan', marker='o', s=30, label='Final Registration', alpha=0.8)
            
            # Calculate and show final residuals
            for i in range(0, len(test_inlier_B), max(1, len(test_inlier_B)//20)):
                ax3.plot([test_inlier_B[i, 0], final_transformed_test[i, 0]], 
                        [test_inlier_B[i, 2], final_transformed_test[i, 2]], 
                        'k--', alpha=0.3, linewidth=0.5)
        
        ax3.set_xlabel('X (Lateral) [m]')
        ax3.set_ylabel('Z (Longitudinal) [m]')
        ax3.set_title(f'(c) Final Registration (Test Set)\nRMSE: {self.__final_rmse_test["overall_rmse"]*1000:.2f}mm' if self.__final_rmse_test else '(c) Final Registration (Test Set)')
        ax3.legend()
        ax3.grid(True, alpha=0.3)
        ax3.axis('equal')
        
        # (d) Before and after deformation accuracy comparison
        ax4 = fig.add_subplot(224)
        
        # Calculate accuracy improvement
        stages = ['After SVD', 'Final Registration']
        train_rmse_values = [self.__ransac_svd_rmse_train['overall_rmse']*1000, 
                            self.__final_rmse_train['overall_rmse']*1000]
        
        if self.__final_rmse_test:
            test_rmse_values = [self.__ransac_svd_rmse_test['overall_rmse']*1000, 
                               self.__final_rmse_test['overall_rmse']*1000]
        else:
            test_rmse_values = [0, 0]
        
        x = np.arange(len(stages))
        width = 0.35
        
        ax4.bar(x - width/2, train_rmse_values, width, label='Training Set', alpha=0.8, color='blue')
        if any(test_rmse_values):
            ax4.bar(x + width/2, test_rmse_values, width, label='Test Set', alpha=0.8, color='orange')
        
        ax4.axhline(y=TARGET_RMSE*1000, color='red', linestyle='--', linewidth=2, 
                   label=f'Target Accuracy ({TARGET_RMSE*1000:.1f}mm)')
        
        # Add value labels
        for i, v in enumerate(train_rmse_values):
            ax4.text(i - width/2, v + 0.05, f'{v:.2f}', ha='center', va='bottom')
        if any(test_rmse_values):
            for i, v in enumerate(test_rmse_values):
                if v > 0:
                    ax4.text(i + width/2, v + 0.05, f'{v:.2f}', ha='center', va='bottom')
        
        ax4.set_xlabel('Registration Stage')
        ax4.set_ylabel('RMSE [mm]')
        ax4.set_title('(d) Deformation Correction Accuracy Comparison')
        ax4.set_xticks(x)
        ax4.set_xticklabels(stages)
        ax4.legend()
        ax4.grid(True, alpha=0.3)
        
        plt.tight_layout()
        plt.savefig(os.path.join(img_dir, 'Figure3_deformation_and_final_results.png'), dpi=300, bbox_inches='tight')
        plt.close()
        
        logger.info("Figure 3: Deformation Field and Final Results visualization saved")

    def create_algorithm_stages_comparison_visualization(self):
        """
        Figure 4: Algorithm Stages Accuracy Comparison
        """
        logger.info("Creating Figure 4: Algorithm Stages Accuracy Comparison...")
        
        fig, ((ax1, ax2), (ax3, ax4)) = plt.subplots(2, 2, figsize=(16, 12))
        fig.suptitle('Figure 4: Algorithm Stages Accuracy Comparison', 
                     fontsize=16, fontweight='bold')
        
        # (a) Overall RMSE comparison
        stages = ['Initial Error', 'SVD Coarse', 'Final Registration']
        
        # Training set data
        train_overall_rmse = [
            self.__baseline_rmse_train['overall_rmse'] * 1000,
            self.__ransac_svd_rmse_train['overall_rmse'] * 1000,
            self.__final_rmse_train['overall_rmse'] * 1000
        ]
        
        # Test set data
        if self.__final_rmse_test:
            test_overall_rmse = [
                self.__baseline_rmse_test['overall_rmse'] * 1000,
                self.__ransac_svd_rmse_test['overall_rmse'] * 1000,
                self.__final_rmse_test['overall_rmse'] * 1000
            ]
        else:
            test_overall_rmse = [0, 0, 0]
        
        x = np.arange(len(stages))
        width = 0.35
        
        bars1 = ax1.bar(x - width/2, train_overall_rmse, width, label='Training Set', alpha=0.8, color='blue')
        if any(test_overall_rmse):
            bars2 = ax1.bar(x + width/2, test_overall_rmse, width, label='Test Set', alpha=0.8, color='orange')
        
        # Add target line
        ax1.axhline(y=TARGET_RMSE*1000, color='red', linestyle='--', linewidth=2, 
                   label=f'Target Accuracy ({TARGET_RMSE*1000:.1f}mm)')
        
        # Add value labels
        for i, v in enumerate(train_overall_rmse):
            ax1.text(i - width/2, v + 0.1, f'{v:.2f}', ha='center', va='bottom', fontsize=9)
        if any(test_overall_rmse):
            for i, v in enumerate(test_overall_rmse):
                if v > 0:
                    ax1.text(i + width/2, v + 0.1, f'{v:.2f}', ha='center', va='bottom', fontsize=9)
        
        ax1.set_xlabel('Algorithm Stage')
        ax1.set_ylabel('Overall RMSE [mm]')
        ax1.set_title('(a) Overall RMSE Comparison')
        ax1.set_xticks(x)
        ax1.set_xticklabels(stages)
        ax1.legend()
        ax1.grid(True, alpha=0.3)
        
        # (b) Lateral accuracy comparison
        train_lateral_rmse = [
            self.__baseline_rmse_train['lateral_rmse'] * 1000,
            self.__ransac_svd_rmse_train['lateral_rmse'] * 1000,
            self.__final_rmse_train['lateral_rmse'] * 1000
        ]
        
        if self.__final_rmse_test:
            test_lateral_rmse = [
                self.__baseline_rmse_test['lateral_rmse'] * 1000,
                self.__ransac_svd_rmse_test['lateral_rmse'] * 1000,
                self.__final_rmse_test['lateral_rmse'] * 1000
            ]
        else:
            test_lateral_rmse = [0, 0, 0]
        
        bars3 = ax2.bar(x - width/2, train_lateral_rmse, width, label='Training Set', alpha=0.8, color='blue')
        if any(test_lateral_rmse):
            bars4 = ax2.bar(x + width/2, test_lateral_rmse, width, label='Test Set', alpha=0.8, color='orange')
        
        ax2.axhline(y=SCALED_LATERAL_TARGET*1000, color='red', linestyle='--', linewidth=2, 
                   label=f'Lateral Target ({SCALED_LATERAL_TARGET*1000:.1f}mm)')
        
        # Add value labels
        for i, v in enumerate(train_lateral_rmse):
            ax2.text(i - width/2, v + 0.05, f'{v:.2f}', ha='center', va='bottom', fontsize=9)
        if any(test_lateral_rmse):
            for i, v in enumerate(test_lateral_rmse):
                if v > 0:
                    ax2.text(i + width/2, v + 0.05, f'{v:.2f}', ha='center', va='bottom', fontsize=9)
        
        ax2.set_xlabel('Algorithm Stage')
        ax2.set_ylabel('Lateral RMSE [mm]')
        ax2.set_title('(b) Lateral Accuracy Comparison')
        ax2.set_xticks(x)
        ax2.set_xticklabels(stages)
        ax2.legend()
        ax2.grid(True, alpha=0.3)
        
        # (c) Longitudinal accuracy comparison
        train_longitudinal_rmse = [
            self.__baseline_rmse_train['longitudinal_rmse'] * 1000,
            self.__ransac_svd_rmse_train['longitudinal_rmse'] * 1000,
            self.__final_rmse_train['longitudinal_rmse'] * 1000
        ]
        
        if self.__final_rmse_test:
            test_longitudinal_rmse = [
                self.__baseline_rmse_test['longitudinal_rmse'] * 1000,
                self.__ransac_svd_rmse_test['longitudinal_rmse'] * 1000,
                self.__final_rmse_test['longitudinal_rmse'] * 1000
            ]
        else:
            test_longitudinal_rmse = [0, 0, 0]
        
        bars5 = ax3.bar(x - width/2, train_longitudinal_rmse, width, label='Training Set', alpha=0.8, color='blue')
        if any(test_longitudinal_rmse):
            bars6 = ax3.bar(x + width/2, test_longitudinal_rmse, width, label='Test Set', alpha=0.8, color='orange')
        
        ax3.axhline(y=SCALED_LONGITUDINAL_TARGET*1000, color='red', linestyle='--', linewidth=2, 
                   label=f'Longitudinal Target ({SCALED_LONGITUDINAL_TARGET*1000:.1f}mm)')
        
        # Add value labels
        for i, v in enumerate(train_longitudinal_rmse):
            ax3.text(i - width/2, v + 0.05, f'{v:.2f}', ha='center', va='bottom', fontsize=9)
        if any(test_longitudinal_rmse):
            for i, v in enumerate(test_longitudinal_rmse):
                if v > 0:
                    ax3.text(i + width/2, v + 0.05, f'{v:.2f}', ha='center', va='bottom', fontsize=9)
        
        ax3.set_xlabel('Algorithm Stage')
        ax3.set_ylabel('Longitudinal RMSE [mm]')
        ax3.set_title('(c) Longitudinal Accuracy Comparison')
        ax3.set_xticks(x)
        ax3.set_xticklabels(stages)
        ax3.legend()
        ax3.grid(True, alpha=0.3)
        
        # (d) Accuracy improvement percentage
        if self.__baseline_rmse_train['overall_rmse'] > 0:
            train_improvement = [
                0,  # Baseline
                (self.__baseline_rmse_train['overall_rmse'] - self.__ransac_svd_rmse_train['overall_rmse']) / self.__baseline_rmse_train['overall_rmse'] * 100,
                (self.__baseline_rmse_train['overall_rmse'] - self.__final_rmse_train['overall_rmse']) / self.__baseline_rmse_train['overall_rmse'] * 100
            ]
        else:
            train_improvement = [0, 0, 0]
        
        if self.__final_rmse_test and self.__baseline_rmse_test['overall_rmse'] > 0:
            test_improvement = [
                0,  # Baseline
                (self.__baseline_rmse_test['overall_rmse'] - self.__ransac_svd_rmse_test['overall_rmse']) / self.__baseline_rmse_test['overall_rmse'] * 100,
                (self.__baseline_rmse_test['overall_rmse'] - self.__final_rmse_test['overall_rmse']) / self.__baseline_rmse_test['overall_rmse'] * 100
            ]
        else:
            test_improvement = [0, 0, 0]
        
        bars7 = ax4.bar(x - width/2, train_improvement, width, label='Training Set', alpha=0.8, color='blue')
        if any(test_improvement):
            bars8 = ax4.bar(x + width/2, test_improvement, width, label='Test Set', alpha=0.8, color='orange')
        
        # Add value labels
        for i, v in enumerate(train_improvement):
            if v != 0:
                ax4.text(i - width/2, v + 1, f'{v:.1f}%', ha='center', va='bottom', fontsize=9)
        if any(test_improvement):
            for i, v in enumerate(test_improvement):
                if v != 0:
                    ax4.text(i + width/2, v + 1, f'{v:.1f}%', ha='center', va='bottom', fontsize=9)
        
        ax4.set_xlabel('Algorithm Stage')
        ax4.set_ylabel('Accuracy Improvement [%]')
        ax4.set_title('(d) Accuracy Improvement vs Initial Error')
        ax4.set_xticks(x)
        ax4.set_xticklabels(stages)
        ax4.legend()
        ax4.grid(True, alpha=0.3)
        
        plt.tight_layout()
        plt.savefig(os.path.join(img_dir, 'Figure4_algorithm_stages_comparison.png'), dpi=300, bbox_inches='tight')
        plt.close()
        
        logger.info("Figure 4: Algorithm Stages Comparison visualization saved")

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

    def three_stage_registration_split_first(self):
        """Perform three-stage registration with Split First approach"""
        if self.__positions_A is None or self.__positions_B is None:
            logger.error("Please provide position data first!")
            return None, None
        
        logger.info("=== Starting Enhanced Three-Stage Registration with Split First Approach ===")
        
        # Step 1: Split data first
        if not self.split_data_first():
            logger.error("Failed to split data")
            return None, None
        
        # Step 1.5: Calculate baseline SVD transformation
        self.calculate_baseline_svd()
        
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
        
        # Step 3: SVD transformation learning (on training inliers only)
        logger.info("=== Step 3: SVD Transformation Learning on Training Inliers ===")
        self.calculate_svd_transformation(train_inlier_A, train_inlier_B)
        
        # Apply SVD transformation to training set
        svd_transformed_train = self.apply_transformation(train_inlier_A, self.T_svd)
        self.__ransac_svd_rmse_train = self.calculate_directional_rmse(train_inlier_B, svd_transformed_train)
        
        logger.info(f"SVD Training RMSE: {self.__ransac_svd_rmse_train['overall_rmse']*1000:.3f}mm")
        
        # Apply SVD transformation to test set
        if test_inlier_A is not None and len(test_inlier_A) > 0:
            svd_transformed_test = self.apply_transformation(test_inlier_A, self.T_svd)
            self.__ransac_svd_rmse_test = self.calculate_directional_rmse(test_inlier_B, svd_transformed_test)
            logger.info(f"SVD Testing RMSE: {self.__ransac_svd_rmse_test['overall_rmse']*1000:.3f}mm")
        else:
            logger.warning("No test inliers available for SVD evaluation")
            svd_transformed_test = None
            self.__ransac_svd_rmse_test = None
        
        # Generate Figure 2: RANSAC Outlier Removal and SVD Coarse Registration
        self.create_ransac_svd_visualization(train_inlier_A, train_inlier_B, train_outlier_A, train_outlier_B,
                                            test_inlier_A, test_inlier_B, svd_transformed_train, svd_transformed_test)
        
        # Calculate residual vectors for training data
        residual_vectors_train = train_inlier_B - svd_transformed_train
        logger.info(f"Calculated training residual vectors for {len(residual_vectors_train)} points")
        
        # Step 4: Non-rigid deformation learning (on training data only)
        logger.info("=== Step 4: Non-rigid Deformation Learning on Training Data ===")
        
        self.deformation_model = NonRigidDeformationModel(
            method='thin_plate_spline',
            smoothing=0.001,
            constrain_y=True
        )
        
        # Learn deformation model (using training data only)
        success = self.deformation_model.learn_deformation(svd_transformed_train, residual_vectors_train)
        
        if not success:
            logger.error("Failed to learn deformation model")
            return None, None
        
        # Apply deformation to training set
        predicted_deformation_train = self.deformation_model.predict_deformation(svd_transformed_train)
        final_transformed_train = svd_transformed_train + predicted_deformation_train
        final_transformed_train[:, 1] = 0  # Ensure Y=0
        
        self.__final_rmse_train = self.calculate_directional_rmse(train_inlier_B, final_transformed_train)
        logger.info(f"Final Training RMSE: {self.__final_rmse_train['overall_rmse']*1000:.3f}mm")
        
        # Apply deformation to test set (this is the real evaluation)
        if svd_transformed_test is not None:
            predicted_deformation_test = self.deformation_model.predict_deformation(svd_transformed_test)
            final_transformed_test = svd_transformed_test + predicted_deformation_test
            final_transformed_test[:, 1] = 0  # Ensure Y=0
            
            self.__final_rmse_test = self.calculate_directional_rmse(test_inlier_B, final_transformed_test)
            logger.info(f"Final Testing RMSE: {self.__final_rmse_test['overall_rmse']*1000:.3f}mm")
        else:
            final_transformed_test = None
            self.__final_rmse_test = None
        
        # Generate Figure 3: Non-rigid Deformation Field and Final Registration Results
        self.create_deformation_field_and_final_results_visualization(svd_transformed_train, train_inlier_B,
                                                                     final_transformed_train, svd_transformed_test,
                                                                     test_inlier_B, final_transformed_test)
        
        # Generate Figure 4: Algorithm Stages Accuracy Comparison
        self.create_algorithm_stages_comparison_visualization()
        
        # Save RANSAC results
        self.ransac_filter.save_ransac_results(
            self.__train_A_raw, self.__train_B_raw, train_inlier_A, train_inlier_B,
            train_outlier_A, train_outlier_B, self.__test_A_raw, self.__test_B_raw,
            test_inlier_A, test_inlier_B, test_outlier_A, test_outlier_B,
            self.__train_df, self.__test_df
        )
        
        # Print detailed results
        self._print_detailed_results_split_first()
        
        # Save transformation matrices and deformation model
        self._save_transformation_data()
        
        # Return test set results (the true performance indicator)
        return self.T_svd, self.__final_rmse_test

    def _print_detailed_results_split_first(self):
        """Print detailed performance analysis with split first approach"""
        logger.info("\n=== Enhanced Detailed Performance Analysis (Split First Approach) ===")
        
        # Data processing summary
        original_count = len(self.__positions_A) if self.__positions_A else 0
        train_count = len(self.__train_A_raw) if self.__train_A_raw is not None else 0
        test_count = len(self.__test_A_raw) if self.__test_A_raw is not None else 0
        train_inlier_count = len(self.__train_inlier_A) if self.__train_inlier_A is not None else 0
        test_inlier_count = len(self.__test_inlier_A) if self.__test_inlier_A is not None else 0
        
        logger.info("Data Processing Summary:")
        logger.info(f"  Original points: {original_count}")
        logger.info(f"  Split into - Train: {train_count}, Test: {test_count}")
        logger.info(f"  After RANSAC - Train inliers: {train_inlier_count}, Test inliers: {test_inlier_count}")
        logger.info(f"  Train inlier ratio: {train_inlier_count/train_count*100:.1f}%" if train_count > 0 else "  Train inlier ratio: N/A")
        logger.info(f"  Test inlier ratio: {test_inlier_count/test_count*100:.1f}%" if test_count > 0 else "  Test inlier ratio: N/A")
        
        # Baseline results
        logger.info("Baseline SVD Results (Before RANSAC):")
        logger.info(f"  Training RMSE: {self.__baseline_rmse_train['overall_rmse']*1000:.3f}mm")
        logger.info(f"  Testing RMSE: {self.__baseline_rmse_test['overall_rmse']*1000:.3f}mm")
        logger.info(f"  Generalization gap (baseline): {(self.__baseline_rmse_test['overall_rmse'] - self.__baseline_rmse_train['overall_rmse'])*1000:.3f}mm")
        
        # SVD stage results  
        logger.info("SVD Stage Results (After RANSAC):")
        logger.info(f"  Training RMSE: {self.__ransac_svd_rmse_train['overall_rmse']*1000:.3f}mm")
        if self.__ransac_svd_rmse_test is not None:
            logger.info(f"  Testing RMSE: {self.__ransac_svd_rmse_test['overall_rmse']*1000:.3f}mm")
            logger.info(f"  Generalization gap: {(self.__ransac_svd_rmse_test['overall_rmse'] - self.__ransac_svd_rmse_train['overall_rmse'])*1000:.3f}mm")
        else:
            logger.info(f"  Testing RMSE: N/A (no test inliers)")
        
        # Final results
        logger.info("Final (Deformation Corrected) Results:")
        logger.info(f"  Training RMSE: {self.__final_rmse_train['overall_rmse']*1000:.3f}mm")
        if self.__final_rmse_test is not None:
            logger.info(f"  Testing RMSE: {self.__final_rmse_test['overall_rmse']*1000:.3f}mm")
            logger.info(f"  Generalization gap: {(self.__final_rmse_test['overall_rmse'] - self.__final_rmse_train['overall_rmse'])*1000:.3f}mm")
        else:
            logger.info(f"  Testing RMSE: N/A (no test inliers)")
        
        # Calculate improvements
        baseline_to_svd_train = (self.__baseline_rmse_train['overall_rmse'] - self.__ransac_svd_rmse_train['overall_rmse']) * 1000
        svd_to_final_train = (self.__ransac_svd_rmse_train['overall_rmse'] - self.__final_rmse_train['overall_rmse']) * 1000
        overall_train_improvement = (self.__baseline_rmse_train['overall_rmse'] - self.__final_rmse_train['overall_rmse']) * 1000
        
        logger.info("Algorithm Stage Improvements (Training Set):")
        logger.info(f"  Baseline → SVD: {baseline_to_svd_train:.3f}mm ({baseline_to_svd_train/(self.__baseline_rmse_train['overall_rmse']*1000)*100:.1f}%)")
        logger.info(f"  SVD → Final: {svd_to_final_train:.3f}mm ({svd_to_final_train/(self.__ransac_svd_rmse_train['overall_rmse']*1000)*100:.1f}%)")
        logger.info(f"  Overall: {overall_train_improvement:.3f}mm ({overall_train_improvement/(self.__baseline_rmse_train['overall_rmse']*1000)*100:.1f}%)")
        
        if self.__final_rmse_test is not None and self.__ransac_svd_rmse_test is not None:
            baseline_to_svd_test = (self.__baseline_rmse_test['overall_rmse'] - self.__ransac_svd_rmse_test['overall_rmse']) * 1000
            svd_to_final_test = (self.__ransac_svd_rmse_test['overall_rmse'] - self.__final_rmse_test['overall_rmse']) * 1000
            overall_test_improvement = (self.__baseline_rmse_test['overall_rmse'] - self.__final_rmse_test['overall_rmse']) * 1000
            
            logger.info("Algorithm Stage Improvements (Testing Set):")
            logger.info(f"  Baseline → SVD: {baseline_to_svd_test:.3f}mm ({baseline_to_svd_test/(self.__baseline_rmse_test['overall_rmse']*1000)*100:.1f}%)")
            logger.info(f"  SVD → Final: {svd_to_final_test:.3f}mm ({svd_to_final_test/(self.__ransac_svd_rmse_test['overall_rmse']*1000)*100:.1f}%)")
            logger.info(f"  Overall: {overall_test_improvement:.3f}mm ({overall_test_improvement/(self.__baseline_rmse_test['overall_rmse']*1000)*100:.1f}%)")
        
        # Target achievement
        train_pass = self.__final_rmse_train['overall_rmse'] <= TARGET_RMSE
        if self.__final_rmse_test is not None:
            test_pass = self.__final_rmse_test['overall_rmse'] <= TARGET_RMSE
        else:
            test_pass = False
        
        logger.info("Target Achievement:")
        logger.info(f"  Training target achieved: {'✓' if train_pass else '✗'}")
        logger.info(f"  Testing target achieved: {'✓' if test_pass else '✗' if self.__final_rmse_test is not None else 'N/A'}")
        logger.info(f"  Target RMSE: {TARGET_RMSE*1000:.2f}mm")
        
        # Real-world equivalent for test set (most important)
        if self.__final_rmse_test is not None:
            test_real_world = self.__final_rmse_test['overall_rmse'] * SCALE_FACTOR * 100
            logger.info(f"Real-world Test Performance: {test_real_world:.2f}cm")
        else:
            logger.info(f"Real-world Test Performance: N/A")

    def _save_transformation_data(self):
        """Save transformation matrices and deformation model data"""
        # Save baseline SVD transformation
        if self.T_baseline is not None:
            np.savetxt(os.path.join(result_dir, "T_baseline.txt"), self.T_baseline)
        
        # Save refined SVD transformation
        if self.T_svd is not None:
            np.savetxt(os.path.join(result_dir, "T_svd.txt"), self.T_svd)
        
        # Save deformation model parameters if available
        if self.deformation_model is not None and self.deformation_model.source_points is not None:
            # Save control points
            np.savetxt(os.path.join(result_dir, "deformation_control_points.txt"), 
                      self.deformation_model.source_points)
            
            # Save model metadata
            with open(os.path.join(result_dir, "enhanced_transformation_info.txt"), "w") as f:
                f.write(f"Enhanced Split First Approach - Baseline + RANSAC + SVD + Deformation\n")
                f.write(f"=" * 70 + "\n")
                f.write(f"Method: {self.deformation_model.method}\n")
                f.write(f"Smoothing: {self.deformation_model.smoothing}\n")
                f.write(f"Y-axis constrained: {self.deformation_model.constrain_y}\n")
                f.write(f"Control points: {len(self.deformation_model.source_points)}\n")
                f.write(f"\nData Statistics:\n")
                f.write(f"  Training set size: {len(self.__train_A_raw) if self.__train_A_raw is not None else 'N/A'}\n")
                f.write(f"  Testing set size: {len(self.__test_A_raw) if self.__test_A_raw is not None else 'N/A'}\n")
                f.write(f"  Train inliers: {len(self.__train_inlier_A) if self.__train_inlier_A is not None else 'N/A'}\n")
                f.write(f"  Test inliers: {len(self.__test_inlier_A) if self.__test_inlier_A is not None else 'N/A'}\n")
                f.write(f"  Split ratio: {TRAIN_TEST_SPLIT_RATIO}\n")
                f.write(f"  Random state: {RANDOM_STATE}\n")
                f.write(f"\nPerformance Summary:\n")
                f.write(f"  Baseline RMSE (train): {self.__baseline_rmse_train['overall_rmse']*1000:.3f}mm\n")
                f.write(f"  Baseline RMSE (test): {self.__baseline_rmse_test['overall_rmse']*1000:.3f}mm\n")
                f.write(f"  SVD RMSE (train): {self.__ransac_svd_rmse_train['overall_rmse']*1000:.3f}mm\n")
                f.write(f"  SVD RMSE (test): {self.__ransac_svd_rmse_test['overall_rmse']*1000:.3f}mm" if self.__ransac_svd_rmse_test else "  SVD RMSE (test): N/A\n")
                f.write(f"  Final RMSE (train): {self.__final_rmse_train['overall_rmse']*1000:.3f}mm\n")
                f.write(f"  Final RMSE (test): {self.__final_rmse_test['overall_rmse']*1000:.3f}mm" if self.__final_rmse_test else "  Final RMSE (test): N/A\n")
                
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
    logger.info("Starting Enhanced 1:32 scale model tracker data processing with Split First approach")
    logger.info("Enhanced with: Baseline SVD + Detailed English Visualizations")
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
    
    # Perform enhanced three-stage registration with split first approach
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
            logger.info(f"SUCCESS: Enhanced registration completed with {test_rmse_msg}")
        else:
            logger.info(f"PARTIAL: Enhanced registration completed, but {test_rmse_msg}")
        
        # Save final enhanced summary
        summary_file = os.path.join(output_dir, "enhanced_registration_summary_split_first.txt")
        with open(summary_file, "w") as f:
            f.write("Enhanced Three-Stage Registration Summary with Split First Approach\n")
            f.write("=" * 75 + "\n")
            f.write(f"Enhancements: Baseline SVD + 4 Detailed English Visualization Figures\n")
            f.write(f"Approach: Split data first, then train on train set, evaluate on test set\n")
            f.write(f"Method: Baseline SVD + RANSAC + Refined SVD + Non-rigid Deformation\n")
            f.write(f"Scale Factor: {SCALE_FACTOR}\n")
            f.write(f"Target RMSE: {TARGET_RMSE*1000:.2f}mm\n")
            f.write(f"Test Set RMSE: {final_rmse_test['overall_rmse']*1000:.3f}mm" if final_rmse_test else "Test Set RMSE: N/A (no test inliers)\n")
            f.write(f"Success: {'Yes' if success else 'No'}\n")
            f.write(f"\nGenerated English Visualizations:\n")
            f.write(f"  Figure 1: Train/Test data spatial distribution\n")
            f.write(f"  Figure 2: RANSAC outlier removal and SVD coarse registration\n")
            f.write(f"  Figure 3: Deformation field and final registration results\n")
            f.write(f"  Figure 4: Algorithm stages accuracy comparison\n")
            f.write(f"\nApproach Details:\n")
            f.write(f"  0. Calculate baseline SVD on raw training data\n")
            f.write(f"  1. Split original data into train/test ({TRAIN_TEST_SPLIT_RATIO:.1f}/{1-TRAIN_TEST_SPLIT_RATIO:.1f})\n")
            f.write(f"  2. Apply RANSAC filtering to training set only\n")
            f.write(f"  3. Learn refined SVD transformation on training inliers\n")
            f.write(f"  4. Learn deformation model on training data\n")
            f.write(f"  5. Apply learned models to test set for evaluation\n")
            
            # Add performance details if available
            if hasattr(transformer, '_CoordinateTransformer__baseline_rmse_train'):
                f.write(f"\nDetailed Performance (mm):\n")
                f.write(f"  Baseline (train): {transformer._CoordinateTransformer__baseline_rmse_train['overall_rmse']*1000:.3f}\n")
                f.write(f"  Baseline (test): {transformer._CoordinateTransformer__baseline_rmse_test['overall_rmse']*1000:.3f}\n")
                f.write(f"  SVD refined (train): {transformer._CoordinateTransformer__ransac_svd_rmse_train['overall_rmse']*1000:.3f}\n")
                f.write(f"  SVD refined (test): {transformer._CoordinateTransformer__ransac_svd_rmse_test['overall_rmse']*1000:.3f}" if transformer._CoordinateTransformer__ransac_svd_rmse_test else "  SVD refined (test): N/A\n")
                f.write(f"  Final (train): {transformer._CoordinateTransformer__final_rmse_train['overall_rmse']*1000:.3f}\n")
                f.write(f"  Final (test): {final_rmse_test['overall_rmse']*1000:.3f}" if final_rmse_test else "  Final (test): N/A\n")
                
                if final_rmse_test is not None:
                    overall_improvement = (transformer._CoordinateTransformer__baseline_rmse_test['overall_rmse'] - final_rmse_test['overall_rmse']) * 1000
                    f.write(f"\nOverall Test Improvement: {overall_improvement:.3f}mm ({overall_improvement/(transformer._CoordinateTransformer__baseline_rmse_test['overall_rmse']*1000)*100:.1f}%)\n")
                    f.write(f"Real-world Test Equivalent: {test_real_world:.2f}cm\n")
            
            f.write(f"\nKey Advantages:\n")
            f.write(f"  - Baseline comparison shows algorithm effectiveness\n")
            f.write(f"  - Comprehensive English visualization of each algorithm stage\n")
            f.write(f"  - Realistic evaluation: test set never seen during training\n")
            f.write(f"  - Better assessment of model generalization capability\n")
            f.write(f"  - Reduced risk of overfitting evaluation\n")
    else:
        logger.error("Enhanced registration failed")
    
    logger.info("Enhanced processing complete")

if __name__ == "__main__":
    main()