#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Tracker data processing script with SVD-Filter-SVD-ICP three-stage transformation (Y-axis constrained)
"""
import sys
import os
import datetime
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from sklearn.neighbors import NearestNeighbors

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

# Create output directory with algorithm-specific name
script_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(script_dir)
result_dir = os.path.join(parent_dir, "results")
output_dir = os.path.join(result_dir, f"SVD_Filter_SVD_ICP_{timestamp}_1to32_scale_RMSE_{TARGET_RMSE*1000:.1f}mm")
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
LASER_TRACKER_CSV = os.path.join(parent_dir, "data/corrected_tracker_data_0630_163030.csv")
CORRECTED_LASER_TRACKER_CSV = os.path.join(data_dir, "corrected_laser_tracker.csv")

ROW_X="X"
ROW_Y="Z"

class ICPRegistration:
    """
    ICP (Iterative Closest Point) implementation for fine registration with Y-axis constraint
    """
    def __init__(self, max_iterations=50, tolerance=1e-6, constrain_y=True):
        self.max_iterations = max_iterations
        self.tolerance = tolerance
        self.constrain_y = constrain_y  # 约束Y轴不变
        
    def find_closest_points(self, source, target):
        """Find closest points between source and target point clouds"""
        if self.constrain_y:
            # 只在XZ平面上寻找最近点
            source_xz = source[:, [0, 2]]
            target_xz = target[:, [0, 2]]
            nbrs = NearestNeighbors(n_neighbors=1, algorithm='kd_tree').fit(target_xz)
            distances, indices = nbrs.kneighbors(source_xz)
        else:
            nbrs = NearestNeighbors(n_neighbors=1, algorithm='kd_tree').fit(target)
            distances, indices = nbrs.kneighbors(source)
        return indices.flatten(), distances.flatten()
    
    def compute_transformation_2d(self, source_xz, target_xz):
        """Compute 2D transformation in XZ plane"""
        # 计算质心
        centroid_source = np.mean(source_xz, axis=0)
        centroid_target = np.mean(target_xz, axis=0)
        
        # 中心化
        source_centered = source_xz - centroid_source
        target_centered = target_xz - centroid_target
        
        # 计算协方差矩阵 (2x2)
        H = np.dot(source_centered.T, target_centered)
        
        # SVD
        U, S, Vt = np.linalg.svd(H)
        R_2d = np.dot(Vt.T, U.T)
        
        # 确保旋转矩阵
        if np.linalg.det(R_2d) < 0:
            Vt[-1, :] *= -1
            R_2d = np.dot(Vt.T, U.T)
        
        # 计算平移
        t_2d = centroid_target - np.dot(R_2d, centroid_source)
        
        # 构造3D变换矩阵（Y轴不变）
        T = np.eye(4)
        T[0, 0] = R_2d[0, 0]  # R11
        T[0, 2] = R_2d[0, 1]  # R13
        T[2, 0] = R_2d[1, 0]  # R31
        T[2, 2] = R_2d[1, 1]  # R33
        T[0, 3] = t_2d[0]     # tx
        T[2, 3] = t_2d[1]     # tz
        # Y轴保持不变: T[1,1] = 1, T[1,3] = 0
        
        return T, R_2d, t_2d
    
    def compute_transformation(self, source, target):
        """Compute transformation matrix using SVD with Y-axis constraint"""
        if self.constrain_y:
            # 只使用XZ坐标进行变换计算
            source_xz = source[:, [0, 2]]
            target_xz = target[:, [0, 2]]
            return self.compute_transformation_2d(source_xz, target_xz)
        else:
            # 原始3D变换
            centroid_source = np.mean(source, axis=0)
            centroid_target = np.mean(target, axis=0)
            
            source_centered = source - centroid_source
            target_centered = target - centroid_target
            
            H = np.dot(source_centered.T, target_centered)
            U, S, Vt = np.linalg.svd(H)
            R = np.dot(Vt.T, U.T)
            
            if np.linalg.det(R) < 0:
                Vt[-1, :] *= -1
                R = np.dot(Vt.T, U.T)
            
            t = centroid_target - np.dot(R, centroid_source)
            
            T = np.eye(4)
            T[:3, :3] = R
            T[:3, 3] = t
            
            return T, R, t
    
    def apply_transformation(self, points, T):
        """Apply transformation to points"""
        points_homo = np.hstack([points, np.ones((points.shape[0], 1))])
        transformed = np.dot(T, points_homo.T).T
        return transformed[:, :3]
    
    def register(self, source, target):
        """Perform ICP registration with Y-axis constraint"""
        source = np.array(source)
        target = np.array(target)
        
        # 验证Y坐标是否都为0
        if self.constrain_y:
            if not np.allclose(source[:, 1], 0) or not np.allclose(target[:, 1], 0):
                logger.warning("Y coordinates are not zero, but Y-axis constraint is enabled")
        
        current_source = source.copy()
        total_transformation = np.eye(4)
        
        rmse_history = []
        
        for iteration in range(self.max_iterations):
            # 寻找最近点
            indices, distances = self.find_closest_points(current_source, target)
            
            # 获取对应的目标点
            corresponding_target = target[indices]
            
            # 计算变换
            T, R, t = self.compute_transformation(current_source, corresponding_target)
            
            # 应用变换
            current_source = self.apply_transformation(current_source, T)
            
            # 如果启用Y轴约束，强制Y坐标为0
            if self.constrain_y:
                current_source[:, 1] = 0
            
            # 更新总变换
            total_transformation = np.dot(T, total_transformation)
            
            # 计算RMSE（只考虑XZ平面）
            if self.constrain_y:
                diff_xz = current_source[:, [0, 2]] - corresponding_target[:, [0, 2]]
                rmse = np.sqrt(np.mean(np.sum(diff_xz**2, axis=1)))
            else:
                rmse = np.sqrt(np.mean(np.sum((current_source - corresponding_target)**2, axis=1)))
            
            rmse_history.append(rmse)
            
            logger.info(f"ICP Iteration {iteration + 1}: RMSE = {rmse*1000:.3f}mm")
            
            # 检查收敛
            if iteration > 0 and abs(rmse_history[-2] - rmse_history[-1]) < self.tolerance:
                logger.info(f"ICP converged after {iteration + 1} iterations")
                break
                
            if rmse < TARGET_RMSE:
                logger.info(f"Target RMSE achieved after {iteration + 1} iterations")
                break
        
        return total_transformation, current_source, rmse_history

class CoordinateTransformer:
    """
    Three-stage coordinate transformation: SVD coarse -> Filter -> SVD refined -> ICP fine registration
    """
    def __init__(self, positions_A=None, positions_B=None):
        self.T_svd1 = None  # First SVD transformation
        self.T_svd2 = None  # Second SVD transformation after filtering
        self.T_icp = None   # ICP transformation
        self.T_total = None # Total transformation
        self.__positions_A = positions_A
        self.__positions_B = positions_B
        self.__filtered_indices = None  # Indices of filtered points
        self.icp = ICPRegistration(constrain_y=True)  # 启用Y轴约束

    @staticmethod
    def construct_transformation_matrix(rotation_matrix, translation):
        T = np.eye(4)
        T[:3, :3] = rotation_matrix
        T[:3, 3] = translation
        return T

    def calculate_svd_transformation(self, positions_A, positions_B, stage_name="SVD"):
        """Calculate SVD-based transformation with Y-axis constraint"""
        positions_A = np.array(positions_A)
        positions_B = np.array(positions_B)

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
        T = np.eye(4)
        T[0, 0] = R_2d[0, 0]
        T[0, 2] = R_2d[0, 1]
        T[2, 0] = R_2d[1, 0]
        T[2, 2] = R_2d[1, 1]
        T[0, 3] = translation_2d[0]
        T[2, 3] = translation_2d[1]
        
        logger.info(f"{stage_name} transformation matrix calculated (Y-axis constrained)")
        return T, R_2d, translation_2d

    def apply_transformation(self, positions, T):
        """Apply transformation matrix to positions"""
        positions = np.array(positions)
        positions_homo = np.hstack([positions, np.ones((positions.shape[0], 1))])
        transformed = np.dot(T, positions_homo.T).T
        result = transformed[:, :3]
        # 确保Y坐标保持为0
        result[:, 1] = 0
        return result

    def filter_high_rmse_points(self, positions_A, positions_B, transformed_positions):
        """Filter out point pairs with RMSE higher than overall RMSE"""
        positions_A = np.array(positions_A)
        positions_B = np.array(positions_B)
        transformed_positions = np.array(transformed_positions)
        
        # 计算每个点对的RMSE（只考虑XZ平面）
        individual_errors = np.sqrt((positions_B[:, 0] - transformed_positions[:, 0])**2 + 
                                   (positions_B[:, 2] - transformed_positions[:, 2])**2)
        
        # 计算总体RMSE
        overall_rmse = np.sqrt(np.mean(individual_errors**2))
        
        # 找出RMSE低于或等于总体RMSE的点对
        good_indices = individual_errors <= overall_rmse
        
        filtered_positions_A = positions_A[good_indices]
        filtered_positions_B = positions_B[good_indices]
        
        logger.info(f"Point filtering: {np.sum(good_indices)} out of {len(positions_A)} points retained")
        logger.info(f"Filtering threshold (overall RMSE): {overall_rmse*1000:.3f}mm")
        logger.info(f"Removed {len(positions_A) - np.sum(good_indices)} high-error points")
        
        return filtered_positions_A, filtered_positions_B, good_indices, overall_rmse

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

    def visualize_transformation_stage(self, stage_name, positions_A, positions_B, transformed_positions, rmse_results):
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
        fig = plt.figure(figsize=(16, 12))
        
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

    def three_stage_registration(self):
        """Perform three-stage registration: SVD -> Filter -> SVD -> ICP"""
        if self.__positions_A is None or self.__positions_B is None:
            logger.error("Please provide position data first!")
            return None, None
        
        logger.info("=== Starting Three-Stage Registration (SVD-Filter-SVD-ICP) ===")
        
        # Stage 1: First SVD coarse registration
        logger.info("Stage 1: First SVD Coarse Registration")
        self.T_svd1, _, _ = self.calculate_svd_transformation(
            self.__positions_A, self.__positions_B, "First SVD"
        )
        
        # Apply first SVD transformation
        svd1_transformed = self.apply_transformation(self.__positions_A, self.T_svd1)
        
        # Evaluate first SVD results
        svd1_rmse_results = self.calculate_directional_rmse(self.__positions_B, svd1_transformed)
        svd1_performance = self.evaluate_performance(self.__positions_B, svd1_transformed)
        
        logger.info(f"First SVD Overall RMSE: {svd1_rmse_results['overall_rmse']*1000:.3f}mm")
        
        # Visualize first SVD results
        self.visualize_transformation_stage("Stage 1 - First SVD", 
                                           self.__positions_A, self.__positions_B, svd1_transformed, 
                                           svd1_rmse_results)
        
        # Stage 2: Filter high RMSE points
        logger.info("Stage 2: Point Filtering Based on RMSE")
        filtered_A, filtered_B, good_indices, filter_threshold = self.filter_high_rmse_points(
            self.__positions_A, self.__positions_B, svd1_transformed
        )
        
        self.__filtered_indices = good_indices
        
        # Stage 3: Second SVD on filtered points (using original coordinates)
        logger.info("Stage 3: Second SVD on Filtered Points")
        self.T_svd2, _, _ = self.calculate_svd_transformation(
            filtered_A, filtered_B, "Second SVD"
        )
        
        # Apply second SVD transformation to filtered points
        svd2_transformed = self.apply_transformation(filtered_A, self.T_svd2)
        
        # Evaluate second SVD results
        svd2_rmse_results = self.calculate_directional_rmse(filtered_B, svd2_transformed)
        svd2_performance = self.evaluate_performance(filtered_B, svd2_transformed)
        
        logger.info(f"Second SVD Overall RMSE: {svd2_rmse_results['overall_rmse']*1000:.3f}mm")
        
        # Visualize second SVD results
        self.visualize_transformation_stage("Stage 3 - Second SVD (Filtered)", 
                                           filtered_A, filtered_B, svd2_transformed, 
                                           svd2_rmse_results)
        
        # Stage 4: ICP fine registration on filtered points
        logger.info("Stage 4: ICP Fine Registration on Filtered Points")
        
        logger.info(f"Using {len(svd2_transformed)} filtered points for ICP registration")
        
        # Perform ICP on second SVD-transformed points vs filtered target points
        self.T_icp, icp_transformed, rmse_history = self.icp.register(svd2_transformed, filtered_B)
        
        # Calculate total transformation
        self.T_total = np.dot(self.T_icp, self.T_svd2)
        
        # Apply total transformation to filtered original points
        final_transformed = self.apply_transformation(filtered_A, self.T_total)
        
        # Evaluate final results
        final_rmse_results = self.calculate_directional_rmse(filtered_B, final_transformed)
        final_performance = self.evaluate_performance(filtered_B, final_transformed)
        
        logger.info(f"Final Overall RMSE: {final_rmse_results['overall_rmse']*1000:.3f}mm")
        
        # Visualize final results
        self.visualize_transformation_stage("Stage 4 - Final ICP (Filtered)", 
                                           filtered_A, filtered_B, final_transformed, 
                                           final_rmse_results)
        
        # Print detailed results
        self._print_detailed_results(svd1_performance, svd2_performance, final_performance, filter_threshold)
        
        # Save transformation matrices and filtered indices
        self._save_transformation_matrices()
        
        return self.T_total, final_rmse_results

    def _print_detailed_results(self, svd1_performance, svd2_performance, final_performance, filter_threshold):
        """Print detailed performance analysis"""
        logger.info("\n=== Detailed Performance Analysis ===")
        
        logger.info("Stage 1 - First SVD Results:")
        svd1_rmse = svd1_performance['rmse_results']
        logger.info(f"  Lateral RMSE: {svd1_rmse['lateral_rmse']*1000:.3f}mm")
        logger.info(f"  Longitudinal RMSE: {svd1_rmse['longitudinal_rmse']*1000:.3f}mm")
        logger.info(f"  Overall RMSE: {svd1_rmse['overall_rmse']*1000:.3f}mm")
        
        logger.info("Stage 2 - Point Filtering:")
        logger.info(f"  Filtering threshold: {filter_threshold*1000:.3f}mm")
        logger.info(f"  Points retained: {np.sum(self.__filtered_indices)} out of {len(self.__filtered_indices)}")
        
        logger.info("Stage 3 - Second SVD Results (Filtered):")
        svd2_rmse = svd2_performance['rmse_results']
        logger.info(f"  Lateral RMSE: {svd2_rmse['lateral_rmse']*1000:.3f}mm")
        logger.info(f"  Longitudinal RMSE: {svd2_rmse['longitudinal_rmse']*1000:.3f}mm")
        logger.info(f"  Overall RMSE: {svd2_rmse['overall_rmse']*1000:.3f}mm")
        
        logger.info("Stage 4 - Final (SVD-Filter-SVD-ICP) Results:")
        final_rmse = final_performance['rmse_results']
        logger.info(f"  Lateral RMSE: {final_rmse['lateral_rmse']*1000:.3f}mm")
        logger.info(f"  Longitudinal RMSE: {final_rmse['longitudinal_rmse']*1000:.3f}mm")
        logger.info(f"  Overall RMSE: {final_rmse['overall_rmse']*1000:.3f}mm")
        
        logger.info("Target Achievement:")
        pass_criteria = final_performance['pass_criteria']
        logger.info(f"  Overall target achieved: {'✓' if pass_criteria['overall_pass'] else '✗'}")
        logger.info(f"  Target RMSE: {TARGET_RMSE*1000:.2f}mm")
        
        # Real-world equivalent
        rw_eq = final_performance['real_world_equivalent']
        logger.info("Real-world Equivalent Performance:")
        logger.info(f"  Overall equivalent: {rw_eq['overall_real_world_eq']*100:.2f}cm")

    def _save_transformation_matrices(self):
        """Save transformation matrices and filtering information to files"""
        # Save first SVD transformation
        if self.T_svd1 is not None:
            np.savetxt(os.path.join(result_dir, "T_svd1.txt"), self.T_svd1)
        
        # Save second SVD transformation
        if self.T_svd2 is not None:
            np.savetxt(os.path.join(result_dir, "T_svd2.txt"), self.T_svd2)
        
        # Save ICP transformation
        if self.T_icp is not None:
            np.savetxt(os.path.join(result_dir, "T_icp.txt"), self.T_icp)
        
        # Save total transformation
        if self.T_total is not None:
            np.savetxt(os.path.join(result_dir, "T_total.txt"), self.T_total)
        
        # Save filtered indices
        if self.__filtered_indices is not None:
            np.savetxt(os.path.join(result_dir, "filtered_indices.txt"), self.__filtered_indices, fmt='%d')
            
        logger.info("Transformation matrices and filtering information saved to results directory")

def process_laser_tracker_data(input_file, output_file):
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
    logger.info("Starting 1:32 scale model tracker data processing with SVD-Filter-SVD-ICP")
    logger.info(f"Scale factor: {SCALE_FACTOR}")
    logger.info(f"Target RMSE: {TARGET_RMSE*1000:.2f}mm")
    
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
    
    # Perform three-stage registration
    transformer = CoordinateTransformer(positions_A, positions_B)
    T_total, final_rmse_results = transformer.three_stage_registration()
    
    if T_total is not None and final_rmse_results is not None:
        success = final_rmse_results['overall_rmse'] <= TARGET_RMSE
        if success:
            logger.info(f"SUCCESS: Registration completed with RMSE {final_rmse_results['overall_rmse']*1000:.3f}mm")
        else:
            logger.info(f"PARTIAL: Registration completed, but RMSE {final_rmse_results['overall_rmse']*1000:.3f}mm exceeds target {TARGET_RMSE*1000:.2f}mm")
        
        # Save final summary
        summary_file = os.path.join(output_dir, "registration_summary.txt")
        with open(summary_file, "w") as f:
            f.write("Three-Stage Registration Summary (SVD-Filter-SVD-ICP)\n")
            f.write("=" * 60 + "\n")
            f.write(f"Scale Factor: {SCALE_FACTOR}\n")
            f.write(f"Target RMSE: {TARGET_RMSE*1000:.2f}mm\n")
            f.write(f"Final RMSE: {final_rmse_results['overall_rmse']*1000:.3f}mm\n")
            f.write(f"Success: {'Yes' if success else 'No'}\n")
            f.write(f"Original points: {len(positions_A)}\n")
            if hasattr(transformer, '_CoordinateTransformer__filtered_indices') and transformer._CoordinateTransformer__filtered_indices is not None:
                f.write(f"Filtered points: {np.sum(transformer._CoordinateTransformer__filtered_indices)}\n")
            f.write(f"Real-world equivalent: {final_rmse_results['overall_rmse']*SCALE_FACTOR*100:.2f}cm\n")
            f.write("\nAlgorithm: First SVD -> Point filtering by RMSE -> Second SVD on filtered points -> ICP fine registration\n")
    else:
        logger.error("Registration failed")
    
    logger.info("Processing complete")

if __name__ == "__main__":
    main()