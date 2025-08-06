#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Tracker data processing script with RANSAC + SVD + Residual Outlier Removal + Non-rigid Deformation Learning (Y-axis constrained)
Enhanced with simple train/test split and comprehensive evaluation
ENHANCED RESIDUAL VECTOR VISUALIZATION + SVD Residual Outlier Removal
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
from matplotlib.patches import FancyArrowPatch
from mpl_toolkits.mplot3d import proj3d
import matplotlib.patches as patches

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
RANSAC_INLIER_THRESHOLD = 0.04  # 内点阈值 30mm (0.030m)

# SVD残差向量过滤参数
SVD_RESIDUAL_THRESHOLD = 0.08  # 8mm，用于过滤SVD变换后残差过大的点对

# 训练/测试集划分参数
TRAIN_TEST_RATIO = 0.8  # 80%用于训练，20%用于测试

# Create output directory
script_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(script_dir)
result_dir = os.path.join(parent_dir, "results")
parent_dir = os.path.dirname(parent_dir)
LASER_TRACKER_CSV = os.path.join(parent_dir, "data/combined_corrected_tracker_data_2.csv")
csv_filename = os.path.basename(LASER_TRACKER_CSV)
output_dir = os.path.join(result_dir, f"Enhanced_RANSAC_SVD_ResidualFilter_Deformation_{timestamp}_1to32_scale_RMSE_{TARGET_RMSE*1000:.1f}mm_{csv_filename}")
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

class SVDResidualFilter:
    """
    SVD变换后的残差向量过滤器
    """
    def __init__(self, residual_threshold=SVD_RESIDUAL_THRESHOLD):
        self.residual_threshold = residual_threshold
        self.filtered_indices = None
        self.removed_indices = None
        
    def filter_svd_residuals(self, source_points, target_points, svd_transformation):
        """
        根据SVD变换后的残差向量大小过滤点对
        
        Args:
            source_points: 源点 (N, 3)
            target_points: 目标点 (N, 3)
            svd_transformation: SVD变换矩阵 (4, 4)
            
        Returns:
            filtered_source: 过滤后的源点
            filtered_target: 过滤后的目标点
            removed_source: 被移除的源点
            removed_target: 被移除的目标点
            residual_vectors: 残差向量
            residual_magnitudes: 残差向量大小
        """
        logger.info("=== Starting SVD Residual Filtering ===")
        
        source_points = np.array(source_points)
        target_points = np.array(target_points)
        
        # 应用SVD变换
        transformed_source = self._apply_transformation(source_points, svd_transformation)
        
        # 计算残差向量
        residual_vectors = target_points - transformed_source
        
        # 计算残差向量大小（只考虑XZ平面）
        residual_magnitudes = np.sqrt(residual_vectors[:, 0]**2 + residual_vectors[:, 2]**2)
        
        # 识别需要保留的点（残差小于阈值）
        keep_mask = residual_magnitudes <= self.residual_threshold
        remove_mask = ~keep_mask
        
        # 分离保留和移除的点
        filtered_source = source_points[keep_mask]
        filtered_target = target_points[keep_mask]
        removed_source = source_points[remove_mask]
        removed_target = target_points[remove_mask]
        
        # 保存索引
        self.filtered_indices = np.where(keep_mask)[0]
        self.removed_indices = np.where(remove_mask)[0]
        
        logger.info(f"SVD Residual Filtering Results:")
        logger.info(f"  - Original points: {len(source_points)}")
        logger.info(f"  - Points kept: {len(filtered_source)} ({len(filtered_source)/len(source_points)*100:.1f}%)")
        logger.info(f"  - Points removed: {len(removed_source)} ({len(removed_source)/len(source_points)*100:.1f}%)")
        logger.info(f"  - Residual threshold: {self.residual_threshold*1000:.1f}mm")
        logger.info(f"  - Max residual (kept): {np.max(residual_magnitudes[keep_mask])*1000:.2f}mm" if np.any(keep_mask) else "  - No points kept")
        logger.info(f"  - Min residual (removed): {np.min(residual_magnitudes[remove_mask])*1000:.2f}mm" if np.any(remove_mask) else "  - No points removed")
        
        return (filtered_source, filtered_target, removed_source, removed_target, 
                residual_vectors, residual_magnitudes)
    
    def _apply_transformation(self, points, T):
        """应用变换矩阵到点集"""
        points = np.array(points)
        points_homo = np.hstack([points, np.ones((points.shape[0], 1))])
        transformed = np.dot(T, points_homo.T).T
        result = transformed[:, :3]
        result[:, 1] = 0  # 确保Y坐标保持为0
        return result
    
    def visualize_residual_filtering(self, original_source, original_target, 
                                   filtered_source, filtered_target,
                                   removed_source, removed_target,
                                   residual_vectors, residual_magnitudes,
                                   stage_name="SVD Residual Filtering"):
        """
        可视化残差向量过滤结果
        """
        logger.info("Creating SVD residual filtering visualization...")
        
        fig = plt.figure(figsize=(24, 18))
        
        # 1. 原始数据 3D视图
        ax1 = fig.add_subplot(231, projection='3d')
        ax1.scatter(original_source[:, 0], original_source[:, 2], original_source[:, 1],
                   c='blue', marker='o', s=30, alpha=0.7, label='Original Source')
        ax1.scatter(original_target[:, 0], original_target[:, 2], original_target[:, 1],
                   c='red', marker='^', s=30, alpha=0.7, label='Original Target')
        ax1.set_xlabel('X [m]')
        ax1.set_ylabel('Z [m]')
        ax1.set_zlabel('Y [m]')
        ax1.set_title(f'Original Data\n({len(original_source)} points)')
        ax1.legend()
        ax1.set_zlim(-0.1, 0.1)
        
        # 2. 保留的点 3D视图
        ax2 = fig.add_subplot(232, projection='3d')
        if len(filtered_source) > 0:
            ax2.scatter(filtered_source[:, 0], filtered_source[:, 2], filtered_source[:, 1],
                       c='green', marker='o', s=30, alpha=0.7, label='Kept Source')
            ax2.scatter(filtered_target[:, 0], filtered_target[:, 2], filtered_target[:, 1],
                       c='darkgreen', marker='^', s=30, alpha=0.7, label='Kept Target')
        ax2.set_xlabel('X [m]')
        ax2.set_ylabel('Z [m]')
        ax2.set_zlabel('Y [m]')
        ax2.set_title(f'Kept Points\n({len(filtered_source)} points)')
        ax2.legend()
        ax2.set_zlim(-0.1, 0.1)
        
        # 3. 移除的点 3D视图
        ax3 = fig.add_subplot(233, projection='3d')
        if len(removed_source) > 0:
            ax3.scatter(removed_source[:, 0], removed_source[:, 2], removed_source[:, 1],
                       c='orange', marker='o', s=30, alpha=0.7, label='Removed Source')
            ax3.scatter(removed_target[:, 0], removed_target[:, 2], removed_target[:, 1],
                       c='red', marker='^', s=30, alpha=0.7, label='Removed Target')
        ax3.set_xlabel('X [m]')
        ax3.set_ylabel('Z [m]')
        ax3.set_zlabel('Y [m]')
        ax3.set_title(f'Removed Points\n({len(removed_source)} points)')
        ax3.legend()
        ax3.set_zlim(-0.1, 0.1)
        
        # 4. 原始数据 XZ平面视图
        ax4 = fig.add_subplot(234)
        ax4.scatter(original_source[:, 0], original_source[:, 2], 
                   c='blue', marker='o', s=30, alpha=0.7, label='Original Source')
        ax4.scatter(original_target[:, 0], original_target[:, 2], 
                   c='red', marker='^', s=30, alpha=0.7, label='Original Target')
        ax4.set_xlabel('X (Lateral) [m]')
        ax4.set_ylabel('Z (Longitudinal) [m]')
        ax4.set_title(f'Original Data - XZ Plane\n({len(original_source)} points)')
        ax4.legend()
        ax4.grid(True, alpha=0.3)
        ax4.axis('equal')
        
        # 5. 过滤结果 XZ平面视图
        ax5 = fig.add_subplot(235)
        if len(filtered_source) > 0:
            ax5.scatter(filtered_source[:, 0], filtered_source[:, 2], 
                       c='green', marker='o', s=30, alpha=0.7, label='Kept Source')
            ax5.scatter(filtered_target[:, 0], filtered_target[:, 2], 
                       c='darkgreen', marker='^', s=30, alpha=0.7, label='Kept Target')
        
        if len(removed_source) > 0:
            ax5.scatter(removed_source[:, 0], removed_source[:, 2], 
                       c='orange', marker='x', s=40, alpha=0.9, label='Removed Source')
            ax5.scatter(removed_target[:, 0], removed_target[:, 2], 
                       c='red', marker='x', s=40, alpha=0.9, label='Removed Target')
        
        ax5.set_xlabel('X (Lateral) [m]')
        ax5.set_ylabel('Z (Longitudinal) [m]')
        ax5.set_title(f'Filtering Results - XZ Plane\nKept: {len(filtered_source)}, Removed: {len(removed_source)}')
        ax5.legend()
        ax5.grid(True, alpha=0.3)
        ax5.axis('equal')
        
        # 6. 残差向量大小分布
        ax6 = fig.add_subplot(236)
        
        # 绘制所有残差的直方图
        ax6.hist(residual_magnitudes * 1000, bins=30, alpha=0.6, color='lightblue', 
                edgecolor='black', label='All Residuals')
        
        # 标记阈值线
        ax6.axvline(x=self.residual_threshold * 1000, color='red', linestyle='--', 
                   linewidth=2, label=f'Threshold: {self.residual_threshold*1000:.1f}mm')
        
        # 分别绘制保留和移除的残差
        if len(self.filtered_indices) > 0:
            kept_residuals = residual_magnitudes[self.filtered_indices] * 1000
            ax6.hist(kept_residuals, bins=20, alpha=0.8, color='green', 
                    label=f'Kept ({len(kept_residuals)})')
        
        if len(self.removed_indices) > 0:
            removed_residuals = residual_magnitudes[self.removed_indices] * 1000
            ax6.hist(removed_residuals, bins=20, alpha=0.8, color='red', 
                    label=f'Removed ({len(removed_residuals)})')
        
        ax6.set_xlabel('Residual Vector Magnitude [mm]')
        ax6.set_ylabel('Frequency')
        ax6.set_title(f'{stage_name} - Residual Distribution')
        ax6.legend()
        ax6.grid(True, alpha=0.3)
        
        plt.tight_layout()
        
        filename = f'svd_residual_filtering_{stage_name.lower().replace(" ", "_")}.png'
        plt.savefig(os.path.join(img_dir, filename), dpi=300, bbox_inches='tight')
        plt.close()
        
        # 创建详细的残差分析图
        self._create_detailed_residual_analysis(original_source, original_target, 
                                               residual_vectors, residual_magnitudes, stage_name)
        
        logger.info(f"SVD residual filtering visualization saved: {filename}")
    
    def _create_detailed_residual_analysis(self, original_source, original_target,
                                         residual_vectors, residual_magnitudes, stage_name):
        """
        创建详细的残差分析图
        """
        fig = plt.figure(figsize=(20, 15))
        
        # 1. 残差向量场（XZ平面）
        ax1 = fig.add_subplot(231)
        
        # 根据残差大小选择显示的向量
        max_display = 100
        if len(residual_vectors) > max_display:
            # 选择一些有代表性的点：包括最大残差和随机采样
            large_residual_indices = np.argsort(residual_magnitudes)[-max_display//2:]
            random_indices = np.random.choice(len(residual_vectors), max_display//2, replace=False)
            display_indices = np.unique(np.concatenate([large_residual_indices, random_indices]))
        else:
            display_indices = np.arange(len(residual_vectors))
        
        # 绘制点
        ax1.scatter(original_source[:, 0], original_source[:, 2], 
                   c='blue', s=20, alpha=0.6, label='Source Points')
        ax1.scatter(original_target[:, 0], original_target[:, 2], 
                   c='red', s=20, alpha=0.6, label='Target Points')
        
        # 绘制残差向量（按大小着色）
        for idx in display_indices:
            color = 'red' if residual_magnitudes[idx] > self.residual_threshold else 'green'
            alpha = 0.8 if residual_magnitudes[idx] > self.residual_threshold else 0.5
            
            # 变换后的源点
            transformed_source_point = original_target[idx] - residual_vectors[idx]
            
            ax1.arrow(transformed_source_point[0], transformed_source_point[2],
                     residual_vectors[idx, 0], residual_vectors[idx, 2],
                     head_width=0.001, head_length=0.002, 
                     fc=color, ec=color, alpha=alpha, linewidth=1)
        
        ax1.set_xlabel('X [m]')
        ax1.set_ylabel('Z [m]')
        ax1.set_title(f'{stage_name} - Residual Vector Field\n(Showing {len(display_indices)} vectors)')
        ax1.legend()
        ax1.grid(True, alpha=0.3)
        ax1.axis('equal')
        
        # 2. 残差大小热力图
        ax2 = fig.add_subplot(232)
        if len(original_source) > 10:
            try:
                x_min, x_max = np.min(original_source[:, 0]), np.max(original_source[:, 0])
                z_min, z_max = np.min(original_source[:, 2]), np.max(original_source[:, 2])
                
                x_range, z_range = x_max - x_min, z_max - z_min
                x_min -= 0.1 * x_range
                x_max += 0.1 * x_range
                z_min -= 0.1 * z_range
                z_max += 0.1 * z_range
                
                xi = np.linspace(x_min, x_max, 50)
                zi = np.linspace(z_min, z_max, 50)
                Xi, Zi = np.meshgrid(xi, zi)
                
                # 插值残差大小
                residual_interp = griddata(
                    (original_source[:, 0], original_source[:, 2]), 
                    residual_magnitudes * 1000,  # 转换为mm
                    (Xi, Zi), method='linear', fill_value=0
                )
                
                im = ax2.contourf(Xi, Zi, residual_interp, levels=20, cmap='viridis')
                plt.colorbar(im, ax=ax2, label='Residual Magnitude [mm]')
                
                # 添加阈值等高线
                ax2.contour(Xi, Zi, residual_interp, levels=[self.residual_threshold*1000], 
                           colors='red', linestyles='--', linewidths=2)
                
                ax2.scatter(original_source[:, 0], original_source[:, 2], 
                           c='white', s=10, alpha=0.8, marker='.')
                
            except Exception as e:
                ax2.text(0.5, 0.5, f'Could not create heatmap:\n{str(e)}', 
                        transform=ax2.transAxes, ha='center', va='center')
        
        ax2.set_xlabel('X [m]')
        ax2.set_ylabel('Z [m]')
        ax2.set_title(f'{stage_name} - Residual Magnitude Heatmap')
        
        # 3. 残差统计分析
        ax3 = fig.add_subplot(233)
        ax3.axis('off')
        
        # 计算统计信息
        mean_residual = np.mean(residual_magnitudes)
        std_residual = np.std(residual_magnitudes)
        median_residual = np.median(residual_magnitudes)
        max_residual = np.max(residual_magnitudes)
        min_residual = np.min(residual_magnitudes)
        
        kept_count = len(self.filtered_indices) if self.filtered_indices is not None else 0
        removed_count = len(self.removed_indices) if self.removed_indices is not None else 0
        
        stats_text = f"""
{stage_name} - Residual Statistics

Total Points: {len(residual_magnitudes)}
Points Kept: {kept_count} ({kept_count/len(residual_magnitudes)*100:.1f}%)
Points Removed: {removed_count} ({removed_count/len(residual_magnitudes)*100:.1f}%)

Residual Magnitude Statistics (mm):
  Mean: {mean_residual*1000:.3f}
  Median: {median_residual*1000:.3f}
  Std Dev: {std_residual*1000:.3f}
  Minimum: {min_residual*1000:.3f}
  Maximum: {max_residual*1000:.3f}

Filtering Threshold: {self.residual_threshold*1000:.1f} mm

Performance:
  Points above threshold: {removed_count}
  Points below threshold: {kept_count}
  Filter efficiency: {kept_count/len(residual_magnitudes)*100:.1f}%
        """
        
        ax3.text(0.05, 0.95, stats_text, transform=ax3.transAxes, fontsize=10,
                verticalalignment='top', fontfamily='monospace',
                bbox=dict(boxstyle='round', facecolor='lightgray', alpha=0.8))
        
        # 4. 空间分布分析 - X方向
        ax4 = fig.add_subplot(234)
        scatter = ax4.scatter(original_source[:, 0], residual_magnitudes * 1000, 
                             c=residual_magnitudes * 1000, cmap='viridis', 
                             s=30, alpha=0.7)
        ax4.axhline(y=self.residual_threshold * 1000, color='red', linestyle='--', 
                   linewidth=2, label=f'Threshold: {self.residual_threshold*1000:.1f}mm')
        plt.colorbar(scatter, ax=ax4, label='Residual [mm]')
        ax4.set_xlabel('X Position [m]')
        ax4.set_ylabel('Residual Magnitude [mm]')
        ax4.set_title('Residual vs X Position')
        ax4.legend()
        ax4.grid(True, alpha=0.3)
        
        # 5. 空间分布分析 - Z方向
        ax5 = fig.add_subplot(235)
        scatter = ax5.scatter(original_source[:, 2], residual_magnitudes * 1000, 
                             c=residual_magnitudes * 1000, cmap='viridis', 
                             s=30, alpha=0.7)
        ax5.axhline(y=self.residual_threshold * 1000, color='red', linestyle='--', 
                   linewidth=2, label=f'Threshold: {self.residual_threshold*1000:.1f}mm')
        plt.colorbar(scatter, ax=ax5, label='Residual [mm]')
        ax5.set_xlabel('Z Position [m]')
        ax5.set_ylabel('Residual Magnitude [mm]')
        ax5.set_title('Residual vs Z Position')
        ax5.legend()
        ax5.grid(True, alpha=0.3)
        
        # 6. 残差分量分析
        ax6 = fig.add_subplot(236)
        ax6.scatter(residual_vectors[:, 0] * 1000, residual_vectors[:, 2] * 1000, 
                   c=residual_magnitudes * 1000, cmap='viridis', s=30, alpha=0.7)
        
        # 添加阈值圆圈
        circle = plt.Circle((0, 0), self.residual_threshold * 1000, fill=False, 
                           color='red', linestyle='--', linewidth=2, 
                           label=f'Threshold: {self.residual_threshold*1000:.1f}mm')
        ax6.add_patch(circle)
        
        ax6.axhline(y=0, color='k', linestyle='-', alpha=0.3)
        ax6.axvline(x=0, color='k', linestyle='-', alpha=0.3)
        
        ax6.set_xlabel('X-direction Residual [mm]')
        ax6.set_ylabel('Z-direction Residual [mm]')
        ax6.set_title('Residual Components Analysis')
        ax6.legend()
        ax6.grid(True, alpha=0.3)
        ax6.axis('equal')
        
        plt.tight_layout()
        
        filename = f'svd_residual_detailed_analysis_{stage_name.lower().replace(" ", "_")}.png'
        plt.savefig(os.path.join(img_dir, filename), dpi=300, bbox_inches='tight')
        plt.close()
        
        logger.info(f"Detailed residual analysis saved: {filename}")
    
    def save_filtering_results(self, original_source, original_target, 
                             filtered_source, filtered_target,
                             removed_source, removed_target, stage_name=""):
        """
        保存过滤结果数据
        """
        # 保存过滤后的数据
        if len(filtered_source) > 0:
            filtered_data = np.hstack([filtered_source, filtered_target])
            np.savetxt(os.path.join(result_dir, f"svd_filtered_data_{stage_name.lower().replace(' ', '_')}.txt"), 
                      filtered_data, 
                      header="filtered_source_x filtered_source_y filtered_source_z filtered_target_x filtered_target_y filtered_target_z")
        
        # 保存被移除的数据
        if len(removed_source) > 0:
            removed_data = np.hstack([removed_source, removed_target])
            np.savetxt(os.path.join(result_dir, f"svd_removed_data_{stage_name.lower().replace(' ', '_')}.txt"), 
                      removed_data, 
                      header="removed_source_x removed_source_y removed_source_z removed_target_x removed_target_y removed_target_z")
        
        # 保存索引
        if self.filtered_indices is not None:
            np.savetxt(os.path.join(result_dir, f"svd_filtered_indices_{stage_name.lower().replace(' ', '_')}.txt"), 
                      self.filtered_indices, fmt='%d')
        
        if self.removed_indices is not None:
            np.savetxt(os.path.join(result_dir, f"svd_removed_indices_{stage_name.lower().replace(' ', '_')}.txt"), 
                      self.removed_indices, fmt='%d')
        
        # 保存过滤摘要
        with open(os.path.join(result_dir, f"svd_filtering_summary_{stage_name.lower().replace(' ', '_')}.txt"), "w") as f:
            f.write(f"SVD Residual Filtering Summary - {stage_name}\n")
            f.write("=" * 50 + "\n")
            f.write(f"Residual threshold: {self.residual_threshold*1000:.2f} mm\n")
            f.write(f"Original points: {len(original_source)}\n")
            f.write(f"Points kept: {len(filtered_source)} ({len(filtered_source)/len(original_source)*100:.1f}%)\n")
            f.write(f"Points removed: {len(removed_source)} ({len(removed_source)/len(original_source)*100:.1f}%)\n")
        
        logger.info(f"SVD filtering results saved for {stage_name}")

class ResidualVectorVisualizer:
    """
    专门用于残差向量可视化的类
    """
    def __init__(self, img_dir):
        self.img_dir = img_dir
        
    def visualize_residual_vectors_comprehensive(self, source_points, target_points, transformed_points, 
                                               residual_vectors, stage_name="", max_vectors_display=200):
        """
        全面的残差向量可视化
        """
        logger.info(f"Creating comprehensive residual vector visualization for {stage_name}...")
        
        source_points = np.array(source_points)
        target_points = np.array(target_points)
        transformed_points = np.array(transformed_points)
        residual_vectors = np.array(residual_vectors)
        
        # 计算残差向量统计信息
        residual_magnitudes = np.linalg.norm(residual_vectors[:, [0, 2]], axis=1)  # 只考虑XZ平面
        mean_magnitude = np.mean(residual_magnitudes)
        std_magnitude = np.std(residual_magnitudes)
        max_magnitude = np.max(residual_magnitudes)
        
        # 创建大型综合图表
        fig = plt.figure(figsize=(24, 18))
        
        # 1. 主要的XZ平面视图 - 残差向量场
        ax1 = fig.add_subplot(331)
        
        # 为了避免图表过于拥挤，可能需要抽样显示
        n_points = len(source_points)
        if n_points > max_vectors_display:
            # 使用uniform sampling
            indices = np.linspace(0, n_points-1, max_vectors_display, dtype=int)
            display_transformed = transformed_points[indices]
            display_residuals = residual_vectors[indices]
            display_targets = target_points[indices]
        else:
            display_transformed = transformed_points
            display_residuals = residual_vectors
            display_targets = target_points
            indices = np.arange(n_points)
        
        # 绘制点
        ax1.scatter(display_transformed[:, 0], display_transformed[:, 2], 
                   c='blue', s=40, alpha=0.7, label='Transformed Points', marker='o')
        ax1.scatter(display_targets[:, 0], display_targets[:, 2], 
                   c='red', s=40, alpha=0.7, label='Target Points', marker='^')
        
        # 绘制残差向量
        vector_scale = 100  # 放大残差向量以便可视化
        for i in range(len(display_transformed)):
            x_start, z_start = display_transformed[i, 0], display_transformed[i, 2]
            dx, dz = display_residuals[i, 0] * vector_scale, display_residuals[i, 2] * vector_scale
            
            # 根据向量大小设置颜色
            magnitude = np.linalg.norm([display_residuals[i, 0], display_residuals[i, 2]])
            color = plt.cm.viridis(magnitude / max_magnitude) if max_magnitude > 0 else 'blue'
            
            ax1.arrow(x_start, z_start, dx, dz, 
                     head_width=0.003, head_length=0.005, fc=color, ec=color, alpha=0.8)
        
        ax1.set_xlabel('X (Lateral) [m]')
        ax1.set_ylabel('Z (Longitudinal) [m]')
        ax1.set_title(f'{stage_name} - Residual Vector Field\n(Vectors scaled ×{vector_scale})')
        ax1.legend()
        ax1.grid(True, alpha=0.3)
        ax1.axis('equal')
        
        # 2. 残差向量大小分布
        ax2 = fig.add_subplot(332)
        ax2.hist(residual_magnitudes * 1000, bins=30, color='lightblue', edgecolor='black', alpha=0.7)
        ax2.axvline(x=mean_magnitude * 1000, color='red', linestyle='--', 
                   label=f'Mean: {mean_magnitude*1000:.2f}mm')
        ax2.axvline(x=(mean_magnitude + std_magnitude) * 1000, color='orange', linestyle='--', 
                   label=f'Mean+Std: {(mean_magnitude + std_magnitude)*1000:.2f}mm')
        ax2.axvline(x=TARGET_RMSE * 1000, color='green', linestyle='--', 
                   label=f'Target: {TARGET_RMSE*1000:.1f}mm')
        ax2.set_xlabel('Residual Vector Magnitude [mm]')
        ax2.set_ylabel('Frequency')
        ax2.set_title(f'{stage_name} - Residual Magnitude Distribution')
        ax2.legend()
        ax2.grid(True, alpha=0.3)
        
        # 3. 残差向量方向分析
        ax3 = fig.add_subplot(333)
        residual_angles = np.arctan2(residual_vectors[:, 2], residual_vectors[:, 0]) * 180 / np.pi
        ax3.hist(residual_angles, bins=36, color='lightcoral', edgecolor='black', alpha=0.7)
        ax3.set_xlabel('Residual Vector Direction [degrees]')
        ax3.set_ylabel('Frequency')
        ax3.set_title(f'{stage_name} - Residual Direction Distribution')
        ax3.grid(True, alpha=0.3)
        
        # 4. 残差向量热力图 (X方向)
        ax4 = fig.add_subplot(334)
        if len(transformed_points) > 10:  # 确保有足够的点进行插值
            try:
                # 创建网格
                x_min, x_max = np.min(transformed_points[:, 0]), np.max(transformed_points[:, 0])
                z_min, z_max = np.min(transformed_points[:, 2]), np.max(transformed_points[:, 2])
                
                # 扩展边界
                x_range = x_max - x_min
                z_range = z_max - z_min
                x_min -= 0.1 * x_range
                x_max += 0.1 * x_range
                z_min -= 0.1 * z_range
                z_max += 0.1 * z_range
                
                xi = np.linspace(x_min, x_max, 50)
                zi = np.linspace(z_min, z_max, 50)
                Xi, Zi = np.meshgrid(xi, zi)
                
                # 插值X方向残差
                residual_x_interp = griddata(
                    (transformed_points[:, 0], transformed_points[:, 2]), 
                    residual_vectors[:, 0] * 1000,  # 转换为mm
                    (Xi, Zi), method='linear', fill_value=0
                )
                
                im = ax4.contourf(Xi, Zi, residual_x_interp, levels=20, cmap='RdBu_r')
                plt.colorbar(im, ax=ax4)
                ax4.scatter(transformed_points[:, 0], transformed_points[:, 2], 
                           c='black', s=10, alpha=0.6, marker='.')
                
            except Exception as e:
                logger.warning(f"Could not create residual heatmap: {e}")
                ax4.text(0.5, 0.5, 'Insufficient data\nfor interpolation', 
                        transform=ax4.transAxes, ha='center', va='center')
        
        ax4.set_xlabel('X [m]')
        ax4.set_ylabel('Z [m]')
        ax4.set_title(f'{stage_name} - X-direction Residual Heatmap [mm]')
        
        # 5. 残差向量热力图 (Z方向)
        ax5 = fig.add_subplot(335)
        if len(transformed_points) > 10:
            try:
                # 插值Z方向残差
                residual_z_interp = griddata(
                    (transformed_points[:, 0], transformed_points[:, 2]), 
                    residual_vectors[:, 2] * 1000,  # 转换为mm
                    (Xi, Zi), method='linear', fill_value=0
                )
                
                im = ax5.contourf(Xi, Zi, residual_z_interp, levels=20, cmap='RdBu_r')
                plt.colorbar(im, ax=ax5)
                ax5.scatter(transformed_points[:, 0], transformed_points[:, 2], 
                           c='black', s=10, alpha=0.6, marker='.')
                
            except Exception as e:
                logger.warning(f"Could not create residual heatmap: {e}")
                ax5.text(0.5, 0.5, 'Insufficient data\nfor interpolation', 
                        transform=ax5.transAxes, ha='center', va='center')
        
        ax5.set_xlabel('X [m]')
        ax5.set_ylabel('Z [m]')
        ax5.set_title(f'{stage_name} - Z-direction Residual Heatmap [mm]')
        
        # 6. 残差向量大小热力图
        ax6 = fig.add_subplot(336)
        if len(transformed_points) > 10:
            try:
                # 插值残差大小
                residual_mag_interp = griddata(
                    (transformed_points[:, 0], transformed_points[:, 2]), 
                    residual_magnitudes * 1000,  # 转换为mm
                    (Xi, Zi), method='linear', fill_value=0
                )
                
                im = ax6.contourf(Xi, Zi, residual_mag_interp, levels=20, cmap='viridis')
                plt.colorbar(im, ax=ax6)
                ax6.scatter(transformed_points[:, 0], transformed_points[:, 2], 
                           c='white', s=10, alpha=0.8, marker='.')
                
            except Exception as e:
                logger.warning(f"Could not create residual heatmap: {e}")
                ax6.text(0.5, 0.5, 'Insufficient data\nfor interpolation', 
                        transform=ax6.transAxes, ha='center', va='center')
        
        ax6.set_xlabel('X [m]')
        ax6.set_ylabel('Z [m]')
        ax6.set_title(f'{stage_name} - Residual Magnitude Heatmap [mm]')
        
        # 7. 残差向量玫瑰图（极坐标）
        ax7 = fig.add_subplot(337, projection='polar')
        
        # 将角度转换为极坐标
        theta = np.arctan2(residual_vectors[:, 2], residual_vectors[:, 0])
        r = residual_magnitudes * 1000  # 转换为mm
        
        # 创建角度bins
        n_bins = 16
        theta_bins = np.linspace(-np.pi, np.pi, n_bins + 1)
        
        # 计算每个bin的平均大小
        bin_means = []
        bin_centers = []
        for i in range(n_bins):
            mask = (theta >= theta_bins[i]) & (theta < theta_bins[i + 1])
            if np.any(mask):
                bin_means.append(np.mean(r[mask]))
            else:
                bin_means.append(0)
            bin_centers.append((theta_bins[i] + theta_bins[i + 1]) / 2)
        
        ax7.bar(bin_centers, bin_means, width=2*np.pi/n_bins, alpha=0.7, color='lightgreen')
        ax7.set_title(f'{stage_name} - Residual Vector Rose Plot\n(Magnitude in mm)')
        ax7.set_theta_zero_location('E')  # 0度在东方
        ax7.set_theta_direction(1)  # 逆时针
        
        # 8. 残差X vs Z散点图
        ax8 = fig.add_subplot(338)
        scatter = ax8.scatter(residual_vectors[:, 0] * 1000, residual_vectors[:, 2] * 1000, 
                             c=residual_magnitudes * 1000, cmap='viridis', alpha=0.7, s=30)
        plt.colorbar(scatter, ax=ax8, label='Magnitude [mm]')
        
        # 添加十字线
        ax8.axhline(y=0, color='k', linestyle='-', alpha=0.3)
        ax8.axvline(x=0, color='k', linestyle='-', alpha=0.3)
        
        # 添加目标圆圈
        circle = plt.Circle((0, 0), TARGET_RMSE * 1000, fill=False, color='red', 
                           linestyle='--', linewidth=2, label=f'Target: {TARGET_RMSE*1000:.1f}mm')
        ax8.add_patch(circle)
        
        ax8.set_xlabel('X-direction Residual [mm]')
        ax8.set_ylabel('Z-direction Residual [mm]')
        ax8.set_title(f'{stage_name} - Residual X vs Z Components')
        ax8.legend()
        ax8.grid(True, alpha=0.3)
        ax8.axis('equal')
        
        # 9. 残差统计摘要
        ax9 = fig.add_subplot(339)
        ax9.axis('off')
        
        # 计算更多统计信息
        median_magnitude = np.median(residual_magnitudes)
        percentile_95 = np.percentile(residual_magnitudes, 95)
        percentile_99 = np.percentile(residual_magnitudes, 99)
        
        # 计算目标达成率
        within_target = np.sum(residual_magnitudes <= TARGET_RMSE) / len(residual_magnitudes) * 100
        
        stats_text = f"""
{stage_name} - Residual Vector Statistics

Total Points: {len(residual_vectors)}
Displayed Vectors: {len(display_transformed)}

Magnitude Statistics (mm):
  Mean: {mean_magnitude*1000:.3f}
  Median: {median_magnitude*1000:.3f}
  Std Dev: {std_magnitude*1000:.3f}
  Maximum: {max_magnitude*1000:.3f}
  95th Percentile: {percentile_95*1000:.3f}
  99th Percentile: {percentile_99*1000:.3f}

Target Achievement:
  Target RMSE: {TARGET_RMSE*1000:.1f} mm
  Within Target: {within_target:.1f}%

Direction Statistics:
  Mean X: {np.mean(residual_vectors[:, 0])*1000:.3f} mm
  Mean Z: {np.mean(residual_vectors[:, 2])*1000:.3f} mm
  Std X: {np.std(residual_vectors[:, 0])*1000:.3f} mm
  Std Z: {np.std(residual_vectors[:, 2])*1000:.3f} mm
        """
        
        ax9.text(0.05, 0.95, stats_text, transform=ax9.transAxes, fontsize=10,
                verticalalignment='top', fontfamily='monospace',
                bbox=dict(boxstyle='round', facecolor='lightgray', alpha=0.8))
        
        plt.tight_layout()
        
        # 保存图像
        filename = f'residual_vectors_comprehensive_{stage_name.lower().replace(" ", "_")}.png'
        plt.savefig(os.path.join(self.img_dir, filename), dpi=300, bbox_inches='tight')
        plt.close()
        
        logger.info(f"Comprehensive residual vector visualization saved: {filename}")
        
        # 保存残差向量数据
        self._save_residual_data(transformed_points, target_points, residual_vectors, stage_name)
    
    def _save_residual_data(self, transformed_points, target_points, residual_vectors, stage_name):
        """保存残差向量数据"""
        residual_data = np.hstack([
            transformed_points,
            target_points,
            residual_vectors,
            np.linalg.norm(residual_vectors[:, [0, 2]], axis=1).reshape(-1, 1)  # 残差大小
        ])
        
        header = "transformed_x transformed_y transformed_z target_x target_y target_z residual_x residual_y residual_z residual_magnitude"
        filename = f'residual_data_{stage_name.lower().replace(" ", "_")}.txt'
        np.savetxt(os.path.join(self.img_dir, filename), residual_data, header=header)
        
        logger.info(f"Residual vector data saved: {filename}")
    
    def create_residual_vector_animation_data(self, points_history, residuals_history, stage_names):
        """
        创建残差向量演化的动画数据（为将来的动画功能准备）
        """
        # 这里可以为将来的动画功能准备数据
        # 当前版本只保存数据，不实际创建动画
        
        animation_data = {
            'points_history': points_history,
            'residuals_history': residuals_history,
            'stage_names': stage_names
        }
        
        # 保存为pickle文件供将来使用
        import pickle
        with open(os.path.join(self.img_dir, 'residual_animation_data.pkl'), 'wb') as f:
            pickle.dump(animation_data, f)
        
        logger.info("Residual vector animation data saved")

class SimpleDataSplitter:
    """
    简单的随机数据集划分器
    """
    def __init__(self, train_ratio=0.8, random_seed=42):
        self.train_ratio = train_ratio
        self.random_seed = random_seed
        
    def split_data(self, positions_A, positions_B, df=None):
        """
        简单随机划分训练集和测试集
        """
        logger.info("=== Starting Simple Data Splitting ===")
        
        positions_A = np.array(positions_A)
        positions_B = np.array(positions_B)
        
        total_points = len(positions_A)
        train_size = int(total_points * self.train_ratio)
        
        # 设置随机种子以确保可重复性
        np.random.seed(self.random_seed)
        
        # 随机选择训练集索引
        all_indices = np.arange(total_points)
        np.random.shuffle(all_indices)
        
        train_indices = all_indices[:train_size]
        test_indices = all_indices[train_size:]
        
        # 提取训练集和测试集
        train_A = positions_A[train_indices]
        train_B = positions_B[train_indices]
        test_A = positions_A[test_indices]
        test_B = positions_B[test_indices]
        
        train_df = df.iloc[train_indices].copy() if df is not None else None
        test_df = df.iloc[test_indices].copy() if df is not None else None
        
        logger.info(f"Data split completed:")
        logger.info(f"  Total points: {total_points}")
        logger.info(f"  Training points: {len(train_indices)} ({len(train_indices)/total_points*100:.1f}%)")
        logger.info(f"  Testing points: {len(test_indices)} ({len(test_indices)/total_points*100:.1f}%)")
        
        # 可视化数据划分
        self._visualize_data_split(positions_A, train_indices, test_indices)
        
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
    
    def _visualize_data_split(self, positions_A, train_indices, test_indices):
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
        
        # 训练/测试集覆盖图
        ax4.scatter(train_positions[:, 0], train_positions[:, 2], c='green', s=20, alpha=0.6, 
                   label=f'Training ({len(train_indices)} points)')
        ax4.scatter(test_positions[:, 0], test_positions[:, 2], c='red', s=20, alpha=0.6, 
                   label=f'Testing ({len(test_indices)} points)')
        ax4.set_xlabel('X [m]')
        ax4.set_ylabel('Z [m]')
        ax4.set_title('Training/Test Split Distribution')
        ax4.legend()
        ax4.grid(True, alpha=0.3)
        ax4.axis('equal')
        
        plt.tight_layout()
        plt.savefig(os.path.join(img_dir, 'simple_data_split.png'), dpi=300, bbox_inches='tight')
        plt.close()
        
        logger.info("Data split visualization saved")
    
    def _save_split_data(self, train_A, train_B, test_A, test_B, train_df, test_df, train_indices, test_indices):
        """保存划分后的数据"""
        # 保存训练集
        train_data = np.hstack([train_A, train_B])
        np.savetxt(os.path.join(result_dir, "simple_train_data.txt"), train_data, 
                  header="train_source_x train_source_y train_source_z train_target_x train_target_y train_target_z")
        
        # 保存测试集
        test_data = np.hstack([test_A, test_B])
        np.savetxt(os.path.join(result_dir, "simple_test_data.txt"), test_data, 
                  header="test_source_x test_source_y test_source_z test_target_x test_target_y test_target_z")
        
        # 保存CSV文件
        if train_df is not None:
            train_df_copy = train_df.copy()
            train_df_copy['split_type'] = 'train'
            train_df_copy.to_csv(os.path.join(result_dir, "simple_train_data.csv"), index=False)
        
        if test_df is not None:
            test_df_copy = test_df.copy()
            test_df_copy['split_type'] = 'test'
            test_df_copy.to_csv(os.path.join(result_dir, "simple_test_data.csv"), index=False)
        
        # 保存索引
        np.savetxt(os.path.join(result_dir, "train_indices.txt"), train_indices, fmt='%d')
        np.savetxt(os.path.join(result_dir, "test_indices.txt"), test_indices, fmt='%d')
        
        # 保存划分摘要
        with open(os.path.join(result_dir, "simple_split_summary.txt"), "w") as f:
            f.write("Simple Data Split Summary\n")
            f.write("=" * 40 + "\n")
            f.write(f"Split method: Random sampling\n")
            f.write(f"Train ratio: {self.train_ratio:.2f}\n")
            f.write(f"Random seed: {self.random_seed}\n")
            f.write(f"Training points: {len(train_indices)}\n")
            f.write(f"Testing points: {len(test_indices)}\n")
            f.write(f"Total points: {len(train_indices) + len(test_indices)}\n")
        
        logger.info("Simple split data saved")

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
    Enhanced coordinate transformation with simple data splitting, SVD residual filtering, and comprehensive evaluation
    ENHANCED with improved residual vector visualization and SVD residual outlier removal
    """
    def __init__(self, positions_A=None, positions_B=None, original_df=None):
        self.T_svd = None
        self.deformation_model_all = None
        self.deformation_model_inliers = None
        self.ransac_filter = None
        self.svd_residual_filter = None
        self.__positions_A = positions_A
        self.__positions_B = positions_B
        self.__original_df = original_df
        
        # Training data (filtered)
        self.__train_positions_A = None
        self.__train_positions_B = None
        self.__train_inlier_A = None
        self.__train_inlier_B = None
        
        # SVD residual filtered data
        self.__svd_filtered_A = None
        self.__svd_filtered_B = None
        
        # Testing data
        self.__test_positions_A = None
        self.__test_positions_B = None
        
        # Data splitter
        self.data_splitter = SimpleDataSplitter(train_ratio=TRAIN_TEST_RATIO)
        
        # Residual vector visualizer
        self.residual_visualizer = ResidualVectorVisualizer(img_dir)
        
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
        """Visualize transformation results for a specific stage with ENHANCED residual vector display"""
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
        
        # 如果没有提供残差向量，则计算它们
        if residual_vectors is None:
            residual_vectors = positions_B - transformed_positions
        
        # === 新增：使用专门的残差向量可视化器 ===
        self.residual_visualizer.visualize_residual_vectors_comprehensive(
            positions_A, positions_B, transformed_positions, residual_vectors, stage_name
        )
        
        # 3D可视化 - 改进的残差向量显示
        fig = plt.figure(figsize=(20, 12))
        
        # 3D散点图 - 改进的残差向量显示
        ax1 = fig.add_subplot(221, projection='3d')
        
        ax1.scatter(positions_A[:, 0], positions_A[:, 2], positions_A[:, 1], 
                   c='blue', marker='o', s=50, label='Source Points', alpha=0.7)
        ax1.scatter(positions_B[:, 0], positions_B[:, 2], positions_B[:, 1], 
                   c='red', marker='^', s=50, label='Target Points', alpha=0.7)
        ax1.scatter(transformed_positions[:, 0], transformed_positions[:, 2], transformed_positions[:, 1], 
                   c='green', marker='x', s=50, label='Transformed Points', alpha=0.7)
        
        # 添加3D残差向量（选择性显示，避免过于拥挤）
        if len(transformed_positions) <= 50:  # 只在点数较少时显示所有向量
            step = 1
        else:
            step = max(1, len(transformed_positions) // 50)  # 最多显示50个向量
        
        for i in range(0, len(transformed_positions), step):
            if individual_errors[i] > TARGET_RMSE:  # 只显示误差较大的向量
                ax1.plot([transformed_positions[i, 0], positions_B[i, 0]], 
                        [transformed_positions[i, 2], positions_B[i, 2]],
                        [transformed_positions[i, 1], positions_B[i, 1]], 
                        'r-', alpha=0.6, linewidth=1)
        
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
        
        # XZ平面视图 - 改进的残差向量显示
        ax4 = fig.add_subplot(224)
        ax4.scatter(positions_A[:, 0], positions_A[:, 2], c='blue', marker='o', s=50, 
                   label='Source Points', alpha=0.7)
        ax4.scatter(positions_B[:, 0], positions_B[:, 2], c='red', marker='^', s=50, 
                   label='Target Points', alpha=0.7)
        ax4.scatter(transformed_positions[:, 0], transformed_positions[:, 2], c='green', marker='x', s=50, 
                   label='Transformed Points', alpha=0.7)
        
        # 改进的残差向量显示
        if residual_vectors is not None:
            residual_vectors = np.array(residual_vectors)
            
            # 根据向量大小决定显示策略
            max_display = 100  # 最多显示100个向量
            if len(residual_vectors) > max_display:
                # 选择误差最大的向量进行显示
                indices = np.argsort(individual_errors)[-max_display:]
            else:
                indices = np.arange(len(residual_vectors))
            
            # 使用颜色编码表示残差大小
            colors = plt.cm.viridis(individual_errors[indices] / np.max(individual_errors))
            
            for i, idx in enumerate(indices):
                # 只显示超过阈值的残差向量
                if individual_errors[idx] > TARGET_RMSE * 0.5:  # 显示超过目标50%的残差
                    ax4.arrow(transformed_positions[idx, 0], transformed_positions[idx, 2],
                             residual_vectors[idx, 0], residual_vectors[idx, 2],
                             head_width=0.002, head_length=0.003, 
                             fc=colors[i], ec=colors[i], alpha=0.8, linewidth=1)
        
        ax4.set_xlabel('X (Lateral) [m]')
        ax4.set_ylabel('Z (Longitudinal) [m]')
        ax4.set_title(f'{stage_name} - XZ Plane View with Enhanced Residual Vectors')
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
        """Enhanced registration with simple data splitting, SVD residual filtering, and comprehensive evaluation"""
        if self.__positions_A is None or self.__positions_B is None:
            logger.error("Please provide position data first!")
            return None, None
        
        logger.info("=== Starting Enhanced Registration with SVD Residual Filtering ===")
        logger.info(f"SVD residual threshold: {SVD_RESIDUAL_THRESHOLD*1000:.1f}mm")
        
        original_positions_A = np.array(self.__positions_A)
        original_positions_B = np.array(self.__positions_B)
        
        # Step 1: Simple data splitting
        logger.info("Step 1: Simple Random Data Splitting")
        split_data = self.data_splitter.split_data(
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
        
        # === 新增：可视化训练集SVD残差向量 ===
        self.residual_visualizer.visualize_residual_vectors_comprehensive(
            train_inlier_A, train_inlier_B, train_svd_transformed, 
            train_residual_vectors, "Training SVD"
        )
        
        # Step 4: 新增 - SVD残差向量过滤（应用到所有数据）
        logger.info("Step 4: SVD Residual Filtering on All Original Data")
        self.svd_residual_filter = SVDResidualFilter(residual_threshold=SVD_RESIDUAL_THRESHOLD)
        
        # 对所有原始数据应用SVD变换
        all_svd_transformed = self.apply_transformation(original_positions_A, self.T_svd)
        
        # 过滤残差过大的点对
        (svd_filtered_A, svd_filtered_B, svd_removed_A, svd_removed_B, 
         all_residual_vectors, all_residual_magnitudes) = self.svd_residual_filter.filter_svd_residuals(
            original_positions_A, original_positions_B, self.T_svd
        )
        
        # 保存SVD过滤后的数据
        self.__svd_filtered_A = svd_filtered_A
        self.__svd_filtered_B = svd_filtered_B
        
        # 可视化SVD残差过滤结果
        self.svd_residual_filter.visualize_residual_filtering(
            original_positions_A, original_positions_B,
            svd_filtered_A, svd_filtered_B,
            svd_removed_A, svd_removed_B,
            all_residual_vectors, all_residual_magnitudes,
            "All Data SVD Residual Filtering"
        )
        
        # 保存SVD过滤结果
        self.svd_residual_filter.save_filtering_results(
            original_positions_A, original_positions_B,
            svd_filtered_A, svd_filtered_B,
            svd_removed_A, svd_removed_B,
            "all_data"
        )
        
        # Step 5: 重新分割过滤后的数据
        logger.info("Step 5: Re-splitting SVD Filtered Data")
        
        # 从过滤后的数据中重新提取训练集和测试集
        if len(svd_filtered_A) == 0:
            logger.error("No data left after SVD residual filtering")
            return None, None
        
        # 重新划分数据
        filtered_split_data = self.data_splitter.split_data(
            svd_filtered_A, svd_filtered_B, None
        )
        
        filtered_train_A = filtered_split_data['train_A']
        filtered_train_B = filtered_split_data['train_B']
        filtered_test_A = filtered_split_data['test_A']
        filtered_test_B = filtered_split_data['test_B']
        
        logger.info(f"After SVD filtering - Training set: {len(filtered_train_A)} points")
        logger.info(f"After SVD filtering - Test set: {len(filtered_test_A)} points")
        
        # Step 6: Learn deformation models on filtered training data
        logger.info("Step 6: Learning Deformation Models on Filtered Data")
        
        # 6a: Learn on all filtered training data
        logger.info("  6a: Learning deformation on ALL filtered training data")
        self.deformation_model_all = NonRigidDeformationModel(
            method='thin_plate_spline',
            smoothing=0.001,
            constrain_y=True
        )
        
        # Apply SVD to filtered training data
        filtered_train_svd_transformed = self.apply_transformation(filtered_train_A, self.T_svd)
        filtered_train_residual_vectors = filtered_train_B - filtered_train_svd_transformed
        
        # === 新增：可视化过滤后训练数据的SVD残差向量 ===
        self.residual_visualizer.visualize_residual_vectors_comprehensive(
            filtered_train_A, filtered_train_B, filtered_train_svd_transformed, 
            filtered_train_residual_vectors, "Filtered Training SVD"
        )
        
        success_all = self.deformation_model_all.learn_deformation(
            filtered_train_svd_transformed, filtered_train_residual_vectors
        )
        
        # 6b: 可选 - 在过滤后的训练数据上再次运行RANSAC然后学习形变
        logger.info("  6b: Learning deformation on RANSAC inliers of filtered training data")
        
        filtered_ransac = RANSACFilter(
            min_samples=min(RANSAC_MIN_SAMPLES, len(filtered_train_A)//2),
            max_iterations=RANSAC_ITERATIONS//2,
            inlier_threshold=RANSAC_INLIER_THRESHOLD
        )
        
        filtered_inlier_A, filtered_inlier_B, _, _ = filtered_ransac.ransac_filter(
            filtered_train_A, filtered_train_B
        )
        
        if filtered_inlier_A is not None and len(filtered_inlier_A) > 0:
            self.deformation_model_inliers = NonRigidDeformationModel(
                method='thin_plate_spline',
                smoothing=0.001,
                constrain_y=True
            )
            
            filtered_inlier_svd_transformed = self.apply_transformation(filtered_inlier_A, self.T_svd)
            filtered_inlier_residual_vectors = filtered_inlier_B - filtered_inlier_svd_transformed
            
            success_inliers = self.deformation_model_inliers.learn_deformation(
                filtered_inlier_svd_transformed, filtered_inlier_residual_vectors
            )
        else:
            logger.warning("No inliers found in filtered training data")
            success_inliers = False
        
        if not success_all and not success_inliers:
            logger.error("Failed to learn any deformation models")
            return None, None
        
        # Step 7: Evaluate on filtered test set
        logger.info("Step 7: Evaluation on Filtered Test Set")
        
        # Apply SVD to filtered test data
        filtered_test_svd_transformed = self.apply_transformation(filtered_test_A, self.T_svd)
        filtered_test_svd_rmse = self.calculate_directional_rmse(filtered_test_B, filtered_test_svd_transformed)
        
        logger.info(f"Filtered Test SVD RMSE: {filtered_test_svd_rmse['overall_rmse']*1000:.3f}mm")
        
        # 计算过滤后测试集SVD残差向量
        filtered_test_svd_residual_vectors = filtered_test_B - filtered_test_svd_transformed
        
        # Evaluate deformation model learned on all filtered training data
        if success_all:
            test_deformation_all = self.deformation_model_all.predict_deformation(filtered_test_svd_transformed)
            test_final_all = filtered_test_svd_transformed + test_deformation_all
            test_final_all[:, 1] = 0  # Ensure Y=0
            test_final_rmse_all = self.calculate_directional_rmse(filtered_test_B, test_final_all)
            
            # 计算修正后的残差向量
            test_final_residual_all = filtered_test_B - test_final_all
            
            logger.info(f"Filtered Test Final RMSE (All Data Model): {test_final_rmse_all['overall_rmse']*1000:.3f}mm")
        else:
            test_final_rmse_all = None
            test_final_all = None
            test_final_residual_all = None
        
        # Evaluate deformation model learned on inliers only
        if success_inliers:
            test_deformation_inliers = self.deformation_model_inliers.predict_deformation(filtered_test_svd_transformed)
            test_final_inliers = filtered_test_svd_transformed + test_deformation_inliers
            test_final_inliers[:, 1] = 0  # Ensure Y=0
            test_final_rmse_inliers = self.calculate_directional_rmse(filtered_test_B, test_final_inliers)
            
            # 计算修正后的残差向量
            test_final_residual_inliers = filtered_test_B - test_final_inliers
            
            logger.info(f"Filtered Test Final RMSE (Inliers Model): {test_final_rmse_inliers['overall_rmse']*1000:.3f}mm")
        else:
            test_final_rmse_inliers = None
            test_final_inliers = None
            test_final_residual_inliers = None
        
        # Step 8: Visualizations and comparisons
        logger.info("Step 8: Creating Visualizations and Comparisons")
        
        # Visualize SVD results on filtered test set with enhanced residual vectors
        self.visualize_transformation_stage("Filtered Test SVD", filtered_test_A, filtered_test_B, 
                                           filtered_test_svd_transformed, 
                                           filtered_test_svd_rmse, filtered_test_svd_residual_vectors)
        
        # Visualize final results with enhanced residual vectors
        if test_final_rmse_all is not None:
            self.visualize_transformation_stage("Filtered Test Final (All Data Model)", 
                                               filtered_test_A, filtered_test_B, test_final_all, 
                                               test_final_rmse_all, test_final_residual_all)
        
        if test_final_rmse_inliers is not None:
            self.visualize_transformation_stage("Filtered Test Final (Inliers Model)", 
                                               filtered_test_A, filtered_test_B, test_final_inliers, 
                                               test_final_rmse_inliers, test_final_residual_inliers)
        
        # Create comparison visualization
        self._create_comparison_visualization(filtered_test_A, filtered_test_B, 
                                            filtered_test_svd_transformed, test_final_all, test_final_inliers,
                                            filtered_test_svd_rmse, test_final_rmse_all, test_final_rmse_inliers)
        
        # Visualize deformation fields
        if len(filtered_test_svd_transformed) > 0:
            x_min, x_max = np.min(filtered_test_svd_transformed[:, 0]), np.max(filtered_test_svd_transformed[:, 0])
            z_min, z_max = np.min(filtered_test_svd_transformed[:, 2]), np.max(filtered_test_svd_transformed[:, 2])
            
            x_range = x_max - x_min
            z_range = z_max - z_min
            bounds = [
                x_min - 0.1 * x_range, x_max + 0.1 * x_range,
                z_min - 0.1 * z_range, z_max + 0.1 * z_range
            ]
            
            if success_all:
                self.deformation_model_all.visualize_deformation_field(bounds, resolution=30, stage_name="Filtered All Data Model")
            
            if success_inliers:
                self.deformation_model_inliers.visualize_deformation_field(bounds, resolution=30, stage_name="Filtered Inliers Model")
        
        # === 新增：创建残差向量演化数据 ===
        points_history = [filtered_test_svd_transformed]
        residuals_history = [filtered_test_svd_residual_vectors]
        stage_names = ["Filtered Test SVD"]
        
        if test_final_all is not None:
            points_history.append(test_final_all)
            residuals_history.append(test_final_residual_all)
            stage_names.append("Filtered Test Final (All Data)")
        
        if test_final_inliers is not None:
            points_history.append(test_final_inliers)
            residuals_history.append(test_final_residual_inliers)
            stage_names.append("Filtered Test Final (Inliers)")
        
        self.residual_visualizer.create_residual_vector_animation_data(
            points_history, residuals_history, stage_names
        )
        
        # Save comprehensive results
        self._save_comprehensive_results(filtered_split_data, filtered_test_svd_rmse, 
                                       test_final_rmse_all, test_final_rmse_inliers,
                                       len(original_positions_A), len(svd_removed_A))
        
        # Print detailed analysis
        self._print_comprehensive_analysis(len(original_positions_A), len(filtered_train_A), 
                                         len(filtered_inlier_A) if filtered_inlier_A is not None else 0, 
                                         len(filtered_test_A), len(svd_removed_A),
                                         filtered_test_svd_rmse, test_final_rmse_all, test_final_rmse_inliers)
        
        return self.T_svd, (test_final_rmse_all, test_final_rmse_inliers)

    def _create_comparison_visualization(self, test_A, test_B, svd_result, final_all, final_inliers, 
                                       svd_rmse, rmse_all, rmse_inliers):
        """Create comprehensive comparison visualization with enhanced residual vector display"""
        logger.info("Creating comparison visualization...")
        
        fig = plt.figure(figsize=(24, 16))
        
        # XZ plane comparisons
        ax1 = fig.add_subplot(231)
        ax1.scatter(test_A[:, 0], test_A[:, 2], c='blue', s=30, alpha=0.7, label='Source')
        ax1.scatter(test_B[:, 0], test_B[:, 2], c='red', s=30, alpha=0.7, label='Target')
        ax1.scatter(svd_result[:, 0], svd_result[:, 2], c='green', s=30, alpha=0.7, label='SVD Result')
        
        # 添加残差向量（选择性显示）
        residual_svd = test_B - svd_result
        individual_errors_svd = np.linalg.norm(residual_svd[:, [0, 2]], axis=1)
        
        # 只显示误差较大的向量
        large_error_indices = individual_errors_svd > TARGET_RMSE
        if np.any(large_error_indices):
            step = max(1, np.sum(large_error_indices) // 20)  # 最多显示20个向量
            display_indices = np.where(large_error_indices)[0][::step]
            
            for idx in display_indices:
                ax1.arrow(svd_result[idx, 0], svd_result[idx, 2],
                         residual_svd[idx, 0], residual_svd[idx, 2],
                         head_width=0.001, head_length=0.002, 
                         fc='red', ec='red', alpha=0.6, linewidth=1)
        
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
            
            # 添加残差向量
            residual_all = test_B - final_all
            individual_errors_all = np.linalg.norm(residual_all[:, [0, 2]], axis=1)
            
            large_error_indices = individual_errors_all > TARGET_RMSE * 0.5
            if np.any(large_error_indices):
                step = max(1, np.sum(large_error_indices) // 20)
                display_indices = np.where(large_error_indices)[0][::step]
                
                for idx in display_indices:
                    ax2.arrow(final_all[idx, 0], final_all[idx, 2],
                             residual_all[idx, 0], residual_all[idx, 2],
                             head_width=0.001, head_length=0.002, 
                             fc='orange', ec='orange', alpha=0.6, linewidth=1)
            
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
            
            # 添加残差向量
            residual_inliers = test_B - final_inliers
            individual_errors_inliers = np.linalg.norm(residual_inliers[:, [0, 2]], axis=1)
            
            large_error_indices = individual_errors_inliers > TARGET_RMSE * 0.5
            if np.any(large_error_indices):
                step = max(1, np.sum(large_error_indices) // 20)
                display_indices = np.where(large_error_indices)[0][::step]
                
            for idx in display_indices:
                    ax3.arrow(final_inliers[idx, 0], final_inliers[idx, 2],
                             residual_inliers[idx, 0], residual_inliers[idx, 2],
                             head_width=0.001, head_length=0.002, 
                             fc='cyan', ec='cyan', alpha=0.6, linewidth=1)
            
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
        ax4.axhline(y=SVD_RESIDUAL_THRESHOLD*1000, color='blue', linestyle='--', alpha=0.7, 
                   label=f'SVD Filter: {SVD_RESIDUAL_THRESHOLD*1000:.1f}mm')
        ax4.set_ylabel('RMSE [mm]')
        ax4.set_title('Method Comparison on Filtered Test Set')
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
        ax5.axvline(x=SVD_RESIDUAL_THRESHOLD*1000, color='blue', linestyle='--', alpha=0.7,
                   label=f'SVD Filter: {SVD_RESIDUAL_THRESHOLD*1000:.1f}mm')
        ax5.set_xlabel('Error [mm]')
        ax5.set_ylabel('Frequency')
        ax5.set_title('Error Distribution Comparison (Filtered Data)')
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
            ax6.set_title('Deformation Model Improvement\n(After SVD Filtering)')
            ax6.grid(True, alpha=0.3)
            
            # Add value labels on bars
            for bar, improvement in zip(bars, improvements):
                height = bar.get_height()
                ax6.text(bar.get_x() + bar.get_width()/2., height,
                        f'{improvement:.2f}mm',
                        ha='center', va='bottom' if height >= 0 else 'top')
        else:
            ax6.text(0.5, 0.5, 'Insufficient data\nfor comparison', 
                    transform=ax6.transAxes, ha='center', va='center')
        
        plt.tight_layout()
        plt.savefig(os.path.join(img_dir, 'comprehensive_comparison_with_svd_filtering.png'), dpi=300, bbox_inches='tight')
        plt.close()
        
        logger.info("Enhanced comparison visualization with SVD filtering saved")

    def _save_comprehensive_results(self, split_data, test_svd_rmse, test_final_rmse_all, test_final_rmse_inliers,
                                   original_count, removed_count):
        """Save comprehensive results including SVD filtering information"""
        # Save test results
        test_data = np.hstack([self.__test_positions_A, self.__test_positions_B])
        np.savetxt(os.path.join(result_dir, "filtered_test_evaluation_data.txt"), test_data, 
                  header="test_source_x test_source_y test_source_z test_target_x test_target_y test_target_z")
        
        # Save transformation matrix
        if self.T_svd is not None:
            np.savetxt(os.path.join(result_dir, "T_svd_with_filtering.txt"), self.T_svd)
        
        # Save SVD filtered data information
        if self.__svd_filtered_A is not None:
            filtered_data = np.hstack([self.__svd_filtered_A, self.__svd_filtered_B])
            np.savetxt(os.path.join(result_dir, "svd_filtered_all_data.txt"), filtered_data,
                      header="filtered_source_x filtered_source_y filtered_source_z filtered_target_x filtered_target_y filtered_target_z")
        
        # Save comprehensive summary
        with open(os.path.join(result_dir, "comprehensive_summary_with_svd_filtering.txt"), "w") as f:
            f.write("Enhanced Registration Summary with SVD Residual Filtering\n")
            f.write("=" * 70 + "\n")
            f.write(f"Data splitting method: Simple random sampling\n")
            f.write(f"Random seed: {self.data_splitter.random_seed}\n")
            f.write(f"Original dataset size: {original_count}\n")
            f.write(f"SVD residual threshold: {SVD_RESIDUAL_THRESHOLD*1000:.1f}mm\n")
            f.write(f"Points removed by SVD filtering: {removed_count} ({removed_count/original_count*100:.1f}%)\n")
            f.write(f"Points kept after SVD filtering: {len(self.__svd_filtered_A)} ({len(self.__svd_filtered_A)/original_count*100:.1f}%)\n")
            f.write(f"Filtered training set size: {len(split_data['train_A'])}\n")
            f.write(f"Filtered test set size: {len(split_data['test_A'])}\n")
            f.write("\n")
            
            f.write("Filtered Test Set Results:\n")
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
            f.write(f"SVD Filter Threshold: {SVD_RESIDUAL_THRESHOLD*1000:.1f}mm\n")
            
            success_svd = test_svd_rmse['overall_rmse'] <= TARGET_RMSE
            f.write(f"SVD meets target: {'Yes' if success_svd else 'No'}\n")
            
            if test_final_rmse_all is not None:
                success_all = test_final_rmse_all['overall_rmse'] <= TARGET_RMSE
                f.write(f"All Data Model meets target: {'Yes' if success_all else 'No'}\n")
            
            if test_final_rmse_inliers is not None:
                success_inliers = test_final_rmse_inliers['overall_rmse'] <= TARGET_RMSE
                f.write(f"Inliers Model meets target: {'Yes' if success_inliers else 'No'}\n")
            
            f.write(f"\nData filtering efficiency: {(original_count-removed_count)/original_count*100:.1f}%\n")
        
        logger.info("Comprehensive results with SVD filtering saved")

    def _print_comprehensive_analysis(self, total_points, train_points, train_inlier_points, test_points, removed_points,
                                    test_svd_rmse, test_final_rmse_all, test_final_rmse_inliers):
        """Print comprehensive analysis including SVD filtering information"""
        logger.info("\n=== Comprehensive Performance Analysis with SVD Filtering ===")
        
        logger.info("Data Distribution:")
        logger.info(f"  Total original points: {total_points}")
        logger.info(f"  Points removed by SVD filtering: {removed_points} ({removed_points/total_points*100:.1f}%)")
        logger.info(f"  Points kept after SVD filtering: {total_points - removed_points} ({(total_points - removed_points)/total_points*100:.1f}%)")
        logger.info(f"  Filtered training points: {train_points} ({train_points/(total_points - removed_points)*100:.1f}% of filtered)")
        logger.info(f"  Filtered training inliers: {train_inlier_points} ({train_inlier_points/train_points*100:.1f}% of filtered training)")
        logger.info(f"  Filtered test points: {test_points} ({test_points/(total_points - removed_points)*100:.1f}% of filtered)")
        
        logger.info("Filtering Thresholds:")
        logger.info(f"  RANSAC threshold: {RANSAC_INLIER_THRESHOLD*1000:.1f}mm")
        logger.info(f"  SVD residual threshold: {SVD_RESIDUAL_THRESHOLD*1000:.1f}mm")
        logger.info(f"  Target RMSE: {TARGET_RMSE*1000:.1f}mm")
        
        logger.info("Filtered Test Set Performance:")
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
        
        logger.info("SVD Filtering Effectiveness:")
        filter_effectiveness = (1 - removed_points/total_points) * 100
        logger.info(f"  Data retention rate: {filter_effectiveness:.1f}%")
        logger.info(f"  Points removed with residual > {SVD_RESIDUAL_THRESHOLD*1000:.1f}mm: {removed_points}")

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
    logger.info("Starting Enhanced Tracker Data Processing with SVD Residual Filtering")
    logger.info("Features: Simple data splitting + RANSAC + SVD + SVD Residual Filtering + Dual deformation learning + Test evaluation")
    logger.info("ENHANCED: Comprehensive residual vector visualization + SVD residual outlier removal")
    logger.info(f"Scale factor: {SCALE_FACTOR}")
    logger.info(f"Target RMSE: {TARGET_RMSE*1000:.2f}mm")
    logger.info(f"SVD residual threshold: {SVD_RESIDUAL_THRESHOLD*1000:.1f}mm")
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
    
    # Perform enhanced registration with SVD filtering
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
        
        # Calculate filtering statistics
        original_count = len(positions_A)
        filtered_count = len(transformer._CoordinateTransformer__svd_filtered_A) if transformer._CoordinateTransformer__svd_filtered_A is not None else 0
        removed_count = original_count - filtered_count
        
        if success:
            logger.info(f"SUCCESS: Registration completed with best RMSE {best_rmse*1000:.3f}mm using {best_model}")
        else:
            logger.info(f"PARTIAL: Registration completed, but best RMSE {best_rmse*1000:.3f}mm exceeds target {TARGET_RMSE*1000:.2f}mm")
        
        logger.info(f"SVD filtering removed {removed_count}/{original_count} points ({removed_count/original_count*100:.1f}%)")
        
        # Save final summary
        summary_file = os.path.join(output_dir, "enhanced_registration_with_svd_filtering_summary.txt")
        with open(summary_file, "w") as f:
            f.write("Enhanced Registration Summary with SVD Residual Filtering\n")
            f.write("=" * 80 + "\n")
            f.write("Features: Simple data splitting + RANSAC + SVD + SVD Residual Filtering + Dual deformation learning + Test evaluation\n")
            f.write("ENHANCED: Comprehensive residual vector visualization + SVD residual outlier removal\n")
            f.write(f"Data splitting: Simple random sampling (seed=42)\n")
            f.write(f"Scale Factor: {SCALE_FACTOR}\n")
            f.write(f"Target RMSE: {TARGET_RMSE*1000:.2f}mm\n")
            f.write(f"SVD Residual Threshold: {SVD_RESIDUAL_THRESHOLD*1000:.1f}mm\n")
            f.write(f"Train/Test ratio: {TRAIN_TEST_RATIO:.1f}/{1-TRAIN_TEST_RATIO:.1f}\n")
            f.write(f"Best RMSE: {best_rmse*1000:.3f}mm\n")
            f.write(f"Best Model: {best_model}\n")
            f.write(f"Success: {'Yes' if success else 'No'}\n")
            f.write(f"Original points: {original_count}\n")
            f.write(f"Points removed by SVD filtering: {removed_count} ({removed_count/original_count*100:.1f}%)\n")
            f.write(f"Points retained: {filtered_count} ({filtered_count/original_count*100:.1f}%)\n")
            f.write(f"Real-world equivalent: {best_rmse*SCALE_FACTOR*100:.2f}cm\n")
            f.write(f"\nMethod: Simple random data splitting + RANSAC outlier removal + SVD registration + SVD residual filtering + Dual RBF deformation learning\n")
            f.write("Evaluation: SVD transformation learned on training data, residual filtering applied to all data, final evaluation on filtered test set\n")
            f.write("Deformation models: (1) Learned on all filtered training data, (2) Learned on RANSAC inliers of filtered training data\n")
            f.write("ENHANCED: Comprehensive residual vector visualization with heatmaps, statistics, and direction analysis\n")
            f.write("NEW: SVD residual vector outlier removal with comprehensive visualization of removed points\n")
    else:
        logger.error("Enhanced registration with SVD filtering failed")
    
    logger.info("Enhanced processing complete with SVD residual filtering and improved residual vector visualization")

if __name__ == "__main__":
    main()