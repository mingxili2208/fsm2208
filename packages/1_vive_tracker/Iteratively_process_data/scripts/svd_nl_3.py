#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Tracker data processing script with SVD + Non-rigid Deformation Learning (Optimized with CV, hyperparameter tuning, and outlier detection)
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
from scipy.interpolate import griddata, RBFInterpolator, LSQBivariateSpline
from scipy.spatial.distance import cdist
from scipy import stats
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
output_dir = os.path.join(result_dir, f"SVD_Deformation_Optimized_{timestamp}_1to32_scale_RMSE_{TARGET_RMSE*1000:.1f}mm")
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
LASER_TRACKER_CSV = os.path.join(parent_dir, "data/combined_corrected_tracker_data_1.csv")
CORRECTED_LASER_TRACKER_CSV = os.path.join(data_dir, "corrected_laser_tracker.csv")

ROW_X="X"
ROW_Y="Z"

class OutlierDetector:
    """
    统计离群点检测器
    """
    def __init__(self, method='iqr', iqr_factor=3.0, z_score_threshold=3.0):
        """
        Parameters:
        method: 'iqr', 'z_score', 'modified_z_score'
        iqr_factor: IQR方法的阈值因子
        z_score_threshold: Z-score方法的阈值
        """
        self.method = method
        self.iqr_factor = iqr_factor
        self.z_score_threshold = z_score_threshold
        self.outlier_indices = []
        self.outlier_stats = {}
        
    def detect_outliers_initial_svd(self, positions_A, positions_B):
        """
        使用初始SVD检测离群点
        """
        logger.info(f"Starting outlier detection using {self.method} method...")
        
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
        logger.info(f"  Min: {np.min(residual_magnitudes)*1000:.3f}mm")
        
        # 检测离群点
        outlier_mask = self._detect_outliers(residual_magnitudes)
        self.outlier_indices = np.where(outlier_mask)[0]
        
        # 统计信息
        self.outlier_stats = {
            'total_points': len(positions_A),
            'outliers_detected': len(self.outlier_indices),
            'outlier_ratio': len(self.outlier_indices) / len(positions_A),
            'outlier_residuals': residual_magnitudes[self.outlier_indices],
            'clean_residuals': residual_magnitudes[~outlier_mask],
            'threshold_used': self._get_threshold(residual_magnitudes)
        }
        
        logger.info(f"Outlier detection results:")
        logger.info(f"  Method: {self.method}")
        logger.info(f"  Total points: {self.outlier_stats['total_points']}")
        logger.info(f"  Outliers detected: {self.outlier_stats['outliers_detected']}")
        logger.info(f"  Outlier ratio: {self.outlier_stats['outlier_ratio']*100:.2f}%")
        logger.info(f"  Threshold: {self.outlier_stats['threshold_used']*1000:.3f}mm")
        
        if len(self.outlier_indices) > 0:
            logger.info(f"  Outlier residuals: {self.outlier_stats['outlier_residuals']*1000}")
            logger.info(f"  Max outlier residual: {np.max(self.outlier_stats['outlier_residuals'])*1000:.3f}mm")
        
        # 可视化离群点检测结果
        self._visualize_outlier_detection(positions_A, positions_B, svd_transformed, 
                                         residual_magnitudes, outlier_mask)
        
        return ~outlier_mask  # 返回非离群点的mask
    
    def _detect_outliers(self, residual_magnitudes):
        """
        根据选择的方法检测离群点
        """
        if self.method == 'iqr':
            return self._iqr_method(residual_magnitudes)
        elif self.method == 'z_score':
            return self._z_score_method(residual_magnitudes)
        elif self.method == 'modified_z_score':
            return self._modified_z_score_method(residual_magnitudes)
        else:
            logger.error(f"Unknown outlier detection method: {self.method}")
            return np.zeros(len(residual_magnitudes), dtype=bool)
    
    def _iqr_method(self, data):
        """IQR方法检测离群点"""
        Q1 = np.percentile(data, 25)
        Q3 = np.percentile(data, 75)
        IQR = Q3 - Q1
        threshold = Q3 + self.iqr_factor * IQR
        return data > threshold
    
    def _z_score_method(self, data):
        """Z-score方法检测离群点"""
        z_scores = np.abs(stats.zscore(data))
        return z_scores > self.z_score_threshold
    
    def _modified_z_score_method(self, data):
        """修正Z-score方法检测离群点"""
        median = np.median(data)
        mad = np.median(np.abs(data - median))
        modified_z_scores = 0.6745 * (data - median) / mad
        return np.abs(modified_z_scores) > self.z_score_threshold
    
    def _get_threshold(self, data):
        """获取使用的阈值"""
        if self.method == 'iqr':
            Q1 = np.percentile(data, 25)
            Q3 = np.percentile(data, 75)
            IQR = Q3 - Q1
            return Q3 + self.iqr_factor * IQR
        elif self.method == 'z_score':
            return np.mean(data) + self.z_score_threshold * np.std(data)
        elif self.method == 'modified_z_score':
            median = np.median(data)
            mad = np.median(np.abs(data - median))
            return median + self.z_score_threshold * mad / 0.6745
        return 0
    
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
        ax1.set_title('Outlier Detection - Source Points')
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
        
        threshold = self._get_threshold(residual_magnitudes)
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
        
        # 4. 时间序列（如果有时间戳）
        ax4 = axes[1, 1]
        indices = np.arange(len(residual_magnitudes))
        ax4.scatter(indices[~outlier_mask], residual_magnitudes[~outlier_mask]*1000, 
                   c='blue', alpha=0.7, s=20, label='Normal Points')
        if np.any(outlier_mask):
            ax4.scatter(indices[outlier_mask], residual_magnitudes[outlier_mask]*1000, 
                       c='red', alpha=0.8, s=50, label='Outliers', marker='x')
        
        ax4.axhline(y=threshold*1000, color='red', linestyle='--', alpha=0.7,
                   label=f'Threshold ({threshold*1000:.1f}mm)')
        ax4.set_xlabel('Data Point Index')
        ax4.set_ylabel('Residual Magnitude [mm]')
        ax4.set_title('Residual Magnitude vs Time')
        ax4.legend()
        ax4.grid(True, alpha=0.3)
        
        plt.tight_layout()
        plt.savefig(os.path.join(img_dir, f'outlier_detection_{self.method}.png'), 
                   dpi=300, bbox_inches='tight')
        plt.close()
        
        logger.info(f"Outlier detection visualization saved: outlier_detection_{self.method}.png")

class BSplineDeformationModel:
    """
    B-Spline based non-rigid deformation model using LSQBivariateSpline (Fixed smoothing)
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
        学习B样条形变模型 (修正了平滑参数)
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
                # 关键修正：正确设置平滑参数
                m = len(source_points)
                s = m * self.smoothing_factor  # 平滑参数应该与数据点数量相关
                
                # 学习X方向形变
                self.spline_x = LSQBivariateSpline(
                    source_xz[:, 0], source_xz[:, 1], residual_xz[:, 0],
                    x_knots, z_knots, 
                    kx=3, ky=3,  # 3次样条
                    s=s  # 修正：添加平滑参数
                )
                
                # 学习Z方向形变
                self.spline_z = LSQBivariateSpline(
                    source_xz[:, 0], source_xz[:, 1], residual_xz[:, 1],
                    x_knots, z_knots,
                    kx=3, ky=3,
                    s=s  # 修正：添加平滑参数
                )
                
                logger.info(f"B-Spline deformation model learned: {self.knot_density}x{self.knot_density} grid, s={s:.2f}")
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

class NonRigidDeformationModel:
    """
    RBF-based non-rigid deformation model with hyperparameter optimization
    """
    def __init__(self, method='thin_plate_spline', smoothing=0.001, constrain_y=True):
        self.method = method
        self.smoothing = smoothing
        self.constrain_y = constrain_y
        self.rbf_x = None
        self.rbf_z = None
        self.source_points = None
        
    def learn_deformation(self, source_points, residual_vectors):
        """学习RBF形变模型"""
        source_points = np.array(source_points)
        residual_vectors = np.array(residual_vectors)
        
        if self.constrain_y:
            source_xz = source_points[:, [0, 2]]
            residual_xz = residual_vectors[:, [0, 2]]
            
            try:
                # 学习X方向的形变
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
                logger.info(f"RBF deformation model learned: {self.method}, smoothing={self.smoothing}")
                return True
                
            except Exception as e:
                logger.error(f"Failed to learn RBF model: {e}")
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
    
    def visualize_deformation_field(self, bounds, resolution=50):
        """可视化形变场"""
        if self.rbf_x is None or self.rbf_z is None:
            logger.warning("Cannot visualize: deformation model not trained")
            return
        
        x_min, x_max, z_min, z_max = bounds
        x_grid = np.linspace(x_min, x_max, resolution)
        z_grid = np.linspace(z_min, z_max, resolution)
        X, Z = np.meshgrid(x_grid, z_grid)
        
        query_points_2d = np.column_stack([X.ravel(), Z.ravel()])
        
        try:
            deformation_x = self.rbf_x(query_points_2d)
            deformation_z = self.rbf_z(query_points_2d)
            
            Dx = deformation_x.reshape(X.shape)
            Dz = deformation_z.reshape(X.shape)
            
            fig, (ax1, ax2, ax3) = plt.subplots(1, 3, figsize=(18, 6))
            
            # X方向形变
            im1 = ax1.contourf(X, Z, Dx*1000, levels=20, cmap='RdBu_r')
            ax1.quiver(X[::5, ::5], Z[::5, ::5], Dx[::5, ::5]*1000, np.zeros_like(Dx[::5, ::5]), 
                      scale=10, alpha=0.7)
            plt.colorbar(im1, ax=ax1)
            ax1.set_title(f'X-direction Deformation [mm]\n{self.method}, smoothing={self.smoothing}')
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
            
            if self.source_points is not None:
                for ax in [ax1, ax2, ax3]:
                    ax.scatter(self.source_points[:, 0], self.source_points[:, 1], 
                             c='black', s=20, marker='o', alpha=0.8, label='Control Points')
                    ax.legend()
            
            plt.tight_layout()
            filename = f'deformation_field_{self.method}_{self.smoothing:.6f}.png'
            plt.savefig(os.path.join(img_dir, filename), dpi=300, bbox_inches='tight')
            plt.close()
            
            logger.info(f"Deformation field visualization saved: {filename}")
            
        except Exception as e:
            logger.error(f"Failed to visualize deformation field: {e}")

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
    超参数优化器，使用交叉验证
    """
    def __init__(self, cv_folds=5, random_state=42):
        self.cv_folds = cv_folds
        self.random_state = random_state
        self.best_params = None
        self.best_score = float('inf')
        self.cv_results = []
        
    def optimize_rbf_params(self, positions_A, positions_B, T_svd):
        """优化RBF模型参数"""
        logger.info("Starting RBF hyperparameter optimization with cross-validation...")
        
        # 定义参数网格
        param_grid = {
            'kernel': ['thin_plate_spline', 'multiquadric', 'inverse_multiquadric', 'gaussian'],
            'smoothing': [0.0, 0.0001, 0.001, 0.01, 0.1, 1.0]
        }
        
        # 生成所有参数组合
        param_combinations = list(itertools.product(param_grid['kernel'], param_grid['smoothing']))
        
        logger.info(f"Testing {len(param_combinations)} parameter combinations with {self.cv_folds}-fold CV")
        
        # 设置交叉验证
        kf = KFold(n_splits=self.cv_folds, shuffle=True, random_state=self.random_state)
        
        positions_A = np.array(positions_A)
        positions_B = np.array(positions_B)
        indices = np.arange(len(positions_A))
        
        best_params = None
        best_mean_rmse = float('inf')
        
        for kernel, smoothing in param_combinations:
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
                    
                    # 训练形变模型
                    model = NonRigidDeformationModel(method=kernel, smoothing=smoothing, constrain_y=True)
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
                        'kernel': kernel,
                        'smoothing': smoothing,
                        'mean_rmse': mean_rmse,
                        'std_rmse': std_rmse,
                        'fold_rmses': fold_rmses
                    })
                    
                    logger.info(f"  {kernel}, smoothing={smoothing:.6f}: "
                              f"RMSE={mean_rmse*1000:.3f}±{std_rmse*1000:.3f}mm")
                    
                    if mean_rmse < best_mean_rmse:
                        best_mean_rmse = mean_rmse
                        best_params = {'kernel': kernel, 'smoothing': smoothing}
                
            except Exception as e:
                logger.warning(f"Failed to evaluate {kernel}, smoothing={smoothing}: {e}")
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

class CoordinateTransformer:
    """
    Optimized two-stage coordinate transformation with hyperparameter tuning and outlier detection
    """
    def __init__(self, positions_A=None, positions_B=None):
        self.T_svd = None
        self.deformation_model = None
        self.best_model_type = None
        self.__positions_A = positions_A
        self.__positions_B = positions_B
        self.optimizer = HyperparameterOptimizer()
        self.outlier_detector = OutlierDetector(method='iqr', iqr_factor=2.5)  # 使用更温和的阈值
        self.residual_analyzer = ResidualAnalyzer()
        self.clean_indices = None
        
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

    def two_stage_registration_optimized(self):
        """Perform optimized two-stage registration with outlier detection and hyperparameter tuning"""
        if self.__positions_A is None or self.__positions_B is None:
            logger.error("Please provide position data first!")
            return None, None
        
        logger.info("=== Starting Optimized Two-Stage Registration with Outlier Detection ===")
        
        positions_A = np.array(self.__positions_A)
        positions_B = np.array(self.__positions_B)
        
        logger.info(f"Original dataset: {len(positions_A)} point pairs")
        
        # Step 0: Outlier Detection
        logger.info("Step 0: Outlier Detection")
        clean_mask = self.outlier_detector.detect_outliers_initial_svd(positions_A, positions_B)
        self.clean_indices = np.where(clean_mask)[0]
        
        # 使用清洁的数据集
        clean_positions_A = positions_A[clean_mask]
        clean_positions_B = positions_B[clean_mask]
        
        logger.info(f"Clean dataset: {len(clean_positions_A)} point pairs")
        logger.info(f"Removed {len(positions_A) - len(clean_positions_A)} outliers")
        
        # 保存清洁数据集信息
        clean_data_df = pd.DataFrame({
            'original_index': self.clean_indices,
            'clean_index': np.arange(len(clean_positions_A)),
            'source_x': clean_positions_A[:, 0],
            'source_z': clean_positions_A[:, 2],
            'target_x': clean_positions_B[:, 0],
            'target_z': clean_positions_B[:, 2]
        })
        clean_data_df.to_csv(os.path.join(result_dir, "clean_dataset.csv"), index=False)
        
        # Stage 1: SVD coarse registration on clean data
        logger.info("Stage 1: SVD Coarse Registration (Clean Data)")
        
        # 重新计算SVD在清洁数据上
        positions_A_xz = clean_positions_A[:, [0, 2]]
        positions_B_xz = clean_positions_B[:, [0, 2]]

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
        
        svd_transformed = self.apply_transformation(clean_positions_A, self.T_svd)
        svd_rmse_results = self.calculate_directional_rmse(clean_positions_B, svd_transformed)
        svd_performance = self.evaluate_performance(clean_positions_B, svd_transformed)
        
        logger.info(f"SVD Overall RMSE (clean data): {svd_rmse_results['overall_rmse']*1000:.3f}mm")
        
        residual_vectors = clean_positions_B - svd_transformed
        residual_magnitudes = np.sqrt(np.sum(residual_vectors**2, axis=1))
        logger.info(f"Residual vector statistics (clean data):")
        logger.info(f"  Mean magnitude: {np.mean(residual_magnitudes)*1000:.3f}mm")
        logger.info(f"  Max magnitude: {np.max(residual_magnitudes)*1000:.3f}mm")
        logger.info(f"  Std magnitude: {np.std(residual_magnitudes)*1000:.3f}mm")
        
        self.visualize_transformation_stage("SVD Coarse Registration (Clean)", 
                                           clean_positions_A, clean_positions_B, svd_transformed, 
                                           svd_rmse_results, residual_vectors)
        
        # Stage 2: Hyperparameter optimization and model selection
        logger.info("Stage 2: Hyperparameter Optimization and Model Selection (Clean Data)")
        
        # 优化RBF参数
        best_rbf_params = self.optimizer.optimize_rbf_params(clean_positions_A, clean_positions_B, self.T_svd)
        
        # 优化B样条参数
        best_bspline_params, bspline_results = self.optimizer.optimize_bspline_params(clean_positions_A, clean_positions_B, self.T_svd)
        
        # 保存CV结果
        self.optimizer.save_cv_results()
        
        # 选择最佳模型
        best_model_info = self._select_best_model(best_rbf_params, best_bspline_params, bspline_results)
        
        if best_model_info is None:
            logger.error("No suitable model found")
            return None, None
        
        # 使用最佳模型训练最终模型
        logger.info(f"Training final model: {best_model_info}")
        
        if best_model_info['type'] == 'rbf':
            self.deformation_model = NonRigidDeformationModel(
                method=best_model_info['params']['kernel'],
                smoothing=best_model_info['params']['smoothing'],
                constrain_y=True
            )
        else:  # bspline
            self.deformation_model = BSplineDeformationModel(
                knot_density=best_model_info['params']['knot_density'],
                smoothing_factor=best_model_info['params']['smoothing_factor'],
                constrain_y=True
            )
        
        self.best_model_type = best_model_info['type']
        
        # 在清洁数据上训练最终模型
        success = self.deformation_model.learn_deformation(svd_transformed, residual_vectors)
        
        if not success:
            logger.error("Failed to train final deformation model")
            return None, None
        
        # 应用最终模型
        predicted_deformation = self.deformation_model.predict_deformation(svd_transformed)
        final_transformed = svd_transformed + predicted_deformation
        final_transformed[:, 1] = 0
        
        # 评估最终结果
        final_rmse_results = self.calculate_directional_rmse(clean_positions_B, final_transformed)
        final_performance = self.evaluate_performance(clean_positions_B, final_transformed)
        
        logger.info(f"Final Overall RMSE (clean data): {final_rmse_results['overall_rmse']*1000:.3f}mm")
        
        final_residuals = clean_positions_B - final_transformed
        final_residual_magnitudes = np.sqrt(np.sum(final_residuals**2, axis=1))
        improvement = (np.mean(residual_magnitudes) - np.mean(final_residual_magnitudes))*1000
        logger.info(f"Final residual statistics (clean data):")
        logger.info(f"  Mean magnitude: {np.mean(final_residual_magnitudes)*1000:.3f}mm")
        logger.info(f"  Max magnitude: {np.max(final_residual_magnitudes)*1000:.3f}mm")
        logger.info(f"  Improvement: {improvement:.3f}mm")
        
        # 可视化最终结果
        self.visualize_transformation_stage(f"Final Optimized {best_model_info['type'].upper()} (Clean)", 
                                           clean_positions_A, clean_positions_B, final_transformed, 
                                           final_rmse_results, final_residuals)
        
        # 残差分析
        self.residual_analyzer.analyze_final_residuals(
            clean_positions_A, clean_positions_B, final_transformed, 
            f"Final {best_model_info['type'].upper()}"
        )
        
        # 可视化形变场（仅RBF支持）
        if best_model_info['type'] == 'rbf' and hasattr(self.deformation_model, 'visualize_deformation_field'):
            x_min, x_max = np.min(svd_transformed[:, 0]), np.max(svd_transformed[:, 0])
            z_min, z_max = np.min(svd_transformed[:, 2]), np.max(svd_transformed[:, 2])
            x_range = x_max - x_min
            z_range = z_max - z_min
            bounds = [
                x_min - 0.1 * x_range, x_max + 0.1 * x_range,
                z_min - 0.1 * z_range, z_max + 0.1 * z_range
            ]
            self.deformation_model.visualize_deformation_field(bounds, resolution=30)
        
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
        logger.info("\n=== Detailed Performance Analysis (Clean Data) ===")
        
        logger.info("SVD Stage Results:")
        svd_rmse = svd_performance['rmse_results']
        logger.info(f"  Lateral RMSE: {svd_rmse['lateral_rmse']*1000:.3f}mm")
        logger.info(f"  Longitudinal RMSE: {svd_rmse['longitudinal_rmse']*1000:.3f}mm")
        logger.info(f"  Overall RMSE: {svd_rmse['overall_rmse']*1000:.3f}mm")
        
        logger.info(f"Final (SVD + {best_model_info['type'].upper()}) Results:")
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
        
        logger.info(f"Best Model: {best_model_info['type'].upper()}")
        logger.info(f"Best Parameters: {best_model_info['params']}")
        
        # 离群点统计
        logger.info("Outlier Detection Summary:")
        logger.info(f"  Method: {self.outlier_detector.method}")
        logger.info(f"  Outliers removed: {self.outlier_detector.outlier_stats['outliers_detected']}")
        logger.info(f"  Outlier ratio: {self.outlier_detector.outlier_stats['outlier_ratio']*100:.2f}%")

    def _save_transformation_data(self):
        """Save transformation matrices and model data"""
        if self.T_svd is not None:
            np.savetxt(os.path.join(result_dir, "T_svd.txt"), self.T_svd)
        
        # 保存最佳模型信息
        model_info_file = os.path.join(result_dir, "best_model_info.txt")
        with open(model_info_file, "w") as f:
            f.write(f"Best Model Type: {self.best_model_type}\n")
            if hasattr(self.deformation_model, 'method'):
                f.write(f"RBF Kernel: {self.deformation_model.method}\n")
                f.write(f"RBF Smoothing: {self.deformation_model.smoothing}\n")
            elif hasattr(self.deformation_model, 'knot_density'):
                f.write(f"B-Spline Knot Density: {self.deformation_model.knot_density}\n")
                f.write(f"B-Spline Smoothing Factor: {self.deformation_model.smoothing_factor}\n")
        
        # 保存离群点检测信息
        outlier_info_file = os.path.join(result_dir, "outlier_detection_info.txt")
        with open(outlier_info_file, "w") as f:
            f.write("Outlier Detection Summary\n")
            f.write("=" * 40 + "\n")
            f.write(f"Method: {self.outlier_detector.method}\n")
            f.write(f"Total points: {self.outlier_detector.outlier_stats['total_points']}\n")
            f.write(f"Outliers detected: {self.outlier_detector.outlier_stats['outliers_detected']}\n")
            f.write(f"Outlier ratio: {self.outlier_detector.outlier_stats['outlier_ratio']*100:.2f}%\n")
            f.write(f"Threshold used: {self.outlier_detector.outlier_stats['threshold_used']*1000:.3f}mm\n")
            if len(self.outlier_detector.outlier_indices) > 0:
                f.write(f"Outlier indices: {self.outlier_detector.outlier_indices.tolist()}\n")
                f.write(f"Max outlier residual: {np.max(self.outlier_detector.outlier_stats['outlier_residuals'])*1000:.3f}mm\n")
        
        logger.info("Transformation data, model info, and outlier detection info saved")

def process_laser_tracker_data(input_file, output_file):
    """Process laser tracker data"""
    logger.info(f"Processing laser tracker data from {input_file}")
    
    try:
        df = pd.read_csv(input_file)
        
        df.rename(columns={"Distance_2": "d_laser_z", "Distance_3": "d_laser_x"}, inplace=True)
        
        df["d_laser_x"] = df["d_laser_x"] + 0.05
        df["d_laser_z"] = df["d_laser_z"] + 0.05
        
        def compute_corrected_distances(laser_x, laser_z, yaw):
            if yaw < 0:
                theta = np.radians(yaw + 137.4)
            else:
                theta = np.radians(yaw - 40.7)
        
            d_perp_x = laser_x * np.cos(theta)
            d_perp_z = laser_z * np.cos(theta)
        
            return round(d_perp_x, 6), round(d_perp_z, 6)
        
        corrected_data = [
            compute_corrected_distances(lx, lz, yaw)
            for lx, lz, yaw in zip(df["d_laser_x"], df["d_laser_z"], df["Yaw"])
        ]
        
        df["laser_x"], df["laser_z"] = zip(*corrected_data)
        
        df["laser_x"] = 4.250 - df["laser_x"]
        df["laser_z"] = 1.660 - df["laser_z"]
        
        condition1 = abs(df["Yaw"] + 137.4) <= 10
        condition2 = abs(df["Yaw"] - 40.7) <= 10
        
        df = df[condition1 | condition2]
        
        df = df[["Timestamp", "d_laser_x", "d_laser_z", "laser_x", "laser_z", "X", "Y", "Z", "Yaw", "Roll", "Pitch"]]
        df = df.round(6)
        
        df.to_csv(output_file, index=False, encoding="utf-8")
        
        logger.info(f"Corrected data saved to {output_file}")
        logger.info(f"Processed {len(df)} data points")
        
        return df
        
    except Exception as e:
        logger.error(f"Error processing laser tracker data: {e}")
        return None

def main():
    """Main function"""
    logger.info("Starting OPTIMIZED 1:32 scale model tracker data processing with OUTLIER DETECTION")
    logger.info(f"Scale factor: {SCALE_FACTOR}")
    logger.info(f"Target RMSE: {TARGET_RMSE*1000:.2f}mm")
    logger.info("Features: Outlier detection, Cross-validation, hyperparameter optimization, model selection, residual analysis")
    
    global LASER_TRACKER_CSV
    if len(sys.argv) > 1:
        LASER_TRACKER_CSV = sys.argv[1]
    
    if not os.path.exists(LASER_TRACKER_CSV):
        logger.error(f"Laser tracker data file {LASER_TRACKER_CSV} not found.")
        logger.info("Usage: python script.py <path_to_laser_tracker.csv>")
        return
    
    # Process laser tracker data
    logger.info("Processing laser tracker data...")
    df = process_laser_tracker_data(LASER_TRACKER_CSV, CORRECTED_LASER_TRACKER_CSV)
    
    if df is None:
        logger.error("Failed to process laser tracker data")
        return
    
    # Prepare data for transformation
    positions_A = [[row[ROW_X], 0, row[ROW_Y]] for _, row in df.iterrows()]
    positions_B = [[row["laser_x"], 0, row["laser_z"]] for _, row in df.iterrows()]
    
    logger.info(f"Loaded {len(positions_A)} point pairs for registration")
    
    # Perform optimized two-stage registration with outlier detection
    transformer = CoordinateTransformer(positions_A, positions_B)
    T_svd, final_rmse_results = transformer.two_stage_registration_optimized()
    
    if T_svd is not None and final_rmse_results is not None:
        success = final_rmse_results['overall_rmse'] <= TARGET_RMSE
        if success:
            logger.info(f"SUCCESS: Optimized registration with outlier detection completed with RMSE {final_rmse_results['overall_rmse']*1000:.3f}mm")
        else:
            logger.info(f"PARTIAL: Optimized registration completed, but RMSE {final_rmse_results['overall_rmse']*1000:.3f}mm exceeds target {TARGET_RMSE*1000:.2f}mm")
        
        # Save final summary
        summary_file = os.path.join(output_dir, "registration_summary.txt")
        with open(summary_file, "w") as f:
            f.write("Optimized Two-Stage Registration with Outlier Detection Summary\n")
            f.write("=" * 80 + "\n")
            f.write(f"Scale Factor: {SCALE_FACTOR}\n")
            f.write(f"Target RMSE: {TARGET_RMSE*1000:.2f}mm\n")
            f.write(f"Final RMSE: {final_rmse_results['overall_rmse']*1000:.3f}mm\n")
            f.write(f"Success: {'Yes' if success else 'No'}\n")
            f.write(f"Original points: {len(positions_A)}\n")
            f.write(f"Clean points used: {len(transformer.clean_indices)}\n")
            f.write(f"Outliers removed: {len(positions_A) - len(transformer.clean_indices)}\n")
            f.write(f"Outlier ratio: {(len(positions_A) - len(transformer.clean_indices))/len(positions_A)*100:.2f}%\n")
            f.write(f"Real-world equivalent: {final_rmse_results['overall_rmse']*SCALE_FACTOR*100:.2f}cm\n")
            f.write(f"Best model type: {transformer.best_model_type}\n")
            f.write(f"Outlier detection method: {transformer.outlier_detector.method}\n")
            f.write("\nOptimization features:\n")
            f.write("- Statistical outlier detection (IQR method)\n")
            f.write("- 5-fold cross-validation\n")
            f.write("- RBF hyperparameter grid search\n")
            f.write("- B-Spline parameter optimization (with corrected smoothing)\n")
            f.write("- Automatic model selection\n")
            f.write("- Comprehensive residual analysis\n")
    else:
        logger.error("Optimized registration failed")
    
    logger.info("Processing complete")

if __name__ == "__main__":
    main()