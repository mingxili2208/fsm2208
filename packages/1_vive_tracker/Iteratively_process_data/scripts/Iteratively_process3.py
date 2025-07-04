#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Tracker data processing script with ICP algorithm and scaled RMSE requirements
"""
import sys
import os
import datetime
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from scipy.spatial.distance import cdist
from scipy.optimize import minimize

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
output_dir = os.path.join(result_dir, f"tdps_icp_{timestamp}_1to32_scale_RMSE_{TARGET_RMSE*1000:.1f}mm")
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
log_file = os.path.join(log_dir, "tracker_process_icp.log")
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

class ICPTransformer:
    """
    ICP-based coordinate transformer with directional RMSE analysis.
    """
    def __init__(self, source_points=None, target_points=None):
        self.T_matrix = None
        self.source_points = None
        self.target_points = None
        self.transformation_history = []
        
        # ICP参数
        self.max_iterations = 50
        self.tolerance = 1e-6
        self.max_correspondence_distance = 0.1  # 增加到10cm maximum correspondence distance
        
        if source_points is not None and target_points is not None:
            self.set_point_clouds(source_points, target_points)

    def set_point_clouds(self, source_points, target_points):
        """设置点云数据"""
        self.source_points = np.array(source_points, dtype=np.float64)
        self.target_points = np.array(target_points, dtype=np.float64)
        
        logger.info(f"Source points shape: {self.source_points.shape}")
        logger.info(f"Target points shape: {self.target_points.shape}")
        logger.info(f"Source points range - X: [{self.source_points[:, 0].min():.3f}, {self.source_points[:, 0].max():.3f}]")
        logger.info(f"Source points range - Z: [{self.source_points[:, 2].min():.3f}, {self.source_points[:, 2].max():.3f}]")
        logger.info(f"Target points range - X: [{self.target_points[:, 0].min():.3f}, {self.target_points[:, 0].max():.3f}]")
        logger.info(f"Target points range - Z: [{self.target_points[:, 2].min():.3f}, {self.target_points[:, 2].max():.3f}]")

    @staticmethod
    def construct_transformation_matrix(rotation_matrix, translation):
        """构建4x4变换矩阵"""
        T = np.eye(4)
        T[:3, :3] = rotation_matrix
        T[:3, 3] = translation
        return T

    def find_closest_points(self, source_transformed, target):
        """找到最近点对应关系"""
        try:
            distances = cdist(source_transformed, target)
            closest_indices = np.argmin(distances, axis=1)
            closest_distances = np.min(distances, axis=1)
            
            # 过滤距离过大的点对
            valid_mask = closest_distances < self.max_correspondence_distance
            valid_source_indices = np.where(valid_mask)[0]
            valid_target_indices = closest_indices[valid_mask]
            
            logger.debug(f"Valid correspondences: {len(valid_source_indices)}/{len(source_transformed)} "
                        f"(threshold: {self.max_correspondence_distance:.3f}m)")
            
            return valid_source_indices, valid_target_indices, closest_distances[valid_mask]
        except Exception as e:
            logger.error(f"Error in find_closest_points: {e}")
            return np.array([]), np.array([]), np.array([])

    def calculate_transformation_svd(self, source_points, target_points):
        """使用SVD计算变换矩阵"""
        try:
            if len(source_points) < 3 or len(target_points) < 3:
                logger.error(f"Insufficient points for SVD: source={len(source_points)}, target={len(target_points)}")
                return None, None
            
            # 计算质心
            source_centroid = np.mean(source_points, axis=0)
            target_centroid = np.mean(target_points, axis=0)
            
            # 去中心化
            source_centered = source_points - source_centroid
            target_centered = target_points - target_centroid
            
            # 计算协方差矩阵
            H = source_centered.T @ target_centered
            
            # SVD分解
            U, S, Vt = np.linalg.svd(H)
            R = Vt.T @ U.T
            
            # 确保旋转矩阵的行列式为正
            if np.linalg.det(R) < 0:
                Vt[-1, :] *= -1
                R = Vt.T @ U.T
            
            # 计算平移
            t = target_centroid - R @ source_centroid
            
            return R, t
        except Exception as e:
            logger.error(f"Error in calculate_transformation_svd: {e}")
            return None, None

    def apply_transformation(self, points, R, t):
        """应用变换到点集"""
        try:
            return (R @ points.T).T + t
        except Exception as e:
            logger.error(f"Error in apply_transformation: {e}")
            return points

    def calculate_initial_alignment(self):
        """计算初始粗对齐"""
        try:
            # 使用质心对齐作为初始变换
            source_centroid = np.mean(self.source_points, axis=0)
            target_centroid = np.mean(self.target_points, axis=0)
            
            initial_translation = target_centroid - source_centroid
            initial_rotation = np.eye(3)
            
            logger.info(f"Initial translation: {initial_translation}")
            
            return self.construct_transformation_matrix(initial_rotation, initial_translation)
        except Exception as e:
            logger.error(f"Error in calculate_initial_alignment: {e}")
            return np.eye(4)

    def icp_registration(self, source_points=None, target_points=None, initial_transform=None):
        """ICP配准主函数"""
        try:
            if source_points is not None and target_points is not None:
                self.set_point_clouds(source_points, target_points)
            
            if self.source_points is None or self.target_points is None:
                logger.error("Source and target points must be provided!")
                return None
            
            source = self.source_points.copy()
            target = self.target_points.copy()
            
            logger.info(f"Starting ICP registration with {len(source)} source points and {len(target)} target points")
            
            # 计算初始对齐
            if initial_transform is None:
                initial_transform = self.calculate_initial_alignment()
            
            R_current = initial_transform[:3, :3]
            t_current = initial_transform[:3, 3]
            
            prev_error = float('inf')
            self.transformation_history = []
            
            for iteration in range(self.max_iterations):
                # 应用当前变换
                source_transformed = self.apply_transformation(source, R_current, t_current)
                
                # 找到最近点对应
                source_indices, target_indices, distances = self.find_closest_points(source_transformed, target)
                
                if len(source_indices) < 3:
                    logger.warning(f"Too few valid correspondences ({len(source_indices)}) at iteration {iteration}")
                    if iteration == 0:
                        logger.error("ICP failed at first iteration - no valid correspondences found")
                        return None
                    break
                
                # 计算当前误差
                current_error = np.mean(distances)
                
                logger.info(f"ICP Iteration {iteration}: Error = {current_error*1000:.3f}mm, "
                           f"Valid correspondences = {len(source_indices)}")
                
                # 记录变换历史
                T_current = self.construct_transformation_matrix(R_current, t_current)
                self.transformation_history.append({
                    'iteration': iteration,
                    'error': current_error,
                    'correspondences': len(source_indices),
                    'transformation': T_current.copy()
                })
                
                # 检查收敛
                if iteration > 0 and abs(prev_error - current_error) < self.tolerance:
                    logger.info(f"ICP converged after {iteration} iterations with error {current_error*1000:.3f}mm")
                    break
                
                # 计算增量变换
                source_subset = source_transformed[source_indices]
                target_subset = target[target_indices]
                
                R_increment, t_increment = self.calculate_transformation_svd(source_subset, target_subset)
                
                if R_increment is None or t_increment is None:
                    logger.warning(f"SVD calculation failed at iteration {iteration}")
                    break
                
                # 更新累积变换
                t_current = R_increment @ t_current + t_increment
                R_current = R_increment @ R_current
                
                prev_error = current_error
            
            # 保存最终变换矩阵
            self.T_matrix = self.construct_transformation_matrix(R_current, t_current)
            
            logger.info(f"ICP registration completed. Final error: {current_error*1000:.3f}mm")
            
            return self.T_matrix
            
        except Exception as e:
            logger.error(f"Error in ICP registration: {e}")
            import traceback
            logger.error(traceback.format_exc())
            return None

    def apply_transformation_matrix(self, points):
        """使用变换矩阵变换点集"""
        if self.T_matrix is None:
            logger.error("No transformation matrix available! Run ICP registration first.")
            return None
        
        try:
            points = np.array(points)
            points_homogeneous = np.column_stack([points, np.ones(len(points))])
            transformed_homogeneous = (self.T_matrix @ points_homogeneous.T).T
            return transformed_homogeneous[:, :3].round(6)
        except Exception as e:
            logger.error(f"Error in apply_transformation_matrix: {e}")
            return None

    def calculate_directional_rmse(self, actual, predicted):
        """Calculate RMSE for each direction separately"""
        try:
            actual = np.array(actual)
            predicted = np.array(predicted)
            
            # 计算各方向的RMSE
            rmse_lateral = np.sqrt(np.mean((actual[:, 0] - predicted[:, 0])**2))    # X方向 (横向)
            rmse_vertical = np.sqrt(np.mean((actual[:, 1] - predicted[:, 1])**2))   # Y方向 (垂直)
            rmse_longitudinal = np.sqrt(np.mean((actual[:, 2] - predicted[:, 2])**2)) # Z方向 (纵向)
            
            # 综合RMSE
            overall_rmse = np.sqrt(rmse_lateral**2 + rmse_vertical**2 + rmse_longitudinal**2)
            
            # 主要关注的水平面RMSE (X-Z平面，忽略Y)
            horizontal_rmse = np.sqrt(rmse_lateral**2 + rmse_longitudinal**2)
            
            return {
                'lateral_rmse': rmse_lateral,
                'vertical_rmse': rmse_vertical,
                'longitudinal_rmse': rmse_longitudinal,
                'horizontal_rmse': horizontal_rmse,
                'overall_rmse': overall_rmse
            }
        except Exception as e:
            logger.error(f"Error in calculate_directional_rmse: {e}")
            return None

    def calculate_individual_position_errors(self, actual, predicted):
        """计算每个点的位置误差"""
        try:
            return [np.linalg.norm(np.array(actual[i]) - np.array(predicted[i])) for i in range(len(actual))]
        except Exception as e:
            logger.error(f"Error in calculate_individual_position_errors: {e}")
            return []

    def evaluate_scaled_performance(self, actual, predicted):
        """Evaluate performance against 1:32 scaled targets"""
        try:
            rmse_results = self.calculate_directional_rmse(actual, predicted)
            if rmse_results is None:
                return None
            
            # 评估是否满足缩放后的目标
            lateral_pass = rmse_results['lateral_rmse'] <= SCALED_LATERAL_TARGET
            longitudinal_pass = rmse_results['longitudinal_rmse'] <= SCALED_LONGITUDINAL_TARGET
            horizontal_pass = rmse_results['horizontal_rmse'] <= SCALED_COMBINED_TARGET
            overall_pass = rmse_results['overall_rmse'] <= TARGET_RMSE
            
            # 计算相对于真实世界的等效精度
            real_world_equivalent = {
                'lateral_real_world_eq': rmse_results['lateral_rmse'] * SCALE_FACTOR,
                'longitudinal_real_world_eq': rmse_results['longitudinal_rmse'] * SCALE_FACTOR,
                'horizontal_real_world_eq': rmse_results['horizontal_rmse'] * SCALE_FACTOR,
                'overall_real_world_eq': rmse_results['overall_rmse'] * SCALE_FACTOR
            }
            
            return {
                'rmse_results': rmse_results,
                'targets': {
                    'lateral_target': SCALED_LATERAL_TARGET,
                    'longitudinal_target': SCALED_LONGITUDINAL_TARGET,
                    'horizontal_target': SCALED_COMBINED_TARGET,
                    'overall_target': TARGET_RMSE
                },
                'pass_criteria': {
                    'lateral_pass': lateral_pass,
                    'longitudinal_pass': longitudinal_pass,
                    'horizontal_pass': horizontal_pass,
                    'overall_pass': overall_pass
                },
                'real_world_equivalent': real_world_equivalent
            }
        except Exception as e:
            logger.error(f"Error in evaluate_scaled_performance: {e}")
            return None

    def calculate_and_print_errors(self, target_points=None, transformed_points=None):
        """Calculate and print detailed errors"""
        results = {}
        
        try:
            if target_points is not None and transformed_points is not None:
                # 详细的方向性RMSE分析
                performance = self.evaluate_scaled_performance(target_points, transformed_points)
                
                if performance is None:
                    logger.error("Failed to evaluate performance")
                    return results
                
                logger.info("=== ICP 1:32 Scale Model Performance Analysis ===")
                logger.info(f"Scale Factor: {SCALE_FACTOR}")
                logger.info(f"Real-world target (local road): {REAL_WORLD_LATERAL_TARGET:.3f}m lateral, {REAL_WORLD_LONGITUDINAL_TARGET:.3f}m longitudinal")
                logger.info("")
                
                logger.info("=== Scaled Model Targets ===")
                logger.info(f"Lateral target: {SCALED_LATERAL_TARGET*1000:.2f}mm")
                logger.info(f"Longitudinal target: {SCALED_LONGITUDINAL_TARGET*1000:.2f}mm")
                logger.info(f"Horizontal combined target: {SCALED_COMBINED_TARGET*1000:.2f}mm")
                logger.info(f"Overall target: {TARGET_RMSE*1000:.2f}mm")
                logger.info("")
                
                logger.info("=== ICP Registration Results ===")
                rmse_results = performance['rmse_results']
                logger.info(f"Lateral RMSE: {rmse_results['lateral_rmse']*1000:.3f}mm")
                logger.info(f"Longitudinal RMSE: {rmse_results['longitudinal_rmse']*1000:.3f}mm")
                logger.info(f"Vertical RMSE: {rmse_results['vertical_rmse']*1000:.3f}mm")
                logger.info(f"Horizontal RMSE: {rmse_results['horizontal_rmse']*1000:.3f}mm")
                logger.info(f"Overall RMSE: {rmse_results['overall_rmse']*1000:.3f}mm")
                logger.info("")
                
                logger.info("=== Real-world Equivalent Performance ===")
                rw_eq = performance['real_world_equivalent']
                logger.info(f"Lateral equivalent: {rw_eq['lateral_real_world_eq']*100:.2f}cm")
                logger.info(f"Longitudinal equivalent: {rw_eq['longitudinal_real_world_eq']*100:.2f}cm")
                logger.info(f"Horizontal equivalent: {rw_eq['horizontal_real_world_eq']*100:.2f}cm")
                logger.info(f"Overall equivalent: {rw_eq['overall_real_world_eq']*100:.2f}cm")
                logger.info("")
                
                logger.info("=== Target Achievement ===")
                pass_criteria = performance['pass_criteria']
                logger.info(f"Lateral target achieved: {'✓' if pass_criteria['lateral_pass'] else '✗'}")
                logger.info(f"Longitudinal target achieved: {'✓' if pass_criteria['longitudinal_pass'] else '✗'}")
                logger.info(f"Horizontal target achieved: {'✓' if pass_criteria['horizontal_pass'] else '✗'}")
                logger.info(f"Overall target achieved: {'✓' if pass_criteria['overall_pass'] else '✗'}")
                logger.info("")
                
                # ICP收敛信息
                if self.transformation_history:
                    logger.info("=== ICP Convergence History ===")
                    for step in self.transformation_history[-5:]:  # 显示最后5次迭代
                        logger.info(f"Iteration {step['iteration']}: Error = {step['error']*1000:.3f}mm, "
                                   f"Correspondences = {step['correspondences']}")
                
                # 计算个体误差
                individual_errors = self.calculate_individual_position_errors(target_points, transformed_points)
                for i, error in enumerate(individual_errors[:10]):  # 只显示前10个点
                    logger.info(f"Point {i} position error: {error*1000:.3f}mm")
                
                results['individual_position_errors'] = individual_errors
                results['performance_analysis'] = performance
                results['convergence_history'] = self.transformation_history
        
        except Exception as e:
            logger.error(f"Error in calculate_and_print_errors: {e}")
            import traceback
            logger.error(traceback.format_exc())
        
        return results

    def demo_icp_transformation(self):
        """ICP变换演示函数"""
        try:
            if self.source_points is None or self.target_points is None:
                logger.error("Please provide source and target points first!")
                return None, None
            
            # 执行ICP配准
            transformation_matrix = self.icp_registration()
            
            if transformation_matrix is None:
                logger.error("ICP registration failed!")
                return None, None
            
            # 应用变换
            transformed_points = self.apply_transformation_matrix(self.source_points)
            
            if transformed_points is None:
                logger.error("Failed to apply transformation!")
                return None, None
            
            logger.info("ICP transformation matrix:")
            logger.info(transformation_matrix)
            
            logger.info("Verifying ICP transformation results:")
            for i in range(min(5, len(self.source_points))):  # 只显示前5个点
                logger.info(f"Point {i} target position: {self.target_points[i]}")
                logger.info(f"Point {i} ICP transformed position: {transformed_points[i]}")
            
            # 计算误差
            error_results = self.calculate_and_print_errors(self.target_points, transformed_points)
            
            return transformed_points, error_results
            
        except Exception as e:
            logger.error(f"Error in demo_icp_transformation: {e}")
            import traceback
            logger.error(traceback.format_exc())
            return None, None

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

def calculate_transformations_iterative_icp(max_iterations=10, target_rmse=None):
    """Iterative ICP transformation calculation"""
    if target_rmse is None:
        target_rmse = TARGET_RMSE
        
    logger.info(f"Starting iterative ICP transformation calculation")
    logger.info(f"1:32 Scale Factor Applied")
    logger.info(f"Target RMSE: {target_rmse*1000:.2f}mm (equivalent to {target_rmse*SCALE_FACTOR*100:.1f}cm in real world)")
    
    input_tracker_csv = CORRECTED_LASER_TRACKER_CSV
    current_rmse = float('inf')
    iteration = 1
    T_matrix = None  # 初始化变量
    
    while current_rmse > target_rmse and iteration <= max_iterations:
        logger.info(f"\n--- ICP Iteration {iteration} ---")
        
        try:
            tracker_laser_df = pd.read_csv(input_tracker_csv)
            logger.info(f"Loaded tracker data with {len(tracker_laser_df)} points")
            
            # 准备点云数据
            source_points = [[row[ROW_X], 0, row[ROW_Y]] for _, row in tracker_laser_df.iterrows()]
            target_points = [[row["laser_x"], 0, row["laser_z"]] for _, row in tracker_laser_df.iterrows()]
            
            # 创建ICP变换器
            icp_transformer = ICPTransformer(source_points, target_points)
            
            # 执行ICP配准
            transformed_points, error_results = icp_transformer.demo_icp_transformation()
            
            if transformed_points is None or error_results is None:
                logger.error("ICP transformation failed!")
                break
            
            # 获取当前RMSE
            if 'performance_analysis' in error_results and error_results['performance_analysis'] is not None:
                current_rmse = error_results['performance_analysis']['rmse_results']['overall_rmse']
            else:
                logger.error("Performance analysis missing or failed!")
                break
            
            logger.info(f"Current Overall RMSE: {current_rmse*1000:.3f}mm")
            
            T_matrix = icp_transformer.T_matrix
            
            if current_rmse <= target_rmse:
                logger.info(f"Target RMSE of {target_rmse*1000:.2f}mm achieved! Stopping iterations.")
                break
            
            # 只有在有足够点的情况下才进行过滤
            if len(tracker_laser_df) <= 20:
                logger.warning("Too few points to filter further. Stopping iterations.")
                break
            
            # 使用动态阈值进行过滤
            error_threshold = max(0.9 * current_rmse, target_rmse * 1.5)
            logger.info(f"Filtering points with error > {error_threshold*1000:.3f}mm")
            
            # 计算个体误差
            individual_errors = icp_transformer.calculate_individual_position_errors(target_points, transformed_points)
            high_error_indices = [i for i, error in enumerate(individual_errors) if error > error_threshold]
            low_error_indices = [i for i, error in enumerate(individual_errors) if error <= error_threshold]
            
            logger.info(f"Removing {len(high_error_indices)} high-error points out of {len(individual_errors)} total points")
            logger.info(f"Keeping {len(low_error_indices)} points for next iteration")
            
            # 过滤数据
            filtered_df = tracker_laser_df.iloc[low_error_indices].copy()
            
            # 保存中间结果
            filtered_csv_path = os.path.join(result_dir, f"filtered_tracker_icp_iter_{iteration}.csv")
            filtered_df.to_csv(filtered_csv_path, index=False)
            
            if T_matrix is not None:
                np.savetxt(os.path.join(result_dir, f"T_matrix_icp_iter_{iteration}.txt"), T_matrix)
            
            # 保存ICP收敛历史
            if icp_transformer.transformation_history:
                history_file = os.path.join(result_dir, f"icp_convergence_iter_{iteration}.txt")
                with open(history_file, 'w') as f:
                    f.write("ICP Convergence History\n")
                    f.write("Iteration\tError(mm)\tCorrespondences\n")
                    for step in icp_transformer.transformation_history:
                        f.write(f"{step['iteration']}\t{step['error']*1000:.3f}\t{step['correspondences']}\n")
            
            input_tracker_csv = filtered_csv_path
            iteration += 1
            
            if len(filtered_df) < 10:
                logger.warning("Too few points remaining after filtering. Stopping iterations.")
                break
                
        except Exception as e:
            logger.error(f"Error in ICP iteration {iteration}: {e}")
            import traceback
            logger.error(traceback.format_exc())
            break
    
    # Final results
    logger.info("\n=== Final ICP Results ===")
    
    if T_matrix is not None and not np.isinf(current_rmse):
        logger.info(f"Final Overall RMSE: {current_rmse*1000:.3f}mm")
        logger.info(f"Real-world equivalent: {current_rmse*SCALE_FACTOR*100:.2f}cm")
    else:
        logger.error("ICP failed to produce valid results")
        current_rmse = float('inf')
    
    logger.info(f"ICP iterations performed: {iteration - 1}")
    
    # Save final results
    final_output_path = os.path.join(output_dir, "final_icp_transformation_matrix.txt")
    try:
        with open(final_output_path, "w") as f:
            f.write(f"ICP 1:32 Scale Model Results\n")
            f.write(f"============================\n")
            f.write(f"Scale Factor: {SCALE_FACTOR}\n")
            if not np.isinf(current_rmse):
                f.write(f"Final RMSE: {current_rmse*1000:.3f}mm\n")
                f.write(f"Real-world equivalent: {current_rmse*SCALE_FACTOR*100:.2f}cm\n")
                f.write(f"Target achieved: {'Yes' if current_rmse <= target_rmse else 'No'}\n")
            else:
                f.write(f"Final RMSE: Failed\n")
                f.write(f"Target achieved: No\n")
            f.write(f"Target RMSE: {target_rmse*1000:.2f}mm\n")
            f.write(f"ICP iterations performed: {iteration - 1}\n")
            f.write(f"ICP Transformation Matrix:\n")
            if T_matrix is not None:
                f.write(str(T_matrix.tolist()) + "\n\n")
            else:
                f.write("No valid transformation matrix produced\n\n")
        
        logger.info(f"Final ICP results saved to {final_output_path}")
    except Exception as e:
        logger.error(f"Error saving final results: {e}")
    
    return T_matrix, current_rmse

def main():
    """Main function"""
    logger.info("Starting ICP-based 1:32 scale model tracker data processing")
    logger.info(f"Scale factor: {SCALE_FACTOR}")
    logger.info(f"Target RMSE: {TARGET_RMSE*1000:.2f}mm (real-world equivalent: {TARGET_RMSE*SCALE_FACTOR*100:.2f}cm)")
    
    global LASER_TRACKER_CSV
    if len(sys.argv) > 1:
        LASER_TRACKER_CSV = sys.argv[1]
    
    if os.path.exists(LASER_TRACKER_CSV):
        logger.info("Processing laser tracker data...")
        process_laser_tracker_data(LASER_TRACKER_CSV, CORRECTED_LASER_TRACKER_CSV)
        
        if os.path.exists(CORRECTED_LASER_TRACKER_CSV):
            logger.info("Calculating ICP transformation matrix iteratively...")
            
            T_matrix, final_rmse = calculate_transformations_iterative_icp(
                max_iterations=10, 
                target_rmse=TARGET_RMSE
            )
            
            if T_matrix is not None and not np.isinf(final_rmse):
                if final_rmse <= TARGET_RMSE:
                    logger.info(f"SUCCESS: ICP transformation calculation completed with RMSE {final_rmse*1000:.3f}mm (target: {TARGET_RMSE*1000:.2f}mm)")
                else:
                    logger.info(f"PARTIAL: ICP transformation calculation completed, but RMSE {final_rmse*1000:.3f}mm exceeds target {TARGET_RMSE*1000:.2f}mm")
            else:
                logger.error("ICP transformation calculation failed")
        else:
            logger.error(f"Missing corrected laser tracker CSV: {CORRECTED_LASER_TRACKER_CSV}")
    else:
        logger.error(f"Laser tracker data file {LASER_TRACKER_CSV} not found.")
        logger.info("Usage: python script.py <path_to_laser_tracker.csv>")
    
    logger.info("ICP processing complete")

if __name__ == "__main__":
    main()