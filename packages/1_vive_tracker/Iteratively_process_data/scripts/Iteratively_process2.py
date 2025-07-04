#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Tracker data processing script with scaled RMSE requirements
"""
import sys
import os
import datetime
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# Create timestamp for output directory
current_time = datetime.datetime.now()
timestamp = current_time.strftime("%Y-%m-%d %H:%M:%S")

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
output_dir = os.path.join(result_dir, f"tdps_{timestamp}_1to32_scale_RMSE_{TARGET_RMSE*1000:.1f}mm")
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

class Point:
    """
    Represents a point in 3D space with position only.
    """
    def __init__(self, position=None):
        if position is not None:
            self.position = np.array(position)
        else:
            self.position = None

class CoordinateTransformer:
    """
    Class for calculating and applying coordinate transformations with directional RMSE analysis.
    """
    def __init__(self, positions_A=None, positions_B=None):
        self.T_pos = None
        self.__positions_A = positions_A
        self.__positions_B = positions_B

        if positions_A is not None and positions_B is not None:
            self.calculate_position_transformation(positions_A, positions_B)

    @staticmethod
    def construct_transformation_matrix(rotation_matrix, translation):
        T = np.eye(4)
        T[:3, :3] = rotation_matrix
        T[:3, 3] = translation
        return T

    def calculate_position_transformation(self, positions_A=None, positions_B=None):
        if positions_A is not None and positions_B is not None:
            self.__positions_A = positions_A
            self.__positions_B = positions_B
        elif self.__positions_A is None or self.__positions_B is None:
            logger.error("Please provide position data first!")
            return

        centroid_A = np.mean(self.__positions_A, axis=0)
        centroid_B = np.mean(self.__positions_B, axis=0)

        H = np.dot((self.__positions_A - centroid_A).T, (self.__positions_B - centroid_B))
        U, S, Vt = np.linalg.svd(H)
        R_pos = np.dot(Vt.T, U.T)
        if np.linalg.det(R_pos) < 0:
            Vt[-1, :] *= -1
            R_pos = np.dot(Vt.T, U.T)
        translation = centroid_B.T - np.dot(R_pos, centroid_A.T)
        self.T_pos = self.construct_transformation_matrix(R_pos, translation)
        return R_pos, translation

    def apply_position_transformation(self, position):
        position_homogeneous = np.append(position, 1)
        transformed_position_homogeneous = np.dot(self.T_pos, position_homogeneous)
        return transformed_position_homogeneous[:3].round(6)  # 增加精度到微米级

    def calculate_directional_rmse(self, actual, predicted):
        """
        Calculate RMSE for each direction separately, scaled for 1:32 model
        """
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

    def calculate_position_error(self, actual, predicted):
        """
        Calculate overall RMSE (backward compatibility)
        """
        return self.calculate_directional_rmse(actual, predicted)['overall_rmse']

    def calculate_individual_position_errors(self, actual, predicted):
        return [np.linalg.norm(np.array(actual[i]) - np.array(predicted[i])) for i in range(len(actual))]

    def evaluate_scaled_performance(self, actual, predicted):
        """
        Evaluate performance against 1:32 scaled targets
        """
        rmse_results = self.calculate_directional_rmse(actual, predicted)
        
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

    def calculate_and_print_errors(self, positions_B=None, transformed_positions=None):
        """
        Calculate and print detailed directional errors with scaling analysis
        """
        results = {}
        
        if positions_B is not None and transformed_positions is not None:
            # 详细的方向性RMSE分析
            performance = self.evaluate_scaled_performance(positions_B, transformed_positions)
            
            logger.info("=== 1:32 Scale Model Performance Analysis ===")
            logger.info(f"Scale Factor: {SCALE_FACTOR}")
            logger.info(f"Real-world target (local road): {REAL_WORLD_LATERAL_TARGET:.3f}m lateral, {REAL_WORLD_LONGITUDINAL_TARGET:.3f}m longitudinal")
            logger.info("")
            
            logger.info("=== Scaled Model Targets ===")
            logger.info(f"Lateral target: {SCALED_LATERAL_TARGET*1000:.2f}mm")
            logger.info(f"Longitudinal target: {SCALED_LONGITUDINAL_TARGET*1000:.2f}mm")
            logger.info(f"Horizontal combined target: {SCALED_COMBINED_TARGET*1000:.2f}mm")
            logger.info(f"Overall target: {TARGET_RMSE*1000:.2f}mm")
            logger.info("")
            
            logger.info("=== Actual Performance ===")
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
            
            # 计算个体误差
            individual_errors = []
            for i in range(len(positions_B)):
                pos_error = np.linalg.norm(np.array(positions_B[i]) - np.array(transformed_positions[i]))
                logger.info(f"Point {i} position error: {pos_error*1000:.3f}mm")
                individual_errors.append(pos_error)
            
            results['individual_position_errors'] = individual_errors
            results['performance_analysis'] = performance
        
        return results

    def demo_transformation(self):
        if self.__positions_A is None or self.__positions_B is None:
            logger.error("Please provide position data first!")
            return

        self.calculate_position_transformation()

        logger.info("Position transformation matrix:")
        logger.info(self.T_pos)

        transformed_positions = [self.apply_position_transformation(position) for position in self.__positions_A]

        logger.info("Verify transformation results:")
        for i in range(len(self.__positions_A)):
            logger.info(f"Point A {i} actual position in coordinate system B: {self.__positions_B[i]}")
            logger.info(f"Point A {i} transformed position in coordinate system B: {transformed_positions[i]}")

        error_results = self.calculate_and_print_errors(
            self.__positions_B, transformed_positions
        )
        
        return transformed_positions, error_results

    def get_transformation_matrix(self):
        return self.T_pos
    
    def visualize_registration(self, error_threshold=None):
        """
        Visualize registration with scaled error threshold
        """
        if error_threshold is None:
            error_threshold = TARGET_RMSE  # 使用缩放后的目标作为默认阈值
            
        transformed_positions = [self.apply_position_transformation(position) for position in self.__positions_A]
        errors = self.calculate_individual_position_errors(self.__positions_B, transformed_positions)
        
        high_error_indices = [i for i, error in enumerate(errors) if error > error_threshold]
        low_error_indices = [i for i, error in enumerate(errors) if error <= error_threshold]
        
        # Create 3D plot
        fig = plt.figure(figsize=(15, 12))
        ax = fig.add_subplot(111, projection='3d')
        
        ax.scatter([p[0] for p in self.__positions_A], [p[2] for p in self.__positions_A], [p[1] for p in self.__positions_A], 
                   c='blue', marker='o', s=50, label='Source (A)', alpha=0.7)
        
        ax.scatter([p[0] for p in self.__positions_B], [p[2] for p in self.__positions_B], [p[1] for p in self.__positions_B], 
                   c='red', marker='^', s=50, label='Target (B)', alpha=0.7)
        
        ax.scatter([p[0] for p in transformed_positions], [p[2] for p in transformed_positions], [p[1] for p in transformed_positions], 
                   c='green', marker='x', s=50, label='Transformed A', alpha=0.7)
        
        if high_error_indices:
            high_error_positions_B = [self.__positions_B[i] for i in high_error_indices]
            high_error_transformed = [transformed_positions[i] for i in high_error_indices]
            
            ax.scatter([p[0] for p in high_error_positions_B], [p[2] for p in high_error_positions_B], [p[1] for p in high_error_positions_B], 
                       c='magenta', marker='o', s=100, label=f'Target points with error > {error_threshold*1000:.1f}mm')
            
            ax.scatter([p[0] for p in high_error_transformed], [p[2] for p in high_error_transformed], [p[1] for p in high_error_transformed], 
                       c='orange', marker='x', s=100, label=f'Transformed points with error > {error_threshold*1000:.1f}mm')
            
            for i, idx in enumerate(high_error_indices):
                ax.plot([high_error_positions_B[i][0], high_error_transformed[i][0]],
                        [high_error_positions_B[i][2], high_error_transformed[i][2]],
                        [high_error_positions_B[i][1], high_error_transformed[i][1]],
                        'k--', alpha=0.3)
        
        ax.set_xlabel('X (Lateral)')
        ax.set_ylabel('Z (Longitudinal)')
        ax.set_zlabel('Y (Vertical)')
        ax.set_title(f'1:32 Scale Model Registration Results\nError Threshold: {error_threshold*1000:.1f}mm')
        ax.legend()
        
        plt.savefig(os.path.join(img_dir, 'scaled_registration_visualization.png'), dpi=300)
        plt.close()
        
        # Error distribution plot
        plt.figure(figsize=(12, 8))
        plt.hist([e*1000 for e in errors], bins=20, color='skyblue', edgecolor='black', alpha=0.7)
        plt.axvline(x=error_threshold*1000, color='r', linestyle='--', linewidth=2, label=f'Target Threshold ({error_threshold*1000:.1f}mm)')
        plt.axvline(x=SCALED_LATERAL_TARGET*1000, color='g', linestyle=':', linewidth=2, label=f'Lateral Target ({SCALED_LATERAL_TARGET*1000:.1f}mm)')
        plt.axvline(x=SCALED_LONGITUDINAL_TARGET*1000, color='b', linestyle=':', linewidth=2, label=f'Longitudinal Target ({SCALED_LONGITUDINAL_TARGET*1000:.1f}mm)')
        plt.xlabel('Position Error (mm)')
        plt.ylabel('Frequency')
        plt.title('1:32 Scale Model - Distribution of Registration Errors')
        plt.legend()
        plt.grid(True, alpha=0.3)
        plt.savefig(os.path.join(img_dir, 'scaled_error_distribution.png'), dpi=300)
        plt.close()
        
        return high_error_indices, low_error_indices, errors

def process_laser_tracker_data(input_file, output_file):
    """
    Process laser tracker data (same as before)
    """
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
        
            return round(d_perp_x, 6), round(d_perp_z, 6)  # 增加精度
        
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
        df = df.round(6)  # 增加精度
        
        df.to_csv(output_file, index=False, encoding="utf-8")
        
        logger.info(f"Corrected data saved to {output_file}")
        logger.info(f"First few rows: \n{df.head()}")
        
        return df
        
    except Exception as e:
        logger.error(f"Error processing laser tracker data: {e}")
        return None

def calculate_transformations_iterative(max_iterations=10, target_rmse=None):
    """
    Iterative transformation calculation with scaled targets
    """
    if target_rmse is None:
        target_rmse = TARGET_RMSE
        
    logger.info(f"Starting iterative transformation calculation")
    logger.info(f"1:32 Scale Factor Applied")
    logger.info(f"Target RMSE: {target_rmse*1000:.2f}mm (equivalent to {target_rmse*SCALE_FACTOR*100:.1f}cm in real world)")
    
    input_tracker_csv = CORRECTED_LASER_TRACKER_CSV
    current_rmse = float('inf')
    iteration = 1
    
    while current_rmse > target_rmse and iteration <= max_iterations:
        logger.info(f"\n--- Iteration {iteration} ---")
        
        try:
            tracker_laser_df = pd.read_csv(input_tracker_csv)
            logger.info(f"Loaded tracker data with {len(tracker_laser_df)} points")
            
            positions_A = [[row[ROW_X], 0, row[ROW_Y]] for _, row in tracker_laser_df.iterrows()]
            positions_B = [[row["laser_x"], 0, row["laser_z"]] for _, row in tracker_laser_df.iterrows()]
            
            transformer = CoordinateTransformer(positions_A, positions_B)
            transformed_positions, error_results = transformer.demo_transformation()
            
            # 使用overall RMSE作为主要指标
            if 'performance_analysis' in error_results:
                current_rmse = error_results['performance_analysis']['rmse_results']['overall_rmse']
            else:
                # 向后兼容
                current_rmse = transformer.calculate_position_error(positions_B, transformed_positions)
            
            logger.info(f"Current Overall RMSE: {current_rmse*1000:.3f}mm")
            
            T_pos = transformer.get_transformation_matrix()
            
            if current_rmse <= target_rmse:
                logger.info(f"Target RMSE of {target_rmse*1000:.2f}mm achieved! Stopping iterations.")
                break
            
            # 使用动态阈值进行过滤
            error_threshold = max(0.9 * current_rmse, target_rmse * 1.5)
            logger.info(f"Filtering points with error > {error_threshold*1000:.3f}mm")
            
            high_error_indices, low_error_indices, errors = transformer.visualize_registration(error_threshold)
            
            logger.info(f"Removing {len(high_error_indices)} high-error points out of {len(errors)} total points")
            logger.info(f"Keeping {len(low_error_indices)} points for next iteration")
            
            filtered_df = tracker_laser_df.iloc[low_error_indices].copy()
            
            filtered_csv_path = os.path.join(result_dir, f"filtered_tracker_iter_{iteration}.csv")
            filtered_df.to_csv(filtered_csv_path, index=False)
            
            np.savetxt(os.path.join(result_dir, f"T_pos_iter_{iteration}.txt"), T_pos)
            
            input_tracker_csv = filtered_csv_path
            iteration += 1
            
            if len(filtered_df) < 10:
                logger.warning("Too few points remaining after filtering. Stopping iterations.")
                break
                
        except Exception as e:
            logger.error(f"Error in iteration {iteration}: {e}")
            import traceback
            logger.error(traceback.format_exc())
            break
    
    # Final results
    logger.info("\n=== Final Results ===")
    logger.info(f"Final Overall RMSE: {current_rmse*1000:.3f}mm")
    logger.info(f"Real-world equivalent: {current_rmse*SCALE_FACTOR*100:.2f}cm")
    logger.info(f"Iterations performed: {iteration - 1}")
    logger.info(f"Final number of points used: {len(tracker_laser_df)}")
    
    # Save final results
    final_output_path = os.path.join(output_dir, "final_transformation_matrix.txt")
    with open(final_output_path, "w") as f:
        f.write(f"1:32 Scale Model Results\n")
        f.write(f"========================\n")
        f.write(f"Scale Factor: {SCALE_FACTOR}\n")
        f.write(f"Final RMSE: {current_rmse*1000:.3f}mm\n")
        f.write(f"Real-world equivalent: {current_rmse*SCALE_FACTOR*100:.2f}cm\n")
        f.write(f"Target RMSE: {target_rmse*1000:.2f}mm\n")
        f.write(f"Target achieved: {'Yes' if current_rmse <= target_rmse else 'No'}\n")
        f.write(f"Iterations performed: {iteration - 1}\n")
        f.write(f"Final number of points used: {len(tracker_laser_df)}\n\n")
        f.write("Position Transformation Matrix (T_pos):\n")
        f.write(str(T_pos.tolist()) + "\n\n")
    
    logger.info(f"Final transformation matrix saved to {final_output_path}")
    
    return T_pos, current_rmse

def main():
    """Main function"""
    logger.info("Starting 1:32 scale model tracker data processing")
    logger.info(f"Scale factor: {SCALE_FACTOR}")
    logger.info(f"Target RMSE: {TARGET_RMSE*1000:.2f}mm (real-world equivalent: {TARGET_RMSE*SCALE_FACTOR*100:.2f}cm)")
    
    global LASER_TRACKER_CSV
    if len(sys.argv) > 1:
        LASER_TRACKER_CSV = sys.argv[1]
    
    if os.path.exists(LASER_TRACKER_CSV):
        logger.info("Processing laser tracker data...")
        process_laser_tracker_data(LASER_TRACKER_CSV, CORRECTED_LASER_TRACKER_CSV)
        
        if os.path.exists(CORRECTED_LASER_TRACKER_CSV):
            logger.info("Calculating transformation matrix iteratively...")
            
            T_pos, final_rmse = calculate_transformations_iterative(
                max_iterations=10, 
                target_rmse=TARGET_RMSE
            )
            
            if T_pos is not None:
                if final_rmse <= TARGET_RMSE:
                    logger.info(f"SUCCESS: Transformation calculation completed with RMSE {final_rmse*1000:.3f}mm (target: {TARGET_RMSE*1000:.2f}mm)")
                else:
                    logger.info(f"PARTIAL: Transformation calculation completed, but RMSE {final_rmse*1000:.3f}mm exceeds target {TARGET_RMSE*1000:.2f}mm")
            else:
                logger.error("Transformation calculation failed")
        else:
            logger.error(f"Missing corrected laser tracker CSV: {CORRECTED_LASER_TRACKER_CSV}")
    else:
        logger.error(f"Laser tracker data file {LASER_TRACKER_CSV} not found.")
        logger.info("Usage: python script.py <path_to_laser_tracker.csv>")
    
    logger.info("Processing complete")

if __name__ == "__main__":
    main()