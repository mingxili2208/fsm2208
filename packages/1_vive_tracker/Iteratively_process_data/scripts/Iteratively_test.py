#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Tracker data validation script for 1:32 scale model transformation matrix
Validation script for transformation matrix effectiveness
"""
import sys
import os
import datetime
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import json
from pathlib import Path
import argparse

# Set matplotlib to use English
plt.rcParams['font.sans-serif'] = ['Arial', 'DejaVu Sans', 'Liberation Sans']
plt.rcParams['axes.unicode_minus'] = False

# 1:32 scale parameters (consistent with original code)
SCALE_FACTOR = 32
REAL_WORLD_LATERAL_TARGET = 0.10  # Real-world local road lateral accuracy requirement (10cm)
REAL_WORLD_LONGITUDINAL_TARGET = 0.10  # Real-world local road longitudinal accuracy requirement (10cm)

# Scaled targets
SCALED_LATERAL_TARGET = REAL_WORLD_LATERAL_TARGET / SCALE_FACTOR  # 3.125mm
SCALED_LONGITUDINAL_TARGET = REAL_WORLD_LONGITUDINAL_TARGET / SCALE_FACTOR  # 3.125mm
SCALED_COMBINED_TARGET = np.sqrt(SCALED_LATERAL_TARGET**2 + SCALED_LONGITUDINAL_TARGET**2)  # ~4.42mm

# Use stricter target
TARGET_RMSE = 0.004  # 4mm

# Data column names
ROW_X = "X"
ROW_Y = "Z"

class TransformationValidator:
    """
    Transformation matrix validator class
    Used to validate the effectiveness of transformation matrices output from TDPS
    """
    
    def __init__(self, result_dir, output_dir=None):
        """
        Initialize validator
        
        Args:
            result_dir (str): TDPS output result directory path
            output_dir (str): Validation result output directory, auto-created if None
        """
        self.result_dir = Path(result_dir)
        
        # Create output directory
        if output_dir is None:
            timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
            self.output_dir = Path(f"validation_results_{timestamp}")
        else:
            self.output_dir = Path(output_dir)
            
        self.output_dir.mkdir(exist_ok=True)
        
        # Create subdirectories
        
        self.img_dir = self.output_dir / "images"
        self.data_dir = self.output_dir / "data"
        self.report_dir = self.output_dir / "reports"
        
        for directory in [self.img_dir, self.data_dir, self.report_dir]:
            directory.mkdir(exist_ok=True)
            
        # Initialize data storage
        self.transformation_matrix = None
        self.original_data = None
        self.positions_A = None
        self.positions_B = None
        self.transformed_positions = None
        self.validation_results = {}
        
        # Setup logging
        self._setup_logging()
        
    def _setup_logging(self):
        """Setup logging system"""
        import logging
        
        log_file = self.output_dir / "validation.log"
        logging.basicConfig(
            filename=log_file,
            level=logging.INFO,
            format='%(asctime)s - %(levelname)s - %(message)s',
            filemode='w'
        )
        
        # Also output to console
        console = logging.StreamHandler()
        console.setLevel(logging.INFO)
        formatter = logging.Formatter('%(levelname)s - %(message)s')
        console.setFormatter(formatter)
        logging.getLogger().addHandler(console)
        
        self.logger = logging.getLogger(__name__)
        
    def load_transformation_matrix(self, matrix_file_path=None):
        """
        Load transformation matrix from result directory or specified file
        
        Args:
            matrix_file_path (str): Transformation matrix file path, use directly if provided
            
        Returns:
            bool: Whether loading was successful
        """
        try:
            if matrix_file_path:
                # If specific file path is provided, use it directly
                matrix_file = Path(matrix_file_path)
                if not matrix_file.exists():
                    self.logger.error(f"Specified transformation matrix file does not exist: {matrix_file}")
                    return False
            else:
                # Search for transformation matrix files
                matrix_files = list(self.result_dir.glob("**/final_transformation_matrix.txt"))
                if not matrix_files:
                    # Try to find latest iteration matrix
                    matrix_files = list(self.result_dir.glob("**/T_pos_iter_*.txt"))
                    if matrix_files:
                        # Select the last iteration matrix
                        matrix_files.sort(key=lambda x: int(x.stem.split('_')[-1]))
                        matrix_file = matrix_files[-1]
                    else:
                        self.logger.error("Transformation matrix file not found")
                        return False
                else:
                    matrix_file = matrix_files[0]
            
            self.logger.info(f"Loading transformation matrix file: {matrix_file}")
            
            # Read matrix file
            if matrix_file.name == "final_transformation_matrix.txt" or "final_icp_transformation_matrix.txt" in matrix_file.name:
                # Handle files containing text descriptions
                with open(matrix_file, 'r', encoding='utf-8') as f:
                    content = f.read()
                    
                # Extract matrix part
                matrix_start = content.find("Position Transformation Matrix")
                if matrix_start == -1:
                    matrix_start = content.find("ICP Transformation Matrix")
                
                if matrix_start != -1:
                    lines = content[matrix_start:].split('\n')
                    for line in lines:
                        if line.strip().startswith('['):
                            matrix_content = line.strip()
                            self.transformation_matrix = np.array(eval(matrix_content))
                            break
                    else:
                        # If matrix not found, try direct numerical reading
                        try:
                            self.transformation_matrix = np.loadtxt(matrix_file)
                        except:
                            self.logger.error("Unable to parse transformation matrix file format")
                            return False
                else:
                    # Try direct numerical matrix file reading
                    try:
                        self.transformation_matrix = np.loadtxt(matrix_file)
                    except:
                        self.logger.error("Unable to parse transformation matrix file format")
                        return False
            else:
                # Direct numerical matrix file reading
                self.transformation_matrix = np.loadtxt(matrix_file)
            
            self.logger.info("Transformation matrix loaded successfully")
            self.logger.info(f"Matrix shape: {self.transformation_matrix.shape}")
            self.logger.info(f"Transformation matrix:\n{self.transformation_matrix}")
            return True
            
        except Exception as e:
            self.logger.error(f"Failed to load transformation matrix: {e}")
            import traceback
            self.logger.error(traceback.format_exc())
            return False
    
    def load_original_data(self):
        """
        Load original data from result directory
        
        Returns:
            bool: Whether loading was successful
        """
        try:
            # Search for original corrected data files
            data_files = list(self.result_dir.glob("**/corrected_laser_tracker.csv"))
            if not data_files:
                # Try to find last iteration filtered data
                filtered_files = list(self.result_dir.glob("**/filtered_tracker_iter_*.csv"))
                if filtered_files:
                    # Select the last iteration data
                    filtered_files.sort(key=lambda x: int(x.stem.split('_')[-1]))
                    data_file = filtered_files[-1]
                else:
                    # Try to find data files in data directory
                    data_files = list(self.result_dir.glob("**/data/corrected_laser_tracker.csv"))
                    if data_files:
                        data_file = data_files[0]
                    else:
                        self.logger.error("Data file not found")
                        return False
            else:
                data_file = data_files[0]
            
            self.logger.info(f"Loading data file: {data_file}")
            
            # Read data
            self.original_data = pd.read_csv(data_file)
            
            # Check if necessary columns exist
            required_cols = [ROW_X, ROW_Y, "laser_x", "laser_z"]
            missing_cols = [col for col in required_cols if col not in self.original_data.columns]
            if missing_cols:
                self.logger.error(f"Data file missing necessary columns: {missing_cols}")
                self.logger.info(f"Available columns: {list(self.original_data.columns)}")
                return False
            
            # Extract coordinate point pairs
            self.positions_A = [[row[ROW_X], 0, row[ROW_Y]] for _, row in self.original_data.iterrows()]
            self.positions_B = [[row["laser_x"], 0, row["laser_z"]] for _, row in self.original_data.iterrows()]
            
            self.logger.info(f"Successfully loaded {len(self.positions_A)} data point pairs")
            return True
            
        except Exception as e:
            self.logger.error(f"Failed to load original data: {e}")
            import traceback
            self.logger.error(traceback.format_exc())
            return False
    
    def apply_transformation(self):
        """
        Apply transformation matrix to data points
        
        Returns:
            bool: Whether transformation was successfully applied
        """
        try:
            if self.transformation_matrix is None or self.positions_A is None:
                self.logger.error("Transformation matrix or data points not loaded")
                return False
            
            self.transformed_positions = []
            
            for position in self.positions_A:
                # Convert to homogeneous coordinates
                position_homogeneous = np.append(position, 1)
                # Apply transformation matrix
                transformed_position_homogeneous = np.dot(self.transformation_matrix, position_homogeneous)
                # Extract first three coordinates and preserve precision
                transformed_position = transformed_position_homogeneous[:3].round(6)
                self.transformed_positions.append(transformed_position)
            
            self.logger.info("Transformation matrix applied successfully")
            return True
            
        except Exception as e:
            self.logger.error(f"Failed to apply transformation matrix: {e}")
            import traceback
            self.logger.error(traceback.format_exc())
            return False
    
    def calculate_validation_metrics(self):
        """
        Calculate validation metrics
        
        Returns:
            dict: Validation results dictionary
        """
        try:
            actual = np.array(self.positions_B)
            predicted = np.array(self.transformed_positions)
            
            # Calculate RMSE for each direction
            rmse_lateral = np.sqrt(np.mean((actual[:, 0] - predicted[:, 0])**2))
            rmse_vertical = np.sqrt(np.mean((actual[:, 1] - predicted[:, 1])**2))
            rmse_longitudinal = np.sqrt(np.mean((actual[:, 2] - predicted[:, 2])**2))
            
            # Overall RMSE
            overall_rmse = np.sqrt(rmse_lateral**2 + rmse_vertical**2 + rmse_longitudinal**2)
            
            # Main focus horizontal plane RMSE (X-Z plane)
            horizontal_rmse = np.sqrt(rmse_lateral**2 + rmse_longitudinal**2)
            
            # Calculate individual errors
            individual_errors = []
            for i in range(len(actual)):
                error = np.linalg.norm(actual[i] - predicted[i])
                individual_errors.append(error)
            
            # Analyze target achievement
            points_within_target = sum(1 for error in individual_errors if error <= TARGET_RMSE)
            points_outside_target = len(individual_errors) - points_within_target
            
            # Analyze directional target achievement
            lateral_errors = np.abs(actual[:, 0] - predicted[:, 0])
            longitudinal_errors = np.abs(actual[:, 2] - predicted[:, 2])
            
            lateral_within_target = sum(1 for error in lateral_errors if error <= SCALED_LATERAL_TARGET)
            longitudinal_within_target = sum(1 for error in longitudinal_errors if error <= SCALED_LONGITUDINAL_TARGET)
            
            # Calculate statistical information
            error_stats = {
                'mean': np.mean(individual_errors),
                'std': np.std(individual_errors),
                'min': np.min(individual_errors),
                'max': np.max(individual_errors),
                'median': np.median(individual_errors),
                'p95': np.percentile(individual_errors, 95),
                'p99': np.percentile(individual_errors, 99)
            }
            
            # Compile validation results
            self.validation_results = {
                'rmse_metrics': {
                    'lateral_rmse': rmse_lateral,
                    'vertical_rmse': rmse_vertical,
                    'longitudinal_rmse': rmse_longitudinal,
                    'horizontal_rmse': horizontal_rmse,
                    'overall_rmse': overall_rmse
                },
                'target_analysis': {
                    'overall_target': TARGET_RMSE,
                    'lateral_target': SCALED_LATERAL_TARGET,
                    'longitudinal_target': SCALED_LONGITUDINAL_TARGET,
                    'points_within_target': points_within_target,
                    'points_outside_target': points_outside_target,
                    'lateral_within_target': lateral_within_target,
                    'longitudinal_within_target': longitudinal_within_target,
                    'total_points': len(individual_errors),
                    'success_rate': points_within_target / len(individual_errors) * 100
                },
                'error_statistics': error_stats,
                'individual_errors': individual_errors,
                'lateral_errors': lateral_errors.tolist(),
                'longitudinal_errors': longitudinal_errors.tolist(),
                'real_world_equivalent': {
                    'lateral_rmse_real': rmse_lateral * SCALE_FACTOR,
                    'longitudinal_rmse_real': rmse_longitudinal * SCALE_FACTOR,
                    'horizontal_rmse_real': horizontal_rmse * SCALE_FACTOR,
                    'overall_rmse_real': overall_rmse * SCALE_FACTOR
                }
            }
            
            self.logger.info("Validation metrics calculation completed")
            return self.validation_results
            
        except Exception as e:
            self.logger.error(f"Failed to calculate validation metrics: {e}")
            import traceback
            self.logger.error(traceback.format_exc())
            return {}
    
    def generate_summary_report(self):
        """
        Generate summary report
        """
        try:
            if not self.validation_results:
                self.logger.error("Validation results are empty, cannot generate report")
                return
            
            rmse = self.validation_results['rmse_metrics']
            target = self.validation_results['target_analysis']
            stats = self.validation_results['error_statistics']
            real_world = self.validation_results['real_world_equivalent']
            
            self.logger.info("\n" + "="*60)
            self.logger.info("Transformation Matrix Validation Results Summary")
            self.logger.info("="*60)
            
            self.logger.info(f"Total data points: {target['total_points']}")
            self.logger.info(f"Scale factor: 1:{SCALE_FACTOR}")
            
            self.logger.info("\n--- RMSE Metrics (Scaled Model) ---")
            self.logger.info(f"Lateral RMSE: {rmse['lateral_rmse']*1000:.3f}mm")
            self.logger.info(f"Longitudinal RMSE: {rmse['longitudinal_rmse']*1000:.3f}mm")
            self.logger.info(f"Vertical RMSE: {rmse['vertical_rmse']*1000:.3f}mm")
            self.logger.info(f"Horizontal RMSE: {rmse['horizontal_rmse']*1000:.3f}mm")
            self.logger.info(f"Overall RMSE: {rmse['overall_rmse']*1000:.3f}mm")
            
            self.logger.info("\n--- Real-world Equivalent Accuracy ---")
            self.logger.info(f"Lateral equivalent accuracy: {real_world['lateral_rmse_real']*100:.2f}cm")
            self.logger.info(f"Longitudinal equivalent accuracy: {real_world['longitudinal_rmse_real']*100:.2f}cm")
            self.logger.info(f"Horizontal equivalent accuracy: {real_world['horizontal_rmse_real']*100:.2f}cm")
            self.logger.info(f"Overall equivalent accuracy: {real_world['overall_rmse_real']*100:.2f}cm")
            
            self.logger.info("\n--- Target Achievement ---")
            self.logger.info(f"Overall target ({TARGET_RMSE*1000:.1f}mm): {target['points_within_target']}/{target['total_points']} points achieved ({target['success_rate']:.1f}%)")
            self.logger.info(f"Lateral target ({SCALED_LATERAL_TARGET*1000:.1f}mm): {target['lateral_within_target']}/{target['total_points']} points achieved")
            self.logger.info(f"Longitudinal target ({SCALED_LONGITUDINAL_TARGET*1000:.1f}mm): {target['longitudinal_within_target']}/{target['total_points']} points achieved")
            
            self.logger.info("\n--- Error Statistics ---")
            self.logger.info(f"Mean error: {stats['mean']*1000:.3f}mm")
            self.logger.info(f"Standard deviation: {stats['std']*1000:.3f}mm")
            self.logger.info(f"Minimum error: {stats['min']*1000:.3f}mm")
            self.logger.info(f"Maximum error: {stats['max']*1000:.3f}mm")
            self.logger.info(f"Median error: {stats['median']*1000:.3f}mm")
            self.logger.info(f"95th percentile: {stats['p95']*1000:.3f}mm")
            self.logger.info(f"99th percentile: {stats['p99']*1000:.3f}mm")
            
            # Overall assessment
            self.logger.info("\n--- Overall Assessment ---")
            if rmse['overall_rmse'] <= TARGET_RMSE:
                self.logger.info("✓ Transformation matrix meets overall RMSE target")
            else:
                self.logger.info("✗ Transformation matrix does not meet overall RMSE target")
            
            if target['success_rate'] >= 80:
                self.logger.info("✓ Most point pairs meet accuracy requirements")
            elif target['success_rate'] >= 60:
                self.logger.info("⚠ Some point pairs meet accuracy requirements")
            else:
                self.logger.info("✗ Most point pairs do not meet accuracy requirements")
            
            self.logger.info("="*60)
            
        except Exception as e:
            self.logger.error(f"Failed to generate summary report: {e}")
    
    def save_detailed_report(self):
        """
        Save detailed validation report to file
        """
        try:
            report_file = self.report_dir / "validation_report.json"
            
            # Prepare report data
            report_data = {
                'metadata': {
                    'validation_time': datetime.datetime.now().isoformat(),
                    'result_directory': str(self.result_dir),
                    'scale_factor': SCALE_FACTOR,
                    'target_rmse_mm': TARGET_RMSE * 1000,
                    'lateral_target_mm': SCALED_LATERAL_TARGET * 1000,
                    'longitudinal_target_mm': SCALED_LONGITUDINAL_TARGET * 1000
                },
                'transformation_matrix': self.transformation_matrix.tolist(),
                'validation_results': self.validation_results
            }
            
            # Save JSON report
            with open(report_file, 'w', encoding='utf-8') as f:
                json.dump(report_data, f, indent=2, ensure_ascii=False)
            
            # Save CSV format point pair error data
            error_df = pd.DataFrame({
                'point_index': range(len(self.validation_results['individual_errors'])),
                'source_x': [p[0] for p in self.positions_A],
                'source_y': [p[1] for p in self.positions_A],
                'source_z': [p[2] for p in self.positions_A],
                'target_x': [p[0] for p in self.positions_B],
                'target_y': [p[1] for p in self.positions_B],
                'target_z': [p[2] for p in self.positions_B],
                'transformed_x': [p[0] for p in self.transformed_positions],
                'transformed_y': [p[1] for p in self.transformed_positions],
                'transformed_z': [p[2] for p in self.transformed_positions],
                'error_mm': [e * 1000 for e in self.validation_results['individual_errors']],
                'lateral_error_mm': [e * 1000 for e in self.validation_results['lateral_errors']],
                'longitudinal_error_mm': [e * 1000 for e in self.validation_results['longitudinal_errors']],
                'within_target': [e <= TARGET_RMSE for e in self.validation_results['individual_errors']]
            })
            
            error_df.to_csv(self.data_dir / "point_errors.csv", index=False)
            
            self.logger.info(f"Detailed report saved to: {report_file}")
            self.logger.info(f"Point pair error data saved to: {self.data_dir / 'point_errors.csv'}")
            
        except Exception as e:
            self.logger.error(f"Failed to save detailed report: {e}")
    
    def plot_point_clouds_comparison(self):
        """
        Plot point cloud comparison (before and after transformation)
        """
        try:
            fig = plt.figure(figsize=(20, 15))
            
            # 1. 3D view - before transformation
            ax1 = fig.add_subplot(2, 3, 1, projection='3d')
            
            # Source point cloud (coordinate system A)
            positions_A = np.array(self.positions_A)
            ax1.scatter(positions_A[:, 0], positions_A[:, 2], positions_A[:, 1], 
                       c='blue', marker='o', s=50, alpha=0.7, label='Source Points (System A)')
            
            # Target point cloud (coordinate system B)
            positions_B = np.array(self.positions_B)
            ax1.scatter(positions_B[:, 0], positions_B[:, 2], positions_B[:, 1], 
                       c='red', marker='^', s=50, alpha=0.7, label='Target Points (System B)')
            
            ax1.set_xlabel('X (Lateral)')
            ax1.set_ylabel('Z (Longitudinal)')
            ax1.set_zlabel('Y (Vertical)')
            ax1.set_title('Before Transformation - Point Clouds in Two Coordinate Systems')
            ax1.legend()
            
            # 2. 3D view - after transformation
            ax2 = fig.add_subplot(2, 3, 2, projection='3d')
            
            # Transformed point cloud
            transformed_positions = np.array(self.transformed_positions)
            ax2.scatter(transformed_positions[:, 0], transformed_positions[:, 2], transformed_positions[:, 1], 
                       c='green', marker='x', s=50, alpha=0.7, label='Transformed Source Points')
            
            # Target point cloud
            ax2.scatter(positions_B[:, 0], positions_B[:, 2], positions_B[:, 1], 
                       c='red', marker='^', s=50, alpha=0.7, label='Target Points (System B)')
            
            # Draw error lines
            for i in range(0, len(positions_B), max(1, len(positions_B)//20)):  # Draw only some lines to avoid overcrowding
                ax2.plot([transformed_positions[i, 0], positions_B[i, 0]],
                        [transformed_positions[i, 2], positions_B[i, 2]],
                        [transformed_positions[i, 1], positions_B[i, 1]],
                        'k--', alpha=0.3, linewidth=0.5)
            
            ax2.set_xlabel('X (Lateral)')
            ax2.set_ylabel('Z (Longitudinal)')
            ax2.set_zlabel('Y (Vertical)')
            ax2.set_title('After Transformation - Registration Results')
            ax2.legend()
            
            # 3. XZ plane view (main focus horizontal plane)
            ax3 = fig.add_subplot(2, 3, 3)
            
            ax3.scatter(positions_A[:, 0], positions_A[:, 2], 
                       c='blue', marker='o', s=30, alpha=0.7, label='Source Points (System A)')
            ax3.scatter(positions_B[:, 0], positions_B[:, 2], 
                       c='red', marker='^', s=30, alpha=0.7, label='Target Points (System B)')
            ax3.scatter(transformed_positions[:, 0], transformed_positions[:, 2], 
                       c='green', marker='x', s=30, alpha=0.7, label='Transformed Source Points')
            
            ax3.set_xlabel('X (Lateral)')
            ax3.set_ylabel('Z (Longitudinal)')
            ax3.set_title('XZ Plane View (Horizontal Plane)')
            ax3.legend()
            ax3.grid(True, alpha=0.3)
            ax3.axis('equal')
            
            # 4. Error distribution heatmap
            ax4 = fig.add_subplot(2, 3, 4)
            
            errors = self.validation_results['individual_errors']
            colors = ['green' if e <= TARGET_RMSE else 'red' for e in errors]
            
            scatter = ax4.scatter(positions_B[:, 0], positions_B[:, 2], 
                                 c=[e*1000 for e in errors], cmap='viridis_r', 
                                 s=60, alpha=0.8)
            
            # Add target threshold reference circles
            for i, (x, z) in enumerate(zip(positions_B[:, 0], positions_B[:, 2])):
                if errors[i] > TARGET_RMSE:
                    circle = plt.Circle((x, z), TARGET_RMSE, fill=False, color='red', alpha=0.3)
                    ax4.add_patch(circle)
            
            plt.colorbar(scatter, ax=ax4, label='Error (mm)')
            ax4.set_xlabel('X (Lateral)')
            ax4.set_ylabel('Z (Longitudinal)')
            ax4.set_title('Spatial Error Distribution')
            ax4.grid(True, alpha=0.3)
            
            # 5. Error statistics histogram
            ax5 = fig.add_subplot(2, 3, 5)
            
            ax5.hist([e*1000 for e in errors], bins=20, color='skyblue', edgecolor='black', alpha=0.7)
            ax5.axvline(x=TARGET_RMSE*1000, color='r', linestyle='--', linewidth=2, 
                       label=f'Target Threshold ({TARGET_RMSE*1000:.1f}mm)')
            ax5.axvline(x=np.mean(errors)*1000, color='g', linestyle='-', linewidth=2, 
                       label=f'Mean Error ({np.mean(errors)*1000:.1f}mm)')
            
            ax5.set_xlabel('Error (mm)')
            ax5.set_ylabel('Frequency')
            ax5.set_title('Error Distribution Histogram')
            ax5.legend()
            ax5.grid(True, alpha=0.3)
            
            # 6. Success rate pie chart
            ax6 = fig.add_subplot(2, 3, 6)
            
            within_target = self.validation_results['target_analysis']['points_within_target']
            outside_target = self.validation_results['target_analysis']['points_outside_target']
            
            labels = [f'Within Target\n({within_target} pts)', f'Outside Target\n({outside_target} pts)']
            sizes = [within_target, outside_target]
            colors = ['lightgreen', 'lightcoral']
            
            ax6.pie(sizes, labels=labels, colors=colors, autopct='%1.1f%%', startangle=90)
            ax6.set_title('Target Achievement Status')
            
            plt.tight_layout()
            plt.savefig(self.img_dir / 'point_clouds_comparison.png', dpi=300, bbox_inches='tight')
            plt.close()
            
            self.logger.info("Point cloud comparison plot saved")
            
        except Exception as e:
            self.logger.error(f"Failed to plot point cloud comparison: {e}")
            import traceback
            self.logger.error(traceback.format_exc())
    
    def plot_rmse_analysis(self):
        """
        Plot RMSE analysis charts
        """
        try:
            fig, axes = plt.subplots(2, 2, figsize=(15, 12))
            
            rmse_metrics = self.validation_results['rmse_metrics']
            
            # 1. RMSE comparison bar chart
            ax1 = axes[0, 0]
            
            rmse_names = ['Lateral', 'Longitudinal', 'Vertical', 'Horizontal', 'Overall']
            rmse_values = [
                rmse_metrics['lateral_rmse'] * 1000,
                rmse_metrics['longitudinal_rmse'] * 1000,
                rmse_metrics['vertical_rmse'] * 1000,
                rmse_metrics['horizontal_rmse'] * 1000,
                rmse_metrics['overall_rmse'] * 1000
            ]
            targets = [
                SCALED_LATERAL_TARGET * 1000,
                SCALED_LONGITUDINAL_TARGET * 1000,
                float('inf'),  # No specific target for vertical direction
                SCALED_COMBINED_TARGET * 1000,
                TARGET_RMSE * 1000
            ]
            
            colors = ['green' if rmse < target else 'red' for rmse, target in zip(rmse_values, targets)]
            
            bars = ax1.bar(rmse_names, rmse_values, color=colors, alpha=0.7)
            
            # Add target lines
            for i, target in enumerate(targets):
                if target != float('inf'):
                    ax1.axhline(y=target, color='red', linestyle='--', alpha=0.5)
            
            ax1.set_ylabel('RMSE (mm)')
            ax1.set_title('Directional RMSE Comparison')
            ax1.grid(True, alpha=0.3)
            
            # Add value labels
            for bar, value in zip(bars, rmse_values):
                height = bar.get_height()
                ax1.text(bar.get_x() + bar.get_width()/2., height + 0.1,
                        f'{value:.2f}', ha='center', va='bottom')
            
            # 2. Real-world equivalent accuracy
            ax2 = axes[0, 1]
            
            real_world = self.validation_results['real_world_equivalent']
            rw_names = ['Lateral', 'Longitudinal', 'Horizontal', 'Overall']
            rw_values = [
                real_world['lateral_rmse_real'] * 100,
                real_world['longitudinal_rmse_real'] * 100,
                real_world['horizontal_rmse_real'] * 100,
                real_world['overall_rmse_real'] * 100
            ]
            rw_targets = [10, 10, 14.14, 20]  # cm
            
            colors = ['green' if val < target else 'red' for val, target in zip(rw_values, rw_targets)]
            bars = ax2.bar(rw_names, rw_values, color=colors, alpha=0.7)
            
            ax2.set_ylabel('Equivalent Accuracy (cm)')
            ax2.set_title('Real-world Equivalent Accuracy')
            ax2.grid(True, alpha=0.3)
            
            # Add target line
            ax2.axhline(y=10, color='red', linestyle='--', alpha=0.5, label='Target: 10cm')
            
            # Add value labels
            for bar, value in zip(bars, rw_values):
                height = bar.get_height()
                ax2.text(bar.get_x() + bar.get_width()/2., height + 0.2,
                        f'{value:.1f}', ha='center', va='bottom')
            
            # 3. Error cumulative distribution function
            ax3 = axes[1, 0]
            
            errors = np.array(self.validation_results['individual_errors']) * 1000
            sorted_errors = np.sort(errors)
            percentiles = np.arange(1, len(sorted_errors) + 1) / len(sorted_errors) * 100
            
            ax3.plot(sorted_errors, percentiles, linewidth=2, color='blue')
            ax3.axvline(x=TARGET_RMSE*1000, color='r', linestyle='--', linewidth=2, 
                       label=f'Target: {TARGET_RMSE*1000:.1f}mm')
            ax3.axhline(y=95, color='g', linestyle=':', linewidth=2, label='95th Percentile')
            
            ax3.set_xlabel('Error (mm)')
            ax3.set_ylabel('Cumulative Percentage (%)')
            ax3.set_title('Error Cumulative Distribution Function')
            ax3.grid(True, alpha=0.3)
            ax3.legend()
            
            # 4. Directional error scatter plot
            ax4 = axes[1, 1]
            
            lateral_errors = np.array(self.validation_results['lateral_errors']) * 1000
            longitudinal_errors = np.array(self.validation_results['longitudinal_errors']) * 1000
            
            colors = ['green' if e <= TARGET_RMSE*1000 else 'red' for e in errors]
            scatter = ax4.scatter(lateral_errors, longitudinal_errors, c=colors, alpha=0.6)
            
            ax4.axvline(x=SCALED_LATERAL_TARGET*1000, color='red', linestyle='--', alpha=0.5)
            ax4.axhline(y=SCALED_LONGITUDINAL_TARGET*1000, color='red', linestyle='--', alpha=0.5)
            
            ax4.set_xlabel('Lateral Error (mm)')
            ax4.set_ylabel('Longitudinal Error (mm)')
            ax4.set_title('Lateral vs Longitudinal Error Distribution')
            ax4.grid(True, alpha=0.3)
            
            plt.tight_layout()
            plt.savefig(self.img_dir / 'rmse_analysis.png', dpi=300, bbox_inches='tight')
            plt.close()
            
            self.logger.info("RMSE analysis plot saved")
            
        except Exception as e:
            self.logger.error(f"Failed to plot RMSE analysis: {e}")
            import traceback
            self.logger.error(traceback.format_exc())
    
    def plot_individual_point_errors(self):
        """
        Plot individual point error analysis charts
        """
        try:
            fig, axes = plt.subplots(2, 2, figsize=(15, 12))
            
            errors = np.array(self.validation_results['individual_errors']) * 1000
            point_indices = range(len(errors))
            
            # 1. Point error sequence plot
            ax1 = axes[0, 0]
            
            colors = ['green' if e <= TARGET_RMSE*1000 else 'red' for e in errors]
            ax1.scatter(point_indices, errors, c=colors, alpha=0.7)
            ax1.axhline(y=TARGET_RMSE*1000, color='red', linestyle='--', linewidth=2, 
                       label=f'Target: {TARGET_RMSE*1000:.1f}mm')
            
            ax1.set_xlabel('Point Index')
            ax1.set_ylabel('Error (mm)')
            ax1.set_title('Individual Point Error Distribution')
            ax1.legend()
            ax1.grid(True, alpha=0.3)
            
            # 2. Error heatmap (by spatial position)
            ax2 = axes[0, 1]
            
            positions_B = np.array(self.positions_B)
            
            # Create grid for interpolation
            xi = np.linspace(positions_B[:, 0].min(), positions_B[:, 0].max(), 50)
            zi = np.linspace(positions_B[:, 2].min(), positions_B[:, 2].max(), 50)
            Xi, Zi = np.meshgrid(xi, zi)
            
            # Use nearest neighbor interpolation
            from scipy.spatial import cKDTree
            tree = cKDTree(positions_B[:, [0, 2]])
            distances, indices = tree.query(np.c_[Xi.ravel(), Zi.ravel()])
            error_grid = errors[indices].reshape(Xi.shape)
            
            im = ax2.contourf(Xi, Zi, error_grid, levels=20, cmap='viridis_r')
            ax2.scatter(positions_B[:, 0], positions_B[:, 2], c=errors, 
                       cmap='viridis_r', s=30, edgecolors='black', linewidth=0.5)
            
            plt.colorbar(im, ax=ax2, label='Error (mm)')
            ax2.set_xlabel('X (Lateral)')
            ax2.set_ylabel('Z (Longitudinal)')
            ax2.set_title('Spatial Error Heatmap')
            
            # 3. Error vs distance relationship
            ax3 = axes[1, 0]
            
            # Calculate distance from origin for each point
            distances = np.sqrt(positions_B[:, 0]**2 + positions_B[:, 2]**2)
            
            colors = ['green' if e <= TARGET_RMSE*1000 else 'red' for e in errors]
            ax3.scatter(distances, errors, c=colors, alpha=0.7)
            ax3.axhline(y=TARGET_RMSE*1000, color='red', linestyle='--', linewidth=2, 
                       label=f'Target: {TARGET_RMSE*1000:.1f}mm')
            
            # Add trend line
            z = np.polyfit(distances, errors, 1)
            p = np.poly1d(z)
            ax3.plot(distances, p(distances), "b--", alpha=0.8, linewidth=1, label='Trend Line')
            
            ax3.set_xlabel('Distance from Origin (m)')
            ax3.set_ylabel('Error (mm)')
            ax3.set_title('Error vs Spatial Distance')
            ax3.legend()
            ax3.grid(True, alpha=0.3)
            
            # 4. Box plot analysis
            ax4 = axes[1, 1]
            
            # Divide errors into several intervals
            distance_bins = np.quantile(distances, [0, 0.25, 0.5, 0.75, 1.0])
            binned_errors = []
            bin_labels = []
            
            for i in range(len(distance_bins)-1):
                mask = (distances >= distance_bins[i]) & (distances < distance_bins[i+1])
                if i == len(distance_bins)-2:  # Last bin includes equal sign
                    mask = (distances >= distance_bins[i]) & (distances <= distance_bins[i+1])
                
                binned_errors.append(errors[mask])
                bin_labels.append(f'{distance_bins[i]:.2f}-{distance_bins[i+1]:.2f}m')
            
            ax4.boxplot(binned_errors, labels=bin_labels)
            ax4.axhline(y=TARGET_RMSE*1000, color='red', linestyle='--', linewidth=2, 
                       label=f'Target: {TARGET_RMSE*1000:.1f}mm')
            
            ax4.set_xlabel('Distance Interval')
            ax4.set_ylabel('Error (mm)')
            ax4.set_title('Error Distribution by Distance Groups')
            ax4.legend()
            ax4.grid(True, alpha=0.3)
            
            plt.tight_layout()
            plt.savefig(self.img_dir / 'individual_point_errors.png', dpi=300, bbox_inches='tight')
            plt.close()
            
            self.logger.info("Individual point error analysis plot saved")
            
        except Exception as e:
            self.logger.error(f"Failed to plot individual point errors: {e}")
            import traceback
            self.logger.error(traceback.format_exc())
    
    def plot_transformation_quality(self):
        """
        Plot transformation quality assessment charts
        """
        try:
            fig, axes = plt.subplots(2, 2, figsize=(15, 12))
            
            # 1. Point correspondence before and after transformation
            ax1 = axes[0, 0]
            
            positions_A = np.array(self.positions_A)
            positions_B = np.array(self.positions_B)
            transformed_positions = np.array(self.transformed_positions)
            
            # Draw correspondence lines
            for i in range(0, len(positions_A), 5):  # Draw line every 5 points
                ax1.plot([positions_A[i, 0], transformed_positions[i, 0]], 
                        [positions_A[i, 2], transformed_positions[i, 2]], 
                        'b-', alpha=0.3, linewidth=0.8)
            
            ax1.scatter(positions_A[:, 0], positions_A[:, 2], 
                       c='blue', marker='o', s=20, alpha=0.7, label='Source Points (System A)')
            ax1.scatter(transformed_positions[:, 0], transformed_positions[:, 2], 
                       c='green', marker='x', s=20, alpha=0.7, label='After Transformation')
            
            ax1.set_xlabel('X (Lateral)')
            ax1.set_ylabel('Z (Longitudinal)')
            ax1.set_title('Point Correspondence Before and After Transformation')
            ax1.legend()
            ax1.grid(True, alpha=0.3)
            ax1.axis('equal')
            
            # 2. Residual analysis
            ax2 = axes[0, 1]
            
            residuals_x = (positions_B[:, 0] - transformed_positions[:, 0]) * 1000
            residuals_z = (positions_B[:, 2] - transformed_positions[:, 2]) * 1000
            
            ax2.scatter(residuals_x, residuals_z, alpha=0.7, c='purple')
            ax2.axhline(y=0, color='red', linestyle='-', alpha=0.5)
            ax2.axvline(x=0, color='red', linestyle='-', alpha=0.5)
            
            # Add error ellipse
            from matplotlib.patches import Ellipse
            mean_x, mean_z = np.mean(residuals_x), np.mean(residuals_z)
            std_x, std_z = np.std(residuals_x), np.std(residuals_z)
            
            ellipse = Ellipse((mean_x, mean_z), 2*std_x, 2*std_z, 
                             alpha=0.3, facecolor='red', edgecolor='red')
            ax2.add_patch(ellipse)
            
            ax2.set_xlabel('X Direction Residual (mm)')
            ax2.set_ylabel('Z Direction Residual (mm)')
            ax2.set_title('Transformation Residual Analysis')
            ax2.grid(True, alpha=0.3)
            ax2.axis('equal')
            
            # 3. Rotation and translation component analysis
            ax3 = axes[1, 0]
            
            # Extract rotation and translation information from transformation matrix
            R = self.transformation_matrix[:3, :3]
            t = self.transformation_matrix[:3, 3]
            
            # Calculate rotation angle
            rotation_angle = np.degrees(np.arccos((np.trace(R) - 1) / 2))
            
            # Plot transformation parameters
            param_names = ['Translation X (mm)', 'Translation Y (mm)', 'Translation Z (mm)', 'Rotation Angle (deg)']
            param_values = [t[0]*1000, t[1]*1000, t[2]*1000, rotation_angle]
            
            bars = ax3.bar(param_names, param_values, color=['skyblue', 'lightgreen', 'lightcoral', 'gold'])
            ax3.set_ylabel('Value')
            ax3.set_title('Transformation Parameters')
            ax3.grid(True, alpha=0.3)
            
            # Add value labels
            for bar, value in zip(bars, param_values):
                height = bar.get_height()
                ax3.text(bar.get_x() + bar.get_width()/2., height + height*0.01,
                        f'{value:.2f}', ha='center', va='bottom')
            
            plt.xticks(rotation=45)
            
            # 4. Registration quality scoring
            ax4 = axes[1, 1]
            
            # Calculate various quality metrics
            rmse_score = min(100, (TARGET_RMSE / self.validation_results['rmse_metrics']['overall_rmse']) * 100)
            success_rate = self.validation_results['target_analysis']['success_rate']
            consistency_score = max(0, 100 - np.std(self.validation_results['individual_errors']) * 10000)
            
            quality_metrics = ['RMSE Quality', 'Success Rate', 'Consistency']
            scores = [rmse_score, success_rate, consistency_score]
            colors = ['green' if s >= 80 else 'orange' if s >= 60 else 'red' for s in scores]
            
            bars = ax4.bar(quality_metrics, scores, color=colors, alpha=0.7)
            ax4.set_ylabel('Score (%)')
            ax4.set_title('Registration Quality Assessment')
            ax4.set_ylim(0, 100)
            ax4.grid(True, alpha=0.3)
            
            # Add value labels
            for bar, score in zip(bars, scores):
                height = bar.get_height()
                ax4.text(bar.get_x() + bar.get_width()/2., height + 1,
                        f'{score:.1f}%', ha='center', va='bottom')
            
            # Add baseline
            ax4.axhline(y=80, color='green', linestyle='--', alpha=0.5, label='Excellent (80%)')
            ax4.axhline(y=60, color='orange', linestyle='--', alpha=0.5, label='Good (60%)')
            ax4.legend()
            
            plt.tight_layout()
            plt.savefig(self.img_dir / 'transformation_quality.png', dpi=300, bbox_inches='tight')
            plt.close()
            
            self.logger.info("Transformation quality assessment plot saved")
            
        except Exception as e:
            self.logger.error(f"Failed to plot transformation quality: {e}")
            import traceback
            self.logger.error(traceback.format_exc())
    
    def run_validation(self, matrix_file_path=None):
        """
        Run complete validation process
        
        Args:
            matrix_file_path (str): Transformation matrix file path
            
        Returns:
            bool: Whether validation completed successfully
        """
        try:
            self.logger.info("Starting transformation matrix validation process...")
            
            # 1. Load transformation matrix
            if not self.load_transformation_matrix(matrix_file_path):
                return False
            
            # 2. Load original data
            if not self.load_original_data():
                return False
            
            # 3. Apply transformation
            if not self.apply_transformation():
                return False
            
            # 4. Calculate validation metrics
            self.calculate_validation_metrics()
            
            # 5. Generate reports
            self.generate_summary_report()
            self.save_detailed_report()
            
            # 6. Generate visualization charts
            self.plot_point_clouds_comparison()
            self.plot_rmse_analysis()
            self.plot_individual_point_errors()
            self.plot_transformation_quality()
            
            self.logger.info(f"Validation completed! Results saved in: {self.output_dir}")
            return True
            
        except Exception as e:
            self.logger.error(f"Validation process failed: {e}")
            import traceback
            self.logger.error(traceback.format_exc())
            return False

def main():
    """Main function"""
    if len(sys.argv) < 2:
        print("Usage: python3 Iteratively_test.py <transformation_matrix_file_path> [output_directory]")
        print("Example: python3 Iteratively_test.py /path/to/final_icp_transformation_matrix.txt")
        return False
    
    matrix_file_path = sys.argv[1]
    output_dir = sys.argv[2] if len(sys.argv) > 2 else None
    
    if not os.path.exists(matrix_file_path):
        print(f"Error: Transformation matrix file does not exist: {matrix_file_path}")
        return False
    script_dir = os.path.dirname(os.path.abspath(__file__))
    parent_dir = os.path.dirname(script_dir)
    result_dir = os.path.join(parent_dir, "results")
    # Derive result directory from transformation matrix file path
    #result_dir = Path(matrix_file_path).parent
    
    # Create validator and run validation
    validator = TransformationValidator(result_dir, output_dir)
    success = validator.run_validation(matrix_file_path)
    
    if success:
        print(f"Validation completed successfully! Results saved in: {validator.output_dir}")
        return True
    else:
        print("Validation failed, please check log file for detailed information")
        return False

if __name__ == "__main__":
    success = main()
    sys.exit(0 if success else 1)