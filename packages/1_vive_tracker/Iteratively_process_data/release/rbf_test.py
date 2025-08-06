#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Robust RANSAC+SVD+RBF Point Cloud Registration Method
Enhanced Version with Fixed RBF Parameter Grid
Author: Enhanced from Academic Benchmark Framework
"""

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from scipy.interpolate import RBFInterpolator
from scipy.spatial.distance import cdist
import logging
import time
from sklearn.model_selection import ParameterGrid
import warnings
warnings.filterwarnings('ignore')

# Setup logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

class RANSACFilter:
    """RANSAC-based data filtering for point cloud registration"""
    
    def __init__(self, min_samples=32, max_iterations=2000, inlier_threshold=0.02):
        self.min_samples = min_samples
        self.max_iterations = max_iterations
        self.inlier_threshold = inlier_threshold
        self.best_inliers = None
        self.best_transformation = None
        
    def compute_2d_svd_transformation(self, points_A, points_B):
        """Compute 2D SVD transformation (XZ plane only)"""
        points_A = np.array(points_A)
        points_B = np.array(points_B)
        
        # Use only XZ coordinates
        points_A_xz = points_A[:, [0, 2]]
        points_B_xz = points_B[:, [0, 2]]
        
        # Compute centroids
        centroid_A = np.mean(points_A_xz, axis=0)
        centroid_B = np.mean(points_B_xz, axis=0)
        
        # Center the points
        A_centered = points_A_xz - centroid_A
        B_centered = points_B_xz - centroid_B
        
        # SVD to compute rotation matrix
        H = np.dot(A_centered.T, B_centered)
        U, S, Vt = np.linalg.svd(H)
        R = np.dot(Vt.T, U.T)
        
        # Ensure proper rotation matrix
        if np.linalg.det(R) < 0:
            Vt[-1, :] *= -1
            R = np.dot(Vt.T, U.T)
        
        # Compute translation vector
        t = centroid_B - np.dot(R, centroid_A)
        
        return R, t
    
    def apply_2d_transformation(self, points, R, t):
        """Apply 2D transformation to point set"""
        points = np.array(points)
        points_xz = points[:, [0, 2]]
        
        # Apply transformation
        transformed_xz = np.dot(points_xz, R.T) + t
        
        # Reconstruct 3D points (keep Y=0)
        transformed = np.zeros_like(points)
        transformed[:, [0, 2]] = transformed_xz
        transformed[:, 1] = 0  # Y-axis constraint
        
        return transformed
    
    def count_inliers(self, positions_A, positions_B, R, t):
        """Count inliers"""
        # Apply transformation
        transformed_A = self.apply_2d_transformation(positions_A, R, t)
        
        # Compute distances after transformation (XZ plane only)
        distances = np.sqrt((transformed_A[:, 0] - positions_B[:, 0])**2 + 
                           (transformed_A[:, 2] - positions_B[:, 2])**2)
        
        # Count inliers
        inliers = distances < self.inlier_threshold
        
        return inliers, distances
    
    def ransac_filter(self, positions_A, positions_B):
        """Execute RANSAC filtering"""
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
        
        # RANSAC iterations
        for iteration in range(self.max_iterations):
            # Random sampling
            if len(positions_A) <= self.min_samples:
                sample_indices = np.arange(len(positions_A))
            else:
                sample_indices = np.random.choice(len(positions_A), self.min_samples, replace=False)
            
            sample_A = positions_A[sample_indices]
            sample_B = positions_B[sample_indices]
            
            try:
                # Compute transformation
                R, t = self.compute_2d_svd_transformation(sample_A, sample_B)
                
                # Validate transformation
                inliers, distances = self.count_inliers(positions_A, positions_B, R, t)
                inlier_count = np.sum(inliers)
                
                # Update best model
                if inlier_count > best_inlier_count:
                    best_inlier_count = inlier_count
                    best_inliers = inliers.copy()
                    best_R = R.copy()
                    best_t = t.copy()
                    
                    if iteration % 500 == 0:
                        logger.info(f"  Iteration {iteration}: New best model with {inlier_count}/{len(positions_A)} inliers ({inlier_count/len(positions_A)*100:.1f}%)")
                
            except Exception as e:
                if iteration % 500 == 0:
                    logger.warning(f"  Iteration {iteration}: Failed to compute transformation: {e}")
                continue
        
        if best_inliers is None:
            logger.error("RANSAC failed to find any valid transformation")
            return None, None, None, None
        
        # Extract best inliers
        inlier_A = positions_A[best_inliers]
        inlier_B = positions_B[best_inliers]
        outlier_A = positions_A[~best_inliers]
        outlier_B = positions_B[~best_inliers]
        
        logger.info(f"RANSAC completed:")
        logger.info(f"  - Best inlier count: {best_inlier_count}/{len(positions_A)} ({best_inlier_count/len(positions_A)*100:.1f}%)")
        logger.info(f"  - Outliers removed: {len(positions_A) - best_inlier_count}")
        
        # Store results
        self.best_inliers = best_inliers
        self.best_transformation = (best_R, best_t)
        
        return inlier_A, inlier_B, outlier_A, outlier_B

class RobustRANSACSVDRBF:
    """
    Robust RANSAC+SVD+RBF Point Cloud Registration Method
    With automatic RBF parameter tuning for improved robustness
    FIXED: Proper parameter grid handling for different kernel types
    """
    
    def __init__(self, 
                 ransac_min_samples=32,
                 ransac_max_iterations=2000,
                 ransac_inlier_threshold=0.02,
                 validation_split=0.2,
                 target_rmse=0.004):
        
        # RANSAC parameters
        self.ransac_min_samples = ransac_min_samples
        self.ransac_max_iterations = ransac_max_iterations
        self.ransac_inlier_threshold = ransac_inlier_threshold
        
        # Validation parameters
        self.validation_split = validation_split
        self.target_rmse = target_rmse
        
        # Model components
        self.ransac_filter = None
        self.transformation_matrix = None
        self.rbf_x = None
        self.rbf_z = None
        self.best_rbf_params = None
        
        # Performance tracking
        self.training_history = {}
        
    def generate_parameter_grid(self):
        """
        Generate proper parameter grid for different RBF kernels
        FIXED: Separate handling for kernels that need epsilon vs those that don't
        """
        # Kernels that don't need epsilon
        kernels_no_epsilon = ['thin_plate_spline', 'cubic', 'quintic', 'linear']
        # Kernels that require epsilon
        kernels_with_epsilon = ['multiquadric', 'inverse_multiquadric', 'gaussian']
        
        smoothing_values = [0.0001, 0.001, 0.01, 0.1, 0.5, 1.0]
        epsilon_values = [0.1, 1.0, 5.0, 10.0]
        
        param_combinations = []
        
        # Add combinations for kernels that don't need epsilon
        for kernel in kernels_no_epsilon:
            for smoothing in smoothing_values:
                param_combinations.append({
                    'kernel': kernel,
                    'smoothing': smoothing,
                    'epsilon': None
                })
        
        # Add combinations for kernels that require epsilon
        for kernel in kernels_with_epsilon:
            for smoothing in smoothing_values:
                for epsilon in epsilon_values:
                    param_combinations.append({
                        'kernel': kernel,
                        'smoothing': smoothing,
                        'epsilon': epsilon
                    })
        
        logger.info(f"Generated {len(param_combinations)} parameter combinations")
        logger.info(f"  - Kernels without epsilon: {len(kernels_no_epsilon)} x {len(smoothing_values)} = {len(kernels_no_epsilon) * len(smoothing_values)}")
        logger.info(f"  - Kernels with epsilon: {len(kernels_with_epsilon)} x {len(smoothing_values)} x {len(epsilon_values)} = {len(kernels_with_epsilon) * len(smoothing_values) * len(epsilon_values)}")
        
        return param_combinations
    
    def split_data(self, train_A, train_B, validation_split=None):
        """Split training data into train and validation sets"""
        if validation_split is None:
            validation_split = self.validation_split
            
        train_A = np.array(train_A)
        train_B = np.array(train_B)
        
        total_points = len(train_A)
        val_size = int(total_points * validation_split)
        
        # Random split
        np.random.seed(42)  # For reproducibility
        indices = np.random.permutation(total_points)
        
        val_indices = indices[:val_size]
        train_indices = indices[val_size:]
        
        train_A_split = train_A[train_indices]
        train_B_split = train_B[train_indices]
        val_A = train_A[val_indices]
        val_B = train_B[val_indices]
        
        logger.info(f"Data split: {len(train_A_split)} training, {len(val_A)} validation points")
        
        return train_A_split, train_B_split, val_A, val_B
    
    def compute_2d_svd_transformation(self, points_A, points_B):
        """Compute 2D SVD transformation matrix"""
        points_A = np.array(points_A)
        points_B = np.array(points_B)
        
        # Use only XZ coordinates
        points_A_xz = points_A[:, [0, 2]]
        points_B_xz = points_B[:, [0, 2]]
        
        # Compute centroids
        centroid_A = np.mean(points_A_xz, axis=0)
        centroid_B = np.mean(points_B_xz, axis=0)
        
        # Center the points
        A_centered = points_A_xz - centroid_A
        B_centered = points_B_xz - centroid_B
        
        # SVD to compute rotation matrix
        H = np.dot(A_centered.T, B_centered)
        U, S, Vt = np.linalg.svd(H)
        R = np.dot(Vt.T, U.T)
        
        # Ensure proper rotation matrix
        if np.linalg.det(R) < 0:
            Vt[-1, :] *= -1
            R = np.dot(Vt.T, U.T)
        
        # Compute translation vector
        t = centroid_B - np.dot(R, centroid_A)
        
        # Create 4x4 transformation matrix
        T = np.eye(4)
        T[0, 0] = R[0, 0]
        T[0, 2] = R[0, 1]
        T[2, 0] = R[1, 0]
        T[2, 2] = R[1, 1]
        T[0, 3] = t[0]
        T[2, 3] = t[1]
        
        return T
    
    def apply_transformation(self, points, T):
        """Apply transformation matrix to points"""
        points = np.array(points)
        points_homo = np.hstack([points, np.ones((points.shape[0], 1))])
        transformed = np.dot(T, points_homo.T).T
        result = transformed[:, :3]
        result[:, 1] = 0  # Ensure Y=0
        return result
    
    def calculate_rmse(self, predicted, actual):
        """Calculate RMSE (XZ plane only)"""
        predicted = np.array(predicted)
        actual = np.array(actual)
        
        lateral_errors = actual[:, 0] - predicted[:, 0]
        longitudinal_errors = actual[:, 2] - predicted[:, 2]
        
        overall_rmse = np.sqrt(np.mean(lateral_errors**2 + longitudinal_errors**2))
        return overall_rmse
    
    def create_rbf_interpolator(self, control_points, values, params):
        """Create RBF interpolator with proper parameter handling"""
        # Build RBF parameters
        rbf_params = {
            'kernel': params['kernel'],
            'smoothing': params['smoothing']
        }
        
        # Add epsilon only if the kernel requires it and it's provided
        if params['kernel'] not in ['thin_plate_spline', 'cubic', 'quintic', 'linear']:
            if params.get('epsilon') is not None:
                rbf_params['epsilon'] = params['epsilon']
            else:
                # This shouldn't happen with our fixed parameter grid, but just in case
                raise ValueError(f"Kernel {params['kernel']} requires epsilon parameter")
        
        return RBFInterpolator(control_points, values, **rbf_params)
    
    def tune_rbf_parameters(self, train_A_split, train_B_split, val_A, val_B):
        """
        Tune RBF parameters based on train/validation performance gap
        to improve model robustness
        FIXED: Proper parameter grid and progress reporting
        """
        logger.info("=== Starting RBF Parameter Tuning ===")
        
        # Generate proper parameter grid
        param_combinations = self.generate_parameter_grid()
        
        # Step 1: Apply RANSAC filtering to training split
        self.ransac_filter = RANSACFilter(
            min_samples=self.ransac_min_samples,
            max_iterations=self.ransac_max_iterations,
            inlier_threshold=self.ransac_inlier_threshold
        )
        
        train_inlier_A, train_inlier_B, _, _ = self.ransac_filter.ransac_filter(train_A_split, train_B_split)
        
        if train_inlier_A is None or len(train_inlier_A) == 0:
            logger.error("RANSAC failed to find inliers for parameter tuning")
            return None
        
        # Step 2: Compute SVD transformation
        T_svd = self.compute_2d_svd_transformation(train_inlier_A, train_inlier_B)
        
        # Step 3: Apply SVD to get coordinates in aligned space
        train_svd_transformed = self.apply_transformation(train_inlier_A, T_svd)
        train_residuals = train_inlier_B - train_svd_transformed
        train_residuals_xz = train_residuals[:, [0, 2]]
        svd_transformed_inlier_xz = train_svd_transformed[:, [0, 2]]
        
        # Store SVD transformation for later use
        self.transformation_matrix = T_svd
        
        # Step 4: Parameter grid search
        best_params = None
        best_score = float('inf')  # We want to minimize the robustness score
        best_train_rmse = float('inf')
        best_val_rmse = float('inf')
        
        successful_evaluations = 0
        failed_evaluations = 0
        
        logger.info(f"Testing {len(param_combinations)} parameter combinations...")
        
        for i, params in enumerate(param_combinations):
            try:
                # Create RBF interpolators with current parameters
                rbf_x = self.create_rbf_interpolator(
                    svd_transformed_inlier_xz,
                    train_residuals_xz[:, 0],
                    params
                )
                
                rbf_z = self.create_rbf_interpolator(
                    svd_transformed_inlier_xz,
                    train_residuals_xz[:, 1],
                    params
                )
                
                # Evaluate on training split (full training split, not just inliers)
                train_svd_full = self.apply_transformation(train_A_split, T_svd)
                train_svd_full_xz = train_svd_full[:, [0, 2]]
                
                train_deformation_x = rbf_x(train_svd_full_xz)
                train_deformation_z = rbf_z(train_svd_full_xz)
                
                train_final = train_svd_full.copy()
                train_final[:, 0] += train_deformation_x
                train_final[:, 2] += train_deformation_z
                train_final[:, 1] = 0
                
                train_rmse = self.calculate_rmse(train_final, train_B_split)
                
                # Evaluate on validation set
                val_svd = self.apply_transformation(val_A, T_svd)
                val_svd_xz = val_svd[:, [0, 2]]
                
                val_deformation_x = rbf_x(val_svd_xz)
                val_deformation_z = rbf_z(val_svd_xz)
                
                val_final = val_svd.copy()
                val_final[:, 0] += val_deformation_x
                val_final[:, 2] += val_deformation_z
                val_final[:, 1] = 0
                
                val_rmse = self.calculate_rmse(val_final, val_B)
                
                # Compute robustness score (penalize large train/val gap)
                rmse_gap = abs(val_rmse - train_rmse)
                avg_rmse = (train_rmse + val_rmse) / 2
                
                # Robustness score: balance between performance and generalization
                # Lower score is better
                robustness_score = avg_rmse + 0.5 * rmse_gap
                
                # Also consider if we meet target RMSE
                if val_rmse <= self.target_rmse:
                    robustness_score *= 0.8  # Bonus for meeting target
                
                if robustness_score < best_score:
                    best_score = robustness_score
                    best_params = params.copy()
                    best_train_rmse = train_rmse
                    best_val_rmse = val_rmse
                
                successful_evaluations += 1
                
                # Progress reporting (every 10%)
                if (i + 1) % max(1, len(param_combinations) // 10) == 0:
                    progress_pct = (i + 1) / len(param_combinations) * 100
                    logger.info(f"  Progress: {progress_pct:.1f}% ({i+1}/{len(param_combinations)})")
                    logger.info(f"    Current best score: {best_score:.6f}")
                    logger.info(f"    Successful evaluations: {successful_evaluations}")
                    logger.info(f"    Failed evaluations: {failed_evaluations}")
                
            except Exception as e:
                failed_evaluations += 1
                # Only log first few failures to avoid spam
                if failed_evaluations <= 5:
                    logger.warning(f"Failed to evaluate parameters {params}: {e}")
                elif failed_evaluations == 6:
                    logger.warning("Suppressing further parameter evaluation warnings...")
                continue
        
        if best_params is None:
            logger.error(f"No valid RBF parameters found. Successful: {successful_evaluations}, Failed: {failed_evaluations}")
            return None
        
        logger.info("=== RBF Parameter Tuning Results ===")
        logger.info(f"Total evaluations: {len(param_combinations)}")
        logger.info(f"Successful evaluations: {successful_evaluations}")
        logger.info(f"Failed evaluations: {failed_evaluations}")
        logger.info(f"Success rate: {successful_evaluations/len(param_combinations)*100:.1f}%")
        logger.info(f"Best parameters: {best_params}")
        logger.info(f"Best robustness score: {best_score:.6f}")
        logger.info(f"Training RMSE: {best_train_rmse*1000:.3f}mm")
        logger.info(f"Validation RMSE: {best_val_rmse*1000:.3f}mm")
        logger.info(f"RMSE gap: {abs(best_val_rmse - best_train_rmse)*1000:.3f}mm")
        
        self.best_rbf_params = best_params
        self.training_history = {
            'best_train_rmse': best_train_rmse,
            'best_val_rmse': best_val_rmse,
            'robustness_score': best_score,
            'rmse_gap': abs(best_val_rmse - best_train_rmse),
            'successful_evaluations': successful_evaluations,
            'failed_evaluations': failed_evaluations,
            'total_evaluations': len(param_combinations)
        }
        
        return best_params
    
    def train(self, train_A, train_B, auto_tune=True):
        """
        Train the complete RANSAC+SVD+RBF model
        """
        logger.info("=== Training Robust RANSAC+SVD+RBF Model ===")
        
        train_A = np.array(train_A)
        train_B = np.array(train_B)
        
        start_time = time.time()
        
        if auto_tune:
            # Split data for parameter tuning
            train_A_split, train_B_split, val_A, val_B = self.split_data(train_A, train_B)
            
            # Tune RBF parameters
            best_params = self.tune_rbf_parameters(train_A_split, train_B_split, val_A, val_B)
            
            if best_params is None:
                logger.error("Parameter tuning failed, using default parameters")
                best_params = {
                    'kernel': 'thin_plate_spline',
                    'smoothing': 0.001,
                    'epsilon': None
                }
        else:
            # Use default parameters
            best_params = {
                'kernel': 'thin_plate_spline',
                'smoothing': 0.001,
                'epsilon': None
            }
            
            # Apply RANSAC to full training data
            self.ransac_filter = RANSACFilter(
                min_samples=self.ransac_min_samples,
                max_iterations=self.ransac_max_iterations,
                inlier_threshold=self.ransac_inlier_threshold
            )
        
        # Final training on full dataset with best parameters
        logger.info("Training final model on full dataset...")
        
        if not auto_tune:
            # Need to do RANSAC filtering
            train_inlier_A, train_inlier_B, _, _ = self.ransac_filter.ransac_filter(train_A, train_B)
            
            if train_inlier_A is None or len(train_inlier_A) == 0:
                logger.error("RANSAC failed on full dataset")
                return False
            
            # Compute SVD transformation
            self.transformation_matrix = self.compute_2d_svd_transformation(train_inlier_A, train_inlier_B)
        else:
            # Re-apply RANSAC to full dataset
            train_inlier_A, train_inlier_B, _, _ = self.ransac_filter.ransac_filter(train_A, train_B)
            
            if train_inlier_A is None or len(train_inlier_A) == 0:
                logger.error("RANSAC failed on full dataset")
                return False
            
            # Re-compute SVD transformation on full dataset
            self.transformation_matrix = self.compute_2d_svd_transformation(train_inlier_A, train_inlier_B)
        
        # Apply SVD transformation
        train_svd_transformed = self.apply_transformation(train_inlier_A, self.transformation_matrix)
        train_residuals = train_inlier_B - train_svd_transformed
        train_residuals_xz = train_residuals[:, [0, 2]]
        svd_transformed_inlier_xz = train_svd_transformed[:, [0, 2]]
        
        # Build final RBF models with best parameters
        self.rbf_x = self.create_rbf_interpolator(
            svd_transformed_inlier_xz,
            train_residuals_xz[:, 0],
            best_params
        )
        
        self.rbf_z = self.create_rbf_interpolator(
            svd_transformed_inlier_xz,
            train_residuals_xz[:, 1],
            best_params
        )
        
        self.best_rbf_params = best_params
        
        training_time = time.time() - start_time
        
        logger.info(f"Training completed in {training_time:.3f}s")
        logger.info(f"Final RBF parameters: {best_params}")
        
        return True
    
    def predict(self, test_A):
        """
        Predict using the trained model
        """
        if self.transformation_matrix is None or self.rbf_x is None or self.rbf_z is None:
            logger.error("Model not trained. Call train() first.")
            return None
        
        test_A = np.array(test_A)
        
        # Step 1: Apply SVD transformation
        test_svd_transformed = self.apply_transformation(test_A, self.transformation_matrix)
        
        # Step 2: Predict deformation using RBF
        test_svd_transformed_xz = test_svd_transformed[:, [0, 2]]
        deformation_x = self.rbf_x(test_svd_transformed_xz)
        deformation_z = self.rbf_z(test_svd_transformed_xz)
        
        # Step 3: Apply deformation correction
        test_final = test_svd_transformed.copy()
        test_final[:, 0] += deformation_x
        test_final[:, 2] += deformation_z
        test_final[:, 1] = 0  # Ensure Y=0
        
        return test_final
    
    def evaluate(self, test_A, test_B):
        """
        Evaluate the model performance
        """
        predictions = self.predict(test_A)
        
        if predictions is None:
            return None
        
        test_B = np.array(test_B)
        
        # Calculate metrics
        lateral_errors = test_B[:, 0] - predictions[:, 0]
        longitudinal_errors = test_B[:, 2] - predictions[:, 2]
        
        overall_rmse = np.sqrt(np.mean(lateral_errors**2 + longitudinal_errors**2))
        lateral_rmse = np.sqrt(np.mean(lateral_errors**2))
        longitudinal_rmse = np.sqrt(np.mean(longitudinal_errors**2))
        
        overall_mae = np.mean(np.sqrt(lateral_errors**2 + longitudinal_errors**2))
        lateral_mae = np.mean(np.abs(lateral_errors))
        longitudinal_mae = np.mean(np.abs(longitudinal_errors))
        
        metrics = {
            'overall_rmse': overall_rmse,
            'lateral_rmse': lateral_rmse,
            'longitudinal_rmse': longitudinal_rmse,
            'overall_mae': overall_mae,
            'lateral_mae': lateral_mae,
            'longitudinal_mae': longitudinal_mae,
            'residual_magnitudes': np.sqrt(lateral_errors**2 + longitudinal_errors**2)
        }
        
        return metrics
    
    def plot_training_analysis(self, save_path=None):
        """
        Plot training analysis including parameter tuning results
        ENHANCED: Better visualization of parameter tuning process
        """
        if not self.training_history:
            logger.warning("No training history available for plotting")
            return
        
        fig, axes = plt.subplots(2, 3, figsize=(18, 10))
        
        # Plot 1: Training vs Validation RMSE
        ax1 = axes[0, 0]
        rmse_values = [self.training_history['best_train_rmse']*1000, 
                      self.training_history['best_val_rmse']*1000]
        rmse_labels = ['Training', 'Validation']
        
        bars = ax1.bar(rmse_labels, rmse_values, color=['lightblue', 'lightcoral'], alpha=0.7)
        ax1.axhline(y=self.target_rmse*1000, color='red', linestyle='--', 
                   label=f'Target: {self.target_rmse*1000:.1f}mm')
        ax1.set_ylabel('RMSE [mm]')
        ax1.set_title('Training vs Validation Performance')
        ax1.legend()
        ax1.grid(True, alpha=0.3)
        
        # Add value labels on bars
        for bar, value in zip(bars, rmse_values):
            ax1.text(bar.get_x() + bar.get_width()/2, bar.get_height() + 0.1,
                    f'{value:.3f}mm', ha='center', va='bottom', fontsize=9)
        
        # Plot 2: Robustness Metrics
        ax2 = axes[0, 1]
        metrics = ['RMSE Gap', 'Robustness Score']
        values = [self.training_history['rmse_gap']*1000, 
                 self.training_history['robustness_score']*1000]
        
        bars2 = ax2.bar(metrics, values, color=['orange', 'purple'], alpha=0.7)
        ax2.set_ylabel('Value [mm]')
        ax2.set_title('Robustness Metrics')
        ax2.grid(True, alpha=0.3)
        
        for bar, value in zip(bars2, values):
            ax2.text(bar.get_x() + bar.get_width()/2, bar.get_height() + 0.01,
                    f'{value:.3f}', ha='center', va='bottom', fontsize=9)
        
        # Plot 3: Parameter Tuning Success Rate
        ax3 = axes[0, 2]
        success_rate = self.training_history['successful_evaluations'] / self.training_history['total_evaluations']
        failure_rate = 1 - success_rate
        
        wedges, texts, autotexts = ax3.pie([success_rate, failure_rate], 
                                          labels=['Successful', 'Failed'],
                                          colors=['lightgreen', 'lightcoral'],
                                          autopct='%1.1f%%',
                                          startangle=90)
        ax3.set_title('Parameter Evaluation Success Rate')
        
        # Plot 4: RBF Parameters
        ax4 = axes[1, 0]
        if self.best_rbf_params:
            param_text = f"Optimal RBF Parameters:\n\n"
            param_text += f"Kernel: {self.best_rbf_params['kernel']}\n"
            param_text += f"Smoothing: {self.best_rbf_params['smoothing']}\n"
            if self.best_rbf_params.get('epsilon') is not None:
                param_text += f"Epsilon: {self.best_rbf_params['epsilon']}\n"
            
            param_text += f"\nEvaluations:\n"
            param_text += f"Total: {self.training_history['total_evaluations']}\n"
            param_text += f"Successful: {self.training_history['successful_evaluations']}\n"
            param_text += f"Failed: {self.training_history['failed_evaluations']}"
            
            ax4.text(0.1, 0.5, param_text, transform=ax4.transAxes, 
                    fontsize=10, verticalalignment='center',
                    bbox=dict(boxstyle="round,pad=0.3", facecolor="lightgray"))
            ax4.set_title('Parameter Tuning Details')
            ax4.axis('off')
        
        # Plot 5: Performance Summary
        ax5 = axes[1, 1]
        summary_text = f"Model Performance Summary\n\n"
        summary_text += f"Training RMSE: {self.training_history['best_train_rmse']*1000:.3f}mm\n"
        summary_text += f"Validation RMSE: {self.training_history['best_val_rmse']*1000:.3f}mm\n"
        summary_text += f"RMSE Gap: {self.training_history['rmse_gap']*1000:.3f}mm\n"
        summary_text += f"Target RMSE: {self.target_rmse*1000:.1f}mm\n\n"
        
        meets_target = self.training_history['best_val_rmse'] <= self.target_rmse
        summary_text += f"Meets Target: {'✓ Yes' if meets_target else '✗ No'}\n"
        
        robustness_rating = "High" if self.training_history['rmse_gap'] < 0.001 else \
                           "Medium" if self.training_history['rmse_gap'] < 0.003 else "Low"
        summary_text += f"Robustness: {robustness_rating}\n"
        
        success_rate = self.training_history['successful_evaluations'] / self.training_history['total_evaluations']
        summary_text += f"Tuning Success Rate: {success_rate*100:.1f}%"
        
        ax5.text(0.1, 0.5, summary_text, transform=ax5.transAxes, 
                fontsize=10, verticalalignment='center',
                bbox=dict(boxstyle="round,pad=0.3", 
                         facecolor="lightgreen" if meets_target else "lightyellow"))
        ax5.set_title('Performance Summary')
        ax5.axis('off')
        
        # Plot 6: Model Architecture Diagram
        ax6 = axes[1, 2]
        ax6.text(0.5, 0.8, 'RANSAC+SVD+RBF Pipeline', 
                transform=ax6.transAxes, ha='center', fontsize=12, fontweight='bold')
        
        # Draw pipeline steps
        steps = ['Input\nPoint Clouds', 'RANSAC\nFiltering', 'SVD\nAlignment', 
                'RBF\nDeformation', 'Final\nRegistration']
        y_positions = [0.65, 0.5, 0.35, 0.2, 0.05]
        
        for i, (step, y_pos) in enumerate(zip(steps, y_positions)):
            # Draw box
            bbox = dict(boxstyle="round,pad=0.2", facecolor="lightblue", alpha=0.7)
            ax6.text(0.5, y_pos, step, transform=ax6.transAxes, ha='center', va='center',
                    bbox=bbox, fontsize=9)
            
            # Draw arrow to next step
            if i < len(steps) - 1:
                ax6.annotate('', xy=(0.5, y_positions[i+1] + 0.04), 
                           xytext=(0.5, y_pos - 0.04),
                           xycoords='axes fraction', textcoords='axes fraction',
                           arrowprops=dict(arrowstyle='->', lw=1.5, color='darkblue'))
        
        ax6.set_title('Model Architecture')
        ax6.axis('off')
        
        plt.tight_layout()
        
        if save_path:
            plt.savefig(save_path, dpi=300, bbox_inches='tight')
            logger.info(f"Training analysis plot saved to {save_path}")
        
        plt.show()

# Example usage
if __name__ == "__main__":
    # Example data loading (replace with your actual data)
    def load_example_data():
        """Load example data for demonstration"""
        # Generate synthetic data for demonstration
        np.random.seed(42)
        n_points = 100
        
        # Create some reference points
        source_points = np.random.rand(n_points, 3) * 10
        source_points[:, 1] = 0  # Keep Y=0
        
        # Create target points with some transformation and noise
        target_points = source_points.copy()
        
        # Apply rotation and translation
        angle = np.pi / 6  # 30 degrees
        cos_a, sin_a = np.cos(angle), np.sin(angle)
        rotation_matrix = np.array([[cos_a, -sin_a], [sin_a, cos_a]])
        
        target_points[:, [0, 2]] = np.dot(source_points[:, [0, 2]], rotation_matrix.T)
        target_points[:, 0] += 2.0  # Translation
        target_points[:, 2] += 1.5
        
        # Add some noise and local deformation
        noise = np.random.normal(0, 0.01, (n_points, 3))
        noise[:, 1] = 0  # Keep Y=0
        target_points += noise
        
        # Add some local deformation
        for i in range(n_points):
            x, z = target_points[i, 0], target_points[i, 2]
            # Sinusoidal deformation
            target_points[i, 0] += 0.02 * np.sin(z * 2)
            target_points[i, 2] += 0.015 * np.cos(x * 1.5)
        
        return source_points, target_points
    
    # Load data
    logger.info("Loading example data...")
    source_points, target_points = load_example_data()
    
    # Split into train and test
    train_size = int(len(source_points) * 0.8)
    train_source = source_points[:train_size]
    train_target = target_points[:train_size]
    test_source = source_points[train_size:]
    test_target = target_points[train_size:]
    
    logger.info(f"Data split: {len(train_source)} training, {len(test_source)} test points")
    
    # Initialize and train the model
    model = RobustRANSACSVDRBF(
        ransac_min_samples=16,
        ransac_max_iterations=1000,
        ransac_inlier_threshold=0.05,
        validation_split=0.2,
        target_rmse=0.01
    )
    
    # Train with automatic parameter tuning
    logger.info("Training model with automatic RBF parameter tuning...")
    success = model.train(train_source, train_target, auto_tune=True)
    
    if success:
        # Evaluate on test set
        logger.info("Evaluating on test set...")
        test_metrics = model.evaluate(test_source, test_target)
        
        if test_metrics:
            logger.info("=== Test Results ===")
            logger.info(f"Overall RMSE: {test_metrics['overall_rmse']*1000:.3f}mm")
            logger.info(f"Lateral RMSE: {test_metrics['lateral_rmse']*1000:.3f}mm")
            logger.info(f"Longitudinal RMSE: {test_metrics['longitudinal_rmse']*1000:.3f}mm")
            logger.info(f"Overall MAE: {test_metrics['overall_mae']*1000:.3f}mm")
            
            # Plot training analysis
            model.plot_training_analysis()
        else:
            logger.error("Evaluation failed")
    else:
        logger.error("Training failed")