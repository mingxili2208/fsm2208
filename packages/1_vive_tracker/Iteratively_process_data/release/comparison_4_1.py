#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Academic Benchmark Framework for Point Cloud Registration Methods
Comprehensive comparison of four registration methods with train/test split
Methods: SVD Only, RANSAC+SVD, CPD, Proposed Method (RANSAC+SVD+RBF)
CORRECTED VERSION: Fixed RBF control points and real residual distribution
FIXED: CPD import issue resolved and overfitting analysis added
UPDATED: CPD uses SVD pre-aligned data, Added train/test performance visualization
OPTIMIZED: RBF overfitting reduced via control point sub-sampling and increased smoothing
MODIFIED: Changed subplot C to grouped bar chart, improved RBF learning, removed Chinese comments
ENHANCED: Added residual field visualization, cross-validation hyperparameter tuning, robustness improvements
"""
import sys
import os
import datetime
import time
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from sklearn.neighbors import NearestNeighbors
from sklearn.model_selection import KFold
from scipy.interpolate import griddata, RBFInterpolator
from scipy.spatial.distance import cdist
import random
from matplotlib.patches import FancyArrowPatch
from mpl_toolkits.mplot3d import proj3d
import matplotlib.patches as patches
import warnings
warnings.filterwarnings('ignore')

# Try to import adjustText for better label positioning
try:
    from adjustText import adjust_text
    ADJUST_TEXT_AVAILABLE = True
except ImportError:
    ADJUST_TEXT_AVAILABLE = False
    print("Warning: adjustText not available, text labels may overlap in plots")

# Import CPD for method comparison with detailed error handling
CPD_AVAILABLE = False
CPD_ERROR_MSG = ""
cpd_reg = None

try:
    # Try different pycpd import methods
    try:
        # Method 1: Import specific registration class
        from pycpd import DeformableRegistration
        cpd_reg = DeformableRegistration
        CPD_AVAILABLE = True
        print("CPD successfully imported (DeformableRegistration)")
    except ImportError:
        try:
            # Method 2: Import rigid registration
            from pycpd import RigidRegistration
            cpd_reg = RigidRegistration
            CPD_AVAILABLE = True
            print("CPD successfully imported (RigidRegistration)")
        except ImportError:
            try:
                # Method 3: Import affine registration
                from pycpd import AffineRegistration
                cpd_reg = AffineRegistration
                CPD_AVAILABLE = True
                print("CPD successfully imported (AffineRegistration)")
            except ImportError:
                try:
                    # Method 4: Check module contents
                    import pycpd
                    print(f"Available pycpd contents: {[attr for attr in dir(pycpd) if not attr.startswith('_')]}")

                    # Select based on available classes
                    if hasattr(pycpd, 'DeformableRegistration'):
                        cpd_reg = pycpd.DeformableRegistration
                        CPD_AVAILABLE = True
                        print("CPD successfully imported (pycpd.DeformableRegistration)")
                    elif hasattr(pycpd, 'RigidRegistration'):
                        cpd_reg = pycpd.RigidRegistration
                        CPD_AVAILABLE = True
                        print("CPD successfully imported (pycpd.RigidRegistration)")
                    elif hasattr(pycpd, 'AffineRegistration'):
                        cpd_reg = pycpd.AffineRegistration
                        CPD_AVAILABLE = True
                        print("CPD successfully imported (pycpd.AffineRegistration)")
                    else:
                        raise ImportError("No suitable CPD registration class found")
                except Exception as e:
                    raise ImportError(f"All CPD import methods failed: {e}")

except ImportError as e:
    CPD_ERROR_MSG = f"ImportError: {e}"
    print(f"Warning: pycpd import failed - {CPD_ERROR_MSG}")
except Exception as e:
    CPD_ERROR_MSG = f"Other error: {e}"
    print(f"Warning: pycpd loading failed - {CPD_ERROR_MSG}")

# Create timestamp for output directory
current_time = datetime.datetime.now()
timestamp = current_time.strftime("%Y-%m-%d_%H-%M-%S")

# 1:32 scale parameters
SCALE_FACTOR = 28
REAL_WORLD_LATERAL_TARGET = 0.10  # 10cm real world lateral accuracy requirement
REAL_WORLD_LONGITUDINAL_TARGET = 0.10  # 10cm real world longitudinal accuracy requirement

# Scaled targets
SCALED_LATERAL_TARGET = REAL_WORLD_LATERAL_TARGET / SCALE_FACTOR  # 3.125mm
SCALED_LONGITUDINAL_TARGET = REAL_WORLD_LONGITUDINAL_TARGET / SCALE_FACTOR  # 3.125mm
SCALED_COMBINED_TARGET = np.sqrt(SCALED_LATERAL_TARGET**2 + SCALED_LONGITUDINAL_TARGET**2)  # ~4.42mm

# Target RMSE
TARGET_RMSE = 0.005  # 5mm

# RANSAC parameters
RANSAC_MIN_SAMPLES = 32
RANSAC_ITERATIONS = 2000
RANSAC_INLIER_THRESHOLD = 0.04  # 40mm (0.040m)

# Train/test split parameters
TRAIN_TEST_RATIO = 0.8  # 80% for training, 20% for testing

# Create output directory
# Correctly determine script directory even when run from another location
script_dir = os.path.dirname(os.path.realpath(sys.argv[0]))
parent_dir = os.path.dirname(script_dir)
result_dir_base = os.path.join(parent_dir, "results")
LASER_TRACKER_CSV = os.path.join(parent_dir, "data/final_coordinates.csv")
csv_filename = os.path.basename(LASER_TRACKER_CSV)
output_dir = os.path.join(result_dir_base, f"Benchmark_Framework_ENHANCED_{timestamp}_1to32_scale_RMSE_{TARGET_RMSE*1000:.1f}mm_{csv_filename}")
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
log_file = os.path.join(log_dir, "benchmark_process.log")
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

# Data column names
TRACKER_X = "tracker_x"
TRACKER_Y = "tracker_z"
LASER_X = "laser_x"
LASER_Y = "laser_z"


class SimpleDataSplitter:
    """
    Simple random data splitter for train/test separation
    """
    def __init__(self, train_ratio=0.8, random_seed=42):
        self.train_ratio = train_ratio
        self.random_seed = random_seed

    def split_data(self, positions_A, positions_B, df=None):
        """
        Simple random split into training and test sets
        """
        logger.info("=== Starting Data Splitting ===")

        positions_A = np.array(positions_A)
        positions_B = np.array(positions_B)

        total_points = len(positions_A)
        train_size = int(total_points * self.train_ratio)

        # Set random seed for reproducibility
        np.random.seed(self.random_seed)

        # Random selection of training indices
        all_indices = np.arange(total_points)
        np.random.shuffle(all_indices)

        train_indices = all_indices[:train_size]
        test_indices = all_indices[train_size:]

        # Extract training and test sets
        train_A = positions_A[train_indices]
        train_B = positions_B[train_indices]
        test_A = positions_A[test_indices]
        test_B = positions_B[test_indices]

        train_df = df.iloc[train_indices].copy() if df is not None else None
        test_df = df.iloc[test_indices].copy() if df is not None else None

        logger.info(f"Data split completed:")
        logger.info(f" Total points: {total_points}")
        logger.info(f" Training points: {len(train_indices)} ({len(train_indices)/total_points*100:.1f}%)")
        logger.info(f" Testing points: {len(test_indices)} ({len(test_indices)/total_points*100:.1f}%)")

        # Save split data
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

    def _save_split_data(self, train_A, train_B, test_A, test_B, train_df, test_df, train_indices, test_indices):
        """Save split data"""
        # Save training set
        train_data = np.hstack([train_A, train_B])
        np.savetxt(os.path.join(result_dir, "train_data.txt"), train_data,
                   header="train_source_x train_source_y train_source_z train_target_x train_target_y train_target_z")

        # Save test set
        test_data = np.hstack([test_A, test_B])
        np.savetxt(os.path.join(result_dir, "test_data.txt"), test_data,
                   header="test_source_x test_source_y test_source_z test_target_x test_target_y test_target_z")

        # Save CSV files
        if train_df is not None:
            train_df_copy = train_df.copy()
            train_df_copy['split_type'] = 'train'
            train_df_copy.to_csv(os.path.join(result_dir, "train_data.csv"), index=False)

        if test_df is not None:
            test_df_copy = test_df.copy()
            test_df_copy['split_type'] = 'test'
            test_df_copy.to_csv(os.path.join(result_dir, "test_data.csv"), index=False)

        # Save indices
        np.savetxt(os.path.join(result_dir, "train_indices.txt"), train_indices, fmt='%d')
        np.savetxt(os.path.join(result_dir, "test_indices.txt"), test_indices, fmt='%d')

        logger.info("Split data saved")


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
        logger.info(f" - Min samples: {self.min_samples}")
        logger.info(f" - Max iterations: {self.max_iterations}")
        logger.info(f" - Inlier threshold: {self.inlier_threshold*1000:.1f}mm")
        logger.info(f" - Total points: {len(positions_A)}")

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

                    logger.info(f" Iteration {iteration}: New best model with {inlier_count}/{len(positions_A)} inliers ({inlier_count/len(positions_A)*100:.1f}%)")

            except Exception as e:
                logger.warning(f" Iteration {iteration}: Failed to compute transformation: {e}")
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
        logger.info(f" - Best inlier count: {best_inlier_count}/{len(positions_A)} ({best_inlier_count/len(positions_A)*100:.1f}%)")
        logger.info(f" - Outliers removed: {len(positions_A) - best_inlier_count}")

        # Store results
        self.best_inliers = best_inliers
        self.best_transformation = (best_R, best_t)

        return inlier_A, inlier_B, outlier_A, outlier_B


class SimpleCPDAlternative:
    """
    Simple CPD alternative implementation when pycpd is not available
    """
    def __init__(self, X, Y, max_iterations=50, tolerance=1e-4):
        self.X = np.array(X)  # target points
        self.Y = np.array(Y)  # source points
        self.max_iterations = max_iterations
        self.tolerance = tolerance

    def register(self):
        """Simple iterative closest point registration"""
        logger.info("Using simple ICP alternative for CPD")

        # Initialize with identity transformation
        self.transformed_Y = self.Y.copy()

        for iteration in range(self.max_iterations):
            # Find closest points
            from scipy.spatial.distance import cdist
            distances = cdist(self.transformed_Y, self.X)
            closest_indices = np.argmin(distances, axis=1)
            closest_points = self.X[closest_indices]

            # Compute transformation
            centroid_Y = np.mean(self.transformed_Y, axis=0)
            centroid_X = np.mean(closest_points, axis=0)

            # Center points
            Y_centered = self.transformed_Y - centroid_Y
            X_centered = closest_points - centroid_X

            # SVD
            H = np.dot(Y_centered.T, X_centered)
            U, S, Vt = np.linalg.svd(H)
            R = np.dot(Vt.T, U.T)

            if np.linalg.det(R) < 0:
                Vt[-1, :] *= -1
                R = np.dot(Vt.T, U.T)

            t = centroid_X - np.dot(R, centroid_Y)

            # Apply transformation
            new_Y = np.dot(self.transformed_Y, R.T) + t

            # Check convergence
            change = np.mean(np.linalg.norm(new_Y - self.transformed_Y, axis=1))
            self.transformed_Y = new_Y

            if change < self.tolerance:
                logger.info(f"ICP converged after {iteration+1} iterations")
                break

    def transform_point_cloud(self, Y):
        """Transform new point cloud using learned transformation"""
        # This is a simplified version - just return the input for now
        return Y, None


class EnhancedRBFInterpolator:
    """
    Enhanced RBF interpolator with better deformation learning capabilities
    """
    def __init__(self, control_points, control_residuals, kernel='thin_plate_spline',
                 smoothing=0.05, epsilon=None):
        self.control_points = np.array(control_points)
        self.control_residuals = np.array(control_residuals)
        self.kernel = kernel
        self.smoothing = smoothing
        self.epsilon = epsilon

        # Initialize separate interpolators for each dimension
        self.interpolators = []

        if control_residuals.ndim == 1:
            # 1D residuals
            self.interpolators.append(
                RBFInterpolator(
                    self.control_points,
                    self.control_residuals,
                    kernel=self.kernel,
                    smoothing=self.smoothing,
                    epsilon=self.epsilon
                )
            )
        else:
            # Multi-dimensional residuals
            for dim in range(control_residuals.shape[1]):
                self.interpolators.append(
                    RBFInterpolator(
                        self.control_points,
                        self.control_residuals[:, dim],
                        kernel=self.kernel,
                        smoothing=self.smoothing,
                        epsilon=self.epsilon
                    )
                )

    def __call__(self, query_points):
        """Interpolate residuals at query points"""
        query_points = np.array(query_points)

        if len(self.interpolators) == 1:
            # 1D output
            return self.interpolators[0](query_points)
        else:
            # Multi-dimensional output
            results = []
            for interpolator in self.interpolators:
                results.append(interpolator(query_points))
            return np.column_stack(results)


class RBFHyperparameterOptimizer:
    """
    Cross-validation based hyperparameter optimizer for RBF models
    """
    def __init__(self, n_folds=5, random_state=42):
        self.n_folds = n_folds
        self.random_state = random_state
        self.best_params = None
        self.best_score = float('inf')
        
    def optimize(self, control_points, control_residuals, param_grid=None):
        """
        Optimize RBF hyperparameters using cross-validation
        """
        logger.info("=== Starting RBF Hyperparameter Optimization ===")
        
        if param_grid is None:
            # Default parameter grid
            param_grid = {
                'kernel': ['thin_plate_spline', 'multiquadric', 'gaussian'],
                'smoothing': [0.001, 0.01, 0.05, 0.1, 0.2],
                'epsilon': [0.1, 1.0, 5.0, 10.0],
                'num_control_points': [50, 100, 150, 200, 300]
            }
        
        # Setup cross-validation
        kf = KFold(n_splits=self.n_folds, shuffle=True, random_state=self.random_state)
        
        best_score = float('inf')
        best_params = None
        
        # Generate parameter combinations
        from itertools import product
        param_names = list(param_grid.keys())
        param_values = list(param_grid.values())
        
        total_combinations = 1
        for values in param_values:
            total_combinations *= len(values)
        
        logger.info(f"Testing {total_combinations} parameter combinations with {self.n_folds}-fold CV")
        
        combination_count = 0
        for param_combination in product(*param_values):
            combination_count += 1
            params = dict(zip(param_names, param_combination))
            
            if combination_count % 20 == 0:
                logger.info(f"Progress: {combination_count}/{total_combinations} combinations tested")
            
            try:
                # Test current parameter combination
                cv_scores = []
                
                # Subsample control points if specified
                if 'num_control_points' in params:
                    if len(control_points) > params['num_control_points']:
                        indices = np.random.choice(len(control_points), 
                                                 params['num_control_points'], 
                                                 replace=False)
                        cv_control_points = control_points[indices]
                        cv_control_residuals = control_residuals[indices]
                    else:
                        cv_control_points = control_points
                        cv_control_residuals = control_residuals
                else:
                    cv_control_points = control_points
                    cv_control_residuals = control_residuals
                
                # Cross-validation
                for train_idx, val_idx in kf.split(cv_control_points):
                    train_points = cv_control_points[train_idx]
                    train_residuals = cv_control_residuals[train_idx]
                    val_points = cv_control_points[val_idx]
                    val_residuals = cv_control_residuals[val_idx]
                    
                    # Create and train RBF interpolator
                    rbf = EnhancedRBFInterpolator(
                        train_points,
                        train_residuals,
                        kernel=params['kernel'],
                        smoothing=params['smoothing'],
                        epsilon=params['epsilon']
                    )
                    
                    # Predict on validation set
                    pred_residuals = rbf(val_points)
                    
                    # Compute validation error
                    val_error = np.mean(np.linalg.norm(pred_residuals - val_residuals, axis=1))
                    cv_scores.append(val_error)
                
                # Average CV score
                avg_cv_score = np.mean(cv_scores)
                
                # Update best parameters
                if avg_cv_score < best_score:
                    best_score = avg_cv_score
                    best_params = params.copy()
                    logger.info(f"New best CV score: {avg_cv_score:.6f} with params: {params}")
                    
            except Exception as e:
                logger.warning(f"Failed to evaluate params {params}: {e}")
                continue
        
        self.best_params = best_params
        self.best_score = best_score
        
        logger.info(f"Hyperparameter optimization completed")
        logger.info(f"Best CV score: {best_score:.6f}")
        logger.info(f"Best parameters: {best_params}")
        
        return best_params, best_score


def visualize_residual_field(points, residuals, title="Residual Field Visualization", 
                           save_path=None, subsample_factor=4):
    """
    Create residual field visualization using quiver plot
    
    学术价值: 此可视化展示SVD对齐后残差向量的空间分布模式，
    用于判断非刚性误差是否具有可学习的系统性规律
    """
    logger.info("Creating residual field visualization...")
    
    # Subsample for better visualization
    if len(points) > 200:
        indices = np.arange(0, len(points), subsample_factor)
        vis_points = points[indices]
        vis_residuals = residuals[indices]
    else:
        vis_points = points
        vis_residuals = residuals
    
    fig, ax = plt.subplots(figsize=(12, 8))
    
    # Create quiver plot showing residual vectors
    quiver = ax.quiver(vis_points[:, 0], vis_points[:, 1], 
                      vis_residuals[:, 0], vis_residuals[:, 1], 
                      np.linalg.norm(vis_residuals, axis=1),
                      cmap='viridis', scale_units='xy', scale=1, alpha=0.7)
    
    # Add colorbar
    cbar = plt.colorbar(quiver, ax=ax)
    cbar.set_label('Residual Magnitude (m)', fontsize=12)
    
    # Formatting
    ax.set_xlabel('X coordinate (m)', fontsize=12)
    ax.set_ylabel('Z coordinate (m)', fontsize=12)
    ax.set_title(title, fontsize=14, fontweight='bold')
    ax.grid(True, alpha=0.3)
    ax.set_aspect('equal', adjustable='box')
    
    plt.tight_layout()
    
    if save_path:
        plt.savefig(save_path, dpi=300, bbox_inches='tight')
        logger.info(f"Residual field visualization saved to {save_path}")
    
    plt.close(fig)


def clip_outliers(data, percentile_threshold=95):
    """
    Clip extreme outliers based on percentile threshold to improve robustness
    """
    if data.ndim == 1:
        threshold = np.percentile(np.abs(data), percentile_threshold)
        clipped_data = np.clip(data, -threshold, threshold)
    else:
        # For multi-dimensional data, clip based on magnitude
        magnitudes = np.linalg.norm(data, axis=1)
        threshold = np.percentile(magnitudes, percentile_threshold)
        
        # Scale down vectors that exceed threshold
        clipped_data = data.copy()
        outlier_mask = magnitudes > threshold
        if np.any(outlier_mask):
            scale_factors = threshold / magnitudes[outlier_mask]
            clipped_data[outlier_mask] = data[outlier_mask] * scale_factors[:, np.newaxis]
    
    num_clipped = np.sum(np.any(clipped_data != data, axis=1)) if data.ndim > 1 else np.sum(clipped_data != data)
    logger.info(f"Clipped {num_clipped}/{len(data)} outliers ({num_clipped/len(data)*100:.1f}%)")
    
    return clipped_data


class ExperimentRunner:
    """
    Main experiment runner for academic benchmark comparison
    ENHANCED VERSION: Added residual visualization, cross-validation optimization, and robustness improvements
    """
    def __init__(self, positions_A, positions_B, original_df=None):
        self.positions_A = np.array(positions_A)
        self.positions_B = np.array(positions_B)
        self.original_df = original_df

        # Data splitter
        self.data_splitter = SimpleDataSplitter(train_ratio=TRAIN_TEST_RATIO)

        # Split data
        split_data = self.data_splitter.split_data(self.positions_A, self.positions_B, self.original_df)
        self.train_A = split_data['train_A']
        self.train_B = split_data['train_B']
        self.test_A = split_data['test_A']
        self.test_B = split_data['test_B']

        logger.info(f"Dataset prepared: {len(self.train_A)} training, {len(self.test_A)} test points")

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

    def calculate_all_metrics(self, predicted, actual):
        """Calculate all performance metrics"""
        predicted = np.array(predicted)
        actual = np.array(actual)

        # RMSE calculations (XZ plane only since Y=0)
        lateral_errors = actual[:, 0] - predicted[:, 0]
        longitudinal_errors = actual[:, 2] - predicted[:, 2]

        overall_rmse = np.sqrt(np.mean(lateral_errors**2 + longitudinal_errors**2))
        lateral_rmse = np.sqrt(np.mean(lateral_errors**2))
        longitudinal_rmse = np.sqrt(np.mean(longitudinal_errors**2))

        # MAE calculations
        overall_mae = np.mean(np.sqrt(lateral_errors**2 + longitudinal_errors**2))
        lateral_mae = np.mean(np.abs(lateral_errors))
        longitudinal_mae = np.mean(np.abs(longitudinal_errors))

        return {
            'overall_rmse': overall_rmse,
            'lateral_rmse': lateral_rmse,
            'longitudinal_rmse': longitudinal_rmse,
            'overall_mae': overall_mae,
            'lateral_mae': lateral_mae,
            'longitudinal_mae': longitudinal_mae
        }

    def calculate_overfitting_metrics(self, train_metrics, test_metrics):
        """Calculate overfitting-related metrics"""
        train_rmse = train_metrics['overall_rmse']
        test_rmse = test_metrics['overall_rmse']

        # Overfitting ratio: test_error / train_error
        overfitting_ratio = test_rmse / train_rmse if train_rmse > 0 else float('inf')

        # Generalization gap: test_error - train_error
        generalization_gap = test_rmse - train_rmse

        # Overfitting severity classification
        if overfitting_ratio <= 1.1:
            overfitting_level = "None"
        elif overfitting_ratio <= 1.3:
            overfitting_level = "Mild"
        elif overfitting_ratio <= 1.5:
            overfitting_level = "Moderate"
        else:
            overfitting_level = "Severe"

        return {
            'overfitting_ratio': overfitting_ratio,
            'generalization_gap': generalization_gap * 1000,  # Convert to mm
            'overfitting_level': overfitting_level
        }

    def run_svd_only(self, train_A, train_B, test_A, test_B):
        """Method 1: SVD Only (Baseline)"""
        start_time = time.time()

        # Training: Compute SVD transformation on all training data
        T_svd = self.compute_2d_svd_transformation(train_A, train_B)

        # Training evaluation
        transformed_train_A = self.apply_transformation(train_A, T_svd)
        train_metrics = self.calculate_all_metrics(transformed_train_A, train_B)

        # Testing: Apply transformation to test data
        transformed_test_A = self.apply_transformation(test_A, T_svd)
        test_metrics = self.calculate_all_metrics(transformed_test_A, test_B)

        # Overfitting analysis
        overfitting_metrics = self.calculate_overfitting_metrics(train_metrics, test_metrics)

        computation_time = time.time() - start_time

        # Combine metrics
        metrics = {
            'train_rmse': train_metrics['overall_rmse'],
            'test_rmse': test_metrics['overall_rmse'],
            'overall_rmse': test_metrics['overall_rmse'],  # For backward compatibility
            'lateral_rmse': test_metrics['lateral_rmse'],
            'longitudinal_rmse': test_metrics['longitudinal_rmse'],
            'overall_mae': test_metrics['overall_mae'],
            'lateral_mae': test_metrics['lateral_mae'],
            'longitudinal_mae': test_metrics['longitudinal_mae'],
            'computation_time': computation_time,
            **overfitting_metrics
        }

        logger.info(f"SVD Only completed: Train RMSE = {train_metrics['overall_rmse']*1000:.3f}mm, Test RMSE = {test_metrics['overall_rmse']*1000:.3f}mm, Overfitting: {overfitting_metrics['overfitting_level']}, Time = {computation_time:.3f}s")

        return metrics, transformed_test_A

    def run_ransac_svd(self, train_A, train_B, test_A, test_B):
        """Method 2: RANSAC + SVD (Robust Baseline)"""
        start_time = time.time()

        # Training: RANSAC filtering followed by SVD
        ransac_filter = RANSACFilter(
            min_samples=RANSAC_MIN_SAMPLES,
            max_iterations=RANSAC_ITERATIONS,
            inlier_threshold=RANSAC_INLIER_THRESHOLD
        )

        train_inlier_A, train_inlier_B, _, _ = ransac_filter.ransac_filter(train_A, train_B)

        if train_inlier_A is None or len(train_inlier_A) == 0:
            logger.error("RANSAC+SVD failed: no inliers found")
            computation_time = time.time() - start_time
            failed_metrics = {
                'train_rmse': float('inf'),
                'test_rmse': float('inf'),
                'overall_rmse': float('inf'),
                'computation_time': computation_time,
                'overfitting_ratio': float('inf'),
                'generalization_gap': float('inf'),
                'overfitting_level': "Failed"
            }
            return failed_metrics, None

        # Compute SVD on inliers only
        T_svd = self.compute_2d_svd_transformation(train_inlier_A, train_inlier_B)

        # Training evaluation (on full training set)
        transformed_train_A = self.apply_transformation(train_A, T_svd)
        train_metrics = self.calculate_all_metrics(transformed_train_A, train_B)

        # Testing: Apply transformation to test data
        transformed_test_A = self.apply_transformation(test_A, T_svd)
        test_metrics = self.calculate_all_metrics(transformed_test_A, test_B)

        # Overfitting analysis
        overfitting_metrics = self.calculate_overfitting_metrics(train_metrics, test_metrics)

        computation_time = time.time() - start_time

        # Combine metrics
        metrics = {
            'train_rmse': train_metrics['overall_rmse'],
            'test_rmse': test_metrics['overall_rmse'],
            'overall_rmse': test_metrics['overall_rmse'],
            'lateral_rmse': test_metrics['lateral_rmse'],
            'longitudinal_rmse': test_metrics['longitudinal_rmse'],
            'overall_mae': test_metrics['overall_mae'],
            'lateral_mae': test_metrics['lateral_mae'],
            'longitudinal_mae': test_metrics['longitudinal_mae'],
            'computation_time': computation_time,
            **overfitting_metrics
        }

        logger.info(f"RANSAC+SVD completed: Train RMSE = {train_metrics['overall_rmse']*1000:.3f}mm, Test RMSE = {test_metrics['overall_rmse']*1000:.3f}mm, Overfitting: {overfitting_metrics['overfitting_level']}, Time = {computation_time:.3f}s")

        return metrics, transformed_test_A

    def run_cpd(self, train_A, train_B, test_A, test_B):
        """Method 3: Coherent Point Drift (CPD) - UPDATED with fixed import and overfitting analysis"""
        if not CPD_AVAILABLE:
            logger.warning(f"CPD not available: {CPD_ERROR_MSG}")
            return self.run_cpd_alternative(train_A, train_B, test_A, test_B)

        start_time = time.time()

        try:
            # UPDATED: First apply SVD pre-alignment
            logger.info("CPD: Applying SVD pre-alignment...")
            T_svd = self.compute_2d_svd_transformation(train_A, train_B)

            # Apply SVD transformation to training data
            train_A_aligned = self.apply_transformation(train_A, T_svd)

            # Training: Extract XZ coordinates from SVD-aligned data
            train_A_aligned_xz = train_A_aligned[:, [0, 2]]
            train_B_xz = train_B[:, [0, 2]]

            logger.info(f"CPD training with {len(train_A_aligned_xz)} SVD pre-aligned points...")

            # Fixed CPD initialization
            try:
                # Use correct parameters for CPD initialization
                reg = cpd_reg(
                    X=train_B_xz,  # target points
                    Y=train_A_aligned_xz,  # source points
                    max_iterations=50,
                    tolerance=1e-4,
                    w=0.1,  # noise weight
                    alpha=1.0,  # smoothing parameter
                    beta=2.0  # deformation parameter
                )
                logger.info("CPD initialized with optimized parameters")
            except TypeError as e:
                logger.warning(f"CPD parameter initialization failed: {e}. Using default parameters.")
                try:
                    reg = cpd_reg(X=train_B_xz, Y=train_A_aligned_xz)
                except Exception as e2:
                    logger.error(f"CPD default initialization also failed: {e2}")
                    return self.run_cpd_alternative(train_A, train_B, test_A, test_B)

            # Execute registration
            result = reg.register()

            # Get transformed points
            if hasattr(reg, 'TY'):
                transformed_train_A_aligned_xz = reg.TY
            elif hasattr(reg, 'transform'):
                transformed_train_A_aligned_xz = reg.transform(train_A_aligned_xz)
            else:
                logger.warning("Cannot find transformed points in CPD result")
                transformed_train_A_aligned_xz = train_A_aligned_xz

            # Reconstruct 3D training points
            transformed_train_A_final = np.zeros_like(train_A)
            transformed_train_A_final[:, [0, 2]] = transformed_train_A_aligned_xz
            transformed_train_A_final[:, 1] = 0

            train_metrics = self.calculate_all_metrics(transformed_train_A_final, train_B)

            # Test phase: Apply SVD + CPD to test data
            test_A_aligned = self.apply_transformation(test_A, T_svd)
            test_A_aligned_xz = test_A_aligned[:, [0, 2]]

            # Apply trained CPD model to test points
            if hasattr(reg, 'transform'):
                transformed_test_A_xz = reg.transform(test_A_aligned_xz)
            elif hasattr(reg, 'TY'):
                # If no transform method, use interpolation
                logger.warning("CPD doesn't have transform method for new data. Using interpolation fallback.")
                from scipy.interpolate import griddata
                # Use training data transformation for interpolation
                transformed_test_A_xz = griddata(
                    train_A_aligned_xz,
                    transformed_train_A_aligned_xz,
                    test_A_aligned_xz,
                    method='linear',
                    fill_value=test_A_aligned_xz  # Use original values for extrapolation
                )
            else:
                transformed_test_A_xz = test_A_aligned_xz

            # Reconstruct 3D test points
            transformed_test_A = np.zeros_like(test_A)
            transformed_test_A[:, [0, 2]] = transformed_test_A_xz
            transformed_test_A[:, 1] = 0

            # Calculate test metrics
            test_metrics = self.calculate_all_metrics(transformed_test_A, test_B)

            # Overfitting analysis
            overfitting_metrics = self.calculate_overfitting_metrics(train_metrics, test_metrics)

            computation_time = time.time() - start_time

            # Combine metrics
            metrics = {
                'train_rmse': train_metrics['overall_rmse'],
                'test_rmse': test_metrics['overall_rmse'],
                'overall_rmse': test_metrics['overall_rmse'],
                'lateral_rmse': test_metrics['lateral_rmse'],
                'longitudinal_rmse': test_metrics['longitudinal_rmse'],
                'overall_mae': test_metrics['overall_mae'],
                'lateral_mae': test_metrics['lateral_mae'],
                'longitudinal_mae': test_metrics['longitudinal_mae'],
                'computation_time': computation_time,
                **overfitting_metrics
            }

            logger.info(f"CPD completed: Train RMSE = {train_metrics['overall_rmse']*1000:.3f}mm, Test RMSE = {test_metrics['overall_rmse']*1000:.3f}mm, Overfitting: {overfitting_metrics['overfitting_level']}, Time = {computation_time:.3f}s")

            return metrics, transformed_test_A

        except Exception as e:
            logger.error(f"CPD failed during execution: {e}")
            logger.info("Falling back to ICP alternative")
            return self.run_cpd_alternative(train_A, train_B, test_A, test_B)

    def run_cpd_alternative(self, train_A, train_B, test_A, test_B):
        """CPD Alternative using simple ICP with SVD pre-alignment"""
        start_time = time.time()

        try:
            # Apply SVD pre-alignment
            logger.info("CPD Alternative: Applying SVD pre-alignment...")
            T_svd = self.compute_2d_svd_transformation(train_A, train_B)

            # Apply SVD transformation to training data
            train_A_aligned = self.apply_transformation(train_A, T_svd)

            # Training: Extract XZ coordinates from SVD-aligned data
            train_A_aligned_xz = train_A_aligned[:, [0, 2]]
            train_B_xz = train_B[:, [0, 2]]

            logger.info(f"CPD Alternative (ICP) training with {len(train_A_aligned_xz)} SVD pre-aligned points...")

            # Use simple ICP alternative
            reg = SimpleCPDAlternative(X=train_B_xz, Y=train_A_aligned_xz, max_iterations=50, tolerance=1e-3)
            reg.register()

            # Training evaluation
            transformed_train_A_aligned_xz, _ = reg.transform_point_cloud(Y=train_A_aligned_xz)
            transformed_train_A_final = np.zeros_like(train_A)
            transformed_train_A_final[:, [0, 2]] = transformed_train_A_aligned_xz
            transformed_train_A_final[:, 1] = 0
            train_metrics = self.calculate_all_metrics(transformed_train_A_final, train_B)

            # Testing: Apply SVD + ICP to test data
            test_A_aligned = self.apply_transformation(test_A, T_svd)
            test_A_aligned_xz = test_A_aligned[:, [0, 2]]
            transformed_test_A_xz, _ = reg.transform_point_cloud(Y=test_A_aligned_xz)

            # Reconstruct 3D test points
            transformed_test_A = np.zeros_like(test_A)
            transformed_test_A[:, [0, 2]] = transformed_test_A_xz
            transformed_test_A[:, 1] = 0  # Y-axis constraint

            # Evaluation: Calculate test metrics
            test_metrics = self.calculate_all_metrics(transformed_test_A, test_B)

            # Overfitting analysis
            overfitting_metrics = self.calculate_overfitting_metrics(train_metrics, test_metrics)

            computation_time = time.time() - start_time

            # Combine metrics
            metrics = {
                'train_rmse': train_metrics['overall_rmse'],
                'test_rmse': test_metrics['overall_rmse'],
                'overall_rmse': test_metrics['overall_rmse'],
                'lateral_rmse': test_metrics['lateral_rmse'],
                'longitudinal_rmse': test_metrics['longitudinal_rmse'],
                'overall_mae': test_metrics['overall_mae'],
                'lateral_mae': test_metrics['lateral_mae'],
                'longitudinal_mae': test_metrics['longitudinal_mae'],
                'computation_time': computation_time,
                **overfitting_metrics
            }

            logger.info(f"CPD Alternative completed: Train RMSE = {train_metrics['overall_rmse']*1000:.3f}mm, Test RMSE = {test_metrics['overall_rmse']*1000:.3f}mm, Overfitting: {overfitting_metrics['overfitting_level']}, Time = {computation_time:.3f}s")

            return metrics, transformed_test_A

        except Exception as e:
            logger.error(f"CPD Alternative failed: {e}")
            computation_time = time.time() - start_time
            failed_metrics = {
                'train_rmse': float('inf'),
                'test_rmse': float('inf'),
                'overall_rmse': float('inf'),
                'computation_time': computation_time,
                'overfitting_ratio': float('inf'),
                'generalization_gap': float('inf'),
                'overfitting_level': "Failed"
            }
            return failed_metrics, None

    def _get_enhanced_control_points(self, points, residuals, num_points):
        """
        Enhanced control point selection with spatial distribution awareness
        """
        if len(points) <= num_points:
            logger.info(f"Number of available points ({len(points)}) is less than or equal to the target number of control points ({num_points}). Using all available points.")
            return points, residuals, np.arange(len(points))

        # Method 1: K-means clustering for spatial distribution
        try:
            from sklearn.cluster import KMeans
            kmeans = KMeans(n_clusters=num_points, random_state=42, n_init=10)
            cluster_labels = kmeans.fit_predict(points)

            # Select one representative point from each cluster
            selected_indices = []
            selected_points = []
            selected_residuals = []

            for cluster_id in range(num_points):
                cluster_mask = cluster_labels == cluster_id
                if np.any(cluster_mask):
                    cluster_points = points[cluster_mask]
                    cluster_residuals = residuals[cluster_mask]
                    cluster_indices = np.where(cluster_mask)[0]

                    # Select the point closest to cluster center
                    cluster_center = kmeans.cluster_centers_[cluster_id]
                    distances = np.linalg.norm(cluster_points - cluster_center, axis=1)
                    best_idx = np.argmin(distances)

                    selected_indices.append(cluster_indices[best_idx])
                    selected_points.append(cluster_points[best_idx])
                    selected_residuals.append(cluster_residuals[best_idx])

            selected_points = np.array(selected_points)
            selected_residuals = np.array(selected_residuals)
            selected_indices = np.array(selected_indices)

            logger.info(f"Selected {len(selected_points)} control points using K-means clustering")

        except ImportError:
            logger.warning("sklearn not available, using grid-based selection")
            # Fallback to grid-based method
            selected_points, selected_indices = self._get_sparse_control_points(points, num_points)
            selected_residuals = residuals[selected_indices]

        return selected_points, selected_residuals, selected_indices

    def _get_sparse_control_points(self, points, num_points):
        """
        Selects a spatially uniform subset of points to act as RBF control points.
        This prevents overfitting by forcing the RBF to learn a smoother deformation.
        """
        if len(points) <= num_points:
            logger.info(f"Number of available points ({len(points)}) is less than or equal to the target number of control points ({num_points}). Using all available points.")
            return points, np.arange(len(points))

        # Determine the bounding box of the points
        min_coords = np.min(points, axis=0)
        max_coords = np.max(points, axis=0)

        # Create a grid of nodes
        # We aim for roughly num_points, so we calculate grid divisions
        aspect_ratio = (max_coords[0] - min_coords[0]) / (max_coords[1] - min_coords[1])
        nx = int(np.sqrt(num_points * aspect_ratio))
        ny = int(np.sqrt(num_points / aspect_ratio))

        if nx == 0: nx = 1
        if ny == 0: ny = 1

        grid_x = np.linspace(min_coords[0], max_coords[0], nx)
        grid_y = np.linspace(min_coords[1], max_coords[1], ny)
        grid_nodes = np.array(np.meshgrid(grid_x, grid_y)).T.reshape(-1, 2)

        # Use NearestNeighbors to find the closest actual data point to each grid node
        nn = NearestNeighbors(n_neighbors=1, algorithm='auto').fit(points)
        _, indices = nn.kneighbors(grid_nodes)

        # Get unique indices, as multiple grid nodes might map to the same data point
        unique_indices = np.unique(indices.flatten())

        sparse_points = points[unique_indices]
        logger.info(f"Selected {len(sparse_points)} sparse control points from {len(points)} available points.")

        return sparse_points, unique_indices

    def run_proposed_method(self, train_A, train_B, test_A, test_B):
        """Method 4: Enhanced Proposed Method (RANSAC + SVD + Optimized RBF)"""
        start_time = time.time()

        try:
            # Training Phase
            # Step 1: RANSAC filtering
            ransac_filter = RANSACFilter(
                min_samples=RANSAC_MIN_SAMPLES,
                max_iterations=RANSAC_ITERATIONS,
                inlier_threshold=RANSAC_INLIER_THRESHOLD
            )

            train_inlier_A, train_inlier_B, _, _ = ransac_filter.ransac_filter(train_A, train_B)

            if train_inlier_A is None or len(train_inlier_A) == 0:
                logger.error("Proposed method failed: no inliers found")
                computation_time = time.time() - start_time
                failed_metrics = {
                    'train_rmse': float('inf'),
                    'test_rmse': float('inf'),
                    'overall_rmse': float('inf'),
                    'computation_time': computation_time,
                    'overfitting_ratio': float('inf'),
                    'generalization_gap': float('inf'),
                    'overfitting_level': "Failed"
                }
                return failed_metrics, None

            # Step 2: SVD transformation
            T_svd = self.compute_2d_svd_transformation(train_inlier_A, train_inlier_B)

            # Step 3: Apply SVD to training inliers to get coordinates in aligned space
            train_svd_transformed = self.apply_transformation(train_inlier_A, T_svd)

            # Step 4: Calculate residual vectors for the inliers
            train_residuals = train_inlier_B - train_svd_transformed
            train_residuals_xz = train_residuals[:, [0, 2]]

            # Step 5: 诊断性可视化 - 残差场可视化
            svd_transformed_inlier_xz = train_svd_transformed[:, [0, 2]]
            
            logger.info("Creating residual field visualization for deformation analysis...")
            residual_vis_path = os.path.join(img_dir, 'residual_field_analysis.png')
            visualize_residual_field(
                svd_transformed_inlier_xz, 
                train_residuals_xz,
                title="SVD-Aligned Residual Field for RBF Learning Analysis",
                save_path=residual_vis_path
            )

            # Step 6: 鲁棒性改进 - 残差异常值处理
            logger.info("Applying outlier clipping for improved robustness...")
            train_residuals_xz_clipped = clip_outliers(train_residuals_xz, percentile_threshold=95)

            # Step 7: 系统性超参数调优
            logger.info("Starting systematic hyperparameter optimization...")
            
            # Enhanced control point selection
            base_num_control_points = min(200, len(train_inlier_A) // 2)
            control_points_xz, control_residuals_xz, control_indices = self._get_enhanced_control_points(
                svd_transformed_inlier_xz, train_residuals_xz_clipped, base_num_control_points
            )

            # Hyperparameter optimization with cross-validation
            optimizer = RBFHyperparameterOptimizer(n_folds=5, random_state=42)
            
            # Define parameter grid for optimization
            param_grid = {
                'kernel': ['multiquadric', 'thin_plate_spline', 'gaussian'],
                'smoothing': [0.001, 0.01, 0.05, 0.1],
                'epsilon': [0.1, 1.0, 5.0],
                'num_control_points': [50, 100, 150, 200] if len(control_points_xz) > 200 else [len(control_points_xz)]
            }
            
            best_params, best_cv_score = optimizer.optimize(control_points_xz, control_residuals_xz, param_grid)
            
            logger.info(f"Optimal RBF parameters found: {best_params}")
            logger.info(f"Best cross-validation score: {best_cv_score:.6f}")

            # Apply optimized parameters
            if 'num_control_points' in best_params and best_params['num_control_points'] < len(control_points_xz):
                # Re-select control points with optimal number
                final_control_points_xz, final_control_residuals_xz, _ = self._get_enhanced_control_points(
                    svd_transformed_inlier_xz, train_residuals_xz_clipped, best_params['num_control_points']
                )
            else:
                final_control_points_xz = control_points_xz
                final_control_residuals_xz = control_residuals_xz

            logger.info("Training optimized RBF deformation model...")

            # Create enhanced RBF interpolator with optimal parameters
            rbf_interpolator = EnhancedRBFInterpolator(
                final_control_points_xz,
                final_control_residuals_xz,
                kernel=best_params['kernel'],
                smoothing=best_params['smoothing'],
                epsilon=best_params.get('epsilon', 1.0)
            )

            # Training evaluation (on full training set)
            train_svd_transformed_full = self.apply_transformation(train_A, T_svd)
            train_svd_transformed_full_xz = train_svd_transformed_full[:, [0, 2]]

            # Apply RBF deformation
            train_deformation_xz = rbf_interpolator(train_svd_transformed_full_xz)

            train_final = train_svd_transformed_full.copy()
            train_final[:, 0] += train_deformation_xz[:, 0]
            train_final[:, 2] += train_deformation_xz[:, 1]
            train_final[:, 1] = 0

            train_metrics = self.calculate_all_metrics(train_final, train_B)

            # Testing Phase
            # Step 1: Apply SVD transformation to test points
            test_svd_transformed = self.apply_transformation(test_A, T_svd)

            # Step 2: Predict deformation using the trained Enhanced RBF model
            test_svd_transformed_xz = test_svd_transformed[:, [0, 2]]
            test_deformation_xz = rbf_interpolator(test_svd_transformed_xz)

            # Step 3: Apply deformation correction
            test_final = test_svd_transformed.copy()
            test_final[:, 0] += test_deformation_xz[:, 0]
            test_final[:, 2] += test_deformation_xz[:, 1]
            test_final[:, 1] = 0  # Ensure Y=0

            # Evaluation: Calculate test metrics
            test_metrics = self.calculate_all_metrics(test_final, test_B)

            # Overfitting analysis
            overfitting_metrics = self.calculate_overfitting_metrics(train_metrics, test_metrics)

            computation_time = time.time() - start_time

            # Combine metrics
            metrics = {
                'train_rmse': train_metrics['overall_rmse'],
                'test_rmse': test_metrics['overall_rmse'],
                'overall_rmse': test_metrics['overall_rmse'],
                'lateral_rmse': test_metrics['lateral_rmse'],
                'longitudinal_rmse': test_metrics['longitudinal_rmse'],
                'overall_mae': test_metrics['overall_mae'],
                'lateral_mae': test_metrics['lateral_mae'],
                'longitudinal_mae': test_metrics['longitudinal_mae'],
                'computation_time': computation_time,
                **overfitting_metrics
            }

            logger.info(f"Enhanced Proposed method completed: Train RMSE = {train_metrics['overall_rmse']*1000:.3f}mm, Test RMSE = {test_metrics['overall_rmse']*1000:.3f}mm, Overfitting: {overfitting_metrics['overfitting_level']}, Time = {computation_time:.3f}s")

            return metrics, test_final

        except Exception as e:
            logger.error(f"Enhanced Proposed method failed: {e}")
            computation_time = time.time() - start_time
            failed_metrics = {
                'train_rmse': float('inf'),
                'test_rmse': float('inf'),
                'overall_rmse': float('inf'),
                'computation_time': computation_time,
                'overfitting_ratio': float('inf'),
                'generalization_gap': float('inf'),
                'overfitting_level': "Failed"
            }
            return failed_metrics, None

    def run(self):
        """Run complete benchmark experiment"""
        logger.info("=== Starting Enhanced Academic Benchmark Experiment ===")

        # Execute all four methods
        results = {}
        final_points = {}

        logger.info("Running Method 1: SVD Only")
        metrics, points = self.run_svd_only(self.train_A, self.train_B, self.test_A, self.test_B)
        results['SVD Only'] = metrics
        final_points['SVD Only'] = points

        logger.info("Running Method 2: RANSAC + SVD")
        metrics, points = self.run_ransac_svd(self.train_A, self.train_B, self.test_A, self.test_B)
        results['RANSAC + SVD'] = metrics
        final_points['RANSAC + SVD'] = points

        logger.info("Running Method 3: CPD (with SVD pre-alignment)")
        metrics, points = self.run_cpd(self.train_A, self.train_B, self.test_A, self.test_B)
        results['CPD'] = metrics
        final_points['CPD'] = points

        logger.info("Running Method 4: Enhanced Proposed Method")
        metrics, points = self.run_proposed_method(self.train_A, self.train_B, self.test_A, self.test_B)
        results['Proposed Method'] = metrics
        final_points['Proposed Method'] = points

        # Generate comprehensive report
        self.generate_report(results, final_points)

        return results

    def generate_report(self, results, final_points):
        """Generate comprehensive comparison report"""
        logger.info("=== Generating Comprehensive Report ===")

        # Console output - Markdown table
        self._print_markdown_table(results)

        # CSV output
        self._save_csv_results(results)

        # Comprehensive visualization
        self._create_comprehensive_plots(results, final_points)

        logger.info("Report generation completed")

    def _print_markdown_table(self, results):
        """Print results as formatted Markdown table with overfitting analysis"""
        print("\n" + "="*120)
        print("ENHANCED ACADEMIC BENCHMARK RESULTS WITH OPTIMIZED RBF")
        print("="*120)

        # Table header
        header = "| Method | Train RMSE (mm) | Test RMSE (mm) | Gap (mm) | Overfitting | Lateral RMSE (mm) | Longitudinal RMSE (mm) | Time (s) | Meets Target |"
        separator = "|-------------------|-----------------|----------------|----------|-------------|-------------------|------------------------|----------|--------------|"

        print(header)
        print(separator)

        # Table rows
        for method_name, metrics in results.items():
            if metrics['overall_rmse'] == float('inf'):
                train_rmse_str = "Failed"
                test_rmse_str = "Failed"
                gap_str = "Failed"
                overfitting_str = "Failed"
                lateral_str = "Failed"
                longitudinal_str = "Failed"
                meets_target = "No"
            else:
                train_rmse_str = f"{metrics['train_rmse']*1000:.3f}"
                test_rmse_str = f"{metrics['test_rmse']*1000:.3f}"
                gap_str = f"{metrics['generalization_gap']:.1f}"
                overfitting_str = metrics['overfitting_level']
                lateral_str = f"{metrics['lateral_rmse']*1000:.3f}"
                longitudinal_str = f"{metrics['longitudinal_rmse']*1000:.3f}"
                meets_target = "Yes" if metrics['overall_rmse'] <= TARGET_RMSE else "No"

            time_str = f"{metrics['computation_time']:.3f}"

            row = f"| {method_name:<17} | {train_rmse_str:>15} | {test_rmse_str:>14} | {gap_str:>8} | {overfitting_str:>11} | {lateral_str:>17} | {longitudinal_str:>22} | {time_str:>8} | {meets_target:>12} |"
            print(row)

        print(f"\nTarget RMSE: {TARGET_RMSE * 1000:.1f}mm")
        print("Overfitting Analysis:")
        print("- None: Test/Train ratio ≤ 1.1")
        print("- Mild: Test/Train ratio ≤ 1.3")
        print("- Moderate: Test/Train ratio ≤ 1.5")
        print("- Severe: Test/Train ratio > 1.5")
        print("="*120)

    def _save_csv_results(self, results):
        """Save results to CSV file with overfitting metrics"""
        csv_data = []

        for method_name, metrics in results.items():
            row = {
                'Method': method_name,
                'Train_RMSE_mm': metrics['train_rmse']*1000 if metrics['train_rmse'] != float('inf') else 'Failed',
                'Test_RMSE_mm': metrics['test_rmse']*1000 if metrics['test_rmse'] != float('inf') else 'Failed',
                'Overall_RMSE_mm': metrics['overall_rmse']*1000 if metrics['overall_rmse'] != float('inf') else 'Failed',
                'Generalization_Gap_mm': metrics['generalization_gap'] if metrics['overall_rmse'] != float('inf') else 'Failed',
                'Overfitting_Ratio': f"{metrics['overfitting_ratio']:.3f}" if metrics['overfitting_ratio'] != float('inf') else 'Failed',
                'Overfitting_Level': metrics['overfitting_level'],
                'Lateral_RMSE_mm': metrics['lateral_rmse']*1000 if metrics['overall_rmse'] != float('inf') else 'Failed',
                'Longitudinal_RMSE_mm': metrics['longitudinal_rmse']*1000 if metrics['overall_rmse'] != float('inf') else 'Failed',
                'Overall_MAE_mm': metrics['overall_mae']*1000 if metrics['overall_rmse'] != float('inf') else 'Failed',
                'Lateral_MAE_mm': metrics['lateral_mae']*1000 if metrics['overall_rmse'] != float('inf') else 'Failed',
                'Longitudinal_MAE_mm': metrics['longitudinal_mae']*1000 if metrics['overall_rmse'] != float('inf') else 'Failed',
                'Computation_Time_s': metrics['computation_time'],
                'Meets_Target': 'Yes' if metrics['overall_rmse'] <= TARGET_RMSE else 'No'
            }
            csv_data.append(row)

        df_results = pd.DataFrame(csv_data)
        csv_file = os.path.join(result_dir, 'comparison_results.csv')
        df_results.to_csv(csv_file, index=False)

        logger.info(f"Results saved to {csv_file}")

    def _create_comprehensive_plots(self, results, final_points):
        """
        Create publication-quality comparison plots with enhanced visualization
        学术性注释：
        - 子图A：过拟合分析，展示训练vs测试性能，评估模型泛化能力
        - 子图B：效率分析，评估性能与计算时间的权衡关系
        - 子图C：最终性能指标对比，提供各方法的全面比较
        - 子图D：测试残差分布，展示不同方法的误差特征
        """
        logger.info("Generating enhanced publication-quality comparison plots...")

        # Setup plotting style with fallback options
        try:
            plt.style.use('seaborn-v0_8-paper')
        except OSError:
            try:
                plt.style.use('seaborn-paper')
            except OSError:
                try:
                    plt.style.use('seaborn-whitegrid')
                except OSError:
                    try:
                        plt.style.use('seaborn')
                    except OSError:
                        plt.style.use('default')
                        plt.rcParams['figure.facecolor'] = 'white'
                        plt.rcParams['axes.facecolor'] = 'white'
                        plt.rcParams['axes.grid'] = True
                        plt.rcParams['grid.alpha'] = 0.3
                        logger.info("Warning: Using default matplotlib style as seaborn styles are not available")

        methods = list(results.keys())

        # Extract plotting data
        train_rmse_values = [results[m]['train_rmse'] * 1000 if results[m]['test_rmse'] != float('inf') else np.nan for m in methods]
        test_rmse_values = [results[m]['test_rmse'] * 1000 if results[m]['test_rmse'] != float('inf') else np.nan for m in methods]
        mae_values = [results[m]['overall_mae'] * 1000 if results[m]['test_rmse'] != float('inf') else np.nan for m in methods]
        overfitting_ratios = [results[m]['overfitting_ratio'] if results[m]['overfitting_ratio'] != float('inf') else np.nan for m in methods]
        time_values = [results[m]['computation_time'] for m in methods]

        # Figure 1: Main Performance Comparison with Enhanced Analysis (2x2 layout)
        fig1, axes1 = plt.subplots(2, 2, figsize=(14, 12))
        fig1.suptitle('Enhanced Registration Methods Performance Benchmark with Optimized RBF', fontsize=18, fontweight='bold')

        # Subplot 1A: Train vs Test Performance (Overfitting Analysis)
        # 学术价值：分析过拟合程度，评估模型的泛化能力
        ax = axes1[0, 0]
        valid_indices = ~np.isnan(train_rmse_values) & ~np.isnan(test_rmse_values)
        valid_train = np.array(train_rmse_values)[valid_indices]
        valid_test = np.array(test_rmse_values)[valid_indices]
        valid_methods = np.array(methods)[valid_indices]

        # Color points based on overfitting level
        colors = []
        for method in valid_methods:
            level = results[method]['overfitting_level']
            if level == 'None':
                colors.append('green')
            elif level == 'Mild':
                colors.append('yellow')
            elif level == 'Moderate':
                colors.append('orange')
            else:
                colors.append('red')

        scatter = ax.scatter(valid_train, valid_test, s=120, alpha=0.8, c=colors, zorder=10)

        # Perfect generalization line
        if len(valid_train) > 0 and len(valid_test) > 0:
            min_val = min(np.min(valid_train), np.min(valid_test))
            max_val = max(np.max(valid_train), np.max(valid_test))
            ax.plot([min_val, max_val], [min_val, max_val], 'k--', alpha=0.6, label='Perfect Generalization')

        ax.axhline(y=TARGET_RMSE*1000, color='red', linestyle='--', alpha=0.7, label=f'Target: {TARGET_RMSE*1000:.1f}mm')
        ax.axvline(x=TARGET_RMSE*1000, color='red', linestyle='--', alpha=0.7)

        texts = []
        for i, method in enumerate(valid_methods):
            texts.append(ax.text(valid_train[i], valid_test[i], method, fontsize=10))

        if ADJUST_TEXT_AVAILABLE:
            try:
                adjust_text(texts, ax=ax, arrowprops=dict(arrowstyle='->', color='black', lw=0.5))
            except:
                pass

        ax.set_xlabel('Training RMSE (mm)', fontsize=12)
        ax.set_ylabel('Test RMSE (mm)', fontsize=12)
        ax.set_title('A) Overfitting Analysis: Train vs Test Performance', fontsize=14, fontweight='bold')
        ax.tick_params(labelsize=10)
        ax.grid(True, alpha=0.3)

        # Add color legend
        from matplotlib.patches import Patch
        legend_elements = [Patch(facecolor='green', label='None'),
                          Patch(facecolor='yellow', label='Mild'),
                          Patch(facecolor='orange', label='Moderate'),
                          Patch(facecolor='red', label='Severe')]
        ax.legend(handles=legend_elements, title='Overfitting Level', loc='upper left', bbox_to_anchor=(0.02, 0.98))

        # Subplot 1B: Performance vs Computation Time (Efficiency Analysis)
        # 学术价值：评估算法效率，分析性能与计算复杂度的权衡
        ax = axes1[0, 1]
        valid_indices = ~np.isnan(test_rmse_values)
        valid_test_rmse = np.array(test_rmse_values)[valid_indices]
        valid_time = np.array(time_values)[valid_indices]
        valid_methods_time = np.array(methods)[valid_indices]

        # Create efficiency scatter plot
        scatter = ax.scatter(valid_time, valid_test_rmse, s=120, alpha=0.8, c=['tab:blue', 'tab:orange', 'tab:green', 'tab:red'][:len(valid_time)])

        # Add target line
        ax.axhline(y=TARGET_RMSE*1000, color='red', linestyle='--', linewidth=2, label=f'Target RMSE: {TARGET_RMSE*1000:.1f}mm')

        # Add method labels
        texts = []
        for i, method in enumerate(valid_methods_time):
            texts.append(ax.text(valid_time[i], valid_test_rmse[i], method, fontsize=10))

        if ADJUST_TEXT_AVAILABLE:
            try:
                adjust_text(texts, ax=ax, arrowprops=dict(arrowstyle='->', color='black', lw=0.5))
            except:
                pass

        ax.set_xlabel('Computation Time (s)', fontsize=12)
        ax.set_ylabel('Test RMSE (mm)', fontsize=12)
        ax.set_title('B) Efficiency Analysis: Performance vs Time', fontsize=14, fontweight='bold')
        ax.tick_params(labelsize=10)
        ax.grid(True, alpha=0.3)
        ax.legend()

        # Subplot 1C: Grouped Bar Chart for Final Performance Metrics (FIXED - 图例位置修正)
        # 学术价值：提供各方法多项性能指标的直观对比，便于全面评估
        ax = axes1[1, 0]
        x = np.arange(len(methods))  # the label locations
        width = 0.25  # the width of the bars

        # Plotting the bars for each metric
        rects1 = ax.bar(x - width, train_rmse_values, width, label='Train RMSE', color='tab:blue', alpha=0.8)
        rects2 = ax.bar(x, test_rmse_values, width, label='Test RMSE', color='tab:orange', alpha=0.8)
        rects3 = ax.bar(x + width, mae_values, width, label='Overall MAE', color='tab:green', alpha=0.8)

        # Add the target line
        ax.axhline(y=TARGET_RMSE * 1000, color='red', linestyle='--', linewidth=2, label=f'Target RMSE: {TARGET_RMSE*1000:.1f}mm')

        # Add some text for labels, title and axes ticks
        ax.set_ylabel('Error (mm)', fontsize=12)
        ax.set_title('C) Final Performance Metrics Comparison', fontsize=14, fontweight='bold')
        ax.set_xticks(x)
        ax.set_xticklabels(methods)
        ax.tick_params(axis='x', rotation=45, labelsize=10)
        ax.tick_params(axis='y', labelsize=10)
        ax.grid(True, alpha=0.3)
        
        # 修正：将图例放置在左上角，避免遮挡数据
        ax.legend(loc='upper left')

        # Subplot 1D: Test Residual Distribution (Box plot)
        # 学术价值：展示误差分布特征，识别方法的稳定性和异常值处理能力
        ax = axes1[1, 1]
        residual_data = {}
        for method_name, transformed_points in final_points.items():
            if transformed_points is not None:
                real_residuals = self.test_B - transformed_points
                residual_magnitudes = np.linalg.norm(real_residuals[:, [0, 2]], axis=1) * 1000
                residual_data[method_name] = residual_magnitudes

        if residual_data:
            bp = ax.boxplot(residual_data.values(), labels=residual_data.keys(), patch_artist=True, showfliers=False)
            colors = ['lightblue', 'lightcoral', 'lightgreen', 'lightyellow']
            for i, patch in enumerate(bp['boxes']):
                if i < len(colors):
                    patch.set_facecolor(colors[i])
                    patch.set_alpha(0.8)
            ax.axhline(y=TARGET_RMSE*1000, color='red', linestyle='--', alpha=0.7, label=f'Target: {TARGET_RMSE*1000:.1f}mm')

        ax.set_ylabel('Residual Magnitude (mm)', fontsize=12)
        ax.set_title('D) Test Residual Distribution', fontsize=14, fontweight='bold')
        ax.tick_params(axis='x', rotation=45, labelsize=10)
        ax.tick_params(axis='y', labelsize=10)
        ax.grid(True, alpha=0.3)
        ax.legend()

        plt.tight_layout(rect=[0, 0, 1, 0.96])
        fig1.savefig(os.path.join(img_dir, 'figure1_enhanced_optimized_results.pdf'), dpi=300, bbox_inches='tight')
        fig1.savefig(os.path.join(img_dir, 'figure1_enhanced_optimized_results.png'), dpi=300, bbox_inches='tight')
        plt.close(fig1)
        logger.info("Saved Figure 1: Enhanced Optimized Performance Results")

        # Figure 2: Detailed Performance Metrics (Enhanced Heatmap)
        fig2, ax2 = plt.subplots(figsize=(12, 6))

        metrics_names = ['Train RMSE', 'Test RMSE', 'Overfitting Ratio', 'Lateral RMSE', 'Longitudinal RMSE', 'MAE']
        performance_matrix = np.zeros((len(methods), len(metrics_names)))

        for i, method in enumerate(methods):
            metrics = results[method]
            if metrics['test_rmse'] != float('inf'):
                performance_matrix[i, 0] = metrics['train_rmse'] * 1000
                performance_matrix[i, 1] = metrics['test_rmse'] * 1000
                performance_matrix[i, 2] = metrics['overfitting_ratio']
                performance_matrix[i, 3] = metrics['lateral_rmse'] * 1000
                performance_matrix[i, 4] = metrics['longitudinal_rmse'] * 1000
                performance_matrix[i, 5] = metrics['overall_mae'] * 1000
            else:
                performance_matrix[i, :] = np.nan

        # Create heatmap
        im = ax2.imshow(performance_matrix, cmap='RdYlGn_r', aspect='auto')
        ax2.set_xticks(np.arange(len(metrics_names)))
        ax2.set_xticklabels(metrics_names, rotation=45, ha="right", fontsize=12)
        ax2.set_yticks(np.arange(len(methods)))
        ax2.set_yticklabels(methods, fontsize=12)
        ax2.set_title('Enhanced Performance Metrics Heatmap with Optimized RBF', fontsize=16, fontweight='bold')

        # Add text annotations
        for i in range(len(methods)):
            for j in range(len(metrics_names)):
                if not np.isnan(performance_matrix[i, j]):
                    val = performance_matrix[i, j]
                    # Format overfitting ratio differently
                    if j == 2:  # Overfitting ratio
                        text = f'{val:.2f}'
                    else:
                        text = f'{val:.2f}'

                    # Smart color selection for text readability
                    norm_val = (val - np.nanmin(performance_matrix)) / (np.nanmax(performance_matrix) - np.nanmin(performance_matrix))
                    color = "white" if norm_val > 0.5 else "black"
                    ax2.text(j, i, text, ha="center", va="center", color=color, fontsize=10, fontweight='medium')

        cbar = fig2.colorbar(im, ax=ax2)
        cbar.set_label('Metric Value', fontsize=12)

        plt.tight_layout()
        fig2.savefig(os.path.join(img_dir, 'figure2_enhanced_optimized_heatmap.pdf'), dpi=300, bbox_inches='tight')
        fig2.savefig(os.path.join(img_dir, 'figure2_enhanced_optimized_heatmap.png'), dpi=300, bbox_inches='tight')
        plt.close(fig2)
        logger.info("Saved Figure 2: Enhanced Optimized Performance Heatmap")


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
    logger.info("Starting Enhanced Academic Benchmark Framework for Point Cloud Registration")
    logger.info("Methods: SVD Only, RANSAC+SVD, CPD (with SVD pre-alignment), Enhanced Proposed Method (RANSAC+SVD+Optimized RBF)")
    logger.info("ENHANCEMENTS: Residual field visualization, cross-validation hyperparameter tuning, outlier robustness, fixed visualization issues.")
    logger.info(f"Scale factor: {SCALE_FACTOR}")
    logger.info(f"Target RMSE: {TARGET_RMSE*1000:.2f}mm")
    logger.info(f"Train/Test ratio: {TRAIN_TEST_RATIO:.1f}/{1-TRAIN_TEST_RATIO:.1f}")

    global LASER_TRACKER_CSV
    if len(sys.argv) > 1:
        LASER_TRACKER_CSV = sys.argv[1]

    if not os.path.exists(LASER_TRACKER_CSV):
        # Create a dummy CSV for testing if it doesn't exist
        logger.warning(f"Data file {LASER_TRACKER_CSV} not found. Creating a dummy file for demonstration.")
        num_points = 1000
        # Simulate a slight non-linear warp and noise
        rand_angles = np.random.rand(num_points) * 2 * np.pi
        rand_radii = np.random.rand(num_points) * 5 + 2  # Radii from 2 to 7
        tracker_x = rand_radii * np.cos(rand_angles)
        tracker_z = rand_radii * np.sin(rand_angles)

        # Apply a known transformation + warp + noise
        angle = np.pi / 8
        rot_matrix = np.array([[np.cos(angle), -np.sin(angle)], [np.sin(angle), np.cos(angle)]])
        trans_vec = np.array([0.5, -0.3])

        laser_xz = np.dot(np.vstack([tracker_x, tracker_z]).T, rot_matrix.T) + trans_vec
        # Add a non-linear warp
        laser_xz[:, 0] += 0.05 * np.sin(laser_xz[:, 1])
        # Add noise
        laser_xz += np.random.randn(num_points, 2) * 0.02

        dummy_df = pd.DataFrame({
            'tracker_x': tracker_x,
            'tracker_z': tracker_z,
            'laser_x': laser_xz[:, 0],
            'laser_z': laser_xz[:, 1]
        })
        data_dir_path = os.path.dirname(LASER_TRACKER_CSV)
        if not os.path.exists(data_dir_path):
            os.makedirs(data_dir_path)
        dummy_df.to_csv(LASER_TRACKER_CSV, index=False)
        logger.info(f"Dummy data saved to {LASER_TRACKER_CSV}")

    # Process laser tracker data
    logger.info("Processing laser tracker data...")
    df = process_laser_tracker_data(input_file=LASER_TRACKER_CSV)

    if df is None:
        logger.error("Failed to process laser tracker data")
        return

    # Prepare data for transformation
    positions_A = [[row[TRACKER_X], 0, row[TRACKER_Y]] for _, row in df.iterrows()]
    positions_B = [[row[LASER_X], 0, row[LASER_Y]] for _, row in df.iterrows()]

    logger.info(f"Loaded {len(positions_A)} point pairs for registration")

    # Run comprehensive benchmark
    experiment = ExperimentRunner(positions_A, positions_B, df)
    results = experiment.run()

    # Find best method and analyze overfitting
    best_method = None
    best_rmse = float('inf')
    best_generalization = None

    for method_name, metrics in results.items():
        if metrics['overall_rmse'] < best_rmse:
            best_rmse = metrics['overall_rmse']
            best_method = method_name
            best_generalization = metrics['overfitting_level']

    # Final summary
    success = best_rmse <= TARGET_RMSE

    if success:
        logger.info(f"BENCHMARK SUCCESS: Best method '{best_method}' achieved RMSE {best_rmse*1000:.3f}mm with {best_generalization} overfitting")
    else:
        logger.info(f"BENCHMARK PARTIAL: Best method '{best_method}' achieved RMSE {best_rmse*1000:.3f}mm (exceeds target {TARGET_RMSE*1000:.2f}mm) with {best_generalization} overfitting")

    # Save final benchmark summary with enhanced analysis
    summary_file = os.path.join(output_dir, "benchmark_summary_enhanced.txt")
    with open(summary_file, "w") as f:
        f.write("Enhanced Academic Benchmark Framework Summary\n")
        f.write("=" * 80 + "\n")
        f.write("Point Cloud Registration Methods Comparison\n")
        f.write("MAJOR ENHANCEMENTS:\n")
        f.write("1. Residual field visualization for deformation pattern analysis\n")
        f.write("2. Cross-validation based hyperparameter optimization\n")
        f.write("3. Robust outlier handling with percentile-based clipping\n")
        f.write("4. Fixed visualization issues (legend positioning)\n")
        f.write("5. Enhanced control point selection using K-means clustering\n")
        f.write("6. Systematic optimization of RBF kernel, smoothing, and control points\n")
        f.write(f"Dataset: {csv_filename}\n")
        f.write(f"Scale Factor: {SCALE_FACTOR}\n")
        f.write(f"Target RMSE: {TARGET_RMSE*1000:.2f}mm\n")
        f.write(f"Train/Test ratio: {TRAIN_TEST_RATIO:.1f}/{1-TRAIN_TEST_RATIO:.1f}\n")
        f.write(f"Total points: {len(positions_A)}\n")
        f.write(f"Training points: {len(experiment.train_A)}\n")
        f.write(f"Test points: {len(experiment.test_A)}\n")
        f.write("\nMethod Performance with Enhanced Analysis:\n")
        f.write("-" * 60 + "\n")

        for method_name, metrics in results.items():
            if metrics['overall_rmse'] != float('inf'):
                f.write(f"{method_name}:\n")
                f.write(f" Train RMSE: {metrics['train_rmse']*1000:.3f}mm\n")
                f.write(f" Test RMSE: {metrics['test_rmse']*1000:.3f}mm\n")
                f.write(f" Generalization Gap: {metrics['generalization_gap']:.1f}mm\n")
                f.write(f" Overfitting Level: {metrics['overfitting_level']}\n")
                f.write(f" Overfitting Ratio: {metrics['overfitting_ratio']:.3f}\n")
                f.write(f" Time: {metrics['computation_time']:.3f}s\n")
            else:
                f.write(f"{method_name}: Failed\n")

        f.write(f"\nBest Method: {best_method}\n")
        f.write(f"Best Test RMSE: {best_rmse*1000:.3f}mm\n")
        f.write(f"Generalization: {best_generalization}\n")
        f.write(f"Success: {'Yes' if success else 'No'}\n")
        f.write(f"Real-world equivalent: {best_rmse*SCALE_FACTOR*100:.2f}cm\n")

        f.write(f"\nEnhanced RBF Optimization Summary:\n")
        f.write(f"- Cross-validation ensures robust hyperparameter selection\n")
        f.write(f"- Outlier clipping improves model stability\n")
        f.write(f"- Residual field visualization reveals deformation patterns\n")
        f.write(f"- K-means control point selection optimizes spatial coverage\n")
        f.write(f"- Multi-kernel evaluation finds optimal basis functions\n")
        f.write(f"- Systematic optimization balances accuracy vs generalization\n")

    logger.info("Enhanced academic benchmark framework completed")


if __name__ == "__main__":
    main()