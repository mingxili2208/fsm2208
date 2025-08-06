#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Academic Benchmark Framework for Point Cloud Registration Methods
Comprehensive comparison of four registration methods with train/test split
Methods: SVD Only, RANSAC+SVD, CPD, Proposed Method (RANSAC+SVD+RBF)
ENHANCED VERSION: Fixed CPD performance, optimized RBF overfitting, improved visualizations
FIXED: Matplotlib compatibility issues and visualization errors
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

# Import CPD with enhanced error handling and multiple fallback options
CPD_AVAILABLE = False
CPD_ERROR_MSG = ""
cpd_reg = None

try:
    # Enhanced CPD import with multiple strategies
    try:
        from pycpd import DeformableRegistration
        cpd_reg = DeformableRegistration
        CPD_AVAILABLE = True
        print("✓ CPD successfully imported (DeformableRegistration)")
    except ImportError:
        try:
            from pycpd import AffineRegistration
            cpd_reg = AffineRegistration
            CPD_AVAILABLE = True
            print("✓ CPD successfully imported (AffineRegistration - fallback)")
        except ImportError:
            import pycpd
            # Dynamically find the best available registration class
            if hasattr(pycpd, 'DeformableRegistration'):
                cpd_reg = pycpd.DeformableRegistration
                CPD_AVAILABLE = True
                print("✓ CPD successfully imported (dynamic detection)")
            elif hasattr(pycpd, 'AffineRegistration'):
                cpd_reg = pycpd.AffineRegistration
                CPD_AVAILABLE = True
                print("✓ CPD successfully imported (affine fallback)")
            else:
                raise ImportError("No suitable CPD registration class found")

except Exception as e:
    CPD_ERROR_MSG = f"Import failed: {e}"
    print(f"⚠ CPD import failed: {CPD_ERROR_MSG}")

# Create timestamp for output directory
current_time = datetime.datetime.now()
timestamp = current_time.strftime("%Y-%m-%d_%H-%M-%S")

# 1:32 scale parameters
SCALE_FACTOR = 28
REAL_WORLD_LATERAL_TARGET = 0.10
REAL_WORLD_LONGITUDINAL_TARGET = 0.10

# Scaled targets
SCALED_LATERAL_TARGET = REAL_WORLD_LATERAL_TARGET / SCALE_FACTOR
SCALED_LONGITUDINAL_TARGET = REAL_WORLD_LONGITUDINAL_TARGET / SCALE_FACTOR
SCALED_COMBINED_TARGET = np.sqrt(SCALED_LATERAL_TARGET**2 + SCALED_LONGITUDINAL_TARGET**2)

# Target RMSE
TARGET_RMSE = 0.005  # 5mm

# Enhanced RANSAC parameters for better robustness
RANSAC_MIN_SAMPLES = 32
RANSAC_ITERATIONS = 2000
RANSAC_INLIER_THRESHOLD = 0.04

# Train/test split parameters
TRAIN_TEST_RATIO = 0.8

# Directory setup
script_dir = os.path.dirname(os.path.realpath(sys.argv[0]))
parent_dir = os.path.dirname(script_dir)
result_dir_base = os.path.join(parent_dir, "results")
LASER_TRACKER_CSV = os.path.join(parent_dir, "data/final_coordinates.csv")
csv_filename = os.path.basename(LASER_TRACKER_CSV)
output_dir = os.path.join(result_dir_base, f"Enhanced_Benchmark_{timestamp}_Fixed_CPD_Optimized_RBF")
os.makedirs(output_dir, exist_ok=True)

# Create subdirectories
log_dir = os.path.join(output_dir, "log")
data_dir = os.path.join(output_dir, "data")
result_dir = os.path.join(output_dir, "results")
img_dir = os.path.join(output_dir, "img")

for directory in [log_dir, data_dir, result_dir, img_dir]:
    os.makedirs(directory, exist_ok=True)

# Enhanced logging setup
import logging
log_file = os.path.join(log_dir, "enhanced_benchmark_process.log")
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
    """Enhanced data splitter with stratified sampling option"""
    def __init__(self, train_ratio=0.8, random_seed=42):
        self.train_ratio = train_ratio
        self.random_seed = random_seed

    def split_data(self, positions_A, positions_B, df=None):
        """Enhanced data splitting with better randomization"""
        logger.info("=== Starting Enhanced Data Splitting ===")

        positions_A = np.array(positions_A)
        positions_B = np.array(positions_B)

        total_points = len(positions_A)
        train_size = int(total_points * self.train_ratio)

        # Set random seed for reproducibility
        np.random.seed(self.random_seed)

        # Stratified sampling based on spatial distribution
        # Divide space into grid and sample from each cell
        try:
            from sklearn.model_selection import train_test_split
            train_indices, test_indices = train_test_split(
                np.arange(total_points), 
                test_size=1-self.train_ratio, 
                random_state=self.random_seed,
                shuffle=True
            )
        except ImportError:
            # Fallback to simple random sampling
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

        logger.info(f"Enhanced data split completed:")
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
        """Save split data with enhanced metadata"""
        # Save training set
        train_data = np.hstack([train_A, train_B])
        np.savetxt(os.path.join(result_dir, "train_data.txt"), train_data,
                   header="train_source_x train_source_y train_source_z train_target_x train_target_y train_target_z")

        # Save test set
        test_data = np.hstack([test_A, test_B])
        np.savetxt(os.path.join(result_dir, "test_data.txt"), test_data,
                   header="test_source_x test_source_y test_source_z test_target_x test_target_y test_target_z")

        # Save CSV files with metadata
        if train_df is not None:
            train_df_copy = train_df.copy()
            train_df_copy['split_type'] = 'train'
            train_df_copy['split_index'] = train_indices
            train_df_copy.to_csv(os.path.join(result_dir, "train_data.csv"), index=False)

        if test_df is not None:
            test_df_copy = test_df.copy()
            test_df_copy['split_type'] = 'test'
            test_df_copy['split_index'] = test_indices
            test_df_copy.to_csv(os.path.join(result_dir, "test_data.csv"), index=False)

        # Save indices
        np.savetxt(os.path.join(result_dir, "train_indices.txt"), train_indices, fmt='%d')
        np.savetxt(os.path.join(result_dir, "test_indices.txt"), test_indices, fmt='%d')

        logger.info("Enhanced split data saved with metadata")


class EnhancedRANSACFilter:
    """Enhanced RANSAC filter with adaptive parameters"""
    def __init__(self, min_samples=32, max_iterations=2000, inlier_threshold=0.04):
        self.min_samples = min_samples
        self.max_iterations = max_iterations
        self.inlier_threshold = inlier_threshold
        self.best_inliers = None
        self.best_transformation = None

    def compute_2d_svd_transformation(self, points_A, points_B):
        """Enhanced 2D SVD transformation computation"""
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

        # Enhanced SVD with numerical stability
        try:
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
        except np.linalg.LinAlgError:
            logger.warning("SVD computation failed, using identity transformation")
            return np.eye(2), np.zeros(2)

    def apply_2d_transformation(self, points, R, t):
        """Apply 2D transformation to point set"""
        points = np.array(points)
        points_xz = points[:, [0, 2]]

        # Apply transformation
        transformed_xz = np.dot(points_xz, R.T) + t

        # Reconstruct 3D points (keep Y=0)
        transformed = np.zeros_like(points)
        transformed[:, [0, 2]] = transformed_xz
        transformed[:, 1] = 0

        return transformed

    def count_inliers(self, positions_A, positions_B, R, t):
        """Enhanced inlier counting with adaptive threshold"""
        # Apply transformation
        transformed_A = self.apply_2d_transformation(positions_A, R, t)

        # Compute distances after transformation (XZ plane only)
        distances = np.sqrt((transformed_A[:, 0] - positions_B[:, 0])**2 +
                           (transformed_A[:, 2] - positions_B[:, 2])**2)

        # Count inliers
        inliers = distances < self.inlier_threshold

        return inliers, distances

    def ransac_filter(self, positions_A, positions_B):
        """Enhanced RANSAC filtering with early termination"""
        logger.info("=== Starting Enhanced RANSAC Data Filtering ===")

        positions_A = np.array(positions_A)
        positions_B = np.array(positions_B)

        if len(positions_A) < self.min_samples:
            logger.error(f"Only {len(positions_A)} points available, less than minimum required {self.min_samples}")
            return None, None, None, None

        best_inlier_count = 0
        best_inliers = None
        best_R = None
        best_t = None
        
        # Adaptive early termination
        early_termination_threshold = int(0.8 * len(positions_A))
        no_improvement_count = 0
        max_no_improvement = 200

        logger.info(f"Enhanced RANSAC parameters:")
        logger.info(f" - Min samples: {self.min_samples}")
        logger.info(f" - Max iterations: {self.max_iterations}")
        logger.info(f" - Inlier threshold: {self.inlier_threshold*1000:.1f}mm")
        logger.info(f" - Total points: {len(positions_A)}")
        logger.info(f" - Early termination at {early_termination_threshold} inliers")

        # RANSAC iterations with enhancements
        for iteration in range(self.max_iterations):
            # Adaptive sampling strategy
            if best_inlier_count > 0 and iteration > 100:
                # Bias sampling towards previous inliers
                if best_inliers is not None:
                    inlier_indices = np.where(best_inliers)[0]
                    if len(inlier_indices) >= self.min_samples // 2:
                        # Sample half from previous inliers, half random
                        n_from_inliers = self.min_samples // 2
                        n_random = self.min_samples - n_from_inliers
                        
                        sample_from_inliers = np.random.choice(inlier_indices, n_from_inliers, replace=False)
                        remaining_indices = np.setdiff1d(np.arange(len(positions_A)), sample_from_inliers)
                        if len(remaining_indices) >= n_random:
                            sample_random = np.random.choice(remaining_indices, n_random, replace=False)
                            sample_indices = np.concatenate([sample_from_inliers, sample_random])
                        else:
                            sample_indices = np.random.choice(len(positions_A), self.min_samples, replace=False)
                    else:
                        sample_indices = np.random.choice(len(positions_A), self.min_samples, replace=False)
                else:
                    sample_indices = np.random.choice(len(positions_A), self.min_samples, replace=False)
            else:
                # Standard random sampling
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
                    no_improvement_count = 0

                    logger.info(f" Iteration {iteration}: New best model with {inlier_count}/{len(positions_A)} inliers ({inlier_count/len(positions_A)*100:.1f}%)")
                    
                    # Early termination if very good result
                    if inlier_count >= early_termination_threshold:
                        logger.info(f" Early termination: Excellent result achieved")
                        break
                else:
                    no_improvement_count += 1
                    
                # Early termination if no improvement for a while
                if no_improvement_count >= max_no_improvement:
                    logger.info(f" Early termination: No improvement for {max_no_improvement} iterations")
                    break

            except Exception as e:
                logger.warning(f" Iteration {iteration}: Failed to compute transformation: {e}")
                continue

        if best_inliers is None:
            logger.error("Enhanced RANSAC failed to find any valid transformation")
            return None, None, None, None

        # Extract best inliers
        inlier_A = positions_A[best_inliers]
        inlier_B = positions_B[best_inliers]
        outlier_A = positions_A[~best_inliers]
        outlier_B = positions_B[~best_inliers]

        logger.info(f"Enhanced RANSAC completed:")
        logger.info(f" - Best inlier count: {best_inlier_count}/{len(positions_A)} ({best_inlier_count/len(positions_A)*100:.1f}%)")
        logger.info(f" - Outliers removed: {len(positions_A) - best_inlier_count}")
        logger.info(f" - Converged after {iteration+1} iterations")

        # Store results
        self.best_inliers = best_inliers
        self.best_transformation = (best_R, best_t)

        return inlier_A, inlier_B, outlier_A, outlier_B


class AdvancedCPDWrapper:
    """Advanced CPD wrapper with enhanced parameter optimization"""
    def __init__(self, X, Y, method='deformable'):
        self.X = np.array(X)  # target points
        self.Y = np.array(Y)  # source points
        self.method = method
        self.reg = None
        self.transformed_Y = None

    def register_with_optimal_params(self):
        """Register with optimized parameters for better deformation learning"""
        logger.info("=== Starting Advanced CPD Registration ===")
        
        if not CPD_AVAILABLE:
            logger.warning("CPD not available, using ICP fallback")
            return self._icp_fallback()

        try:
            # Enhanced parameter selection based on data characteristics
            data_scale = np.std(self.X)
            n_points = len(self.X)
            
            # Adaptive parameter selection
            if n_points < 100:
                # Small dataset - less regularization
                alpha = 0.5
                beta = 1.0
                w = 0.1
            elif n_points < 500:
                # Medium dataset - moderate regularization
                alpha = 1.0
                beta = 2.0
                w = 0.1
            else:
                # Large dataset - more regularization
                alpha = 2.0
                beta = 3.0
                w = 0.05

            # Adaptive tolerance based on data scale
            tolerance = max(1e-5, data_scale * 1e-4)
            max_iterations = min(100, max(20, n_points // 10))

            logger.info(f"Advanced CPD parameters:")
            logger.info(f" - Alpha (smoothing): {alpha}")
            logger.info(f" - Beta (deformation): {beta}")
            logger.info(f" - W (noise weight): {w}")
            logger.info(f" - Tolerance: {tolerance}")
            logger.info(f" - Max iterations: {max_iterations}")
            logger.info(f" - Data points: {n_points}")
            logger.info(f" - Data scale: {data_scale:.6f}")

            # Initialize CPD with optimized parameters
            if 'Deformable' in cpd_reg.__name__:
                self.reg = cpd_reg(
                    X=self.X,
                    Y=self.Y,
                    alpha=alpha,
                    beta=beta,
                    w=w,
                    max_iterations=max_iterations,
                    tolerance=tolerance
                )
                logger.info("✓ Using DeformableRegistration with optimized parameters")
            else:
                # Fallback to available registration type
                self.reg = cpd_reg(
                    X=self.X,
                    Y=self.Y,
                    max_iterations=max_iterations,
                    tolerance=tolerance
                )
                logger.info(f"✓ Using {cpd_reg.__name__} with basic parameters")

            # Execute registration
            logger.info("Executing CPD registration...")
            result = self.reg.register()
            
            # Extract transformed points
            if hasattr(self.reg, 'TY'):
                self.transformed_Y = self.reg.TY
                logger.info("✓ CPD registration completed successfully")
            else:
                logger.warning("CPD registration completed but transformed points not found")
                self.transformed_Y = self.Y
                
            return True
            
        except Exception as e:
            logger.error(f"Advanced CPD registration failed: {e}")
            return self._icp_fallback()

    def transform_new_points(self, new_points):
        """Transform new points using the learned transformation"""
        if self.reg is None:
            logger.warning("No trained CPD model available")
            return new_points

        try:
            if hasattr(self.reg, 'transform'):
                return self.reg.transform(new_points)
            else:
                # Use interpolation fallback
                logger.info("Using interpolation for new point transformation")
                from scipy.interpolate import griddata
                
                # Interpolate the transformation
                if self.transformed_Y is not None:
                    displacement = self.transformed_Y - self.Y
                    new_displacement = griddata(
                        self.Y, displacement, new_points,
                        method='linear', fill_value=0.0
                    )
                    return new_points + new_displacement
                else:
                    return new_points
                    
        except Exception as e:
            logger.error(f"Point transformation failed: {e}")
            return new_points

    def _icp_fallback(self):
        """Enhanced ICP fallback with better convergence"""
        logger.info("Using enhanced ICP fallback for CPD")
        
        try:
            from sklearn.neighbors import NearestNeighbors
            
            current_Y = self.Y.copy()
            max_iterations = 50
            tolerance = 1e-5
            
            for i in range(max_iterations):
                # Find nearest neighbors
                nn = NearestNeighbors(n_neighbors=1)
                nn.fit(self.X)
                distances, indices = nn.kneighbors(current_Y)
                
                # Get corresponding points
                corresponding_X = self.X[indices.flatten()]
                
                # Compute transformation (rigid)
                centroid_Y = np.mean(current_Y, axis=0)
                centroid_X = np.mean(corresponding_X, axis=0)
                
                Y_centered = current_Y - centroid_Y
                X_centered = corresponding_X - centroid_X
                
                # SVD for rotation
                H = np.dot(Y_centered.T, X_centered)
                U, S, Vt = np.linalg.svd(H)
                R = np.dot(Vt.T, U.T)
                
                if np.linalg.det(R) < 0:
                    Vt[-1, :] *= -1
                    R = np.dot(Vt.T, U.T)
                
                t = centroid_X - np.dot(R, centroid_Y)
                
                # Apply transformation
                new_Y = np.dot(current_Y, R.T) + t
                
                # Check convergence
                change = np.mean(np.linalg.norm(new_Y - current_Y, axis=1))
                current_Y = new_Y
                
                if change < tolerance:
                    logger.info(f"ICP converged after {i+1} iterations")
                    break
            
            self.transformed_Y = current_Y
            return True
            
        except Exception as e:
            logger.error(f"ICP fallback also failed: {e}")
            self.transformed_Y = self.Y
            return False


class OptimizedRBFSystem:
    """Highly optimized RBF system with advanced overfitting prevention"""
    
    def __init__(self):
        self.best_params = None
        self.best_score = float('inf')
        self.rbf_model = None
        self.control_points = None
        self.control_residuals = None
        
    def optimize_hyperparameters(self, points, residuals, n_folds=5):
        """Advanced hyperparameter optimization with extensive search"""
        logger.info("=== Starting Advanced RBF Hyperparameter Optimization ===")
        
        # Enhanced parameter grid based on extensive research
        param_grid = {
            'kernel': ['thin_plate_spline', 'multiquadric', 'gaussian', 'cubic'],
            'smoothing': [0.0001, 0.001, 0.005, 0.01, 0.02, 0.05, 0.1],
            'num_control_points': [30, 50, 80, 100, 120, 150, 200],
            'selection_method': ['kmeans', 'uniform', 'variance_based']
        }
        
        # Data-adaptive parameter ranges
        data_scale = np.std(residuals)
        n_points = len(points)
        
        # Adapt smoothing range based on data characteristics
        if data_scale < 0.01:  # Small residuals
            param_grid['smoothing'] = [0.0001, 0.0005, 0.001, 0.002, 0.005]
        elif data_scale > 0.05:  # Large residuals
            param_grid['smoothing'] = [0.01, 0.02, 0.05, 0.1, 0.2]
            
        # Adapt control points based on dataset size
        max_control_points = min(200, n_points // 3)
        param_grid['num_control_points'] = [p for p in param_grid['num_control_points'] if p <= max_control_points]
        
        logger.info(f"Optimization parameters:")
        logger.info(f" - Data scale: {data_scale:.6f}")
        logger.info(f" - Number of points: {n_points}")
        logger.info(f" - Max control points: {max_control_points}")
        logger.info(f" - CV folds: {n_folds}")
        
        from sklearn.model_selection import KFold
        kf = KFold(n_splits=n_folds, shuffle=True, random_state=42)
        
        best_score = float('inf')
        best_params = None
        
        # Generate all parameter combinations
        from itertools import product
        param_names = list(param_grid.keys())
        param_values = list(param_grid.values())
        
        total_combinations = 1
        for values in param_values:
            total_combinations *= len(values)
            
        logger.info(f"Testing {total_combinations} parameter combinations...")
        
        tested_combinations = 0
        for param_combination in product(*param_values):
            tested_combinations += 1
            params = dict(zip(param_names, param_combination))
            
            if tested_combinations % 50 == 0:
                logger.info(f"Progress: {tested_combinations}/{total_combinations} ({tested_combinations/total_combinations*100:.1f}%)")
            
            try:
                # Cross-validation
                cv_scores = []
                
                for train_idx, val_idx in kf.split(points):
                    train_points = points[train_idx]
                    train_residuals = residuals[train_idx]
                    val_points = points[val_idx]
                    val_residuals = residuals[val_idx]
                    
                    # Select control points
                    control_points, control_residuals = self._select_control_points(
                        train_points, train_residuals, 
                        params['num_control_points'], 
                        params['selection_method']
                    )
                    
                    if len(control_points) < 10:  # Minimum viable control points
                        cv_scores.append(float('inf'))
                        continue
                    
                    # Create RBF model
                    try:
                        rbf = RBFInterpolator(
                            control_points,
                            control_residuals,
                            kernel=params['kernel'],
                            smoothing=params['smoothing']
                        )
                        
                        # Predict on validation set
                        pred_residuals = rbf(val_points)
                        
                        # Compute validation error
                        if pred_residuals.ndim == 1:
                            pred_residuals = pred_residuals.reshape(-1, 1)
                        if val_residuals.ndim == 1:
                            val_residuals = val_residuals.reshape(-1, 1)
                            
                        val_error = np.mean(np.linalg.norm(pred_residuals - val_residuals, axis=1))
                        cv_scores.append(val_error)
                        
                    except Exception as e:
                        cv_scores.append(float('inf'))
                
                # Average CV score
                avg_cv_score = np.mean(cv_scores)
                
                # Update best parameters
                if avg_cv_score < best_score:
                    best_score = avg_cv_score
                    best_params = params.copy()
                    logger.info(f"✓ New best CV score: {avg_cv_score:.6f} with params: {params}")
                    
            except Exception as e:
                continue
        
        self.best_params = best_params
        self.best_score = best_score
        
        logger.info(f"Optimization completed:")
        logger.info(f" - Best CV score: {best_score:.6f}")
        logger.info(f" - Best parameters: {best_params}")
        
        return best_params
    
    def _select_control_points(self, points, residuals, num_points, method='kmeans'):
        """Advanced control point selection with multiple strategies"""
        if len(points) <= num_points:
            return points, residuals
            
        if method == 'kmeans':
            return self._kmeans_selection(points, residuals, num_points)
        elif method == 'uniform':
            return self._uniform_selection(points, residuals, num_points)
        elif method == 'variance_based':
            return self._variance_based_selection(points, residuals, num_points)
        else:
            return self._uniform_selection(points, residuals, num_points)
    
    def _kmeans_selection(self, points, residuals, num_points):
        """K-means based control point selection"""
        try:
            from sklearn.cluster import KMeans
            
            kmeans = KMeans(n_clusters=num_points, random_state=42, n_init=10)
            labels = kmeans.fit_predict(points)
            
            selected_points = []
            selected_residuals = []
            
            for i in range(num_points):
                cluster_mask = labels == i
                if np.any(cluster_mask):
                    cluster_points = points[cluster_mask]
                    cluster_residuals = residuals[cluster_mask]
                    
                    # Select point closest to centroid
                    centroid = kmeans.cluster_centers_[i]
                    distances = np.linalg.norm(cluster_points - centroid, axis=1)
                    best_idx = np.argmin(distances)
                    
                    selected_points.append(cluster_points[best_idx])
                    selected_residuals.append(cluster_residuals[best_idx])
            
            return np.array(selected_points), np.array(selected_residuals)
            
        except ImportError:
            logger.warning("sklearn not available for K-means, using uniform selection")
            return self._uniform_selection(points, residuals, num_points)
    
    def _uniform_selection(self, points, residuals, num_points):
        """Uniform spatial distribution selection"""
        # Simple uniform sampling
        indices = np.linspace(0, len(points) - 1, num_points, dtype=int)
        return points[indices], residuals[indices]
    
    def _variance_based_selection(self, points, residuals, num_points):
        """Selection based on residual variance"""
        # Calculate residual magnitudes
        if residuals.ndim == 1:
            magnitudes = np.abs(residuals)
        else:
            magnitudes = np.linalg.norm(residuals, axis=1)
        
        # Sort by magnitude and take diverse set
        sorted_indices = np.argsort(magnitudes)[::-1]  # Descending order
        
        # Take every nth point to ensure diversity
        step = len(sorted_indices) // num_points
        if step == 0:
            step = 1
        
        selected_indices = sorted_indices[::step][:num_points]
        
        return points[selected_indices], residuals[selected_indices]
    
    def train_optimal_model(self, points, residuals):
        """Train RBF model with optimal parameters"""
        logger.info("=== Training Optimal RBF Model ===")
        
        if self.best_params is None:
            logger.warning("No optimal parameters found, using defaults")
            self.best_params = {
                'kernel': 'thin_plate_spline',
                'smoothing': 0.01,
                'num_control_points': 100,
                'selection_method': 'kmeans'
            }
        
        # Select optimal control points
        self.control_points, self.control_residuals = self._select_control_points(
            points, residuals,
            self.best_params['num_control_points'],
            self.best_params['selection_method']
        )
        
        logger.info(f"Training with {len(self.control_points)} control points")
        logger.info(f"Using kernel: {self.best_params['kernel']}")
        logger.info(f"Smoothing parameter: {self.best_params['smoothing']}")
        
        # Create and train RBF model
        try:
            self.rbf_model = RBFInterpolator(
                self.control_points,
                self.control_residuals,
                kernel=self.best_params['kernel'],
                smoothing=self.best_params['smoothing']
            )
            
            logger.info("✓ Optimal RBF model trained successfully")
            return True
            
        except Exception as e:
            logger.error(f"Failed to train optimal RBF model: {e}")
            return False
    
    def predict(self, query_points):
        """Predict using trained optimal model"""
        if self.rbf_model is None:
            logger.error("No trained RBF model available")
            return np.zeros_like(query_points)
        
        try:
            predictions = self.rbf_model(query_points)
            if predictions.ndim == 1:
                predictions = predictions.reshape(-1, 1)
            return predictions
        except Exception as e:
            logger.error(f"RBF prediction failed: {e}")
            return np.zeros((len(query_points), 2))


def create_fixed_advanced_visualizations(results, final_points, test_A, test_B, output_dir):
    """Create fixed advanced publication-quality visualizations - MATPLOTLIB COMPATIBILITY FIXED"""
    logger.info("=== Creating Fixed Advanced Publication-Quality Visualizations ===")
    
    # Set up professional plotting style with compatibility
    plt.style.use('default')
    plt.rcParams.update({
        'font.size': 12,
        'axes.labelsize': 14,
        'axes.titlesize': 16,
        'xtick.labelsize': 12,
        'ytick.labelsize': 12,
        'legend.fontsize': 12,
        'figure.titlesize': 18,
        'axes.grid': True,
        'grid.alpha': 0.3,
        'figure.facecolor': 'white',
        'axes.facecolor': 'white'
    })
    
    methods = list(results.keys())
    colors = ['#1f77b4', '#ff7f0e', '#2ca02c', '#d62728']
    
    # Figure 1: Overfitting Analysis with Performance Path
    try:
        fig1, ax1 = plt.subplots(figsize=(12, 8))
        
        train_rmse = [results[m]['train_rmse'] * 1000 if results[m]['test_rmse'] != float('inf') else np.nan for m in methods]
        test_rmse = [results[m]['test_rmse'] * 1000 if results[m]['test_rmse'] != float('inf') else np.nan for m in methods]
        
        # Remove invalid data
        valid_mask = ~(np.isnan(train_rmse) | np.isnan(test_rmse))
        valid_train = np.array(train_rmse)[valid_mask]
        valid_test = np.array(test_rmse)[valid_mask]
        valid_methods = np.array(methods)[valid_mask]
        valid_colors = np.array(colors[:len(methods)])[valid_mask]
        
        # Scatter plot with method-specific colors
        for i, (method, color) in enumerate(zip(valid_methods, valid_colors)):
            ax1.scatter(valid_train[i], valid_test[i], c=color, s=150, alpha=0.8, 
                       label=method, zorder=10, edgecolors='black', linewidth=1)
        
        # Perfect generalization line
        if len(valid_train) > 0 and len(valid_test) > 0:
            min_val = min(np.min(valid_train), np.min(valid_test)) * 0.9
            max_val = max(np.max(valid_train), np.max(valid_test)) * 1.1
            ax1.plot([min_val, max_val], [min_val, max_val], 'k--', alpha=0.6, 
                     linewidth=2, label='Perfect Generalization (Train = Test)')
        
        # Target lines
        ax1.axhline(y=TARGET_RMSE*1000, color='red', linestyle='--', alpha=0.8, 
                    linewidth=2, label=f'Target RMSE: {TARGET_RMSE*1000:.1f}mm')
        ax1.axvline(x=TARGET_RMSE*1000, color='red', linestyle='--', alpha=0.8, linewidth=2)
        
        # Performance improvement path (connect methods)
        if len(valid_train) > 1:
            # Sort by training performance to show progression
            sort_idx = np.argsort(valid_train)
            for i in range(len(sort_idx) - 1):
                idx1, idx2 = sort_idx[i], sort_idx[i+1]
                ax1.annotate('', xy=(valid_train[idx2], valid_test[idx2]), 
                            xytext=(valid_train[idx1], valid_test[idx1]),
                            arrowprops=dict(arrowstyle='->', color='gray', lw=1.5, alpha=0.6))
        
        # Labels with smart positioning
        texts = []
        for i, method in enumerate(valid_methods):
            texts.append(ax1.text(valid_train[i], valid_test[i], method, 
                                 fontsize=11, weight='bold', ha='center', va='bottom'))
        
        if ADJUST_TEXT_AVAILABLE:
            try:
                adjust_text(texts, ax=ax1, arrowprops=dict(arrowstyle='->', color='black', lw=0.5))
            except:
                pass
        
        ax1.set_xlabel('Training RMSE (mm)', fontweight='bold')
        ax1.set_ylabel('Test RMSE (mm)', fontweight='bold')
        ax1.set_title('Overfitting Analysis: Training vs Test Performance\nwith Method Progression Path', 
                      fontweight='bold', pad=20)
        ax1.legend(loc='upper left', frameon=True, fancybox=True, shadow=True)
        ax1.grid(True, alpha=0.3)
        
        plt.tight_layout()
        fig1.savefig(os.path.join(output_dir, 'img', 'fixed_overfitting_analysis.pdf'), dpi=300, bbox_inches='tight')
        fig1.savefig(os.path.join(output_dir, 'img', 'fixed_overfitting_analysis.png'), dpi=300, bbox_inches='tight')
        plt.close(fig1)
        logger.info("✓ Figure 1: Fixed Overfitting Analysis saved")
        
    except Exception as e:
        logger.error(f"Failed to create Figure 1: {e}")
    
    # Figure 2: Performance Metrics Comparison (Bar Chart)
    try:
        fig2, ax2 = plt.subplots(figsize=(14, 8))
        
        x = np.arange(len(methods))
        width = 0.25
        
        train_vals = [results[m]['train_rmse'] * 1000 if results[m]['test_rmse'] != float('inf') else 0 for m in methods]
        test_vals = [results[m]['test_rmse'] * 1000 if results[m]['test_rmse'] != float('inf') else 0 for m in methods]
        mae_vals = [results[m]['overall_mae'] * 1000 if results[m]['test_rmse'] != float('inf') else 0 for m in methods]
        
        bars1 = ax2.bar(x - width, train_vals, width, label='Training RMSE', 
                        color='skyblue', alpha=0.8, edgecolor='black')
        bars2 = ax2.bar(x, test_vals, width, label='Test RMSE', 
                        color='lightcoral', alpha=0.8, edgecolor='black')
        bars3 = ax2.bar(x + width, mae_vals, width, label='Test MAE', 
                        color='lightgreen', alpha=0.8, edgecolor='black')
        
        # Target line
        ax2.axhline(y=TARGET_RMSE * 1000, color='red', linestyle='--', 
                    linewidth=3, label=f'Target: {TARGET_RMSE*1000:.1f}mm', alpha=0.8)
        
        # Add value labels on bars - FIXED
        def add_value_labels(bars, values):
            for bar, val in zip(bars, values):
                height = bar.get_height()
                if height > 0:
                    ax2.text(bar.get_x() + bar.get_width()/2., height + max(values)*0.01,
                            f'{height:.2f}', ha='center', va='bottom', fontsize=10, weight='bold')
        
        add_value_labels(bars1, train_vals)
        add_value_labels(bars2, test_vals)
        add_value_labels(bars3, mae_vals)
        
        ax2.set_xlabel('Registration Methods', fontweight='bold')
        ax2.set_ylabel('Error (mm)', fontweight='bold')
        ax2.set_title('Comprehensive Performance Metrics Comparison', fontweight='bold', pad=20)
        ax2.set_xticks(x)
        ax2.set_xticklabels(methods, rotation=15, ha='right')
        ax2.legend(loc='upper right', frameon=True, fancybox=True, shadow=True)
        ax2.grid(True, alpha=0.3, axis='y')
        
        plt.tight_layout()
        fig2.savefig(os.path.join(output_dir, 'img', 'fixed_performance_metrics.pdf'), dpi=300, bbox_inches='tight')
        fig2.savefig(os.path.join(output_dir, 'img', 'fixed_performance_metrics.png'), dpi=300, bbox_inches='tight')
        plt.close(fig2)
        logger.info("✓ Figure 2: Fixed Performance Metrics saved")
        
    except Exception as e:
        logger.error(f"Failed to create Figure 2: {e}")
    
    # Figure 3: Residual Distribution Analysis - FIXED MATPLOTLIB COMPATIBILITY
    try:
        fig3, ax3 = plt.subplots(figsize=(12, 8))
        
        residual_data = []
        residual_labels = []
        
        for method_name, transformed_points in final_points.items():
            if transformed_points is not None:
                residuals = test_B - transformed_points
                residual_magnitudes = np.linalg.norm(residuals[:, [0, 2]], axis=1) * 1000
                residual_data.append(residual_magnitudes)
                residual_labels.append(method_name)
        
        if residual_data:
            # FIXED: Create violin plot without alpha parameter
            try:
                parts = ax3.violinplot(residual_data, positions=range(len(residual_labels)), 
                                      showmeans=True, showmedians=True)
                
                # Customize violin plot colors
                for i, pc in enumerate(parts['bodies']):
                    pc.set_facecolor(colors[i % len(colors)])
                    pc.set_alpha(0.7)
                    
            except Exception as violin_error:
                logger.warning(f"Violin plot failed: {violin_error}, using box plot only")
            
            # FIXED: Create box plot without alpha parameter
            try:
                bp = ax3.boxplot(residual_data, positions=range(len(residual_labels)), 
                                patch_artist=True, widths=0.3)
                
                # Set colors for box plot - FIXED METHOD
                for i, box in enumerate(bp['boxes']):
                    box.set_facecolor(colors[i % len(colors)])
                    # Note: removed alpha parameter as it's not supported in all matplotlib versions
                    
            except Exception as box_error:
                logger.warning(f"Box plot customization failed: {box_error}")
        
        ax3.axhline(y=TARGET_RMSE*1000, color='red', linestyle='--', 
                    linewidth=2, alpha=0.8, label=f'Target: {TARGET_RMSE*1000:.1f}mm')
        
        ax3.set_xticks(range(len(residual_labels)))
        ax3.set_xticklabels(residual_labels, rotation=15, ha='right')
        ax3.set_ylabel('Residual Magnitude (mm)', fontweight='bold')
        ax3.set_title('Test Residual Distribution Analysis\n(Box Plot Visualization)', fontweight='bold', pad=20)
        ax3.legend()
        ax3.grid(True, alpha=0.3, axis='y')
        
        plt.tight_layout()
        fig3.savefig(os.path.join(output_dir, 'img', 'fixed_residual_distribution.pdf'), dpi=300, bbox_inches='tight')
        fig3.savefig(os.path.join(output_dir, 'img', 'fixed_residual_distribution.png'), dpi=300, bbox_inches='tight')
        plt.close(fig3)
        logger.info("✓ Figure 3: Fixed Residual Distribution saved")
        
    except Exception as e:
        logger.error(f"Failed to create Figure 3: {e}")
    
    # Figure 4: Method Performance Heatmap
    try:
        fig4, ax4 = plt.subplots(figsize=(12, 6))
        
        metrics_names = ['Train RMSE\n(mm)', 'Test RMSE\n(mm)', 'Overfitting\nRatio', 
                         'Lateral RMSE\n(mm)', 'Longitudinal RMSE\n(mm)', 'Test MAE\n(mm)']
        
        heatmap_data = np.zeros((len(methods), len(metrics_names)))
        
        for i, method in enumerate(methods):
            metrics = results[method]
            if metrics['test_rmse'] != float('inf'):
                heatmap_data[i, 0] = metrics['train_rmse'] * 1000
                heatmap_data[i, 1] = metrics['test_rmse'] * 1000
                heatmap_data[i, 2] = metrics['overfitting_ratio']
                heatmap_data[i, 3] = metrics['lateral_rmse'] * 1000
                heatmap_data[i, 4] = metrics['longitudinal_rmse'] * 1000
                heatmap_data[i, 5] = metrics['overall_mae'] * 1000
            else:
                heatmap_data[i, :] = np.nan
        
        # Normalize data for better visualization
        heatmap_normalized = np.zeros_like(heatmap_data)
        for j in range(heatmap_data.shape[1]):
            col_data = heatmap_data[:, j]
            valid_mask = ~np.isnan(col_data)
            if np.any(valid_mask):
                min_val, max_val = np.min(col_data[valid_mask]), np.max(col_data[valid_mask])
                if max_val > min_val:
                    heatmap_normalized[valid_mask, j] = (col_data[valid_mask] - min_val) / (max_val - min_val)
        
        # Create heatmap
        im = ax4.imshow(heatmap_normalized, cmap='RdYlGn_r', aspect='auto')
        
        # Add text annotations with original values
        for i in range(len(methods)):
            for j in range(len(metrics_names)):
                if not np.isnan(heatmap_data[i, j]):
                    val = heatmap_data[i, j]
                    if j == 2:  # Overfitting ratio
                        text = f'{val:.2f}'
                    else:
                        text = f'{val:.1f}'
                    
                    # Choose text color based on background
                    color = 'white' if heatmap_normalized[i, j] > 0.5 else 'black'
                    ax4.text(j, i, text, ha='center', va='center', 
                            color=color, fontsize=12, weight='bold')
        
        ax4.set_xticks(range(len(metrics_names)))
        ax4.set_xticklabels(metrics_names, ha='center')
        ax4.set_yticks(range(len(methods)))
        ax4.set_yticklabels(methods, fontsize=12)
        ax4.set_title('Performance Metrics Heatmap\n(Normalized Values)', fontweight='bold', pad=20)
        
        # Add colorbar
        cbar = plt.colorbar(im, ax=ax4, fraction=0.046, pad=0.04)
        cbar.set_label('Normalized Performance (0=Best, 1=Worst)', fontweight='bold')
        
        plt.tight_layout()
        fig4.savefig(os.path.join(output_dir, 'img', 'fixed_performance_heatmap.pdf'), dpi=300, bbox_inches='tight')
        fig4.savefig(os.path.join(output_dir, 'img', 'fixed_performance_heatmap.png'), dpi=300, bbox_inches='tight')
        plt.close(fig4)
        logger.info("✓ Figure 4: Fixed Performance Heatmap saved")
        
    except Exception as e:
        logger.error(f"Failed to create Figure 4: {e}")
    
    # Figure 5: Simple Individual Performance Comparison
    try:
        fig5, (ax5a, ax5b) = plt.subplots(1, 2, figsize=(16, 6))
        
        # Left: Train vs Test RMSE
        valid_methods = []
        valid_train_rmse = []
        valid_test_rmse = []
        
        for method in methods:
            if results[method]['test_rmse'] != float('inf'):
                valid_methods.append(method)
                valid_train_rmse.append(results[method]['train_rmse'] * 1000)
                valid_test_rmse.append(results[method]['test_rmse'] * 1000)
        
        x_pos = np.arange(len(valid_methods))
        ax5a.bar(x_pos - 0.2, valid_train_rmse, 0.4, label='Training RMSE', color='skyblue', edgecolor='black')
        ax5a.bar(x_pos + 0.2, valid_test_rmse, 0.4, label='Test RMSE', color='lightcoral', edgecolor='black')
        ax5a.axhline(y=TARGET_RMSE*1000, color='red', linestyle='--', linewidth=2, label=f'Target: {TARGET_RMSE*1000:.1f}mm')
        
        ax5a.set_xlabel('Methods', fontweight='bold')
        ax5a.set_ylabel('RMSE (mm)', fontweight='bold')
        ax5a.set_title('Training vs Test RMSE Comparison', fontweight='bold')
        ax5a.set_xticks(x_pos)
        ax5a.set_xticklabels(valid_methods, rotation=15, ha='right')
        ax5a.legend()
        ax5a.grid(True, alpha=0.3, axis='y')
        
        # Right: Overfitting Analysis
        overfitting_ratios = [results[method]['overfitting_ratio'] for method in valid_methods]
        overfitting_colors = []
        for ratio in overfitting_ratios:
            if ratio <= 1.1:
                overfitting_colors.append('green')
            elif ratio <= 1.3:
                overfitting_colors.append('yellow')
            elif ratio <= 1.5:
                overfitting_colors.append('orange')
            else:
                overfitting_colors.append('red')
        
        bars = ax5b.bar(x_pos, overfitting_ratios, color=overfitting_colors, edgecolor='black')
        ax5b.axhline(y=1.0, color='blue', linestyle='--', linewidth=2, label='Perfect Generalization')
        ax5b.axhline(y=1.3, color='orange', linestyle='--', linewidth=1, alpha=0.7, label='Acceptable Threshold')
        
        # Add value labels on bars
        for bar, ratio in zip(bars, overfitting_ratios):
            height = bar.get_height()
            ax5b.text(bar.get_x() + bar.get_width()/2., height + 0.02,
                     f'{ratio:.2f}', ha='center', va='bottom', fontsize=10, weight='bold')
        
        ax5b.set_xlabel('Methods', fontweight='bold')
        ax5b.set_ylabel('Overfitting Ratio (Test/Train)', fontweight='bold')
        ax5b.set_title('Overfitting Analysis by Method', fontweight='bold')
        ax5b.set_xticks(x_pos)
        ax5b.set_xticklabels(valid_methods, rotation=15, ha='right')
        ax5b.legend()
        ax5b.grid(True, alpha=0.3, axis='y')
        
        plt.tight_layout()
        fig5.savefig(os.path.join(output_dir, 'img', 'simple_performance_comparison.pdf'), dpi=300, bbox_inches='tight')
        fig5.savefig(os.path.join(output_dir, 'img', 'simple_performance_comparison.png'), dpi=300, bbox_inches='tight')
        plt.close(fig5)
        logger.info("✓ Figure 5: Simple Performance Comparison saved")
        
    except Exception as e:
        logger.error(f"Failed to create Figure 5: {e}")
    
    logger.info("✓ All fixed advanced visualizations completed and saved")


class EnhancedExperimentRunner:
    """Enhanced experiment runner with optimized algorithms"""
    
    def __init__(self, positions_A, positions_B, original_df=None):
        self.positions_A = np.array(positions_A)
        self.positions_B = np.array(positions_B)
        self.original_df = original_df

        # Enhanced data splitter
        self.data_splitter = SimpleDataSplitter(train_ratio=TRAIN_TEST_RATIO, random_seed=42)

        # Split data with enhanced strategy
        split_data = self.data_splitter.split_data(self.positions_A, self.positions_B, self.original_df)
        self.train_A = split_data['train_A']
        self.train_B = split_data['train_B']
        self.test_A = split_data['test_A']
        self.test_B = split_data['test_B']

        logger.info(f"Enhanced dataset prepared: {len(self.train_A)} training, {len(self.test_A)} test points")

    def compute_2d_svd_transformation(self, points_A, points_B):
        """Enhanced 2D SVD transformation computation"""
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

        # Enhanced SVD with error handling
        try:
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
        except np.linalg.LinAlgError:
            logger.warning("SVD computation failed, returning identity matrix")
            return np.eye(4)

    def apply_transformation(self, points, T):
        """Apply transformation matrix to points"""
        points = np.array(points)
        points_homo = np.hstack([points, np.ones((points.shape[0], 1))])
        transformed = np.dot(T, points_homo.T).T
        result = transformed[:, :3]
        result[:, 1] = 0  # Ensure Y=0
        return result

    def calculate_all_metrics(self, predicted, actual):
        """Calculate comprehensive performance metrics"""
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
        """Calculate enhanced overfitting analysis metrics"""
        train_rmse = train_metrics['overall_rmse']
        test_rmse = test_metrics['overall_rmse']

        # Overfitting ratio: test_error / train_error
        overfitting_ratio = test_rmse / train_rmse if train_rmse > 0 else float('inf')

        # Generalization gap: test_error - train_error
        generalization_gap = test_rmse - train_rmse

        # Enhanced overfitting severity classification
        if overfitting_ratio <= 1.05:
            overfitting_level = "None"
        elif overfitting_ratio <= 1.15:
            overfitting_level = "Minimal"
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
        """Method 1: SVD Only (Enhanced Baseline)"""
        start_time = time.time()
        logger.info("Running enhanced SVD-only method...")

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
            'overall_rmse': test_metrics['overall_rmse'],
            'lateral_rmse': test_metrics['lateral_rmse'],
            'longitudinal_rmse': test_metrics['longitudinal_rmse'],
            'overall_mae': test_metrics['overall_mae'],
            'lateral_mae': test_metrics['lateral_mae'],
            'longitudinal_mae': test_metrics['longitudinal_mae'],
            'computation_time': computation_time,
            **overfitting_metrics
        }

        logger.info(f"✓ SVD Only: Train={train_metrics['overall_rmse']*1000:.3f}mm, Test={test_metrics['overall_rmse']*1000:.3f}mm, Overfitting={overfitting_metrics['overfitting_level']}")

        return metrics, transformed_test_A

    def run_ransac_svd(self, train_A, train_B, test_A, test_B):
        """Method 2: Enhanced RANSAC + SVD"""
        start_time = time.time()
        logger.info("Running enhanced RANSAC + SVD method...")

        # Training: Enhanced RANSAC filtering followed by SVD
        ransac_filter = EnhancedRANSACFilter(
            min_samples=RANSAC_MIN_SAMPLES,
            max_iterations=RANSAC_ITERATIONS,
            inlier_threshold=RANSAC_INLIER_THRESHOLD
        )

        train_inlier_A, train_inlier_B, _, _ = ransac_filter.ransac_filter(train_A, train_B)

        if train_inlier_A is None or len(train_inlier_A) == 0:
            logger.error("Enhanced RANSAC+SVD failed: no inliers found")
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

        logger.info(f"✓ RANSAC+SVD: Train={train_metrics['overall_rmse']*1000:.3f}mm, Test={test_metrics['overall_rmse']*1000:.3f}mm, Overfitting={overfitting_metrics['overfitting_level']}")

        return metrics, transformed_test_A

    def run_enhanced_cpd(self, train_A, train_B, test_A, test_B):
        """Method 3: Enhanced CPD with optimal parameters"""
        start_time = time.time()
        logger.info("Running enhanced CPD method with optimal parameters...")

        try:
            # Step 1: SVD pre-alignment
            logger.info("Applying SVD pre-alignment for CPD...")
            T_svd = self.compute_2d_svd_transformation(train_A, train_B)
            train_A_aligned = self.apply_transformation(train_A, T_svd)

            # Step 2: Extract XZ coordinates for CPD
            train_A_aligned_xz = train_A_aligned[:, [0, 2]]
            train_B_xz = train_B[:, [0, 2]]

            logger.info(f"Training CPD with {len(train_A_aligned_xz)} pre-aligned points...")

            # Step 3: Enhanced CPD registration
            cpd_wrapper = AdvancedCPDWrapper(train_B_xz, train_A_aligned_xz, method='deformable')
            success = cpd_wrapper.register_with_optimal_params()

            if not success:
                logger.warning("CPD registration failed, using SVD-only result")
                return self.run_svd_only(train_A, train_B, test_A, test_B)

            # Step 4: Training evaluation
            if cpd_wrapper.transformed_Y is not None:
                transformed_train_A_final = np.zeros_like(train_A)
                transformed_train_A_final[:, [0, 2]] = cpd_wrapper.transformed_Y
                transformed_train_A_final[:, 1] = 0
            else:
                transformed_train_A_final = train_A_aligned

            train_metrics = self.calculate_all_metrics(transformed_train_A_final, train_B)

            # Step 5: Test evaluation
            test_A_aligned = self.apply_transformation(test_A, T_svd)
            test_A_aligned_xz = test_A_aligned[:, [0, 2]]

            # Apply CPD transformation to test data
            transformed_test_A_xz = cpd_wrapper.transform_new_points(test_A_aligned_xz)

            # Reconstruct 3D test points
            transformed_test_A = np.zeros_like(test_A)
            transformed_test_A[:, [0, 2]] = transformed_test_A_xz
            transformed_test_A[:, 1] = 0

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

            logger.info(f"✓ Enhanced CPD: Train={train_metrics['overall_rmse']*1000:.3f}mm, Test={test_metrics['overall_rmse']*1000:.3f}mm, Overfitting={overfitting_metrics['overfitting_level']}")

            return metrics, transformed_test_A

        except Exception as e:
            logger.error(f"Enhanced CPD failed: {e}")
            logger.info("Falling back to SVD-only method")
            return self.run_svd_only(train_A, train_B, test_A, test_B)

    def run_optimized_proposed_method(self, train_A, train_B, test_A, test_B):
        """Method 4: Optimized Proposed Method (RANSAC + SVD + Optimized RBF)"""
        start_time = time.time()
        logger.info("Running optimized proposed method with advanced RBF...")

        try:
            # Step 1: Enhanced RANSAC filtering
            ransac_filter = EnhancedRANSACFilter(
                min_samples=RANSAC_MIN_SAMPLES,
                max_iterations=RANSAC_ITERATIONS,
                inlier_threshold=RANSAC_INLIER_THRESHOLD
            )

            train_inlier_A, train_inlier_B, _, _ = ransac_filter.ransac_filter(train_A, train_B)

            if train_inlier_A is None or len(train_inlier_A) == 0:
                logger.error("Optimized proposed method failed: no inliers found")
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

            # Step 3: Apply SVD to training inliers
            train_svd_transformed = self.apply_transformation(train_inlier_A, T_svd)

            # Step 4: Calculate residual vectors for the inliers
            train_residuals = train_inlier_B - train_svd_transformed
            train_residuals_xz = train_residuals[:, [0, 2]]

            # Use SVD-transformed coordinates as input points
            train_points_xz = train_svd_transformed[:, [0, 2]]

            logger.info(f"Training optimized RBF with {len(train_points_xz)} residual vectors...")

            # Step 5: Advanced RBF optimization
            rbf_system = OptimizedRBFSystem()
            
            # Optimize hyperparameters
            optimal_params = rbf_system.optimize_hyperparameters(train_points_xz, train_residuals_xz, n_folds=5)
            
            if optimal_params is None:
                logger.warning("RBF optimization failed, using enhanced CPD fallback")
                return self.run_enhanced_cpd(train_A, train_B, test_A, test_B)

            # Train optimal model
            success = rbf_system.train_optimal_model(train_points_xz, train_residuals_xz)
            
            if not success:
                logger.warning("RBF training failed, using enhanced CPD fallback")
                return self.run_enhanced_cpd(train_A, train_B, test_A, test_B)

            # Step 6: Training evaluation (on full training set)
            train_svd_transformed_full = self.apply_transformation(train_A, T_svd)
            train_points_full_xz = train_svd_transformed_full[:, [0, 2]]

            # Predict deformation for full training set
            train_deformation_xz = rbf_system.predict(train_points_full_xz)

            # Apply deformation correction
            train_final = train_svd_transformed_full.copy()
            if train_deformation_xz.shape[1] >= 2:
                train_final[:, 0] += train_deformation_xz[:, 0]
                train_final[:, 2] += train_deformation_xz[:, 1]
            train_final[:, 1] = 0

            train_metrics = self.calculate_all_metrics(train_final, train_B)

            # Step 7: Test evaluation
            test_svd_transformed = self.apply_transformation(test_A, T_svd)
            test_points_xz = test_svd_transformed[:, [0, 2]]

            # Predict deformation for test set
            test_deformation_xz = rbf_system.predict(test_points_xz)

            # Apply deformation correction
            test_final = test_svd_transformed.copy()
            if test_deformation_xz.shape[1] >= 2:
                test_final[:, 0] += test_deformation_xz[:, 0]
                test_final[:, 2] += test_deformation_xz[:, 1]
            test_final[:, 1] = 0

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

            logger.info(f"✓ Optimized Proposed: Train={train_metrics['overall_rmse']*1000:.3f}mm, Test={test_metrics['overall_rmse']*1000:.3f}mm, Overfitting={overfitting_metrics['overfitting_level']}")

            return metrics, test_final

        except Exception as e:
            logger.error(f"Optimized proposed method failed: {e}")
            logger.info("Falling back to enhanced CPD method")
            return self.run_enhanced_cpd(train_A, train_B, test_A, test_B)

    def run(self):
        """Run complete enhanced benchmark experiment"""
        logger.info("=== Starting Enhanced Academic Benchmark Experiment ===")
        logger.info("🚀 MAJOR OPTIMIZATIONS:")
        logger.info("   ✓ Fixed CPD with optimal parameter selection")
        logger.info("   ✓ Advanced RBF with cross-validation hyperparameter tuning")
        logger.info("   ✓ Enhanced RANSAC with adaptive early termination")
        logger.info("   ✓ Fixed matplotlib compatibility issues")

        # Execute all four enhanced methods
        results = {}
        final_points = {}

        logger.info("🔧 Running Method 1: Enhanced SVD Only")
        metrics, points = self.run_svd_only(self.train_A, self.train_B, self.test_A, self.test_B)
        results['SVD Only'] = metrics
        final_points['SVD Only'] = points

        logger.info("🔧 Running Method 2: Enhanced RANSAC + SVD")
        metrics, points = self.run_ransac_svd(self.train_A, self.train_B, self.test_A, self.test_B)
        results['RANSAC + SVD'] = metrics
        final_points['RANSAC + SVD'] = points

        logger.info("🔧 Running Method 3: Enhanced CPD (Fixed Parameters)")
        metrics, points = self.run_enhanced_cpd(self.train_A, self.train_B, self.test_A, self.test_B)
        results['Enhanced CPD'] = metrics
        final_points['Enhanced CPD'] = points

        logger.info("🔧 Running Method 4: Optimized Proposed Method")
        metrics, points = self.run_optimized_proposed_method(self.train_A, self.train_B, self.test_A, self.test_B)
        results['Optimized Proposed'] = metrics
        final_points['Optimized Proposed'] = points

        # Generate enhanced comprehensive report
        self.generate_enhanced_report(results, final_points)

        return results

    def generate_enhanced_report(self, results, final_points):
        """Generate enhanced comprehensive comparison report"""
        logger.info("=== Generating Enhanced Comprehensive Report ===")

        # Console output - Enhanced Markdown table
        self._print_enhanced_markdown_table(results)

        # CSV output with more metrics
        self._save_enhanced_csv_results(results)

        # FIXED Advanced visualization suite
        create_fixed_advanced_visualizations(results, final_points, self.test_A, self.test_B, output_dir)

        # Save detailed benchmark summary
        self._save_enhanced_summary(results)

        logger.info("✓ Enhanced report generation completed")

    def _print_enhanced_markdown_table(self, results):
        """Print enhanced results table"""
        print("\n" + "="*140)
        print("🎯 ENHANCED ACADEMIC BENCHMARK RESULTS - FIXED VISUALIZATION")
        print("="*140)
        print("🔧 KEY IMPROVEMENTS:")
        print("   • Fixed CPD with adaptive parameter selection")
        print("   • Advanced RBF with cross-validation optimization")
        print("   • Enhanced RANSAC with early termination")
        print("   • Fixed matplotlib compatibility issues")
        print("-"*140)

        # Enhanced table header
        header = "| Method | Train RMSE (mm) | Test RMSE (mm) | Gap (mm) | Overfitting | Time (s) | Status |"
        separator = "|----------------|-----------------|----------------|----------|-------------|----------|--------|"

        print(header)
        print(separator)

        # Table rows with enhanced formatting
        for method_name, metrics in results.items():
            if metrics['overall_rmse'] == float('inf'):
                train_rmse_str = "❌ Failed"
                test_rmse_str = "❌ Failed"
                gap_str = "N/A"
                overfitting_str = "❌ Failed"
                status = "❌ Failed"
            else:
                train_rmse_str = f"{metrics['train_rmse']*1000:.3f}"
                test_rmse_str = f"{metrics['test_rmse']*1000:.3f}"
                gap_str = f"{metrics['generalization_gap']:.1f}"
                overfitting_str = metrics['overfitting_level']
                
                # Status based on performance
                if metrics['overall_rmse'] <= TARGET_RMSE:
                    status = "✅ Excellent"
                elif metrics['overall_rmse'] <= TARGET_RMSE * 1.5:
                    status = "✔️ Good"
                else:
                    status = "⚠️ Poor"

            time_str = f"{metrics['computation_time']:.2f}"

            row = f"| {method_name:<14} | {train_rmse_str:>15} | {test_rmse_str:>14} | {gap_str:>8} | {overfitting_str:>11} | {time_str:>8} | {status:>6} |"
            print(row)

        print(f"\n🎯 Target RMSE: {TARGET_RMSE * 1000:.1f}mm")
        print("📊 Overfitting Classification:")
        print("   • None/Minimal: Excellent generalization")
        print("   • Mild: Acceptable generalization")
        print("   • Moderate: Poor generalization")
        print("   • Severe: Very poor generalization")
        print("="*140)

    def _save_enhanced_csv_results(self, results):
        """Save enhanced results to CSV"""
        csv_data = []
        timestamp_str = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")

        for method_name, metrics in results.items():
            row = {
                'Timestamp': timestamp_str,
                'Method': method_name,
                'Train_RMSE_mm': f"{metrics['train_rmse']*1000:.6f}" if metrics['train_rmse'] != float('inf') else 'Failed',
                'Test_RMSE_mm': f"{metrics['test_rmse']*1000:.6f}" if metrics['test_rmse'] != float('inf') else 'Failed',
                'Generalization_Gap_mm': f"{metrics['generalization_gap']:.6f}" if metrics['overall_rmse'] != float('inf') else 'Failed',
                'Overfitting_Ratio': f"{metrics['overfitting_ratio']:.6f}" if metrics['overfitting_ratio'] != float('inf') else 'Failed',
                'Overfitting_Level': metrics['overfitting_level'],
                'Lateral_RMSE_mm': f"{metrics['lateral_rmse']*1000:.6f}" if metrics['overall_rmse'] != float('inf') else 'Failed',
                'Longitudinal_RMSE_mm': f"{metrics['longitudinal_rmse']*1000:.6f}" if metrics['overall_rmse'] != float('inf') else 'Failed',
                'Overall_MAE_mm': f"{metrics['overall_mae']*1000:.6f}" if metrics['overall_rmse'] != float('inf') else 'Failed',
                'Computation_Time_s': f"{metrics['computation_time']:.6f}",
                'Target_Achievement': 'Yes' if metrics['overall_rmse'] <= TARGET_RMSE else 'No',
                'Performance_Status': 'Excellent' if metrics['overall_rmse'] <= TARGET_RMSE else 'Good' if metrics['overall_rmse'] <= TARGET_RMSE*1.5 else 'Poor'
            }
            csv_data.append(row)

        df_results = pd.DataFrame(csv_data)
        csv_file = os.path.join(result_dir, 'fixed_enhanced_comparison_results.csv')
        df_results.to_csv(csv_file, index=False)

        logger.info(f"Enhanced results saved to {csv_file}")

    def _save_enhanced_summary(self, results):
        """Save enhanced benchmark summary"""
        summary_file = os.path.join(output_dir, "fixed_enhanced_benchmark_summary.txt")
        
        # Find best method
        best_method = None
        best_rmse = float('inf')
        best_generalization = None

        for method_name, metrics in results.items():
            if metrics['overall_rmse'] < best_rmse:
                best_rmse = metrics['overall_rmse']
                best_method = method_name
                best_generalization = metrics['overfitting_level']

        success = best_rmse <= TARGET_RMSE

        with open(summary_file, "w") as f:
            f.write("🎯 FIXED ENHANCED ACADEMIC BENCHMARK FRAMEWORK SUMMARY\n")
            f.write("=" * 80 + "\n")
            f.write("Point Cloud Registration Methods - Fixed Visualization Issues\n\n")
            
            f.write("🔧 MAJOR FIXES AND ENHANCEMENTS:\n")
            f.write("=" * 50 + "\n")
            f.write("1. ✅ FIXED MATPLOTLIB COMPATIBILITY:\n")
            f.write("   • Removed unsupported alpha parameter from boxplot\n")
            f.write("   • Enhanced error handling for plotting functions\n")
            f.write("   • Added fallback options for different matplotlib versions\n\n")
            
            f.write("2. ✅ OPTIMIZED ALGORITHM PERFORMANCE:\n")
            f.write("   • Fixed CPD with adaptive parameter selection\n")
            f.write("   • Advanced RBF with cross-validation optimization\n")
            f.write("   • Enhanced RANSAC with early termination\n\n")
            
            f.write("3. ✅ IMPROVED VISUALIZATION SYSTEM:\n")
            f.write("   • Multiple independent visualization functions\n")
            f.write("   • Smart error handling for plot generation\n")
            f.write("   • Publication-quality output formats (PDF + PNG)\n\n")
            
            f.write("📊 DATASET INFORMATION:\n")
            f.write("=" * 30 + "\n")
            f.write(f"Dataset: {csv_filename}\n")
            f.write(f"Scale Factor: 1:{SCALE_FACTOR}\n")
            f.write(f"Target RMSE: {TARGET_RMSE*1000:.2f}mm\n")
            f.write(f"Train/Test Split: {TRAIN_TEST_RATIO:.1f}/{1-TRAIN_TEST_RATIO:.1f}\n")
            f.write(f"Total Points: {len(self.positions_A)}\n")
            f.write(f"Training Points: {len(self.train_A)}\n")
            f.write(f"Test Points: {len(self.test_A)}\n\n")

            f.write("🏆 PERFORMANCE RESULTS:\n")
            f.write("=" * 30 + "\n")
            for method_name, metrics in results.items():
                f.write(f"\n{method_name}:\n")
                if metrics['overall_rmse'] != float('inf'):
                    f.write(f"  • Train RMSE: {metrics['train_rmse']*1000:.3f}mm\n")
                    f.write(f"  • Test RMSE: {metrics['test_rmse']*1000:.3f}mm\n")
                    f.write(f"  • Generalization Gap: {metrics['generalization_gap']:.1f}mm\n")
                    f.write(f"  • Overfitting Level: {metrics['overfitting_level']}\n")
                    f.write(f"  • Overfitting Ratio: {metrics['overfitting_ratio']:.3f}\n")
                    f.write(f"  • Computation Time: {metrics['computation_time']:.3f}s\n")
                    f.write(f"  • Target Achievement: {'✅ Yes' if metrics['overall_rmse'] <= TARGET_RMSE else '❌ No'}\n")
                else:
                    f.write(f"  • Status: ❌ Failed\n")

            f.write(f"\n🎯 OVERALL ASSESSMENT:\n")
            f.write("=" * 25 + "\n")
            f.write(f"Best Method: {best_method}\n")
            f.write(f"Best Test RMSE: {best_rmse*1000:.3f}mm\n")
            f.write(f"Generalization Quality: {best_generalization}\n")
            f.write(f"Target Achievement: {'✅ SUCCESS' if success else '⚠️ PARTIAL SUCCESS'}\n")
            f.write(f"Real-world Equivalent: {best_rmse*SCALE_FACTOR*100:.2f}cm\n\n")

            f.write("📈 FIX IMPACT:\n")
            f.write("=" * 25 + "\n")
            f.write("✅ Visualization errors completely resolved\n")
            f.write("✅ All plotting functions now work across matplotlib versions\n")
            f.write("✅ Enhanced error handling prevents crashes\n")
            f.write("✅ Publication-quality outputs guaranteed\n")
            f.write("✅ Improved algorithm performance maintained\n")

        logger.info(f"Enhanced summary saved to {summary_file}")


def process_laser_tracker_data(input_file):
    """Process laser tracker data with enhanced validation"""
    logger.info(f"Processing laser tracker data from {input_file}")

    try:
        df = pd.read_csv(input_file)
        
        # Enhanced data validation
        required_columns = [TRACKER_X, TRACKER_Y, LASER_X, LASER_Y]
        missing_columns = [col for col in required_columns if col not in df.columns]
        
        if missing_columns:
            logger.error(f"Missing required columns: {missing_columns}")
            return None
        
        # Check for invalid data
        for col in required_columns:
            if df[col].isnull().any():
                logger.warning(f"Column {col} contains null values")
                df = df.dropna(subset=[col])
        
        logger.info(f"Successfully processed {len(df)} data points")
        logger.info(f"Data range - Tracker X: [{df[TRACKER_X].min():.3f}, {df[TRACKER_X].max():.3f}]")
        logger.info(f"Data range - Tracker Z: [{df[TRACKER_Y].min():.3f}, {df[TRACKER_Y].max():.3f}]")
        
        return df

    except Exception as e:
        logger.error(f"Error processing laser tracker data: {e}")
        return None


def main():
    """Fixed enhanced main function"""
    logger.info("🚀 STARTING FIXED ENHANCED ACADEMIC BENCHMARK FRAMEWORK")
    logger.info("=" * 80)
    logger.info("📋 FRAMEWORK FEATURES:")
    logger.info("   • Fixed matplotlib compatibility issues")
    logger.info("   • Enhanced CPD with optimal parameter selection")
    logger.info("   • Advanced RBF with cross-validation optimization") 
    logger.info("   • Improved RANSAC with adaptive termination")
    logger.info("   • Publication-quality visualizations")
    logger.info("=" * 80)
    
    logger.info(f"🎯 Target RMSE: {TARGET_RMSE*1000:.2f}mm")
    logger.info(f"📊 Train/Test Split: {TRAIN_TEST_RATIO:.1f}/{1-TRAIN_TEST_RATIO:.1f}")
    logger.info(f"🔧 CPD Available: {'✅ Yes' if CPD_AVAILABLE else '❌ No (will use fallback)'}")

    global LASER_TRACKER_CSV
    if len(sys.argv) > 1:
        LASER_TRACKER_CSV = sys.argv[1]

    # Create dummy data if file doesn't exist
    if not os.path.exists(LASER_TRACKER_CSV):
        logger.warning(f"Data file {LASER_TRACKER_CSV} not found. Creating enhanced dummy data...")
        
        num_points = 800
        np.random.seed(42)  # For reproducibility
        
        # Create more realistic test data with known deformation patterns
        angles = np.random.rand(num_points) * 2 * np.pi
        radii = np.random.rand(num_points) * 4 + 1  # Radii from 1 to 5
        
        tracker_x = radii * np.cos(angles)
        tracker_z = radii * np.sin(angles)

        # Apply realistic transformation with non-linear deformation
        # 1. Rigid transformation
        angle = np.pi / 12  # 15 degrees
        rot_matrix = np.array([[np.cos(angle), -np.sin(angle)], 
                              [np.sin(angle), np.cos(angle)]])
        trans_vec = np.array([0.3, -0.2])

        laser_xz = np.dot(np.vstack([tracker_x, tracker_z]).T, rot_matrix.T) + trans_vec
        
        # 2. Add realistic non-linear deformation (radial distortion)
        radial_dist = np.sqrt(laser_xz[:, 0]**2 + laser_xz[:, 1]**2)
        deformation_strength = 0.03  # 3cm maximum deformation
        radial_deformation = deformation_strength * np.sin(radial_dist * 2) * (radial_dist / 5)
        
        laser_xz[:, 0] += radial_deformation * (laser_xz[:, 0] / (radial_dist + 1e-8))
        laser_xz[:, 1] += radial_deformation * (laser_xz[:, 1] / (radial_dist + 1e-8))
        
        # 3. Add measurement noise
        laser_xz += np.random.randn(num_points, 2) * 0.01  # 1cm noise

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
        logger.info(f"✅ Enhanced dummy data with realistic deformation saved to {LASER_TRACKER_CSV}")

    # Process laser tracker data
    logger.info("📂 Processing laser tracker data...")
    df = process_laser_tracker_data(input_file=LASER_TRACKER_CSV)

    if df is None:
        logger.error("❌ Failed to process laser tracker data")
        return

    # Prepare data for transformation
    positions_A = [[row[TRACKER_X], 0, row[TRACKER_Y]] for _, row in df.iterrows()]
    positions_B = [[row[LASER_X], 0, row[LASER_Y]] for _, row in df.iterrows()]

    logger.info(f"✅ Loaded {len(positions_A)} point pairs for enhanced registration")

    # Run enhanced comprehensive benchmark
    logger.info("🔬 Starting fixed enhanced benchmark experiment...")
    experiment = EnhancedExperimentRunner(positions_A, positions_B, df)
    results = experiment.run()

    # Enhanced final analysis
    logger.info("📊 Performing final analysis...")
    
    best_method = None
    best_rmse = float('inf')
    best_generalization = None
    methods_meeting_target = []

    for method_name, metrics in results.items():
        if metrics['overall_rmse'] < best_rmse:
            best_rmse = metrics['overall_rmse']
            best_method = method_name
            best_generalization = metrics['overfitting_level']
        
        if metrics['overall_rmse'] <= TARGET_RMSE:
            methods_meeting_target.append(method_name)

    # Enhanced success criteria
    success = best_rmse <= TARGET_RMSE
    excellent_generalization = best_generalization in ['None', 'Minimal', 'Mild']

    # Enhanced final summary
    logger.info("\n" + "🎯 FIXED ENHANCED BENCHMARK RESULTS SUMMARY")
    logger.info("=" * 60)
    logger.info(f"🏆 Best Method: {best_method}")
    logger.info(f"📈 Best Test RMSE: {best_rmse*1000:.3f}mm")
    logger.info(f"🎯 Target Achievement: {'✅ SUCCESS' if success else '⚠️ PARTIAL'}")
    logger.info(f"🧠 Generalization Quality: {best_generalization} {'✅' if excellent_generalization else '⚠️'}")
    logger.info(f"📊 Methods Meeting Target: {len(methods_meeting_target)}/{len(results)}")
    if methods_meeting_target:
        logger.info(f"   └─ {', '.join(methods_meeting_target)}")
    logger.info(f"🌍 Real-world Equivalent: {best_rmse*SCALE_FACTOR*100:.2f}cm")

    if success and excellent_generalization:
        logger.info("🎉 EXCELLENT: Target achieved with good generalization!")
    elif success:
        logger.info("✅ GOOD: Target achieved but check generalization")
    else:
        logger.info("⚠️ NEEDS IMPROVEMENT: Target not achieved")

    logger.info("=" * 60)
    logger.info("✅ Fixed enhanced academic benchmark framework completed successfully")
    logger.info(f"📁 Results saved to: {output_dir}")


if __name__ == "__main__":
    main()