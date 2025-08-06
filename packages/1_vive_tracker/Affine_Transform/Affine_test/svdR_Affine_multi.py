#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Enhanced Hybrid Model Offline Training Script with Comparative Methods

This script implements a comprehensive comparison of transformation models:
1. Known Rotation + RANSAC Affine + MLP (Original Hybrid)
2. RBF-based Registration
3. CPD-based Registration
4. Enhanced Ensemble Hybrid Model

Optimizations include:
- Enhanced numerical stability
- Adaptive RANSAC parameters
- Rich MLP input features
- Comprehensive data validation
- Improved error handling
- Advanced feature engineering for sub-10mm RMSE target
- Comparative analysis with multiple registration methods
"""
import os
import datetime
import logging
import joblib
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from sklearn.model_selection import train_test_split
from sklearn.neural_network import MLPRegressor
from sklearn.preprocessing import StandardScaler, PolynomialFeatures
from sklearn.metrics import mean_squared_error, mean_absolute_error
from sklearn.ensemble import RandomForestRegressor
from sklearn.linear_model import Ridge
from sklearn.neighbors import NearestNeighbors
from scipy.interpolate import RBFInterpolator
import warnings
warnings.filterwarnings('ignore')

# Try to import CPD with enhanced error handling
CPD_AVAILABLE = False
CPD_ERROR_MSG = ""
cpd_reg = None

try:
    try:
        from pycpd import DeformableRegistration
        cpd_reg = DeformableRegistration
        CPD_AVAILABLE = True
        print("CPD successfully imported (DeformableRegistration)")
    except ImportError:
        try:
            from pycpd import AffineRegistration
            cpd_reg = AffineRegistration
            CPD_AVAILABLE = True
            print("CPD successfully imported (AffineRegistration - fallback)")
        except ImportError:
            import pycpd
            if hasattr(pycpd, 'DeformableRegistration'):
                cpd_reg = pycpd.DeformableRegistration
                CPD_AVAILABLE = True
                print("CPD successfully imported (dynamic detection)")
            elif hasattr(pycpd, 'AffineRegistration'):
                cpd_reg = pycpd.AffineRegistration
                CPD_AVAILABLE = True
                print("CPD successfully imported (affine fallback)")
            else:
                raise ImportError("No suitable CPD registration class found")
except Exception as e:
    CPD_ERROR_MSG = f"Import failed: {e}"
    print(f"CPD import failed: {CPD_ERROR_MSG}")

# =============================================================================
# 1. Configuration and Constants
# =============================================================================

# --- File and Path Configuration ---
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
PARENT_DIR = os.path.dirname(SCRIPT_DIR)
LASER_TRACKER_CSV = os.path.join(PARENT_DIR, "data/final_coordinates.csv")

# --- Output Directory Configuration ---
TIMESTAMP = datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
OUTPUT_DIR = os.path.join(PARENT_DIR, f"results/Enhanced_Comparative_Model_{TIMESTAMP}")
IMG_DIR = os.path.join(OUTPUT_DIR, "img")
MODEL_DIR = os.path.join(OUTPUT_DIR, "models")
LOG_DIR = os.path.join(OUTPUT_DIR, "log")

# --- Data Column Configuration ---
TRACKER_X = "tracker_x"
TRACKER_Z = "tracker_z"
LASER_X = "laser_x"
LASER_Z = "laser_z"

# --- Model Parameters Configuration ---
TRAIN_TEST_RATIO = 0.8
RANDOM_STATE = 42

# RANSAC Parameters (Dynamic Adjustment)
RANSAC_ITERATIONS = 3000
RANSAC_MIN_SAMPLE_SIZE = 8
RANSAC_MAX_SAMPLE_SIZE = 100
RANSAC_BASE_THRESHOLD = 0.02  # 15mm base threshold
RANSAC_INLIER_THRESHOLD = 0.04  # 20mm threshold for initial inlier filtering

# Enhanced MLP Parameters for sub-10mm target
MLP_HIDDEN_LAYERS = (512, 256, 128, 64, 32)
MLP_MAX_ITER = 5000
MLP_EARLY_STOPPING = True
MLP_VALIDATION_FRACTION = 0.15
MLP_LEARNING_RATE = 0.0005
MLP_ALPHA = 0.0001  # L2 regularization

# Numerical Stability Parameters
REGULARIZATION_LAMBDA = 1e-8
MIN_POINTS_FOR_TRAINING = 50

# --- Corrected Euler Rotation Matrix ---
R_EULER_3D = np.array([
    [ 0.67654891, -0.7363977 ,  0.0        ],
    [ 0.7363977 ,  0.67654891,  0.0        ],
    [ 0.0       ,  0.0       ,  1.0        ]
])

# =============================================================================
# 2. Environment Setup
# =============================================================================

def setup_environment():
    """Create all output directories and configure logging."""
    for directory in [OUTPUT_DIR, IMG_DIR, MODEL_DIR, LOG_DIR]:
        os.makedirs(directory, exist_ok=True)

    log_file = os.path.join(LOG_DIR, "enhanced_training_process.log")
    logging.basicConfig(
        level=logging.INFO,
        format='%(asctime)s - %(levelname)s - %(message)s',
        handlers=[
            logging.FileHandler(log_file, encoding='utf-8'),
            logging.StreamHandler()
        ]
    )
    logging.info("Environment setup completed.")

# =============================================================================
# 3. Data Validation and Preprocessing
# =============================================================================

def validate_data(source_points, target_points):
    """Validate input data quality and consistency."""
    logging.info("Starting data validation...")
    
    if len(source_points) != len(target_points):
        raise ValueError(f"Source and target points count mismatch: {len(source_points)} vs {len(target_points)}")
    
    if len(source_points) < MIN_POINTS_FOR_TRAINING:
        raise ValueError(f"Insufficient data points ({len(source_points)}), need at least {MIN_POINTS_FOR_TRAINING}")
    
    # Check for NaN or infinite values
    if np.any(np.isnan(source_points)) or np.any(np.isnan(target_points)):
        raise ValueError("Data contains NaN values")
    
    if np.any(np.isinf(source_points)) or np.any(np.isinf(target_points)):
        raise ValueError("Data contains infinite values")
    
    # Check data distribution
    src_std = np.std(source_points, axis=0)
    tgt_std = np.std(target_points, axis=0)
    
    if np.any(src_std < 1e-6) or np.any(tgt_std < 1e-6):
        logging.warning("Detected extremely small data variance, may affect training")
    
    # Check for outliers
    src_distances = np.linalg.norm(source_points - np.mean(source_points, axis=0), axis=1)
    tgt_distances = np.linalg.norm(target_points - np.mean(target_points, axis=0), axis=1)
    
    src_outliers = src_distances > (np.mean(src_distances) + 3 * np.std(src_distances))
    tgt_outliers = tgt_distances > (np.mean(tgt_distances) + 3 * np.std(tgt_distances))
    
    outlier_ratio = (np.sum(src_outliers) + np.sum(tgt_outliers)) / (2 * len(source_points))
    if outlier_ratio > 0.1:
        logging.warning(f"Detected {outlier_ratio*100:.1f}% potential outliers")
    
    logging.info(f"Data validation completed. Source range: X[{np.min(source_points[:, 0]):.3f}, {np.max(source_points[:, 0]):.3f}], "
                f"Z[{np.min(source_points[:, 1]):.3f}, {np.max(source_points[:, 1]):.3f}]")

def initial_inlier_filtering(source_points, target_points, threshold=RANSAC_INLIER_THRESHOLD):
    """Initial RANSAC filtering to identify and keep only inliers."""
    logging.info(f"Starting initial inlier filtering with threshold: {threshold} m ({threshold*1000:.1f} mm)")
    
    # Apply known rotation to source points
    R_known_2D = R_EULER_3D[0:2, 0:2]
    source_rotated = (R_known_2D @ source_points.T).T
    
    # Run RANSAC to find inliers
    T_affine, inlier_indices = multi_stage_ransac_affine(source_rotated, target_points, 
                                                        threshold_override=threshold)
    
    if T_affine is None or inlier_indices is None:
        logging.warning("Initial RANSAC filtering failed, using all points")
        return source_points, target_points, np.arange(len(source_points))
    
    # Filter to keep only inliers
    inlier_source = source_points[inlier_indices]
    inlier_target = target_points[inlier_indices]
    
    outlier_count = len(source_points) - len(inlier_source)
    outlier_ratio = outlier_count / len(source_points) * 100
    
    logging.info(f"Initial inlier filtering completed:")
    logging.info(f"  Total points: {len(source_points)}")
    logging.info(f"  Inlier points: {len(inlier_source)}")
    logging.info(f"  Outlier points: {outlier_count}")
    logging.info(f"  Outlier ratio: {outlier_ratio:.1f}%")
    
    return inlier_source, inlier_target, inlier_indices

def advanced_outlier_removal(source_points, target_points, method='isolation', contamination=0.05):
    """Advanced outlier removal using multiple methods."""
    from sklearn.ensemble import IsolationForest
    from sklearn.neighbors import LocalOutlierFactor
    
    initial_count = len(source_points)
    
    if method == 'isolation':
        # Isolation Forest method
        combined_data = np.hstack([source_points, target_points])
        iso_forest = IsolationForest(contamination=contamination, random_state=RANDOM_STATE)
        outlier_labels = iso_forest.fit_predict(combined_data)
        inlier_mask = outlier_labels == 1
        
    elif method == 'lof':
        # Local Outlier Factor method
        combined_data = np.hstack([source_points, target_points])
        lof = LocalOutlierFactor(contamination=contamination)
        outlier_labels = lof.fit_predict(combined_data)
        inlier_mask = outlier_labels == 1
        
    else:  # statistical method
        initial_distances = np.linalg.norm(target_points - source_points, axis=1)
        mean_dist = np.mean(initial_distances)
        std_dist = np.std(initial_distances)
        inlier_mask = initial_distances < (mean_dist + 2.5 * std_dist)
    
    cleaned_source = source_points[inlier_mask]
    cleaned_target = target_points[inlier_mask]
    
    removed_count = initial_count - len(cleaned_source)
    if removed_count > 0:
        logging.info(f"Removed {removed_count} outlier points ({removed_count/initial_count*100:.1f}%) using {method} method")
    
    return cleaned_source, cleaned_target, inlier_mask

# =============================================================================
# 4. Advanced Feature Engineering
# =============================================================================

def extract_comprehensive_features(points, center_point=None, include_polynomial=True, degree=2):
    """Extract comprehensive features for enhanced MLP training."""
    if center_point is None:
        center_point = np.mean(points, axis=0)
    
    features = points.copy()
    
    # Basic geometric features
    radial_dist = np.linalg.norm(points - center_point, axis=1, keepdims=True)
    features = np.hstack([features, radial_dist])
    
    # Angular features
    relative_points = points - center_point
    angles = np.arctan2(relative_points[:, 1], relative_points[:, 0]).reshape(-1, 1)
    features = np.hstack([features, angles])
    
    # Add sine and cosine of angles for periodicity
    features = np.hstack([features, np.sin(angles), np.cos(angles)])
    
    # Relative coordinates
    features = np.hstack([features, relative_points])
    
    # Quadratic features
    quad_features = np.column_stack([
        points[:, 0]**2,
        points[:, 1]**2,
        points[:, 0] * points[:, 1]
    ])
    features = np.hstack([features, quad_features])
    
    # Cubic features for higher-order distortions
    cubic_features = np.column_stack([
        points[:, 0]**3,
        points[:, 1]**3,
        points[:, 0]**2 * points[:, 1],
        points[:, 0] * points[:, 1]**2
    ])
    features = np.hstack([features, cubic_features])
    
    # Radial basis features
    radial_powers = np.column_stack([
        radial_dist**2,
        radial_dist**3,
        np.sqrt(radial_dist + 1e-8)
    ])
    features = np.hstack([features, radial_powers])
    
    # Trigonometric spatial features
    spatial_freq = [0.5, 1.0, 2.0]
    for freq in spatial_freq:
        trig_features = np.column_stack([
            np.sin(freq * points[:, 0]),
            np.cos(freq * points[:, 0]),
            np.sin(freq * points[:, 1]),
            np.cos(freq * points[:, 1]),
            np.sin(freq * (points[:, 0] + points[:, 1])),
            np.cos(freq * (points[:, 0] + points[:, 1]))
        ])
        features = np.hstack([features, trig_features])
    
    # Polynomial features (if requested)
    if include_polynomial and degree > 1:
        poly = PolynomialFeatures(degree=degree, include_bias=False, interaction_only=False)
        poly_features = poly.fit_transform(points)
        # Remove the original linear terms to avoid duplication
        poly_features = poly_features[:, 2:]  # Skip x, z terms
        features = np.hstack([features, poly_features])
    
    return features

# =============================================================================
# 5. Enhanced Transformation Functions
# =============================================================================

def find_affine_transform_robust(source_points, target_points, regularization=REGULARIZATION_LAMBDA):
    """Calculate robust affine transformation matrix using regularized least squares."""
    num_points = source_points.shape[0]
    if num_points < 3:
        return None
    
    # Build coefficient matrix
    A = np.zeros((2 * num_points, 6))
    for i in range(num_points):
        x, z = source_points[i]
        A[2 * i] = [x, z, 1, 0, 0, 0]
        A[2 * i + 1] = [0, 0, 0, x, z, 1]
    
    B = target_points.flatten()
    
    # Regularized solution
    ATA = A.T @ A + regularization * np.eye(6)
    ATB = A.T @ B
    
    try:
        p = np.linalg.solve(ATA, ATB)
        a, b, tx, c, d, ty = p
        return np.array([[a, b, tx], [c, d, ty], [0, 0, 1]])
    except np.linalg.LinAlgError:
        logging.warning("Regularized solution failed, trying SVD method")
        try:
            p, _, _, _ = np.linalg.lstsq(A, B, rcond=regularization)
            a, b, tx, c, d, ty = p
            return np.array([[a, b, tx], [c, d, ty], [0, 0, 1]])
        except:
            return None

def apply_affine_transform(points, T_affine):
    """Apply affine transformation matrix to points."""
    if points.ndim == 1:
        points = points.reshape(1, -1)
    points_homogeneous = np.hstack([points, np.ones((points.shape[0], 1))])
    transformed_homogeneous = T_affine @ points_homogeneous.T
    return transformed_homogeneous[:2, :].T

def multi_stage_ransac_affine(source_points, target_points, threshold_override=None):
    """Multi-stage RANSAC with progressive refinement for higher accuracy."""
    logging.info("Starting multi-stage RANSAC affine transformation...")
    
    total_points = len(source_points)
    base_threshold = threshold_override if threshold_override is not None else RANSAC_BASE_THRESHOLD
    
    # Stage 1: Coarse estimation with larger threshold
    coarse_threshold = base_threshold * 2
    best_T, best_inliers = single_stage_ransac(source_points, target_points, 
                                              coarse_threshold, RANSAC_ITERATIONS // 2)
    
    if best_T is None or len(best_inliers) < total_points * 0.3:
        logging.warning("Coarse RANSAC failed, using single-stage approach")
        return single_stage_ransac(source_points, target_points, base_threshold, RANSAC_ITERATIONS)
    
    # Stage 2: Fine estimation on inliers with tighter threshold
    inlier_source = source_points[best_inliers]
    inlier_target = target_points[best_inliers]
    
    fine_threshold = base_threshold
    refined_T, refined_inliers_local = single_stage_ransac(inlier_source, inlier_target, 
                                                          fine_threshold, RANSAC_ITERATIONS // 2)
    
    if refined_T is not None:
        # Map local inlier indices back to global indices
        global_refined_inliers = best_inliers[refined_inliers_local]
        logging.info(f"Multi-stage RANSAC completed. Final inliers: {len(global_refined_inliers)}/{total_points}")
        return refined_T, global_refined_inliers
    else:
        return best_T, best_inliers

def single_stage_ransac(source_points, target_points, threshold, iterations):
    """Single stage RANSAC implementation."""
    total_points = len(source_points)
    sample_size = max(RANSAC_MIN_SAMPLE_SIZE, 
                     min(RANSAC_MAX_SAMPLE_SIZE, int(0.03 * total_points)))
    
    best_inlier_count = -1
    best_T_affine = None
    best_inlier_indices = None
    best_score = -np.inf
    
    for i in range(iterations):
        # Random sampling
        sample_indices = np.random.choice(total_points, sample_size, replace=False)
        sample_source = source_points[sample_indices]
        sample_target = target_points[sample_indices]
        
        # Calculate temporary model
        T_temp = find_affine_transform_robust(sample_source, sample_target)
        if T_temp is None:
            continue
        
        # Test on all points
        predicted_points = apply_affine_transform(source_points, T_temp)
        distances = np.linalg.norm(target_points - predicted_points, axis=1)
        
        # Calculate inliers
        inlier_indices = np.where(distances < threshold)[0]
        inlier_count = len(inlier_indices)
        
        # Calculate comprehensive score
        if inlier_count > 0:
            inlier_error = np.mean(distances[inlier_indices])
            score = inlier_count - inlier_error * 10000  # Balance inlier count and quality
        else:
            score = -np.inf
        
        # Update best model
        if score > best_score and inlier_count > max(sample_size, total_points * 0.1):
            best_score = score
            best_inlier_count = inlier_count
            best_inlier_indices = inlier_indices
            # Refit using all inliers for better accuracy
            best_T_affine = find_affine_transform_robust(
                source_points[best_inlier_indices], 
                target_points[best_inlier_indices]
            )
    
    return best_T_affine, best_inlier_indices

# =============================================================================
# 6. RBF Registration Method
# =============================================================================

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
        logging.info("Starting RBF hyperparameter optimization...")
        
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
        
        logging.info(f"Data scale: {data_scale:.6f}, Number of points: {n_points}, Max control points: {max_control_points}")
        
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
            
        logging.info(f"Testing {total_combinations} parameter combinations...")
        
        tested_combinations = 0
        for param_combination in product(*param_values):
            tested_combinations += 1
            params = dict(zip(param_names, param_combination))
            
            if tested_combinations % 50 == 0:
                logging.info(f"Progress: {tested_combinations}/{total_combinations}")
            
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
                    logging.info(f"New best CV score: {avg_cv_score:.6f}")
                    
            except Exception as e:
                continue
        
        self.best_params = best_params
        self.best_score = best_score
        
        logging.info(f"RBF optimization completed. Best CV score: {best_score:.6f}")
        
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
            logging.warning("sklearn not available for K-means, using uniform selection")
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
        logging.info("Training optimal RBF model...")
        
        if self.best_params is None:
            logging.warning("No optimal parameters found, using defaults")
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
        
        logging.info(f"Training with {len(self.control_points)} control points")
        
        # Create and train RBF model
        try:
            self.rbf_model = RBFInterpolator(
                self.control_points,
                self.control_residuals,
                kernel=self.best_params['kernel'],
                smoothing=self.best_params['smoothing']
            )
            
            logging.info("Optimal RBF model trained successfully")
            return True
            
        except Exception as e:
            logging.error(f"Failed to train optimal RBF model: {e}")
            return False
    
    def predict(self, query_points):
        """Predict using trained optimal model"""
        if self.rbf_model is None:
            logging.error("No trained RBF model available")
            return np.zeros_like(query_points)
        
        try:
            predictions = self.rbf_model(query_points)
            if predictions.ndim == 1:
                predictions = predictions.reshape(-1, 1)
            return predictions
        except Exception as e:
            logging.error(f"RBF prediction failed: {e}")
            return np.zeros((len(query_points), 2))

# =============================================================================
# 7. CPD Registration Method
# =============================================================================

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
        logging.info("Starting CPD registration...")
        
        if not CPD_AVAILABLE:
            logging.warning("CPD not available, using ICP fallback")
            return self._icp_fallback()

        try:
            # Enhanced parameter selection based on data characteristics
            data_scale = np.std(self.X)
            n_points = len(self.X)
            
            # Adaptive parameter selection
            if n_points < 100:
                alpha = 0.5
                beta = 1.0
                w = 0.1
            elif n_points < 500:
                alpha = 1.0
                beta = 2.0
                w = 0.1
            else:
                alpha = 2.0
                beta = 3.0
                w = 0.05

            # Adaptive tolerance based on data scale
            tolerance = max(1e-5, data_scale * 1e-4)
            max_iterations = min(100, max(20, n_points // 10))

            logging.info(f"CPD parameters: alpha={alpha}, beta={beta}, w={w}")

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
                logging.info("Using DeformableRegistration")
            else:
                self.reg = cpd_reg(
                    X=self.X,
                    Y=self.Y,
                    max_iterations=max_iterations,
                    tolerance=tolerance
                )
                logging.info(f"Using {cpd_reg.__name__}")

            # Execute registration
            result = self.reg.register()
            
            # Extract transformed points
            if hasattr(self.reg, 'TY'):
                self.transformed_Y = self.reg.TY
                logging.info("CPD registration completed successfully")
            else:
                logging.warning("CPD registration completed but transformed points not found")
                self.transformed_Y = self.Y
                
            return True
            
        except Exception as e:
            logging.error(f"CPD registration failed: {e}")
            return self._icp_fallback()

    def transform_new_points(self, new_points):
        """Transform new points using the learned transformation"""
        if self.reg is None:
            logging.warning("No trained CPD model available")
            return new_points

        try:
            if hasattr(self.reg, 'transform'):
                return self.reg.transform(new_points)
            else:
                # Use interpolation fallback
                from scipy.interpolate import griddata
                
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
            logging.error(f"Point transformation failed: {e}")
            return new_points

    def _icp_fallback(self):
        """Enhanced ICP fallback with better convergence"""
        logging.info("Using ICP fallback for CPD")
        
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
                    logging.info(f"ICP converged after {i+1} iterations")
                    break
            
            self.transformed_Y = current_Y
            return True
            
        except Exception as e:
            logging.error(f"ICP fallback also failed: {e}")
            self.transformed_Y = self.Y
            return False

# =============================================================================
# 8. Data Loading and Processing
# =============================================================================

def load_and_prepare_data(csv_path):
    """Load data from CSV and prepare as Numpy arrays."""
    logging.info(f"Loading data from {csv_path}...")
    
    if not os.path.exists(csv_path):
        logging.warning(f"Data file not found: {csv_path}")
        logging.info("Generating simulation data for demonstration...")
        return generate_simulation_data()
        
    try:
        df = pd.read_csv(csv_path)
        logging.info(f"Successfully loaded {len(df)} data points.")
        
        source_points = df[[TRACKER_X, TRACKER_Z]].values
        target_points = df[[LASER_X, LASER_Z]].values
        
        return source_points, target_points
    except Exception as e:
        logging.error(f"Error loading data: {e}")
        logging.info("Generating simulation data for demonstration...")
        return generate_simulation_data()

def generate_simulation_data(n_points=2500):
    """Generate simulation data for demonstration."""
    logging.info(f"Generating {n_points} simulation data points...")
    
    np.random.seed(RANDOM_STATE)
    
    # Generate source point cloud (in rectangular area)
    source_x = np.random.uniform(-2.0, 2.0, n_points)
    source_z = np.random.uniform(-1.5, 1.5, n_points)
    source_points = np.column_stack([source_x, source_z])
    
    # Apply known rotation
    R_2D = R_EULER_3D[0:2, 0:2]
    rotated_points = (R_2D @ source_points.T).T
    
    # Apply affine transformation (simulating residual transformation)
    true_affine = np.array([
        [1.015, 0.08, 0.12],   # Slight scaling and shear
        [-0.06, 0.985, -0.08], 
        [0, 0, 1]
    ])
    affine_points = apply_affine_transform(rotated_points, true_affine)
    
    # Add complex nonlinear distortions
    nonlinear_distortion = np.zeros_like(affine_points)
    
    # Radial distortion
    radial_dist = np.linalg.norm(affine_points, axis=1)
    distortion_strength = 0.008
    radial_factor = distortion_strength * radial_dist**2
    
    angles = np.arctan2(affine_points[:, 1], affine_points[:, 0])
    nonlinear_distortion[:, 0] = radial_factor * np.cos(angles)
    nonlinear_distortion[:, 1] = radial_factor * np.sin(angles)
    
    # Multi-frequency spatial distortions
    freqs = [0.8, 1.6, 3.2]
    amplitudes = [0.008, 0.004, 0.002]
    
    for freq, amp in zip(freqs, amplitudes):
        nonlinear_distortion[:, 0] += amp * np.sin(freq * affine_points[:, 0]) * np.cos(freq * affine_points[:, 1])
        nonlinear_distortion[:, 1] += amp * np.cos(freq * affine_points[:, 0]) * np.sin(freq * affine_points[:, 1])
    
    # Add barrel distortion
    r_squared = affine_points[:, 0]**2 + affine_points[:, 1]**2
    barrel_factor = 0.003 * r_squared
    nonlinear_distortion[:, 0] += barrel_factor * affine_points[:, 0]
    nonlinear_distortion[:, 1] += barrel_factor * affine_points[:, 1]
    
    target_points = affine_points + nonlinear_distortion
    
    # Add realistic measurement noise
    noise_level = 0.001  # 1mm std
    target_points += np.random.normal(0, noise_level, target_points.shape)
    
    logging.info("Simulation data generated with rotation, affine transform, complex nonlinear distortions and noise.")
    
    return source_points, target_points

# =============================================================================
# 9. Enhanced Evaluation and Visualization
# =============================================================================

def comprehensive_evaluation(stage_name, source, target, predicted, save_prefix):
    """Calculate multiple evaluation metrics and generate detailed visualizations."""
    # Calculate various error metrics
    errors = np.linalg.norm(target - predicted, axis=1)
    rmse = np.sqrt(np.mean(errors**2))
    mae = np.mean(errors)
    max_error = np.max(errors)
    percentile_95 = np.percentile(errors, 95)
    percentile_99 = np.percentile(errors, 99)
    
    # Convert to millimeters
    rmse_mm = rmse * 1000
    mae_mm = mae * 1000
    max_error_mm = max_error * 1000
    percentile_95_mm = percentile_95 * 1000
    percentile_99_mm = percentile_99 * 1000
    
    logging.info(f"[{stage_name}] Performance Metrics:")
    logging.info(f"  RMSE: {rmse_mm:.3f} mm")
    logging.info(f"  MAE:  {mae_mm:.3f} mm")
    logging.info(f"  Max:  {max_error_mm:.3f} mm")
    logging.info(f"  95%:  {percentile_95_mm:.3f} mm")
    logging.info(f"  99%:  {percentile_99_mm:.3f} mm")

    # Create detailed visualizations
    fig, ((ax1, ax2), (ax3, ax4)) = plt.subplots(2, 2, figsize=(20, 16))
    
    # 1. Scatter plot comparison
    ax1.scatter(target[:, 0], target[:, 1], c='red', s=12, alpha=0.7, label='Target Points (Laser)')
    ax1.scatter(predicted[:, 0], predicted[:, 1], c='green', s=12, alpha=0.7, label='Predicted Points')
    
    # Draw error vectors (sampled for clarity)
    sample_indices = np.random.choice(len(target), min(200, len(target)), replace=False)
    for i in sample_indices:
        ax1.arrow(predicted[i, 0], predicted[i, 1], 
                  target[i, 0] - predicted[i, 0], target[i, 1] - predicted[i, 1],
                  head_width=0.003, head_length=0.006, fc='gray', ec='gray', alpha=0.6)
    
    ax1.set_xlabel("X (m)")
    ax1.set_ylabel("Z (m)")
    ax1.set_title(f"{stage_name} - Prediction vs Target\nRMSE: {rmse_mm:.3f} mm")
    ax1.legend()
    ax1.grid(True, linestyle='--', alpha=0.6)
    ax1.axis('equal')
    
    # 2. Error distribution histogram
    ax2.hist(errors * 1000, bins=50, color='skyblue', edgecolor='black', alpha=0.7)
    ax2.axvline(mae_mm, color='red', linestyle='--', linewidth=2, label=f'MAE: {mae_mm:.2f} mm')
    ax2.axvline(rmse_mm, color='orange', linestyle='--', linewidth=2, label=f'RMSE: {rmse_mm:.2f} mm')
    ax2.axvline(percentile_95_mm, color='purple', linestyle='--', linewidth=2, label=f'95%: {percentile_95_mm:.2f} mm')
    ax2.set_xlabel("Error (mm)")
    ax2.set_ylabel("Frequency")
    ax2.set_title("Point-to-Point Error Distribution")
    ax2.legend()
    ax2.grid(True, linestyle='--', alpha=0.6)
    
    # 3. Spatial error distribution
    scatter = ax3.scatter(target[:, 0], target[:, 1], c=errors*1000, 
                         cmap='viridis', s=15, alpha=0.8)
    plt.colorbar(scatter, ax=ax3, label='Error (mm)')
    ax3.set_xlabel("X (m)")
    ax3.set_ylabel("Z (m)")
    ax3.set_title("Spatial Error Distribution")
    ax3.grid(True, linestyle='--', alpha=0.6)
    ax3.axis('equal')
    
    # 4. Error cumulative distribution
    sorted_errors = np.sort(errors * 1000)
    percentiles = np.arange(1, len(sorted_errors) + 1) / len(sorted_errors) * 100
    ax4.plot(sorted_errors, percentiles, 'b-', linewidth=2)
    ax4.axvline(mae_mm, color='red', linestyle='--', label=f'MAE: {mae_mm:.2f} mm')
    ax4.axvline(rmse_mm, color='orange', linestyle='--', label=f'RMSE: {rmse_mm:.2f} mm')
    ax4.axhline(95, color='purple', linestyle='--', alpha=0.7)
    ax4.axhline(99, color='brown', linestyle='--', alpha=0.7)
    ax4.set_xlabel("Error (mm)")
    ax4.set_ylabel("Cumulative Percentage (%)")
    ax4.set_title("Error Cumulative Distribution Function")
    ax4.legend()
    ax4.grid(True, linestyle='--', alpha=0.6)
    
    plt.suptitle(f"'{stage_name}' Stage Performance Assessment", fontsize=18)
    plt.tight_layout(rect=[0, 0.03, 1, 0.95])
    plt.savefig(os.path.join(IMG_DIR, f"{save_prefix}_comprehensive_evaluation.png"), dpi=300, bbox_inches='tight')
    plt.close()
    
    return {
        'rmse': rmse,
        'mae': mae,
        'max_error': max_error,
        'percentile_95': percentile_95,
        'percentile_99': percentile_99,
        'rmse_mm': rmse_mm,
        'mae_mm': mae_mm,
        'max_error_mm': max_error_mm,
        'percentile_95_mm': percentile_95_mm,
        'percentile_99_mm': percentile_99_mm
    }

def create_residual_vector_plot(stage_name, source, target, predicted, save_prefix, train_data=None):
    """Create comprehensive residual vector visualization."""
    logging.info(f"Creating residual vector plot for {stage_name}...")
    
    plt.style.use('default')
    plt.rcParams.update({
        'font.size': 11,
        'axes.labelsize': 12,
        'axes.titlesize': 14,
        'xtick.labelsize': 10,
        'ytick.labelsize': 10,
        'legend.fontsize': 10,
        'figure.titlesize': 16
    })
    
    # Calculate residuals and errors
    residual_vectors = target - predicted
    errors = np.linalg.norm(residual_vectors, axis=1)
    
    # Create comprehensive plot
    if train_data is not None:
        fig, axes = plt.subplots(2, 4, figsize=(24, 12))
        train_source, train_target, train_predicted = train_data
        train_residuals = train_target - train_predicted
        train_errors = np.linalg.norm(train_residuals, axis=1)
    else:
        fig, axes = plt.subplots(1, 4, figsize=(24, 6))
        axes = [axes]
    
    for row_idx, (data_type, src, tgt, pred, res, err) in enumerate([
        ("Training", *train_data, train_residuals, train_errors) if train_data else (None,) * 6,
        ("Test", source, target, predicted, residual_vectors, errors)
    ]):
        if data_type is None:
            continue
            
        ax_row = axes[row_idx] if train_data else axes[0]
        
        # 1. Residual Vector Field
        ax1 = ax_row[0]
        sample_indices = np.random.choice(len(src), min(300, len(src)), replace=False)
        
        # Color points by error magnitude
        scatter = ax1.scatter(tgt[sample_indices, 0], tgt[sample_indices, 1], 
                            c=err[sample_indices] * 1000, cmap='viridis', 
                            s=20, alpha=0.7)
        
        # Draw residual vectors
        scale_factor = 100  # Amplify vectors for visibility
        for i in sample_indices[::2]:  # Show every other vector to avoid clutter
            ax1.arrow(tgt[i, 0], tgt[i, 1], 
                     res[i, 0] * scale_factor, res[i, 1] * scale_factor,
                     head_width=0.01, head_length=0.02, 
                     fc='red', ec='red', alpha=0.6, linewidth=0.8)
        
        plt.colorbar(scatter, ax=ax1, label='Error (mm)')
        ax1.set_xlabel("X (m)")
        ax1.set_ylabel("Z (m)")
        ax1.set_title(f"{data_type} - Residual Vector Field")
        ax1.grid(True, alpha=0.3)
        ax1.axis('equal')
        
        # 2. Residual Magnitude Distribution
        ax2 = ax_row[1]
        ax2.hist(err * 1000, bins=50, color='lightblue', edgecolor='black', alpha=0.7)
        ax2.axvline(np.mean(err) * 1000, color='red', linestyle='--', 
                   label=f'Mean: {np.mean(err)*1000:.2f} mm')
        ax2.axvline(np.sqrt(np.mean(err**2)) * 1000, color='orange', linestyle='--', 
                   label=f'RMSE: {np.sqrt(np.mean(err**2))*1000:.2f} mm')
        ax2.set_xlabel("Residual Magnitude (mm)")
        ax2.set_ylabel("Frequency")
        ax2.set_title(f"{data_type} - Residual Magnitude Distribution")
        ax2.legend()
        ax2.grid(True, alpha=0.3)
        
        # 3. X-direction Residuals
        ax3 = ax_row[2]
        ax3.hist(res[:, 0] * 1000, bins=50, color='lightcoral', 
                edgecolor='black', alpha=0.7)
        ax3.axvline(np.mean(res[:, 0]) * 1000, color='darkred', linestyle='--',
                   label=f'Mean: {np.mean(res[:, 0])*1000:.2f} mm')
        ax3.axvline(0, color='black', linestyle='-', linewidth=1)
        ax3.set_xlabel("X Residual (mm)")
        ax3.set_ylabel("Frequency")
        ax3.set_title(f"{data_type} - X Direction Residuals")
        ax3.legend()
        ax3.grid(True, alpha=0.3)
        
        # 4. Z-direction Residuals  
        ax4 = ax_row[3]
        ax4.hist(res[:, 1] * 1000, bins=50, color='lightgreen', 
                edgecolor='black', alpha=0.7)
        ax4.axvline(np.mean(res[:, 1]) * 1000, color='darkgreen', linestyle='--',
                   label=f'Mean: {np.mean(res[:, 1])*1000:.2f} mm')
        ax4.axvline(0, color='black', linestyle='-', linewidth=1)
        ax4.set_xlabel("Z Residual (mm)")
        ax4.set_ylabel("Frequency")
        ax4.set_title(f"{data_type} - Z Direction Residuals")
        ax4.legend()
        ax4.grid(True, alpha=0.3)
    
    plt.suptitle(f"{stage_name} - Comprehensive Residual Analysis", fontsize=18)
    plt.tight_layout(rect=[0, 0.03, 1, 0.95])
    
    # Save plot
    filename = f"{save_prefix}_residual_vector_comprehensive.png"
    plt.savefig(os.path.join(IMG_DIR, filename), dpi=300, bbox_inches='tight')
    plt.close()
    
    logging.info(f"Residual vector plot saved: {filename}")

def create_comparative_visualizations(all_results, test_source, test_target):
    """Create comparative visualizations for all methods."""
    logging.info("Creating comparative visualizations...")
    
    # Set up professional plotting style
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
    
    methods = list(all_results.keys())
    colors = ['#1f77b4', '#ff7f0e', '#2ca02c', '#d62728', '#9467bd', '#8c564b']
    
    # Figure 1: Performance Comparison Bar Chart
    fig1, ax1 = plt.subplots(figsize=(16, 10))
    
    x = np.arange(len(methods))
    width = 0.15
    
    # Collect all metrics
    train_rmse_vals = [all_results[m]['test']['train_rmse'] * 1000 if 'test' in all_results[m] else 0 for m in methods]
    test_rmse_vals = [all_results[m]['test']['test_rmse'] * 1000 if 'test' in all_results[m] else 0 for m in methods]
    train_mae_vals = [all_results[m]['test']['train_mae'] * 1000 if 'test' in all_results[m] else 0 for m in methods]
    test_mae_vals = [all_results[m]['test']['test_mae'] * 1000 if 'test' in all_results[m] else 0 for m in methods]
    
    # Add Affine metrics if available
    if 'Affine' in all_results:
        affine_rmse = all_results['Affine']['test']['test_rmse'] * 1000
        affine_mae = all_results['Affine']['test']['test_mae'] * 1000
    else:
        affine_rmse = 0
        affine_mae = 0
    
    bars1 = ax1.bar(x - 1.5*width, train_rmse_vals, width, label='Training RMSE', 
                    color='skyblue', alpha=0.8, edgecolor='black')
    bars2 = ax1.bar(x - 0.5*width, test_rmse_vals, width, label='Test RMSE', 
                    color='lightcoral', alpha=0.8, edgecolor='black')
    bars3 = ax1.bar(x + 0.5*width, train_mae_vals, width, label='Training MAE', 
                    color='lightgreen', alpha=0.8, edgecolor='black')
    bars4 = ax1.bar(x + 1.5*width, test_mae_vals, width, label='Test MAE', 
                    color='lightyellow', alpha=0.8, edgecolor='black')
    
    # Add value labels on bars
    def add_value_labels(bars, values):
        for bar, val in zip(bars, values):
            height = bar.get_height()
            if height > 0:
                ax1.text(bar.get_x() + bar.get_width()/2., height + max(values)*0.01,
                        f'{height:.2f}', ha='center', va='bottom', fontsize=9, weight='bold')
    
    add_value_labels(bars1, train_rmse_vals)
    add_value_labels(bars2, test_rmse_vals)
    add_value_labels(bars3, train_mae_vals)
    add_value_labels(bars4, test_mae_vals)
    
    ax1.set_xlabel('Registration Methods')
    ax1.set_ylabel('Error (mm)')
    ax1.set_title('Comparative Performance Metrics - All Methods')
    ax1.set_xticks(x)
    ax1.set_xticklabels(methods, rotation=15, ha='right')
    ax1.legend(loc='upper right')
    ax1.grid(True, alpha=0.3, axis='y')
    
    plt.tight_layout()
    fig1.savefig(os.path.join(IMG_DIR, 'comparative_performance_metrics.png'), dpi=300, bbox_inches='tight')
    plt.close(fig1)
    
    # Figure 2: Method Comparison Heatmap
    fig2, ax2 = plt.subplots(figsize=(14, 8))
    
    metrics_names = ['Train RMSE\n(mm)', 'Test RMSE\n(mm)', 'Train MAE\n(mm)', 'Test MAE\n(mm)', 
                     'Max Error\n(mm)', '95th Percentile\n(mm)']
    
    heatmap_data = np.zeros((len(methods), len(metrics_names)))
    
    for i, method in enumerate(methods):
        if 'test' in all_results[method]:
            metrics = all_results[method]['test']
            heatmap_data[i, 0] = metrics['train_rmse'] * 1000
            heatmap_data[i, 1] = metrics['test_rmse'] * 1000
            heatmap_data[i, 2] = metrics['train_mae'] * 1000
            heatmap_data[i, 3] = metrics['test_mae'] * 1000
            heatmap_data[i, 4] = metrics['max_error'] * 1000
            heatmap_data[i, 5] = metrics['percentile_95'] * 1000
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
    im = ax2.imshow(heatmap_normalized, cmap='RdYlGn_r', aspect='auto')
    
    # Add text annotations with original values
    for i in range(len(methods)):
        for j in range(len(metrics_names)):
            if not np.isnan(heatmap_data[i, j]):
                val = heatmap_data[i, j]
                text = f'{val:.1f}'
                
                # Choose text color based on background
                color = 'white' if heatmap_normalized[i, j] > 0.5 else 'black'
                ax2.text(j, i, text, ha='center', va='center', 
                        color=color, fontsize=11, weight='bold')
    
    ax2.set_xticks(range(len(metrics_names)))
    ax2.set_xticklabels(metrics_names, ha='center')
    ax2.set_yticks(range(len(methods)))
    ax2.set_yticklabels(methods, fontsize=12)
    ax2.set_title('Performance Metrics Heatmap - All Methods')
    
    # Add colorbar
    cbar = plt.colorbar(im, ax=ax2, fraction=0.046, pad=0.04)
    cbar.set_label('Normalized Performance (0=Best, 1=Worst)')
    
    plt.tight_layout()
    fig2.savefig(os.path.join(IMG_DIR, 'comparative_performance_heatmap.png'), dpi=300, bbox_inches='tight')
    plt.close(fig2)
    
    logging.info("Comparative visualizations completed and saved")

# =============================================================================
# 10. Ensemble Model for Enhanced Performance
# =============================================================================

def train_ensemble_mlp(train_features, train_residuals, test_features):
    """Train ensemble of different models for better performance."""
    
    # Model 1: Deep MLP
    mlp1 = MLPRegressor(
        hidden_layer_sizes=(512, 256, 128, 64),
        activation='relu',
        solver='adam',
        max_iter=MLP_MAX_ITER,
        random_state=RANDOM_STATE,
        early_stopping=True,
        validation_fraction=0.15,
        learning_rate_init=MLP_LEARNING_RATE,
        alpha=MLP_ALPHA
    )
    
    # Model 2: Wider MLP
    mlp2 = MLPRegressor(
        hidden_layer_sizes=(256, 256, 256),
        activation='tanh',
        solver='adam',
        max_iter=MLP_MAX_ITER,
        random_state=RANDOM_STATE + 1,
        early_stopping=True,
        validation_fraction=0.15,
        learning_rate_init=MLP_LEARNING_RATE * 0.8,
        alpha=MLP_ALPHA
    )
    
    # Model 3: Random Forest (for comparison)
    rf = RandomForestRegressor(
        n_estimators=200,
        max_depth=20,
        random_state=RANDOM_STATE,
        n_jobs=-1
    )
    
    # Train all models
    models = [mlp1, mlp2, rf]
    model_names = ['Deep_MLP', 'Wide_MLP', 'Random_Forest']
    
    predictions = []
    for i, (model, name) in enumerate(zip(models, model_names)):
        logging.info(f"Training {name}...")
        model.fit(train_features, train_residuals)
        pred = model.predict(test_features)
        predictions.append(pred)
        
        if hasattr(model, 'n_iter_'):
            logging.info(f"  {name} completed in {model.n_iter_} iterations")
    
    # Ensemble prediction (weighted average)
    weights = [0.5, 0.3, 0.2]  # Favor the deep MLP
    ensemble_pred = np.average(predictions, axis=0, weights=weights)
    
    return models, ensemble_pred

# =============================================================================
# 11. Registration Methods Implementation
# =============================================================================

def run_rbf_registration(src_train, tgt_train, src_test, tgt_test):
    """Run RBF-based registration method."""
    logging.info("Running RBF registration method...")
    
    try:
        # Step 1: Compute initial SVD transformation for alignment
        centroid_src = np.mean(src_train, axis=0)
        centroid_tgt = np.mean(tgt_train, axis=0)
        
        src_centered = src_train - centroid_src
        tgt_centered = tgt_train - centroid_tgt
        
        # SVD for initial alignment
        H = np.dot(src_centered.T, tgt_centered)
        U, S, Vt = np.linalg.svd(H)
        R = np.dot(Vt.T, U.T)
        
        if np.linalg.det(R) < 0:
            Vt[-1, :] *= -1
            R = np.dot(Vt.T, U.T)
        
        t = centroid_tgt - np.dot(R, centroid_src)
        
        # Apply initial transformation to training data
        train_aligned = (R @ src_train.T).T + t
        
        # Step 2: Compute residuals
        train_residuals = tgt_train - train_aligned
        
        # Step 3: Train RBF system
        rbf_system = OptimizedRBFSystem()
        optimal_params = rbf_system.optimize_hyperparameters(train_aligned, train_residuals, n_folds=3)
        
        if optimal_params is None:
            logging.warning("RBF optimization failed")
            # Fallback to simple transformation
            test_aligned = (R @ src_test.T).T + t
            
            train_errors = np.sqrt(np.sum((tgt_train - train_aligned)**2, axis=1))
            test_errors = np.sqrt(np.sum((tgt_test - test_aligned)**2, axis=1))
            
            return {
                'train_rmse': np.sqrt(np.mean(train_errors**2)),
                'test_rmse': np.sqrt(np.mean(test_errors**2)),
                'train_mae': np.mean(train_errors),
                'test_mae': np.mean(test_errors),
                'max_error': np.max(test_errors),
                'percentile_95': np.percentile(test_errors, 95),
                'percentile_99': np.percentile(test_errors, 99)
            }, test_aligned, (src_train, tgt_train, train_aligned)
        
        success = rbf_system.train_optimal_model(train_aligned, train_residuals)
        if not success:
            logging.warning("RBF training failed")
            test_aligned = (R @ src_test.T).T + t
            
            train_errors = np.sqrt(np.sum((tgt_train - train_aligned)**2, axis=1))
            test_errors = np.sqrt(np.sum((tgt_test - test_aligned)**2, axis=1))
            
            return {
                'train_rmse': np.sqrt(np.mean(train_errors**2)),
                'test_rmse': np.sqrt(np.mean(test_errors**2)),
                'train_mae': np.mean(train_errors),
                'test_mae': np.mean(test_errors),
                'max_error': np.max(test_errors),
                'percentile_95': np.percentile(test_errors, 95),
                'percentile_99': np.percentile(test_errors, 99)
            }, test_aligned, (src_train, tgt_train, train_aligned)
        
        # Step 4: Training evaluation
        train_deformation = rbf_system.predict(train_aligned)
        train_final = train_aligned + train_deformation
        
        # Step 5: Test evaluation
        test_aligned = (R @ src_test.T).T + t
        test_deformation = rbf_system.predict(test_aligned)
        test_final = test_aligned + test_deformation
        
        # Calculate metrics
        train_errors = np.sqrt(np.sum((tgt_train - train_final)**2, axis=1))
        test_errors = np.sqrt(np.sum((tgt_test - test_final)**2, axis=1))
        
        metrics = {
            'train_rmse': np.sqrt(np.mean(train_errors**2)),
            'test_rmse': np.sqrt(np.mean(test_errors**2)),
            'train_mae': np.mean(train_errors),
            'test_mae': np.mean(test_errors),
            'max_error': np.max(test_errors),
            'percentile_95': np.percentile(test_errors, 95),
            'percentile_99': np.percentile(test_errors, 99)
        }
        
        logging.info(f"RBF registration completed. Train RMSE: {metrics['train_rmse']*1000:.3f}mm, Test RMSE: {metrics['test_rmse']*1000:.3f}mm")
        
        return metrics, test_final, (src_train, tgt_train, train_final)
        
    except Exception as e:
        logging.error(f"RBF registration failed: {e}")
        # Return identity transformation as fallback
        return {
            'train_rmse': float('inf'),
            'test_rmse': float('inf'),
            'train_mae': float('inf'),
            'test_mae': float('inf'),
            'max_error': float('inf'),
            'percentile_95': float('inf'),
            'percentile_99': float('inf')
        }, src_test, (src_train, tgt_train, src_train)

def run_cpd_registration(src_train, tgt_train, src_test, tgt_test):
    """Run CPD-based registration method."""
    logging.info("Running CPD registration method...")
    
    try:
        # Step 1: Initial alignment using SVD
        centroid_src = np.mean(src_train, axis=0)
        centroid_tgt = np.mean(tgt_train, axis=0)
        
        src_centered = src_train - centroid_src
        tgt_centered = tgt_train - centroid_tgt
        
        # SVD for initial alignment
        H = np.dot(src_centered.T, tgt_centered)
        U, S, Vt = np.linalg.svd(H)
        R = np.dot(Vt.T, U.T)
        
        if np.linalg.det(R) < 0:
            Vt[-1, :] *= -1
            R = np.dot(Vt.T, U.T)
        
        t = centroid_tgt - np.dot(R, centroid_src)
        
        # Apply initial transformation
        train_aligned = (R @ src_train.T).T + t
        
        # Step 2: CPD registration
        cpd_wrapper = AdvancedCPDWrapper(tgt_train, train_aligned, method='deformable')
        success = cpd_wrapper.register_with_optimal_params()
        
        if not success:
            logging.warning("CPD registration failed, using SVD-only result")
            test_aligned = (R @ src_test.T).T + t
            
            train_errors = np.sqrt(np.sum((tgt_train - train_aligned)**2, axis=1))
            test_errors = np.sqrt(np.sum((tgt_test - test_aligned)**2, axis=1))
            
            return {
                'train_rmse': np.sqrt(np.mean(train_errors**2)),
                'test_rmse': np.sqrt(np.mean(test_errors**2)),
                'train_mae': np.mean(train_errors),
                'test_mae': np.mean(test_errors),
                'max_error': np.max(test_errors),
                'percentile_95': np.percentile(test_errors, 95),
                'percentile_99': np.percentile(test_errors, 99)
            }, test_aligned, (src_train, tgt_train, train_aligned)
        
        # Step 3: Training evaluation
        if cpd_wrapper.transformed_Y is not None:
            train_final = cpd_wrapper.transformed_Y
        else:
            train_final = train_aligned
        
        # Step 4: Test evaluation
        test_aligned = (R @ src_test.T).T + t
        test_final = cpd_wrapper.transform_new_points(test_aligned)
        
        # Calculate metrics
        train_errors = np.sqrt(np.sum((tgt_train - train_final)**2, axis=1))
        test_errors = np.sqrt(np.sum((tgt_test - test_final)**2, axis=1))
        
        metrics = {
            'train_rmse': np.sqrt(np.mean(train_errors**2)),
            'test_rmse': np.sqrt(np.mean(test_errors**2)),
            'train_mae': np.mean(train_errors),
            'test_mae': np.mean(test_errors),
            'max_error': np.max(test_errors),
            'percentile_95': np.percentile(test_errors, 95),
            'percentile_99': np.percentile(test_errors, 99)
        }
        
        logging.info(f"CPD registration completed. Train RMSE: {metrics['train_rmse']*1000:.3f}mm, Test RMSE: {metrics['test_rmse']*1000:.3f}mm")
        
        return metrics, test_final, (src_train, tgt_train, train_final)
        
    except Exception as e:
        logging.error(f"CPD registration failed: {e}")
        # Return identity transformation as fallback
        return {
            'train_rmse': float('inf'),
            'test_rmse': float('inf'),
            'train_mae': float('inf'),
            'test_mae': float('inf'),
            'max_error': float('inf'),
            'percentile_95': float('inf'),
            'percentile_99': float('inf')
        }, src_test, (src_train, tgt_train, src_train)

# =============================================================================
# 12. Main Execution Flow
# =============================================================================

def main():
    """Enhanced main function with comprehensive offline training flow and method comparison."""
    setup_environment()
    logging.info("Starting enhanced comparative model offline training process...")

    try:
        # --- Step 0: Data Loading and Validation ---
        logging.info("\n" + "="*60)
        logging.info("Step 0: Data Loading and Validation")
        logging.info("="*60)
        
        source_points, target_points = load_and_prepare_data(LASER_TRACKER_CSV)
        if source_points is None:
            logging.error("Data loading failed, exiting program")
            return

        # Data validation
        validate_data(source_points, target_points)
        
        # --- Step 0.5: Initial Inlier Filtering with RANSAC ---
        logging.info("\n" + "="*60)
        logging.info("Step 0.5: Initial Inlier Filtering with RANSAC")
        logging.info("="*60)
        
        # Apply initial RANSAC filtering to identify inliers
        inlier_source, inlier_target, initial_inlier_indices = initial_inlier_filtering(
            source_points, target_points, threshold=RANSAC_INLIER_THRESHOLD
        )
        
        # Save inlier filtering results
        np.save(os.path.join(MODEL_DIR, "initial_inlier_indices.npy"), initial_inlier_indices)
        
        # Use inliers for all subsequent operations
        source_points = inlier_source
        target_points = inlier_target
        
        # Split inlier data
        (src_train, src_test, tgt_train, tgt_test) = train_test_split(
            source_points, target_points, train_size=TRAIN_TEST_RATIO, random_state=RANDOM_STATE
        )
        logging.info(f"Data split completed: {len(src_train)} training points, {len(src_test)} test points")

        # Dictionary to store all results
        all_results = {}

        # --- Evaluate Affine Transform Performance ---
        logging.info("\n" + "="*60)
        logging.info("Evaluating Affine Transform Performance")
        logging.info("="*60)
        
        # Apply known rotation to get post-rotation points
        R_known_2D = R_EULER_3D[0:2, 0:2]
        src_train_rotated = (R_known_2D @ src_train.T).T
        src_test_rotated = (R_known_2D @ src_test.T).T
        
        # Find affine transform using training data
        T_residual_affine, inlier_indices = multi_stage_ransac_affine(src_train_rotated, tgt_train)
        
        if T_residual_affine is not None:
            # Apply affine to both train and test
            train_affine_corrected = apply_affine_transform(src_train_rotated, T_residual_affine)
            test_affine_corrected = apply_affine_transform(src_test_rotated, T_residual_affine)
            
            # Calculate affine performance
            train_errors = np.sqrt(np.sum((tgt_train - train_affine_corrected)**2, axis=1))
            test_errors = np.sqrt(np.sum((tgt_test - test_affine_corrected)**2, axis=1))
            
            affine_metrics = {
                'train_rmse': np.sqrt(np.mean(train_errors**2)),
                'test_rmse': np.sqrt(np.mean(test_errors**2)),
                'train_mae': np.mean(train_errors),
                'test_mae': np.mean(test_errors),
                'max_error': np.max(test_errors),
                'percentile_95': np.percentile(test_errors, 95),
                'percentile_99': np.percentile(test_errors, 99)
            }
            
            all_results['Affine'] = {
                'test': affine_metrics,
                'result': test_affine_corrected,
                'train_data': (src_train, tgt_train, train_affine_corrected)
            }
            
            logging.info(f"Affine transform completed. Train RMSE: {affine_metrics['train_rmse']*1000:.3f}mm, Test RMSE: {affine_metrics['test_rmse']*1000:.3f}mm")
            
            # Create residual vector plot for Affine
            create_residual_vector_plot("Affine Transform", src_test, tgt_test, test_affine_corrected, 
                                      "0_Affine", (src_train, tgt_train, train_affine_corrected))

        # --- Method 1: RBF Registration ---
        logging.info("\n" + "="*60)
        logging.info("Method 1: RBF Registration")
        logging.info("="*60)
        
        rbf_metrics, rbf_test_result, rbf_train_data = run_rbf_registration(src_train, tgt_train, src_test, tgt_test)
        all_results['RBF Registration'] = {
            'test': rbf_metrics,
            'result': rbf_test_result,
            'train_data': rbf_train_data
        }
        
        # Evaluate RBF method
        rbf_eval = comprehensive_evaluation("RBF Registration", src_test, tgt_test, rbf_test_result, "1_RBF")
        all_results['RBF Registration']['detailed'] = rbf_eval
        
        # Create residual vector plot for RBF
        create_residual_vector_plot("RBF Registration", src_test, tgt_test, rbf_test_result, 
                                  "1_RBF", rbf_train_data)

        # --- Method 2: CPD Registration ---
        logging.info("\n" + "="*60)
        logging.info("Method 2: CPD Registration")
        logging.info("="*60)
        
        cpd_metrics, cpd_test_result, cpd_train_data = run_cpd_registration(src_train, tgt_train, src_test, tgt_test)
        all_results['CPD Registration'] = {
            'test': cpd_metrics,
            'result': cpd_test_result,
            'train_data': cpd_train_data
        }
        
        # Evaluate CPD method
        cpd_eval = comprehensive_evaluation("CPD Registration", src_test, tgt_test, cpd_test_result, "2_CPD")
        all_results['CPD Registration']['detailed'] = cpd_eval
        
        # Create residual vector plot for CPD
        create_residual_vector_plot("CPD Registration", src_test, tgt_test, cpd_test_result, 
                                  "2_CPD", cpd_train_data)

        # --- Method 3: Original Hybrid Model ---
        logging.info("\n" + "="*60)
        logging.info("Method 3: Known Rotation + RANSAC Affine + MLP (Original Hybrid)")
        logging.info("="*60)
        
        if T_residual_affine is not None:
            # Step 3: Calculate Nonlinear Residuals and Prepare Enhanced MLP Data
            train_final_residuals = tgt_train - train_affine_corrected
            
            # Extract comprehensive features
            center_point = np.mean(train_affine_corrected, axis=0)
            train_features = extract_comprehensive_features(train_affine_corrected, center_point)
            
            logging.info(f"Enhanced MLP feature dimension: {train_features.shape[1]}")

            # Step 4: Train Enhanced MLP Model
            feature_scaler = StandardScaler()
            train_features_scaled = feature_scaler.fit_transform(train_features)
            
            target_scaler = StandardScaler()
            train_residuals_scaled = target_scaler.fit_transform(train_final_residuals)
            
            # Train MLP
            mlp_model = MLPRegressor(
                hidden_layer_sizes=MLP_HIDDEN_LAYERS,
                activation='relu',
                solver='adam',
                max_iter=MLP_MAX_ITER,
                random_state=RANDOM_STATE,
                early_stopping=MLP_EARLY_STOPPING,
                validation_fraction=MLP_VALIDATION_FRACTION,
                learning_rate_init=MLP_LEARNING_RATE,
                alpha=MLP_ALPHA
            )
            
            mlp_model.fit(train_features_scaled, train_residuals_scaled)
            logging.info(f"MLP model training completed. Final iterations: {mlp_model.n_iter_}")

            # Step 5: Test Set Evaluation
            test_features = extract_comprehensive_features(test_affine_corrected, center_point)
            test_features_scaled = feature_scaler.transform(test_features)
            
            # Training evaluation
            train_pred_residuals = target_scaler.inverse_transform(mlp_model.predict(train_features_scaled))
            train_final_predicted = train_affine_corrected + train_pred_residuals
            
            # Test evaluation
            test_pred_residuals = target_scaler.inverse_transform(mlp_model.predict(test_features_scaled))
            test_final_predicted = test_affine_corrected + test_pred_residuals
            
            # Calculate metrics
            train_errors = np.sqrt(np.sum((tgt_train - train_final_predicted)**2, axis=1))
            test_errors = np.sqrt(np.sum((tgt_test - test_final_predicted)**2, axis=1))
            
            hybrid_metrics = {
                'train_rmse': np.sqrt(np.mean(train_errors**2)),
                'test_rmse': np.sqrt(np.mean(test_errors**2)),
                'train_mae': np.mean(train_errors),
                'test_mae': np.mean(test_errors),
                'max_error': np.max(test_errors),
                'percentile_95': np.percentile(test_errors, 95),
                'percentile_99': np.percentile(test_errors, 99)
            }
            
            all_results['Original Hybrid'] = {
                'test': hybrid_metrics,
                'result': test_final_predicted,
                'train_data': (src_train, tgt_train, train_final_predicted)
            }
            
            # Save model artifacts
            np.save(os.path.join(MODEL_DIR, "R_known_2D.npy"), R_known_2D)
            np.save(os.path.join(MODEL_DIR, "T_residual_affine.npy"), T_residual_affine)
            np.save(os.path.join(MODEL_DIR, "center_point.npy"), center_point)
            joblib.dump(feature_scaler, os.path.join(MODEL_DIR, "feature_scaler.pkl"))
            joblib.dump(target_scaler, os.path.join(MODEL_DIR, "target_scaler.pkl"))
            joblib.dump(mlp_model, os.path.join(MODEL_DIR, "mlp_model.pkl"))
            
            logging.info(f"Original hybrid model completed. Train RMSE: {hybrid_metrics['train_rmse']*1000:.3f}mm, Test RMSE: {hybrid_metrics['test_rmse']*1000:.3f}mm")
            
            # Evaluate Original Hybrid method
            hybrid_eval = comprehensive_evaluation("Original Hybrid", src_test, tgt_test, test_final_predicted, "3_Original_Hybrid")
            all_results['Original Hybrid']['detailed'] = hybrid_eval
            
            # Create residual vector plot for Original Hybrid
            create_residual_vector_plot("Original Hybrid", src_test, tgt_test, test_final_predicted, 
                                      "3_Original_Hybrid", (src_train, tgt_train, train_final_predicted))
        else:
            all_results['Original Hybrid'] = {
                'test': {'train_rmse': float('inf'), 'test_rmse': float('inf'), 'train_mae': float('inf'), 'test_mae': float('inf'),
                        'max_error': float('inf'), 'percentile_95': float('inf'), 'percentile_99': float('inf')},
                'result': src_test,
                'train_data': (src_train, tgt_train, src_train)
            }

        # --- Method 4: Enhanced Ensemble Hybrid Model ---
        logging.info("\n" + "="*60)
        logging.info("Method 4: Enhanced Ensemble Hybrid Model")
        logging.info("="*60)
        
        if T_residual_affine is not None:
            # Use same preprocessing as original hybrid
            test_features_scaled = feature_scaler.transform(test_features)
            
            # Train ensemble models
            models, ensemble_pred_scaled = train_ensemble_mlp(
                train_features_scaled, train_residuals_scaled, test_features_scaled
            )
            
            # Transform back to original scale
            ensemble_pred_residuals = target_scaler.inverse_transform(ensemble_pred_scaled)
            ensemble_final_predicted = test_affine_corrected + ensemble_pred_residuals
            
            # Training evaluation with ensemble
            ensemble_train_pred = np.mean([
                model.predict(train_features_scaled) for model in models[:2]  # Use only MLP models
            ], axis=0)
            ensemble_train_residuals = target_scaler.inverse_transform(ensemble_train_pred)
            ensemble_train_final = train_affine_corrected + ensemble_train_residuals
            
            # Calculate metrics
            train_errors = np.sqrt(np.sum((tgt_train - ensemble_train_final)**2, axis=1))
            test_errors = np.sqrt(np.sum((tgt_test - ensemble_final_predicted)**2, axis=1))
            
            ensemble_metrics = {
                'train_rmse': np.sqrt(np.mean(train_errors**2)),
                'test_rmse': np.sqrt(np.mean(test_errors**2)),
                'train_mae': np.mean(train_errors),
                'test_mae': np.mean(test_errors),
                'max_error': np.max(test_errors),
                'percentile_95': np.percentile(test_errors, 95),
                'percentile_99': np.percentile(test_errors, 99)
            }
            
            all_results['Enhanced Ensemble Hybrid'] = {
                'test': ensemble_metrics,
                'result': ensemble_final_predicted,
                'train_data': (src_train, tgt_train, ensemble_train_final)
            }
            
            # Save ensemble models
            for i, model in enumerate(models):
                joblib.dump(model, os.path.join(MODEL_DIR, f"ensemble_model_{i}.pkl"))
            
            logging.info(f"Enhanced ensemble hybrid completed. Train RMSE: {ensemble_metrics['train_rmse']*1000:.3f}mm, Test RMSE: {ensemble_metrics['test_rmse']*1000:.3f}mm")
            
            # Evaluate Enhanced Ensemble method
            ensemble_eval = comprehensive_evaluation("Enhanced Ensemble Hybrid", src_test, tgt_test, ensemble_final_predicted, "4_Enhanced_Ensemble")
            all_results['Enhanced Ensemble Hybrid']['detailed'] = ensemble_eval
            
            # Create residual vector plot for Enhanced Ensemble
            create_residual_vector_plot("Enhanced Ensemble Hybrid", src_test, tgt_test, ensemble_final_predicted, 
                                      "4_Enhanced_Ensemble", (src_train, tgt_train, ensemble_train_final))
        else:
            all_results['Enhanced Ensemble Hybrid'] = {
                'test': {'train_rmse': float('inf'), 'test_rmse': float('inf'), 'train_mae': float('inf'), 'test_mae': float('inf'),
                        'max_error': float('inf'), 'percentile_95': float('inf'), 'percentile_99': float('inf')},
                'result': src_test,
                'train_data': (src_train, tgt_train, src_train)
            }

        # --- Step 6: Create Comparative Visualizations ---
        logging.info("\n" + "="*60)
        logging.info("Step 6: Create Comparative Visualizations")
        logging.info("="*60)
        
        create_comparative_visualizations(all_results, src_test, tgt_test)

        # --- Final Summary Report ---
        logging.info("\n" + "="*60)
        logging.info("COMPARATIVE ANALYSIS SUMMARY")
        logging.info("="*60)
        
        # Find best method
        best_method = None
        best_rmse = float('inf')
        
        print("\nComparative Results Summary:")
        print("="*80)
        print(f"{'Method':<25} {'Train RMSE (mm)':<15} {'Test RMSE (mm)':<15} {'Train MAE (mm)':<15} {'Test MAE (mm)':<15}")
        print("-"*80)
        
        for method_name, results in all_results.items():
            if 'test' in results:
                metrics = results['test']
                train_rmse_mm = metrics['train_rmse'] * 1000 if metrics['train_rmse'] != float('inf') else float('inf')
                test_rmse_mm = metrics['test_rmse'] * 1000 if metrics['test_rmse'] != float('inf') else float('inf')
                train_mae_mm = metrics['train_mae'] * 1000 if metrics['train_mae'] != float('inf') else float('inf')
                test_mae_mm = metrics['test_mae'] * 1000 if metrics['test_mae'] != float('inf') else float('inf')
                
                print(f"{method_name:<25} {train_rmse_mm:<15.3f} {test_rmse_mm:<15.3f} {train_mae_mm:<15.3f} {test_mae_mm:<15.3f}")
                
                if test_rmse_mm < best_rmse:
                    best_rmse = test_rmse_mm
                    best_method = method_name
        
        print("="*80)
        print(f"Best Method: {best_method}")
        print(f"Best Test RMSE: {best_rmse:.3f} mm")
        
        # Save configuration
        config = {
            'R_EULER_3D': R_EULER_3D.tolist(),
            'MLP_HIDDEN_LAYERS': MLP_HIDDEN_LAYERS,
            'RANSAC_ITERATIONS': RANSAC_ITERATIONS,
            'RANSAC_BASE_THRESHOLD': RANSAC_BASE_THRESHOLD,
            'RANSAC_INLIER_THRESHOLD': RANSAC_INLIER_THRESHOLD,
            'original_points': len(source_points) + len(initial_inlier_indices),
            'inlier_points': len(source_points),
            'train_points': len(src_train),
            'test_points': len(src_test),
            'best_method': best_method,
            'best_rmse_mm': float(best_rmse) if best_rmse != float('inf') else None,
            'cpd_available': CPD_AVAILABLE
        }
        
        import json
        with open(os.path.join(MODEL_DIR, "comparative_model_config.json"), "w") as f:
            json.dump(config, f, indent=2)
        
        logging.info(f"All model artifacts saved to: {MODEL_DIR}")

        # Save detailed summary
        summary = f"""
=========================================================
     Enhanced Comparative Model Training Summary
=========================================================
Initial Inlier Filtering: {RANSAC_INLIER_THRESHOLD*1000:.1f} mm threshold
Original Data Points: {len(source_points) + len(initial_inlier_indices)}
Inlier Data Points: {len(source_points)}
Outlier Ratio: {len(initial_inlier_indices)/(len(source_points) + len(initial_inlier_indices))*100:.1f}%

Best Method: {best_method}
Best Test RMSE: {best_rmse:.3f} mm

Method Performance Comparison:
"""
        for method_name, results in all_results.items():
            if 'test' in results:
                metrics = results['test']
                summary += f"\n{method_name}:\n"
                summary += f"  Train RMSE: {metrics['train_rmse']*1000:.3f} mm\n"
                summary += f"  Test RMSE: {metrics['test_rmse']*1000:.3f} mm\n"
                summary += f"  Train MAE: {metrics['train_mae']*1000:.3f} mm\n"
                summary += f"  Test MAE: {metrics['test_mae']*1000:.3f} mm\n"
                summary += f"  Max Error: {metrics['max_error']*1000:.3f} mm\n"
                summary += f"  95th Percentile: {metrics['percentile_95']*1000:.3f} mm\n"

        summary += f"""
=========================================================
Model Files Location: {MODEL_DIR}
Log Files Location: {os.path.join(LOG_DIR, "enhanced_training_process.log")}
Visualization Files Location: {IMG_DIR}
=========================================================
"""
        
        print(summary)
        logging.info(summary)
        
        with open(os.path.join(OUTPUT_DIR, "comparative_summary.txt"), "w", encoding='utf-8') as f:
            f.write(summary)
        
        logging.info(f"Enhanced comparative training process completed successfully! All results saved in: {OUTPUT_DIR}")
        
    except Exception as e:
        logging.error(f"Error occurred during training process: {str(e)}")
        import traceback
        logging.error(f"Detailed error information:\n{traceback.format_exc()}")
        raise


if __name__ == "__main__":
    main()