#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Enhanced Hybrid Model Offline Training Script

This script implements a "Known Rotation + RANSAC Affine + MLP" hybrid transformation model.
Optimizations include:
- Enhanced numerical stability
- Adaptive RANSAC parameters
- Rich MLP input features
- Comprehensive data validation
- Improved error handling
- Advanced feature engineering for sub-10mm RMSE target
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
import warnings
warnings.filterwarnings('ignore')

# =============================================================================
# 1. Configuration and Constants
# =============================================================================

# --- File and Path Configuration ---
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
PARENT_DIR = os.path.dirname(SCRIPT_DIR)
LASER_TRACKER_CSV = os.path.join(PARENT_DIR, "data/final_coordinates.csv")

# --- Output Directory Configuration ---
TIMESTAMP = datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
OUTPUT_DIR = os.path.join(PARENT_DIR, f"results/Enhanced_Hybrid_Model_{TIMESTAMP}")
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
RANSAC_BASE_THRESHOLD = 0.015  # 15mm base threshold - more stringent

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

def multi_stage_ransac_affine(source_points, target_points):
    """Multi-stage RANSAC with progressive refinement for higher accuracy."""
    logging.info("Starting multi-stage RANSAC affine transformation...")
    
    total_points = len(source_points)
    
    # Stage 1: Coarse estimation with larger threshold
    coarse_threshold = RANSAC_BASE_THRESHOLD * 2
    best_T, best_inliers = single_stage_ransac(source_points, target_points, 
                                              coarse_threshold, RANSAC_ITERATIONS // 2)
    
    if best_T is None or len(best_inliers) < total_points * 0.3:
        logging.warning("Coarse RANSAC failed, using single-stage approach")
        return single_stage_ransac(source_points, target_points, RANSAC_BASE_THRESHOLD, RANSAC_ITERATIONS)
    
    # Stage 2: Fine estimation on inliers with tighter threshold
    inlier_source = source_points[best_inliers]
    inlier_target = target_points[best_inliers]
    
    fine_threshold = RANSAC_BASE_THRESHOLD
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
# 6. Data Loading and Processing
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
# 7. Enhanced Evaluation and Visualization
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

# =============================================================================
# 8. Ensemble Model for Enhanced Performance
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
# 9. Main Execution Flow
# =============================================================================

def main():
    """Enhanced main function with comprehensive offline training flow."""
    setup_environment()
    logging.info("Starting enhanced hybrid model offline training process...")

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
        
        # Advanced outlier removal
        source_points, target_points, inlier_mask = advanced_outlier_removal(
            source_points, target_points, method='isolation'
        )
        
        # Split data first (as requested)
        (src_train, src_test, tgt_train, tgt_test) = train_test_split(
            source_points, target_points, train_size=TRAIN_TEST_RATIO, random_state=RANDOM_STATE
        )
        logging.info(f"Data split completed: {len(src_train)} training points, {len(src_test)} test points")

        # --- Step 1: Apply Known Rotation ---
        logging.info("\n" + "="*60)
        logging.info("Step 1: Apply Known Rotation Transform")
        logging.info("="*60)
        
        R_known_2D = R_EULER_3D[0:2, 0:2]
        logging.info(f"2D rotation matrix extracted from 3D matrix:\n{R_known_2D}")
        
        # Verify rotation matrix orthogonality
        orthogonality_check = np.allclose(R_known_2D @ R_known_2D.T, np.eye(2), atol=1e-6)
        det_check = np.allclose(np.linalg.det(R_known_2D), 1.0, atol=1e-6)
        
        if not orthogonality_check or not det_check:
            logging.warning("Warning: Provided rotation matrix may not be a valid orthogonal matrix")
        
        src_train_rotated = (R_known_2D @ src_train.T).T
        logging.info("Rotation transform applied to training set source points")

        # --- Step 2: Multi-stage RANSAC for Residual Affine Transform ---
        logging.info("\n" + "="*60)
        logging.info("Step 2: Multi-stage RANSAC for Residual Affine Transform")
        logging.info("="*60)
        
        T_residual_affine, inlier_indices = multi_stage_ransac_affine(src_train_rotated, tgt_train)
        if T_residual_affine is None:
            logging.error("RANSAC failed to find valid affine transformation, exiting program")
            return
        
        logging.info(f"Calculated residual affine transformation matrix:\n{T_residual_affine}")
        
        # Analyze affine transformation properties
        A = T_residual_affine[:2, :2]
        t = T_residual_affine[:2, 2]
        det_A = np.linalg.det(A)
        U, S, Vt = np.linalg.svd(A)
        
        logging.info(f"Affine transformation analysis:")
        logging.info(f"  Determinant: {det_A:.6f} (area scaling factor)")
        logging.info(f"  Singular values: [{S[0]:.6f}, {S[1]:.6f}] (principal axis scaling)")
        logging.info(f"  Translation vector: [{t[0]:.6f}, {t[1]:.6f}] m")

        # --- Step 3: Calculate Nonlinear Residuals and Prepare Enhanced MLP Data ---
        logging.info("\n" + "="*60)
        logging.info("Step 3: Calculate Nonlinear Residuals and Prepare Enhanced MLP Data")
        logging.info("="*60)
        
        # Apply complete linear transformation
        train_affine_corrected = apply_affine_transform(src_train_rotated, T_residual_affine)
        
        # Calculate final residuals (MLP training target)
        train_final_residuals = tgt_train - train_affine_corrected
        residual_magnitude = np.linalg.norm(train_final_residuals, axis=1)
        
        logging.info(f"Nonlinear residual statistics:")
        logging.info(f"  Mean magnitude: {np.mean(residual_magnitude)*1000:.3f} mm")
        logging.info(f"  Standard deviation: {np.std(residual_magnitude)*1000:.3f} mm")
        logging.info(f"  Maximum: {np.max(residual_magnitude)*1000:.3f} mm")
        
        # Extract comprehensive features
        center_point = np.mean(train_affine_corrected, axis=0)
        train_features = extract_comprehensive_features(train_affine_corrected, center_point)
        
        logging.info(f"Enhanced MLP feature dimension: {train_features.shape[1]}")

        # --- Step 4: Train Enhanced Ensemble MLP Model ---
        logging.info("\n" + "="*60)
        logging.info("Step 4: Train Enhanced Ensemble MLP Model")
        logging.info("="*60)
        
        # Feature standardization
        feature_scaler = StandardScaler()
        train_features_scaled = feature_scaler.fit_transform(train_features)
        
        # Target standardization
        target_scaler = StandardScaler()
        train_residuals_scaled = target_scaler.fit_transform(train_final_residuals)
        
        # Prepare test features for ensemble training
        src_test_rotated = (R_known_2D @ src_test.T).T
        test_affine_corrected = apply_affine_transform(src_test_rotated, T_residual_affine)
        test_features = extract_comprehensive_features(test_affine_corrected, center_point)
        test_features_scaled = feature_scaler.transform(test_features)
        
        # Train ensemble models
        models, ensemble_pred_scaled = train_ensemble_mlp(
            train_features_scaled, train_residuals_scaled, test_features_scaled
        )
        
        # Get the primary MLP model (first in ensemble)
        mlp_model = models[0]
        logging.info(f"Primary MLP model training completed. Final iterations: {mlp_model.n_iter_}")

        # --- Step 5: Comprehensive Test Set Evaluation ---
        logging.info("\n" + "="*60)
        logging.info("Step 5: Comprehensive Test Set Performance Evaluation")
        logging.info("="*60)
        
        results = {}
        
        # Stage A: Only known rotation
        logging.info("Evaluating Stage A: Only known rotation...")
        results['stage_A'] = comprehensive_evaluation(
            "Stage A - Known Rotation Only", src_test, tgt_test, src_test_rotated, "A_rotation_only"
        )

        # Stage B: Rotation + affine transformation
        logging.info("Evaluating Stage B: Rotation + affine transformation...")
        results['stage_B'] = comprehensive_evaluation(
            "Stage B - Rotation + Affine", src_test, tgt_test, test_affine_corrected, "B_affine"
        )

        # Stage C: Complete hybrid model (single MLP)
        logging.info("Evaluating Stage C: Complete hybrid model (single MLP)...")
        single_pred_scaled = mlp_model.predict(test_features_scaled)
        single_pred_residuals = target_scaler.inverse_transform(single_pred_scaled)
        single_final_predicted = test_affine_corrected + single_pred_residuals
        
        results['stage_C'] = comprehensive_evaluation(
            "Stage C - Complete Hybrid Model", src_test, tgt_test, single_final_predicted, "C_single_hybrid"
        )
        
        # Stage D: Enhanced ensemble model
        logging.info("Evaluating Stage D: Enhanced ensemble model...")
        ensemble_pred_residuals = target_scaler.inverse_transform(ensemble_pred_scaled)
        ensemble_final_predicted = test_affine_corrected + ensemble_pred_residuals
        
        results['stage_D'] = comprehensive_evaluation(
            "Stage D - Enhanced Ensemble Model", src_test, tgt_test, ensemble_final_predicted, "D_ensemble_hybrid"
        )

        # --- Step 6: Save All Model Artifacts ---
        logging.info("\n" + "="*60)
        logging.info("Step 6: Save Model Artifacts and Configuration")
        logging.info("="*60)
        
        # Save model parameters
        np.save(os.path.join(MODEL_DIR, "R_known_2D.npy"), R_known_2D)
        np.save(os.path.join(MODEL_DIR, "T_residual_affine.npy"), T_residual_affine)
        np.save(os.path.join(MODEL_DIR, "center_point.npy"), center_point)
        
        # Save scalers
        joblib.dump(feature_scaler, os.path.join(MODEL_DIR, "feature_scaler.pkl"))
        joblib.dump(target_scaler, os.path.join(MODEL_DIR, "target_scaler.pkl"))
        
        # Save all models
        for i, model in enumerate(models):
            joblib.dump(model, os.path.join(MODEL_DIR, f"model_{i}.pkl"))
        
        # Save configuration
        config = {
            'R_EULER_3D': R_EULER_3D.tolist(),
            'MLP_HIDDEN_LAYERS': MLP_HIDDEN_LAYERS,
            'RANSAC_ITERATIONS': RANSAC_ITERATIONS,
            'RANSAC_BASE_THRESHOLD': RANSAC_BASE_THRESHOLD,
            'train_points': len(src_train),
            'test_points': len(src_test),
            'feature_dim': train_features.shape[1],
            'primary_mlp_iterations': int(mlp_model.n_iter_)
        }
        
        import json
        with open(os.path.join(MODEL_DIR, "model_config.json"), "w") as f:
            json.dump(config, f, indent=2)
        
        logging.info(f"All model artifacts saved to: {MODEL_DIR}")

        # --- Final Summary Report ---
        improvement_B_to_C = results['stage_B']['rmse_mm'] - results['stage_C']['rmse_mm']
        improvement_C_to_D = results['stage_C']['rmse_mm'] - results['stage_D']['rmse_mm']
        improvement_A_to_D = results['stage_A']['rmse_mm'] - results['stage_D']['rmse_mm']
        
        summary = f"""
=========================================================
          Enhanced Hybrid Model Training Summary
=========================================================
Data Source: {os.path.basename(LASER_TRACKER_CSV) if os.path.exists(LASER_TRACKER_CSV) else "Simulation Data"}
Total Data Points: {len(source_points)}
Training Points: {len(src_train)}
Test Points: {len(src_test)}

Model Configuration:
---------------------------------------------------------
RANSAC Iterations: {RANSAC_ITERATIONS}
RANSAC Inlier Threshold: {RANSAC_BASE_THRESHOLD*1000:.1f} mm
Primary MLP Structure: {MLP_HIDDEN_LAYERS}
Enhanced Feature Dimension: {train_features.shape[1]}
Primary MLP Training Iterations: {mlp_model.n_iter_}

Test Set Performance Evaluation (RMSE):
---------------------------------------------------------
Stage A (Known Rotation Only):    {results['stage_A']['rmse_mm']:.4f} mm
Stage B (Rotation + Affine):      {results['stage_B']['rmse_mm']:.4f} mm  
Stage C (Single Hybrid Model):    {results['stage_C']['rmse_mm']:.4f} mm
Stage D (Enhanced Ensemble):      {results['stage_D']['rmse_mm']:.4f} mm
---------------------------------------------------------

Performance Improvement Analysis:
---------------------------------------------------------
Stage A to Stage D Improvement:   {improvement_A_to_D:.4f} mm ({improvement_A_to_D/results['stage_A']['rmse_mm']*100:.1f}%)
Stage B to Stage C Improvement:   {improvement_B_to_C:.4f} mm ({improvement_B_to_C/results['stage_B']['rmse_mm']*100:.1f}%)
Stage C to Stage D Improvement:   {improvement_C_to_D:.4f} mm ({improvement_C_to_D/results['stage_C']['rmse_mm']*100:.1f}%)

Detailed Performance Metrics (Stage D - Best Model):
---------------------------------------------------------
RMSE: {results['stage_D']['rmse_mm']:.4f} mm
MAE:  {results['stage_D']['mae_mm']:.4f} mm
Max:  {results['stage_D']['max_error_mm']:.4f} mm
95%:  {results['stage_D']['percentile_95_mm']:.4f} mm
99%:  {results['stage_D']['percentile_99_mm']:.4f} mm
=========================================================

Model Files Location: {MODEL_DIR}
Log Files Location: {os.path.join(LOG_DIR, "enhanced_training_process.log")}
Visualization Files Location: {IMG_DIR}
=========================================================
"""
        
        print(summary)
        logging.info(summary)
        
        with open(os.path.join(OUTPUT_DIR, "comprehensive_summary.txt"), "w", encoding='utf-8') as f:
            f.write(summary)
        
        # Additional recommendations for sub-10mm performance
        if results['stage_D']['rmse_mm'] > 10.0:
            recommendations = f"""
=========================================================
          Recommendations for Sub-10mm Performance
=========================================================
Current Best RMSE: {results['stage_D']['rmse_mm']:.4f} mm

Suggested Improvements:
1. Increase training data size (current: {len(src_train)} points)
2. Fine-tune RANSAC threshold (current: {RANSAC_BASE_THRESHOLD*1000:.1f} mm)
3. Experiment with deeper/wider MLP architectures
4. Add more sophisticated feature engineering
5. Consider spatially-varying transformations
6. Implement cross-validation for hyperparameter tuning
7. Use more advanced ensemble methods (e.g., gradient boosting)
8. Investigate data quality and measurement noise sources
=========================================================
"""
            logging.info(recommendations)
            with open(os.path.join(OUTPUT_DIR, "performance_recommendations.txt"), "w", encoding='utf-8') as f:
                f.write(recommendations)
        
        logging.info(f"Enhanced training process completed successfully! All results saved in: {OUTPUT_DIR}")
        
    except Exception as e:
        logging.error(f"Error occurred during training process: {str(e)}")
        import traceback
        logging.error(f"Detailed error information:\n{traceback.format_exc()}")
        raise


if __name__ == "__main__":
    main()