#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Enhanced Hybrid Model Offline Training Script with Adaptive Quadtree

This script implements a comprehensive comparison of transformation models:
1. Known Rotation + RANSAC Affine + Adaptive Quadtree (New AQ Hybrid)
2. RBF-based Registration
3. CPD-based Registration
4. Enhanced Ensemble Hybrid Model
5. SVD Registration (Baseline)
6. Affine-only Registration (Baseline)
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
OUTPUT_DIR = os.path.join(PARENT_DIR, f"results/AQ_Comparative_Model_{TIMESTAMP}")
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

# RANSAC Parameters
RANSAC_ITERATIONS = 3000
RANSAC_MIN_SAMPLE_SIZE = 8
RANSAC_MAX_SAMPLE_SIZE = 100
RANSAC_BASE_THRESHOLD = 0.015
RANSAC_INLIER_THRESHOLD = 0.04

# Adaptive Quadtree Parameters
QUADTREE_MAX_DEPTH = 10
QUADTREE_MIN_POINTS_PER_CELL = 15
QUADTREE_ERROR_THRESHOLD = 0.004

# Enhanced MLP Parameters
MLP_HIDDEN_LAYERS = (512, 256, 128, 64, 32)
MLP_MAX_ITER = 5000
MLP_EARLY_STOPPING = True
MLP_VALIDATION_FRACTION = 0.15
MLP_LEARNING_RATE = 0.0005
MLP_ALPHA = 0.0001

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
# 2. Adaptive Quadtree Model Implementation (from first code)
# =============================================================================

class QuadtreeNode:
    """Quadtree node for spatially adaptive regression."""
    
    def __init__(self, boundary, points=None):
        """
        Initialize a quadtree node.
        
        Args:
            boundary: tuple (x_min, z_min, width, height)
            points: numpy array of point indices belonging to this node
        """
        self.boundary = boundary  # (x_min, z_min, width, height)
        self.points = points if points is not None else []
        self.children = None  # Will be a list of 4 children if split
        self.model = None  # Local regression model for leaf nodes
        self.is_leaf = True
        
    def subdivide(self, X):
        """Subdivide this node into 4 children and distribute points."""
        x_min, z_min, width, height = self.boundary
        half_width = width / 2
        half_height = height / 2
        
        # Define child boundaries
        child_boundaries = [
            (x_min, z_min, half_width, half_height),  # Bottom-left
            (x_min + half_width, z_min, half_width, half_height),  # Bottom-right
            (x_min, z_min + half_height, half_width, half_height),  # Top-left
            (x_min + half_width, z_min + half_height, half_width, half_height)  # Top-right
        ]
        
        # Create child nodes
        self.children = []
        for boundary in child_boundaries:
            child_points = []
            for point_idx in self.points:
                x, z = X[point_idx]
                if (boundary[0] <= x < boundary[0] + boundary[2] and 
                    boundary[1] <= z < boundary[1] + boundary[3]):
                    child_points.append(point_idx)
            
            child = QuadtreeNode(boundary, child_points)
            self.children.append(child)
        
        self.is_leaf = False
        
    def contains_point(self, point):
        """Check if a point is within this node's boundary."""
        x, z = point
        x_min, z_min, width, height = self.boundary
        return (x_min <= x <= x_min + width and 
                z_min <= z <= z_min + height)

class ConstantModel:
    """Simple constant model for single-point or two-point cases."""
    
    def __init__(self, value):
        self.value = np.array(value)
        
    def predict(self, X):
        return np.tile(self.value, (len(X), 1))

class AdaptiveQuadtreeModel:
    """Adaptive Quadtree-based Local Regression Model for spatially varying distortions."""
    
    def __init__(self, max_depth=8, min_points_per_cell=10, error_threshold=0.005):
        """
        Initialize the Adaptive Quadtree Model.
        
        Args:
            max_depth: Maximum depth of the quadtree
            min_points_per_cell: Minimum points required to split a cell
            error_threshold: RMSE threshold below which splitting stops
        """
        self.max_depth = max_depth
        self.min_points_per_cell = min_points_per_cell
        self.error_threshold = error_threshold
        self.root = None
        self.is_fitted = False
        
    def fit(self, X, y):
        """
        Fit the adaptive quadtree model.
        
        Args:
            X: Input coordinates (N, 2) - corrected points after affine transformation
            y: Target residuals (N, 2) - nonlinear corrections needed
        """
        logging.info(f"Training Adaptive Quadtree Model with {len(X)} points...")
        
        # Store data for tree building
        self.X_train = X.copy()
        self.y_train = y.copy()
        
        # Calculate bounding box
        x_min, z_min = np.min(X, axis=0)
        x_max, z_max = np.max(X, axis=0)
        
        # Add small margin
        margin = 0.01
        x_min -= margin
        z_min -= margin
        width = x_max - x_min + 2 * margin
        height = z_max - z_min + 2 * margin
        
        # Initialize root node with all points
        root_boundary = (x_min, z_min, width, height)
        self.root = QuadtreeNode(root_boundary, list(range(len(X))))
        
        # Build the tree recursively
        self._build_tree(self.root, depth=0)
        
        self.is_fitted = True
        
        # Count nodes for reporting
        total_nodes, leaf_nodes = self._count_nodes(self.root)
        logging.info(f"Quadtree construction completed: {total_nodes} total nodes, {leaf_nodes} leaf nodes")
        
    def _build_tree(self, node, depth):
        """
        Recursively build the quadtree.
        
        Args:
            node: Current QuadtreeNode
            depth: Current depth in the tree
        """
        # Stopping condition 1: Maximum depth reached
        if depth >= self.max_depth:
            self._train_leaf_model(node)
            return
            
        # Stopping condition 2: Too few points
        if len(node.points) <= self.min_points_per_cell:
            self._train_leaf_model(node)
            return
            
        # Check if local model is already good enough
        if len(node.points) >= 3:  # Need at least 3 points for Ridge regression
            X_local = self.X_train[node.points]
            y_local = self.y_train[node.points]
            
            # Train temporary model to check error
            temp_model = Ridge(alpha=1e-6, random_state=RANDOM_STATE)
            temp_model.fit(X_local, y_local)
            y_pred = temp_model.predict(X_local)
            
            rmse = np.sqrt(np.mean(np.sum((y_local - y_pred)**2, axis=1)))
            
            # Stopping condition 3: Error is below threshold
            if rmse < self.error_threshold:
                node.model = temp_model
                return
        
        # Split the node
        node.subdivide(self.X_train)
        
        # Recursively build children
        for child in node.children:
            if len(child.points) > 0:  # Only build non-empty children
                self._build_tree(child, depth + 1)
            else:
                # Empty child becomes a dummy leaf
                child.is_leaf = True
                child.model = None
                
    def _train_leaf_model(self, node):
        """Train a local regression model for a leaf node."""
        if len(node.points) == 0:
            node.model = None
            return
            
        X_local = self.X_train[node.points]
        y_local = self.y_train[node.points]
        
        if len(node.points) == 1:
            # Single point: create a constant model
            node.model = ConstantModel(y_local[0])
        elif len(node.points) == 2:
            # Two points: use simple mean
            node.model = ConstantModel(np.mean(y_local, axis=0))
        else:
            # Multiple points: use Ridge regression
            model = Ridge(alpha=1e-6, random_state=RANDOM_STATE)
            model.fit(X_local, y_local)
            node.model = model
            
    def predict(self, X):
        """
        Predict residual corrections for input points.
        
        Args:
            X: Input coordinates (N, 2)
            
        Returns:
            Predicted residuals (N, 2)
        """
        if not self.is_fitted:
            raise ValueError("Model must be fitted before prediction")
            
        predictions = np.zeros_like(X)
        
        for i, point in enumerate(X):
            leaf_node = self._find_leaf(point, self.root)
            if leaf_node is not None and leaf_node.model is not None:
                if isinstance(leaf_node.model, ConstantModel):
                    predictions[i] = leaf_node.model.predict(point.reshape(1, -1))[0]
                else:
                    predictions[i] = leaf_node.model.predict(point.reshape(1, -1))[0]
            # If no model found, prediction remains zero
            
        return predictions
        
    def _find_leaf(self, point, node):
        """
        Find the leaf node containing the given point.
        
        Args:
            point: 2D coordinates
            node: Current node to search from
            
        Returns:
            Leaf QuadtreeNode containing the point, or None if not found
        """
        if not node.contains_point(point):
            return None
            
        if node.is_leaf:
            return node
            
        # Search children
        if node.children is not None:
            for child in node.children:
                result = self._find_leaf(point, child)
                if result is not None:
                    return result
                    
        return None
        
    def _count_nodes(self, node):
        """Count total and leaf nodes in the tree."""
        if node.is_leaf:
            return 1, 1
            
        total = 1
        leaves = 0
        
        if node.children is not None:
            for child in node.children:
                child_total, child_leaves = self._count_nodes(child)
                total += child_total
                leaves += child_leaves
                
        return total, leaves

# =============================================================================
# 3. Environment Setup
# =============================================================================

def setup_environment():
    """Create all output directories and configure logging."""
    for directory in [OUTPUT_DIR, IMG_DIR, MODEL_DIR, LOG_DIR]:
        os.makedirs(directory, exist_ok=True)

    log_file = os.path.join(LOG_DIR, "aq_comparative_training_process.log")
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
# 4. Data Validation and Preprocessing Functions
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

# =============================================================================
# 5. Transformation Functions
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
# 7. Baseline Registration Methods
# =============================================================================

def run_svd_registration(src_train, tgt_train, src_test, tgt_test):
    """Run SVD-based rigid registration method."""
    logging.info("Running SVD Registration (Rigid Transformation)...")
    
    try:
        # Step 1: Compute centroids
        centroid_src = np.mean(src_train, axis=0)
        centroid_tgt = np.mean(tgt_train, axis=0)
        
        # Step 2: Center the point sets
        src_centered = src_train - centroid_src
        tgt_centered = tgt_train - centroid_tgt
        
        # Step 3: Compute the cross-covariance matrix
        H = np.dot(src_centered.T, tgt_centered)
        
        # Step 4: SVD
        U, S, Vt = np.linalg.svd(H)
        
        # Step 5: Compute rotation matrix
        R = np.dot(Vt.T, U.T)
        
        # Ensure proper rotation (det(R) = 1)
        if np.linalg.det(R) < 0:
            Vt[-1, :] *= -1
            R = np.dot(Vt.T, U.T)
        
        # Step 6: Compute translation
        t = centroid_tgt - np.dot(R, centroid_src)
        
        # Step 7: Apply transformation to training and test data
        train_transformed = (R @ src_train.T).T + t
        test_transformed = (R @ src_test.T).T + t
        
        # Step 8: Calculate metrics
        train_errors = np.sqrt(np.sum((tgt_train - train_transformed)**2, axis=1))
        test_errors = np.sqrt(np.sum((tgt_test - test_transformed)**2, axis=1))
        
        metrics = {
            'train_rmse': np.sqrt(np.mean(train_errors**2)),
            'test_rmse': np.sqrt(np.mean(test_errors**2)),
            'train_mae': np.mean(train_errors),
            'test_mae': np.mean(test_errors),
            'max_error': np.max(test_errors),
            'percentile_95': np.percentile(test_errors, 95),
            'percentile_99': np.percentile(test_errors, 99)
        }
        
        logging.info(f"SVD registration completed. Train RMSE: {metrics['train_rmse']*1000:.3f}mm, Test RMSE: {metrics['test_rmse']*1000:.3f}mm")
        
        return metrics, test_transformed, (src_train, tgt_train, train_transformed)
        
    except Exception as e:
        logging.error(f"SVD registration failed: {e}")
        return {
            'train_rmse': float('inf'),
            'test_rmse': float('inf'),
            'train_mae': float('inf'),
            'test_mae': float('inf'),
            'max_error': float('inf'),
            'percentile_95': float('inf'),
            'percentile_99': float('inf')
        }, src_test, (src_train, tgt_train, src_train)

def run_affine_only_registration(src_train, tgt_train, src_test, tgt_test):
    """Run Affine-only registration method."""
    logging.info("Running Affine-only Registration...")
    
    try:
        # Find affine transformation using training data
        T_affine = find_affine_transform_robust(src_train, tgt_train)
        
        if T_affine is None:
            logging.error("Failed to compute affine transformation")
            return {
                'train_rmse': float('inf'),
                'test_rmse': float('inf'),
                'train_mae': float('inf'),
                'test_mae': float('inf'),
                'max_error': float('inf'),
                'percentile_95': float('inf'),
                'percentile_99': float('inf')
            }, src_test, (src_train, tgt_train, src_train)
        
        # Apply affine transformation
        train_transformed = apply_affine_transform(src_train, T_affine)
        test_transformed = apply_affine_transform(src_test, T_affine)
        
        # Calculate metrics
        train_errors = np.sqrt(np.sum((tgt_train - train_transformed)**2, axis=1))
        test_errors = np.sqrt(np.sum((tgt_test - test_transformed)**2, axis=1))
        
        metrics = {
            'train_rmse': np.sqrt(np.mean(train_errors**2)),
            'test_rmse': np.sqrt(np.mean(test_errors**2)),
            'train_mae': np.mean(train_errors),
            'test_mae': np.mean(test_errors),
            'max_error': np.max(test_errors),
            'percentile_95': np.percentile(test_errors, 95),
            'percentile_99': np.percentile(test_errors, 99)
        }
        
        logging.info(f"Affine-only registration completed. Train RMSE: {metrics['train_rmse']*1000:.3f}mm, Test RMSE: {metrics['test_rmse']*1000:.3f}mm")
        
        return metrics, test_transformed, (src_train, tgt_train, train_transformed)
        
    except Exception as e:
        logging.error(f"Affine-only registration failed: {e}")
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
# 8. Adaptive Quadtree Hybrid Registration
# =============================================================================

def run_adaptive_quadtree_hybrid(src_train, tgt_train, src_test, tgt_test):
    """Run Known Rotation + RANSAC Affine + Adaptive Quadtree hybrid method."""
    logging.info("Running Adaptive Quadtree Hybrid Registration...")
    
    try:
        # Step 1: Apply known rotation
        R_known_2D = R_EULER_3D[0:2, 0:2]
        src_train_rotated = (R_known_2D @ src_train.T).T
        src_test_rotated = (R_known_2D @ src_test.T).T
        
        # Step 2: RANSAC affine transformation
        T_residual_affine, inlier_indices = multi_stage_ransac_affine(src_train_rotated, tgt_train)
        
        if T_residual_affine is None:
            logging.error("RANSAC affine transformation failed")
            return {
                'train_rmse': float('inf'),
                'test_rmse': float('inf'),
                'train_mae': float('inf'),
                'test_mae': float('inf'),
                'max_error': float('inf'),
                'percentile_95': float('inf'),
                'percentile_99': float('inf')
            }, src_test, (src_train, tgt_train, src_train)
        
        # Step 3: Apply affine transformation
        train_affine_corrected = apply_affine_transform(src_train_rotated, T_residual_affine)
        test_affine_corrected = apply_affine_transform(src_test_rotated, T_residual_affine)
        
        # Step 4: Calculate residuals for quadtree training
        train_final_residuals = tgt_train - train_affine_corrected
        
        # Step 5: Train Adaptive Quadtree model
        quadtree_model = AdaptiveQuadtreeModel(
            max_depth=QUADTREE_MAX_DEPTH,
            min_points_per_cell=QUADTREE_MIN_POINTS_PER_CELL,
            error_threshold=QUADTREE_ERROR_THRESHOLD
        )
        
        quadtree_model.fit(train_affine_corrected, train_final_residuals)
        
        # Step 6: Make predictions
        train_predicted_residuals = quadtree_model.predict(train_affine_corrected)
        train_final_predicted = train_affine_corrected + train_predicted_residuals
        
        test_predicted_residuals = quadtree_model.predict(test_affine_corrected)
        test_final_predicted = test_affine_corrected + test_predicted_residuals
        
        # Step 7: Calculate metrics
        train_errors = np.sqrt(np.sum((tgt_train - train_final_predicted)**2, axis=1))
        test_errors = np.sqrt(np.sum((tgt_test - test_final_predicted)**2, axis=1))
        
        metrics = {
            'train_rmse': np.sqrt(np.mean(train_errors**2)),
            'test_rmse': np.sqrt(np.mean(test_errors**2)),
            'train_mae': np.mean(train_errors),
            'test_mae': np.mean(test_errors),
            'max_error': np.max(test_errors),
            'percentile_95': np.percentile(test_errors, 95),
            'percentile_99': np.percentile(test_errors, 99)
        }
        
        # Save model artifacts
        joblib.dump(quadtree_model, os.path.join(MODEL_DIR, "adaptive_quadtree_model.pkl"))
        np.save(os.path.join(MODEL_DIR, "R_known_2D_aq.npy"), R_known_2D)
        np.save(os.path.join(MODEL_DIR, "T_residual_affine_aq.npy"), T_residual_affine)
        
        logging.info(f"Adaptive Quadtree hybrid completed. Train RMSE: {metrics['train_rmse']*1000:.3f}mm, Test RMSE: {metrics['test_rmse']*1000:.3f}mm")
        
        return metrics, test_final_predicted, (src_train, tgt_train, train_final_predicted)
        
    except Exception as e:
        logging.error(f"Adaptive Quadtree hybrid registration failed: {e}")
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
# 9. RBF and CPD Registration Methods (same as before)
# =============================================================================

class OptimizedRBFSystem:
    """Highly optimized RBF system with advanced overfitting prevention"""
    
    def __init__(self):
        self.best_params = None
        self.best_score = float('inf')
        self.rbf_model = None
        self.control_points = None
        self.control_residuals = None
        
    def optimize_hyperparameters(self, points, residuals, n_folds=3):
        """Advanced hyperparameter optimization with extensive search"""
        logging.info("Starting RBF hyperparameter optimization...")
        
        # Enhanced parameter grid based on extensive research
        param_grid = {
            'kernel': ['thin_plate_spline', 'multiquadric', 'gaussian'],
            'smoothing': [0.001, 0.01, 0.05],
            'num_control_points': [50, 100, 150],
            'selection_method': ['kmeans', 'uniform']
        }
        
        # Data-adaptive parameter ranges
        data_scale = np.std(residuals)
        n_points = len(points)
        
        # Adapt control points based on dataset size
        max_control_points = min(150, n_points // 3)
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
        else:  # uniform
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
            max_iterations = min(50, max(20, n_points // 10))

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
            max_iterations = 30
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
# 10. Enhanced Evaluation and Visualization (NO Chinese text)
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

    # Create detailed visualizations with English labels only
    fig, ((ax1, ax2), (ax3, ax4)) = plt.subplots(2, 2, figsize=(20, 16))
    
    # 1. Scatter plot comparison
    ax1.scatter(target[:, 0], target[:, 1], c='red', s=12, alpha=0.7, label='Target Points')
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
    """Create comprehensive residual vector visualization with English labels only."""
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
    """Create comparative visualizations for all methods with English labels only."""
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
    train_rmse_vals = [all_results[m]['test']['train_rmse'] * 1000 if 'test' in all_results[m] and all_results[m]['test']['train_rmse'] != float('inf') else 0 for m in methods]
    test_rmse_vals = [all_results[m]['test']['test_rmse'] * 1000 if 'test' in all_results[m] and all_results[m]['test']['test_rmse'] != float('inf') else 0 for m in methods]
    train_mae_vals = [all_results[m]['test']['train_mae'] * 1000 if 'test' in all_results[m] and all_results[m]['test']['train_mae'] != float('inf') else 0 for m in methods]
    test_mae_vals = [all_results[m]['test']['test_mae'] * 1000 if 'test' in all_results[m] and all_results[m]['test']['test_mae'] != float('inf') else 0 for m in methods]
    
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
            heatmap_data[i, 0] = metrics['train_rmse'] * 1000 if metrics['train_rmse'] != float('inf') else np.nan
            heatmap_data[i, 1] = metrics['test_rmse'] * 1000 if metrics['test_rmse'] != float('inf') else np.nan
            heatmap_data[i, 2] = metrics['train_mae'] * 1000 if metrics['train_mae'] != float('inf') else np.nan
            heatmap_data[i, 3] = metrics['test_mae'] * 1000 if metrics['test_mae'] != float('inf') else np.nan
            heatmap_data[i, 4] = metrics['max_error'] * 1000 if metrics['max_error'] != float('inf') else np.nan
            heatmap_data[i, 5] = metrics['percentile_95'] * 1000 if metrics['percentile_95'] != float('inf') else np.nan
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
# 11. Main Execution Flow
# =============================================================================

def main():
    """Enhanced main function with Adaptive Quadtree integration and comprehensive method comparison."""
    setup_environment()
    logging.info("Starting Adaptive Quadtree comparative model offline training process...")

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

        # --- Method 1: SVD Registration (Baseline) ---
        logging.info("\n" + "="*60)
        logging.info("Method 1: SVD Registration (Rigid Baseline)")
        logging.info("="*60)
        
        svd_metrics, svd_test_result, svd_train_data = run_svd_registration(src_train, tgt_train, src_test, tgt_test)
        all_results['SVD Registration'] = {
            'test': svd_metrics,
            'result': svd_test_result,
            'train_data': svd_train_data
        }
        
        # Evaluate SVD method
        svd_eval = comprehensive_evaluation("SVD Registration", src_test, tgt_test, svd_test_result, "1_SVD")
        all_results['SVD Registration']['detailed'] = svd_eval
        
        # Create residual vector plot for SVD
        create_residual_vector_plot("SVD Registration", src_test, tgt_test, svd_test_result, 
                                  "1_SVD", svd_train_data)

        # --- Method 2: Affine-only Registration (Baseline) ---
        logging.info("\n" + "="*60)
        logging.info("Method 2: Affine-only Registration (Baseline)")
        logging.info("="*60)
        
        affine_metrics, affine_test_result, affine_train_data = run_affine_only_registration(src_train, tgt_train, src_test, tgt_test)
        all_results['Affine Registration'] = {
            'test': affine_metrics,
            'result': affine_test_result,
            'train_data': affine_train_data
        }
        
        # Evaluate Affine method
        affine_eval = comprehensive_evaluation("Affine Registration", src_test, tgt_test, affine_test_result, "2_Affine")
        all_results['Affine Registration']['detailed'] = affine_eval
        
        # Create residual vector plot for Affine
        create_residual_vector_plot("Affine Registration", src_test, tgt_test, affine_test_result, 
                                  "2_Affine", affine_train_data)

        # --- Method 3: Adaptive Quadtree Hybrid (NEW) ---
        logging.info("\n" + "="*60)
        logging.info("Method 3: Adaptive Quadtree Hybrid Registration")
        logging.info("="*60)
        
        aq_metrics, aq_test_result, aq_train_data = run_adaptive_quadtree_hybrid(src_train, tgt_train, src_test, tgt_test)
        all_results['Adaptive Quadtree Hybrid'] = {
            'test': aq_metrics,
            'result': aq_test_result,
            'train_data': aq_train_data
        }
        
        # Evaluate Adaptive Quadtree method
        aq_eval = comprehensive_evaluation("Adaptive Quadtree Hybrid", src_test, tgt_test, aq_test_result, "3_AQ_Hybrid")
        all_results['Adaptive Quadtree Hybrid']['detailed'] = aq_eval
        
        # Create residual vector plot for Adaptive Quadtree
        create_residual_vector_plot("Adaptive Quadtree Hybrid", src_test, tgt_test, aq_test_result, 
                                  "3_AQ_Hybrid", aq_train_data)

        # --- Method 4: RBF Registration ---
        logging.info("\n" + "="*60)
        logging.info("Method 4: RBF Registration")
        logging.info("="*60)
        
        rbf_metrics, rbf_test_result, rbf_train_data = run_rbf_registration(src_train, tgt_train, src_test, tgt_test)
        all_results['RBF Registration'] = {
            'test': rbf_metrics,
            'result': rbf_test_result,
            'train_data': rbf_train_data
        }
        
        # Evaluate RBF method
        rbf_eval = comprehensive_evaluation("RBF Registration", src_test, tgt_test, rbf_test_result, "4_RBF")
        all_results['RBF Registration']['detailed'] = rbf_eval
        
        # Create residual vector plot for RBF
        create_residual_vector_plot("RBF Registration", src_test, tgt_test, rbf_test_result, 
                                  "4_RBF", rbf_train_data)

        # --- Method 5: CPD Registration ---
        logging.info("\n" + "="*60)
        logging.info("Method 5: CPD Registration")
        logging.info("="*60)
        
        cpd_metrics, cpd_test_result, cpd_train_data = run_cpd_registration(src_train, tgt_train, src_test, tgt_test)
        all_results['CPD Registration'] = {
            'test': cpd_metrics,
            'result': cpd_test_result,
            'train_data': cpd_train_data
        }
        
        # Evaluate CPD method
        cpd_eval = comprehensive_evaluation("CPD Registration", src_test, tgt_test, cpd_test_result, "5_CPD")
        all_results['CPD Registration']['detailed'] = cpd_eval
        
        # Create residual vector plot for CPD
        create_residual_vector_plot("CPD Registration", src_test, tgt_test, cpd_test_result, 
                                  "5_CPD", cpd_train_data)

        # --- Step 6: Create Comparative Visualizations ---
        logging.info("\n" + "="*60)
        logging.info("Step 6: Create Comparative Visualizations")
        logging.info("="*60)
        
        create_comparative_visualizations(all_results, src_test, tgt_test)

        # --- Final Summary Report ---
        logging.info("\n" + "="*60)
        logging.info("ADAPTIVE QUADTREE COMPARATIVE ANALYSIS SUMMARY")
        logging.info("="*60)
        
        # Find best method
        best_method = None
        best_rmse = float('inf')
        
        print("\nComparative Results Summary:")
        print("="*90)
        print(f"{'Method':<30} {'Train RMSE (mm)':<15} {'Test RMSE (mm)':<15} {'Train MAE (mm)':<15} {'Test MAE (mm)':<15}")
        print("-"*90)
        
        for method_name, results in all_results.items():
            if 'test' in results:
                metrics = results['test']
                train_rmse_mm = metrics['train_rmse'] * 1000 if metrics['train_rmse'] != float('inf') else float('inf')
                test_rmse_mm = metrics['test_rmse'] * 1000 if metrics['test_rmse'] != float('inf') else float('inf')
                train_mae_mm = metrics['train_mae'] * 1000 if metrics['train_mae'] != float('inf') else float('inf')
                test_mae_mm = metrics['test_mae'] * 1000 if metrics['test_mae'] != float('inf') else float('inf')
                
                print(f"{method_name:<30} {train_rmse_mm:<15.3f} {test_rmse_mm:<15.3f} {train_mae_mm:<15.3f} {test_mae_mm:<15.3f}")
                
                if test_rmse_mm < best_rmse:
                    best_rmse = test_rmse_mm
                    best_method = method_name
        
        print("="*90)
        print(f"Best Method: {best_method}")
        print(f"Best Test RMSE: {best_rmse:.3f} mm")
        
        # Save configuration
        config = {
            'R_EULER_3D': R_EULER_3D.tolist(),
            'QUADTREE_MAX_DEPTH': QUADTREE_MAX_DEPTH,
            'QUADTREE_MIN_POINTS_PER_CELL': QUADTREE_MIN_POINTS_PER_CELL,
            'QUADTREE_ERROR_THRESHOLD': QUADTREE_ERROR_THRESHOLD,
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
        with open(os.path.join(MODEL_DIR, "aq_comparative_model_config.json"), "w") as f:
            json.dump(config, f, indent=2)
        
        logging.info(f"All model artifacts saved to: {MODEL_DIR}")

        # Save detailed summary
        summary = f"""
=========================================================
     Adaptive Quadtree Comparative Model Training Summary
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
Log Files Location: {os.path.join(LOG_DIR, "aq_comparative_training_process.log")}
Visualization Files Location: {IMG_DIR}
=========================================================
"""
        
        print(summary)
        logging.info(summary)
        
        with open(os.path.join(OUTPUT_DIR, "aq_comparative_summary.txt"), "w", encoding='utf-8') as f:
            f.write(summary)
        
        logging.info(f"Adaptive Quadtree comparative training process completed successfully! All results saved in: {OUTPUT_DIR}")
        
    except Exception as e:
        logging.error(f"Error occurred during training process: {str(e)}")
        import traceback
        logging.error(f"Detailed error information:\n{traceback.format_exc()}")
        raise


if __name__ == "__main__":
    main()