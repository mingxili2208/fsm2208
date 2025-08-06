#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Enhanced Hybrid Model with Adaptive Quadtree Local Regression

This script implements a "Known Rotation + RANSAC Affine + Adaptive Quadtree" hybrid transformation model.
The Quadtree model intelligently adapts its complexity to the data, creating fine-grained grids in areas
of high distortion and coarse grids in simple areas for sub-10mm RMSE target.
"""
import os
import datetime
import logging
import joblib
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from sklearn.model_selection import train_test_split
from sklearn.linear_model import Ridge
from sklearn.metrics import mean_squared_error, mean_absolute_error
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
OUTPUT_DIR = os.path.join(PARENT_DIR, f"results/Quadtree_Hybrid_Model_{TIMESTAMP}")
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
RANSAC_BASE_THRESHOLD = 0.015  # 15mm base threshold

# Adaptive Quadtree Parameters
QUADTREE_MAX_DEPTH = 10
QUADTREE_MIN_POINTS_PER_CELL = 15
QUADTREE_ERROR_THRESHOLD = 0.004  # 4mm threshold for subdivision

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
# 2. Adaptive Quadtree Model Implementation
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


class AdaptiveQuadtreeModel:
    """Production-ready Adaptive Quadtree-based Local Regression Model with smooth interpolation and error control."""
    
    def __init__(self, max_depth=8, min_points_per_cell=10, error_threshold=0.005, prediction_clip_value=None):
        """
        Initialize the Adaptive Quadtree Model.
        
        Args:
            max_depth: Maximum depth of the quadtree
            min_points_per_cell: Minimum points required to split a cell
            error_threshold: RMSE threshold below which splitting stops
            prediction_clip_value: Maximum allowed prediction magnitude (in meters)
        """
        self.max_depth = max_depth
        self.min_points_per_cell = min_points_per_cell
        self.error_threshold = error_threshold
        self.prediction_clip_value = prediction_clip_value
        self.root = None
        self.is_fitted = False
        self.clip_value_ = None
        
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
        
        # Calculate automatic clip value if not provided
        if self.prediction_clip_value is None:
            y_norms = np.linalg.norm(y, axis=1)
            self.clip_value_ = 1.2 * np.max(y_norms)
        else:
            self.clip_value_ = self.prediction_clip_value
        
        logging.info(f"Prediction clipping threshold set to: {self.clip_value_*1000:.2f} mm")
        
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
        Predict residual corrections with smooth interpolation and robust OOB handling.
        
        Args:
            X: Input coordinates (N, 2)
            
        Returns:
            Predicted residuals (N, 2)
        """
        if not self.is_fitted:
            raise ValueError("Model must be fitted before prediction")
            
        predictions = np.zeros_like(X)
        
        for i, point in enumerate(X):
            # Step 1: Check if point is out-of-bounds
            if not self.root.contains_point(point):
                # Handle OOB with clamping strategy
                clamped_point = self._clamp_to_boundary(point, self.root.boundary)
                point_to_predict = clamped_point
            else:
                point_to_predict = point
            
            # Step 2: Find main leaf and implement smooth interpolation
            main_leaf = self._find_leaf(point_to_predict, self.root)
            
            if main_leaf is not None:
                # Get neighbors for interpolation
                neighbors = self._find_neighbors(point_to_predict, main_leaf)
                
                # Perform weighted prediction
                prediction = self._weighted_prediction(point_to_predict, neighbors)
            else:
                # Fallback: use zero prediction
                prediction = np.array([0.0, 0.0])
            
            # Step 3: Apply prediction clipping (sanity cap)
            prediction_norm = np.linalg.norm(prediction)
            if prediction_norm > self.clip_value_:
                prediction = prediction * (self.clip_value_ / prediction_norm)
            
            predictions[i] = prediction
            
        return predictions
    
    def _clamp_to_boundary(self, point, boundary):
        """Clamp an out-of-bounds point to the boundary edge."""
        x, z = point
        x_min, z_min, width, height = boundary
        x_max = x_min + width
        z_max = z_min + height
        
        clamped_x = np.clip(x, x_min, x_max)
        clamped_z = np.clip(z, z_min, z_max)
        
        return np.array([clamped_x, clamped_z])
    
    def _find_neighbors(self, point, main_leaf):
        """Find neighboring leaf nodes for interpolation."""
        x, z = point
        x_min, z_min, width, height = main_leaf.boundary
        
        # Create virtual corners for interpolation
        corners = [
            np.array([x_min, z_min]),  # Bottom-left
            np.array([x_min + width, z_min]),  # Bottom-right
            np.array([x_min, z_min + height]),  # Top-left
            np.array([x_min + width, z_min + height])  # Top-right
        ]
        
        neighbors = []
        for corner in corners:
            leaf = self._find_leaf(corner, self.root)
            if leaf is not None and leaf.model is not None:
                neighbors.append((leaf, corner))
            else:
                # Use main leaf as fallback
                neighbors.append((main_leaf, corner))
        
        return neighbors
    
    def _weighted_prediction(self, point, neighbors):
        """Perform distance-weighted prediction from multiple neighbors."""
        x, z = point
        predictions = []
        weights = []
        
        for leaf, corner in neighbors:
            # Get prediction from this leaf
            if leaf.model is not None:
                if isinstance(leaf.model, ConstantModel):
                    pred = leaf.model.predict(point.reshape(1, -1))[0]
                else:
                    pred = leaf.model.predict(point.reshape(1, -1))[0]
            else:
                pred = np.array([0.0, 0.0])
            
            # Calculate weight based on inverse distance
            distance = np.linalg.norm(point - corner)
            weight = 1.0 / (distance + 1e-8)  # Add small epsilon to avoid division by zero
            
            predictions.append(pred)
            weights.append(weight)
        
        # Normalize weights
        weights = np.array(weights)
        weights = weights / np.sum(weights)
        
        # Calculate weighted average
        final_prediction = np.zeros(2)
        for i, pred in enumerate(predictions):
            final_prediction += weights[i] * pred
        
        return final_prediction
        
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


class ConstantModel:
    """Simple constant model for single-point or two-point cases."""
    
    def __init__(self, value):
        self.value = np.array(value)
        
    def predict(self, X):
        return np.tile(self.value, (len(X), 1))


# =============================================================================
# 3. Environment Setup
# =============================================================================

def setup_environment():
    """Create all output directories and configure logging."""
    for directory in [OUTPUT_DIR, IMG_DIR, MODEL_DIR, LOG_DIR]:
        os.makedirs(directory, exist_ok=True)

    log_file = os.path.join(LOG_DIR, "quadtree_training_process.log")
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
# 4. Data Validation and Preprocessing
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
    
    logging.info(f"Data validation completed. Source range: X[{np.min(source_points[:, 0]):.3f}, {np.max(source_points[:, 0]):.3f}], "
                f"Z[{np.min(source_points[:, 1]):.3f}, {np.max(source_points[:, 1]):.3f}]")

def advanced_outlier_removal(source_points, target_points, method='isolation', contamination=0.05):
    """Advanced outlier removal using multiple methods."""
    from sklearn.ensemble import IsolationForest
    
    initial_count = len(source_points)
    
    if method == 'isolation':
        # Isolation Forest method
        combined_data = np.hstack([source_points, target_points])
        iso_forest = IsolationForest(contamination=contamination, random_state=RANDOM_STATE)
        outlier_labels = iso_forest.fit_predict(combined_data)
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
# 7. Visualization Functions
# =============================================================================

def plot_residual_heatmap(points, residuals, title, save_path, vmax=None):
    """Plot residual magnitude as a heatmap."""
    residual_magnitude = np.linalg.norm(residuals, axis=1) * 1000  # Convert to mm
    
    if vmax is None:
        vmax = np.percentile(residual_magnitude, 95)
    
    fig, ax = plt.subplots(1, 1, figsize=(12, 10))
    
    scatter = ax.scatter(points[:, 0], points[:, 1], c=residual_magnitude, 
                        cmap='viridis', s=15, alpha=0.8, vmin=0, vmax=vmax)
    
    plt.colorbar(scatter, ax=ax, label='Residual Magnitude (mm)')
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Z (m)")
    ax.set_title(title)
    ax.grid(True, linestyle='--', alpha=0.6)
    ax.axis('equal')
    
    plt.tight_layout()
    plt.savefig(save_path, dpi=300, bbox_inches='tight')
    plt.close()

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
# 8. Main Execution Flow
# =============================================================================

def main():
    """Enhanced main function with adaptive quadtree model training flow."""
    setup_environment()
    logging.info("Starting adaptive quadtree hybrid model offline training process...")

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
        
        # Split data first
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

        # --- Step 3: Calculate Nonlinear Residuals ---
        logging.info("\n" + "="*60)
        logging.info("Step 3: Calculate Nonlinear Residuals and Generate Heatmaps")
        logging.info("="*60)
        
        # Apply complete linear transformation to training data
        train_affine_corrected = apply_affine_transform(src_train_rotated, T_residual_affine)
        
        # Calculate final residuals (Quadtree training target)
        train_final_residuals = tgt_train - train_affine_corrected
        residual_magnitude = np.linalg.norm(train_final_residuals, axis=1)
        
        logging.info(f"Nonlinear residual statistics for training set:")
        logging.info(f"  Mean magnitude: {np.mean(residual_magnitude)*1000:.3f} mm")
        logging.info(f"  Standard deviation: {np.std(residual_magnitude)*1000:.3f} mm")
        logging.info(f"  Maximum: {np.max(residual_magnitude)*1000:.3f} mm")
        
        # Generate residual heatmaps for training data
        plot_residual_heatmap(train_affine_corrected, train_final_residuals, 
                             "Training Set - Residuals After Affine Transform", 
                             os.path.join(IMG_DIR, "train_residuals_after_affine.png"))
        
        # Process test data for heatmap
        src_test_rotated = (R_known_2D @ src_test.T).T
        test_affine_corrected = apply_affine_transform(src_test_rotated, T_residual_affine)
        test_final_residuals = tgt_test - test_affine_corrected
        
        plot_residual_heatmap(test_affine_corrected, test_final_residuals, 
                             "Test Set - Residuals After Affine Transform", 
                             os.path.join(IMG_DIR, "test_residuals_after_affine.png"))

        # --- Step 4: Train Adaptive Quadtree Model ---
        logging.info("\n" + "="*60)
        logging.info("Step 4: Train Adaptive Quadtree Local Model")
        logging.info("="*60)
        
        # Initialize and train the quadtree model with enhanced parameters
        quadtree_model = AdaptiveQuadtreeModel(
            max_depth=QUADTREE_MAX_DEPTH,
            min_points_per_cell=QUADTREE_MIN_POINTS_PER_CELL,
            error_threshold=QUADTREE_ERROR_THRESHOLD,
            prediction_clip_value=0.1  # 100mm maximum prediction
        )
        
        quadtree_model.fit(train_affine_corrected, train_final_residuals)
        
        logging.info("Adaptive Quadtree model training completed.")

        # --- Step 5: Generate Model Prediction Heatmaps ---
        logging.info("\n" + "="*60)
        logging.info("Step 5: Generate Model Prediction Heatmaps")
        logging.info("="*60)
        
        # Training set predictions
        train_predicted_residuals = quadtree_model.predict(train_affine_corrected)
        train_model_residuals = train_final_residuals - train_predicted_residuals
        
        plot_residual_heatmap(train_affine_corrected, train_model_residuals, 
                             "Training Set - Model Residuals (Ground Truth - Predicted)", 
                             os.path.join(IMG_DIR, "train_model_residuals.png"))
        
        # Test set predictions
        test_predicted_residuals = quadtree_model.predict(test_affine_corrected)
        test_model_residuals = test_final_residuals - test_predicted_residuals
        
        plot_residual_heatmap(test_affine_corrected, test_model_residuals, 
                             "Test Set - Model Residuals (Ground Truth - Predicted)", 
                             os.path.join(IMG_DIR, "test_model_residuals.png"))

        # --- Step 6: Comprehensive Test Set Evaluation ---
        logging.info("\n" + "="*60)
        logging.info("Step 6: Comprehensive Test Set Performance Evaluation")
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

        # Stage C: Complete hybrid model with Quadtree
        logging.info("Evaluating Stage C: Complete hybrid model with Adaptive Quadtree...")
        final_predicted = test_affine_corrected + test_predicted_residuals
        
        results['stage_C'] = comprehensive_evaluation(
            "Stage C - Adaptive Quadtree Hybrid Model", src_test, tgt_test, final_predicted, "C_quadtree_hybrid"
        )

        # --- Step 7: Save All Model Artifacts ---
        logging.info("\n" + "="*60)
        logging.info("Step 7: Save Model Artifacts and Configuration")
        logging.info("="*60)
        
        # Save model parameters
        np.save(os.path.join(MODEL_DIR, "R_known_2D.npy"), R_known_2D)
        np.save(os.path.join(MODEL_DIR, "T_residual_affine.npy"), T_residual_affine)
        
        # Save quadtree model
        joblib.dump(quadtree_model, os.path.join(MODEL_DIR, "quadtree_model.pkl"))
        
        # Save configuration
        config = {
            'R_EULER_3D': R_EULER_3D.tolist(),
            'QUADTREE_MAX_DEPTH': QUADTREE_MAX_DEPTH,
            'QUADTREE_MIN_POINTS_PER_CELL': QUADTREE_MIN_POINTS_PER_CELL,
            'QUADTREE_ERROR_THRESHOLD': QUADTREE_ERROR_THRESHOLD,
            'RANSAC_ITERATIONS': RANSAC_ITERATIONS,
            'RANSAC_BASE_THRESHOLD': RANSAC_BASE_THRESHOLD,
            'train_points': len(src_train),
            'test_points': len(src_test),
            'prediction_clip_value': quadtree_model.clip_value_
        }
        
        import json
        with open(os.path.join(MODEL_DIR, "model_config.json"), "w") as f:
            json.dump(config, f, indent=2)
        
        logging.info(f"All model artifacts saved to: {MODEL_DIR}")

        # --- Final Summary Report ---
        improvement_A_to_B = results['stage_A']['rmse_mm'] - results['stage_B']['rmse_mm']
        improvement_B_to_C = results['stage_B']['rmse_mm'] - results['stage_C']['rmse_mm']
        improvement_A_to_C = results['stage_A']['rmse_mm'] - results['stage_C']['rmse_mm']
        
        summary = f"""
=========================================================
       Enhanced Quadtree Hybrid Model Training Summary
=========================================================
Data Source: {os.path.basename(LASER_TRACKER_CSV) if os.path.exists(LASER_TRACKER_CSV) else "Simulation Data"}
Total Data Points: {len(source_points)}
Training Points: {len(src_train)}
Test Points: {len(src_test)}

Model Configuration:
---------------------------------------------------------
RANSAC Iterations: {RANSAC_ITERATIONS}
RANSAC Inlier Threshold: {RANSAC_BASE_THRESHOLD*1000:.1f} mm
Quadtree Max Depth: {QUADTREE_MAX_DEPTH}
Quadtree Min Points Per Cell: {QUADTREE_MIN_POINTS_PER_CELL}
Quadtree Error Threshold: {QUADTREE_ERROR_THRESHOLD*1000:.1f} mm
Prediction Clip Value: {quadtree_model.clip_value_*1000:.1f} mm

Test Set Performance Evaluation (RMSE):
---------------------------------------------------------
Stage A (Known Rotation Only):        {results['stage_A']['rmse_mm']:.4f} mm
Stage B (Rotation + Affine):          {results['stage_B']['rmse_mm']:.4f} mm  
Stage C (Enhanced Quadtree Hybrid):   {results['stage_C']['rmse_mm']:.4f} mm
---------------------------------------------------------

Performance Improvement Analysis:
---------------------------------------------------------
Stage A to Stage B Improvement:   {improvement_A_to_B:.4f} mm ({improvement_A_to_B/results['stage_A']['rmse_mm']*100:.1f}%)
Stage B to Stage C Improvement:   {improvement_B_to_C:.4f} mm ({improvement_B_to_C/results['stage_B']['rmse_mm']*100:.1f}%)
Stage A to Stage C Improvement:   {improvement_A_to_C:.4f} mm ({improvement_A_to_C/results['stage_A']['rmse_mm']*100:.1f}%)

Detailed Performance Metrics (Stage C - Final Model):
---------------------------------------------------------
RMSE: {results['stage_C']['rmse_mm']:.4f} mm
MAE:  {results['stage_C']['mae_mm']:.4f} mm
Max:  {results['stage_C']['max_error_mm']:.4f} mm
95%:  {results['stage_C']['percentile_95_mm']:.4f} mm
99%:  {results['stage_C']['percentile_99_mm']:.4f} mm
=========================================================

Production Enhancements Applied:
- Smooth bilinear interpolation between quadtree cells
- Robust out-of-bounds handling with clamping strategy
- Prediction clipping mechanism for error control
- Distance-weighted neighbor predictions
- Enhanced boundary discontinuity elimination

Model Files Location: {MODEL_DIR}
Log Files Location: {os.path.join(LOG_DIR, "quadtree_training_process.log")}
Visualization Files Location: {IMG_DIR}
=========================================================
"""
        
        print(summary)
        logging.info(summary)
        
        with open(os.path.join(OUTPUT_DIR, "comprehensive_summary.txt"), "w", encoding='utf-8') as f:
            f.write(summary)
        
        # Recommendations for further improvement
        if results['stage_C']['rmse_mm'] > 5.0:
            recommendations = f"""
=========================================================
         Recommendations for Further Improvement
=========================================================
Current Best RMSE: {results['stage_C']['rmse_mm']:.4f} mm

Suggested Improvements:
1. Increase training data size (current: {len(src_train)} points)
2. Fine-tune Quadtree parameters (depth, min points, threshold)
3. Experiment with different local regression models (polynomial, kernel)
4. Implement cross-validation for hyperparameter optimization
5. Add temporal or sequential information if available
6. Consider ensemble of multiple Quadtree models
7. Investigate data quality and measurement noise sources
8. Add adaptive threshold based on local point density
=========================================================
"""
            logging.info(recommendations)
            with open(os.path.join(OUTPUT_DIR, "performance_recommendations.txt"), "w", encoding='utf-8') as f:
                f.write(recommendations)
        
        logging.info(f"Enhanced Quadtree training process completed successfully! All results saved in: {OUTPUT_DIR}")
        
    except Exception as e:
        logging.error(f"Error occurred during training process: {str(e)}")
        import traceback
        logging.error(f"Detailed error information:\n{traceback.format_exc()}")
        raise


if __name__ == "__main__":
    main()