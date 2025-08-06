#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Enhanced Comparative Registration Framework with Advanced Methods

This script implements a comprehensive comparison of transformation models:
1. SVD Rigid Transform (Baseline)
2. Hybrid (Affine + MLP) - formerly "Original Hybrid"
3. Hybrid (Affine + Ensemble) - formerly "Enhanced Ensemble Hybrid"
4. RBF-based Registration
5. CPD-based Registration
6. Adaptive Quadtree Hybrid Model (NEW)

Key Features:
- Enhanced RBF parameter logging for analysis
- Comprehensive comparative evaluation
- Advanced feature engineering for sub-10mm RMSE target
- Improved error handling and stability
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
OUTPUT_DIR = os.path.join(PARENT_DIR, f"results/Enhanced_Comparative_Framework_{TIMESTAMP}")
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
RANSAC_BASE_THRESHOLD = 0.02  # 20mm base threshold
RANSAC_INLIER_THRESHOLD = 0.04  # 40mm threshold for initial inlier filtering

# Enhanced MLP Parameters for sub-10mm target
MLP_HIDDEN_LAYERS = (512, 256, 128, 64, 32)
MLP_MAX_ITER = 5000
MLP_EARLY_STOPPING = True
MLP_VALIDATION_FRACTION = 0.15
MLP_LEARNING_RATE = 0.0005
MLP_ALPHA = 0.0001  # L2 regularization

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
# 2. Environment Setup
# =============================================================================

def setup_environment():
    """Create all output directories and configure logging."""
    for directory in [OUTPUT_DIR, IMG_DIR, MODEL_DIR, LOG_DIR]:
        os.makedirs(directory, exist_ok=True)

    log_file = os.path.join(LOG_DIR, "enhanced_comparative_training.log")
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
# 3. Adaptive Quadtree Model Implementation
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
    """Production-ready Adaptive Quadtree-based Local Regression Model with smooth interpolation."""
    
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
        Predict residual corrections with smooth interpolation.
        
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
# 6. SVD Rigid Transform Method
# =============================================================================

def run_svd_rigid_registration(src_train, tgt_train, src_test, tgt_test):
    """Run SVD-based rigid registration method (rotation + translation only)."""
    logging.info("Running SVD Rigid Transform registration method...")
    
    try:
        # Step 1: Calculate centroids
        centroid_src = np.mean(src_train, axis=0)
        centroid_tgt = np.mean(tgt_train, axis=0)
        
        # Step 2: Center the point sets
        src_centered = src_train - centroid_src
        tgt_centered = tgt_train - centroid_tgt
        
        # Step 3: Compute cross-covariance matrix
        H = np.dot(src_centered.T, tgt_centered)
        
        # Step 4: SVD decomposition
        U, S, Vt = np.linalg.svd(H)
        
        # Step 5: Compute rotation matrix
        R = np.dot(Vt.T, U.T)
        
        # Ensure proper rotation (det(R) = 1)
        if np.linalg.det(R) < 0:
            Vt[-1, :] *= -1
            R = np.dot(Vt.T, U.T)
        
        # Step 6: Compute translation vector
        t = centroid_tgt - np.dot(R, centroid_src)
        
        # Step 7: Apply transformation to training and test sets
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
        
        logging.info(f"SVD Rigid registration completed. Train RMSE: {metrics['train_rmse']*1000:.3f}mm, Test RMSE: {metrics['test_rmse']*1000:.3f}mm")
        
        return metrics, test_transformed, (src_train, tgt_train, train_transformed)
        
    except Exception as e:
        logging.error(f"SVD Rigid registration failed: {e}")
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
# 7. Enhanced RBF Registration with Better Logging
# =============================================================================

class OptimizedRBFSystem:
    """Highly optimized RBF system with enhanced parameter logging"""
    
    def __init__(self):
        self.best_params = None
        self.best_score = float('inf')
        self.rbf_model = None
        self.control_points = None
        self.control_residuals = None
        
    def optimize_hyperparameters(self, points, residuals, n_folds=5):
        """Advanced hyperparameter optimization with enhanced logging."""
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
                
                # Enhanced logging: log parameter combinations that achieve new best scores
                if avg_cv_score < best_score:
                    best_score = avg_cv_score
                    best_params = params.copy()
                    logging.info(f"NEW BEST RBF Parameters found with CV score: {avg_cv_score:.6f}")
                    logging.info(f"  Kernel: {params['kernel']}")
                    logging.info(f"  Smoothing: {params['smoothing']}")
                    logging.info(f"  Control Points: {params['num_control_points']}")
                    logging.info(f"  Selection Method: {params['selection_method']}")
                    
            except Exception as e:
                continue
        
        self.best_params = best_params
        self.best_score = best_score
        
        # Final logging of best parameters
        if best_params is not None:
            logging.info(f"RBF optimization completed. Final best parameters:")
            logging.info(f"  Final Best CV Score: {best_score:.6f}")
            logging.info(f"  Final Best Kernel: {best_params['kernel']}")
            logging.info(f"  Final Best Smoothing: {best_params['smoothing']}")
            logging.info(f"  Final Best Control Points: {best_params['num_control_points']}")
            logging.info(f"  Final Best Selection Method: {best_params['selection_method']}")
        else:
            logging.warning("RBF optimization failed to find any valid parameters")
        
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

# =============================================================================
# 8. CPD Registration Method
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
# 9. Advanced Feature Engineering
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
# 11. Data Loading and Processing
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
# 12. Enhanced Evaluation and Visualization
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
# 13. Main Execution Flow
# =============================================================================

def main():
    """Enhanced main function with comprehensive offline training flow and method comparison."""
    setup_environment()
    logging.info("Starting enhanced comparative registration framework training process...")

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

        # --- Method 1: SVD Rigid Transform (NEW) ---
        logging.info("\n" + "="*60)
        logging.info("Method 1: SVD Rigid Transform (Baseline)")
        logging.info("="*60)
        
        svd_metrics, svd_test_result, svd_train_data = run_svd_rigid_registration(src_train, tgt_train, src_test, tgt_test)
        all_results['SVD Rigid Transform'] = {
            'test': svd_metrics,
            'result': svd_test_result,
            'train_data': svd_train_data
        }
        
        # Evaluate SVD method
        svd_eval = comprehensive_evaluation("SVD Rigid Transform", src_test, tgt_test, svd_test_result, "1_SVD_Rigid")
        all_results['SVD Rigid Transform']['detailed'] = svd_eval

        # --- Method 2: RBF Registration ---
        logging.info("\n" + "="*60)
        logging.info("Method 2: RBF Registration")
        logging.info("="*60)
        
        rbf_metrics, rbf_test_result, rbf_train_data = run_rbf_registration(src_train, tgt_train, src_test, tgt_test)
        all_results['RBF Registration'] = {
            'test': rbf_metrics,
            'result': rbf_test_result,
            'train_data': rbf_train_data
        }
        
        # Evaluate RBF method
        rbf_eval = comprehensive_evaluation("RBF Registration", src_test, tgt_test, rbf_test_result, "2_RBF")
        all_results['RBF Registration']['detailed'] = rbf_eval

        # --- Method 3: CPD Registration ---
        logging.info("\n" + "="*60)
        logging.info("Method 3: CPD Registration")
        logging.info("="*60)
        
        cpd_metrics, cpd_test_result, cpd_train_data = run_cpd_registration(src_train, tgt_train, src_test, tgt_test)
        all_results['CPD Registration'] = {
            'test': cpd_metrics,
            'result': cpd_test_result,
            'train_data': cpd_train_data
        }
        
        # Evaluate CPD method
        cpd_eval = comprehensive_evaluation("CPD Registration", src_test, tgt_test, cpd_test_result, "3_CPD")
        all_results['CPD Registration']['detailed'] = cpd_eval

        # --- Method 4: Hybrid (Affine + MLP) - formerly "Original Hybrid" ---
        logging.info("\n" + "="*60)
        logging.info("Method 4: Hybrid (Affine + MLP)")
        logging.info("="*60)
        
        # Apply known rotation and affine transformation
        R_known_2D = R_EULER_3D[0:2, 0:2]
        src_train_rotated = (R_known_2D @ src_train.T).T
        src_test_rotated = (R_known_2D @ src_test.T).T
        
        T_residual_affine, inlier_indices = multi_stage_ransac_affine(src_train_rotated, tgt_train)
        
        if T_residual_affine is not None:
            # Apply affine to both train and test
            train_affine_corrected = apply_affine_transform(src_train_rotated, T_residual_affine)
            test_affine_corrected = apply_affine_transform(src_test_rotated, T_residual_affine)
            
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
            
            hybrid_mlp_metrics = {
                'train_rmse': np.sqrt(np.mean(train_errors**2)),
                'test_rmse': np.sqrt(np.mean(test_errors**2)),
                'train_mae': np.mean(train_errors),
                'test_mae': np.mean(test_errors),
                'max_error': np.max(test_errors),
                'percentile_95': np.percentile(test_errors, 95),
                'percentile_99': np.percentile(test_errors, 99)
            }
            
            all_results['Hybrid (Affine + MLP)'] = {
                'test': hybrid_mlp_metrics,
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
            
            logging.info(f"Hybrid (Affine + MLP) completed. Train RMSE: {hybrid_mlp_metrics['train_rmse']*1000:.3f}mm, Test RMSE: {hybrid_mlp_metrics['test_rmse']*1000:.3f}mm")
            
            # Evaluate Hybrid (Affine + MLP) method
            hybrid_mlp_eval = comprehensive_evaluation("Hybrid (Affine + MLP)", src_test, tgt_test, test_final_predicted, "4_Hybrid_Affine_MLP")
            all_results['Hybrid (Affine + MLP)']['detailed'] = hybrid_mlp_eval
        else:
            all_results['Hybrid (Affine + MLP)'] = {
                'test': {'train_rmse': float('inf'), 'test_rmse': float('inf'), 'train_mae': float('inf'), 'test_mae': float('inf'),
                        'max_error': float('inf'), 'percentile_95': float('inf'), 'percentile_99': float('inf')},
                'result': src_test,
                'train_data': (src_train, tgt_train, src_train)
            }

        # --- Method 5: Hybrid (Affine + Ensemble) - formerly "Enhanced Ensemble Hybrid" ---
        logging.info("\n" + "="*60)
        logging.info("Method 5: Hybrid (Affine + Ensemble)")
        logging.info("="*60)
        
        if T_residual_affine is not None:
            # Use same preprocessing as hybrid MLP
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
            
            hybrid_ensemble_metrics = {
                'train_rmse': np.sqrt(np.mean(train_errors**2)),
                'test_rmse': np.sqrt(np.mean(test_errors**2)),
                'train_mae': np.mean(train_errors),
                'test_mae': np.mean(test_errors),
                'max_error': np.max(test_errors),
                'percentile_95': np.percentile(test_errors, 95),
                'percentile_99': np.percentile(test_errors, 99)
            }
            
            all_results['Hybrid (Affine + Ensemble)'] = {
                'test': hybrid_ensemble_metrics,
                'result': ensemble_final_predicted,
                'train_data': (src_train, tgt_train, ensemble_train_final)
            }
            
            # Save ensemble models
            for i, model in enumerate(models):
                joblib.dump(model, os.path.join(MODEL_DIR, f"ensemble_model_{i}.pkl"))
            
            logging.info(f"Hybrid (Affine + Ensemble) completed. Train RMSE: {hybrid_ensemble_metrics['train_rmse']*1000:.3f}mm, Test RMSE: {hybrid_ensemble_metrics['test_rmse']*1000:.3f}mm")
            
            # Evaluate Hybrid (Affine + Ensemble) method
            hybrid_ensemble_eval = comprehensive_evaluation("Hybrid (Affine + Ensemble)", src_test, tgt_test, ensemble_final_predicted, "5_Hybrid_Affine_Ensemble")
            all_results['Hybrid (Affine + Ensemble)']['detailed'] = hybrid_ensemble_eval
        else:
            all_results['Hybrid (Affine + Ensemble)'] = {
                'test': {'train_rmse': float('inf'), 'test_rmse': float('inf'), 'train_mae': float('inf'), 'test_mae': float('inf'),
                        'max_error': float('inf'), 'percentile_95': float('inf'), 'percentile_99': float('inf')},
                'result': src_test,
                'train_data': (src_train, tgt_train, src_train)
            }

        # --- Method 6: Adaptive Quadtree Hybrid Model (NEW) ---
        logging.info("\n" + "="*60)
        logging.info("Method 6: Adaptive Quadtree Hybrid Model")
        logging.info("="*60)
        
        if T_residual_affine is not None:
            # Use same affine-corrected points as base
            quadtree_model = AdaptiveQuadtreeModel(
                max_depth=QUADTREE_MAX_DEPTH,
                min_points_per_cell=QUADTREE_MIN_POINTS_PER_CELL,
                error_threshold=QUADTREE_ERROR_THRESHOLD,
                prediction_clip_value=0.1  # 100mm maximum prediction
            )
            
            # Train on residuals after affine correction
            quadtree_model.fit(train_affine_corrected, train_final_residuals)
            
            # Training evaluation
            train_quad_residuals = quadtree_model.predict(train_affine_corrected)
            train_quad_final = train_affine_corrected + train_quad_residuals
            
            # Test evaluation
            test_quad_residuals = quadtree_model.predict(test_affine_corrected)
            test_quad_final = test_affine_corrected + test_quad_residuals
            
            # Calculate metrics
            train_errors = np.sqrt(np.sum((tgt_train - train_quad_final)**2, axis=1))
            test_errors = np.sqrt(np.sum((tgt_test - test_quad_final)**2, axis=1))
            
            quadtree_metrics = {
                'train_rmse': np.sqrt(np.mean(train_errors**2)),
                'test_rmse': np.sqrt(np.mean(test_errors**2)),
                'train_mae': np.mean(train_errors),
                'test_mae': np.mean(test_errors),
                'max_error': np.max(test_errors),
                'percentile_95': np.percentile(test_errors, 95),
                'percentile_99': np.percentile(test_errors, 99)
            }
            
            all_results['Adaptive Quadtree Hybrid'] = {
                'test': quadtree_metrics,
                'result': test_quad_final,
                'train_data': (src_train, tgt_train, train_quad_final)
            }
            
            # Save quadtree model
            joblib.dump(quadtree_model, os.path.join(MODEL_DIR, "quadtree_model.pkl"))
            
            logging.info(f"Adaptive Quadtree Hybrid completed. Train RMSE: {quadtree_metrics['train_rmse']*1000:.3f}mm, Test RMSE: {quadtree_metrics['test_rmse']*1000:.3f}mm")
            
            # Evaluate Adaptive Quadtree method
            quadtree_eval = comprehensive_evaluation("Adaptive Quadtree Hybrid", src_test, tgt_test, test_quad_final, "6_Adaptive_Quadtree")
            all_results['Adaptive Quadtree Hybrid']['detailed'] = quadtree_eval
        else:
            all_results['Adaptive Quadtree Hybrid'] = {
                'test': {'train_rmse': float('inf'), 'test_rmse': float('inf'), 'train_mae': float('inf'), 'test_mae': float('inf'),
                        'max_error': float('inf'), 'percentile_95': float('inf'), 'percentile_99': float('inf')},
                'result': src_test,
                'train_data': (src_train, tgt_train, src_train)
            }

        # --- Step 7: Create Comparative Visualizations ---
        logging.info("\n" + "="*60)
        logging.info("Step 7: Create Comparative Visualizations")
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
        print("="*100)
        print(f"{'Method':<30} {'Train RMSE (mm)':<15} {'Test RMSE (mm)':<15} {'Train MAE (mm)':<15} {'Test MAE (mm)':<15}")
        print("-"*100)
        
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
        
        print("="*100)
        print(f"Best Method: {best_method}")
        print(f"Best Test RMSE: {best_rmse:.3f} mm")
        
        # Save configuration
        config = {
            'R_EULER_3D': R_EULER_3D.tolist(),
            'MLP_HIDDEN_LAYERS': MLP_HIDDEN_LAYERS,
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
        with open(os.path.join(MODEL_DIR, "comparative_framework_config.json"), "w") as f:
            json.dump(config, f, indent=2)
        
        logging.info(f"All model artifacts saved to: {MODEL_DIR}")

        # Save detailed summary
        summary = f"""
=========================================================
   Enhanced Comparative Registration Framework Summary
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
Log Files Location: {os.path.join(LOG_DIR, "enhanced_comparative_training.log")}
Visualization Files Location: {IMG_DIR}
=========================================================
"""
        
        print(summary)
        logging.info(summary)
        
        with open(os.path.join(OUTPUT_DIR, "comparative_framework_summary.txt"), "w", encoding='utf-8') as f:
            f.write(summary)
        
        logging.info(f"Enhanced comparative training process completed successfully! All results saved in: {OUTPUT_DIR}")
        
    except Exception as e:
        logging.error(f"Error occurred during training process: {str(e)}")
        import traceback
        logging.error(f"Detailed error information:\n{traceback.format_exc()}")
        raise


if __name__ == "__main__":
    main()