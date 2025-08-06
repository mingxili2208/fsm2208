#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
CPD Parameter Optimization Framework - BROADCASTING FIX
Fixed broadcasting issues and improved test data transformation
Flow: SVD Pre-alignment -> CPD Training -> Test Transformation
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
from scipy.spatial.distance import cdist
import random
from itertools import product
import json
import logging

# Import CPD with detailed error handling
CPD_AVAILABLE = False
CPD_ERROR_MSG = ""
cpd_classes = {}

try:
    import pycpd
    
    # Check available classes
    available_classes = dir(pycpd)
    print(f"Available pycpd classes: {[attr for attr in available_classes if not attr.startswith('_')]}")
    
    # Try to import specific classes
    if hasattr(pycpd, 'DeformableRegistration'):
        cpd_classes['deformable'] = pycpd.DeformableRegistration
        print("✓ DeformableRegistration imported")
    
    if hasattr(pycpd, 'RigidRegistration'):
        cpd_classes['rigid'] = pycpd.RigidRegistration
        print("✓ RigidRegistration imported")
    
    if hasattr(pycpd, 'AffineRegistration'):
        cpd_classes['affine'] = pycpd.AffineRegistration
        print("✓ AffineRegistration imported")
    
    if cpd_classes:
        CPD_AVAILABLE = True
        print(f"CPD successfully imported with {len(cpd_classes)} registration types")
    else:
        raise ImportError("No CPD registration classes found")
                
except ImportError as e:
    CPD_ERROR_MSG = f"ImportError: {e}"
    print(f"Warning: pycpd import failed - {CPD_ERROR_MSG}")
except Exception as e:
    CPD_ERROR_MSG = f"Other error: {e}"
    print(f"Warning: pycpd loading failed - {CPD_ERROR_MSG}")

# Create timestamp for output directory
current_time = datetime.datetime.now()
timestamp = current_time.strftime("%Y-%m-%d_%H-%M-%S")

# Parameters
SCALE_FACTOR = 28
TARGET_RMSE = 0.004  # 4mm
TRAIN_TEST_RATIO = 0.8

# Data column names
TRACKER_X = "tracker_x"
TRACKER_Y = "tracker_z"
LASER_X = "laser_x"
LASER_Y = "laser_z"

# Create output directory
script_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(script_dir)
result_dir = os.path.join(parent_dir, "results")
LASER_TRACKER_CSV = os.path.join(parent_dir, "data/final_coordinates.csv")
csv_filename = os.path.basename(LASER_TRACKER_CSV)
output_dir = os.path.join(result_dir, f"CPD_Parameter_Optimization_BROADCAST_FIX_{timestamp}_{csv_filename}")
os.makedirs(output_dir, exist_ok=True)

# Create subdirectories
log_dir = os.path.join(output_dir, "log")
data_dir = os.path.join(output_dir, "data")
result_dir = os.path.join(output_dir, "results")
img_dir = os.path.join(output_dir, "img")

for directory in [log_dir, data_dir, result_dir, img_dir]:
    os.makedirs(directory, exist_ok=True)

# Setup logging
log_file = os.path.join(log_dir, "cpd_optimization_broadcast_fix.log")
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

class CPDParameterOptimizer:
    """
    CPD Parameter Optimization Framework - Broadcasting Fix
    
    WORKFLOW:
    1. SVD Pre-alignment: Rough alignment using SVD transformation
    2. CPD Training: Train CPD model on SVD-aligned training data
    3. Test Transformation: Apply learned transformations to test data
    """
    
    def __init__(self, train_A, train_B, test_A, test_B):
        self.train_A = np.array(train_A)
        self.train_B = np.array(train_B)
        self.test_A = np.array(test_A)
        self.test_B = np.array(test_B)
        
        # Results storage
        self.optimization_results = []
        self.best_config = None
        self.best_rmse = float('inf')
        
        logger.info(f"CPD Optimizer initialized with {len(train_A)} training and {len(test_A)} test points")
        logger.info("WORKFLOW: SVD Pre-alignment -> CPD Training -> Test Transformation")
    
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
        
        return R, t
    
    def apply_2d_transformation(self, points, R, t):
        """Apply 2D transformation to points"""
        points = np.array(points)
        points_xz = points[:, [0, 2]]
        
        # Apply transformation
        transformed_xz = np.dot(points_xz, R.T) + t
        
        # Reconstruct 3D points (keep Y=0)
        transformed = np.zeros_like(points)
        transformed[:, [0, 2]] = transformed_xz
        transformed[:, 1] = 0
        
        return transformed
    
    def calculate_rmse(self, predicted, actual):
        """Calculate RMSE metrics"""
        predicted = np.array(predicted)
        actual = np.array(actual)
        
        # RMSE calculations (XZ plane only since Y=0)
        lateral_errors = actual[:, 0] - predicted[:, 0]
        longitudinal_errors = actual[:, 2] - predicted[:, 2]
        
        overall_rmse = np.sqrt(np.mean(lateral_errors**2 + longitudinal_errors**2))
        lateral_rmse = np.sqrt(np.mean(lateral_errors**2))
        longitudinal_rmse = np.sqrt(np.mean(longitudinal_errors**2))
        
        return {
            'overall_rmse': overall_rmse,
            'lateral_rmse': lateral_rmse,
            'longitudinal_rmse': longitudinal_rmse
        }
    
    def get_cpd_parameter_configurations(self):
        """Define parameter configurations to test"""
        
        # Only test available registration types
        available_types = list(cpd_classes.keys())
        
        if not available_types:
            logger.error("No CPD registration types available")
            return {'quick': {}}
        
        # Quick test configurations (reduced search space)
        quick_configs = {
            'registration_type': available_types,
            'max_iterations': [50, 100],
            'tolerance': [1e-3, 1e-4],
            'w': [0.1, 0.3]
        }
        
        # Add deformable-specific parameters only if deformable is available
        if 'deformable' in available_types:
            quick_configs['alpha'] = [1.0, 2.0]
            quick_configs['beta'] = [1.0, 2.0]
        else:
            quick_configs['alpha'] = [1.0]  # dummy value
            quick_configs['beta'] = [1.0]   # dummy value
        
        # Basic test configurations
        basic_configs = {
            'registration_type': available_types,
            'max_iterations': [30, 50, 100],
            'tolerance': [1e-3, 1e-4, 1e-5],
            'w': [0.1, 0.2, 0.3]
        }
        
        if 'deformable' in available_types:
            basic_configs['alpha'] = [1.0, 2.0, 5.0]
            basic_configs['beta'] = [1.0, 2.0, 5.0]
        else:
            basic_configs['alpha'] = [1.0]
            basic_configs['beta'] = [1.0]
        
        # Comprehensive test configurations
        comprehensive_configs = {
            'registration_type': available_types,
            'max_iterations': [30, 50, 100, 150, 200],
            'tolerance': [1e-2, 1e-3, 1e-4, 1e-5],
            'w': [0.05, 0.1, 0.2, 0.3, 0.4, 0.5]
        }
        
        if 'deformable' in available_types:
            comprehensive_configs['alpha'] = [0.5, 1.0, 2.0, 5.0, 10.0]
            comprehensive_configs['beta'] = [0.5, 1.0, 2.0, 5.0, 10.0]
        else:
            comprehensive_configs['alpha'] = [1.0]
            comprehensive_configs['beta'] = [1.0]
        
        return {
            'basic': basic_configs,
            'quick': quick_configs,
            'comprehensive': comprehensive_configs
        }
    
    def transform_test_data_deformable(self, reg, test_points_xz):
        """Transform test data for deformable registration - FIXED"""
        try:
            # For deformable registration, we need to handle the transformation differently
            # The deformable registration learns a non-linear transformation
            
            # Method 1: Use the transformation matrix if available
            if hasattr(reg, 'transform_point_cloud'):
                try:
                    transformed_points, _ = reg.transform_point_cloud(test_points_xz)
                    return transformed_points
                except Exception as e:
                    logger.warning(f"transform_point_cloud failed: {e}")
            
            # Method 2: Use interpolation based on learned deformation
            if hasattr(reg, 'TY') and hasattr(reg, 'Y'):
                try:
                    # Get the deformation from training
                    original_train_points = reg.Y  # Original training source points
                    transformed_train_points = reg.TY  # Transformed training source points
                    deformation = transformed_train_points - original_train_points
                    
                    # Use nearest neighbor interpolation to apply deformation to test points
                    from scipy.spatial import cKDTree
                    
                    # Build KD-tree for fast nearest neighbor search
                    tree = cKDTree(original_train_points)
                    
                    # Find nearest neighbors for each test point
                    distances, indices = tree.query(test_points_xz, k=3)  # Use 3 nearest neighbors
                    
                    # Interpolate deformation using inverse distance weighting
                    weights = 1.0 / (distances + 1e-8)  # Add small epsilon to avoid division by zero
                    weights = weights / np.sum(weights, axis=1, keepdims=True)  # Normalize weights
                    
                    # Apply weighted deformation
                    interpolated_deformation = np.sum(
                        deformation[indices] * weights[:, :, np.newaxis], axis=1
                    )
                    
                    # Apply deformation to test points
                    transformed_test_points = test_points_xz + interpolated_deformation
                    
                    return transformed_test_points
                    
                except Exception as e:
                    logger.warning(f"Interpolation method failed: {e}")
            
            # Method 3: Fallback - use rigid transformation approximation
            if hasattr(reg, 'R') and hasattr(reg, 't'):
                try:
                    transformed_points = np.dot(test_points_xz, reg.R.T) + reg.t
                    return transformed_points
                except Exception as e:
                    logger.warning(f"Rigid fallback failed: {e}")
            
            # Method 4: Last resort - return original points
            logger.warning("All transformation methods failed, returning original test points")
            return test_points_xz
            
        except Exception as e:
            logger.error(f"Test data transformation failed completely: {e}")
            return test_points_xz
    
    def transform_test_data_rigid_affine(self, reg, test_points_xz):
        """Transform test data for rigid/affine registration - FIXED"""
        try:
            # Method 1: Use direct transformation if available
            if hasattr(reg, 'transform_point_cloud'):
                try:
                    transformed_points, _ = reg.transform_point_cloud(test_points_xz)
                    return transformed_points
                except Exception as e:
                    logger.warning(f"transform_point_cloud failed: {e}")
            
            # Method 2: Use transformation parameters directly
            if hasattr(reg, 'R') and hasattr(reg, 't'):
                try:
                    # Apply rigid/affine transformation: Y_new = Y * R.T + t
                    transformed_points = np.dot(test_points_xz, reg.R.T) + reg.t
                    return transformed_points
                except Exception as e:
                    logger.warning(f"Direct transformation failed: {e}")
            
            # Method 3: Use scale parameter if available (for affine)
            if hasattr(reg, 's') and hasattr(reg, 'R') and hasattr(reg, 't'):
                try:
                    # Apply affine transformation: Y_new = s * Y * R.T + t
                    transformed_points = reg.s * np.dot(test_points_xz, reg.R.T) + reg.t
                    return transformed_points
                except Exception as e:
                    logger.warning(f"Affine transformation failed: {e}")
            
            # Fallback: return original points
            logger.warning("No transformation method available, returning original test points")
            return test_points_xz
            
        except Exception as e:
            logger.error(f"Test data transformation failed: {e}")
            return test_points_xz
    
    def test_cpd_configuration(self, config):
        """Test a single CPD configuration - BROADCASTING FIXED"""
        start_time = time.time()
        
        try:
            logger.info(f"STEP 1: SVD Pre-alignment")
            # Apply SVD pre-alignment
            R_svd, t_svd = self.compute_2d_svd_transformation(self.train_A, self.train_B)
            train_A_aligned = self.apply_2d_transformation(self.train_A, R_svd, t_svd)
            
            # Extract XZ coordinates for CPD
            train_A_aligned_xz = train_A_aligned[:, [0, 2]]
            train_B_xz = self.train_B[:, [0, 2]]
            
            logger.info(f"Training data shapes: Source={train_A_aligned_xz.shape}, Target={train_B_xz.shape}")
            
            # Get registration type and corresponding class
            reg_type = config['registration_type']
            
            if reg_type not in cpd_classes:
                raise ValueError(f"Registration type '{reg_type}' not available")
            
            RegistrationClass = cpd_classes[reg_type]
            
            # Prepare parameters based on registration type
            reg_params = {
                'X': train_B_xz,  # target points
                'Y': train_A_aligned_xz,  # source points
                'max_iterations': config['max_iterations'],
                'tolerance': config['tolerance'],
                'w': config.get('w', 0.1)
            }
            
            # Add deformable-specific parameters
            if reg_type == 'deformable':
                reg_params['alpha'] = config.get('alpha', 2.0)
                reg_params['beta'] = config.get('beta', 2.0)
            
            logger.info(f"STEP 2: CPD {reg_type} registration training")
            logger.info(f"Parameters: max_iter={reg_params['max_iterations']}, tol={reg_params['tolerance']}, w={reg_params['w']}")
            if reg_type == 'deformable':
                logger.info(f"Deformable params: alpha={reg_params['alpha']}, beta={reg_params['beta']}")
            
            # Create and run registration
            reg = RegistrationClass(**reg_params)
            
            # Perform registration
            reg.register(callback=None)
            
            logger.info(f"STEP 3: Training evaluation")
            # Get transformed points from training
            if hasattr(reg, 'TY'):
                transformed_train_A_xz = reg.TY
            else:
                # Fallback: use original aligned points
                transformed_train_A_xz = train_A_aligned_xz
                logger.warning(f"No TY attribute found for {reg_type}, using aligned points")
            
            # Reconstruct 3D training points
            transformed_train_A_final = np.zeros_like(self.train_A)
            transformed_train_A_final[:, [0, 2]] = transformed_train_A_xz
            transformed_train_A_final[:, 1] = 0
            
            train_metrics = self.calculate_rmse(transformed_train_A_final, self.train_B)
            
            logger.info(f"STEP 4: Test data transformation")
            # Testing evaluation - FIXED BROADCASTING ISSUE
            test_A_aligned = self.apply_2d_transformation(self.test_A, R_svd, t_svd)
            test_A_aligned_xz = test_A_aligned[:, [0, 2]]
            
            logger.info(f"Test data shapes: Source={test_A_aligned_xz.shape}, Expected output shape: ({test_A_aligned_xz.shape[0]}, 2)")
            
            # Transform test points using the trained model - REGISTRATION TYPE SPECIFIC
            if reg_type == 'deformable':
                transformed_test_A_xz = self.transform_test_data_deformable(reg, test_A_aligned_xz)
            else:  # rigid or affine
                transformed_test_A_xz = self.transform_test_data_rigid_affine(reg, test_A_aligned_xz)
            
            logger.info(f"Transformed test data shape: {transformed_test_A_xz.shape}")
            
            # Ensure correct shape
            if transformed_test_A_xz.shape != test_A_aligned_xz.shape:
                logger.warning(f"Shape mismatch: expected {test_A_aligned_xz.shape}, got {transformed_test_A_xz.shape}")
                # Try to fix shape issues
                if transformed_test_A_xz.ndim == 1 and len(transformed_test_A_xz) == test_A_aligned_xz.shape[0] * 2:
                    transformed_test_A_xz = transformed_test_A_xz.reshape(-1, 2)
                elif transformed_test_A_xz.shape[0] != test_A_aligned_xz.shape[0]:
                    # Fallback to original points if shape cannot be fixed
                    logger.error(f"Cannot fix shape mismatch, using original aligned points")
                    transformed_test_A_xz = test_A_aligned_xz
            
            # Reconstruct 3D test points
            transformed_test_A_final = np.zeros_like(self.test_A)
            transformed_test_A_final[:, [0, 2]] = transformed_test_A_xz
            transformed_test_A_final[:, 1] = 0
            
            logger.info(f"STEP 5: Test evaluation")
            test_metrics = self.calculate_rmse(transformed_test_A_final, self.test_B)
            
            computation_time = time.time() - start_time
            
            # Collect results
            result = {
                'config': config.copy(),
                'train_rmse': train_metrics['overall_rmse'],
                'test_rmse': test_metrics['overall_rmse'],
                'lateral_rmse': test_metrics['lateral_rmse'],
                'longitudinal_rmse': test_metrics['longitudinal_rmse'],
                'computation_time': computation_time,
                'success': True,
                'error_message': None
            }
            
            # Update best configuration
            if test_metrics['overall_rmse'] < self.best_rmse:
                self.best_rmse = test_metrics['overall_rmse']
                self.best_config = config.copy()
            
            logger.info(f"✓ {reg_type} completed: Train={train_metrics['overall_rmse']*1000:.3f}mm, Test={test_metrics['overall_rmse']*1000:.3f}mm, Time={computation_time:.3f}s")
            
            return result
            
        except Exception as e:
            computation_time = time.time() - start_time
            logger.warning(f"✗ {config.get('registration_type', 'unknown')} failed: {e}")
            
            return {
                'config': config.copy(),
                'train_rmse': float('inf'),
                'test_rmse': float('inf'),
                'lateral_rmse': float('inf'),
                'longitudinal_rmse': float('inf'),
                'computation_time': computation_time,
                'success': False,
                'error_message': str(e)
            }
    
    def run_parameter_optimization(self, test_mode='quick'):
        """Run parameter optimization with specified test mode"""
        if not CPD_AVAILABLE:
            logger.error(f"CPD not available: {CPD_ERROR_MSG}")
            return None
        
        if not cpd_classes:
            logger.error("No CPD registration classes available")
            return None
        
        logger.info(f"=== Starting CPD Parameter Optimization ({test_mode} mode) ===")
        logger.info(f"Available registration types: {list(cpd_classes.keys())}")
        logger.info("WORKFLOW: SVD Pre-alignment -> CPD Training -> Test Transformation")
        
        # Get parameter configurations
        all_configs = self.get_cpd_parameter_configurations()
        configs_to_test = all_configs.get(test_mode, all_configs['quick'])
        
        if not configs_to_test:
            logger.error("No configurations to test")
            return None
        
        # Generate all parameter combinations
        param_names = list(configs_to_test.keys())
        param_values = list(configs_to_test.values())
        
        # Create all combinations
        param_combinations = list(product(*param_values))
        
        # Filter combinations to only include relevant parameters for each registration type
        filtered_combinations = []
        for combo in param_combinations:
            config = dict(zip(param_names, combo))
            
            # For non-deformable types, ignore alpha and beta
            if config['registration_type'] != 'deformable':
                if 'alpha' in config:
                    config['alpha'] = None
                if 'beta' in config:
                    config['beta'] = None
            
            filtered_combinations.append(config)
        
        # Remove duplicates
        unique_combinations = []
        seen_configs = set()
        for config in filtered_combinations:
            # Create a hashable representation
            config_key = tuple(sorted((k, v) for k, v in config.items() if v is not None))
            if config_key not in seen_configs:
                seen_configs.add(config_key)
                unique_combinations.append(config)
        
        total_combinations = len(unique_combinations)
        
        logger.info(f"Testing {total_combinations} unique parameter combinations")
        logger.info(f"Parameters to optimize: {param_names}")
        
        # Test each configuration
        for i, config in enumerate(unique_combinations):
            logger.info(f"Testing configuration {i+1}/{total_combinations}: {config}")
            
            result = self.test_cpd_configuration(config)
            self.optimization_results.append(result)
            
            # Progress report every 5 configurations
            if (i + 1) % 5 == 0 or i == total_combinations - 1:
                successful_configs = [r for r in self.optimization_results if r['success']]
                success_rate = len(successful_configs) / len(self.optimization_results) * 100
                
                if successful_configs:
                    best_so_far = min(successful_configs, key=lambda x: x['test_rmse'])
                    logger.info(f"Progress: {i+1}/{total_combinations} ({success_rate:.1f}% success)")
                    logger.info(f"Best so far: {best_so_far['test_rmse']*1000:.3f}mm with {best_so_far['config']}")
                else:
                    logger.info(f"Progress: {i+1}/{total_combinations} (no successful configs yet)")
        
        # Generate optimization report
        self.generate_optimization_report()
        
        return self.optimization_results
    
    def generate_optimization_report(self):
        """Generate comprehensive optimization report"""
        logger.info("=== Generating CPD Optimization Report ===")
        
        # Filter successful results
        successful_results = [r for r in self.optimization_results if r['success']]
        failed_results = [r for r in self.optimization_results if not r['success']]
        
        if not successful_results:
            logger.warning("No successful configurations found")
            # Still save failed results for analysis
            self._save_failed_results(failed_results)
            return
        
        # Sort by test RMSE
        successful_results.sort(key=lambda x: x['test_rmse'])
        
        # Print top 10 configurations
        print("\n" + "="*130)
        print("CPD PARAMETER OPTIMIZATION RESULTS - BROADCASTING FIXED")
        print("WORKFLOW: SVD Pre-alignment -> CPD Training -> Test Transformation")
        print("="*130)
        print(f"Total configurations tested: {len(self.optimization_results)}")
        print(f"Successful configurations: {len(successful_results)}")
        print(f"Failed configurations: {len(failed_results)}")
        print(f"Success rate: {len(successful_results)/len(self.optimization_results)*100:.1f}%")
        
        print("\nTOP 10 CONFIGURATIONS:")
        print("-" * 130)
        
        header = "| Rank | Type       | Max_Iter | Tolerance | W    | Alpha | Beta | Train_RMSE | Test_RMSE | Time  | Target |"
        separator = "|------|------------|----------|-----------|------|-------|------|------------|-----------|-------|--------|"
        
        print(header)
        print(separator)
        
        for i, result in enumerate(successful_results[:10]):
            config = result['config']
            alpha_str = f"{config.get('alpha', 0):5.1f}" if config.get('alpha') is not None else "  N/A"
            beta_str = f"{config.get('beta', 0):4.1f}" if config.get('beta') is not None else " N/A"
            target_met = "Yes" if result['test_rmse'] <= TARGET_RMSE else "No"
            
            row = f"| {i+1:4d} | {config['registration_type']:10s} | {config['max_iterations']:8d} | {config['tolerance']:9.0e} | {config.get('w', 0):4.2f} | {alpha_str} | {beta_str} | {result['train_rmse']*1000:10.3f} | {result['test_rmse']*1000:9.3f} | {result['computation_time']:5.2f} | {target_met:6s} |"
            print(row)
        
        # Best configuration summary
        best_config = successful_results[0]
        print(f"\nBEST CONFIGURATION:")
        print(f"  Registration Type: {best_config['config']['registration_type']}")
        print(f"  Max Iterations: {best_config['config']['max_iterations']}")
        print(f"  Tolerance: {best_config['config']['tolerance']:.0e}")
        print(f"  W (outlier weight): {best_config['config'].get('w', 'N/A')}")
        print(f"  Alpha (regularization): {best_config['config'].get('alpha', 'N/A')}")
        print(f"  Beta (smoothness): {best_config['config'].get('beta', 'N/A')}")
        print(f"  Train RMSE: {best_config['train_rmse']*1000:.3f}mm")
        print(f"  Test RMSE: {best_config['test_rmse']*1000:.3f}mm")
        print(f"  Target achieved: {'Yes' if best_config['test_rmse'] <= TARGET_RMSE else 'No'}")
        print(f"  Computation time: {best_config['computation_time']:.3f}s")
        
        # Save results to files
        self._save_optimization_results(successful_results, failed_results)
        
        # Create visualization
        self._create_optimization_plots(successful_results)
        
        print("="*130)
    
    def _save_failed_results(self, failed_results):
        """Save failed results for debugging"""
        if failed_results:
            df_failed = pd.DataFrame([
                {
                    'registration_type': r['config']['registration_type'],
                    'max_iterations': r['config']['max_iterations'],
                    'tolerance': r['config']['tolerance'],
                    'w': r['config'].get('w', None),
                    'alpha': r['config'].get('alpha', None),
                    'beta': r['config'].get('beta', None),
                    'error_message': r['error_message'],
                    'computation_time_s': r['computation_time']
                }
                for r in failed_results
            ])
            
            df_failed.to_csv(os.path.join(result_dir, 'failed_cpd_configs.csv'), index=False)
            logger.info("Failed configurations saved for debugging")
    
    def _save_optimization_results(self, successful_results, failed_results):
        """Save optimization results to files"""
        
        # Save successful results as CSV
        if successful_results:
            df_successful = pd.DataFrame([
                {
                    'rank': i+1,
                    'registration_type': r['config']['registration_type'],
                    'max_iterations': r['config']['max_iterations'],
                    'tolerance': r['config']['tolerance'],
                    'w': r['config'].get('w', None),
                    'alpha': r['config'].get('alpha', None),
                    'beta': r['config'].get('beta', None),
                    'train_rmse_mm': r['train_rmse'] * 1000,
                    'test_rmse_mm': r['test_rmse'] * 1000,
                    'lateral_rmse_mm': r['lateral_rmse'] * 1000,
                    'longitudinal_rmse_mm': r['longitudinal_rmse'] * 1000,
                    'computation_time_s': r['computation_time'],
                    'meets_target': r['test_rmse'] <= TARGET_RMSE
                }
                for i, r in enumerate(successful_results)
            ])
            
            df_successful.to_csv(os.path.join(result_dir, 'successful_cpd_configs.csv'), index=False)
        
        # Save failed results
        self._save_failed_results(failed_results)
        
        # Save all results as JSON
        with open(os.path.join(result_dir, 'all_cpd_results.json'), 'w') as f:
            json.dump(self.optimization_results, f, indent=2, default=str)
        
        # Save best configuration for easy use
        if successful_results:
            best_config = successful_results[0]['config']
            with open(os.path.join(result_dir, 'best_cpd_config.json'), 'w') as f:
                json.dump(best_config, f, indent=2)
        
        logger.info("Optimization results saved to files")
    
    def _create_optimization_plots(self, successful_results):
        """Create optimization visualization plots"""
        if not successful_results:
            logger.warning("No successful results to plot")
            return
        
        fig, axes = plt.subplots(2, 3, figsize=(18, 12))
        
        # Extract data for plotting
        registration_types = [r['config']['registration_type'] for r in successful_results]
        max_iterations = [r['config']['max_iterations'] for r in successful_results]
        tolerances = [r['config']['tolerance'] for r in successful_results]
        test_rmse = [r['test_rmse'] * 1000 for r in successful_results]
        train_rmse = [r['train_rmse'] * 1000 for r in successful_results]
        computation_times = [r['computation_time'] for r in successful_results]
        
        # Plot 1: RMSE vs Registration Type
        ax1 = axes[0, 0]
        type_rmse = {}
        for reg_type, rmse in zip(registration_types, test_rmse):
            if reg_type not in type_rmse:
                type_rmse[reg_type] = []
            type_rmse[reg_type].append(rmse)
        
        types = list(type_rmse.keys())
        rmse_values = [type_rmse[t] for t in types]
        ax1.boxplot(rmse_values, labels=types)
        ax1.axhline(y=TARGET_RMSE*1000, color='red', linestyle='--', label=f'Target: {TARGET_RMSE*1000:.1f}mm')
        ax1.set_ylabel('Test RMSE [mm]')
        ax1.set_title('RMSE by Registration Type')
        ax1.legend()
        ax1.grid(True, alpha=0.3)
        
        # Plot 2: RMSE vs Max Iterations
        ax2 = axes[0, 1]
        scatter = ax2.scatter(max_iterations, test_rmse, c=computation_times, cmap='viridis', alpha=0.6)
        ax2.axhline(y=TARGET_RMSE*1000, color='red', linestyle='--', label=f'Target: {TARGET_RMSE*1000:.1f}mm')
        ax2.set_xlabel('Max Iterations')
        ax2.set_ylabel('Test RMSE [mm]')
        ax2.set_title('RMSE vs Max Iterations')
        ax2.legend()
        ax2.grid(True, alpha=0.3)
        plt.colorbar(scatter, ax=ax2, label='Time [s]')
        
        # Plot 3: RMSE vs Tolerance
        ax3 = axes[0, 2]
        ax3.semilogx(tolerances, test_rmse, 'o', alpha=0.6)
        ax3.axhline(y=TARGET_RMSE*1000, color='red', linestyle='--', label=f'Target: {TARGET_RMSE*1000:.1f}mm')
        ax3.set_xlabel('Tolerance')
        ax3.set_ylabel('Test RMSE [mm]')
        ax3.set_title('RMSE vs Tolerance')
        ax3.legend()
        ax3.grid(True, alpha=0.3)
        
        # Plot 4: Train vs Test RMSE
        ax4 = axes[1, 0]
        ax4.scatter(train_rmse, test_rmse, alpha=0.6)
        # Perfect correlation line
        min_rmse = min(min(train_rmse), min(test_rmse))
        max_rmse = max(max(train_rmse), max(test_rmse))
        ax4.plot([min_rmse, max_rmse], [min_rmse, max_rmse], 'k--', alpha=0.5, label='Perfect Correlation')
        ax4.axhline(y=TARGET_RMSE*1000, color='red', linestyle='--', alpha=0.7)
        ax4.axvline(x=TARGET_RMSE*1000, color='red', linestyle='--', alpha=0.7)
        ax4.set_xlabel('Train RMSE [mm]')
        ax4.set_ylabel('Test RMSE [mm]')
        ax4.set_title('Train vs Test RMSE')
        ax4.legend()
        ax4.grid(True, alpha=0.3)
        
        # Plot 5: RMSE vs Computation Time
        ax5 = axes[1, 1]
        colors = ['red' if t > TARGET_RMSE*1000 else 'green' for t in test_rmse]
        ax5.scatter(computation_times, test_rmse, c=colors, alpha=0.6)
        ax5.axhline(y=TARGET_RMSE*1000, color='red', linestyle='--', label=f'Target: {TARGET_RMSE*1000:.1f}mm')
        ax5.set_xlabel('Computation Time [s]')
        ax5.set_ylabel('Test RMSE [mm]')
        ax5.set_title('RMSE vs Computation Time')
        ax5.legend()
        ax5.grid(True, alpha=0.3)
        
        # Plot 6: Top 10 Configuration Comparison
        ax6 = axes[1, 2]
        top_10 = successful_results[:10]
        top_10_rmse = [r['test_rmse'] * 1000 for r in top_10]
        top_10_labels = [f"{r['config']['registration_type'][:3]}\n{r['config']['max_iterations']}" for r in top_10]
        
        bars = ax6.bar(range(len(top_10)), top_10_rmse)
        ax6.axhline(y=TARGET_RMSE*1000, color='red', linestyle='--', label=f'Target: {TARGET_RMSE*1000:.1f}mm')
        ax6.set_xlabel('Configuration Rank')
        ax6.set_ylabel('Test RMSE [mm]')
        ax6.set_title('Top 10 Configurations')
        ax6.set_xticks(range(len(top_10)))
        ax6.set_xticklabels(top_10_labels, rotation=45, ha='right')
        ax6.legend()
        ax6.grid(True, alpha=0.3)
        
        # Color bars based on target achievement
        for bar, rmse in zip(bars, top_10_rmse):
            if rmse <= TARGET_RMSE * 1000:
                bar.set_color('green')
            else:
                bar.set_color('red')
        
        plt.tight_layout()
        plt.savefig(os.path.join(img_dir, 'cpd_optimization_analysis_broadcast_fix.png'), dpi=300, bbox_inches='tight')
        plt.close()
        
        logger.info("Optimization plots saved")

class SimpleDataSplitter:
    """Simple data splitter for train/test separation"""
    def __init__(self, train_ratio=0.8, random_seed=42):
        self.train_ratio = train_ratio
        self.random_seed = random_seed
        
    def split_data(self, positions_A, positions_B):
        """Split data into training and test sets"""
        positions_A = np.array(positions_A)
        positions_B = np.array(positions_B)
        
        total_points = len(positions_A)
        train_size = int(total_points * self.train_ratio)
        
        np.random.seed(self.random_seed)
        all_indices = np.arange(total_points)
        np.random.shuffle(all_indices)
        
        train_indices = all_indices[:train_size]
        test_indices = all_indices[train_size:]
        
        train_A = positions_A[train_indices]
        train_B = positions_B[train_indices]
        test_A = positions_A[test_indices]
        test_B = positions_B[test_indices]
        
        logger.info(f"Data split: {len(train_A)} training, {len(test_A)} test points")
        
        return train_A, train_B, test_A, test_B

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
    """Main function for CPD parameter optimization"""
    logger.info("Starting CPD Parameter Optimization Framework - BROADCASTING FIXED")
    logger.info("WORKFLOW: SVD Pre-alignment -> CPD Training -> Test Transformation")
    
    global LASER_TRACKER_CSV
    if len(sys.argv) > 1:
        LASER_TRACKER_CSV = sys.argv[1]
    
    if not os.path.exists(LASER_TRACKER_CSV):
        logger.error(f"Laser tracker data file {LASER_TRACKER_CSV} not found.")
        logger.info("Usage: python cpd_optimizer_broadcast_fix.py <path_to_laser_tracker.csv> [test_mode]")
        return
    
    # Get test mode from command line
    test_mode = 'quick'  # default
    if len(sys.argv) > 2:
        test_mode = sys.argv[2].lower()
        if test_mode not in ['basic', 'quick', 'comprehensive']:
            logger.warning(f"Unknown test mode '{test_mode}', using 'quick'")
            test_mode = 'quick'
    
    logger.info(f"Test mode: {test_mode}")
    
    # Process data
    df = process_laser_tracker_data(LASER_TRACKER_CSV)
    if df is None:
        logger.error("Failed to process laser tracker data")
        return
    
    # Prepare data
    positions_A = [[row[TRACKER_X], 0, row[TRACKER_Y]] for _, row in df.iterrows()]
    positions_B = [[row[LASER_X], 0, row[LASER_Y]] for _, row in df.iterrows()]
    
    logger.info(f"Loaded {len(positions_A)} point pairs for registration")
    
    # Split data
    splitter = SimpleDataSplitter(train_ratio=TRAIN_TEST_RATIO)
    train_A, train_B, test_A, test_B = splitter.split_data(positions_A, positions_B)
    
    # Run CPD optimization
    optimizer = CPDParameterOptimizer(train_A, train_B, test_A, test_B)
    results = optimizer.run_parameter_optimization(test_mode=test_mode)
    
    if results:
        successful_results = [r for r in results if r['success']]
        if successful_results:
            best_result = min(successful_results, key=lambda x: x['test_rmse'])
            logger.info(f"CPD optimization completed successfully")
            logger.info(f"Best configuration achieved {best_result['test_rmse']*1000:.3f}mm RMSE")
            logger.info(f"Best config: {best_result['config']}")
        else:
            logger.warning("No successful CPD configurations found")
    else:
        logger.error("CPD optimization failed")

if __name__ == "__main__":
    main()