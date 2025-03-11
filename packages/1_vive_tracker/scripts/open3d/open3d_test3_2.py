import numpy as np
import open3d as o3d
import pandas as pd
import copy
import logging
import os
import datetime
import matplotlib.pyplot as plt
from matplotlib.colors import LinearSegmentedColormap

# === Setup Logger ===
def setup_logger(log_file_path=None):
    """Set up the logger"""
    if log_file_path is None:
        # Create log filename with timestamp
        timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        log_dir = r"C:\Users\13366\Desktop\fsm_2208\packages\1_vive_tracker\logs"
        os.makedirs(log_dir, exist_ok=True)
        log_file_path = os.path.join(log_dir, f"registration_log_{timestamp}.txt")
    
    # Configure logger
    logger = logging.getLogger("registration")
    logger.setLevel(logging.INFO)
    
    # Create file handler
    file_handler = logging.FileHandler(log_file_path, encoding='utf-8')
    file_handler.setLevel(logging.INFO)
    
    # Create console handler
    console_handler = logging.StreamHandler()
    console_handler.setLevel(logging.INFO)
    
    # Set format
    formatter = logging.Formatter('%(asctime)s - %(levelname)s - %(message)s')
    file_handler.setFormatter(formatter)
    console_handler.setFormatter(formatter)
    
    # Add handlers to logger
    logger.addHandler(file_handler)
    logger.addHandler(console_handler)
    
    logger.info(f"Log file created at: {log_file_path}")
    return logger

# Create logger
logger = setup_logger()

# === Load Data ===
def load_data(file_path):
    """Load data and log information"""
    logger.info(f"Reading data file: {file_path}")
    try:
        df = pd.read_csv(file_path)
        XZ_data = df[["X", "Z"]].values
        laser_data = df[["laser_x", "laser_z"]].values
        
        logger.info(f"Data loaded successfully. XZ_data points: {len(XZ_data)}, laser_data points: {len(laser_data)}")
        
        # Log data statistics
        logger.info("XZ_data statistics:")
        logger.info(f"  X range: [{np.min(XZ_data[:, 0]):.4f}, {np.max(XZ_data[:, 0]):.4f}]")
        logger.info(f"  Z range: [{np.min(XZ_data[:, 1]):.4f}, {np.max(XZ_data[:, 1]):.4f}]")
        
        logger.info("laser_data statistics:")
        logger.info(f"  X range: [{np.min(laser_data[:, 0]):.4f}, {np.max(laser_data[:, 0]):.4f}]")
        logger.info(f"  Z range: [{np.min(laser_data[:, 1]):.4f}, {np.max(laser_data[:, 1]):.4f}]")
        
        return df, XZ_data, laser_data
    except Exception as e:
        logger.error(f"Failed to load data: {e}")
        raise

# Load data
try:
    df, XZ_data, laser_data = load_data(r"C:\Users\13366\Desktop\fsm_2208\packages\1_vive_tracker\scripts\data\corrected_tracker_log_0303.csv")
except Exception as e:
    logger.critical(f"Program terminated: {e}")
    exit(1)

# === Improvement 1: Enhanced Point Cloud Preprocessing ===
def preprocess_point_cloud(points, voxel_size=0.05, name=""):
    """Preprocess the point cloud: downsample, remove outliers, compute normals"""
    logger.info(f"Preprocessing point cloud '{name}', input points: {len(points)}")
    # Create point cloud
    pcd = o3d.geometry.PointCloud()
    
    # Add small random noise to avoid coplanar issues
    points_with_noise = points + np.random.rand(points.shape[0], points.shape[1]) * voxel_size * 0.01
    
    # Convert 2D points to 3D, add small random Y values to avoid strict coplanarity
    points_3d = np.hstack((
        points_with_noise, 
        np.random.rand(len(points), 1) * voxel_size * 0.01  # Add small random Y values
    ))
    
    pcd.points = o3d.utility.Vector3dVector(points_3d)
    
    # Voxel downsampling
    logger.info(f"Performing voxel downsampling, voxel size: {voxel_size}")
    pcd_down = pcd.voxel_down_sample(voxel_size)
    logger.info(f"Points after downsampling: {len(pcd_down.points)}")
    
    # If too few points, may not be able to perform outlier removal
    if len(pcd_down.points) > 30:
        try:
            logger.info("Performing statistical outlier removal")
            cl, ind = pcd_down.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)
            pcd_clean = pcd_down.select_by_index(ind)
            logger.info(f"Points after outlier removal: {len(pcd_clean.points)}")
        except RuntimeError as e:
            logger.warning(f"Outlier removal failed: {e}, using original downsampled point cloud")
            pcd_clean = pcd_down
    else:
        logger.warning(f"Too few points ({len(pcd_down.points)}), skipping outlier removal")
        pcd_clean = pcd_down
    
    # Estimate normals (using more robust parameters)
    try:
        logger.info("Computing normals")
        pcd_clean.estimate_normals(
            search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=voxel_size * 3, max_nn=30)
        )
        logger.info("Normals computed successfully")
    except RuntimeError as e:
        logger.warning(f"Normal estimation failed: {e}, using default normals")
        # If normal estimation fails, manually set default normals
        default_normals = np.zeros((len(pcd_clean.points), 3))
        default_normals[:, 2] = 1.0  # All normals point to positive Z
        pcd_clean.normals = o3d.utility.Vector3dVector(default_normals)
    
    return pcd_clean

# === Create Open3D Point Cloud Objects ===
def create_colored_point_cloud(points, color):
    """Create a colored point cloud"""
    pcd = o3d.geometry.PointCloud()
    # Add small Y values to avoid strict coplanarity
    points_3d = np.hstack((
        points, 
        np.random.rand(len(points), 1) * 0.001  # Add small random Y values
    ))
    pcd.points = o3d.utility.Vector3dVector(points_3d)
    pcd.paint_uniform_color(color)
    return pcd

# === Improvement 2: Simplified Feature Computation ===
def compute_enhanced_features(pcd, voxel_size=0.05, name=""):
    """Compute enhanced features for point cloud, with error handling"""
    logger.info(f"Computing features for '{name}', input points: {len(pcd.points)}")
    # Ensure point cloud is downsampled
    if len(pcd.points) > 5000:  # If too many points, perform voxel downsampling
        logger.info(f"Too many points ({len(pcd.points)}), performing voxel downsampling")
        pcd = pcd.voxel_down_sample(voxel_size)
        logger.info(f"Points after downsampling: {len(pcd.points)}")
    
    # Recompute normals (using larger search radius and more neighbors)
    try:
        logger.info("Recomputing normals")
        pcd.estimate_normals(
            search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=voxel_size * 3, max_nn=50)
        )
        logger.info("Normals computed successfully")
    except RuntimeError as e:
        logger.warning(f"Normal estimation failed: {e}, using default normals")
        # Set default normals
        default_normals = np.zeros((len(pcd.points), 3))
        default_normals[:, 2] = 1.0
        pcd.normals = o3d.utility.Vector3dVector(default_normals)
    
    # Compute FPFH features
    try:
        logger.info("Computing FPFH features")
        fpfh = o3d.pipelines.registration.compute_fpfh_feature(
            pcd,
            search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=voxel_size * 8, max_nn=150)
        )
        logger.info(f"FPFH features computed successfully, dimension: {fpfh.dimension()}")
        return pcd, fpfh
    except RuntimeError as e:
        logger.warning(f"FPFH computation failed: {e}, using simple features instead")
        # If FPFH computation fails, return simple coordinates as features
        dummy_fpfh = o3d.pipelines.registration.Feature()
        points = np.asarray(pcd.points)
        dummy_fpfh.data = np.hstack([points, np.zeros((len(points), 30-points.shape[1]))])
        logger.info("Using point coordinates as simple features")
        return pcd, dummy_fpfh

# === Improvement 3: Optimized RANSAC Registration with Error Handling ===
def compute_optimized_ransac(source, target, voxel_size=0.05):
    """Perform initial alignment using optimized RANSAC, with error handling"""
    logger.info("Starting RANSAC initial alignment")
    try:
        logger.info("Computing features for source point cloud")
        source_down, source_fpfh = compute_enhanced_features(source, voxel_size, name="source")
        
        logger.info("Computing features for target point cloud")
        target_down, target_fpfh = compute_enhanced_features(target, voxel_size, name="target")
        
        # Increase iterations and adjust parameters for better initial registration
        distance_threshold = voxel_size * 1.5
        logger.info(f"RANSAC parameters: distance_threshold={distance_threshold}, ransac_n=3")
        
        # Try using RANSAC
        try:
            logger.info("Executing RANSAC feature matching")
            reg_ransac = o3d.pipelines.registration.registration_ransac_based_on_feature_matching(
                source_down, target_down, source_fpfh, target_fpfh, mutual_filter=True,
                max_correspondence_distance=distance_threshold,
                estimation_method=o3d.pipelines.registration.TransformationEstimationPointToPoint(False),
                ransac_n=3,
                checkers=[
                    o3d.pipelines.registration.CorrespondenceCheckerBasedOnEdgeLength(0.8),
                    o3d.pipelines.registration.CorrespondenceCheckerBasedOnDistance(distance_threshold)
                ],
                criteria=o3d.pipelines.registration.RANSACConvergenceCriteria(100000, 0.9999)
            )
            
            logger.info(f"RANSAC registration result: fitness={reg_ransac.fitness}, RMSE={reg_ransac.inlier_rmse}")
            # Log transformation matrix
            logger.info("RANSAC transformation matrix:")
            for row in reg_ransac.transformation:
                logger.info(f"  {row}")
                
            return reg_ransac.transformation
            
        except RuntimeError as e:
            logger.warning(f"RANSAC failed: {e}, using centroid alignment instead")
            # If RANSAC fails, try using centroid alignment as fallback
            source_center = np.mean(np.asarray(source_down.points), axis=0)
            target_center = np.mean(np.asarray(target_down.points), axis=0)
            
            # Create translation transformation
            init_translation = np.eye(4)
            init_translation[:3, 3] = target_center - source_center
            
            logger.info("Centroid alignment transformation matrix:")
            for row in init_translation:
                logger.info(f"  {row}")
                
            return init_translation
            
    except Exception as e:
        logger.error(f"Error during RANSAC registration: {e}, using identity matrix")
        return np.identity(4)

# === Improvement 4: Multi-Stage ICP Registration ===
def multi_stage_icp(source, target, init_transform=np.identity(4), voxel_sizes=[0.1, 0.05, 0.02, 0.01]):
    """Multi-resolution ICP registration strategy, with error handling"""
    logger.info("Starting multi-stage ICP registration")
    current_transform = init_transform
    
    # Record initial registration evaluation
    try:
        evaluation = o3d.pipelines.registration.evaluate_registration(
            source, target, 0.05, current_transform)
        logger.info(f"Initial registration evaluation: fitness={evaluation.fitness}, RMSE={evaluation.inlier_rmse}")
    except Exception as e:
        logger.warning(f"Initial registration evaluation failed: {e}")
    
    # Gradually decrease threshold for ICP registration
    for i, voxel_size in enumerate(voxel_sizes):
        threshold = voxel_size * 2
        logger.info(f"Stage {i+1} ICP: voxel_size={voxel_size}, threshold={threshold}")
        
        try:
            # Select appropriate ICP method
            if i < len(voxel_sizes) // 2:
                # Early stages use point-to-point registration
                method = o3d.pipelines.registration.TransformationEstimationPointToPoint()
                logger.info("Using point-to-point registration method")
            else:
                # Later stages use point-to-plane registration
                method = o3d.pipelines.registration.TransformationEstimationPointToPlane()
                logger.info("Using point-to-plane registration method")
            
            reg_p2x = o3d.pipelines.registration.registration_icp(
                source, target, threshold, current_transform,
                method,
                o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=100)
            )
            
            current_transform = reg_p2x.transformation
            logger.info(f"Stage {i+1} completed: fitness={reg_p2x.fitness}, RMSE={reg_p2x.inlier_rmse}")
            
            # Log current transformation matrix
            logger.info(f"Stage {i+1} transformation matrix:")
            for row in current_transform:
                logger.info(f"  {row}")
            
        except Exception as e:
            logger.warning(f"Stage {i+1} ICP failed: {e}, continuing to next stage")
    
    # Final stage
    try:
        logger.info("Executing final ICP fine registration")
        final_icp = o3d.pipelines.registration.registration_icp(
            source, target, voxel_sizes[-1] * 1.5, current_transform,
            o3d.pipelines.registration.TransformationEstimationPointToPoint(),
            o3d.pipelines.registration.ICPConvergenceCriteria(
                relative_fitness=1e-6, 
                relative_rmse=1e-6,
                max_iteration=100
            )
        )
        
        logger.info(f"Final ICP result: fitness={final_icp.fitness}, RMSE={final_icp.inlier_rmse}")
        
        # Log final transformation matrix
        logger.info("Final transformation matrix:")
        for row in final_icp.transformation:
            logger.info(f"  {row}")
            
        return final_icp.transformation
        
    except Exception as e:
        logger.warning(f"Final ICP stage failed: {e}, using current transformation")
        return current_transform

# === Calculate Original Point Cloud RMSE and Identify High-Error Points ===
def calculate_original_point_rmse_and_identify_outliers(source_points, target_points, transformation, error_threshold=0.15):
    """Calculate RMSE between transformed source points and target points, identify high-error points"""
    logger.info("Calculating original point cloud RMSE and identifying high-error points")
    # Convert source points to homogeneous coordinates
    source_homo = np.hstack((source_points, np.zeros((source_points.shape[0], 1)), np.ones((source_points.shape[0], 1))))
    
    # Apply transformation
    transformed_homo = (transformation @ source_homo.T).T
    transformed_points = transformed_homo[:, :2]  # Take first two columns (X, Z)
    
    # Create point cloud objects for distance calculation
    source_pcd = o3d.geometry.PointCloud()
    source_pcd.points = o3d.utility.Vector3dVector(np.hstack((transformed_points, np.zeros((transformed_points.shape[0], 1)))))
    
    target_pcd = o3d.geometry.PointCloud()
    target_pcd.points = o3d.utility.Vector3dVector(np.hstack((target_points, np.zeros((target_points.shape[0], 1)))))
    
    # Calculate point cloud distances
    try:
        logger.info("Computing distances between point clouds")
        distances = source_pcd.compute_point_cloud_distance(target_pcd)
        distances = np.asarray(distances)
        
        # Calculate statistics
        min_dist = np.min(distances)
        max_dist = np.max(distances)
        mean_dist = np.mean(distances)
        median_dist = np.median(distances)
        rmse = np.sqrt(np.mean(np.square(distances)))
        std_dev = np.std(distances)
        
        logger.info("=== Final matching result statistics ===")
        logger.info(f"Minimum distance: {min_dist:.5f}")
        logger.info(f"Maximum distance: {max_dist:.5f}")
        logger.info(f"Average distance: {mean_dist:.5f}")
        logger.info(f"Median distance: {median_dist:.5f}")
        logger.info(f"Standard deviation: {std_dev:.5f}")
        logger.info(f"RMSE: {rmse:.5f}")
        
        # Log distance distribution histogram
        hist, bins = np.histogram(distances, bins=20)
        logger.info("Distance distribution histogram:")
        for i in range(len(hist)):
            logger.info(f"  [{bins[i]:.4f}, {bins[i+1]:.4f}): {hist[i]}")
        
        # Identify high-error points
        high_error_indices = np.where(distances > error_threshold)[0]
        logger.info(f"Identified {len(high_error_indices)} points with error > {error_threshold}")
        
        return rmse, distances, high_error_indices, transformed_points, bins
        
    except Exception as e:
        logger.error(f"RMSE calculation failed: {e}")
        return None, None, [], None, None

# === New Function: Visualize Error Distribution ===
def visualize_error_distribution(original_points, transformed_points, target_points, distances, high_error_indices, bins, error_threshold):
    """Visualize the error distribution with matplotlib"""
    logger.info("Generating error visualization plots")
    
    # Create output directory if it doesn't exist
    output_dir = r"C:\Users\13366\Desktop\fsm_2208\packages\1_vive_tracker\visualizations"
    os.makedirs(output_dir, exist_ok=True)
    
    # Create a timestamp for unique filenames
    timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    
    try:
        # Plot 1: Scatter plot showing all points with error color coding
        plt.figure(figsize=(12, 10))
        
        # Create custom colormap: blue->green->yellow->red
        colors = [(0, 0, 1), (0, 1, 0), (1, 1, 0), (1, 0, 0)]
        cmap_name = 'error_colormap'
        cm = LinearSegmentedColormap.from_list(cmap_name, colors, N=100)
        
        # Normalize colors based on distance
        max_error_for_color = max(error_threshold * 2, np.max(distances))
        normalized_distances = np.clip(distances / max_error_for_color, 0, 1)
        
        # Plot target points (reference)
        plt.scatter(target_points[:, 0], target_points[:, 1], s=10, color='gray', alpha=0.5, label='Target (Laser)')
        
        # Plot transformed points with color based on error
        scatter = plt.scatter(transformed_points[:, 0], transformed_points[:, 1], 
                             c=normalized_distances, cmap=cm, s=30, alpha=0.8, 
                             label='Transformed Points')
        
        # Highlight high-error points with a different marker
        if len(high_error_indices) > 0:
            plt.scatter(transformed_points[high_error_indices, 0], 
                      transformed_points[high_error_indices, 1], 
                      s=100, facecolors='none', edgecolors='red', linewidth=2,
                      label=f'High Error (>{error_threshold})')
        
        # Add colorbar
        cbar = plt.colorbar(scatter)
        cbar.set_label('Error Distance')
        
        # Add threshold line in colorbar
        if error_threshold < max_error_for_color:
            normalized_threshold = error_threshold / max_error_for_color
            cbar.ax.axhline(y=normalized_threshold, color='r', linestyle='--')
            cbar.ax.text(0.5, normalized_threshold, f'Threshold: {error_threshold:.3f}', 
                         va='bottom', ha='center', transform=cbar.ax.transAxes, color='r')
        
        plt.title('Point Cloud Registration Result with Error Visualization')
        plt.xlabel('X Coordinate')
        plt.ylabel('Z Coordinate')
        plt.legend(loc='upper right')
        plt.grid(True, alpha=0.3)
        plt.axis('equal')
        
        # Save the plot
        scatter_plot_path = os.path.join(output_dir, f'error_scatter_{timestamp}.png')
        plt.savefig(scatter_plot_path, dpi=300, bbox_inches='tight')
        logger.info(f"Scatter plot saved to: {scatter_plot_path}")
        plt.close()
        
        # Plot 2: Before and After comparison
        plt.figure(figsize=(15, 8))
        
        # Before registration
        plt.subplot(1, 2, 1)
        plt.scatter(target_points[:, 0], target_points[:, 1], s=20, color='green', alpha=0.7, label='Target (Laser)')
        plt.scatter(original_points[:, 0], original_points[:, 1], s=20, color='red', alpha=0.7, label='Source (XZ)')
        plt.title('Before Registration')
        plt.xlabel('X Coordinate')
        plt.ylabel('Z Coordinate')
        plt.legend()
        plt.grid(True, alpha=0.3)
        plt.axis('equal')
        
        # After registration
        plt.subplot(1, 2, 2)
        plt.scatter(target_points[:, 0], target_points[:, 1], s=20, color='green', alpha=0.7, label='Target (Laser)')
        plt.scatter(transformed_points[:, 0], transformed_points[:, 1], s=20, color='blue', alpha=0.7, label='Transformed Source')
        
        # Highlight high-error points
        if len(high_error_indices) > 0:
            plt.scatter(transformed_points[high_error_indices, 0], 
                      transformed_points[high_error_indices, 1], 
                      s=100, facecolors='none', edgecolors='red', linewidth=2,
                      label=f'High Error (>{error_threshold})')
        
        plt.title('After Registration')
        plt.xlabel('X Coordinate')
        plt.ylabel('Z Coordinate')
        plt.legend()
        plt.grid(True, alpha=0.3)
        plt.axis('equal')
        
        plt.tight_layout()
        
        # Save the plot
        comparison_plot_path = os.path.join(output_dir, f'before_after_comparison_{timestamp}.png')
        plt.savefig(comparison_plot_path, dpi=300, bbox_inches='tight')
        logger.info(f"Comparison plot saved to: {comparison_plot_path}")
        plt.close()
        
        # Plot 3: Error histogram
        plt.figure(figsize=(12, 6))
        plt.hist(distances, bins=bins, alpha=0.7, color='skyblue', edgecolor='black')
        plt.axvline(x=error_threshold, color='r', linestyle='--', 
                   label=f'Error Threshold: {error_threshold:.3f}')
        plt.title('Error Distance Distribution Histogram')
        plt.xlabel('Error Distance')
        plt.ylabel('Count')
        plt.grid(True, alpha=0.3)
        plt.legend()
        
        # Save the plot
        histogram_plot_path = os.path.join(output_dir, f'error_histogram_{timestamp}.png')
        plt.savefig(histogram_plot_path, dpi=300, bbox_inches='tight')
        logger.info(f"Histogram plot saved to: {histogram_plot_path}")
        plt.close()
        
        # Plot 4: 3D visualization of high error regions
        # Create a heatmap of error density
        plt.figure(figsize=(12, 10))
        
        # Create a 2D histogram of errors
        x_bins = np.linspace(min(transformed_points[:, 0]), max(transformed_points[:, 0]), 50)
        z_bins = np.linspace(min(transformed_points[:, 1]), max(transformed_points[:, 1]), 50)
        
        error_grid, xedges, yedges = np.histogram2d(
            transformed_points[:, 0], transformed_points[:, 1], 
            bins=[x_bins, z_bins], weights=distances)
        
        count_grid, _, _ = np.histogram2d(
            transformed_points[:, 0], transformed_points[:, 1], 
            bins=[x_bins, z_bins])
        
        # Avoid division by zero
        count_grid[count_grid == 0] = 1
        avg_error_grid = error_grid / count_grid
        
        # Plot heatmap
        plt.imshow(avg_error_grid.T, origin='lower', aspect='auto', interpolation='bilinear',
                  extent=[xedges[0], xedges[-1], yedges[0], yedges[-1]],
                  cmap='hot')
        
        plt.colorbar(label='Average Error')
        plt.scatter(transformed_points[:, 0], transformed_points[:, 1], s=1, color='blue', alpha=0.3)
        
        # Highlight high-error points
        if len(high_error_indices) > 0:
            plt.scatter(transformed_points[high_error_indices, 0], 
                      transformed_points[high_error_indices, 1], 
                      s=50, facecolors='none', edgecolors='white', linewidth=1,
                      label=f'High Error (>{error_threshold})')
            
        plt.title('Error Density Heatmap')
        plt.xlabel('X Coordinate')
        plt.ylabel('Z Coordinate')
        plt.legend()
        
        # Save the plot
        heatmap_plot_path = os.path.join(output_dir, f'error_heatmap_{timestamp}.png')
        plt.savefig(heatmap_plot_path, dpi=300, bbox_inches='tight')
        logger.info(f"Heatmap plot saved to: {heatmap_plot_path}")
        plt.close()
        
        return scatter_plot_path, comparison_plot_path, histogram_plot_path, heatmap_plot_path
        
    except Exception as e:
        logger.error(f"Error generating visualization: {e}")
        return None, None, None, None

# === Main Processing Flow ===
def main():
    logger.info("=== Starting Point Cloud Registration Process ===")
    
    # Set voxel size
    voxel_size = 0.05  # Adjust based on data characteristics
    logger.info(f"Setting voxel size: {voxel_size}")
    
    logger.info("Preprocessing point clouds...")
    # Preprocess point clouds
    source_pcd = preprocess_point_cloud(XZ_data, voxel_size, name="source")
    target_pcd = preprocess_point_cloud(laser_data, voxel_size, name="target")
    
    logger.info("Computing initial transformation...")
    # Compute initial transformation
    trans_init = compute_optimized_ransac(source_pcd, target_pcd, voxel_size=voxel_size)
    
    logger.info("Executing multi-stage ICP registration...")
    # Multi-stage ICP registration (using fewer stages to reduce chances of errors)
    voxel_sizes = [0.1, 0.05, 0.02]  # Simplified multi-stage strategy
    final_transformation = multi_stage_icp(source_pcd, target_pcd, trans_init, voxel_sizes)
    
    logger.info("=== Transformation Matrix ===")
    for row in final_transformation:
        logger.info(f"{row}")
    
    # Calculate and output final RMSE, identify high-error points
    # Define error threshold (adjust based on your requirements)
    error_threshold = 0.15  # Points with error > 0.15 will be flagged
    rmse, distances, high_error_indices, transformed_points, bins = calculate_original_point_rmse_and_identify_outliers(
        XZ_data, laser_data, final_transformation, error_threshold)
    
    logger.info(f"=== Final Registration RMSE: {rmse:.5f} ===")
    
    # Visualize the error distribution
    if distances is not None and transformed_points is not None:
        scatter_path, comparison_path, histogram_path, heatmap_path = visualize_error_distribution(
            XZ_data, transformed_points, laser_data, distances, high_error_indices, bins, error_threshold)
        
        if scatter_path:
            logger.info("Error visualization completed successfully")
            logger.info(f"Generated visualizations:")
            logger.info(f"  - Scatter plot: {scatter_path}")
            logger.info(f"  - Before/After comparison: {comparison_path}")
            logger.info(f"  - Error histogram: {histogram_path}")
            logger.info(f"  - Error heatmap: {heatmap_path}")
    
    # === Save transformation results ===
    try:
        # Transform original data
        XZ_homo = np.hstack((XZ_data, np.zeros((XZ_data.shape[0], 1)), np.ones((XZ_data.shape[0], 1))))
        transformed_XZ_homo = (final_transformation @ XZ_homo.T).T
        transformed_XZ = transformed_XZ_homo[:, :2]  # Take first two columns (X, Z)
        
        # Create DataFrame with transformed points and error information
        transformed_df = pd.DataFrame({
            "X_aligned": transformed_XZ[:, 0],
            "Z_aligned": transformed_XZ[:, 1],
            "Error": distances,
            "High_Error_Flag": np.where(distances > error_threshold, 1, 0)
        })
        
        # Combine with original data
        result_df = pd.concat([df, transformed_df], axis=1)
        
        # Save main results
        output_path = r"C:\Users\13366\Desktop\fsm_2208\packages\1_vive_tracker\scripts\data\aligned_tracker_log.csv"
        result_df.to_csv(output_path, index=False)
        logger.info(f"Transformation results saved to CSV file: {output_path}")
        
        # Save high-error points to separate file for analysis
        if len(high_error_indices) > 0:
            high_error_df = result_df.iloc[high_error_indices].copy()
            high_error_path = r"C:\Users\13366\Desktop\fsm_2208\packages\1_vive_tracker\scripts\data\high_error_points.csv"
            high_error_df.to_csv(high_error_path, index=False)
            logger.info(f"High-error points saved to: {high_error_path}")
        
        # Add transformation matrix information to log
        logger.info("Saving transformation matrix information")
        transform_info = pd.DataFrame({
            'description': ['Transformation_Matrix', 'RMSE', 'Error_Threshold', 'High_Error_Points_Count'],
            'value': [str(final_transformation), str(rmse), str(error_threshold), str(len(high_error_indices))]
        })
        
        transform_info_path = r"C:\Users\13366\Desktop\fsm_2208\packages\1_vive_tracker\scripts\data\transform_info.csv"
        transform_info.to_csv(transform_info_path, index=False)
        logger.info(f"Transformation matrix information saved to CSV file: {transform_info_path}")
        
        # Generate error statistics per bin
        error_stats = []
        for i in range(len(bins)-1):
            bin_start = bins[i]
            bin_end = bins[i+1]
            bin_mask = (distances >= bin_start) & (distances < bin_end)
            bin_count = np.sum(bin_mask)
            
            if bin_count > 0:
                bin_points = result_df[bin_mask].copy()
                
                # Calculate statistics for points in this bin
                x_mean = bin_points['X'].mean() if 'X' in bin_points.columns else np.nan
                z_mean = bin_points['Z'].mean() if 'Z' in bin_points.columns else np.nan
                x_aligned_mean = bin_points['X_aligned'].mean()
                z_aligned_mean = bin_points['Z_aligned'].mean()
                
                error_stats.append({
                    'Bin_Start': bin_start,
                    'Bin_End': bin_end,
                    'Count': bin_count,
                    'Average_X': x_mean,
                    'Average_Z': z_mean,
                    'Average_X_Aligned': x_aligned_mean,
                    'Average_Z_Aligned': z_aligned_mean,
                    'Average_Error': bin_points['Error'].mean()
                })
        
        # Save error statistics
        if error_stats:
            error_stats_df = pd.DataFrame(error_stats)
            error_stats_path = r"C:\Users\13366\Desktop\fsm_2208\packages\1_vive_tracker\scripts\data\error_statistics.csv"
            error_stats_df.to_csv(error_stats_path, index=False)
            logger.info(f"Error statistics saved to: {error_stats_path}")
        
    except Exception as e:
        logger.error(f"Failed to save CSV: {e}")
    
    logger.info("=== Point Cloud Registration Process Completed ===")

if __name__ == "__main__":
    try:
        main()
    except Exception as e:
        logger.critical(f"Program terminated abnormally: {e}", exc_info=True)