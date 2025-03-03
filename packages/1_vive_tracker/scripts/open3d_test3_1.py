import numpy as np
import open3d as o3d
import pandas as pd
import copy
import logging
import os
import datetime

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
    df, XZ_data, laser_data = load_data(r"C:\Users\13366\Desktop\fsm_2208\packages\1_vive_tracker\scripts\corrected_tracker_log_0303.csv")
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

# === Calculate Original Point Cloud RMSE ===
def calculate_original_point_rmse(source_points, target_points, transformation):
    """Calculate RMSE between transformed source points and target points"""
    logger.info("Calculating original point cloud RMSE")
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
        
        return rmse
        
    except Exception as e:
        logger.error(f"RMSE calculation failed: {e}")
        return None

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
    
    # Calculate and output final RMSE
    rmse = calculate_original_point_rmse(XZ_data, laser_data, final_transformation)
    logger.info(f"=== Final Registration RMSE: {rmse:.5f} ===")
    
    # === Save transformation results ===
    try:
        # Transform original data
        XZ_homo = np.hstack((XZ_data, np.zeros((XZ_data.shape[0], 1)), np.ones((XZ_data.shape[0], 1))))
        transformed_XZ_homo = (final_transformation @ XZ_homo.T).T
        transformed_XZ = transformed_XZ_homo[:, :2]  # Take first two columns (X, Z)
        
        transformed_df = pd.DataFrame(transformed_XZ, columns=["X_aligned", "Z_aligned"])
        result_df = pd.concat([df, transformed_df], axis=1)
        
        output_path = r"C:\Users\13366\Desktop\fsm_2208\packages\1_vive_tracker\scripts\aligned_tracker_log.csv"
        result_df.to_csv(output_path, index=False)
        logger.info(f"Transformation results saved to CSV file: {output_path}")
        
        # Add transformation matrix information to log
        logger.info("Saving transformation matrix information")
        transform_info = pd.DataFrame({
            'description': ['Transformation_Matrix', 'RMSE'],
            'value': [str(final_transformation), str(rmse)]
        })
        
        transform_info_path = r"C:\Users\13366\Desktop\fsm_2208\packages\1_vive_tracker\scripts\transform_info.csv"
        transform_info.to_csv(transform_info_path, index=False)
        logger.info(f"Transformation matrix information saved to CSV file: {transform_info_path}")
        
    except Exception as e:
        logger.error(f"Failed to save CSV: {e}")
    
    logger.info("=== Point Cloud Registration Process Completed ===")

if __name__ == "__main__":
    try:
        main()
    except Exception as e:
        logger.critical(f"Program terminated abnormally: {e}", exc_info=True)