# -*- coding: utf-8 -*-
"""transform_release_v0.4.py

Modified from transform_release_v0.3.py to include visualization
and error analysis with CSV output, adapted for Ubuntu environment
"""

import numpy as np
from scipy.spatial.transform import Rotation as R
import pandas as pd
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import os
import sys

# Create result directory if it doesn't exist
result_dir = "result"
os.makedirs(result_dir, exist_ok=True)

# Load data - adapt the path to your environment
try:
    tracker_laser_df = pd.read_csv("filtered_tracker_log.csv")
    #tracker_laser_df = pd.read_csv("corrected_tracker_log_0703.csv")
    print("Successfully loaded data file")
except FileNotFoundError:
    print("Error: Could not find the data file 'corrected_tracker_log_0703.csv'")
    print("Please ensure the file is in the current directory or provide the full path")
    sys.exit(1)

# Extract x, z (tracker position) and insert y=0
positions_A = [[row["X"], 0, row["Y"]] for _, row in tracker_laser_df.iterrows()]

# Extract laser_x, laser_z (laser position) and insert y=0
positions_B = [[row["laser_x"], 0, row["laser_z"]] for _, row in tracker_laser_df.iterrows()]

# Print a sample of data for verification
print("Extracted positions_A (x, 0, z):")
for i in range(min(5, len(positions_A))):  # Only print first 5 data points
    print(positions_A[i])

print("\nExtracted positions_B (laser_x, 0, laser_z):")
for i in range(min(5, len(positions_B))):  # Only print first 5 data points
    print(positions_B[i])

"""Class for rigid body transformation from vive_tracker to sandbox"""

class Point:
    """
    Represents a point in 3D space with position and orientation.
    """
    def __init__(self, position=None, orientation=None):
        """
        Initialize Point object.
        :param position: Position of the point, 3D vector [x, y, z], can be None
        :param orientation: Orientation of the point, Euler angles [roll,yaw,pitch], can be None
        """
        if position is not None:
            self.position = np.array(position)
        else:
            self.position = None
        if orientation is not None:
            self.orientation = np.array(orientation)
        else:
            self.orientation = None

class CoordinateTransformer:
    """
    Class for calculating and applying coordinate transformations.
    """
    def __init__(self, orientations_A=None, orientations_B=None, positions_A=None, positions_B=None):
        """
        Initialize CoordinateTransformer object.
        T_pos: Position transformation matrix
        R_euler: Orientation transformation matrix
        """
        self.T_pos = None
        self.R_euler = None

        # Store input data
        self.__orientations_A = orientations_A
        self.__orientations_B = orientations_B
        self.__positions_A = positions_A
        self.__positions_B = positions_B

        # If data is provided during initialization, calculate transformation matrices directly
        if orientations_A is not None and orientations_B is not None:
            self.calculate_orientation_transformation(orientations_A, orientations_B)
        if positions_A is not None and positions_B is not None:
            self.calculate_position_transformation(positions_A, positions_B)

    @staticmethod
    def euler_to_matrix(euler):
        """
        Convert Euler angles (yxz) to rotation matrix.
        """
        return R.from_euler('yxz', [euler[2], euler[0], euler[1]]).as_matrix()

    @staticmethod
    def matrix_to_euler(matrix):
        """
        Convert rotation matrix to Euler angles (yxz).
        Returns: Euler angles [yaw, pitch, roll] in radians.
        """
        euler = R.from_matrix(matrix).as_euler('yxz')
        return [euler[0], euler[2], euler[1]]

    @staticmethod
    def construct_transformation_matrix(rotation_matrix, translation):
        """
        Construct transformation matrix from rotation matrix and translation vector.
        """
        T = np.eye(4)
        T[:3, :3] = rotation_matrix
        T[:3, 3] = translation
        return T

    def calculate_position_transformation(self, positions_A=None, positions_B=None):
        """
        Calculate position transformation matrix between two sets of points.
        Returns T_pos and R_pos
        """
        if positions_A is not None and positions_B is not None:
            self.__positions_A = positions_A
            self.__positions_B = positions_B
        elif self.__positions_A is None or self.__positions_B is None:
            print("Please provide position data first!")
            return

        # Calculate transformation using stored data
        centroid_A = np.mean(self.__positions_A, axis=0)
        centroid_B = np.mean(self.__positions_B, axis=0)

        H = np.dot((self.__positions_A - centroid_A).T, (self.__positions_B - centroid_B))
        U, S, Vt = np.linalg.svd(H)
        R_pos = np.dot(Vt.T, U.T)
        if np.linalg.det(R_pos) < 0:
            Vt[-1, :] *= -1
            R_pos = np.dot(Vt.T, U.T)
        translation = centroid_B.T - np.dot(R_pos, centroid_A.T)
        self.T_pos = self.construct_transformation_matrix(R_pos, translation)
        return R_pos, translation

    def calculate_orientation_transformation(self, orientations_A=None, orientations_B=None):
        """
        Calculate orientation transformation matrix between two sets of orientations.
        """
        if orientations_A is not None and orientations_B is not None:
            self.__orientations_A = orientations_A
            self.__orientations_B = orientations_B
        elif self.__orientations_A is None or self.__orientations_B is None:
            print("Please provide orientation data first!")
            return

        # Calculate transformation using stored data
        R_A = [self.euler_to_matrix(orientation) for orientation in self.__orientations_A]
        R_B = [self.euler_to_matrix(orientation) for orientation in self.__orientations_B]

        R_diff = [np.dot(R_B[i], R_A[i].T) for i in range(len(R_A))]
        R_mean = np.mean(R_diff, axis=0)
        U, _, Vt = np.linalg.svd(R_mean)
        self.R_euler = np.dot(U, Vt)

    def apply_position_transformation(self, position):
        """
        Apply position transformation to a given point.
        """
        position_homogeneous = np.append(position, 1)
        transformed_position_homogeneous = np.dot(self.T_pos, position_homogeneous)
        return transformed_position_homogeneous[:3].round(3)

    def apply_orientation_transformation(self, orientation):
        """
        Apply orientation transformation to a given orientation.
        """
        R_point = self.euler_to_matrix(orientation)
        R_transformed = np.dot(self.R_euler, R_point)
        transformed_orientation = self.matrix_to_euler(R_transformed)
        return transformed_orientation

    def transform_point(self, point):
        """
        Apply complete transformation (position and orientation) to a given point.
        """
        transformed_position = self.apply_position_transformation(point.position)
        transformed_orientation = self.apply_orientation_transformation(point.orientation)
        return Point(transformed_position, transformed_orientation)

    def calculate_position_error(self, actual, predicted):
        """
        Calculate root mean square error (RMSE) for positions.
        """
        return np.sqrt(np.mean(np.sum((np.array(actual) - np.array(predicted))**2, axis=1)))

    def calculate_individual_position_errors(self, actual, predicted):
        """
        Calculate individual position errors for each point
        """
        return [np.linalg.norm(np.array(actual[i]) - np.array(predicted[i])) for i in range(len(actual))]

    def calculate_orientation_error(self, actual, predicted):
        """
        Calculate average angular error (degrees)
        Input and output are in radians
        """
        actual = np.array(actual)
        predicted = np.array(predicted)

        # Calculate the angular difference for each axis
        diff = np.abs(actual - predicted)

        # Handle cases where the angular difference is greater than π
        diff = np.where(diff > np.pi, 2 * np.pi - diff, diff)

        # Calculate average angular error (degrees)
        mean_error_deg = np.rad2deg(np.mean(diff))  # Convert radians to degrees

        return mean_error_deg

    def calculate_and_print_errors(self, positions_B=None, orientations_B=None, transformed_positions=None, transformed_orientations=None):
        """
        Calculate and print position and orientation errors
        """
        results = {}
        
        if positions_B is not None and transformed_positions is not None:
            position_error = self.calculate_position_error(positions_B, transformed_positions)
            print(f"Position RMSE: {position_error:.4f} meters")
            results['position_rmse'] = position_error
            
            # Calculate individual position errors
            individual_errors = []
            for i in range(len(positions_B)):
                pos_error = np.linalg.norm(np.array(positions_B[i]) - np.array(transformed_positions[i]))
                print(f"Point {i} position error: {pos_error:.4f} meters")
                individual_errors.append(pos_error)
            
            results['individual_position_errors'] = individual_errors

        if orientations_B is not None and transformed_orientations is not None:
            orientation_error = self.calculate_orientation_error(orientations_B, transformed_orientations)
            print(f"Average orientation error: {orientation_error:.4f} degrees")
            results['orientation_error'] = orientation_error
            
            # Print each point's orientation error
            individual_ori_errors = []
            for i in range(len(orientations_B)):
                ori_error = self.calculate_orientation_error([orientations_B[i]], [transformed_orientations[i]])
                print(f"Point {i} orientation error: {ori_error:.4f} degrees")
                individual_ori_errors.append(ori_error)
            
            results['individual_orientation_errors'] = individual_ori_errors
        
        return results

    def demo_transformation(self):
        """
        Demonstrate coordinate transformation process, including creating coordinate systems,
        calculating transformations, and validating results.
        """
        if self.__orientations_A is None or self.__orientations_B is None:
            print("Please provide orientation data first!")
            return
        if self.__positions_A is None or self.__positions_B is None:
            print("Please provide position data first!")
            return

        # Calculate transformations
        self.calculate_position_transformation()
        self.calculate_orientation_transformation()

        # Print transformation matrices
        print("Position transformation matrix:")
        print(self.T_pos)

        print("Orientation transformation matrix:")
        print(self.R_euler)

        # Apply transformations
        transformed_positions = [self.apply_position_transformation(position) for position in self.__positions_A]
        transformed_orientations = [self.apply_orientation_transformation(orientation) for orientation in self.__orientations_A]

        # Validate and print results
        print("Verify transformation results:")
        for i in range(len(self.__positions_A)):
            print(f"Point A {i} actual position in coordinate system B: {self.__positions_B[i]}")
            print(f"Point A {i} transformed position in coordinate system B: {transformed_positions[i]}")
        for i in range(len(self.__orientations_A)):
            print(f"Point A {i} actual Euler angles in coordinate system B: {np.rad2deg(self.__orientations_B[i])}")
            print(f"Point A {i} transformed Euler angles in coordinate system B: {np.rad2deg(transformed_orientations[i])}")
            print()

        # Calculate and print errors
        error_results = self.calculate_and_print_errors(
            self.__positions_B, self.__orientations_B, transformed_positions, transformed_orientations
        )
        
        return transformed_positions, transformed_orientations, error_results

    def get_transformation_matrix(self):
        """
        Return transformation matrices
        Returns T_pos, R_euler in that order
        """
        return self.T_pos, self.R_euler
    
    def visualize_registration(self, error_threshold=0.05):
        """
        Visualize the registration results with error analysis
        
        Args:
            error_threshold: Threshold for highlighting high-error points
        """
        # Apply transformations to get transformed positions
        transformed_positions = [self.apply_position_transformation(position) for position in self.__positions_A]
        
        # Calculate individual errors
        errors = self.calculate_individual_position_errors(self.__positions_B, transformed_positions)
        
        # Filter points based on error threshold
        high_error_indices = [i for i, error in enumerate(errors) if error > error_threshold]
        low_error_indices = [i for i, error in enumerate(errors) if error <= error_threshold]
        
        # Create 3D plot
        fig = plt.figure(figsize=(12, 10))
        ax = fig.add_subplot(111, projection='3d')
        
        # Plot original source points (positions_A)
        ax.scatter([p[0] for p in self.__positions_A], [p[2] for p in self.__positions_A], [p[1] for p in self.__positions_A], 
                   c='blue', marker='o', s=50, label='Source (A)')
        
        # Plot target points (positions_B)
        ax.scatter([p[0] for p in self.__positions_B], [p[2] for p in self.__positions_B], [p[1] for p in self.__positions_B], 
                   c='red', marker='^', s=50, label='Target (B)')
        
        # Plot transformed positions
        ax.scatter([p[0] for p in transformed_positions], [p[2] for p in transformed_positions], [p[1] for p in transformed_positions], 
                   c='green', marker='x', s=50, label='Transformed A')
        
        # Highlight high error points
        if high_error_indices:
            high_error_positions_B = [self.__positions_B[i] for i in high_error_indices]
            high_error_transformed = [transformed_positions[i] for i in high_error_indices]
            
            ax.scatter([p[0] for p in high_error_positions_B], [p[2] for p in high_error_positions_B], [p[1] for p in high_error_positions_B], 
                       c='magenta', marker='o', s=100, label=f'Target points with error > {error_threshold}')
            
            ax.scatter([p[0] for p in high_error_transformed], [p[2] for p in high_error_transformed], [p[1] for p in high_error_transformed], 
                       c='orange', marker='x', s=100, label=f'Transformed points with error > {error_threshold}')
            
            # Draw lines between corresponding points with high error
            for i, idx in enumerate(high_error_indices):
                ax.plot([high_error_positions_B[i][0], high_error_transformed[i][0]],
                        [high_error_positions_B[i][2], high_error_transformed[i][2]],
                        [high_error_positions_B[i][1], high_error_transformed[i][1]],
                        'k--', alpha=0.3)
        
        ax.set_xlabel('X')
        ax.set_ylabel('Z')
        ax.set_zlabel('Y')
        ax.set_title('Registration Results with Error Analysis')
        ax.legend()
        
        # Save the figure
        plt.savefig(os.path.join(result_dir, 'registration_visualization.png'), dpi=300)
        plt.close()
        
        # Create a figure showing error distribution
        plt.figure(figsize=(10, 6))
        plt.hist(errors, bins=20, color='skyblue', edgecolor='black')
        plt.axvline(x=error_threshold, color='r', linestyle='--', label=f'Threshold ({error_threshold})')
        plt.xlabel('Position Error (m)')
        plt.ylabel('Frequency')
        plt.title('Distribution of Registration Errors')
        plt.legend()
        plt.savefig(os.path.join(result_dir, 'error_distribution.png'), dpi=300)
        plt.close()
        
        # Create a 2D scatter plot showing points in X-Z plane
        plt.figure(figsize=(12, 8))
        plt.scatter([p[0] for p in self.__positions_A], [p[2] for p in self.__positions_A], 
                    c='blue', marker='o', s=50, label='Source (A)')
        plt.scatter([p[0] for p in self.__positions_B], [p[2] for p in self.__positions_B], 
                    c='red', marker='^', s=50, label='Target (B)')
        plt.scatter([p[0] for p in transformed_positions], [p[2] for p in transformed_positions], 
                    c='green', marker='x', s=50, label='Transformed A')
        
        if high_error_indices:
            high_error_positions_B = [self.__positions_B[i] for i in high_error_indices]
            high_error_transformed = [transformed_positions[i] for i in high_error_indices]
            
            plt.scatter([p[0] for p in high_error_positions_B], [p[2] for p in high_error_positions_B], 
                        c='magenta', marker='o', s=100, label=f'Target points with error > {error_threshold}')
            plt.scatter([p[0] for p in high_error_transformed], [p[2] for p in high_error_transformed], 
                        c='orange', marker='x', s=100, label=f'Transformed points with error > {error_threshold}')
            
            for i, idx in enumerate(high_error_indices):
                plt.plot([high_error_positions_B[i][0], high_error_transformed[i][0]],
                        [high_error_positions_B[i][2], high_error_transformed[i][2]],
                        'k--', alpha=0.3)
        
        plt.xlabel('X')
        plt.ylabel('Y')
        plt.title('Registration Results in X-Y Plane')
        plt.legend()
        plt.grid(True)
        plt.axis('equal')
        plt.savefig(os.path.join(result_dir, 'registration_2d_view.png'), dpi=300)
        plt.close()
        
        return high_error_indices, low_error_indices, errors


def transform_point_pose(position, orientation, T_pos, R_euler):
    """
    Transform a point's position and orientation using given transformation matrices.

    Args:
        position: Original position, 3D vector [x, y, z].
        orientation: Original orientation, Euler angles [roll, pitch, yaw] in radians.
        T_pos: Position transformation matrix, 4x4 matrix.
        R_euler: Orientation transformation matrix, 3x3 matrix.

    Returns:
        transformed_position: Transformed position, 3D vector [x', y', z'].
        transformed_orientation: Transformed orientation, Euler angles [roll', pitch', yaw'] in radians.
    """
    # Convert position to homogeneous coordinates
    position_homogeneous = np.append(position, 1)

    # Apply position transformation
    transformed_position_homogeneous = T_pos @ position_homogeneous
    transformed_position = transformed_position_homogeneous[:3]

    # Convert Euler angles to rotation matrix
    R_point = R.from_euler('yxz', [orientation[2], orientation[0], orientation[1]]).as_matrix()

    # Apply orientation transformation
    R_transformed = R_euler @ R_point

    # Convert rotation matrix back to Euler angles
    transformed_orientation = R.from_matrix(R_transformed).as_euler('yxz')

    # Adjust Euler angle order
    transformed_orientation = [transformed_orientation[0], transformed_orientation[2], transformed_orientation[1]]

    return transformed_position, transformed_orientation


# Define orientation data (in this example using data from the original code)
orientations_A = [
    [np.radians(0), np.radians(-153.6356), np.radians(0)],  #1
    [np.radians(0), np.radians(116.3060), np.radians(0)],  #2
    [np.radians(0), np.radians(26.8196), np.radians(0)], #3
    [np.radians(0), np.radians(-64.2517), np.radians(0)],#4
    [np.radians(0), np.radians(72.2323), np.radians(0)], #5
    [np.radians(0), np.radians(-108.0868), np.radians(0)],#6
]

orientations_B = [
    [np.radians(0), np.radians(90.00), np.radians(0)],    #1
    [np.radians(0), np.radians(0.00), np.radians(0)],    #2
    [np.radians(0), np.radians(-90.00), np.radians(0)],      #3
    [np.radians(0), np.radians(180.00), np.radians(0)],     #4
    [np.radians(0), np.radians(-45.00), np.radians(0)],    #5
    [np.radians(0), np.radians(135.00), np.radians(0)],    #6
]

if __name__ == "__main__":
    # Create transformer and compute transformation
    transformer = CoordinateTransformer(orientations_A, orientations_B, positions_A, positions_B)

    # Get transformation matrices
    T_pos, R_euler = transformer.get_transformation_matrix()

    # Use demo_transformation method to demonstrate transformation process
    transformed_positions, transformed_orientations, error_results = transformer.demo_transformation()

    # Print transformation matrices
    print("Position transformation matrix:")
    print(T_pos)

    print("Orientation transformation matrix:")
    print(R_euler)

    # Visualize registration results and get high error indices
    error_threshold = 0.05
    high_error_indices, low_error_indices, errors = transformer.visualize_registration(error_threshold)

    # Print information about high error points
    if high_error_indices:
        print(f"\nPoints with error > {error_threshold}:")
        for idx in high_error_indices:
            print(f"Point {idx}: Error = {errors[idx]:.4f} meters")
            print(f"  Original position: {positions_A[idx]}")
            print(f"  Target position: {positions_B[idx]}")
            print(f"  Transformed position: {transformed_positions[idx]}")
            print()

    # Create filtered dataframe without high error points
    filtered_df = tracker_laser_df.iloc[low_error_indices].copy()

    # Save filtered data to CSV
    filtered_csv_path = os.path.join(result_dir, "filtered_tracker_log.csv")
    filtered_df.to_csv(filtered_csv_path, index=False)
    print(f"Filtered data saved to {filtered_csv_path}")

    # Create a data frame with error information
    error_df = pd.DataFrame({
        'point_index': range(len(errors)),
        'error': errors,
        'original_x': [p[0] for p in positions_A],
        'original_y': [p[1] for p in positions_A],
        'original_z': [p[2] for p in positions_A],
        'target_x': [p[0] for p in positions_B],
        'target_y': [p[1] for p in positions_B],
        'target_z': [p[2] for p in positions_B],
        'transformed_x': [p[0] for p in transformed_positions],
        'transformed_y': [p[1] for p in transformed_positions],
        'transformed_z': [p[2] for p in transformed_positions],
        'exceeds_threshold': [1 if e > error_threshold else 0 for e in errors]
    })

    # Save error analysis to CSV
    error_csv_path = os.path.join(result_dir, "registration_errors.csv")
    error_df.to_csv(error_csv_path, index=False)
    print(f"Error analysis saved to {error_csv_path}")

    # Test two non-benchmark points
    test_positions = [
        [0.7333, 0, -3.7766],  # real---[0.2070,0,-2.346]
        [-0.1066, 0, -4.3522]  # real---[0.1980,0,-1.337]
    ]
    test_orientation = [np.radians(0), np.radians(0), np.radians(0)]

    print("\nTesting transformation on non-benchmark points:")
    for i, position in enumerate(test_positions):
        transformed_position, transformed_orientation = transform_point_pose(position, test_orientation, T_pos, R_euler)
        print(f"Test point {i+1}:")
        print(f"  Original position: {position}")
        print(f"  Transformed position: {transformed_position}")

    # Save transformation matrices to files for future use
    np.savetxt(os.path.join(result_dir, "T_pos_matrix.txt"), T_pos)
    np.savetxt(os.path.join(result_dir, "R_euler_matrix.txt"), R_euler)
    print(f"Transformation matrices saved to {result_dir}")

    # Create summary report
    with open(os.path.join(result_dir, "registration_summary.txt"), "w") as f:
        f.write("Rigid Body Registration Summary\n")
        f.write("==============================\n\n")
        f.write(f"Total points: {len(errors)}\n")
        f.write(f"Points with error <= {error_threshold}: {len(low_error_indices)}\n")
        f.write(f"Points with error > {error_threshold}: {len(high_error_indices)}\n\n")
        f.write(f"Overall position RMSE: {error_results['position_rmse']:.4f} meters\n")
        if 'orientation_error' in error_results:
            f.write(f"Overall orientation error: {error_results['orientation_error']:.4f} degrees\n\n")
        f.write("Position Transformation Matrix:\n")
        f.write(str(T_pos) + "\n\n")
        f.write("Orientation Transformation Matrix:\n")
        f.write(str(R_euler) + "\n\n")
        
        if high_error_indices:
            f.write("High Error Points:\n")
            for idx in high_error_indices:
                f.write(f"Point {idx}: Error = {errors[idx]:.4f} meters\n")

    print(f"Summary report saved to {os.path.join(result_dir, 'registration_summary.txt')}")

    print("\nAll processing complete")