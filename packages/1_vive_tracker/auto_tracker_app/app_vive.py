#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Combined tracker pose recording and processing script with Flask web interface
"""
from flask import Flask, render_template, request, jsonify
import termios
import tty
import sys
import time
import struct
import serial
import threading
import queue
import os
import datetime
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from pynput import keyboard
from scipy.spatial.transform import Rotation as R
from werkzeug.serving import run_simple

try:
    from vive_tracker import ViveTrackerModule
except ImportError:
    print("Error: vive_tracker module not found. Please install it first.")
    sys.exit(1)

# Create Flask application
app = Flask(__name__)

# Disable werkzeug request logging
import logging
logging.getLogger('werkzeug').setLevel(logging.ERROR)

# Global variables to store system state
system_state = {
    "trackerInitialized": False,
    "serialConnected": False,
    "isRecording": False,
    "recordedDataPoints": 0,
    "eulerPoints": 0,
    "outputDirectory": "",
    "logMessages": []
}

# Global variables to store system instance
tracker_system = None
recording_thread = None

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)

# Custom log handler to send logs to system state
class WebLogHandler(logging.Handler):
    def emit(self, record):
        log_entry = self.format(record)
        log_type = record.levelname.lower()
        system_state["logMessages"].append({"message": log_entry, "type": log_type})

weblog_handler = WebLogHandler()
weblog_handler.setFormatter(logging.Formatter('%(levelname)s - %(message)s'))
logger.addHandler(weblog_handler)

# Global variables for file paths
output_dir = None
log_dir = None
data_dir = None
result_dir = None
img_dir = None
LASER_TRACKER_CSV = None
EULER_CSV = None
CORRECTED_LASER_TRACKER_CSV = None

def initialize_directories(output_dir_name):
    """Create required directories for the tracker system"""
    global output_dir, log_dir, data_dir, result_dir, img_dir
    global LASER_TRACKER_CSV, EULER_CSV, CORRECTED_LASER_TRACKER_CSV
    
    output_dir = output_dir_name
    os.makedirs(output_dir, exist_ok=True)
    
    # Create subdirectories
    log_dir = os.path.join(output_dir, "log")
    data_dir = os.path.join(output_dir, "data")
    result_dir = os.path.join(output_dir, "result") 
    img_dir = os.path.join(output_dir, "img")
    
    for directory in [log_dir, data_dir, result_dir, img_dir]:
        os.makedirs(directory, exist_ok=True)
        
    # Update file path variables
    LASER_TRACKER_CSV = os.path.join(data_dir, "laser_tracker.csv")
    EULER_CSV = os.path.join(data_dir, "euler.csv")
    CORRECTED_LASER_TRACKER_CSV = os.path.join(data_dir, "corrected_laser_tracker.csv")
    
    # Initialize euler.csv if it doesn't exist
    if not os.path.exists(EULER_CSV):
        with open(EULER_CSV, "w") as euler_file:
            euler_file.write("Timestamp,X,Y,Z,Roll,Yaw,Pitch,DesiredAngle\n")
    
    # Setup log file
    log_file = os.path.join(log_dir, "tracker_process.log")
    file_handler = logging.FileHandler(log_file)
    file_handler.setLevel(logging.INFO)
    file_formatter = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s')
    file_handler.setFormatter(file_formatter)
    logger.addHandler(file_handler)
    
    return log_dir, data_dir, result_dir, img_dir

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
        Returns R_pos and translation
        """
        if positions_A is not None and positions_B is not None:
            self.__positions_A = positions_A
            self.__positions_B = positions_B
        elif self.__positions_A is None or self.__positions_B is None:
            logger.error("Please provide position data first!")
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
            logger.error("Please provide orientation data first!")
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
            logger.info(f"Position RMSE: {position_error:.4f} meters")
            results['position_rmse'] = position_error
            
            # Calculate individual position errors
            individual_errors = []
            for i in range(len(positions_B)):
                pos_error = np.linalg.norm(np.array(positions_B[i]) - np.array(transformed_positions[i]))
                logger.info(f"Point {i} position error: {pos_error:.4f} meters")
                individual_errors.append(pos_error)
            
            results['individual_position_errors'] = individual_errors

        if orientations_B is not None and transformed_orientations is not None:
            orientation_error = self.calculate_orientation_error(orientations_B, transformed_orientations)
            logger.info(f"Average orientation error: {orientation_error:.4f} degrees")
            results['orientation_error'] = orientation_error
            
            # Print each point's orientation error
            individual_ori_errors = []
            for i in range(len(orientations_B)):
                ori_error = self.calculate_orientation_error([orientations_B[i]], [transformed_orientations[i]])
                logger.info(f"Point {i} orientation error: {ori_error:.4f} degrees")
                individual_ori_errors.append(ori_error)
            
            results['individual_orientation_errors'] = individual_ori_errors
        
        return results

    def demo_transformation(self):
        """
        Demonstrate coordinate transformation process, including creating coordinate systems,
        calculating transformations, and validating results.
        """
        if self.__orientations_A is None or self.__orientations_B is None:
            logger.error("Please provide orientation data first!")
            return None, None, None
        if self.__positions_A is None or self.__positions_B is None:
            logger.error("Please provide position data first!")
            return None, None, None

        # Calculate transformations
        self.calculate_position_transformation()
        self.calculate_orientation_transformation()

        # Print transformation matrices
        logger.info("Position transformation matrix:")
        logger.info(self.T_pos)

        logger.info("Orientation transformation matrix:")
        logger.info(self.R_euler)

        # Apply transformations
        transformed_positions = [self.apply_position_transformation(position) for position in self.__positions_A]
        transformed_orientations = [self.apply_orientation_transformation(orientation) for orientation in self.__orientations_A]

        # Validate and print results
        logger.info("Verify transformation results:")
        for i in range(len(self.__positions_A)):
            logger.info(f"Point A {i} actual position in coordinate system B: {self.__positions_B[i]}")
            logger.info(f"Point A {i} transformed position in coordinate system B: {transformed_positions[i]}")
        for i in range(len(self.__orientations_A)):
            logger.info(f"Point A {i} actual Euler angles in coordinate system B: {np.rad2deg(self.__orientations_B[i])}")
            logger.info(f"Point A {i} transformed Euler angles in coordinate system B: {np.rad2deg(transformed_orientations[i])}")
            logger.info("")

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
        if self.__positions_A is None or self.__positions_B is None:
            logger.error("Missing position data for visualization")
            return [], [], []
            
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
        plt.savefig(os.path.join(img_dir, 'registration_visualization.png'), dpi=300)
        plt.close()
        
        # Create a figure showing error distribution
        plt.figure(figsize=(10, 6))
        plt.hist(errors, bins=20, color='skyblue', edgecolor='black')
        plt.axvline(x=error_threshold, color='r', linestyle='--', label=f'Threshold ({error_threshold})')
        plt.xlabel('Position Error (m)')
        plt.ylabel('Frequency')
        plt.title('Distribution of Registration Errors')
        plt.legend()
        plt.savefig(os.path.join(img_dir, 'error_distribution.png'), dpi=300)
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
        plt.ylabel('Z')
        plt.title('Registration Results in X-Z Plane')
        plt.legend()
        plt.grid(True)
        plt.axis('equal')
        plt.savefig(os.path.join(img_dir, 'registration_2d_view.png'), dpi=300)
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

def calculate_checksum_r(data):
    """Calculate A69 checksum (XOR)"""
    return data[3] ^ data[4] ^ data[5] ^ data[6]

def parse_a69_data(response):
    """Parse A69 device data packet"""
    if len(response) != 10 or response[0] != 0x55 or response[1] != 0x7E or response[8] != 0x7E or response[9] != 0x55:
        logger.error(f"Invalid data packet: {list(map(hex, response))}")
        return None

    checksum = response[7]
    computed_checksum = calculate_checksum_r(response)

    if checksum != computed_checksum:
        logger.error(f"Checksum failed: Computed {hex(computed_checksum)}, Received {hex(checksum)}")
        return None

    distance_2 = (response[3] << 8) | response[4]
    distance_3 = (response[5] << 8) | response[6]

    return distance_2 / 1000.0, distance_3 / 1000.0  # Assume mm, convert to m

def send_a69_data(ser, flag, distance_2, distance_3):
    """Send data in A69 format"""
    distance_2_int = int(distance_2 * 1000)  # Convert to integer
    distance_3_int = int(distance_3 * 1000)

    tx_buf = bytearray(10)
    tx_buf[0] = 0x55
    tx_buf[1] = 0x7E
    tx_buf[2] = flag
    tx_buf[3] = (distance_2_int >> 8) & 0xFF  # High byte
    tx_buf[4] = distance_2_int & 0xFF         # Low byte
    tx_buf[5] = (distance_3_int >> 8) & 0xFF  # High byte
    tx_buf[6] = distance_3_int & 0xFF         # Low byte
    tx_buf[7] = tx_buf[3] ^ tx_buf[4] ^ tx_buf[5] ^ tx_buf[6]  # XOR checksum
    tx_buf[8] = 0x7E
    tx_buf[9] = 0x55

    ser.write(tx_buf)
    ser.flush()
    logger.info(f"Sent A69 data: {list(map(hex, tx_buf))}")

def send_a69_data_request(ser):
    """Send A69 format data request with correct checksum"""
    tx_buf = bytearray([0x55, 0x7E, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x7E, 0x55])
    
    # Calculate checksum
    tx_buf[7] = calculate_checksum_r(tx_buf)

    ser.write(tx_buf)
    ser.flush()
    logger.info(f"Sent A69 data request command: {hex(tx_buf[2])}")

def process_laser_tracker_data(input_file, output_file, angle_neg=122, angle_pos=55):
    """
    Process laser tracker data to correct measurements
    
    Args:
        input_file: Input CSV file path
        output_file: Output CSV file path
        angle_neg: Angle offset for negative yaw values (default: 122)
        angle_pos: Angle offset for positive yaw values (default: 55)
    """
    logger.info(f"Processing laser tracker data from {input_file}")
    logger.info(f"Using angle offsets: negative yaw: +{angle_neg}°, positive yaw: -{angle_pos}°")
    
    try:
        df = pd.read_csv(input_file)
        
        # Rename columns
        df.rename(columns={"Distance_2": "d_laser_z", "Distance_3": "d_laser_x"}, inplace=True)
        
        # Apply offsets
        df["d_laser_x"] = df["d_laser_x"] + 0.05
        df["d_laser_z"] = df["d_laser_z"] + 0.05
        
        # Calculate corrected distances
        def compute_corrected_distances(laser_x, laser_z, yaw):
            if yaw < 0:
                theta = np.radians(yaw + angle_neg)
            else:
                theta = np.radians(yaw - angle_pos)  # Calculate offset angle (in radians)
        
            # Rotation transformation, recover real vertical distance in 30° direction
            d_perp_x = laser_x * np.cos(theta)   # X direction corrected distance
            d_perp_z = laser_z * np.cos(theta)   # Z direction corrected distance
        
            return round(d_perp_x, 3), round(d_perp_z, 3)  # Keep 3 decimal places
        
        # Calculate corrected data
        corrected_data = [
            compute_corrected_distances(lx, lz, yaw)
            for lx, lz, yaw in zip(df["d_laser_x"], df["d_laser_z"], df["Yaw"])
        ]
        
        # Split calculation results
        df["laser_x"], df["laser_z"] = zip(*corrected_data)
        
        df["laser_x"] = 4.250 - df["laser_x"]
        df["laser_z"] = 1.660 - df["laser_z"]
        
        # Filter rows based on Yaw value
        condition1 = abs(df["Yaw"] - angle_pos) <= 10      # Filter data with yaw in angle_pos±10° range
        condition2 = abs(df["Yaw"] - (-angle_neg)) <= 10  # Filter data with yaw in -angle_neg±10° range
        
        df = df[condition1 | condition2]
        
        # Keep only needed columns and sort in required order
        df = df[["Timestamp", "d_laser_x", "d_laser_z", "laser_x", "laser_z", "X", "Y", "Z", "Yaw", "Roll", "Pitch"]]
        df = df.round(4)  # Keep 4 decimal places for all numeric columns
        
        # Save corrected data as CSV
        df.to_csv(output_file, index=False, encoding="utf-8")
        
        logger.info(f"Corrected data saved to {output_file}")
        logger.info(f"First few rows: \n{df.head()}")
        
        # Display rows where laser_x < 0
        negative_laser_x = df[df['laser_x'] < 0]
        if not negative_laser_x.empty:
            logger.info(f"Rows with negative laser_x: \n{negative_laser_x}")
            
        return df
        
    except Exception as e:
        logger.error(f"Error processing laser tracker data: {e}")
        return None

class TrackerSystem:
    def __init__(self, serial_port="/dev/ttyUSB0", baud_rate=9600, tracker_name="tracker_1"):
        self.serial_port = serial_port
        self.baud_rate = baud_rate
        self.tracker_name = tracker_name
        
        # Initialize Vive Tracker Module
        self.vtm = None
        self.tracker = None
        self.initialize_tracker()
        
        # Initialize serial port
        self.ser = None
        self.initialize_serial()
        
        # Thread-safe queue
        self.data_queue = queue.Queue()
        
        # Data storage
        self.recorded_data = []
        self.euler_data = []
        self.running = True
        self.recording = False  # Recording flag
        
        # Initialize euler.csv if it doesn't exist
        if not os.path.exists(EULER_CSV):
            with open(EULER_CSV, "w") as euler_file:
                euler_file.write("Timestamp,X,Y,Z,Roll,Yaw,Pitch,DesiredAngle\n")
                logger.info(f"Created new Euler data file: {EULER_CSV}")
        
        # Start serial listener thread
        self.serial_thread = threading.Thread(target=self.serial_listener, daemon=True)
        self.serial_thread.start()
        
    def initialize_tracker(self):
        """Initialize Vive Tracker"""
        try:
            self.vtm = ViveTrackerModule()
            self.vtm.print_discovered_objects()
            self.tracker = self.vtm.devices.get(self.tracker_name)
            
            if self.tracker is None:
                logger.error(f"Error: Tracker '{self.tracker_name}' not found!")
                raise Exception(f"Tracker '{self.tracker_name}' not found!")
            else:
                logger.info(f"Successfully initialized tracker: {self.tracker_name}")
        except Exception as e:
            logger.error(f"Error initializing tracker: {e}")
            raise
    
    def initialize_serial(self):
        """Initialize serial port"""
        try:
            self.ser = serial.Serial(self.serial_port, self.baud_rate, timeout=0.1)
            if self.ser.is_open:
                logger.info(f"Successfully opened serial port: {self.serial_port}")
        except serial.SerialException as e:
            logger.error(f"Error: Cannot open serial port {self.serial_port}: {e}")
            raise
    
    def get_tracker_pose(self):
        """Get current tracker pose"""
        return self.tracker.get_pose_euler()
    
    def serial_listener(self):
        """Serial port listener thread"""
        while self.running:
            if self.recording:
                send_a69_data_request(self.ser)  # Send 0x02 data request
                
                try:
                    response = self.ser.read(10)  # Read 10 bytes
                    parsed_data = parse_a69_data(response)
                    
                    if parsed_data:
                        distance_2, distance_3 = parsed_data
                        timestamp = time.time()
                        
                        logger.info(f"[Data] A69 Pose: distance_2={distance_2:.3f}, distance_3={distance_3:.3f}")
                        
                        # Request Vive Tracker pose
                        cam_coord = self.tracker.get_pose_euler()
                        logger.info(f"[Data] Vive Tracker Pose: x={cam_coord[0]:.4f}, y={cam_coord[1]:.4f}, z={cam_coord[2]:.4f}, "
                                   f"roll={cam_coord[3]:.4f}, yaw={cam_coord[4]:.4f}, pitch={cam_coord[5]:.4f}")
                        
                        # Store data
                        self.recorded_data.append((timestamp, distance_2, distance_3, cam_coord))
                        
                        # Update system state
                        system_state["recordedDataPoints"] = len(self.recorded_data)
                    else:
                        logger.warning("[Warning] No valid data received, resending request...")
                
                except serial.SerialException as e:
                    logger.error(f"[Error] Serial read failed: {e}")
            
            time.sleep(0.02)  # 20ms polling
            
    def calculate_transformations(self):
        """Calculate transformation matrices from recorded data"""
        try:
            # Load corrected tracker data
            tracker_laser_df = pd.read_csv(CORRECTED_LASER_TRACKER_CSV)
            logger.info("Successfully loaded corrected tracker data")
            
            # Load Euler data
            euler_df = pd.read_csv(EULER_CSV)
            logger.info("Successfully loaded Euler data")
            
            # Extract x, z (tracker position) and insert y=0
            positions_A = [[row["X"], 0, row["Z"]] for _, row in tracker_laser_df.iterrows()]
            
            # Extract laser_x, laser_z (laser position) and insert y=0
            positions_B = [[row["laser_x"], 0, row["laser_z"]] for _, row in tracker_laser_df.iterrows()]
            
            # Print sample data for verification
            logger.info("Extracted positions_A (x, 0, z):")
            for i in range(min(5, len(positions_A))):  # Only print first 5 data points
                logger.info(positions_A[i])
            
            logger.info("\nExtracted positions_B (laser_x, 0, laser_z):")
            for i in range(min(5, len(positions_B))):  # Only print first 5 data points
                logger.info(positions_B[i])
            
            # Process orientation data from Euler data
            orientations_A = []
            orientations_B = []
            
            for _, row in euler_df.iterrows():
                # Original tracker orientation
                orientations_A.append([
                    np.radians(row["Roll"]), 
                    np.radians(row["Yaw"]), 
                    np.radians(row["Pitch"])
                ])
                
                # Convert desired angle to orientation
                desired_angle = row["DesiredAngle"]
                orientations_B.append([
                    np.radians(0),
                    np.radians(desired_angle),
                    np.radians(0)
                ])
            
            logger.info(f"Loaded {len(orientations_A)} orientation points from Euler data")
            
            # Create transformer and compute transformation
            transformer = CoordinateTransformer(orientations_A, orientations_B, positions_A, positions_B)
            
            # Get transformation matrices
            T_pos, R_euler = transformer.get_transformation_matrix()
            
            # Use demo_transformation method to demonstrate transformation process
            transformed_positions, transformed_orientations, error_results = transformer.demo_transformation()
            
            # Print transformation matrices
            logger.info("Position transformation matrix (T_pos):")
            logger.info(T_pos)
            
            logger.info("Orientation transformation matrix (R_euler):")
            logger.info(R_euler)
            
            # Visualize registration results and get high error indices
            error_threshold = 0.05
            high_error_indices, low_error_indices, errors = transformer.visualize_registration(error_threshold)
            
            # Print information about high error points
            if high_error_indices:
                logger.info(f"\nPoints with error > {error_threshold}:")
                for idx in high_error_indices:
                    logger.info(f"Point {idx}: Error = {errors[idx]:.4f} meters")
                    logger.info(f"  Original position: {positions_A[idx]}")
                    logger.info(f"  Target position: {positions_B[idx]}")
                    logger.info(f"  Transformed position: {transformed_positions[idx]}")
                    logger.info("")
            
            # Create filtered dataframe without high error points
            filtered_df = tracker_laser_df.iloc[low_error_indices].copy()
            
            # Save filtered data to CSV
            filtered_csv_path = os.path.join(result_dir, "filtered_tracker_log.csv")
            filtered_df.to_csv(filtered_csv_path, index=False)
            logger.info(f"Filtered data saved to {filtered_csv_path}")
            
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
            logger.info(f"Error analysis saved to {error_csv_path}")
            
            # Test two non-benchmark points
            test_positions = [
                [0.7333, 0, -3.7766],  # real---[0.2070,0,-2.346]
                [-0.1066, 0, -4.3522]  # real---[0.1980,0,-1.337]
            ]
            test_orientation = [np.radians(0), np.radians(0), np.radians(0)]
            
            logger.info("\nTesting transformation on non-benchmark points:")
            for i, position in enumerate(test_positions):
                transformed_position, transformed_orientation = transform_point_pose(position, test_orientation, T_pos, R_euler)
                logger.info(f"Test point {i+1}:")
                logger.info(f"  Original position: {position}")
                logger.info(f"  Transformed position: {transformed_position}")
            
            # Save transformation matrices to files for future use
            np.savetxt(os.path.join(result_dir, "T_pos_matrix.txt"), T_pos)
            np.savetxt(os.path.join(result_dir, "R_euler_matrix.txt"), R_euler)
            logger.info(f"Transformation matrices saved to {result_dir}")
            
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
                f.write("Position Transformation Matrix (T_pos):\n")
                f.write(str(T_pos) + "\n\n")
                f.write("Orientation Transformation Matrix (R_euler):\n")
                f.write(str(R_euler) + "\n\n")
                
                if high_error_indices:
                    f.write("High Error Points:\n")
                    for idx in high_error_indices:
                        f.write(f"Point {idx}: Error = {errors[idx]:.4f} meters\n")
            
            logger.info(f"Summary report saved to {os.path.join(result_dir, 'registration_summary.txt')}")
            
            # Save the final transformation matrices in a more easily accessible format
            # This is the main output we want
            with open(os.path.join(output_dir, "transformation_matrices.txt"), "w") as f:
                f.write("Position Transformation Matrix (self.T_pos):\n")
                f.write(str(T_pos.tolist()) + "\n\n")
                f.write("Orientation Transformation Matrix (self.R_euler):\n")
                f.write(str(R_euler.tolist()) + "\n\n")
            
            logger.info(f"Final Transformation Matrices:")
            logger.info(f"T_pos = \n{T_pos}")
            logger.info(f"R_euler = \n{R_euler}")
            logger.info(f"Transformation matrices saved to {os.path.join(output_dir, 'transformation_matrices.txt')}")
            
            return True
            
        except Exception as e:
            logger.error(f"Error calculating transformations: {e}")
            import traceback
            logger.error(traceback.format_exc())
            return False

# Flask routes
@app.route('/')
def index():
    """Render main interface"""
    return render_template('index.html')

@app.route('/api/status')
def get_status():
    """Return current system status"""
    return jsonify(system_state)

@app.route('/api/initialize', methods=['POST'])
def initialize_tracker():
    """Initialize tracker"""
    global tracker_system, system_state
    
    if system_state["trackerInitialized"]:
        return jsonify({"success": False, "message": "Tracker already initialized"})

    data = request.json
    tracker_name = data.get('trackerName', 'tracker_1')
    serial_port = data.get('serialPort', '/dev/ttyUSB0')
    baud_rate = int(data.get('baudRate', 9600))
    
    logger.info(f"Initializing tracker system: {tracker_name} on port {serial_port} at {baud_rate} baud...")
    
    try:
        # Create timestamp directory
        current_time = datetime.datetime.now()
        timestamp = current_time.strftime("%Y-%B-%d-%a-%H-%M-%S")
        output_dir_name = f"tracker_data_process_{timestamp}"
        
        initialize_directories(output_dir_name)

        system_state["outputDirectory"] = output_dir_name
        
        # Create tracker system instance
        tracker_system = TrackerSystem(
            serial_port=serial_port, 
            baud_rate=baud_rate, 
            tracker_name=tracker_name
        )
        
        # Update status
        system_state["trackerInitialized"] = True
        system_state["serialConnected"] = True
        
        logger.info(f"Tracker '{tracker_name}' successfully initialized")
        logger.info(f"Output directory created: {output_dir_name}")
        
        return jsonify({"success": True, "message": "Tracker initialized successfully"})
    
    except Exception as e:
        logger.error(f"Error initializing tracker: {e}")
        return jsonify({"success": False, "message": f"Error: {str(e)}"})

@app.route('/api/start_recording', methods=['POST'])
def start_recording():
    """Start continuous data recording"""
    global tracker_system, system_state
    
    if not system_state["trackerInitialized"]:
        return jsonify({"success": False, "message": "Please initialize the tracker first"})
    
    try:
        # Set recording flag
        system_state["isRecording"] = True
        tracker_system.recording = True
        
        logger.info("Starting continuous recording (laser_tracker.csv)...")
        
        # Update frontend status
        return jsonify({"success": True, "message": "Recording started"})
    
    except Exception as e:
        logger.error(f"Error starting recording: {e}")
        return jsonify({"success": False, "message": f"Error: {str(e)}"})

@app.route('/api/stop_recording', methods=['POST'])
def stop_recording():
    """Stop continuous data recording"""
    global tracker_system, system_state
    
    if not system_state["isRecording"]:
        return jsonify({"success": False, "message": "Not currently recording"})
    
    try:
        # Stop recording
        system_state["isRecording"] = False
        if tracker_system:
            tracker_system.recording = False
        
        logger.info("Stopped recording")
        
        # Update recorded points count
        if hasattr(tracker_system, 'recorded_data'):
            system_state["recordedDataPoints"] = len(tracker_system.recorded_data)
        
        return jsonify({"success": True, "message": "Recording stopped"})
    
    except Exception as e:
        logger.error(f"Error stopping recording: {e}")
        return jsonify({"success": False, "message": f"Error: {str(e)}"})

@app.route('/api/record_euler', methods=['POST'])
def record_euler():
    """Record single Euler point data"""
    global tracker_system, system_state
    
    if not system_state["trackerInitialized"]:
        return jsonify({"success": False, "message": "Please initialize the tracker first"})
    
    data = request.json
    angle = float(data.get('angle', 0))
    
    try:
        # Request current Vive Tracker pose
        cam_coord = tracker_system.get_tracker_pose()
        timestamp = time.time()
        
        # Record Euler angle data
        tracker_system.euler_data.append((timestamp, cam_coord, angle))
        
        # Add to euler.csv
        with open(EULER_CSV, "a") as euler_file:
            euler_file.write(f"{timestamp:.3f},{cam_coord[0]:.4f},{cam_coord[1]:.4f},{cam_coord[2]:.4f},"
                             f"{cam_coord[3]:.4f},{cam_coord[4]:.4f},{cam_coord[5]:.4f},{angle:.2f}\n")
        
        logger.info(f"Euler data recorded: timestamp={timestamp:.3f}, "
                  f"x={cam_coord[0]:.4f}, y={cam_coord[1]:.4f}, z={cam_coord[2]:.4f}, "
                  f"roll={cam_coord[3]:.4f}, yaw={cam_coord[4]:.4f}, pitch={cam_coord[5]:.4f}, "
                  f"angle={angle:.2f}")
        
        # Update Euler points count
        system_state["eulerPoints"] += 1
        
        return jsonify({
            "success": True, 
            "message": "Euler point recorded",
            "data": {
                "timestamp": timestamp,
                "x": cam_coord[0],
                "y": cam_coord[1],
                "z": cam_coord[2],
                "roll": cam_coord[3],
                "yaw": cam_coord[4],
                "pitch": cam_coord[5],
                "angle": angle
            }
        })
    
    except Exception as e:
        logger.error(f"Error recording euler point: {e}")
        return jsonify({"success": False, "message": f"Error: {str(e)}"})

@app.route('/api/save_all_data', methods=['POST'])
def save_all_data():
    """Save all recorded data to files"""
    global tracker_system, system_state
    
    if not system_state["trackerInitialized"]:
        return jsonify({"success": False, "message": "Please initialize the tracker first"})
    
    try:
        logger.info("Saving all data to log files...")
        
        # Save laser tracker data
        if tracker_system.recorded_data:
            with open(LASER_TRACKER_CSV, "w") as log_file:
                log_file.write("Timestamp,Distance_2,Distance_3,X,Y,Z,Roll,Yaw,Pitch\n")
                for timestamp, dis2, dis3, coord in tracker_system.recorded_data:
                    log_entry = f"{timestamp:.3f},{dis2:.3f},{dis3:.3f},{coord[0]:.4f},{coord[1]:.4f},{coord[2]:.4f},{coord[3]:.4f},{coord[4]:.4f},{coord[5]:.4f}\n"
                    log_file.write(log_entry)
            logger.info(f"Laser tracker data saved to {LASER_TRACKER_CSV}")
            
            system_state["recordedDataPoints"] = len(tracker_system.recorded_data)
        else:
            logger.warning("No laser tracker data recorded.")
        
        return jsonify({
            "success": True, 
            "message": "All data saved successfully",
            "recordedPoints": system_state["recordedDataPoints"],
            "eulerPoints": system_state["eulerPoints"]
        })
    
    except Exception as e:
        logger.error(f"Error saving data: {e}")
        return jsonify({"success": False, "message": f"Error: {str(e)}"})

@app.route('/api/process_data', methods=['POST'])
def process_data():
    """Process laser tracker data"""
    data = request.json
    angle_neg = float(data.get('negativeYawOffset', 122))
    angle_pos = float(data.get('positiveYawOffset', 55))
    
    try:
        if os.path.exists(LASER_TRACKER_CSV):
            logger.info("Processing laser tracker data...")
            
            # Process laser tracker data
            processed_df = process_laser_tracker_data(
                LASER_TRACKER_CSV, 
                CORRECTED_LASER_TRACKER_CSV, 
                angle_neg, 
                angle_pos
            )
            
            if processed_df is not None:
                rows_count = len(processed_df)
                logger.info(f"Processed {rows_count} data points")
                return jsonify({
                    "success": True, 
                    "message": f"Data processed successfully. {rows_count} data points processed.",
                    "dataPoints": rows_count
                })
            else:
                return jsonify({"success": False, "message": "Error processing data"})
        else:
            return jsonify({"success": False, "message": f"Laser tracker data file {LASER_TRACKER_CSV} not found"})
    
    except Exception as e:
        logger.error(f"Error processing data: {e}")
        return jsonify({"success": False, "message": f"Error: {str(e)}"})

@app.route('/api/calculate_transformations', methods=['POST'])
def calculate_transformations():
    """Calculate coordinate transformation matrices"""
    try:
        if not os.path.exists(EULER_CSV) or not os.path.exists(CORRECTED_LASER_TRACKER_CSV):
            missing_files = []
            if not os.path.exists(EULER_CSV):
                missing_files.append("Euler data (euler.csv)")
            if not os.path.exists(CORRECTED_LASER_TRACKER_CSV):
                missing_files.append("Corrected laser tracker data (corrected_laser_tracker.csv)")
                
            return jsonify({
                "success": False, 
                "message": f"Missing required files: {', '.join(missing_files)}"
            })
        
        logger.info("Calculating transformation matrices...")
        
        # Call calculate_transformations method
        if tracker_system:
            success = tracker_system.calculate_transformations()
            
            if success:
                # Load matrices from results directory
                T_pos = None
                R_euler = None
                
                try:
                    T_pos = np.loadtxt(os.path.join(result_dir, "T_pos_matrix.txt"))
                    R_euler = np.loadtxt(os.path.join(result_dir, "R_euler_matrix.txt"))
                    
                    return jsonify({
                        "success": True,
                        "message": "Transformations calculated successfully",
                        "T_pos": T_pos.tolist() if T_pos is not None else None,
                        "R_euler": R_euler.tolist() if R_euler is not None else None
                    })
                except Exception as e:
                    return jsonify({
                        "success": False,
                        "message": f"Error loading transformation matrices: {str(e)}"
                    })
            else:
                return jsonify({
                    "success": False,
                    "message": "Error calculating transformations"
                })
        else:
            return jsonify({"success": False, "message": "Tracker system not initialized"})
    
    except Exception as e:
        logger.error(f"Error calculating transformations: {e}")
        return jsonify({"success": False, "message": f"Error: {str(e)}"})

@app.route('/api/visualize', methods=['POST'])
def visualize_data():
    """Visualize data points and transformation results"""
    data = request.json
    error_threshold = float(data.get('errorThreshold', 0.05))
    
    try:
        # Get visualization image file paths from result_dir
        visualization_files = {
            "registration_3d": os.path.join(img_dir, "registration_visualization.png"),
            "error_distribution": os.path.join(img_dir, "error_distribution.png"),
            "registration_2d": os.path.join(img_dir, "registration_2d_view.png")
        }
        
        # Check if files exist
        available_files = {}
        for key, path in visualization_files.items():
            if os.path.exists(path):
                # Convert to relative path
                relative_path = os.path.relpath(path, os.path.dirname(__file__))
                available_files[key] = relative_path
        
        return jsonify({
            "success": True,
            "message": "Visualization data retrieved",
            "visualizations": available_files
        })
    
    except Exception as e:
        logger.error(f"Error retrieving visualization data: {e}")
        return jsonify({"success": False, "message": f"Error: {str(e)}"})

@app.route('/api/get_errors', methods=['GET'])
def get_error_data():
    """Get error analysis data"""
    try:
        error_csv_path = os.path.join(result_dir, "registration_errors.csv")
        summary_path = os.path.join(result_dir, "registration_summary.txt")
        
        error_data = {}
        
        if os.path.exists(error_csv_path):
            # Read error CSV file
            error_df = pd.read_csv(error_csv_path)
            
            # Extract RMSE and high error points
            rmse = error_df['error'].mean()
            high_error_points = error_df[error_df['exceeds_threshold'] == 1]
            
            error_data["rmse"] = float(rmse)
            error_data["high_error_points"] = high_error_points.to_dict(orient="records")
        
        if os.path.exists(summary_path):
            # Read summary file
            with open(summary_path, 'r') as f:
                summary_text = f.read()
            error_data["summary"] = summary_text
        
        return jsonify({
            "success": True,
            "message": "Error data retrieved",
            "error_data": error_data
        })
    
    except Exception as e:
        logger.error(f"Error retrieving error data: {e}")
        return jsonify({"success": False, "message": f"Error: {str(e)}"})

@app.route('/api/modify_euler', methods=['POST'])
def modify_euler():
    """Modify Euler point data angle"""
    data = request.json
    record_id = data.get('recordId')
    new_angle = float(data.get('newAngle', 0))
    
    try:
        # Open and read Euler CSV file
        euler_df = pd.read_csv(EULER_CSV)
        
        # Modify corresponding record's angle
        if 0 <= record_id < len(euler_df):
            euler_df.at[record_id, 'DesiredAngle'] = new_angle
            
            # Save modified file
            euler_df.to_csv(EULER_CSV, index=False)
            
            logger.info(f"Modified Euler record {record_id} angle to {new_angle}")
            
            return jsonify({
                "success": True, 
                "message": f"Angle for record at index {record_id} modified to {new_angle}."
            })
        else:
            return jsonify({"success": False, "message": f"Invalid record ID: {record_id}"})
            
    except Exception as e:
        logger.error(f"Error modifying Euler record: {e}")
        return jsonify({"success": False, "message": f"Error: {str(e)}"})

@app.route('/api/delete_euler', methods=['POST'])
def delete_euler():
    """Delete Euler point data"""
    data = request.json
    record_id = data.get('recordId')
    
    try:
        # Open and read Euler CSV file
        euler_df = pd.read_csv(EULER_CSV)
        
        # Delete corresponding record
        if 0 <= record_id < len(euler_df):
            euler_df = euler_df.drop(record_id).reset_index(drop=True)
            
            # Save modified file
            euler_df.to_csv(EULER_CSV, index=False)
            
            logger.info(f"Deleted Euler record {record_id}")
            
            return jsonify({
                "success": True, 
                "message": f"Record at index {record_id} deleted."
            })
        else:
            return jsonify({"success": False, "message": f"Invalid record ID: {record_id}"})
            
    except Exception as e:
        logger.error(f"Error deleting Euler record: {e}")
        return jsonify({"success": False, "message": f"Error: {str(e)}"})

@app.route('/api/get_euler_records', methods=['GET'])
def get_euler_records():
    """Get all Euler point data"""
    try:
        # Check if Euler CSV file exists
        if not os.path.exists(EULER_CSV):
            return jsonify({
                "success": False, 
                "message": "No Euler data file found."
            })
        
        # Open and read Euler CSV file
        euler_df = pd.read_csv(EULER_CSV)
        
        # Convert to JSON format
        records = []
        for idx, row in euler_df.iterrows():
            records.append({
                "id": idx,
                "timestamp": row['Timestamp'],
                "angle": row['DesiredAngle'],
                "position": {
                    "x": row['X'],
                    "y": row['Y'],
                    "z": row['Z']
                },
                "orientation": {
                    "roll": row['Roll'],
                    "yaw": row['Yaw'],
                    "pitch": row['Pitch']
                }
            })
        
        return jsonify({
            "success": True,
            "records": records
        })
            
    except Exception as e:
        logger.error(f"Error retrieving Euler records: {e}")
        return jsonify({"success": False, "message": f"Error: {str(e)}"})

@app.route('/api/restart', methods=['POST'])
def restart_application():
    """Restart application"""
    try:
        logger.info("Restarting application...")
        
        # Create a function to restart after a brief delay
        def restart():
            time.sleep(1)  # Give client time to receive response
            os.execv(sys.executable, [sys.executable] + sys.argv)
        
        # Execute restart in new thread to allow response to be sent
        threading.Thread(target=restart).start()
        
        return jsonify({
            "success": True,
            "message": "Application restarting..."
        })
    
    except Exception as e:
        logger.error(f"Error restarting application: {e}")
        return jsonify({"success": False, "message": f"Error: {str(e)}"})

@app.route('/api/exit', methods=['POST'])
def exit_application():
    """Close application"""
    try:
        logger.info("Shutting down application...")
        
        # Create a function to shutdown after a brief delay
        def shutdown():
            time.sleep(1)  # Give client time to receive response
            os._exit(0)  # Force exit
        
        # Execute shutdown in new thread to allow response to be sent
        threading.Thread(target=shutdown).start()
        
        return jsonify({
            "success": True,
            "message": "Application shutting down..."
        })
    
    except Exception as e:
        logger.error(f"Error shutting down application: {e}")
        return jsonify({"success": False, "message": f"Error: {str(e)}"})

if __name__ == '__main__':
    # Run Flask application
    port = 5000
    print(f"Starting Auto Tracker Data Process web interface on http://127.0.0.1:{port}/")
    run_simple('127.0.0.1', port, app, use_reloader=True, use_debugger=True)