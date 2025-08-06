"""
Advanced Data Processing Script
Philosophy: Transform raw data into insights
"""
import numpy as np
import matplotlib.pyplot as plt
import plotly.graph_objects as go
import plotly.express as px
from plotly.subplots import make_subplots
from scipy.signal import savgol_filter
import pandas as pd
import os
import json
import argparse
from datetime import datetime
import warnings
warnings.filterwarnings('ignore')

class DataProcessor:
    def __init__(self, raw_data_file, config_file="process_config.json"):
        self.raw_data_file = raw_data_file
        self.config = self.load_config(config_file)
        self.results = {}
        
        # Load and validate raw data
        self.load_raw_data()
        
    def load_config(self, config_file):
        """Load processing configuration"""
        default_config = {
            "synchronization": {
                "interpolation_tolerance_ms": 100,
                "outlier_detection": True,
                "outlier_threshold_sigma": 3.0
            },
            "segmentation": {
                "max_time_gap_ms": 150,
                "min_segment_length": 10
            },
            "filtering": {
                "sg_window_length": 7,
                "sg_polyorder": 2,
                "auto_optimize_parameters": True
            },
            "visualization": {
                "generate_interactive_plots": True,
                "generate_static_plots": True,
                "plot_3d_trajectories": True,
                "show_segment_boundaries": True
            },
            "output": {
                "save_synchronized_csv": True,
                "save_filtered_csv": True,
                "output_dir": "processed_data"
            }
        }
        
        try:
            with open(config_file, 'r') as f:
                user_config = json.load(f)
            # Deep merge configurations
            def deep_update(base, update):
                for key, value in update.items():
                    if key in base and isinstance(base[key], dict) and isinstance(value, dict):
                        deep_update(base[key], value)
                    else:
                        base[key] = value
            deep_update(default_config, user_config)
        except FileNotFoundError:
            print(f"Config file {config_file} not found, using defaults")
            with open(config_file, 'w') as f:
                json.dump(default_config, f, indent=2)
        
        return default_config
    
    def load_raw_data(self):
        """Load and validate raw data from NPZ file"""
        print(f"Loading raw data from: {self.raw_data_file}")
        
        try:
            data = np.load(self.raw_data_file, allow_pickle=True)
            self.lidar_data = data['lidar_data']
            self.tracker_data = data['tracker_data']
            self.metadata = data['metadata'].item()
            
            print(f"Loaded {len(self.lidar_data)} LiDAR samples")
            print(f"Loaded {len(self.tracker_data)} Tracker samples")
            print(f"Collection duration: {self.metadata.get('duration_seconds', 0):.1f}s")
            
            # Validate data quality
            self.validate_raw_data()
            
        except Exception as e:
            print(f"Error loading raw data: {e}")
            raise
    
    def validate_raw_data(self):
        """Validate raw data quality and print diagnostics"""
        print("\n=== Raw Data Quality Report ===")
        
        # Time span validation
        lidar_timespan = self.lidar_data['timestamp'][-1] - self.lidar_data['timestamp'][0]
        tracker_timespan = self.tracker_data['timestamp'][-1] - self.tracker_data['timestamp'][0]
        
        print(f"LiDAR time span: {lidar_timespan:.1f}s")
        print(f"Tracker time span: {tracker_timespan:.1f}s")
        print(f"Time span difference: {abs(lidar_timespan - tracker_timespan):.1f}s")
        
        # Sampling rate analysis
        lidar_intervals = np.diff(self.lidar_data['timestamp'])
        tracker_intervals = np.diff(self.tracker_data['timestamp'])
        
        print(f"LiDAR mean interval: {np.mean(lidar_intervals)*1000:.1f}ms (std: {np.std(lidar_intervals)*1000:.1f}ms)")
        print(f"Tracker mean interval: {np.mean(tracker_intervals)*1000:.1f}ms (std: {np.std(tracker_intervals)*1000:.1f}ms)")
        
        # Latency analysis
        print(f"LiDAR mean latency: {np.mean(self.lidar_data['latency_ms']):.1f}ms")
        print(f"Tracker mean latency: {np.mean(self.tracker_data['latency_ms']):.1f}ms")
        
        # Data overlap check
        lidar_start, lidar_end = self.lidar_data['timestamp'][0], self.lidar_data['timestamp'][-1]
        tracker_start, tracker_end = self.tracker_data['timestamp'][0], self.tracker_data['timestamp'][-1]
        
        overlap_start = max(lidar_start, tracker_start)
        overlap_end = min(lidar_end, tracker_end)
        overlap_duration = max(0, overlap_end - overlap_start)
        
        print(f"Data overlap duration: {overlap_duration:.1f}s")
        
        if overlap_duration < 5.0:
            print("WARNING: Very short data overlap, synchronization may be poor")
        elif overlap_duration < 0.1:
            print("ERROR: No data overlap, synchronization impossible")
            raise ValueError("Insufficient data overlap for synchronization")
        else:
            print("Sufficient data overlap for synchronization")
        
        print("===============================\n")
    
    def synchronize_data(self):
        """Synchronize LiDAR and Tracker data using interpolation"""
        print("Synchronizing data...")
        
        # Extract time arrays
        lidar_times = self.lidar_data['timestamp']
        tracker_times = self.tracker_data['timestamp']
        
        # Find overlap region
        start_time = max(lidar_times[0], tracker_times[0])
        end_time = min(lidar_times[-1], tracker_times[-1])
        
        # Filter LiDAR data to overlap region
        lidar_mask = (lidar_times >= start_time) & (lidar_times <= end_time)
        lidar_subset = self.lidar_data[lidar_mask]
        
        synchronized_data = []
        interpolation_stats = {'success': 0, 'failed': 0, 'outliers_removed': 0}
        tolerance = self.config["synchronization"]["interpolation_tolerance_ms"] / 1000.0
        
        for lidar_point in lidar_subset:
            t_lidar = lidar_point['timestamp']
            
            # Find closest tracker points for interpolation
            time_diffs = np.abs(tracker_times - t_lidar)
            closest_idx = np.argmin(time_diffs)
            
            # Check if we can interpolate
            if closest_idx == 0 or closest_idx == len(tracker_times) - 1:
                interpolation_stats['failed'] += 1
                continue
            
            # Determine interpolation points
            if tracker_times[closest_idx] <= t_lidar:
                idx_before, idx_after = closest_idx, closest_idx + 1
            else:
                idx_before, idx_after = closest_idx - 1, closest_idx
            
            t_before = tracker_times[idx_before]
            t_after = tracker_times[idx_after]
            
            # Check interpolation tolerance
            if (t_lidar - t_before) > tolerance or (t_after - t_lidar) > tolerance:
                interpolation_stats['failed'] += 1
                continue
            
            # Linear interpolation
            alpha = (t_lidar - t_before) / (t_after - t_before) if t_after != t_before else 0
            
            # Interpolate all tracker components
            tracker_before = self.tracker_data[idx_before]
            tracker_after = self.tracker_data[idx_after]
            
            interpolated_pose = {}
            for field in ['x', 'y', 'z', 'roll', 'pitch', 'yaw']:
                interpolated_pose[field] = tracker_before[field] + alpha * (tracker_after[field] - tracker_before[field])
            
            # Outlier detection (optional)
            if self.config["synchronization"]["outlier_detection"]:
                # Simple outlier detection based on distance values
                distances = [lidar_point['distance_2'], lidar_point['distance_3']]
                if self.is_outlier(distances):
                    interpolation_stats['outliers_removed'] += 1
                    continue
            
            # Store synchronized point
            sync_point = {
                'timestamp': t_lidar,
                'distance_2': lidar_point['distance_2'],
                'distance_3': lidar_point['distance_3'],
                'x': interpolated_pose['x'],
                'y': interpolated_pose['y'],
                'z': interpolated_pose['z'],
                'roll': interpolated_pose['roll'],
                'pitch': interpolated_pose['pitch'],
                'yaw': interpolated_pose['yaw']
            }
            synchronized_data.append(sync_point)
            interpolation_stats['success'] += 1
        
        success_rate = interpolation_stats['success'] / (interpolation_stats['success'] + interpolation_stats['failed']) * 100 if (interpolation_stats['success'] + interpolation_stats['failed']) > 0 else 0
        
        print(f"Synchronization completed:")
        print(f"  Success: {interpolation_stats['success']} points")
        print(f"  Failed: {interpolation_stats['failed']} points")
        print(f"  Outliers removed: {interpolation_stats['outliers_removed']} points")
        print(f"  Success rate: {success_rate:.1f}%")
        
        self.synchronized_data = synchronized_data
        self.results['synchronization_stats'] = interpolation_stats
        return synchronized_data
    
    def is_outlier(self, values):
        """Simple outlier detection using z-score"""
        threshold = self.config["synchronization"]["outlier_threshold_sigma"]
        if len(values) < 2:
            return False
        
        mean_val = np.mean(values)
        std_val = np.std(values)
        
        if std_val == 0:
            return False
        
        z_scores = [(val - mean_val) / std_val for val in values]
        return any(abs(z) > threshold for z in z_scores)
    
    def segment_data(self, data):
        """Segment data based on time gaps"""
        print("Segmenting data...")
        
        if len(data) < 2:
            return [data] if data else []
        
        segments = []
        current_segment = [data[0]]
        gap_threshold = self.config["segmentation"]["max_time_gap_ms"] / 1000.0
        min_length = self.config["segmentation"]["min_segment_length"]
        
        for i in range(1, len(data)):
            time_gap = data[i]['timestamp'] - data[i-1]['timestamp']
            
            if time_gap > gap_threshold:
                # End current segment if it's long enough
                if len(current_segment) >= min_length:
                    segments.append(current_segment)
                    print(f"  Segment {len(segments)}: {len(current_segment)} points, duration {current_segment[-1]['timestamp'] - current_segment[0]['timestamp']:.1f}s")
                
                # Start new segment
                current_segment = [data[i]]
            else:
                current_segment.append(data[i])
        
        # Add final segment
        if len(current_segment) >= min_length:
            segments.append(current_segment)
            print(f"  Segment {len(segments)}: {len(current_segment)} points, duration {current_segment[-1]['timestamp'] - current_segment[0]['timestamp']:.1f}s")
        
        print(f"Created {len(segments)} segments")
        self.results['segment_count'] = len(segments)
        return segments
    
    def optimize_sg_parameters(self, segment_data):
        """Automatically optimize SG filter parameters for a data segment"""
        if len(segment_data) < 20:
            return self.config["filtering"]["sg_window_length"], self.config["filtering"]["sg_polyorder"]
        
        # Extract distance data for optimization
        d2_data = np.array([point['distance_2'] for point in segment_data])
        
        best_params = None
        best_score = float('inf')
        
        # Test different parameter combinations
        window_lengths = [5, 7, 9, 11, 13, 15]
        poly_orders = [2, 3, 4]
        
        for window in window_lengths:
            if window >= len(segment_data):
                continue
            for poly in poly_orders:
                if poly >= window:
                    continue
                
                try:
                    # Apply filter
                    filtered = savgol_filter(d2_data, window, poly)
                    
                    # Calculate quality score (residual variance)
                    residuals = d2_data - filtered
                    score = np.var(residuals)
                    
                    if score < best_score:
                        best_score = score
                        best_params = (window, poly)
                
                except:
                    continue
        
        if best_params is None:
            return self.config["filtering"]["sg_window_length"], self.config["filtering"]["sg_polyorder"]
        
        return best_params
    
    def apply_sg_filtering(self, segments):
        """Apply Savitzky-Golay filtering to each segment"""
        print("Applying Savitzky-Golay filtering...")
        
        filtered_segments = []
        filter_stats = []
        
        for seg_idx, segment in enumerate(segments):
            if len(segment) < 5:
                print(f"  Segment {seg_idx}: Too short ({len(segment)} points), skipping")
                continue
            
            # Optimize parameters if requested
            if self.config["filtering"]["auto_optimize_parameters"]:
                window_length, polyorder = self.optimize_sg_parameters(segment)
                print(f"  Segment {seg_idx}: Optimized parameters - window={window_length}, poly={polyorder}")
            else:
                window_length = self.config["filtering"]["sg_window_length"]
                polyorder = self.config["filtering"]["sg_polyorder"]
            
            # Ensure parameters are valid
            if window_length >= len(segment):
                window_length = len(segment) - 1 if len(segment) % 2 == 0 else len(segment) - 2
            if window_length % 2 == 0:
                window_length -= 1
            if polyorder >= window_length:
                polyorder = window_length - 1
            
            if window_length < 3 or polyorder < 1:
                print(f"  Segment {seg_idx}: Invalid parameters after adjustment, skipping")
                continue
            
            # Extract data arrays
            timestamps = np.array([point['timestamp'] for point in segment])
            d2_raw = np.array([point['distance_2'] for point in segment])
            d3_raw = np.array([point['distance_3'] for point in segment])
            poses_raw = np.array([[point['x'], point['y'], point['z'], 
                                 point['roll'], point['pitch'], point['yaw']] for point in segment])
            
            try:
                # Apply SG filter
                d2_filtered = savgol_filter(d2_raw, window_length, polyorder)
                d3_filtered = savgol_filter(d3_raw, window_length, polyorder)
                
                # Filter pose data
                poses_filtered = np.zeros_like(poses_raw)
                for i in range(poses_raw.shape[1]):
                    poses_filtered[:, i] = savgol_filter(poses_raw[:, i], window_length, polyorder)
                
                # Calculate filtering statistics
                d2_improvement = (np.std(d2_raw) - np.std(d2_filtered)) / np.std(d2_raw) * 100 if np.std(d2_raw) > 0 else 0
                d3_improvement = (np.std(d3_raw) - np.std(d3_filtered)) / np.std(d3_raw) * 100 if np.std(d3_raw) > 0 else 0
                
                stats = {
                    'segment_id': seg_idx,
                    'points': len(segment),
                    'window_length': window_length,
                    'polyorder': polyorder,
                    'd2_std_improvement': d2_improvement,
                    'd3_std_improvement': d3_improvement
                }
                filter_stats.append(stats)
                
                # Reconstruct filtered segment
                filtered_segment = []
                for i in range(len(segment)):
                    filtered_point = {
                        'timestamp': timestamps[i],
                        'distance_2': d2_filtered[i],
                        'distance_3': d3_filtered[i],
                        'x': poses_filtered[i, 0],
                        'y': poses_filtered[i, 1],
                        'z': poses_filtered[i, 2],
                        'roll': poses_filtered[i, 3],
                        'pitch': poses_filtered[i, 4],
                        'yaw': poses_filtered[i, 5],
                        'segment_id': seg_idx
                    }
                    filtered_segment.append(filtered_point)
                
                filtered_segments.append(filtered_segment)
                print(f"  Segment {seg_idx}: Filtered {len(filtered_segment)} points, D2 improvement: {d2_improvement:.1f}%, D3 improvement: {d3_improvement:.1f}%")
                
            except Exception as e:
                print(f"  Segment {seg_idx}: Filtering failed - {e}")
                continue
        
        print(f"Filtering completed on {len(filtered_segments)} segments")
        self.results['filtering_stats'] = filter_stats
        return filtered_segments
    
    def generate_static_plots(self, synchronized_data, filtered_segments):
        """Generate comprehensive static plots"""
        print("Generating static plots...")
        
        output_dir = self.config["output"]["output_dir"]
        os.makedirs(output_dir, exist_ok=True)
        
        # Flatten filtered data
        filtered_data = []
        for segment in filtered_segments:
            filtered_data.extend(segment)
        
        if not filtered_data:
            print("No filtered data available for plotting")
            return
        
        # Create comprehensive figure
        fig, axes = plt.subplots(3, 3, figsize=(20, 15))
        fig.suptitle('Advanced Data Processing Results', fontsize=16, fontweight='bold')
        
        # Extract data arrays
        sync_times = np.array([p['timestamp'] for p in synchronized_data])
        sync_d2 = np.array([p['distance_2'] for p in synchronized_data])
        sync_d3 = np.array([p['distance_3'] for p in synchronized_data])
        sync_x = np.array([p['x'] for p in synchronized_data])
        sync_y = np.array([p['y'] for p in synchronized_data])
        sync_z = np.array([p['z'] for p in synchronized_data])
        
        filt_times = np.array([p['timestamp'] for p in filtered_data])
        filt_d2 = np.array([p['distance_2'] for p in filtered_data])
        filt_d3 = np.array([p['distance_3'] for p in filtered_data])
        filt_x = np.array([p['x'] for p in filtered_data])
        filt_y = np.array([p['y'] for p in filtered_data])
        filt_z = np.array([p['z'] for p in filtered_data])
        
        # Normalize time to start from 0
        sync_times_norm = sync_times - sync_times[0]
        filt_times_norm = filt_times - filt_times[0]
        
        # Plot 1: Distance 2 comparison
        axes[0, 0].plot(sync_times_norm, sync_d2 * 1000, 'b-', alpha=0.6, linewidth=1, label='Synchronized')
        axes[0, 0].plot(filt_times_norm, filt_d2 * 1000, 'r-', linewidth=2, label='SG Filtered')
        axes[0, 0].set_title('Distance Channel 2')
        axes[0, 0].set_ylabel('Distance (mm)')
        axes[0, 0].legend()
        axes[0, 0].grid(True, alpha=0.3)
        
        # Plot 2: Distance 3 comparison
        axes[0, 1].plot(sync_times_norm, sync_d3 * 1000, 'b-', alpha=0.6, linewidth=1, label='Synchronized')
        axes[0, 1].plot(filt_times_norm, filt_d3 * 1000, 'r-', linewidth=2, label='SG Filtered')
        axes[0, 1].set_title('Distance Channel 3')
        axes[0, 1].set_ylabel('Distance (mm)')
        axes[0, 1].legend()
        axes[0, 1].grid(True, alpha=0.3)
        
        # Plot 3: 3D trajectory (X-Y view)
        if self.config["visualization"]["plot_3d_trajectories"]:
            axes[0, 2].plot(sync_x * 1000, sync_y * 1000, 'b-', alpha=0.6, linewidth=1, label='Synchronized')
            axes[0, 2].plot(filt_x * 1000, filt_y * 1000, 'r-', linewidth=2, label='SG Filtered')
            axes[0, 2].set_title('Trajectory (X-Y View)')
            axes[0, 2].set_xlabel('X (mm)')
            axes[0, 2].set_ylabel('Y (mm)')
            axes[0, 2].legend()
            axes[0, 2].grid(True, alpha=0.3)
            axes[0, 2].axis('equal')
        
        # Plot 4: X coordinate
        axes[1, 0].plot(sync_times_norm, sync_x * 1000, 'b-', alpha=0.6, linewidth=1, label='Synchronized')
        axes[1, 0].plot(filt_times_norm, filt_x * 1000, 'r-', linewidth=2, label='SG Filtered')
        axes[1, 0].set_title('X Coordinate')
        axes[1, 0].set_ylabel('X (mm)')
        axes[1, 0].legend()
        axes[1, 0].grid(True, alpha=0.3)
        
        # Plot 5: Y coordinate
        axes[1, 1].plot(sync_times_norm, sync_y * 1000, 'b-', alpha=0.6, linewidth=1, label='Synchronized')
        axes[1, 1].plot(filt_times_norm, filt_y * 1000, 'r-', linewidth=2, label='SG Filtered')
        axes[1, 1].set_title('Y Coordinate')
        axes[1, 1].set_ylabel('Y (mm)')
        axes[1, 1].legend()
        axes[1, 1].grid(True, alpha=0.3)
        
        # Plot 6: Z coordinate
        axes[1, 2].plot(sync_times_norm, sync_z * 1000, 'b-', alpha=0.6, linewidth=1, label='Synchronized')
        axes[1, 2].plot(filt_times_norm, filt_z * 1000, 'r-', linewidth=2, label='SG Filtered')
        axes[1, 2].set_title('Z Coordinate')
        axes[1, 2].set_ylabel('Z (mm)')
        axes[1, 2].legend()
        axes[1, 2].grid(True, alpha=0.3)
        
        # Plot 7-9: Residual analysis (if same length)
        if len(sync_d2) == len(filt_d2):
            residuals_d2 = (sync_d2 - filt_d2) * 1000
            residuals_d3 = (sync_d3 - filt_d3) * 1000
            residuals_x = (sync_x - filt_x) * 1000
            
            axes[2, 0].plot(sync_times_norm, residuals_d2, 'g-', alpha=0.7, linewidth=1)
            axes[2, 0].axhline(y=0, color='k', linestyle='--', alpha=0.5)
            axes[2, 0].set_title('D2 Filtering Residuals')
            axes[2, 0].set_ylabel('Residual (mm)')
            axes[2, 0].set_xlabel('Time (s)')
            axes[2, 0].grid(True, alpha=0.3)
            
            axes[2, 1].plot(sync_times_norm, residuals_d3, 'g-', alpha=0.7, linewidth=1)
            axes[2, 1].axhline(y=0, color='k', linestyle='--', alpha=0.5)
            axes[2, 1].set_title('D3 Filtering Residuals')
            axes[2, 1].set_ylabel('Residual (mm)')
            axes[2, 1].set_xlabel('Time (s)')
            axes[2, 1].grid(True, alpha=0.3)
            
            axes[2, 2].plot(sync_times_norm, residuals_x, 'g-', alpha=0.7, linewidth=1)
            axes[2, 2].axhline(y=0, color='k', linestyle='--', alpha=0.5)
            axes[2, 2].set_title('X Coordinate Residuals')
            axes[2, 2].set_ylabel('Residual (mm)')
            axes[2, 2].set_xlabel('Time (s)')
            axes[2, 2].grid(True, alpha=0.3)
        
        plt.tight_layout()
        
        # Save plot
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        plot_file = os.path.join(output_dir, f"processing_results_{timestamp}.png")
        plt.savefig(plot_file, dpi=300, bbox_inches='tight')
        plt.close()
        
        print(f"Static plots saved to: {plot_file}")
        return plot_file
    
    def generate_interactive_plots(self, synchronized_data, filtered_segments):
        """Generate interactive plots using Plotly"""
        print("Generating interactive plots...")
        
        output_dir = self.config["output"]["output_dir"]
        os.makedirs(output_dir, exist_ok=True)
        
        # Flatten filtered data with segment information
        filtered_data = []
        for segment in filtered_segments:
            filtered_data.extend(segment)
        
        if not filtered_data:
            print("No filtered data available for interactive plotting")
            return
        
        # Convert to DataFrames for easier plotting
        sync_df = pd.DataFrame(synchronized_data)
        sync_df['data_type'] = 'Synchronized'
        
        filt_df = pd.DataFrame(filtered_data)
        filt_df['data_type'] = 'SG Filtered'
        
        # Normalize timestamps
        start_time = min(sync_df['timestamp'].min(), filt_df['timestamp'].min())
        sync_df['time_norm'] = sync_df['timestamp'] - start_time
        filt_df['time_norm'] = filt_df['timestamp'] - start_time
        
        # Create subplots
        fig = make_subplots(
            rows=3, cols=2,
            subplot_titles=('Distance 2', 'Distance 3', 'X Coordinate', 'Y Coordinate', 'Z Coordinate', '3D Trajectory'),
            specs=[[{"secondary_y": False}, {"secondary_y": False}],
                   [{"secondary_y": False}, {"secondary_y": False}],
                   [{"secondary_y": False}, {"type": "scene"}]]
        )
        
        # Distance plots
        fig.add_trace(go.Scatter(x=sync_df['time_norm'], y=sync_df['distance_2']*1000,
                                mode='lines', name='D2 Synchronized', 
                                line=dict(color='blue', width=1), opacity=0.6), row=1, col=1)
        fig.add_trace(go.Scatter(x=filt_df['time_norm'], y=filt_df['distance_2']*1000,
                                mode='lines', name='D2 Filtered',
                                line=dict(color='red', width=2)), row=1, col=1)
        
        fig.add_trace(go.Scatter(x=sync_df['time_norm'], y=sync_df['distance_3']*1000,
                                mode='lines', name='D3 Synchronized',
                                line=dict(color='blue', width=1), opacity=0.6), row=1, col=2)
        fig.add_trace(go.Scatter(x=filt_df['time_norm'], y=filt_df['distance_3']*1000,
                                mode='lines', name='D3 Filtered',
                                line=dict(color='red', width=2)), row=1, col=2)
        
        # Position plots
        fig.add_trace(go.Scatter(x=sync_df['time_norm'], y=sync_df['x']*1000,
                                mode='lines', name='X Synchronized',
                                line=dict(color='blue', width=1), opacity=0.6), row=2, col=1)
        fig.add_trace(go.Scatter(x=filt_df['time_norm'], y=filt_df['x']*1000,
                                mode='lines', name='X Filtered',
                                line=dict(color='red', width=2)), row=2, col=1)
        
        fig.add_trace(go.Scatter(x=sync_df['time_norm'], y=sync_df['y']*1000,
                                mode='lines', name='Y Synchronized',
                                line=dict(color='blue', width=1), opacity=0.6), row=2, col=2)
        fig.add_trace(go.Scatter(x=filt_df['time_norm'], y=filt_df['y']*1000,
                                mode='lines', name='Y Filtered',
                                line=dict(color='red', width=2)), row=2, col=2)
        
        fig.add_trace(go.Scatter(x=sync_df['time_norm'], y=sync_df['z']*1000,
                                mode='lines', name='Z Synchronized',
                                line=dict(color='blue', width=1), opacity=0.6), row=3, col=1)
        fig.add_trace(go.Scatter(x=filt_df['time_norm'], y=filt_df['z']*1000,
                                mode='lines', name='Z Filtered',
                                line=dict(color='red', width=2)), row=3, col=1)
        
        # 3D trajectory
        if self.config["visualization"]["plot_3d_trajectories"]:
            fig.add_trace(go.Scatter3d(x=sync_df['x']*1000, y=sync_df['y']*1000, z=sync_df['z']*1000,
                                     mode='lines', name='3D Synchronized',
                                     line=dict(color='blue', width=3), opacity=0.6), row=3, col=2)
            fig.add_trace(go.Scatter3d(x=filt_df['x']*1000, y=filt_df['y']*1000, z=filt_df['z']*1000,
                                     mode='lines', name='3D Filtered',
                                     line=dict(color='red', width=5)), row=3, col=2)
        
        # Update layout
        fig.update_layout(height=1200, title_text="Interactive Data Processing Results")
        fig.update_xaxes(title_text="Time (s)")
        fig.update_yaxes(title_text="Distance (mm)", row=1, col=1)
        fig.update_yaxes(title_text="Distance (mm)", row=1, col=2)
        fig.update_yaxes(title_text="X (mm)", row=2, col=1)
        fig.update_yaxes(title_text="Y (mm)", row=2, col=2)
        fig.update_yaxes(title_text="Z (mm)", row=3, col=1)
        
        # Save interactive plot
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        html_file = os.path.join(output_dir, f"interactive_results_{timestamp}.html")
        fig.write_html(html_file)
        
        print(f"Interactive plots saved to: {html_file}")
        return html_file
    
    def save_processed_data(self, synchronized_data, filtered_segments):
        """Save processed data to CSV files"""
        output_dir = self.config["output"]["output_dir"]
        os.makedirs(output_dir, exist_ok=True)
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        
        saved_files = []
        
        # Save synchronized data
        if self.config["output"]["save_synchronized_csv"]:
            sync_file = os.path.join(output_dir, f"synchronized_data_{timestamp}.csv")
            sync_df = pd.DataFrame(synchronized_data)
            sync_df.to_csv(sync_file, index=False, float_format='%.6f')
            saved_files.append(sync_file)
            print(f"Synchronized data saved to: {sync_file}")
        
        # Save filtered data
        if self.config["output"]["save_filtered_csv"]:
            filtered_file = os.path.join(output_dir, f"filtered_data_{timestamp}.csv")
            filtered_data = []
            for segment in filtered_segments:
                filtered_data.extend(segment)
            
            if filtered_data:
                filt_df = pd.DataFrame(filtered_data)
                filt_df.to_csv(filtered_file, index=False, float_format='%.6f')
                saved_files.append(filtered_file)
                print(f"Filtered data saved to: {filtered_file}")
        
        return saved_files
    
    def generate_analysis_report(self):
        """Generate comprehensive analysis report"""
        output_dir = self.config["output"]["output_dir"]
        os.makedirs(output_dir, exist_ok=True)
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        
        report_file = os.path.join(output_dir, f"analysis_report_{timestamp}.txt")
        
        with open(report_file, 'w') as f:
            f.write("=" * 60 + "\n")
            f.write("ADVANCED DATA PROCESSING ANALYSIS REPORT\n")
            f.write("=" * 60 + "\n\n")
            
            f.write(f"Processing Date: {datetime.now().isoformat()}\n")
            f.write(f"Raw Data File: {self.raw_data_file}\n")
            f.write(f"Collection Duration: {self.metadata.get('duration_seconds', 0):.1f}s\n\n")
            
            # Raw data statistics
            f.write("RAW DATA STATISTICS:\n")
            f.write("-" * 30 + "\n")
            f.write(f"LiDAR Samples: {len(self.lidar_data)}\n")
            f.write(f"Tracker Samples: {len(self.tracker_data)}\n")
            f.write(f"LiDAR Mean Latency: {np.mean(self.lidar_data['latency_ms']):.1f}ms\n")
            f.write(f"Tracker Mean Latency: {np.mean(self.tracker_data['latency_ms']):.1f}ms\n\n")
            
            # Synchronization results
            if 'synchronization_stats' in self.results:
                stats = self.results['synchronization_stats']
                f.write("SYNCHRONIZATION RESULTS:\n")
                f.write("-" * 30 + "\n")
                f.write(f"Successful Points: {stats['success']}\n")
                f.write(f"Failed Points: {stats['failed']}\n")
                f.write(f"Outliers Removed: {stats['outliers_removed']}\n")
                success_rate = stats['success'] / (stats['success'] + stats['failed']) * 100 if (stats['success'] + stats['failed']) > 0 else 0
                f.write(f"Success Rate: {success_rate:.1f}%\n\n")
            
            # Segmentation results
            if 'segment_count' in self.results:
                f.write("SEGMENTATION RESULTS:\n")
                f.write("-" * 30 + "\n")
                f.write(f"Number of Segments: {self.results['segment_count']}\n\n")
            
            # Filtering results
            if 'filtering_stats' in self.results:
                f.write("FILTERING RESULTS:\n")
                f.write("-" * 30 + "\n")
                for stats in self.results['filtering_stats']:
                    f.write(f"Segment {stats['segment_id']}:\n")
                    f.write(f"  Points: {stats['points']}\n")
                    f.write(f"  SG Parameters: window={stats['window_length']}, poly={stats['polyorder']}\n")
                    f.write(f"  D2 Improvement: {stats['d2_std_improvement']:.1f}%\n")
                    f.write(f"  D3 Improvement: {stats['d3_std_improvement']:.1f}%\n\n")
            
            f.write("CONFIGURATION USED:\n")
            f.write("-" * 30 + "\n")
            f.write(json.dumps(self.config, indent=2))
        
        print(f"Analysis report saved to: {report_file}")
        return report_file
    
    def process(self):
        """Main processing pipeline"""
        print("\n" + "=" * 60)
        print("STARTING ADVANCED DATA PROCESSING PIPELINE")
        print("=" * 60)
        
        try:
            # Step 1: Synchronization
            synchronized_data = self.synchronize_data()
            if not synchronized_data:
                print("ERROR: Synchronization failed, cannot continue")
                return False
            
            # Step 2: Segmentation
            segments = self.segment_data(synchronized_data)
            if not segments:
                print("ERROR: Segmentation failed, cannot continue")
                return False
            
            # Step 3: Filtering
            filtered_segments = self.apply_sg_filtering(segments)
            if not filtered_segments:
                print("ERROR: Filtering failed, cannot continue")
                return False
            
            # Step 4: Save processed data
            saved_files = self.save_processed_data(synchronized_data, filtered_segments)
            
            # Step 5: Generate visualizations
            if self.config["visualization"]["generate_static_plots"]:
                static_plot = self.generate_static_plots(synchronized_data, filtered_segments)
            
            if self.config["visualization"]["generate_interactive_plots"]:
                interactive_plot = self.generate_interactive_plots(synchronized_data, filtered_segments)
            
            # Step 6: Generate analysis report
            report_file = self.generate_analysis_report()
            
            print("\n" + "=" * 60)
            print("PROCESSING COMPLETED SUCCESSFULLY")
            print("=" * 60)
            print(f"Processed {len(synchronized_data)} synchronized points")
            print(f"Generated {len(filtered_segments)} filtered segments")
            print(f"Output files saved to: {self.config['output']['output_dir']}")
            
            return True
            
        except Exception as e:
            print(f"ERROR: Processing failed: {e}")
            import traceback
            traceback.print_exc()
            return False

def main():
    parser = argparse.ArgumentParser(description="Advanced Data Processing for LiDAR-Tracker Data")
    parser.add_argument("input_file", help="Raw data NPZ file to process")
    parser.add_argument("-c", "--config", default="process_config.json", 
                       help="Processing configuration file")
    parser.add_argument("--window", type=int, help="Override SG filter window length")
    parser.add_argument("--poly", type=int, help="Override SG filter polynomial order")
    parser.add_argument("--no-interactive", action="store_true", 
                       help="Skip interactive plot generation")
    
    args = parser.parse_args()
    
    if not os.path.exists(args.input_file):
        print(f"ERROR: Input file not found: {args.input_file}")
        return 1
    
    # Load processor
    processor = DataProcessor(args.input_file, args.config)
    
    # Apply command line overrides
    if args.window:
        processor.config["filtering"]["sg_window_length"] = args.window
        processor.config["filtering"]["auto_optimize_parameters"] = False
    if args.poly:
        processor.config["filtering"]["sg_polyorder"] = args.poly
        processor.config["filtering"]["auto_optimize_parameters"] = False
    if args.no_interactive:
        processor.config["visualization"]["generate_interactive_plots"] = False
    
    # Process data
    success = processor.process()
    return 0 if success else 1

if __name__ == "__main__":
    exit(main())