"""
Timestamp-Synchronized NPZ Data Extractor
Author: Data Processing Team ✨
Description: Extract and synchronize LiDAR-Tracker data based on timestamps
"""

import numpy as np
import pandas as pd
import os
import argparse
from datetime import datetime
import logging
from scipy.interpolate import interp1d

class TimestampSynchronizedExtractor:
    def __init__(self, npz_file_path, interpolation_tolerance_ms=100):
        """
        Initialize the timestamp-synchronized extractor
        Args:
            npz_file_path: Path to the NPZ file containing raw data
            interpolation_tolerance_ms: Maximum time difference for synchronization (ms)
        """
        self.npz_file_path = npz_file_path
        self.interpolation_tolerance_ms = interpolation_tolerance_ms
        self.tolerance_seconds = interpolation_tolerance_ms / 1000.0
        self.setup_logging()
        self.load_npz_data()
        
    def setup_logging(self):
        """Setup logging configuration 📝"""
        logging.basicConfig(
            level=logging.INFO,
            format='%(asctime)s - %(levelname)s - %(message)s'
        )
        self.logger = logging.getLogger(__name__)
        
    def load_npz_data(self):
        """Load data from NPZ file 📂"""
        try:
            self.logger.info(f"Loading NPZ data from: {self.npz_file_path} 🔄")
            data = np.load(self.npz_file_path, allow_pickle=True)
            
            self.lidar_data = data['lidar_data']
            self.tracker_data = data['tracker_data']
            self.metadata = data['metadata'].item() if 'metadata' in data else {}
            
            self.logger.info(f"✅ Successfully loaded:")
            self.logger.info(f"   📡 LiDAR samples: {len(self.lidar_data)}")
            self.logger.info(f"   🎯 Tracker samples: {len(self.tracker_data)}")
            
            # Convert to DataFrames for easier processing
            self.lidar_df = pd.DataFrame(self.lidar_data)
            self.tracker_df = pd.DataFrame(self.tracker_data)
            
            # Sort by timestamp
            self.lidar_df = self.lidar_df.sort_values('timestamp').reset_index(drop=True)
            self.tracker_df = self.tracker_df.sort_values('timestamp').reset_index(drop=True)
            
            self.logger.info(f"   ⏰ LiDAR time range: {self.lidar_df['timestamp'].min():.3f} - {self.lidar_df['timestamp'].max():.3f}s")
            self.logger.info(f"   ⏰ Tracker time range: {self.tracker_df['timestamp'].min():.3f} - {self.tracker_df['timestamp'].max():.3f}s")
            
        except Exception as e:
            self.logger.error(f"❌ Failed to load NPZ file: {e}")
            raise
    
    def synchronize_data(self):
        """
        Synchronize LiDAR and Tracker data based on timestamps
        LiDAR data serves as the reference (fewer points)
        Returns synchronized DataFrame
        """
        self.logger.info("🔄 Synchronizing LiDAR and Tracker data based on timestamps...")
        
        lidar_times = self.lidar_df['timestamp'].values
        tracker_times = self.tracker_df['timestamp'].values
        
        # Find overlap period
        start_time = max(lidar_times[0], tracker_times[0])
        end_time = min(lidar_times[-1], tracker_times[-1])
        
        self.logger.info(f"   ⏰ Overlap period: {start_time:.3f} - {end_time:.3f}s")
        
        # Filter LiDAR data to overlap period
        lidar_mask = (self.lidar_df['timestamp'] >= start_time) & (self.lidar_df['timestamp'] <= end_time)
        lidar_subset = self.lidar_df[lidar_mask].copy()
        
        self.logger.info(f"   📡 LiDAR points in overlap: {len(lidar_subset)}")
        
        synchronized_data = []
        sync_stats = {'success': 0, 'failed': 0, 'interpolated': 0, 'nearest': 0}
        
        for idx, lidar_row in lidar_subset.iterrows():
            t_lidar = lidar_row['timestamp']
            
            # Find closest tracker timestamps
            time_diffs = np.abs(tracker_times - t_lidar)
            closest_idx = np.argmin(time_diffs)
            
            # Check if within tolerance
            if time_diffs[closest_idx] > self.tolerance_seconds:
                sync_stats['failed'] += 1
                continue
            
            # Determine interpolation strategy
            if closest_idx == 0 or closest_idx == len(tracker_times) - 1:
                # Use nearest neighbor at boundaries
                tracker_data = self.tracker_df.iloc[closest_idx]
                sync_stats['nearest'] += 1
                interp_method = 'nearest'
            else:
                # Check if we can do linear interpolation
                if tracker_times[closest_idx] <= t_lidar:
                    idx_before, idx_after = closest_idx, closest_idx + 1
                else:
                    idx_before, idx_after = closest_idx - 1, closest_idx
                
                t_before = tracker_times[idx_before]
                t_after = tracker_times[idx_after]
                
                if t_after != t_before:
                    # Linear interpolation
                    alpha = (t_lidar - t_before) / (t_after - t_before)
                    
                    tracker_before = self.tracker_df.iloc[idx_before]
                    tracker_after = self.tracker_df.iloc[idx_after]
                    
                    # Interpolate position data only (not angles - they can be discontinuous)
                    interpolated_data = {
                        'timestamp': t_lidar,
                        'x': tracker_before['x'] + alpha * (tracker_after['x'] - tracker_before['x']),
                        'y': tracker_before['y'] + alpha * (tracker_after['y'] - tracker_before['y']),
                        'z': tracker_before['z'] + alpha * (tracker_after['z'] - tracker_before['z']),
                        'roll': tracker_before['roll'],    # Use nearest for angles
                        'pitch': tracker_before['pitch'],
                        'yaw': tracker_before['yaw']
                    }
                    tracker_data = pd.Series(interpolated_data)
                    sync_stats['interpolated'] += 1
                    interp_method = 'linear'
                else:
                    # Times are the same, use nearest
                    tracker_data = self.tracker_df.iloc[closest_idx]
                    sync_stats['nearest'] += 1
                    interp_method = 'nearest'
            
            # Create synchronized point
            sync_point = {
                # LiDAR data (reference)
                'timestamp': t_lidar,
                'distance_2': lidar_row['distance_2'],
                'distance_3': lidar_row['distance_3'],
                
                # Tracker data (synchronized/interpolated)
                'x': tracker_data['x'],
                'y': tracker_data['y'], 
                'z': tracker_data['z'],
                'roll': tracker_data['roll'],
                'pitch': tracker_data['pitch'],
                'yaw': tracker_data['yaw'],
                
                # Metadata
                'interpolation_method': interp_method,
                'time_diff_ms': time_diffs[closest_idx] * 1000,
                'lidar_point_id': idx,
                'data_source': 'synchronized'
            }
            
            synchronized_data.append(sync_point)
            sync_stats['success'] += 1
        
        # Log synchronization results
        total_attempts = sync_stats['success'] + sync_stats['failed']
        success_rate = sync_stats['success'] / total_attempts * 100 if total_attempts > 0 else 0
        
        self.logger.info(f"✅ Synchronization completed:")
        self.logger.info(f"   🎯 Successful: {sync_stats['success']} points ({success_rate:.1f}%)")
        self.logger.info(f"   ❌ Failed: {sync_stats['failed']} points")
        self.logger.info(f"   📈 Linear interpolated: {sync_stats['interpolated']} points")
        self.logger.info(f"   📍 Nearest neighbor: {sync_stats['nearest']} points")
        
        self.sync_stats = sync_stats
        self.synchronized_df = pd.DataFrame(synchronized_data)
        
        return self.synchronized_df
    
    def extract_raw_lidar_csv(self, output_dir="extracted_data"):
        """Extract raw LiDAR data to CSV"""
        os.makedirs(output_dir, exist_ok=True)
        
        # Add metadata columns
        lidar_export = self.lidar_df.copy()
        lidar_export['data_source'] = 'raw_lidar'
        lidar_export['point_id'] = range(len(lidar_export))
        
        # Reorder columns
        column_order = ['point_id', 'timestamp', 'distance_2', 'distance_3', 'data_source']
        lidar_export = lidar_export[column_order]
        
        # Save to CSV
        csv_path = os.path.join(output_dir, "raw_lidar_data.csv")
        lidar_export.to_csv(csv_path, index=False, float_format='%.6f')
        
        self.logger.info(f"✅ Raw LiDAR data saved to: {csv_path}")
        return csv_path
    
    def extract_raw_tracker_csv(self, output_dir="extracted_data"):
        """Extract raw Tracker data to CSV"""
        os.makedirs(output_dir, exist_ok=True)
        
        # Add metadata columns
        tracker_export = self.tracker_df.copy()
        tracker_export['data_source'] = 'raw_tracker'
        tracker_export['point_id'] = range(len(tracker_export))
        
        # Reorder columns
        column_order = ['point_id', 'timestamp', 'x', 'y', 'z', 'roll', 'pitch', 'yaw', 'data_source']
        tracker_export = tracker_export[column_order]
        
        # Save to CSV
        csv_path = os.path.join(output_dir, "raw_tracker_data.csv")
        tracker_export.to_csv(csv_path, index=False, float_format='%.6f')
        
        self.logger.info(f"✅ Raw Tracker data saved to: {csv_path}")
        return csv_path
    
    def extract_synchronized_csv(self, output_dir="extracted_data"):
        """Extract synchronized data to CSV (this is the main output)"""
        if not hasattr(self, 'synchronized_df'):
            self.logger.info("🔄 Synchronizing data first...")
            self.synchronize_data()
        
        os.makedirs(output_dir, exist_ok=True)
        
        # Save synchronized data
        csv_path = os.path.join(output_dir, "synchronized_raw_data.csv")
        self.synchronized_df.to_csv(csv_path, index=False, float_format='%.6f')
        
        self.logger.info(f"✅ Synchronized data saved to: {csv_path}")
        self.logger.info(f"   📊 Shape: {self.synchronized_df.shape}")
        self.logger.info(f"   📋 Columns: {list(self.synchronized_df.columns)}")
        
        return csv_path
    
    def extract_all_data(self, output_dir="extracted_data"):
        """Extract all data formats"""
        # Create timestamped output directory
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        full_output_dir = os.path.join(output_dir, f"synchronized_extraction_{timestamp}")
        
        self.logger.info("🚀 Starting complete synchronized extraction...")
        
        # First synchronize data
        self.synchronize_data()
        
        # Extract all formats
        raw_lidar_path = self.extract_raw_lidar_csv(full_output_dir)
        raw_tracker_path = self.extract_raw_tracker_csv(full_output_dir)
        synchronized_path = self.extract_synchronized_csv(full_output_dir)
        
        # Create detailed synchronization report
        report_path = os.path.join(full_output_dir, "synchronization_report.txt")
        with open(report_path, 'w') as f:
            f.write("🕒 TIMESTAMP SYNCHRONIZATION REPORT\n")
            f.write("=" * 60 + "\n\n")
            f.write(f"📅 Extraction Time: {datetime.now().isoformat()}\n")
            f.write(f"📁 Source NPZ File: {self.npz_file_path}\n")
            f.write(f"⏱️ Interpolation Tolerance: {self.interpolation_tolerance_ms}ms\n\n")
            
            f.write("📊 Raw Data Statistics:\n")
            f.write("-" * 30 + "\n")
            f.write(f"📡 LiDAR Points: {len(self.lidar_df):,}\n")
            f.write(f"🎯 Tracker Points: {len(self.tracker_df):,}\n")
            f.write(f"⏰ LiDAR Duration: {self.lidar_df['timestamp'].max() - self.lidar_df['timestamp'].min():.1f}s\n")
            f.write(f"⏰ Tracker Duration: {self.tracker_df['timestamp'].max() - self.tracker_df['timestamp'].min():.1f}s\n\n")
            
            f.write("🔗 Synchronization Results:\n")
            f.write("-" * 30 + "\n")
            f.write(f"✅ Synchronized Points: {self.sync_stats['success']:,}\n")
            f.write(f"❌ Failed Synchronizations: {self.sync_stats['failed']:,}\n")
            f.write(f"📈 Linear Interpolated: {self.sync_stats['interpolated']:,}\n")
            f.write(f"📍 Nearest Neighbor: {self.sync_stats['nearest']:,}\n")
            
            success_rate = self.sync_stats['success'] / (self.sync_stats['success'] + self.sync_stats['failed']) * 100
            f.write(f"📊 Success Rate: {success_rate:.1f}%\n")
            f.write(f"🗑️ Data Reduction: {(1 - len(self.synchronized_df) / len(self.lidar_df)) * 100:.1f}%\n\n")
            
            f.write("📂 Generated Files:\n")
            f.write("-" * 30 + "\n")
            f.write(f"📡 Raw LiDAR: {os.path.basename(raw_lidar_path)}\n")
            f.write(f"🎯 Raw Tracker: {os.path.basename(raw_tracker_path)}\n")
            f.write(f"🔗 Synchronized: {os.path.basename(synchronized_path)}\n")
        
        result_paths = {
            'raw_lidar_csv': raw_lidar_path,
            'raw_tracker_csv': raw_tracker_path,
            'synchronized_csv': synchronized_path,
            'report': report_path,
            'output_directory': full_output_dir
        }
        
        self.logger.info("🎉 Synchronized extraction completed!")
        self.logger.info(f"📂 All files saved to: {full_output_dir}")
        
        return result_paths

def main():
    """Main function for command line usage 🖥️"""
    parser = argparse.ArgumentParser(
        description="Timestamp-Synchronized NPZ Data Extractor 🕒"
    )
    parser.add_argument(
        "npz_file", 
        help="Path to the NPZ file containing raw data"
    )
    parser.add_argument(
        "-o", "--output", 
        default="extracted_data",
        help="Output directory for CSV files (default: extracted_data)"
    )
    parser.add_argument(
        "-t", "--tolerance", 
        type=int, 
        default=100,
        help="Interpolation tolerance in milliseconds (default: 100ms)"
    )
    
    args = parser.parse_args()
    
    # Check if input file exists
    if not os.path.exists(args.npz_file):
        print(f"❌ Error: NPZ file not found: {args.npz_file}")
        return 1
    
    try:
        # Create extractor and process
        extractor = TimestampSynchronizedExtractor(args.npz_file, args.tolerance)
        result_paths = extractor.extract_all_data(args.output)
        
        print("\n🎊 Synchronization Summary:")
        print(f"📂 Output Directory: {result_paths['output_directory']}")
        print(f"🔗 Main Output: {os.path.basename(result_paths['synchronized_csv'])}")
        print(f"📊 Success Rate: {extractor.sync_stats['success']/(extractor.sync_stats['success']+extractor.sync_stats['failed'])*100:.1f}%")
        print("✨ Ready for processing pipeline!")
        
        return 0
        
    except Exception as e:
        print(f"❌ Extraction failed: {e}")
        import traceback
        traceback.print_exc()
        return 1

if __name__ == "__main__":
    exit(main())