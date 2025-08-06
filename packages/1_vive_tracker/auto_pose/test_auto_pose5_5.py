"""
=== SAMPLING LOGIC OVERVIEW ===

1. BASELINE CALIBRATION:
   - Each filter requires 50 samples to establish noise characteristics
   - Baseline calculates noise level and reference value for zero drift filtering
   - Baseline resets when 'P' is pressed (position change)

2. ZERO DRIFT FILTERING:
   - Adaptive filtering based on noise level learned from baseline
   - No time-based compensation, only noise-level based filtering
   - Separate noise profiles for D2 and D3 channels

3. SAMPLING SEGMENTS:
   - Segment 0: Initial recording after pressing 'R'
   - Segment N: Each time 'P' is pressed and resumed, increment segment number
   - Each segment has independent baseline calibration

4. DATA RECORDING CONDITIONS:
   - Must have valid baseline for both D2 and D3 channels
   - Significant change detection based on filtered data
   - Maximum change protection and continuity validation
   - Tracker yaw stability check
"""

import sys
import time
import struct
import serial
import threading
import queue
import os
import numpy as np
import matplotlib.pyplot as plt
from datetime import datetime
from pynput import keyboard
from vive_tracker import ViveTrackerModule

# Debug level control
DEBUG_LEVEL = {
    "SERIAL_RAW": False,     # Serial raw data
    "PROTOCOL": False,       # Protocol parsing details
    "PERFORMANCE": True,     # Performance monitoring
    "CONNECTION": True       # Connection status
}

# System state enumeration
class SystemState:
    IDLE = "IDLE"
    BASELINE_TESTING = "BASELINE_TESTING"
    RECORDING = "RECORDING"
    PAUSED = "PAUSED"
    WAITING_BASELINE = "WAITING_BASELINE"

# Simple zero drift filter based on noise characteristics
class ZeroDriftFilter:
    def __init__(self, name, baseline_samples=50):
        self.name = name
        self.baseline_samples_required = baseline_samples
        
        # Baseline data collection
        self.baseline_data = []
        self.baseline_ready = False
        
        # Noise characteristics learned from baseline
        self.reference_value = None      # Static reference value
        self.noise_std = None           # Noise standard deviation
        self.noise_threshold = None     # Filtering threshold
        
        # Current filtering state
        self.current_smooth_value = None
        
        # Filter parameters
        self.filter_strength = {
            'strong': 0.2,    # Strong filtering coefficient for noise
            'medium': 0.6,    # Medium filtering for small changes
            'weak': 1.0       # No filtering for significant changes
        }
        
    def start_baseline_calibration(self):
        """Start baseline calibration"""
        self.baseline_data.clear()
        self.baseline_ready = False
        self.reference_value = None
        self.noise_std = None
        self.noise_threshold = None
        self.current_smooth_value = None
        write_log(f"[ZeroDrift-{self.name}] Starting baseline calibration...", "IMPORTANT")
    
    def add_baseline_sample(self, value, timestamp):
        """Add sample during baseline calibration"""
        if self.baseline_ready:
            return True  # Already calibrated
            
        self.baseline_data.append(value)
        
        if len(self.baseline_data) >= self.baseline_samples_required:
            self._analyze_baseline()
            return True
        
        return False  # Still need more samples
    
    def _analyze_baseline(self):
        """Analyze baseline data to extract noise characteristics"""
        values = np.array(self.baseline_data)
        
        # Calculate noise statistics
        self.reference_value = np.median(values)  # Use median as reference (robust to outliers)
        self.noise_std = np.std(values)
        
        # Set filtering thresholds based on noise level
        self.noise_threshold = 2.0 * self.noise_std  # 2-sigma threshold
        
        # Initialize current smooth value
        self.current_smooth_value = self.reference_value
        self.baseline_ready = True
        
        write_log(f"[ZeroDrift-{self.name}] Baseline analysis complete:", "IMPORTANT")
        write_log(f"  Reference: {self.reference_value:.4f}m", "IMPORTANT")
        write_log(f"  Noise Std: {self.noise_std*1000:.2f}mm", "IMPORTANT")
        write_log(f"  Filter Threshold: {self.noise_threshold*1000:.2f}mm", "IMPORTANT")
    
    def apply_zero_drift_filter(self, raw_value):
        """Apply zero drift filtering based on noise characteristics"""
        if not self.baseline_ready:
            return raw_value  # No filtering available
        
        if self.current_smooth_value is None:
            self.current_smooth_value = raw_value
            return raw_value
        
        # Calculate deviation from current smooth value
        deviation = abs(raw_value - self.current_smooth_value)
        
        # Apply adaptive filtering based on deviation magnitude
        if deviation < self.noise_threshold:
            # Small deviation - likely noise, apply strong filtering
            alpha = self.filter_strength['strong']
            filtered_value = alpha * raw_value + (1 - alpha) * self.current_smooth_value
            
        elif deviation < 3.0 * self.noise_threshold:
            # Medium deviation - apply medium filtering
            alpha = self.filter_strength['medium']
            filtered_value = alpha * raw_value + (1 - alpha) * self.current_smooth_value
            
        else:
            # Large deviation - likely real change, minimal filtering
            alpha = self.filter_strength['weak']
            filtered_value = raw_value
        
        # Update smooth value
        self.current_smooth_value = filtered_value
        
        # Log significant filtering
        correction = raw_value - filtered_value
        if abs(correction) > 0.001:  # 1mm threshold
            write_log(f"[ZeroDrift-{self.name}] Applied filtering: {correction*1000:.1f}mm correction (deviation: {deviation*1000:.1f}mm)")
        
        return filtered_value
    
    def is_ready(self):
        return self.baseline_ready
    
    def get_status(self):
        if not self.baseline_ready:
            return f"Calibrating: {len(self.baseline_data)}/{self.baseline_samples_required} samples"
        else:
            return f"Ready - Ref: {self.reference_value:.4f}m, Noise: {self.noise_std*1000:.2f}mm"
    
    def reset(self):
        """Reset all states for new position"""
        self.baseline_data.clear()
        self.baseline_ready = False
        self.reference_value = None
        self.noise_std = None
        self.noise_threshold = None
        self.current_smooth_value = None
        write_log(f"[ZeroDrift-{self.name}] Filter reset for new position", "IMPORTANT")

# Enhanced zero drift validator for static testing
class ZeroDriftValidator:
    def __init__(self, name=""):
        self.name = name
        self.static_test_data = []
        self.test_duration = 60
        self.start_time = None
        self.is_testing = False
        self.accumulated_time = 0
        self.last_valid_time = None
        
    def start_static_test(self):
        self.start_time = time.time()
        self.static_test_data.clear()
        self.is_testing = True
        self.accumulated_time = 0
        self.last_valid_time = self.start_time
        write_log(f"[Validation-{self.name}] Starting 60-second static zero drift test", "IMPORTANT")
        
    def add_static_sample(self, raw_value, filtered_value):
        if not self.is_testing:
            return
            
        current_time = time.time()
        
        if self.last_valid_time and (current_time - self.last_valid_time) > 1.0:
            disconnection_duration = current_time - self.last_valid_time
            write_log(f"[Validation-{self.name}] Connection recovery detected, disconnection duration: {disconnection_duration:.1f}s", "IMPORTANT")
        
        if self.last_valid_time:
            time_since_last = min(current_time - self.last_valid_time, 1.0)
            self.accumulated_time += time_since_last
        
        self.last_valid_time = current_time
        
        if self.accumulated_time >= self.test_duration:
            write_log(f"[Validation-{self.name}] Test duration reached, ending test", "IMPORTANT")
            self.is_testing = False
            self.analyze_static_performance()
            return
        
        self.static_test_data.append({
            'time': self.accumulated_time,
            'raw': raw_value,
            'filtered': filtered_value,
            'real_time': current_time
        })
        
        if len(self.static_test_data) % 200 == 0:
            progress = (self.accumulated_time / self.test_duration) * 100
            remaining = self.test_duration - self.accumulated_time
            write_log(f"[Validation-{self.name}] Test progress: {progress:.1f}% (remaining {remaining:.1f}s)", "IMPORTANT")
                
    def analyze_static_performance(self):
        if len(self.static_test_data) < 50:
            write_log(f"[Validation-{self.name}] Insufficient test data ({len(self.static_test_data)} samples)")
            return
            
        raw_values = [d['raw'] for d in self.static_test_data]
        filtered_values = [d['filtered'] for d in self.static_test_data]
        
        raw_std = np.std(raw_values) * 1000
        filtered_std = np.std(filtered_values) * 1000
        raw_range = (np.max(raw_values) - np.min(raw_values)) * 1000
        filtered_range = (np.max(filtered_values) - np.min(filtered_values)) * 1000
        
        improvement_std = (raw_std - filtered_std) / raw_std * 100 if raw_std > 0 else 0
        improvement_range = (raw_range - filtered_range) / raw_range * 100 if raw_range > 0 else 0
        
        effective_data_rate = len(self.static_test_data) / self.accumulated_time if self.accumulated_time > 0 else 0
        
        report = f"""
=== {self.name} Zero Drift Validation Report ===
Test Configuration:
  Duration: {self.accumulated_time:.1f}s
  Samples: {len(self.static_test_data)}
  Data Rate: {effective_data_rate:.1f} Hz

Performance Statistics:
  Raw Data Std Dev: {raw_std:.2f}mm
  Filtered Data Std Dev: {filtered_std:.2f}mm
  Std Dev Improvement: {improvement_std:.1f}%
  
  Raw Data Range: {raw_range:.2f}mm  
  Filtered Data Range: {filtered_range:.2f}mm
  Range Improvement: {improvement_range:.1f}%
        """
        write_log(report, "IMPORTANT")
        self.plot_drift_comparison()
    
    def plot_drift_comparison(self):
        try:
            times = [d['time'] for d in self.static_test_data]
            raw_values = [d['raw']*1000 for d in self.static_test_data]
            filtered_values = [d['filtered']*1000 for d in self.static_test_data]
            
            plt.figure(figsize=(15, 8))
            
            plt.subplot(2, 1, 1)
            plt.plot(times, raw_values, 'b-', alpha=0.7, label='Raw Data', linewidth=1)
            plt.plot(times, filtered_values, 'r-', linewidth=2, label='Filtered Data')
            plt.ylabel('Distance (mm)')
            plt.title(f'{self.name} Zero Drift Filtering Performance')
            plt.legend()
            plt.grid(True, alpha=0.3)
            
            plt.subplot(2, 1, 2)
            differences = [(raw - filtered) * 1000 for raw, filtered in zip([d['raw'] for d in self.static_test_data], [d['filtered'] for d in self.static_test_data])]
            plt.plot(times, differences, 'g-', alpha=0.7, label='Filter Correction', linewidth=1)
            plt.axhline(y=0, color='k', linestyle='--', alpha=0.5)
            plt.ylabel('Correction (mm)')
            plt.xlabel('Time (s)')
            plt.legend()
            plt.grid(True, alpha=0.3)
            
            plt.tight_layout()
            plt.savefig(os.path.join(DATA_DIR, f'zero_drift_test_{self.name}.png'), dpi=150)
            write_log(f"[Validation-{self.name}] Zero drift test chart saved")
            plt.close()
        except Exception as e:
            write_log(f"[Validation-{self.name}] Chart generation failed: {e}")
    
    def get_test_status(self):
        if not self.is_testing:
            return "Not testing"
        
        progress = (self.accumulated_time / self.test_duration) * 100
        remaining = self.test_duration - self.accumulated_time
        return f"In progress {progress:.1f}% (Remaining: {remaining:.1f}s, Samples: {len(self.static_test_data)})"
    
    def force_finish_test(self):
        if self.is_testing:
            write_log(f"[Validation-{self.name}] Manually ending test", "IMPORTANT")
            self.is_testing = False
            if len(self.static_test_data) >= 50:
                self.analyze_static_performance()

# Sampling state machine
class SamplingStateMachine:
    def __init__(self):
        self.state = SystemState.IDLE
        self.zero_drift_filters = {
            'd2': ZeroDriftFilter("D2"),
            'd3': ZeroDriftFilter("D3")
        }
        
    def start_baseline_test(self):
        """Start standalone baseline test (for drift analysis only)"""
        if self.state != SystemState.IDLE:
            return False
        
        self.state = SystemState.BASELINE_TESTING
        for filter_obj in self.zero_drift_filters.values():
            filter_obj.start_baseline_calibration()
        return True
    
    def start_data_collection(self):
        """Start data collection with baseline calibration"""
        if self.state != SystemState.IDLE:
            return False
        
        self.state = SystemState.WAITING_BASELINE
        for filter_obj in self.zero_drift_filters.values():
            filter_obj.start_baseline_calibration()
        return True
    
    def process_sample(self, timestamp, d2_raw, d3_raw, cam_coord):
        """Process a single sample based on current state"""
        
        if self.state == SystemState.BASELINE_TESTING:
            # Baseline test mode - only for analysis
            d2_ready = self.zero_drift_filters['d2'].add_baseline_sample(d2_raw, timestamp)
            d3_ready = self.zero_drift_filters['d3'].add_baseline_sample(d3_raw, timestamp)
            
            if d2_ready and d3_ready:
                write_log("[System] Baseline test completed", "IMPORTANT")
                return "baseline_complete", None, None
            
            return "baseline_testing", None, None
            
        elif self.state == SystemState.WAITING_BASELINE:
            # Data collection mode - baseline calibration phase
            d2_ready = self.zero_drift_filters['d2'].add_baseline_sample(d2_raw, timestamp)
            d3_ready = self.zero_drift_filters['d3'].add_baseline_sample(d3_raw, timestamp)
            
            if d2_ready and d3_ready:
                self.state = SystemState.RECORDING
                write_log("[System] 基线校准完成，开始数据记录", "IMPORTANT")
                return "recording_ready", None, None
            
            return "calibrating", None, None
            
        elif self.state == SystemState.RECORDING:
            # Data recording mode - apply zero drift filtering
            d2_filtered = self.zero_drift_filters['d2'].apply_zero_drift_filter(d2_raw)
            d3_filtered = self.zero_drift_filters['d3'].apply_zero_drift_filter(d3_raw)
            
            return "recording", d2_filtered, d3_filtered
            
        return "idle", None, None
    
    def pause_for_position_change(self):
        """Pause and reset for position change"""
        if self.state == SystemState.RECORDING:
            self.state = SystemState.PAUSED
            return True
        return False
    
    def resume_after_position_change(self):
        """Resume with new baseline calibration"""
        if self.state == SystemState.PAUSED:
            self.state = SystemState.WAITING_BASELINE
            for filter_obj in self.zero_drift_filters.values():
                filter_obj.reset()
                filter_obj.start_baseline_calibration()
            return True
        return False

# Latency detection and filtering class
class LatencyFilter:
    def __init__(self, max_acceptable_latency_ms=30):
        self.max_latency = max_acceptable_latency_ms / 1000.0
        self.recent_latencies = []
        self.rejected_count = 0
        self.total_requests = 0
        
    def check_lidar_latency(self, request_time, response_time):
        latency = response_time - request_time
        self.recent_latencies.append(latency)
        self.total_requests += 1
        
        if len(self.recent_latencies) > 20:
            self.recent_latencies.pop(0)
            
        if latency > self.max_latency:
            write_log(f"[Latency] Rejecting high latency data: {latency*1000:.1f}ms > {self.max_latency*1000}ms", "DEBUG")
            return False
        return True
    
    def check_tracker_freshness(self, tracker_request_time, tracker_response_time):
        processing_time = tracker_response_time - tracker_request_time
        if processing_time > self.max_latency:
            write_log(f"[Latency] Rejecting slow tracker data: fetch took {processing_time*1000:.1f}ms", "DEBUG")
            return False
        return True
    
    def get_latency_stats(self):
        if self.recent_latencies:
            avg_latency = np.mean(self.recent_latencies) * 1000
            max_latency = np.max(self.recent_latencies) * 1000
            rejection_rate = (self.rejected_count / self.total_requests) * 100 if self.total_requests > 0 else 0
            return f"Avg: {avg_latency:.1f}ms, Max: {max_latency:.1f}ms, Reject: {rejection_rate:.1f}%"
        return "No latency data"

# LiDAR continuity validator class
class LiDARContinuityValidator:
    def __init__(self, name, max_interval=0.1):
        self.name = name
        self.max_interval = max_interval
        self.last_valid_value = None
        self.pending_value = None
        self.consecutive_invalid_count = 0
        
    def validate_continuity(self, current_value):
        """Validate LiDAR data continuity"""
        if self.last_valid_value is None:
            self.last_valid_value = current_value
            return True, True
        
        interval = abs(current_value - self.last_valid_value)
        
        if interval <= self.max_interval:
            if self.pending_value is not None:
                write_log(f"[Continuity-{self.name}] Valid data resumed: interval {interval:.3f}m <= {self.max_interval}m", "IMPORTANT")
                self.pending_value = None
                self.consecutive_invalid_count = 0
            
            self.last_valid_value = current_value
            return True, True
        else:
            self.consecutive_invalid_count += 1
            write_log(f"[Continuity-{self.name}] Invalid interval detected: {interval:.3f}m > {self.max_interval}m", "DEBUG")
            
            if self.pending_value is None:
                self.pending_value = current_value
                return False, False
            else:
                self.pending_value = current_value
                return False, False
    
    def reset(self):
        """Reset validator state"""
        self.last_valid_value = None
        self.pending_value = None
        self.consecutive_invalid_count = 0
        write_log(f"[Continuity-{self.name}] Validator reset", "DEBUG")

# Tracker yaw stability validator class
class TrackerYawValidator:
    def __init__(self, max_sudden_change=10.0, window_size=10):
        self.max_sudden_change = np.radians(max_sudden_change)
        self.window_size = window_size
        self.yaw_history = []
        self.yaw_mean = None
        
    def validate_yaw_stability(self, current_yaw_rad):
        """Validate tracker yaw stability"""
        if self.yaw_mean is None:
            self.yaw_history.append(current_yaw_rad)
            self.yaw_mean = current_yaw_rad
            return True
        
        yaw_diff = self._angle_difference(current_yaw_rad, self.yaw_mean)
        
        if abs(yaw_diff) > self.max_sudden_change:
            write_log(f"[Yaw] Sudden yaw change detected: {np.degrees(yaw_diff):.1f} deg", "DEBUG")
            return False
        
        self.yaw_history.append(current_yaw_rad)
        if len(self.yaw_history) > self.window_size:
            self.yaw_history.pop(0)
        
        self.yaw_mean = np.mean(self.yaw_history)
        return True
    
    def _angle_difference(self, angle1, angle2):
        """Calculate the difference between two angles, handling wrapping"""
        diff = angle1 - angle2
        while diff > np.pi:
            diff -= 2 * np.pi
        while diff < -np.pi:
            diff += 2 * np.pi
        return diff
    
    def reset(self):
        """Reset validator state"""
        self.yaw_history.clear()
        self.yaw_mean = None
        write_log("[Yaw] Validator reset", "DEBUG")

# Data status management class
class DataStatus:
    def __init__(self):
        self.invalid_count = 0
        self.had_valid_data = False
        self.last_recorded_data = None
        self.consecutive_failures = 0
        self.recorded_data_raw = []
        self.recorded_data_filtered = []
        self.pause_count = 0
        self.current_segment = 0
        self.has_paused = False
    
    def on_invalid_data(self):
        self.invalid_count += 1
        self.consecutive_failures += 1
        return self.invalid_count
    
    def on_valid_data(self):
        recovery_needed = self.invalid_count > 3 and self.had_valid_data
        self.invalid_count = 0
        self.had_valid_data = True
        self.consecutive_failures = 0
        return recovery_needed
    
    def reset_for_recovery(self):
        write_log("[System] Executing data recovery reset", "IMPORTANT")
        self.last_recorded_data = None
    
    def increment_segment_on_pause(self):
        """Increment segment number on pause"""
        self.pause_count += 1
        self.current_segment = self.pause_count
        self.has_paused = True
        self.last_recorded_data = None
        write_log(f"[Sampling] 切换到采样段 #{self.current_segment} (总暂停次数: {self.pause_count})", "IMPORTANT")
    
    def record_data_point(self, timestamp, distance_2_raw, distance_3_raw, distance_2_filtered, distance_3_filtered, cam_coord):
        self.recorded_data_raw.append((timestamp, distance_2_raw, distance_3_raw, cam_coord, self.current_segment))
        self.recorded_data_filtered.append((timestamp, distance_2_filtered, distance_3_filtered, cam_coord, self.current_segment))
        
        write_log(f"[Data] Segment#{self.current_segment} recorded: raw d2={distance_2_raw:.3f}m, d3={distance_3_raw:.3f}m | filtered d2={distance_2_filtered:.3f}m, d3={distance_3_filtered:.3f}m")

# Serial buffer management class
class SerialBufferManager:
    def __init__(self, serial_port, expected_packet_size=10):
        self.ser = serial_port
        self.expected_size = expected_packet_size
        self.buffer = bytearray()
        self.sync_lost_count = 0
        self.recovery_attempts = 0
        
    def find_valid_packet(self):
        """Find valid packet in buffer"""
        while len(self.buffer) >= self.expected_size:
            start_idx = -1
            for i in range(len(self.buffer) - 1):
                if self.buffer[i] == 0x55 and self.buffer[i + 1] == 0x7E:
                    start_idx = i
                    break
            
            if start_idx == -1:
                self.buffer = self.buffer[len(self.buffer)//2:]
                return None
            
            if start_idx > 0:
                self.buffer = self.buffer[start_idx:]
            
            if len(self.buffer) >= self.expected_size:
                packet = self.buffer[:self.expected_size]
                
                if packet[8] == 0x7E and packet[9] == 0x55:
                    self.buffer = self.buffer[self.expected_size:]
                    return bytes(packet)
                else:
                    self.buffer = self.buffer[1:]
            else:
                break
        
        return None
    
    def read_packet_with_recovery(self, timeout=0.1):
        """Packet reading with recovery mechanism"""
        start_time = time.time()
        
        while time.time() - start_time < timeout:
            try:
                available = self.ser.in_waiting
                if available > 0:
                    new_data = self.ser.read(available)
                    self.buffer.extend(new_data)
                
                packet = self.find_valid_packet()
                if packet:
                    self.sync_lost_count = 0
                    self.recovery_attempts = 0
                    return packet
                
                if len(self.buffer) > 100:
                    self.buffer = self.buffer[-50:]
                
                time.sleep(0.001)
                
            except serial.SerialException as e:
                write_log(f"Serial read exception: {e}", "ERROR")
                return None
        
        self.sync_lost_count += 1
        if self.sync_lost_count > 3:
            self.attempt_recovery()
        
        return None
    
    def attempt_recovery(self):
        """Connection recovery attempt"""
        self.recovery_attempts += 1
        write_log(f"Attempting connection recovery (attempt {self.recovery_attempts})", "DEBUG")
        
        try:
            self.ser.reset_input_buffer()
            self.ser.reset_output_buffer()
            self.buffer.clear()
            
            for i in range(3):
                self.send_request()
                time.sleep(0.02)
            
        except Exception as e:
            write_log(f"Recovery operation failed: {e}", "ERROR")
    
    def send_request(self):
        """Send A69 data request"""
        tx_buf = bytearray([0x55, 0x7E, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x7E, 0x55])
        tx_buf[7] = calculate_checksum_r(tx_buf)
        try:
            self.ser.write(tx_buf)
            self.ser.flush()
        except Exception as e:
            write_log(f"Request send failed: {e}", "ERROR")

# Create data storage directory
def create_data_directory():
    timestamp = datetime.now().strftime("%m%d_%H%M%S")
    data_dir = os.path.join(os.getcwd(), "data", f"sampling_{timestamp}")
    os.makedirs(data_dir, exist_ok=True)
    print(f"Data will be saved to directory: {data_dir}")
    return data_dir, timestamp

# Configuration
SERIAL_PORT = "/dev/ttyUSB0"
BAUD_RATE = 9600
TRACKER_NAME = "tracker_1"

DATA_DIR, TIMESTAMP = create_data_directory()
CSV_FILE_RAW = os.path.join(DATA_DIR, f"tracker_data_raw_{TIMESTAMP}.csv")
CSV_FILE_FILTERED = os.path.join(DATA_DIR, f"tracker_data_filtered_{TIMESTAMP}.csv")
LOG_FILE = os.path.join(DATA_DIR, "tracker_log.txt")

# Initialize log
with open(LOG_FILE, "w") as log_file:
    log_file.write(f"Log created: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n")
    log_file.write("=== System Log ===\n")

def write_log(message, level="INFO"):
    timestamp = datetime.now().strftime('%H:%M:%S.%f')[:-3]
    log_message = f"[{timestamp}] {message}"
    
    try:
        with open(LOG_FILE, "a", encoding='utf-8') as log_file:
            file_message = log_message
            file_message = file_message.replace("基线校准完成，开始数据记录", "Baseline calibration completed, starting data recording")
            file_message = file_message.replace("切换到采样段", "Switched to sampling segment")
            file_message = file_message.replace("总暂停次数", "total pause count")
            log_file.write(file_message + "\n")
    except Exception as e:
        print(f"Error writing to log file: {e}")
    
    if level in ["IMPORTANT", "ERROR"]:
        print(message)
    elif level == "DEBUG" and DEBUG_LEVEL["PROTOCOL"]:
        print(f"[DEBUG] {message}")

# Initialize devices
write_log("[System] Initializing ViveTrackerModule...", "IMPORTANT")
vtm = ViveTrackerModule()
vtm.print_discovered_objects()
tracker = vtm.devices.get(TRACKER_NAME)

if tracker is None:
    write_log(f"[Error] Tracker '{TRACKER_NAME}' not found!", "ERROR")
    sys.exit(1)
else:
    write_log(f"[System] Tracker '{TRACKER_NAME}' connected successfully", "IMPORTANT")

# Initialize serial port
try:
    ser = serial.Serial(
        port=SERIAL_PORT, 
        baudrate=BAUD_RATE, 
        timeout=0.1,
        write_timeout=0.1,
        bytesize=serial.EIGHTBITS,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        xonxoff=False,
        rtscts=False,
        dsrdtr=False
    )
    
    ser.reset_input_buffer()
    ser.reset_output_buffer()
    
    if ser.is_open:
        write_log(f"Serial port configured: {SERIAL_PORT}, {BAUD_RATE} baud", "IMPORTANT")
except serial.SerialException as e:
    write_log(f"Cannot open serial port {SERIAL_PORT}: {e}", "ERROR")
    sys.exit(1)

# Global object initialization
data_status = DataStatus()
running = True
recording = False
state_machine = SamplingStateMachine()
latency_filter = LatencyFilter(max_acceptable_latency_ms=80)

# Validators
continuity_validators = {
    'd2': LiDARContinuityValidator("D2", max_interval=0.1),
    'd3': LiDARContinuityValidator("D3", max_interval=0.1)
}
yaw_validator = TrackerYawValidator(max_sudden_change=10.0)

# Zero drift validators for static testing
drift_validators = {
    'd2': ZeroDriftValidator("Distance_2"),
    'd3': ZeroDriftValidator("Distance_3")
}

# Threshold settings
min_change_thresholds = {
    "Distance_2": 0.008,
    "Distance_3": 0.008,
    "X": 0.005,
    "Z": 0.005
}
max_change_threshold = 0.2

# A69 protocol functions
def calculate_checksum_r(data):
    return data[3] ^ data[4] ^ data[5] ^ data[6]

def parse_a69_data_with_debug(response):
    """A69 data parsing"""
    if len(response) != 10:
        return None
    
    if (response[0] != 0x55 or response[1] != 0x7E or 
        response[8] != 0x7E or response[9] != 0x55):
        return None

    checksum = response[7]
    computed_checksum = calculate_checksum_r(response)
    
    if checksum != computed_checksum:
        return None

    distance_2 = (response[3] << 8) | response[4]
    distance_3 = (response[5] << 8) | response[6]
    
    d2_meters = distance_2 / 1000.0
    d3_meters = distance_3 / 1000.0
    
    return d2_meters, d3_meters

def save_to_csv(data_raw, data_filtered):
    """Save data to CSV files"""
    with open(CSV_FILE_RAW, 'w', encoding='utf-8') as csv_file:
        csv_file.write("Timestamp,Distance_2,Distance_3,X,Y,Z,Roll,Yaw,Pitch,Segment\n")
        for timestamp, dis2, dis3, coord, segment in data_raw:
            csv_entry = f"{timestamp:.3f},{dis2:.3f},{dis3:.3f},{coord[0]:.4f},{coord[1]:.4f},{coord[2]:.4f},{coord[3]:.4f},{coord[4]:.4f},{coord[5]:.4f},{segment}\n"
            csv_file.write(csv_entry)
    
    with open(CSV_FILE_FILTERED, 'w', encoding='utf-8') as csv_file:
        csv_file.write("Timestamp,Distance_2,Distance_3,X,Y,Z,Roll,Yaw,Pitch,Segment\n")
        for timestamp, dis2, dis3, coord, segment in data_filtered:
            csv_entry = f"{timestamp:.3f},{dis2:.3f},{dis3:.3f},{coord[0]:.4f},{coord[1]:.4f},{coord[2]:.4f},{coord[3]:.4f},{coord[4]:.4f},{coord[5]:.4f},{segment}\n"
            csv_file.write(csv_entry)
            
    write_log(f"[System] Saved {len(data_raw)} raw data entries and {len(data_filtered)} filtered data entries", "IMPORTANT")

def validate_data_quality(distance_2, distance_3, cam_coord):
    """Validate data quality using continuity and yaw validators"""
    d2_valid, d2_should_record = continuity_validators['d2'].validate_continuity(distance_2)
    d3_valid, d3_should_record = continuity_validators['d3'].validate_continuity(distance_3)
    yaw_valid = yaw_validator.validate_yaw_stability(cam_coord[4])
    
    data_valid_for_recording = d2_valid and d3_valid and yaw_valid and d2_should_record and d3_should_record
    
    if not data_valid_for_recording:
        if not d2_valid or not d2_should_record:
            write_log(f"[Validation] D2 data rejected due to continuity check", "DEBUG")
        if not d3_valid or not d3_should_record:
            write_log(f"[Validation] D3 data rejected due to continuity check", "DEBUG")
        if not yaw_valid:
            write_log(f"[Validation] Data rejected due to yaw instability", "DEBUG")
    
    return data_valid_for_recording

def should_record_data(d2_filtered, d3_filtered, cam_coord):
    """Determine if data should be recorded based on significant changes"""
    if data_status.last_recorded_data is None:
        data_status.last_recorded_data = (d2_filtered, d3_filtered, cam_coord[0], cam_coord[2])
        return True
    
    current_data = (d2_filtered, d3_filtered, cam_coord[0], cam_coord[2])
    changes = [abs(current_data[i] - data_status.last_recorded_data[i]) for i in range(4)]
    
    significant_changes = []
    thresholds = [0.008, 0.008, 0.005, 0.005]  # D2, D3, X, Z
    names = ["Distance_2", "Distance_3", "X", "Z"]
    
    for i, (change, threshold) in enumerate(zip(changes, thresholds)):
        if change > threshold:
            significant_changes.append(f"{names[i]}={change:.4f}>{threshold}")
    
    if any(change > max_change_threshold for change in changes):
        write_log(f"[Warning] Data change too large, update baseline but do not record", "IMPORTANT")
        data_status.last_recorded_data = current_data
        return False
    
    if significant_changes:
        write_log(f"[Info] Significant change detected: {', '.join(significant_changes)}", "IMPORTANT")
        data_status.last_recorded_data = current_data
        return True
    else:
        write_log("[Info] No significant change, skip recording")
        return False

def process_data(timestamp, distance_2, distance_3, cam_coord):
    """Process data with zero drift filtering"""
    global state_machine, data_status
    
    result, d2_filtered, d3_filtered = state_machine.process_sample(
        timestamp, distance_2, distance_3, cam_coord
    )
    
    if result == "baseline_testing":
        # For static testing - add to validators
        drift_validators['d2'].add_static_sample(distance_2, distance_2)  # No filtering during baseline test
        drift_validators['d3'].add_static_sample(distance_3, distance_3)
        write_log(f"[Baseline] D2: {distance_2:.3f}m, D3: {distance_3:.3f}m")
        return
        
    elif result == "baseline_complete":
        state_machine.state = SystemState.IDLE
        write_log("[System] Baseline test completed, returning to idle", "IMPORTANT")
        return
        
    elif result == "calibrating":
        progress_d2 = len(state_machine.zero_drift_filters['d2'].baseline_data)
        progress_d3 = len(state_machine.zero_drift_filters['d3'].baseline_data)
        if progress_d2 % 10 == 0:
            write_log(f"[Calibration] Progress: D2={progress_d2}/50, D3={progress_d3}/50")
        return
        
    elif result == "recording_ready":
        write_log("[System] Ready for data recording - perform linear movements", "IMPORTANT")
        return
        
    elif result == "recording":
        # Apply other validations
        if not validate_data_quality(distance_2, distance_3, cam_coord):
            return
        
        # For static testing during recording - add filtered data to validators
        if any(validator.is_testing for validator in drift_validators.values()):
            drift_validators['d2'].add_static_sample(distance_2, d2_filtered)
            drift_validators['d3'].add_static_sample(distance_3, d3_filtered)
        
        # Apply change detection logic
        if should_record_data(d2_filtered, d3_filtered, cam_coord):
            data_status.record_data_point(
                timestamp, 
                distance_2, distance_3,
                d2_filtered, d3_filtered,
                cam_coord
            )

def force_stop_all_operations():
    """Force stop all operations"""
    global recording, state_machine, data_status
    
    if recording:
        recording = False
        write_log("[System] Force stop recording", "IMPORTANT")
        if len(data_status.recorded_data_raw) > 0 or len(data_status.recorded_data_filtered) > 0:
            save_to_csv(data_status.recorded_data_raw, data_status.recorded_data_filtered)
    
    # Stop static tests
    for validator in drift_validators.values():
        if validator.is_testing:
            validator.force_finish_test()
    
    state_machine.state = SystemState.IDLE
    write_log(f"[System] System state: {state_machine.state}", "IMPORTANT")
    display_help()

def display_help():
    """Display control instructions"""
    help_text = """
=== Control Instructions ===
Basic Operations:
  'T' - Start 60-second baseline zero drift test (test only, data not saved)
  'R' - Start data collection (segment 0, with baseline calibration)
  'E' - Stop data collection  
  'P' - Pause/Resume data collection (switch position, recalibrate baseline)
  'S' - Show current performance statistics
  'D' - Toggle Debug detailed information
  'I' - Show test status
  'Q' - Manually end test
  'X' - Force exit current mode, return to idle state
  'H' - Show help information
  'Esc' - Exit program and save data

Zero Drift Filtering:
  - Learns noise characteristics during baseline calibration
  - Applies adaptive filtering based on deviation magnitude
  - No time-based compensation, only noise-level filtering
=========================
    """
    write_log(help_text, "IMPORTANT")

def enhanced_serial_listener():
    """Enhanced serial listener"""
    global recording, data_status, latency_filter, state_machine
    
    buffer_manager = SerialBufferManager(ser)
    request_count = 0
    success_count = 0
    last_stats_time = time.time()
    
    while running:
        if recording or state_machine.state == SystemState.BASELINE_TESTING:
            
            request_start_time = time.time()
            request_count += 1
            
            buffer_manager.send_request()
            response = buffer_manager.read_packet_with_recovery(timeout=0.15)
            response_time = time.time()
            
            if response:
                if not latency_filter.check_lidar_latency(request_start_time, response_time):
                    continue
                
                parsed_data = parse_a69_data_with_debug(response)
                if parsed_data:
                    success_count += 1
                    distance_2, distance_3 = parsed_data
                    
                    tracker_request_time = time.time()
                    try:
                        cam_coord = tracker.get_pose_euler()
                        tracker_response_time = time.time()
                        
                        if not latency_filter.check_tracker_freshness(tracker_request_time, tracker_response_time):
                            continue
                        
                        recovery_needed = data_status.on_valid_data()
                        if recovery_needed:
                            data_status.reset_for_recovery()
                        
                        process_data(response_time, distance_2, distance_3, cam_coord)
                        
                        # Check if baseline testing is complete
                        if state_machine.state == SystemState.BASELINE_TESTING:
                            all_tests_complete = all(filter_obj.is_ready() for filter_obj in state_machine.zero_drift_filters.values())
                            if all_tests_complete:
                                recording = False
                                state_machine.state = SystemState.IDLE
                                write_log("[System] Baseline testing completed", "IMPORTANT")
                        
                    except Exception as e:
                        write_log(f"Failed to get tracker data: {e}", "ERROR")
                        data_status.on_invalid_data()
                
                else:
                    data_status.on_invalid_data()
            
            else:
                consecutive_failures = data_status.on_invalid_data()
                if consecutive_failures > 10:
                    buffer_manager.attempt_recovery()
            
            current_time = time.time()
            if current_time - last_stats_time > 30:
                success_rate = (success_count / request_count * 100) if request_count > 0 else 0
                write_log(f"Serial statistics: {request_count} requests, {success_count} successes, {success_rate:.1f}% success rate", "IMPORTANT")
                last_stats_time = current_time
        else:
            state_machine.state = SystemState.IDLE

        time.sleep(0.015)

def generate_performance_report():
    report = f"""
=== Performance Analysis Report ===
System State: {state_machine.state}
Sampling Segments: {data_status.current_segment}
Pause Count: {data_status.pause_count}
Latency Statistics: {latency_filter.get_latency_stats()}
Filter Statistics:
  D2: {state_machine.zero_drift_filters['d2'].get_status()}
  D3: {state_machine.zero_drift_filters['d3'].get_status()}
Data Statistics:
  Raw Data Points: {len(data_status.recorded_data_raw)}
  Filtered Data Points: {len(data_status.recorded_data_filtered)}
    """
    write_log(report, "IMPORTANT")
    return report

# Start threads
serial_thread = threading.Thread(target=enhanced_serial_listener, daemon=True)
serial_thread.start()

# Key listener
def on_press(key):
    global recording, data_status, state_machine
    try:
        initial_state = state_machine.state
        operation_successful = False
        
        if key.char == 't' and state_machine.state == SystemState.IDLE:
            if state_machine.start_baseline_test():
                recording = True
                # Start static testing
                drift_validators['d2'].start_static_test()
                drift_validators['d3'].start_static_test()
                write_log("[System] 开始基线测试，数据不会保存到CSV", "IMPORTANT")
                operation_successful = True
            
        elif key.char == 'r' and state_machine.state == SystemState.IDLE:
            if state_machine.start_data_collection():
                recording = True
                data_status = DataStatus()
                
                continuity_validators['d2'].reset()
                continuity_validators['d3'].reset()
                yaw_validator.reset()
                
                write_log("[System] 开始数据采集 (段0)，正在进行基线校准...", "IMPORTANT")
                operation_successful = True
            
        elif key.char == 'e' and (state_machine.state in [SystemState.RECORDING, SystemState.PAUSED, SystemState.WAITING_BASELINE]):
            recording = False
            state_machine.state = SystemState.IDLE
            write_log("[System] 停止数据采集", "IMPORTANT")
            save_to_csv(data_status.recorded_data_raw, data_status.recorded_data_filtered)
            generate_performance_report()
            operation_successful = True
            
        elif key.char == 'p' and (state_machine.state in [SystemState.RECORDING, SystemState.PAUSED, SystemState.WAITING_BASELINE]):
            if state_machine.state == SystemState.RECORDING:
                if state_machine.pause_for_position_change():
                    data_status.increment_segment_on_pause()
                    write_log("[Sampling] 已暂停，请移动到新位置后按'P'继续", "IMPORTANT")
                    operation_successful = True
            elif state_machine.state == SystemState.PAUSED:
                if state_machine.resume_after_position_change():
                    write_log("[Sampling] 开始恢复采样并校准基线", "IMPORTANT")
                    operation_successful = True
            elif state_machine.state == SystemState.WAITING_BASELINE:
                write_log("[Sampling] 基线校准进行中，请等待完成", "IMPORTANT")
                operation_successful = True
                    
        elif key.char == 'q':
            if state_machine.state == SystemState.BASELINE_TESTING:
                recording = False
                state_machine.state = SystemState.IDLE
                # Stop static tests
                for validator in drift_validators.values():
                    if validator.is_testing:
                        validator.force_finish_test()
                write_log(f"[System] 手动结束基线测试", "IMPORTANT")
                operation_successful = True
            else:
                write_log("[System] 没有活跃的基线测试", "IMPORTANT")
                operation_successful = True
                
        elif key.char == 'i':
            write_log(f"Current System State: {state_machine.state}", "IMPORTANT")
            write_log(f"Sampling Segment: {data_status.current_segment}, Pause Count: {data_status.pause_count}", "IMPORTANT")
            
            for name, filter_obj in state_machine.zero_drift_filters.items():
                write_log(f"Filter {name}: {filter_obj.get_status()}", "IMPORTANT")
            
            # Display static test status
            for name, validator in drift_validators.items():
                if validator.is_testing:
                    status = validator.get_test_status()
                    write_log(f"Static Test {name}: {status}", "IMPORTANT")
            
            operation_successful = True
                
        elif key.char == 's':
            generate_performance_report()
            operation_successful = True
            
        elif key.char == 'd':
            DEBUG_LEVEL["SERIAL_RAW"] = not DEBUG_LEVEL["SERIAL_RAW"]
            DEBUG_LEVEL["PROTOCOL"] = not DEBUG_LEVEL["PROTOCOL"]
            status = "ON" if DEBUG_LEVEL["SERIAL_RAW"] else "OFF"
            write_log(f"[System] Debug mode {status}", "IMPORTANT")
            operation_successful = True
            
        elif key.char == 'x':
            if state_machine.state != SystemState.IDLE:
                write_log(f"[System] 强制退出当前模式: {state_machine.state}", "IMPORTANT")
                force_stop_all_operations()
                operation_successful = True
            else:
                write_log("[System] 当前处于空闲状态", "IMPORTANT")
                display_help()
                operation_successful = True
                
        elif key.char == 'h':
            display_help()
            operation_successful = True
            
        if not operation_successful and key.char in ['r', 'e', 't', 'p']:
            if key.char == 'r' and initial_state != SystemState.IDLE:
                write_log(f"Warning: Current state is {initial_state}, cannot start data collection. Press 'X' to exit current mode first", "IMPORTANT")
            elif key.char == 'e' and initial_state not in [SystemState.RECORDING, SystemState.PAUSED, SystemState.WAITING_BASELINE]:
                write_log(f"Warning: Current state is {initial_state}, cannot stop data collection", "IMPORTANT")
            elif key.char == 't' and initial_state != SystemState.IDLE:
                write_log(f"Warning: Current state is {initial_state}, cannot start baseline test. Press 'X' to exit current mode first", "IMPORTANT")
            elif key.char == 'p' and initial_state not in [SystemState.RECORDING, SystemState.PAUSED, SystemState.WAITING_BASELINE]:
                write_log(f"Warning: Current state is {initial_state}, cannot pause/resume data collection", "IMPORTANT")
                
    except AttributeError:
        pass

def on_release(key):
    global running, data_status
    if key == keyboard.Key.esc:
        write_log("[System] Program exit, saving all data...", "IMPORTANT")
        force_stop_all_operations()
        running = False
        return False

listener = keyboard.Listener(on_press=on_press, on_release=on_release)
listener.start()

# Display initial control instructions
display_help()
write_log(f"[System] Current state: {state_machine.state}", "IMPORTANT")

try:
    while running:
        time.sleep(0.1)
except KeyboardInterrupt:
    write_log("[System] Received Ctrl+C, program exit...", "IMPORTANT")
    force_stop_all_operations()
    running = False
finally:
    if ser and ser.is_open:
        ser.close()
        write_log("[System] Serial port closed", "IMPORTANT")