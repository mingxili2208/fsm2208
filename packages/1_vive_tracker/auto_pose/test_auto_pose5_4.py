"""
=== SAMPLING LOGIC OVERVIEW ===

1. BASELINE CALIBRATION:
   - Each filter requires 50 samples to establish baseline and analyze drift
   - Baseline automatically calculates drift rate and noise characteristics
   - Baseline resets when 'P' is pressed (position change)

2. SAMPLING SEGMENTS:
   - Segment 0: Initial recording after pressing 'R'
   - Segment N: Each time 'P' is pressed and resumed, increment segment number
   - Each segment has independent baseline calibration

3. DRIFT COMPENSATION:
   - Real-time drift compensation based on baseline analysis
   - Adaptive filtering based on deviation magnitude
   - Separate drift models for D2 and D3 channels

4. DATA RECORDING CONDITIONS:
   - Must have valid baseline for both D2 and D3 channels
   - Significant change detection based on compensated data
   - Maximum change protection and continuity validation
   - Tracker yaw stability check

5. PAUSE/RESUME WORKFLOW:
   - Press 'P' during recording: pause + reset baseline + increment segment
   - Move to new position during pause
   - Press 'P' again: wait for baseline calibration (50 samples)
   - Resume recording when baseline ready
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

# Enhanced drift compensation filter
class DriftCompensationFilter:
    def __init__(self, name, baseline_samples=50):
        self.name = name
        self.baseline_samples_required = baseline_samples
        
        # Baseline calibration state
        self.baseline_data = []
        self.baseline_mean = None
        self.baseline_std = None
        self.baseline_trend = None  # Linear drift trend
        self.baseline_ready = False
        
        # Dynamic filtering state
        self.reference_value = None
        self.reference_time = None
        self.drift_rate = 0.0  # m/s drift rate
        
        # Adaptive parameters
        self.noise_threshold = 0.005  # 5mm
        self.max_correction = 0.02    # 20mm max correction
        
    def start_baseline_calibration(self):
        """Start baseline calibration - for static measurement"""
        self.baseline_data.clear()
        self.baseline_ready = False
        self.reference_value = None
        self.reference_time = None
        write_log(f"[DriftFilter-{self.name}] Starting baseline calibration...", "IMPORTANT")
    
    def add_baseline_sample(self, value, timestamp):
        """Add sample during baseline calibration"""
        if self.baseline_ready:
            return True  # Already calibrated
            
        self.baseline_data.append((value, timestamp))
        
        if len(self.baseline_data) >= self.baseline_samples_required:
            self._analyze_baseline()
            return True
        
        return False  # Still need more samples
    
    def _analyze_baseline(self):
        """Analyze baseline data to extract drift characteristics"""
        values = [item[0] for item in self.baseline_data]
        timestamps = [item[1] for item in self.baseline_data]
        
        self.baseline_mean = np.mean(values)
        self.baseline_std = np.std(values)
        
        # Calculate linear drift trend
        if len(self.baseline_data) > 10:
            # Use linear regression to find drift rate
            time_diffs = [(t - timestamps[0]) for t in timestamps]
            coeffs = np.polyfit(time_diffs, values, 1)
            self.drift_rate = coeffs[0]  # m/s
        else:
            self.drift_rate = 0.0
        
        # Set reference point (last baseline sample)
        self.reference_value = self.baseline_mean
        self.reference_time = timestamps[-1]
        self.baseline_ready = True
        
        write_log(f"[DriftFilter-{self.name}] Baseline analysis complete:", "IMPORTANT")
        write_log(f"  Mean: {self.baseline_mean:.4f}m", "IMPORTANT")
        write_log(f"  Std Dev: {self.baseline_std*1000:.2f}mm", "IMPORTANT")
        write_log(f"  Drift Rate: {self.drift_rate*1000:.3f}mm/s", "IMPORTANT")
    
    def apply_drift_compensation(self, raw_value, current_time):
        """Apply drift compensation to dynamic measurement"""
        if not self.baseline_ready:
            return raw_value  # No compensation available
        
        # Calculate expected drift since reference time
        time_elapsed = current_time - self.reference_time
        expected_drift = self.drift_rate * time_elapsed
        
        # Calculate current deviation from reference
        deviation = raw_value - (self.reference_value + expected_drift)
        
        # Apply adaptive correction
        if abs(deviation) < self.noise_threshold:
            # Small deviation - likely noise, apply partial correction
            correction_factor = 0.7
            compensated_value = raw_value - (deviation * correction_factor)
        elif abs(deviation) < self.max_correction:
            # Medium deviation - apply drift compensation
            compensated_value = raw_value - expected_drift
        else:
            # Large deviation - likely real movement, minimal compensation
            compensated_value = raw_value
        
        # Log significant corrections
        total_correction = raw_value - compensated_value
        if abs(total_correction) > 0.001:  # 1mm threshold
            write_log(f"[DriftFilter-{self.name}] Applied correction: {total_correction*1000:.1f}mm (drift: {expected_drift*1000:.1f}mm, deviation: {deviation*1000:.1f}mm)")
        
        return compensated_value
    
    def is_ready(self):
        return self.baseline_ready
    
    def get_status(self):
        if not self.baseline_ready:
            return f"Calibrating: {len(self.baseline_data)}/{self.baseline_samples_required} samples"
        else:
            return f"Ready - Drift: {self.drift_rate*1000:.3f}mm/s, Ref: {self.baseline_mean:.4f}m"
    
    def reset(self):
        """Reset all states for new position"""
        self.baseline_data.clear()
        self.baseline_ready = False
        self.reference_value = None
        self.reference_time = None
        self.drift_rate = 0.0
        write_log(f"[DriftFilter-{self.name}] Filter reset for new position", "IMPORTANT")

# Sampling state machine
class SamplingStateMachine:
    def __init__(self):
        self.state = SystemState.IDLE
        self.drift_filters = {
            'd2': DriftCompensationFilter("D2"),
            'd3': DriftCompensationFilter("D3")
        }
        
    def start_baseline_test(self):
        """Start standalone baseline test (for drift analysis only)"""
        if self.state != SystemState.IDLE:
            return False
        
        self.state = SystemState.BASELINE_TESTING
        for filter_obj in self.drift_filters.values():
            filter_obj.start_baseline_calibration()
        return True
    
    def start_data_collection(self):
        """Start data collection with baseline calibration"""
        if self.state != SystemState.IDLE:
            return False
        
        self.state = SystemState.WAITING_BASELINE
        for filter_obj in self.drift_filters.values():
            filter_obj.start_baseline_calibration()
        return True
    
    def process_sample(self, timestamp, d2_raw, d3_raw, cam_coord):
        """Process a single sample based on current state"""
        
        if self.state == SystemState.BASELINE_TESTING:
            # Baseline test mode - only for analysis
            d2_ready = self.drift_filters['d2'].add_baseline_sample(d2_raw, timestamp)
            d3_ready = self.drift_filters['d3'].add_baseline_sample(d3_raw, timestamp)
            
            if d2_ready and d3_ready:
                write_log("[System] Baseline test completed", "IMPORTANT")
                return "baseline_complete", None, None
            
            return "baseline_testing", None, None
            
        elif self.state == SystemState.WAITING_BASELINE:
            # Data collection mode - baseline calibration phase
            d2_ready = self.drift_filters['d2'].add_baseline_sample(d2_raw, timestamp)
            d3_ready = self.drift_filters['d3'].add_baseline_sample(d3_raw, timestamp)
            
            if d2_ready and d3_ready:
                self.state = SystemState.RECORDING
                write_log("[System] 基线校准完成，开始数据记录", "IMPORTANT")
                return "recording_ready", None, None
            
            return "calibrating", None, None
            
        elif self.state == SystemState.RECORDING:
            # Data recording mode - apply drift compensation
            d2_compensated = self.drift_filters['d2'].apply_drift_compensation(d2_raw, timestamp)
            d3_compensated = self.drift_filters['d3'].apply_drift_compensation(d3_raw, timestamp)
            
            return "recording", d2_compensated, d3_compensated
            
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
            for filter_obj in self.drift_filters.values():
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
        """Validate LiDAR data continuity
        Returns: (is_valid, should_record)
        """
        if self.last_valid_value is None:
            # First data point
            self.last_valid_value = current_value
            return True, True
        
        interval = abs(current_value - self.last_valid_value)
        
        if interval <= self.max_interval:
            # Valid interval
            if self.pending_value is not None:
                # Had pending value, now we have two consecutive valid points
                write_log(f"[Continuity-{self.name}] Valid data resumed: interval {interval:.3f}m <= {self.max_interval}m", "IMPORTANT")
                self.pending_value = None
                self.consecutive_invalid_count = 0
            
            self.last_valid_value = current_value
            return True, True
        else:
            # Invalid interval
            self.consecutive_invalid_count += 1
            write_log(f"[Continuity-{self.name}] Invalid interval detected: {interval:.3f}m > {self.max_interval}m (consecutive: {self.consecutive_invalid_count})", "DEBUG")
            
            if self.pending_value is None:
                # First invalid point, set as pending
                self.pending_value = current_value
                write_log(f"[Continuity-{self.name}] Setting pending value, waiting for next valid interval", "DEBUG")
                return False, False
            else:
                # Second invalid point, reset and set new pending
                write_log(f"[Continuity-{self.name}] Resetting, both previous values invalid", "DEBUG")
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
        self.max_sudden_change = np.radians(max_sudden_change)  # Convert to radians
        self.window_size = window_size
        self.yaw_history = []
        self.yaw_mean = None
        
    def validate_yaw_stability(self, current_yaw_rad):
        """Validate tracker yaw stability
        Returns: is_valid (bool)
        """
        if self.yaw_mean is None:
            # Initialize with first value
            self.yaw_history.append(current_yaw_rad)
            self.yaw_mean = current_yaw_rad
            return True
        
        # Calculate angular difference (handle wrapping)
        yaw_diff = self._angle_difference(current_yaw_rad, self.yaw_mean)
        
        if abs(yaw_diff) > self.max_sudden_change:
            write_log(f"[Yaw] Sudden yaw change detected: {np.degrees(yaw_diff):.1f} deg > {np.degrees(self.max_sudden_change):.1f} deg, rejecting data", "DEBUG")
            return False
        
        # Update history and mean
        self.yaw_history.append(current_yaw_rad)
        if len(self.yaw_history) > self.window_size:
            self.yaw_history.pop(0)
        
        # Update running mean
        self.yaw_mean = np.mean(self.yaw_history)
        return True
    
    def _angle_difference(self, angle1, angle2):
        """Calculate the difference between two angles, handling wrapping"""
        diff = angle1 - angle2
        # Normalize to [-pi, pi]
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
        self.recorded_data_raw = []      # Raw data
        self.recorded_data_filtered = [] # Filtered data
        self.pause_count = 0             # Pause count (number of times P is pressed)
        self.current_segment = 0         # Current sampling segment number
        self.has_paused = False          # Whether pause has been used
    
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
        # Record raw data
        self.recorded_data_raw.append((timestamp, distance_2_raw, distance_3_raw, cam_coord, self.current_segment))
        # Record filtered data
        self.recorded_data_filtered.append((timestamp, distance_2_filtered, distance_3_filtered, cam_coord, self.current_segment))
        
        write_log(f"[Data] Segment#{self.current_segment} recorded data point: raw d2={distance_2_raw:.3f}m, d3={distance_3_raw:.3f}m | filtered d2={distance_2_filtered:.3f}m, d3={distance_3_filtered:.3f}m | x={cam_coord[0]:.3f}, z={cam_coord[2]:.3f}")

# Serial buffer management class
class SerialBufferManager:
    def __init__(self, serial_port, expected_packet_size=10):
        self.ser = serial_port
        self.expected_size = expected_packet_size
        self.buffer = bytearray()
        self.sync_lost_count = 0
        self.recovery_attempts = 0
        self.last_flush_time = time.time()
        
    def find_valid_packet(self):
        """Find valid packet in buffer"""
        while len(self.buffer) >= self.expected_size:
            # Find frame header 0x55 0x7E
            start_idx = -1
            for i in range(len(self.buffer) - 1):
                if self.buffer[i] == 0x55 and self.buffer[i + 1] == 0x7E:
                    start_idx = i
                    break
            
            if start_idx == -1:
                # No frame header found, clear part of buffer
                self.buffer = self.buffer[len(self.buffer)//2:]
                write_log(f"Frame header not found, buffer remaining: {len(self.buffer)} bytes", "SERIAL_DEBUG")
                return None
            
            # Remove invalid data before frame header
            if start_idx > 0:
                removed_data = self.buffer[:start_idx]
                write_log(f"Removing invalid data before frame header: {' '.join([f'{b:02X}' for b in removed_data])}", "DEBUG")
                self.buffer = self.buffer[start_idx:]
            
            # Check if complete packet is available
            if len(self.buffer) >= self.expected_size:
                packet = self.buffer[:self.expected_size]
                
                # Verify frame tail
                if packet[8] == 0x7E and packet[9] == 0x55:
                    # Found valid packet
                    self.buffer = self.buffer[self.expected_size:]
                    return bytes(packet)
                else:
                    # Frame tail mismatch, continue searching
                    write_log(f"Frame tail mismatch: expected [7E 55], actual [{packet[8]:02X} {packet[9]:02X}]", "DEBUG")
                    self.buffer = self.buffer[1:]  # Remove first byte and continue searching
            else:
                # Not enough data, wait for more
                break
        
        return None
    
    def read_packet_with_recovery(self, timeout=0.1):
        """Packet reading with recovery mechanism"""
        start_time = time.time()
        
        while time.time() - start_time < timeout:
            try:
                # Read available data
                available = self.ser.in_waiting
                if available > 0:
                    new_data = self.ser.read(available)
                    self.buffer.extend(new_data)
                    write_log(f"Read {len(new_data)} bytes, total buffer length: {len(self.buffer)}", "SERIAL_DEBUG")
                
                # Try to parse packet
                packet = self.find_valid_packet()
                if packet:
                    self.sync_lost_count = 0
                    self.recovery_attempts = 0
                    return packet
                
                # Buffer overload protection
                if len(self.buffer) > 100:
                    write_log(f"Buffer overload ({len(self.buffer)} bytes), performing cleanup", "DEBUG")
                    self.buffer = self.buffer[-50:]  # Keep latest 50 bytes
                
                time.sleep(0.001)  # Brief wait
                
            except serial.SerialException as e:
                write_log(f"Serial read exception: {e}", "ERROR")
                return None
        
        # Timeout handling
        self.sync_lost_count += 1
        if self.sync_lost_count > 3:
            self.attempt_recovery()
        
        return None
    
    def attempt_recovery(self):
        """Connection recovery attempt"""
        self.recovery_attempts += 1
        write_log(f"Attempting connection recovery (attempt {self.recovery_attempts})", "DEBUG")
        
        try:
            # Clear serial buffers
            self.ser.reset_input_buffer()
            self.ser.reset_output_buffer()
            self.buffer.clear()
            
            # Send several test requests
            for i in range(3):
                self.send_request()
                time.sleep(0.02)
            
            write_log("Recovery operations performed: cleared buffers and sent test requests", "DEBUG")
            
        except Exception as e:
            write_log(f"Recovery operation failed: {e}", "ERROR")
    
    def send_request(self):
        """Send A69 data request"""
        tx_buf = bytearray([0x55, 0x7E, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x7E, 0x55])
        tx_buf[7] = calculate_checksum_r(tx_buf)
        try:
            self.ser.write(tx_buf)
            self.ser.flush()
            write_log(f"Request sent: {' '.join([f'{b:02X}' for b in tx_buf])}", "SERIAL_DEBUG")
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
    timestamp = datetime.now().strftime('%H:%M:%S.%f')[:-3]  # Include milliseconds
    log_message = f"[{timestamp}] {message}"
    
    # Write to file (English only)
    try:
        with open(LOG_FILE, "a", encoding='utf-8') as log_file:
            # Convert Chinese to English for file output
            file_message = log_message
            file_message = file_message.replace("基线校准完成，开始数据记录", "Baseline calibration completed, starting data recording")
            file_message = file_message.replace("切换到采样段", "Switched to sampling segment")
            file_message = file_message.replace("总暂停次数", "total pause count")
            log_file.write(file_message + "\n")
    except Exception as e:
        print(f"Error writing to log file: {e}")
    
    # Display in console (allow Chinese)
    if level in ["IMPORTANT", "ERROR"]:
        print(message)
    elif level == "DEBUG" and DEBUG_LEVEL["PROTOCOL"]:
        print(f"[DEBUG] {message}")
    elif level == "SERIAL_DEBUG" and DEBUG_LEVEL["SERIAL_RAW"]:
        print(f"[SERIAL] {message}")

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
    
    # Clear serial buffers
    ser.reset_input_buffer()
    ser.reset_output_buffer()
    
    if ser.is_open:
        write_log(f"Serial port configured: {SERIAL_PORT}, {BAUD_RATE} baud, timeout {ser.timeout}s", "IMPORTANT")
except serial.SerialException as e:
    write_log(f"Cannot open serial port {SERIAL_PORT}: {e}", "ERROR")
    sys.exit(1)

# Global object initialization
data_status = DataStatus()
running = True
recording = False
state_machine = SamplingStateMachine()

latency_filter = LatencyFilter(max_acceptable_latency_ms=80)

# LiDAR continuity validators
continuity_validators = {
    'd2': LiDARContinuityValidator("D2", max_interval=0.1),
    'd3': LiDARContinuityValidator("D3", max_interval=0.1)
}

# Tracker yaw validator
yaw_validator = TrackerYawValidator(max_sudden_change=10.0)

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
    """Enhanced A69 data parsing with detailed debug information"""
    
    # Record received raw data
    hex_data = ' '.join([f'{b:02X}' for b in response])
    write_log(f"Received raw data: [{hex_data}] (length: {len(response)})", "SERIAL_DEBUG")
    
    # Check packet length
    if len(response) != 10:
        write_log(f"Packet length error: expected 10 bytes, actual {len(response)} bytes", "DEBUG")
        write_log(f"Error data details: {hex_data}", "DEBUG")
        return None
    
    # Check frame header and tail
    if response[0] != 0x55:
        write_log(f"Frame header 1 error: expected 0x55, actual 0x{response[0]:02X}", "DEBUG")
        return None
    if response[1] != 0x7E:
        write_log(f"Frame header 2 error: expected 0x7E, actual 0x{response[1]:02X}", "DEBUG")
        return None
    if response[8] != 0x7E:
        write_log(f"Frame tail 1 error: expected 0x7E, actual 0x{response[8]:02X}", "DEBUG")
        return None
    if response[9] != 0x55:
        write_log(f"Frame tail 2 error: expected 0x55, actual 0x{response[9]:02X}", "DEBUG")
        return None

    # Checksum verification
    checksum = response[7]
    computed_checksum = calculate_checksum_r(response)
    
    if checksum != computed_checksum:
        write_log(f"Checksum error: calculated 0x{computed_checksum:02X}, received 0x{checksum:02X}", "DEBUG")
        write_log(f"Data used for checksum: {response[3]:02X} {response[4]:02X} {response[5]:02X} {response[6]:02X}", "DEBUG")
        return None

    # Parse distance data
    distance_2 = (response[3] << 8) | response[4]
    distance_3 = (response[5] << 8) | response[6]
    
    # Distance reasonableness check
    d2_meters = distance_2 / 1000.0
    d3_meters = distance_3 / 1000.0
    
    write_log(f"Parse successful: D2={distance_2}({d2_meters:.3f}m), D3={distance_3}({d3_meters:.3f}m)", "SERIAL_DEBUG")
    
    # Check distance reasonableness
    if d2_meters < 0.02 or d2_meters > 5.0:
        write_log(f"D2 distance abnormal: {d2_meters:.3f}m exceeds reasonable range [0.02-5.0]", "DEBUG")
    if d3_meters < 0.02 or d3_meters > 5.0:
        write_log(f"D3 distance abnormal: {d3_meters:.3f}m exceeds reasonable range [0.02-5.0]", "DEBUG")
    
    return d2_meters, d3_meters

def save_to_csv(data_raw, data_filtered):
    """Save raw data and filtered data to two separate CSV files"""
    
    # Save raw data
    with open(CSV_FILE_RAW, 'w', encoding='utf-8') as csv_file:
        csv_file.write("Timestamp,Distance_2,Distance_3,X,Y,Z,Roll,Yaw,Pitch,Segment\n")
        
        for timestamp, dis2, dis3, coord, segment in data_raw:
            csv_entry = f"{timestamp:.3f},{dis2:.3f},{dis3:.3f},{coord[0]:.4f},{coord[1]:.4f},{coord[2]:.4f},{coord[3]:.4f},{coord[4]:.4f},{coord[5]:.4f},{segment}\n"
            csv_file.write(csv_entry)
    
    # Save filtered data
    with open(CSV_FILE_FILTERED, 'w', encoding='utf-8') as csv_file:
        csv_file.write("Timestamp,Distance_2,Distance_3,X,Y,Z,Roll,Yaw,Pitch,Segment\n")
        
        for timestamp, dis2, dis3, coord, segment in data_filtered:
            csv_entry = f"{timestamp:.3f},{dis2:.3f},{dis3:.3f},{coord[0]:.4f},{coord[1]:.4f},{coord[2]:.4f},{coord[3]:.4f},{coord[4]:.4f},{coord[5]:.4f},{segment}\n"
            csv_file.write(csv_entry)
            
    write_log(f"[System] Saved raw data {len(data_raw)} entries to {CSV_FILE_RAW}", "IMPORTANT")
    write_log(f"[System] Saved filtered data {len(data_filtered)} entries to {CSV_FILE_FILTERED}", "IMPORTANT")

def format_data_for_log(data_name, prev_value, curr_value, change):
    return f"{data_name}: {prev_value:.4f} -> {curr_value:.4f} (Change: {change:.4f})"

def validate_data_quality(distance_2, distance_3, cam_coord):
    """Validate data quality using continuity and yaw validators"""
    # Validate LiDAR continuity
    d2_valid, d2_should_record = continuity_validators['d2'].validate_continuity(distance_2)
    d3_valid, d3_should_record = continuity_validators['d3'].validate_continuity(distance_3)
    
    # Validate tracker yaw stability
    yaw_valid = yaw_validator.validate_yaw_stability(cam_coord[4])  # Yaw is at index 4
    
    # Check if data is valid for recording
    data_valid_for_recording = d2_valid and d3_valid and yaw_valid and d2_should_record and d3_should_record
    
    if not data_valid_for_recording:
        if not d2_valid or not d2_should_record:
            write_log(f"[Validation] D2 data rejected due to continuity check", "DEBUG")
        if not d3_valid or not d3_should_record:
            write_log(f"[Validation] D3 data rejected due to continuity check", "DEBUG")
        if not yaw_valid:
            write_log(f"[Validation] Data rejected due to yaw instability", "DEBUG")
    
    return data_valid_for_recording

def should_record_data(d2_comp, d3_comp, cam_coord):
    """Determine if data should be recorded based on significant changes"""
    if data_status.last_recorded_data is None:
        data_status.last_recorded_data = (d2_comp, d3_comp, cam_coord[0], cam_coord[2])
        return True
    
    # Calculate changes using compensated data
    current_data = (d2_comp, d3_comp, cam_coord[0], cam_coord[2])
    changes = [abs(current_data[i] - data_status.last_recorded_data[i]) for i in range(4)]
    
    # Check thresholds
    significant_changes = []
    thresholds = [0.008, 0.008, 0.005, 0.005]  # D2, D3, X, Z
    names = ["Distance_2", "Distance_3", "X", "Z"]
    
    for i, (change, threshold) in enumerate(zip(changes, thresholds)):
        if change > threshold:
            significant_changes.append(f"{names[i]}={change:.4f}>{threshold}")
    
    # Check for abnormal changes
    if any(change > max_change_threshold for change in changes):
        write_log(f"[Warning] Data change too large, update comparison baseline but do not record", "IMPORTANT")
        data_status.last_recorded_data = current_data
        return False
    
    if significant_changes:
        write_log(f"[Info] Significant change detected: {', '.join(significant_changes)}", "IMPORTANT")
        data_status.last_recorded_data = current_data
        return True
    else:
        write_log("[Info] No significant change, skip recording")
        return False

def optimized_process_data(timestamp, distance_2, distance_3, cam_coord):
    """Optimized data processing with drift compensation"""
    global state_machine, data_status
    
    processing_start = time.time()
    
    # Process through state machine
    result, d2_compensated, d3_compensated = state_machine.process_sample(
        timestamp, distance_2, distance_3, cam_coord
    )
    
    # Handle different processing results
    if result == "baseline_testing":
        # Only for baseline analysis, don't record data
        write_log(f"[Baseline] D2: {distance_2:.3f}m, D3: {distance_3:.3f}m")
        return
        
    elif result == "baseline_complete":
        # Baseline test finished
        state_machine.state = SystemState.IDLE
        write_log("[System] Baseline test completed, returning to idle", "IMPORTANT")
        return
        
    elif result == "calibrating":
        # Still calibrating baseline
        progress_d2 = len(state_machine.drift_filters['d2'].baseline_data)
        progress_d3 = len(state_machine.drift_filters['d3'].baseline_data)
        if progress_d2 % 10 == 0:  # Update every 10 samples
            write_log(f"[Calibration] Progress: D2={progress_d2}/50, D3={progress_d3}/50")
        return
        
    elif result == "recording_ready":
        # Baseline calibration complete, ready for recording
        write_log("[System] Ready for data recording - perform linear movements", "IMPORTANT")
        return
        
    elif result == "recording":
        # Normal data recording with drift compensation
        
        # Apply other validations (continuity, yaw stability, etc.)
        if not validate_data_quality(distance_2, distance_3, cam_coord):
            return
        
        # Apply change detection logic
        if should_record_data(d2_compensated, d3_compensated, cam_coord):
            data_status.record_data_point(
                timestamp, 
                distance_2, distance_3,  # Raw data
                d2_compensated, d3_compensated,  # Compensated data
                cam_coord
            )
            
            write_log(f"[Record] Raw: D2={distance_2:.3f}, D3={distance_3:.3f} | Compensated: D2={d2_compensated:.3f}, D3={d3_compensated:.3f}")
    
    # Record processing time
    processing_time = time.time() - processing_start
    if processing_time > 0.005:
        write_log(f"[Latency] Data processing time: {processing_time*1000:.2f}ms", "DEBUG")

def force_stop_all_operations():
    """Force stop all operations, return to idle state"""
    global recording, state_machine, data_status
    
    # Stop recording
    if recording:
        recording = False
        write_log("[System] Force stop recording", "IMPORTANT")
        if len(data_status.recorded_data_raw) > 0 or len(data_status.recorded_data_filtered) > 0:
            save_to_csv(data_status.recorded_data_raw, data_status.recorded_data_filtered)
            write_log(f"[System] Saved raw data {len(data_status.recorded_data_raw)} entries, filtered data {len(data_status.recorded_data_filtered)} entries", "IMPORTANT")
    
    # Reset system state
    state_machine.state = SystemState.IDLE
    write_log(f"[System] System state: {state_machine.state}", "IMPORTANT")
    
    # Display control instructions
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

Data Collection Process:
  1. Press 'T' for baseline test (optional, 60s test, data not saved)
  2. Press 'R' to start data collection (segment 0, baseline calibration first)
  3. After baseline calibration, perform linear movement
  4. Press 'P' to pause and switch to new position
  5. Press 'P' again to resume (new baseline calibration + new segment)
  6. Repeat steps 4-5 as needed
  7. Press 'E' to finish and save data to CSV files

Data Validation:
  - LiDAR continuity: adjacent data interval must be <0.1m
  - Tracker yaw stability: sudden changes >10 degrees rejected
  - Drift compensation applied based on baseline analysis
=========================
    """
    write_log(help_text, "IMPORTANT")

# Enhanced serial listener function
def enhanced_serial_listener():
    global recording, data_status, latency_filter, state_machine
    
    # Create serial buffer manager
    buffer_manager = SerialBufferManager(ser)
    
    # Performance monitoring
    request_count = 0
    success_count = 0
    last_stats_time = time.time()
    
    while running:
        if recording or state_machine.state == SystemState.BASELINE_TESTING:  # Recording mode includes normal recording and baseline testing
            
            request_start_time = time.time()
            request_count += 1
            
            # Send request
            buffer_manager.send_request()
            
            # Read response
            response = buffer_manager.read_packet_with_recovery(timeout=0.15)
            response_time = time.time()
            
            if response:
                # Check latency
                latency = response_time - request_start_time
                if not latency_filter.check_lidar_latency(request_start_time, response_time):
                    write_log(f"Data rejected due to high latency: {latency*1000:.1f}ms", "DEBUG")
                    continue
                
                # Parse data
                parsed_data = parse_a69_data_with_debug(response)
                if parsed_data:
                    success_count += 1
                    distance_2, distance_3 = parsed_data
                    
                    # Get tracker data
                    tracker_request_time = time.time()
                    try:
                        cam_coord = tracker.get_pose_euler()
                        tracker_response_time = time.time()
                        
                        if not latency_filter.check_tracker_freshness(tracker_request_time, tracker_response_time):
                            write_log(f"Tracker response too slow: {(tracker_response_time-tracker_request_time)*1000:.1f}ms", "DEBUG")
                            continue
                        
                        # Process data
                        recovery_needed = data_status.on_valid_data()
                        if recovery_needed:
                            data_status.reset_for_recovery()
                        
                        optimized_process_data(response_time, distance_2, distance_3, cam_coord)
                        
                        # Check if baseline testing is complete
                        if state_machine.state == SystemState.BASELINE_TESTING:
                            all_tests_complete = all(filter_obj.is_ready() for filter_obj in state_machine.drift_filters.values())
                            if all_tests_complete:
                                recording = False
                                state_machine.state = SystemState.IDLE
                                write_log("[System] Baseline testing completed, returning to idle state", "IMPORTANT")
                                write_log(f"[System] System state: {state_machine.state}", "IMPORTANT")
                        
                    except Exception as e:
                        write_log(f"Failed to get tracker data: {e}", "ERROR")
                        data_status.on_invalid_data()
                
                else:
                    # Parse failed
                    data_status.on_invalid_data()
                    write_log("Data parse failed", "DEBUG")
            
            else:
                # No response received
                consecutive_failures = data_status.on_invalid_data()
                write_log(f"No response received (consecutive failures: {consecutive_failures})", "DEBUG")
                
                # Handle severe failures
                if consecutive_failures > 10:
                    write_log("Too many consecutive failures, performing deep recovery", "ERROR")
                    buffer_manager.attempt_recovery()
            
            # Periodic statistics output
            current_time = time.time()
            if current_time - last_stats_time > 30:  # Every 30 seconds
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
  D2: {state_machine.drift_filters['d2'].get_status()}
  D3: {state_machine.drift_filters['d3'].get_status()}
Data Statistics:
  Raw Data Points: {len(data_status.recorded_data_raw)}
  Filtered Data Points: {len(data_status.recorded_data_filtered)}
    """
    write_log(report, "IMPORTANT")
    return report

# Start threads
serial_thread = threading.Thread(target=enhanced_serial_listener, daemon=True)
serial_thread.start()

# Enhanced key listener
def on_press(key):
    global recording, data_status, state_machine
    try:
        # Record state before operation
        initial_state = state_machine.state
        operation_successful = False
        
        if key.char == 't' and state_machine.state == SystemState.IDLE:
            # Start baseline testing
            if state_machine.start_baseline_test():
                recording = True
                write_log("[System] 开始基线测试，数据不会保存到CSV", "IMPORTANT")
                write_log(f"[System] System state: {state_machine.state}", "IMPORTANT")
                operation_successful = True
            
        elif key.char == 'r' and state_machine.state == SystemState.IDLE:
            # Start data collection
            if state_machine.start_data_collection():
                recording = True
                data_status = DataStatus()
                
                # Reset validators
                continuity_validators['d2'].reset()
                continuity_validators['d3'].reset()
                yaw_validator.reset()
                
                write_log("[System] 开始数据采集 (段0)，正在进行基线校准...", "IMPORTANT")
                write_log(f"[System] System state: {state_machine.state}", "IMPORTANT")
                operation_successful = True
            
        elif key.char == 'e' and (state_machine.state in [SystemState.RECORDING, SystemState.PAUSED, SystemState.WAITING_BASELINE]):
            recording = False
            state_machine.state = SystemState.IDLE
            write_log("[System] 停止数据采集", "IMPORTANT")
            write_log(f"[System] System state: {state_machine.state}", "IMPORTANT")
            save_to_csv(data_status.recorded_data_raw, data_status.recorded_data_filtered)
            generate_performance_report()
            operation_successful = True
            
        elif key.char == 'p' and (state_machine.state in [SystemState.RECORDING, SystemState.PAUSED, SystemState.WAITING_BASELINE]):
            if state_machine.state == SystemState.RECORDING:
                # Pause recording
                if state_machine.pause_for_position_change():
                    data_status.increment_segment_on_pause()
                    write_log("[Sampling] 已暂停，请移动到新位置后按'P'继续", "IMPORTANT")
                    operation_successful = True
            elif state_machine.state == SystemState.PAUSED:
                # Resume recording and start baseline calibration
                if state_machine.resume_after_position_change():
                    write_log("[Sampling] 开始恢复采样并校准基线", "IMPORTANT")
                    operation_successful = True
            elif state_machine.state == SystemState.WAITING_BASELINE:
                write_log("[Sampling] 基线校准进行中，请等待完成", "IMPORTANT")
                operation_successful = True
                    
        elif key.char == 'q':
            # Manually end static test
            if state_machine.state == SystemState.BASELINE_TESTING:
                recording = False
                state_machine.state = SystemState.IDLE
                write_log(f"[System] 手动结束基线测试", "IMPORTANT")
                write_log(f"[System] System state: {state_machine.state}", "IMPORTANT")
                operation_successful = True
            else:
                write_log("[System] 没有活跃的基线测试", "IMPORTANT")
                operation_successful = True
                
        elif key.char == 'i':
            # Display test status information
            write_log(f"Current System State: {state_machine.state}", "IMPORTANT")
            write_log(f"Sampling Segment: {data_status.current_segment}, Pause Count: {data_status.pause_count}", "IMPORTANT")
            
            # Display baseline status
            for name, filter_obj in state_machine.drift_filters.items():
                write_log(f"Filter {name}: {filter_obj.get_status()}", "IMPORTANT")
            
            operation_successful = True
                
        elif key.char == 's':
            generate_performance_report()
            operation_successful = True
            
        elif key.char == 'd':
            # Toggle debug mode
            DEBUG_LEVEL["SERIAL_RAW"] = not DEBUG_LEVEL["SERIAL_RAW"]
            DEBUG_LEVEL["PROTOCOL"] = not DEBUG_LEVEL["PROTOCOL"]
            status = "ON" if DEBUG_LEVEL["SERIAL_RAW"] else "OFF"
            write_log(f"[System] Debug mode {status}", "IMPORTANT")
            operation_successful = True
            
        elif key.char == 'x':
            # Force exit current mode
            if state_machine.state != SystemState.IDLE:
                write_log(f"[System] 强制退出当前模式: {state_machine.state}", "IMPORTANT")
                force_stop_all_operations()
                operation_successful = True
            else:
                write_log("[System] 当前处于空闲状态", "IMPORTANT")
                display_help()
                operation_successful = True
                
        elif key.char == 'h':
            # Display help
            display_help()
            operation_successful = True
            
        # Show status check hint only when operation fails
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
        force_stop_all_operations()  # Ensure all operations are properly stopped
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