"""
Pure Data Collection Script
Philosophy: Collect everything, decide nothing
"""
import time
import serial
import threading
import numpy as np
import queue
import os
import json
from datetime import datetime
from pynput import keyboard
from vive_tracker import ViveTrackerModule

class DataCollector:
    def __init__(self, config_file="config.json"):
        self.config = self.load_config(config_file)
        self.running = False
        self.collecting = False
        
        # Pure data storage - no filtering, no decisions
        self.lidar_data = []
        self.tracker_data = []
        self.collection_metadata = {}
        
        # Thread-safe queues for cross-thread communication
        self.lidar_queue = queue.Queue()
        self.tracker_queue = queue.Queue()
        
        # Initialize hardware
        self.setup_hardware()
        
    def load_config(self, config_file):
        """Load configuration from JSON file"""
        default_config = {
            "serial_port": "/dev/ttyUSB0",
            "baud_rate": 9600,
            "tracker_name": "tracker_1",
            "lidar_target_hz": 10,
            "tracker_target_hz": 50,
            "collection_timeout": 3600,  # 1 hour max
            "output_dir": "raw_data"
        }
        
        try:
            with open(config_file, 'r') as f:
                user_config = json.load(f)
            default_config.update(user_config)
        except FileNotFoundError:
            print(f"Config file {config_file} not found, using defaults")
            # Create default config file
            with open(config_file, 'w') as f:
                json.dump(default_config, f, indent=2)
        
        return default_config
    
    def setup_hardware(self):
        """Initialize hardware connections"""
        print("Initializing hardware...")
        
        # Setup serial port
        try:
            self.ser = serial.Serial(
                port=self.config["serial_port"],
                baudrate=self.config["baud_rate"],
                timeout=0.1,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE
            )
            self.ser.reset_input_buffer()
            self.ser.reset_output_buffer()
            print(f"Serial port {self.config['serial_port']} connected successfully")
        except Exception as e:
            print(f"Serial port error: {e}")
            raise
        
        # Setup Vive Tracker
        try:
            self.vtm = ViveTrackerModule()
            self.tracker = self.vtm.devices.get(self.config["tracker_name"])
            if self.tracker is None:
                raise Exception(f"Tracker '{self.config['tracker_name']}' not found")
            print(f"Tracker '{self.config['tracker_name']}' connected successfully")
        except Exception as e:
            print(f"Tracker error: {e}")
            raise
    
    def calculate_checksum(self, data):
        """Calculate A69 protocol checksum"""
        return data[3] ^ data[4] ^ data[5] ^ data[6]
    
    def parse_lidar_packet(self, packet):
        """Parse A69 LiDAR packet"""
        if len(packet) != 10:
            return None
        
        if (packet[0] != 0x55 or packet[1] != 0x7E or 
            packet[8] != 0x7E or packet[9] != 0x55):
            return None
        
        # Verify checksum
        if packet[7] != self.calculate_checksum(packet):
            return None
        
        # Extract distances
        d2 = ((packet[3] << 8) | packet[4]) / 1000.0  # Convert to meters
        d3 = ((packet[5] << 8) | packet[6]) / 1000.0
        
        return d2, d3
    
    def lidar_collection_thread(self):
        """Dedicated thread for LiDAR data collection"""
        print("LiDAR collection thread started")
        request_packet = bytearray([0x55, 0x7E, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x7E, 0x55])
        request_packet[7] = self.calculate_checksum(request_packet)
        
        collection_count = 0
        error_count = 0
        
        while self.running:
            if not self.collecting:
                time.sleep(0.1)
                continue
            
            try:
                # Send request
                self.ser.write(request_packet)
                self.ser.flush()
                request_time = time.time()
                
                # Read response with timeout
                response = self.ser.read(10)
                response_time = time.time()
                
                if len(response) == 10:
                    parsed = self.parse_lidar_packet(response)
                    if parsed:
                        d2, d3 = parsed
                        # Store raw data with high-precision timestamp
                        data_point = {
                            'timestamp': response_time,
                            'request_time': request_time,
                            'distance_2': d2,
                            'distance_3': d3,
                            'latency_ms': (response_time - request_time) * 1000
                        }
                        self.lidar_queue.put(data_point)
                        collection_count += 1
                        
                        if collection_count % 100 == 0:
                            print(f"LiDAR: {collection_count} samples collected, {error_count} errors")
                    else:
                        error_count += 1
                else:
                    error_count += 1
                    
                # Target frequency control
                time.sleep(max(0, 1.0/self.config["lidar_target_hz"] - (response_time - request_time)))
                
            except Exception as e:
                error_count += 1
                if error_count % 10 == 0:
                    print(f"LiDAR error (count: {error_count}): {e}")
                time.sleep(0.1)
    
    def tracker_collection_thread(self):
        """Dedicated thread for Tracker data collection"""
        print("Tracker collection thread started")
        collection_count = 0
        error_count = 0
        
        while self.running:
            if not self.collecting:
                time.sleep(0.1)
                continue
            
            try:
                request_time = time.time()
                pose = self.tracker.get_pose_euler()
                response_time = time.time()
                
                # Store raw data with high-precision timestamp
                data_point = {
                    'timestamp': response_time,
                    'request_time': request_time,
                    'x': pose[0],
                    'y': pose[1], 
                    'z': pose[2],
                    'roll': pose[3],
                    'pitch': pose[4],
                    'yaw': pose[5],
                    'latency_ms': (response_time - request_time) * 1000
                }
                self.tracker_queue.put(data_point)
                collection_count += 1
                
                if collection_count % 500 == 0:
                    print(f"Tracker: {collection_count} samples collected, {error_count} errors")
                
                # Target frequency control
                time.sleep(max(0, 1.0/self.config["tracker_target_hz"] - (response_time - request_time)))
                
            except Exception as e:
                error_count += 1
                if error_count % 50 == 0:
                    print(f"Tracker error (count: {error_count}): {e}")
                time.sleep(0.02)
    
    def data_aggregation_thread(self):
        """Thread to aggregate data from queues into main storage"""
        while self.running:
            # Process LiDAR queue
            try:
                while True:
                    data_point = self.lidar_queue.get_nowait()
                    self.lidar_data.append(data_point)
            except queue.Empty:
                pass
            
            # Process Tracker queue
            try:
                while True:
                    data_point = self.tracker_queue.get_nowait()
                    self.tracker_data.append(data_point)
            except queue.Empty:
                pass
            
            time.sleep(0.1)
    
    def start_collection(self):
        """Start data collection"""
        if self.collecting:
            print("Collection already in progress")
            return False
        
        print("Starting data collection...")
        self.collecting = True
        self.collection_metadata = {
            'start_time': time.time(),
            'start_time_str': datetime.now().isoformat(),
            'config': self.config.copy()
        }
        
        # Clear previous data
        self.lidar_data.clear()
        self.tracker_data.clear()
        
        print("Collection started. Press 'E' to stop.")
        return True
    
    def stop_collection(self):
        """Stop data collection and save data"""
        if not self.collecting:
            print("No collection in progress")
            return None
        
        print("Stopping data collection...")
        self.collecting = False
        
        # Wait a moment for final data points
        time.sleep(0.5)
        
        # Process remaining queue items
        final_lidar_count = 0
        final_tracker_count = 0
        
        try:
            while True:
                data_point = self.lidar_queue.get_nowait()
                self.lidar_data.append(data_point)
                final_lidar_count += 1
        except queue.Empty:
            pass
        
        try:
            while True:
                data_point = self.tracker_queue.get_nowait()
                self.tracker_data.append(data_point)
                final_tracker_count += 1
        except queue.Empty:
            pass
        
        if final_lidar_count > 0 or final_tracker_count > 0:
            print(f"Processed {final_lidar_count} final LiDAR samples, {final_tracker_count} final Tracker samples")
        
        # Update metadata
        self.collection_metadata.update({
            'end_time': time.time(),
            'end_time_str': datetime.now().isoformat(),
            'duration_seconds': time.time() - self.collection_metadata['start_time'],
            'lidar_samples': len(self.lidar_data),
            'tracker_samples': len(self.tracker_data)
        })
        
        # Save data
        filename = self.save_data()
        print(f"Collection stopped. Data saved to: {filename}")
        return filename
    
    def save_data(self):
        """Save collected data to NPZ file"""
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        os.makedirs(self.config["output_dir"], exist_ok=True)
        filename = os.path.join(self.config["output_dir"], f"raw_data_{timestamp}.npz")
        
        # Convert lists to structured arrays for efficient storage
        lidar_array = np.array([(d['timestamp'], d['request_time'], d['distance_2'], 
                                d['distance_3'], d['latency_ms']) for d in self.lidar_data],
                              dtype=[('timestamp', 'f8'), ('request_time', 'f8'), 
                                   ('distance_2', 'f4'), ('distance_3', 'f4'), ('latency_ms', 'f4')])
        
        tracker_array = np.array([(d['timestamp'], d['request_time'], d['x'], d['y'], d['z'],
                                  d['roll'], d['pitch'], d['yaw'], d['latency_ms']) for d in self.tracker_data],
                                dtype=[('timestamp', 'f8'), ('request_time', 'f8'),
                                      ('x', 'f4'), ('y', 'f4'), ('z', 'f4'),
                                      ('roll', 'f4'), ('pitch', 'f4'), ('yaw', 'f4'), ('latency_ms', 'f4')])
        
        # Save with compression
        np.savez_compressed(filename,
                           lidar_data=lidar_array,
                           tracker_data=tracker_array,
                           metadata=self.collection_metadata)
        
        return filename
    
    def display_status(self):
        """Display current collection status"""
        if self.collecting:
            duration = time.time() - self.collection_metadata['start_time']
            lidar_rate = len(self.lidar_data) / duration if duration > 0 else 0
            tracker_rate = len(self.tracker_data) / duration if duration > 0 else 0
            
            print(f"""
=== Collection Status ===
Duration: {duration:.1f}s
LiDAR: {len(self.lidar_data)} samples ({lidar_rate:.1f} Hz)
Tracker: {len(self.tracker_data)} samples ({tracker_rate:.1f} Hz)
Target rates: LiDAR {self.config['lidar_target_hz']}Hz, Tracker {self.config['tracker_target_hz']}Hz
=========================""")
        else:
            print("No collection in progress")
    
    def run(self):
        """Main run loop with keyboard control"""
        print("""
=== Pure Data Collector ===
Controls:
  'S' - Start collection
  'E' - End collection and save
  'T' - Show current status
  'Q' - Quit program
  
Hardware initialized. Ready to collect.
=============================""")
        
        self.running = True
        
        # Start collection threads
        lidar_thread = threading.Thread(target=self.lidar_collection_thread, daemon=True)
        tracker_thread = threading.Thread(target=self.tracker_collection_thread, daemon=True)
        aggregation_thread = threading.Thread(target=self.data_aggregation_thread, daemon=True)
        
        lidar_thread.start()
        tracker_thread.start()
        aggregation_thread.start()
        
        def on_key_press(key):
            try:
                if key.char == 's':
                    self.start_collection()
                elif key.char == 'e':
                    self.stop_collection()
                elif key.char == 't':
                    self.display_status()
                elif key.char == 'q':
                    print("Shutting down...")
                    if self.collecting:
                        self.stop_collection()
                    self.running = False
                    return False
            except AttributeError:
                if key == keyboard.Key.esc:
                    print("Shutting down...")
                    if self.collecting:
                        self.stop_collection()
                    self.running = False
                    return False
        
        # Start keyboard listener
        listener = keyboard.Listener(on_press=on_key_press)
        listener.start()
        
        try:
            while self.running:
                time.sleep(0.1)
        except KeyboardInterrupt:
            print("Interrupted by user")
        finally:
            self.running = False
            if self.collecting:
                self.stop_collection()
            if self.ser and self.ser.is_open:
                self.ser.close()
            print("Data collector shut down")

if __name__ == "__main__":
    collector = DataCollector()
    collector.run()