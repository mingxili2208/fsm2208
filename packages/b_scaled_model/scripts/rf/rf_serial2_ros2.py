import serial
import struct
import time
import argparse
import logging
import os
import datetime
import threading
import queue
import math
from pynput import keyboard
import rclpy
from rclpy.node import Node
from autoware_auto_control_msgs.msg import AckermannControlCommand

# Communication protocol definition
FRAME_HEADER1 = 0xAA  # Primary frame header identifier
FRAME_HEADER2 = 0x7E  # Secondary frame header identifier
FRAME_TAIL = 0x55     # Frame tail identifier

# Data packet type definition
class PacketType:
    PACKET_MOTION = 0x01   # Motion data packet (contains linear velocity, angle, etc.)
    PACKET_COMMAND = 0x02  # Command data packet
    PACKET_STATUS = 0x03   # Status data packet
    PACKET_ACK = 0x04      # Acknowledgment data packet

# Speed limit constants (in mm/s)
MAX_SPEED = 1000     # 1 m/s
MIN_SPEED = -1000    # -1 m/s
MIN_REVERSE_START = -600  # Minimum reverse speed to start moving from stop (-0.6 m/s)

# Thresholds for command filtering
SPEED_CHANGE_THRESHOLD = 0.2  # m/s (don't send if change is less than this, except for deceleration)
ANGLE_CHANGE_THRESHOLD = 0.2  # degrees
EMERGENCY_BRAKE_THRESHOLD = -1.1  # m/s²

# New acceleration control constants
MAX_ACCELERATION_STEP = 0.4  # m/s - each acceleration step will be at most this value
ACCELERATION_THRESHOLD = 0.5  # m/s - when speed change exceeds this, use gradual acceleration
ACCELERATION_DELAY = 0.1     # seconds between acceleration steps

# High speed handling constants
HIGH_SPEED_THRESHOLD = 1.0   # m/s - threshold for high speed command
HIGH_SPEED_INTERMEDIATE_RATIO = 0.5  # ratio of high speed to use as intermediate step
HIGH_SPEED_INTERMEDIATE_DELAY = 0.01  # seconds (10ms) between intermediate and full speed commands

# Watchdog timer constant
WATCHDOG_TIMEOUT = 1.5  # seconds - resend command if no new command received within this time

class RF24Sender:
    def __init__(self, port, baudrate=115200):
        """Initialize RF24 sender"""
        # Create logs directory if not exists
        log_dir = "logs"
        if not os.path.exists(log_dir):
            os.makedirs(log_dir)
        
        # Configure logging
        timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        comm_log_filename = os.path.join(log_dir, f"comm_details_{timestamp}.log")
        
        file_handler = logging.FileHandler(comm_log_filename)
        file_handler.setLevel(logging.DEBUG)
        file_format = logging.Formatter('%(asctime)s - %(levelname)s - %(message)s')
        file_handler.setFormatter(file_format)
        
        console_handler = logging.StreamHandler()
        console_handler.setLevel(logging.INFO)
        console_handler.setFormatter(file_format)
        
        self.logger = logging.getLogger(__name__)
        self.logger.setLevel(logging.DEBUG)
        self.logger.addHandler(file_handler)
        self.logger.addHandler(console_handler)
        
        self.logger.info(f"=== RF24 Sender initialized at {timestamp} ===")
        self.logger.info(f"Communication log file: {comm_log_filename}")
        
        self.serial = serial.Serial(port, baudrate, timeout=1)
        self.sequence_counter = 0
        self.last_linear_vel = 0  # Track last sent linear velocity for logging
        self.packets_sent = 0     # Track total packets sent
        self.logger.info(f"Connected to {port} at {baudrate} baud")
        time.sleep(2)  # Wait for serial port to stabilize
    
    def calculate_checksum(self, data):
        """Calculate checksum"""
        return sum(data) & 0xFF
    
    def pack_motion_data(self, device_id, linear_vel, angular_vel, linear_acc=0, angular_acc=0):
        """Pack motion data packet - following RF24L01 specifications with dual headers"""
        # Enforce speed limits
        linear_vel = max(MIN_SPEED, min(MAX_SPEED, linear_vel))
        
        # Apply minimum reverse start threshold when changing from stop to reverse
        if self.last_linear_vel == 0 and linear_vel < 0:
            linear_vel = min(linear_vel, MIN_REVERSE_START)
        
        # Initialize packet without the length byte (system will generate it)
        # We now use 31 bytes instead of 32 since byte 0 is automatically generated
        packet = bytearray(31)
        
        # Start filling packet content from position 0 (which will be position 1 in the final packet)
        packet[0] = FRAME_HEADER1  # First frame header
        packet[1] = FRAME_HEADER2  # Second frame header
        packet[2] = PacketType.PACKET_MOTION  # Packet type
        packet[3] = device_id  # Device ID
        
        # Fill velocity and acceleration data (using little-endian byte order, compatible with Arduino)
        struct.pack_into('<h', packet, 4, linear_vel)    # Linear velocity (2 bytes)
        struct.pack_into('<h', packet, 6, angular_vel)   # Angular velocity (2 bytes)
        struct.pack_into('<h', packet, 8, linear_acc)    # Linear acceleration (2 bytes)
        struct.pack_into('<h', packet, 10, angular_acc)  # Angular acceleration (2 bytes)
        
        # Fill sequence number
        packet[12] = self.sequence_counter
        self.sequence_counter = (self.sequence_counter + 1) & 0xFF
        
        # Calculate checksum (excluding the checksum field itself and frame tail)
        checksum = self.calculate_checksum(packet[2:13])
        packet[13] = checksum
        
        # Fill frame tail
        packet[14] = FRAME_TAIL
        
        return packet
    
    def send_motion_data(self, device_id, linear_vel, angular_vel, linear_acc=0, angular_acc=0):
        """Send motion data packet"""
        # Enforce speed limits
        linear_vel = max(MIN_SPEED, min(MAX_SPEED, linear_vel))
        
        # Apply minimum reverse start threshold when changing from stop to reverse
        if self.last_linear_vel == 0 and linear_vel < 0:
            original_vel = linear_vel
            linear_vel = min(linear_vel, MIN_REVERSE_START)
            if original_vel != linear_vel:
                self.logger.info(f"Note: Adjusted reverse speed from {original_vel} to {linear_vel} for better startup")
        
        # Check speed difference (just for logging, not enforcing)
        speed_diff = abs(linear_vel - self.last_linear_vel)
        if speed_diff > 1000:
            self.logger.warning(f"Warning: Large speed change detected: {self.last_linear_vel/1000:.2f} m/s -> {linear_vel/1000:.2f} m/s ({speed_diff/1000:.2f} m/s difference)")
        
        packet = self.pack_motion_data(device_id, linear_vel, angular_vel, linear_acc, angular_acc)
        bytes_written = self.serial.write(packet)
        self.serial.flush()
        
        # Increment packet counter
        self.packets_sent += 1
        
        # Update last velocity for next comparison
        self.last_linear_vel = linear_vel
        
        # Calculate the effective data length (system will add this as byte 0)
        effective_length = 15  # Dual headers to tail length
        
        # Log sent data
        self.logger.info(f"Sending motion data: Device ID={device_id}, Sequence={packet[12]}, PacketCount={self.packets_sent}")
        self.logger.info(f"  Packet length: {effective_length} bytes (automatically added by system)")
        self.logger.info(f"  Linear velocity: {linear_vel} ({linear_vel/1000:.2f} m/s), Angular velocity: {angular_vel}")
        self.logger.info(f"  Linear acceleration: {linear_acc}, Angular acceleration: {angular_acc}")
        self.logger.info(f"  Bytes sent: {bytes_written}")
        self.logger.debug(f"  Raw data: {packet.hex(' ')}")
        
        return bytes_written
    
    def close(self):
        """Close serial connection"""
        if self.serial and self.serial.is_open:
            self.serial.close()
            self.logger.info("Serial port closed")


class ROSTopic2RF24Bridge(Node):
    MODE_AUTO = 'AUTO'
    MODE_MANUAL = 'MANUAL'
    
    def __init__(self):
        super().__init__('ros_topic_to_rf24_bridge')
        
        # Declare parameters
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('device_id', 1)
        
        # Get parameters
        port = self.get_parameter('port').get_parameter_value().string_value
        baudrate = self.get_parameter('baudrate').get_parameter_value().integer_value
        self.device_id = self.get_parameter('device_id').get_parameter_value().integer_value
        
        # Initialize RF24 sender
        self.rf24_sender = RF24Sender(port, baudrate)
        self.logger = self.rf24_sender.logger  # Use the same logger
        
        # ROS subscription
        self.subscription = self.create_subscription(
            AckermannControlCommand,
            '/control/command/control_cmd',
            self.cmd_callback,
            10)
        
        # Initialize state variables
        self.current_speed = 0.0
        self.current_angle = 0.0
        self.prev_speed = 0.0
        self.prev_angle = 0.0
        self.target_speed = 0.0  # New variable for acceleration control
        self.mode = self.MODE_AUTO
        self.mode_lock = threading.RLock()
        self.cmd_lock = threading.RLock()
        self.key_state = set()
        self.last_cmd_time = time.time()
        self.stop_event = threading.Event()
        self.emergency_stop_flag = False
        
        # Variables for tracking command updates
        self.last_sent_command_time = time.time()
        self.last_sent_speed = 0.0
        self.last_sent_angle = 0.0
        self.command_update_lock = threading.RLock()
        
        # Setup acceleration control queue and thread
        self.accel_queue = queue.Queue()
        self.accel_thread = threading.Thread(target=self.acceleration_control_thread, daemon=True)
        self.accel_thread.start()
        
        # Initialize keyboard listener
        self.keyboard_thread = threading.Thread(target=self.keyboard_input_listener, daemon=True)
        self.keyboard_thread.start()
        
        # Initialize main control loop
        self.main_thread = threading.Thread(target=self.main_loop, daemon=True)
        self.main_thread.start()
        
        # Initialize watchdog timer thread
        self.watchdog_thread = threading.Thread(target=self.watchdog_timer_thread, daemon=True)
        self.watchdog_thread.start()
        
        # Send initial stop command
        self.send_stop_command()
        
        self.logger.info("ROS Topic to RF24 Bridge initialized")
        self.logger.info("Press 'm' to toggle between AUTO and MANUAL modes")
        self.logger.info("In MANUAL mode: 'w'/'s' to control speed, 'a'/'d' to control steering")
        self.logger.info(f"Acceleration control: Steps of {MAX_ACCELERATION_STEP} m/s for changes > {ACCELERATION_THRESHOLD} m/s")
        self.logger.info(f"High speed handling: Commands > {HIGH_SPEED_THRESHOLD} m/s will first send {HIGH_SPEED_INTERMEDIATE_RATIO*100}% speed, then full speed after {HIGH_SPEED_INTERMEDIATE_DELAY*1000}ms")
        self.logger.info(f"Watchdog timer: Commands resent after {WATCHDOG_TIMEOUT} seconds of inactivity")
        self.logger.info("Current mode: AUTO")
    
    def watchdog_timer_thread(self):
        """Thread to monitor command activity and resend commands when needed"""
        self.logger.debug("Watchdog timer thread started")
        
        while not self.stop_event.is_set():
            try:
                current_time = time.time()
                
                with self.command_update_lock:
                    time_since_last_cmd = current_time - self.last_sent_command_time
                    last_speed = self.last_sent_speed
                    last_angle = self.last_sent_angle
                
                # If it's been longer than the timeout since the last command was sent
                # and we have a non-zero speed or angle (no need to resend stop commands)
                if time_since_last_cmd >= WATCHDOG_TIMEOUT and (abs(last_speed) > 0.001 or abs(last_angle) > 0.001):
                    self.logger.info(f"Watchdog timer: No new command for {time_since_last_cmd:.2f}s, resending last command")
                    
                    # Resend the last command
                    mapped_speed = self.map_speed(last_speed)
                    mapped_angle = self.map_angle(last_angle)
                    
                    self.rf24_sender.send_motion_data(self.device_id, mapped_speed, mapped_angle)
                    
                    # Update the timestamp of the last sent command
                    with self.command_update_lock:
                        self.last_sent_command_time = current_time
                
                # Check every 100ms - this is frequent enough for the 1.5s timeout
                time.sleep(0.1)
                
            except Exception as e:
                self.logger.error(f"Error in watchdog timer thread: {e}")
                time.sleep(0.1)
    
    def send_high_speed_command(self, speed_mps, angle_deg):
        """Handle high speed commands with intermediate step"""
        self.logger.info(f"High speed command detected: {speed_mps:.2f} m/s - applying two-step approach")
        
        # Calculate intermediate speed (half of target speed)
        intermediate_speed = speed_mps * HIGH_SPEED_INTERMEDIATE_RATIO
        
        # Send intermediate speed command
        mapped_intermediate_speed = self.map_speed(intermediate_speed)
        mapped_angle = self.map_angle(angle_deg)
        
        self.logger.info(f"Sending intermediate speed command: {intermediate_speed:.2f} m/s ({mapped_intermediate_speed} mm/s)")
        self.rf24_sender.send_motion_data(self.device_id, mapped_intermediate_speed, mapped_angle)
        
        # Update current speed to intermediate speed
        with self.cmd_lock:
            self.current_speed = intermediate_speed
        
        # Update the last sent command info
        with self.command_update_lock:
            self.last_sent_command_time = time.time()
            self.last_sent_speed = intermediate_speed
            self.last_sent_angle = angle_deg
        
        # Wait for specified delay
        time.sleep(HIGH_SPEED_INTERMEDIATE_DELAY)
        
        # Send full speed command
        mapped_full_speed = self.map_speed(speed_mps)
        
        self.logger.info(f"Sending full speed command: {speed_mps:.2f} m/s ({mapped_full_speed} mm/s)")
        self.rf24_sender.send_motion_data(self.device_id, mapped_full_speed, mapped_angle)
        
        # Update current speed to full speed
        with self.cmd_lock:
            self.current_speed = speed_mps
        
        # Update the last sent command info again
        with self.command_update_lock:
            self.last_sent_command_time = time.time()
            self.last_sent_speed = speed_mps
            self.last_sent_angle = angle_deg
    
    def acceleration_control_thread(self):
        """Thread to handle gradual acceleration commands"""
        self.logger.debug("Acceleration control thread started")
        
        while not self.stop_event.is_set():
            try:
                # Get target speed and angle from queue
                target_speed, target_angle, current_speed = self.accel_queue.get(timeout=1.0)
                
                # If the target speed is the same as current or it's deceleration, just send directly
                if target_speed <= current_speed:
                    mapped_speed = self.map_speed(target_speed)
                    mapped_angle = self.map_angle(target_angle)
                    self.rf24_sender.send_motion_data(self.device_id, mapped_speed, mapped_angle)
                    
                    with self.cmd_lock:
                        self.current_speed = target_speed
                    
                    # Update the last sent command info
                    with self.command_update_lock:
                        self.last_sent_command_time = time.time()
                        self.last_sent_speed = target_speed
                        self.last_sent_angle = target_angle
                    
                    self.logger.info(f"Direct deceleration to {target_speed:.2f} m/s")
                    self.accel_queue.task_done()
                    continue
                
                # Check if this is a high-speed command
                if target_speed >= HIGH_SPEED_THRESHOLD and current_speed < HIGH_SPEED_THRESHOLD:
                    self.send_high_speed_command(target_speed, target_angle)
                    self.accel_queue.task_done()
                    continue
                
                # Calculate number of steps needed
                speed_diff = target_speed - current_speed
                num_steps = math.ceil(speed_diff / MAX_ACCELERATION_STEP)
                
                self.logger.info(f"Starting gradual acceleration from {current_speed:.2f} to {target_speed:.2f} m/s in {num_steps} steps")
                
                # Perform acceleration in steps
                for i in range(1, num_steps + 1):
                    # Check if we should abort due to program exit
                    if self.stop_event.is_set():
                        break
                    
                    # Calculate intermediate speed (for last step, use exact target)
                    if i == num_steps:
                        intermediate_speed = target_speed
                    else:
                        intermediate_speed = current_speed + (i * MAX_ACCELERATION_STEP)
                    
                    # Send command
                    mapped_speed = self.map_speed(intermediate_speed)
                    mapped_angle = self.map_angle(target_angle)
                    self.rf24_sender.send_motion_data(self.device_id, mapped_speed, mapped_angle)
                    
                    # Update current speed
                    with self.cmd_lock:
                        self.current_speed = intermediate_speed
                    
                    # Update the last sent command info for the watchdog
                    with self.command_update_lock:
                        self.last_sent_command_time = time.time()
                        self.last_sent_speed = intermediate_speed
                        self.last_sent_angle = target_angle
                    
                    self.logger.info(f"Acceleration step {i}/{num_steps}: {intermediate_speed:.2f} m/s")
                    
                    # Check if we're at the final step
                    if i < num_steps:
                        time.sleep(ACCELERATION_DELAY)
                
                self.accel_queue.task_done()
                
            except queue.Empty:
                # Just continue if no commands in queue
                pass
            except Exception as e:
                self.logger.error(f"Error in acceleration control thread: {e}")
                time.sleep(0.1)
    
    def cmd_callback(self, msg):
        """Process ROS command messages"""
        # Only process if in AUTO mode
        with self.mode_lock:
            if self.mode != self.MODE_AUTO:
                return
        
        # Extract speed, steering angle and acceleration from message
        speed_mps = msg.longitudinal.speed
        angle_rad = msg.lateral.steering_tire_angle
        angle_deg = math.degrees(angle_rad)
        acceleration = msg.longitudinal.acceleration
        
        # Check for emergency stop condition
        if acceleration < EMERGENCY_BRAKE_THRESHOLD:
            self.logger.warning(f"Emergency stop triggered! Acceleration: {acceleration:.2f} m/s² < threshold: {EMERGENCY_BRAKE_THRESHOLD} m/s²")
            self.send_stop_command()
            self.emergency_stop_flag = True
            return
        
        # Store previous values for comparison
        with self.cmd_lock:
            prev_speed = self.current_speed
            prev_angle = self.current_angle
            
            # Update target speed (will be updated to current during execution)
            self.target_speed = speed_mps
            self.current_angle = angle_deg
        
        # Calculate changes
        speed_change = speed_mps - prev_speed
        angle_change = abs(angle_deg - prev_angle)
        
        # Log received command
        self.logger.info(f"ROS command received - Speed: {speed_mps:.2f} m/s, Angle: {angle_deg:.2f}°, Accel: {acceleration:.2f} m/s²")
        
        # Determine if we should send the command
        should_send = False
        
        # Always send if emergency stop flag was set previously (to resume normal operation)
        if self.emergency_stop_flag:
            self.logger.info("Resuming normal operation after emergency stop")
            should_send = True
            self.emergency_stop_flag = False
        # For speed changes:
        # - Always send if decelerating
        # - Only send if accelerating and change is significant
        elif speed_change < 0 or abs(speed_change) >= SPEED_CHANGE_THRESHOLD:
            should_send = True
        # For angle changes: send if change is significant
        elif angle_change >= ANGLE_CHANGE_THRESHOLD:
            should_send = True
            
        if should_send:
            # Special handling for high speed commands
            if speed_mps >= HIGH_SPEED_THRESHOLD and prev_speed < HIGH_SPEED_THRESHOLD:
                self.send_high_speed_command(speed_mps, angle_deg)
            # Check if we need gradual acceleration (only for positive acceleration above threshold)
            elif speed_change > ACCELERATION_THRESHOLD:
                self.logger.info(f"Large acceleration detected: {speed_change:.2f} m/s - using gradual acceleration control")
                # Add to acceleration control queue
                self.accel_queue.put((speed_mps, angle_deg, prev_speed))
            else:
                # For deceleration or small acceleration, send directly
                mapped_speed = self.map_speed(speed_mps)
                mapped_angle = self.map_angle(angle_deg)
                
                self.logger.info(f"Sending direct command - Mapped values - Speed: {mapped_speed} mm/s, Angular velocity: {mapped_angle}")
                
                # Send command to RF24
                self.rf24_sender.send_motion_data(self.device_id, mapped_speed, mapped_angle)
                
                # Update current speed
                with self.cmd_lock:
                    self.current_speed = speed_mps
                
                # Update the last sent command info for the watchdog
                with self.command_update_lock:
                    self.last_sent_command_time = time.time()
                    self.last_sent_speed = speed_mps
                    self.last_sent_angle = angle_deg
            
            self.last_cmd_time = time.time()
        else:
            self.logger.debug(f"Skipping command - Speed change: {speed_change:.2f} m/s, Angle change: {angle_change:.2f}°")
            
            # Even though we're not sending a new command, update the watchdog timer
            # to acknowledge that we've received a command (even if it's the same)
            with self.command_update_lock:
                self.last_sent_command_time = time.time()
    
    def map_speed(self, speed_mps):
        """Map speed from m/s to mm/s with limits"""
        # Directly convert m/s to mm/s (1 m/s = 1000 mm/s)
        speed_mms = int(speed_mps * 1000)
        
        # Apply limits
        return max(MIN_SPEED, min(MAX_SPEED, speed_mms))
    
    def map_angle(self, angle_deg):
        """Map steering angle (degrees) to angular velocity for RF24"""
        # Mapping from degrees to some angular velocity value
        # Assuming ±25° steering range maps to some appropriate angular velocity range
        # This is a simplified mapping - adjust based on your specific requirements
        MAX_ANGLE_DEG = 25.0
        # Map to a value that works with your system
        # For this example, we're using a simple linear mapping to an arbitrary range [-3000, 3000]
        MAX_ANGULAR_VEL = 3000
        
        # Ensure angle is within ±MAX_ANGLE_DEG
        clamped_angle = max(-MAX_ANGLE_DEG, min(MAX_ANGLE_DEG, angle_deg))
        
        # Map to angular velocity value
        # Negative angle (turning left) maps to positive angular velocity in this example
        # Adjust the sign based on your system's conventions
        angular_vel = -int((clamped_angle / MAX_ANGLE_DEG) * MAX_ANGULAR_VEL)
        
        return angular_vel
    
    def keyboard_input_listener(self):
        """Keyboard input listener thread"""
        self.logger.debug("Keyboard input listener thread started")
        with keyboard.Listener(on_press=self.on_press, on_release=self.on_release) as listener:
            listener.join()
    
    def on_press(self, key):
        try:
            # Handle special keys
            if key == keyboard.Key.space:
                self.logger.info("Space key pressed - sending stop command")
                self.send_stop_command()
                return
            
            # Handle character keys
            try:
                key_char = key.char.lower()
                self.key_state.add(key_char)
                
                if key_char == 'm':
                    self.toggle_mode()
                    # Send stop command on mode change
                    self.send_stop_command()
            except AttributeError:
                pass  # Not a character key
        except Exception as e:
            self.logger.error(f"Error handling key press: {e}")
    
    def on_release(self, key):
        try:
            # Handle character keys
            try:
                key_char = key.char.lower()
                self.key_state.discard(key_char)
            except AttributeError:
                pass  # Not a character key
            
            # Exit on Escape key
            if key == keyboard.Key.esc:
                self.logger.info("Escape key pressed - exiting program")
                self.stop_event.set()
                rclpy.shutdown()
                return False  # Stop listener
        except Exception as e:
            self.logger.error(f"Error handling key release: {e}")
    
    def toggle_mode(self):
        """Toggle between AUTO and MANUAL modes"""
        with self.mode_lock:
            if self.mode == self.MODE_AUTO:
                self.mode = self.MODE_MANUAL
                self.logger.info("Switched to MANUAL mode")
            else:
                self.mode = self.MODE_AUTO
                self.logger.info("Switched to AUTO mode")
    
    def send_stop_command(self):
        """Send stop command: speed=0, angle=0"""
        self.logger.info("Sending stop command")
        self.rf24_sender.send_motion_data(self.device_id, 0, 0)
        
        with self.cmd_lock:
            self.prev_speed = self.current_speed
            self.prev_angle = self.current_angle
            self.current_speed = 0.0
            self.current_angle = 0.0
            self.target_speed = 0.0
        
        # Update the last sent command info for the watchdog
        with self.command_update_lock:
            self.last_sent_command_time = time.time()
            self.last_sent_speed = 0.0
            self.last_sent_angle = 0.0
    
    def main_loop(self):
        """Main control loop for manual keyboard control"""
        self.logger.debug("Main control loop thread started")
        
        # Keyboard control parameters
        SPEED_INCREMENT = 0.1      # m/s
        ANGLE_INCREMENT = 2.0      # degrees
        ANGLE_RETURN_RATE = 2.0    # degrees per iteration
        MAX_MANUAL_SPEED = 1.0     # m/s
        MIN_MANUAL_SPEED = -1.0    # m/s
        MAX_MANUAL_ANGLE = 25.0    # degrees
        
        last_sent_speed = 0.0
        last_sent_angle = 0.0
        
        while not self.stop_event.is_set():
            try:
                # Only process keyboard controls in MANUAL mode
                with self.mode_lock:
                    if self.mode != self.MODE_MANUAL:
                        time.sleep(0.05)
                        continue
                
                with self.cmd_lock:
                    new_speed = self.current_speed
                    new_angle = self.current_angle
                    
                    # Process speed controls
                    if 'w' in self.key_state:
                        new_speed = min(new_speed + SPEED_INCREMENT, MAX_MANUAL_SPEED)
                    elif 's' in self.key_state:
                        new_speed = max(new_speed - SPEED_INCREMENT, MIN_MANUAL_SPEED)
                    else:
                        # Gradually reduce speed to zero when no keys are pressed
                        if new_speed > 0:
                            new_speed = max(0, new_speed - SPEED_INCREMENT)
                        elif new_speed < 0:
                            new_speed = min(0, new_speed + SPEED_INCREMENT)
                    
                    # Process steering controls
                    if 'a' in self.key_state:
                        new_angle = min(new_angle + ANGLE_INCREMENT, MAX_MANUAL_ANGLE)
                    elif 'd' in self.key_state:
                        new_angle = max(new_angle - ANGLE_INCREMENT, -MAX_MANUAL_ANGLE)
                    else:
                        # Return steering to center when no keys are pressed
                        if new_angle > 0:
                            new_angle = max(0, new_angle - ANGLE_RETURN_RATE)
                        elif new_angle < 0:
                            new_angle = min(0, new_angle + ANGLE_RETURN_RATE)
                    
                    # Round to 3 decimal places
                    new_speed = round(new_speed, 3)
                    new_angle = round(new_angle, 3)
                    
                    # Determine if we should send the command based on changes
                    speed_change = new_speed - last_sent_speed
                    angle_change = abs(new_angle - last_sent_angle)
                    
                    should_send = False
                    
                    # For manual control:
                    # - Always send if decelerating
                    # - Only send if accelerating and change is significant
                    # - Send if angle change is significant
                    if speed_change < 0 or abs(speed_change) >= SPEED_CHANGE_THRESHOLD:
                        should_send = True
                    elif angle_change >= ANGLE_CHANGE_THRESHOLD:
                        should_send = True
                    
                    if should_send:
                        # Check for high speed command in manual mode
                        if new_speed >= HIGH_SPEED_THRESHOLD and last_sent_speed < HIGH_SPEED_THRESHOLD:
                            self.send_high_speed_command(new_speed, new_angle)
                            # Update for future comparison
                            self.current_speed = new_speed
                            self.current_angle = new_angle
                            last_sent_speed = new_speed
                            last_sent_angle = new_angle
                        # Apply gradual acceleration for manual control as well
                        elif speed_change > ACCELERATION_THRESHOLD:
                            self.logger.info(f"Manual large acceleration detected: {speed_change:.2f} m/s - using gradual acceleration")
                            # Add to acceleration control queue
                            self.accel_queue.put((new_speed, new_angle, last_sent_speed))
                            # Update for future comparison
                            self.current_speed = new_speed  # Target speed
                            self.current_angle = new_angle
                            last_sent_speed = new_speed
                            last_sent_angle = new_angle
                        else:
                            # For deceleration or small acceleration, send directly
                            self.current_speed = new_speed
                            self.current_angle = new_angle
                            
                            # Map values for RF24
                            mapped_speed = self.map_speed(new_speed)
                            mapped_angle = self.map_angle(new_angle)
                            
                            # Send command
                            self.rf24_sender.send_motion_data(self.device_id, mapped_speed, mapped_angle)
                            self.logger.debug(f"Manual control - Speed: {new_speed:.2f} m/s, Angle: {new_angle:.2f}°")
                            self.logger.debug(f"Mapped values - Speed: {mapped_speed} mm/s, Angular velocity: {mapped_angle}")
                            
                            # Update the last sent command info for the watchdog
                            with self.command_update_lock:
                                self.last_sent_command_time = time.time()
                                self.last_sent_speed = new_speed
                                self.last_sent_angle = new_angle
                            
                            # Update last sent values
                            last_sent_speed = new_speed
                            last_sent_angle = new_angle
                    else:
                        self.logger.debug(f"Skipping manual command - Speed change: {speed_change:.2f} m/s, Angle change: {angle_change:.2f}°")
                        
                        # Even though we're not sending a new command due to minimal changes,
                        # update the watchdog timer to indicate that the current state is still desired
                        with self.command_update_lock:
                            self.last_sent_command_time = time.time()
                
                time.sleep(0.05)  # Control rate
                
            except Exception as e:
                self.logger.error(f"Error in main loop: {e}")
                time.sleep(0.1)
    
    def destroy_node(self):
        """Clean up resources when node is destroyed"""
        self.logger.info("Shutting down ROS Topic to RF24 Bridge")
        self.stop_event.set()
        self.send_stop_command()
        time.sleep(0.5)  # Allow time for stop command to be sent
        self.rf24_sender.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    
    parser = argparse.ArgumentParser(description='ROS Topic to RF24 Bridge')
    parser.add_argument('--port', type=str, default='/dev/ttyUSB0', help='Serial port, default is /dev/ttyUSB0')
    parser.add_argument('--baud', type=int, default=115200, help='Baud rate, default is 115200')
    parser.add_argument('--device', type=int, default=1, help='Device ID, default is 1')
    args = parser.parse_args()
    
    # Override ROS params with command line args if provided
    rclpy.Parameter('port', rclpy.Parameter.Type.STRING, args.port)
    rclpy.Parameter('baudrate', rclpy.Parameter.Type.INTEGER, args.baud)
    rclpy.Parameter('device_id', rclpy.Parameter.Type.INTEGER, args.device)
    
    try:
        node = ROSTopic2RF24Bridge()
        
        print("\n========== ROS Topic to RF24 Bridge ==========")
        print(f"Serial port: {args.port}")
        print(f"Baud rate: {args.baud}")
        print(f"Device ID: {args.device}")
        print("Command filtering:")
        print(f"  - Speed change threshold: {SPEED_CHANGE_THRESHOLD} m/s (except deceleration)")
        print(f"  - Angle change threshold: {ANGLE_CHANGE_THRESHOLD} degrees")
        print(f"  - Emergency brake threshold: {EMERGENCY_BRAKE_THRESHOLD} m/s²")
        print("Acceleration control:")
        print(f"  - Acceleration threshold: {ACCELERATION_THRESHOLD} m/s")
        print(f"  - Acceleration step size: {MAX_ACCELERATION_STEP} m/s")
        print(f"  - Step delay: {ACCELERATION_DELAY} seconds")
        print("High speed handling:")
        print(f"  - High speed threshold: {HIGH_SPEED_THRESHOLD} m/s")
        print(f"  - Intermediate speed ratio: {HIGH_SPEED_INTERMEDIATE_RATIO*100}%")
        print(f"  - Delay before full speed: {HIGH_SPEED_INTERMEDIATE_DELAY*1000} ms")
        print("Watchdog timer:")
        print(f"  - Timeout: {WATCHDOG_TIMEOUT} seconds")
        print("Control modes:")
        print("  - AUTO: Listens to ROS topic '/control/command/control_cmd'")
        print("  - MANUAL: Use keyboard controls")
        print("Keyboard controls:")
        print("  - 'm': Toggle between AUTO and MANUAL modes")
        print("  - 'w'/'s': Increase/decrease speed")
        print("  - 'a'/'d': Turn left/right")
        print("  - Space: Emergency stop (sets speed and angle to zero)")
        print("  - ESC: Exit program")
        print("==============================================\n")
        
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error: {e}")
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()