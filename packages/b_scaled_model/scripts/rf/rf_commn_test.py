import serial
import struct
import time
import argparse
import random

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

class RF24Sender:
    def __init__(self, port, baudrate=115200):
        """Initialize RF24 sender"""
        self.serial = serial.Serial(port, baudrate, timeout=1)
        self.sequence_counter = 0
        self.last_linear_vel = 0  # Track last sent linear velocity for logging
        self.packets_sent = 0     # Track total packets sent
        self.last_send_time = time.time()  # Track last send time for acceleration calculation
        print(f"Connected to {port} at {baudrate} baud")
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
    
    def threshold_motion_check(self, linear_vel, angular_vel, linear_acc=0, angular_acc=0):
        """
        Check if the motion command should be sent based on thresholds
        Returns True if the command should be sent, False otherwise
        """
        # Check for emergency stop condition (high deceleration)
        if linear_acc < -1100:  # -1.1m/ss converted to mm/s²
            print(f"EMERGENCY STOP: High deceleration detected ({linear_acc/1000:.2f}m/ss)")
            # Send a stop command immediately
            self.send_motion_data(1, 0, 0, linear_acc, angular_acc)
            return False
        
        # Check speed difference
        speed_diff = abs(linear_vel - self.last_linear_vel)
        if speed_diff < 200:
            print(f"Skipping: Speed difference too small ({speed_diff} mm/s)")
            return False
            
        return True
    
    def send_motion_data(self, device_id, linear_vel, angular_vel, linear_acc=0, angular_acc=0):
        """Send motion data packet"""
        # Enforce speed limits
        linear_vel = max(MIN_SPEED, min(MAX_SPEED, linear_vel))
        
        # Apply minimum reverse start threshold when changing from stop to reverse
        if self.last_linear_vel == 0 and linear_vel < 0:
            original_vel = linear_vel
            linear_vel = min(linear_vel, MIN_REVERSE_START)
            if original_vel != linear_vel:
                print(f"Note: Adjusted reverse speed from {original_vel} to {linear_vel} for better startup")
        
        # Check speed difference (just for logging, not enforcing)
        speed_diff = abs(linear_vel - self.last_linear_vel)
        if speed_diff > 1000:
            print(f"Warning: Large speed change detected: {self.last_linear_vel/1000:.2f} m/s -> {linear_vel/1000:.2f} m/s ({speed_diff/1000:.2f} m/s difference)")
        
        # Calculate actual acceleration based on time difference
        current_time = time.time()
        time_diff = current_time - self.last_send_time
        if time_diff > 0:
            actual_acc = (linear_vel - self.last_linear_vel) / time_diff
            print(f"Calculated acceleration: {actual_acc/1000:.2f}m/ss")
        self.last_send_time = current_time
        
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
        print(f"Sending motion data: Device ID={device_id}, Sequence={packet[12]}, PacketCount={self.packets_sent}")
        print(f"  Packet length: {effective_length} bytes (automatically added by system)")
        print(f"  Linear velocity: {linear_vel} ({linear_vel/1000:.2f} m/s), Angular velocity: {angular_vel}")
        print(f"  Linear acceleration: {linear_acc}, Angular acceleration: {angular_acc}")
        print(f"  Bytes sent: {bytes_written}")
        print(f"  Raw data: {packet.hex(' ')}")
        print()
        
        return bytes_written
    
    def close(self):
        """Close serial connection"""
        if self.serial and self.serial.is_open:
            self.serial.close()
            print("Serial port closed")

class TestSequence:
    """Test sequence generator"""
    
    @staticmethod
    def straight_line_test(sender, device_id=1, interval=0.5):
        """Straight line test: first forward, then backward with incremental speed changes"""
        print("\n===== Straight Line Test =====")
        # Stop
        sender.send_motion_data(device_id, 0, 0)
        time.sleep(interval)
        
        # Gradual speed increase
        print("-> Slow forward")
        sender.send_motion_data(device_id, 300, 0)  # 0.3 m/s
        time.sleep(1.5)
        
        print("-> Medium speed forward")
        sender.send_motion_data(device_id, 600, 0)  # 0.6 m/s
        time.sleep(1.5)
        
        print("-> Fast forward")
        sender.send_motion_data(device_id, 1000, 0)  # 1.0 m/s (max)
        time.sleep(2)
        
        # Gradual speed decrease
        print("-> Medium speed forward")
        sender.send_motion_data(device_id, 600, 0)  # 0.6 m/s
        time.sleep(1)
        
        print("-> Slow forward")
        sender.send_motion_data(device_id, 300, 0)  # 0.3 m/s
        time.sleep(1)
        
        # Stop gradually
        print("-> Very slow forward")
        sender.send_motion_data(device_id, 100, 0)  # 0.1 m/s
        time.sleep(1)
        
        print("-> Stop")
        sender.send_motion_data(device_id, 0, 0)
        time.sleep(1.5)
        
        # Start backward with higher initial speed for better startup
        print("-> Backward start (higher initial speed)")
        sender.send_motion_data(device_id, -600, 0)  # -0.6 m/s (minimum for good startup)
        time.sleep(1.5)
        
        print("-> Medium speed backward")
        sender.send_motion_data(device_id, -800, 0)  # -0.8 m/s
        time.sleep(1.5)
        
        print("-> Maximum backward")
        sender.send_motion_data(device_id, -1000, 0)  # -1.0 m/s (min)
        time.sleep(2)
        
        # Gradual backward speed decrease
        print("-> Medium speed backward")
        sender.send_motion_data(device_id, -700, 0)  # -0.7 m/s
        time.sleep(1)
        
        # Stop gradually
        print("-> Slow backward")
        sender.send_motion_data(device_id, -400, 0)  # -0.4 m/s
        time.sleep(1)
        
        print("-> Very slow backward")
        sender.send_motion_data(device_id, -100, 0)  # -0.1 m/s
        time.sleep(1)
        
        print("-> Stop")
        sender.send_motion_data(device_id, 0, 0)
    
    @staticmethod
    def packet_loss_test(sender, device_id=1, num_packets=100, interval=0.02):
        """Send a sequence of numbered packets to test packet loss rate with fine-grained speed variations"""
        print(f"\n===== Enhanced Packet Loss Test ({num_packets} packets) =====")
        
        # Reset counter for this test
        initial_counter = sender.sequence_counter
        initial_packet_count = sender.packets_sent
        
        print(f"Starting sequence number: {initial_counter}")
        print(f"Sending {num_packets} packets with {interval:.3f}s interval...")
        
        # Generate a more varied speed profile
        speeds = []
        current_speed = 0
        target_patterns = [
            # [target speed, duration in packets]
            [600, 10],   # Accelerate to medium speed
            [300, 5],    # Slow down a bit
            [800, 15],   # Speed up to high speed
            [0, 8],      # Stop completely
            [-400, 12],  # Reverse at medium speed
            [-700, 10],  # Faster reverse
            [-300, 5],   # Slow down reverse
            [0, 5],      # Stop again
            [500, 10],   # Forward again
            [200, 8],    # Slow forward
            [0, 5],      # Final stop
        ]
        
        # Generate fine-grained speed transitions
        packet_index = 0
        for target, duration in target_patterns:
            start_speed = current_speed
            for i in range(duration):
                # Calculate intermediate speed with easing
                progress = (i + 1) / duration
                # Use ease-in-out curve for smoother acceleration/deceleration
                if progress < 0.5:
                    factor = 2 * progress * progress
                else:
                    factor = 1 - pow(-2 * progress + 2, 2) / 2
                
                intermediate_speed = int(start_speed + (target - start_speed) * factor)
                speeds.append(intermediate_speed)
                packet_index += 1
                if packet_index >= num_packets:
                    break
            current_speed = target
            if packet_index >= num_packets:
                break
                
        # Fill any remaining packets with zeros (stop commands)
        while len(speeds) < num_packets:
            speeds.append(0)
        
        # Track command sending
        commands_sent = 0
        commands_skipped = 0
        emergency_stops = 0
        last_speed = sender.last_linear_vel
        
        for i in range(num_packets):
            # Set speed and turning parameters
            current_speed = speeds[i]
            
            # Varied turning based on speed pattern
            if current_speed > 400:
                angular = 100 * (i % 7 - 3)  # Small oscillations when going forward fast
            elif current_speed < -400:
                angular = -100 * (i % 5 - 2)  # Different oscillations when going backward
            elif abs(current_speed) > 0:
                angular = 300 * (i % 3 - 1)  # Larger turns at moderate speed
            else:
                angular = 0  # No turning when stopped
            
            # Calculate acceleration
            time_now = time.time()
            dt = time_now - sender.last_send_time
            if dt > 0:
                acceleration = (current_speed - last_speed) / dt
            else:
                acceleration = 0
                
            # Acceleration in mm/s² to be included in the packet
            linear_acc = int(acceleration)
            
            # Apply threshold check
            if sender.threshold_motion_check(current_speed, angular, linear_acc, 0):
                # Only send if passing the threshold check
                sender.send_motion_data(device_id, current_speed, angular, i+1, 0)
                commands_sent += 1
                last_speed = current_speed
            else:
                commands_skipped += 1
                if linear_acc < -1100:
                    emergency_stops += 1
            
            # Wait between packets
            time.sleep(interval)
        
        # Final stop command - always send this regardless of thresholds
        sender.send_motion_data(device_id, 0, 0)
        
        # Display summary
        final_counter = sender.sequence_counter
        expected_counter = (initial_counter + commands_sent + 1) & 0xFF  # +1 for final stop
        final_packet_count = sender.packets_sent
        
        print("\n=== Enhanced Packet Loss Test Summary ===")
        print(f"Initial sequence number: {initial_counter}")
        print(f"Final sequence number: {final_counter}")
        print(f"Expected final sequence (considering overflow): {expected_counter}")
        print(f"Commands requested: {num_packets}")
        print(f"Commands sent: {commands_sent} (includes final stop)")
        print(f"Commands skipped due to thresholds: {commands_skipped}")
        print(f"Emergency stops triggered: {emergency_stops}")
        print(f"Total packets sent since start: {final_packet_count}")
        
        if final_counter == expected_counter:
            print("Sequence counter matches expected value. All packets likely sent.")
        else:
            print("Sequence counter doesn't match expected value.")
            print("Note: This doesn't necessarily indicate packet loss at the RF24 level.")
            print("      It only confirms sequence generation by this script.")

def main():
    """Main function"""
    parser = argparse.ArgumentParser(description='RF24L01 Receiver Testing Tool')
    parser.add_argument('--port', type=str, default='COM12', help='Serial port, default is COM12')
    parser.add_argument('--baud', type=int, default=115200, help='Baud rate, default is 115200')
    parser.add_argument('--test', type=str, default='all', 
                       help='Test type: straight, packets, all')
    parser.add_argument('--device', type=int, default=1, help='Device ID, default is 1')
    parser.add_argument('--interval', type=float, default=0.1, 
                       help='Command sending interval (seconds), default 0.1, note RF24L01 requires interval >10ms')
    parser.add_argument('--packets', type=int, default=100,
                       help='Number of packets to send in packet loss test, default 100')
    
    args = parser.parse_args()
    
    print(f"Preparing to connect to serial port: {args.port}")
    sender = RF24Sender(args.port, args.baud)
    interval = max(0.01, args.interval)  # Ensure interval is at least 10ms
    
    try:
        if args.test in ['straight', 'all']:
            TestSequence.straight_line_test(sender, args.device, interval)
            time.sleep(1)
        
        if args.test in ['packets', 'all']:
            TestSequence.packet_loss_test(sender, args.device, args.packets, interval)
            time.sleep(1)
        
        print("\nAll tests completed!")
    
    except KeyboardInterrupt:
        print("\nUser interrupted, stopping tests")
    
    except Exception as e:
        print(f"\nError occurred during testing: {e}")
    
    finally:
        # Ensure vehicle is stopped
        try:
            sender.send_motion_data(args.device, 0, 0)
            time.sleep(0.5)
            sender.close()
        except:
            print("Error while closing, serial port may be disconnected")

if __name__ == "__main__":
    main()