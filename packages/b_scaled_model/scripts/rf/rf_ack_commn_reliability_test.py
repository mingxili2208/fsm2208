import serial
import struct
import time
import argparse
import random
import threading
import statistics
from enum import Enum

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

class RF24Sender:
    def __init__(self, port, baudrate=115200):
        """Initialize RF24 sender"""
        self.serial = serial.Serial(port, baudrate, timeout=1)
        self.sequence_counter = 0
        self.sent_packets = 0
        self.acked_packets = 0
        self.send_timestamps = {}
        self.ack_timestamps = {}
        self.ack_latencies = []
        self.lock = threading.Lock()  # Thread safety
        print(f"Connected to {port} at {baudrate} baud")
        time.sleep(2)  # Wait for serial port to stabilize
        
        # Start ACK listener thread
        self.ack_listener = threading.Thread(target=self._listen_for_acks)
        self.ack_listener.daemon = True
        self.ack_listener.start()
    
    def calculate_checksum(self, data):
        """Calculate checksum"""
        return sum(data) & 0xFF
    
    def pack_motion_data(self, device_id, linear_vel, angular_vel, linear_acc=0, angular_acc=0):
        """Pack motion data packet - following RF24L01 specifications with dual headers"""
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
    
    def send_motion_data(self, device_id, linear_vel, angular_vel, linear_acc=0, angular_acc=0, 
                         log=True, wait_for_ack=False, ack_timeout=0.5):
        """Send motion data packet"""
        packet = self.pack_motion_data(device_id, linear_vel, angular_vel, linear_acc, angular_acc)
        start_time = time.time()
        bytes_written = self.serial.write(packet)
        self.serial.flush()
        
        # Record transmission 
        seq_num = packet[12]
        with self.lock:
            self.sent_packets += 1
            self.send_timestamps[seq_num] = start_time
        
        # Calculate the effective data length 
        effective_length = 15  # Dual headers to tail length
        
        # Log sent data
        if log:
            print(f"Sending motion data: Device ID={device_id}, Sequence={seq_num}")
            print(f"  Packet length: {effective_length} bytes (automatically added by system)")
            print(f"  Linear velocity: {linear_vel} ({linear_vel/1000:.2f} m/s), Angular velocity: {angular_vel}")
            print(f"  Linear acceleration: {linear_acc}, Angular acceleration: {angular_acc}")
            print(f"  Bytes sent: {bytes_written}")
            print()
        
        # Wait for ACK if requested
        if wait_for_ack:
            ack_received = self._wait_for_ack(seq_num, ack_timeout)
            if log:
                if ack_received:
                    print(f"  ACK received for sequence {seq_num}")
                else:
                    print(f"  No ACK received for sequence {seq_num} after {ack_timeout}s")
            return ack_received
        
        return bytes_written
    
    def _listen_for_acks(self):
        """Background thread to listen for ACK packets"""
        while True:
            try:
                if self.serial.in_waiting > 0:
                    # Read the first byte to check for frame header
                    header1 = self.serial.read(1)
                    if header1 and header1[0] == FRAME_HEADER1:
                        # Read the second byte to check for second header
                        header2 = self.serial.read(1)
                        if header2 and header2[0] == FRAME_HEADER2:
                            # Read packet type
                            packet_type = self.serial.read(1)
                            if packet_type and packet_type[0] == PacketType.PACKET_ACK:
                                # Read sequence number
                                seq_bytes = self.serial.read(1)
                                if seq_bytes:
                                    seq_num = seq_bytes[0]
                                    # Record ACK time
                                    now = time.time()
                                    with self.lock:
                                        self.acked_packets += 1
                                        self.ack_timestamps[seq_num] = now
                                        
                                        # Calculate latency if we have the send timestamp
                                        if seq_num in self.send_timestamps:
                                            latency = (now - self.send_timestamps[seq_num]) * 1000  # ms
                                            self.ack_latencies.append(latency)
                            else:
                                # Discard rest of packet if not an ACK
                                self.serial.reset_input_buffer()
                        else:
                            # Not a valid ACK, discard
                            self.serial.reset_input_buffer()
                    else:
                        # Not a valid ACK, discard
                        self.serial.reset_input_buffer()
            except Exception as e:
                print(f"Error in ACK listener: {e}")
            
            # Sleep to prevent CPU hogging
            time.sleep(0.001)
    
    def _wait_for_ack(self, seq_num, timeout):
        """Wait for an ACK with the specified sequence number"""
        start_time = time.time()
        while time.time() - start_time < timeout:
            with self.lock:
                if seq_num in self.ack_timestamps:
                    return True
            time.sleep(0.01)  # Small sleep to prevent CPU hogging
        return False
    
    def close(self):
        """Close serial connection"""
        if self.serial and self.serial.is_open:
            self.serial.close()
            print("Serial port closed")
    
    def get_statistics(self):
        """Get transmission statistics"""
        with self.lock:
            stats = {}
            stats["sent_packets"] = self.sent_packets
            stats["acked_packets"] = self.acked_packets
            
            # Calculate packet loss
            if self.sent_packets > 0:
                stats["packet_loss_percent"] = 100 * (self.sent_packets - self.acked_packets) / self.sent_packets
            else:
                stats["packet_loss_percent"] = 0.0
            
            # Calculate latency
            if self.ack_latencies:
                stats["avg_latency_ms"] = statistics.mean(self.ack_latencies)
                stats["min_latency_ms"] = min(self.ack_latencies)
                stats["max_latency_ms"] = max(self.ack_latencies)
            
            return stats

class VehicleController:
    """Simplified vehicle control interface"""
    
    def __init__(self, rf_sender):
        self.sender = rf_sender
        self.device_id = 1
        self.last_cmd_time = 0
        self.min_cmd_interval = 0.05  # Minimum time between commands (50ms)
    
    def set_motion(self, linear_vel, angular_vel, wait_for_ack=True):
        """Set vehicle motion with linear and angular velocity"""
        # Convert to integer values expected by the protocol
        linear_vel_int = int(linear_vel * 1000)  # Convert m/s to mm/s
        angular_vel_int = int(angular_vel * 1000)  # Convert rad/s to mrad/s
        
        # Enforce minimum command interval
        current_time = time.time()
        if current_time - self.last_cmd_time < self.min_cmd_interval:
            time.sleep(self.min_cmd_interval - (current_time - self.last_cmd_time))
        
        # Send the command
        result = self.sender.send_motion_data(
            self.device_id, 
            linear_vel_int, 
            angular_vel_int,
            wait_for_ack=wait_for_ack
        )
        
        self.last_cmd_time = time.time()
        return result
    
    def stop(self, wait_for_ack=True):
        """Stop the vehicle"""
        return self.set_motion(0, 0, wait_for_ack)
    
    def move_forward(self, speed=1.0, wait_for_ack=True):
        """Move forward at specified speed (m/s)"""
        return self.set_motion(speed, 0, wait_for_ack)
    
    def move_backward(self, speed=0.5, wait_for_ack=True):
        """Move backward at specified speed (m/s)"""
        return self.set_motion(-speed, 0, wait_for_ack)
    
    def turn_left(self, speed=0.8, turn_rate=1.5, wait_for_ack=True):
        """Turn left while moving"""
        return self.set_motion(speed, -turn_rate, wait_for_ack)
    
    def turn_right(self, speed=0.8, turn_rate=1.5, wait_for_ack=True):
        """Turn right while moving"""
        return self.set_motion(speed, turn_rate, wait_for_ack)
    
    def rotate_left(self, rate=2.0, wait_for_ack=True):
        """Rotate left in place"""
        return self.set_motion(0, -rate, wait_for_ack)
    
    def rotate_right(self, rate=2.0, wait_for_ack=True):
        """Rotate right in place"""
        return self.set_motion(0, rate, wait_for_ack)


class TestSequence:
    """Test sequence generator"""
    
    @staticmethod
    def straight_line_test(vehicle, interval=0.5):
        """Straight line test: first forward, then backward"""
        print("\n===== Straight Line Test =====")
        # Stop
        vehicle.stop()
        time.sleep(interval)
        
        # Slow forward
        print("-> Slow forward")
        vehicle.move_forward(0.5)
        time.sleep(2)
        
        # Medium speed forward
        print("-> Medium speed forward")
        vehicle.move_forward(1.0)
        time.sleep(2)
        
        # Fast forward
        print("-> Fast forward")
        vehicle.move_forward(2.0)
        time.sleep(2)
        
        # Stop
        print("-> Stop")
        vehicle.stop()
        time.sleep(1)
        
        # Slow backward
        print("-> Slow backward")
        vehicle.move_backward(0.3)
        time.sleep(2)
        
        # Medium speed backward
        print("-> Medium speed backward")
        vehicle.move_backward(0.6)
        time.sleep(2)
        
        # Maximum backward
        print("-> Maximum backward")
        vehicle.move_backward(1.0)
        time.sleep(2)
        
        # Stop
        print("-> Stop")
        vehicle.stop()
    
    @staticmethod
    def steering_test(vehicle, interval=0.5):
        """Steering test: left and right turn while moving forward"""
        print("\n===== Steering Test =====")
        # Stop
        vehicle.stop()
        time.sleep(interval)
        
        # Forward and left turn
        print("-> Forward and left turn")
        vehicle.turn_left(0.8, 1.5)
        time.sleep(3)
        
        # Forward straight
        print("-> Forward straight")
        vehicle.move_forward(0.8)
        time.sleep(2)
        
        # Forward and right turn
        print("-> Forward and right turn")
        vehicle.turn_right(0.8, 1.5)
        time.sleep(3)
        
        # Stop
        print("-> Stop")
        vehicle.stop()
        time.sleep(interval)
        
        # Rotate left in place
        print("-> Rotate left in place")
        vehicle.rotate_left(3.0)
        time.sleep(3)
        
        # Rotate right in place
        print("-> Rotate right in place")
        vehicle.rotate_right(3.0)
        time.sleep(3)
        
        # Stop
        print("-> Stop")
        vehicle.stop()
    
    @staticmethod
    def timeout_test(vehicle, interval=1.0):
        """Timeout test: send command then stop sending, test receiver timeout function"""
        print("\n===== Timeout Test =====")
        
        # Forward
        print("-> Forward, then stop sending data")
        vehicle.move_forward(1.0)
        
        # Wait longer than receiver timeout (5 seconds)
        print("-> Wait 6 seconds, receiver should stop after 5 seconds...")
        for i in range(6):
            print(f"    Waiting: {i+1} seconds")
            time.sleep(1)
            
        # Resume communication
        print("-> Resume communication, send stop command")
        vehicle.stop()
    
    @staticmethod
    def zigzag_test(vehicle, interval=0.5):
        """Variable speed and direction test: Z-shaped path"""
        print("\n===== Z-shaped Path Test =====")
        # Stop
        vehicle.stop()
        time.sleep(interval)
        
        # Forward and right turn
        print("-> Forward and right turn")
        vehicle.turn_right(1.0, 2.0)
        time.sleep(2)
        
        # Forward and left turn
        print("-> Forward and left turn")
        vehicle.turn_left(1.2, 2.0)
        time.sleep(2)
        
        # Forward and right turn
        print("-> Forward and right turn")
        vehicle.turn_right(1.4, 2.0)
        time.sleep(2)
        
        # Slow down and go straight
        print("-> Slow down and go straight")
        vehicle.move_forward(0.7)
        time.sleep(2)
        
        # Stop
        print("-> Stop")
        vehicle.stop()
    
    @staticmethod
    def rapid_switch_test(vehicle, duration=5):
        """Rapid switching test: quickly alternate between forward and backward commands"""
        print("\n===== Rapid Switching Test =====")
        
        start_time = time.time()
        toggle = True
        
        print(f"-> Start rapid switching forward/backward, duration {duration} seconds")
        while time.time() - start_time < duration:
            if toggle:
                vehicle.move_forward(1.0)
            else:
                vehicle.move_backward(0.5)
            toggle = not toggle
            time.sleep(0.2)  # Switch every 200ms
        
        # Stop
        print("-> Stop")
        vehicle.stop()
    
    @staticmethod
    def random_movement_test(vehicle, commands=20, interval=0.5):
        """Random movement test: randomly generate speed and direction"""
        print("\n===== Random Movement Test =====")
        # Stop
        vehicle.stop()
        time.sleep(interval)
        
        for i in range(commands):
            # Randomly generate linear velocity (-1.0 to 2.5 m/s)
            linear_vel = random.uniform(-1.0, 2.5)
            # Randomly generate angular velocity (-3.0 to 3.0 rad/s)
            angular_vel = random.uniform(-3.0, 3.0)
            
            print(f"-> Random command {i+1}/{commands}: Linear velocity={linear_vel:.2f}, Angular velocity={angular_vel:.2f}")
            vehicle.set_motion(linear_vel, angular_vel)
            time.sleep(interval)
        
        # Stop
        print("-> Stop")
        vehicle.stop()
    
    @staticmethod
    def ack_test(vehicle, commands=20, interval=0.2):
        """Test ACK reliability"""
        print("\n===== ACK Reliability Test =====")
        # Stop
        vehicle.stop()
        time.sleep(interval)
        
        success_count = 0
        fail_count = 0
        
        print(f"Sending {commands} commands and tracking ACKs...")
        for i in range(commands):
            # Alternate between forward and backward
            if i % 2 == 0:
                speed = 0.5 + (i % 5) * 0.2  # 0.5, 0.7, 0.9, 1.1, 1.3
                print(f"-> Command {i+1}: Forward {speed:.1f} m/s")
                result = vehicle.move_forward(speed)
            else:
                speed = 0.3 + (i % 3) * 0.2  # 0.3, 0.5, 0.7
                print(f"-> Command {i+1}: Backward {speed:.1f} m/s")
                result = vehicle.move_backward(speed)
            
            if result:
                success_count += 1
                print("  ACK received ✓")
            else:
                fail_count += 1
                print("  ACK missing ✗")
            
            time.sleep(interval)
        
        # Print results
        print("\n----- ACK Test Results -----")
        print(f"Commands sent: {commands}")
        print(f"ACKs received: {success_count}")
        print(f"ACKs missed: {fail_count}")
        print(f"Success rate: {success_count/commands*100:.1f}%")
        
        # Get detailed statistics
        stats = vehicle.sender.get_statistics()
        print(f"Average round-trip latency: {stats.get('avg_latency_ms', 0):.1f} ms")
        
        # Stop
        print("\n-> Stop")
        vehicle.stop()

def main():
    """Main function"""
    parser = argparse.ArgumentParser(description='RF24L01 Vehicle Control Tool')
    parser.add_argument('--port', type=str, default='COM12', help='Serial port, default is COM12')
    parser.add_argument('--baud', type=int, default=115200, help='Baud rate, default is 115200')
    parser.add_argument('--test', type=str, default='all', 
                       help='Test type: straight, steering, timeout, zigzag, rapid, random, ack, all')
    parser.add_argument('--interval', type=float, default=0.1, 
                       help='Command sending interval (seconds), default 0.1')
    
    args = parser.parse_args()
    
    print(f"Preparing to connect to serial port: {args.port}")
    sender = RF24Sender(args.port, args.baud)
    vehicle = VehicleController(sender)
    interval = max(0.05, args.interval)  # Ensure interval is at least 50ms
    
    try:
        if args.test in ['straight', 'all']:
            TestSequence.straight_line_test(vehicle, interval)
            time.sleep(1)
        
        if args.test in ['steering', 'all']:
            TestSequence.steering_test(vehicle, interval)
            time.sleep(1)
        
        if args.test in ['zigzag', 'all']:
            TestSequence.zigzag_test(vehicle, interval)
            time.sleep(1)
        
        if args.test in ['rapid', 'all']:
            TestSequence.rapid_switch_test(vehicle)
            time.sleep(1)
        
        if args.test in ['random', 'all']:
            TestSequence.random_movement_test(vehicle, interval=interval)
            time.sleep(1)
        
        if args.test in ['timeout', 'all']:
            TestSequence.timeout_test(vehicle)
            time.sleep(1)
        
        if args.test in ['ack', 'all']:
            TestSequence.ack_test(vehicle)
            time.sleep(1)
        
        print("\nAll tests completed!")
        
        # Print final statistics
        stats = sender.get_statistics()
        print("\n===== Overall Communication Statistics =====")
        print(f"Total packets sent: {stats['sent_packets']}")
        print(f"ACKs received: {stats['acked_packets']}")
        print(f"Packet loss rate: {stats['packet_loss_percent']:.2f}%")
        
        if 'avg_latency_ms' in stats:
            print(f"Average round-trip latency: {stats['avg_latency_ms']:.2f} ms")
            print(f"Min latency: {stats['min_latency_ms']:.2f} ms")
            print(f"Max latency: {stats['max_latency_ms']:.2f} ms")
    
    except KeyboardInterrupt:
        print("\nUser interrupted, stopping tests")
    
    except Exception as e:
        print(f"\nError occurred during testing: {e}")
    
    finally:
        # Ensure vehicle is stopped
        try:
            vehicle.stop()
            time.sleep(0.5)
            sender.close()
        except:
            print("Error while closing, serial port may be disconnected")

if __name__ == "__main__":
    main()