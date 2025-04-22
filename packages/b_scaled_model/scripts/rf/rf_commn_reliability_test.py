import serial
import struct
import time
import argparse
import random
import threading
import statistics

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
        self.send_timestamps = {}
        self.sent_sequences = []  # Store sequence numbers for analysis
        self.transmission_times = []  # Store transmission times
        print(f"Connected to {port} at {baudrate} baud")
        time.sleep(2)  # Wait for serial port to stabilize
    
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
    
    def send_motion_data(self, device_id, linear_vel, angular_vel, linear_acc=0, angular_acc=0, log=True):
        """Send motion data packet"""
        packet = self.pack_motion_data(device_id, linear_vel, angular_vel, linear_acc, angular_acc)
        start_time = time.time()
        bytes_written = self.serial.write(packet)
        self.serial.flush()
        end_time = time.time()
        
        # Record transmission time and sequence number
        self.sent_packets += 1
        seq_num = packet[12]
        self.sent_sequences.append(seq_num)
        self.send_timestamps[seq_num] = start_time
        self.transmission_times.append((end_time - start_time) * 1000)  # Convert to ms
        
        # Calculate the effective data length (system will add this as byte 0)
        effective_length = 15  # Dual headers to tail length
        
        # Log sent data
        if log:
            print(f"Sending motion data: Device ID={device_id}, Sequence={packet[12]}")
            print(f"  Packet length: {effective_length} bytes (automatically added by system)")
            print(f"  Linear velocity: {linear_vel} ({linear_vel/1000:.2f} m/s), Angular velocity: {angular_vel}")
            print(f"  Linear acceleration: {linear_acc}, Angular acceleration: {angular_acc}")
            print(f"  Bytes sent: {bytes_written}")
            print(f"  Transmission time: {(end_time - start_time) * 1000:.3f} ms")
            print(f"  Raw data: {packet.hex(' ')}")
            print()
        
        return bytes_written
    
    def close(self):
        """Close serial connection"""
        if self.serial and self.serial.is_open:
            self.serial.close()
            print("Serial port closed")
    
    def get_statistics(self):
        """Get transmission statistics"""
        stats = {}
        if self.transmission_times:
            stats["avg_time_ms"] = statistics.mean(self.transmission_times)
            stats["min_time_ms"] = min(self.transmission_times)
            stats["max_time_ms"] = max(self.transmission_times)
            stats["total_packets"] = self.sent_packets
            stats["data_rate_pps"] = 1000 / stats["avg_time_ms"] if stats["avg_time_ms"] > 0 else 0
            stats["data_rate_bps"] = stats["data_rate_pps"] * 15  # 15 bytes per packet
        return stats

class ACK_Receiver(threading.Thread):
    """Thread to handle ACK responses from the receiver"""
    def __init__(self, serial_conn, sender):
        threading.Thread.__init__(self)
        self.serial = serial_conn
        self.sender = sender
        self.running = True
        self.received_acks = 0
        self.received_sequences = []
        self.ack_timestamps = {}
        self.ack_latencies = []
        self.daemon = True
    
    def run(self):
        """Thread main loop"""
        while self.running:
            if self.serial.in_waiting > 0:
                try:
                    data = bytearray(self.serial.read(self.serial.in_waiting))
                    if len(data) >= 4 and data[0] == FRAME_HEADER1 and data[1] == FRAME_HEADER2:
                        if data[2] == PacketType.PACKET_ACK:
                            seq_num = data[3]
                            now = time.time()
                            self.received_acks += 1
                            self.received_sequences.append(seq_num)
                            self.ack_timestamps[seq_num] = now
                            
                            # Calculate latency if we have the send timestamp
                            if seq_num in self.sender.send_timestamps:
                                latency = (now - self.sender.send_timestamps[seq_num]) * 1000  # ms
                                self.ack_latencies.append(latency)
                                print(f"  ACK received for seq={seq_num}, latency={latency:.3f} ms")
                except Exception as e:
                    print(f"Error reading ACK: {e}")
            time.sleep(0.001)  # Short sleep to prevent CPU hogging
    
    def stop(self):
        """Stop the thread"""
        self.running = False
    
    def get_statistics(self):
        """Get ACK statistics"""
        stats = {}
        stats["received_acks"] = self.received_acks
        
        # Calculate packet loss
        total_sent = self.sender.sent_packets
        stats["packet_loss_percent"] = 0 if total_sent == 0 else 100 * (total_sent - self.received_acks) / total_sent
        
        # Calculate average latency
        if self.ack_latencies:
            stats["avg_latency_ms"] = statistics.mean(self.ack_latencies)
            stats["min_latency_ms"] = min(self.ack_latencies)
            stats["max_latency_ms"] = max(self.ack_latencies)
        
        # Calculate sequence number sync
        if self.received_sequences and self.sender.sent_sequences:
            # Find common sequences
            common_seq = set(self.received_sequences).intersection(set(self.sender.sent_sequences))
            time_diffs = []
            
            for seq in common_seq:
                if seq in self.ack_timestamps and seq in self.sender.send_timestamps:
                    time_diff = abs(self.ack_timestamps[seq] - self.sender.send_timestamps[seq]) * 1000  # ms
                    time_diffs.append(time_diff)
            
            if time_diffs:
                stats["avg_time_diff_ms"] = statistics.mean(time_diffs)
                stats["sync_within_10ms"] = sum(1 for t in time_diffs if t <= 10) / len(time_diffs) * 100
        
        return stats

class TestSequence:
    """Test sequence generator"""
    
    @staticmethod
    def straight_line_test(sender, device_id=1, interval=0.5):
        """Straight line test: first forward, then backward"""
        print("\n===== Straight Line Test =====")
        # Stop
        sender.send_motion_data(device_id, 0, 0)
        time.sleep(interval)
        
        # Slow forward
        print("-> Slow forward")
        sender.send_motion_data(device_id, 500, 0)  # 0.5 m/s
        time.sleep(2)
        
        # Medium speed forward
        print("-> Medium speed forward")
        sender.send_motion_data(device_id, 1000, 0)  # 1.0 m/s
        time.sleep(2)
        
        # Fast forward
        print("-> Fast forward")
        sender.send_motion_data(device_id, 2000, 0)  # 2.0 m/s
        time.sleep(2)
        
        # Stop
        print("-> Stop")
        sender.send_motion_data(device_id, 0, 0)
        time.sleep(1)
        
        # Slow backward
        print("-> Slow backward")
        sender.send_motion_data(device_id, -300, 0)  # -0.3 m/s
        time.sleep(2)
        
        # Medium speed backward
        print("-> Medium speed backward")
        sender.send_motion_data(device_id, -600, 0)  # -0.6 m/s
        time.sleep(2)
        
        # Maximum backward
        print("-> Maximum backward")
        sender.send_motion_data(device_id, -1000, 0)  # -1.0 m/s
        time.sleep(2)
        
        # Stop
        print("-> Stop")
        sender.send_motion_data(device_id, 0, 0)
    
    @staticmethod
    def steering_test(sender, device_id=1, interval=0.5):
        """Steering test: left and right turn while moving forward"""
        print("\n===== Steering Test =====")
        # Stop
        sender.send_motion_data(device_id, 0, 0)
        time.sleep(interval)
        
        # Forward and left turn
        print("-> Forward and left turn")
        sender.send_motion_data(device_id, 800, -1500)  # Forward 0.8m/s, left turn
        time.sleep(3)
        
        # Forward straight
        print("-> Forward straight")
        sender.send_motion_data(device_id, 800, 0)  # Forward 0.8m/s, straight
        time.sleep(2)
        
        # Forward and right turn
        print("-> Forward and right turn")
        sender.send_motion_data(device_id, 800, 1500)  # Forward 0.8m/s, right turn
        time.sleep(3)
        
        # Stop
        print("-> Stop")
        sender.send_motion_data(device_id, 0, 0)
        time.sleep(interval)
        
        # Rotate left in place
        print("-> Rotate left in place")
        sender.send_motion_data(device_id, 0, -3000)  # Rotate left in place
        time.sleep(3)
        
        # Rotate right in place
        print("-> Rotate right in place")
        sender.send_motion_data(device_id, 0, 3000)  # Rotate right in place
        time.sleep(3)
        
        # Stop
        print("-> Stop")
        sender.send_motion_data(device_id, 0, 0)
    
    @staticmethod
    def timeout_test(sender, device_id=1, interval=1.0):
        """Timeout test: send command then stop sending, test receiver timeout function"""
        print("\n===== Timeout Test =====")
        
        # Forward
        print("-> Forward, then stop sending data")
        sender.send_motion_data(device_id, 1000, 0)  # 1.0 m/s
        
        # Wait longer than receiver timeout (5 seconds)
        print("-> Wait 6 seconds, receiver should stop after 5 seconds...")
        for i in range(6):
            print(f"    Waiting: {i+1} seconds")
            time.sleep(1)
            
        # Resume communication
        print("-> Resume communication, send stop command")
        sender.send_motion_data(device_id, 0, 0)
    
    @staticmethod
    def zigzag_test(sender, device_id=1, interval=0.5):
        """Variable speed and direction test: Z-shaped path"""
        print("\n===== Z-shaped Path Test =====")
        # Stop
        sender.send_motion_data(device_id, 0, 0)
        time.sleep(interval)
        
        # Forward and right turn
        print("-> Forward and right turn")
        sender.send_motion_data(device_id, 1000, 2000)
        time.sleep(2)
        
        # Forward and left turn
        print("-> Forward and left turn")
        sender.send_motion_data(device_id, 1200, -2000)
        time.sleep(2)
        
        # Forward and right turn
        print("-> Forward and right turn")
        sender.send_motion_data(device_id, 1400, 2000)
        time.sleep(2)
        
        # Slow down and go straight
        print("-> Slow down and go straight")
        sender.send_motion_data(device_id, 700, 0)
        time.sleep(2)
        
        # Stop
        print("-> Stop")
        sender.send_motion_data(device_id, 0, 0)
    
    @staticmethod
    def rapid_switch_test(sender, device_id=1, duration=5):
        """Rapid switching test: quickly alternate between forward and backward commands"""
        print("\n===== Rapid Switching Test =====")
        
        start_time = time.time()
        toggle = True
        
        print(f"-> Start rapid switching forward/backward, duration {duration} seconds")
        while time.time() - start_time < duration:
            if toggle:
                sender.send_motion_data(device_id, 1000, 0, log=False)  # Forward
            else:
                sender.send_motion_data(device_id, -500, 0, log=False)  # Backward
            toggle = not toggle
            time.sleep(0.2)  # Switch every 200ms
        
        # Stop
        print("-> Stop")
        sender.send_motion_data(device_id, 0, 0)
    
    @staticmethod
    def random_movement_test(sender, device_id=1, commands=20, interval=0.5):
        """Random movement test: randomly generate speed and direction"""
        print("\n===== Random Movement Test =====")
        # Stop
        sender.send_motion_data(device_id, 0, 0)
        time.sleep(interval)
        
        for i in range(commands):
            # Randomly generate linear velocity (-1000 to 2500)
            linear_vel = random.randint(-1000, 2500)
            # Randomly generate angular velocity (-3000 to 3000)
            angular_vel = random.randint(-3000, 3000)
            
            print(f"-> Random command {i+1}/{commands}: Linear velocity={linear_vel}, Angular velocity={angular_vel}")
            sender.send_motion_data(device_id, linear_vel, angular_vel)
            time.sleep(interval)
        
        # Stop
        print("-> Stop")
        sender.send_motion_data(device_id, 0, 0)
    
    @staticmethod
    def data_rate_test(sender, device_id=1, duration=5, min_interval=0.01):
        """Test maximum data rate over a period of time with minimal interval"""
        print(f"\n===== Data Rate Test (duration: {duration}s, min interval: {min_interval}s) =====")
        
        # Stop any previous motion
        sender.send_motion_data(device_id, 0, 0, log=False)
        time.sleep(0.1)
        
        # Reset counters for this test
        sender.sent_packets = 0
        sender.sent_sequences = []
        sender.send_timestamps = {}
        sender.transmission_times = []
        
        packet_count = 0
        start_time = time.time()
        print(f"-> Starting data rate test for {duration} seconds...")
        
        try:
            while time.time() - start_time < duration:
                # Send with alternating values to mimic real commands but disable logging
                linear_vel = 1000 if packet_count % 2 == 0 else -1000
                angular_vel = 500 if packet_count % 4 < 2 else -500
                
                sender.send_motion_data(device_id, linear_vel, angular_vel, log=False)
                packet_count += 1
                
                # Small delay to prevent CPU hogging and to respect minimum interval
                time.sleep(min_interval)
        
        except KeyboardInterrupt:
            print("\nTest interrupted by user")
        
        # Calculate elapsed time
        elapsed_time = time.time() - start_time
        
        # Display results
        if packet_count > 0:
            packet_rate = packet_count / elapsed_time
            byte_rate = packet_rate * 15  # Each packet is 15 bytes
            
            print(f"\n----- Data Rate Test Results -----")
            print(f"Test duration: {elapsed_time:.2f} seconds")
            print(f"Packets sent: {packet_count}")
            print(f"Sending rate: {packet_rate:.2f} packets/second")
            print(f"Data rate: {byte_rate:.2f} bytes/second ({byte_rate*8:.2f} bits/second)")
            
            # Get detailed statistics
            stats = sender.get_statistics()
            print(f"Average transmission time: {stats.get('avg_time_ms', 0):.3f} ms")
            print(f"Min transmission time: {stats.get('min_time_ms', 0):.3f} ms")
            print(f"Max transmission time: {stats.get('max_time_ms', 0):.3f} ms")
        
        # Stop any motion
        sender.send_motion_data(device_id, 0, 0, log=False)
        return packet_count, elapsed_time
    
    @staticmethod
    def packet_loss_test(sender, ack_receiver, device_id=1, packets=100, interval=0.05):
        """Test packet loss by sending packets and counting received ACKs"""
        print(f"\n===== Packet Loss Test (packets: {packets}, interval: {interval}s) =====")
        
        # Reset counters
        sender.sent_packets = 0
        sender.sent_sequences = []
        sender.send_timestamps = {}
        ack_receiver.received_acks = 0
        ack_receiver.received_sequences = []
        ack_receiver.ack_timestamps = {}
        ack_receiver.ack_latencies = []
        
        # Send packets
        print(f"-> Sending {packets} packets at {interval}s intervals...")
        
        for i in range(packets):
            # Send with alternating values
            linear_vel = 500 if i % 2 == 0 else -500
            angular_vel = 250 if i % 4 < 2 else -250
            
            sender.send_motion_data(device_id, linear_vel, angular_vel, log=False)
            if i % 10 == 0:
                print(f"  Sent {i} packets...")
            time.sleep(interval)
        
        # Give some time for all ACKs to be received
        time.sleep(1.0)
        
        # Calculate results
        stats = ack_receiver.get_statistics()
        
        print(f"\n----- Packet Loss Test Results -----")
        print(f"Packets sent: {sender.sent_packets}")
        print(f"ACKs received: {stats['received_acks']}")
        print(f"Packet loss: {stats['packet_loss_percent']:.2f}%")
        
        if 'avg_latency_ms' in stats:
            print(f"Average round-trip latency: {stats['avg_latency_ms']:.3f} ms")
            print(f"Min latency: {stats['min_latency_ms']:.3f} ms")
            print(f"Max latency: {stats['max_latency_ms']:.3f} ms")
        
        if 'avg_time_diff_ms' in stats:
            print(f"Average time difference between send/receive: {stats['avg_time_diff_ms']:.3f} ms")
            print(f"Percentage of packets with timing within 10ms: {stats['sync_within_10ms']:.2f}%")
        
        # Stop any motion
        sender.send_motion_data(device_id, 0, 0, log=False)
        return stats

def main():
    """Main function"""
    parser = argparse.ArgumentParser(description='RF24L01 Receiver Testing Tool')
    parser.add_argument('--port', type=str, default='COM12', help='Serial port, default is COM12')
    parser.add_argument('--baud', type=int, default=115200, help='Baud rate, default is 115200')
    parser.add_argument('--test', type=str, default='all', 
                       help='Test type: straight, steering, timeout, zigzag, rapid, random, datarate, packetloss, all')
    parser.add_argument('--device', type=int, default=1, help='Device ID, default is 1')
    parser.add_argument('--interval', type=float, default=0.1, 
                       help='Command sending interval (seconds), default 0.1, note RF24L01 requires interval >10ms')
    parser.add_argument('--duration', type=int, default=5, help='Test duration in seconds for datarate test')
    parser.add_argument('--packets', type=int, default=100, help='Number of packets to send for packetloss test')
    
    args = parser.parse_args()
    
    print(f"Preparing to connect to serial port: {args.port}")
    sender = RF24Sender(args.port, args.baud)
    interval = max(0.01, args.interval)  # Ensure interval is at least 10ms
    
    # Start ACK receiver thread
    ack_receiver = ACK_Receiver(sender.serial, sender)
    ack_receiver.start()
    
    try:
        if args.test in ['straight', 'all']:
            TestSequence.straight_line_test(sender, args.device, interval)
            time.sleep(1)
        
        if args.test in ['steering', 'all']:
            TestSequence.steering_test(sender, args.device, interval)
            time.sleep(1)
        
        if args.test in ['zigzag', 'all']:
            TestSequence.zigzag_test(sender, args.device, interval)
            time.sleep(1)
        
        if args.test in ['rapid', 'all']:
            TestSequence.rapid_switch_test(sender, args.device)
            time.sleep(1)
        
        if args.test in ['random', 'all']:
            TestSequence.random_movement_test(sender, args.device, interval=interval)
            time.sleep(1)
        
        if args.test in ['timeout', 'all']:
            TestSequence.timeout_test(sender, args.device)
            time.sleep(1)
        
        if args.test in ['datarate', 'all']:
            TestSequence.data_rate_test(sender, args.device, duration=args.duration, min_interval=args.interval)
            time.sleep(1)
        
        if args.test in ['packetloss', 'all']:
            TestSequence.packet_loss_test(sender, ack_receiver, args.device, 
                                         packets=args.packets, interval=args.interval)
            time.sleep(1)
        
        print("\nAll tests completed!")
    
    except KeyboardInterrupt:
        print("\nUser interrupted, stopping tests")
    
    except Exception as e:
        print(f"\nError occurred during testing: {e}")
    
    finally:
        # Stop ACK receiver thread
        ack_receiver.stop()
        
        # Ensure vehicle is stopped
        try:
            sender.send_motion_data(args.device, 0, 0)
            time.sleep(0.5)
            sender.close()
        except:
            print("Error while closing, serial port may be disconnected")

if __name__ == "__main__":
    main()