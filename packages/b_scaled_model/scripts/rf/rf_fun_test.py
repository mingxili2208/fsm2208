import serial
import struct
import time
import argparse
import random

# Communication protocol definition
FRAME_HEADER = 0xAA  # Frame header identifier
FRAME_TAIL = 0x55    # Frame tail identifier

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
        print(f"Connected to {port} at {baudrate} baud")
        time.sleep(2)  # Wait for serial port to stabilize
    
    def calculate_checksum(self, data):
        """Calculate checksum"""
        return sum(data) & 0xFF
    
    def pack_motion_data(self, device_id, linear_vel, angular_vel, linear_acc=0, angular_acc=0):
        """Pack motion data packet - following RF24L01 specifications"""
        # Initialize packet without the length byte (system will generate it)
        # We now use 31 bytes instead of 32 since byte 0 is automatically generated
        packet = bytearray(31)
        
        # Start filling packet content from position 0 (which will be position 1 in the final packet)
        packet[0] = FRAME_HEADER  # Frame header
        packet[1] = PacketType.PACKET_MOTION  # Packet type
        packet[2] = device_id  # Device ID
        
        # Fill velocity and acceleration data (using little-endian byte order, compatible with Arduino)
        struct.pack_into('<h', packet, 3, linear_vel)    # Linear velocity (2 bytes)
        struct.pack_into('<h', packet, 5, angular_vel)   # Angular velocity (2 bytes)
        struct.pack_into('<h', packet, 7, linear_acc)    # Linear acceleration (2 bytes)
        struct.pack_into('<h', packet, 9, angular_acc)   # Angular acceleration (2 bytes)
        
        # Fill sequence number
        packet[11] = self.sequence_counter
        self.sequence_counter = (self.sequence_counter + 1) & 0xFF
        
        # Calculate checksum (excluding the checksum field itself and frame tail)
        checksum = self.calculate_checksum(packet[0:12])
        packet[12] = checksum
        
        # Fill frame tail
        packet[13] = FRAME_TAIL
        
        return packet
    
    def send_motion_data(self, device_id, linear_vel, angular_vel, linear_acc=0, angular_acc=0):
        """Send motion data packet"""
        packet = self.pack_motion_data(device_id, linear_vel, angular_vel, linear_acc, angular_acc)
        bytes_written = self.serial.write(packet)
        self.serial.flush()
        
        # Calculate the effective data length (system will add this as byte 0)
        effective_length = 14  # Frame header to tail length
        
        # Log sent data
        print(f"Sending motion data: Device ID={device_id}, Sequence={packet[11]}")
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
                sender.send_motion_data(device_id, 1000, 0)  # Forward
            else:
                sender.send_motion_data(device_id, -500, 0)  # Backward
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

def main():
    """Main function"""
    parser = argparse.ArgumentParser(description='RF24L01 Receiver Testing Tool')
    parser.add_argument('--port', type=str, default='COM12', help='Serial port, default is COM12')
    parser.add_argument('--baud', type=int, default=115200, help='Baud rate, default is 115200')
    parser.add_argument('--test', type=str, default='all', 
                       help='Test type: straight, steering, timeout, zigzag, rapid, random, all')
    parser.add_argument('--device', type=int, default=1, help='Device ID, default is 1')
    parser.add_argument('--interval', type=float, default=0.1, 
                       help='Command sending interval (seconds), default 0.1, note RF24L01 requires interval >10ms')
    
    args = parser.parse_args()
    
    print(f"Preparing to connect to serial port: {args.port}")
    sender = RF24Sender(args.port, args.baud)
    interval = max(0.01, args.interval)  # Ensure interval is at least 10ms
    
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