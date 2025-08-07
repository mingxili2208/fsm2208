#!/usr/bin/env python3

"""
Patch for CARLA vehicle handler to add timing instrumentation for Experiment B.
This file provides the code modifications needed for your existing CARLA control node.
"""

# Add these imports to your existing CarlaUpdateVehicleHandler node
from sensor_msgs.msg import TimeReference
from autoware_auto_planning_msgs.msg import Trajectory

class CarlaVehicleHandlerPatch:
    """
    Mixin class to add timing instrumentation to CARLA vehicle handler.
    Include this in your existing CARLA control node.
    """
    
    def __init__(self):
        # Add to your existing __init__ method
        
        # Publisher for timing information (T_plan, T_actuate)
        self.actuation_timing_pub = self.create_publisher(
            TimeReference, 
            '/fsm_sandbox/timing/t_plan_t_actuate', 
            10
        )
        
        self.get_logger().info("CARLA handler timing instrumentation enabled")
    
    def instrumented_control_callback(self, msg):
        """
        Enhanced control callback that records timing information.
        Replace your existing control callback with this method.
        """
        # Record T_plan (timestamp from incoming message)
        t_plan_stamp = msg.header.stamp
        
        # Record T_actuate (current time, just before processing)
        t_actuate_stamp = self.get_clock().now().to_msg()
        
        # Publish timing information for the logger
        timing_msg = TimeReference()
        timing_msg.header.stamp = t_plan_stamp  # T_plan
        timing_msg.time_ref = t_actuate_stamp   # T_actuate
        self.actuation_timing_pub.publish(timing_msg)
        
        # Continue with your normal control logic
        self.process_control_command(msg)
        
        # Optional: Log timing for debugging
        t_plan_ns = t_plan_stamp.sec * 1_000_000_000 + t_plan_stamp.nanosec
        t_actuate_ns = t_actuate_stamp.sec * 1_000_000_000 + t_actuate_stamp.nanosec
        latency_ms = (t_actuate_ns - t_plan_ns) / 1e6
        
        if latency_ms > 10:  # Log if latency > 10ms
            self.get_logger().debug(f"Control latency: {latency_ms:.2f}ms")
    
    def process_control_command(self, msg):
        """
        Your existing control logic goes here.
        This method should contain whatever your original control callback did.
        """
        # Example control logic (replace with your actual implementation)
        try:
            # Extract control commands from trajectory message
            if hasattr(msg, 'points') and len(msg.points) > 0:
                target_point = msg.points[0]  # Use first point
                
                # Extract velocity and steering commands
                target_velocity = target_point.longitudinal_velocity_mps
                target_steering = target_point.front_wheel_angle_rad
                
                # Apply to CARLA vehicle (your existing logic)
                self.apply_control_to_carla_vehicle(target_velocity, target_steering)
                
        except Exception as e:
            self.get_logger().error(f"Error processing control command: {e}")
    
    def apply_control_to_carla_vehicle(self, velocity, steering):
        """
        Apply control commands to CARLA vehicle.
        Replace this with your actual CARLA interface code.
        """
        # Your existing CARLA control application code
        pass

# Example of how to modify your existing CARLA node:
"""
class YourExistingCarlaNode(Node, CarlaVehicleHandlerPatch):
    def __init__(self):
        Node.__init__(self, 'your_carla_node')
        CarlaVehicleHandlerPatch.__init__(self)
        
        # Your existing initialization
        
        # Replace your control subscription with:
        self.control_sub = self.create_subscription(
            Trajectory,
            "/planning/trajectory",
            self.instrumented_control_callback,  # Use the instrumented version
            10
        )
"""

# Alternative: Direct modification template
def add_timing_to_existing_callback(existing_callback):
    """
    Decorator to add timing instrumentation to existing control callback.
    Usage: @add_timing_to_existing_callback
    """
    def wrapper(self, msg):
        # Record timing
        t_plan_stamp = msg.header.stamp
        t_actuate_stamp = self.get_clock().now().to_msg()
        
        # Publish timing if publisher exists
        if hasattr(self, 'actuation_timing_pub'):
            timing_msg = TimeReference()
            timing_msg.header.stamp = t_plan_stamp
            timing_msg.time_ref = t_actuate_stamp
            self.actuation_timing_pub.publish(timing_msg)
        
        # Call original callback
        return existing_callback(self, msg)
    
    return wrapper

# Example usage of decorator:
"""
class YourCarlaNode(Node):
    def __init__(self):
        super().__init__('your_carla_node')
        
        # Add timing publisher
        self.actuation_timing_pub = self.create_publisher(
            TimeReference, '/fsm_sandbox/timing/t_plan_t_actuate', 10)
    
    @add_timing_to_existing_callback
    def your_existing_control_callback(self, msg):
        # Your existing control logic unchanged
        pass
"""