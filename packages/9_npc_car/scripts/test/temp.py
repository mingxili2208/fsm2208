#!/usr/bin/env python

import rclpy
from rclpy.node import Node
import math
import threading
import time
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from autoware_auto_vehicle_msgs.msg import Engage
import numpy as np
from transforms3d.euler import quat2euler
import signal
import sys
from rosgraph_msgs.msg import Clock
from builtin_interfaces.msg import Time

class AutowareGoalManager(Node):
    def __init__(self):
        super().__init__("autoware_goal_manager")
        
        # Define two sets of goal positions
        self.goal_sets = {
            "npc_test": [
                # Format: [x, y, z, qx, qy, qz, qw]
                [-48.4787, -32.6236, 0.0, 0.0, 0.0, -0.715607, 0.698503],  # start point
                [29.7123, -36.908, 0.0, 0.0, 0.0, 0.248181, 0.968714],     # end point
                [-6.22938, 50.2089, 0.0, 0.0, 0.0, -0.999831, 0.0183731]   # out point
            ],
            "circle": [
                [-48.3957, -19.3472, 0.0, 0.0, 0.0, -0.723422, 0.690406],
                [54.3231, -47.6682, 0.0, 0.0, 0.0, 0.300639, 0.953738],
                [26.589, 50.3549, 0.0, 0.0, 0.0, -0.999824, 0.01874]
            ]
        }
        
        # Default to use npc_test goals initially
        self.active_goal_set = "npc_test"
        self.goals = self.goal_sets[self.active_goal_set]
        
        self.current_goal_index = 0
        self.current_pose = None
        self.last_position = None  # Save the last position
        self.is_engaged = False
        self.reached_goal = False
        self.distance_threshold = 1
        
        # Movement detection parameters
        self.movement_check_timer = None
        self.engage_time = None
        self.movement_detected = False
        self.retry_count = 0
        self.max_retries = 3
        self.movement_threshold = 0.05  # 5cm movement threshold
        
        # Add subscription to /clock topic
        self.latest_clock = Time()
        self.clock_received = False
        self.clock_subscription = self.create_subscription(
            Clock, '/clock', self.clock_callback, 10
        )
        
        # Create publishers
        self.goal_publisher = self.create_publisher(
            PoseStamped, "/planning/mission_planning/goal", 10
        )
        
        self.engage_publisher = self.create_publisher(
            Engage, "/autoware/engage", 10
        )
        
        # Create subscription to vehicle position
        self.position_subscription = self.create_subscription(
            PoseWithCovarianceStamped,
            "/real_world/follow_adtruck/transformed_with_covariance",
            self.vehicle_position_callback,
            10
        )
        
        # Add a timer for regular status reporting
        self.status_timer = self.create_timer(5.0, self.report_status)
        
        # Add a timer for automatically starting the goal sequence
        self.auto_start_timer = self.create_timer(3.0, self.auto_start_sequence)
        
        # Set signal handler for clean shutdown
        signal.signal(signal.SIGINT, self.signal_handler)
        
        self.running = True
        
        self.get_logger().info("Autoware Goal Manager initialized")
        self.get_logger().info(f"Starting with goal set: {self.active_goal_set}")
    
    def auto_start_sequence(self):
        """Automatically start the goal sequence after initialization"""
        # Call this only once
        self.auto_start_timer.cancel()
        
        self.get_logger().info("Auto-starting goal sequence")
        self.start_goal_sequence()
    
    def clock_callback(self, msg: Clock):
        """Process callback from /clock topic"""
        self.latest_clock = msg.clock
        self.clock_received = True
        
    def signal_handler(self, sig, frame):
        """Handle SIGINT (Ctrl+C) for clean shutdown"""
        self.get_logger().info("Received shutdown signal")
        self.running = False
        self.get_logger().info("Cleaning up and shutting down...")
        self.clean_up_timers()
        sys.exit(0)
    
    def clean_up_timers(self):
        """Clean up all active timers"""
        if self.movement_check_timer:
            self.movement_check_timer.cancel()
            self.movement_check_timer = None
    
    def report_status(self):
        """Report current status"""
        status_msg = (
            f"Status: Goal Set={self.active_goal_set}, "
            f"Goal Index={self.current_goal_index+1}/{len(self.goals)}, "
            f"Engaged={self.is_engaged}, Reached Goal={self.reached_goal}"
        )
        
        if self.current_pose:
            x = self.current_pose.pose.pose.position.x
            y = self.current_pose.pose.pose.position.y
            status_msg += f", Current Position=({x:.2f}, {y:.2f})"
            
            if self.current_goal_index < len(self.goals):
                goal = self.goals[self.current_goal_index]
                distance = self.calculate_distance(x, y, goal[0], goal[1])
                status_msg += f", Distance to Goal={distance:.4f}m"
        
        if self.retry_count > 0:
            status_msg += f", Retries={self.retry_count}/{self.max_retries}"
            
        self.get_logger().info(status_msg)
    
    def vehicle_position_callback(self, msg):
        """Callback for vehicle position updates"""
        self.current_pose = msg
        
        # If currently engaged, update movement status
        if self.is_engaged and self.engage_time is not None:
            curr_x = msg.pose.pose.position.x
            curr_y = msg.pose.pose.position.y
            
            # If this is the first position after engagement, just save it
            if self.last_position is None:
                self.last_position = (curr_x, curr_y)
            else:
                # Calculate the distance from the last position
                distance_moved = self.calculate_distance(
                    curr_x, curr_y, 
                    self.last_position[0], self.last_position[1]
                )
                
                # If movement distance exceeds threshold, mark as moved
                if distance_moved > self.movement_threshold:
                    self.movement_detected = True
                    self.get_logger().debug(f"Movement detected: {distance_moved:.4f}m")
                    # Update position
                    self.last_position = (curr_x, curr_y)
        
        # Check if we should calculate distance to goal
        if self.is_engaged and not self.reached_goal and self.current_goal_index < len(self.goals):
            # Record current time
            current_time = self.get_clock().now()
            if hasattr(self, 'last_distance_check_time'):
                delta = (current_time - self.last_distance_check_time).nanoseconds / 1e9
                check_freq = 1.0 / delta if delta > 0 else 0
                self.get_logger().debug(f"Distance check frequency: {check_freq:.2f} Hz")
            self.last_distance_check_time = current_time
            
            # Calculate distance
            current_goal = self.goals[self.current_goal_index]
            distance = self.calculate_distance(
                msg.pose.pose.position.x, 
                msg.pose.pose.position.y,
                current_goal[0],
                current_goal[1]
            )
            
            # Check if goal reached
            if distance < self.distance_threshold:
                self.get_logger().info(f"Goal {self.current_goal_index + 1} reached! Distance: {distance:.4f} m")
                self.reached_goal = True
                
                # Clean up movement detection state
                self.clean_up_movement_check()
                
                # Schedule next action
                threading.Thread(target=self.handle_goal_reached).start()
    
    def clean_up_movement_check(self):
        """Clean up movement detection related state"""
        if self.movement_check_timer:
            self.movement_check_timer.cancel()
            self.movement_check_timer = None
        self.engage_time = None
        self.movement_detected = False
        self.last_position = None
    
    def check_vehicle_movement(self):
        """Check if vehicle has moved after engagement"""
        if not self.movement_detected and self.is_engaged and not self.reached_goal:
            # If no movement detected within 5 seconds, retry
            self.retry_count += 1
            self.get_logger().warn(f"No movement detected after 5 seconds! Retry {self.retry_count}/{self.max_retries}")
            
            if self.retry_count < self.max_retries:
                # Resend goal and engage
                self.get_logger().info("Resending goal and engage commands...")
                self.publish_goal(self.goals[self.current_goal_index])
                time.sleep(1)  # Wait 1 second
                self.publish_engage(True)
                
                # Reset movement detection
                self.engage_time = time.time()
                self.movement_detected = False
                self.last_position = None
                
                # Set new check timer
                self.movement_check_timer = threading.Timer(5.0, self.check_vehicle_movement)
                self.movement_check_timer.daemon = True
                self.movement_check_timer.start()
            else:
                self.get_logger().error(f"Maximum retry attempts ({self.max_retries}) reached. Please check vehicle status.")
    
    def calculate_distance(self, x1, y1, x2, y2):
        """Calculate Euclidean distance between two points"""
        return math.sqrt((x2 - x1)**2 + (y2 - y1)**2)
    
    def handle_goal_reached(self):
        """Handle actions after goal is reached"""
        self.get_logger().info("Waiting 10 seconds at goal...")
        time.sleep(10)
        
        # Disengage
        self.disengage_and_move_to_next_goal()
    
    def disengage_and_move_to_next_goal(self):
        """Disengage and move to next goal"""
        self.get_logger().info("Disengaging...")
        self.publish_engage(False)
        self.is_engaged = False
        
        # Clean up movement detection state
        self.clean_up_movement_check()
        
        # Wait 5 seconds before moving to next goal
        time.sleep(5)
        
        # Proceed to next goal
        self.current_goal_index += 1
        if self.current_goal_index < len(self.goals):
            self.reached_goal = False
            self.retry_count = 0
            self.start_goal_sequence()
        else:
            self.get_logger().info("All goals in current set completed!")
            # Switch to the other goal set
            if self.active_goal_set == "npc_test":
                self.active_goal_set = "circle"
            else:
                self.active_goal_set = "npc_test"
            
            self.goals = self.goal_sets[self.active_goal_set]
            self.current_goal_index = 0
            self.get_logger().info(f"Switching to goal set: {self.active_goal_set}")
            
            # Start new goal set after a short delay
            time.sleep(3)
            self.reached_goal = False
            self.retry_count = 0
            self.start_goal_sequence()
    
    def start_goal_sequence(self):
        """Start the goal sequence from the current index"""
        if self.current_goal_index < len(self.goals):
            # Publish current goal
            self.publish_goal(self.goals[self.current_goal_index])
            self.get_logger().info(f"Published goal {self.current_goal_index + 1}/{len(self.goals)} from set {self.active_goal_set}")
            
            # Wait 5 seconds and then engage
            threading.Thread(target=self.engage_after_delay).start()
        else:
            self.get_logger().info("All goals have been sent.")
    
    def engage_after_delay(self):
        """Engage the system after a delay"""
        self.get_logger().info("Waiting 5 seconds before engaging...")
        time.sleep(5)
        
        # Send engage command
        self.publish_engage(True)
        self.is_engaged = True
        self.get_logger().info("System engaged!")
        
        # Record engage time and reset movement detection state
        self.engage_time = time.time()
        self.movement_detected = False
        self.last_position = None
        
        # Set timer to check movement after 5 seconds
        self.movement_check_timer = threading.Timer(5.0, self.check_vehicle_movement)
        self.movement_check_timer.daemon = True
        self.movement_check_timer.start()
    
    def publish_goal(self, goal_pos):
        """Publish a goal position to Autoware"""
        msg = PoseStamped()
        
        # Correctly use timestamp from /clock topic
        if self.clock_received:
            msg.header.stamp = self.latest_clock
            self.get_logger().debug(f"Using /clock time for goal: {self.latest_clock.sec}.{self.latest_clock.nanosec}")
        else:
            # If no clock messages received yet, use node clock as fallback
            msg.header.stamp = self.get_clock().now().to_msg()
            self.get_logger().warn("No /clock messages received, using node clock for goal")
        
        msg.header.frame_id = "map"
        
        # Set position - ensure explicit conversion to float
        msg.pose.position.x = float(goal_pos[0])
        msg.pose.position.y = float(goal_pos[1])
        msg.pose.position.z = float(goal_pos[2])
        
        # Set orientation (quaternion) - ensure as float
        msg.pose.orientation.x = float(goal_pos[3])
        msg.pose.orientation.y = float(goal_pos[4])
        msg.pose.orientation.z = float(goal_pos[5])
        msg.pose.orientation.w = float(goal_pos[6])
        
        # Convert quaternion to angle for logging
        _, _, yaw = quat2euler([goal_pos[6], goal_pos[3], goal_pos[4], goal_pos[5]])
        
        self.goal_publisher.publish(msg)
        
        self.get_logger().info(
            f"Setting goal pose: Frame:map, Position({goal_pos[0]}, {goal_pos[1]}, {goal_pos[2]}), "
            f"Orientation({goal_pos[3]}, {goal_pos[4]}, {goal_pos[5]}, {goal_pos[6]}) = Angle: {yaw}"
        )
    
    def publish_engage(self, engage_state):
        """Publish engage command to Autoware"""
        msg = Engage()
        
        # Correctly use timestamp from /clock topic
        if self.clock_received:
            msg.stamp = self.latest_clock
            self.get_logger().debug(f"Using /clock time for engage: {self.latest_clock.sec}.{self.latest_clock.nanosec}")
        else:
            # If no clock messages received yet, use node clock as fallback
            msg.stamp = self.get_clock().now().to_msg()
            self.get_logger().warn("No /clock messages received, using node clock for engage")
        
        msg.engage = engage_state
        
        self.engage_publisher.publish(msg)
        
        self.get_logger().info(f"Published engage state: {engage_state}")

def main():
    rclpy.init()
    goal_manager = None
    try:
        goal_manager = AutowareGoalManager()
        rclpy.spin(goal_manager)
    except KeyboardInterrupt:
        if goal_manager:
            goal_manager.get_logger().info("Shutting down due to keyboard interrupt")
    except Exception as e:
        if goal_manager:
            goal_manager.get_logger().error(f"Error occurred: {str(e)}")
        else:
            print(f"Error occurred during initialization: {str(e)}")
    finally:
        if goal_manager:
            goal_manager.clean_up_timers()
        if goal_manager:
            goal_manager.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()