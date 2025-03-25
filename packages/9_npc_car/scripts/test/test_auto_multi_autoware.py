#!/usr/bin/env python

import rclpy
from rclpy.node import Node
import math
import threading
import time
import pygame
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from autoware_auto_vehicle_msgs.msg import Engage
import numpy as np
from transforms3d.euler import quat2euler
import signal
import sys

class AutowareGoalManager(Node):
    def __init__(self):
        super().__init__("autoware_goal_manager")
        
        # Goal positions list - 确保所有值都是float类型
        self.goals = [
            # Format: [x, y, z, qx, qy, qz, qw]
            [49.233, -51.3687, 0.0, 0.0, 0.0, 0.289784, 0.957092],
            [-47.2839, 27.3298, 0.0, 0.0, 0.0, -0.707107, 0.707107],
            [-48.9462, -44.6544, 0.0, 0.0, 0.0, -0.740755, 0.671775]
        ]
        
        self.current_goal_index = 0
        self.current_pose = None
        self.last_position = None  # 保存上次的位置
        self.is_engaged = False
        self.reached_goal = False
        self.distance_threshold = 1
        
        # 位置变化监测相关参数
        self.movement_check_timer = None
        self.engage_time = None
        self.movement_detected = False
        self.retry_count = 0
        self.max_retries = 3
        self.movement_threshold = 0.05  # 5厘米的位置变化视为移动
        
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
        
        # Initialize pygame for keyboard input
        pygame.init()
        self.screen = pygame.display.set_mode((400, 100))
        pygame.display.set_caption("Autoware Goal Manager - Press 'e' to start")
        
        # Add a timer for regular status reporting
        self.status_timer = self.create_timer(5.0, self.report_status)
        
        # Setup a timer for pygame event processing
        self.pygame_timer = self.create_timer(0.1, self.process_pygame_events)
        
        # Set signal handler for clean shutdown
        signal.signal(signal.SIGINT, self.signal_handler)
        
        self.running = True
        
        self.get_logger().info("Autoware Goal Manager initialized")
        self.get_logger().info("Press 'e' to start the goal sequence")
        
        # Draw initial instruction screen
        self.update_display()
    
    def signal_handler(self, sig, frame):
        """Handle SIGINT (Ctrl+C) for clean shutdown"""
        self.get_logger().info("Received shutdown signal")
        self.running = False
        self.get_logger().info("Cleaning up and shutting down...")
        self.clean_up_timers()
        pygame.quit()
        sys.exit(0)
    
    def clean_up_timers(self):
        """清理所有活动的定时器"""
        if self.movement_check_timer:
            self.movement_check_timer.cancel()
            self.movement_check_timer = None
    
    def update_display(self):
        """Update the pygame display with current status"""
        self.screen.fill((0, 0, 0))  # Black background
        font = pygame.font.Font(None, 24)
        
        # Display instructions
        text1 = font.render("Press 'e' to start/restart goal sequence", True, (255, 255, 255))
        text2 = font.render("Press 'q' to quit", True, (255, 255, 255))
        self.screen.blit(text1, (20, 20))
        self.screen.blit(text2, (20, 50))
        
        # Display current status
        status_text = f"Goal: {self.current_goal_index+1}/{len(self.goals)}, Engaged: {self.is_engaged}"
        text3 = font.render(status_text, True, (0, 255, 0))
        self.screen.blit(text3, (20, 80))
        
        pygame.display.flip()
    
    def process_pygame_events(self):
        """Process pygame events for keyboard input"""
        if not self.running:
            return
            
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                self.running = False
                self.clean_up_timers()
                pygame.quit()
                sys.exit(0)
            
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_e:  # 'e' key
                    self.get_logger().info("'e' key pressed, starting goal sequence")
                    # Reset sequence and start from first goal
                    self.current_goal_index = 0
                    self.reached_goal = False
                    self.retry_count = 0
                    self.start_goal_sequence()
                    
                elif event.key == pygame.K_q:  # 'q' key
                    self.get_logger().info("'q' key pressed, shutting down")
                    self.running = False
                    self.clean_up_timers()
                    pygame.quit()
                    sys.exit(0)
        
        # Update the display
        self.update_display()
    
    def report_status(self):
        """Report current status"""
        status_msg = (
            f"Status: Goal Index={self.current_goal_index+1}/{len(self.goals)}, "
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
        
        # 如果当前已激活，更新移动状态
        if self.is_engaged and self.engage_time is not None:
            curr_x = msg.pose.pose.position.x
            curr_y = msg.pose.pose.position.y
            
            # 如果这是激活后的第一个位置，只保存但不检查
            if self.last_position is None:
                self.last_position = (curr_x, curr_y)
            else:
                # 计算与上次位置的距离
                distance_moved = self.calculate_distance(
                    curr_x, curr_y, 
                    self.last_position[0], self.last_position[1]
                )
                
                # 如果移动距离超过阈值，标记为已移动
                if distance_moved > self.movement_threshold:
                    self.movement_detected = True
                    self.get_logger().debug(f"Movement detected: {distance_moved:.4f}m")
                    # 更新位置
                    self.last_position = (curr_x, curr_y)
        
        # 检查是否应该计算到目标的距离
        if self.is_engaged and not self.reached_goal and self.current_goal_index < len(self.goals):
            # 记录当前时间
            current_time = self.get_clock().now()
            if hasattr(self, 'last_distance_check_time'):
                delta = (current_time - self.last_distance_check_time).nanoseconds / 1e9
                check_freq = 1.0 / delta if delta > 0 else 0
                self.get_logger().debug(f"Distance check frequency: {check_freq:.2f} Hz")
            self.last_distance_check_time = current_time
            
            # 计算距离
            current_goal = self.goals[self.current_goal_index]
            distance = self.calculate_distance(
                msg.pose.pose.position.x, 
                msg.pose.pose.position.y,
                current_goal[0],
                current_goal[1]
            )
            
            # 检查是否达到目标
            if distance < self.distance_threshold:
                self.get_logger().info(f"Goal {self.current_goal_index + 1} reached! Distance: {distance:.4f} m")
                self.reached_goal = True
                
                # 清理移动检测状态
                self.clean_up_movement_check()
                
                # 安排下一步操作
                threading.Thread(target=self.handle_goal_reached).start()
    
    def clean_up_movement_check(self):
        """清理移动检测相关的状态"""
        if self.movement_check_timer:
            self.movement_check_timer.cancel()
            self.movement_check_timer = None
        self.engage_time = None
        self.movement_detected = False
        self.last_position = None
    
    def check_vehicle_movement(self):
        """检查车辆在激活后是否有移动"""
        if not self.movement_detected and self.is_engaged and not self.reached_goal:
            # 如果5秒内没有检测到移动，重试
            self.retry_count += 1
            self.get_logger().warn(f"No movement detected after 5 seconds! Retry {self.retry_count}/{self.max_retries}")
            
            if self.retry_count < self.max_retries:
                # 重新发送goal和engage
                self.get_logger().info("Resending goal and engage commands...")
                self.publish_goal(self.goals[self.current_goal_index])
                time.sleep(1)  # 等待1秒
                self.publish_engage(True)
                
                # 重置移动检测
                self.engage_time = time.time()
                self.movement_detected = False
                self.last_position = None
                
                # 设置新的检查定时器
                self.movement_check_timer = threading.Timer(5.0, self.check_vehicle_movement)
                self.movement_check_timer.daemon = True
                self.movement_check_timer.start()
            else:
                self.get_logger().error(f"Maximum retry attempts ({self.max_retries}) reached. Please check vehicle status.")
                # 可以选择是否要自动进入下一个目标
                # self.disengage_and_move_to_next_goal()
    
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
        """断开连接并移动到下一个目标"""
        self.get_logger().info("Disengaging...")
        self.publish_engage(False)
        self.is_engaged = False
        
        # 清理移动检测状态
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
            self.get_logger().info("All goals completed! Press 'e' to restart sequence.")
    
    def start_goal_sequence(self):
        """Start the goal sequence from the current index"""
        if self.current_goal_index < len(self.goals):
            # Publish current goal
            self.publish_goal(self.goals[self.current_goal_index])
            self.get_logger().info(f"Published goal {self.current_goal_index + 1}/{len(self.goals)}")
            
            # Wait 5 seconds and then engage
            threading.Thread(target=self.engage_after_delay).start()
        else:
            self.get_logger().info("All goals have been sent. Press 'e' to restart.")
    
    def engage_after_delay(self):
        """Engage the system after a delay"""
        self.get_logger().info("Waiting 5 seconds before engaging...")
        time.sleep(5)
        
        # 发送engage命令
        self.publish_engage(True)
        self.is_engaged = True
        self.get_logger().info("System engaged!")
        
        # 记录engage时间和重置移动检测状态
        self.engage_time = time.time()
        self.movement_detected = False
        self.last_position = None
        
        # 设置5秒后检查移动的定时器
        self.movement_check_timer = threading.Timer(5.0, self.check_vehicle_movement)
        self.movement_check_timer.daemon = True
        self.movement_check_timer.start()
    
    def publish_goal(self, goal_pos):
        """Publish a goal position to Autoware"""
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        
        # Set position - 确保显式转换为float类型
        msg.pose.position.x = float(goal_pos[0])
        msg.pose.position.y = float(goal_pos[1])
        msg.pose.position.z = float(goal_pos[2])
        
        # Set orientation (quaternion) - 同样确保是float类型
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
        msg.stamp = self.get_clock().now().to_msg()
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
        pygame.quit()
        if goal_manager:
            goal_manager.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()