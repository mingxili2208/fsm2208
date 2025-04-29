#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from autoware_auto_planning_msgs.msg import Path, Trajectory
from geometry_msgs.msg import PoseWithCovarianceStamped
import os
import json
from datetime import datetime


class TopicLogger(Node):
    def __init__(self):
        super().__init__('topic_logger')
        
        # 创建日志目录
        self.log_dir = os.path.expanduser('./ros2_topic_logs')
        os.makedirs(self.log_dir, exist_ok=True)
        
        # 创建时间戳，用于日志文件名
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        
        # 初始化日志文件
        self.log_files = {
            'behavior_planning_path': open(f'{self.log_dir}/behavior_planning_path_{timestamp}.txt', 'w'),
            'trajectory': open(f'{self.log_dir}/trajectory_{timestamp}.txt', 'w'),
            'start_planner': open(f'{self.log_dir}/start_planner_{timestamp}.txt', 'w'),
            'transformed_with_covariance': open(f'{self.log_dir}/transformed_with_covariance_{timestamp}.txt', 'w'),
        }
        
        # 为每个订阅创建不同的回调组，以避免阻塞
        callback_group1 = MutuallyExclusiveCallbackGroup()
        callback_group2 = MutuallyExclusiveCallbackGroup()
        callback_group3 = MutuallyExclusiveCallbackGroup()
        callback_group4 = MutuallyExclusiveCallbackGroup()
        
        # 创建四个订阅，每个使用不同的回调组
        self.subscription1 = self.create_subscription(
            Path,
            '/planning/scenario_planning/lane_driving/behavior_planning/path',
            self.behavior_planning_path_callback,
            10,
            callback_group=callback_group1
        )
        
        self.subscription2 = self.create_subscription(
            Trajectory,
            '/planning/scenario_planning/trajectory',
            self.trajectory_callback,
            10,
            callback_group=callback_group2
        )
        
        self.subscription3 = self.create_subscription(
            Path,
            '/planning/path_candidate/start_planner',
            self.start_planner_callback,
            10,
            callback_group=callback_group3
        )
        
        self.subscription4 = self.create_subscription(
            PoseWithCovarianceStamped,
            '/real_world/follow_adtruck/transformed_with_covariance',
            self.transformed_with_covariance_callback,
            10,
            callback_group=callback_group4
        )
        
        self.get_logger().info('Topic logger initialized')
        self.get_logger().info(f'Logs will be saved to: {self.log_dir}')
        
    def behavior_planning_path_callback(self, msg):
        log_entry = self._create_log_entry('behavior_planning_path', msg)
        self.log_files['behavior_planning_path'].write(log_entry + '\n')
        self.log_files['behavior_planning_path'].flush()  # 确保数据立即写入文件
        
    def trajectory_callback(self, msg):
        log_entry = self._create_log_entry('trajectory', msg)
        self.log_files['trajectory'].write(log_entry + '\n')
        self.log_files['trajectory'].flush()
        
    def start_planner_callback(self, msg):
        log_entry = self._create_log_entry('start_planner', msg)
        self.log_files['start_planner'].write(log_entry + '\n')
        self.log_files['start_planner'].flush()
        
    def transformed_with_covariance_callback(self, msg):
        log_entry = self._create_log_entry('transformed_with_covariance', msg)
        self.log_files['transformed_with_covariance'].write(log_entry + '\n')
        self.log_files['transformed_with_covariance'].flush()
    
    def _create_log_entry(self, topic_name, msg):
        """创建包含时间戳和消息内容的日志条目"""
        timestamp = self.get_clock().now().to_msg()
        
        # 为不同类型的消息准备数据
        if topic_name in ['behavior_planning_path', 'start_planner']:
            # 处理 Path 消息
            points_data = []
            for point in msg.points:
                point_data = {
                    'x': point.pose.position.x,
                    'y': point.pose.position.y,
                    'z': point.pose.position.z,
                    'orientation': {
                        'x': point.pose.orientation.x,
                        'y': point.pose.orientation.y,
                        'z': point.pose.orientation.z,
                        'w': point.pose.orientation.w
                    },
                    'longitudinal_velocity_mps': point.longitudinal_velocity_mps,
                    'lateral_velocity_mps': point.lateral_velocity_mps,
                    'heading_rate_rps': point.heading_rate_rps
                }
                points_data.append(point_data)
            
            data = {
                'header': {
                    'stamp': {
                        'sec': msg.header.stamp.sec,
                        'nanosec': msg.header.stamp.nanosec
                    },
                    'frame_id': msg.header.frame_id
                },
                'points': points_data
            }
            
        elif topic_name == 'trajectory':
            # 处理 Trajectory 消息
            points_data = []
            for point in msg.points:
                point_data = {
                    'time_from_start': {
                        'sec': point.time_from_start.sec,
                        'nanosec': point.time_from_start.nanosec
                    },
                    'x': point.pose.position.x,
                    'y': point.pose.position.y,
                    'z': point.pose.position.z,
                    'orientation': {
                        'x': point.pose.orientation.x,
                        'y': point.pose.orientation.y,
                        'z': point.pose.orientation.z,
                        'w': point.pose.orientation.w
                    },
                    'longitudinal_velocity_mps': point.longitudinal_velocity_mps,
                    'lateral_velocity_mps': point.lateral_velocity_mps,
                    'acceleration_mps2': point.acceleration_mps2,
                    'heading_rate_rps': point.heading_rate_rps
                }
                points_data.append(point_data)
            
            data = {
                'header': {
                    'stamp': {
                        'sec': msg.header.stamp.sec,
                        'nanosec': msg.header.stamp.nanosec
                    },
                    'frame_id': msg.header.frame_id
                },
                'points': points_data
            }
            
        elif topic_name == 'transformed_with_covariance':
            # 处理 PoseWithCovarianceStamped 消息
            data = {
                'header': {
                    'stamp': {
                        'sec': msg.header.stamp.sec,
                        'nanosec': msg.header.stamp.nanosec
                    },
                    'frame_id': msg.header.frame_id
                },
                'pose': {
                    'position': {
                        'x': msg.pose.pose.position.x,
                        'y': msg.pose.pose.position.y,
                        'z': msg.pose.pose.position.z
                    },
                    'orientation': {
                        'x': msg.pose.pose.orientation.x,
                        'y': msg.pose.pose.orientation.y,
                        'z': msg.pose.pose.orientation.z,
                        'w': msg.pose.pose.orientation.w
                    },
                    'covariance': list(msg.pose.covariance)
                }
            }
        
        # 创建带有时间戳的日志条目
        log_entry = {
            'log_time': {
                'sec': timestamp.sec,
                'nanosec': timestamp.nanosec
            },
            'topic': topic_name,
            'data': data
        }
        
        return json.dumps(log_entry)
    
    def close(self):
        """关闭所有打开的日志文件"""
        for file in self.log_files.values():
            file.close()


def main(args=None):
    rclpy.init(args=args)
    
    logger = TopicLogger()
    
    try:
        rclpy.spin(logger)
    except KeyboardInterrupt:
        pass
    finally:
        logger.close()
        logger.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()