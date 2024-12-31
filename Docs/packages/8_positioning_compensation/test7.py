#!/usr/bin/env python

import os
os.environ['PYGAME_HIDE_SUPPORT_PROMPT'] = "hide"
import scipy.signal
import sys
sys.path.append("/home/jarvislee-carla/Workspace/Carlas/op_carla/op_bridge/op_bridge/fsm_lab_simulation")
import rclpy
import math
from transforms3d.euler import euler2mat, quat2euler, euler2quat
from vr2sx_transformer import VR2SandBoxTransformer
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseWithCovariance, Pose, Point, Quaternion
from std_msgs.msg import Header
from rclpy.node import Node 
import numpy as np
from rclpy.time import Time
import carla
from matplotlib.path import Path
from shapely.geometry import Polygon 
from shapely.geometry import Point as PPoint

from collections import deque

VEHICLE_NAME = "follow_adtruck"

class TransformedSandBoxCoorPublisherNode(Node):
    def __init__(self, update_rate=0.02):
        super().__init__("transformed_SandBox_coordinate_publisher_node")
        self.publisher = self.create_publisher(
            PoseWithCovarianceStamped, f"/real_world/{VEHICLE_NAME}/transformed_with_covariance", 1
        )
        self.timer = self.create_timer(update_rate, self.publish_transformed_coor)

        self.sandbox_transformer = VR2SandBoxTransformer()
        self.previous_position = None
        self.previous_yaw = None
        self.current_position = None
        self.current_yaw = None
        self.moving_flag = False

        self.yaw_history = deque(maxlen=10)  # 保存最近6个Yaw值

        # 添加位置历史记录队列
        self.position_history = deque(maxlen=10)  # 保存最近6个位置信息

        self.direction = None  # 当前旋转方向：'CW'--Clockwise 或 'CCW'--Counterclockwise
        
        self.similarity_matrix = np.array([[ 3.27650037e+01, -9.68207899e-01,  0.00000000e+00, -5.41650474e+01],
                             [ 9.68207899e-01,  3.27650037e+01,  0.00000000e+00,  7.28873065e+01],
                             [ 0.00000000e+00,  0.00000000e+00,  3.27793059e+01, -5.08468894e-02],
                             [ 0.00000000e+00,  0.00000000e+00,  0.00000000e+00,  1.00000000e+00]])

    def check_threshold(self, position, yaw, threshold=0.005):
        
        position_changed = (
            abs(position[0] - self.previous_position[0]) > threshold or
            abs(position[1] - self.previous_position[1]) > threshold or
            abs(position[2] - self.previous_position[2]) > threshold
        )
        yaw_changed = abs(yaw - self.previous_yaw) > np.radians(0.286)  # 约0.286度

        if position_changed or yaw_changed:
            self.previous_position = position
            self.previous_yaw = yaw
            self.yaw_history.append(yaw)
            #self.get_logger().info(f"yaw_changed is {yaw_changed}")
            self.position_history.append(position)
            return True
        else:
            return False
    def determine_direction(self):
        if len(self.yaw_history) < 2 or len(self.position_history) < 3:
            self.direction = None
            return

        # 航向角变化判定
        yaw_diffs = []
        for i in range(1, len(self.yaw_history)):
            diff = self.yaw_history[i] - self.yaw_history[i-1]
            # 处理绕0度的情况
            if diff > np.pi:
                diff -= 2 * np.pi
            elif diff < -np.pi:
                diff += 2 * np.pi
            yaw_diffs.append(diff)
        avg_yaw_diff = np.mean(yaw_diffs)

        # 判断航向角变化方向
        if avg_yaw_diff > np.radians(2):  # 大于2度，逆时针
            yaw_direction = 'CCW'
        elif avg_yaw_diff < -np.radians(2):  # 小于-2度，顺时针
            yaw_direction = 'CW'
        else:
            yaw_direction = None
        #self.get_logger().info(f"yaw_direction is {yaw_direction}")
        # 位置变化曲率判定（简单的三点法计算曲率方向）
        if len(self.position_history) >= 3:
            p1, p2, p3 = list(self.position_history)[-3:]
            # 计算向量
            v1 = np.array([p2[0] - p1[0], p2[1] - p1[1]])
            v2 = np.array([p3[0] - p2[0], p3[1] - p2[1]])
            # 计算外积来判断曲率方向
            cross = v1[0] * v2[1] - v1[1] * v2[0]
            if cross > 0:
                position_direction = 'CCW'
            elif cross < 0:
                position_direction = 'CW'
            else:
                position_direction = None
        else:
            position_direction = None
        #self.get_logger().info(f"position_direction is {position_direction}")
   

        # 综合判断旋转方向
        if yaw_direction and position_direction:
            if yaw_direction == position_direction:
                self.direction = yaw_direction
            else:
                # 如果两者不一致，可能由于噪声或复杂转向，暂不判定
                self.direction = None
        elif yaw_direction:
            self.direction = yaw_direction
        elif position_direction:
            self.direction = position_direction
        else:
            pass
            #self.direction = None
            #self.get_logger().info("can not make sure")

        # 如果方向确定，并且处于移动状态
        if self.direction is not None and self.moving_flag:
            # 这里只同步到self.direction，不进行发布
            pass  # 可以保留日志记录，如果需要调试，可取消注释
            # self.get_logger().info(f"当前旋转方向: {self.direction}")
    def calculate_B_position(self, A_position, A_yaw, transition_width=0.15):
        """
        根据 A 在 C 中的位置和 yaw 角，计算 B 在 C 中的位置，并通过插值平滑局部额外补偿。
        :param A_position: A 在 C 中的坐标 (x_A, y_A, z_A)
        :param A_yaw: A 在 C 中的 yaw 角（弧度制）
        :param transition_width: 过渡带宽度，用于平滑过渡的范围大小
        :return: B 在 C 中的坐标 (x_B, y_B, z_B)
        """
        x = A_position[0]
        y = A_position[1]
        yaw = A_yaw

        # 全局基础补偿
        global_offset_CCW_A_to_B = np.array([+0.0125, 0, 0])
        global_offset_CW_A_to_B = np.array([-0.0425, 0, 0])
        #global_offset_A_to_B = np.array([0, 0, 0])
        # 区域补偿定义
        region0_1_offset_A_to_B = np.array([-0.0325, 0, 0])  # 区域 0_1 补偿
        region1_offset_A_to_B = np.array([-0.045, 0, 0])  # 区域 1 补偿
        region2_offset_A_to_B = np.array([-0.0155, 0, 0])  # 区域 2 补偿
        region2_1_offset_A_to_B=np.array([+0.010,0,0]) #region2_1
        region2_2_offset_A_to_B=np.array([-0.025,0,0]) #region2_2
        region3_offset_A_to_B = np.array([-0.125, 0, 0])  # 区域 3 补偿
        #region4_offset_A_to_B = np.array([+0.00, 0, 0])  # 区域 4 补偿
        region4_offset_A_to_B = np.array([-0.01, 0, 0])
        region5_offset_A_to_B = np.array([-0.02, 0, 0])
        region5_1_offset_A_to_B = np.array([0, 0, 0])

        # 定义局部区域的坐标范围
        local_region_x_min = -1
        local_region_x_max = 0.66
        local_region_y_min = -3.09
        local_region_y_max = -0.3
        
        polygon_region0 = Polygon([
            (-1, -3.09),  # A
            (-1, -0.3),  # C 
            (0.66, -0.3),   # D
            (0.66, -3.09) # B
        ])

        # 区域 3 和 区域 4 多边形定义
        polygon_region3 = Polygon([
            #[3.942,-1.983,]
            #[3.993,-1.557]
            (3.8408, -2.2162),  # A
            (3.5942, -2.1935),  # C 
            (3.1053, -1.7821),   # D
            (3.8339, -1.4712)  # B
        ])
        polygon_region4 = Polygon([
            (1.1455, -1.0390),
            (0.8830, -0.8656),
            (0.8239, -2.0134),
            (1.3043, -2.5719),
            (1.7373, -2.7863),
            (2.4617, -2.9485),
            (3.0875, -2.8259),
            (3.2439, -2.1172),
            (2.7218, -2.5038),
            (1.9899, -2.1543),
            (1.6561, -2.0650),
            (1.6256, -1.6862),
            (1.4803, -1.0702)
        ])

        polygon_region5 = Polygon([

            ( 4.13, -2.92),
            ( 4.13, -4.56),
            ( 3.34, -4.56),
            #( 3.19, -4.56),
            ( 2.54, -4.56),
            ( 1.54, -4.56),
            ( 1.14, -4.56),
            #(-1.14, -4.56),
            #(-1.14, -3.56),
            ( 1.14, -3.56),
            ( 1.54, -3.56),
            ( 2.54, -3.56),
            #( 3.19, -3.56),
            ( 3.34, -3.56),
            ( 3.34, -2.92)
        
        ])

        # 扩展多边形区域（创建过渡带）
        expanded_polygon_region0 = polygon_region0.buffer(transition_width)
        expanded_polygon_region3 = polygon_region3.buffer(transition_width)
        expanded_polygon_region4 = polygon_region4.buffer(transition_width)
        expanded_polygon_region5 = polygon_region5.buffer(transition_width)
        #expanded_polygon_region0_1 = polygon_region0_1.buffer(transition_width)

        # 判断点是否在局部区域
        # def is_in_local_region(x, y):
        #     """
        #     检查点是否在局部区域范围内
        #     """
        #     return (local_region_x_min <= x <= local_region_x_max) and \
        #         (local_region_y_min <= y <= local_region_y_max)

        # 计算过渡权重的函数
        def calculate_transition_weight(value, min_value, max_value, transition_width):
            """
            计算在过渡带内的权重。返回值在 [0, 1] 之间，0 表示没有局部补偿，1 表示完全应用局部补偿。
            :param value: 当前值
            :param min_value: 区域的最小边界
            :param max_value: 区域的最大边界
            :param transition_width: 过渡带宽度
            """
            if value < min_value:
                return 0.0
            elif value > max_value:
                return 0.0
            elif min_value <= value <= (min_value + transition_width):
                return abs(value - min_value) / transition_width
            elif (max_value - transition_width) <= value <= max_value:
                return abs(max_value - value) / transition_width
            else:
                return 1.0

        # 计算点到多边形边界的距离
        def distance_to_polygon_boundary(point, polygon):
            """
            计算点到多边形边界的距离
            :param point: (x, y) 坐标
            :param polygon: Shapely Polygon 对象
            :return: 距离值
            """
            return polygon.exterior.distance(PPoint(point))
        def interpolate_offset(offset1, offset2, weight):
            return offset1 * (1-weight) + offset2 *weight
        # 分别计算 x 和 y 的过渡权重
        # x_weight = calculate_transition_weight(x, local_region_x_min, local_region_x_max, transition_width)
        # y_weight = calculate_transition_weight(y, local_region_y_min, local_region_y_max, transition_width)
        P0=-2.2022
        P1=-1.3268

        # 区域判断逻辑
        region_offset_A_to_B=global_offset_CW_A_to_B

        if polygon_region0.contains(PPoint(x, y)) and (-130 <= np.degrees(yaw) <= 50 or(50 <= np.degrees(yaw) <= 180) or (-180 <= np.degrees(yaw) <= -130) ): 
            if -130 <= np.degrees(yaw) <= 50:
                self.direction='CW'
                if y>-0.9:
                    region_offset_A_to_B = region0_1_offset_A_to_B
                    if self.moving_flag is True:
                        self.get_logger().info(f"------------区域 0_1: 偏移为 {region_offset_A_to_B}")
                elif -0.9-transition_width<y<-0.9:
                    # 区域 0 和区域0_1之间的过渡带
                    transition_weight = calculate_transition_weight(y, -0.9-transition_width, -0.9, transition_width)
                    
                    region_offset_A_to_B = interpolate_offset(region1_offset_A_to_B,region0_1_offset_A_to_B,transition_weight)
                    if self.moving_flag is True:
                        print(transition_weight)
                        self.get_logger().info(f"--------------区域0_1_to_0 过渡带: 偏移为 {region_offset_A_to_B}")
                elif y<=-0.9-transition_width:
                # 区域 1
                    region_offset_A_to_B = region1_offset_A_to_B
                    if self.moving_flag is True:
                        self.get_logger().info(f"------------区域 0: 偏移为 {region_offset_A_to_B}")

                
            elif (50 <= np.degrees(yaw) <= 180) or (-180 <= np.degrees(yaw) <= -130):
                # 区域 2
                self.direction='CCW'
                if y < P0 - transition_width:
                    # 区域 2 的默认逻辑
                    # yaw_weight = calculate_transition_weight(np.degrees(yaw), -180, -93, transition_width) + \
                    #             calculate_transition_weight(np.degrees(yaw), 88, 180, transition_width)
                    # region_weight = x_weight * y_weight * yaw_weight
                    region_offset_A_to_B = region2_offset_A_to_B
                    if self.moving_flag is True:
                        self.get_logger().info(f"-------------区域 2: 偏移为 {region_offset_A_to_B}")
                elif P0 - transition_width < y < P0:
                    # 区域 2_1 和区域2之间的过渡带
                    transition_weight = calculate_transition_weight(y, P0 - transition_width, P0, transition_width)
                    
                    region_offset_A_to_B = interpolate_offset(region2_offset_A_to_B, region2_1_offset_A_to_B,transition_weight)
                    if self.moving_flag is True:
                        print(transition_weight)
                        self.get_logger().info(f"--------------区域2_to_2_1 过渡带: 偏移为 {region_offset_A_to_B}")
                elif P0 < y < P1 - transition_width:
                    # 区域 2_1
                    # region_weight = calculate_transition_weight(y, -2.2022, -1.0268, transition_width)
                    
                    region_offset_A_to_B = region2_1_offset_A_to_B
                    if self.moving_flag is True:
                        self.get_logger().info(f"----------------区域 2_1:  偏移为 {region_offset_A_to_B}")
                elif P1 - transition_width < y < P1 :
                    # 区域 2_1 和区域 2_2 之间的过渡带
                    transition_weight = calculate_transition_weight(y, P1 - transition_width, P1, transition_width)
                    
                    region_offset_A_to_B = interpolate_offset(region2_1_offset_A_to_B, region2_2_offset_A_to_B, transition_weight)
                    if self.moving_flag is True:
                        print(transition_weight)
                        self.get_logger().info(f"------------区域 2_1 和 2_2 过渡带:  偏移为 {region_offset_A_to_B}")
                elif y>P1:
                    # 区域 2_2
                    region_offset_A_to_B = region2_2_offset_A_to_B
                    if self.moving_flag is True:
                        
                        self.get_logger().info(f"--------------区域 2_2: 偏移为 {region_offset_A_to_B}")
                else:
                    region_offset_A_to_B = global_offset_CCW_A_to_B
                    if self.moving_flag is True:
                        self.get_logger().info(f"from_local-------全局区域: 点不在任何区域内,偏移为 {region_offset_A_to_B}")
        elif expanded_polygon_region0.contains(PPoint(x, y)) and (-130 <= np.degrees(yaw) <= 50):
            # region 0_expanded CW
            self.direction='CW'
            distance = distance_to_polygon_boundary((x, y), polygon_region0)
            transition_weight = max(0.0, min(1.0, 1 - distance / transition_width))
            region_offset_A_to_B = interpolate_offset(global_offset_CW_A_to_B, region0_1_offset_A_to_B,transition_weight)
            if self.moving_flag is True:
                print(transition_weight)
                self.get_logger().info(f"------------区域 0_expanded: 偏移为 {region_offset_A_to_B}")  
        elif expanded_polygon_region0.contains(PPoint(x, y)) and ((50 <= np.degrees(yaw) <= 180) or (-180 <= np.degrees(yaw) <= -130)):
            # region_0_local_2 expanded CCW
            self.direction='CCW'
            # local_2_expanded
            distance = distance_to_polygon_boundary((x, y), polygon_region0)
            transition_weight = max(0.0, min(1.0, 1 - distance / transition_width))
            region_offset_A_to_B = interpolate_offset(global_offset_CCW_A_to_B, region2_offset_A_to_B,transition_weight)
            if self.moving_flag is True:
                print(transition_weight)
                self.get_logger().info(f"------------区域 2_expanded: 偏移为 {region_offset_A_to_B}")
            # else:
            #     region_offset_A_to_B = global_offset_A_to_B
            #     if self.moving_flag is True:
            #         self.get_logger().info(f"frome_expaned------全局区域: 点不在任何区域内,偏移为 {region_offset_A_to_B}")
        elif polygon_region3.contains(PPoint(x, y)) and -130 <= np.degrees(yaw) <= -20:
            # 区域 3 CCW
            self.direction='CCW'
            region_offset_A_to_B = region3_offset_A_to_B
            if self.moving_flag is True:
                self.get_logger().info(f"区域 3: 点在原始多边形内部, ,偏移为 {region_offset_A_to_B}")
        elif expanded_polygon_region3.contains(PPoint(x, y)) and -130 <= np.degrees(yaw) <= -20 :
            # 区域 3 过渡带 CCW
            self.direction='CW'
            distance = distance_to_polygon_boundary((x, y), polygon_region3)
            transition_weight = max(0.0, min(1.0, 1 - distance / transition_width))*0.8
            region_offset_A_to_B = interpolate_offset(global_offset_CCW_A_to_B, region3_offset_A_to_B,transition_weight)
            if self.moving_flag is True:
                print(transition_weight)
                self.get_logger().info(f"区域 3 过渡带: 距离原始边界 {distance:.3f}偏移为 {region_offset_A_to_B}")
        elif polygon_region4.contains(PPoint(x, y)):
            # 区域 4 CW
            self.direction='CW'
            region_offset_A_to_B = region4_offset_A_to_B
            if self.moving_flag is True:
                self.get_logger().info(f"区域 4: 点在原始多边形内部偏移为 {region_offset_A_to_B}")
        elif expanded_polygon_region4.contains(PPoint(x, y)):
            # 区域 4 过渡带
            self.direction='CW'
            distance = distance_to_polygon_boundary((x, y), polygon_region4)
            transition_weight = max(0.0, min(1.0, 1 - distance / transition_width))*0.8
            region_offset_A_to_B = interpolate_offset(global_offset_CCW_A_to_B, region4_offset_A_to_B,transition_weight)
            if self.moving_flag is True:
                print(transition_weight)
                self.get_logger().info(f"区域 4 过渡带: 距离原始边界 {distance:.3f}, 偏移为 {region_offset_A_to_B}")
        elif polygon_region5.contains(PPoint(x, y)) and -115< np.degrees(yaw)< 25:
            # 区域 5
            self.direction='CCW'
            region_offset_A_to_B = region5_offset_A_to_B
            if self.moving_flag is True:
                self.get_logger().info(f"区域 5: 点在原始多边形内部偏移为 {region_offset_A_to_B}")
        elif polygon_region5.contains(PPoint(x, y)) and 60< np.degrees(yaw)< 120:
            self.direction='CW'
            region_offset_A_to_B = region5_1_offset_A_to_B
            if self.moving_flag is True:
                self.get_logger().info(f"区域 5: 点在原始多边形内部偏移为 {region_offset_A_to_B}")
        elif expanded_polygon_region5.contains(PPoint(x, y)) and 60< np.degrees(yaw)< 120:
            # 区域 5_1 过渡带
            self.direction='CW'
            #self.point_state=determine_transition_state(polygon_region0,expanded_polygon_region0)
            #transition_weight=calculate_transition_weight_with_state((x,y),polygon_region0,transition_width)*0.8
            distance = distance_to_polygon_boundary((x, y), polygon_region5)
            transition_weight = max(0.0, min(1.0, 1 - distance / transition_width))*0.8
            region_offset_A_to_B = interpolate_offset(global_offset_CW_A_to_B, region5_1_offset_A_to_B,transition_weight)
            if self.moving_flag is True:
                print(transition_weight)
                self.get_logger().info(f"区域 5 过渡带: 距离原始边界 {distance:.3f}, 偏移为 {region_offset_A_to_B}")
        elif expanded_polygon_region5.contains(PPoint(x, y)) and -50< np.degrees(yaw)< 25:
            # 区域 5 过渡带
            self.direction='CCW'
            #self.point_state=determine_transition_state(polygon_region0,expanded_polygon_region0)
            #transition_weight=calculate_transition_weight_with_state((x,y),polygon_region0,transition_width)*0.8
            distance = distance_to_polygon_boundary((x, y), polygon_region5)
            transition_weight = max(0.0, min(1.0, 1 - distance / transition_width))*0.8
            region_offset_A_to_B = interpolate_offset(global_offset_CCW_A_to_B, region5_offset_A_to_B,transition_weight)
            if self.moving_flag is True:
                print(transition_weight)
                self.get_logger().info(f"区域 5 过渡带: 距离原始边界 {distance:.3f}, 偏移为 {region_offset_A_to_B}")
        elif expanded_polygon_region5.contains(PPoint(x, y)) and -115< np.degrees(yaw)< -50:
            # 区域 5 过渡带 2 
            self.direction='CCW'
            #self.point_state=determine_transition_state(polygon_region0,expanded_polygon_region0)
            #transition_weight=calculate_transition_weight_with_state((x,y),polygon_region0,transition_width)*0.8
            distance = distance_to_polygon_boundary((x, y), polygon_region5)
            transition_weight = max(0.0, min(1.0, 1 - distance / transition_width))*0.8
            region_offset_A_to_B = interpolate_offset(region5_offset_A_to_B,global_offset_CCW_A_to_B,transition_weight)
            if self.moving_flag is True:
                print(transition_weight)
                self.get_logger().info(f"区域 5 过渡带: 距离原始边界 {distance:.3f}, 偏移为 {region_offset_A_to_B}")
        else:
            # 全局区域
            if self.direction ==  'CCW':
                region_offset_A_to_B = global_offset_CCW_A_to_B
            elif self.direction == 'CW':
                region_offset_A_to_B = global_offset_CW_A_to_B
            
            if self.moving_flag is True:
                self.get_logger().info(f"----------全局区域: 点不在任何区域内,偏移为 {region_offset_A_to_B}, deriction is {self.direction}")

        # 最终补偿融合
        # final_offset_A_to_B = region_offset_A_to_B
        # if self.moving_flag is True:
        #     self.get_logger().info(f"最终补偿值: {final_offset_A_to_B}")

        # 旋转矩阵 (绕 Z 轴旋转 yaw 角)
        rotation_matrix = np.array([
            [np.cos(A_yaw), -np.sin(A_yaw), 0],
            [np.sin(A_yaw),  np.cos(A_yaw), 0],
            [0, 0, 1]
        ])

        # 计算 A 相对于 B 的偏移
        offset_in_C = np.dot(rotation_matrix, region_offset_A_to_B)

        # 计算 B 在 C 中的坐标
        B_position = A_position - offset_in_C
        return B_position

    def transform_coordinates_from_sandbox2carla(self,transformed_position__,transformed_yaw__):
        
        #pose.position.z=0.0016
        if self.moving_flag == True:
            self.get_logger().info(f"this is orginal pose: x: {transformed_position__[0]}, y: {transformed_position__[2]}, z: 0.0016,yaw: {math.degrees(transformed_yaw__)},")
        #x=transformed_position[0], y=transformed_position[2], z=transformed_position[1]
        # 注意: Carla的y轴和ROS的y轴方向相反, 因此需要对y轴进行翻转
        B_position_offseted=self.calculate_B_position(np.array([transformed_position__[0],transformed_position__[2],0.0017]),transformed_yaw__)

        original_point=np.array([B_position_offseted[0], B_position_offseted[1], B_position_offseted[2]])
        #original_point=np.array([transformed_position__[0], transformed_position__[2], 0.0016])

        homogeneous_point = np.append(original_point, 1)
        transformed_homogeneous_point = self.similarity_matrix @ homogeneous_point
        transformed_point = transformed_homogeneous_point[:3] / transformed_homogeneous_point[3]

        #yaw=transformed_yaw
        _transformed_yaw=math.radians(-(math.degrees(transformed_yaw__)-90))#-90#90-transformed_yaw__#+15#-90
        transformed_point[1]=-transformed_point[1]
        if self.moving_flag==True:
            self.get_logger().info(f"____transformed___pose x: {transformed_point[0]}, y: {transformed_point[1]}, z:{transformed_point[2]},yaw: {math.degrees(_transformed_yaw)},")
        #new_pose=carla.Transform(carla.Location(x, y, z), carla.Rotation(0, yaw,0 ))
        return transformed_point,_transformed_yaw    

    def RPY2quaternion(self, roll, pitch, yaw):
        """
        Transform Euler angles to quaternion.
        Parameters:
            roll, pitch, yaw
        Return: (x, y, z, w)
        """
        # roll = math.radians(roll)
        # pitch = math.radians(pitch)
        # yaw = math.radians(yaw)

        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        w = cr * cp * cy + sr * sp * sy
        x = sr * cp * cy - cr * sp * sy
        y = cr * sp * cy + sr * cp * sy
        z = cr * cp * sy - sr * sp * cy

        return x, y, z, w    

    # def check_threshold(self, position, yaw, threshold=0.005):
    #     """
    #     Check if the position and yaw are within a threshold.
    #     """
    #     if abs(position[0] - self.previous_position[0]) > threshold or abs(position[1] - self.previous_position[1]) > threshold or abs(position[2] - self.previous_position[2]) > threshold:
    #         self.previous_position = position
    #         self.previous_yaw = yaw
    #         return True
    #     elif abs(yaw - self.previous_yaw) > np.degrees(threshold):
    #         self.previous_yaw = yaw
    #         return True
    #     else:
    #         return False

    def publish_transformed_coor(self):
        
        sdb_transformed_position, sdb_transformed_yaw = self.sandbox_transformer.get_transformed_coor()
        #transformed_position_[1]-=0.05
        
        if self.previous_position is None or self.previous_yaw is None:
            self.previous_position = sdb_transformed_position
            self.previous_yaw = sdb_transformed_yaw
        elif self.check_threshold(sdb_transformed_position,sdb_transformed_yaw):
            self.moving_flag=True
            self.previous_position = sdb_transformed_position
            self.previous_yaw = sdb_transformed_yaw
        else:
            self.moving_flag=False

        #self.determine_direction()
        
        transformed_position, transformed_yaw=self.transform_coordinates_from_sandbox2carla(sdb_transformed_position, sdb_transformed_yaw)
        # Set up the PoseWithCovarianceStamped message
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = "real_world"

        pose = Pose()
        pose.position = Point(x=transformed_position[0], y=transformed_position[1], z=transformed_position[2])
        pose.orientation = Quaternion(
            x=self.RPY2quaternion(0, 0, transformed_yaw)[0],
            y=self.RPY2quaternion(0, 0, transformed_yaw)[1],
            z=self.RPY2quaternion(0, 0, transformed_yaw)[2],
            w=self.RPY2quaternion(0, 0, transformed_yaw)[3]
        )

        # Create a covariance matrix for pose (6x6 matrix flattened to 36 elements)
        covariance = [0.0] * 36
        covariance[0] = 0.01  # variance in x
        covariance[7] = 0.01  # variance in y
        covariance[14] = 0.01 # variance in z
        covariance[21] = 0.01 # variance in roll
        covariance[28] = 0.01 # variance in pitch
        covariance[35] = 0.01 # variance in yaw

        # Create PoseWithCovariance object
        pose_with_cov = PoseWithCovariance(pose=pose, covariance=covariance)

        # Create PoseWithCovarianceStamped message
        msg = PoseWithCovarianceStamped(header=header, pose=pose_with_cov)

        # Publish the message
        self.publisher.publish(msg)
        # self.get_logger().info(
        #     f"\n Transformed Published! x is {transformed_position[0]}, y is {transformed_position[1]}, z is {transformed_position[2]}, yaw is {math.degrees(transformed_yaw)}"
        # )
        # self.get_logger().info(
        #     f"\n Transformed Published with covariance! [{msg.pose.pose.position.x}, {msg.pose.pose.position.y}, {msg.pose.pose.position.z}, {msg.pose.pose.orientation.w}, {msg.pose.pose.orientation.x}, {msg.pose.pose.orientation.y}, {msg.pose.pose.orientation.z}]"
        # )

        # 更新最后一次发布的时间



if __name__ == "__main__":
    rclpy.init(args=None)

    # Create the node
    transformed_sandbox_coor_publisher_node= TransformedSandBoxCoorPublisherNode()

    # Spin the node
    rclpy.spin(transformed_sandbox_coor_publisher_node)

    rclpy.shutdown()