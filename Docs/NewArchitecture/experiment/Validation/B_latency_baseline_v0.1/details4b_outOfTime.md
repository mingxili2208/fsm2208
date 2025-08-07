好的，这是一个非常出色的实验设计。它将抽象的系统延迟与可测量的物理误差直接挂钩，精准地回答了“平台自身的误差基线是多少”这一核心问题。这对于任何严谨的仿真测试都是至关重要的一步。

下面，我为您提供一个针对实验B的完整、详细的实施方案，包含实验设计、代码、步骤和说明。

---

## 实验B：延迟特性及其对基线跟踪误差的影响分析 - 完整实施方案

### 1. 实验设计与集成策略

#### 1.1. 核心理念：逻辑解耦，操作耦合

本实验的目标（测量系统延迟、量化物理误差）与实验A（测量传感器延迟、验证保真度）有本质区别。因此，为了保持代码的模块化和清晰性，我们将为实验B设计一套全新的、独立的节点。这是一种**逻辑上的解耦**。

然而，在执行实验时，我们完全可以在一次驾驶任务中，**同时运行**实验A和实验B所需的所有数据记录节点。这极大地提升了效率。这是一种**操作上的耦合**。

**最终决策：我们将为实验B设计一套独立的节点，但在使用说明中，将指导您与实验A的节点并行运行，一次性完成所有数据采集。**

#### 1.2. 关键时间戳捕获策略 (`T_actual` -> `T_actuate`)

这是整个设计的技术核心。为了对被测的ADS（如Autoware）保持非侵入性，我们将采用一个中央日志节点，通过订阅不同的话题来“拼凑”出完整的延迟时间链。

* **`T_actual` (物理位姿捕获时间)**: 这个时间戳直接来源于实验A.2中 `ground_truth_recorder.py` 记录的真值位姿。我们将在数据分析阶段将两份数据关联起来，这是最高效且最准确的方式。
* **`T_percept` (感知信息接收时间)**: 这是ADS规划器接收其主要输入（如NDT定位结果）的时间。通常是 `/localization/kinematic_state` 这类话题的`header.stamp`。
* **`T_plan` (规划指令发出时间)**: 这是规划器发出控制指令（如轨迹）的时间。通常是 `/planning/trajectory` 这类话题的`header.stamp`。
* **`T_actuate` (控制指令执行时间)**: 这是最 tricky 的一个点。我们将其定义为我们的 `CarlaUpdateVehicleHandler.py` 节点**接收到控制指令并即将应用到CARLA车辆的那一刻**。这需要对该节点进行一次微小但关键的“埋点”或“插桩”。

### 2. 第一部分：延迟测量 - 实施方案

我们需要一个全新的日志节点，并对您现有的CARLA控制节点进行微小修改。

#### 代码1: `latency_impact_logger.py` (实验B的中央日志节点 - 新建)

这个节点是延迟测量的核心，它监听三个数据流并进行关联。

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import csv
import os
import datetime
from collections import deque

# 导入所需的消息类型 - 请根据您的ADS系统进行修改
from geometry_msgs.msg import PoseWithCovarianceStamped  # 用于 T_percept
from autoware_auto_planning_msgs.msg import Trajectory   # 用于 T_plan
from sensor_msgs.msg import TimeReference                # 用于 (T_plan, T_actuate) 对

class LatencyImpactLogger(Node):
    """
    为实验B记录关键时间戳 (T_percept, T_plan, T_actuate)。
    通过关联这些时间戳，建立完整的延迟分析时间链。
    """
    
    def __init__(self):
        super().__init__('latency_impact_logger')
        
        # --- 配置 ---
        # !!! 请根据您的系统修改这些话题名称 !!!
        self.perception_topic = "/localization/kinematic_state"
        self.plan_topic = "/planning/trajectory"
        self.actuation_topic = "/fsm_sandbox/timing/t_plan_t_actuate"
        
        # 用于关联数据的数据结构
        self.pending_plans = {}  # {t_plan_ns: t_percept_ns}
        
        # 设置CSV日志
        self.setup_csv_logging()
        
        # 创建订阅者
        self.perception_sub = self.create_subscription(
            PoseWithCovarianceStamped, self.perception_topic, self.perception_callback, 10)
        
        self.plan_sub = self.create_subscription(
            Trajectory, self.plan_topic, self.plan_callback, 10)
            
        self.actuation_sub = self.create_subscription(
            TimeReference, self.actuation_topic, self.actuation_callback, 10)
        
        self.get_logger().info("延迟影响日志节点 (实验B) 已启动。")
        self.get_logger().info(f"监听感知话题: {self.perception_topic}")
        self.get_logger().info(f"监听规划话题: {self.plan_topic}")
        self.get_logger().info(f"监听执行时间话题: {self.actuation_topic}")

    def setup_csv_logging(self):
        """设置用于记录延迟数据的CSV文件。"""
        timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        self.csv_filename = f'exp_b_latency_log_{timestamp}.csv'
        self.csv_file = open(self.csv_filename, 'w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        # 注意：T_actual 将在后处理阶段从另一个文件合并进来
        self.csv_writer.writerow(['t_percept_ns', 't_plan_ns', 't_actuate_ns'])
        self.csv_file.flush()
        self.get_logger().info(f"延迟数据记录至: {os.path.abspath(self.csv_filename)}")

    def to_nanoseconds(self, stamp):
        return stamp.sec * 1_000_000_000 + stamp.nanosec

    def perception_callback(self, msg: PoseWithCovarianceStamped):
        """
        接收感知结果 (T_percept)。
        我们将使用其header.stamp作为T_percept。
        """
        self.last_t_percept_ns = self.to_nanoseconds(msg.header.stamp)

    def plan_callback(self, msg: Trajectory):
        """
        接收规划器输出 (T_plan)。
        我们将T_plan与最近的T_percept关联起来。
        """
        if not hasattr(self, 'last_t_percept_ns'):
            return
            
        t_plan_ns = self.to_nanoseconds(msg.header.stamp)
        
        # 假设规划是基于最近一次的感知结果
        self.pending_plans[t_plan_ns] = self.last_t_percept_ns

    def actuation_callback(self, msg: TimeReference):
        """接收 (T_plan, T_actuate) 对，并完成一次完整的记录。"""
        t_plan_ns = self.to_nanoseconds(msg.header.stamp)
        t_actuate_ns = self.to_nanoseconds(msg.time_ref)
        
        if t_plan_ns in self.pending_plans:
            t_percept_ns = self.pending_plans.pop(t_plan_ns)
            
            # 记录我们拥有的时间链
            self.csv_writer.writerow([t_percept_ns, t_plan_ns, t_actuate_ns])
            self.csv_file.flush()
            self.get_logger().debug(f"记录完整时间链: T_plan={t_plan_ns}")

    def destroy_node(self):
        if hasattr(self, 'csv_file') and self.csv_file:
            self.csv_file.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    logger_node = LatencyImpactLogger()
    try:
        rclpy.spin(logger_node)
    except KeyboardInterrupt:
        pass
    finally:
        logger_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

#### 代码2: `CarlaUpdateVehicleHandler.py` 的修改

您需要在此节点中添加一个发布者，每当它处理来自规划器的命令时，就发布`(T_plan, T_actuate)`时间对。

```python
# 在 CarlaUpdateVehicleHandler.py 中

# 添加到导入部分
from sensor_msgs.msg import TimeReference
from autoware_auto_planning_msgs.msg import Trajectory # 或者您实际使用的控制指令类型

class CarlaUpdateVehicleHandler(Node):
    def __init__(self):
        # ... 您现有的初始化代码 ...
        
        # 为实验B新增的发布者
        self.actuation_timing_pub = self.create_publisher(
            TimeReference, '/fsm_sandbox/timing/t_plan_t_actuate', 10)
            
        # 修改您对规划器输出的订阅
        self.control_sub = self.create_subscription(
            Trajectory,
            "/planning/trajectory", # 您规划器输出的话题
            self.control_callback,
            10)

    def control_callback(self, msg: Trajectory):
        # T_plan 是接收到的消息头中的时间戳
        t_plan_stamp = msg.header.stamp
        
        # T_actuate 是当前时间，即将在模拟中应用该指令的时刻
        t_actuate_stamp = self.get_clock().now()
        
        # 发布时间信息给日志节点
        timing_msg = TimeReference()
        timing_msg.header.stamp = t_plan_stamp
        timing_msg.time_ref = t_actuate_stamp.to_msg()
        self.actuation_timing_pub.publish(timing_msg)
        
        # --- 在这里，执行您正常的控制逻辑 ---
        # (例如：解析轨迹，找到目标点，应用控制到CARLA车辆)
        # ... 您控制车辆的逻辑 ...
```

### 3. 第二部分：跟踪误差测量 - 实施方案

这需要一个在“直线行驶”测试期间运行的、全新的专用节点。

#### 代码3: `tracking_error_analyzer.py` (实验B的新建节点)

该节点用于实时计算跟踪误差。

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import csv
import os
import datetime
import numpy as np
from geometry_msgs.msg import PoseWithCovarianceStamped
from shapely.geometry import LineString, Point

class TrackingErrorAnalyzer(Node):
    """
    根据预定义的路径，计算并记录纵向和横向跟踪误差。
    """
    def __init__(self):
        super().__init__('tracking_error_analyzer')
        
        # --- 配置 ---
        # 定义用于测试的、简单的直线路径
        self.reference_path = LineString([(0, 0), (200, 0)]) # 在x轴上200米长的直线
        self.target_velocity = 0.5  # 目标速度 (m/s)
        
        # 车辆当前位姿的话题
        self.pose_topic = "/localization/kinematic_state"
        
        # 设置CSV日志
        self.setup_csv_logging()
        
        # 状态变量
        self.start_time_ns = None
        
        # 订阅者
        self.pose_sub = self.create_subscription(
            PoseWithCovarianceStamped, self.pose_topic, self.pose_callback, 10)
            
        self.get_logger().info("跟踪误差分析节点已启动。")
        self.get_logger().info(f"目标速度: {self.target_velocity} m/s")

    def setup_csv_logging(self):
        """设置用于记录跟踪误差数据的CSV文件。"""
        timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        self.csv_filename = f'exp_b_tracking_error_log_{timestamp}.csv'
        self.csv_file = open(self.csv_filename, 'w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(['timestamp_ns', 's_actual', 'd_actual', 's_target', 'error_longitudinal'])
        self.csv_file.flush()
        self.get_logger().info(f"误差数据记录至: {os.path.abspath(self.csv_filename)}")

    def pose_callback(self, msg: PoseWithCovarianceStamped):
        """为每个接收到的位姿计算跟踪误差。"""
        timestamp_ns = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        
        if self.start_time_ns is None:
            self.start_time_ns = timestamp_ns
            
        # 车辆当前位置
        vehicle_point = Point(msg.pose.pose.position.x, msg.pose.pose.position.y)
        
        # 计算Frenet坐标 (s, d)
        # s: 沿路径的投影距离 (纵向)
        s_actual = self.reference_path.project(vehicle_point)
        # d: 与路径的垂直距离 (横向)
        d_actual = self.reference_path.distance(vehicle_point)
        
        # 根据时间和恒定速度计算目标纵向位置
        elapsed_time_s = (timestamp_ns - self.start_time_ns) / 1e9
        s_target = self.target_velocity * elapsed_time_s
        
        # 计算纵向误差
        error_longitudinal = s_actual - s_target
        
        # 记录数据
        self.csv_writer.writerow([timestamp_ns, s_actual, d_actual, s_target, error_longitudinal])
        self.csv_file.flush()

    def destroy_node(self):
        if hasattr(self, 'csv_file') and self.csv_file:
            self.csv_file.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    analyzer_node = TrackingErrorAnalyzer()
    try:
        rclpy.spin(analyzer_node)
    except KeyboardInterrupt:
        pass
    finally:
        analyzer_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### 4. 数据分析与可视化

这个最终脚本将处理上面生成的两个CSV文件，并生成所有要求的表格和图表。

#### 代码4: `analyze_experiment_b.py` (新建的分析脚本)

```python
#!/usr/bin/env python3

import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import seaborn as sns
import argparse
import os

class ExperimentBAnalyzer:
    def __init__(self, latency_csv, error_csv, ground_truth_csv, target_velocity):
        self.latency_df = pd.read_csv(latency_csv)
        self.error_df = pd.read_csv(error_csv)
        self.target_velocity = target_velocity
        
        # 合并真值数据以获得 T_actual
        try:
            gt_df = pd.read_csv(ground_truth_csv)
            # 为了获取T_actual, 我们将t_percept与最近的真值时间戳对齐
            self.latency_df = pd.merge_asof(
                self.latency_df.sort_values('t_percept_ns'),
                gt_df[['timestamp_ns']].rename(columns={'timestamp_ns': 't_actual_ns'}),
                left_on='t_percept_ns',
                right_on='t_actual_ns',
                direction='backward', # 寻找t_percept之前的最后一个t_actual
                tolerance=100_000_000 # 100ms容忍窗口
            )
            self.latency_df.dropna(subset=['t_actual_ns'], inplace=True)
            print("成功合并真值数据以获取 T_actual。")
        except FileNotFoundError:
            print("警告: 未找到 ground_truth_poses.csv。T_actual 将被估算。")
            # 估算 T_actual, 假设它比 T_percept 早一个固定的时间（感知模块处理时间）
            self.latency_df['t_actual_ns'] = self.latency_df['t_percept_ns'] - 30_000_000 # 假设30ms

    def analyze_latency(self):
        """执行第一部分：延迟分析"""
        df = self.latency_df
        df['delta_IA'] = (df['t_percept_ns'] - df['t_actual_ns']) / 1e6      # 信息年龄 (ms)
        df['delta_planner'] = (df['t_plan_ns'] - df['t_percept_ns']) / 1e6   # ADS内部处理延迟 (ms)
        df['delta_D2A'] = (df['t_actuate_ns'] - df['t_plan_ns']) / 1e6        # 决策到执行延迟 (ms)
        df['delta_total'] = df['delta_IA'] + df['delta_planner'] + df['delta_D2A']
        
        # 过滤掉明显的异常值
        df = df[df['delta_total'] < df['delta_total'].quantile(0.995)]
        self.latency_df_processed = df

        print("\n--- B3: 关键延迟特性分析 (单位: 毫秒) ---")
        stats = df[['delta_IA', 'delta_planner', 'delta_D2A', 'delta_total']].describe(percentiles=[.5, .95, .99])
        print(stats.round(4))
        
        # 绘制堆叠条形图
        plt.style.use('seaborn-v0_8-whitegrid')
        means = df[['delta_IA', 'delta_planner', 'delta_D2A']].mean()
        proportions = means / means.sum()
        
        fig, ax = plt.subplots(figsize=(12, 3))
        ax.barh('总延迟构成', proportions['delta_IA'], label=f"信息年龄 ({proportions['delta_IA']:.1f}ms, {proportions['delta_IA']/means.sum():.1%})")
        ax.barh('总延迟构成', proportions['delta_planner'], left=proportions['delta_IA'], label=f"规划延迟 ({proportions['delta_planner']:.1f}ms, {proportions['delta_planner']/means.sum():.1%})")
        ax.barh('总延迟构成', proportions['delta_D2A'], left=proportions['delta_IA']+proportions['delta_planner'], label=f"执行延迟 ({proportions['delta_D2A']:.1f}ms, {proportions['delta_D2A']/means.sum():.1%})")
        
        ax.set_xlabel("占总延迟的比例")
        ax.set_title("端到端总延迟的构成分析")
        ax.legend(loc='center left', bbox_to_anchor=(1, 0.5))
        plt.tight_layout()
        fig.savefig("exp_b_latency_composition.png")
        print("\n延迟构成图已保存至: exp_b_latency_composition.png")

    def analyze_error_impact(self):
        """执行第二部分：延迟对跟踪误差的影响分析"""
        # 计算理论误差
        mean_total_latency_s = self.latency_df_processed['delta_total'].mean() / 1000.0
        error_long_theory = self.target_velocity * mean_total_latency_s
        
        # 获取实际误差的均值
        mean_actual_error = self.error_df['error_longitudinal'].mean()
        
        print("\n--- B4: 延迟对基线跟踪误差的量化影响分析 ---")
        print(f"平均总延迟: {mean_total_latency_s * 1000:.2f} ms")
        print(f"测试目标速度: {self.target_velocity} m/s")
        print(f"理论纵向误差 (v * ΔT): {error_long_theory:.4f} 米")
        print(f"实测平均纵向误差: {mean_actual_error:.4f} 米")
        
        # 绘制时间序列对比图
        fig, ax = plt.subplots(figsize=(14, 7))
        time_sec = (self.error_df['timestamp_ns'] - self.error_df['timestamp_ns'].iloc[0]) / 1e9
        ax.plot(time_sec, self.error_df['error_longitudinal'], label='实际测量的跟踪误差', alpha=0.7)
        ax.axhline(y=error_long_theory, color='r', linestyle='--', linewidth=2.5, label=f'理论误差基线 ({error_long_theory:.4f}m)')
        ax.axhline(y=mean_actual_error, color='g', linestyle=':', linewidth=2.5, label=f'实测误差均值 ({mean_actual_error:.4f}m)')
        
        ax.set_xlabel('时间 (秒)')
        ax.set_ylabel('纵向跟踪误差 (米)')
        ax.set_title('实测跟踪误差 vs. 理论误差基线')
        ax.legend()
        ax.grid(True)
        plt.tight_layout()
        fig.savefig("exp_b_error_comparison.png")
        print("误差对比图已保存至: exp_b_error_comparison.png")

def main():
    parser = argparse.ArgumentParser(description="分析实验B的数据。")
    parser.add_argument('latency_csv', help="实验B生成的延迟日志CSV文件路径。")
    parser.add_argument('error_csv', help="实验B生成的跟踪误差日志CSV文件路径。")
    parser.add_argument('ground_truth_csv', help="实验A.2生成的真值位姿CSV文件路径。")
    parser.add_argument('--velocity', type=float, default=0.5, help="直线测试中使用的目标速度。")
    args = parser.parse_args()
    
    analyzer = ExperimentBAnalyzer(args.latency_csv, args.error_csv, args.ground_truth_csv, args.velocity)
    analyzer.analyze_latency()
    analyzer.analyze_error_impact()
    
    plt.show()

if __name__ == '__main__':
    main()
```

### 5. 详细使用指南

#### **阶段一：在线数据收集**

1. **准备节点**: 将所有新建/修改的Python脚本放入您的ROS 2工作空间并编译。
2. **启动记录器**: 在不同的终端中，启动所有需要的数据记录器。
    * `ros2 run your_pkg latency_impact_logger.py` (用于实验B延迟)
    * `ros2 run your_pkg tracking_error_analyzer.py` (用于实验B误差)
    * **强烈建议**: 同时运行实验A的记录器。
        * `ros2 run your_pkg log_recorder_node.py` (用于A.1时效性)
        * `ros2 run your_pkg ground_truth_recorder.py` (用于A.2保真度 和 B.1的T_actual)
3. **启动系统**: 启动您的完整FSM Sandbox环境 (CARLA, Autoware等)。
4. **执行任务**:
    * 首先，让车辆在一个通用的点到点导航任务中行驶几分钟，为延迟分析收集丰富的数据。
    * 然后，指令车辆沿预定义的直线路径，以配置的恒定速度（0.5 m/s）行驶，为误差分析收集数据。
5. **停止**: 停止所有记录器和仿真。您现在应该拥有 `exp_b_latency_log_...csv` 和 `exp_b_tracking_error_log_...csv` 文件，以及实验A的日志文件。

#### **阶段二：数据分析**

1. **准备文件**: 将分析脚本 (`analyze_experiment_b.py`) 与所有生成的CSV文件放在同一个目录下。**确保 `ground_truth_poses.csv` 也存在**，以便进行最准确的分析。
2. **运行分析**:

    ```bash
    # 请替换为您的实际文件名
    python analyze_experiment_b.py exp_b_latency_log_....csv exp_b_tracking_error_log_....csv ground_truth_poses_....csv --velocity 0.5
    ```

3. **审查结果**:
    * 终端将打印出延迟统计表和误差对比摘要。
    * 将弹出两个图表窗口，并保存为PNG文件：
        * `exp_b_latency_composition.png` (延迟构成图)
        * `exp_b_error_comparison.png` (误差对比图)

这个全面的方案为您提供了执行实验B所需的全部工具和清晰的工作流程，能够严谨地将抽象的系统延迟与可量化的物理误差联系起来。