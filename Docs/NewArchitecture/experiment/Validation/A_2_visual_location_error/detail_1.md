*Thinking...*

> **Assessing the Experiment**
> 
> I'm currently dissecting the user's request, aiming to solidify the validation strategy for Experiment A.2. The primary objective is to ascertain the fidelity of the FSM Sandbox, focusing on how well the virtual LiDAR data mirrors real-world scenarios. My core strategy revolves around CAR's virtual LiDAR generation capabilities.
> 
> 
> **Mapping the Data Flow**
> 
> I've mapped the data flow: I'll collect ground truth poses and corresponding virtual LiDAR data during an online run. Offline, the recorded LiDAR data will be fed to a standalone NDT module to generate poses. I'll then compare the offline NDT pose with the ground truth, focusing on error metrics and visualizing the results.
> 
> 
> **Formulating a Plan**
> 
> I'm now piecing together the specific code components needed. The primary task involves creating `offline_ndt_recorder.py`, which will record the NDT output during offline bag playback, along with the `analyze_fidelity_data.py` script, that generates the data validation output. I am considering the exact data streams and the best way to capture them to minimize processing later.
> 
> 
> **Detailing the Implementation**
> 
> I'm now architecting the "Implementation Steps" section. I'm breaking down the online data collection phase, specifying the `ros2 bag record` command and the exact topics to capture for both experiments. The clarity of the data collection phase is my focus right now, to simplify its practical application for the user.
> 
> 
> **Structuring the Report**
> 
> I'm outlining the document structure, which now includes a "Plan & Rationale" section to reinforce the fidelity goal and the crucial point about concurrent data collection. Implementation steps will be carefully organized. Data collection will be clearly described with concrete `ros2 bag record` command examples.
> 
> 
> **Finalizing Code Components**
> 
> I'm deep in the implementation of the code, now designing the key scripts. I've solidified the need for `ground_truth_recorder.py`, which will streamline data analysis, and I'm focusing on the structure of `analyze_fidelity_data.py`. Ensuring the handling of input CSVs and the generation of clear error metrics are now the priority, and that this script provides the requested statistics and visualization output.
> 
> 
> **Developing Code Solutions**
> 
> I'm currently focused on coding the crucial analysis script, `analyze_fidelity_data.py`. My design approach incorporates `argparse` for user input, data alignment using `pd.merge_asof` with a defined tolerance, and calculation of translation and rotation errors. I'm also considering how to best handle yaw angle wrapping for accurate error analysis.
> 
> 
> **Defining Code Requirements**
> 
> I'm now diving deep into the script designs, clarifying the inputs and outputs, especially for `analyze_fidelity_data.py`. I've identified the essential use of `argparse`, the necessity of data alignment with `pd.merge_asof` using a tolerance, and accurate error calculations including handling yaw angle wrapping for precise results.
> 
> 
> **Developing the Analysis Script**
> 
> I'm now focused on the design of the central script, `analyze_fidelity_data.py`. The inputs will be the ground truth pose and the offline NDT poses from separate CSV files. I'll use `argparse` for parameter handling and will leverage `pd.merge_asof` to align the time series data. The output will be both statistical metrics and clear visualizations.
> 
> 
好的，这是一个非常棒的后续实验设计，它直击**保真度 (Fidelity)** 的核心。您的思路完全正确：如果虚拟LiDAR数据是“高保真”的，那么一个标准的定位算法（如NDT）应该能从中反解出与原始真值几乎完全一致的位姿。

下面，我为您提供一个针对子实验A.2的完整实施方案，它包含了实验计划、步骤、所需的全套代码、设计思路以及详细的使用说明。

---

## 实验A.2: 保真度与自洽性验证 - 完整实施方案

### 1. 实验计划与核心思想

#### 1.1. 目标
本实验旨在定量验证FSM Sandbox生成的虚拟LiDAR数据的**保真度**。我们将通过一个闭环测试来证明，该虚拟数据质量足够高，足以让一个标准的NDT定位模块精确地恢复出车辆的原始真值位姿。

#### 1.2. 核心方法：在线记录，离线验证
我们采用一种在机器人和自动驾驶测试中非常标准且可靠的方法：

1.  **在线数据收集**：在FSM Sandbox正常运行时，同步记录两个核心数据流：
    *   **真值位姿 (`actual_pose`)**: 这是驱动CARLA虚拟车辆运动的源头。
    *   **虚拟点云 (`virtual_lidar`)**: 这是CARLA根据真值位姿渲染生成的结果。

2.  **离线闭环验证**：在一个独立的、干净的环境中，将记录的虚拟点云数据回放给一个与在线环境配置完全相同的NDT模块。然后，将NDT的定位输出与原始的真值位姿进行比对。

这种方法能完美地隔离变量，确保我们评估的仅仅是**虚拟点云数据本身的质量**，排除了在线运行时可能存在的其他干扰（如网络抖动、系统负载等）。

#### 1.3. 与实验A.1同步进行
**是的，您可以（也应该）将本实验的数据收集部分与上一个时效性实验（A.1）完全同步进行。** 这是一个巨大的效率提升。

在您运行A.1时，只需在`ros2 bag record`命令中额外添加本实验所需的两个话题即可。一次驾驶，完成两项实验的数据采集。

### 2. 实施步骤

整个实验分为三个阶段：在线收集、离线处理、分析出图。

#### **阶段一：在线数据收集 (与A.1同步)**

1.  **启动数据记录节点**：除了启动A.1的`log_recorder_node.py`外，我们还需要一个简单的节点来将真值位姿实时保存为CSV格式，以便后续分析。
2.  **启动ROS Bag录制**：使用`ros2 bag record`命令同时录制虚拟LiDAR数据和真值位姿话题。
3.  **执行驾驶任务**：让车辆在场景中行驶数分钟，确保覆盖不同的位置和姿态。

#### **阶段二：离线NDT处理**

1.  **准备离线环境**：启动一个仅包含您的独立NDT模块和我们提供的`offline_ndt_recorder.py`节点的ROS环境。
2.  **回放LiDAR数据**：使用`ros2 bag play`回放之前录制的ROS Bag，但只播放LiDAR数据流。
3.  **记录NDT输出**：`offline_ndt_recorder.py`会自动监听NDT的定位结果，并将其保存为第二个CSV文件。

#### **阶段三：数据分析与结果生成**

1.  **运行分析脚本**：执行`analyze_fidelity_data.py`脚本，它会读取第一阶段生成的“真值位姿CSV”和第二阶段生成的“NDT输出CSV”。
2.  **数据对齐与计算**：脚本会自动按时间戳对齐两份数据，计算每一时刻的平移和旋转误差。
3.  **生成结果**：脚本会自动打印统计表格，并生成误差随时间变化的曲线图。

---

### 3. 所需代码

我们总共需要三个新的Python脚本。

#### **代码1: `ground_truth_recorder.py` (用于阶段一)**
这个节点在**在线**运行时使用，订阅真值位姿话题并将其存为CSV。

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import csv
import os
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped # 根据你的真值话题类型选择
from transforms3d.euler import quat2euler
import numpy as np

# --- 配置 ---
# !!! 修改为你实际的真值位姿话题和消息类型 !!!
GROUND_TRUTH_TOPIC = "/real_world/follow_adtruck/transformed_with_covariance"
# 如果你的话题类型是 PoseStamped，请取消下一行的注释
# from geometry_msgs.msg import PoseStamped
# MESSAGE_TYPE = PoseStamped
MESSAGE_TYPE = PoseWithCovarianceStamped

class GroundTruthRecorder(Node):
    def __init__(self):
        super().__init__('ground_truth_recorder')
        
        self.subscription = self.create_subscription(
            MESSAGE_TYPE,
            GROUND_TRUTH_TOPIC,
            self.pose_callback,
            10)
            
        # 设置CSV文件
        self.csv_filename = 'ground_truth_poses.csv'
        self.csv_file = open(self.csv_filename, 'w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(['timestamp_ns', 'x', 'y', 'z', 'roll_rad', 'pitch_rad', 'yaw_rad'])
        
        self.get_logger().info(f"Ground truth recorder started. Listening to '{GROUND_TRUTH_TOPIC}'.")
        self.get_logger().info(f"Saving data to '{os.path.abspath(self.csv_filename)}'.")

    def pose_callback(self, msg):
        # 从消息中提取位姿和时间戳
        if isinstance(msg, PoseWithCovarianceStamped):
            pose = msg.pose.pose
        elif isinstance(msg, PoseStamped):
            pose = msg.pose
        else:
            self.get_logger().error(f"Unsupported message type: {type(msg)}")
            return
            
        timestamp_ns = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        pos = pose.position
        q = pose.orientation
        
        # 四元数转欧拉角
        roll, pitch, yaw = quat2euler([q.w, q.x, q.y, q.z])
        
        # 写入CSV
        self.csv_writer.writerow([timestamp_ns, pos.x, pos.y, pos.z, roll, pitch, yaw])

    def destroy_node(self):
        self.csv_file.close()
        self.get_logger().info("CSV file closed.")
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    recorder = GroundTruthRecorder()
    try:
        rclpy.spin(recorder)
    except KeyboardInterrupt:
        pass
    finally:
        recorder.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

#### **代码2: `offline_ndt_recorder.py` (用于阶段二)**
这个节点在**离线**验证时使用，记录独立NDT模块的输出。

```python
#!/usr/bin/env python3
# (此文件与 ground_truth_recorder.py 几乎完全相同，只是话题和文件名不同)
import rclpy
from rclpy.node import Node
import csv
import os
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
from transforms3d.euler import quat2euler
import numpy as np

# --- 配置 ---
# !!! 修改为你独立NDT模块输出的位姿话题和消息类型 !!!
NDT_OUTPUT_TOPIC = "/localization/ndt_pose"
MESSAGE_TYPE = PoseWithCovarianceStamped

class OfflineNdtRecorder(Node):
    def __init__(self):
        super().__init__('offline_ndt_recorder')
        
        self.subscription = self.create_subscription(
            MESSAGE_TYPE,
            NDT_OUTPUT_TOPIC,
            self.pose_callback,
            10)
            
        self.csv_filename = 'offline_ndt_poses.csv'
        self.csv_file = open(self.csv_filename, 'w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(['timestamp_ns', 'x', 'y', 'z', 'roll_rad', 'pitch_rad', 'yaw_rad'])
        
        self.get_logger().info(f"Offline NDT recorder started. Listening to '{NDT_OUTPUT_TOPIC}'.")
        self.get_logger().info(f"Saving data to '{os.path.abspath(self.csv_filename)}'.")

    def pose_callback(self, msg):
        if isinstance(msg, PoseWithCovarianceStamped):
            pose = msg.pose.pose
        elif isinstance(msg, PoseStamped):
            pose = msg.pose
        else:
            return
            
        timestamp_ns = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        pos = pose.position
        q = pose.orientation
        
        roll, pitch, yaw = quat2euler([q.w, q.x, q.y, q.z])
        
        self.csv_writer.writerow([timestamp_ns, pos.x, pos.y, pos.z, roll, pitch, yaw])

    def destroy_node(self):
        self.csv_file.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    recorder = OfflineNdtRecorder()
    try:
        rclpy.spin(recorder)
    except KeyboardInterrupt:
        pass
    finally:
        recorder.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

#### **代码3: `analyze_fidelity_data.py` (用于阶段三)**
这个脚本是核心，负责对齐数据、计算误差并生成最终结果。

```python
#!/usr/bin/env python3

import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import seaborn as sns
import argparse
import os

def normalize_angle(angle):
    """将角度归一化到 [-pi, pi] 范围"""
    return (angle + np.pi) % (2 * np.pi) - np.pi

def analyze_fidelity(ground_truth_csv, ndt_csv):
    """
    分析虚拟感知的保真度。
    
    Args:
        ground_truth_csv (str): 真值位姿CSV文件路径。
        ndt_csv (str): 离线NDT输出位姿CSV文件路径。
    """
    # --- 1. 加载数据 ---
    try:
        df_gt = pd.read_csv(ground_truth_csv)
        df_ndt = pd.read_csv(ndt_csv)
        df_gt.sort_values('timestamp_ns', inplace=True)
        df_ndt.sort_values('timestamp_ns', inplace=True)
    except FileNotFoundError as e:
        print(f"错误: 文件未找到 -> {e}")
        return

    # --- 2. 数据对齐 (核心步骤) ---
    # 使用 pd.merge_asof 按时间戳对齐两个数据集
    # 它会为df_ndt中的每一行，找到df_gt中时间戳最接近且不晚于它的那一行
    df_aligned = pd.merge_asof(
        df_ndt,
        df_gt,
        on='timestamp_ns',
        direction='nearest',
        tolerance=50_000_000,  # 50ms 的容忍窗口
        suffixes=('_ndt', '_gt')
    )
    df_aligned.dropna(inplace=True)
    print(f"数据对齐完成: 成功匹配 {len(df_aligned)} / {len(df_ndt)} 条记录。")
    if df_aligned.empty:
        print("错误: 没有数据成功对齐，请检查时间戳或增加tolerance。")
        return

    # --- 3. 计算误差 ---
    # 平移误差 (欧氏距离)
    df_aligned['error_x'] = df_aligned['x_ndt'] - df_aligned['x_gt']
    df_aligned['error_y'] = df_aligned['y_ndt'] - df_aligned['y_gt']
    df_aligned['error_z'] = df_aligned['z_ndt'] - df_aligned['z_gt']
    df_aligned['translation_error'] = np.sqrt(
        df_aligned['error_x']**2 + df_aligned['error_y']**2 + df_aligned['error_z']**2
    )

    # 旋转误差 (欧拉角差值，并转换为度)
    df_aligned['roll_error_deg'] = np.rad2deg(normalize_angle(df_aligned['roll_rad_ndt'] - df_aligned['roll_rad_gt']))
    df_aligned['pitch_error_deg'] = np.rad2deg(normalize_angle(df_aligned['pitch_rad_ndt'] - df_aligned['pitch_rad_gt']))
    df_aligned['yaw_error_deg'] = np.rad2deg(normalize_angle(df_aligned['yaw_rad_ndt'] - df_aligned['yaw_rad_gt']))
    
    # --- 4. 结果呈现: 表格 ---
    stats = {
        'Translation Error (m)': df_aligned['translation_error'].describe(percentiles=[.5, .95, .99]),
        'Yaw Error (deg)': df_aligned['yaw_error_deg'].abs().describe(percentiles=[.5, .95, .99])
    }
    
    print("\n" + "="*60)
    print("           保真度与自洽性验证 - 分析结果")
    print("="*60)
    for name, data in stats.items():
        print(f"\n--- {name} ---")
        print(f"  均值 (Mean):   {data['mean']:.4f}")
        print(f"  标准差 (Std):  {data['std']:.4f}")
        print(f"  最大值 (Max):    {data['max']:.4f}")
        print(f"  99百分位 (P99): {data['99%']:.4f}")
    print("="*60)

    # --- 5. 结果呈现: 绘图 ---
    plt.style.use('seaborn-v0_8-whitegrid')
    fig, axes = plt.subplots(2, 1, figsize=(15, 10), sharex=True)
    fig.suptitle('虚拟感知定位误差分析', fontsize=16)

    # 绘制平移误差图
    axes[0].plot(df_aligned['timestamp_ns'], df_aligned['translation_error'], label='平移误差', color='b', alpha=0.7)
    axes[0].set_ylabel('平移误差 (m)')
    axes[0].set_title('平移误差随时间变化')
    axes[0].legend()

    # 绘制旋转误差图
    axes[1].plot(df_aligned['timestamp_ns'], df_aligned['yaw_error_deg'].abs(), label='偏航角误差 (绝对值)', color='r', alpha=0.7)
    axes[1].set_ylabel('旋转误差 (度)')
    axes[1].set_title('偏航角误差随时间变化')
    axes[1].set_xlabel('时间戳 (ns)')
    axes[1].legend()

    plt.tight_layout(rect=[0, 0.03, 1, 0.95])
    
    # 保存图表
    output_filename = 'fidelity_analysis_plot.png'
    plt.savefig(output_filename, dpi=300)
    print(f"\n分析图表已保存至: {os.path.abspath(output_filename)}")
    plt.show()

def main():
    parser = argparse.ArgumentParser(description="分析虚拟LiDAR数据的保真度")
    parser.add_argument('ground_truth_csv', help="包含真值位姿的CSV文件")
    parser.add_argument('ndt_csv', help="包含离线NDT输出位姿的CSV文件")
    args = parser.parse_args()
    
    analyze_fidelity(args.ground_truth_csv, args.ndt_csv)

if __name__ == '__main__':
    main()
```

---

### 4. 代码设计思路

*   **分离原则**：我们将整个流程分解为三个独立的、职责单一的脚本。这使得每一步都清晰可控，易于调试。例如，如果最终没有结果，我们可以先检查两个CSV文件是否正确生成，再排查分析脚本的问题。
*   **CSV作为中间格式**：使用CSV作为数据交换格式，具有良好的可读性和通用性。它解耦了ROS环境和数据分析环境，我们可以用任何喜欢的数据分析工具（Python, MATLAB, Excel等）来处理它。
*   **鲁棒的数据对齐**：在分析脚本中，最关键的一步是数据对齐。我们没有使用简单的循环查找，而是利用了Pandas库中专为时间序列设计的`merge_asof`函数。它非常高效，并且能通过`tolerance`参数处理真实世界中时间戳不完全一致的问题，使对齐过程既快速又健壮。
*   **角度归一化**：在计算旋转误差时，特别是偏航角（Yaw），必须进行归一化处理。例如，`1°`和`359°`的差值应该是`2°`而不是`358°`。`normalize_angle`函数确保了角度差计算的正确性。

---

### 5. 完整使用说明

#### **步骤一：在线数据收集**

1.  **准备节点**：将`ground_truth_recorder.py`放入您的ROS 2工作空间并编译。
2.  **启动环境**：启动您的完整FSM Sandbox仿真环境（CARLA, Autoware, RBF变换节点等）。
3.  **启动记录器**：
    *   在一个新终端中，启动真值记录节点：
        ```bash
        ros2 run your_package_name ground_truth_recorder.py
        ```
        此时，会开始生成 `ground_truth_poses.csv` 文件。
    *   在另一个终端中，启动`ros2 bag`来录制LiDAR数据：
        ```bash
        # 确保话题名称正确！
        ros2 bag record -o lidar_bag /carla/ego_vehicle/lidar
        ```
4.  **执行任务**：驾驶车辆几分钟。
5.  **停止记录**：在各个终端按`Ctrl+C`。您现在应该有一个`ground_truth_poses.csv`文件和一个`lidar_bag`目录。

#### **步骤二：离线NDT处理**

1.  **准备节点**：将`offline_ndt_recorder.py`放入您的工作空间并编译。
2.  **启动离线环境**：
    *   在一个终端中，启动您的**独立NDT模块**（确保它加载了正确的地图）。
    *   在另一个终端中，启动离线NDT记录器：
        ```bash
        ros2 run your_package_name offline_ndt_recorder.py
        ```
        此时，会开始生成`offline_ndt_poses.csv`文件。
3.  **回放数据**：在第三个终端中，播放之前录制的LiDAR数据：
    ```bash
    ros2 bag play lidar_bag
    ```
4.  **等待完成**：等待bag播放完毕。然后按`Ctrl+C`停止NDT和记录器节点。您现在应该有一个`offline_ndt_poses.csv`文件。

#### **步骤三：分析与出图**

1.  **准备环境**：确保您已安装`pandas`, `numpy`, `matplotlib`, `seaborn`, `transforms3d`。
    ```bash
    pip install pandas numpy matplotlib seaborn transforms3d
    ```
2.  **运行脚本**：将`analyze_fidelity_data.py`, `ground_truth_poses.csv`, 和 `offline_ndt_poses.csv`放在同一个目录下。然后执行：
    ```bash
    python analyze_fidelity_data.py ground_truth_poses.csv offline_ndt_poses.csv
    ```
3.  **查看结果**：
    *   终端会打印出详细的误差统计表格。
    *   会自动弹出一个包含两张误差曲线图的窗口，并将其保存为`fidelity_analysis_plot.png`。

这个完整的方案为您提供了从数据采集到最终报告生成所需的一切，确保您可以严谨、高效地完成保真度验证实验。