# FSM Sandbox 时效性验证实验 - 设计说明

## 1. 项目背景与目标

### 1.1. 背景
FSM Sandbox 作为高保真度的自动驾驶仿真平台，其核心价值在于精确模拟真实世界的物理和感知。其中，从“控制指令发出”到“感知结果反馈”的端到端（End-to-End）延迟是衡量仿真系统“时效性”和“可信度”的关键指标。过高或不稳定的延迟会严重影响上层规控算法的性能和安全性。

### 1.2. 目标
本项目旨在设计并实现一套完整的、自动化的时效性验证方案，用于精确测量FSM Sandbox中以下核心反馈回路的延迟：
`定位更新 -> 仿真车辆位姿同步 -> 传感器数据生成 -> 感知模块处理`

该方案需达成以下目标：
1.  **精确测量**: 捕获并记录流水线中四个关键节点的时间戳（T1, T2, T3, T4）。
2.  **量化分析**: 计算渲染延迟、感知处理延迟和端到端总延迟，并提供均值、中位数、标准差和P95/P99百分位等关键统计指标。
3.  **自动评估**: 根据预设标准自动评估系统性能，并给出结论。
4.  **结果可视化**: 生成直观的数据图表，便于快速定位性能瓶颈和分析问题。
5.  **高可复用性**: 提供完整的工具包和使用说明，确保实验过程可重复、结果可复现。

## 2. 系统架构与数据流

本实验的核心是追踪一次定位更新所引发的连锁反应。我们定义了四个关键时间戳来描绘这个过程：

![Data Flow Diagram](https://i.imgur.com/your-diagram-placeholder.png)  <!-- 此处建议用实际图表替换 -->

**数据流与时间戳定义:**

1.  **T1: 指令接收 (Instruction Reception)**
    *   **事件**: `CarlaUpdateVehicleHandler` 节点通过其订阅器接收到来自 Autoware (或其他定位源) 的 `PoseWithCovarianceStamped` 消息。
    *   **来源**: 消息头中的 `header.stamp`。
    *   **意义**: 标志着一次车辆位姿更新请求的开始。

2.  **T2: 指令执行 (Instruction Execution)**
    *   **事件**: `CarlaUpdateVehicleHandler` 节点调用 CARLA 的同步API `ego_vehicle.set_transform()`，将新位姿强制应用到仿真世界中的车辆模型上。
    *   **来源**: 调用API前，通过 `get_clock().now()` 获取的当前ROS时间。
    *   **意义**: 标志着控制指令已送达仿真引擎。

3.  **T3: 效果呈现 (Effect Manifestation)**
    *   **事件**: 位于新位姿上的车载LiDAR传感器完成一次扫描，并发布对应的 `PointCloud2` 消息。
    *   **来源**: `PointCloud2` 消息头中的 `header.stamp`。
    *   **意义**: 标志着控制指令的效果已经体现在了传感器数据上。

4.  **T4: 反馈闭环 (Feedback Loop Closure)**
    *   **事件**: `log_recorder_node` 节点接收到 NDT（或其他定位算法）模块处理完 T3 点云后，输出的最新定位结果。
    *   **来源**: `log_recorder_node` 接收到NDT结果消息时，通过 `get_clock().now()` 获取的当前ROS时间。
    *   **意义**: 标志着整个“控制-感知-定位”反馈回路的完成。

**延迟计算公式:**
*   **渲染延迟 (Render Latency)**: `T3 - T2` (从指令执行到传感器生成数据的延迟)
*   **NDT处理延迟 (NDT Processing Latency)**: `T4 - T3` (从获得传感器数据到输出定位结果的延迟)
*   **总流水线延迟 (Total Pipeline Latency)**: `T4 - T1` (从接收指令到完成反馈闭环的端到端延迟)

## 3. 核心组件实现

### 3.1. `CarlaUpdateVehicleHandler.py` (修改)
*   **职责**: 成为T1和T2时间戳的生产者。
*   **修改逻辑**:
    1.  新增一个 `/fsm_sandbox/timing/t1_t2` 主题的发布器，消息类型为 `sensor_msgs/TimeReference`。
    2.  在 `update_Vehicle_handler_callback` 中，当接收到新的位姿消息时，将其 `header` (包含T1) 缓存起来，并设置一个 `pose_update_pending` 标志位。
    3.  在原有的 `apply_control_50hz` 定时器回调中，检查 `pose_update_pending` 标志。
    4.  若为真，则：
        a. 获取当前时间作为 T2。
        b. 构建 `TimeReference` 消息，将缓存的 `header.stamp` (T1) 和当前时间 (T2) 填入。
        c. 发布该消息。
        d. **紧接着**调用 `self.ego_vehicle.set_transform()`。
        e. 重置标志位。

### 3.2. `log_recorder_node.py` (数据收集节点)
*   **职责**: 关联T1, T2, T3, T4，计算延迟并存入CSV文件。
*   **核心逻辑**:
    1.  **订阅三个主题**:
        *   `/fsm_sandbox/timing/t1_t2` (获取 T1, T2)
        *   `lidar_topic_name` (获取 T3)
        *   `ndt_pose_topic_name` (获取 T4 和 T3的引用)
    2.  **数据关联**:
        *   使用 `t1_t2_map = {t1: t2}` 存储收到的T1-T2对。
        *   当收到LiDAR消息(T3)时，在 `t1_t2_map` 中查找与T3时间最接近的T1（在`correlation_window_ns`阈值内），建立 `t3_to_t1_map = {t3: t1}` 的映射。
        *   当收到NDT位姿消息(T4)时，通过其`header.stamp`(T3) 在 `t3_to_t1_map` 中找到T1，再通过T1在 `t1_t2_map` 中找到T2。
    3.  **数据记录**:
        *   成功关联后，计算各项延迟。
        *   将 `Timestamp_ISO`, `T1_ns`, `T2_ns`, `T3_ns`, `T4_ns` 及所有计算出的延迟值写入CSV文件。
        *   从map中移除已处理的条目，防止重复关联。

### 3.3. `analyze_timing_data.py` (数据分析脚本)
*   **职责**: 对CSV数据进行深度分析和可视化。
*   **功能模块**:
    1.  **数据加载与验证**: 使用 `pandas` 读取CSV，并过滤掉 `Sequence_Valid` 为 `False` 的无效数据。
    2.  **统计分析**: 对延迟数据列计算均值、中位数、标准差及P95/P99/P99.9百分位数。
    3.  **性能评估**: 将统计结果与预定义阈值比较，输出PASS/FAIL结论。
    4.  **可视化引擎**: 使用 `matplotlib` 和 `seaborn` 生成两种核心图表：
        *   **综合分析图 (PNG)**: 包含直方图、箱形图、时序图和CDF图。
        *   **性能总结表 (PNG)**: 将关键数据制成美观的表格图片。
    5.  **命令行接口**: 使用 `argparse` 接收CSV文件路径作为输入，并提供 `--show-plots` 选项。

## 4. 实验步骤与交付物

### 4.1. 实验流程
1.  **准备**: 安装 `pandas`, `matplotlib`, `seaborn` 等依赖。将代码文件部署到工作区。
2.  **配置**: 根据实际ROS环境，修改 `log_recorder_node.py` 中的LiDAR和NDT话题名称。
3.  **执行**: 启动完整的仿真环境（CARLA, Autoware, 修改后的`CarlaUpdateVehicleHandler`等）。
4.  **采集**: 在新终端中运行 `python3 log_recorder_node.py`，让系统稳定运行3-5分钟以收集足够样本。按 `Ctrl+C` 结束采集。
5.  **分析**: 运行 `python3 analyze_timing_data.py <path_to_generated_csv_file>`。
6.  **审查**: 查看控制台输出的统计报告，并检查当前目录下生成的PNG图表文件。

### 4.2. 交付物
每次成功执行分析后，将产出以下文件：
1.  **原始数据**: `fsm_timing_log_YYYYMMDD_HHMMSS.csv`
2.  **综合分析图**: `fsm_latency_analysis_YYYYMMDD_HHMMSS.png`
3.  **性能总结表**: `fsm_performance_summary_YYYYMMDD_HHMMSS.png`
4.  **控制台报告**: 包含详细统计数据和性能评估的文本输出。

## 5. 结论
本设计方案提供了一个端到端的、自动化的时效性验证工具链。通过精确的数据捕获、深入的统计分析和直观的可视化报告，能够有效地量化FSM Sandbox的性能表现，为系统优化、版本迭代和算法评估提供强有力的数据支持。