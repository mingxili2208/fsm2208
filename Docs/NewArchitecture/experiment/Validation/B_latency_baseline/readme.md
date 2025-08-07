#

### 1. `latency_chain_analyzer.py` - 延迟链路分析器

#### 设计逻辑分析

该脚本的核心设计思想是一个**被动的、事件驱动的在线监听器**。它不主动发起任何请求，而是订阅自动驾驶系统中代表关键时间节点的多个话题。其最终目标是精确测量 `ΔT_R2V` (信息年龄) 和 `ΔT_planner` (规划器延迟)。

- **因果链重构**: 它通过**时间戳关联**的方式，在数据到达的终点（`planning_output_callback`）触发，然后**向后追溯**，从内存缓冲区中寻找时间上最接近的、符合因果关系的“上游”消息。这是一种非常聪明的非侵入式测量方法。
- **数据缓冲**: 使用`deque`作为高效的环形缓冲区，既能保证对近期数据的快速访问，又能自动淘汰过时数据，有效控制内存占用。
- **健壮性设计**:
  - `correlation_window_ns`: 定义了时间戳关联的“有效半径”，避免将时间上相差过远的消息错误地关联起来。
  - `valid_correlation`检查: 对计算出的延迟值进行范围检查，剔除明显不合理的异常测量值。
  - `confidence`评分: 基于链条上各环节的时间差，为每次成功关联的质量打分，这是一个非常精细的设计，能在后续分析中对数据进行加权或筛选。

#### 程序流程分析

1. **初始化 (`__init__`)**:
    - 建立会话（session），创建唯一的输出文件名。
    - 创建并初始化用于存储数据的CSV文件，写入表头。
    - 初始化所有数据缓冲区 (`deque`) 和统计计数器。
    - 创建对四个关键话题的ROS 2订阅者。
    - 启动定时器，用于周期性地报告统计信息和清理旧数据。

2. **数据采集 (Callbacks)**:
    - `ground_truth_callback`, `perception_callback`, `planning_input_callback`: 这三个回调函数是纯粹的数据“摄入”端。它们接收到消息后，仅提取时间戳和少量元数据，并将其存入各自的缓冲区。

3. **核心关联逻辑 (`planning_output_callback`)**:
    - 这是整个程序的**触发点**。当代表因果链末端的`/planning/trajectory`消息到达时，程序开始工作。
    - 它首先在`planning_input_buffer`中寻找与当前消息时间戳最接近的“规划输入”事件。
    - 如果找到，它会使用该匹配项中已经预先关联好的`t_ground_truth_ns`和`t_perception_ns`。
    - 至此，一条完整的`T_ground_truth` -> `T_perception` -> `T_planning_input` -> `T_planning_output`时间链被重构出来。

4. **计算与记录**:
    - 基于重构出的时间链，计算`r2v_delay_ms`和`planner_delay_ms`。
    - 进行有效性验证和置信度评分。
    - 如果验证通过，将整条数据链的详细信息写入CSV文件。

5. **维护与关闭**:
    - `report_statistics`定时器周期性地打印当前的数据收集和关联成功率，便于实时监控。
    - `cleanup_old_entries`定时器防止因消息丢失导致某些缓冲区无限增长，保证了系统的长期运行稳定性。
    - `destroy_node`在节点关闭时，确保文件句柄被正确关闭。

#### Markdown格式设计说明

```markdown
# 设计说明: latency_chain_analyzer.py

## 1. 概述

`latency_chain_analyzer.py`是一个ROS 2节点，其核心功能是**在线、被动地**测量自动驾驶系统中从“真实世界状态感知”到“规划决策输出”这一段链路的延迟。它具体量化了以下两个关键指标：

-   **ΔT_R2V (信息年龄)**: 从外部真值系统捕获位姿到ADS接收到可用的规划输入（如定位结果）的延迟。
-   **ΔT_planner (规划器延迟)**: ADS规划算法从接收到输入到生成轨迹的内部处理耗时。

## 2. 核心设计原则

-   **非侵入式监听 (Passive Listener)**: 本节点不修改任何现有系统行为，仅通过订阅相关话题来收集数据，对被测系统无干扰。
-   **事件驱动关联 (Event-Driven Correlation)**: 以后续事件（规划输出）的到达为触发器，回溯性地在内存缓冲区中寻找与之匹配的前序事件，重构因果链。
-   **数据质量感知 (Quality-Aware)**: 不仅记录延迟值，还通过有效性检查和置信度评分来评估每次测量的可靠性，便于后续进行高质量的数据分析。

## 3. 关键组件与数据结构

-   **ROS 2订阅者**:
    -   `ground_truth_topic`: 接收外部真值系统位姿，作为`T_ground_truth`。
    -   `perception_topic`: 接收虚拟传感器数据，作为`T_perception`。
    -   `planning_input_topic`: 接收ADS定位模块输出，作为`T_planning_input`。
    -   `planning_output_topic`: 接收规划器输出轨迹，作为`T_planning_output`，并作为关联逻辑的触发器。
-   **数据缓冲区 (`collections.deque`)**:
    -   为每个订阅的话题维护一个固定长度的`deque`。
    -   `deque`中存储包含`timestamp_ns`和其他必要元数据（如已关联的前序时间戳）的字典。

## 4. 程序流程

1.  **初始化**: 启动节点，创建唯一的会话ID，并初始化CSV日志文件。
2.  **监听与缓冲**: 各个回调函数独立、异步地接收消息，并将时间戳信息存入对应的`deque`。
3.  **关联与计算**:
    -   当`planning_output_callback`被触发时，代表一条潜在的因果链已完成。
    -   程序在`planning_input_buffer`中寻找时间上最接近的匹配项。
    -   若找到，则一条完整的`T_ground_truth` -> `T_perception` -> `T_planning_input` -> `T_planning_output`时间链被建立。
    -   计算`ΔT_R2V`和`ΔT_planner`。
4.  **验证与记录**:
    -   对计算出的延迟值进行范围检查。
    -   根据各环节时间差计算置信度。
    -   将完整的、有效的数据点写入CSV文件。
5.  **周期维护**: 定时器周期性地输出统计信息并清理缓冲区中的过时数据。

## 5. 输出

-   **`latency_chain_YYYYMMDD_HHMMSS.csv`**:
    -   **核心列**: `r2v_delay_ms`, `planner_delay_ms`。
    -   **辅助列**: 包含构成该计算的四个原始时间戳、会话ID、有效性标志和置信度评分，为数据分析和调试提供充分信息。

```

---

### 2. `v2r_latency_analyzer.py` - V2R延迟分析器

#### 设计逻辑分析

此脚本的设计思想与上一个完全不同，它是一个**离线的、基于日志的分析工具**。它的目标专一而明确：精确测量`ΔT_V2R`，即从PC端发出控制指令到物理小车完成无线发送的延迟。

- **RTT测量法**: 核心方法是计算**往返时间 (Round-Trip Time, RTT)**。它通过匹配PC控制器日志中的`CMD_SENT`事件和由Arduino回应并被PC记录的`CMD_SENT_BY_NRF`事件，来计算一次命令交互的完整闭环时间。
- **对称延迟假设**: 脚本基于一个关键且合理的假设：数据在PC到小车（下行）和小车到PC（上行）的延迟是**对称的**。因此，单向的V2R延迟可以估算为`RTT / 2`。
- **数据驱动**: 整个分析过程完全由输入的`timing_log.csv`驱动，不依赖任何ROS环境，具有很好的可移植性和可重复性。
- **数据清洗**: 在计算时，脚本隐式地通过`time_window`和`if 0 < rtt_ms < 500:`等条件对数据进行了清洗，排除了无匹配或延迟极端的异常点。

#### 程序流程分析

1. **初始化 (`__init__`)**:
    - 接收一个`timing_log.csv`文件路径作为输入。
    - 调用`load_timing_log()`方法。

2. **数据加载 (`load_timing_log`)**:
    - 使用`pandas`库高效地将CSV文件读入DataFrame。
    - 对DataFrame的列名进行存在性校验，确保日志格式正确。

3. **核心分析 (`analyze_v2r_latency`)**:
    - 将DataFrame按`event_type`过滤成两个子集：`cmd_sent_events`和`nrf_sent_events`。
    - 遍历`cmd_sent_events`中的每一个命令发送事件。
    - 对于每个`cmd_sent`事件，在`nrf_sent_events`中寻找时间戳最接近且在其后的匹配项。
    - 找到匹配后，计算两者时间戳之差，得到`RTT`。
    - 计算`v2r_latency_ms = rtt_ms / 2`。
    - 将计算结果（包括RTT和V2R延迟）存储在一个列表中。

4. **结果输出**:
    - `get_v2r_statistics()`: 对所有测量到的V2R延迟值进行统计计算（均值、标准差、百分位等）。
    - `save_v2r_data()`: 将逐条计算出的V2R延迟数据保存到一个新的、干净的CSV文件中，供后续综合分析使用。

#### Markdown格式设计说明

```markdown
# 设计说明: v2r_latency_analyzer.py

## 1. 概述

`v2r_latency_analyzer.py`是一个**离线数据分析脚本**，其唯一目的是从PC控制器产生的`timing_log.csv`中，精确地计算和分析**V2R (Virtual-to-Real) 延迟**。该延迟代表了从PC端发出控制指令到物理小车底层完成无线发送的完整链路耗时。

## 2. 核心设计原则

-   **离线分析 (Offline Analysis)**: 脚本独立于ROS环境，对已生成的日志文件进行处理，保证了分析结果的可重复性。
-   **RTT测量法 (Round-Trip Time Measurement)**: 通过匹配成对的命令发送(`CMD_SENT`)和命令确认(`CMD_SENT_BY_NRF`)事件，计算命令的往返时间(RTT)，这是测量网络或通信延迟的经典方法。
-   **对称延迟假设 (Symmetric Latency Assumption)**: 脚本的核心估算模型基于`ΔT_V2R ≈ RTT / 2`，其物理假设是下行（PC→小车）和上行（小车→PC）的通信延迟是对称的。对于USB串口和nRF24L01组成的链路，这是一个高度合理的假设。

## 3. 输入

-   **`timing_log.csv`**: 一个CSV文件，必须包含以下列：
    -   `event_type`: 事件类型字符串 (如 'CMD_SENT', 'CMD_SENT_BY_NRF')。
    -   `pc_time`: PC记录的事件时间戳（毫秒）。
    -   `steering_angle`, `speed`: 原始控制命令值，用于上下文分析。

## 4. 程序流程

1.  **加载**: 使用`pandas`将输入的`timing_log.csv`加载到DataFrame。
2.  **过滤**: 将DataFrame分解为两个子集：所有`CMD_SENT`事件和所有`CMD_SENT_BY_NRF`事件。
3.  **匹配与计算**:
    -   遍历`CMD_SENT`事件。
    -   为每个`CMD_SENT`事件，在`CMD_SENT_BY_NRF`事件子集中寻找时间上最接近的、合法的（在其后发生）确认事件。
    -   计算两个事件时间戳的差值，得到`RTT`。
    -   计算`V2R_latency = RTT / 2`。
4.  **存储**: 将所有成功计算的`V2R_latency`数据点保存到一个新的CSV文件中。
5.  **统计**: 提供方法以计算所有`V2R_latency`测量值的完整统计特性（均值、中位数、标准差等）。

## 5. 输出

-   **`v2r_latency_YYYYMMDD_HHMMSS.csv`**:
    -   **核心列**: `v2r_latency_ms`。
    -   **辅助列**: `rtt_ms`以及原始的命令和时间戳，便于调试和深入分析。
-   **统计摘要 (标准输出)**: 在命令行直接打印V2R延迟的均值、P95、P99等关键统计指标。

```

---

### 3. `tracking_error_measurer.py` - 跟踪误差测量器

#### 设计逻辑分析

该脚本是一个**目标明确的在线测试执行与测量节点**。它的设计目的是在一个高度受控的场景（恒速直线运动）下，精确测量车辆的**纵向和横向跟踪误差**。

- **自动化测试状态机**: 脚本内置一个简单的`Idle -> Testing -> Finished`状态机。它通过`is_near_start()`自动检测测试开始条件，并通过定时器或路径终点检测来结束测试，实现了测试过程的自动化。
- **Frenet坐标系变换**: 核心计算是将在Cartesian坐标系下的车辆位置(`x, y`)，投影到一个预定义的直线`reference_path`上，从而得到Frenet坐标系下的`s`（沿路径的距离）和`d`（偏离路径的距离）。这是计算跟踪误差的标准方法。
- **理论模型验证**: 它通过`s_target = self.target_velocity_ms * elapsed_time_s`计算出理论上车辆应该在的位置，然后与实际位置`s_actual`比较，得到`longitudinal_error_m`。这个误差值是验证“延迟-误差”物理模型的直接数据来源。
- **鲁棒性增强**: 增加了基于有限差分的速度估计，使得记录的数据更加完整和真实。

#### 程序流程分析

1. **初始化 (`__init__`)**:
    - 创建会话，设置日志文件。
    - 定义测试参数，如目标速度、测试时长。
    - 使用`shapely`库创建一个`LineString`对象作为参考路径。
    - 初始化测试状态`self.test_active = False`。
    - 创建位姿话题的订阅者和测试管理定时器。

2. **测试状态转换**:
    - 在`pose_callback`中，如果`test_active`为`False`，则持续调用`is_near_start()`检查车辆位置。
    - 一旦车辆靠近起点，`test_active`置为`True`，记录`start_time_ns`，测试正式开始。
    - 测试的结束由两个条件触发：
        1. 在`pose_callback`中，检测到`elapsed_time_s`超过预设时长或`s_actual`超过路径长度。
        2. 在`check_test_status`定时器中，检测到测试超时。

3. **核心计算 (`pose_callback`中当`test_active`为`True`时)**:
    - 计算从测试开始的流逝时间`elapsed_time_s`。
    - 调用`calculate_frenet_coordinates()`计算`s_actual`和`lateral_error`。
    - 计算理论目标位置`s_target`。
    - 计算核心指标`longitudinal_error_m = s_actual - s_target`。
    - （已优化）计算瞬时速度估计。
    - 将所有计算结果写入CSV文件。

4. **关闭**: 节点关闭时，确保文件被保存，并打印简单的总结信息。

#### Markdown格式设计说明

```markdown
# 设计说明: tracking_error_measurer.py

## 1. 概述

`tracking_error_measurer.py`是一个ROS 2节点，专用于执行和测量**恒速直线跟踪误差基线测试**。其核心任务是在一个高度受控的场景下，精确记录车辆的纵向和横向跟踪误差，为验证“延迟-误差”物理模型提供关键的实验数据。

## 2. 核心设计原则

-   **自动化测试执行 (Automated Test Execution)**: 节点内置一个简单的状态机，能自动检测车辆是否到达测试起点来启动测试，并通过超时或到达终点来自动结束测试，提高了实验的可重复性。
-   **Frenet坐标系变换 (Frenet Frame Transformation)**: 采用标准的Frenet坐标系来描述车辆状态。通过将车辆的全局位姿投影到预定义的参考路径上，精确地分解出沿路径的进度(`s`)和垂直路径的偏差(`d`)。
-   **理论模型验证支持 (Model Validation Support)**: 节点不仅测量实际误差，还计算理论上的目标进度(`s_target`)，直接生成用于验证`Error ≈ v * ΔT`模型的`longitudinal_error_m`数据。

## 3. 关键组件与数据结构

-   **ROS 2订阅者**:
    -   `pose_topic`: 订阅车辆的定位信息（如`/localization/kinematic_state`），作为计算误差的输入。
-   **参考路径 (`shapely.geometry.LineString`)**:
    -   在代码中硬编码或从配置中读取一个`LineString`对象，作为理想的、无误差的参考轨迹。
-   **测试状态机变量**:
    -   `self.test_active` (boolean): 标记测试是否正在进行。
    -   `self.start_time_ns` (int): 记录测试开始的精确时间戳。

## 4. 程序流程

1.  **初始化**: 定义参考路径和测试参数。节点启动后处于**待机(Idle)**状态。
2.  **测试启动**: `pose_callback`持续检查车辆位置。当车辆进入起点附近区域时，状态切换为**测试中(Testing)**，并记录开始时间。
3.  **数据测量 (Testing状态)**:
    -   对于每一帧位姿数据，计算相对于测试开始的时间`elapsed_time_s`。
    -   计算理论目标进度 `s_target = target_velocity * elapsed_time_s`。
    -   调用Frenet变换函数，计算实际进度`s_actual`和横向误差`lateral_error_m`。
    -   计算纵向误差 `longitudinal_error_m = s_actual - s_target`。
    -   将所有数据点写入CSV文件。
4.  **测试结束**: 当测试时间超时、或车辆跑完预定路程时，状态切换为**完成(Finished)**，停止数据记录。

## 5. 输出

-   **`tracking_error_YYYYMMDD_HHMMSS.csv`**:
    -   **核心列**: `longitudinal_error_m`, `lateral_error_m`。
    -   **辅助列**: `elapsed_time_s`, `s_actual`, `s_target`以及车辆的原始位姿，为后续的详细分析和可视化提供了完整上下文。

```

---

### 4. `experiment_b_analyzer.py` - 综合分析器

#### 设计逻辑分析

这个脚本是整个实验B的**大脑和成果展示中心**。它是一个离线的、集大成的分析工具，其设计逻辑是将前面三个独立收集的数据源进行**融合、分析、可视化和报告**。

- **数据融合 (Data Fusion)**: 它的核心价值在于将三个独立的CSV文件（`R2V+Planner`延迟、`V2R`延迟、跟踪误差）联系起来。它通过计算`ΔT_total = mean(ΔT_R2V) + mean(ΔT_planner) + mean(ΔT_V2R)`，首次得到了系统端到端的完整延迟画像。
- **分部式分析 (Component-wise Analysis)**: 脚本清晰地将分析过程分为“延迟特性分析”和“跟踪误差基线分析”两部分，使得逻辑清晰，结果易于理解。
- **模型验证**: 在第二部分分析中，它用第一部分计算出的`ΔT_total`和预设的`target_velocity`来计算`theoretical_error_m`，并将其与实测的`actual_error_mean_m`进行对比，完成了对核心物理模型的定量验证。
- **自动化报告**: 脚本的最终目标是自动化地生成一套完整的分析产物，包括多角度的可视化图表、可供进一步处理的JSON数据，以及一份图文并茂、带有执行摘要和结论的Markdown报告。

#### 程序流程分析

1. **初始化 (`__init__`)**:
    - 接收三个CSV文件的路径作为输入。
    - 创建本次分析的唯一会话，并建立用于存放结果的目录结构。
    - 调用`load_data()`。

2. **数据加载与清洗 (`load_data`, `clean_data`)**:
    - 使用`pandas`加载三个数据文件。
    - 对每个DataFrame应用清洗规则（例如，只保留`valid_correlation == True`的数据，剔除延迟或误差值极端的离群点），保证分析数据的质量。

3. **第一部分分析 (`analyze_latency_characteristics`)**:
    - 分别从`latency_chain_df`和`v2r_df`中提取`r2v_delay_ms`, `planner_delay_ms`, `v2r_latency_ms`数据。
    - 对这三个延迟分量分别计算详细的统计数据（均值、中位数、P95等）。
    - 计算总延迟的均值。
    - 将所有统计结果存入`self.analysis_results`字典。
    - 调用`print_latency_table`和`assess_latency_performance`在控制台输出结果。

4. **第二部分分析 (`analyze_tracking_error_baseline`)**:
    - 从`self.analysis_results`中获取已计算出的总延迟均值。
    - 计算`theoretical_error_m`。
    - 从`tracking_error_df`中计算`actual_error_mean_m`等统计量。
    - 计算`agreement_ratio`来量化理论与实际的符合程度。
    - 将结果存入`self.analysis_results`并打印到控制台。

5. **输出生成 (`create_...`, `generate_...`, `save_...`)**:
    - `create_comprehensive_plots`: 使用`matplotlib`绘制一个包含9个子图的、信息密度极高的大图，全面展示延迟和误差的特性。
    - `generate_report`: 使用f-string模板，将所有分析结果动态地填充到一个结构化的Markdown报告中。
    - `save_results_json`: 将分析结果以机器友好的JSON格式保存。
    - 所有生成的文件都以唯一的分析时间戳命名，并存放在专属的分析结果目录中。

#### Markdown格式设计说明

```markdown
# 设计说明: experiment_b_analyzer.py

## 1. 概述

`experiment_b_analyzer.py`是实验B的**终极分析与报告生成引擎**。它是一个离线脚本，负责将来自`latency_chain_analyzer`、`v2r_latency_analyzer`和`tracking_error_measurer`的多个独立数据源进行**融合、综合分析、可视化和报告生成**。

## 2. 核心设计原则

-   **数据融合 (Data Fusion)**: 脚本的核心是将三个独立的延迟分量（R2V, Planner, V2R）的测量结果进行整合，计算出系统端到端的总延迟，并将其与独立的跟踪误差测量数据进行关联。
-   **分步式分析 (Two-Part Analysis)**: 逻辑上将分析过程清晰地划分为两个阶段：
    1.  **延迟特性分析**: 全面刻画系统各环节的延迟分布与统计特性。
    2.  **误差基线验证**: 利用第一阶段的结果，定量验证`Error ≈ v * ΔT`物理模型。
-   **自动化报告 (Automated Reporting)**: 旨在实现“一键式”分析，输入原始数据，自动产出一整套标准化的、高质量的分析产物，包括图表、数据和总结报告。

## 3. 输入

-   **`latency_chain.csv`**: 由`latency_chain_analyzer.py`生成，提供R2V和Planner延迟数据。
-   **`v2r_latency.csv`**: 由`v2r_latency_analyzer.py`生成，提供V2R延迟数据。
-   **`tracking_error.csv`**: 由`tracking_error_measurer.py`生成，提供恒速直线测试的跟踪误差数据。

## 4. 程序流程

1.  **加载与清洗**: 加载所有输入的CSV文件到`pandas` DataFrame，并执行数据清洗操作，移除无效或异常的数据点。
2.  **延迟分析**:
    -   对R2V、Planner、V2R三个延迟分量分别进行完整的统计分析。
    -   通过求和各分量的均值，计算出系统端到端的平均总延迟`ΔT_total`。
3.  **误差分析**:
    -   使用`ΔT_total`和测试配置中的目标速度，计算**理论纵向误差**。
    -   计算**实际纵向误差**的均值。
    -   通过比较理论与实际误差，计算**符合度比率(Agreement Ratio)**。
4.  **输出生成**:
    -   **可视化**: 调用`matplotlib`生成一张包含9个子图的综合分析图，涵盖延迟分布、构成、时间序列以及误差对比等。
    -   **报告**: 将所有关键统计数据和分析结论，动态填充到一个预设的Markdown模板中，生成一份图文并茂的分析报告。
    -   **数据归档**: 将所有分析的数字结果保存到一个JSON文件中，便于后续的程序化处理或元分析。

## 5. 输出

-   **`experiment_b_analysis_YYYYMMDD_HHMMSS.png`**: 综合分析图。
-   **`experiment_b_report_YYYYMMDD_HHMMSS.md`**: 人类可读的详细分析报告。
-   **`experiment_b_results_YYYYMMDD_HHMMSS.json`**: 机器可读的数字结果摘要。

