# 增强版自动驾驶测试系统 - 用户手册

欢迎使用增强版自动驾驶测试系统！本手册将指导您完成系统的安装、配置和日常使用。

## 1. 系统概述

本系统旨在提供一个强大且易于使用的平台，用于在CARLA仿真环境中测试Autoware自动驾驶系统的性能。它通过一个交互式命令行菜单，简化了复杂测试场景的执行、监控和管理。

## 2. 准备工作

### 2.1. 环境要求

在开始之前，请确保您的系统满足以下条件：

- **操作系统**: Ubuntu 20.04 或 22.04
- **Python**: 版本 3.8 或更高
- **ROS2**: Humble Hawksbill
- **CARLA**: 版本 0.9.13 或更高
- **Autoware**: 已正确安装并配置

### 2.2. 安装依赖

本系统依赖于几个Python库。打开终端并运行以下命令进行安装：

```bash
pip3 install pynput numpy transforms3d setproctitle psutil
```

## 3. 首次配置

在第一次运行系统之前，建议生成一个默认的配置文件。这可以帮助您快速了解所有可用的配置选项。

1. **导航到脚本目录**:

    ```bash
    cd _integrated_scenario_test/scripts
    ```

2. **生成默认配置文件**:
    运行以下命令，将在 `_integrated_scenario_test/config/` 目录下创建一个名为 `my_config.json` 的文件。

    ```bash
    python3 enhanced_integrated_scenario_test.py --create-default-config ../config/my_config.json
    ```

3. **检查配置文件** (可选):
    您可以打开 `my_config.json` 文件，查看所有默认参数，如目标点坐标、NPC车辆数量等。

## 4. 运行测试系统

请严格按照以下顺序启动各个组件，以确保系统正常工作。

### 步骤 1: 启动 CARLA 仿真器

打开一个新的终端，进入您的CARLA安装目录并启动它。

```bash
# 示例路径，请替换为您自己的CARLA路径
cd /opt/carla-simulator
./CarlaUE4.sh
```

### 步骤 2: 启动 Autoware

打开另一个新的终端，加载您的Autoware工作空间并启动规划仿真器。

```bash
# 示例路径，请替换为您自己的Autoware工作空间路径
source ~/autoware/install/setup.bash
ros2 launch autoware_launch planning_simulator.launch.xml
```

**注意**: 等待Autoware完全加载，直到您在RViz中看到ego车辆和地图。

### 步骤 3: 启动本测试系统

打开第三个终端，进入项目脚本目录并运行启动器。

```bash
cd _integrated_scenario_test/scripts
chmod +x enhanced_integrated_scenario_launcher.sh
./enhanced_integrated_scenario_launcher.sh
```

## 5. 使用交互式菜单

系统启动后，您将看到以下主菜单：

```bash
==================== 主菜单 ====================
导航测试:
  1. 到达任务起点 (为NPC测试做准备)
  2. 点对点导航 (从当前位置到终点2)
  8. 位置准备 (导航到回环测试起点)

NPC环境测试:
  4. 完整NPC测试 (从起点到终点1，带5辆NPC)
  5. NPC路径测试 (无NPC，仅路径)

回环稳定性测试:
  3. 单次回环测试
  6. 3次回环测试
  7. N次回环/无限回环测试

系统:
  9. 清理所有CARLA车辆
  Q. 退出程序
==============================================
请输入您的选择:
```

**菜单选项详解**:

- **1. 到达任务起点**: 自动导航到NPC测试的官方起始点。这是执行`选项4`的前置步骤。
- **2. 点对点导航**: 一个简单的导航功能，让车辆从当前位置直接开到最终目标点。
- **8. 位置准备**: 自动导航到回环测试的起始点。这是执行`选项3, 6, 7`的前置步骤。
- **4. 完整NPC测试**: 核心测试功能。系统会生成5辆NPC车辆，然后命令ego车辆在复杂的交通流中进行导航。
- **5. NPC路径测试**: 与`选项4`使用相同的路径，但不生成NPC车辆，用于基准测试或路径验证。
- **3. 单次回环测试**: 车辆将沿着预设的多个路点行驶一圈，然后停止。用于测试系统在固定路线上的稳定性。
- **6. 3次回环测试**: 连续执行三次`选项3`的测试。
- **7. N次回环/无限回环测试**: 您可以自定义循环次数。如果输入0，则会进入无限循环模式，直到手动停止。
- **9. 清理所有CARLA车辆**: 一个实用的工具，用于手动清除仿真世界中所有由本系统生成的NPC车辆。
- **Q. 退出程序**: 安全地关闭所有组件并退出程序。

## 6. 实时键盘控制

在测试过程中，您可以使用以下快捷键进行实时干预：

- **T 键**: **(碰撞恢复)** 仅在NPC测试（选项4）中有效。当发生碰撞时，按下T键会立即移除所有NPC车辆，让ego车辆可以继续完成导航。
- **R 键**: **(重启测试)** 主要用于从回环测试模式快速切换回NPC测试模式。
- **P 键**: **(暂停/继续)** 暂停当前的导航或测试。再次按下则继续。
- **S 键**: **(停止)** 立即停止当前的测试任务（如回环测试），并返回主菜单。
- **Q 键**: **(全局退出)** 在任何时候按下Q键，都会安全地终止整个程序。

## 7. 配置文件详解 (`test_config.json`)

配置文件允许您深度自定义测试行为，而无需修改代码。

```json
{
  "carla_host": "localhost",
  "carla_port": 2000,
  "wait_before_engage": 2.0,
  "goal_distance_threshold": 8.0,
  "navigation_timeout": 120.0,
  "num_vehicles": 5,
  "vehicle_model": "vehicle.tesla.model3",
  "log_level": "INFO",
  "positions": {
    "npc_test_start": {"x": 100.0, "y": 0.0},
    "task_goal_1": {"x": 200.0, "y": 50.0},
    "task_goal_2": {"x": 0.0, "y": 100.0}
  },
  "loop_waypoints": [
    {"x": 50.0, "y": 50.0},
    {"x": 50.0, "y": -50.0},
    {"x": -50.0, "y": -50.0}
  ]
}
```

- `goal_distance_threshold`: 判断车辆是否“到达”目标的距离（米）。
- `navigation_timeout`: 单次导航任务的超时时间（秒）。
- `num_vehicles`: 在NPC测试中生成的车辆数量。
- `positions`: 定义各个关键点的坐标。
- `loop_waypoints`: 定义回环测试的路径点列表。

## 8. 日志与故障排除

### 查看日志

所有的测试活动都会被记录下来。日志文件位于 `_integrated_scenario_test/logs/` 目录下，并以时间戳命名，例如 `test_run_2025-08-07_10-30-00.log`。

### 开启调试模式

如果遇到问题，您可以通过命令行参数以`DEBUG`模式启动程序，这将输出更详细的日志信息。

```bash
python3 enhanced_integrated_scenario_test.py --log-level DEBUG
```

### 常见问题 (FAQ)

- **问: 脚本提示无法连接到CARLA？**
  **答**: 请确认CARLA服务器已成功启动，并且配置文件中的`carla_host`和`carla_port`与服务器设置一致。

- **问: Autoware没有反应？**
  **答**: 请确认Autoware已完全加载，并且ROS2环境已正确设置（特别是`ROS_DOMAIN_ID`）。

- **问: 车辆在原地不动？**
  **答**: 检查`wait_before_engage`时间是否足够。同时，在RViz中确认目标点是否在可行驶区域内。

---
感谢使用本系统！祝您测试顺利！
