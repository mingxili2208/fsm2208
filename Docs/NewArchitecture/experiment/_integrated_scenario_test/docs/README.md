# 增强版自动驾驶测试系统

一个集成CARLA仿真器和Autoware自动驾驶系统的综合测试平台，提供交互式菜单、NPC环境测试、回环导航测试和实时键盘控制功能。

## ✨ 主要特性

- 🎮 **交互式菜单系统** - 9种不同测试模式，用户友好的操作界面
- 🚦 **NPC环境测试** - 复杂交通环境下的自动驾驶性能测试
- 🔄 **回环导航测试** - 循环路径稳定性和一致性验证
- ⌨️ **实时键盘控制** - T/R/P/S/Q键实时干预和控制
- 🛡️ **智能恢复机制** - 碰撞检测、自动恢复和异常处理
- 📊 **完整日志系统** - 详细的测试记录、统计分析和性能监控
- ⚙️ **高度可配置** - JSON配置文件、命令行参数和环境变量支持
- 🧹 **资源管理** - 自动清理CARLA和ROS2资源，确保系统稳定

## 🚀 快速开始

### 环境要求

- Ubuntu 20.04/22.04
- Python 3.8+
- ROS2 Humble+
- CARLA 0.9.13+
- Autoware (已配置)

### 安装依赖

```bash
pip3 install pynput numpy transforms3d setproctitle psutil
```

### 启动系统

```bash
# 1. 启动CARLA服务器
cd $CARLA_ROOT && ./CarlaUE4.sh

# 2. 启动Autoware
source $AUTOWARE_ROOT/install/setup.bash
ros2 launch autoware_launch planning_simulator.launch.xml

# 3. 运行测试系统
cd _integrated_scenario_test/scripts
chmod +x enhanced_integrated_scenario_launcher.sh
./enhanced_integrated_scenario_launcher.sh
```

## 📋 测试模式

### 导航测试

1. **到达任务起点** - 导航到NPC测试起始位置
2. **点对点导航** - 直接导航到指定目标点
3. **位置准备** - 为特定测试做准备

### NPC环境测试

2. **完整NPC测试** - 在复杂交通环境下的导航测试
4. **NPC路径测试** - 执行NPC路径但不生成车辆

### 回环测试

3. **单次回环** - 一次完整的循环路径测试
4. **3次回环** - 连续执行3次回环测试
5. **N次回环** - 可配置次数或无限循环测试

## ⌨️ 键盘控制

- **T键** - 碰撞恢复：立即移除NPC车辆并继续导航
- **R键** - 重启测试：从回环测试切换回NPC测试
- **P键** - 暂停测试：临时暂停当前操作
- **S键** - 停止测试：结束当前测试序列
- **Q键** - 退出程序：完全退出系统

## 📁 项目结构

```bash
_integrated_scenario_test/
├── scripts/
│   ├── enhanced_integrated_scenario_launcher.sh  # Bash启动器
│   └── enhanced_integrated_scenario_test.py      # Python主程序
├── config/
│   └── test_config.json                          # 配置文件
├── docs/
│   ├── README.md                                 # 项目说明
│   ├── DESIGN.md                                 # 设计文档
│   └── USER_GUIDE.md                             # 使用手册
└── logs/                                         # 日志目录
```

## ⚙️ 配置选项

### 创建默认配置

```bash
python3 enhanced_integrated_scenario_test.py --create-default-config config/my_config.json
```

### 主要参数

- `wait_before_engage`: engage前等待时间 (默认: 2.0s)
- `goal_distance_threshold`: 目标到达距离 (默认: 8.0m)
- `num_vehicles`: NPC车辆数量 (默认: 5)
- `navigation_timeout`: 导航超时时间 (默认: 120.0s)

## 📊 测试流程

### NPC测试流程

```
起点检查 → 目标设置 → NPC生成 → 导航执行 → 碰撞监控 → 清理车辆
```

### 回环测试流程

```
起点检查 → 路点1 → 路点2 → 路点3 → 返回起点
```

## 🔧 故障排除

### 常见问题

- **CARLA连接失败**: 确认服务器启动和端口设置
- **ROS2通信问题**: 检查环境变量和Autoware状态
- **车辆生成失败**: 验证生成点可用性和车辆数量

### 调试模式

```bash
python3 enhanced_integrated_scenario_test.py --log-level DEBUG
```

## 📈 性能特性

- ⚡ **高响应性** - 实时键盘响应和快速状态更新
- 🛡️ **高可靠性** - 完善的异常处理和资源清理
- 📊 **可监控性** - 详细的日志记录和统计信息
- 🔧 **可维护性** - 模块化设计和清晰的接口

## 🤝 贡献

欢迎提交Issue和Pull Request！请确保:

- 遵循PEP 8代码规范
- 包含适当的测试用例
- 更新相关文档

## 📄 许可证

本项目基于 [MIT License] 开源。

## 👨‍💻 作者

**James LI**

- 邮箱: <james.li@example.com>
- 项目: Enhanced Autonomous Vehicle Testing System

## 📚 相关文档

- [设计说明](DESIGN.md) - 系统架构和设计理念
- [用户手册](USER_GUIDE.md) - 详细的使用说明和配置指南
- [MIT License] - 开源许可证详情

---

⭐ 如果这个项目对您有帮助，请给我们一个星标！

```
