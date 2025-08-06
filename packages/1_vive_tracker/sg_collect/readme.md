LiDAR-Tracker数据采集与处理系统说明文档
系统概述
本系统是一套用于LiDAR距离传感器和Vive Tracker位姿数据采集与处理的完整解决方案。采用分离式架构设计，将数据采集和数据处理完全分离，确保系统的稳定性、可扩展性和科研工作的可重现性。

设计理念
1. 分离式架构
采集阶段：专注于硬件数据收集，保证数据完整性和时间精度
处理阶段：专注于算法优化，支持反复迭代和参数调优
2. 纯粹性原则
采集脚本：收集一切，决策无关（Collect everything, decide nothing）
处理脚本：将原始数据转化为洞察（Transform raw data into insights）
3. 科研友好
一次采集，多次处理
参数可配置
结果可重现
系统组成
1. 数据采集模块 (collect_data.py)
功能特性
异步多线程采集：LiDAR和Tracker独立线程，消除时间耦合
高精度时间戳：微秒级时间记录，支持精确同步
实时状态监控：采集过程中提供状态反馈和错误诊断
数据完整性保证：校验和验证、连接恢复机制
硬件支持
LiDAR传感器：A69协议距离传感器，串口通信
Vive Tracker：SteamVR追踪系统，6DOF位姿数据
配置参数
json

复制
{
  "serial_port": "/dev/ttyUSB0",     // 串口设备路径
  "baud_rate": 9600,                // 串口波特率
  "tracker_name": "tracker_1",      // Tracker设备名称
  "lidar_target_hz": 10,            // LiDAR目标采样率
  "tracker_target_hz": 50,          // Tracker目标采样率
  "collection_timeout": 3600,       // 最大采集时长（秒）
  "output_dir": "raw_data"          // 数据输出目录
}
操作控制
'S'键：开始数据采集
'E'键：结束采集并保存数据
'T'键：显示当前采集状态
'Q'键：退出程序
2. 数据处理模块 (process_data.py)
核心算法
数据同步与插值
线性插值算法：基于时间戳的高精度数据对齐
异常值检测：基于统计方法的噪声点识别
同步质量评估：提供详细的同步成功率统计
自动数据分段
时间间隔检测：识别数据丢失和人工暂停点
最小段长度保护：确保每个段有足够的数据点用于滤波
分段质量验证：提供每个段的统计信息
Savitzky-Golay滤波
多项式拟合：保持信号形状的同时降低噪声
参数自动优化：基于残差方差的最优参数搜索
分段独立处理：避免段边界效应
可视化输出
静态图表 (PNG格式)
距离数据对比图（滤波前后）
位姿坐标时间序列图
3D轨迹投影图
滤波残差分析图
数据分布直方图
交互式图表 (HTML格式)
基于Plotly的动态可视化
支持缩放、平移、悬停信息
3D轨迹交互式浏览
多数据源对比显示
配置参数
json

复制
{
  "synchronization": {
    "interpolation_tolerance_ms": 100,   // 插值容忍时间间隔
    "outlier_detection": true,           // 是否启用异常值检测
    "outlier_threshold_sigma": 3.0       // 异常值检测阈值
  },
  "segmentation": {
    "max_time_gap_ms": 150,             // 分段时间间隔阈值
    "min_segment_length": 10            // 最小段长度
  },
  "filtering": {
    "sg_window_length": 7,              // SG滤波器窗口长度
    "sg_polyorder": 2,                  // SG滤波器多项式阶数
    "auto_optimize_parameters": true     // 是否自动优化参数
  },
  "visualization": {
    "generate_interactive_plots": true,  // 生成交互式图表
    "generate_static_plots": true,       // 生成静态图表
    "plot_3d_trajectories": true,       // 绘制3D轨迹
    "show_segment_boundaries": true      // 显示分段边界
  },
  "output": {
    "save_synchronized_csv": true,      // 保存同步数据
    "save_filtered_csv": true,          // 保存滤波数据
    "output_dir": "processed_data"      // 输出目录
  }
}

使用指南
1. 环境准备
系统要求
Python 3.7+
Linux/Windows系统
USB串口支持
SteamVR运行时环境
依赖安装
bash

复制
pip install numpy matplotlib plotly pandas scipy serial pynput
硬件连接
连接LiDAR传感器到USB串口
启动SteamVR基站和Tracker
确认设备识别正常
2. 数据采集流程
步骤1：配置检查
bash

复制
# 检查并编辑采集配置
cat config.json
vim config.json  # 如需修改
步骤2：启动采集
bash

复制
python collect_data.py
步骤3：采集操作
程序启动后，显示硬件连接状态
按'S'键开始数据采集
进行需要的运动或测量
按'E'键结束采集并自动保存
步骤4：数据验证
检查输出的.npz文件
确认采集时长和数据点数量
查看采集过程中的错误日志
3. 数据处理流程
基本处理
bash

复制
python process_data.py raw_data/raw_data_20241125_143022.npz
高级处理选项
bash

复制
# 自定义滤波参数
python process_data.py data.npz --window 9 --poly 3

# 跳过交互式图表生成（节省时间）
python process_data.py data.npz --no-interactive

# 使用自定义配置文件
python process_data.py data.npz -c custom_config.json
处理结果解读
同步成功率：理想情况下应>95%
分段数量：反映数据连续性
滤波改善度：标准差降低百分比
可视化图表：直观判断滤波效果
输出文件说明
1. 原始数据文件
格式：NPZ压缩格式
命名：raw_data_YYYYMMDD_HHMMSS.npz
内容：
lidar_data：LiDAR距离数据数组
tracker_data：Tracker位姿数据数组
metadata：采集元信息
2. 处理结果文件
同步数据：synchronized_data_*.csv - 插值对齐后的数据
滤波数据：filtered_data_*.csv - SG滤波后的最终数据
分析报告：analysis_report_*.txt - 详细处理统计
3. 可视化文件
静态图表：processing_results_*.png - 高质量分析图表
交互图表：interactive_results_*.html - 可交互的在线图表
技术特性
1. 时间同步精度
微秒级时间戳记录
线性插值算法，理论精度优于1ms
同步容错机制，处理网络抖动
2. 滤波性能
Savitzky-Golay滤波器保持信号形状
自动参数优化，适应不同运动模式
分段处理避免边界效应
3. 数据完整性
校验和验证确保数据正确性
连接恢复机制处理通信中断
异常值检测识别硬件故障
4. 可扩展性
模块化设计，易于添加新传感器
配置文件驱动，无需修改代码
标准数据格式，便于后续分析
故障排除
1. 采集问题
串口连接失败：检查设备路径和权限
Tracker无法识别：确认SteamVR运行状态
数据采集中断：检查USB连接稳定性
2. 处理问题
同步失败：检查时间重叠区间
滤波效果差：调整窗口长度和多项式阶数
内存不足：处理大文件时增加虚拟内存
3. 性能优化
提高采集稳定性：降低目标采样率
改善滤波效果：增加数据采集时长
加速处理速度：禁用交互式图表生成
最佳实践
1. 数据采集
采集前进行设备连接测试
保持稳定的USB连接
避免在采集过程中操作其他程序
2. 数据处理
先用默认参数处理，再根据结果调优
重要数据建议备份原始文件
批量处理时使用脚本自动化
3. 结果分析
重点关注同步成功率和滤波改善度
使用交互式图表进行详细分析
结合3D轨迹图判断空间运动质量
这套系统为LiDAR-Tracker数据的采集、处理和分析提供了完整的解决方案，特别适合需要高精度位姿测量的科研和工程应用。