# 分离式解决方案

数据采集阶段 (collect_data.py)
    ├── 异步LiDAR采集线程 (10Hz)
    ├── 异步Tracker采集线程 (50Hz)
    ├── 数据聚合线程
    └── 原始数据存储 (.npz格式)
         ↓
数据处理阶段 (process_data.py)
    ├── 数据同步与插值
    ├── 自动数据分段
    ├── Savitzky-Golay滤波
    ├── 性能分析
    └── 多格式输出

核心特性
架构纯粹性

采集脚本：只负责数据收集，不做任何处理决策
处理脚本：只负责算法处理，不依赖硬件
真正异步采集

LiDAR和Tracker独立线程采集
消除系统性时间延迟
高精度时间戳记录
高级数据同步

基于插值的时间对齐
自动异常值检测
同步质量统计
智能数据分段

基于时间间隔的自动分段
最小段长度保护
分段质量验证
先进滤波算法

Savitzky-Golay滤波器
自动参数优化
分段独立处理
全面可视化分析

静态图表 (PNG)
交互式图表 (HTML)
3D轨迹可视化
残差分析
使用流程
配置系统

bash

复制
# 编辑采集配置
vim config.json

# 编辑处理配置
vim process_config.json
数据采集

bash

复制
python collect_data.py
# 按 'S' 开始采集
# 按 'E' 结束并保存
数据处理

bash

复制
# 基本处理
python process_data.py raw_data/raw_data_20241125_143022.npz

# 自定义参数
python process_data.py data.npz --window 9 --poly 3
输出文件
原始数据: raw_data_YYYYMMDD_HHMMSS.npz
同步数据: synchronized_data_YYYYMMDD_HHMMSS.csv
滤波数据: filtered_data_YYYYMMDD_HHMMSS.csv
静态图表: processing_results_YYYYMMDD_HHMMSS.png
交互图表: interactive_results_YYYYMMDD_HHMMSS.html
分析报告: analysis_report_YYYYMMDD_HHMMSS.txt