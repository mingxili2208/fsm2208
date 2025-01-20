#!/bin/bash

# 定义日志文件路径
LOG_FILE="./gpustat_log.log"  # 请将此路径替换为您希望存储日志的实际路径

# 设置时间间隔（秒）
INTERVAL=0.1

# 定义函数以处理退出信号，确保脚本能优雅地关闭
cleanup() {
    echo "Logging stopped at $(date "+%Y-%m-%d %H:%M:%S")." | tee -a "$LOG_FILE"
    exit 0
}

# 捕捉退出信号（如 Ctrl+C）以调用 cleanup 函数
trap cleanup SIGINT SIGTERM

# 定义函数以记录 gpustat 输出带有时间戳
log_gpustat() {
    echo "Starting gpustat logging at $(date "+%Y-%m-%d %H:%M:%S")." | tee -a "$LOG_FILE"
    while true; do
        # 获取当前时间戳，精确到毫秒
        TIMESTAMP=$(date "+%Y-%m-%d %H:%M:%S.%3N")
        
        # 运行 gpustat 并捕获输出
        GPUSTAT_OUTPUT=$(gpustat)

        # 将时间戳和 gpustat 输出写入日志文件
        echo "[$TIMESTAMP] $GPUSTAT_OUTPUT" >> "$LOG_FILE"
        
        # 等待指定的时间间隔
        sleep "$INTERVAL"
    done
}

# 启动 gpustat 记录函数
log_gpustat