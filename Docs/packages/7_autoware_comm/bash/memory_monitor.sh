#!/bin/bash

# 输出文件路径
output_file="memory_change.log"

# 设置监控间隔（秒）
interval=0.05  # 每0.25秒检查一次

# 设置内存变动的阈值（MB）
threshold=100  # 只有内存使用变化超过100MB时才记录

# 获取初始的已用内存（单位：MB）
previous_used=$(free -m | awk '/^Mem:/ {print $3}')

echo "开始监控内存变化，阈值为 $threshold MB" > $output_file

# 持续监控内存使用情况
while true; do
    # 获取当前的已用内存（单位：MB）
    current_used=$(free -m | awk '/^Mem:/ {print $3}')
    
    # 计算内存使用的变化量
    memory_change=$((current_used - previous_used))
    memory_change_abs=$(echo $memory_change | awk '{print ($1 >= 0 ? $1 : -$1)}')  # 取绝对值

    # 仅当内存变化超过阈值时记录
    if (( memory_change_abs >= threshold )); then
        # 获取当前时间
        timestamp=$(date +"%Y-%m-%d %H:%M:%S")

        # 记录内存使用情况
        echo "[$timestamp] Memory change detected: $memory_change MB" >> $output_file
        free -h >> $output_file
        echo "---------------------------------" >> $output_file

        # 更新 previous_used
        previous_used=$current_used
    fi

    # 等待指定的间隔时间
    sleep $interval
done