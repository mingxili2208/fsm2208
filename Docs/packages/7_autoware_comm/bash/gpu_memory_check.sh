#!/bin/bash

# 输出文件路径
output_file="gpu_memory_change.log"

# 设置监控间隔（秒）
interval=0.05  # 每0.05秒检查一次

# 设置显存变动的阈值（MB）
threshold=200  # 只有显存使用变化超过100MB时才记录

# GPU ID (假设监控第0块GPU)
gpu_id=0

# 获取初始的已用显存（单位：MB）
previous_used=$(nvidia-smi --id=$gpu_id --query-gpu=memory.used --format=csv,noheader,nounits)

echo "开始监控GPU显存变化，阈值为 $threshold MB" > $output_file

# 持续监控显存使用情况
while true; do
    # 获取当前的已用显存（单位：MB）
    current_used=$(nvidia-smi --id=$gpu_id --query-gpu=memory.used --format=csv,noheader,nounits)
    
    # 计算显存使用的变化量
    memory_change=$((current_used - previous_used))
    memory_change_abs=$(echo $memory_change | awk '{print ($1 >= 0 ? $1 : -$1)}')  # 取绝对值

    # 仅当显存变化超过阈值时记录
    if (( memory_change_abs >= threshold )); then
        # 获取当前时间
        timestamp=$(date +"%Y-%m-%d %H:%M:%S")

        # 记录显存使用情况
        echo "[$timestamp] GPU Memory change detected: $memory_change MB" >> $output_file
        nvidia-smi --id=$gpu_id >> $output_file
        echo "---------------------------------" >> $output_file

        # 更新 previous_used
        previous_used=$current_used
    fi

    # 等待指定的间隔时间
    sleep $interval
done