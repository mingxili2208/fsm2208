#!/bin/bash

# 获取当前脚本的目录（即bash文件夹的路径）
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"

# 导航到Python脚本所在的目录
PYTHON_SCRIPT_DIR="$SCRIPT_DIR/../scripts/test"
PYTHON_SCRIPT="$PYTHON_SCRIPT_DIR/scenario_test4.py"

# Configuration file for CARLA traffic generation
CONFIG_FILE="$SCRIPT_DIR/traffic_config.json"

# Default values
VEHICLES=5
WALKERS=0
HOST="127.0.0.1"
PORT=2000
SEED=$(date +%s)
SAFE_MODE=true
BATCH_SPAWN=false

# Parse command line arguments
while getopts "v:w:h:p:c:s:b" opt; do
  case $opt in
    v) VEHICLES=$OPTARG ;;
    w) WALKERS=$OPTARG ;;
    h) HOST=$OPTARG ;;
    p) PORT=$OPTARG ;;
    c) CONFIG_PATH=$OPTARG ;;
    s) SEED=$OPTARG ;;
    b) BATCH_SPAWN=true ;;
    *) echo "Usage: $0 [-v vehicles] [-w walkers] [-h host] [-p port] [-c config_path] [-s seed] [-b batch_spawn]" >&2
       exit 1 ;;
  esac
done

# 创建配置文件 - 包含完整设置
cat > $CONFIG_FILE << EOF
{
  "number-of-vehicles": $VEHICLES,
  "number-of-walkers": $WALKERS,
  "host": "$HOST",
  "port": $PORT,
  "safe": $SAFE_MODE,
  "filterv": "vehicle.*",
  "generationv": "All",
  "filterw": "walker.pedestrian.*",
  "generationw": "2",
  "asynch": false,
  "seed": $SEED
}
EOF

echo "配置文件已创建: $CONFIG_FILE"
echo "Starting CARLA traffic simulation with $VEHICLES vehicles and $WALKERS pedestrians"
echo "Python script path: $PYTHON_SCRIPT"
echo "Host: $HOST, Port: $PORT, Seed: $SEED"

# 注意捕获Ctrl+C信号，确保清理
cleanup() {
    echo "接收到中断信号，正在清理..."
    rm -f "$CONFIG_FILE"
    # 给Python脚本时间进行清理
    sleep 1
    exit 1
}

trap cleanup SIGINT SIGTERM

# 设置批处理模式参数
BATCH_ARG=""
if [ "$BATCH_SPAWN" = true ]; then
    BATCH_ARG="--batch-spawn"
    echo "使用批量生成模式"
fi

# 运行Python脚本 - 使用配置文件或命令行参数
if [ -f "$CONFIG_FILE" ]; then
    echo "通过配置文件运行: $CONFIG_FILE"
    python3 "$PYTHON_SCRIPT" --config "$CONFIG_FILE" $BATCH_ARG
else
    echo "通过命令行参数运行"
    python3 "$PYTHON_SCRIPT" --host "$HOST" --port "$PORT" --seed "$SEED" --num-vehicles "$VEHICLES" $BATCH_ARG
fi

# 检查Python脚本退出状态
SCRIPT_STATUS=$?
if [ $SCRIPT_STATUS -ne 0 ]; then
    echo "Python脚本异常退出，状态码: $SCRIPT_STATUS"
fi

# 清理配置文件
if [ -f "$CONFIG_FILE" ]; then
    rm "$CONFIG_FILE"
    echo "已删除配置文件: $CONFIG_FILE"
fi

echo "脚本执行完成"
exit $SCRIPT_STATUS