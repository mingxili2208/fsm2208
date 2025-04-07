#!/bin/bash

# Configuration for NPC vehicles in CARLA
CONFIG_FILE="npc_config.json"

# Default values
NUM_VEHICLES=5
HOST="127.0.0.1"
PORT=2000
NAV_STYLE="circular"
TARGET_SPEED=20.0
SAFETY_DISTANCE=3.0
CHECK_INTERVAL=100

# Parse command line arguments
while getopts "n:h:p:s:t:d:c:" opt; do
  case $opt in
    n) NUM_VEHICLES=$OPTARG ;;
    h) HOST=$OPTARG ;;
    p) PORT=$OPTARG ;;
    s) NAV_STYLE=$OPTARG ;;
    t) TARGET_SPEED=$OPTARG ;;
    d) SAFETY_DISTANCE=$OPTARG ;;
    c) CHECK_INTERVAL=$OPTARG ;;
    *) echo "Usage: $0 [-n num_vehicles] [-h host] [-p port] [-s nav_style] [-t target_speed] [-d safety_distance] [-c check_interval]" >&2
       exit 1 ;;
  esac
done

# Create config file with the specified parameters
cat > $CONFIG_FILE << EOF
{
  "num-vehicles": $NUM_VEHICLES,
  "host": "$HOST",
  "port": $PORT,
  "nav-style": "$NAV_STYLE",
  "target-speed": $TARGET_SPEED,
  "safety-distance": $SAFETY_DISTANCE,
  "check-interval": $CHECK_INTERVAL,
  "safe": true,
  "asynch": false
}
EOF

echo "Starting CARLA simulation with $NUM_VEHICLES NPC vehicles"
echo "Navigation style: $NAV_STYLE"
echo "Target speed: $TARGET_SPEED km/h"
echo "Safety distance: $SAFETY_DISTANCE meters"
echo "Check interval: $CHECK_INTERVAL ticks"

cleanup() {
  echo "Cleaning up..."
  rm -f $CONFIG_FILE
}

# Register cleanup function
trap cleanup EXIT INT TERM

# Run the Python script with the config file
python3 /home/cityu-fsm-lab-carla/lmx/packages/9_npc_car/scripts/test/pseudo_random_test.py --config-file $CONFIG_FILE

# Cleanup happens automatically through trap