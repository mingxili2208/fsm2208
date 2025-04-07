#!/bin/bash

# Configuration file for CARLA traffic generation
CONFIG_FILE="traffic_config.json"

# Default values
VEHICLES=10
WALKERS=0
HOST="127.0.0.1"
PORT=2000

# Parse command line arguments
while getopts "v:w:h:p:c:" opt; do
  case $opt in
    v) VEHICLES=$OPTARG ;;
    w) WALKERS=$OPTARG ;;
    h) HOST=$OPTARG ;;
    p) PORT=$OPTARG ;;
    c) CONFIG_PATH=$OPTARG ;;
    *) echo "Usage: $0 [-v vehicles] [-w walkers] [-h host] [-p port] [-c config_path]" >&2
       exit 1 ;;
  esac
done

# Create config file with the specified parameters
cat > $CONFIG_FILE << EOF
{
  "number-of-vehicles": $VEHICLES,
  "number-of-walkers": $WALKERS,
  "host": "$HOST",
  "port": $PORT,
  "safe": true,
  "filterv": "vehicle.*",
  "generationv": "All",
  "filterw": "walker.pedestrian.*",
  "generationw": "2",
  "asynch": false,
  "seed": $(date +%s)
}
EOF

echo "Starting CARLA traffic simulation with $VEHICLES vehicles and $WALKERS pedestrians"

# Run the Python script with the config file
python3 /home/cityu-fsm-lab-carla/lmx/packages/9_npc_car/scripts/test/mutiple_npc_test.py --config-file $CONFIG_FILE

# Clean up the config file when done
rm $CONFIG_FILE