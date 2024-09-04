# this file is description of how to start the carla_simulator

1. start carla

    ```bash
    cd $CARLA_ROOT
    make launch
    ```

    then have to click the button play

2. run_vehicle

    ```s
    cd ./Workspace/Carla/op_carla/op_bridge/op_scripts/fsm_lab_simulation/
    ./run_vehicle_ros2.sh
    ```

    this is the containings of run_vehicle_ros2.sh

    ```s
    #############################
    # Launch the Carla Ego-Vehicle
    #############################

    export SIMULATOR_LOCAL_HOST="localhost"
    export SIMULATOR_PORT="2000"
    export TRAFFIC_MANAGER_PORT="8000"
    export TEAM_AGENT=${OP_BRIDGE_ROOT}/op_bridge/fsm_lab_simulation/ego_vehicle_initializer.py
    export PYTHONPATH="${CARLA_ROOT}/PythonAPI/carla/":"${SCENARIO_RUNNER_ROOT}":"${OP_BRIDGE_ROOT}":${PYTHONPATH}
    export AGENT_FRAME_RATE="20"
    export REMOTE_CONNECTION="False"
    # Autonomous actor default role_name and type
    export AGENT_ROLE_NAME="pygame-adtruck"
    export AGENT_MODEL_TYPE="vehicle.tesla.model3"

    # modes are 
    #   * "leaderboard" : when runner the leaderboard (route based) scenario collections 
    #   * "srunner" : when scenario is loaded using scenario runner, and only agent is attached
    #   * "free" : when loading empty map only , either carla town or any OpenDRIVE map 
    export OP_BRIDGE_MODE="free"

    # CARLA town name or custom OpenDRIVE absolute path, when BRIDGE_MODE is free 
    export FREE_MAP_NAME="fsm_lab_sandbox_right_hand_driving_scene"

    # Spawn point for the autonomous agent, when BRIDGE_MODE is free 
    # "x,y,z,roll,pitch,yaw"
    # Empty string means random starting position
    # export FREE_AGENT_POSE="175.4,195.14,0,0,0,180" 
    # export FREE_AGENT_POSE="88.6,-226,0,0,0,0" 
    export FREE_AGENT_POSE="0"

    # Set controller. ["autoware", "pygame", "follow", "teleop"]
    export CONTROL_MODE="pygame"

    # Set the running mode. ["normal", "record"]
    export RUNNING_MODE="normal"

    source /opt/ros/humble/setup.bash
    export ROS_DOMAIN_ID=46

    if [ ${CONTROL_MODE} = "autoware" ]
    then
        source ${AUTOWARE_ROOT}/install/setup.bash
        if [[ ${FREE_MAP_NAME} == *fsm_lab* ]]
        then
            source ${OP_ROS_PLUGINS_ROOT}/install/setup.bash
            echo "Apply the official map in FSM Lab."
        else
            echo "Apply the official map in Carla Simulator."
        fi
        ros2 run rviz2 rviz2 -d ${OP_AGENT_ROOT}/rviz/carla_autoware.rviz -s ${OP_AGENT_ROOT}/rviz/image/autoware.png & 
        python3 ${OP_BRIDGE_ROOT}/op_bridge/fsm_lab_simulation/world_launcher.py
        python3 ${OP_BRIDGE_ROOT}/op_bridge/fsm_lab_simulation/vehicle_launcher.py
    else
        python3 ${OP_BRIDGE_ROOT}/op_bridge/fsm_lab_simulation/world_launcher.py
        python3 ${OP_BRIDGE_ROOT}/op_bridge/fsm_lab_simulation/vehicle_launcher.py
    fi

    ```

3. run_npc_car

   ```s
   cd $CARLA_ROOT
   cd ./PythonAPI/examples
   python3 generate_traffic.py
   ```