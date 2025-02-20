# this is the cmd of the sync_mode

## init resources monitor

cd /home/cityu-fsm-lab-carla/lmx/packages/7_autoware_comm/scripts

python3 ./monitor/test_sys_monitor.py

## init carla

```sh
cd $CARLA_ROOT
make launch
```

then have to click the button play

## init npc_world

1. init terminal

    cd  /home/cityu-fsm-lab-carla/lmx/packages/9_npc_car/bash

    source ./initial_npc_param.bash

2. init npc world(only if not init any world)

    cd  /home/cityu-fsm-lab-carla/lmx/packages/9_npc_car/bash

    bash ./npc_world_init.bash

3. init npc_car

    cd  /home/cityu-fsm-lab-carla/lmx/packages/9_npc_car/bash

    bash ./npc_car_init.bash

## init navigation

1. if in ps2

2. if in waypoints(spawn_points)

## init sandbox_car

1. start steam vr
2. start vr_pose transmit

    cd /home/cityu-fsm-lab-carla/lmx/packages/7_autoware_comm/bash

    source initial_config.bash

    **this is the compensated one**

    cd /home/cityu-fsm-lab-carla/lmx/packages/8_positioning_compensation

    python3 test8.py

    **this is the original pose**

    cd /home/cityu-fsm-lab-carla/lmx/packages/7_autoware_comm/scripts/ekf

    python3 test_fix_ekf.py

3. start serial transmit

    - check if the serial is not online

    ```bash
    ls /dev/ttyUSB0

    sudo chmod 777 /dev/ttyUSB0

    ```

    - start transmit

    ```bash

    cd /home/cityu-fsm-lab-carla/lmx/packages/7_autoware_comm/bash

    source initial_config.bash 

    cd /home/cityu-fsm-lab-carla/lmx/packages/7_autoware_comm/scripts/debug

    python3 debug5_serial.py
    
    ```

4. start autoware vehicle agent

    ```bash
    cd ~/Workspace/Carla/op_carla/op_bridge/op_scripts/fsm_lab_simulation/

    ./debug_run_vehicle_ros2.sh

    ```

5. start car update

    ```bash
    cd /home/cityu-fsm-lab-carla/lmx/packages/7_autoware_comm/bash

    source initial_config.bash 

    cd /home/cityu-fsm-lab-carla/lmx/packages/7_autoware_comm/scripts/debug

   python3 debug_update_vehicle_3.py
