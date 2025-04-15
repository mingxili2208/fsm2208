# this is the step for open multi_npc test

## 0. start monitor

open a new terminal

type in

```bash
    cd /home/cityu-fsm-lab-carla/lmx/packages/7_autoware_comm/scripts

    python3 ./monitor/test_sys_monitor.py

```

## 1. start carla ue4 editor

open a new terminal tap in the terminal of monitor(ctrl + shift + T)

```sh
    cd $CARLA_ROOT
    make launch
```

then have to click the button play (this do not need to be Automatic Implementation)

## 2. start steam vr

open the client of steam vr ,making sure that the steam is in the mode of no head mode;

## 2.1. open coordinate transformer forward program

open a new terminal

```bash
    cd /home/cityu-fsm-lab-carla/lmx/packages/7_autoware_comm/bash

    source initial_config.bash

    cd /home/cityu-fsm-lab-carla/lmx/packages/8_positioning_compensation

    python3 _transformer_test.py
```

## 3. enable bash

open a new terminal tap in the terminal of monitor(which include monitor & carla)

```bash

    cd /home/cityu-fsm-lab-carla/lmx/packages/7_autoware_comm/bash

    bash ./test_run_car.sh

```

## 4. open serial forward program

open a new terminal tab at above one(coordinate transformer), typing in

```bash
    cd /home/cityu-fsm-lab-carla/lmx/packages/7_autoware_comm/bash

    source initial_config.bash

    cd /home/cityu-fsm-lab-carla/lmx/packages/7_autoware_comm/scripts/test

    python3 test_serail_tms5.py
```

## 5. open coordinate update program

open a new terminal tab at above one(the same as coordinate transformer & serial_forward), typing in

```bash
    cd /home/cityu-fsm-lab-carla/lmx/packages/7_autoware_comm/bash

    source initial_config.bash

    cd /home/cityu-fsm-lab-carla/lmx/packages/7_autoware_comm/scripts/debug

    python3 debug_update_vehicle_3.py 
```

## 6. open new cameral window

open a new terminal, typing in

```sh

    cd  /home/cityu-fsm-lab-carla/lmx/packages/9_npc_car/bash
    
    bash new_camera_pygame.bash

```

## 7. set a new destination for autoware

set a goal for autoware in rviz

## 8. open npc bash

open a new teminal tap of above one (new_camera_pygame), typing in

```sh

    bash scenario_test.sh

```
