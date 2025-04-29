# this is the guideline of how to use the Iteratively_process_data

1. init auto_tracker_app

    ```bash

    cd /home/cityu-fsm-lab-carla/lmx/packages/1_vive_tracker/auto_tracker_app

    python3 app.py

    ```

2. Collect data according to the instructions;
    first collect 8 sets of angle changes, then collect at least 4 long straight line data
    Note that the direction of the vehicle needs to be recorded
3. find the original data of euler & cordinate
4. Move the data to the ata directory of Iteratively_process_data and use the Iteratively_process.py script to calculate
5. Move the two transformation matrices of the final result to 8
