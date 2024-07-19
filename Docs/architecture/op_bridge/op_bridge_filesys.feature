op_bridge/
|-- data/
|   |-- all_towns_traffic_scenarios_public.json         # Traffic Configuration
|   |-- routes_*
|   #waypoints as path <waypoint pitch="360.0" roll="0.0" x="338.7027893066406" y="226.75003051757812" yaw="269.9790954589844" z="0.0" />
|-- leaderboard/                                        # Autonomous driving algorithm or system
|-- op_bridge/                                          # basic 
|   |-- fsm_lab_simulation/                             # handlers_codes for carla
|   |   |-- messages/
|   |   |   |-- sensors/
|   |   |   |   |-- lidar.py                            # Handling and creating PointCloud2 messages in ROS 2
|   |   |-- testing_scripts/
|   |   |   |-- debug_tracker.py                        # print tracker coordinates
|   |   |   |-- init_transform_publisher.py             # Publish the initial vehicle position from carla
|   |   |   |-- tracker_init_transform_publisher.py     # Publishing the initial position of the vehicle using the Vive Tracker 
|   |   |   |-- transform_publisher.py                  # Real-time release of vehicle location from carla    
|   |   |-- ego_vehicle_initializer.py                  # Customized ROS autonomous agent interface to control the ego vehicle
|   |   |-- trans_utils.py                              # Carla--ROS--Sand       
|   |   |-- vehicle_launcher.py                         # launch vehicle
|   |   |-- vive_tracker.py                             # vive_tracker to get Sand coordinates    
|   |   |-- world_launcher.py                           # launch world 
|   |-- op_bridge_ros2.py                               # A bridge program to connect the CARLA simulator and the ROS 2 autonomous driving agent
|   |-- op_ros2_agent.py                                # This module provides a ROS autonomous agent interface to control the ego vehicle via a ROS stack
|   |-- sensors.json                                    # sensor Configuration
|-- op_scripts/                                         # bash script to run fsm_lab_simulation
|-- scripts/                                            #  Create a human-readable version of the leaderboard score