op_agent/                                                   # 项目根目录
|-- autoware_carla_launch/                                  # autoware launchs
|   |-- maincarla_autoware_sensors_interface.xml            # launch for interface&lidar_cloud&rviz2
|   |-- configcarla_simulator_carla.launch.xml              # launch for simulator with carla
|       `-- essential parameters                            # path for map&vehicle&sensor&data              
|       `-- launch nodes carla_pointcloud & op_fix2pose & rviz2
|       `-- launch autoware 
|   |-- carla_simulator_fsm_lab.launch                      # without op_fix2pose compare2above
|-- autoware-contents/                                      # basic setup for autoware 
|-- darknet/                                                # yolo
|-- hdmaps/                                                 # Road network description file---for opendrive
|-- rviz/                                                   # rviz setup for carla
|-- start_ros2.sh                                           # source setup for launch 
    `-- set of role&map&explore&route                                        