# this is the prototype_design for the road map

1. runcar
2. start the transformation node
3. start the follow node

## questions

1. error zero points  for carla & sandbox
   **1. the middle of the SandBox is the zero point of the carla**
2. need to add scare
3. set transform will lead to a jump and flash
4. the code needs to be rewritten
5. the framework needs to be reconstructed
6. at carla the zero of yaw is **the derection along the positive x-axis**

## implement

1. the coordinates in ros2 is using the z axis as the upwards
2. the steamvr is using the y axis as the the upwards
3. the carla is using the z axis as the upwards
4. for the 1,2,3 we have to set the coordinate of carla and ros2 as the same
5. steamvr to ros_coor needs a 30x magnification

## data

1
    Carla
    location: x=9.7408, y=37.0096, z=0.0016;
    rotation: pitch=6.830188794992864e-06, yaw=-72.99797058105469, roll=-6.103515261202119e-05
    SandBox
    x=1.915,y=-1.15,z=0.0016

2
    location: x=17.2589, y=33.2976, z=0.0016;
    rotation: pitch=0.00010245283192489296, yaw=8.195829391479492, roll=-6.103515261202119e-05
    x=2.145,y=-1.275,z=0.0016

3
    location: x=24.5744, y=28.7530, z=0.0016;
    rotation: pitch=6.830188794992864e-06, yaw=-38.61204147338867, roll=-6.103515261202119e-05
    x=2.360,y=-1.415,z=0.0016

on the lawn
    location: x=-33.50742721557617, y=-34.10329818725586, z=0.1695745885372162;
    rotation: pitch=0.33282142877578735, yaw=-20.795249938964844, roll=-0.55413818359375

make a transformation between carla & sandbox
