# this is the guidelines how to use vive_tracker and combine it with the sandbox

1. coordinate_system of the vive_tracker

![coor_1](img/2024-07-09-14-30-59.png)

![coor_2](img/2024-07-09-14-31-22.png)

![coor_3](img/2024-07-05-10-12-00.png)

2. tracker_orientation

```python
import numpy as np
from scipy.spatial.transform import Rotation as R

#         r                   y                     p
#np.radians(-179.4732), np.radians(-145.5065), np.radians(-90.4384)
# 假设初始欧拉角为 (yaw1, pitch1, roll1)
    #6                                          r                       y                   p
   # Point([-1.3939, -1.3780, -1.0774], [np.radians(1.3633), np.radians(-55.5799), np.radians(90.2613)]),
   # Point([-1.3939, -1.3780, -1.0774], [np.radians(-178.4583), np.radians(-146.8788), np.radians(-89.9101)]),
   #roll=-178.4583，yaw=-146.8788，pitch=-89.9101
   # Point([-1.3939, -1.3780, -1.0774], [np.radians(1.5277), np.radians(34.5399), np.radians(89.8219)]),
yaw_z1, pitch_y1, roll_x1 =  -55.5799,90.2613,1.3633

# 假设要旋转的欧拉角为 (yaw2, pitch2, roll2)
yaw_z2, pitch_y2, roll_x2 = 90,0,0

# 假设tracker使用Z-Y-X的欧拉角合成顺序
rotation1 = R.from_euler('yxz', [pitch_y1,roll_x1, yaw_z1], degrees=True)
rotation2 = R.from_euler('yxz', [pitch_y2 ,roll_x2,yaw_z2], degrees=True)

# 获取旋转矩阵
matrix1 = rotation1.as_matrix()
matrix2 = rotation2.as_matrix()

# 将两个旋转矩阵相乘
rotated_matrix = np.dot(matrix2, matrix1)

# 从旋转矩阵创建一个新的 Rotation 对象
rotated_rotation = R.from_matrix(rotated_matrix)

# 将旋转后的矩阵转换回欧拉角
rotated_euler = rotated_rotation.as_euler('yxz', degrees=True)

# 打印旋转后的欧拉角
print("Rotated Euler Angles (yaw, pitch, roll):", rotated_euler)
#                                             pitch                 roll                   yaw

#Rotated Euler Angles (yaw, pitch, roll): [90.2613  1.3633 34.4201]
```

through this test we know that the rotation of the Euler is by the order of pitch roll yaw ---yxz (while   pitch_y, roll_x ,yaw_z in func R.from_euler)
In this proj, we know that the tracker useing a coordinate while the y is the height, so we have to change the input order of the Euler numpy like the following codes:


3. definition of the point_orientation

we need to define the coordinate_system of tracker and the orientation of how to turn it

the guidelines have given a defination of tracker_coordinate as the picture shows

![3.1](img/2024-07-10-09-57-36.png)

we may use this as the definition of the tracker_coordinate

then we define four derictions of the point_orientation

all the following disgree is describing the derictions of the the tracker_coordinate

3.1. y=0,p=0,r=0

the axis_orientation of the tracker_coordinate should be the same with the sandbox_coordinate
_orientation itself

![3.1-1](img/2024-07-10-17-59-59.png)

3.2. y=-90,p=30,r=0

![3.2-2](img/2024-07-10-17-46-11.png)

3.3. y=30,p=0,r=-20

![3.3-3](img/2024-07-10-17-47-00.png)

When choosing the direction, we should exclude the situation of pitch=90 as much as possible to avoid singular points.

At the same time, the azimuth angles obtained should be varied in all axes as much as possible, because the axes of the two coordinate systems are not parallel.

4. points in SandBox_coordinate

![4](img/2024-07-11-15-58-57.png)

Note that the points 1-9 are used as fitting data, the points 10-11 are used as testing data.

5. tracker_cmd

```shell

# 1. we have to start steam vr at first: start a new terminal then type the command

steam steamvr

# 2. for get the pose of the tracker we need to open the python code

cd ./Workspace/Carla/op_carla/op_bridge/op_bridge/fsm_lab_simulation/testing_scripts/
python3 debug_tracker.py

```

the source code of the script is as follows:

```python
import sys
sys.path.append("/home/cityu-fsm-lab-carla/Workspace/Carla/op_carla/op_bridge/op_bridge/fsm_lab_simulation")
import time
import numpy as np
from vive_tracker import ViveTrackerModule


TRACKER_NAME = "tracker_1"


vtm = ViveTrackerModule()
vtm.print_discovered_objects()
tracker = vtm.devices[TRACKER_NAME]
while True:
    try:
        cam_coord = tracker.get_pose_euler()
       # print(cam_coord)
        print(f"\rCamera coordinate: [x={cam_coord[0]:.4f}, y={cam_coord[1]:.4f}, z={cam_coord[2]:.4f}, roll={cam_coord[3]:.4f}, yaw={cam_coord[4]:.4f}, pitch={cam_coord[5]:.4f}]", end='')
    except Exception:
        continue
    finally:
        time.sleep(0.1)
```

use the api of vive_tracker, and display the positions as x,y,z,roll,yaw and pitch; (Note that y is the height, which perpendicular to the horizontal plane.)

6. the collection of the points

the following pictures are the original recordings of the points:

![6-1](img/2024-07-11-16-15-01.png)

![6-2](img/2024-07-11-16-15-23.png)

![6-3](img/2024-07-11-16-15-48.png)

## the data are as follows:

```python
points_A = [
{'position': [0.4799, -1.3278, -3.9353], 'orientation': [np.radians(0.1025), np.radians(-55.7886), np.radians(91.8146)]}, # 1
{'position': [0.4839, -1.3176, -3.9501], 'orientation': [np.radians(-179.7870), np.radians(-144.8536), np.radians(-120.9998)]}, # 1'
{'position': [0.4874, -1.3246, -3.9531], 'orientation': [np.radians(-22.6521), np.radians(-32.9473), np.radians(90.0343)]}, # 1''
{'position': [0.0849, -1.3360, -3.3517], 'orientation': [np.radians(2.3271), np.radians(-59.5605), np.radians(92.3895)]}, # 2
{'position': [0.0897, -1.3227, -3.3566], 'orientation': [np.radians(-178.6286), np.radians(-147.6686), np.radians(-120.2366)]}, # 2'
{'position': [0.0915, -1.3165, -3.3709], 'orientation': [np.radians(-22.1813), np.radians(-34.4269), np.radians(86.8067)]}, # 2''
{'position': [-1.8325, -1.3999, -3.2219], 'orientation': [np.radians(0.6676), np.radians(-60.7822), np.radians(90.0414)]}, # 3
{'position': [-1.8274, -1.3918, -3.2343], 'orientation': [np.radians(-177.4059), np.radians(-147.7991), np.radians(-120.0628)]},# 3'
{'position': [-1.8318, -1.3937, -3.2280], 'orientation': [np.radians(-22.4285), np.radians(-34.8151), np.radians(87.4277)]}, # 3''
{'position': [-1.4225, -1.4020, -2.9691], 'orientation': [np.radians(-0.0218), np.radians(-61.0372), np.radians(88.9864)]}, # 4
{'position': [-1.4202, -1.3885, -2.9732], 'orientation': [np.radians(-179.0243), np.radians(-148.9208), np.radians(-120.0628)]},# 4'
{'position': [-1.4256, -1.3908, -2.9636], 'orientation': [np.radians(-23.1839), np.radians(-37.6294), np.radians(86.9329)]}, # 4''
{'position': [-0.5136, -1.3886, -2.3799], 'orientation': [np.radians(-0.2046), np.radians(-58.0093), np.radians(89.8167)]}, # 5
{'position': [-0.5125, -1.3719, -2.3902], 'orientation': [np.radians(-178.9994), np.radians(-147.5538), np.radians(-120.1558)]},# 5'
{'position': [-0.5127, -1.3798, -2.3802], 'orientation': [np.radians(-22.7840), np.radians(-33.8897), np.radians(88.7093)]}, # 5''
{'position': [-1.3815, -1.3870, -1.0412], 'orientation': [np.radians(0.4369), np.radians(-54.3746), np.radians(89.1041)]}, # 6
{'position': [-1.3752, -1.3760, -1.0495], 'orientation': [np.radians(-176.0058), np.radians(-148.8773), np.radians(-117.6707)]},# 6'
{'position': [-1.3782, -1.3806, -1.0446], 'orientation': [np.radians(-22.0088), np.radians(-33.5773), np.radians(87.1209)]}, # 6''
{'position': [-1.4998, -1.3819, -0.8639], 'orientation': [np.radians(-0.4083), np.radians(-60.5462), np.radians(89.7344)]}, # 7
{'position': [-1.5154, -1.3534, -0.8967], 'orientation': [np.radians(-177.6953), np.radians(-145.8537), np.radians(-119.6940)]},# 7'
{'position': [-1.5270, -1.3621, -0.8903], 'orientation': [np.radians(-21.8242), np.radians(-32.5503), np.radians(88.6979)]}, # 7''
{'position': [0.8669, -1.3785, -1.4911], 'orientation': [np.radians(-2.5103), np.radians(-57.6808), np.radians(87.8262)]}, # 8
{'position': [0.8693, -1.3655, -1.4974], 'orientation': [np.radians(-177.6567), np.radians(-147.0740), np.radians(-119.2254)]}, # 8'
{'position': [0.8660, -1.3679, -1.4915], 'orientation': [np.radians(-23.1031), np.radians(-33.9084), np.radians(88.1074)]}, # 8''
{'position': [-0.7374, -1.4125, -4.5271], 'orientation': [np.radians(0.0960), np.radians(-54.4806), np.radians(91.2216)]}, # 9
{'position': [-0.7443, -1.3969, -4.5221], 'orientation': [np.radians(-178.8582), np.radians(-145.7330), np.radians(-120.3380)]},#9'
{'position': [-0.7475, -1.3988, -4.5180], 'orientation': [np.radians(-21.3784), np.radians(-29.6438), np.radians(89.6642)]}, #9''
{'position': [1.2270, -1.3718, -2.4375], 'orientation': [np.radians(0.7148), np.radians(-58.8044), np.radians(90.8907)]}, # 10
{'position': [1.2464, -1.3614, -2.4503], 'orientation': [np.radians(-179.4645), np.radians(-147.4631), np.radians(-120.9428)]}, #10'
{'position': [1.2365, -1.3931, -2.4204], 'orientation': [np.radians(-21.9546), np.radians(-34.9327), np.radians(94.6225)]}, #10''
{'position': [0.1257, -1.3777, -0.7007], 'orientation': [np.radians(-2.7652), np.radians(-57.0650), np.radians(87.6298)]}, # 11
{'position': [0.1327, -1.3646, -0.7114], 'orientation': [np.radians(-179.1399), np.radians(-144.1354), np.radians(-120.2723)]}, #11'
{'position': [0.1294, -1.3712, -0.6966], 'orientation': [np.radians(-23.9618), np.radians(-37.5303), np.radians(92.6984)]}, #11''
]

points_B = [
{'position': [0.217, 0, -2.05], 'orientation': [np.radians(0), np.radians(0), np.radians(0)]}, #1
{'position': [0.217, 0, -2.05], 'orientation': [np.radians(-90), np.radians(30), np.radians(0)]}, #1'
{'position': [0.217, 0, -2.05], 'orientation': [np.radians(30), np.radians(0), np.radians(-20)]}, #1''
{'position': [0.92, 0, -2.05], 'orientation': [np.radians(0), np.radians(0), np.radians(0)]}, #2
{'position': [0.92, 0, -2.05], 'orientation': [np.radians(-90), np.radians(30), np.radians(0)]}, #2'
{'position': [0.92, 0, -2.05], 'orientation': [np.radians(30), np.radians(0), np.radians(-20)]}, #2''
{'position': [2.05, 0, -0.97], 'orientation': [np.radians(0), np.radians(0), np.radians(0)]}, #3
{'position': [2.05, 0, -0.97], 'orientation': [np.radians(-90), np.radians(30), np.radians(0)]}, #3'
{'position': [2.05, 0, -0.97], 'orientation': [np.radians(30), np.radians(0), np.radians(-20)]}, #3''
{'position': [2.05, 0, -0.49], 'orientation': [np.radians(0), np.radians(0), np.radians(0)]}, #4
{'position': [2.05, 0, -0.49], 'orientation': [np.radians(-90), np.radians(30), np.radians(0)]}, #4'
{'position': [2.05, 0, -0.49], 'orientation': [np.radians(30), np.radians(0), np.radians(-20)]}, #4''
{'position': [2.05, 0, -2.05], 'orientation': [np.radians(0), np.radians(0), np.radians(0)]}, #5
{'position': [2.05, 0, -2.05], 'orientation': [np.radians(-90), np.radians(30), np.radians(0)]}, #5'
{'position': [2.05, 0, -2.05], 'orientation': [np.radians(30), np.radians(0), np.radians(-20)]}, #5''
{'position': [3.648, 0, -2.05], 'orientation': [np.radians(0), np.radians(0), np.radians(0)]}, #6
{'position': [3.648, 0, -2.05], 'orientation': [np.radians(-90), np.radians(30), np.radians(0)]}, #6'
{'position': [3.648, 0, -2.05], 'orientation': [np.radians(30), np.radians(0), np.radians(-20)]}, #6''
{'position': [3.858, 0, -2.05], 'orientation': [np.radians(0), np.radians(0), np.radians(0)]}, #7
{'position': [3.858, 0, -2.05], 'orientation': [np.radians(-90), np.radians(30), np.radians(0)]}, #7'
{'position': [3.858, 0, -2.05], 'orientation': [np.radians(30), np.radians(0), np.radians(-20)]}, #7''
{'position': [2.05, 0, -3.72], 'orientation': [np.radians(0), np.radians(0), np.radians(0)]}, #8
{'position': [2.05, 0, -3.72], 'orientation': [np.radians(-90), np.radians(30), np.radians(0)]}, #8'
{'position': [2.05, 0, -3.72], 'orientation': [np.radians(30), np.radians(0), np.radians(-20)]}, #8''
{'position': [0.405, 0.004,-1.725], 'orientation': [np.radians(0), np.radians(0), np.radians(0)]}, #9
{'position': [0.405, 0.004,-1.725], 'orientation': [np.radians(-90), np.radians(30), np.radians(0)]}, #9'
{'position': [0.405, 0.004,-1.725], 'orientation': [np.radians(30), np.radians(0), np.radians(-20)]}, #9''
{'position': [1.053, 0.004, -3.507], 'orientation': [np.radians(0), np.radians(0), np.radians(0)]}, #10
{'position': [1.053, 0.004, -3.507], 'orientation': [np.radians(-90), np.radians(30), np.radians(0)]}, #10'
{'position': [1.053, 0.004, -3.507], 'orientation': [np.radians(30), np.radians(0), np.radians(-20)]}, #10''
{'position': [3.01, 0.004, -3.527], 'orientation': [np.radians(0), np.radians(0), np.radians(0)]}, #11
{'position': [3.01, 0.004, -3.527], 'orientation': [np.radians(-90), np.radians(30), np.radians(0)]}, #11'
{'position': [3.01, 0.004, -3.527], 'orientation': [np.radians(30), np.radians(0), np.radians(-20)]}, #11''
]

```

## the followings are the img of the points

![6-4-1](img/2024-07-15-10-28-03.png)

for red is tracker, blue is sandbox

![6-4-2](img/2024-07-15-10-29-19.png)

![6-4-3](img/2024-07-15-10-29-42.png)

![6-4-4](img/2024-07-15-10-30-02.png)

The following two pictures show what we observe when we make the viewing angle parallel to the xoz plane of the two Cartesian coordinate systems A (tracker) and B (sandbox) (so that the points of the point cloud fall on the same plane).

![6-5-1](img/2024-07-15-10-30-44.png)

![6-5-2](img/2024-07-15-10-33-57.png)

It can be seen that there is a complex rotation relationship between the two coordinate systems A and B.

the result can be seen in tracker_fun_test_result.md

## for more acurate we add 16 more points for positioning

[x=-1.1228, y=-1.3120, z=-4.3271, roll=0.8608, yaw=-53.6911, pitch=92.4604]
[x=-1.4121, y=-1.3142, z=-3.9064, roll=0.4280, yaw=-61.1639, pitch=92.2939]
[x=-0.9877, y=-1.3225, z=-3.6291, roll=-6.8813, yaw=65.2652, pitch=97.5612]
[x=-0.7143, y=-1.4099, z=-4.0170, roll=-0.1467, yaw=-64.8302, pitch=89.7913]

[x=-2.2414, y=-1.3942, z=-2.6190, roll=-1.0939, yaw=-55.2967, pitch=89.3419]
[x=-2.5227, y=-1.3905, z=-2.2047, roll=-1.0837, yaw=-55.8826, pitch=89.3874]
[x=-2.1095, y=-1.3928, z=-1.9267, roll=-0.0076, yaw=-55.9562, pitch=89.9423]
[x=-1.8286, y=-1.3967, z=-2.3420, roll=-0.3516, yaw=-61.4611, pitch=89.5441]

[x=1.0222, y=-1.4046, z=-2.8521, roll=-0.1376, yaw=-60.9930, pitch=90.1337]
[x=0.7512, y=-1.3999, z=-2.4198, roll=-1.7505, yaw=-65.6642, pitch=88.0347]
[x=1.1732, y=-1.4018, z=-2.1531, roll=-0.6117, yaw=-59.5986, pitch=88.9130]
[x=1.4524, y=-1.4044, z=-2.5697, roll=-0.4654, yaw=-58.3503, pitch=89.0828]

[x=-0.0673, y=-1.3908, z=-1.1899, roll=-0.8849, yaw=-56.3085, pitch=89.3281]
[x=-0.3366, y=-1.3817, z=-0.7836, roll=-0.3274, yaw=-54.8635, pitch=89.7061]
[x=0.0801, y=-1.3763, z=-0.5090, roll=-0.5654, yaw=-55.0822, pitch=89.4644]
[x=0.3583, y=-1.3796, z=-0.9285, roll=-2.0204, yaw=-54.3347, pitch=88.1416]

[0.8,0,-0.5]
[1.3,0,-0.5]
[1.3,0,-1]
[0.8,0,1]

[2.8,0,-0.5]
[3.3,0,-0.5]
[3.3,0,-1]
[2.8,0,-1]

[2.8,0,-3.1]
[3.3,0,-3.1]
[3.3,0,-3.6]
[2.8,0,-3.1]

[0.8,0,-3.1]
[1.3,0,-3.1]
[1.3,0,-3.6]
[0.8,0,-3.6]


