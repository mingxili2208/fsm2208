# TODO: change the manually computed transform matrix to a config file

```python
def cam_to_world(cam_coord):
    points_A = np.array([
        [0.0735, -1.256, -0.864],
        [1.2985, -1.2337, -0.7771],
        [2.6265, -1.2139, -1.4418],
        [0.7060, -1.2437, -1.3144],
        [1.7504, -1.2264, -1.3332],
        [2.2345, -1.2219, -1.9145],
        [0.1044, -1.2530, -1.9240],
        [-0.5540, -1.2630, -2.5030],
        [0.1983, -1.2510, -2.6737],
        [-0.7560, -1.2705, -3.1767],
        [0.0263, -1.2603, -3.7120],
        [1.009,-1.188,-3.468], # [1.0052, -1.2399, -3.4513],  
        [1.442,-1.1759,-3.978], # [1.4389, -1.2370, -3.9670],   
        # ------------------------
    ])

    points_B = np.array([
        [0.755, 0, 0.455],
        [0.283, 0, 1.738],
        [0.532, 0, 3.198],
        [0.966, 0, 1.335],
        [0.68, 0, 2.336],
        [1.11, 0, 2.960],
        [1.725, 0, 0.935],
        [2.47, 0, 0.467],
        [2.426, 0, 1.425],
        [3.175, 0, 0.471],
        [3.461, 0, 1.380],
        [2.927, 0, 2.238],
        [3.299, 0, 2.805]
    ])

    A_matrix = []
    B_vector = []

    for i in range(len(points_A)):
        x_A, y_A, z_A = points_A[i]
        x_B, y_B, z_B = points_B[i]
        A_matrix.append([x_A, y_A, z_A, 1, 0, 0, 0, 0, 0, 0, 0, 0])
        A_matrix.append([0, 0, 0, 0, x_A, y_A, z_A, 1, 0, 0, 0, 0])
        A_matrix.append([0, 0, 0, 0, 0, 0, 0, 0, x_A, y_A, z_A, 1])
        B_vector.append(x_B)
        B_vector.append(y_B)
        B_vector.append(z_B)

    A_matrix = np.array(A_matrix)
    B_vector = np.array(B_vector)

    params, _, _, _ = np.linalg.lstsq(A_matrix, B_vector, rcond=None)

    a, b, c, d, e, f, g, h, i, j, k, l = params

    transform_matrix = np.array([
        [a, b, c, d],
        [e, f, g, h],
        [i, j, k, l],
        [0, 0, 0, 1]
    ])

    A_point = np.array(cam_coord + [1])
    B_point_computed = np.dot(transform_matrix, A_point)
    return B_point_computed[:3]

def euler_to_rotation_matrix(roll, pitch, yaw):
    """
    将欧拉角 (roll, pitch, yaw) 转换为旋转矩阵。
    """
    Rx = np.array([
        [1, 0, 0],
        [0, np.cos(roll), -np.sin(roll)],
        [0, np.sin(roll), np.cos(roll)]
    ])
    Ry = np.array([
        [np.cos(pitch), 0, np.sin(pitch)],
        [0, 1, 0],
        [-np.sin(pitch), 0, np.cos(pitch)]
    ])
    Rz = np.array([
        [np.cos(yaw), -np.sin(yaw), 0],
        [np.sin(yaw), np.cos(yaw), 0],
        [0, 0, 1]
    ])
    R = np.dot(Rz, np.dot(Ry, Rx))
    return R

def construct_transformation_matrix(roll, pitch, yaw, translation):
    """
    根据欧拉角 (roll, pitch, yaw) 和平移向量 translation 构建转换矩阵。
    """
    R = euler_to_rotation_matrix(roll, pitch, yaw)
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = translation
    return T

def apply_transformation(transformation_matrix, points):
    """
    将变换矩阵应用于点列表。
   
    :param transformation_matrix: 4x4 变换矩阵
    :param points: 点列表，每个点包含 'position' 和 'orientation' (欧拉角)
    :return: 转换后的点列表
    """
    transformed_points = []
    for point in points:
        # 将位置转换为齐次坐标
        position_homogeneous = np.append(point['position'], 1)
        # 应用变换矩阵
        transformed_position_homogeneous = np.dot(transformation_matrix, position_homogeneous)
        transformed_position = transformed_position_homogeneous[:3]

        # 旋转部分
        R_point = euler_to_rotation_matrix(*point['orientation'])
        R_transformed = np.dot(transformation_matrix[:3, :3], R_point)

        # 计算转换后的欧拉角
        roll_transformed = np.arctan2(R_transformed[2, 1], R_transformed[2, 2])
        pitch_transformed = np.arctan2(-R_transformed[2, 0], np.sqrt(R_transformed[2, 1] ** 2 + R_transformed[2, 2] ** 2))
        yaw_transformed = np.arctan2(R_transformed[1, 0], R_transformed[0, 0])

        transformed_orientation = [roll_transformed, pitch_transformed, yaw_transformed]
       
        transformed_points.append({
            'position': transformed_position,
            'orientation': transformed_orientation
        })
    return transformed_points


def switch_hand(points):
    return [{'position': np.array(point['position']) * np.array([1, -1, 1]), 'orientation': point['orientation']} for point in points]


# 定义坐标系A下的点的姿态 (roll, pitch, yaw) 和位置 (x, y, z)
points_A = [
    {'position': [0.1706, -1.2442, -0.8407], 'orientation': [np.radians(-89.5), np.radians(95.9), np.radians(179)]},      # 1
    {'position': [0.1621, -1.2434, -0.8424], 'orientation': [np.radians(88.6), np.radians(-84.16), np.radians(-2.1)]},      # 1'
    {'position': [1.4092, -1.2436, -0.2610], 'orientation': [np.radians(-87.8), np.radians(96.0), np.radians(178.5)]},    # 2
    {'position': [2.8897, -1.2410, -0.3832], 'orientation': [np.radians(-89.8), np.radians(-167), np.radians(-179.6)]},    # 3
    {'position': [2.8871, -1.2396, -0.3889], 'orientation': [np.radians(90.5), np.radians(3.14), np.radians(-0.4)]},    # 3'
    {'position': [1.0596, -1.2391, -0.9761], 'orientation': [np.radians(-97.9), np.radians(95.2), np.radians(-172)]},    # 4
    {'position': [2.0351, -1.2394, -0.6069], 'orientation': [np.radians(-90.5), np.radians(98.6), np.radians(-179.4)]},    # 5
    {'position': [2.6981, -1.2376, -0.9729], 'orientation': [np.radians(-91.2), np.radians(97.6), np.radians(-178)]},    # 6
    {'position': [0.7266, -1.2373, -1.7677], 'orientation': [np.radians(-104.6), np.radians(93.2), np.radians(-164.9)]},    # 7
    {'position': [0.3203, -1.2274, -2.5383], 'orientation': [np.radians(-103), np.radians(92.16), np.radians(-166.6)]},   # 8
    {'position': [1.0885, -1.2293, -2.4300], 'orientation': [np.radians(-84.6), np.radians(95.4), np.radians(174.9)]},    # 9
    {'position': [1.0853, -1.2274, -2.4247], 'orientation': [np.radians(-89.4), np.radians(-175.9), np.radians(179.6)]},    # 9'
    {'position': [1.0800, -1.2299, -2.4316], 'orientation': [np.radians(88.5), np.radians(-86.7), np.radians(-1.75)]},    # 9''
    {'position': [1.0888, -1.2335, -2.4431], 'orientation': [np.radians(90.1), np.radians(6.15), np.radians(-0.13)]},    # 9'''
    {'position': [0.3832, -1.2276, -3.2410], 'orientation': [np.radians(-90.0), np.radians(-177.4), np.radians(179.4)]},   # 10
    {'position': [0.3831, -1.2272, -3.2487], 'orientation': [np.radians(90.3), np.radians(7.0), np.radians(-0.22)]},   # 10'
    {'position': [1.3158, -1.2295, -3.4664], 'orientation': [np.radians(-92.0), np.radians(97.8), np.radians(-177.5)]},    # 11
    {'position': [2.1200, -1.2232, -2.8468], 'orientation': [np.radians(-91.7), np.radians(93.4), np.radians(-178.3)]},         # 12
    {'position': [2.7144, -1.2222, -3.1710], 'orientation': [np.radians(-91.7), np.radians(96.0), np.radians(-178.1)]},        # 13
    {'position': [2.7082, -1.2217, -3.1711], 'orientation': [np.radians(93.1), np.radians(-81.9), np.radians(2.41)]},        # 13'
]

# 定义坐标系B下的对应点的姿态 (roll, pitch, yaw) 和位置 (x, y, z)
points_B = [
    {'position': [0.455, -0.755, 0], 'orientation': [np.radians(0), np.radians(0), np.radians(0)]},         # 1
    {'position': [0.455, -0.755, 0], 'orientation': [np.radians(0), np.radians(0), np.radians(180)]},       # 1'
    {'position': [1.738, -0.283, 0], 'orientation': [np.radians(0), np.radians(0), np.radians(0)]},         # 2
    {'position': [3.198, -0.532, 0], 'orientation': [np.radians(0), np.radians(0), np.radians(90)]},        # 3
    {'position': [3.198, -0.532, 0], 'orientation': [np.radians(0), np.radians(0), np.radians(270)]},       # 3'
    {'position': [1.335, -0.966, 0], 'orientation': [np.radians(0), np.radians(0), np.radians(0)]},         # 4
    {'position': [2.336, -0.68, 0], 'orientation': [np.radians(0), np.radians(0), np.radians(0)]},          # 5
    {'position': [2.960, -1.11, 0], 'orientation': [np.radians(0), np.radians(0), np.radians(0)]},          # 6
    {'position': [0.935, -1.725, 0], 'orientation': [np.radians(0), np.radians(0), np.radians(0)]},         # 7
    {'position': [0.467, -2.47, 0], 'orientation': [np.radians(0), np.radians(0), np.radians(0)]},          # 8
    {'position': [1.425, -2.426, 0], 'orientation': [np.radians(0), np.radians(0), np.radians(0)]},         # 9
    {'position': [1.425, -2.426, 0], 'orientation': [np.radians(0), np.radians(0), np.radians(90)]},        # 9'
    {'position': [1.425, -2.426, 0], 'orientation': [np.radians(0), np.radians(0), np.radians(180)]},       # 9''
    {'position': [1.425, -2.426, 0], 'orientation': [np.radians(0), np.radians(0), np.radians(270)]},       # 9'''
    {'position': [0.471, -3.175, 0], 'orientation': [np.radians(0), np.radians(0), np.radians(90)]},        # 10
    {'position': [0.471, -3.175, 0], 'orientation': [np.radians(0), np.radians(0), np.radians(270)]},       # 10'
    {'position': [1.380, -3.461, 0], 'orientation': [np.radians(0), np.radians(0), np.radians(0)]},         # 11
    {'position': [2.238, -2.927, 0], 'orientation': [np.radians(0), np.radians(0), np.radians(0)]},         # 12
    {'position': [2.805, -3.299, 0], 'orientation': [np.radians(0), np.radians(0), np.radians(0)]},         # 13
    {'position': [2.805, -3.299, 0], 'orientation': [np.radians(0), np.radians(0), np.radians(180)]},       # 13'
]
points_B = switch_hand(points_B)

# 构建坐标系A和B下的变换矩阵
transformation_matrices_A = [construct_transformation_matrix(*point['orientation'], point['position']) for point in points_A]
transformation_matrices_B = [construct_transformation_matrix(*point['orientation'], point['position']) for point in points_B]

# 计算从A到B的相对变换矩阵
relative_transformations = [np.dot(np.linalg.inv(TA), TB) for TA, TB in zip(transformation_matrices_A, transformation_matrices_B)]

# 平均相对变换矩阵
mean_transformation = np.mean(relative_transformations, axis=0)

print("从坐标系A到坐标系B的变换矩阵:")
print(mean_transformation)

# 验证变换矩阵
transformed_points_B = apply_transformation(mean_transformation, points_A)

print("验证转换结果:")
for i, point in enumerate(transformed_points_B):
    print(f"点 {i} 在坐标系B中的实际位置: {points_B[i]['position']}")
    print(f"点 {i} 在坐标系B中的转换位置: {point['position']}")
    print(f"点 {i} 在坐标系B中的实际欧拉角: {points_B[i]['orientation']}")
    print(f"点 {i} 在坐标系B中的转换欧拉角: {point['orientation']}")
    print()


def camera_to_sandbox(points_cam):
    points_sandbox = apply_transformation(mean_transformation, points_cam)
    return switch_hand(points_sandbox)


def to_point(coords):
    to_radian = lambda x: x / 180 * np.pi
    return {
        'position': np.array(coords[:3]),
        'orientation': np.array([to_radian(coords[-1]), to_radian(coords[-2]), to_radian(coords[-3])])
    }


def to_array(point):
    return [*point['position'], *point['orientation'][::-1]]

```