# this is the function & test & result for the points without 16 points


```python
import numpy as np
from scipy.spatial.transform import Rotation as R
class Point:
    """
    表示三维空间中的一个点，包含位置和方向信息。
    """
    def __init__(self, position, orientation):
        """
        初始化Point对象。
        :param position: 点的位置，3D向量 [x, y, z]
        :param orientation: 点的方向，欧拉角 [roll,yaw,pitch]
        """
        self.position = np.array(position)
        self.orientation = np.array(orientation)

class CoordinateTransformer:
    """
    用于计算和应用坐标变换的类。
    """
    def __init__(self,points_A,points_B):
        """
        初始化CoordinateTransformer对象。
        T_pos: 位置变换矩阵
        R_euler: 方向变换矩阵
        """
        self.T_pos = None
        self.R_euler = None
        self.__Points_A=points_A
        self.__Points_B=points_B
        self.caculate_transformation_matrix()

    @staticmethod
    def euler_to_matrix(euler):
        """
        将欧拉角 (yxz) 转换为旋转矩阵。
        """
        #return R.from_euler('yxz', euler).as_matrix()
        return R.from_euler('yxz', [euler[2], euler[1], euler[0]]).as_matrix()

    @staticmethod
    def matrix_to_euler(matrix):
        """
        将旋转矩阵转换为欧拉角 (yxz)。
        返回: [yaw, pitch, roll] 顺序的欧拉角（弧度）。
        """
        euler = R.from_matrix(matrix).as_euler('yxz')
        #return euler
        return [euler[2], euler[1], euler[0]]

    @staticmethod
    def construct_transformation_matrix(rotation_matrix, translation):
        """
        根据旋转矩阵和平移向量构建变换矩阵。
        """
        T = np.eye(4)
        T[:3, :3] = rotation_matrix
        T[:3, 3] = translation
        return T

    def calculate_position_transformation(self, points_A, points_B):
        """
        计算两组点之间的位置变换矩阵。
        返回T_pos与R_pos
        """
        positions_A = np.array([point.position for point in points_A])
        positions_B = np.array([point.position for point in points_B])

        centroid_A = np.mean(positions_A, axis=0)
        centroid_B = np.mean(positions_B, axis=0)

        H = np.dot((positions_A - centroid_A).T, (positions_B - centroid_B))
        U, S, Vt = np.linalg.svd(H)
        R_pos = np.dot(Vt.T, U.T)
        if np.linalg.det(R_pos) < 0:
            Vt[-1, :] *= -1
            R_pos = np.dot(Vt.T, U.T)
        translation = centroid_B.T - np.dot(R_pos, centroid_A.T)
        self.T_pos = self.construct_transformation_matrix(R_pos, translation)
        return R_pos,translation

    def calculate_orientation_transformation(self, points_A, points_B):
        """
        计算两组点之间的方向变换矩阵。
        """
        orientations_A = [point.orientation for point in points_A]
        orientations_B = [point.orientation for point in points_B]

        R_A = [self.euler_to_matrix(orientation) for orientation in orientations_A]
        R_B = [self.euler_to_matrix(orientation) for orientation in orientations_B]

        R_diff = [np.dot(R_B[i], R_A[i].T) for i in range(len(R_A))]
        R_mean = np.mean(R_diff, axis=0)
        U, _, Vt = np.linalg.svd(R_mean)
        self.R_euler = np.dot(U, Vt)

    def apply_position_transformation(self, point):
        """
        应用位置变换到给定点。
        """
        position_homogeneous = np.append(point.position, 1)
        transformed_position_homogeneous = np.dot(self.T_pos, position_homogeneous)
        return Point(transformed_position_homogeneous[:3], point.orientation)

    def apply_orientation_transformation(self, point):
        """
        应用方向变换到给定点。
        """
        R_point = self.euler_to_matrix(point.orientation)
        R_transformed = np.dot(self.R_euler, R_point)
        transformed_orientation = self.matrix_to_euler(R_transformed)
        return Point(point.position, transformed_orientation)

    def transform_point(self, point):
        """
        对给定点应用完整的变换（位置和方向）。
        """
        transformed_position = self.apply_position_transformation(point)
        return self.apply_orientation_transformation(transformed_position)


    def calculate_position_error(self, actual, predicted):
        """
        计算位置的均方根误差（RMSE）
        """
        return np.sqrt(np.mean(np.sum((np.array(actual) - np.array(predicted))**2, axis=1)))

    def calculate_orientation_error(self, actual, predicted):
        """
        计算方向的平均角度误差（度）
        输入和输出都是以度为单位
        """
        actual = np.array(actual)
        predicted = np.array(predicted)

        # 计算每个轴的角度差
        diff = np.abs(actual - predicted)

        # 处理角度差大于180度的情况
        diff = np.where(diff > 180, 360 - diff, diff)

        # 计算平均角度误差（度）
        mean_error_deg = np.mean(diff)

        return mean_error_deg

    def calculate_and_print_errors(self, points_B, transformed_points_B):
        """
        计算并打印位置和方向的误差
        """
        actual_positions = [point.position for point in points_B]
        predicted_positions = [point.position for point in transformed_points_B]
        position_error = self.calculate_position_error(actual_positions, predicted_positions)

        actual_orientations = [point.orientation for point in points_B]
        predicted_orientations = [point.orientation for point in transformed_points_B]
        orientation_error = self.calculate_orientation_error(actual_orientations, predicted_orientations)

        print(f"位置均方根误差 (RMSE): {position_error:.4f} 米")
        print(f"方向平均角度误差: {orientation_error:.4f} 度")

        # 打印每个点的误差
        for i in range(len(points_B)):
            pos_error = np.linalg.norm(np.array(actual_positions[i]) - np.array(predicted_positions[i]))
            ori_error = self.calculate_orientation_error([actual_orientations[i]], [predicted_orientations[i]])
            print(f"点 {i}:")
            print(f"  位置误差: {pos_error:.4f} 米")
            print(f"  方向误差: {ori_error:.4f} 度")


    def demo_transformation(self):
        """
        演示坐标变换过程，包括创建坐标系、计算变换和验证结果。

        :param points_A: 坐标系A中的点列表
        :param points_B: 坐标系B中的点列表
        """
        # 创建坐标系对象
        coordinate_system_A = CoordinateSystem(self.__Points_A)
        coordinate_system_B = CoordinateSystem(self.__Points_B)

        # 计算变换
        self.calculate_position_transformation(coordinate_system_A.points, coordinate_system_B.points)
        self.calculate_orientation_transformation(coordinate_system_A.points, coordinate_system_B.points)

        # 打印变换矩阵
        print("位置变换矩阵:")
        print(self.T_pos)

        print("方向变换矩阵:")
        print(self.R_euler)

        # 应用变换并创建新的坐标系
        transformed_system = coordinate_system_A.transform(self)

        for point in transformed_system.points:
          print(point.position)
        # 验证和打印结果
        print("验证转换结果:")
        for i, (original, transformed) in enumerate(zip(coordinate_system_B.points, transformed_system.points)):
            print(f"点A {i} 在坐标系B中的实际位置: {original.position}")
            print(f"点A {i} 在坐标系B中的转换位置: {transformed.position}")
            print(f"点A {i} 在坐标系B中的实际欧拉角: {original.orientation}")
            print(f"点A {i} 在坐标系B中的转换欧拉角: {transformed.orientation}")
            print()

        # 计算和打印误差
        self.calculate_and_print_errors(coordinate_system_B.points, transformed_system.points)

 #   def caculate_transformation_matrix(self):

  #      coordinate_system_A = CoordinateSystem(self.__Points_A)
   #     coordinate_system_B = CoordinateSystem(self.__Points_B)
#
        # 计算变换
 #       self.calculate_position_transformation(coordinate_system_A.points, coordinate_system_B.points)
  #      self.calculate_orientation_transformation(coordinate_system_A.points, coordinate_system_B.points)

    def caculate_transformation_matrix(self,Points_A=None,Points_B=None):

        if Points_A is None:
            Points_A = self.__Points_A
        if Points_B is None:
            Points_B = self.__Points_B
        coordinate_system_A = CoordinateSystem(Points_A)
        coordinate_system_B = CoordinateSystem(Points_B)

        # 计算变换
        self.calculate_position_transformation(coordinate_system_A.points, coordinate_system_B.points)
        self.calculate_orientation_transformation(coordinate_system_A.points, coordinate_system_B.points)

    def get_transformation_matrix(self):
        """
        返回变换矩阵
        以 T_pos,R_euler的顺序返回两个变换矩阵
        """
        return self.T_pos,self.R_euler


class CoordinateSystem:
    """
    表示一个坐标系，包含多个点。
    """
    def __init__(self, points):
        """
        初始化CoordinateSystem对象。
        :param points: Point对象的列表
        """
        self.points = points

    def transform(self, transformer):
        """
        使用给定的变换器对坐标系中的所有点进行变换。
        :param transformer: CoordinateTransformer对象
        :return: 新的CoordinateSystem对象，包含变换后的点
        """
        return CoordinateSystem([transformer.transform_point(point) for point in self.points])
def Test(points_A,points_B):
"""
主函数，用于创建CoordinateTransformer对象，定义点集，并调用演示方法。
"""
transformer = CoordinateTransformer(points_A,points_B)
transformer.demo_transformation()
print(transformer.get_transformation_matrix())

if __name__ == "__main__":
    Test(points_A,points_B)
```

the test for this
位置变换矩阵:
[[-5.36832026e-01 -1.91152012e-02  8.43472575e-01  3.76743995e+00]
 [-3.39758331e-02  9.99422129e-01  1.02533240e-03  1.34261411e+00]
 [-8.43004756e-01 -2.81072522e-02 -5.37171261e-01 -3.80657419e+00]
 [ 0.00000000e+00  0.00000000e+00  0.00000000e+00  1.00000000e+00]]
方向变换矩阵:
[[-0.23391438 -0.53547566 -0.81151087]
 [-0.15430441  0.84453421 -0.51278856]
 [ 0.95993448  0.00527109 -0.28017495]]
[ 2.04642322e-01 -4.01365726e-04 -2.05740975e+00]
[ 2.04642322e-01 -4.01365726e-04 -2.05740975e+00]
[ 2.04642322e-01 -4.01365726e-04 -2.05740975e+00]
[ 0.91133579  0.01192133 -2.03937232]
[ 0.91133579  0.01192133 -2.03937232]
[ 0.91133579  0.01192133 -2.03937232]
[ 2.05404912  0.00720329 -0.49025273]
[ 2.05404912  0.00720329 -0.49025273]
[ 2.05404912  0.00720329 -0.49025273]
[ 2.05389905 -0.00498689 -0.97341164]
[ 2.05389905 -0.00498689 -0.97341164]
[ 2.05389905 -0.00498689 -0.97341164]
[ 2.05882946 -0.02170594 -2.05511225]
[ 2.05882946 -0.02170594 -2.05511225]
[ 2.05882946 -0.02170594 -2.05511225]
[ 3.65224426  0.00796958 -2.04444131]
[ 3.65224426  0.00796958 -2.04444131]
[ 3.65224426  0.00796958 -2.04444131]
验证转换结果:
点A 0 在坐标系B中的实际位置: [ 0.217  0.    -2.05 ]
点A 0 在坐标系B中的转换位置: [ 2.04642322e-01 -4.01365726e-04 -2.05740975e+00]
点A 0 在坐标系B中的实际欧拉角: [0. 0. 0.]
点A 0 在坐标系B中的转换欧拉角: [-0.39066585  0.2358752  -0.12799818]

点A 1 在坐标系B中的实际位置: [ 0.217  0.    -2.05 ]
点A 1 在坐标系B中的转换位置: [ 2.04642322e-01 -4.01365726e-04 -2.05740975e+00]
点A 1 在坐标系B中的实际欧拉角: [-1.57079633  0.52359878  0.        ]
点A 1 在坐标系B中的转换欧拉角: [-0.0304349   0.16340595 -0.77435356]

点A 2 在坐标系B中的实际位置: [ 0.217  0.    -2.05 ]
点A 2 在坐标系B中的转换位置: [ 2.04642322e-01 -4.01365726e-04 -2.05740975e+00]
点A 2 在坐标系B中的实际欧拉角: [ 0.52359878  0.         -0.34906585]
点A 2 在坐标系B中的转换欧拉角: [ 0.05536215  0.4855616  -0.03491161]

点A 3 在坐标系B中的实际位置: [ 0.92  0.   -2.05]
点A 3 在坐标系B中的转换位置: [ 0.91133579  0.01192133 -2.03937232]
点A 3 在坐标系B中的实际欧拉角: [0. 0. 0.]
点A 3 在坐标系B中的转换欧拉角: [-0.46090073  0.22640741 -0.13469619]

点A 4 在坐标系B中的实际位置: [ 0.92  0.   -2.05]
点A 4 在坐标系B中的转换位置: [ 0.91133579  0.01192133 -2.03937232]
点A 4 在坐标系B中的实际欧拉角: [-1.57079633  0.52359878  0.        ]
点A 4 在坐标系B中的转换欧拉角: [ 0.01371745  0.13529437 -0.77883743]

点A 5 在坐标系B中的实际位置: [ 0.92  0.   -2.05]
点A 5 在坐标系B中的转换位置: [ 0.91133579  0.01192133 -2.03937232]
点A 5 在坐标系B中的实际欧拉角: [ 0.52359878  0.         -0.34906585]
点A 5 在坐标系B中的转换欧拉角: [ 0.02598449  0.47952672 -0.0821688 ]

点A 6 在坐标系B中的实际位置: [ 2.05  0.   -0.97]
点A 6 在坐标系B中的转换位置: [ 2.05404912  0.00720329 -0.49025273]
点A 6 在坐标系B中的实际欧拉角: [0. 0. 0.]
点A 6 在坐标系B中的转换欧拉角: [-0.48015577  0.24405725 -0.14606182]

点A 7 在坐标系B中的实际位置: [ 2.05  0.   -0.97]
点A 7 在坐标系B中的转换位置: [ 2.05404912  0.00720329 -0.49025273]
点A 7 在坐标系B中的实际欧拉角: [-1.57079633  0.52359878  0.        ]
点A 7 在坐标系B中的转换欧拉角: [ 0.01132078  0.11726263 -0.786895  ]

点A 8 在坐标系B中的实际位置: [ 2.05  0.   -0.97]
点A 8 在坐标系B中的转换位置: [ 2.05404912  0.00720329 -0.49025273]
点A 8 在坐标系B中的实际欧拉角: [ 0.52359878  0.         -0.34906585]
点A 8 在坐标系B中的转换欧拉角: [ 0.01843651  0.4832352  -0.0653843 ]

点A 9 在坐标系B中的实际位置: [ 2.05  0.   -0.49]
点A 9 在坐标系B中的转换位置: [ 2.05389905 -0.00498689 -0.97341164]
点A 9 在坐标系B中的实际欧拉角: [0. 0. 0.]
点A 9 在坐标系B中的转换欧拉角: [-0.48385194  0.25047485 -0.15305585]

点A 10 在坐标系B中的实际位置: [ 2.05  0.   -0.49]
点A 10 在坐标系B中的转换位置: [ 2.05389905 -0.00498689 -0.97341164]
点A 10 在坐标系B中的实际欧拉角: [-1.57079633  0.52359878  0.        ]
点A 10 在坐标系B中的转换欧拉角: [ 0.03655598  0.13556152 -0.77525789]

点A 11 在坐标系B中的实际位置: [ 2.05  0.   -0.49]
点A 11 在坐标系B中的转换位置: [ 2.05389905 -0.00498689 -0.97341164]
点A 11 在坐标系B中的实际欧拉角: [ 0.52359878  0.         -0.34906585]
点A 11 在坐标系B中的转换欧拉角: [-0.03714983  0.49404941 -0.04012758]

点A 12 在坐标系B中的实际位置: [ 2.05  0.   -2.05]
点A 12 在坐标系B中的转换位置: [ 2.05882946 -0.02170594 -2.05511225]
点A 12 在坐标系B中的实际欧拉角: [0. 0. 0.]
点A 12 在坐标系B中的转换欧拉角: [-0.42964706  0.24466824 -0.1490998 ]

点A 13 在坐标系B中的实际位置: [ 2.05  0.   -2.05]
点A 13 在坐标系B中的转换位置: [ 2.05882946 -0.02170594 -2.05511225]
点A 13 在坐标系B中的实际欧拉角: [-1.57079633  0.52359878  0.        ]
点A 13 在坐标系B中的转换欧拉角: [ 0.01311858  0.14108469 -0.77387807]

点A 14 在坐标系B中的实际位置: [ 2.05  0.   -2.05]
点A 14 在坐标系B中的转换位置: [ 2.05882946 -0.02170594 -2.05511225]
点A 14 在坐标系B中的实际欧拉角: [ 0.52359878  0.         -0.34906585]
点A 14 在坐标系B中的转换欧拉角: [ 0.03682763  0.48798455 -0.0480991 ]

点A 15 在坐标系B中的实际位置: [ 3.648  0.    -2.05 ]
点A 15 在坐标系B中的转换位置: [ 3.65224426  0.00796958 -2.04444131]
点A 15 在坐标系B中的实际欧拉角: [0. 0. 0.]
点A 15 在坐标系B中的转换欧拉角: [-0.36621378  0.22852872 -0.18571858]

点A 16 在坐标系B中的实际位置: [ 3.648  0.    -2.05 ]
点A 16 在坐标系B中的转换位置: [ 3.65224426  0.00796958 -2.04444131]
点A 16 在坐标系B中的实际欧拉角: [-1.57079633  0.52359878  0.        ]
点A 16 在坐标系B中的转换欧拉角: [ 0.02399552  0.0922064  -0.75929794]

点A 17 在坐标系B中的实际位置: [ 3.648  0.    -2.05 ]
点A 17 在坐标系B中的转换位置: [ 3.65224426  0.00796958 -2.04444131]
点A 17 在坐标系B中的实际欧拉角: [ 0.52359878  0.         -0.34906585]
点A 17 在坐标系B中的转换欧拉角: [ 0.04258957  0.47657088 -0.08600795]

**位置均方根误差 (RMSE): 0.2784 米
方向平均角度误差: 0.5385 度**

点 0:
  位置误差: 0.0144 米
  方向误差: 0.2515 度
点 1:
  位置误差: 0.0144 米
  方向误差: 0.8916 度
点 2:
  位置误差: 0.0144 米
  方向误差: 0.4227 度
点 3:
  位置误差: 0.0182 米
  方向误差: 0.2740 度
点 4:
  位置误差: 0.0182 米
  方向误差: 0.9172 度
点 5:
  位置误差: 0.0182 米
  方向误差: 0.4147 度
点 6:
  位置误差: 0.4798 米
  方向误差: 0.2901 度
点 7:
  位置误差: 0.4798 米
  方向误差: 0.9251 度
点 8:
  位置误差: 0.4798 米
  方向误差: 0.4240 度
点 9:
  位置误差: 0.4835 米
  方向误差: 0.2958 度
点 10:
  位置误差: 0.4835 米
  方向误差: 0.9235 度
点 11:
  位置误差: 0.4835 米
  方向误差: 0.4546 度
点 12:
  位置误差: 0.0240 米
  方向误差: 0.2745 度
点 13:
  位置误差: 0.0240 米
  方向误差: 0.9134 度
点 14:
  位置误差: 0.0240 米
  方向误差: 0.4252 度
点 15:
  位置误差: 0.0106 米
  方向误差: 0.2602 度
点 16:
  位置误差: 0.0106 米
  方向误差: 0.9285 度
点 17:
  位置误差: 0.0106 米
  方向误差: 0.4069 度
  