# intro of 7_paramaters helmert with the least_squares errors

## Euler_Angles

使用最小二乘法进行七参数赫尔默特 (Helmert) 变换，需要以下步骤：

**1. 赫尔默特变换模型:**

七参数赫尔默特变换模型如下：

```
[X, Y, Z] = [ΔX, ΔY, ΔZ] + (1 + m) * R * [x, y, z]
```

其中：

* `[x, y, z]` 是源坐标系下的坐标
* `[X, Y, Z]` 是目标坐标系下的坐标
* `[ΔX, ΔY, ΔZ]` 是平移参数
* `R` 是旋转矩阵，由三个旋转角 (`ω`, `φ`, `χ`) 决定
* `m` 是尺度因子

**2. 线性化观测方程:**

将赫尔默特变换模型线性化，得到观测方程：

```
V = A * x - L
```

其中：

* `V` 是观测值残差向量
* `A` 是设计矩阵，包含观测点坐标和变换参数的偏导数
* `x` 是待求解的七参数向量：`[ΔX, ΔY, ΔZ, ω, φ, χ, m]`
* `L` 是观测值向量，包含目标坐标系下坐标与源坐标系下坐标的差值

**3. 最小二乘解:**

根据最小二乘原理，最小化残差平方和：

```
Vᵀ * P * V = min
```

其中：

* `P` 是观测值的权重矩阵

解得七参数向量：

```
x = (Aᵀ * P * A)⁻¹ * Aᵀ * P * L
```

**4. 迭代计算:**

由于赫尔默特变换模型是非线性的，需要进行迭代计算。每次迭代后，使用计算得到的七参数更新观测方程，并重新计算残差和七参数，直到残差满足精度要求。

**具体步骤:**

1. **准备数据:** 收集至少三个已知点在源坐标系和目标坐标系下的坐标。
2. **初始化参数:** 设置七参数初始值，例如 `[0, 0, 0, 0, 0, 0, 0]`。
3. **构建设计矩阵和观测向量:** 根据观测方程，构建设计矩阵 `A` 和观测向量 `L`。
4. **计算七参数:** 使用最小二乘公式计算七参数向量 `x`。
5. **更新观测方程:** 使用计算得到的七参数更新观测方程。
6. **判断收敛:** 计算残差平方和，如果小于预设阈值，则迭代结束；否则，返回步骤 4 继续迭代。

**代码示例 (Python):**

```python
import numpy as np

def helmert_7param(source_coords, target_coords, iterations=10, tolerance=1e-6):
    """
    使用最小二乘法进行七参数赫尔默特变换。

    参数:
        source_coords: 源坐标系下坐标，形状为 (n, 3)
        target_coords: 目标坐标系下坐标，形状为 (n, 3)
        iterations: 最大迭代次数
        tolerance: 收敛阈值

    返回值:
        七参数向量: [ΔX, ΔY, ΔZ, ω, φ, χ, m]
    """

    n = source_coords.shape[0]
    x = np.zeros(7)  # 初始化七参数

    for i in range(iterations):
        # 构建设计矩阵和观测向量
        A = np.zeros((3 * n, 7))
        L = target_coords.flatten() - source_coords.flatten()

        for j in range(n):
            x0, y0, z0 = source_coords[j]
            X0, Y0, Z0 = target_coords[j]

            A[3 * j: 3 * j + 3] = np.array([
                [1, 0, 0, z0, 0, -y0, x0],
                [0, 1, 0, 0, z0, x0, y0],
                [0, 0, 1, -y0, x0, 0, z0]
            ])

        # 计算七参数
        P = np.identity(3 * n)  # 假设权重相等
        x += np.linalg.solve(A.T @ P @ A, A.T @ P @ L)

        # 更新观测方程
        source_coords = transform_coords(source_coords, x)

        # 判断收敛
        residuals = target_coords - source_coords
        rmse = np.sqrt(np.sum(residuals ** 2) / n)
        if rmse < tolerance:
            break

    return x

def transform_coords(coords, params):
    """
    应用七参数赫尔默特变换到坐标。

    参数:
        coords: 坐标，形状为 (n, 3)
        params: 七参数向量

    返回值:
        变换后的坐标
    """

    ΔX, ΔY, ΔZ, ω, φ, χ, m = params

    Rx = np.array([
        [1, 0, 0],
        [0, np.cos(ω), -np.sin(ω)],
        [0, np.sin(ω), np.cos(ω)]
    ])

    Ry = np.array([
        [np.cos(φ), 0, np.sin(φ)],
        [0, 1, 0],
        [-np.sin(φ), 0, np.cos(φ)]
    ])

    Rz = np.array([
        [np.cos(χ), -np.sin(χ), 0],
        [np.sin(χ), np.cos(χ), 0],
        [0, 0, 1]
    ])

    R = Rz @ Ry @ Rx

    transformed_coords = (1 + m) * np.dot(coords, R.T) + np.array([ΔX, ΔY, ΔZ])

    return transformed_coords
```

**注意:**

* 以上代码仅供参考，实际应用中需要根据具体情况进行调整。
* 赫尔默特变换的精度取决于已知点的分布和精度，以及观测值的权重设置。

## quaternion

```python
import numpy as np

def helmert_7param_quaternion(source_coords, target_coords, iterations=10, tolerance=1e-6):
    """
    使用四元数和最小二乘法进行七参数赫尔默特变换。

    参数:
        source_coords: 源坐标系下坐标，形状为 (n, 3)
        target_coords: 目标坐标系下坐标，形状为 (n, 3)
        iterations: 最大迭代次数
        tolerance: 收敛阈值

    返回值:
        七参数向量: [ΔX, ΔY, ΔZ, q₀, q₁, q₂, q₃, m]
    """

    n = source_coords.shape[0]
    x = np.array([0, 0, 0, 1, 0, 0, 0, 0])  # 初始化七参数，四元数初始化为(1, 0, 0, 0)

    for i in range(iterations):
        # 构建设计矩阵和观测向量
        A = np.zeros((3 * n, 8))
        L = target_coords.flatten() - source_coords.flatten()

        for j in range(n):
            x0, y0, z0 = source_coords[j]
            X0, Y0, Z0 = target_coords[j]
            q0, q1, q2, q3 = x[3:7]  # 获取当前迭代的四元数

            A[3 * j: 3 * j + 3] = np.array([
                [1, 0, 0, -2 * (q2 * z0 - q3 * y0), 2 * (q1 * z0 + q0 * y0), -2 * (q0 * z0 - q3 * x0), 2 * (q1 * y0 - q2 * x0), x0],
                [0, 1, 0, -2 * (q3 * x0 - q1 * z0), -2 * (q0 * x0 - q2 * z0), 2 * (q1 * x0 + q0 * z0), 2 * (q2 * x0 + q3 * y0), y0],
                [0, 0, 1, -2 * (q1 * y0 - q2 * x0), -2 * (q2 * y0 + q1 * x0), -2 * (q3 * y0 - q0 * x0), 2 * (q0 * y0 + q3 * z0), z0]
            ])

        # 计算七参数
        P = np.identity(3 * n)  # 假设权重相等
        x += np.linalg.solve(A.T @ P @ A, A.T @ P @ L)

        # 归一化四元数
        x[3:7] /= np.linalg.norm(x[3:7])

        # 更新观测方程
        source_coords = transform_coords_quaternion(source_coords, x)

        # 判断收敛
        residuals = target_coords - source_coords
        rmse = np.sqrt(np.sum(residuals ** 2) / n)
        if rmse < tolerance:
            break

    return x

def transform_coords_quaternion(coords, params):
    """
    应用七参数赫尔默特变换 (四元数形式) 到坐标。

    参数:
        coords: 坐标，形状为 (n, 3)
        params: 七参数向量

    返回值:
        变换后的坐标
    """

    ΔX, ΔY, ΔZ, q0, q1, q2, q3, m = params
    q = np.array([q0, q1, q2, q3])

    transformed_coords = []
    for coord in coords:
        # 将坐标转换为四元数形式 (虚部为坐标值，实部为0)
        p = np.array([0, *coord]) 
        
        # 应用四元数旋转
        transformed_p = quaternion_rotate(q, p)
        
        # 将结果转换回三维坐标
        transformed_coord = transformed_p[1:] 

        # 应用缩放和平移
        transformed_coord = (1 + m) * transformed_coord + np.array([ΔX, ΔY, ΔZ])
        transformed_coords.append(transformed_coord)

    return np.array(transformed_coords)

def quaternion_rotate(q, p):
    """
    使用四元数进行旋转。

    参数:
        q: 四元数
        p: 待旋转的点 (以四元数形式表示)

    返回值:
        旋转后的点 (以四元数形式表示)
    """
    return q * p * quaternion_conjugate(q)

def quaternion_conjugate(q):
    """
    计算四元数的共轭。
    """
    return np.array([q[0], -q[1], -q[2], -q[3]])

```
