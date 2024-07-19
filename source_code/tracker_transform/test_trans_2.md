# code for trans

```python

import numpy as np
from scipy.optimize import least_squares

def quaternion_multiply(q1, q2):
    """计算两个四元数的乘积"""
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
    x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
    y = w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2
    z = w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2
    return np.array([w, x, y, z])

def residuals(params, pA_list, qA_list, pB_list, qB_list, wp=1.0, wq=1.0):
    """计算残差向量"""
    t = params[:3]  # 平移向量
    qAB = params[3:]  # 旋转四元数
    residuals = []
    for pA, qA, pB, qB in zip(pA_list, qA_list, pB_list, qB_list):
        pB_est, qB_est = transform(pA, qA, t, qAB)
        residuals.append(wp * (pB - pB_est))
        residuals.append(wq * (qB - qB_est))
    return np.concatenate(residuals)

def compute_transformation_quaternion(pA_list, qA_list, pB_list, qB_list):
    """计算使用四元数表示旋转的坐标系变换"""
    # 初始化参数估计值
    t0 = np.array([0, 0, 0])  # 初始平移为零向量
    qAB0 = np.array([1, 0, 0, 0])  # 初始旋转为单位四元数
    params0 = np.concatenate((t0, qAB0))

    # 使用 Levenberg-Marquardt 算法优化参数
    result = least_squares(
        residuals,
        params0,
        args=(pA_list, qA_list, pB_list, qB_list),
        method='lm'
    )

    # 获取优化后的参数
    t = result.x[:3]
    qAB = result.x[3:]
    return t, qAB

# 生成示例数据
pA_list = [np.random.rand(3) for _ in range(10)]
qA_list = [np.random.rand(4) for _ in range(10)]
# ... (根据实际情况生成 pB_list 和 qB_list)

# 计算变换
t, qAB = compute_transformation_quaternion(pA_list, qA_list, pB_list, qB_list)

# 打印结果
print("平移向量：", t)
print("旋转四元数：", qAB)

```