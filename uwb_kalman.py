# 使用卡尔曼滤波处理uwb得到的位置数据，实现更精确的定位

import numpy as np

# 初始化状态向量和协方差矩阵
x = np.array([0, 0, 0, 0])  # 初始位置和速度
P = np.eye(4)  # 初始协方差矩阵

# 状态转移矩阵
dt = 0.02  # 时间间隔，1秒更新50次
F = np.array([
    [1, dt, 0, 0],
    [0, 1, 0, 0],
    [0, 0, 1, dt],
    [0, 0, 0, 1]
])

# 观测矩阵
H = np.array([
    [1, 0, 0, 0],
    [0, 0, 1, 0]
])

# 过程噪声协方差矩阵
Q = np.eye(4) * 0.01

# 测量噪声协方差矩阵
R = np.eye(2) * 2


# 卡尔曼滤波
def kalman_filter(x, P, z):
    # 预测步骤
    x_pred = np.dot(F, x)
    P_pred = np.dot(F, np.dot(P, F.T)) + Q

    # 更新步骤
    y = z - np.dot(H, x_pred)
    S = np.dot(H, np.dot(P_pred, H.T)) + R
    K = np.dot(P_pred, np.dot(H.T, np.linalg.inv(S)))

    x = x_pred + np.dot(K, y)
    P = P_pred - np.dot(K, np.dot(H, P_pred))

    return x, P


# # 示例位置数据
# positions = [
#     [1.0, 1.0],
#     [1.1, 1.05],
#     # ... 更多位置数据
# ]

# # 使用卡尔曼滤波处理位置数据
# for pos in positions:
#     x, P = kalman_filter(x, P, pos)
#     print(f"Filtered position: {x[0]:.2f}, {x[2]:.2f}")
