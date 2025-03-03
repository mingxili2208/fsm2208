import torch
import torch.nn as nn
import torch.optim as optim
import numpy as np
import pandas as pd
import open3d as o3d
from sklearn.preprocessing import StandardScaler
from sklearn.metrics import mean_squared_error

# === 设备选择（自动检测 GPU） ===
device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
print(f"Using device: {device}")

# === 读取数据 ===
df = pd.read_csv(r"C:\Users\13366\Desktop\fsm_2208\packages\1_vive_tracker\scripts\corrected_tracker_log_0303.csv")
XZ_data = df[["X", "Z"]].values
laser_data = df[["laser_x", "laser_z"]].values

# === Step 1: 计算刚体变换（ICP/SVD） ===
def apply_icp(source_points, target_points):
    """ 计算并应用 ICP 刚体变换 """
    source = o3d.geometry.PointCloud()
    target = o3d.geometry.PointCloud()
    source.points = o3d.utility.Vector3dVector(np.hstack((source_points, np.zeros((len(source_points), 1)))))
    target.points = o3d.utility.Vector3dVector(np.hstack((target_points, np.zeros((len(target_points), 1)))))

    threshold = 0.02  
    trans_init = np.eye(4)  
    reg_p2p = o3d.pipelines.registration.registration_icp(
        source, target, threshold, trans_init,
        o3d.pipelines.registration.TransformationEstimationPointToPoint()
    )

    transformation = reg_p2p.transformation[:2, :2]  # 旋转
    translation = reg_p2p.transformation[:2, 3]  # 平移
    transformed_XZ = (XZ_data @ transformation.T) + translation
    return transformed_XZ

XZ_data = apply_icp(XZ_data, laser_data)  # 预对齐数据

# === Step 2: 数据归一化 ===
scaler_XZ = StandardScaler()
scaler_laser = StandardScaler()
XZ_data = scaler_XZ.fit_transform(XZ_data)
laser_data = scaler_laser.fit_transform(laser_data)

# === Step 3: 数据增强（随机旋转+噪声） ===
def augment_data(X, noise_std=0.01):
    theta = np.random.uniform(-np.pi / 6, np.pi / 6)  # 旋转 ±30°
    rot_matrix = np.array([[np.cos(theta), -np.sin(theta)], [np.sin(theta), np.cos(theta)]])
    X_rot = X @ rot_matrix.T
    noise = np.random.normal(0, noise_std, X.shape)
    return X_rot + noise

XZ_data = augment_data(XZ_data)

# === 转换为 Tensor 并移动到 GPU ===
XZ_tensor = torch.tensor(XZ_data, dtype=torch.float32).to(device)
laser_tensor = torch.tensor(laser_data, dtype=torch.float32).to(device)

# === Step 4: Huber Loss（替换 RMSE） ===
class HuberLoss(nn.Module):
    def __init__(self, delta=1.0):
        super(HuberLoss, self).__init__()
        self.delta = delta

    def forward(self, pred, target):
        abs_error = torch.abs(pred - target)
        loss = torch.where(abs_error < self.delta, 0.5 * abs_error ** 2, self.delta * (abs_error - 0.5 * self.delta))
        return loss.mean()

# === Step 5: 用 MLP 代替 Transformer ===
class MLPModel(nn.Module):
    def __init__(self, input_dim=2, output_dim=2, hidden_dim=256):
        super(MLPModel, self).__init__()
        self.fc = nn.Sequential(
            nn.Linear(input_dim, hidden_dim),
            nn.ReLU(),
            nn.Linear(hidden_dim, hidden_dim),
            nn.ReLU(),
            nn.Linear(hidden_dim, output_dim)
        )

    def forward(self, x):
        return self.fc(x)

# === 初始化模型，并移动到 GPU ===
model = MLPModel().to(device)
criterion = HuberLoss(delta=0.1)  # Huber Loss
optimizer = optim.AdamW(model.parameters(), lr=0.001, weight_decay=1e-3)
scheduler = torch.optim.lr_scheduler.CosineAnnealingLR(optimizer, T_max=1000)

# === 训练模型 ===
epochs = 5000
batch_size = 128
for epoch in range(epochs):
    indices = np.random.choice(len(XZ_data), batch_size)
    batch_XZ = XZ_tensor[indices].to(device)
    batch_laser = laser_tensor[indices].to(device)

    optimizer.zero_grad()
    outputs = model(batch_XZ)
    loss = criterion(outputs, batch_laser)
    loss.backward()
    optimizer.step()
    scheduler.step()  # 调整学习率

    if epoch % 1000 == 0:
        print(f"Epoch {epoch}, Loss: {loss.item():.6f}")

# === 计算 RMSE（确保正确反归一化） ===
def inverse_transform(X_val, Z_val):
    """ 反归一化，并返回预测值 """
    input_data = np.array([[X_val, Z_val]])
    input_data = scaler_XZ.transform(input_data)
    input_tensor = torch.tensor(input_data, dtype=torch.float32).to(device)

    model.eval()
    with torch.no_grad():
        predicted = model(input_tensor).cpu().numpy()

    predicted = scaler_laser.inverse_transform(predicted)  # 反归一化
    return predicted[0]

# 计算所有数据的预测值
laser_pred = np.array([inverse_transform(X, Z) for X, Z in df[["X", "Z"]].values])

# 计算 RMSE（反归一化后的误差）
rmse_x = np.sqrt(mean_squared_error(df["laser_x"].values, laser_pred[:, 0]))
rmse_z = np.sqrt(mean_squared_error(df["laser_z"].values, laser_pred[:, 1]))

print(f"RMSE for laser_x: {rmse_x:.4f}")
print(f"RMSE for laser_z: {rmse_z:.4f}")