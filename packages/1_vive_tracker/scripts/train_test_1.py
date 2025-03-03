import numpy as np
import pandas as pd
from sklearn.preprocessing import StandardScaler
from sklearn.metrics import mean_squared_error
import torch
import torch.nn as nn
import torch.optim as optim

# === 读取数据 ===
df = pd.read_csv("corrected_tracker_log.csv")

# === 获取数据 ===
XZ_data = df[["X", "Z"]].values  # 输入 (X, Z)
laser_data = df[["laser_x", "laser_z"]].values  # 目标输出 (laser_x, laser_z)

# === 数据归一化（标准化到 0 均值, 1 标准差） ===
scaler_XZ = StandardScaler()
scaler_laser = StandardScaler()

XZ_data = scaler_XZ.fit_transform(XZ_data)
laser_data = scaler_laser.fit_transform(laser_data)

# ============ 1️⃣ 训练神经网络（X, Z → laser_x, laser_z） ============
class InverseMLP(nn.Module):
    def __init__(self):
        super(InverseMLP, self).__init__()
        self.fc1 = nn.Linear(2, 256)  # 增加神经元
        self.fc2 = nn.Linear(256, 256)
        self.fc3 = nn.Linear(256, 256)
        self.fc4 = nn.Linear(256, 2)  # 输出 laser_x, laser_z
        self.bn1 = nn.BatchNorm1d(256)  # 批归一化
        self.bn2 = nn.BatchNorm1d(256)
        self.bn3 = nn.BatchNorm1d(256)
        self.dropout = nn.Dropout(0.1)  # Dropout 防止过拟合

    def forward(self, x):
        x = torch.relu(self.bn1(self.fc1(x)))
        x = self.dropout(x)
        x = torch.relu(self.bn2(self.fc2(x)))
        x = self.dropout(x)
        x = torch.relu(self.bn3(self.fc3(x)))
        x = self.dropout(x)
        x = self.fc4(x)
        return x

# **初始化神经网络**
inverse_model = InverseMLP()
criterion = nn.MSELoss()
optimizer = optim.AdamW(inverse_model.parameters(), lr=0.001, weight_decay=1e-5)
scheduler = optim.lr_scheduler.ReduceLROnPlateau(optimizer, mode='min', factor=0.5, patience=500, verbose=True)

# **转换数据**
XZ_tensor = torch.tensor(XZ_data, dtype=torch.float32)
laser_tensor = torch.tensor(laser_data, dtype=torch.float32)

# **训练神经网络**
epochs = 20000  # 增加训练轮数
for epoch in range(epochs):
    optimizer.zero_grad()
    outputs = inverse_model(XZ_tensor)
    loss = criterion(outputs, laser_tensor)
    loss.backward()
    optimizer.step()
    
    scheduler.step(loss)  # 自动调整学习率

    if epoch % 1000 == 0:
        print(f"Epoch {epoch}, Loss: {loss.item()}")

# ============ 2️⃣ 计算 RMSE 误差（X, Z -> laser_x, laser_z） ============
def inverse_transform(X_val, Z_val):
    """使用 (X, Z) 预测 (laser_x, laser_z)"""
    input_data = np.array([[X_val, Z_val]])
    input_data = scaler_XZ.transform(input_data)  # 归一化
    input_tensor = torch.tensor(input_data, dtype=torch.float32)

    # **切换到推理模式，防止 BN 计算错误**
    inverse_model.eval()  

    with torch.no_grad():
        predicted = inverse_model(input_tensor).numpy()

    predicted = scaler_laser.inverse_transform(predicted)  # 反归一化
    return predicted[0]  # 返回 (laser_x, laser_z)
# **计算所有测试点的预测值**
laser_pred = np.array([inverse_transform(X, Z) for X, Z in df[["X", "Z"]].values])

# **计算 RMSE 误差**
rmse_x = np.sqrt(mean_squared_error(df["laser_x"].values, laser_pred[:, 0]))  # laser_x 的 RMSE
rmse_z = np.sqrt(mean_squared_error(df["laser_z"].values, laser_pred[:, 1]))  # laser_z 的 RMSE

print(f"✅ RMSE for laser_x: {rmse_x:.6f}")
print(f"✅ RMSE for laser_z: {rmse_z:.6f}")