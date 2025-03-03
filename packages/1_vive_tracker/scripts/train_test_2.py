import torch
import torch.nn as nn
import torch.optim as optim
import numpy as np
import pandas as pd
from sklearn.preprocessing import StandardScaler
from sklearn.metrics import mean_squared_error

# === 设备选择（自动检测 GPU） ===
device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
print(f"Using device: {device}")

# === 读取数据 ===
df = pd.read_csv(r"C:\Users\13366\Desktop\fsm_2208\packages\1_vive_tracker\scripts\corrected_tracker_log.csv")
XZ_data = df[["X", "Z"]].values
laser_data = df[["laser_x", "laser_z"]].values

# === 数据归一化 ===
scaler_XZ = StandardScaler()
scaler_laser = StandardScaler()
XZ_data = scaler_XZ.fit_transform(XZ_data)
laser_data = scaler_laser.fit_transform(laser_data)

# === 转换为 Tensor 并移动到 GPU（如果可用） ===
XZ_tensor = torch.tensor(XZ_data, dtype=torch.float32).to(device)
laser_tensor = torch.tensor(laser_data, dtype=torch.float32).to(device)

# === RMSE Loss ===
class RMSELoss(nn.Module):
    def forward(self, pred, target):
        return torch.sqrt(torch.mean((pred - target) ** 2))

# === Transformer Model ===
class TransformerModel(nn.Module):
    def __init__(self, input_dim=2, output_dim=2, d_model=64, num_heads=4, num_layers=4):
        super(TransformerModel, self).__init__()
        self.embedding = nn.Linear(input_dim, d_model)
        encoder_layer = nn.TransformerEncoderLayer(d_model=d_model, nhead=num_heads)
        self.transformer = nn.TransformerEncoder(encoder_layer, num_layers=num_layers)
        self.dropout = nn.Dropout(0.2)  # 防止过拟合
        self.fc = nn.Linear(d_model, output_dim)

    def forward(self, x):
        x = self.embedding(x).unsqueeze(1)  # [batch, 1, d_model]
        x = self.dropout(x)  # 加入 Dropout
        x = self.transformer(x).squeeze(1)  # [batch, d_model]
        x = self.fc(x)
        return x

# === 初始化模型，并移动到 GPU（如果可用） ===
model = TransformerModel().to(device)
criterion = RMSELoss()  # 使用 RMSE Loss
optimizer = optim.AdamW(model.parameters(), lr=0.0005, weight_decay=1e-4)

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

    if epoch % 1000 == 0:  # 调整打印间隔，提高训练速度
        print(f"Epoch {epoch}, Loss: {loss.item():.6f}")

# === 计算 RMSE（确保正确反归一化） ===
def inverse_transform(X_val, Z_val):
    """ 反归一化，并返回预测值 """
    input_data = np.array([[X_val, Z_val]])
    input_data = scaler_XZ.transform(input_data)
    input_tensor = torch.tensor(input_data, dtype=torch.float32).to(device)

    model.eval()
    with torch.no_grad():
        predicted = model(input_tensor).cpu().numpy()  # 移回 CPU 进行计算

    predicted = scaler_laser.inverse_transform(predicted)  # 反归一化
    return predicted[0]

# 计算所有数据的预测值
laser_pred = np.array([inverse_transform(X, Z) for X, Z in df[["X", "Z"]].values])

# 计算 RMSE（反归一化后的误差）
rmse_x = np.sqrt(mean_squared_error(df["laser_x"].values, laser_pred[:, 0]))
rmse_z = np.sqrt(mean_squared_error(df["laser_z"].values, laser_pred[:, 1]))

print(f"RMSE for laser_x: {rmse_x:.4f}")
print(f"RMSE for laser_z: {rmse_z:.4f}")