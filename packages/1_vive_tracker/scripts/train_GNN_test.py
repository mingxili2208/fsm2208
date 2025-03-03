import torch
import torch.nn as nn
import torch.optim as optim
import numpy as np
import pandas as pd
from sklearn.preprocessing import MinMaxScaler
from sklearn.metrics import mean_squared_error
from torch_geometric.data import Data
from torch_geometric.nn import GINConv
from scipy.spatial import KDTree

# === 读取数据 ===
df_path = r"C:\Users\13366\Desktop\fsm_2208\packages\1_vive_tracker\scripts\corrected_tracker_log.csv"
df = pd.read_csv(df_path)

XZ_data = df[["X", "Z"]].values
laser_data = df[["laser_x", "laser_z"]].values

# === 归一化（使用相同的 MinMaxScaler）===
scaler = MinMaxScaler()
XZ_data = scaler.fit_transform(XZ_data)
laser_data = scaler.transform(laser_data)  # 确保 laser_data 也使用相同的 scaler

# === 选择 CUDA 或 CPU ===
device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
print(f"Using device: {device}")

# === 构造 K 近邻图（K=10）===
k = 10
tree = KDTree(XZ_data)
_, neighbors = tree.query(XZ_data, k=k + 1)  # 查询所有点的 k 近邻

edges = []
for i, nbrs in enumerate(neighbors):
    for neighbor in nbrs:
        if i != neighbor:  # 确保不连接自己
            edges.append([i, neighbor])
            edges.append([neighbor, i])

edges = torch.tensor(edges, dtype=torch.long).t().contiguous().to(device)

# === 创建 PyG 数据 ===
data = Data(x=torch.tensor(XZ_data, dtype=torch.float32).to(device),
            y=torch.tensor(laser_data, dtype=torch.float32).to(device),
            edge_index=edges)

# === GNN 模型（GINConv）===
class GNNModel(nn.Module):
    def __init__(self, input_dim=2, hidden_dim=128, output_dim=2):
        super(GNNModel, self).__init__()
        self.conv1 = GINConv(nn.Sequential(nn.Linear(input_dim, hidden_dim), nn.ReLU(),
                                           nn.Linear(hidden_dim, hidden_dim)))
        self.batch_norm1 = nn.BatchNorm1d(hidden_dim)
        
        self.conv2 = GINConv(nn.Sequential(nn.Linear(hidden_dim, hidden_dim), nn.ReLU(),
                                           nn.Linear(hidden_dim, hidden_dim)))
        self.batch_norm2 = nn.BatchNorm1d(hidden_dim)
        
        self.fc = nn.Linear(hidden_dim, output_dim)
        self.dropout = nn.Dropout(0.2)  # 加入 Dropout 以防止过拟合

    def forward(self, data):
        x, edge_index = data.x, data.edge_index
        x = torch.relu(self.batch_norm1(self.conv1(x, edge_index)))
        x = self.dropout(x)  # 应用 Dropout
        x = torch.relu(self.batch_norm2(self.conv2(x, edge_index)))
        x = self.dropout(x)
        x = self.fc(x)
        return x

# === 初始化模型 ===
model = GNNModel().to(device)
criterion = nn.MSELoss()
optimizer = optim.AdamW(model.parameters(), lr=0.001, weight_decay=1e-4)  # 提高学习率

# === 训练模型（20000 轮）===
epochs = 10000
for epoch in range(epochs):
    model.train()
    optimizer.zero_grad()
    outputs = model(data)
    loss = criterion(outputs, data.y)
    loss.backward()
    optimizer.step()

    if epoch % 1000 == 0:
        print(f"Epoch {epoch}, Loss: {loss.item():.6f}")

# === 计算 RMSE（确保反归一化正确）===
def inverse_transform(X_val, Z_val):
    input_data = np.array([[X_val, Z_val]])
    input_data = scaler.transform(input_data)  # 归一化

    input_tensor = torch.tensor(input_data, dtype=torch.float32).to(device)

    model.eval()
    with torch.no_grad():
        predicted = model(input_tensor.unsqueeze(0))  # 只对单个样本进行推理
        predicted = predicted.cpu().numpy()

    predicted = scaler.inverse_transform(predicted)  # 反归一化
    return predicted[0]

laser_pred = np.array([inverse_transform(X, Z) for X, Z in df[["X", "Z"]].values])
rmse_x = np.sqrt(mean_squared_error(df["laser_x"].values, laser_pred[:, 0]))
rmse_z = np.sqrt(mean_squared_error(df["laser_z"].values, laser_pred[:, 1]))

print(f"RMSE for laser_x: {rmse_x:.4f}")
print(f"RMSE for laser_z: {rmse_z:.4f}")