import pandas as pd
import numpy as np
import torch
import torch.nn as nn
import torch.optim as optim
from torch.utils.data import Dataset, DataLoader, TensorDataset
from sklearn.preprocessing import StandardScaler
from sklearn.model_selection import KFold
from sklearn.metrics import mean_squared_error
import matplotlib.pyplot as plt
import math

# 定义设备（使用GPU如果可用）
device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")
print(f"Using {device} device")

# 1. 空间变换网络定义
class SpatialTransformerNetwork(nn.Module):
    def __init__(self, input_dim=2, output_dim=2, hidden_dim=64):
        super(SpatialTransformerNetwork, self).__init__()
        
        # 定位网络 - 预测空间变换参数
        self.localization = nn.Sequential(
            nn.Linear(input_dim, hidden_dim),
            nn.ReLU(True),
            nn.BatchNorm1d(hidden_dim),
            nn.Linear(hidden_dim, hidden_dim),
            nn.ReLU(True),
            nn.BatchNorm1d(hidden_dim),
            nn.Linear(hidden_dim, 6)  # 2D仿射变换参数 (2x3矩阵)
        )
        
        # 初始化为恒等变换
        self.localization[-1].weight.data.zero_()
        self.localization[-1].bias.data.copy_(torch.tensor(
            [1, 0, 0, 0, 1, 0], dtype=torch.float))
        
        # 非线性残差网络 - 捕获非线性映射关系
        self.nonlinear_residual = nn.Sequential(
            nn.Linear(input_dim, hidden_dim),
            nn.LeakyReLU(0.2),
            nn.BatchNorm1d(hidden_dim),
            nn.Dropout(0.3),
            nn.Linear(hidden_dim, hidden_dim*2),
            nn.LeakyReLU(0.2),
            nn.BatchNorm1d(hidden_dim*2),
            nn.Dropout(0.3),
            nn.Linear(hidden_dim*2, hidden_dim),
            nn.LeakyReLU(0.2),
            nn.BatchNorm1d(hidden_dim),
            nn.Linear(hidden_dim, output_dim)
        )
        
        # 初始化残差为零
        self.nonlinear_residual[-1].weight.data.zero_()
        self.nonlinear_residual[-1].bias.data.zero_()
    
    def forward(self, x):
        # 获取变换参数
        theta = self.localization(x)
        theta = theta.view(-1, 2, 3)
        
        # 应用仿射变换
        # 将输入点从形状 [batch_size, 2] 转换为 [batch_size, 3]（齐次坐标）
        grid = torch.cat([x, torch.ones(x.size(0), 1, device=x.device)], dim=1)
        grid = grid.unsqueeze(2)  # [batch_size, 3, 1]
        
        # 应用变换
        transformed = torch.bmm(theta, grid)  # [batch_size, 2, 1]
        transformed = transformed.squeeze(2)  # [batch_size, 2]
        
        # 应用非线性残差
        residual = self.nonlinear_residual(x)
        
        # 合并结果
        result = transformed + residual
        
        return result

# 2. 训练函数
def train_model(model, train_loader, criterion, optimizer, scheduler=None):
    model.train()
    running_loss = 0.0
    
    for inputs, targets in train_loader:
        inputs, targets = inputs.to(device), targets.to(device)
        
        # 前向传播
        outputs = model(inputs)
        loss = criterion(outputs, targets)
        
        # 反向传播和优化
        optimizer.zero_grad()
        loss.backward()
        optimizer.step()
        
        running_loss += loss.item() * inputs.size(0)
    
    epoch_loss = running_loss / len(train_loader.dataset)
    
    if scheduler:
        scheduler.step(epoch_loss)
        
    return epoch_loss

# 3. 评估函数
def evaluate_model(model, test_loader, criterion):
    model.eval()
    running_loss = 0.0
    all_targets = []
    all_outputs = []
    
    with torch.no_grad():
        for inputs, targets in test_loader:
            inputs, targets = inputs.to(device), targets.to(device)
            
            # 前向传播
            outputs = model(inputs)
            loss = criterion(outputs, targets)
            
            running_loss += loss.item() * inputs.size(0)
            
            # 收集结果用于计算RMSE
            all_targets.append(targets.cpu().numpy())
            all_outputs.append(outputs.cpu().numpy())
    
    # 计算平均损失
    epoch_loss = running_loss / len(test_loader.dataset)
    
    # 合并所有批次的结果
    all_targets = np.vstack(all_targets)
    all_outputs = np.vstack(all_outputs)
    
    # 计算RMSE
    rmse_laser_x = math.sqrt(mean_squared_error(all_targets[:, 0], all_outputs[:, 0]))
    rmse_laser_z = math.sqrt(mean_squared_error(all_targets[:, 1], all_outputs[:, 1]))
    rmse_overall = math.sqrt(mean_squared_error(all_targets, all_outputs))
    
    return epoch_loss, rmse_laser_x, rmse_laser_z, rmse_overall

# 4. 主函数：加载数据，进行5折交叉验证
def main():
    # 加载数据
    print("Loading data...")
    data = pd.read_csv(r"C:\Users\13366\Desktop\fsm_2208\packages\1_vive_tracker\scripts\corrected_tracker_log_0303.csv")
    
    # 输入特征和目标变量
    X = data[["X", "Z"]].values
    y = data[["laser_x", "laser_z"]].values
    
    # 设置5折交叉验证
    k_folds = 5
    kf = KFold(n_splits=k_folds, shuffle=True, random_state=42)
    
    # 存储每折的结果
    fold_results = []
    
    # 为每个折运行训练和评估
    for fold, (train_idx, test_idx) in enumerate(kf.split(X)):
        print(f"\nFold {fold+1}/{k_folds}")
        
        # 分割数据
        X_train, X_test = X[train_idx], X[test_idx]
        y_train, y_test = y[train_idx], y[test_idx]
        
        # 标准化
        scaler_X = StandardScaler()
        scaler_y = StandardScaler()
        
        X_train_scaled = scaler_X.fit_transform(X_train)
        X_test_scaled = scaler_X.transform(X_test)
        
        y_train_scaled = scaler_y.fit_transform(y_train)
        y_test_scaled = scaler_y.transform(y_test)
        
        # 转换为PyTorch张量
        X_train_tensor = torch.tensor(X_train_scaled, dtype=torch.float32)
        y_train_tensor = torch.tensor(y_train_scaled, dtype=torch.float32)
        X_test_tensor = torch.tensor(X_test_scaled, dtype=torch.float32)
        y_test_tensor = torch.tensor(y_test_scaled, dtype=torch.float32)
        
        # 创建数据加载器
        train_dataset = TensorDataset(X_train_tensor, y_train_tensor)
        test_dataset = TensorDataset(X_test_tensor, y_test_tensor)
        
        train_loader = DataLoader(train_dataset, batch_size=32, shuffle=True)
        test_loader = DataLoader(test_dataset, batch_size=32)
        
        # 创建模型
        model = SpatialTransformerNetwork(input_dim=2, output_dim=2, hidden_dim=128).to(device)
        
        # 定义损失函数和优化器
        criterion = nn.MSELoss()
        optimizer = optim.Adam(model.parameters(), lr=0.001, weight_decay=1e-5)
        scheduler = optim.lr_scheduler.ReduceLROnPlateau(optimizer, 'min', patience=5, factor=0.5)
        
        # 训练模型
        epochs = 100
        train_losses = []
        test_losses = []
        best_rmse = float('inf')
        best_model_state = None
        
        print("Training model...")
        for epoch in range(epochs):
            # 训练
            train_loss = train_model(model, train_loader, criterion, optimizer)
            train_losses.append(train_loss)
            
            # 评估
            test_loss, rmse_x, rmse_z, rmse_overall = evaluate_model(model, test_loader, criterion)
            test_losses.append(test_loss)
            
            # 保存最佳模型
            if rmse_overall < best_rmse:
                best_rmse = rmse_overall
                best_model_state = model.state_dict().copy()
            
            # 打印进度
            if (epoch+1) % 10 == 0:
                print(f'Epoch {epoch+1}/{epochs}, Train Loss: {train_loss:.6f}, Test Loss: {test_loss:.6f}, RMSE Overall: {rmse_overall:.6f}')
        
        # 加载最佳模型
        model.load_state_dict(best_model_state)
        
        # 在未缩放的数据上进行最终评估
        model.eval()
        with torch.no_grad():
            # 获取所有测试预测
            predictions_scaled = model(X_test_tensor.to(device)).cpu().numpy()
            
            # 转换回原始尺度
            predictions = scaler_y.inverse_transform(predictions_scaled)
            
            # 计算RMSE
            rmse_laser_x = math.sqrt(mean_squared_error(y_test[:, 0], predictions[:, 0]))
            rmse_laser_z = math.sqrt(mean_squared_error(y_test[:, 1], predictions[:, 1]))
            rmse_overall = math.sqrt(mean_squared_error(y_test, predictions))
        
        print(f"\nFinal results for fold {fold+1}:")
        print(f"RMSE laser_x: {rmse_laser_x:.6f}")
        print(f"RMSE laser_z: {rmse_laser_z:.6f}")
        print(f"RMSE overall: {rmse_overall:.6f}")
        
        # 存储结果
        fold_results.append({
            'fold': fold+1,
            'rmse_laser_x': rmse_laser_x,
            'rmse_laser_z': rmse_laser_z,
            'rmse_overall': rmse_overall,
            'train_losses': train_losses,
            'test_losses': test_losses,
            'model_state': best_model_state,
            'scaler_X': scaler_X,
            'scaler_y': scaler_y,
        })
        
        # 绘制损失曲线
        plt.figure(figsize=(10, 5))
        plt.plot(train_losses, label='Training Loss')
        plt.plot(test_losses, label='Validation Loss')
        plt.xlabel('Epochs')
        plt.ylabel('Loss')
        plt.title(f'Fold {fold+1} - Training and Validation Loss')
        plt.legend()
        plt.grid(True)
        plt.savefig(f'fold_{fold+1}_loss.png')
        plt.close()
        
        # 绘制预测vs实际值
        plt.figure(figsize=(12, 5))
        
        # laser_x 预测vs实际
        plt.subplot(1, 2, 1)
        plt.scatter(y_test[:, 0], predictions[:, 0], alpha=0.5)
        min_val = min(y_test[:, 0].min(), predictions[:, 0].min())
        max_val = max(y_test[:, 0].max(), predictions[:, 0].max())
        plt.plot([min_val, max_val], [min_val, max_val], 'r--')
        plt.xlabel('Actual laser_x')
        plt.ylabel('Predicted laser_x')
        plt.title(f'laser_x: RMSE = {rmse_laser_x:.6f}')
        
        # laser_z 预测vs实际
        plt.subplot(1, 2, 2)
        plt.scatter(y_test[:, 1], predictions[:, 1], alpha=0.5)
        min_val = min(y_test[:, 1].min(), predictions[:, 1].min())
        max_val = max(y_test[:, 1].max(), predictions[:, 1].max())
        plt.plot([min_val, max_val], [min_val, max_val], 'r--')
        plt.xlabel('Actual laser_z')
        plt.ylabel('Predicted laser_z')
        plt.title(f'laser_z: RMSE = {rmse_laser_z:.6f}')
        
        plt.tight_layout()
        plt.savefig(f'fold_{fold+1}_predictions.png')
        plt.close()
    
    # 5. 计算和显示所有折的平均结果
    avg_rmse_x = np.mean([result['rmse_laser_x'] for result in fold_results])
    avg_rmse_z = np.mean([result['rmse_laser_z'] for result in fold_results])
    avg_rmse_overall = np.mean([result['rmse_overall'] for result in fold_results])
    
    std_rmse_x = np.std([result['rmse_laser_x'] for result in fold_results])
    std_rmse_z = np.std([result['rmse_laser_z'] for result in fold_results])
    std_rmse_overall = np.std([result['rmse_overall'] for result in fold_results])
    
    print("\n====== Cross-Validation Results ======")
    print(f"Average RMSE laser_x: {avg_rmse_x:.6f} ± {std_rmse_x:.6f}")
    print(f"Average RMSE laser_z: {avg_rmse_z:.6f} ± {std_rmse_z:.6f}")
    print(f"Average RMSE overall: {avg_rmse_overall:.6f} ± {std_rmse_overall:.6f}")
    
    # 6. 保存最佳模型（基于总体RMSE）
    best_fold_idx = np.argmin([result['rmse_overall'] for result in fold_results])
    best_result = fold_results[best_fold_idx]
    
    print(f"\nBest model from fold {best_result['fold']} with RMSE: {best_result['rmse_overall']:.6f}")
    
    # 重新创建最佳模型并加载权重
    best_model = SpatialTransformerNetwork(input_dim=2, output_dim=2, hidden_dim=128).to(device)
    best_model.load_state_dict(best_result['model_state'])
    
    # 保存模型和缩放器
    torch.save({
        'model_state_dict': best_result['model_state'],
        'scaler_X': best_result['scaler_X'],
        'scaler_y': best_result['scaler_y'],
        'rmse_x': best_result['rmse_laser_x'],
        'rmse_z': best_result['rmse_laser_z'],
        'rmse_overall': best_result['rmse_overall'],
    }, 'best_spatial_transformer_model.pth')
    
    print("Best model saved successfully!")
    
    # 7. 比较STN和线性回归
    print("\n====== Comparing with Linear Regression ======")
    from sklearn.linear_model import LinearRegression
    
    # 将所有折的结果合并为一个DataFrame，方便分析
    results_df = pd.DataFrame(fold_results)
    
    # 为线性回归执行相同的5折交叉验证过程
    lr_fold_results = []
    
    for fold, (train_idx, test_idx) in enumerate(kf.split(X)):
        # 分割数据
        X_train, X_test = X[train_idx], X[test_idx]
        y_train, y_test = y[train_idx], y[test_idx]
        
        # 训练线性回归模型 - laser_x
        lr_x = LinearRegression()
        lr_x.fit(X_train, y_train[:, 0])
        
        # 训练线性回归模型 - laser_z
        lr_z = LinearRegression()
        lr_z.fit(X_train, y_train[:, 1])
        
        # 预测
        y_pred_x = lr_x.predict(X_test)
        y_pred_z = lr_z.predict(X_test)
        
        # 合并预测结果
        y_pred = np.column_stack((y_pred_x, y_pred_z))
        
        # 计算RMSE
        rmse_x = math.sqrt(mean_squared_error(y_test[:, 0], y_pred[:, 0]))
        rmse_z = math.sqrt(mean_squared_error(y_test[:, 1], y_pred[:, 1]))
        rmse_overall = math.sqrt(mean_squared_error(y_test, y_pred))
        
        lr_fold_results.append({
            'fold': fold+1,
            'rmse_laser_x': rmse_x,
            'rmse_laser_z': rmse_z,
            'rmse_overall': rmse_overall
        })
    
    # 计算线性回归的平均结果
    lr_avg_rmse_x = np.mean([result['rmse_laser_x'] for result in lr_fold_results])
    lr_avg_rmse_z = np.mean([result['rmse_laser_z'] for result in lr_fold_results])
    lr_avg_rmse_overall = np.mean([result['rmse_overall'] for result in lr_fold_results])
    
    lr_std_rmse_x = np.std([result['rmse_laser_x'] for result in lr_fold_results])
    lr_std_rmse_z = np.std([result['rmse_laser_z'] for result in lr_fold_results])
    lr_std_rmse_overall = np.std([result['rmse_overall'] for result in lr_fold_results])
    
    print("\nLinear Regression Results:")
    print(f"Average RMSE laser_x: {lr_avg_rmse_x:.6f} ± {lr_std_rmse_x:.6f}")
    print(f"Average RMSE laser_z: {lr_avg_rmse_z:.6f} ± {lr_std_rmse_z:.6f}")
    print(f"Average RMSE overall: {lr_avg_rmse_overall:.6f} ± {lr_std_rmse_overall:.6f}")
    
    print("\nSpatial Transformer Network Results:")
    print(f"Average RMSE laser_x: {avg_rmse_x:.6f} ± {std_rmse_x:.6f}")
    print(f"Average RMSE laser_z: {avg_rmse_z:.6f} ± {std_rmse_z:.6f}")
    print(f"Average RMSE overall: {avg_rmse_overall:.6f} ± {std_rmse_overall:.6f}")
    
    # 计算改进百分比
    improvement_x = ((lr_avg_rmse_x - avg_rmse_x) / lr_avg_rmse_x) * 100
    improvement_z = ((lr_avg_rmse_z - avg_rmse_z) / lr_avg_rmse_z) * 100
    improvement_overall = ((lr_avg_rmse_overall - avg_rmse_overall) / lr_avg_rmse_overall) * 100
    
    print(f"\nImprovement over Linear Regression:")
    print(f"RMSE laser_x: {improvement_x:.2f}%")
    print(f"RMSE laser_z: {improvement_z:.2f}%")
    print(f"RMSE overall: {improvement_overall:.2f}%")
    
    # 8. 创建预测函数，以便稍后使用
    def predict_coordinates(X_value, Z_value):
        # 加载最佳模型
        checkpoint = torch.load('best_spatial_transformer_model.pth')
        model = SpatialTransformerNetwork(input_dim=2, output_dim=2, hidden_dim=128).to(device)
        model.load_state_dict(checkpoint['model_state_dict'])
        scaler_X = checkpoint['scaler_X']
        scaler_y = checkpoint['scaler_y']
        
        # 缩放输入
        input_scaled = scaler_X.transform([[X_value, Z_value]])
        input_tensor = torch.tensor(input_scaled, dtype=torch.float32).to(device)
        
        # 预测
        model.eval()
        with torch.no_grad():
            output_scaled = model(input_tensor)
            output_np = output_scaled.cpu().numpy()
            output = scaler_y.inverse_transform(output_np)
        
        return output[0][0], output[0][1]  # laser_x, laser_z
    
    # 示例预测
    X_value = 0.5
    Z_value = -4.5
    laser_x_pred, laser_z_pred = predict_coordinates(X_value, Z_value)
    print(f"\nExample prediction for X={X_value}, Z={Z_value}:")
    print(f"Predicted laser_x: {laser_x_pred:.4f}, predicted laser_z: {laser_z_pred:.4f}")

if __name__ == "__main__":
    main()