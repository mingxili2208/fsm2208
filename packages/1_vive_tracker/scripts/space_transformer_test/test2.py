import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import seaborn as sns
from sklearn.preprocessing import StandardScaler, RobustScaler, QuantileTransformer, PolynomialFeatures
from sklearn.model_selection import train_test_split
from sklearn.feature_selection import SelectKBest, f_regression
from sklearn.ensemble import RandomForestRegressor, GradientBoostingRegressor, AdaBoostRegressor, VotingRegressor
from sklearn.svm import SVR
from sklearn.kernel_ridge import KernelRidge
from sklearn.metrics import mean_squared_error
from sklearn.cluster import KMeans
from sklearn.base import BaseEstimator, RegressorMixin
import torch
import torch.nn as nn
import torch.optim as optim
from torch.utils.data import TensorDataset, DataLoader
import time
import math
import os
import warnings
warnings.filterwarnings('ignore')

# 修复样式设置 - 使用新的matplotlib样式命名
# plt.style.use('seaborn-whitegrid')  # 旧版本写法
plt.style.use('seaborn-v0_8-whitegrid')  # 新版本写法，如果这个仍然不工作，可以直接去掉这一行
sns.set_palette("coolwarm")

# Check if CUDA is available
device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
print(f"Using {device} device")

# Create output directory for results
os.makedirs("z_optimization_results", exist_ok=True)

# Z Coordinate Neural Network
class ZCoordinateTransformer(nn.Module):
    def __init__(self, input_dim=2):
        super(ZCoordinateTransformer, self).__init__()
        
        # Feature extraction
        self.feature_extractor = nn.Sequential(
            nn.Linear(input_dim, 128),
            nn.LeakyReLU(0.2),
            nn.BatchNorm1d(128),
            nn.Dropout(0.3),
            nn.Linear(128, 64),
            nn.LeakyReLU(0.2),
            nn.BatchNorm1d(64),
        )
        
        # Z-specific processing branch
        self.z_branch = nn.Sequential(
            nn.Linear(64, 32),
            nn.LeakyReLU(0.2),
            nn.BatchNorm1d(32),
            nn.Linear(32, 16),
            nn.LeakyReLU(0.2),
            nn.Linear(16, 1)
        )
        
        # Residual connection
        self.residual = nn.Linear(input_dim, 1)
        
        # Initialize weights
        self._initialize_weights()
    
    def _initialize_weights(self):
        for m in self.modules():
            if isinstance(m, nn.Linear):
                nn.init.kaiming_normal_(m.weight, nonlinearity='leaky_relu')
                if m.bias is not None:
                    nn.init.constant_(m.bias, 0)
    
    def forward(self, x):
        # Feature extraction
        features = self.feature_extractor(x)
        
        # Z branch
        z_out = self.z_branch(features)
        
        # Residual connection
        res = self.residual(x)
        
        # Combine
        return z_out + res

# Piecewise Regression Model
class PiecewiseRegression:
    """Piecewise regression model - uses different models for different regions"""
    
    def __init__(self, n_segments=3, base_model=None):
        self.n_segments = n_segments
        self.kmeans = KMeans(n_clusters=n_segments, random_state=42)
        
        # Base model for each segment
        if base_model is None:
            self.models = [RandomForestRegressor(n_estimators=100, random_state=42) 
                          for _ in range(n_segments)]
        else:
            self.models = [base_model() for _ in range(n_segments)]
        
        self.segment_bounds = None
    
    def fit(self, X, y):
        # Use KMeans to segment the input space
        # Only use Z coordinate for clustering
        Z = X[:, 1].reshape(-1, 1)
        self.kmeans.fit(Z)
        segments = self.kmeans.predict(Z)
        
        # Record the boundaries of each segment (for visualization and explanation)
        self.segment_bounds = []
        for i in range(self.n_segments):
            segment_points = Z[segments == i]
            if len(segment_points) > 0:
                self.segment_bounds.append((np.min(segment_points), np.max(segment_points)))
            else:
                self.segment_bounds.append((0, 0))  # Empty segment
        
        # Train a separate model for each segment
        for i in range(self.n_segments):
            segment_mask = (segments == i)
            if np.sum(segment_mask) > 0:  # Ensure segment has data points
                self.models[i].fit(X[segment_mask], y[segment_mask])
        
        return self
    
    def predict(self, X):
        # Use KMeans to predict segment for each point
        Z = X[:, 1].reshape(-1, 1)
        segments = self.kmeans.predict(Z)
        
        # Initialize prediction array
        if len(X) == 0:
            return np.array([])
            
        # Initialize based on dimensionality of output
        sample_pred = self.models[0].predict(X[0:1])
        if len(sample_pred.shape) > 1:
            predictions = np.zeros((X.shape[0], sample_pred.shape[1]))
        else:
            predictions = np.zeros(X.shape[0])
        
        # Predict using appropriate model for each segment
        for i in range(self.n_segments):
            segment_mask = (segments == i)
            if np.sum(segment_mask) > 0:  # Ensure segment has data points
                segment_indices = np.where(segment_mask)[0]
                segment_predictions = self.models[i].predict(X[segment_mask])
                
                # Insert predictions
                for j, idx in enumerate(segment_indices):
                    predictions[idx] = segment_predictions[j] if isinstance(segment_predictions, np.ndarray) and len(segment_predictions.shape) == 1 else segment_predictions[j, 0]
        
        return predictions

# Complete Coordinate Transformer
class CoordinateTransformer(BaseEstimator, RegressorMixin):
    """
    Complete coordinate transformer using different specialized models
    for X and Z coordinates
    """
    
    def __init__(self, x_model=None, z_model=None):
        self.x_model = x_model  # X coordinate prediction model
        self.z_model = z_model  # Z coordinate prediction model
    
    def fit(self, X, y):
        """
        Train X and Z coordinate transformation models
        
        Parameters:
        X: input features (n_samples, n_features)
        y: target coordinates (n_samples, 2) - [laser_x, laser_z]
        """
        # Separate X and Z targets
        y_x = y[:, 0]  # laser_x
        y_z = y[:, 1]  # laser_z
        
        # If no X model provided, use Random Forest (best performing method)
        if self.x_model is None:
            self.x_model = RandomForestRegressor(n_estimators=200, random_state=42)
        
        # Train X coordinate model
        print("Training X coordinate model...")
        self.x_model.fit(X, y_x)
        
        # Train Z coordinate model if not provided
        if self.z_model is None:
            print("Creating default Z coordinate model...")
            self.z_model = RandomForestRegressor(n_estimators=200, random_state=42)
            self.z_model.fit(X, y_z)
        
        return self
    
    def predict(self, X):
        """
        Predict transformed coordinates
        
        Parameters:
        X: input features (n_samples, n_features)
        
        Returns:
        Predicted coordinates (n_samples, 2) - [laser_x, laser_z]
        """
        # Predict X coordinate
        pred_x = self.x_model.predict(X)
        
        # Use specialized model for Z coordinate
        if hasattr(self, 'z_model') and self.z_model is not None:
            if isinstance(self.z_model, ZCoordinateTransformer):
                # If it's a PyTorch model
                X_tensor = torch.tensor(X, dtype=torch.float32)
                with torch.no_grad():
                    pred_z = self.z_model(X_tensor).cpu().numpy().ravel()
            else:
                # Other model types
                pred_z = self.z_model.predict(X)
                if isinstance(pred_z, np.ndarray) and len(pred_z.shape) > 1:
                    pred_z = pred_z.ravel()
        else:
            # If no Z model provided, use Random Forest
            z_model = RandomForestRegressor(n_estimators=200, random_state=42)
            z_model.fit(X, y[:, 1])
            pred_z = z_model.predict(X)
        
        # Combine predictions
        predictions = np.column_stack((pred_x, pred_z))
        
        return predictions

# Extended feature creation for Z coordinate
def create_z_features(X_input):
    """Create extended feature set for Z coordinate"""
    # Original features
    features = X_input.copy()
    
    # Add polynomial features for Z
    z_values = features[:, 1].reshape(-1, 1)
    
    # Polynomial features
    poly = PolynomialFeatures(degree=3, include_bias=False)
    z_poly = poly.fit_transform(z_values)
    
    # Add trigonometric features (useful for periodic data)
    z_sin = np.sin(z_values)
    z_cos = np.cos(z_values)
    
    # Add log transform (useful for power-law distributions)
    # Ensure data is positive
    z_min = z_values.min()
    if z_min <= 0:
        z_log = np.log(z_values - z_min + 1)
    else:
        z_log = np.log(z_values)
    
    # Add X and Z interaction features
    x_values = features[:, 0].reshape(-1, 1)
    xz_product = x_values * z_values
    x_squared_z = x_values**2 * z_values
    x_z_squared = x_values * z_values**2
    
    # Combine all features
    all_features = np.column_stack((
        features,           # Original X,Z
        z_poly,             # Z polynomial features
        z_sin, z_cos,       # Trigonometric features
        z_log,              # Log features
        xz_product,         # X*Z interaction
        x_squared_z,        # X²*Z interaction
        x_z_squared         # X*Z² interaction
    ))
    
    return all_features

# Train specialized Z coordinate model
def train_z_optimized_model(X_train, z_train, X_val, z_val, epochs=200, batch_size=32, lr=0.001):
    try:
        # Convert to PyTorch tensors
        X_train_tensor = torch.tensor(X_train, dtype=torch.float32).to(device)
        z_train_tensor = torch.tensor(z_train, dtype=torch.float32).to(device)
        X_val_tensor = torch.tensor(X_val, dtype=torch.float32).to(device)
        z_val_tensor = torch.tensor(z_val, dtype=torch.float32).to(device)
        
        # Create data loaders
        train_dataset = TensorDataset(X_train_tensor, z_train_tensor)
        train_loader = DataLoader(train_dataset, batch_size=batch_size, shuffle=True)
        
        val_dataset = TensorDataset(X_val_tensor, z_val_tensor)
        val_loader = DataLoader(val_dataset, batch_size=batch_size)
        
        # Create model
        model = ZCoordinateTransformer(input_dim=X_train.shape[1]).to(device)
        
        # Loss function and optimizer
        criterion = nn.MSELoss()
        optimizer = optim.Adam(model.parameters(), lr=lr, weight_decay=1e-5)
        scheduler = optim.lr_scheduler.ReduceLROnPlateau(optimizer, 'min', patience=10, factor=0.5)
        
        # Training loop
        best_val_loss = float('inf')
        best_model_state = None
        train_losses = []
        val_losses = []
        patience = 20
        patience_counter = 0
        
        print("Training Z coordinate optimization model...")
        for epoch in range(epochs):
            # Training
            model.train()
            running_loss = 0.0
            
            for inputs, targets in train_loader:
                # Forward pass
                outputs = model(inputs)
                loss = criterion(outputs, targets)
                
                # Backward pass and optimize
                optimizer.zero_grad()
                loss.backward()
                optimizer.step()
                
                running_loss += loss.item() * inputs.size(0)
            
            epoch_loss = running_loss / len(train_loader.dataset)
            train_losses.append(epoch_loss)
            
            # Validation
            model.eval()
            val_loss = 0.0
            with torch.no_grad():
                for inputs, targets in val_loader:
                    outputs = model(inputs)
                    loss = criterion(outputs, targets)
                    val_loss += loss.item() * inputs.size(0)
            
            val_loss = val_loss / len(val_loader.dataset)
            val_losses.append(val_loss)
            
            # Learning rate adjustment
            scheduler.step(val_loss)
            
            # Save best model
            if val_loss < best_val_loss:
                best_val_loss = val_loss
                best_model_state = model.state_dict().copy()
                patience_counter = 0
            else:
                patience_counter += 1
            
            # Early stopping
            if patience_counter >= patience:
                print(f"Early stopping at epoch {epoch+1}")
                break
            
            # Print progress
            if (epoch + 1) % 20 == 0:
                print(f"Epoch {epoch+1}/{epochs}, Train loss: {epoch_loss:.6f}, Val loss: {val_loss:.6f}")
        
        # Load best model
        model.load_state_dict(best_model_state)
        
        # Calculate RMSE
        model.eval()
        with torch.no_grad():
            z_pred = model(X_val_tensor).cpu().numpy()
            rmse = np.sqrt(mean_squared_error(z_val.ravel(), z_pred.ravel()))
        
        print(f"Z coordinate optimized model RMSE: {rmse:.6f}")
        
        # Plot training curves
        plt.figure(figsize=(10, 5))
        plt.plot(train_losses, label='Training Loss')
        plt.plot(val_losses, label='Validation Loss')
        plt.xlabel('Epoch')
        plt.ylabel('Loss')
        plt.title('Z Coordinate Optimization Model Training Curves')
        plt.legend()
        plt.grid(True)
        plt.savefig('z_optimization_results/z_optimized_model_training.png')
        plt.close()
        
        return model, rmse, train_losses, val_losses
    
    except Exception as e:
        print(f"Error in neural network training: {e}")
        print("Falling back to RandomForest model for Z coordinate")
        
        # Fallback to RandomForest model
        rf_model = RandomForestRegressor(n_estimators=200, random_state=42)
        rf_model.fit(X_train, z_train.ravel())
        
        z_pred_rf = rf_model.predict(X_val)
        rmse = np.sqrt(mean_squared_error(z_val.ravel(), z_pred_rf))
        
        print(f"Fallback RandomForest model RMSE: {rmse:.6f}")
        
        # Create dummy losses for compatibility
        train_losses = [0]
        val_losses = [0]
        
        return rf_model, rmse, train_losses, val_losses

# Main function
def main():
    # Load data
    print("Loading data...")
    data = pd.read_csv(r"C:\Users\13366\Desktop\fsm_2208\packages\1_vive_tracker\scripts\space_transformer_test\corrected_tracker_log_0303.csv")
    
    # Display dataset information
    print(f"Dataset size: {data.shape}")
    print(f"Columns: {data.columns.tolist()}")
    print("First 5 rows:")
    print(data.head())
    
    # Check for missing values
    missing_values = data.isnull().sum()
    if missing_values.sum() > 0:
        print(f"Missing values:\n{missing_values[missing_values > 0]}")
        print("Warning: Dataset contains missing values. Cleaning required.")
    else:
        print("Dataset has no missing values. Data is clean.")
    
    # Input features and target variables
    X = data[["X", "Z"]].values
    y = data[["laser_x", "laser_z"]].values
    
    # Display data ranges
    print("\nData ranges:")
    print(f"X range: [{X[:, 0].min():.4f}, {X[:, 0].max():.4f}]")
    print(f"Z range: [{X[:, 1].min():.4f}, {X[:, 1].max():.4f}]")
    print(f"laser_x range: [{y[:, 0].min():.4f}, {y[:, 0].max():.4f}]")
    print(f"laser_z range: [{y[:, 1].min():.4f}, {y[:, 1].max():.4f}]")
    
    #-----------------------------------------------------------
    # STEP 1: Data Analysis and Visualization
    #-----------------------------------------------------------
    print("\nPerforming data analysis and visualization...")
    
    try:
        # Plot data distributions
        plt.figure(figsize=(12, 10))
        
        plt.subplot(2, 2, 1)
        sns.histplot(X[:, 0], kde=True, color="blue")
        plt.title('X Distribution')
        
        plt.subplot(2, 2, 2)
        sns.histplot(X[:, 1], kde=True, color="blue")
        plt.title('Z Distribution')
        
        plt.subplot(2, 2, 3)
        sns.histplot(y[:, 0], kde=True, color="red")
        plt.title('laser_x Distribution')
        
        plt.subplot(2, 2, 4)
        sns.histplot(y[:, 1], kde=True, color="red")
        plt.title('laser_z Distribution')
        
        plt.tight_layout()
        plt.savefig('z_optimization_results/data_distributions.png')
        plt.close()
        
        # Plot data relationships
        plt.figure(figsize=(12, 10))
        
        plt.subplot(2, 2, 1)
        plt.scatter(X[:, 0], y[:, 0], alpha=0.5, color="blue")
        plt.xlabel('X')
        plt.ylabel('laser_x')
        plt.title('X vs laser_x')
        
        plt.subplot(2, 2, 2)
        plt.scatter(X[:, 1], y[:, 0], alpha=0.5, color="green")
        plt.xlabel('Z')
        plt.ylabel('laser_x')
        plt.title('Z vs laser_x')
        
        plt.subplot(2, 2, 3)
        plt.scatter(X[:, 0], y[:, 1], alpha=0.5, color="orange")
        plt.xlabel('X')
        plt.ylabel('laser_z')
        plt.title('X vs laser_z')
        
        plt.subplot(2, 2, 4)
        plt.scatter(X[:, 1], y[:, 1], alpha=0.5, color="red")
        plt.xlabel('Z')
        plt.ylabel('laser_z')
        plt.title('Z vs laser_z')
        
        plt.tight_layout()
        plt.savefig('z_optimization_results/data_relationships.png')
        plt.close()
        
        # Z coordinate specific analysis
        plt.figure(figsize=(15, 10))
        
        # Z coordinate distribution
        plt.subplot(2, 2, 1)
        sns.histplot(X[:, 1], kde=True, color="blue")
        plt.title('Input Z Distribution')
        
        plt.subplot(2, 2, 2)
        sns.histplot(y[:, 1], kde=True, color="red")
        plt.title('Target laser_z Distribution')
        
        # Z coordinate relationship scatter plot
        plt.subplot(2, 2, 3)
        plt.scatter(X[:, 1], y[:, 1], alpha=0.5, color="purple")
        plt.xlabel('Z')
        plt.ylabel('laser_z')
        plt.title('Z vs laser_z Relationship')
        
        # Check for non-linear relationship
        plt.subplot(2, 2, 4)
        z_range = np.linspace(X[:, 1].min(), X[:, 1].max(), 100)
        plt.scatter(X[:, 1], y[:, 1], alpha=0.3, color="grey")
        
        # Fit polynomials of different degrees
        z_data = X[:, 1]
        laser_z_data = y[:, 1]
        
        plt.plot(z_range, np.poly1d(np.polyfit(z_data, laser_z_data, 1))(z_range), 
                color='red', linewidth=2, label='Linear fit')
        plt.plot(z_range, np.poly1d(np.polyfit(z_data, laser_z_data, 2))(z_range), 
                color='green', linewidth=2, label='Quadratic fit')
        plt.plot(z_range, np.poly1d(np.polyfit(z_data, laser_z_data, 3))(z_range), 
                color='orange', linewidth=2, label='Cubic fit')
        plt.legend()
        plt.title('Z to laser_z Polynomial Fits')
        
        plt.tight_layout()
        plt.savefig('z_optimization_results/z_coordinate_analysis.png')
        plt.close()
        
        # Check correlation
        correlation = np.corrcoef(np.column_stack((X, y)).T)
        plt.figure(figsize=(10, 8))
        sns.heatmap(correlation, annot=True, cmap='coolwarm', 
                    xticklabels=['X', 'Z', 'laser_x', 'laser_z'],
                    yticklabels=['X', 'Z', 'laser_x', 'laser_z'])
        plt.title("Variable Correlation Heatmap")
        plt.savefig('z_optimization_results/correlation_heatmap.png')
        plt.close()
        
        # Check for outliers
        z_zscore = np.abs((X[:, 1] - np.mean(X[:, 1])) / np.std(X[:, 1]))
        laser_z_zscore = np.abs((y[:, 1] - np.mean(y[:, 1])) / np.std(y[:, 1]))
        
        # Output outlier statistics
        print(f"Z coordinate outliers (Z-score>3): {(z_zscore > 3).sum()}")
        print(f"laser_z coordinate outliers (Z-score>3): {(laser_z_zscore > 3).sum()}")
        
        # Show outliers in Z-laser_z relationship
        plt.figure(figsize=(10, 6))
        plt.scatter(X[:, 1], y[:, 1], alpha=0.3, label='Normal data', color="blue")
        outliers = (z_zscore > 3) | (laser_z_zscore > 3)
        if np.any(outliers):
            plt.scatter(X[outliers, 1], y[outliers, 1], 
                        color='red', alpha=0.7, label='Potential outliers')
        plt.xlabel("Z")
        plt.ylabel("laser_z")
        plt.title("Z-laser_z Relationship with Outliers Marked")
        plt.legend()
        plt.savefig('z_optimization_results/z_outliers.png')
        plt.close()
    except Exception as e:
        print(f"Error in data visualization: {e}")
        print("Continuing with analysis...")
    
    #-----------------------------------------------------------
    # STEP 2: Data Preparation and Split
    #-----------------------------------------------------------
    print("\nPreparing data for modeling...")
    
    # Split data into train and test sets
    X_train, X_test, y_train, y_test = train_test_split(
        X, y, test_size=0.2, random_state=42)
    
    print(f"Training set size: {len(X_train)}")
    print(f"Test set size: {len(X_test)}")
    
    # Create validation set for neural network training
    val_size = int(0.2 * len(X_train))
    X_val = X_train[-val_size:]
    X_train_nn = X_train[:-val_size]
    y_val = y_train[-val_size:]
    y_train_nn = y_train[:-val_size]
    
    # Extract Z coordinate data separately
    Z_train = X_train[:, 1].reshape(-1, 1)
    Z_test = X_test[:, 1].reshape(-1, 1)
    laser_z_train = y_train[:, 1].reshape(-1, 1)
    laser_z_test = y_test[:, 1].reshape(-1, 1)
    
    Z_val = X_val[:, 1].reshape(-1, 1)
    Z_train_nn = X_train_nn[:, 1].reshape(-1, 1)
    laser_z_val = y_val[:, 1].reshape(-1, 1)
    laser_z_train_nn = y_train_nn[:, 1].reshape(-1, 1)
    
    # Scale data - different scalers for different purposes
    # Standard scaling
    scaler_X = StandardScaler()
    scaler_y = StandardScaler()
    
    X_train_scaled = scaler_X.fit_transform(X_train)
    X_test_scaled = scaler_X.transform(X_test)
    
    y_train_scaled = scaler_y.fit_transform(y_train)
    y_test_scaled = scaler_y.transform(y_test)
    
    # Robust scaling for Z (more robust to outliers)
    robust_scaler_z = RobustScaler()
    Z_train_robust = robust_scaler_z.fit_transform(Z_train)
    Z_test_robust = robust_scaler_z.transform(Z_test)
    
    # Quantile transform for Z (normalizes distribution)
    quantile_transformer = QuantileTransformer(output_distribution='normal')
    Z_train_quantile = quantile_transformer.fit_transform(Z_train)
    Z_test_quantile = quantile_transformer.transform(Z_test)
    
    #-----------------------------------------------------------
    # STEP 3: Feature Engineering for Z Coordinate
    #-----------------------------------------------------------
    print("\nPerforming feature engineering for Z coordinate...")
    
    # Create extended features for Z
    X_train_extended = create_z_features(X_train)
    X_test_extended = create_z_features(X_test)
    
    # Feature selection - find most relevant features
    selector = SelectKBest(f_regression, k=10)  # Select 10 best features
    X_train_selected = selector.fit_transform(X_train_extended, y_train[:, 1])  # Target is laser_z
    X_test_selected = selector.transform(X_test_extended)
    
    # Print selected feature indices
    selected_indices = selector.get_support(indices=True)
    print("Best features selected for Z coordinate:", selected_indices)
    
    # Feature importance visualization
    try:
        feature_scores = selector.scores_
        plt.figure(figsize=(12, 6))
        plt.bar(range(len(feature_scores)), feature_scores)
        plt.xlabel('Feature Index')
        plt.ylabel('F-Score')
        plt.title('Feature Importance for Z Coordinate Prediction')
        plt.savefig('z_optimization_results/z_feature_importance.png')
        plt.close()
    except Exception as e:
        print(f"Error in feature importance visualization: {e}")
    
    #-----------------------------------------------------------
    # STEP 4: Baseline Model for Z Coordinate
    #-----------------------------------------------------------
    print("\nTraining baseline Random Forest model for Z coordinate...")
    
    # Train Random Forest (baseline)
    start_time = time.time()
    rf_z = RandomForestRegressor(
        n_estimators=200,
        max_depth=15,
        min_samples_split=5,
        min_samples_leaf=2,
        random_state=42,
        n_jobs=-1
    )
    
    rf_z.fit(X_train, laser_z_train.ravel())
    z_pred_rf = rf_z.predict(X_test)
    rmse_rf = np.sqrt(mean_squared_error(laser_z_test, z_pred_rf))
    print(f"Random Forest baseline RMSE for Z: {rmse_rf:.6f}")
    print(f"Training time: {time.time() - start_time:.2f} seconds")
    
    # Train model with extended features
    start_time = time.time()
    rf_z_extended = RandomForestRegressor(
        n_estimators=200,
        max_depth=15,
        min_samples_split=5,
        min_samples_leaf=2,
        random_state=42,
        n_jobs=-1
    )
    
    rf_z_extended.fit(X_train_selected, laser_z_train.ravel())
    z_pred_rf_extended = rf_z_extended.predict(X_test_selected)
    rmse_rf_extended = np.sqrt(mean_squared_error(laser_z_test, z_pred_rf_extended))
    print(f"Random Forest with extended features RMSE for Z: {rmse_rf_extended:.6f}")
    print(f"Training time: {time.time() - start_time:.2f} seconds")
    print(f"Improvement: {((rmse_rf - rmse_rf_extended) / rmse_rf) * 100:.2f}%")
    
    #-----------------------------------------------------------
    # STEP 5: Ensemble Model for Z Coordinate
    #-----------------------------------------------------------
    print("\nTraining ensemble model for Z coordinate...")
    
    start_time = time.time()
    
    # Base models
    rf = RandomForestRegressor(
        n_estimators=200,
        max_depth=15,
        min_samples_split=5,
        min_samples_leaf=2,
        max_features='sqrt',
        bootstrap=True,
        random_state=42,
        n_jobs=-1
    )
    
    gb = GradientBoostingRegressor(
        n_estimators=200,
        learning_rate=0.05,
        max_depth=5,
        min_samples_split=5,
        min_samples_leaf=2,
        subsample=0.8,
        random_state=42
    )
    
    svr = SVR(
        kernel='rbf',
        C=10,
        gamma='scale',
        epsilon=0.1
    )
    
    kr = KernelRidge(
        alpha=1.0,
        kernel='rbf',
        gamma=0.1
    )
    
    # Combine into voting regressor
    z_ensemble = VotingRegressor([
        ('rf', rf),
        ('gb', gb),
        ('svr', svr),
        ('kr', kr)
    ])
    
    # Train ensemble model
    z_ensemble.fit(X_train, laser_z_train.ravel())
    z_pred_ensemble = z_ensemble.predict(X_test)
    rmse_ensemble = np.sqrt(mean_squared_error(laser_z_test, z_pred_ensemble))
    
    print(f"Z coordinate ensemble model RMSE: {rmse_ensemble:.6f}")
    print(f"Training time: {time.time() - start_time:.2f} seconds")
    
    # Train ensemble model with extended features
    start_time = time.time()
    
    z_ensemble_extended = VotingRegressor([
        ('rf', RandomForestRegressor(n_estimators=200, random_state=42)),
        ('gb', GradientBoostingRegressor(n_estimators=200, random_state=42)),
        ('svr', SVR(kernel='rbf', C=10)),
        ('kr', KernelRidge(alpha=1.0, kernel='rbf'))
    ])
    
    z_ensemble_extended.fit(X_train_selected, laser_z_train.ravel())
    z_pred_ensemble_extended = z_ensemble_extended.predict(X_test_selected)
    rmse_ensemble_extended = np.sqrt(mean_squared_error(laser_z_test, z_pred_ensemble_extended))
    
    print(f"Z coordinate ensemble model with extended features RMSE: {rmse_ensemble_extended:.6f}")
    print(f"Training time: {time.time() - start_time:.2f} seconds")
    print(f"Improvement over basic ensemble: {((rmse_ensemble - rmse_ensemble_extended) / rmse_ensemble) * 100:.2f}%")
    
    #-----------------------------------------------------------
    # STEP 6: Piecewise Regression for Z Coordinate
    #-----------------------------------------------------------
    print("\nTraining piecewise regression model for Z coordinate...")
    
    start_time = time.time()
    
    # Using Random Forest as base model
    def rf_factory():
        return RandomForestRegressor(
            n_estimators=100, 
            max_depth=10,
            random_state=42
        )
    
    # Try different number of segments
    best_piecewise_model = None
    best_piecewise_rmse = float('inf')
    best_n_segments = 0
    
    for n_segments in [2, 3, 4, 5]:
        piecewise_model = PiecewiseRegression(n_segments=n_segments, base_model=rf_factory)
        piecewise_model.fit(X_train, laser_z_train.ravel())
        
        # Predict and evaluate
        z_pred_piecewise = piecewise_model.predict(X_test)
        rmse_piecewise = np.sqrt(mean_squared_error(laser_z_test.ravel(), z_pred_piecewise))
        
        print(f"Piecewise model (n={n_segments}) RMSE: {rmse_piecewise:.6f}")
        
        if rmse_piecewise < best_piecewise_rmse:
            best_piecewise_rmse = rmse_piecewise
            best_piecewise_model = piecewise_model
            best_n_segments = n_segments
    
    print(f"Best piecewise model: {best_n_segments} segments, RMSE: {best_piecewise_rmse:.6f}")
    print(f"Training time: {time.time() - start_time:.2f} seconds")
    
    # Visualize piecewise model
    try:
        plt.figure(figsize=(12, 6))
        
        # Original data points
        plt.scatter(Z_train, laser_z_train, alpha=0.3, label='Training data', color="grey")
        
        # Visualize each segment boundary and prediction
        colors = ['red', 'green', 'blue', 'purple', 'orange']
        for i, (low, high) in enumerate(best_piecewise_model.segment_bounds):
            # Generate evenly spaced Z values in this segment
            z_range = np.linspace(low, high, 100).reshape(-1, 1)
            
            # Create complete input vectors (copy X values)
            X_dummy = np.zeros((len(z_range), X_train.shape[1]))
            X_dummy[:, 1] = z_range.ravel()
            X_dummy[:, 0] = np.mean(X_train[:, 0])  # Use mean X
            
            # Predict values in this segment
            predictions = best_piecewise_model.models[i].predict(X_dummy)
            
            # Plot prediction curve for this segment
            plt.plot(z_range, predictions, '-', color=colors[i % len(colors)], 
                    linewidth=2, label=f'Segment {i+1} prediction')
            
            # Mark segment boundaries
            plt.axvline(x=low, color=colors[i % len(colors)], linestyle='--', alpha=0.5)
            plt.axvline(x=high, color=colors[i % len(colors)], linestyle='--', alpha=0.5)
        
        plt.xlabel('Z coordinate')
        plt.ylabel('laser_z coordinate')
        plt.title(f'Z Coordinate Piecewise Regression (n={best_n_segments}, RMSE={best_piecewise_rmse:.6f})')
        plt.legend()
        plt.grid(True)
        plt.savefig('z_optimization_results/z_piecewise_regression.png')
        plt.close()
    except Exception as e:
        print(f"Error in piecewise model visualization: {e}")
    
    #-----------------------------------------------------------
    # STEP 7: Neural Network for Z Coordinate
    #-----------------------------------------------------------
    print("\nTraining neural network model for Z coordinate...")
    
    # Train Z-optimized neural network
    z_nn_model, z_nn_rmse, train_losses, val_losses = train_z_optimized_model(
        X_train_nn, laser_z_train_nn, X_val, laser_z_val, epochs=200)
    
    # Predict on test set
    try:
        if isinstance(z_nn_model, ZCoordinateTransformer):
            X_test_tensor = torch.tensor(X_test, dtype=torch.float32).to(device)
            with torch.no_grad():
                z_pred_nn = z_nn_model(X_test_tensor).cpu().numpy()
        else:
            # It's a fallback RandomForest model
            z_pred_nn = z_nn_model.predict(X_test).reshape(-1, 1)
        
        rmse_nn_test = np.sqrt(mean_squared_error(laser_z_test, z_pred_nn))
        print(f"Neural network model test RMSE for Z: {rmse_nn_test:.6f}")
    except Exception as e:
        print(f"Error in neural network prediction: {e}")
        # Use a fallback prediction
        z_pred_nn = z_pred_rf_extended.reshape(-1, 1)
        rmse_nn_test = rmse_rf_extended
        print(f"Using fallback prediction with RMSE: {rmse_nn_test:.6f}")
    
    #-----------------------------------------------------------
    # STEP 8: Combined Z Prediction Model
    #-----------------------------------------------------------
    print("\nCombining multiple Z coordinate prediction models...")
    
    # Get predictions from all models
    z_pred_models = {
        'Random Forest': {
            'pred': z_pred_rf.reshape(-1, 1),
            'rmse': rmse_rf
        },
        'Extended Features RF': {
            'pred': z_pred_rf_extended.reshape(-1, 1),
            'rmse': rmse_rf_extended
        },
        'Ensemble': {
            'pred': z_pred_ensemble.reshape(-1, 1),
            'rmse': rmse_ensemble
        },
        'Extended Ensemble': {
            'pred': z_pred_ensemble_extended.reshape(-1, 1),
            'rmse': rmse_ensemble_extended
        },
        'Piecewise': {
            'pred': best_piecewise_model.predict(X_test).reshape(-1, 1),
            'rmse': best_piecewise_rmse
        },
        'Neural Network': {
            'pred': z_pred_nn,
            'rmse': rmse_nn_test
        }
    }
    
    # Determine weights based on RMSE (lower RMSE = higher weight)
    rmse_values = np.array([model_info['rmse'] for _, model_info in z_pred_models.items()])
    weights = 1.0 / (rmse_values + 1e-10)  # Avoid division by zero
    weights = weights / np.sum(weights)  # Normalize
    
    print("Model weights:")
    for i, (model_name, _) in enumerate(z_pred_models.items()):
        print(f"{model_name}: {weights[i]:.4f}")
    
    # Combine predictions using weighted average
    z_pred_combined = np.zeros_like(laser_z_test)
    for i, (_, model_info) in enumerate(z_pred_models.items()):
        z_pred_combined += weights[i] * model_info['pred']
    
    rmse_combined = np.sqrt(mean_squared_error(laser_z_test, z_pred_combined))
    print(f"Combined model RMSE for Z: {rmse_combined:.6f}")
    
    # Determine the best performing model for Z
    best_model_name = min(z_pred_models.items(), key=lambda x: x[1]['rmse'])[0]
    best_model_rmse = min(rmse_values)
    print(f"Best individual model: {best_model_name}, RMSE: {best_model_rmse:.6f}")
    
    if rmse_combined < best_model_rmse:
        print(f"Combined model is better by {((best_model_rmse - rmse_combined) / best_model_rmse) * 100:.2f}%")
        z_final_pred = z_pred_combined
        z_final_rmse = rmse_combined
        z_best_model = "Combined"
    else:
        print(f"Best individual model ({best_model_name}) is better")
        z_final_pred = z_pred_models[best_model_name]['pred']
        z_final_rmse = best_model_rmse
        z_best_model = best_model_name
    
    # Visualize model comparison
    try:
        plt.figure(figsize=(12, 8))
        
        # Actual vs predicted scatter plot
        plt.subplot(2, 2, 1)
        for model_name, model_info in z_pred_models.items():
            if model_name == best_model_name:
                plt.scatter(laser_z_test, model_info['pred'], alpha=0.5, label=f'{model_name}')
        plt.scatter(laser_z_test, z_pred_combined, alpha=0.5, label='Combined')
        plt.plot([laser_z_test.min(), laser_z_test.max()], 
                [laser_z_test.min(), laser_z_test.max()], 'k--')
        plt.xlabel('Actual laser_z')
        plt.ylabel('Predicted laser_z')
        plt.title('Actual vs Predicted laser_z')
        plt.legend()
        plt.grid(True)
        
        # Prediction error distribution
        plt.subplot(2, 2, 2)
        plt.hist(laser_z_test.ravel() - z_pred_models[best_model_name]['pred'].ravel(), alpha=0.5, bins=30, 
                label=f'{best_model_name} error', color="blue")
        plt.hist(laser_z_test.ravel() - z_pred_combined.ravel(), alpha=0.5, bins=30, 
                label='Combined error', color="red")
        plt.xlabel('Prediction Error')
        plt.ylabel('Frequency')
        plt.title('Prediction Error Distribution')
        plt.legend()
        plt.grid(True)
        
        # Model RMSE comparison
        plt.subplot(2, 2, 3)
        model_names = list(z_pred_models.keys()) + ['Combined']
        all_rmse_values = list(rmse_values) + [rmse_combined]
        plt.bar(model_names, all_rmse_values)
        plt.xlabel('Model')
        plt.ylabel('RMSE')
        plt.title('Model RMSE Comparison')
        plt.xticks(rotation=45, ha='right')
        plt.grid(True, axis='y')
        
        # Z error vs Z input relationship
        plt.subplot(2, 2, 4)
        plt.scatter(X_test[:, 1], laser_z_test.ravel() - z_final_pred.ravel(), alpha=0.5)
        plt.axhline(y=0, color='r', linestyle='-')
        plt.xlabel('Input Z value')
        plt.ylabel('Prediction Error')
        plt.title('Prediction Error vs Input Z')
        plt.grid(True)
        
        plt.tight_layout()
        plt.savefig('z_optimization_results/z_models_comparison.png')
        plt.close()
    except Exception as e:
        print(f"Error in model comparison visualization: {e}")
    
    #-----------------------------------------------------------
    # STEP 9: Complete Coordinate Transformation
    #-----------------------------------------------------------
    print("\nBuilding complete coordinate transformation model...")
    
    # Train X coordinate model (using Random Forest)
    x_model = RandomForestRegressor(
        n_estimators=200,
        max_depth=15,
        min_samples_split=5,
        min_samples_leaf=2,
        random_state=42,
        n_jobs=-1
    )
    
    x_model.fit(X_train, y_train[:, 0])
    x_pred = x_model.predict(X_test)
    rmse_x = np.sqrt(mean_squared_error(y_test[:, 0], x_pred))
    print(f"Random Forest model RMSE for X: {rmse_x:.6f}")
    
    # Determine the best Z model for the complete transformer
    if z_best_model == "Neural Network" and isinstance(z_nn_model, ZCoordinateTransformer):
        best_z_model = z_nn_model
    elif z_best_model == "Piecewise":
        best_z_model = best_piecewise_model
    elif z_best_model == "Extended Ensemble":
        best_z_model = z_ensemble_extended
    elif z_best_model == "Extended Features RF":
        best_z_model = rf_z_extended
    else:
        # Default to ensemble or rf
        best_z_model = z_ensemble if z_best_model == "Ensemble" else rf_z
    
    # Create complete transformer
    if z_best_model == "Combined":
        # For combined model, we need to create a special wrapper
        class CombinedZModel:
            def __init__(self, models, weights):
                self.models = models
                self.weights = weights
                
            def predict(self, X):
                preds = []
                for i, (model_name, model_info) in enumerate(self.models.items()):
                    if model_name == "Neural Network" and isinstance(model_info['model'], ZCoordinateTransformer):
                        try:
                            X_tensor = torch.tensor(X, dtype=torch.float32).to(device)
                            with torch.no_grad():
                                pred = model_info['model'](X_tensor).cpu().numpy().ravel()
                        except:
                            # Fallback to RandomForest if neural network fails
                            pred = rf_z.predict(X)
                    elif model_name == "Extended Features RF" or model_name == "Extended Ensemble":
                        # Need to transform X to extended features
                        X_ext = create_z_features(X)
                        X_ext_selected = selector.transform(X_ext)
                        pred = model_info['model'].predict(X_ext_selected)
                    else:
                        pred = model_info['model'].predict(X)
                    
                    preds.append(pred.reshape(-1, 1))
                
                # Combine predictions
                z_pred_combined = np.zeros((X.shape[0], 1))
                for i in range(len(preds)):
                    z_pred_combined += self.weights[i] * preds[i]
                
                return z_pred_combined.ravel()
        
        # Prepare models dictionary with actual model objects
        combined_models = {}
        for model_name, model_info in z_pred_models.items():
            if model_name == "Neural Network":
                combined_models[model_name] = {
                    'model': z_nn_model,
                    'rmse': model_info['rmse']
                }
            elif model_name == "Extended Features RF":
                combined_models[model_name] = {
                    'model': rf_z_extended,
                    'rmse': model_info['rmse']
                }
            elif model_name == "Extended Ensemble":
                combined_models[model_name] = {
                    'model': z_ensemble_extended,
                    'rmse': model_info['rmse']
                }
            elif model_name == "Piecewise":
                combined_models[model_name] = {
                    'model': best_piecewise_model,
                    'rmse': model_info['rmse']
                }
            elif model_name == "Ensemble":
                combined_models[model_name] = {
                    'model': z_ensemble,
                    'rmse': model_info['rmse']
                }
            else:  # Random Forest
                combined_models[model_name] = {
                    'model': rf_z,
                    'rmse': model_info['rmse']
                }
        
        # Create combined model
        combined_z_model = CombinedZModel(combined_models, weights)
        best_z_model = combined_z_model
    
    # Create the complete transformer
    transformer = CoordinateTransformer(
        x_model=x_model,
        z_model=best_z_model
    )
    
    # Predict and evaluate
    predictions = transformer.predict(X_test)
    
    rmse_x_final = np.sqrt(mean_squared_error(y_test[:, 0], predictions[:, 0]))
    rmse_z_final = np.sqrt(mean_squared_error(y_test[:, 1], predictions[:, 1]))
    rmse_overall = np.sqrt(mean_squared_error(y_test, predictions))
    
    print("\nFinal coordinate transformation results:")
    print(f"X coordinate RMSE: {rmse_x_final:.6f}")
    print(f"Z coordinate RMSE: {rmse_z_final:.6f}")
    print(f"Overall RMSE: {rmse_overall:.6f}")
    print(f"Z coordinate improvement: {((0.08 - rmse_z_final) / 0.08) * 100:.2f}%")
    
    # Visualize final results
    try:
        plt.figure(figsize=(15, 10))
        
        # X coordinate transformation
        plt.subplot(2, 2, 1)
        plt.scatter(y_test[:, 0], predictions[:, 0], alpha=0.5, color="blue")
        min_x = min(y_test[:, 0].min(), predictions[:, 0].min())
        max_x = max(y_test[:, 0].max(), predictions[:, 0].max())
        plt.plot([min_x, max_x], [min_x, max_x], 'k--')
        plt.xlabel('Actual laser_x')
        plt.ylabel('Predicted laser_x')
        plt.title(f'X Coordinate Transformation (RMSE={rmse_x_final:.6f})')
        plt.grid(True)
        
        # Z coordinate transformation
        plt.subplot(2, 2, 2)
        plt.scatter(y_test[:, 1], predictions[:, 1], alpha=0.5, color="red")
        min_z = min(y_test[:, 1].min(), predictions[:, 1].min())
        max_z = max(y_test[:, 1].max(), predictions[:, 1].max())
        plt.plot([min_z, max_z], [min_z, max_z], 'k--')
        plt.xlabel('Actual laser_z')
        plt.ylabel('Predicted laser_z')
        plt.title(f'Z Coordinate Transformation (RMSE={rmse_z_final:.6f})')
        plt.grid(True)
        
        # Coordinate space transformation
        plt.subplot(2, 2, 3)
        plt.scatter(X_test[:, 0], X_test[:, 1], alpha=0.3, label='Input (X,Z)', color="blue")
        plt.scatter(y_test[:, 0], y_test[:, 1], alpha=0.3, label='Target (laser_x,laser_z)', color="red")
        plt.scatter(predictions[:, 0], predictions[:, 1], alpha=0.3, label='Predicted', color="green")
        plt.xlabel('X / laser_x')
        plt.ylabel('Z / laser_z')
        plt.title('Coordinate Space Transformation')
        plt.legend()
        plt.grid(True)
        
        # Error distribution
        plt.subplot(2, 2, 4)
        x_errors = y_test[:, 0] - predictions[:, 0]
        z_errors = y_test[:, 1] - predictions[:, 1]
        plt.hist(x_errors, alpha=0.5, bins=30, label='X error', color="blue")
        plt.hist(z_errors, alpha=0.5, bins=30, label='Z error', color="red")
        plt.xlabel('Error')
        plt.ylabel('Frequency')
        plt.title('Error Distribution')
        plt.legend()
        plt.grid(True)
        
        plt.tight_layout()
        plt.savefig('z_optimization_results/complete_transformation_results.png')
        plt.close()
    except Exception as e:
        print(f"Error in final results visualization: {e}")
    
    #-----------------------------------------------------------
    # STEP 10: Summary Results
    #-----------------------------------------------------------
    
    # Create summary table
    summary_data = {
        'Model': ['Original Random Forest', 'Z-Optimized Model'],
        'X_RMSE': [rmse_x, rmse_x_final],
        'Z_RMSE': [rmse_rf, rmse_z_final],
        'Overall_RMSE': [np.sqrt(mean_squared_error(y_test, np.column_stack((x_pred, z_pred_rf)))), rmse_overall],
        'Z_Improvement': [0, ((rmse_rf - rmse_z_final) / rmse_rf) * 100]
    }
    
    summary_df = pd.DataFrame(summary_data)
    print("\nSummary Results:")
    print(summary_df.to_string(index=False))
    
    # Save summary to CSV
    summary_df.to_csv('z_optimization_results/optimization_summary.csv', index=False)
    
    # Create final comparison visualization
    try:
        plt.figure(figsize=(12, 8))
        
        # RMSE comparison
        plt.subplot(2, 2, 1)
        models = summary_df['Model']
        x_rmse = summary_df['X_RMSE']
        z_rmse = summary_df['Z_RMSE']
        overall_rmse = summary_df['Overall_RMSE']
        
        x = np.arange(len(models))
        width = 0.25
        
        plt.bar(x - width, x_rmse, width, label='X RMSE', color="blue")
        plt.bar(x, z_rmse, width, label='Z RMSE', color="red")
        plt.bar(x + width, overall_rmse, width, label='Overall RMSE', color="green")
        
        plt.xlabel('Model')
        plt.ylabel('RMSE')
        plt.title('RMSE Comparison')
        plt.xticks(x, models)
        plt.legend()
        plt.grid(True, axis='y')
        
        # Z improvement
        plt.subplot(2, 2, 2)
        plt.bar(models, summary_df['Z_Improvement'], color="purple")
        plt.xlabel('Model')
        plt.ylabel('Improvement (%)')
        plt.title('Z Coordinate Improvement')
        plt.grid(True, axis='y')
        
        # Before-After Z prediction
        plt.subplot(2, 2, 3)
        plt.scatter(y_test[:, 1], z_pred_rf, alpha=0.5, label='Before Optimization', color="blue")
        plt.scatter(y_test[:, 1], predictions[:, 1], alpha=0.5, label='After Optimization', color="red")
        plt.plot([min_z, max_z], [min_z, max_z], 'k--')
        plt.xlabel('Actual Z')
        plt.ylabel('Predicted Z')
        plt.title('Z Coordinate Prediction Improvement')
        plt.legend()
        plt.grid(True)
        
        # Error reduction
        plt.subplot(2, 2, 4)
        before_errors = np.abs(y_test[:, 1] - z_pred_rf)
        after_errors = np.abs(y_test[:, 1] - predictions[:, 1])
        
        plt.hist(before_errors, alpha=0.5, bins=30, label='Before Opt', color="blue")
        plt.hist(after_errors, alpha=0.5, bins=30, label='After Opt', color="red")
        plt.xlabel('Absolute Error')
        plt.ylabel('Frequency')
        plt.title('Error Distribution Before and After Optimization')
        plt.legend()
        plt.grid(True)
        
        plt.tight_layout()
        plt.savefig('z_optimization_results/final_comparison.png')
        plt.close()
    except Exception as e:
        print(f"Error in final comparison visualization: {e}")
    
    print("\nAll results saved to 'z_optimization_results' directory.")
    print("Z coordinate optimization complete.")

# Run the main function
if __name__ == "__main__":
    try:
        main()
    except Exception as e:
        print(f"Error in main function: {e}")