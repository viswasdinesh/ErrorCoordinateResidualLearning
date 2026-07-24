# !/usr/bin/env python3
import os
import sys
import glob
import json
import argparse
import numpy as np
import pandas as pd
import torch
import torch.nn as nn
import torch.optim as optim
from torch.utils.data import Dataset, DataLoader
from torch.nn.utils import spectral_norm, remove_spectral_norm
import matplotlib.pyplot as plt
from tqdm import tqdm

class ResidualDataset(Dataset):
    def __init__(self, inputs, targets):
        self.inputs = torch.tensor(inputs, dtype=torch.float32)
        self.targets = torch.tensor(targets, dtype=torch.float32)

    def __len__(self):
        return len(self.inputs)

    def __getitem__(self, idx):
        return self.inputs[idx], self.targets[idx]

class ResidualMLP(nn.Module):
    def __init__(self, input_dim=16, output_dim=9, hidden_dims=[128, 128, 128], activation='tanh'):
        super().__init__()
        layers = []
        prev_dim = input_dim
        act_fn = {'relu': nn.ReLU, 'tanh': nn.Tanh, 'elu': nn.ELU}[activation]
        for hidden_dim in hidden_dims:
            layers.append(spectral_norm(nn.Linear(prev_dim, hidden_dim)))
            layers.append(act_fn())
            prev_dim = hidden_dim
        layers.append(spectral_norm(nn.Linear(prev_dim, output_dim)))
        self.network = nn.Sequential(*layers)

    def forward(self, x):
        return self.network(x)

class NormalizedResidualMLP(nn.Module):
    def __init__(self, base_model, x_mean, x_std, y_mean, y_std):
        super().__init__()
        self.net = base_model
        self.register_buffer('x_mean', torch.tensor(x_mean, dtype=torch.float32))
        self.register_buffer('x_std',  torch.tensor(x_std,  dtype=torch.float32))
        self.register_buffer('y_mean', torch.tensor(y_mean, dtype=torch.float32))
        self.register_buffer('y_std',  torch.tensor(y_std,  dtype=torch.float32))

    def forward(self, x):
        x_norm = (x - self.x_mean) / self.x_std
        y_norm = self.net(x_norm)
        return y_norm * self.y_std + self.y_mean

def load_data(dataset_dir, num_files=None):
    """
    Loads ECRL datasets from CSV files.
    Inputs: dx_0..dx_8 (9) + du_0..du_3 (4) + r_ref_0..r_ref_2 (3)
    Targets: res_0..res_8 (9)
    """
    csv_pattern = os.path.join(dataset_dir, "polynomial*.csv")
    all_files = sorted(glob.glob(csv_pattern))
    
    if len(all_files) == 0:
        # Fallback to look for other CSVs if polynomial files aren't found
        csv_pattern = os.path.join(dataset_dir, "*.csv")
        all_files = sorted(glob.glob(csv_pattern))
        # Exclude giant gazebo dataset or logs if needed, or include them
        all_files = [f for f in all_files if "gazebo" not in os.path.basename(f)]
        
    if len(all_files) == 0:
        raise FileNotFoundError(f"No CSV files found in {dataset_dir}")
        
    if num_files is not None:
        all_files = all_files[:num_files]
        
    print(f"Loading {len(all_files)} files...")
    
    input_list = []
    target_list = []
    
    # Feature columns
    dx_cols = [f"dx_{i}" for i in range(9)]
    du_cols = [f"du_{i}" for i in range(4)]
    res_cols = [f"res_{i}" for i in range(9)]
    r_ref_cols = [f"r_ref_{i}" for i in range(3)]
    
    for f in tqdm(all_files, desc="Reading CSV files"):
        df = pd.read_csv(f)
        
        # Verify columns exist
        missing = [c for c in dx_cols + du_cols + res_cols + r_ref_cols if c not in df.columns]
        if missing:
            raise ValueError(f"Missing columns in {f}: {missing}. Regenerate dataset with r_ref logging.")
            
        # Drop repeated header rows (where 'timestamp' is a string or starts with 'timestamp')
        df = df[pd.to_numeric(df['timestamp'], errors='coerce').notna()]
        
        # Convert all columns to numeric type
        df = df.apply(pd.to_numeric, errors='coerce')
        df = df.dropna()
        
        dx = df[dx_cols].values
        du = df[du_cols].values
        res = df[res_cols].values
        r_ref = df[r_ref_cols].values
        
        inputs = np.hstack([dx, du, r_ref])  # always 16D
            
        input_list.append(inputs)
        target_list.append(res)
        
    if not input_list:
        raise ValueError("No valid data loaded!")
        
    X = np.vstack(input_list)
    y = np.vstack(target_list)
    
    print(f"Loaded dataset: inputs shape {X.shape}, targets shape {y.shape}")
    return X, y

def main():
    parser = argparse.ArgumentParser(description="Train Residual Learning Model for On-Manifold MPC")
    parser.add_argument("--dataset_dir", type=str, default="/home/dinesh/ecrl_datasets", help="Directory with CSV datasets")
    parser.add_argument("--output_dir", type=str, default="/home/dinesh/ecrl_ws/src/ommpc_ros2/models", help="Directory to save output models")
    parser.add_argument("--epochs", type=int, default=50, help="Number of epochs to train")
    parser.add_argument("--batch_size", type=int, default=256, help="Batch size for training")
    parser.add_argument("--lr", type=float, default=1e-3, help="Learning rate")
    parser.add_argument("--num_files", type=int, default=None, help="Number of files to load (default: all)")
    parser.add_argument("--hidden_dims", type=int, nargs="+", default=[128, 128, 128], help="Hidden layers dimensions")
    parser.add_argument("--activation", type=str, default="tanh", choices=["relu", "tanh", "elu"], help="Activation function")
    
    args = parser.parse_args()
    
    os.makedirs(args.output_dir, exist_ok=True)
    
    # 1. Load Data
    try:
        X, y = load_data(args.dataset_dir, num_files=args.num_files)
    except Exception as e:
        print(f"Error loading data: {e}")
        sys.exit(1)
        
    # 2. Normalize Data
    X_mean = np.mean(X, axis=0)
    X_std = np.std(X, axis=0)
    # Avoid division by zero
    X_std[X_std < 1e-8] = 1.0
    
    y_mean = np.mean(y, axis=0)
    y_std = np.std(y, axis=0)
    y_std[y_std < 1e-8] = 1.0
    
    X_norm = (X - X_mean) / X_std
    y_norm = (y - y_mean) / y_std
    
    # 3. Train/Val/Test Split (80% / 10% / 10%)
    num_samples = len(X_norm)
    indices = np.arange(num_samples)
    np.random.seed(42)
    np.random.shuffle(indices)
    
    train_end = int(0.8 * num_samples)
    val_end = int(0.9 * num_samples)
    
    train_idx = indices[:train_end]
    val_idx = indices[train_end:val_end]
    test_idx = indices[val_end:]
    
    train_dataset = ResidualDataset(X_norm[train_idx], y_norm[train_idx])
    val_dataset = ResidualDataset(X_norm[val_idx], y_norm[val_idx])
    test_dataset = ResidualDataset(X_norm[test_idx], y_norm[test_idx])
    
    train_loader = DataLoader(train_dataset, batch_size=args.batch_size, shuffle=True)
    val_loader = DataLoader(val_dataset, batch_size=args.batch_size, shuffle=False)
    test_loader = DataLoader(test_dataset, batch_size=args.batch_size, shuffle=False)
    
    # 4. Initialize Model
    input_dim = X.shape[1]
    output_dim = y.shape[1]
    
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    print(f"Using device: {device}")
    
    model = ResidualMLP(input_dim=input_dim, output_dim=output_dim, hidden_dims=args.hidden_dims, activation=args.activation).to(device)
    optimizer = optim.Adam(model.parameters(), lr=args.lr)
    criterion = nn.MSELoss()
    scheduler = optim.lr_scheduler.ReduceLROnPlateau(optimizer, mode="min", factor=0.5, patience=5)
    
    # 5. Training Loop
    best_val_loss = float("inf")
    train_losses = []
    val_losses = []
    
    for epoch in range(1, args.epochs + 1):
        model.train()
        epoch_train_loss = 0.0
        for inputs, targets in train_loader:
            inputs, targets = inputs.to(device), targets.to(device)
            
            optimizer.zero_grad()
            outputs = model(inputs)
            loss = criterion(outputs, targets)
            loss.backward()
            optimizer.step()
            
            epoch_train_loss += loss.item() * len(inputs)
            
        epoch_train_loss /= len(train_idx)
        train_losses.append(epoch_train_loss)
        
        # Validation
        model.eval()
        epoch_val_loss = 0.0
        with torch.no_grad():
            for inputs, targets in val_loader:
                inputs, targets = inputs.to(device), targets.to(device)
                outputs = model(inputs)
                loss = criterion(outputs, targets)
                epoch_val_loss += loss.item() * len(inputs)
                
        epoch_val_loss /= len(val_idx)
        val_losses.append(epoch_val_loss)
        
        scheduler.step(epoch_val_loss)
        
        print(f"Epoch {epoch:2d}/{args.epochs:2d} | Train Loss: {epoch_train_loss:.6f} | Val Loss: {epoch_val_loss:.6f}")
        
        # Save best model
        if epoch_val_loss < best_val_loss:
            best_val_loss = epoch_val_loss
            torch.save(model.state_dict(), os.path.join(args.output_dir, "best_model.pt"))
            
    print(f"Training finished. Best Val Loss: {best_val_loss:.6f}")
    
    # 6. Load Best Model and Evaluate on Test Set
    model.load_state_dict(torch.load(os.path.join(args.output_dir, "best_model.pt")))
    model.eval()
    
    test_loss = 0.0
    all_predictions = []
    all_targets = []
    
    with torch.no_grad():
        for inputs, targets in test_loader:
            inputs, targets = inputs.to(device), targets.to(device)
            outputs = model(inputs)
            loss = criterion(outputs, targets)
            test_loss += loss.item() * len(inputs)
            
            all_predictions.append(outputs.cpu().numpy())
            all_targets.append(targets.cpu().numpy())
            
    test_loss /= len(test_idx)
    print(f"Test MSE Loss (normalized): {test_loss:.6f}")
    
    predictions = np.vstack(all_predictions)
    targets = np.vstack(all_targets)
    
    # De-normalize predictions and targets for physical coordinate evaluation
    predictions_denorm = predictions * y_std + y_mean
    targets_denorm = targets * y_std + y_mean
    
    # Calculate coordinate-wise RMSE
    coordinate_rmse = np.sqrt(np.mean((predictions_denorm - targets_denorm)**2, axis=0))
    coordinate_names = [
        "Position X (res_0)", "Position Y (res_1)", "Position Z (res_2)",
        "Velocity X (res_3)", "Velocity Y (res_4)", "Velocity Z (res_5)",
        "Attitude Roll (res_6)", "Attitude Pitch (res_7)", "Attitude Yaw (res_8)"
    ]
    
    print("\n--- Test Set RMSE in Physical Coordinates ---")
    for name, rmse in zip(coordinate_names, coordinate_rmse):
        print(f"{name:25s}: {rmse:.6f}")
        
    # 7. Save Normalization Metadata
    metadata = {
        "input_dim": input_dim,
        "output_dim": output_dim,
        "input_mean": X_mean.tolist(),
        "input_std": X_std.tolist(),
        "output_mean": y_mean.tolist(),
        "output_std": y_std.tolist(),
        "coordinate_names": coordinate_names,
        "test_rmse": coordinate_rmse.tolist()
    }
    
    with open(os.path.join(args.output_dir, "ecrl_metadata.json"), "w") as f:
        json.dump(metadata, f, indent=4)
    print(f"Saved normalization metadata to {os.path.join(args.output_dir, 'ecrl_metadata.json')}")
    
    # Move model to CPU and remove spectral norm hooks before scripting/ONNX export
    model.cpu()
    # Perform a dummy forward pass on CPU to ensure spectral norm coefficients are fully initialized/updated
    _ = model(torch.zeros(1, input_dim))
    
    for m in model.modules():
        if isinstance(m, nn.Linear):
            try:
                remove_spectral_norm(m)
            except ValueError:
                pass
                
    # Wrap in normalization model
    normalized_model = NormalizedResidualMLP(model, X_mean, X_std, y_mean, y_std)

    # 8. Export to ONNX
    try:
        dummy_input = torch.randn(1, input_dim)
        onnx_path = os.path.join(args.output_dir, "ecrl_model.onnx")
        torch.onnx.export(
            model, 
            dummy_input, 
            onnx_path, 
            export_params=True, 
            opset_version=11, 
            do_constant_folding=True, 
            input_names=["input"], 
            output_names=["output"]
        )
        print(f"Exported unnormalized model to ONNX format: {onnx_path}")
        
        onnx_norm_path = os.path.join(args.output_dir, "ecrl_model_normalized.onnx")
        torch.onnx.export(
            normalized_model,
            dummy_input,
            onnx_norm_path,
            export_params=True,
            opset_version=11,
            do_constant_folding=True,
            input_names=["input"],
            output_names=["output"]
        )
        print(f"Exported normalized model to ONNX format: {onnx_norm_path}")
    except Exception as e:
        print(f"Failed to export to ONNX: {e}")
    
    # 9. Export to TorchScript
    try:
        # Export unnormalized
        scripted_model = torch.jit.script(model)
        torchscript_path = os.path.join(args.output_dir, "ecrl_model.pt")
        scripted_model.save(torchscript_path)
        print(f"Exported unnormalized model to TorchScript format: {torchscript_path}")
        
        # Export normalized
        scripted_norm_model = torch.jit.script(normalized_model)
        torchscript_norm_path = os.path.join(args.output_dir, "ecrl_model_normalized.pt")
        scripted_norm_model.save(torchscript_norm_path)
        print(f"Exported normalized model to TorchScript format: {torchscript_norm_path}")
    except Exception as e:
        print(f"Failed to export to TorchScript: {e}")
        
    # 10. Plot Training Curves
    plt.figure(figsize=(10, 5))
    plt.plot(train_losses, label="Train Loss")
    plt.plot(val_losses, label="Val Loss")
    plt.xlabel("Epoch")
    plt.ylabel("MSE Loss (Normalized)")
    plt.title("Residual Learning Model Training Curves")
    plt.legend()
    plt.grid(True)
    plot_path = os.path.join(args.output_dir, "training_curves.png")
    plt.savefig(plot_path)
    plt.close()
    print(f"Saved training curve plot to {plot_path}")

if __name__ == "__main__":
    main()
