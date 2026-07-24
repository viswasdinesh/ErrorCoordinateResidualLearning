import os
import sys
import glob
import json
import numpy as np
import pandas as pd
import torch
import torch.nn as nn
import torch.optim as optim
from torch.utils.data import TensorDataset, DataLoader
from torch.nn.utils import spectral_norm
import matplotlib.pyplot as plt
from tqdm import tqdm

# Rodrigues formula for SO(3) exponential map
def SO3_exp(w):
    theta = np.linalg.norm(w, axis=1, keepdims=True)
    mask = theta[:, 0] < 1e-8
    axis = np.zeros_like(w)
    axis[~mask] = w[~mask] / theta[~mask]
    sin_t = np.sin(theta)
    cos_t = np.cos(theta)
    N = len(w)
    I = np.repeat(np.eye(3)[np.newaxis, :, :], N, axis=0)
    axis_outer = np.einsum('ni,nj->nij', axis, axis)
    K = np.zeros((N, 3, 3))
    K[:, 0, 1] = -axis[:, 2]
    K[:, 0, 2] = axis[:, 1]
    K[:, 1, 0] = axis[:, 2]
    K[:, 1, 2] = -axis[:, 0]
    K[:, 2, 0] = -axis[:, 1]
    K[:, 2, 1] = axis[:, 0]
    R = I * cos_t[:, :, np.newaxis] + (1.0 - cos_t[:, :, np.newaxis]) * axis_outer + K * sin_t[:, :, np.newaxis]
    R[mask] = np.eye(3)
    return R

def reconstruct_R(r_ref, dtheta):
    R_ref = SO3_exp(r_ref)
    R_err = SO3_exp(dtheta)
    R = np.einsum('nij,njk->nik', R_ref, R_err)
    return R.reshape(-1, 9)

# Load data and prepare both ECRL and Ambient inputs
def prepare_data(dataset_dir):
    csv_pattern = os.path.join(dataset_dir, "polynomial*.csv")
    all_files = sorted(glob.glob(csv_pattern))
    
    if len(all_files) == 0:
        raise FileNotFoundError(f"No CSV files found in {dataset_dir}")
        
    print(f"Loading {len(all_files)} files...")
    
    dx_cols = [f"dx_{i}" for i in range(9)]
    du_cols = [f"du_{i}" for i in range(4)]
    res_cols = [f"res_{i}" for i in range(9)]
    r_ref_cols = [f"r_ref_{i}" for i in range(3)]
    
    dx_list, du_list, res_list, r_ref_list = [], [], [], []
    
    # Load all representative files for diverse training distribution
    files_to_load = all_files
    
    for f in tqdm(files_to_load, desc="Reading CSV files"):
        df = pd.read_csv(f)
        df = df[pd.to_numeric(df['timestamp'], errors='coerce').notna()]
        df = df.apply(pd.to_numeric, errors='coerce').dropna()
        if len(df) == 0:
            continue
            
        dx_list.append(df[dx_cols].values)
        du_list.append(df[du_cols].values)
        res_list.append(df[res_cols].values)
        r_ref_list.append(df[r_ref_cols].values)
        
    dx = np.vstack(dx_list)
    du = np.vstack(du_list)
    res = np.vstack(res_list)
    r_ref = np.vstack(r_ref_list)
    
    # 1. ECRL inputs (16D)
    X_ecrl = np.hstack([dx, du, r_ref])
    
    # 2. Ambient inputs (19D)
    print("Reconstructing rotation matrices for ambient representation...")
    dtheta = dx[:, 6:9]
    R_flat = reconstruct_R(r_ref, dtheta)
    X_ambient = np.hstack([dx[:, 0:6], R_flat, du])
    
    print(f"Prepared datasets: ECRL {X_ecrl.shape}, Ambient {X_ambient.shape}")
    return X_ecrl, X_ambient, res

class SimpleMLP(nn.Module):
    def __init__(self, input_dim, output_dim, hidden_dims=[128, 128, 128]):
        super().__init__()
        layers = []
        prev_dim = input_dim
        for h in hidden_dims:
            layers.append(spectral_norm(nn.Linear(prev_dim, h)))
            layers.append(nn.Tanh())
            prev_dim = h
        layers.append(spectral_norm(nn.Linear(prev_dim, output_dim)))
        self.net = nn.Sequential(*layers)
        
    def forward(self, x):
        return self.net(x)

def train_and_eval(X_train, y_train, X_test, y_test, epochs=40, batch_size=128, lr=1e-3):
    input_dim = X_train.shape[1]
    output_dim = y_train.shape[1]
    
    # Scale data
    X_mean, X_std = np.mean(X_train, axis=0), np.std(X_train, axis=0)
    X_std[X_std < 1e-8] = 1.0
    y_mean, y_std = np.mean(y_train, axis=0), np.std(y_train, axis=0)
    y_std[y_std < 1e-8] = 1.0
    
    X_train_norm = (X_train - X_mean) / X_std
    X_test_norm = (X_test - X_mean) / X_std
    y_train_norm = (y_train - y_mean) / y_std
    
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    
    train_ds = TensorDataset(torch.tensor(X_train_norm, dtype=torch.float32), torch.tensor(y_train_norm, dtype=torch.float32))
    train_loader = DataLoader(train_ds, batch_size=min(batch_size, len(X_train)), shuffle=True)
    
    model = SimpleMLP(input_dim, output_dim).to(device)
    optimizer = optim.Adam(model.parameters(), lr=lr)
    criterion = nn.MSELoss()
    
    scheduler = optim.lr_scheduler.OneCycleLR(
        optimizer, max_lr=lr,
        steps_per_epoch=len(train_loader),
        epochs=epochs,
        pct_start=0.1
    )
    
    # Training
    model.train()
    for epoch in range(epochs):
        for bx, by in train_loader:
            bx, by = bx.to(device), by.to(device)
            optimizer.zero_grad()
            pred = model(bx)
            loss = criterion(pred, by)
            loss.backward()
            torch.nn.utils.clip_grad_norm_(model.parameters(), max_norm=1.0)
            optimizer.step()
            scheduler.step()
            
    # Eval
    model.eval()
    with torch.no_grad():
        tx = torch.tensor(X_test_norm, dtype=torch.float32).to(device)
        pred_norm = model(tx).cpu().numpy()
        
    # De-normalize
    pred = pred_norm * y_std + y_mean
    rmse = np.sqrt(np.mean((pred - y_test)**2, axis=0))
    return rmse

def main():
    dataset_dir = "/home/dinesh/ecrl_datasets/windy"
    
    X_ecrl, X_ambient, y = prepare_data(dataset_dir)
    
    # Split: 80% train / 20% test (fixed test set)
    num_samples = len(y)
    indices = np.arange(num_samples)
    np.random.seed(42)
    np.random.shuffle(indices)
    
    split = int(0.8 * num_samples)
    train_indices = indices[:split]
    test_indices = indices[split:]
    
    X_ecrl_train, X_ecrl_test = X_ecrl[train_indices], X_ecrl[test_indices]
    X_ambient_train, X_ambient_test = X_ambient[train_indices], X_ambient[test_indices]
    y_train, y_test = y[train_indices], y[test_indices]
    
    N_sweep = [100, 250, 500, 1000, 2500, 5000, 10000, 20000]
    num_trials = 5
    
    results = {
        "N": N_sweep,
        "ecrl_pos_mean": [],
        "ecrl_pos_std": [],
        "ambient_pos_mean": [],
        "ambient_pos_std": []
    }
    
    print(f"\nStarting sample efficiency sweep with {num_trials} trials per N...")
    for N in N_sweep:
        ecrl_trial_pos = []
        ambient_trial_pos = []
        
        for trial in range(num_trials):
            # Subsample N training points
            sub_indices = np.random.choice(len(y_train), N, replace=False)
            
            # Train ECRL
            ecrl_rmse = train_and_eval(X_ecrl_train[sub_indices], y_train[sub_indices], X_ecrl_test, y_test)
            ecrl_pos = np.sqrt(np.mean(ecrl_rmse[0:3]**2))
            ecrl_trial_pos.append(ecrl_pos)
            
            # Train Ambient
            ambient_rmse = train_and_eval(X_ambient_train[sub_indices], y_train[sub_indices], X_ambient_test, y_test)
            ambient_pos = np.sqrt(np.mean(ambient_rmse[0:3]**2))
            ambient_trial_pos.append(ambient_pos)
            
        mean_epos = np.mean(ecrl_trial_pos)
        std_epos = np.std(ecrl_trial_pos)
        mean_apos = np.mean(ambient_trial_pos)
        std_apos = np.std(ambient_trial_pos)
        
        results["ecrl_pos_mean"].append(float(mean_epos))
        results["ecrl_pos_std"].append(float(std_epos))
        results["ambient_pos_mean"].append(float(mean_apos))
        results["ambient_pos_std"].append(float(std_apos))
        
        print(f"N = {N:5d} | ECRL Pos: {mean_epos:.6f} (±{std_epos:.6f}) | Ambient Pos: {mean_apos:.6f} (±{std_apos:.6f})")

    # Output table
    print("\n" + "="*85)
    print("                SAMPLE EFFICIENCY SWEEP RESULTS (AVERAGED OVER 5 TRIALS)")
    print("="*85)
    print(f"{'N Samples':10s} | {'ECRL Pos RMSE (m)':20s} | {'Ambient Pos RMSE (m)':22s} | {'ECRL Improvement':18s}")
    print("-"*85)
    for i in range(len(results["N"])):
        N = results["N"][i]
        epos = results["ecrl_pos_mean"][i]
        estep = results["ecrl_pos_std"][i]
        apos = results["ambient_pos_mean"][i]
        astep = results["ambient_pos_std"][i]
        imp = (apos - epos) / apos * 100.0
        print(f"{N:10d} | {epos:9.6f} ± {estep:8.6f} | {apos:10.6f} ± {astep:9.6f} | {imp:16.2f}%")
    print("="*85)
    
    # Save results to JSON
    out_dir = "/home/dinesh/ecrl_ws/src/ommpc_ros2/models"
    os.makedirs(out_dir, exist_ok=True)
    with open(os.path.join(out_dir, "sample_efficiency_results.json"), "w") as f:
        json.dump(results, f, indent=4)
        
    # Plot Results
    plt.figure(figsize=(10, 6))
    
    plt.errorbar(results["N"], results["ecrl_pos_mean"], yerr=results["ecrl_pos_std"], 
                 fmt='o-', capsize=5, label="ECRL (Error Coordinates)", linewidth=2)
    plt.errorbar(results["N"], results["ambient_pos_mean"], yerr=results["ambient_pos_std"], 
                 fmt='s-', capsize=5, label="Ambient (Flat vec(R))", linewidth=2)
    
    plt.xscale('log')
    plt.xlabel("Number of Training Samples (N)", fontsize=12)
    plt.ylabel("Test Set 3D Position RMSE (m)", fontsize=12)
    plt.title("Sample Efficiency: ECRL vs. Ambient MLP", fontsize=14)
    plt.grid(True, which="both", ls="--")
    plt.legend(fontsize=12)
    
    plot_path = os.path.join(out_dir, "sample_efficiency.png")
    plt.savefig(plot_path, dpi=150)
    plt.close()
    print(f"Saved sample efficiency plot to {plot_path}")

if __name__ == '__main__':
    main()
