#!/usr/bin/env python3
"""
train_residuals_v2.py — Diversity-Aware Residual Training for ECRL

Strategy to avoid overfitting to specific wind parameters:
  1. Mix both windy and non-windy data so the residual learns
     *general* model mismatch, not just wind compensation.
  2. Subsample trajectory files (not all 300+300) to limit data volume.
  3. Filter out near-zero residual rows (hovering/stationary) that
     add bulk without information.
  4. Stratified spatial binning over the 16-D input space so every
     region of the dynamics is represented roughly equally.
  5. Strong regularization: weight decay, early stopping, gradient
     clipping, and moderate network capacity.

Data format (see docs/residual_conventions.md and scripts/residuals.py):
  Trains on the *ecrl* adapter's (X, Y) pair from residuals.py, computed from
  raw schema_version=2 logs produced by ecrl_data_collector_node.cpp:
    X: 16-D [controller error state dx (9) ; control error du (4) ;
             Log(R_ref_k) r_ref (3)]  -- same layout as the pre-refactor
       dataset, so the feature semantics below (dx_0..dx_5, du_0) still hold.
    Y: 9-D residual against the exact nonlinear one-step model (not the old
       linearized-model residual), with its attitude block (Y[:,6:9])
       reparametrized into error coordinates relative to R_ref_k. A model
       trained on this Y predicts in that SAME error frame -- transport its
       output back to world frame with
       residuals.py::transport_ecrl_prediction_to_world() before using it as
       a world-frame correction (e.g. once wired into the MPC as an affine
       QP offset). Do NOT point --calm_dir/--windy_dir at old
       (schema_version=1) datasets; load_csv_files() below will reject them.
"""

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

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import residuals as R  # noqa: E402


# ─── Dataset ────────────────────────────────────────────────────────────
class ResidualDataset(Dataset):
    def __init__(self, inputs, targets):
        self.inputs = torch.tensor(inputs, dtype=torch.float32)
        self.targets = torch.tensor(targets, dtype=torch.float32)

    def __len__(self):
        return len(self.inputs)

    def __getitem__(self, idx):
        return self.inputs[idx], self.targets[idx]


# ─── Model ──────────────────────────────────────────────────────────────
class ResidualMLP(nn.Module):
    def __init__(self, input_dim=16, output_dim=9,
                 hidden_dims=[128, 128, 128], activation='tanh'):
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


# ─── Data Loading ───────────────────────────────────────────────────────
def load_csv_files(dataset_dir, max_files=None, rng=None):
    """Load raw schema_version=2 CSVs from a single directory (as produced by
    ecrl_data_collector_node.cpp), optionally subsampling files, and turn each
    into (X, y) via residuals.py's shared target + ecrl adapter. Every
    per-row residual definition lives in residuals.py, not here -- this
    function only does file discovery/subsampling and stacks the results.
    """
    csv_pattern = os.path.join(dataset_dir, "polynomial*.csv")
    all_files = sorted(glob.glob(csv_pattern))
    # Exclude the giant polynomial300 aggregated file if present
    all_files = [f for f in all_files if "polynomial300" not in os.path.basename(f)]

    if len(all_files) == 0:
        raise FileNotFoundError(f"No polynomial CSV files in {dataset_dir}")

    if max_files is not None and max_files < len(all_files):
        if rng is None:
            rng = np.random.default_rng(42)
        chosen = rng.choice(len(all_files), max_files, replace=False)
        all_files = [all_files[i] for i in sorted(chosen)]

    print(f"  Loading {len(all_files)} files from {dataset_dir}")

    input_list, target_list, source_tags = [], [], []

    for f in tqdm(all_files, desc=f"  {os.path.basename(dataset_dir)}"):
        try:
            df = R.load_raw_csv(f)
        except ValueError as e:
            print(f"  SKIP {f}: {e}")
            continue

        if len(df) < 10:
            continue

        df = R.compute_targets(df)
        X, Y, _transports = R.ecrl_features_and_target(df)

        valid = df["target_valid"].values & ~np.isnan(Y).any(axis=1)
        if valid.sum() < 10:
            continue

        input_list.append(X[valid])
        target_list.append(Y[valid])
        source_tags.append(os.path.basename(f))

    if not input_list:
        raise ValueError(f"No valid data loaded from {dataset_dir}")

    X = np.vstack(input_list)
    y = np.vstack(target_list)
    return X, y


# ─── Diversity-Aware Sampling ───────────────────────────────────────────
def filter_trivial_residuals(X, y, percentile=10):
    """Remove samples whose residual magnitude is below the given percentile.

    This drops near-zero residual rows (hovering / steady-state) that carry
    little signal and would dilute the training set.
    """
    res_mag = np.linalg.norm(y, axis=1)
    threshold = np.percentile(res_mag, percentile)
    mask = res_mag > threshold
    print(f"  Residual filter: keeping {mask.sum()}/{len(mask)} samples "
          f"(threshold = {threshold:.2e})")
    return X[mask], y[mask]


def stratified_bin_sampling(X, y, n_target, n_bins_per_dim=5, rng=None):
    """Select ~n_target samples by stratified binning over key input dimensions.

    We bin over a few high-information dimensions (position errors, velocity
    errors, thrust) so that every region of the dynamics is represented,
    rather than over-representing the most common operating point.

    For generalization, this ensures the model has seen diverse states instead
    of being dominated by whatever wind condition produced the most data.
    """
    if rng is None:
        rng = np.random.default_rng(42)

    N = len(X)
    if N <= n_target:
        print(f"  Stratified sampling: data ({N}) <= target ({n_target}), keeping all.")
        return X, y

    # Key dimensions for binning:
    #   dx_0..dx_2 (position error), dx_3..dx_5 (velocity error), du_0 (thrust)
    key_dims = [0, 1, 2, 3, 4, 5, 9]   # indices into the 16-D input
    X_key = X[:, key_dims]

    # Quantile-based bins to handle non-uniform distributions
    bin_edges = []
    for d in range(X_key.shape[1]):
        edges = np.percentile(X_key[:, d],
                              np.linspace(0, 100, n_bins_per_dim + 1))
        # Make edges unique
        edges = np.unique(edges)
        bin_edges.append(edges)

    # Assign each sample to a bin (tuple of per-dim indices)
    bin_ids = np.zeros(N, dtype=int)
    multiplier = 1
    for d in range(len(key_dims)):
        digit = np.digitize(X_key[:, d], bin_edges[d][1:-1])  # 0..n_bins-1
        bin_ids += digit * multiplier
        multiplier *= (len(bin_edges[d]) - 1) if len(bin_edges[d]) > 1 else 1

    unique_bins = np.unique(bin_ids)
    n_bins = len(unique_bins)
    samples_per_bin = max(1, n_target // n_bins)

    selected = []
    for b in unique_bins:
        members = np.where(bin_ids == b)[0]
        k = min(len(members), samples_per_bin)
        chosen = rng.choice(members, k, replace=False)
        selected.append(chosen)

    selected = np.concatenate(selected)

    # If we're short, top up randomly from remaining
    if len(selected) < n_target:
        remaining = np.setdiff1d(np.arange(N), selected)
        extra = rng.choice(remaining,
                           min(n_target - len(selected), len(remaining)),
                           replace=False)
        selected = np.concatenate([selected, extra])

    # If we're over, trim randomly
    if len(selected) > n_target:
        selected = rng.choice(selected, n_target, replace=False)

    selected = np.sort(selected)
    print(f"  Stratified sampling: {N} → {len(selected)} samples "
          f"across {n_bins} bins")
    return X[selected], y[selected]


# ─── Main ───────────────────────────────────────────────────────────────
def main():
    parser = argparse.ArgumentParser(
        description="Train ECRL residual with diversity-aware sample selection")

    parser.add_argument("--calm_dir", type=str,
                        default="/home/dinesh/ecrl_datasets_v2",
                        help="Non-windy (calm) dataset directory (raw schema_version=2 CSVs)")
    parser.add_argument("--windy_dir", type=str,
                        default="/home/dinesh/ecrl_datasets_v2/windy",
                        help="Windy dataset directory (raw schema_version=2 CSVs)")
    parser.add_argument("--output_dir", type=str,
                        default="/home/dinesh/ecrl_ws/src/ommpc_ros2/models",
                        help="Directory to save output models")

    # ── Sample selection knobs ──
    parser.add_argument("--calm_files", type=int, default=30,
                        help="Number of calm trajectory files to use")
    parser.add_argument("--windy_files", type=int, default=40,
                        help="Number of windy trajectory files to use")
    parser.add_argument("--target_samples", type=int, default=4000,
                        help="Target number of training samples after selection")
    parser.add_argument("--residual_filter_pct", type=float, default=15,
                        help="Percentile below which residuals are considered trivial")
    parser.add_argument("--n_bins", type=int, default=5,
                        help="Bins per dimension for stratified sampling")

    # ── Training knobs ──
    parser.add_argument("--epochs", type=int, default=80,
                        help="Maximum number of training epochs")
    parser.add_argument("--batch_size", type=int, default=128,
                        help="Batch size")
    parser.add_argument("--lr", type=float, default=5e-4,
                        help="Peak learning rate")
    parser.add_argument("--weight_decay", type=float, default=1e-4,
                        help="AdamW weight decay")
    parser.add_argument("--patience", type=int, default=12,
                        help="Early stopping patience (epochs)")
    parser.add_argument("--hidden_dims", type=int, nargs="+",
                        default=[128, 128],
                        help="Hidden layer dimensions (smaller = less overfit)")
    parser.add_argument("--activation", type=str, default="tanh",
                        choices=["relu", "tanh", "elu"])
    parser.add_argument("--seed", type=int, default=42)

    args = parser.parse_args()
    os.makedirs(args.output_dir, exist_ok=True)
    rng = np.random.default_rng(args.seed)
    torch.manual_seed(args.seed)

    # ──────────────────────────────────────────────────────────────────
    # 1. Load and merge data from both pools
    # ──────────────────────────────────────────────────────────────────
    print("="*60)
    print("STEP 1: Loading data")
    print("="*60)

    X_calm, y_calm = load_csv_files(args.calm_dir,
                                     max_files=args.calm_files, rng=rng)
    X_windy, y_windy = load_csv_files(args.windy_dir,
                                       max_files=args.windy_files, rng=rng)

    print(f"\n  Calm pool:  {X_calm.shape[0]:>8d} samples")
    print(f"  Windy pool: {X_windy.shape[0]:>8d} samples")

    X_all = np.vstack([X_calm, X_windy])
    y_all = np.vstack([y_calm, y_windy])
    print(f"  Merged:     {X_all.shape[0]:>8d} samples")

    # ──────────────────────────────────────────────────────────────────
    # 2. Filter trivial (near-zero) residuals
    # ──────────────────────────────────────────────────────────────────
    print("\n" + "="*60)
    print("STEP 2: Filtering trivial residuals")
    print("="*60)

    X_filt, y_filt = filter_trivial_residuals(
        X_all, y_all, percentile=args.residual_filter_pct)

    # ──────────────────────────────────────────────────────────────────
    # 3. Stratified diversity sampling
    # ──────────────────────────────────────────────────────────────────
    print("\n" + "="*60)
    print("STEP 3: Stratified diversity sampling")
    print("="*60)

    X_sel, y_sel = stratified_bin_sampling(
        X_filt, y_filt, n_target=args.target_samples,
        n_bins_per_dim=args.n_bins, rng=rng)

    # ──────────────────────────────────────────────────────────────────
    # 4. Normalize
    # ──────────────────────────────────────────────────────────────────
    X_mean = np.mean(X_sel, axis=0)
    X_std  = np.std(X_sel, axis=0)
    X_std[X_std < 1e-8] = 1.0

    y_mean = np.mean(y_sel, axis=0)
    y_std  = np.std(y_sel, axis=0)
    y_std[y_std < 1e-8] = 1.0

    X_norm = (X_sel - X_mean) / X_std
    y_norm = (y_sel - y_mean) / y_std

    # ──────────────────────────────────────────────────────────────────
    # 5. Train / Val / Test split  (70 / 15 / 15)
    # ──────────────────────────────────────────────────────────────────
    num = len(X_norm)
    idx = np.arange(num)
    rng.shuffle(idx)

    tr_end  = int(0.70 * num)
    val_end = int(0.85 * num)

    tr_idx  = idx[:tr_end]
    val_idx = idx[tr_end:val_end]
    te_idx  = idx[val_end:]

    print(f"\n  Train: {len(tr_idx)}  Val: {len(val_idx)}  Test: {len(te_idx)}")

    train_ds = ResidualDataset(X_norm[tr_idx], y_norm[tr_idx])
    val_ds   = ResidualDataset(X_norm[val_idx], y_norm[val_idx])
    test_ds  = ResidualDataset(X_norm[te_idx], y_norm[te_idx])

    train_loader = DataLoader(train_ds, batch_size=args.batch_size, shuffle=True)
    val_loader   = DataLoader(val_ds,   batch_size=args.batch_size, shuffle=False)
    test_loader  = DataLoader(test_ds,  batch_size=args.batch_size, shuffle=False)

    # ──────────────────────────────────────────────────────────────────
    # 6. Build model
    # ──────────────────────────────────────────────────────────────────
    input_dim  = X_sel.shape[1]
    output_dim = y_sel.shape[1]

    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    print(f"\n  Device: {device}")
    print(f"  Model: MLP {[input_dim] + args.hidden_dims + [output_dim]}")

    model = ResidualMLP(input_dim=input_dim, output_dim=output_dim,
                        hidden_dims=args.hidden_dims,
                        activation=args.activation).to(device)

    optimizer = optim.AdamW(model.parameters(),
                            lr=args.lr, weight_decay=args.weight_decay)
    criterion = nn.MSELoss()

    # OneCycleLR for controlled learning rate schedule
    scheduler = optim.lr_scheduler.OneCycleLR(
        optimizer, max_lr=args.lr,
        steps_per_epoch=len(train_loader),
        epochs=args.epochs,
        pct_start=0.15)

    # ──────────────────────────────────────────────────────────────────
    # 7. Training loop with early stopping
    # ──────────────────────────────────────────────────────────────────
    print("\n" + "="*60)
    print("STEP 7: Training")
    print("="*60)

    best_val = float("inf")
    wait = 0
    train_losses, val_losses = [], []

    for epoch in range(1, args.epochs + 1):
        # ── Train ──
        model.train()
        epoch_loss = 0.0
        for bx, by in train_loader:
            bx, by = bx.to(device), by.to(device)
            optimizer.zero_grad()
            pred = model(bx)
            loss = criterion(pred, by)
            loss.backward()
            torch.nn.utils.clip_grad_norm_(model.parameters(), max_norm=1.0)
            optimizer.step()
            scheduler.step()
            epoch_loss += loss.item() * len(bx)
        epoch_loss /= len(tr_idx)
        train_losses.append(epoch_loss)

        # ── Validate ──
        model.eval()
        vloss = 0.0
        with torch.no_grad():
            for bx, by in val_loader:
                bx, by = bx.to(device), by.to(device)
                vloss += criterion(model(bx), by).item() * len(bx)
        vloss /= len(val_idx)
        val_losses.append(vloss)

        if epoch % 5 == 0 or epoch == 1:
            print(f"  Epoch {epoch:3d}/{args.epochs}  "
                  f"Train: {epoch_loss:.6f}  Val: {vloss:.6f}")

        # ── Early stopping ──
        if vloss < best_val:
            best_val = vloss
            wait = 0
            torch.save(model.state_dict(),
                       os.path.join(args.output_dir, "best_model.pt"))
        else:
            wait += 1
            if wait >= args.patience:
                print(f"\n  Early stopping at epoch {epoch} "
                      f"(best val = {best_val:.6f})")
                break

    print(f"\n  Training finished. Best Val Loss: {best_val:.6f}")

    # ──────────────────────────────────────────────────────────────────
    # 8. Evaluate on test set
    # ──────────────────────────────────────────────────────────────────
    model.load_state_dict(
        torch.load(os.path.join(args.output_dir, "best_model.pt"),
                   weights_only=True))
    model.eval()

    all_preds, all_tgts = [], []
    with torch.no_grad():
        for bx, by in test_loader:
            bx = bx.to(device)
            all_preds.append(model(bx).cpu().numpy())
            all_tgts.append(by.numpy())

    preds = np.vstack(all_preds)
    tgts  = np.vstack(all_tgts)

    # De-normalise
    preds_phys = preds * y_std + y_mean
    tgts_phys  = tgts  * y_std + y_mean

    coord_rmse = np.sqrt(np.mean((preds_phys - tgts_phys)**2, axis=0))
    coord_names = [
        "Position X (res_0)", "Position Y (res_1)", "Position Z (res_2)",
        "Velocity X (res_3)", "Velocity Y (res_4)", "Velocity Z (res_5)",
        "Attitude res_6 (error-frame, ref. R_ref_k)",
        "Attitude res_7 (error-frame, ref. R_ref_k)",
        "Attitude res_8 (error-frame, ref. R_ref_k)",
    ]

    print("\n" + "="*60)
    print("TEST SET RMSE (Physical Coordinates)")
    print("="*60)
    for name, rmse in zip(coord_names, coord_rmse):
        print(f"  {name:28s}: {rmse:.6f}")

    pos_rmse_3d = np.sqrt(np.mean(coord_rmse[:3]**2))
    print(f"\n  3D Position RMSE: {pos_rmse_3d:.6f} m")

    # ──────────────────────────────────────────────────────────────────
    # 9. Save metadata
    # ──────────────────────────────────────────────────────────────────
    metadata = {
        "input_dim": int(input_dim),
        "output_dim": int(output_dim),
        "input_mean": X_mean.tolist(),
        "input_std": X_std.tolist(),
        "output_mean": y_mean.tolist(),
        "output_std": y_std.tolist(),
        "coordinate_names": coord_names,
        "test_rmse": coord_rmse.tolist(),
        "residual_definition": "ecrl_adapter_v2",  # see docs/residual_conventions.md; output attitude
                                                    # block is in error coordinates relative to R_ref_k,
                                                    # NOT world frame -- transport before deployment.
        "training_info": {
            "strategy": "diversity-aware: mixed calm+windy, residual filtering, stratified spatial binning",
            "calm_files_used": args.calm_files,
            "windy_files_used": args.windy_files,
            "target_samples": args.target_samples,
            "actual_train_samples": int(len(tr_idx)),
            "residual_filter_percentile": args.residual_filter_pct,
            "hidden_dims": args.hidden_dims,
            "weight_decay": args.weight_decay,
            "best_val_loss": float(best_val),
        }
    }

    meta_path = os.path.join(args.output_dir, "ecrl_metadata.json")
    with open(meta_path, "w") as f:
        json.dump(metadata, f, indent=4)
    print(f"\n  Saved metadata → {meta_path}")

    # ──────────────────────────────────────────────────────────────────
    # 10. Export models (remove spectral norm hooks first)
    # ──────────────────────────────────────────────────────────────────
    model.cpu()
    _ = model(torch.zeros(1, input_dim))   # prime spectral norm buffers
    for m in model.modules():
        if isinstance(m, nn.Linear):
            try:
                remove_spectral_norm(m)
            except ValueError:
                pass

    norm_model = NormalizedResidualMLP(model, X_mean, X_std, y_mean, y_std)
    dummy = torch.randn(1, input_dim)

    # ONNX – unnormalised
    try:
        onnx_path = os.path.join(args.output_dir, "ecrl_model.onnx")
        torch.onnx.export(model, dummy, onnx_path,
                          export_params=True, opset_version=11,
                          do_constant_folding=True,
                          input_names=["input"], output_names=["output"])
        print(f"  ONNX (unnorm)  → {onnx_path}")
    except Exception as e:
        print(f"  ONNX export failed: {e}")

    # ONNX – normalised
    try:
        onnx_norm = os.path.join(args.output_dir, "ecrl_model_normalized.onnx")
        torch.onnx.export(norm_model, dummy, onnx_norm,
                          export_params=True, opset_version=11,
                          do_constant_folding=True,
                          input_names=["input"], output_names=["output"])
        print(f"  ONNX (norm)    → {onnx_norm}")
    except Exception as e:
        print(f"  ONNX norm export failed: {e}")

    # TorchScript – unnormalised
    try:
        ts_path = os.path.join(args.output_dir, "ecrl_model.pt")
        torch.jit.script(model).save(ts_path)
        print(f"  TorchScript    → {ts_path}")
    except Exception as e:
        print(f"  TorchScript export failed: {e}")

    # TorchScript – normalised
    try:
        ts_norm = os.path.join(args.output_dir, "ecrl_model_normalized.pt")
        torch.jit.script(norm_model).save(ts_norm)
        print(f"  TorchScript (norm) → {ts_norm}")
    except Exception as e:
        print(f"  TorchScript norm export failed: {e}")

    # ──────────────────────────────────────────────────────────────────
    # 11. Training curves plot
    # ──────────────────────────────────────────────────────────────────
    plt.figure(figsize=(10, 5))
    plt.plot(train_losses, label="Train Loss")
    plt.plot(val_losses, label="Val Loss")
    plt.xlabel("Epoch")
    plt.ylabel("MSE Loss (Normalised)")
    plt.title("Diversity-Aware Residual Training Curves")
    plt.legend()
    plt.grid(True)
    plot_path = os.path.join(args.output_dir, "training_curves.png")
    plt.savefig(plot_path, dpi=150)
    plt.close()
    print(f"  Training curves → {plot_path}")

    print("\n" + "="*60)
    print("DONE — model ready for deployment")
    print("="*60)


if __name__ == "__main__":
    main()
