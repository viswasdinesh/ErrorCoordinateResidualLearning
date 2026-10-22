#!/usr/bin/env python3
"""sample_efficiency_sweep.py -- ecrl vs ambient sample-efficiency comparison
on the current v2 pipeline (residuals.py / ecrl_datasets_v2), replacing the
stale pre-refactor version that read baked-in dx_*/res_* columns from
~/ecrl_datasets/windy (old schema_version=1 layout).

Trains fresh ecrl (16-D) and ambient (8-D) models at N in a sweep, several
trials per N, on random subsamples of a fixed 80/20 train/test split of the
combined calm+windy ecrl_datasets_v2 pool, reporting 3D position RMSE vs N
to show ecrl's sample-efficiency advantage over ambient (or the lack of one,
at whatever N it shows up).

Reuses train_residuals_v2.py's load_csv_files() (residuals.py-backed
loading, both adapters) and ResidualMLP (spectral_norm, [128,128] hidden --
matching models/ecrl_model.pt / models/ambient_model.pt's actual
architecture, unlike the old script's ad hoc 3-hidden-layer SimpleMLP)
unmodified, so results are directly comparable to the production models.

Unlike train_residuals_v2.py's diversity-aware sampling (residual filtering,
stratified spatial binning), this sweep intentionally does NOT apply those
-- it's asking "how much *raw* data is needed", not reproducing the
production training recipe.

Usage:
    conda run -n ecrl python3 scripts/sample_efficiency_sweep.py
"""
import os
import sys
import json

import numpy as np
import torch
import torch.nn as nn
import torch.optim as optim
from torch.utils.data import TensorDataset, DataLoader
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from train_residuals_v2 import load_csv_files, ResidualMLP  # noqa: E402

CALM_DIR = os.path.expanduser("~/ecrl_datasets_v2")
WINDY_DIR = os.path.expanduser("~/ecrl_datasets_v2/windy")
N_SWEEP = [100, 250, 500, 1000, 2500, 5000, 10000, 20000, 30000, 40000, 50000]
NUM_TRIALS = 5
# 9-D output block structure (residuals.py::compute_targets' d_0..d_8 order).
# Position's output_std is ~40-80x smaller than velocity's (models/
# ecrl_metadata.json) -- one physics-integration step barely moves position
# regardless of wind, so it's nearly deterministic and a weak place to look
# for a model-quality gap. Velocity is where f_wind(v,R) directly enters the
# dynamics (v_dot = a_thrust R e3 - g e3 + f_wind), so report all blocks
# rather than defaulting to position (the old script's only reported metric).
BLOCKS = {"position": slice(0, 3), "velocity": slice(3, 6), "attitude": slice(6, 9), "overall": slice(0, 9)}
HIDDEN_DIMS = [128, 128]  # matches production models/{ecrl,ambient}_model.pt
EPOCHS = 40
BATCH_SIZE = 128
LR = 1e-3
SPLIT_SEED = 42
SUBSAMPLE_SEED = 123
OUT_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "models")


def load_adapter_pool(adapter: str):
    print(f"\nLoading '{adapter}' adapter pool...")
    Xc, yc = load_csv_files(CALM_DIR, adapter=adapter)
    Xw, yw = load_csv_files(WINDY_DIR, adapter=adapter)
    X = np.vstack([Xc, Xw])
    y = np.vstack([yc, yw])
    print(f"  {adapter}: {X.shape[0]} rows, X.shape[1]={X.shape[1]}")
    return X, y


def split_train_test(X, y, seed=SPLIT_SEED, train_frac=0.8):
    rng = np.random.default_rng(seed)
    idx = rng.permutation(len(X))
    n_train = int(train_frac * len(X))
    train_idx, test_idx = idx[:n_train], idx[n_train:]
    return X[train_idx], y[train_idx], X[test_idx], y[test_idx]


def train_and_eval(X_train, y_train, X_test, y_test, device,
                    epochs=EPOCHS, batch_size=BATCH_SIZE, lr=LR):
    input_dim = X_train.shape[1]
    output_dim = y_train.shape[1]

    X_mean, X_std = X_train.mean(axis=0), X_train.std(axis=0)
    X_std[X_std < 1e-8] = 1.0
    y_mean, y_std = y_train.mean(axis=0), y_train.std(axis=0)
    y_std[y_std < 1e-8] = 1.0

    X_train_n = (X_train - X_mean) / X_std
    X_test_n = (X_test - X_mean) / X_std
    y_train_n = (y_train - y_mean) / y_std

    train_ds = TensorDataset(torch.tensor(X_train_n, dtype=torch.float32),
                              torch.tensor(y_train_n, dtype=torch.float32))
    bs = min(batch_size, len(X_train))
    train_loader = DataLoader(train_ds, batch_size=bs, shuffle=True)

    model = ResidualMLP(input_dim=input_dim, output_dim=output_dim, hidden_dims=HIDDEN_DIMS).to(device)
    optimizer = optim.Adam(model.parameters(), lr=lr)
    criterion = nn.MSELoss()
    scheduler = optim.lr_scheduler.OneCycleLR(optimizer, max_lr=lr, steps_per_epoch=len(train_loader),
                                               epochs=epochs, pct_start=0.1)

    model.train()
    for _ in range(epochs):
        for bx, by in train_loader:
            bx, by = bx.to(device), by.to(device)
            optimizer.zero_grad()
            pred = model(bx)
            loss = criterion(pred, by)
            loss.backward()
            torch.nn.utils.clip_grad_norm_(model.parameters(), max_norm=1.0)
            optimizer.step()
            scheduler.step()

    model.eval()
    with torch.no_grad():
        tx = torch.tensor(X_test_n, dtype=torch.float32).to(device)
        pred_n = model(tx).cpu().numpy()
    pred = pred_n * y_std + y_mean
    rmse = np.sqrt(np.mean((pred - y_test) ** 2, axis=0))
    return rmse


def main():
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    print(f"Device: {device}")

    X_ecrl, y_ecrl = load_adapter_pool("ecrl")
    X_amb, y_amb = load_adapter_pool("ambient")

    # Fixed 80/20 split, independently per adapter -- load_csv_files' NaN
    # filtering can drop a slightly different row set per adapter, so the
    # two pools aren't guaranteed row-aligned. Each adapter is trained/
    # evaluated self-consistently against its own held-out 20%, which is
    # all a sample-efficiency comparison needs (both draw from the same
    # underlying flights either way).
    X_ecrl_train, y_ecrl_train, X_ecrl_test, y_ecrl_test = split_train_test(X_ecrl, y_ecrl)
    X_amb_train, y_amb_train, X_amb_test, y_amb_test = split_train_test(X_amb, y_amb)
    print(f"\necrl train/test:    {len(X_ecrl_train)} / {len(X_ecrl_test)}")
    print(f"ambient train/test: {len(X_amb_train)} / {len(X_amb_test)}")

    n_sweep = [n for n in N_SWEEP if n <= min(len(X_ecrl_train), len(X_amb_train))]
    if len(n_sweep) < len(N_SWEEP):
        print(f"Dropping N > available train pool: {[n for n in N_SWEEP if n not in n_sweep]}")

    results = {"N": n_sweep}
    for block in BLOCKS:
        results[f"ecrl_{block}_mean"] = []
        results[f"ecrl_{block}_std"] = []
        results[f"ambient_{block}_mean"] = []
        results[f"ambient_{block}_std"] = []

    rng = np.random.default_rng(SUBSAMPLE_SEED)
    print(f"\nStarting sample efficiency sweep: N={n_sweep}, {NUM_TRIALS} trials each")
    for N in n_sweep:
        ecrl_trials = {block: [] for block in BLOCKS}
        ambient_trials = {block: [] for block in BLOCKS}
        for _ in range(NUM_TRIALS):
            sub_e = rng.choice(len(X_ecrl_train), N, replace=False)
            sub_a = rng.choice(len(X_amb_train), N, replace=False)

            rmse_e = train_and_eval(X_ecrl_train[sub_e], y_ecrl_train[sub_e], X_ecrl_test, y_ecrl_test, device)
            rmse_a = train_and_eval(X_amb_train[sub_a], y_amb_train[sub_a], X_amb_test, y_amb_test, device)
            for block, sl in BLOCKS.items():
                ecrl_trials[block].append(float(np.sqrt(np.mean(rmse_e[sl] ** 2))))
                ambient_trials[block].append(float(np.sqrt(np.mean(rmse_a[sl] ** 2))))

        line = f"N = {N:6d} |"
        for block in BLOCKS:
            mean_e, std_e = float(np.mean(ecrl_trials[block])), float(np.std(ecrl_trials[block]))
            mean_a, std_a = float(np.mean(ambient_trials[block])), float(np.std(ambient_trials[block]))
            results[f"ecrl_{block}_mean"].append(mean_e)
            results[f"ecrl_{block}_std"].append(std_e)
            results[f"ambient_{block}_mean"].append(mean_a)
            results[f"ambient_{block}_std"].append(std_a)
            imp = (mean_a - mean_e) / mean_a * 100.0 if mean_a > 0 else float("nan")
            line += f" {block}: ecrl={mean_e:.5f} amb={mean_a:.5f} ({imp:+.1f}%) |"
        print(line)

    print("\n" + "=" * 100)
    print(f"        SAMPLE EFFICIENCY SWEEP RESULTS (v2 pipeline, AVERAGED OVER {NUM_TRIALS} TRIALS)")
    print("=" * 100)
    for block in BLOCKS:
        print(f"\n--- {block} ---")
        print(f"{'N':8s} | {'ECRL RMSE':>12s} | {'Ambient RMSE':>12s} | {'ECRL Improvement':>16s}")
        print("-" * 60)
        for i, N in enumerate(results["N"]):
            me = results[f"ecrl_{block}_mean"][i]
            ma = results[f"ambient_{block}_mean"][i]
            imp = (ma - me) / ma * 100.0 if ma > 0 else float("nan")
            print(f"{N:8d} | {me:12.6f} | {ma:12.6f} | {imp:15.2f}%")
    print("=" * 100)

    os.makedirs(OUT_DIR, exist_ok=True)
    out_json = os.path.join(OUT_DIR, "sample_efficiency_results_v2.json")
    with open(out_json, "w") as f:
        json.dump(results, f, indent=4)
    print(f"\nSaved results to {out_json}")

    fig, axes = plt.subplots(1, len(BLOCKS), figsize=(6 * len(BLOCKS), 5))
    for ax, block in zip(axes, BLOCKS):
        ax.errorbar(results["N"], results[f"ecrl_{block}_mean"], yerr=results[f"ecrl_{block}_std"],
                     fmt='o-', capsize=4, label="ECRL", linewidth=2)
        ax.errorbar(results["N"], results[f"ambient_{block}_mean"], yerr=results[f"ambient_{block}_std"],
                     fmt='s-', capsize=4, label="Ambient", linewidth=2)
        ax.set_xscale('log')
        ax.set_xlabel("Training samples (N)")
        ax.set_ylabel(f"Test RMSE ({block})")
        ax.set_title(block)
        ax.grid(True, which="both", ls="--", alpha=0.3)
        ax.legend()
    fig.suptitle("Sample Efficiency: ECRL vs. Ambient MLP, per output block (v2 pipeline)")
    fig.tight_layout()
    out_png = os.path.join(OUT_DIR, "sample_efficiency_v2.png")
    fig.savefig(out_png, dpi=150)
    plt.close(fig)
    print(f"Saved plot to {out_png}")


if __name__ == '__main__':
    main()
