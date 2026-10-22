#!/usr/bin/env python3
"""residual_data.py -- data loading, diversity-aware sample selection, and
normalization/splitting for residual dynamics training.

Split out of train_residuals_v2.py (which is now the CLI entry point/
orchestrator only) so the loading/filtering/splitting logic can be read and
reused independently of the training loop and export code. Also imported
directly by scripts/knode_training/train_knode.py and
scripts/sample_efficiency_sweep.py (via train_residuals_v2.py's
re-exports -- see that file).
"""
import os
import sys
import glob

import numpy as np
import torch
from torch.utils.data import Dataset
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


# ─── Data Loading ───────────────────────────────────────────────────────
def load_csv_files(dataset_dir, max_files=None, rng=None, adapter="ecrl"):
    """Load raw schema_version=2 CSVs from a single directory (as produced by
    ecrl_data_collector_node.cpp), optionally subsampling files, and turn each
    into (X, y) via residuals.py's shared target + the selected adapter
    ("ecrl" or "ambient"). Every per-row residual definition lives in
    residuals.py, not here -- this function only does file discovery/
    subsampling and stacks the results.
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
        if adapter == "ecrl":
            X, Y = R.ecrl_features_and_target(df)
        elif adapter == "ambient":
            X = R.ambient_features(df)
            Y = df[[f"d_{i}" for i in range(9)]].values
        else:
            raise ValueError(f"unknown adapter '{adapter}'")

        valid = df["target_valid"].values & ~np.isnan(X).any(axis=1) & ~np.isnan(Y).any(axis=1)
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


def stratified_bin_sampling(X, y, n_target, key_dims, n_bins_per_dim=5, rng=None):
    """Select ~n_target samples by stratified binning over key input dimensions.

    We bin over a few high-information dimensions (position/velocity errors
    and thrust for ecrl; body velocity/rate and thrust for ambient -- see
    key_dims below) so that every region of the dynamics is represented,
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


# ─── Normalization + Splitting ──────────────────────────────────────────
def normalize(X, y):
    """Z-score normalize X/y using statistics computed over the full input
    set (i.e. before splitting -- matches the original train_residuals_v2.py
    behavior; not train-split-only stats)."""
    X_mean = np.mean(X, axis=0)
    X_std = np.std(X, axis=0)
    X_std[X_std < 1e-8] = 1.0

    y_mean = np.mean(y, axis=0)
    y_std = np.std(y, axis=0)
    y_std[y_std < 1e-8] = 1.0

    X_norm = (X - X_mean) / X_std
    y_norm = (y - y_mean) / y_std
    return X_norm, y_norm, X_mean, X_std, y_mean, y_std


def split_train_val_test(X_norm, y_norm, train_frac=0.70, val_frac=0.15, rng=None):
    """Shuffled train/val/test split (default 70/15/15), returning
    ResidualDataset objects plus the index arrays used (the latter mainly so
    callers can report len(tr_idx) etc.)."""
    if rng is None:
        rng = np.random.default_rng(42)

    num = len(X_norm)
    idx = np.arange(num)
    rng.shuffle(idx)

    tr_end = int(train_frac * num)
    val_end = tr_end + int(val_frac * num)

    tr_idx = idx[:tr_end]
    val_idx = idx[tr_end:val_end]
    te_idx = idx[val_end:]

    print(f"\n  Train: {len(tr_idx)}  Val: {len(val_idx)}  Test: {len(te_idx)}")

    train_ds = ResidualDataset(X_norm[tr_idx], y_norm[tr_idx])
    val_ds = ResidualDataset(X_norm[val_idx], y_norm[val_idx])
    test_ds = ResidualDataset(X_norm[te_idx], y_norm[te_idx])

    return train_ds, val_ds, test_ds, tr_idx, val_idx, te_idx
