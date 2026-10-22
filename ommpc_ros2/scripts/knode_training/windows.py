#!/usr/bin/env python3
"""windows.py -- carves raw schema_version=2 CSVs (already run through
../residuals.py::compute_targets) into fixed-length, contiguous rollout
windows for train_knode.py's multi-step training loss.

A window of length W is W consecutive, contiguous rows [row_0..row_{W-1}]
from the SAME source file such that row_t's k+1 tick is exactly row_{t+1}'s
k tick (residuals.py::linearized_residual_diagnostic establishes and uses
this same contiguity property; reused verbatim here). Each window carries:
  - the rollout's starting MEASURED state (row_0's x_k) -- the only place a
    logged state is fed into the rollout itself,
  - the externally-given reference state/control and the actually-applied
    control at every step (needed to drive nominal_step and build adapter
    features at each step -- these are known in advance, not rolled out),
  - the measured ground-truth state at every step (row_t's x_{k+1}), to
    score the rollout against.
"""
import os
import sys

import numpy as np
import torch
from torch.utils.data import Dataset

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
import residuals as R  # noqa: E402


def _contiguous_to_next(df):
    """Per-row boolean: does this row's k+1 tick exactly match the NEXT
    row's (same source file, sorted by odom_ts_k_us) k tick? Same check as
    residuals.py::linearized_residual_diagnostic."""
    next_ts_k = df.groupby("_source_file")["odom_ts_k_us"].shift(-1)
    return (next_ts_k == df["odom_ts_kp1_us"]).values


def build_rollout_windows(df, rollout_steps, stride):
    """df must already have compute_targets() applied (needs target_valid,
    d_0..d_8 -- the latter unused here but its presence confirms the caller
    didn't skip that step). Returns a list of dicts, one per window, each
    holding NumPy arrays (see module docstring for the per-step fields)."""
    if "target_valid" not in df.columns:
        raise ValueError("build_rollout_windows requires compute_targets(df) to have been run first")

    df = df.sort_values(["_source_file", "odom_ts_k_us"]).reset_index(drop=True)
    contiguous = _contiguous_to_next(df)
    valid = df["target_valid"].values

    windows = []
    for fname, group in df.groupby("_source_file", sort=False):
        idx = group.index.values
        n = len(idx)
        if n < rollout_steps:
            continue
        for start in range(0, n - rollout_steps + 1, stride):
            rows = idx[start:start + rollout_steps]
            if not valid[rows].all():
                continue
            # contiguity needed for every consecutive pair WITHIN the window
            # (t -> t+1 for t=0..W-2); rows[-1]'s own contiguity to
            # whatever comes after is irrelevant here.
            if not contiguous[rows[:-1]].all():
                continue
            windows.append(_extract_window(df, rows, fname))

    return windows


def _extract_window(df, rows, fname):
    sub = df.loc[rows]
    r0 = df.loc[rows[0]]

    def cols(prefix, suffixes):
        return np.stack([sub[f"{prefix}{s}"].values for s in suffixes], axis=-1)

    return {
        "p0": np.array([r0.p_k_x, r0.p_k_y, r0.p_k_z]),
        "v0": np.array([r0.v_k_x, r0.v_k_y, r0.v_k_z]),
        "q0": np.array([r0.q_k_w, r0.q_k_x, r0.q_k_y, r0.q_k_z]),
        "p_ref": cols("p_ref_k_", ["x", "y", "z"]),
        "v_ref": cols("v_ref_k_", ["x", "y", "z"]),
        "q_ref": cols("q_ref_k_", ["w", "x", "y", "z"]),
        "thracc_ref": sub["thracc_ref_k"].values,
        "w_ref": np.stack([sub["wx_ref_k"].values, sub["wy_ref_k"].values, sub["wz_ref_k"].values], axis=-1),
        "thracc_cmd": sub["thracc_cmd_k"].values,
        "w_cmd": np.stack([sub["wx_cmd_k"].values, sub["wy_cmd_k"].values, sub["wz_cmd_k"].values], axis=-1),
        "wm": np.stack([sub["wm_k_x"].values, sub["wm_k_y"].values, sub["wm_k_z"].values], axis=-1),
        "thrust_norm_cmd": sub["thrust_norm_cmd_k"].values,
        "thr2acc": sub["thr2acc_k"].values,
        "dt": sub["dt_s"].values,
        "p_kp1": cols("p_kp1_", ["x", "y", "z"]),
        "v_kp1": cols("v_kp1_", ["x", "y", "z"]),
        "q_kp1": cols("q_kp1_", ["w", "x", "y", "z"]),
        "source_file": fname,
    }


_ARRAY_KEYS = ["p0", "v0", "q0", "p_ref", "v_ref", "q_ref", "thracc_ref", "w_ref",
               "thracc_cmd", "w_cmd", "wm", "thrust_norm_cmd", "thr2acc", "dt",
               "p_kp1", "v_kp1", "q_kp1"]


class RolloutWindowDataset(Dataset):
    """Wraps a list of window dicts (from build_rollout_windows) as a
    PyTorch Dataset; __getitem__ returns a dict of float32 tensors."""

    def __init__(self, windows):
        self.windows = windows

    def __len__(self):
        return len(self.windows)

    def __getitem__(self, idx):
        w = self.windows[idx]
        return {k: torch.tensor(w[k], dtype=torch.float32) for k in _ARRAY_KEYS}
