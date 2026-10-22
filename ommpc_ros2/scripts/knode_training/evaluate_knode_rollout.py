#!/usr/bin/env python3
"""evaluate_knode_rollout.py -- the actual proof-of-value check for
train_knode.py: rolls out the new rollout-trained model AND the existing
production one-step-trained model recursively over the SAME held-out
windows, and reports RMSE as a function of rollout step for both, plus a
zero-correction (physics-only) reference line.

Defaults to evaluating on ecrl_datasets_v2_eval_windy/figure8/ -- the
figure-8-in-wind SITL flight collected this session, which is genuinely
held out (not "polynomial*" trajectories, never touched by
train_residuals_v2.py's or train_knode.py's training pools) and real
closed-loop flight data, not synthetic. A single aggregate RMSE number
would hide the actual point of this exercise: does the new model's error
grow more slowly than the old one's as the rollout gets longer? That's the
step-vs-RMSE curve this script produces.
"""
import argparse
import os
import sys

import numpy as np
import torch
import matplotlib.pyplot as plt

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
import residuals as R  # noqa: E402
import so3_torch as T  # noqa: E402
from windows import build_rollout_windows, RolloutWindowDataset  # noqa: E402
from train_knode import build_features  # noqa: E402
from torch.utils.data import DataLoader  # noqa: E402


def load_eval_windows(csv_path, rollout_steps, stride, tracking_only=True):
    df = R.load_raw_csv(csv_path)
    if tracking_only and "fsm_state_k" in df.columns:
        n_before = len(df)
        df = df[df["fsm_state_k"] == 4].reset_index(drop=True)  # TRACKING, see ommpc_node.cpp's FsmState enum
        print(f"  {csv_path}: {n_before} rows -> {len(df)} TRACKING rows")
    df = R.compute_targets(df)
    windows = build_rollout_windows(df, rollout_steps, stride)
    print(f"  -> {len(windows)} rollout windows (rollout_steps={rollout_steps}, stride={stride})")
    return windows


@torch.no_grad()
def rollout_stepwise_norms(model, batch, adapter, rollout_steps, zero_correction=False):
    """Returns (rollout_steps, B) array of per-step boxminus error norms
    (world-frame, physical units). `model` must be self-normalizing
    (NormalizedResidualMLP-style TorchScript module: takes raw features,
    returns raw-scale d_k) or None if zero_correction."""
    p_hat, v_hat, q_hat = batch["p0"], batch["v0"], batch["q0"]
    norms = np.zeros((rollout_steps, p_hat.shape[0]))
    for t in range(rollout_steps):
        if zero_correction:
            d_pred = torch.zeros(p_hat.shape[0], 9, dtype=p_hat.dtype)
        else:
            feat = build_features(adapter, p_hat, v_hat, q_hat, batch, t)
            d_pred = model(feat)

        thracc_cmd_t = batch["thracc_cmd"][:, t]
        w_cmd_t = batch["w_cmd"][:, t]
        dt_t = batch["dt"][:, t]
        nom_p, nom_v, nom_q = T.nominal_step(p_hat, v_hat, q_hat, thracc_cmd_t, w_cmd_t, dt_t)
        p_hat, v_hat, q_hat = T.compose_from_residual(nom_p, nom_v, nom_q, d_pred)

        p_meas_t = batch["p_kp1"][:, t]
        v_meas_t = batch["v_kp1"][:, t]
        q_meas_t = batch["q_kp1"][:, t]
        err = T.boxminus(p_meas_t, v_meas_t, q_meas_t, p_hat, v_hat, q_hat)
        norms[t] = err.norm(dim=-1).cpu().numpy()
    return norms


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--csv_path", default=os.path.expanduser("~/ecrl_datasets_v2_eval_windy/figure8/figure8_windy_run1.csv"),
                         help="Held-out schema_version=2 CSV to evaluate on. Default: this session's "
                              "figure-8-in-wind SITL flight, never seen by any training pool.")
    parser.add_argument("--adapter", default="ecrl", choices=["ecrl", "ambient"])
    parser.add_argument("--rollout_steps", type=int, default=20,
                         help="Default 20 = the live MPC's horizon (nstep in ommpc_controller.hpp) -- "
                              "the operationally relevant length, even if it exceeds what either model "
                              "was trained with; degrading gracefully past the training horizon is itself "
                              "informative.")
    parser.add_argument("--stride", type=int, default=5)
    parser.add_argument("--knode_model",
                         default=os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                               "output", "ecrl_knode_model_normalized.pt"))
    parser.add_argument("--baseline_model",
                         default=os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..",
                                               "models", "ecrl_model_normalized.pt"),
                         help="The existing, one-step-trained production model.")
    parser.add_argument("--out",
                         default=os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                               "output", "rollout_rmse_comparison.png"))
    args = parser.parse_args()

    print(f"Loading held-out windows from {args.csv_path}")
    windows = load_eval_windows(args.csv_path, args.rollout_steps, args.stride)
    if not windows:
        raise ValueError("No windows built -- check --csv_path/--rollout_steps.")
    loader = DataLoader(RolloutWindowDataset(windows), batch_size=256, shuffle=False)

    print(f"Loading knode model: {args.knode_model}")
    knode_model = torch.jit.load(args.knode_model)
    knode_model.eval()
    print(f"Loading baseline model: {args.baseline_model}")
    baseline_model = torch.jit.load(args.baseline_model)
    baseline_model.eval()

    all_norms = {"zero-correction (physics-only)": [], "baseline (one-step-trained)": [], "knode (rollout-trained)": []}
    for batch in loader:
        all_norms["zero-correction (physics-only)"].append(
            rollout_stepwise_norms(None, batch, args.adapter, args.rollout_steps, zero_correction=True))
        all_norms["baseline (one-step-trained)"].append(
            rollout_stepwise_norms(baseline_model, batch, args.adapter, args.rollout_steps))
        all_norms["knode (rollout-trained)"].append(
            rollout_stepwise_norms(knode_model, batch, args.adapter, args.rollout_steps))

    rmse_by_step = {}
    for name, chunks in all_norms.items():
        norms = np.concatenate(chunks, axis=1)  # (rollout_steps, N_windows)
        rmse_by_step[name] = np.sqrt(np.mean(norms ** 2, axis=1))

    print("\nRMSE by rollout step:")
    header = "step".rjust(6) + "".join(name.rjust(28) for name in rmse_by_step)
    print(header)
    for t in range(args.rollout_steps):
        row = str(t + 1).rjust(6) + "".join(f"{rmse_by_step[name][t]:.5f}".rjust(28) for name in rmse_by_step)
        print(row)

    colors = {"zero-correction (physics-only)": "#767671",
              "baseline (one-step-trained)": "#eb6834",
              "knode (rollout-trained)": "#1baf7a"}
    plt.figure(figsize=(8, 5.5))
    steps = np.arange(1, args.rollout_steps + 1)
    for name, rmse in rmse_by_step.items():
        plt.plot(steps, rmse, marker="o", markersize=3, label=name, color=colors.get(name))
    plt.xlabel("Rollout step (100Hz cycles ahead)")
    plt.ylabel("RMSE, |boxminus(measured, predicted)| (world frame, 9-D)")
    plt.title(f"Multi-step rollout error vs. horizon depth\n{os.path.basename(args.csv_path)}, adapter={args.adapter}")
    plt.legend()
    plt.grid(True, alpha=0.3)
    plt.tight_layout()
    os.makedirs(os.path.dirname(args.out), exist_ok=True)
    plt.savefig(args.out, dpi=150)
    print(f"\nSaved -> {args.out}")


if __name__ == "__main__":
    main()
