#!/usr/bin/env python3
"""plot_figure8_windy.py -- visualize a collected figure-8-in-wind flight:
reference path vs. actual measured path, plus the residual signal itself
(true nominal-dynamics mismatch d_k, and how much of it each trained model
removes), from a schema_version=2 raw CSV (e.g. produced by
collect_figure8_windy.py). TRACKING-only (fsm_state_k == 4); takeoff/hover
ramp-up is excluded so the plot shows just the commanded figure-8 maneuver.

Reuses residuals.py's compute_targets/ecrl_features_and_target/ambient_features
verbatim for the residual panels -- no residual math is reimplemented here.

Usage:
    conda run -n ecrl python3 scripts/plot_figure8_windy.py \\
        --csv ~/ecrl_datasets_v2_eval_windy/figure8/figure8_windy_run1.csv
"""
import argparse
import os
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import residuals as R  # noqa: E402

FSM_TRACKING = 4
SMOOTH_WINDOW = 60  # rows (~0.5s at the controller's ~125Hz odom rate) for the correction-comparison panel

REF_COLOR = "#2a78d6"       # categorical slot 1 (blue) -- commanded reference (position panels)
ACTUAL_COLOR = "#eb6834"    # categorical slot 2 (orange) -- measured/actual (position panels)
BASELINE_COLOR = "#767671"  # neutral gray -- zero-residual baseline (no correction)
ECRL_COLOR = "#1baf7a"      # categorical slot 3 (aqua) -- ecrl-corrected
AMBIENT_COLOR = "#4a3aa7"   # categorical slot 7 (violet) -- ambient-corrected
GRID_COLOR = "#c3c2b7"


def load_tracking_segment(csv_path: str) -> pd.DataFrame:
    with open(csv_path, "r") as f:
        first_line = f.readline()
    if not first_line.startswith("# ecrl_schema_version="):
        raise ValueError(f"{csv_path}: missing '# ecrl_schema_version=' header line")
    df = pd.read_csv(csv_path, comment="#")
    df = df.apply(pd.to_numeric, errors="coerce").dropna()
    df = df[df["fsm_state_k"] == FSM_TRACKING].sort_values("odom_ts_k_us").reset_index(drop=True)
    if len(df) == 0:
        raise ValueError(f"{csv_path}: no TRACKING rows (fsm_state_k == {FSM_TRACKING}) found")
    df["t_s"] = (df["odom_ts_k_us"] - df["odom_ts_k_us"].iloc[0]) * 1e-6
    return df


def model_corrected_errors(df: pd.DataFrame, ecrl_model_path: str, ambient_model_path: str):
    """Returns (t_s, baseline_norm, ecrl_err_norm, ambient_err_norm) aligned on the
    same valid-row mask, or None if torch/model files aren't available."""
    try:
        import torch
    except ImportError:
        print("  torch not importable -- skipping model-correction panel")
        return None

    for p in (ecrl_model_path, ambient_model_path):
        if not os.path.exists(p):
            print(f"  model not found: {p} -- skipping model-correction panel")
            return None

    df = R.compute_targets(df)
    X_ecrl, Y_ecrl = R.ecrl_features_and_target(df)
    X_amb = R.ambient_features(df)
    Y_amb = df[[f"d_{i}" for i in range(9)]].values

    valid = (df["target_valid"].values
             & ~np.isnan(X_ecrl).any(axis=1) & ~np.isnan(Y_ecrl).any(axis=1)
             & ~np.isnan(X_amb).any(axis=1) & ~np.isnan(Y_amb).any(axis=1))

    t_s = df["t_s"].values[valid]
    Y = Y_ecrl[valid]  # == Y_amb[valid], same world-frame d_k target either way

    ecrl_model = torch.jit.load(ecrl_model_path).eval()
    ambient_model = torch.jit.load(ambient_model_path).eval()
    with torch.no_grad():
        pred_ecrl = ecrl_model(torch.tensor(X_ecrl[valid], dtype=torch.float32)).numpy()
        pred_amb = ambient_model(torch.tensor(X_amb[valid], dtype=torch.float32)).numpy()

    baseline_norm = np.linalg.norm(Y, axis=1)
    ecrl_err_norm = np.linalg.norm(Y - pred_ecrl, axis=1)
    ambient_err_norm = np.linalg.norm(Y - pred_amb, axis=1)
    return t_s, baseline_norm, ecrl_err_norm, ambient_err_norm


def smooth(x: np.ndarray, window: int) -> np.ndarray:
    return pd.Series(x).rolling(window, center=True, min_periods=1).mean().values


def plot(df: pd.DataFrame, csv_path: str, out_path: str, ecrl_model_path: str, ambient_model_path: str):
    err_xy = np.hypot(df["p_k_x"] - df["p_ref_k_x"], df["p_k_y"] - df["p_ref_k_y"])
    correction = model_corrected_errors(df.copy(), ecrl_model_path, ambient_model_path)

    fig, axes = plt.subplots(2, 3, figsize=(16, 9))
    fig.suptitle(f"Figure-8, windy eval scene -- {os.path.basename(csv_path)}\n"
                 f"({len(df)} TRACKING rows, {df['t_s'].iloc[-1]:.0f}s)", fontsize=11)

    # --- XY plane: reference vs actual path ---
    ax = axes[0, 0]
    ax.plot(df["p_ref_k_x"], df["p_ref_k_y"], color=REF_COLOR, lw=2, label="reference")
    ax.plot(df["p_k_x"], df["p_k_y"], color=ACTUAL_COLOR, lw=1.5, alpha=0.9, label="actual")
    ax.scatter(df["p_k_x"].iloc[0], df["p_k_y"].iloc[0], color=ACTUAL_COLOR, s=40, zorder=5, marker="o")
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    ax.set_title("XY plane: reference vs. actual")
    ax.set_aspect("equal", adjustable="datalim")
    ax.grid(True, color=GRID_COLOR, alpha=0.3)
    ax.legend(frameon=False)

    # --- x, y vs time ---
    ax = axes[0, 1]
    ax.plot(df["t_s"], df["p_ref_k_x"], color=REF_COLOR, lw=1.5, label="ref x")
    ax.plot(df["t_s"], df["p_k_x"], color=ACTUAL_COLOR, lw=1.5, label="actual x")
    ax.plot(df["t_s"], df["p_ref_k_y"], color=REF_COLOR, lw=1.5, ls="--", label="ref y")
    ax.plot(df["t_s"], df["p_k_y"], color=ACTUAL_COLOR, lw=1.5, ls="--", label="actual y")
    ax.set_xlabel("t (s)")
    ax.set_ylabel("position (m)")
    ax.set_title("x, y vs. time")
    ax.grid(True, color=GRID_COLOR, alpha=0.3)
    ax.legend(frameon=False, fontsize=8, ncol=2)

    # --- altitude vs time ---
    ax = axes[0, 2]
    ax.plot(df["t_s"], df["p_ref_k_z"], color=REF_COLOR, lw=1.5, label="reference")
    ax.plot(df["t_s"], df["p_k_z"], color=ACTUAL_COLOR, lw=1.5, label="actual")
    ax.set_xlabel("t (s)")
    ax.set_ylabel("z (m)")
    ax.set_title("Altitude hold vs. time")
    ax.grid(True, color=GRID_COLOR, alpha=0.3)
    ax.legend(frameon=False)

    # --- XY tracking error norm vs time (the wind's visible effect) ---
    ax = axes[1, 0]
    ax.plot(df["t_s"], err_xy, color=ACTUAL_COLOR, lw=1.5)
    ax.axhline(err_xy.mean(), color=GRID_COLOR, lw=1, ls=":", label=f"mean {err_xy.mean():.3f} m")
    ax.set_xlabel("t (s)")
    ax.set_ylabel("|p_actual - p_ref| (m)")
    ax.set_title("XY tracking error under wind")
    ax.grid(True, color=GRID_COLOR, alpha=0.3)
    ax.legend(frameon=False)

    # --- true residual norm |d_k| vs time ---
    ax = axes[1, 1]
    if correction is not None:
        t_s, baseline_norm, _, _ = correction
        ax.plot(t_s, baseline_norm, color=BASELINE_COLOR, lw=0.6, alpha=0.5)
        ax.plot(t_s, smooth(baseline_norm, SMOOTH_WINDOW), color=BASELINE_COLOR, lw=2,
                label=f"mean {baseline_norm.mean():.4f}")
        ax.set_ylabel("|d_k| (world frame, 9-D)")
        ax.legend(frameon=False)
    else:
        ax.text(0.5, 0.5, "residuals.py / torch unavailable", ha="center", va="center", transform=ax.transAxes)
    ax.set_xlabel("t (s)")
    ax.set_title("True residual norm (nominal-model mismatch)")
    ax.grid(True, color=GRID_COLOR, alpha=0.3)

    # --- remaining error after correction: baseline vs ecrl vs ambient ---
    ax = axes[1, 2]
    if correction is not None:
        t_s, baseline_norm, ecrl_err, ambient_err = correction
        for series, color, label in [
            (baseline_norm, BASELINE_COLOR, "baseline (no correction)"),
            (ecrl_err, ECRL_COLOR, "ecrl-corrected"),
            (ambient_err, AMBIENT_COLOR, "ambient-corrected"),
        ]:
            ax.plot(t_s, smooth(series, SMOOTH_WINDOW), color=color, lw=2, label=label)
        ax.set_ylabel("remaining |error| (world frame, 9-D)")
        ax.legend(frameon=False, fontsize=8)
    else:
        ax.text(0.5, 0.5, "residuals.py / torch unavailable", ha="center", va="center", transform=ax.transAxes)
    ax.set_xlabel("t (s)")
    ax.set_title(f"Correction quality (smoothed, window={SMOOTH_WINDOW} rows)")
    ax.grid(True, color=GRID_COLOR, alpha=0.3)

    fig.tight_layout(rect=[0, 0, 1, 0.94])
    fig.savefig(out_path, dpi=150)
    plt.close(fig)
    print(f"Saved {out_path}")
    print(f"  mean XY tracking error: {err_xy.mean():.4f} m (max {err_xy.max():.4f} m)")
    if correction is not None:
        _, baseline_norm, ecrl_err, ambient_err = correction
        print(f"  mean |d_k| (baseline):        {baseline_norm.mean():.5f}")
        print(f"  mean remaining error (ecrl):    {ecrl_err.mean():.5f} "
              f"({100*(1 - ecrl_err.mean()/baseline_norm.mean()):+.1f}% vs baseline)")
        print(f"  mean remaining error (ambient): {ambient_err.mean():.5f} "
              f"({100*(1 - ambient_err.mean()/baseline_norm.mean()):+.1f}% vs baseline)")


def main():
    default_models_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "models")
    parser = argparse.ArgumentParser(description="Plot a collected figure-8-in-wind flight vs. its reference, "
                                      "with the true residual and each model's correction quality")
    parser.add_argument("--csv", required=True, help="Path to a schema_version=2 raw CSV "
                        "(e.g. produced by collect_figure8_windy.py)")
    parser.add_argument("--out", default=None, help="Output PNG path (default: alongside --csv)")
    parser.add_argument("--ecrl_model", default=os.path.join(default_models_dir, "ecrl_model_normalized.pt"))
    parser.add_argument("--ambient_model", default=os.path.join(default_models_dir, "ambient_model_normalized.pt"))
    args = parser.parse_args()

    out_path = args.out or os.path.splitext(args.csv)[0] + "_trajectory.png"
    df = load_tracking_segment(args.csv)
    plot(df, args.csv, out_path, args.ecrl_model, args.ambient_model)


if __name__ == "__main__":
    main()
