#!/usr/bin/env python3
"""evaluate_model.py -- score a trained ECRL residual model against
held-out data, in world frame, against a zero-residual (no-correction)
baseline.

Usage:
    conda run -n ecrl python3 scripts/evaluate_model.py \
        --model models/ecrl_model_normalized.pt --adapter ecrl \
        --data_dir /path/to/held_out_csvs

The model is expected to be the *normalized* TorchScript export from
train_residuals_v2.py (raw adapter X in, raw world-frame d_k Y out --
normalization is baked into the exported module). --adapter must match
whatever the model was trained with (train_residuals_v2.py --adapter), since
that determines the feature layout X was built with. No further transform of
the output is needed before scoring -- see docs/residual_conventions.md.
"""
import argparse
import glob
import os
import sys

import numpy as np
import torch

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import residuals as R  # noqa: E402


def load_dataset(data_dir: str, adapter: str = "ecrl"):
    files = sorted(glob.glob(os.path.join(data_dir, "polynomial*.csv")))
    if not files:
        raise FileNotFoundError(f"No polynomial*.csv files in {data_dir}")

    X_list, Y_list = [], []
    for f in files:
        df = R.load_raw_csv(f)
        df = R.compute_targets(df)
        if adapter == "ecrl":
            X, Y = R.ecrl_features_and_target(df)
        elif adapter == "ambient":
            X = R.ambient_features(df)
            Y = df[[f"d_{i}" for i in range(9)]].values
        else:
            raise ValueError(f"unknown adapter '{adapter}'")
        valid = df["target_valid"].values & ~np.isnan(X).any(axis=1) & ~np.isnan(Y).any(axis=1)

        X_list.append(X[valid])
        Y_list.append(Y[valid])

    X = np.vstack(X_list)
    Y = np.vstack(Y_list)
    print(f"  Loaded {len(files)} files, {len(X)} valid rows")
    return X, Y


def rmse_report(err: np.ndarray, label: str):
    coord_names = ["pos_x", "pos_y", "pos_z", "vel_x", "vel_y", "vel_z", "att_x", "att_y", "att_z"]
    per_dim = np.sqrt(np.mean(err ** 2, axis=0))
    overall = np.sqrt(np.mean(np.sum(err ** 2, axis=1)))
    print(f"  [{label}]")
    for name, v in zip(coord_names, per_dim):
        print(f"    {name:8s}: {v:.5f}")
    print(f"    {'overall':8s}: {overall:.5f} (RMS of full 9-D residual norm)")
    return overall, per_dim


def main():
    parser = argparse.ArgumentParser(description="Evaluate a trained ECRL model vs. zero-residual baseline")
    parser.add_argument("--model", required=True, help="Path to normalized TorchScript model (.pt)")
    parser.add_argument("--data_dir", required=True, help="Directory of held-out raw CSVs to evaluate on")
    parser.add_argument("--adapter", type=str, default="ecrl", choices=["ecrl", "ambient"],
                        help="Must match the adapter the model was trained with")
    args = parser.parse_args()

    print(f"Loading held-out data from {args.data_dir}")
    X, d_world_true = load_dataset(args.data_dir, adapter=args.adapter)

    print(f"Loading model from {args.model}")
    model = torch.jit.load(args.model)
    model.eval()
    with torch.no_grad():
        pred_world = model(torch.tensor(X, dtype=torch.float32)).numpy()

    baseline_err = d_world_true  # "predicting" zero residual: error = true residual itself
    model_err = d_world_true - pred_world

    print("\n" + "=" * 60)
    print(f"Evaluation on {args.data_dir}")
    print("=" * 60)
    base_overall, _ = rmse_report(baseline_err, "zero-residual baseline (no correction)")
    print()
    model_overall, _ = rmse_report(model_err, "trained model (world frame)")

    reduction = 100.0 * (1.0 - model_overall / base_overall)
    print(f"\n  Overall residual RMS reduction vs. baseline: {reduction:+.1f}%")
    if reduction > 0:
        print("  (positive = model's correction reduces the residual, i.e. it generalizes here)")
    else:
        print("  (negative = model makes things WORSE than not correcting at all on this data)")


if __name__ == "__main__":
    main()
