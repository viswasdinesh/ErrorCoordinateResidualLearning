#!/usr/bin/env python3
"""evaluate_figure8_windy.py -- score BOTH trained ECRL residual models
(the `ecrl` and `ambient` adapters) against a single held-out figure-8 raw
CSV collected in the windy eval Gazebo scene, in world frame, against a
zero-residual (no-correction) baseline.

Mirrors evaluate_model.py's methodology exactly (same rmse_report, same
zero-residual-baseline framing) but:
  - loads ONE collected CSV (via residuals.py::load_raw_csv) instead of a
    --data_dir glob of polynomial*.csv files -- this is meant for the single
    fresh figure8-in-wind flight produced by collect_figure8_windy.py, not a
    directory of files,
  - evaluates both adapters in one run and prints them side by side, mirroring
    the "ecrl vs ambient adapter, same data" table in
    docs/wind_generalization_experiment.md.

Reuses residuals.py's load_raw_csv/compute_targets/ecrl_features_and_target/
ambient_features verbatim -- no residual math is reimplemented here.

Usage:
    conda run -n ecrl python3 scripts/evaluate_figure8_windy.py \\
        --csv /home/dinesh/ecrl_datasets_v2_eval_windy/figure8/figure8_windy_run1.csv \\
        --ecrl_model models/ecrl_model_normalized.pt \\
        --ambient_model models/ambient_model_normalized.pt
"""
import argparse
import os
import sys

import numpy as np
import torch

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import residuals as R  # noqa: E402

COORD_NAMES = ["pos_x", "pos_y", "pos_z", "vel_x", "vel_y", "vel_z", "att_x", "att_y", "att_z"]
FEATURE_DESC = {
    "ecrl": "error state rel. to reference (16-D)",
    "ambient": "body-frame vel/rate + thrust (8-D)",
}


def load_single_csv(csv_path: str, adapter: str):
    df = R.load_raw_csv(csv_path)
    df = R.compute_targets(df)
    if adapter == "ecrl":
        X, Y = R.ecrl_features_and_target(df)
    elif adapter == "ambient":
        X = R.ambient_features(df)
        Y = df[[f"d_{i}" for i in range(9)]].values
    else:
        raise ValueError(f"unknown adapter '{adapter}'")

    valid = df["target_valid"].values & ~np.isnan(X).any(axis=1) & ~np.isnan(Y).any(axis=1)
    X, Y = X[valid], Y[valid]
    print(f"  [{adapter}] {len(df)} rows loaded from {os.path.basename(csv_path)}, "
          f"{len(X)} valid rows after target/feature validity filter")
    return X, Y


def rmse_report(err: np.ndarray, label: str):
    per_dim = np.sqrt(np.mean(err ** 2, axis=0))
    overall = np.sqrt(np.mean(np.sum(err ** 2, axis=1)))
    print(f"  [{label}]")
    for name, v in zip(COORD_NAMES, per_dim):
        print(f"    {name:8s}: {v:.5f}")
    print(f"    {'overall':8s}: {overall:.5f} (RMS of full 9-D residual norm)")
    return overall, per_dim


def evaluate_adapter(csv_path: str, model_path: str, adapter: str):
    print(f"\n--- adapter={adapter} (model={model_path}) ---")
    X, Y = load_single_csv(csv_path, adapter)

    model = torch.jit.load(model_path)
    model.eval()
    with torch.no_grad():
        pred = model(torch.tensor(X, dtype=torch.float32)).numpy()

    baseline_err = Y             # "predicting" zero residual: error = true residual itself
    model_err = Y - pred

    base_overall, base_per_dim = rmse_report(baseline_err, f"{adapter}: zero-residual baseline")
    print()
    model_overall, model_per_dim = rmse_report(model_err, f"{adapter}: trained model (world frame)")

    reduction = 100.0 * (1.0 - model_overall / base_overall)
    per_dim_reduction = 100.0 * (1.0 - model_per_dim / base_per_dim)
    print(f"\n  {adapter}: overall residual RMS reduction vs. baseline: {reduction:+.1f}%")

    return {
        "adapter": adapter,
        "n_rows": len(X),
        "baseline_overall": base_overall,
        "model_overall": model_overall,
        "reduction_pct": reduction,
        "per_dim_reduction_pct": per_dim_reduction,
    }


def main():
    default_models_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "models")
    parser = argparse.ArgumentParser(
        description="Evaluate the ecrl and ambient trained models side by side on one held-out "
                    "figure-8-in-wind CSV, vs. a zero-residual baseline")
    parser.add_argument("--csv", required=True, help="Path to a single schema_version=2 raw CSV "
                        "(e.g. produced by collect_figure8_windy.py)")
    parser.add_argument("--ecrl_model", default=os.path.join(default_models_dir, "ecrl_model_normalized.pt"))
    parser.add_argument("--ambient_model", default=os.path.join(default_models_dir, "ambient_model_normalized.pt"))
    args = parser.parse_args()

    print(f"Evaluating held-out figure-8 (windy eval scene) data: {args.csv}")

    results = [
        evaluate_adapter(args.csv, args.ecrl_model, "ecrl"),
        evaluate_adapter(args.csv, args.ambient_model, "ambient"),
    ]

    print("\n" + "=" * 78)
    print("ecrl vs ambient adapter -- figure-8, windy eval scene (held-out)")
    print(f"data: {args.csv}")
    print("=" * 78)
    print(f"{'Adapter':10s} {'Features':38s} {'Rows':>7s} {'RMS reduction vs baseline':>26s}")
    for r in results:
        print(f"{r['adapter']:10s} {FEATURE_DESC[r['adapter']]:38s} {r['n_rows']:7d} {r['reduction_pct']:+25.1f}%")

    print("\nPer-axis RMS reduction vs. zero-residual baseline:")
    print(f"{'axis':10s} {'ecrl':>10s} {'ambient':>10s}")
    for i, name in enumerate(COORD_NAMES):
        print(f"{name:10s} {results[0]['per_dim_reduction_pct'][i]:+9.1f}% "
              f"{results[1]['per_dim_reduction_pct'][i]:+9.1f}%")

    print("\n(positive = model's correction reduces the residual, i.e. it generalizes here;")
    print(" negative = model makes things WORSE than not correcting at all on this data)")


if __name__ == "__main__":
    main()
