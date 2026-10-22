#!/usr/bin/env python3
"""
train_residuals_v2.py — Diversity-Aware Residual Training for ECRL

CLI entry point / orchestrator only -- the implementation is split across:
  residual_data.py     -- loading, filtering, diversity sampling, splitting
  residual_model.py    -- model architecture + training loop
  residual_evaluate.py -- test-set evaluation, metadata, export, plots
This file wires them together via argparse and re-exports ResidualMLP/
NormalizedResidualMLP/load_csv_files at module level so existing importers
(scripts/knode_training/train_knode.py, scripts/sample_efficiency_sweep.py)
keep working unmodified against `from train_residuals_v2 import ...`.

Strategy to avoid overfitting to specific wind parameters:
  1. Mix both windy and non-windy data so the residual learns
     *general* model mismatch, not just wind compensation.
  2. Subsample trajectory files (not all 300+300) to limit data volume.
  3. Filter out near-zero residual rows (hovering/stationary) that
     add bulk without information.
  4. Stratified spatial binning over key input dimensions (adapter-dependent
     -- see key_dims below) so every region of the dynamics is represented
     roughly equally.
  5. Strong regularization: weight decay, early stopping, gradient
     clipping, and moderate network capacity.

Data format (see docs/residual_conventions.md and scripts/residuals.py):
  Trains on one of residuals.py's two adapters, selected via --adapter:
    ecrl (default): X is 16-D [controller error state dx (9) ; control error
       du (4) ; Log(R_ref_k) r_ref (3)] -- same layout as the pre-refactor
       dataset, so the feature semantics below (dx_0..dx_5, du_0) still hold.
    ambient: X is 8-D [body-frame velocity (3) ; body-frame measured rate (3) ;
       thrust_norm_cmd (1) ; thr2acc (1)] -- no reference trajectory at all.
  Both adapters regress onto the exact same Y: d_k, the residual against the
  exact nonlinear one-step model (not the old linearized-model residual), in
  world frame -- the model predicts directly in the frame it will be used in
  (e.g. once wired into the MPC as an affine QP offset), no further transform
  needed. (An earlier version of the ecrl adapter reanchored Y's attitude
  block onto R_ref_k; that made the target dominated by reference-tracking
  error rather than the true residual whenever tracking error was
  non-negligible -- e.g. under wind -- and measurably made trained models
  worse than a zero-residual baseline both in- and out-of-distribution. See
  residuals.py::ecrl_features_and_target and docs/residual_conventions.md
  Sec.4.) Do NOT point --calm_dir/--windy_dir at old (schema_version=1)
  datasets; load_csv_files() below will reject them.
"""

import os
import sys
import argparse
import numpy as np
import torch

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from residual_data import (  # noqa: E402
    load_csv_files, filter_trivial_residuals, stratified_bin_sampling,
    normalize, split_train_val_test,
)
from residual_model import ResidualMLP, NormalizedResidualMLP, train_model  # noqa: E402
from residual_evaluate import (  # noqa: E402
    evaluate_test_set, save_metadata, export_models, plot_training_curves,
)

__all__ = ["ResidualMLP", "NormalizedResidualMLP", "load_csv_files"]


def main():
    parser = argparse.ArgumentParser(
        description="Train ECRL residual with diversity-aware sample selection")

    parser.add_argument("--calm_dir", type=str,
                        default=os.path.expanduser("~/ecrl_datasets_v2"),
                        help="Non-windy (calm) dataset directory (raw schema_version=2 CSVs)")
    parser.add_argument("--windy_dir", type=str,
                        default=os.path.expanduser("~/ecrl_datasets_v2/windy"),
                        help="Windy dataset directory (raw schema_version=2 CSVs)")
    parser.add_argument("--output_dir", type=str,
                        default=os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "models"),
                        help="Directory to save output models")
    parser.add_argument("--adapter", type=str, default="ecrl", choices=["ecrl", "ambient"],
                        help="Feature adapter: 'ecrl' (error-state, reference-relative) or "
                             "'ambient' (body-frame velocity/rate/thrust, no reference). "
                             "Both regress onto the same world-frame d_k target.")

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
    parser.add_argument("--lipschitz_lambda", type=float, default=0.0,
                        help="Weight of the prod_i ||W_i||_2 Lipschitz penalty (0 = off). When > 0 "
                             "the model is built WITHOUT spectral_norm (it would make the penalty a no-op).")
    parser.add_argument("--lipschitz_max", type=float, default=None,
                        help="If set, penalize only max(0, prod - lipschitz_max)^2 (hinge); "
                             "otherwise penalize prod^2.")
    parser.add_argument("--loss_weights", type=float, nargs=9, default=None,
                        help="Per-output (p_x,p_y,p_z,v_x,v_y,v_z,th_x,th_y,th_z) weights on the "
                             "normalized squared error, e.g. 0.1 0.1 0.1 1 1 1 1 1 1 to down-weight "
                             "the noise-floor position residual. Default: uniform (plain MSE).")
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

    print(f"  Adapter: {args.adapter}")
    X_calm, y_calm = load_csv_files(args.calm_dir,
                                     max_files=args.calm_files, rng=rng, adapter=args.adapter)
    print(f"\n  Calm pool:  {X_calm.shape[0]:>8d} samples")

    # windy_files=0 means "no windy pool this round" (e.g. a calm-only
    # collection run) rather than an error -- load_csv_files() would otherwise
    # raise FileNotFoundError trying to glob/subsample down to zero files.
    if args.windy_files > 0:
        X_windy, y_windy = load_csv_files(args.windy_dir,
                                           max_files=args.windy_files, rng=rng, adapter=args.adapter)
        print(f"  Windy pool: {X_windy.shape[0]:>8d} samples")
        X_all = np.vstack([X_calm, X_windy])
        y_all = np.vstack([y_calm, y_windy])
    else:
        print("  Windy pool:        0 samples (skipped, --windy_files 0)")
        X_all, y_all = X_calm, y_calm

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

    # ecrl X layout: dx_0..dx_8 (9) du_0..du_3 (4) r_ref_0..r_ref_2 (3) -- bin on
    #   position/velocity error (dx_0..dx_5) and thrust error (du_0, index 9).
    # ambient X layout: v_body (3) w_body (3) thrust_norm_cmd (1) thr2acc (1) --
    #   bin on velocity/rate (0..5) and thrust command (index 6).
    key_dims = [0, 1, 2, 3, 4, 5, 9] if args.adapter == "ecrl" else [0, 1, 2, 3, 4, 5, 6]
    X_sel, y_sel = stratified_bin_sampling(
        X_filt, y_filt, n_target=args.target_samples, key_dims=key_dims,
        n_bins_per_dim=args.n_bins, rng=rng)

    # ──────────────────────────────────────────────────────────────────
    # 4. Normalize + split (70/15/15)
    # ──────────────────────────────────────────────────────────────────
    X_norm, y_norm, X_mean, X_std, y_mean, y_std = normalize(X_sel, y_sel)
    train_ds, val_ds, test_ds, tr_idx, val_idx, te_idx = split_train_val_test(
        X_norm, y_norm, train_frac=0.70, val_frac=0.15, rng=rng)

    # ──────────────────────────────────────────────────────────────────
    # 5. Build + train model
    # ──────────────────────────────────────────────────────────────────
    input_dim = X_sel.shape[1]
    output_dim = y_sel.shape[1]

    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    print(f"\n  Device: {device}")
    print(f"  Model: MLP {[input_dim] + args.hidden_dims + [output_dim]}")

    model = ResidualMLP(input_dim=input_dim, output_dim=output_dim,
                        hidden_dims=args.hidden_dims,
                        activation=args.activation,
                        use_spectral_norm=(args.lipschitz_lambda == 0.0)).to(device)

    print("\n" + "="*60)
    print("STEP 5: Training")
    print("="*60)

    train_losses, val_losses, best_val, best_model_path = train_model(
        model, train_ds, val_ds, device, args.output_dir, args.adapter,
        epochs=args.epochs, batch_size=args.batch_size, lr=args.lr,
        weight_decay=args.weight_decay, patience=args.patience,
        lipschitz_lambda=args.lipschitz_lambda, lipschitz_max=args.lipschitz_max,
        loss_weights=args.loss_weights)

    # ──────────────────────────────────────────────────────────────────
    # 6. Evaluate on test set (reload best checkpoint -- last epoch isn't
    #    necessarily the best one)
    # ──────────────────────────────────────────────────────────────────
    model.load_state_dict(torch.load(best_model_path, weights_only=True))
    coord_rmse = evaluate_test_set(model, test_ds, device, y_mean, y_std,
                                    batch_size=args.batch_size)

    # ──────────────────────────────────────────────────────────────────
    # 7. Save metadata, export models, plot training curves
    # ──────────────────────────────────────────────────────────────────
    save_metadata(args.output_dir, args.adapter, input_dim, output_dim,
                  X_mean, X_std, y_mean, y_std, coord_rmse, best_val,
                  args.calm_files, args.windy_files, args.target_samples,
                  len(tr_idx), args.residual_filter_pct, args.hidden_dims,
                  args.weight_decay, loss_weights=args.loss_weights,
                  lipschitz_lambda=args.lipschitz_lambda, lipschitz_max=args.lipschitz_max)

    export_models(model, args.output_dir, args.adapter, input_dim,
                 X_mean, X_std, y_mean, y_std)

    plot_training_curves(train_losses, val_losses, args.output_dir, args.adapter)

    print("\n" + "="*60)
    print("DONE — model ready for deployment")
    print("="*60)


if __name__ == "__main__":
    main()
