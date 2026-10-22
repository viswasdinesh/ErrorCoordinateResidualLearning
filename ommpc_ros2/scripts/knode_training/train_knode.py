#!/usr/bin/env python3
"""train_knode.py -- KNODE-MPC-style multi-step rollout training for the
ECRL residual model, in this new scripts/knode_training/ pipeline (NOT
touching ../train_residuals_v2.py or the production models/*.pt).

Where ../train_residuals_v2.py trains on isolated one-step (X_row, d_row)
pairs, this script rolls the combined (physics + NN) model forward over a
window of `--rollout_steps` consecutive, contiguous log rows -- driven by
the REAL flight's logged applied controls (open-loop replay, not
re-optimized) and REAL logged reference trajectory -- and backprops through
the composed multi-step prediction against the real measured trajectory.
This directly scores what the live controller actually does with the model
(fold its prediction into the dynamics recursively, every 100Hz cycle, over
a horizon) instead of only ever grading it on isolated one-step deltas.

Reuses (does not duplicate):
  - ../residuals.py: raw CSV loading, compute_targets(), and (purely for
    computing a representative normalization mean/std -- see
    _compute_norm_stats below) its existing one-step feature builders.
  - ../train_residuals_v2.py: ResidualMLP / NormalizedResidualMLP model
    classes and load_csv_files() (again, only for norm stats) -- so a
    resulting model is drop-in export-compatible with
    ../export_residual_weights.py without any format changes.
  - so3_torch.py / windows.py (this directory): the batched/differentiable
    physics core and the contiguous-window builder.

Output format (metadata.json, {adapter}_model[_normalized].pt/.onnx) is
identical to train_residuals_v2.py's, plus a `training_info.knode` block
recording the rollout-specific hyperparameters -- so
export_residual_weights.py works unmodified on the result, IF a resulting
model is later promoted (out of scope for this script itself -- see the
plan's explicit non-goals).
"""
import argparse
import glob
import json
import os
import sys

import numpy as np
import torch
import torch.nn as nn
import torch.optim as optim
from torch.nn.utils import remove_spectral_norm
from torch.utils.data import DataLoader
import matplotlib.pyplot as plt

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
import residuals as R  # noqa: E402
import so3_torch as T  # noqa: E402
from windows import build_rollout_windows, RolloutWindowDataset  # noqa: E402
from train_residuals_v2 import ResidualMLP, NormalizedResidualMLP, load_csv_files  # noqa: E402
from residual_model import lipschitz_penalty, spectral_norm_product  # noqa: E402


# ─── Data loading (windows) ────────────────────────────────────────────
def load_windows_from_dir(dataset_dir, max_files, rollout_steps, stride, rng):
    csv_pattern = os.path.join(dataset_dir, "polynomial*.csv")
    all_files = sorted(glob.glob(csv_pattern))
    all_files = [f for f in all_files if "polynomial300" not in os.path.basename(f)]
    if not all_files:
        raise FileNotFoundError(f"No polynomial CSV files in {dataset_dir}")
    if max_files is not None and max_files < len(all_files):
        chosen = rng.choice(len(all_files), max_files, replace=False)
        all_files = [all_files[i] for i in sorted(chosen)]

    print(f"  Loading {len(all_files)} files from {dataset_dir}")
    all_windows = []
    for f in all_files:
        try:
            df = R.load_raw_csv(f)
        except ValueError as e:
            print(f"  SKIP {f}: {e}")
            continue
        if len(df) < rollout_steps:
            continue
        df = R.compute_targets(df)
        wins = build_rollout_windows(df, rollout_steps, stride)
        all_windows.extend(wins)
    print(f"  -> {len(all_windows)} rollout windows (rollout_steps={rollout_steps}, stride={stride})")
    return all_windows


def _compute_norm_stats(args, rng, adapter):
    """Normalization statistics are computed from the SAME existing one-step
    feature/target pipeline train_residuals_v2.py uses (residuals.py's
    ecrl_features_and_target/ambient_features via load_csv_files) rather
    than from the rollout windows themselves: at t=0 a window's rollout
    state exactly equals the logged state by construction, and for t>0 it
    stays close (that's the whole premise of a residual-correction model),
    so the one-step feature distribution is a representative and much
    simpler-to-compute stand-in. This does NOT affect what's trained on --
    only the fixed normalization constants baked into the exported model."""
    X_calm, y_calm = load_csv_files(args.calm_dir, max_files=args.calm_files, rng=rng, adapter=adapter)
    if args.windy_files > 0:
        X_windy, y_windy = load_csv_files(args.windy_dir, max_files=args.windy_files, rng=rng, adapter=adapter)
        X_all = np.vstack([X_calm, X_windy])
        y_all = np.vstack([y_calm, y_windy])
    else:
        X_all, y_all = X_calm, y_calm

    x_mean = np.mean(X_all, axis=0)
    x_std = np.std(X_all, axis=0)
    x_std[x_std < 1e-8] = 1.0
    y_mean = np.mean(y_all, axis=0)
    y_std = np.std(y_all, axis=0)
    y_std[y_std < 1e-8] = 1.0
    return x_mean, x_std, y_mean, y_std, X_all.shape[1], y_all.shape[1]


# ─── Rollout loss ───────────────────────────────────────────────────────
def build_features(adapter, p_hat, v_hat, q_hat, batch, t):
    p_ref_t = batch["p_ref"][:, t]
    v_ref_t = batch["v_ref"][:, t]
    q_ref_t = batch["q_ref"][:, t]
    thracc_ref_t = batch["thracc_ref"][:, t]
    w_ref_t = batch["w_ref"][:, t]
    thracc_cmd_t = batch["thracc_cmd"][:, t]
    w_cmd_t = batch["w_cmd"][:, t]

    if adapter == "ecrl":
        dx = T.controller_error_state(p_ref_t, v_ref_t, q_ref_t, p_hat, v_hat, q_hat)
        du = torch.cat([(thracc_ref_t - thracc_cmd_t).unsqueeze(-1), w_ref_t - w_cmd_t], dim=-1)
        r_ref = T.so3_log(T.quat_to_rot(q_ref_t))
        return torch.cat([dx, du, r_ref], dim=-1)
    elif adapter == "ambient":
        R_hat = T.quat_to_rot(q_hat)
        v_body = torch.matmul(R_hat.transpose(-2, -1), v_hat.unsqueeze(-1)).squeeze(-1)
        w_body = batch["wm"][:, t]
        thrust_norm_cmd_t = batch["thrust_norm_cmd"][:, t].unsqueeze(-1)
        thr2acc_t = batch["thr2acc"][:, t].unsqueeze(-1)
        return torch.cat([v_body, w_body, thrust_norm_cmd_t, thr2acc_t], dim=-1)
    else:
        raise ValueError(f"unknown adapter '{adapter}'")


def rollout_loss(model, batch, adapter, x_mean, x_std, y_mean, y_std, loss_weights,
                  one_step_weight, rollout_steps, return_final_err=False):
    """Rolls the model forward `rollout_steps` steps starting from batch's
    x_0, driven by the logged applied controls, scored against the logged
    measured states at every step. `one_step_weight` in [0,1]: per-step,
    per-batch-element probability of resetting the rollout state back to
    the true measured state before that step (scheduled-sampling-style
    interpolation between pure free-running rollout (0.0, the KNODE-style
    default) and something close to train_residuals_v2.py's original
    one-step training (1.0))."""
    device = next(model.parameters()).device
    p_hat, v_hat, q_hat = batch["p0"], batch["v0"], batch["q0"]

    total = p_hat.new_zeros(())
    final_err = None
    for t in range(rollout_steps):
        if t > 0 and one_step_weight > 0.0:
            p_true_prev = batch["p_kp1"][:, t - 1]
            v_true_prev = batch["v_kp1"][:, t - 1]
            q_true_prev = batch["q_kp1"][:, t - 1]
            reset = (torch.rand(p_hat.shape[0], device=device) < one_step_weight)
            reset_p = reset.unsqueeze(-1)
            p_hat = torch.where(reset_p, p_true_prev, p_hat)
            v_hat = torch.where(reset_p, v_true_prev, v_hat)
            q_hat = torch.where(reset_p, q_true_prev, q_hat)
            q_hat = q_hat / q_hat.norm(dim=-1, keepdim=True)

        feat = build_features(adapter, p_hat, v_hat, q_hat, batch, t)
        feat_norm = (feat - x_mean) / x_std
        d_pred = model(feat_norm) * y_std + y_mean

        thracc_cmd_t = batch["thracc_cmd"][:, t]
        w_cmd_t = batch["w_cmd"][:, t]
        dt_t = batch["dt"][:, t]
        nom_p, nom_v, nom_q = T.nominal_step(p_hat, v_hat, q_hat, thracc_cmd_t, w_cmd_t, dt_t)
        p_hat, v_hat, q_hat = T.compose_from_residual(nom_p, nom_v, nom_q, d_pred)

        p_meas_t = batch["p_kp1"][:, t]
        v_meas_t = batch["v_kp1"][:, t]
        q_meas_t = batch["q_kp1"][:, t]
        err = T.boxminus(p_meas_t, v_meas_t, q_meas_t, p_hat, v_hat, q_hat)
        total = total + (loss_weights * err ** 2).sum(dim=-1).mean()
        if t == rollout_steps - 1:
            final_err = err

    total = total / rollout_steps
    if return_final_err:
        return total, final_err
    return total


# ─── Main ───────────────────────────────────────────────────────────────
def main():
    parser = argparse.ArgumentParser(description="KNODE-MPC-style multi-step rollout training for the ECRL residual")

    parser.add_argument("--calm_dir", type=str, default=os.path.expanduser("~/ecrl_datasets_v2"))
    parser.add_argument("--windy_dir", type=str, default=os.path.expanduser("~/ecrl_datasets_v2/windy"))
    parser.add_argument("--output_dir", type=str,
                         default=os.path.join(os.path.dirname(os.path.abspath(__file__)), "output"),
                         help="Scratch output directory -- NOT ommpc_ros2/models/. Promoting a resulting "
                              "model into production is a separate, later decision.")
    parser.add_argument("--adapter", type=str, default="ecrl", choices=["ecrl", "ambient"])

    parser.add_argument("--calm_files", type=int, default=30)
    parser.add_argument("--windy_files", type=int, default=40)

    parser.add_argument("--rollout_steps", type=int, default=10)
    parser.add_argument("--stride", type=int, default=None,
                         help="Window stride, rows. Default rollout_steps // 2 (50%% overlap).")
    parser.add_argument("--one_step_weight", type=float, default=0.0,
                         help="0.0 = pure free-running rollout (KNODE-style default). 1.0 = reset to "
                              "ground truth every step (close to train_residuals_v2.py's original "
                              "one-step training). See rollout_loss()'s docstring.")
    parser.add_argument("--loss_weights", type=float, nargs=9, default=None,
                         help="Per-coordinate (p_x,p_y,p_z,v_x,v_y,v_z,th_x,th_y,th_z) loss weights. "
                              "Default: uniform.")

    parser.add_argument("--epochs", type=int, default=80)
    parser.add_argument("--batch_size", type=int, default=64,
                         help="Windows per batch (each window unrolls rollout_steps steps of BPTT, so "
                              "this is smaller than train_residuals_v2.py's row-batch-size by design).")
    parser.add_argument("--lr", type=float, default=5e-4)
    parser.add_argument("--weight_decay", type=float, default=1e-4)
    parser.add_argument("--patience", type=int, default=12)
    parser.add_argument("--hidden_dims", type=int, nargs="+", default=[128, 128])
    parser.add_argument("--activation", type=str, default="tanh", choices=["relu", "tanh", "elu"])
    parser.add_argument("--lipschitz_lambda", type=float, default=0.0,
                         help="Weight of the prod_i ||W_i||_2 Lipschitz penalty (0 = off). When > 0 "
                              "the model is built WITHOUT spectral_norm (it would make the penalty a no-op).")
    parser.add_argument("--lipschitz_max", type=float, default=None,
                         help="If set, penalize only max(0, prod - lipschitz_max)^2 (hinge); "
                              "otherwise penalize prod^2.")
    parser.add_argument("--seed", type=int, default=42)

    args = parser.parse_args()
    if args.stride is None:
        args.stride = max(1, args.rollout_steps // 2)
    os.makedirs(args.output_dir, exist_ok=True)
    rng = np.random.default_rng(args.seed)
    torch.manual_seed(args.seed)

    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    print(f"Device: {device}")

    # ── 1. Normalization stats (existing one-step pipeline, see docstring) ──
    print("=" * 60)
    print("STEP 1: Normalization statistics (one-step feature distribution)")
    print("=" * 60)
    x_mean_np, x_std_np, y_mean_np, y_std_np, input_dim, output_dim = _compute_norm_stats(args, rng, args.adapter)
    x_mean = torch.tensor(x_mean_np, dtype=torch.float32, device=device)
    x_std = torch.tensor(x_std_np, dtype=torch.float32, device=device)
    y_mean = torch.tensor(y_mean_np, dtype=torch.float32, device=device)
    y_std = torch.tensor(y_std_np, dtype=torch.float32, device=device)
    print(f"  input_dim={input_dim} output_dim={output_dim}")

    # ── 2. Rollout windows ──
    print("\n" + "=" * 60)
    print("STEP 2: Building rollout windows")
    print("=" * 60)
    windows = load_windows_from_dir(args.calm_dir, args.calm_files, args.rollout_steps, args.stride, rng)
    if args.windy_files > 0:
        windows += load_windows_from_dir(args.windy_dir, args.windy_files, args.rollout_steps, args.stride, rng)
    if not windows:
        raise ValueError("No rollout windows built -- check --calm_dir/--windy_dir and --rollout_steps.")

    idx = np.arange(len(windows))
    rng.shuffle(idx)
    tr_end = int(0.70 * len(idx))
    val_end = int(0.85 * len(idx))
    tr_windows = [windows[i] for i in idx[:tr_end]]
    val_windows = [windows[i] for i in idx[tr_end:val_end]]
    te_windows = [windows[i] for i in idx[val_end:]]
    print(f"  Train: {len(tr_windows)}  Val: {len(val_windows)}  Test: {len(te_windows)}")

    train_loader = DataLoader(RolloutWindowDataset(tr_windows), batch_size=args.batch_size, shuffle=True)
    val_loader = DataLoader(RolloutWindowDataset(val_windows), batch_size=args.batch_size, shuffle=False)
    test_loader = DataLoader(RolloutWindowDataset(te_windows), batch_size=args.batch_size, shuffle=False)

    # ── 3. Model ──
    print("\n" + "=" * 60)
    print("STEP 3: Model")
    print("=" * 60)
    print(f"  MLP {[input_dim] + args.hidden_dims + [output_dim]}, rollout_steps={args.rollout_steps}, "
          f"one_step_weight={args.one_step_weight}")
    model = ResidualMLP(input_dim=input_dim, output_dim=output_dim,
                         hidden_dims=args.hidden_dims, activation=args.activation,
                         use_spectral_norm=(args.lipschitz_lambda == 0.0)).to(device)

    loss_weights = torch.tensor(args.loss_weights if args.loss_weights else [1.0] * 9,
                                 dtype=torch.float32, device=device)

    optimizer = optim.AdamW(model.parameters(), lr=args.lr, weight_decay=args.weight_decay)
    scheduler = optim.lr_scheduler.OneCycleLR(optimizer, max_lr=args.lr,
                                               steps_per_epoch=max(1, len(train_loader)),
                                               epochs=args.epochs, pct_start=0.15)

    # ── 4. Training loop ──
    print("\n" + "=" * 60)
    print("STEP 4: Training")
    print("=" * 60)
    best_val = float("inf")
    wait = 0
    train_losses, val_losses = [], []
    best_path = os.path.join(args.output_dir, f"{args.adapter}_knode_best_model.pt")

    for epoch in range(1, args.epochs + 1):
        model.train()
        epoch_loss = 0.0
        n_batches = 0
        for batch in train_loader:
            batch = {k: v.to(device) for k, v in batch.items()}
            optimizer.zero_grad()
            loss = rollout_loss(model, batch, args.adapter, x_mean, x_std, y_mean, y_std,
                                 loss_weights, args.one_step_weight, args.rollout_steps)
            penalty, _ = lipschitz_penalty(model, args.lipschitz_lambda, args.lipschitz_max)
            (loss + penalty).backward()
            torch.nn.utils.clip_grad_norm_(model.parameters(), max_norm=1.0)
            optimizer.step()
            scheduler.step()
            epoch_loss += loss.item()
            n_batches += 1
        epoch_loss /= max(1, n_batches)
        train_losses.append(epoch_loss)

        model.eval()
        vloss = 0.0
        n_vbatches = 0
        with torch.no_grad():
            for batch in val_loader:
                batch = {k: v.to(device) for k, v in batch.items()}
                loss = rollout_loss(model, batch, args.adapter, x_mean, x_std, y_mean, y_std,
                                     loss_weights, one_step_weight=0.0, rollout_steps=args.rollout_steps)
                vloss += loss.item()
                n_vbatches += 1
        vloss /= max(1, n_vbatches)
        val_losses.append(vloss)

        if epoch % 5 == 0 or epoch == 1:
            with torch.no_grad():
                lip = spectral_norm_product(model).item()
            print(f"  Epoch {epoch:3d}/{args.epochs}  Train: {epoch_loss:.6f}  Val (pure rollout): {vloss:.6f}  "
                  f"prod||W||_2: {lip:.3f}")

        if vloss < best_val:
            best_val = vloss
            wait = 0
            torch.save(model.state_dict(), best_path)
        else:
            wait += 1
            if wait >= args.patience:
                print(f"\n  Early stopping at epoch {epoch} (best val = {best_val:.6f})")
                break

    print(f"\n  Training finished. Best Val Loss: {best_val:.6f}")

    # ── 5. Test-set rollout evaluation ──
    model.load_state_dict(torch.load(best_path, weights_only=True))
    model.eval()

    coord_names = ["Position X", "Position Y", "Position Z", "Velocity X", "Velocity Y", "Velocity Z",
                   "Attitude X", "Attitude Y", "Attitude Z"]
    final_errs = []
    with torch.no_grad():
        for batch in test_loader:
            batch = {k: v.to(device) for k, v in batch.items()}
            _, final_err = rollout_loss(model, batch, args.adapter, x_mean, x_std, y_mean, y_std,
                                         loss_weights, one_step_weight=0.0, rollout_steps=args.rollout_steps,
                                         return_final_err=True)
            final_errs.append(final_err.cpu().numpy())
    final_errs = np.vstack(final_errs)
    final_rmse = np.sqrt(np.mean(final_errs ** 2, axis=0))

    print("\n" + "=" * 60)
    print(f"TEST SET: RMSE at final rollout step (step {args.rollout_steps}, physical coordinates)")
    print("=" * 60)
    for name, rmse in zip(coord_names, final_rmse):
        print(f"  {name:14s}: {rmse:.6f}")

    # ── 6. Metadata + export (same format as train_residuals_v2.py) ──
    metadata = {
        "input_dim": int(input_dim),
        "output_dim": int(output_dim),
        "input_mean": x_mean_np.tolist(),
        "input_std": x_std_np.tolist(),
        "output_mean": y_mean_np.tolist(),
        "output_std": y_std_np.tolist(),
        "coordinate_names": coord_names,
        "adapter": args.adapter,
        "residual_definition": "d_k_v3",
        "training_info": {
            "strategy": "knode_rollout",
            "calm_files_used": args.calm_files,
            "windy_files_used": args.windy_files,
            "hidden_dims": args.hidden_dims,
            "weight_decay": args.weight_decay,
            "lipschitz_lambda": args.lipschitz_lambda,
            "lipschitz_max": args.lipschitz_max,
            "best_val_loss": float(best_val),
            "knode": {
                "rollout_steps": args.rollout_steps,
                "stride": args.stride,
                "one_step_weight": args.one_step_weight,
                "loss_weights": args.loss_weights if args.loss_weights else [1.0] * 9,
                "final_step_test_rmse": final_rmse.tolist(),
                "n_train_windows": len(tr_windows),
                "n_val_windows": len(val_windows),
                "n_test_windows": len(te_windows),
            },
        },
    }
    with open(os.path.join(args.output_dir, f"{args.adapter}_knode_metadata.json"), "w") as f:
        json.dump(metadata, f, indent=4)

    model.cpu()
    _ = model(torch.zeros(1, input_dim))
    for m in model.modules():
        if isinstance(m, nn.Linear):
            try:
                remove_spectral_norm(m)
            except ValueError:
                pass

    norm_model = NormalizedResidualMLP(model, x_mean_np, x_std_np, y_mean_np, y_std_np)
    dummy = torch.randn(1, input_dim)
    ts_path = os.path.join(args.output_dir, f"{args.adapter}_knode_model.pt")
    torch.jit.script(model).save(ts_path)
    ts_norm_path = os.path.join(args.output_dir, f"{args.adapter}_knode_model_normalized.pt")
    torch.jit.script(norm_model).save(ts_norm_path)
    print(f"\n  TorchScript -> {ts_path}")
    print(f"  TorchScript (normalized) -> {ts_norm_path}")

    plt.figure(figsize=(10, 5))
    plt.plot(train_losses, label="Train (rollout, one_step_weight={:.2f})".format(args.one_step_weight))
    plt.plot(val_losses, label="Val (pure rollout)")
    plt.xlabel("Epoch")
    plt.ylabel("Mean per-step weighted MSE (normalized-scale residual, physical-scale error)")
    plt.title(f"KNODE rollout training curves ({args.adapter}, rollout_steps={args.rollout_steps})")
    plt.legend()
    plt.grid(True)
    plot_path = os.path.join(args.output_dir, f"{args.adapter}_knode_training_curves.png")
    plt.savefig(plot_path, dpi=150)
    plt.close()
    print(f"  Training curves -> {plot_path}")

    print("\nDONE.")


if __name__ == "__main__":
    main()
