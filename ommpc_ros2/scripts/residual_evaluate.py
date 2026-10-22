#!/usr/bin/env python3
"""residual_evaluate.py -- test-set evaluation, metadata, and model export
for trained residual models.

Split out of train_residuals_v2.py (which is now the CLI entry point/
orchestrator only).
"""
import json
import os

import matplotlib.pyplot as plt
import numpy as np
import torch
import torch.nn as nn
from torch.nn.utils import remove_spectral_norm
from torch.utils.data import DataLoader

from residual_model import NormalizedResidualMLP

COORD_NAMES = [
    "Position X (res_0)", "Position Y (res_1)", "Position Z (res_2)",
    "Velocity X (res_3)", "Velocity Y (res_4)", "Velocity Z (res_5)",
    "Attitude X (res_6)", "Attitude Y (res_7)", "Attitude Z (res_8)",
]


def evaluate_test_set(model, test_ds, device, y_mean, y_std, batch_size=128):
    """Runs the model over test_ds and returns per-coordinate RMSE in
    physical (de-normalized) units, printed in COORD_NAMES order."""
    test_loader = DataLoader(test_ds, batch_size=batch_size, shuffle=False)
    model.eval()

    all_preds, all_tgts = [], []
    with torch.no_grad():
        for bx, by in test_loader:
            bx = bx.to(device)
            all_preds.append(model(bx).cpu().numpy())
            all_tgts.append(by.numpy())

    preds = np.vstack(all_preds)
    tgts = np.vstack(all_tgts)

    preds_phys = preds * y_std + y_mean
    tgts_phys = tgts * y_std + y_mean

    coord_rmse = np.sqrt(np.mean((preds_phys - tgts_phys) ** 2, axis=0))

    print("\n" + "=" * 60)
    print("TEST SET RMSE (Physical Coordinates)")
    print("=" * 60)
    for name, rmse in zip(COORD_NAMES, coord_rmse):
        print(f"  {name:28s}: {rmse:.6f}")

    pos_rmse_3d = np.sqrt(np.mean(coord_rmse[:3] ** 2))
    print(f"\n  3D Position RMSE: {pos_rmse_3d:.6f} m")

    return coord_rmse


def save_metadata(output_dir, adapter, input_dim, output_dim, X_mean, X_std, y_mean, y_std,
                   coord_rmse, best_val, calm_files, windy_files, target_samples,
                   actual_train_samples, residual_filter_pct, hidden_dims, weight_decay,
                   loss_weights=None, lipschitz_lambda=0.0, lipschitz_max=None):
    metadata = {
        "input_dim": int(input_dim),
        "output_dim": int(output_dim),
        "input_mean": X_mean.tolist(),
        "input_std": X_std.tolist(),
        "output_mean": y_mean.tolist(),
        "output_std": y_std.tolist(),
        "coordinate_names": COORD_NAMES,
        "test_rmse": coord_rmse.tolist(),
        "adapter": adapter,
        "residual_definition": "d_k_v3",  # see docs/residual_conventions.md; output is world-frame d_k
                                           # directly (the ecrl adapter's earlier R_ref_k-reanchored
                                           # attitude target was dropped -- it made tracking error
                                           # dominate d_k). ambient regresses onto the identical target.
        "training_info": {
            "strategy": "diversity-aware: mixed calm+windy, residual filtering, stratified spatial binning",
            "calm_files_used": calm_files,
            "windy_files_used": windy_files,
            "target_samples": target_samples,
            "actual_train_samples": int(actual_train_samples),
            "residual_filter_percentile": residual_filter_pct,
            "hidden_dims": hidden_dims,
            "weight_decay": weight_decay,
            "loss_weights": loss_weights if loss_weights else [1.0] * int(output_dim),
            "lipschitz_lambda": lipschitz_lambda,
            "lipschitz_max": lipschitz_max,
            "best_val_loss": float(best_val),
        }
    }

    meta_path = os.path.join(output_dir, f"{adapter}_metadata.json")
    with open(meta_path, "w") as f:
        json.dump(metadata, f, indent=4)
    print(f"\n  Saved metadata → {meta_path}")
    return meta_path


def export_models(model, output_dir, adapter, input_dim, X_mean, X_std, y_mean, y_std):
    """Removes spectral-norm hooks (needed before scripting/tracing), wraps
    the plain model in NormalizedResidualMLP, and exports both the raw and
    normalized model as ONNX and TorchScript."""
    model.cpu()
    _ = model(torch.zeros(1, input_dim))  # prime spectral norm buffers
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
        onnx_path = os.path.join(output_dir, f"{adapter}_model.onnx")
        torch.onnx.export(model, dummy, onnx_path,
                          export_params=True, opset_version=11,
                          do_constant_folding=True,
                          input_names=["input"], output_names=["output"])
        print(f"  ONNX (unnorm)  → {onnx_path}")
    except Exception as e:
        print(f"  ONNX export failed: {e}")

    # ONNX – normalised
    try:
        onnx_norm = os.path.join(output_dir, f"{adapter}_model_normalized.onnx")
        torch.onnx.export(norm_model, dummy, onnx_norm,
                          export_params=True, opset_version=11,
                          do_constant_folding=True,
                          input_names=["input"], output_names=["output"])
        print(f"  ONNX (norm)    → {onnx_norm}")
    except Exception as e:
        print(f"  ONNX norm export failed: {e}")

    # TorchScript – unnormalised
    try:
        ts_path = os.path.join(output_dir, f"{adapter}_model.pt")
        torch.jit.script(model).save(ts_path)
        print(f"  TorchScript    → {ts_path}")
    except Exception as e:
        print(f"  TorchScript export failed: {e}")

    # TorchScript – normalised
    try:
        ts_norm = os.path.join(output_dir, f"{adapter}_model_normalized.pt")
        torch.jit.script(norm_model).save(ts_norm)
        print(f"  TorchScript (norm) → {ts_norm}")
    except Exception as e:
        print(f"  TorchScript norm export failed: {e}")


def plot_training_curves(train_losses, val_losses, output_dir, adapter):
    plt.figure(figsize=(10, 5))
    plt.plot(train_losses, label="Train Loss")
    plt.plot(val_losses, label="Val Loss")
    plt.xlabel("Epoch")
    plt.ylabel("MSE Loss (Normalised)")
    plt.title("Diversity-Aware Residual Training Curves")
    plt.legend()
    plt.grid(True)
    plot_path = os.path.join(output_dir, f"{adapter}_training_curves.png")
    plt.savefig(plot_path, dpi=150)
    plt.close()
    print(f"  Training curves → {plot_path}")
    return plot_path
