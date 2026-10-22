#!/usr/bin/env python3
"""residual_model.py -- model architecture and training loop for residual
dynamics learning.

Split out of train_residuals_v2.py (which is now the CLI entry point/
orchestrator only). ResidualMLP/NormalizedResidualMLP are also imported
directly by scripts/knode_training/train_knode.py and
scripts/sample_efficiency_sweep.py (via train_residuals_v2.py's
re-exports -- see that file), so their behavior/signatures must stay
unchanged.
"""
import os

import torch
import torch.nn as nn
import torch.optim as optim
from torch.nn.utils import spectral_norm
from torch.utils.data import DataLoader


# ─── Model ──────────────────────────────────────────────────────────────
class ResidualMLP(nn.Module):
    def __init__(self, input_dim=16, output_dim=9,
                 hidden_dims=[128, 128, 128], activation='tanh', use_spectral_norm=True):
        # use_spectral_norm=False gives plain nn.Linear layers -- required when
        # training with lipschitz_penalty() below, since spectral_norm() already
        # rescales every ||W_i||_2 to ~1 and would make that penalty a no-op.
        super().__init__()
        layers = []
        prev_dim = input_dim
        act_fn = {'relu': nn.ReLU, 'tanh': nn.Tanh, 'elu': nn.ELU}[activation]
        wrap = spectral_norm if use_spectral_norm else (lambda m: m)
        for hidden_dim in hidden_dims:
            layers.append(wrap(nn.Linear(prev_dim, hidden_dim)))
            layers.append(act_fn())
            prev_dim = hidden_dim
        layers.append(wrap(nn.Linear(prev_dim, output_dim)))
        self.network = nn.Sequential(*layers)

    def forward(self, x):
        return self.network(x)


class NormalizedResidualMLP(nn.Module):
    def __init__(self, base_model, x_mean, x_std, y_mean, y_std):
        super().__init__()
        self.net = base_model
        self.register_buffer('x_mean', torch.tensor(x_mean, dtype=torch.float32))
        self.register_buffer('x_std',  torch.tensor(x_std,  dtype=torch.float32))
        self.register_buffer('y_mean', torch.tensor(y_mean, dtype=torch.float32))
        self.register_buffer('y_std',  torch.tensor(y_std,  dtype=torch.float32))

    def forward(self, x):
        x_norm = (x - self.x_mean) / self.x_std
        y_norm = self.net(x_norm)
        return y_norm * self.y_std + self.y_mean


# ─── Lipschitz regularization ───────────────────────────────────────────
def spectral_norm_product(model):
    """prod_i ||W_i||_2 over every nn.Linear in `model` (exact largest
    singular value via torch.linalg.matrix_norm(ord=2), differentiable).
    With 1-Lipschitz activations (tanh/relu/elu) this upper-bounds the
    Lipschitz constant of the network in normalized input/output space."""
    prod = None
    for m in model.modules():
        if isinstance(m, nn.Linear):
            sigma = torch.linalg.matrix_norm(m.weight, ord=2)
            prod = sigma if prod is None else prod * sigma
    return prod


def lipschitz_penalty(model, lam, l_max=None):
    """Method B Lipschitz penalty, added to the data loss:
      l_max is None:  lam * (prod_i ||W_i||_2)^2
      l_max given:    lam * max(0, prod_i ||W_i||_2 - l_max)^2   (hinge)
    Returns (penalty, product); penalty is a zero tensor if lam == 0."""
    prod = spectral_norm_product(model)
    if lam == 0.0:
        return prod.new_zeros(()), prod
    excess = prod if l_max is None else torch.clamp(prod - l_max, min=0.0)
    return lam * excess ** 2, prod


# Output groups of the 9-D d_k target (residuals.py::compute_targets order),
# reported separately because position's one-step residual is near the noise
# floor and otherwise dominates the equally-weighted normalized MSE.
OUTPUT_GROUPS = {"pos": slice(0, 3), "vel": slice(3, 6), "att": slice(6, 9)}


# ─── Training loop ──────────────────────────────────────────────────────
def train_model(model, train_ds, val_ds, device, output_dir, adapter,
                 epochs=80, batch_size=128, lr=5e-4, weight_decay=1e-4, patience=12,
                 lipschitz_lambda=0.0, lipschitz_max=None, loss_weights=None):
    """AdamW + OneCycleLR + gradient clipping + early stopping. Saves the
    best-val-loss checkpoint to {output_dir}/{adapter}_best_model.pt every
    time it improves (the caller reloads this after training, since the
    final epoch isn't necessarily the best one). Returns
    (train_losses, val_losses, best_val, best_model_path).

    lipschitz_lambda/lipschitz_max: see lipschitz_penalty(). The penalty is
    added to the training loss only; train/val losses reported and used for
    early stopping are the pure data MSE.

    loss_weights: optional per-output weights (len output_dim) applied to the
    normalized squared error, mean(w * err^2) -- all-ones (the default) is
    identical to plain MSE. The weighted val loss drives early stopping; the
    per-group (pos/vel/att) val MSE printed alongside is always unweighted."""
    train_loader = DataLoader(train_ds, batch_size=batch_size, shuffle=True)
    val_loader = DataLoader(val_ds, batch_size=batch_size, shuffle=False)

    optimizer = optim.AdamW(model.parameters(), lr=lr, weight_decay=weight_decay)
    out_dim = train_ds[0][1].shape[-1]
    w = torch.as_tensor(loss_weights if loss_weights is not None else [1.0] * out_dim,
                        dtype=torch.float32, device=device)

    def criterion(pred, target):
        return (w * (pred - target) ** 2).mean()

    scheduler = optim.lr_scheduler.OneCycleLR(
        optimizer, max_lr=lr,
        steps_per_epoch=len(train_loader),
        epochs=epochs,
        pct_start=0.15)

    best_model_path = os.path.join(output_dir, f"{adapter}_best_model.pt")
    best_val = float("inf")
    wait = 0
    train_losses, val_losses = [], []

    for epoch in range(1, epochs + 1):
        # ── Train ──
        model.train()
        epoch_loss = 0.0
        for bx, by in train_loader:
            bx, by = bx.to(device), by.to(device)
            optimizer.zero_grad()
            pred = model(bx)
            loss = criterion(pred, by)
            penalty, _ = lipschitz_penalty(model, lipschitz_lambda, lipschitz_max)
            (loss + penalty).backward()
            torch.nn.utils.clip_grad_norm_(model.parameters(), max_norm=1.0)
            optimizer.step()
            scheduler.step()
            epoch_loss += loss.item() * len(bx)
        epoch_loss /= len(train_ds)
        train_losses.append(epoch_loss)

        # ── Validate ──
        model.eval()
        sq_err_sum = torch.zeros(out_dim, device=device)
        with torch.no_grad():
            for bx, by in val_loader:
                bx, by = bx.to(device), by.to(device)
                sq_err_sum += ((model(bx) - by) ** 2).sum(dim=0)
        coord_mse = sq_err_sum / len(val_ds)
        vloss = (w * coord_mse).mean().item()
        val_losses.append(vloss)

        if epoch % 5 == 0 or epoch == 1:
            with torch.no_grad():
                lip = spectral_norm_product(model).item()
            groups = "  ".join(f"{g}={coord_mse[sl].mean().item():.3f}"
                               for g, sl in OUTPUT_GROUPS.items() if sl.stop <= out_dim)
            print(f"  Epoch {epoch:3d}/{epochs}  "
                  f"Train: {epoch_loss:.6f}  Val: {vloss:.6f}  [{groups}]  prod||W||_2: {lip:.3f}")

        # ── Early stopping ──
        if vloss < best_val:
            best_val = vloss
            wait = 0
            torch.save(model.state_dict(), best_model_path)
        else:
            wait += 1
            if wait >= patience:
                print(f"\n  Early stopping at epoch {epoch} "
                      f"(best val = {best_val:.6f})")
                break

    print(f"\n  Training finished. Best Val Loss: {best_val:.6f}")
    return train_losses, val_losses, best_val, best_model_path
