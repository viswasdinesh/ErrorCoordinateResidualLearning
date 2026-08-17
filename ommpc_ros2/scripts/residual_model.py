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
                 hidden_dims=[128, 128, 128], activation='tanh'):
        super().__init__()
        layers = []
        prev_dim = input_dim
        act_fn = {'relu': nn.ReLU, 'tanh': nn.Tanh, 'elu': nn.ELU}[activation]
        for hidden_dim in hidden_dims:
            layers.append(spectral_norm(nn.Linear(prev_dim, hidden_dim)))
            layers.append(act_fn())
            prev_dim = hidden_dim
        layers.append(spectral_norm(nn.Linear(prev_dim, output_dim)))
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


# ─── Training loop ──────────────────────────────────────────────────────
def train_model(model, train_ds, val_ds, device, output_dir, adapter,
                 epochs=80, batch_size=128, lr=5e-4, weight_decay=1e-4, patience=12):
    """AdamW + OneCycleLR + gradient clipping + early stopping. Saves the
    best-val-loss checkpoint to {output_dir}/{adapter}_best_model.pt every
    time it improves (the caller reloads this after training, since the
    final epoch isn't necessarily the best one). Returns
    (train_losses, val_losses, best_val, best_model_path)."""
    train_loader = DataLoader(train_ds, batch_size=batch_size, shuffle=True)
    val_loader = DataLoader(val_ds, batch_size=batch_size, shuffle=False)

    optimizer = optim.AdamW(model.parameters(), lr=lr, weight_decay=weight_decay)
    criterion = nn.MSELoss()
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
            loss.backward()
            torch.nn.utils.clip_grad_norm_(model.parameters(), max_norm=1.0)
            optimizer.step()
            scheduler.step()
            epoch_loss += loss.item() * len(bx)
        epoch_loss /= len(train_ds)
        train_losses.append(epoch_loss)

        # ── Validate ──
        model.eval()
        vloss = 0.0
        with torch.no_grad():
            for bx, by in val_loader:
                bx, by = bx.to(device), by.to(device)
                vloss += criterion(model(bx), by).item() * len(bx)
        vloss /= len(val_ds)
        val_losses.append(vloss)

        if epoch % 5 == 0 or epoch == 1:
            print(f"  Epoch {epoch:3d}/{epochs}  "
                  f"Train: {epoch_loss:.6f}  Val: {vloss:.6f}")

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
