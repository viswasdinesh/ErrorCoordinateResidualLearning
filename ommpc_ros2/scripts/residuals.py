#!/usr/bin/env python3
"""residuals.py -- offline, definition-agnostic residual layer for ECRL.

Mirrors ommpc_ros2/include/ommpc_ros2/nominal_dynamics.h exactly (see
test/test_residuals_parity.py for the cross-language check, and
docs/residual_conventions.md for the derivation). ecrl_data_collector_node.cpp
logs only raw state/control/reference/timestamp data (schema_version=2); this
module is where every residual definition lives, so a new definition never
requires re-flying.

Public API:
    load_raw_csv(path)              -> DataFrame (validated schema_version=2)
    compute_targets(df)             -> DataFrame with d_0..d_8 columns added
    ambient_features(df)            -> (X, feature_names)
    ecrl_features_and_target(df)    -> (X, Y) -- Y is d_k, world frame, same as ambient's target
    linearized_residual_diagnostic(df) -> DataFrame with old-style d_lin, |delta_x|
"""
import argparse
import io
import os

import numpy as np
import pandas as pd

GRAVITY = 9.81
E3 = np.array([0.0, 0.0, 1.0])
SCHEMA_VERSION = 2


# ---------------------------------------------------------------------------
# SO(3) helpers -- direct port of include/ommpc_ros2/so3_math.h
# ---------------------------------------------------------------------------
def hat(w):
    w = np.asarray(w, dtype=float)
    return np.array([
        [0.0, -w[2], w[1]],
        [w[2], 0.0, -w[0]],
        [-w[1], w[0], 0.0],
    ])


def vee(W):
    return np.array([W[2, 1], W[0, 2], W[1, 0]])


def so3_exp(w):
    w = np.asarray(w, dtype=float)
    theta = np.linalg.norm(w)
    if theta < 1e-10:
        return np.eye(3) + hat(w)
    axis = w / theta
    axis_hat = hat(axis)
    return np.eye(3) + np.sin(theta) * axis_hat + (1.0 - np.cos(theta)) * (axis_hat @ axis_hat)


def so3_log(R):
    # Clipped only to guard against floating-point overshoot of acos's domain
    # right at +/-1 -- does not change behavior for any valid rotation matrix,
    # and so3_math.h's own small-angle branch already handles theta ~ 0.
    theta = np.arccos(np.clip((np.trace(R) - 1.0) / 2.0, -1.0, 1.0))
    if theta < 1e-10:
        return vee(R - np.eye(3))
    return vee((theta / (2.0 * np.sin(theta))) * (R - R.T))


def left_jacobian(w):
    w = np.asarray(w, dtype=float)
    theta = np.linalg.norm(w)
    if theta < 1e-10:
        w_hat = hat(w)
        return np.eye(3) + 0.5 * w_hat + (1.0 / 6.0) * (w_hat @ w_hat)
    a = w / theta
    a_hat = hat(a)
    return (np.eye(3) + ((1.0 - np.cos(theta)) / theta) * a_hat
            + (1.0 - np.sin(theta) / theta) * (a_hat @ a_hat))


def quat_to_rot(q):
    """q = [w, x, y, z] (Hamilton convention, matches Eigen::Quaterniond)."""
    w, x, y, z = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def rot_to_quat(R):
    """Shepperd's method. Returns [w,x,y,z], normalized. Sign is whichever
    branch fires first -- callers must compare rotations via so3_log-based
    geodesic distance, never raw quaternion components, since q and -q
    represent the same rotation."""
    tr = np.trace(R)
    if tr > 0:
        S = np.sqrt(tr + 1.0) * 2
        w = 0.25 * S
        x = (R[2, 1] - R[1, 2]) / S
        y = (R[0, 2] - R[2, 0]) / S
        z = (R[1, 0] - R[0, 1]) / S
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        S = np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
        w = (R[2, 1] - R[1, 2]) / S
        x = 0.25 * S
        y = (R[0, 1] + R[1, 0]) / S
        z = (R[0, 2] + R[2, 0]) / S
    elif R[1, 1] > R[2, 2]:
        S = np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
        w = (R[0, 2] - R[2, 0]) / S
        x = (R[0, 1] + R[1, 0]) / S
        y = 0.25 * S
        z = (R[1, 2] + R[2, 1]) / S
    else:
        S = np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
        w = (R[1, 0] - R[0, 1]) / S
        x = (R[0, 2] + R[2, 0]) / S
        y = (R[1, 2] + R[2, 1]) / S
        z = 0.25 * S
    q = np.array([w, x, y, z])
    return q / np.linalg.norm(q)


# ---------------------------------------------------------------------------
# Nominal dynamics -- must stay byte-for-byte in sync with nominal_dynamics.h.
# See test/test_residuals_parity.py.
# ---------------------------------------------------------------------------
class State:
    __slots__ = ("p", "v", "q")

    def __init__(self, p, v, q):
        self.p = np.asarray(p, dtype=float)
        self.v = np.asarray(v, dtype=float)
        self.q = np.asarray(q, dtype=float)  # [w,x,y,z]


class Control:
    __slots__ = ("thracc", "omega")

    def __init__(self, thracc, omega):
        self.thracc = float(thracc)
        self.omega = np.asarray(omega, dtype=float)


def nominal_step(x: State, u: Control, dt: float) -> State:
    """p_{k+1}=p_k+dt*v_k ; v_{k+1}=v_k+dt*(R_k*e3*thracc_k-g*e3) ; R_{k+1}=R_k*Exp(omega_k*dt)."""
    R = quat_to_rot(x.q)
    p_next = x.p + dt * x.v
    v_next = x.v + dt * (R @ E3 * u.thracc - GRAVITY * E3)
    R_next = R @ so3_exp(u.omega * dt)
    return State(p_next, v_next, rot_to_quat(R_next))


def boxminus(a: State, b: State) -> np.ndarray:
    """[p_a-p_b ; v_a-v_b ; Log(R_a^T R_b)] in R^9. Right-perturbation
    difference: R_b = R_a * Exp(boxminus(a,b)[6:9]). See
    docs/residual_conventions.md Sec.4 -- NOT the same argument-order
    convention as the controller's internal (asymmetric) error state."""
    Ra = quat_to_rot(a.q)
    Rb = quat_to_rot(b.q)
    out = np.empty(9)
    out[0:3] = a.p - b.p
    out[3:6] = a.v - b.v
    out[6:9] = so3_log(Ra.T @ Rb)
    return out


def controller_error_state(x_ref: State, x_actual: State) -> np.ndarray:
    """The MPC's own (asymmetric) error state, docs/residual_conventions.md
    Sec.1: delta_p = p_ref-p_actual, delta_v = v_ref-v_actual,
    delta_theta = Log(R_actual^T R_ref)."""
    R_actual = quat_to_rot(x_actual.q)
    R_ref = quat_to_rot(x_ref.q)
    out = np.empty(9)
    out[0:3] = x_ref.p - x_actual.p
    out[3:6] = x_ref.v - x_actual.v
    out[6:9] = so3_log(R_actual.T @ R_ref)
    return out


# ---------------------------------------------------------------------------
# Raw CSV loading
# ---------------------------------------------------------------------------
def load_raw_csv(path: str) -> pd.DataFrame:
    with open(path, "r") as f:
        first_line = f.readline()
    if not first_line.startswith("# ecrl_schema_version="):
        raise ValueError(
            f"{path}: missing '# ecrl_schema_version=' header line. "
            "This loader only accepts schema_version=2 raw logs from "
            "ecrl_data_collector_node.cpp; do not feed it old (pre-refactor) "
            "CSVs that already contain baked-in residual columns.")
    version = int(first_line.strip().split("=")[1])
    if version != SCHEMA_VERSION:
        raise ValueError(f"{path}: schema_version={version}, this module only supports {SCHEMA_VERSION}. "
                          "Refusing to silently mix schemas.")
    df = pd.read_csv(path, comment="#")
    # A file can contain a stray *repeated* header row (not just the leading
    # one pandas already consumed) if ecrl_data_collector_node.cpp was
    # restarted mid-collection while appending to the same path -- each
    # restart reopens in append mode and rewrites its own header/comment
    # line. Every real column is numeric, so coerce and drop whatever
    # doesn't parse (the embedded header row's string values, in particular)
    # rather than letting it silently corrupt dtypes downstream.
    n_before = len(df)
    df = df.apply(pd.to_numeric, errors="coerce")
    df = df.dropna()
    if len(df) < n_before:
        print(f"  {path}: dropped {n_before - len(df)} non-numeric row(s) (likely a repeated in-file header)")
    df["_source_file"] = os.path.basename(path)
    return df


# ---------------------------------------------------------------------------
# Shared target d_k
# ---------------------------------------------------------------------------
def _row_state_k(row) -> State:
    return State([row.p_k_x, row.p_k_y, row.p_k_z],
                 [row.v_k_x, row.v_k_y, row.v_k_z],
                 [row.q_k_w, row.q_k_x, row.q_k_y, row.q_k_z])


def _row_state_kp1(row) -> State:
    return State([row.p_kp1_x, row.p_kp1_y, row.p_kp1_z],
                 [row.v_kp1_x, row.v_kp1_y, row.v_kp1_z],
                 [row.q_kp1_w, row.q_kp1_x, row.q_kp1_y, row.q_kp1_z])


def _row_control_applied(row) -> Control:
    return Control(row.thracc_cmd_k, [row.wx_cmd_k, row.wy_cmd_k, row.wz_cmd_k])


def _row_ref_k(row) -> State:
    return State([row.p_ref_k_x, row.p_ref_k_y, row.p_ref_k_z],
                 [row.v_ref_k_x, row.v_ref_k_y, row.v_ref_k_z],
                 [row.q_ref_k_w, row.q_ref_k_x, row.q_ref_k_y, row.q_ref_k_z])


def compute_targets(df: pd.DataFrame, dt_clock_tol_s: float = 0.01, dt_min_s: float = 1e-4,
                     dt_max_s: float = 0.5) -> pd.DataFrame:
    """d_k = boxminus(x_{k+1}^meas, nominal_step(x_k^meas, u_k^applied, dt)).

    Rejects rows where:
      - the monotonic-clock tick spacing and the PX4-timestamp-derived dt_s
        disagree by more than dt_clock_tol_s (catches PX4 timestamp jumps/
        replays that the wall clock didn't see, or vice versa), or
      - dt_s is non-positive or implausibly large (dropout).
    """
    out = df.copy()
    dt_mono_s = (out["t_kp1_mono_ns"] - out["t_k_mono_ns"]) * 1e-9
    dt_odom_s = out["dt_s"]

    valid = (dt_odom_s > dt_min_s) & (dt_odom_s < dt_max_s) & ((dt_mono_s - dt_odom_s).abs() < dt_clock_tol_s)

    d = np.full((len(out), 9), np.nan)
    for i, row in enumerate(out.itertuples()):
        if not valid.iloc[i]:
            continue
        x_k = _row_state_k(row)
        x_kp1 = _row_state_kp1(row)
        u_k = _row_control_applied(row)
        dt = float(row.dt_s)
        x_pred = nominal_step(x_k, u_k, dt)
        d[i, :] = boxminus(x_kp1, x_pred)

    for j in range(9):
        out[f"d_{j}"] = d[:, j]
    out["target_valid"] = valid.values
    return out


# ---------------------------------------------------------------------------
# Ambient adapter: features/output entirely in world/body frame, no reference.
# ---------------------------------------------------------------------------
AMBIENT_FEATURE_NAMES = ["v_body_x", "v_body_y", "v_body_z", "wm_body_x", "wm_body_y", "wm_body_z",
                          "thrust_norm_cmd", "thr2acc"]


def ambient_features(df: pd.DataFrame) -> np.ndarray:
    """Body-frame velocity, body-frame MEASURED angular rate, thrust command,
    and the live thr2acc estimate -- no reference trajectory involved at all.
    Output (the regression target) is scored directly in world frame, i.e.
    it is exactly d_k with no coordinate change."""
    n = len(df)
    X = np.full((n, len(AMBIENT_FEATURE_NAMES)), np.nan)
    for i, row in enumerate(df.itertuples()):
        R_k = quat_to_rot([row.q_k_w, row.q_k_x, row.q_k_y, row.q_k_z])
        v_body = R_k.T @ np.array([row.v_k_x, row.v_k_y, row.v_k_z])
        w_body = np.array([row.wm_k_x, row.wm_k_y, row.wm_k_z])
        X[i, 0:3] = v_body
        X[i, 3:6] = w_body
        X[i, 6] = row.thrust_norm_cmd_k
        X[i, 7] = row.thr2acc_k
    return X


# ---------------------------------------------------------------------------
# ECRL adapter: features in error coordinates relative to x_ref_k; target is
# the SAME world-frame d_k as ambient (see docs/residual_conventions.md Sec.4
# for why this changed from an earlier, reanchored-to-R_ref_k design).
# ---------------------------------------------------------------------------
ECRL_FEATURE_NAMES = ([f"dx_{i}" for i in range(9)] + [f"du_{i}" for i in range(4)] + [f"r_ref_{i}" for i in range(3)])


def ecrl_features_and_target(df: pd.DataFrame) -> "tuple[np.ndarray, np.ndarray]":
    """Returns (X, Y).

    X: 16-D [controller error state (9) ; control error (4) ; Log(R_ref_k) (3)],
       matching the feature width used by the pre-refactor train_residuals*.py
       (dx/du/r_ref), for continuity with earlier experiments.
    Y: d_k, unmodified, in world frame -- identical target to ambient_features'
       caller. Requires d_k already computed via compute_targets().

       Y used to reparametrize d_k's rotation block onto R_ref_k instead of
       R_meas_{k+1} ("error coordinates" for the output too, not just the
       input). That reanchoring is exact but not safe to train on: under any
       real reference-tracking error (which is exactly when a wind
       disturbance makes d_k interesting), Log(R_ref_k^T R_meas_kp1) is the
       tracking error itself and dominates the reanchored value -- e.g.
       tracking error ~0.17 rad swamping a true d_k rotation norm of
       ~0.002 rad in one measured run. The network ends up fitting the
       tracking-error term, and small absolute errors there are enormous
       relative to the tiny true residual once transported back, making
       predictions *worse than a zero-residual baseline* both in- and
       out-of-distribution. Keeping the target in world frame (unreanchored)
       avoids this: ecrl and ambient now differ only in their input features,
       not in what they're scored against.
    """
    if not all(f"d_{i}" in df.columns for i in range(9)):
        raise ValueError("ecrl_features_and_target requires compute_targets(df) to have been run first")

    n = len(df)
    X = np.full((n, 16), np.nan)
    Y = np.full((n, 9), np.nan)

    for i, row in enumerate(df.itertuples()):
        if not getattr(row, "target_valid"):
            continue
        x_k = _row_state_k(row)
        u_k = _row_control_applied(row)
        x_ref_k = _row_ref_k(row)
        u_ref_k = Control(row.thracc_ref_k, [row.wx_ref_k, row.wy_ref_k, row.wz_ref_k])

        dx = controller_error_state(x_ref_k, x_k)
        du = np.array([u_ref_k.thracc - u_k.thracc, *(u_ref_k.omega - u_k.omega)])
        r_ref = so3_log(quat_to_rot(x_ref_k.q))
        X[i, 0:9] = dx
        X[i, 9:13] = du
        X[i, 13:16] = r_ref

        Y[i, :] = [getattr(row, f"d_{j}") for j in range(9)]

    return X, Y


# ---------------------------------------------------------------------------
# Diagnostic: new d_k vs. the OLD linearized residual, and their difference
# (the truncation term the old dataset silently baked in). This is the
# Task 3 "paper figure regardless of how it comes out".
# ---------------------------------------------------------------------------
def linearized_residual_diagnostic(df: pd.DataFrame) -> pd.DataFrame:
    """Reconstructs the OLD (pre-refactor) linearized residual d_lin, which
    needs x_ref at k+1 as well as k. The raw schema only logs x_ref_k, but
    consecutive rows within one contiguous CSV overlap (row i's k+1 tick is
    row i+1's k tick), so x_ref_{k+1} for row i is just row i+1's x_ref_k --
    reconstructed here via a shift rather than re-flying or re-logging.
    """
    out = df.copy()
    out = out.sort_values(["_source_file", "odom_ts_k_us"]).reset_index(drop=True)

    for col in ["p_ref_k_x", "p_ref_k_y", "p_ref_k_z", "v_ref_k_x", "v_ref_k_y", "v_ref_k_z",
                "q_ref_k_w", "q_ref_k_x", "q_ref_k_y", "q_ref_k_z"]:
        out[col.replace("_ref_k_", "_ref_kp1_")] = out.groupby("_source_file")[col].shift(-1)
    # contiguity check: row i's k+1 tick must equal row i+1's k tick
    next_ts_k = out.groupby("_source_file")["odom_ts_k_us"].shift(-1)
    contiguous = (next_ts_k == out["odom_ts_kp1_us"])

    n = len(out)
    d_lin = np.full((n, 9), np.nan)
    delta_x_norm = np.full(n, np.nan)

    for i, row in enumerate(out.itertuples()):
        if not (bool(getattr(row, "target_valid", False)) and bool(contiguous.iloc[i])):
            continue
        dt = float(row.dt_s)
        x_ref_k = _row_ref_k(row)
        u_ref_k = Control(row.thracc_ref_k, [row.wx_ref_k, row.wy_ref_k, row.wz_ref_k])
        x_k = _row_state_k(row)
        u_k = _row_control_applied(row)

        dx_prev = controller_error_state(x_ref_k, x_k)
        du_prev = np.array([u_ref_k.thracc - u_k.thracc, *(u_ref_k.omega - u_k.omega)])

        R_ref_k = quat_to_rot(x_ref_k.q)
        Fx = np.zeros((9, 9))
        Fx[0:3, 0:3] = np.eye(3)
        Fx[3:6, 3:6] = np.eye(3)
        Fx[0:3, 3:6] = dt * np.eye(3)
        Fx[6:9, 6:9] = so3_exp(-u_ref_k.omega * dt)
        Fx[3:6, 6:9] = dt * R_ref_k @ hat(np.array([0.0, 0.0, -u_ref_k.thracc]))

        Fu = np.zeros((9, 4))
        Fu[3:6, 0] = dt * R_ref_k @ np.array([0.0, 0.0, 1.0])
        Fu[6:9, 1:4] = dt * left_jacobian(u_ref_k.omega * dt).T

        dx_nom = Fx @ dx_prev + Fu @ du_prev

        x_ref_kp1 = State([row.p_ref_kp1_x, row.p_ref_kp1_y, row.p_ref_kp1_z],
                           [row.v_ref_kp1_x, row.v_ref_kp1_y, row.v_ref_kp1_z],
                           [row.q_ref_kp1_w, row.q_ref_kp1_x, row.q_ref_kp1_y, row.q_ref_kp1_z])
        x_kp1 = _row_state_kp1(row)
        dx_curr_meas = controller_error_state(x_ref_kp1, x_kp1)

        d_lin[i, :] = dx_curr_meas - dx_nom
        delta_x_norm[i] = np.linalg.norm(dx_prev)

    out["d_lin_valid"] = contiguous.values & out["target_valid"].values
    for j in range(9):
        out[f"d_lin_{j}"] = d_lin[:, j]
    out["delta_x_norm"] = delta_x_norm
    return out


def plot_truncation_diagnostic(df_diag: pd.DataFrame, out_path: str):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    valid = df_diag["d_lin_valid"].values.astype(bool)
    d_cols = [f"d_{j}" for j in range(9)]
    dlin_cols = [f"d_lin_{j}" for j in range(9)]
    d_norm = np.linalg.norm(df_diag.loc[valid, d_cols].values, axis=1)
    dlin_norm = np.linalg.norm(df_diag.loc[valid, dlin_cols].values, axis=1)
    trunc_norm = np.linalg.norm(df_diag.loc[valid, d_cols].values - df_diag.loc[valid, dlin_cols].values, axis=1)
    delta_x_norm = df_diag.loc[valid, "delta_x_norm"].values

    fig, ax = plt.subplots(figsize=(7, 5))
    ax.scatter(delta_x_norm, d_norm, s=4, alpha=0.4, label=r"$\|d_k\|$ (new, nonlinear-model target)")
    ax.scatter(delta_x_norm, dlin_norm, s=4, alpha=0.4, label=r"$\|d_k^{lin}\|$ (old, linearized target)")
    ax.scatter(delta_x_norm, trunc_norm, s=4, alpha=0.4, label=r"$\|d_k - d_k^{lin}\|$ (truncation term)")
    ax.set_xscale("log")
    ax.set_yscale("log")
    ax.set_xlabel(r"$\|\delta x_k\|$ (controller error-state norm)")
    ax.set_ylabel("residual norm")
    ax.set_title("New vs. old residual target, and the truncation term between them")
    ax.legend()
    ax.grid(True, which="both", alpha=0.3)
    fig.tight_layout()
    fig.savefig(out_path, dpi=150)
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description="Compute ECRL residual targets from a raw (schema_version=2) CSV")
    parser.add_argument("csv_path")
    parser.add_argument("--diagnostic-plot", default=None,
                         help="Path to save the new-vs-old-residual truncation diagnostic PNG")
    args = parser.parse_args()

    df = load_raw_csv(args.csv_path)
    df = compute_targets(df)
    n_valid = int(df["target_valid"].sum())
    print(f"Loaded {len(df)} rows, {n_valid} with a valid target d_k.")

    if args.diagnostic_plot:
        df_diag = linearized_residual_diagnostic(df)
        n_diag = int(df_diag["d_lin_valid"].sum())
        print(f"{n_diag} rows usable for the new-vs-old diagnostic (need contiguous k+1 reference too).")
        plot_truncation_diagnostic(df_diag, args.diagnostic_plot)
        print(f"Diagnostic plot -> {args.diagnostic_plot}")


if __name__ == "__main__":
    main()
