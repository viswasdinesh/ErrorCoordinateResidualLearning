#!/usr/bin/env python3
"""test_so3_torch_parity.py -- validates so3_torch.py against ../residuals.py
(the existing, already-trusted NumPy implementation) on random batched
inputs, and separately checks that backprop through so3_torch's small-angle
branches never produces NaN/Inf gradients (the actual risk this port
introduces beyond a plain value port -- see so3_torch.py's module docstring).

Run manually (matches this project's existing parity-test convention, e.g.
test/test_residual_mlp_parity.py): conda run -n ecrl python3
scripts/knode_training/test_so3_torch_parity.py
"""
import os
import sys

import numpy as np
import torch

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
import so3_torch as T  # noqa: E402
import residuals as R  # noqa: E402

torch.set_default_dtype(torch.float64)  # match residuals.py's float64 for a tight parity tolerance
VALUE_TOL = 1e-8
N = 500
SEED = 20260803


def random_rotvecs(rng, n, theta_max):
    """Random axis-angle vectors, including a chunk pinned very close to 0
    to specifically exercise the small-angle branch."""
    axis = rng.normal(size=(n, 3))
    axis /= np.linalg.norm(axis, axis=1, keepdims=True)
    theta = rng.uniform(0.0, theta_max, size=(n, 1))
    w = axis * theta
    # Force ~10% of rows to be extremely close to zero (well under so3_torch's
    # EPS=1e-4 switch and residuals.py's 1e-10 switch both).
    n_small = max(1, n // 10)
    idx = rng.choice(n, n_small, replace=False)
    w[idx] = rng.normal(scale=1e-9, size=(n_small, 3))
    return w


def rotvec_to_quat_np(w):
    return np.array([R.rot_to_quat(R.so3_exp(wi)) for wi in w])


def main():
    rng = np.random.default_rng(SEED)

    w1 = random_rotvecs(rng, N, theta_max=2.5)  # exp() input
    q1 = rotvec_to_quat_np(w1)                   # log()/rot_to_quat input, via a valid rotation
    w2 = random_rotvecs(rng, N, theta_max=2.5)
    q2 = rotvec_to_quat_np(w2)

    p_a = rng.normal(size=(N, 3))
    v_a = rng.normal(size=(N, 3))
    p_b = rng.normal(size=(N, 3))
    v_b = rng.normal(size=(N, 3))
    thracc = rng.uniform(5.0, 15.0, size=N)
    omega = rng.normal(scale=1.0, size=(N, 3))
    dt = rng.uniform(0.005, 0.05, size=N)
    d = rng.normal(scale=0.03, size=(N, 9))

    w1_t, q1_t, w2_t, q2_t = (torch.tensor(x) for x in (w1, q1, w2, q2))
    p_a_t, v_a_t, p_b_t, v_b_t = (torch.tensor(x) for x in (p_a, v_a, p_b, v_b))
    thracc_t, omega_t, dt_t, d_t = (torch.tensor(x) for x in (thracc, omega, dt, d))

    failures = []

    def check(name, np_vals, torch_vals):
        np_vals = np.asarray(np_vals)
        torch_vals = torch_vals.detach().numpy()
        err = np.max(np.abs(np_vals - torch_vals))
        status = "OK" if err < VALUE_TOL else "FAIL"
        print(f"  [{status}] {name}: max abs err = {err:.3e}")
        if err >= VALUE_TOL:
            failures.append(name)

    print("=== so3_exp ===")
    exp_np = np.array([R.so3_exp(wi) for wi in w1])
    exp_t = T.so3_exp(w1_t)
    check("so3_exp", exp_np, exp_t)

    print("=== so3_log ===")
    log_np = np.array([R.so3_log(R.quat_to_rot(qi)) for qi in q1])
    log_t = T.so3_log(T.quat_to_rot(q1_t))
    check("so3_log", log_np, log_t)

    print("=== quat_to_rot ===")
    qtr_np = np.array([R.quat_to_rot(qi) for qi in q1])
    qtr_t = T.quat_to_rot(q1_t)
    check("quat_to_rot", qtr_np, qtr_t)

    print("=== rot_to_quat (geodesic distance, since q/-q both valid) ===")
    R_from_np = np.array([R.quat_to_rot(qi) for qi in q1])
    rtq_np = np.array([R.rot_to_quat(Ri) for Ri in R_from_np])
    rtq_t = T.rot_to_quat(torch.tensor(R_from_np))
    # Compare via geodesic angle between the two quaternions' rotations, not
    # raw components (q and -q are the same rotation).
    geo_err = []
    for qi_np, qi_t in zip(rtq_np, rtq_t.detach().numpy()):
        Ra = R.quat_to_rot(qi_np)
        Rb = R.quat_to_rot(qi_t)
        geo_err.append(np.linalg.norm(R.so3_log(Ra.T @ Rb)))
    max_geo_err = max(geo_err)
    status = "OK" if max_geo_err < 1e-6 else "FAIL"
    print(f"  [{status}] rot_to_quat: max geodesic err = {max_geo_err:.3e}")
    if max_geo_err >= 1e-6:
        failures.append("rot_to_quat")

    print("=== nominal_step ===")
    p_next_np, v_next_np, q_next_np = [], [], []
    for i in range(N):
        x = R.State(p_a[i], v_a[i], q1[i])
        u = R.Control(thracc[i], omega[i])
        x_next = R.nominal_step(x, u, dt[i])
        p_next_np.append(x_next.p)
        v_next_np.append(x_next.v)
        q_next_np.append(x_next.q)
    p_next_t, v_next_t, q_next_t = T.nominal_step(p_a_t, v_a_t, q1_t, thracc_t, omega_t, dt_t)
    check("nominal_step.p", p_next_np, p_next_t)
    check("nominal_step.v", v_next_np, v_next_t)
    geo_err = [np.linalg.norm(R.so3_log(R.quat_to_rot(a).T @ R.quat_to_rot(b)))
               for a, b in zip(q_next_np, q_next_t.detach().numpy())]
    max_geo_err = max(geo_err)
    status = "OK" if max_geo_err < 1e-6 else "FAIL"
    print(f"  [{status}] nominal_step.q: max geodesic err = {max_geo_err:.3e}")
    if max_geo_err >= 1e-6:
        failures.append("nominal_step.q")

    print("=== boxminus ===")
    bm_np = np.array([R.boxminus(R.State(p_a[i], v_a[i], q1[i]), R.State(p_b[i], v_b[i], q2[i]))
                       for i in range(N)])
    bm_t = T.boxminus(p_a_t, v_a_t, q1_t, p_b_t, v_b_t, q2_t)
    check("boxminus", bm_np, bm_t)

    print("=== controller_error_state ===")
    ces_np = np.array([R.controller_error_state(R.State(p_a[i], v_a[i], q1[i]), R.State(p_b[i], v_b[i], q2[i]))
                        for i in range(N)])
    ces_t = T.controller_error_state(p_a_t, v_a_t, q1_t, p_b_t, v_b_t, q2_t)
    check("controller_error_state", ces_np, ces_t)

    print("=== compose_from_residual (round-trip against boxminus) ===")
    # x_true = compose_from_residual(nominal_pred, boxminus(x_true, nominal_pred)) exactly.
    p_true_t, v_true_t, q_true_t = T.compose_from_residual(p_a_t, v_a_t, q1_t, d_t)
    d_recovered = T.boxminus(p_true_t, v_true_t, q_true_t, p_a_t, v_a_t, q1_t)
    err = (d_recovered - d_t).abs().max().item()
    status = "OK" if err < VALUE_TOL else "FAIL"
    print(f"  [{status}] compose_from_residual round-trip: max abs err = {err:.3e}")
    if err >= VALUE_TOL:
        failures.append("compose_from_residual")

    # ------------------------------------------------------------------
    # Gradient safety check: backprop a scalar loss through every function,
    # evaluated AT theta~0 inputs specifically (the whole point of this
    # port), and assert every input gradient is finite. A NaN gradient here
    # would silently poison every parameter update once training hits a
    # near-identity rotation -- which happens constantly at hover.
    # ------------------------------------------------------------------
    print("\n=== gradient finiteness at theta~0 ===")
    grad_failures = []

    def grad_check(name, inputs, output):
        loss = output.sum()
        grads = torch.autograd.grad(loss, inputs, retain_graph=False, allow_unused=True)
        ok = all(g is None or torch.isfinite(g).all() for g in grads)
        print(f"  [{'OK' if ok else 'FAIL'}] {name}")
        if not ok:
            grad_failures.append(name)

    w0 = torch.zeros(8, 3, dtype=torch.float64, requires_grad=True)
    grad_check("so3_exp @ theta=0", [w0], T.so3_exp(w0))

    R0 = torch.eye(3, dtype=torch.float64).expand(8, 3, 3).contiguous()
    R0.requires_grad_(True)
    grad_check("so3_log @ identity", [R0], T.so3_log(R0))

    q_id = torch.zeros(8, 4, dtype=torch.float64)
    q_id[:, 0] = 1.0
    q_id.requires_grad_(True)
    grad_check("quat_to_rot -> so3_log round trip @ identity", [q_id], T.so3_log(T.quat_to_rot(q_id)))

    R_id = torch.eye(3, dtype=torch.float64).expand(8, 3, 3).contiguous()
    R_id.requires_grad_(True)
    grad_check("rot_to_quat @ identity", [R_id], T.rot_to_quat(R_id))

    p0 = torch.zeros(8, 3, dtype=torch.float64, requires_grad=True)
    v0 = torch.zeros(8, 3, dtype=torch.float64, requires_grad=True)
    thracc0 = 9.81 * torch.ones(8, dtype=torch.float64, requires_grad=True)
    omega0 = torch.zeros(8, 3, dtype=torch.float64, requires_grad=True)
    dt0 = 0.01 * torch.ones(8, dtype=torch.float64, requires_grad=True)
    p1, v1, q1_out = T.nominal_step(p0, v0, q_id.detach().requires_grad_(True), thracc0, omega0, dt0)
    grad_check("nominal_step @ hover/theta=0", [p0, v0, omega0, dt0], torch.cat([p1, v1, q1_out], dim=-1))

    d0 = torch.zeros(8, 9, dtype=torch.float64, requires_grad=True)
    p_t2, v_t2, q_t2 = T.compose_from_residual(p0.detach(), v0.detach(), q_id.detach(), d0)
    grad_check("compose_from_residual @ d=0", [d0], torch.cat([p_t2, v_t2, q_t2], dim=-1))

    print("\n" + "=" * 60)
    if failures or grad_failures:
        print(f"FAILED: value mismatches={failures} gradient issues={grad_failures}")
        sys.exit(1)
    else:
        print(f"ALL PASSED ({N} random samples, {8} theta~0 gradient probes)")


if __name__ == "__main__":
    main()
