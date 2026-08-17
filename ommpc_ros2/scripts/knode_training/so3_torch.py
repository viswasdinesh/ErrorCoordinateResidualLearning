#!/usr/bin/env python3
"""so3_torch.py -- batched, differentiable PyTorch port of ../residuals.py's
SO(3)/nominal-dynamics math, for use inside a training loop that backprops
through a multi-step rollout (see train_knode.py).

Every function here operates on tensors with an arbitrary leading batch
shape (`...`) and a trailing per-quantity dim (3 for vectors, 4 for
quaternions [w,x,y,z] Hamilton convention, 3x3 for rotation matrices) -- no
Python-level branching on tensor values, since that would silently break
autograd for whichever batch elements take the untaken branch. Every
small-angle / degenerate-rotation branch below is computed *unconditionally*
for the whole batch and selected via torch.where, so no NaN or inf produced
by a branch's formula (e.g. 1/theta at theta=0) can ever contaminate a
gradient -- see test_so3_torch_parity.py's gradient check, which is the
actual thing this design protects against (a value mismatch would be
obvious; a NaN gradient silently poisons every parameter after one bad
batch).

Mirrors, in order: include/ommpc_ros2/so3_math.h (C++, used live by the
controller), ../residuals.py (NumPy, used by the existing offline training
pipeline). Keep all three in sync -- see test_so3_torch_parity.py.
"""
import torch

GRAVITY = 9.81

# Small-angle switch threshold. so3_math.h/residuals.py use 1e-10 (float64,
# forward-pass-only concern: just avoid literal 0/0). That's too tight here:
# this runs in float32 and needs the *backward* pass to stay well-conditioned
# near theta=0 too (the regular branch divides by sin(theta), whose gradient
# blows up faster than the value does as theta->0). Both branches are
# continuous and nearly identical in the gap between 1e-10 and 1e-4, so this
# doesn't create a value-parity issue -- see test_so3_torch_parity.py.
EPS = 1e-4


def hat(w: torch.Tensor) -> torch.Tensor:
    """(...,3) -> (...,3,3) skew-symmetric matrix."""
    zeros = torch.zeros_like(w[..., 0])
    wx, wy, wz = w[..., 0], w[..., 1], w[..., 2]
    row0 = torch.stack([zeros, -wz, wy], dim=-1)
    row1 = torch.stack([wz, zeros, -wx], dim=-1)
    row2 = torch.stack([-wy, wx, zeros], dim=-1)
    return torch.stack([row0, row1, row2], dim=-2)


def vee(W: torch.Tensor) -> torch.Tensor:
    """(...,3,3) -> (...,3)."""
    return torch.stack([W[..., 2, 1], W[..., 0, 2], W[..., 1, 0]], dim=-1)


def _eye_like(R: torch.Tensor) -> torch.Tensor:
    eye = torch.eye(3, dtype=R.dtype, device=R.device)
    return eye.expand(R.shape)


def so3_exp(w: torch.Tensor) -> torch.Tensor:
    """(...,3) -> (...,3,3). Both branches computed unconditionally."""
    theta = torch.norm(w, dim=-1, keepdim=True)  # (...,1)
    small = theta < EPS

    theta_safe = torch.where(small, torch.ones_like(theta), theta)
    axis = w / theta_safe
    axis_hat = hat(axis)
    theta_b = theta.unsqueeze(-1)  # (...,1,1) for broadcasting into 3x3
    I = _eye_like(axis_hat)
    R_regular = I + torch.sin(theta_b) * axis_hat + (1.0 - torch.cos(theta_b)) * (axis_hat @ axis_hat)

    R_small = I + hat(w)

    small_b = small.unsqueeze(-1)  # (...,1,1)
    return torch.where(small_b, R_small, R_regular)


def so3_log(R: torch.Tensor) -> torch.Tensor:
    """(...,3,3) -> (...,3). Both branches computed unconditionally."""
    trace = R[..., 0, 0] + R[..., 1, 1] + R[..., 2, 2]
    # Clamp strictly *inside* [-1,1] (not just to it): acos'(x) = -1/sqrt(1-x^2)
    # is genuinely infinite at x=+-1, and torch.where's masking of the
    # small-angle branch below only zeroes the gradient *coefficient* -- it
    # doesn't protect against 0 * inf = NaN upstream of it. Keeping the
    # argument a hair inside the domain keeps this derivative large but
    # finite, which is what actually makes the masking work.
    cos_theta = torch.clamp((trace - 1.0) / 2.0, -1.0 + 1e-7, 1.0 - 1e-7)
    theta = torch.acos(cos_theta)  # (...,)
    small = theta < EPS

    theta_safe = torch.where(small, torch.ones_like(theta), theta)
    sin_theta_safe = torch.sin(theta_safe)
    coef = (theta_safe / (2.0 * sin_theta_safe)).unsqueeze(-1).unsqueeze(-1)  # (...,1,1)
    w_regular = vee(coef * (R - R.transpose(-2, -1)))

    w_small = vee(R - _eye_like(R))

    small_b = small.unsqueeze(-1)  # (...,1)
    return torch.where(small_b, w_small, w_regular)


def quat_to_rot(q: torch.Tensor) -> torch.Tensor:
    """(...,4) [w,x,y,z] -> (...,3,3)."""
    w, x, y, z = q[..., 0], q[..., 1], q[..., 2], q[..., 3]
    row0 = torch.stack([1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)], dim=-1)
    row1 = torch.stack([2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)], dim=-1)
    row2 = torch.stack([2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)], dim=-1)
    return torch.stack([row0, row1, row2], dim=-2)


def rot_to_quat(R: torch.Tensor, eps: float = 1e-8) -> torch.Tensor:
    """(...,3,3) -> (...,4) [w,x,y,z], normalized. Shepperd's method, all 4
    branches computed unconditionally (clamped sqrt args so an unselected
    branch's formula can't produce NaN/inf) and selected via torch.where --
    same rationale as so3_exp/so3_log above. Matches residuals.py::rot_to_quat
    and Eigen::Quaterniond's convention: q and -q represent the same
    rotation, so compare rotations via so3_log-based geodesic distance, never
    raw quaternion components."""
    tr = R[..., 0, 0] + R[..., 1, 1] + R[..., 2, 2]
    r00, r01, r02 = R[..., 0, 0], R[..., 0, 1], R[..., 0, 2]
    r10, r11, r12 = R[..., 1, 0], R[..., 1, 1], R[..., 1, 2]
    r20, r21, r22 = R[..., 2, 0], R[..., 2, 1], R[..., 2, 2]

    def _sqrt(x):
        return torch.sqrt(torch.clamp(x, min=eps))

    S_A = _sqrt(tr + 1.0) * 2
    qw_A, qx_A, qy_A, qz_A = 0.25 * S_A, (r21 - r12) / S_A, (r02 - r20) / S_A, (r10 - r01) / S_A

    S_B = _sqrt(1.0 + r00 - r11 - r22) * 2
    qw_B, qx_B, qy_B, qz_B = (r21 - r12) / S_B, 0.25 * S_B, (r01 + r10) / S_B, (r02 + r20) / S_B

    S_C = _sqrt(1.0 + r11 - r00 - r22) * 2
    qw_C, qx_C, qy_C, qz_C = (r02 - r20) / S_C, (r01 + r10) / S_C, 0.25 * S_C, (r12 + r21) / S_C

    S_D = _sqrt(1.0 + r22 - r00 - r11) * 2
    qw_D, qx_D, qy_D, qz_D = (r10 - r01) / S_D, (r02 + r20) / S_D, (r12 + r21) / S_D, 0.25 * S_D

    cond_A = tr > 0
    cond_B = (~cond_A) & (r00 > r11) & (r00 > r22)
    cond_C = (~cond_A) & (~cond_B) & (r11 > r22)
    # cond_D = everything else (implicit, innermost torch.where fallback)

    def _select(a, b, c, d):
        return torch.where(cond_A, a, torch.where(cond_B, b, torch.where(cond_C, c, d)))

    qw = _select(qw_A, qw_B, qw_C, qw_D)
    qx = _select(qx_A, qx_B, qx_C, qx_D)
    qy = _select(qy_A, qy_B, qy_C, qy_D)
    qz = _select(qz_A, qz_B, qz_C, qz_D)

    q = torch.stack([qw, qx, qy, qz], dim=-1)
    return q / torch.norm(q, dim=-1, keepdim=True)


def nominal_step(p, v, q, thracc, omega, dt):
    """p,v: (...,3). q: (...,4). thracc,dt: (...,). omega: (...,3).
    Mirrors nominal_dynamics.h::nominalStep / residuals.py::nominal_step:
      p_{k+1} = p_k + dt*v_k
      v_{k+1} = v_k + dt*(R_k*e3*thracc_k - g*e3)
      R_{k+1} = R_k * Exp(omega_k*dt)
    """
    R = quat_to_rot(q)
    dt_v = dt.unsqueeze(-1)
    e3 = torch.zeros_like(p)
    e3[..., 2] = 1.0

    p_next = p + dt_v * v
    accel_world = torch.matmul(R, e3.unsqueeze(-1)).squeeze(-1) * thracc.unsqueeze(-1) - GRAVITY * e3
    v_next = v + dt_v * accel_world
    R_next = R @ so3_exp(omega * dt_v)
    q_next = rot_to_quat(R_next)
    return p_next, v_next, q_next


def boxminus(pa, va, qa, pb, vb, qb):
    """[p_a-p_b ; v_a-v_b ; Log(R_a^T R_b)] -> (...,9). Right-perturbation
    difference, same convention as nominal_dynamics.h::boxminus /
    residuals.py::boxminus -- NOT the same argument order as
    controller_error_state below."""
    Ra = quat_to_rot(qa)
    Rb = quat_to_rot(qb)
    dp = pa - pb
    dv = va - vb
    dtheta = so3_log(Ra.transpose(-2, -1) @ Rb)
    return torch.cat([dp, dv, dtheta], dim=-1)


def controller_error_state(p_ref, v_ref, q_ref, p_actual, v_actual, q_actual):
    """delta_p = p_ref-p_actual, delta_v = v_ref-v_actual,
    delta_theta = Log(R_actual^T R_ref). Same convention as
    nominal_dynamics.h::controllerErrorState / residuals.py::controller_error_state."""
    R_actual = quat_to_rot(q_actual)
    R_ref = quat_to_rot(q_ref)
    dp = p_ref - p_actual
    dv = v_ref - v_actual
    dtheta = so3_log(R_actual.transpose(-2, -1) @ R_ref)
    return torch.cat([dp, dv, dtheta], dim=-1)


def compose_from_residual(p_nom, v_nom, q_nom, d):
    """Exact inverse of boxminus(): given a nominal-model prediction and a
    residual d = boxminus(x_true, nominal_pred), reconstructs x_true. Mirrors
    nominal_dynamics.h::composeFromResidual. d: (...,9)."""
    R_nom = quat_to_rot(q_nom)
    p_true = p_nom + d[..., 0:3]
    v_true = v_nom + d[..., 3:6]
    R_true = R_nom @ so3_exp(-d[..., 6:9])
    q_true = rot_to_quat(R_true)
    return p_true, v_true, q_true
