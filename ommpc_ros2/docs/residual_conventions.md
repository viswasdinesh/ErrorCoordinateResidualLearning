# Residual conventions

This document pins down, from the actual code (not from general MPC/ESKF folklore),
the three facts every residual definition in this project depends on:

1. the perturbation convention for the SO(3) error state used by the controller,
2. the discrete nominal map implied by `setStateMatricesandBounds`,
3. the composition/frame of the control vector `u`.

It then defines the **new**, definition-agnostic residual target used from
`ecrl_data_collector_node.cpp` onward, and states explicitly how it differs
from the controller's internal error state — they are related but are not
the same quantity, and conflating them is the bug this refactor removes.

## 1. The controller's existing error-state convention (as built, not as usually assumed)

Source: `MpcController::execMPC` in
[`include/ommpc_ros2/ommpc_controller.hpp`](../include/ommpc_ros2/ommpc_controller.hpp):

```cpp
Eigen::Vector3d err_q = SO3::log(est_q.toRotationMatrix().transpose() * des_q.toRotationMatrix());
Eigen::Vector3d err_p = x_des_start.head(3) - odom.p;
Eigen::Vector3d err_v = x_des_start.tail(3) - odom.v;
delta_x_init << err_p, err_v, err_q;
```

`est_q` is the measured/actual attitude, `des_q` is the reference. Reading this
literally:

- **Position / velocity blocks**: `δp = p_ref − p_actual`, `δv = v_ref − v_actual`
  (reference minus actual — ordinary vector subtraction).
- **Attitude block**: `δθ = Log(R_actual^T R_ref)`, i.e. the tangent vector such
  that `R_ref = R_actual · Exp(δθ)`. This is a **right (body-frame) perturbation
  of the actual attitude that reaches the reference** — the twist is expressed
  in the *actual* body frame, not the reference's.

Note the asymmetry: the vector blocks are ordered (ref, actual) while the
rotation block is ordered (actual, ref) inside the transpose. This is not an
error in the existing code — it's what makes `Fx`'s `(6,6)` block come out as
`Exp(−ω·dt)` (a negative sign) rather than `Exp(+ω·dt)`, and it is exactly the
convention validated by the `test/test_nominal_dynamics.cpp` gtest added in
this change (see §4). **Do not silently symmetrize this when refactoring the
controller** — the Jacobians in `setStateMatricesandBounds` are correct for
*this specific* asymmetric definition.

The control-vector error uses the same "ref minus actual" direction:
`u_actual = u_des − δu` (see `execMPC`'s `u.bodyrates = u_des_start − solution.delta_u[0]`
and the QP bound derivation in `setStateMatricesandBounds`).

## 2. The discrete nominal map implied by the linearization

`setStateMatricesandBounds(i, q, omg, t_step, thracc)` is called with `q`,
`omg`, `thracc` all taken from the **reference** trajectory at step `i`, i.e.
it linearizes the one-step dynamics *about the reference*. Reading the
Jacobian blocks off:

- `Fx` block `(0,3) = t_step·I`: `p_{k+1} = p_k + dt·v_k`.
- `Fx` block `(3,6) = dt·R_k·hat(−e3·thracc_k)`: this is exactly
  `∂/∂δθ [ R_k·Exp(δθ)·e3·thracc_k ]` at `δθ=0`, i.e. the model is
  `v_{k+1} = v_k + dt·(R_k·e3·thracc_k − g·e3)` — forward-Euler integration
  of translational dynamics using the **current-step** rotation `R_k`
  (not `R_{k+1}`), with gravity a constant additive term that cancels in the
  error-state formulation (hence it never appears in `Fx`/`Fu`).
- `Fx` block `(6,6) = Exp(−ω_k·dt)` and `Fu` block `(6,1) = J_l(ω_k·dt)^T·dt`:
  consistent with the reference attitude propagating via the **exact
  Lie-group step** `R_{k+1} = R_k · Exp(ω_k·dt)` (not an Euler/first-order
  approximation of the rotation itself — only the *error* dynamics are
  linearized, the nominal attitude step is exact).

So the canonical nonlinear one-step map, matching the linearization exactly, is:

```
p_{k+1} = p_k + dt·v_k
v_{k+1} = v_k + dt·(R_k·e3·thracc_k − g·e3)
R_{k+1} = R_k · Exp(ω_k·dt)
```

with `g = 9.81` (see `MpcController::gravity_`) and `e3 = [0,0,1]^T`. This is
implemented once in
[`include/ommpc_ros2/nominal_dynamics.h`](../include/ommpc_ros2/nominal_dynamics.h)
as `ommpc::nominalStep`, and mirrored in `scripts/residuals.py::nominal_step`.
Both must stay in sync — that's what the parity test in Task 3 checks.

## 3. Control vector composition

`u = [thracc, ω_x, ω_y, ω_z]` (matches `nu = 4` and every `u_des_start <<
thracc, omg(0), omg(1), omg(2)` call site):

- `thracc`: collective, **mass-normalized thrust acceleration along the body
  +Z axis**, in m/s² (not the `[0,1]`-normalized thrust sent to PX4 — that's
  `thracc / thr2acc`, see `execMPC`'s `normalized_thrust = thrustacc / thr2acc`).
- `ω = [ω_x, ω_y, ω_z]`: commanded body rate, rad/s, expressed in the **body
  frame** (FLU, after the PX4 FRD→FLU conversion in `Odom_Data_t::feed`).

## 4. The new residual target `d_k` (definition-agnostic, logged raw, computed offline)

This is a **different quantity** from §1's `δx`. §1 is an error state relative
to a possibly dynamically-infeasible *reference* trajectory, used internally
by the QP. `d_k` is a one-step **prediction error against the nonlinear
model**, evaluated along the **measured** trajectory — it doesn't care what
the reference was.

Define, for measured states `x_k^meas, x_{k+1}^meas` and applied control `u_k^applied`:

```
d_k = boxminus(x_{k+1}^meas, nominalStep(x_k^meas, u_k^applied, dt))   ∈ R^9
```

where `boxminus(x_a, x_b) := [ p_a − p_b ; v_a − v_b ; Log(R_a^T R_b) ]`
— a single, uniform, right-perturbation difference (unlike §1, there is no
argument-order asymmetry here: the rotation block's transposed/left operand
`a` is always the same operand as the vector blocks' minuend `a`). By
convention, `a` is always the *measured/true* state and `b` is always the
*model-predicted* state — mirroring the pattern in §1 where the transposed
operand inside `Log(·)` is always the actual/current state, just applied
consistently across all three blocks here.

`d_k` absorbs exactly two things: unmodeled physics (drag, motor lag, ground
effect, wind — the thing ECRL is trying to learn) and, if `dt` deviates from
the nominal control period, integration/measurement-timing error. It does
**not** absorb reference-trajectory infeasibility, because there is no
reference in this formula at all. This is what makes it definition-agnostic:
`ambient` and `ecrl` adapters (Task 3) both regress onto the exact same
`d_k`, unmodified, in world frame — they only differ in what *features* they
use to predict it (`ambient`: body-frame velocity/rate/thrust; `ecrl`: the
controller's error state relative to the reference). Neither adapter
transforms the target.

### Why the `ecrl` target is *not* reanchored onto R_ref_k (tried, reverted)

An earlier version of `ecrl_features_and_target` additionally reparametrized
`d_k`'s rotation block from being anchored at `R_meas_{k+1}` onto `R_ref_k`
— i.e. both the *features* and the *target* would live in "error
coordinates relative to the reference." The reanchoring itself,
`Log(R_new^T R_old Exp(v))`, is an exact, invertible SO(3) change of basis
with no approximation error.

The problem is what dominates it. `Log(R_ref_k^T R_meas_{k+1})` is the
reference-tracking error, and whenever it isn't tiny — which is exactly the
condition a wind disturbance produces — it dominates the reanchored value,
because the true `d_k` rotation norm is on the order of the one-step
model-mismatch (~1e-3–1e-2 rad), while tracking error under wind was
measured at ~0.17 rad in this project's own data: two orders of magnitude
larger. A network trained on that target learns to reproduce the
tracking-error term, and even a small *absolute* error there is enormous
relative to the true residual once transported back to world frame for
scoring. Measured effect: trained models scored *worse than a zero-residual
baseline* (predicting exactly zero every time), both in-distribution on the
training wind scene and out-of-distribution on a held-out wind scene.

Conclusion: reanchoring a *target* onto a reference frame is only safe when
that reference tracks tightly (tracking error ≪ the signal you're trying to
isolate). Reanchoring *features* (the input side) has no such requirement,
since a bad/uninformative feature only costs you sample efficiency, not a
corrupted supervision signal — that's why `ecrl`'s error-state *features*
were kept and only the *target* reanchoring was reverted.

### Relationship to Task 4's affine QP term (not implemented in this change)

If `d_k` is later fed back into the QP as an affine offset `c_k` on the
equality constraint, note it must first be re-expressed in the §1 error-state
sign convention (transposed operand = actual, but subtracted in the `ref −
actual` direction for the vector blocks) — it is not a drop-in substitution
without that conversion. Left as a TODO for Task 4.
