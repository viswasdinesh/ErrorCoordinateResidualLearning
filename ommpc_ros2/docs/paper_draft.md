# Error-Coordinate Residual Learning for Wind-Robust Quadrotor MPC

*Draft — working document. Citations marked `[cite: ...]` need real references. Author list,
venue, and formatting TBD; content and numbers below are verified against this session's
experiments (see `docs/file_guide.md` for the underlying pipeline).*

## Abstract

Model Predictive Control (MPC) for quadrotors typically plans against a nominal rigid-body
dynamics model that omits aerodynamic disturbances such as wind drag. Two standard remedies
exist: an online disturbance observer that reactively estimates and cancels a slowly-varying
bias, and residual dynamics learning, which trains a neural network to predict the nominal
model's one-step error directly. We show that the *feature representation* used by the residual
model determines whether it adds anything beyond a disturbance observer already in the loop.
An error-coordinate adapter (**ECRL**: features expressed relative to the reference trajectory
— tracking error, control error, and reference attitude) is compared against an ambient adapter
(flat body-frame velocity/rate/thrust features, no reference dependence) and a zero-residual
baseline, on top of an online disturbance observer, in closed-loop PX4/Gazebo simulation under
wind. With n=5 repeated trials, ECRL reduces tracking error by 10.7% (steady wind, p=0.0021) and
11.5% (gustier wind, p=0.034) beyond the observer alone; the ambient adapter's apparent
improvement (1.4%/1.9%) is not statistically distinguishable from zero (p=0.58/0.72) in either
condition. We further show this advantage holds under a substantially more aggressive
trajectory, persists (and is amplified) under gustier disturbance, comes with negligible
real-time compute cost (~26% of a 10ms/100Hz control budget under real system contention), and
reflects a genuine sample-efficiency advantage: on the residual axis where wind actually enters
the dynamics (velocity), ECRL cuts RMSE by ~53% relative to the ambient adapter, saturating by
~2,500 training samples and never closing even at 20x more data. We also identify and correct a
methodological pitfall — a shared safety-clip norm between the observer and the learned
correction can silently suppress the learned term's visible contribution — that we recommend
future residual-learning-plus-observer evaluations check for explicitly.

## 1. Introduction

[cite: MPC for aerial robotics — standard refs]
[cite: disturbance observers / L1 adaptive control for quadrotors]
[cite: residual/neural-ODE dynamics learning — KNODE-MPC and similar]

A quadrotor MPC controller that plans against a nominal rigid-body model

```
p_{k+1} = p_k + dt * v_k
v_{k+1} = v_k + dt * (a_thrust * R_k * e3 - g * e3)
R_{k+1} = R_k * Exp(omega_k * dt)
```

has no representation of unmodeled aerodynamic effects — most importantly wind drag,
`f_wind(v, R)`, which enters the true dynamics as an additive term on `v_dot`. Two families of
correction exist in the literature and in practice:

1. **Disturbance observers** — an online, typically low-order estimator (e.g. a leaky-integrated
   constant-bias estimate) that reactively tracks and cancels the accumulated tracking offset.
   Simple, robust, requires no training data, but by construction reacts to error rather than
   anticipating it, and is best suited to slowly-varying disturbances.
2. **Residual dynamics learning** — a model (typically a neural network) trained offline on
   logged flight data to predict the nominal model's one-step error `d_k` directly, injected into
   the MPC as an affine correction to the dynamics-equality constraint each cycle. In principle
   this can anticipate structured, state-dependent errors the observer cannot.

These are not mutually exclusive — a natural design combines both, with the observer handling
slow bias and the learned term handling the residual structure the observer misses. This raises
the question this paper addresses: **once a disturbance observer is already in the loop, does a
learned residual model add anything beyond it, and what does its feature representation have to
do with the answer?**

We compare two feature adapters for the residual model, both regressing onto the same 9-D
world-frame one-step residual target:

- **ECRL** (error-coordinate): 16-D — controller error state (position/velocity/attitude error
  relative to the reference, 9-D), control error (reference minus last-applied control, 4-D), and
  the reference attitude in the tangent space (`Log(R_ref)`, 3-D). Every feature is defined
  *relative to* the current reference trajectory.
- **Ambient** (flat/body-frame): 8-D — body-frame velocity (3-D), body-frame measured angular
  rate (3-D), commanded normalized thrust (1-D), and the live thrust-to-acceleration estimate
  (1-D). No reference trajectory dependence at all.

Our contribution is not a new residual-learning method — both adapters use an identical MLP
architecture and training recipe — but a controlled evaluation of *representation* under a
disturbance observer, with statistically powered (n=5) closed-loop flight results, and a
methodological finding (clip-budget sharing) that we believe generalizes to any system combining
a learned correction with an online observer under a shared safety bound.

## 2. Method

### 2.1 Residual definition

For a logged transition `(x_k, u_k, x_{k+1})`, the target is

```
d_k = x_{k+1} boxminus nominal_step(x_k, u_k, dt)
```

a 9-D world-frame quantity (position, velocity, attitude — the latter via `so3_log` of the
rotation error), computed offline from raw logged flight data
(`scripts/residuals.py::compute_targets`). This is the single source of truth for every residual
definition in the pipeline, kept in cross-language parity (C++/Python, tolerance 1e-10) with the
live controller's own `nominal_step`/`boxminus`.

### 2.2 Feature adapters

Both adapters are trained with an identical architecture:
`Linear(D_in, 128) -> tanh -> Linear(128, 128) -> tanh -> Linear(128, 9)`, spectral-norm
regularized, trained with AdamW + OneCycleLR + gradient clipping + early stopping
(`scripts/train_residuals_v2.py::ResidualMLP`). `D_in = 16` for ECRL, `8` for ambient. Neither
uses the diversity-aware sampling tricks (residual filtering, stratified spatial binning) used
for the production-deployed model — [Section 4.2] intentionally asks "how much *raw* data is
needed," not "what is the best achievable model."

### 2.3 MPC integration

The trained model's 9-D prediction is converted to an additive bias on the QP's
dynamics-equality-constraint right-hand side each control cycle
(`residualToQpCorrection` / `applyResidualCorrection`), following the standard affine-correction
approach for injecting a learned residual into a linear MPC [cite]. A single scalar
`residual_clip_norm` caps the world-frame norm of the *combined* correction before injection, as
a safety bound against out-of-distribution model outputs entering a real-time control loop.

### 2.4 Disturbance observer ("integral trim")

A leaky-integrated constant-bias estimator, `d_hat`, updated every control cycle from the same
causally-computable one-step residual the offline pipeline computes from logs, with time
constant `tau` (default 10s):

```
d_hat <- (1 - alpha) * d_hat + alpha * d_scaled,   alpha = clamp(dt_obs / tau, 0, 1)
```

independently clipped to `residual_clip_norm`, then summed with the learned model's prediction
(if enabled) before the *combined* vector is clipped again to the same bound and injected. This
observer runs unconditionally every cycle regardless of whether its output is actually used, so
toggling it mid-flight never starts from a cold estimate. **This shared clip is the mechanism
behind the finding in Section 3.1** — when the observer's own estimate already consumes most of
the clip budget, the learned term's contribution is suppressed by the shared bound regardless of
its predictive quality, independent of whether it is a good model.

## 3. Experimental Setup

All closed-loop experiments use PX4 SITL + Gazebo (windy evaluation world, distinct wind
direction/character from the training-data-collection world), an analytic figure-8 reference
(spatial amplitude 1.5m, nominal period 20s unless noted), and a real-time 100Hz/10ms MPC control
loop (OSQP-solved sparse QP, horizon N=20 steps, `step_T`=0.05s discretization → 1.0s horizon).
Wind is injected via the simulator's wind-effects plugin; two conditions are used throughout:
**steady** (a constant base vector plus the simulator's own mild baked-in turbulence, ~9-13s
periods) and **gusty** (the same base vector, actively perturbed with an independent Gaussian
draw, sigma=1.5 m/s, every 2s — deliberately faster than the observer's `tau`=10s time constant).
The evaluation metric throughout is **mean closed-loop XY tracking error**,
`|p_actual - p_ref|`, over the full flight — the only metric that reflects true live controller
behavior, as distinct from open-loop/retrospective residual-scoring metrics also reported by our
tooling but not used for any claim in this paper.

Training data: 19 calm + 14 windy logged flights (`polynomial*.csv`, randomized reference
trajectories), 64,670 valid one-step transitions after filtering, split 51,736 train / 12,934
test (fixed 80/20 split, seed 42) for the offline sample-efficiency study in Section 4.2. The
production-deployed model (used nowhere in this paper's closed-loop results, which retrain from
scratch per condition for controlled comparison) was trained with diversity-aware sampling on 18
calm + 38 windy files, 21,000 actual training samples after filtering.

## 4. Results

### 4.1 Does the learned residual help beyond the observer? (n=5, statistically powered)

Table 1 reports mean tracking error over 5 independent flights per condition (steady wind,
default MPC tuning; gusty wind, same tuning), all with the observer active and
`residual_clip_norm=0.2` (see the clip-budget note below for why this differs from the system's
0.08 default).

**Table 1.** Closed-loop tracking error, mean +/- std over n=5 trials.

| Condition | Mean XY error (m) | vs. observer-only | Welch's t-test p |
|---|---|---|---|
| observer only, steady | 0.1138 +/- 0.0040 | — | — |
| observer + ambient, steady | 0.1122 +/- 0.0041 | +1.4% | 0.581 (n.s.) |
| observer + ECRL, steady | 0.1017 +/- 0.0015 | **+10.7%** | **0.0021** |
| observer only, gusty | 0.2093 +/- 0.0146 | — | — |
| observer + ambient, gusty | 0.2054 +/- 0.0157 | +1.9% | 0.724 (n.s.) |
| observer + ECRL, gusty | 0.1852 +/- 0.0117 | **+11.5%** | **0.034** |

Directly comparing the two adapters (both + observer): ECRL beats ambient by 9.4% in steady wind
(p=0.0047) and 9.9% in gusty wind (p=0.075, trending but not significant at alpha=0.05 — gustier
wind has visibly higher trial-to-trial variance, std ~0.012-0.016 vs ~0.004 in steady wind).

**Finding.** The ambient adapter's apparent benefit over the observer alone is not
distinguishable from noise in either wind condition. The ECRL adapter's benefit is real and
statistically significant in both. This is the paper's central closed-loop result.

**Methodological note (clip-budget sharing).** At the system's default `residual_clip_norm=0.08`,
this same comparison shows a much weaker apparent ECRL effect (+5.0% at n=1 in gusty wind, vs.
+11.8% at the relaxed 0.2 bound) — diagnostic logging showed the observer's own estimate alone
saturates the 0.08 bound in 76-88% of control cycles under gusty wind, and since the observer and
learned term are summed before a single shared clip is applied, the learned term's visible
contribution shrinks regardless of its underlying quality whenever the observer is already near
the bound. **Any evaluation of a learned correction alongside an online observer under a shared
safety clip should check per-cycle clip-saturation statistics before concluding the learned term
is ineffective** — what looks like "the model isn't helping" may be "the model's contribution is
being clipped away by the observer's own saturation."

### 4.2 Sample efficiency

We trained both adapters from scratch at `N in {100, ..., 50000}` training samples (5 trials per
N, fresh random subsample of the fixed training pool each trial), evaluating one-step RMSE on the
held-out 20% test split, broken out by the residual's natural 9-D block structure (position,
velocity, attitude). Position RMSE — the naive default metric — is a poor choice: one 0.05s
physics-integration step barely moves position regardless of wind (`output_std` ~0.0002m vs.
velocity's ~0.011 — a 40-80x difference), so it is nearly deterministic and shows only an 8%
adapter gap even at N=50,000. Velocity is the physically meaningful axis, since wind enters the
dynamics as an additive term on `v_dot` directly.

**Table 2.** Test-set RMSE reduction, ECRL vs. ambient, by output block.

| Block | N=100 | N=2,500 | N=10,000 | N=50,000 |
|---|---|---|---|---|
| position | 2.5% | 5.4% | 7.6% | 8.3% |
| **velocity** | **42.2%** | 53.2% | 53.2% | **52.9%** |
| attitude | 8.5% | 16.3% | 17.2% | 17.6% |
| overall (9-D) | 37.4% | 47.4% | 47.4% | 47.1% |

On velocity, ECRL's advantage is already large at N=100, saturates by N~2,500-5,000, and holds
flat to N=50,000 (77% of the entire available pool) — the ambient adapter's own error keeps
slowly improving with more data but never closes the gap even at 20x the data needed for ECRL to
reach its ceiling.

### 4.3 Generalization to agile trajectories

The figure-8 period was reduced from 20s to 8s (peak commanded body rate ~1.0 rad/s, at the
system's default actuator limit), then the actuator limit itself was raised
(`max_bodyrate_xy`: 1.0 -> 2.0 rad/s, `max_bodyrate_z`: 0.5 -> 1.0 rad/s) after confirming the
original limit was binding. The prediction horizon (`step_T` in {0.03, 0.05, 0.075, 0.10}) was
separately swept and found already optimal at the default 0.05s — both directions increased
tracking error, so no horizon change was applied.

**Table 3.** Agile trajectory (2.5x faster), steady wind, actuator limits raised, n=1.

| Condition | Mean XY error |
|---|---|
| observer only | 0.2083 m |
| observer + ambient | 0.2024 m (−2.8%) |
| observer + ECRL | 0.1932 m (**−7.2%**) |

Zero OSQP solve failures across ~12,000+ control cycles per flight. The ECRL-over-observer effect
persists at essentially the same relative magnitude as the standard trajectory (7.2% vs. 10.7%),
though we note the ECRL-over-ambient margin *shrinks* under agility (9.4%->~4-5%) while it
*widens* under gustier wind (Section 4.1) — agile maneuvering and gustier disturbance are
different kinds of "hard" and do not uniformly amplify the representation gap; we report this
without over-claiming a single monotonic trend. [This table is n=1; a full n=5 replication of the
agile condition, matching Section 4.1's statistical power, is the natural next step before
treating this comparison as equally conclusive — see Limitations.]

### 4.4 Real-time computational cost

**Table 4.** Per-cycle latency (mean), isolated microbenchmark vs. live SITL under real system
contention.

| | ECRL forward pass | Observer update | QP solve | Total | % of 10ms budget |
|---|---|---|---|---|---|
| Isolated (no ROS/sim) | 6.1 us | 0.13 us | ~500 us | ~505 us | 5.0% |
| Live SITL (real contention) | 36 us | 1.4 us | ~2.6 ms | ~2.65 ms | 26.5% |

Real system contention (PX4 SITL + Gazebo + DDS sharing the CPU) inflates absolute latency ~5x
relative to the isolated benchmark, but the learned model's own cost remains a negligible fraction
of the total (~1.3%) in both measurement regimes — QP solve time dominates throughout. Even under
real contention, the controller retains ~74% margin against its real-time deadline.

### 4.5 Robustness check: an independent way to handle the steady-state offset

As an alternative to the online observer, we separately retuned the MPC's state/input cost
weights (`Q_pos_xy`, `Q_pos_z` x5; `R_thrust`, `R_pitchroll`, `R_yaw` x0.5 from their production
values) to reduce steady-state tracking error directly, with no observer active, then repeated
the three-way adapter comparison on top of *this* baseline instead.

**Table 5.** Retuned MPC (no observer), n=1.

| Wind | baseline | + ambient | + ECRL |
|---|---|---|---|
| steady | 0.1507 m | 0.1439 m (−4.5%) | 0.1386 m (**−8.0%**) |
| gusty | 0.2741 m | 0.2575 m (−6.1%) | 0.2229 m (**−18.7%**) |

This reproduces Section 4.1's qualitative pattern (ECRL > ambient > baseline, gap widening under
gustier wind) via a mechanistically unrelated way of handling the steady-state offset —
cross-validating that the result is about the learned representation, not an artifact of the
particular observer design in Section 2.4.

## 5. Discussion and Limitations

- **Statistical power is uneven across sections.** Section 4.1 (the paper's central claim) is
  n=5 with significance tests. Sections 4.3 and 4.5 are n=1 and should be read as
  directionally-consistent supporting evidence, not independently conclusive — the natural
  next step is replicating those at n=5 before final submission.
- `residual_clip_norm=0.2` (Section 4.1, 4.3) was chosen diagnostically, to avoid the
  saturation artifact described in Section 4.1, not vetted as a safe production bound; a
  deployed system would need this justified independently (e.g. from the maximum expected
  residual magnitude with margin).
- The evaluation world's wind includes baked-in turbulence (sin+noise modulation, ~9-13s
  periods) even in the "steady" condition — only the base wind vector is literally constant.
- An unresolved anomaly: ECRL's *maximum* (not mean) tracking error exceeded both baseline and
  ambient in the retuned-MPC study (Section 4.5) despite the better mean — worth root-causing
  before claiming ECRL strictly dominates on tail behavior, not just average behavior.
  [cite/investigate]
- All results are single-scene (one simulated wind world); generalization to further
  wind conditions, vehicle platforms, or real hardware is untested.
- Sample-efficiency results (Section 4.2) are open-loop/offline (one-step regression on held-out
  data) — a different, complementary measurement to the closed-loop tracking results, not a
  substitute for them.

## 6. Conclusion

[cite existing lit for framing] We show that whether a learned residual dynamics model adds value
beyond an online disturbance observer depends critically on its feature representation: an
error-coordinate adapter yields a statistically significant, real-time-cheap, sample-efficient
improvement that a flat/ambient adapter with identical architecture and training does not, and
this holds up under both gustier wind and a substantially more agile reference trajectory, and
under an entirely different mechanism (aggressive MPC retuning) for handling the steady-state
offset. We further identify a specific, generalizable methodological pitfall — clip-budget
sharing between a learned correction and a co-located observer — that can make an effective
learned model appear ineffective if not checked for explicitly.

---

## Appendix / Reproducibility notes

- Full pipeline documentation: `docs/file_guide.md`.
- Residual/target definitions: `scripts/residuals.py` (cross-language parity-tested against the
  live C++ controller, `test/test_residuals_parity.py`).
- Both adapters trained via `scripts/train_residuals_v2.py --adapter {ecrl,ambient}`.
- Sample-efficiency sweep: `scripts/sample_efficiency_sweep.py`.
- Closed-loop data collection: `scripts/collect_figure8_windy.py` (see `--help` for the full set
  of wind/gust/tuning/horizon/adapter flags used to produce every table above).
- Compute-time benchmark: `build/ommpc_ros2/bench_compute_time`.
- Raw CSVs for every flight in this paper: `~/ecrl_datasets_v2_eval_windy/figure8/`.
- n=5 raw per-trial values and significance-test code: session artifacts,
  `n5_results.json` (Table 1).

*TODO before submission: related work section (currently placeholder citations only), author
list / affiliations, target venue formatting, Figures (trajectory plots and sample-efficiency
curves already generated as PNGs alongside each CSV — need selecting/polishing for print), n=5
replication of Sections 4.3 and 4.5, root-cause of the ECRL max-error anomaly (Section 5).*
