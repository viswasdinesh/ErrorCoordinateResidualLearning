# ECRL under wind disturbance: idea, rejected approaches, results

Companion to [`residual_conventions.md`](residual_conventions.md), which defines the
residual target itself. This document covers the experiment built on top of it:
does a learned residual model trained under wind disturbance actually generalize
to *different* wind, or does it just memorize one condition.

## The idea

1. Collect calm (no-disturbance) and windy (disturbed) flight data in PX4 SITL +
   Gazebo, using the raw-logging pipeline (`ecrl_data_collector_node.cpp`,
   `scripts/residuals.py`).
2. Train the `ecrl` residual adapter (error-state features relative to the
   reference, target `d_k` — see `residual_conventions.md` §4) on calm + windy
   data.
3. Evaluate the trained model against a **zero-residual baseline** (i.e. "how
   much does the correction actually help") on two held-out sets:
   - in-distribution: unseen trajectories under the *same* wind condition as
     training,
   - out-of-distribution: unseen trajectories under a **different** wind
     condition never seen in training.
4. The out-of-distribution number is the one that matters — it's the actual
   claim ECRL depends on (the residual model captures general aerodynamic
   disturbance, not a lookup table for one flight).

## Rejected ideas

### 1. Reanchoring the `ecrl` target onto the reference attitude (`R_ref_k`)

**Tried:** in addition to expressing the `ecrl` adapter's *features* in error
coordinates relative to the reference (standard, kept), also reparametrize the
*target* `d_k`'s rotation block from being anchored at the measured attitude
onto the reference attitude, via an exact SO(3) reanchor
`Log(R_ref_k^T R_meas_kp1 Exp(v))`.

**Rejected because:** the reanchor is exact and lossless, but not safe to
*train on*. `Log(R_ref_k^T R_meas_kp1)` is the reference-tracking error, and
under any real disturbance it stops being negligible — measured at **~0.17
rad** in this project's own windy data, versus a true `d_k` rotation norm of
**~0.002-0.02 rad**: nearly two orders of magnitude smaller. The reanchored
target is dominated by tracking error, not the residual signal ECRL is
supposed to learn. Effect measured: models trained this way scored **worse
than predicting zero every time**, both in-distribution (-71%) and
out-of-distribution (-446%).

**Fix:** dropped the target reanchoring. `ecrl` and `ambient` now regress onto
the *exact same* world-frame `d_k`; they only differ in input features. See
`residual_conventions.md` §4 for the full writeup.

### 2. Wind magnitude ~4.5-6.4 m/s

**Tried:** initial wind scenes used `linear_velocity` around 4.5 m/s (train)
and 6.4 m/s (eval), picked to be clearly "windy."

**Rejected because:** the controller has no integral action against a
sustained disturbance. At this magnitude the vehicle drifted continuously
(~18m+ in a few seconds), climbed uncommanded, and eventually tripped a PX4
failsafe and disarmed mid-flight. Not just "harder data" — an unrecoverable
flight, unusable for collection.

**Fix:** reduced to ~1.3-1.6 m/s steady wind (still a two-order-of-magnitude
disturbance relative to the calm baseline's residual, per the `d_k` norm
comparison below) — survivable, sustained hover maintained.

### 3. Single, fixed wind direction for all training data

**Tried:** first windy-train dataset used one constant wind vector
(`[1.3, -0.7]` m/s) for the entire collection run.

**Rejected because:** the model learned to compensate along that one wind axis
and nothing else. Evaluating on a held-out scene with a *different* wind
direction (`[-0.8, 1.4]` m/s) gave only **+6.0%** residual reduction overall,
and the eval wind's dominant axis (`vel_y`) actually got **3% worse** than
doing nothing — the model had no signal for that direction and extrapolated
badly.

**Fix:** see Results — randomized wind direction across 8 compass headings.

### 4. Restarting PX4/Gazebo per wind direction

**Considered, not used:** the obvious way to get different wind conditions is
a different world file per condition (which is what `ecrl_windy_train.sdf`
vs `ecrl_windy_eval.sdf` already are). Doing this per-direction would mean 8
separate SITL boot cycles, each 20-30s+ of setup, and each one hit the same
process-teardown flakiness documented below.

**Rejected in favor of:** Gazebo's `WindEffects` system subscribes to a
runtime topic (`/world/<world>/wind`, message type `gz.msgs.Wind`) — the wind
vector can be changed **live**, without restarting the simulation:

```bash
gz topic -t /world/ecrl_windy_train/wind -m gz.msgs.Wind \
  -p "linear_velocity: {x: 1.4, y: 0.0, z: 0.0}, enable_wind: true"
```

Verified this actually changes the physical drift direction (not just a
topic value) before relying on it for data collection.

## Results

### Data collected

| Dataset | Condition | Files (clean) | Rows |
|---|---|---:|---:|
| `ecrl_datasets_v2/` | calm | 18 | ~41k |
| `ecrl_datasets_v2/windy/` | single direction, ~1.4 m/s | 14 | ~21k |
| `ecrl_datasets_v2/windy_multidir/dir{0..7}/` | 8 compass directions, 1.3-1.6 m/s | 24 (3/direction) | ~33k |
| `ecrl_datasets_v2_eval_windy/` | held-out, different direction+turbulence | 9 | ~14k |

"Clean" excludes files quarantined in `_contaminated/` subdirectories — see
Infrastructure issues below.

`d_k` norm (residual magnitude) by condition, confirming wind is a real,
learnable disturbance and not noise: calm mean **0.0039**, single-direction
windy mean **0.0175**, multidirectional windy mean **0.0165** — roughly 4-4.5x
larger under wind than calm.

### Generalization: single direction vs. randomized direction

Both rows use the fixed `ecrl` adapter (no target reanchoring, item 1 above).
"Held-out" is always the eval scene, a wind condition in neither training set.

| Training wind data | In-distribution | Held-out (unseen wind) |
|---|---:|---:|
| Single direction (14 files) | +60.6% | **+6.0%** |
| 8 randomized directions (38 files incl. single-direction set) | +60.6% | **+58.6%** |

The in-distribution number barely moves; the held-out number is the whole
story. With only one training wind direction, the model overfits to it. With
direction diversity, in-distribution and out-of-distribution performance
converge — the signature of learning the underlying disturbance response
rather than memorizing one flight condition. Per-axis detail: `vel_y` (the
held-out scene's dominant wind axis) went from **+3% worse than baseline** to
**-64% better** between the two rows.

### `ecrl` vs `ambient` adapter, same data

`train_residuals_v2.py` and `evaluate_model.py` both gained an `--adapter
{ecrl,ambient}` flag so this comparison didn't need two separate scripts.
Both adapters trained on the *identical* pool (18 calm + 38 randomized-wind
files, same stratified sample of 30k rows) and regress onto the identical
target `d_k` — the only difference is the input features:

| Adapter | Features | In-distribution | Held-out (unseen wind) |
|---|---|---:|---:|
| `ecrl` | error state relative to reference (16-D: `dx`, `du`, `r_ref`) | **+60.6%** | **+58.6%** |
| `ambient` | body-frame velocity/rate + thrust, no reference (8-D) | +9.9% | **-4.3%** |

`ambient` barely helps in-distribution and actively *hurts* on the held-out
scene. This is the expected shape of the result, not a bug: the residual is
fundamentally a function of the *discrepancy* between what the controller
commanded and what happened. `ecrl`'s features hand the network that
discrepancy directly (`dx`, `du` are literally reference-minus-actual).
`ambient` only sees the absolute state — it has no way to distinguish "moving
fast because the trajectory calls for it" from "moving fast because wind
pushed the vehicle off course," and has to try to infer that distinction
indirectly from the training distribution, which doesn't transfer to a new
wind condition.

### Infrastructure issues found and fixed along the way

These aren't the research result, but they'd silently corrupt future runs of
this same experiment if undocumented:

- **PX4 arming blocked by "no GCS connection"** in a no-QGroundControl sandbox
  — fixed by setting PX4 param `NAV_DLL_ACT=0` (per-SITL-instance, not a code
  change).
- **RLS thrust-estimator gating regression**: an earlier session change
  restricted the thrust estimator to `HOVER`/`TRACKING` only, which also
  blocked it during `TAKEOFF` — the initial `thr2acc` guess never got
  corrected and the vehicle stalled below target altitude. Fixed to also run
  during `TAKEOFF` once climbing.
- **`hover_start_time` never reset** on return-to-hover after a trajectory —
  invisible under calm conditions, but under wind it meant the FSM kept
  re-anchoring the hover setpoint to wherever the vehicle currently was, with
  no restoring force, producing unbounded drift. Fixed to reset on every
  HOVER entry.
- **Data contamination from launch-file defaults and trailing idle time**: the
  launch file always defaults `csv_filepath` to `polynomial1.csv` before
  `trajectory_generator.py` redirects it, so every relaunch during debugging
  appended stray rows into that file; and whatever file was active when
  collection *finished* kept accumulating idle-hover rows for however long the
  node stayed up afterward (up to the wall-clock time it took to notice and
  tear down). Both are real per-file artifacts, not one-offs — every
  collection run in this experiment needed its contaminated first/last file
  identified (by row count vs. its siblings) and quarantined before training.
- **Process teardown was unreliable**: `pkill -f` against `px4`/`gz sim`
  frequently reported success while the process was still alive (confirmed via
  `ps -o etimes`, not just presence/absence); direct `kill -9 <pid>` in a
  retry-and-verify loop was the only reliable method found.

## Open questions / next steps

- **Sample efficiency** (still not run): how many directions and how many
  trajectories per direction are actually needed? 4 trajectories x 8
  directions was chosen for session time, not measured necessity.
- **Wind magnitude diversity**: direction was randomized, magnitude only
  jittered slightly (1.3-1.6 m/s). Unknown whether the model generalizes to
  stronger/weaker wind than trained on.
- **`sample_efficiency_sweep.py`** still expects the pre-refactor CSV format
  and hasn't been updated to use `residuals.py`'s adapters (it should now be a
  thin wrapper that sweeps `--calm_files`/`--windy_files`/`--target_samples`
  for a given `--adapter` and records the two generalization numbers above,
  rather than a separate training path).
