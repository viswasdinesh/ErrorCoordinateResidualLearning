# ommpc_ros2 scripts

Command reference for every runnable script in `scripts/` and `scripts/knode_training/`.

All commands assume you are in the package root (`ommpc_ros2/`) and use the `ecrl` conda env
(it has PyTorch; the system `python3` does not):

```bash
cd ~/ecrl_ws/src/ommpc_ros2
conda run -n ecrl python3 scripts/<script>.py [options]
```

Every argparse-based script also accepts `-h/--help`.

**Typical pipeline**

1. Collect data: `trajectory_generator.py` (polynomial training flights) or `collect_figure8_windy.py` (held-out figure-8 in wind)
2. Inspect residual targets: `residuals.py`
3. Train: `train_residuals_v2.py` (one-step) or `knode_training/train_knode.py` (multi-step rollout)
4. Evaluate: `evaluate_model.py`, `evaluate_figure8_windy.py`, `plot_figure8_windy.py`, `knode_training/evaluate_knode_rollout.py`
5. Export to C++: `export_residual_weights.py`, then rebuild the package

---

## 1. Data collection

### `trajectory_generator.py` (ROS 2 node)

Publishes random polynomial reference trajectories (`PolyTraj` on `/drone_0_planning/trajectory`)
for training data collection. It needs a sourced ROS 2 workspace, not the conda env. It is
installed by CMake, so run it with `ros2 run`:

```bash
ros2 run ommpc_ros2 trajectory_generator.py --ros-args -p num_trajectories:=300 -p windy:=true
```

| ROS parameter | Default | Meaning |
|---|---|---|
| `num_trajectories` | `300` | Number of trajectories to fly |
| `traj_duration_min` / `traj_duration_max` | `8.0` / `12.0` | Trajectory duration range, s |
| `hover_height` | `1.5` | Hover height, m |
| `xy_range` | `2.0` | Horizontal extent, m |
| `z_min` / `z_max` | `0.8` / `2.2` | Altitude range, m |
| `delay_between_trajectories` | `20.0` | Pause between trajectories, s |
| `save_file` | `~/ecrl_datasets_v2/generated_trajectories.json` | Where generated trajectories are saved |
| `time_scales` | `[1.0, 1.25, 1.5]` | Time-scaling factors applied to trajectories |
| `windy` | `false` | Tag the run as windy |

### `collect_figure8_windy.py`

Runs one complete held-out data-collection flight: MicroXRCEAgent, PX4 SITL + Gazebo (headless),
wind injection, `ommpc_node` with `trajectory=figure8`, and `ecrl_data_collector_node`. Then it
tears everything down. See the script docstring for the full pipeline.

```bash
python3 scripts/collect_figure8_windy.py \
    --csv_path ~/ecrl_datasets_v2_eval_windy/figure8/figure8_windy_run1.csv \
    --wind_x 1.3 --wind_y 0.7 --periods 6
```

| Option | Default | Meaning |
|---|---|---|
| `--csv_path` | `~/ecrl_datasets_v2_eval_windy/figure8/figure8_windy_run1.csv` | Output CSV (schema_version=2) |
| `--px4_dir` | `~/PX4-Autopilot` | PX4 checkout |
| `--ws_dir` | `~/ecrl_ws` | ROS 2 workspace |
| `--world` | `ecrl_windy_eval` | Gazebo world |
| `--wind_x`, `--wind_y` | `1.3`, `0.7` | Constant wind velocity, m/s (direction is held out from the train/eval worlds) |
| `--gusty` | off | Also re-publish wind every `--gust_interval_s` with Gaussian perturbations |
| `--gust_std` | `1.5` | Gust std dev, m/s (only with `--gusty`) |
| `--gust_interval_s` | `2.0` | Seconds between gusts (only with `--gusty`) |
| `--gust_seed` | `42` | Gust RNG seed |
| `--periods` | `6` | Figure-8 periods recorded in TRACKING |
| `--period_s` | `20.0` | Figure-8 period, s (shorter = more agile). Forwarded to `ommpc_node` as `figure8_period_s` |
| `--px4_ready_timeout` | `300.0` | Max wait for PX4 "Ready for takeoff!", s |
| `--tracking_timeout` | `180.0` | Max wait for HOVER→TRACKING, s |
| `--no_teardown` | off | Debug: leave everything running |
| `--enable_residual_correction` | off | Live ecrl residual QP correction |
| `--enable_ambient_correction` | off | Live ambient-adapter correction (ecrl wins if both are set) |
| `--residual_clip_norm` | `0.08` | Cap on the combined NN + trim correction norm |
| `--enable_integral_trim` | off | Live disturbance-observer integral trim |
| `--integral_trim_tau` | `10.0` | Trim leaky-integrator time constant, s |
| `--mpc_param KEY=VALUE` | none | Override an `MPC_params.*` field from `config/params.yaml`. Repeatable, e.g. `--mpc_param Q_pos_xy=10.0` |

---

## 2. Residual targets

### `residuals.py`

Library for CSV loading, residual targets (`compute_targets`) and features for both adapters.
Run it directly to check a single CSV:

```bash
conda run -n ecrl python3 scripts/residuals.py /path/to/run.csv --diagnostic-plot diag.png
```

| Option | Default | Meaning |
|---|---|---|
| `csv_path` (positional) | required | Raw schema_version=2 CSV |
| `--diagnostic-plot` | none | Save a PNG comparing new and old residuals (truncation diagnostic) |

---

## 3. Training

### `train_residuals_v2.py` (one-step, production recipe)

Diversity-aware one-step training: mixes calm and windy data, filters trivial residuals,
stratified sampling, AdamW + OneCycleLR + early stopping. Exports TorchScript/ONNX and
`{adapter}_metadata.json`. The implementation is split across `residual_data.py`,
`residual_model.py` and `residual_evaluate.py`; those three are not run directly.

```bash
conda run -n ecrl python3 scripts/train_residuals_v2.py --adapter ecrl
```

| Option | Default | Meaning |
|---|---|---|
| `--calm_dir` | `~/ecrl_datasets_v2` | Calm dataset directory |
| `--windy_dir` | `~/ecrl_datasets_v2/windy` | Windy dataset directory |
| `--output_dir` | `~/ecrl_ws/src/ommpc_ros2/models` | Output directory (**production models**) |
| `--adapter` | `ecrl` | `ecrl` (16-D error-state features) or `ambient` (8-D body-frame features) |
| `--calm_files` | `30` | Calm files to use |
| `--windy_files` | `40` | Windy files to use |
| `--target_samples` | `4000` | Training samples after selection |
| `--residual_filter_pct` | `15` | Drop residuals below this percentile |
| `--n_bins` | `5` | Bins per dimension for stratified sampling |
| `--epochs` | `80` | Max epochs |
| `--batch_size` | `128` | Batch size |
| `--lr` | `5e-4` | Peak learning rate |
| `--weight_decay` | `1e-4` | AdamW weight decay |
| `--patience` | `12` | Early-stopping patience, epochs |
| `--hidden_dims` | `128 128` | Hidden layer widths |
| `--activation` | `tanh` | `relu`, `tanh` or `elu` |
| `--lipschitz_lambda` | `0.0` | Lipschitz penalty weight (see [below](#lipschitz-penalty)) |
| `--lipschitz_max` | none | Hinge threshold for the Lipschitz penalty |
| `--loss_weights` | uniform | 9 per-output weights `p_xyz v_xyz th_xyz` on the normalized squared error, e.g. `0.1 0.1 0.1 1 1 1 1 1 1` to down-weight position |
| `--seed` | `42` | RNG seed |

Each logged epoch prints the unweighted validation MSE per output group, e.g. `[pos=1.02 vel=0.19 att=0.23]`.
Position's one-step residual is about 0.2 mm, so `pos` near 1.0 is the noise floor, not a bug.
`Val` (which drives early stopping) is the **weighted** loss. Compare runs with different
`--loss_weights` using the group numbers, not `Val`.

### `knode_training/train_knode.py` (multi-step rollout)

KNODE-MPC-style training: rolls physics + NN forward over `--rollout_steps` contiguous log
rows using the logged controls, and backpropagates through the whole rollout. Writes to a scratch
directory, not `models/`.

```bash
conda run -n ecrl python3 scripts/knode_training/train_knode.py --rollout_steps 10
```

| Option | Default | Meaning |
|---|---|---|
| `--calm_dir` | `~/ecrl_datasets_v2` | Calm dataset directory |
| `--windy_dir` | `~/ecrl_datasets_v2/windy` | Windy dataset directory |
| `--output_dir` | `scripts/knode_training/output` | Output directory |
| `--adapter` | `ecrl` | `ecrl` or `ambient` |
| `--calm_files` | `30` | Calm files to use |
| `--windy_files` | `40` | Windy files to use |
| `--rollout_steps` | `10` | Rollout window length |
| `--stride` | `rollout_steps // 2` | Window stride, rows |
| `--one_step_weight` | `0.0` | Probability of resetting to ground truth each step (0 = free rollout, 1 ≈ one-step training) |
| `--loss_weights` | uniform | 9 per-coordinate weights `p_xyz v_xyz th_xyz` |
| `--epochs` | `80` | Max epochs |
| `--batch_size` | `64` | Windows per batch |
| `--lr` | `5e-4` | Peak learning rate |
| `--weight_decay` | `1e-4` | AdamW weight decay |
| `--patience` | `12` | Early-stopping patience |
| `--hidden_dims` | `128 128` | Hidden layer widths |
| `--activation` | `tanh` | `relu`, `tanh` or `elu` |
| `--lipschitz_lambda` | `0.0` | Lipschitz penalty weight (see [below](#lipschitz-penalty)) |
| `--lipschitz_max` | none | Hinge threshold for the Lipschitz penalty |
| `--seed` | `42` | RNG seed |

### Lipschitz penalty

Both trainers can add a penalty on the product of the layer spectral norms. This bounds the
Lipschitz constant of the network in normalized space (tanh/relu/elu are 1-Lipschitz). The code is
`spectral_norm_product()` and `lipschitz_penalty()` in `residual_model.py`.

| Flags | Loss |
|---|---|
| `--lipschitz_lambda λ` | `L_data + λ (∏ ‖W_i‖₂)²` |
| `--lipschitz_lambda λ --lipschitz_max L` | `L_data + λ max(0, ∏ ‖W_i‖₂ − L)²` |

- With `--lipschitz_lambda > 0` the model is built **without** `spectral_norm`. That wrapper fixes
  each ‖W_i‖₂ at about 1, which would make the penalty do nothing.
- The penalty affects training only. Reported train/val losses and early stopping use data MSE.
- Each logged epoch prints `prod||W||_2`.

```bash
conda run -n ecrl python3 scripts/knode_training/train_knode.py --lipschitz_lambda 0.1 --lipschitz_max 2.0
```

### `sample_efficiency_sweep.py`

ecrl vs ambient sample-efficiency comparison: trains fresh models at N ∈
{100 … 50000} samples, 5 trials each, and reports RMSE per output block. **No command-line options**.
Settings are constants at the top of the file (`N_SWEEP`, `NUM_TRIALS`, `EPOCHS=40`, `LR=1e-3`, ...).
Writes `models/sample_efficiency_results_v2.json` and `models/sample_efficiency_v2.png`.
Because it has no argument parser, `--help` starts a full run.

```bash
conda run -n ecrl python3 scripts/sample_efficiency_sweep.py
```

---

## 4. Evaluation

### `evaluate_model.py`

One-step RMSE of a trained model vs a zero-residual baseline on a directory of held-out CSVs.

```bash
conda run -n ecrl python3 scripts/evaluate_model.py \
    --model models/ecrl_model_normalized.pt --data_dir /path/to/heldout --adapter ecrl
```

| Option | Default | Meaning |
|---|---|---|
| `--model` | required | Normalized TorchScript model (`*_normalized.pt`) |
| `--data_dir` | required | Directory of held-out raw CSVs |
| `--adapter` | `ecrl` | Must match the model's adapter |

### `evaluate_figure8_windy.py`

Scores the ecrl and ambient models side by side on one figure-8-in-wind CSV, vs zero-residual.

```bash
conda run -n ecrl python3 scripts/evaluate_figure8_windy.py \
    --csv ~/ecrl_datasets_v2_eval_windy/figure8/figure8_windy_run1.csv
```

| Option | Default |
|---|---|
| `--csv` | required |
| `--ecrl_model` | `models/ecrl_model_normalized.pt` |
| `--ambient_model` | `models/ambient_model_normalized.pt` |

### `plot_figure8_windy.py`

Plots a figure-8 flight vs its reference, the true residual, and each model's correction.

```bash
conda run -n ecrl python3 scripts/plot_figure8_windy.py --csv /path/to/figure8.csv
```

| Option | Default |
|---|---|
| `--csv` | required |
| `--out` | PNG next to `--csv` |
| `--ecrl_model` | `models/ecrl_model_normalized.pt` |
| `--ambient_model` | `models/ambient_model_normalized.pt` |

### `knode_training/evaluate_knode_rollout.py`

Rolls out the KNODE model and the production one-step model recursively over the same held-out
windows. Plots RMSE vs rollout step for both, plus a physics-only reference line.

```bash
conda run -n ecrl python3 scripts/knode_training/evaluate_knode_rollout.py --rollout_steps 20
```

| Option | Default | Meaning |
|---|---|---|
| `--csv_path` | `~/ecrl_datasets_v2_eval_windy/figure8/figure8_windy_run1.csv` | Held-out CSV |
| `--adapter` | `ecrl` | `ecrl` or `ambient` |
| `--rollout_steps` | `20` | Rollout length (20 = live MPC horizon) |
| `--stride` | `5` | Window stride |
| `--knode_model` | `knode_training/output/ecrl_knode_model_normalized.pt` | Rollout-trained model |
| `--baseline_model` | `models/ecrl_model_normalized.pt` | Production one-step model |
| `--out` | `knode_training/output/rollout_rmse_comparison.png` | Output plot |

---

## 5. Export to the C++ controller

### `export_residual_weights.py`

Generates a header-only Eigen port of a trained model (`include/ommpc_ros2/residual_model_{adapter}.h`)
for the live MPC. Re-run after every retrain, then rebuild with colcon. Don't hand-edit the header.

```bash
conda run -n ecrl python3 scripts/export_residual_weights.py --adapter ecrl
# export a KNODE model instead:
conda run -n ecrl python3 scripts/export_residual_weights.py --adapter ecrl \
    --model_path scripts/knode_training/output/ecrl_knode_model.pt \
    --metadata_path scripts/knode_training/output/ecrl_knode_metadata.json
```

| Option | Default | Meaning |
|---|---|---|
| `--adapter` | `ecrl` | Names the function (`{adapter}Forward`) and symbol prefixes |
| `--model_path` | `models/{adapter}_model.pt` | Plain (non-normalized) TorchScript model |
| `--metadata_path` | `models/{adapter}_metadata.json` | Matching metadata JSON |
| `--out_header` | `include/ommpc_ros2/residual_model_{adapter}.h` | Generated header |

---

## 6. Parity tests

These are run by hand, not by colcon test, because they need the `ecrl` conda env.

```bash
conda run -n ecrl python3 scripts/knode_training/test_so3_torch_parity.py   # so3_torch.py vs residuals.py, incl. NaN-free gradients
conda run -n ecrl python3 test/test_residuals_parity.py                     # residuals.py vs C++ fixtures
conda run -n ecrl python3 test/test_residual_mlp_parity.py                  # exported Eigen MLP vs PyTorch
```

The two `test/` scripts need fixtures generated first by the C++ binaries in
`build/ommpc_ros2/` (`gen_nominal_dynamics_fixture`, `gen_residual_mlp_fixture`, ...).
See each script's docstring.

---

## Library modules (not run directly)

| File | Contents |
|---|---|
| `residual_data.py` | Loading, filtering, diversity sampling, splitting |
| `residual_model.py` | `ResidualMLP`, `NormalizedResidualMLP`, `train_model`, Lipschitz penalty |
| `residual_evaluate.py` | Test-set evaluation, metadata, ONNX/TorchScript export, plots |
| `knode_training/so3_torch.py` | Batched, differentiable SO(3)/physics core |
| `knode_training/windows.py` | Contiguous rollout-window builder |
