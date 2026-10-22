# OMMPC-ROS2 with Error-Coordinate Residual Learning (ECRL)

A ROS 2 / PX4 port of **On-Manifold Model Predictive Control (OMMPC)** for quadrotor trajectory
tracking, extended with **Error-Coordinate Residual Learning (ECRL)**. ECRL learns the
nominal model's one-step prediction error in the controller's own on-manifold error coordinates and
feeds it back into the MPC as an exact affine correction.

- **Controller:** OMMPC on R³ × R³ × SO(3), solved with OSQP (N = 20, Δt = 0.05 s). It sends
  body-rate and collective-thrust setpoints to PX4 over uXRCE-DDS.
- **Live corrections (optional, all off by default):** a learned ECRL residual, an alternative
  "ambient" body-frame residual model, and a disturbance-observer integral trim, with a shared safety clip.
- **Offline pipeline:** raw flight logging, residual targets, one-step and multi-step
  (KNODE-style rollout) training with optional Lipschitz regularization, held-out evaluation, and
  export of the trained network to a dependency-free Eigen header used by the controller.
- **Simulation:** PX4 SITL + Gazebo Harmonic, with calm and windy training/evaluation worlds.

The ROS 1 implementation this port is based on is
[OliverWu515/ommpc_controller](https://github.com/OliverWu515/ommpc_controller). See
[Acknowledgements and citation](#acknowledgements-and-citation).

---

## How ECRL works (short version)

The nominal model `f_nom` is an explicit-Euler rigid-body step. At every log row the
residual target is

```
d_k = x_{k+1}^meas ⊟ f_nom(x_k^meas, u_k^applied, Δt)        ∈ R⁹  (Δp, Δv, Δθ)
```

where ⊟ is the right-perturbation difference on R³ × R³ × SO(3). The `ecrl` adapter predicts
`d_k` from the controller's **error state** relative to the reference (δx ∈ R⁹, δu ∈ R⁴,
Log(R_ref) ∈ R³). These are the coordinates the MPC is linearized in, so the same features are
available at run time. At run time the prediction is composed onto the nominal one-step
prediction exactly on SO(3), and its effect on the QP's error-state dynamics is injected as an
affine term.

Full derivation and conventions: [`ommpc_ros2/docs/residual_conventions.md`](ommpc_ros2/docs/residual_conventions.md).

## Results (simulation)

All numbers are from PX4 SITL + Gazebo, on held-out flights whose wind condition is not in the training data.

| Experiment | Result |
|---|---|
| One-step residual RMS reduction vs. no correction, held-out wind direction ([details](ommpc_ros2/docs/wind_generalization_experiment.md)) | **+58.6%** when trained on 8 randomized wind directions (vs. +6.0% when trained on one direction) |
| Held-out figure-8 in wind: `ecrl` vs `ambient` adapter, one-step RMS reduction | ecrl **+44.5%**, ambient +16.1% |
| Same flight, 20-step open-loop rollout RMSE (rollout-trained `ecrl` model) | 0.111 vs 0.362 with physics only |

---

## Repository layout

```
.
├── ommpc_ros2/                    ROS 2 package (ament_cmake)
│   ├── src/
│   │   ├── ommpc_node.cpp         controller node: FSM (takeoff/hover/track), PX4 offboard I/O
│   │   └── ecrl_data_collector_node.cpp   raw schema_version=2 CSV logger
│   ├── include/ommpc_ros2/
│   │   ├── ommpc_controller.hpp   OMMPC formulation + OSQP solve, residual/trim injection
│   │   ├── nominal_dynamics.h     nominal step, ⊟ / ⊞ operators, residual→QP correction
│   │   ├── so3_math.h             SO(3) exp/log/hat/vee/Jacobians
│   │   ├── polynomial_trajectory.h  piecewise-polynomial trajectories (MIT, Zhepei Wang)
│   │   └── residual_model_{ecrl,ambient}.h   GENERATED Eigen MLPs (scripts/export_residual_weights.py)
│   ├── msg/                       PolyTraj, ReferenceState
│   ├── config/params.yaml         controller parameters (MPC weights, limits, horizon)
│   ├── launch/ommpc.launch.py     controller + data collector
│   ├── sim_worlds/                Gazebo worlds: ecrl_windy_train.sdf, ecrl_windy_eval.sdf
│   ├── scripts/                   offline pipeline (Python), see scripts/README.md
│   │   └── knode_training/        multi-step rollout training + differentiable SO(3) (PyTorch)
│   ├── test/                      gtest suite, C++ fixture generators, Python parity tests
│   ├── docs/                      conventions, experiment write-ups, per-file guide
│   └── models/                    (git-ignored) training outputs
└── px4_msgs/                      git submodule: PX4 ROS 2 message definitions
```

## Requirements

Tested on:

| Component | Version |
|---|---|
| OS | Ubuntu 22.04 |
| ROS 2 | Humble |
| PX4-Autopilot | `main` (v1.17 development series) |
| Simulator | Gazebo Harmonic (gz-sim 8), `gz_x500` |
| DDS bridge | [Micro-XRCE-DDS Agent](https://github.com/eProsima/Micro-XRCE-DDS-Agent) |
| C++ deps | Eigen 3, [OSQP](https://osqp.org) |
| Python (offline pipeline) | 3.10 with PyTorch, NumPy, pandas, matplotlib, tqdm (see [`requirements.txt`](ommpc_ros2/scripts/requirements.txt)) |

## Build

```bash
mkdir -p ~/ecrl_ws && cd ~/ecrl_ws
git clone --recursive <this-repo-url> src
cd ~/ecrl_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --cmake-args -DOSQP_SEARCH_PATH=/path/to/osqp/install
source install/setup.bash
```

- **OSQP.** CMake looks for OSQP's package config under `OSQP_SEARCH_PATH` (default
  `~/acados`, which bundles OSQP). Point it at wherever OSQP is installed.
- **px4_msgs must match your PX4 firmware.** The submodule is pinned to a `PX4/px4_msgs` `main`
  commit. If your PX4 checkout is a different version, sync the message definitions from it
  before building:
  ```bash
  cp ~/PX4-Autopilot/msg/*.msg ~/PX4-Autopilot/msg/versioned/*.msg ~/ecrl_ws/src/px4_msgs/msg/
  ```

Python environment for the offline pipeline:

```bash
conda create -n ecrl python=3.10 && conda activate ecrl
pip install -r ~/ecrl_ws/src/ommpc_ros2/scripts/requirements.txt
```

## Run in simulation

```bash
# 1. DDS bridge
MicroXRCEAgent udp4 -p 8888

# 2. PX4 SITL + Gazebo. For the windy worlds, first copy them into PX4:
#    cp ommpc_ros2/sim_worlds/*.sdf ~/PX4-Autopilot/Tools/simulation/gz/worlds/
cd ~/PX4-Autopilot && PX4_GZ_WORLD=ecrl_windy_train make px4_sitl gz_x500
#    Without a ground station connected, allow arming from the pxh> console:
#    pxh> param set NAV_DLL_ACT 0

# 3. Controller + data logger (arms and switches to OFFBOARD by itself)
ros2 launch ommpc_ros2 ommpc.launch.py

# 4. Reference trajectories (for trajectory: "poly" in config/params.yaml)
ros2 run ommpc_ros2 trajectory_generator.py
```

The built-in references (`trajectory:=figure8 | circle | hover`) and the live corrections are
read once at start-up, so set them at launch:

```bash
ros2 run ommpc_ros2 ommpc_node --ros-args \
    --params-file ~/ecrl_ws/src/ommpc_ros2/config/params.yaml \
    -p trajectory:=figure8 \
    -p enable_residual_correction:=true \
    -p enable_integral_trim:=true
```

The wind can be changed live, without restarting the simulator:

```bash
gz topic -t /world/ecrl_windy_train/wind -m gz.msgs.Wind \
  -p "linear_velocity: {x: 1.4, y: 0.0, z: 0.0}, enable_wind: true"
```

[`scripts/collect_figure8_windy.py`](ommpc_ros2/scripts/collect_figure8_windy.py) automates
steps 1–3 (plus wind injection, recording and teardown) for one held-out evaluation flight.

## Offline pipeline

```bash
cd ~/ecrl_ws/src/ommpc_ros2
# train (one-step) and export to the controller
python3 scripts/train_residuals_v2.py --adapter ecrl --output_dir /tmp/ecrl_run
# or multi-step rollout training (the model currently compiled into the controller)
python3 scripts/knode_training/train_knode.py --rollout_steps 10
# evaluate on a held-out flight
python3 scripts/evaluate_figure8_windy.py --csv ~/ecrl_datasets_v2_eval_windy/figure8/figure8_windy_run1.csv
# regenerate include/ommpc_ros2/residual_model_ecrl.h, then rebuild with colcon
python3 scripts/export_residual_weights.py --adapter ecrl \
    --model_path scripts/knode_training/output/ecrl_knode_model.pt \
    --metadata_path scripts/knode_training/output/ecrl_knode_metadata.json
```

Every script and option is documented in [`ommpc_ros2/scripts/README.md`](ommpc_ros2/scripts/README.md).
Datasets are not included. Collect them with the data collector node; the scripts default to
`~/ecrl_datasets_v2` (calm), `~/ecrl_datasets_v2/windy` and `~/ecrl_datasets_v2_eval_windy`.

## Tests

```bash
colcon test --packages-select ommpc_ros2 && colcon test-result --verbose   # C++ gtest suite
# Python ports vs. C++ / PyTorch (need the conda env)
python3 ommpc_ros2/test/test_residuals_parity.py
python3 ommpc_ros2/test/test_residual_mlp_parity.py
python3 ommpc_ros2/scripts/knode_training/test_so3_torch_parity.py
```

## Documentation

| Document | Contents |
|---|---|
| [`docs/residual_conventions.md`](ommpc_ros2/docs/residual_conventions.md) | Error-state and residual conventions, ⊟/⊞, how the residual enters the QP |
| [`docs/wind_generalization_experiment.md`](ommpc_ros2/docs/wind_generalization_experiment.md) | Wind experiments: approaches tried and rejected, datasets, results |
| [`docs/file_guide.md`](ommpc_ros2/docs/file_guide.md) | What every file does |
| [`scripts/README.md`](ommpc_ros2/scripts/README.md) | Command reference for the offline pipeline |

---

## Acknowledgements and citation

The controller core of this package (`ommpc_controller.hpp`, `so3_math.h`, the node's state
machine, `PolyTraj.msg`) is a ROS 2 / PX4 uXRCE-DDS port of the ROS 1 implementation by Oliver Wu:

> **ommpc_controller**: C++ implementation of On-Manifold MPC.
> https://github.com/OliverWu515/ommpc_controller

which implements the method of:

> G. Lu, W. Xu, and F. Zhang, "On-Manifold Model Predictive Control for Trajectory Tracking on
> Robotic Systems," *IEEE Transactions on Industrial Electronics*, vol. 70, no. 9,
> pp. 9192–9202, 2023. [arXiv:2106.15233](https://arxiv.org/abs/2106.15233)

If you use this code, please cite the OMMPC paper and credit the ROS 1 implementation:

```bibtex
@article{lu2023ommpc,
  author  = {Lu, Guozheng and Xu, Wei and Zhang, Fu},
  title   = {On-Manifold Model Predictive Control for Trajectory Tracking on Robotic Systems},
  journal = {IEEE Transactions on Industrial Electronics},
  volume  = {70},
  number  = {9},
  pages   = {9192--9202},
  year    = {2023}
}

@misc{wu_ommpc_controller,
  author       = {Wu, Oliver},
  title        = {ommpc\_controller: C++ implementation of On-Manifold MPC},
  howpublished = {\url{https://github.com/OliverWu515/ommpc_controller}},
  note         = {ROS 1 implementation}
}
```

Other credits:

- `polynomial_trajectory.h`: Zhepei Wang, MIT License (notice retained in the file), via the ROS 1 implementation.
- The multi-step rollout training in `scripts/knode_training/` follows the idea of
  K. Y. Chee et al., "KNODE-MPC: A Knowledge-Based Data-Driven Predictive Control Framework for
  Aerial Robots," *IEEE RA-L*, vol. 7, no. 2, pp. 2819–2826, 2022.
- [PX4 Autopilot](https://px4.io) and [px4_msgs](https://github.com/PX4/px4_msgs).

## License

<!-- TODO(before publishing): choose the repository license. The ROS 1 code this package is
     ported from (OliverWu515/ommpc_controller) did not declare a license at the time of the
     port; confirm terms with its author before applying one repository-wide. -->

Third-party components keep their own licenses as noted above.
