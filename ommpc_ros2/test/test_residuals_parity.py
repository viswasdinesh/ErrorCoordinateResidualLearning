#!/usr/bin/env python3
"""Parity test: scripts/residuals.py's nominal_step()/boxminus() must agree
with the C++ nominal_dynamics.h to numerical precision, or the two have
drifted and neither should be trusted as "the nominal model" any more (see
docs/residual_conventions.md).

Usage:
    ./test/gen_nominal_dynamics_fixture.cpp built as
    build/ommpc_ros2/gen_nominal_dynamics_fixture, run once to produce
    test/fixtures/nominal_dynamics_fixture.csv, then:

        conda run -n ecrl python3 test/test_residuals_parity.py \\
            test/fixtures/nominal_dynamics_fixture.csv

This is intentionally NOT wired into ament/colcon's test runner: it depends
on the `ecrl` conda environment (numpy/pandas), not the ROS python, and colcon
test would need a python interpreter switch mid-build to run it.
"""
import argparse
import sys

import numpy as np
import pandas as pd

sys.path.insert(0, __file__.rsplit("/", 2)[0] + "/scripts")
import residuals as R  # noqa: E402

TOL = 1e-10


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("fixture_csv")
    args = parser.parse_args()

    df = pd.read_csv(args.fixture_csv)
    n = len(df)
    assert n > 0, "empty fixture"

    max_pos_err = 0.0
    max_vel_err = 0.0
    max_rot_err = 0.0
    max_bm_err = 0.0

    for row in df.itertuples():
        x = R.State([row.p_x, row.p_y, row.p_z], [row.v_x, row.v_y, row.v_z],
                     [row.q_w, row.q_x, row.q_y, row.q_z])
        u = R.Control(row.thracc, [row.wx, row.wy, row.wz])
        dt = row.dt

        x_next = R.nominal_step(x, u, dt)

        pos_err = np.abs(x_next.p - np.array([row.step_p_x, row.step_p_y, row.step_p_z])).max()
        vel_err = np.abs(x_next.v - np.array([row.step_v_x, row.step_v_y, row.step_v_z])).max()

        # Quaternions may differ by sign (q vs -q, same rotation) -- compare
        # via geodesic distance instead of raw components.
        R_py = R.quat_to_rot(x_next.q)
        R_cpp = R.quat_to_rot([row.step_q_w, row.step_q_x, row.step_q_y, row.step_q_z])
        rot_err = np.linalg.norm(R.so3_log(R_py.T @ R_cpp))

        x2 = R.State([row.p2_x, row.p2_y, row.p2_z], [row.v2_x, row.v2_y, row.v2_z],
                      [row.q2_w, row.q2_x, row.q2_y, row.q2_z])
        bm_py = R.boxminus(x, x2)
        bm_cpp = np.array([getattr(row, f"bm_{i}") for i in range(9)])
        bm_err = np.abs(bm_py - bm_cpp).max()

        max_pos_err = max(max_pos_err, pos_err)
        max_vel_err = max(max_vel_err, vel_err)
        max_rot_err = max(max_rot_err, rot_err)
        max_bm_err = max(max_bm_err, bm_err)

    print(f"rows checked:        {n}")
    print(f"max position error:  {max_pos_err:.3e}")
    print(f"max velocity error:  {max_vel_err:.3e}")
    print(f"max rotation error:  {max_rot_err:.3e} (geodesic, nominal_step)")
    print(f"max boxminus error:  {max_bm_err:.3e}")

    ok = max(max_pos_err, max_vel_err, max_rot_err, max_bm_err) < TOL
    if not ok:
        print(f"FAIL: exceeds tolerance {TOL:.1e}", file=sys.stderr)
        sys.exit(1)
    print(f"PASS: all errors below {TOL:.1e}")


if __name__ == "__main__":
    main()
