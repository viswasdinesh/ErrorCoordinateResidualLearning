// Standalone compute-time benchmark, no ROS runtime/Gazebo/DDS involved --
// exercises the same C++ code paths execMPC runs in production, driven
// synthetically (same harness pattern as
// test_nominal_dynamics.cpp::IntegralTrim.RejectsConstantDisturbanceDuringHover),
// for a clean, reproducible reading of raw compute cost uncontaminated by
// sim/ROS scheduling noise. Complements the live-flight in-loop timing
// columns logged by ecrl_data_collector_node.cpp (schema_version=4:
// trim_time_us_k/ecrl_time_us_k/qp_solve_time_us_k/total_time_us_k), which
// capture the same quantities under real system contention (PX4 SITL +
// Gazebo + DDS sharing the CPU).
//
// Two sections:
//   1. Isolated ecrlForward() forward-pass latency (no controller/QP at
//      all -- just the generated MLP header).
//   2. Synthetic in-loop execMPC() latency, broken down into trim
//      bookkeeping / ecrl NN / QP solve / total, across the four
//      correction configs (baseline, trim-only, ecrl-only, ecrl+trim),
//      driven by the same closed-loop hover-with-constant-disturbance
//      scenario the IntegralTrim gtest uses (so the QP sees realistic
//      problem conditioning, not a degenerate all-zero state).
//
// Run: build/ommpc_ros2/bench_compute_time [n_iters]
#include <algorithm>
#include <chrono>
#include <cstdio>
#include <random>
#include <string>
#include <vector>

#include <Eigen/Eigen>

#include "ommpc_ros2/ommpc_controller.hpp"
#include "ommpc_ros2/residual_model_ecrl.h"

// MpcController/Odom_Data_t/Controller_Output_t/Parameter_t are declared in
// the global namespace by ommpc_controller.hpp; only the pure-math types
// (NominalState/NominalInput/nominalStep/...) live in namespace ommpc, from
// nominal_dynamics.h -- matches test_nominal_dynamics.cpp's using-declarations.
using ommpc::NominalInput;
using ommpc::NominalState;

namespace
{

struct Stats
{
  double mean, median, p95, p99, max;
};

Stats computeStats(std::vector<double> v)
{
  std::sort(v.begin(), v.end());
  const size_t n = v.size();
  double sum = 0.0;
  for (double x : v) sum += x;
  Stats s;
  s.mean = sum / n;
  s.median = v[n / 2];
  s.p95 = v[static_cast<size_t>(0.95 * (n - 1))];
  s.p99 = v[static_cast<size_t>(0.99 * (n - 1))];
  s.max = v.back();
  return s;
}

void printStats(const std::string &label, const Stats &s)
{
  std::printf("  %-28s mean %8.2f us  median %8.2f us  p95 %8.2f us  p99 %8.2f us  max %8.2f us\n",
              label.c_str(), s.mean, s.median, s.p95, s.p99, s.max);
}

Parameter_t makeBenchParams()
{
  Parameter_t p;
  p.step_T = 0.05;
  p.hover_percent = 0.5;
  // Flight-tuned-ish weights (matches the IntegralTrim gtest's closed-loop
  // config, not the uniform-1.0 weights used only for the Jacobian-parity
  // tests) so the QP sees realistic conditioning, not a degenerate problem.
  p.Q_pos_xy = 2.0;
  p.Q_pos_z = 2.0;
  p.Q_velocity = 1.0;
  p.Q_attitude_rp = 1.0;
  p.Q_attitude_yaw = 10.0;
  p.R_thrust = 0.01;
  p.R_pitchroll = 0.01;
  p.R_yaw = 0.1;
  p.state_cost_exponential = 0.0;
  p.input_cost_exponential = 0.0;
  p.max_bodyrate_xy = 5.0;
  p.max_bodyrate_z = 5.0;
  p.min_thrust = 0.1;
  p.max_thrust = 0.9;
  p.use_fix_yaw = false;
  p.use_trajectory_ending_pos = true;
  p.use_ref_txt = false;
  p.ref_filename = "";
  p.ref_time_step = 0.01;
  p.takeoff_land_speed = 0.3;
  p.takeoff_height = 1.0;
  p.integral_trim_tau = 10.0;    // flight default
  p.residual_clip_norm = 0.08;   // flight default
  return p;
}

// Isolated ecrlForward() latency: no controller, no QP, just the generated
// MLP header, called on random-but-in-distribution-scale inputs.
void benchEcrlForwardIsolated(int n_iters)
{
  std::printf("=== 1. Isolated ecrlForward() (no controller/QP) ===\n");
  std::mt19937 rng(20260806);
  std::uniform_real_distribution<double> unif(-0.5, 0.5);

  std::vector<Eigen::VectorXd> inputs(n_iters);
  for (int i = 0; i < n_iters; ++i)
  {
    Eigen::VectorXd x(16);
    for (int j = 0; j < 16; ++j) x(j) = unif(rng);
    inputs[i] = x;
  }

  std::vector<double> times_us;
  times_us.reserve(n_iters);
  volatile double sink = 0.0;  // prevent the call from being optimized away
  for (int i = 0; i < n_iters; ++i)
  {
    const auto t0 = std::chrono::steady_clock::now();
    const Eigen::VectorXd y = ommpc::residual::ecrlForward(inputs[i]);
    const auto t1 = std::chrono::steady_clock::now();
    sink += y(0);
    times_us.push_back(std::chrono::duration<double, std::micro>(t1 - t0).count());
  }
  (void)sink;

  printStats("ecrlForward()", computeStats(times_us));
  std::printf("  (n=%d calls)\n\n", n_iters);
}

// Synthetic in-loop execMPC() latency, one correction config at a time --
// same hover + constant-disturbance closed-loop scenario as the IntegralTrim
// gtest, run long enough (kSteps) for the trim observer to reach steady
// state and for stats to be meaningful.
void benchExecMpcConfig(const std::string &label, bool enable_residual_correction, bool enable_integral_trim,
                         int kSteps)
{
  Parameter_t p = makeBenchParams();
  p.enable_residual_correction = enable_residual_correction;
  p.enable_integral_trim = enable_integral_trim;

  const Eigen::Vector3d kHoverRefP(0.0, 0.0, 1.0);
  const Eigen::Vector3d kDisturbanceAccel(0.15, 0.1, 0.0);  // same modest unmodeled disturbance as the gtest
  const double dt = 0.01;

  MpcController controller;
  controller.init(p);

  NominalState x_true;
  x_true.p = kHoverRefP;
  x_true.v = Eigen::Vector3d::Zero();
  x_true.q = Eigen::Quaterniond::Identity();

  rclcpp::Time t(0, 0, RCL_ROS_TIME);

  std::vector<double> total_us, trim_us, ecrl_us, qp_us;
  total_us.reserve(kSteps);
  trim_us.reserve(kSteps);
  ecrl_us.reserve(kSteps);
  qp_us.reserve(kSteps);

  for (int i = 0; i < kSteps; ++i)
  {
    controller.setHoverReference(Eigen::Vector4d(0.0, 0.0, 1.0, 0.0));

    Odom_Data_t odom;
    odom.p = x_true.p;
    odom.v = x_true.v;
    odom.q = x_true.q;
    odom.w = Eigen::Vector3d::Zero();

    Controller_Output_t u;
    c_int osqp_status;
    const bool ok = controller.execMPC(odom, u, t, osqp_status);
    if (!ok)
    {
      std::fprintf(stderr, "  WARNING: %s: MPC failed to solve at step %d, skipping\n", label.c_str(), i);
      continue;
    }

    total_us.push_back(controller.last_total_time_us);
    trim_us.push_back(controller.last_trim_time_us);
    if (enable_residual_correction) ecrl_us.push_back(controller.last_ecrl_time_us);
    qp_us.push_back(controller.last_qp_solve_time_us);

    NominalInput u_applied;
    u_applied.thracc = u.thrust * controller.thr2acc;
    u_applied.omega = u.bodyrates;

    NominalState x_next = ommpc::nominalStep(x_true, u_applied, dt);
    x_next.v += dt * kDisturbanceAccel;
    x_true = x_next;
    t = rclcpp::Time(t.nanoseconds() + static_cast<int64_t>(dt * 1e9), RCL_ROS_TIME);
  }

  std::printf("--- %s ---\n", label.c_str());
  printStats("total (execMPC)", computeStats(total_us));
  printStats("trim bookkeeping", computeStats(trim_us));
  if (!ecrl_us.empty()) printStats("ecrl NN forward", computeStats(ecrl_us));
  printStats("QP solve", computeStats(qp_us));
  const double budget_us = 10000.0;  // 100Hz control loop, ommpc_node.cpp
  std::printf("  mean total / 10ms budget: %.1f%%\n\n",
              100.0 * computeStats(total_us).mean / budget_us);
}

}  // namespace

int main(int argc, char **argv)
{
  const int n_iters = argc > 1 ? std::atoi(argv[1]) : 20000;
  const int kSteps = 2000;  // 20s simulated @ 100Hz, matches IntegralTrim gtest

  benchEcrlForwardIsolated(n_iters);

  std::printf("=== 2. Synthetic in-loop execMPC() (hover + constant disturbance, no ROS/Gazebo) ===\n");
  std::printf("(n=%d cycles per config, 100Hz simulated, 10ms real-time budget)\n\n", kSteps);
  benchExecMpcConfig("baseline (no correction)", false, false, kSteps);
  benchExecMpcConfig("trim only", false, true, kSteps);
  benchExecMpcConfig("ecrl only", true, false, kSteps);
  benchExecMpcConfig("ecrl + trim", true, true, kSteps);

  return 0;
}
