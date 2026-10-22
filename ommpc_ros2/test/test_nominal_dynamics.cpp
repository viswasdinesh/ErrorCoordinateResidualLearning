// Validates that MpcController::setStateMatricesandBounds() produces Fx/Fu
// that are the true first-order Jacobians of ommpc::nominalStep(), under the
// controller's own (asymmetric) error-state convention documented in
// docs/residual_conventions.md §1. If this test fails after touching either
// nominal_dynamics.h or setStateMatricesandBounds(), the two have drifted
// out of sync and neither should be trusted as "the nominal model".
#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <random>
#include <vector>

#include <Eigen/Eigen>

#include "ommpc_ros2/nominal_dynamics.h"
#include "ommpc_ros2/ommpc_controller.hpp"

using ommpc::NominalInput;
using ommpc::NominalState;
using ommpc::nominalStep;
using ommpc::residualToQpCorrection;

namespace
{

Parameter_t makeTestParams()
{
  Parameter_t p;
  p.step_T = 0.05;
  p.hover_percent = 0.5;
  p.Q_pos_xy = 1.0;
  p.Q_pos_z = 1.0;
  p.Q_velocity = 1.0;
  p.Q_attitude_rp = 1.0;
  p.Q_attitude_yaw = 1.0;
  p.R_thrust = 1.0;
  p.R_pitchroll = 1.0;
  p.R_yaw = 1.0;
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
  return p;
}

// Inverts the controller's error-state definition (docs §1):
//   delta_p = p_ref - p_actual,  delta_v = v_ref - v_actual
//   delta_theta = Log(R_actual^T R_ref)  =>  R_ref = R_actual * Exp(delta_theta)
NominalState actualFromError(const NominalState &ref, const Eigen::Matrix<double, 9, 1> &delta_x)
{
  NominalState actual;
  actual.p = ref.p - delta_x.segment<3>(0);
  actual.v = ref.v - delta_x.segment<3>(3);
  const Eigen::Matrix3d R_actual = ref.q.toRotationMatrix() * SO3::exp(-delta_x.segment<3>(6));
  actual.q = Eigen::Quaterniond(R_actual);
  actual.q.normalize();
  return actual;
}

// The function under test in this file's ResidualQpInjection cases,
// ommpc::residualToQpCorrection, now lives in nominal_dynamics.h as
// production code (execMPC uses it directly) -- see its doc comment there
// for what it computes and why. Only actualFromError (a test-only inverse
// used to synthesize operating points) stays local to this file.

} // namespace

TEST(NominalDynamicsJacobian, MatchesSecondOrderTruncation)
{
  const double dt = 0.05;

  // Representative non-trivial reference operating point: tilted, rotating,
  // above-hover thrust. Testing only at the identity/hover point would not
  // exercise the R*hat(e3*thracc) or Exp(-omega*dt) terms meaningfully.
  NominalState x_ref;
  x_ref.p = Eigen::Vector3d(1.0, -0.5, 1.5);
  x_ref.v = Eigen::Vector3d(0.3, 0.2, -0.1);
  x_ref.q = Eigen::Quaterniond(Eigen::AngleAxisd(0.3, Eigen::Vector3d(0.2, 0.5, 1.0).normalized()));
  x_ref.q.normalize();

  NominalInput u_ref;
  u_ref.thracc = ommpc::kGravity * 1.15;
  u_ref.omega = Eigen::Vector3d(0.4, -0.3, 0.15);

  MpcController controller;
  controller.init(makeTestParams());
  controller.setStateMatricesandBounds(0, x_ref.q, u_ref.omega, dt, u_ref.thracc);

  ASSERT_EQ(controller.Fx[0].rows(), 9);
  ASSERT_EQ(controller.Fx[0].cols(), 9);
  ASSERT_EQ(controller.Fu[0].rows(), 9);
  ASSERT_EQ(controller.Fu[0].cols(), 4);

  const Eigen::MatrixXd Fx = Eigen::MatrixXd(controller.Fx[0]);
  const Eigen::MatrixXd Fu = Eigen::MatrixXd(controller.Fu[0]);

  const NominalState x_next_ref = nominalStep(x_ref, u_ref, dt);

  std::mt19937 rng(12345);
  std::normal_distribution<double> gauss(0.0, 1.0);
  Eigen::Matrix<double, 9, 1> dir_x;
  Eigen::Matrix<double, 4, 1> dir_u;
  for (int i = 0; i < 9; ++i) dir_x(i) = gauss(rng);
  for (int i = 0; i < 4; ++i) dir_u(i) = gauss(rng);
  dir_x.normalize();
  dir_u.normalize();

  const std::vector<double> epsilons = {1e-1, 1e-2, 1e-3, 1e-4};
  std::vector<double> residual_norms;
  residual_norms.reserve(epsilons.size());

  for (double eps : epsilons)
  {
    const Eigen::Matrix<double, 9, 1> delta_x = eps * dir_x;
    const Eigen::Matrix<double, 4, 1> delta_u = eps * dir_u;

    const NominalState x_actual = actualFromError(x_ref, delta_x);
    NominalInput u_actual;
    u_actual.thracc = u_ref.thracc - delta_u(0);
    u_actual.omega = u_ref.omega - delta_u.segment<3>(1);

    const NominalState x_next_actual = nominalStep(x_actual, u_actual, dt);
    const Eigen::Matrix<double, 9, 1> delta_x_next_true = ommpc::controllerErrorState(x_next_ref, x_next_actual);
    const Eigen::Matrix<double, 9, 1> delta_x_next_linear = Fx * delta_x + Fu * delta_u;

    residual_norms.push_back((delta_x_next_true - delta_x_next_linear).norm());
  }

  // log-log slope between consecutive decades must be ~2 (quadratic
  // truncation) -- this is what proves Fx/Fu are first-order Jacobians of
  // nominalStep(), not just "some" matrices that happen to look plausible.
  for (size_t i = 0; i + 1 < epsilons.size(); ++i)
  {
    ASSERT_GT(residual_norms[i], 0.0) << "residual collapsed to exactly zero, slope undefined "
                                       << "(epsilon=" << epsilons[i] << ")";
    const double slope = (std::log(residual_norms[i + 1]) - std::log(residual_norms[i])) /
                          (std::log(epsilons[i + 1]) - std::log(epsilons[i]));
    EXPECT_NEAR(slope, 2.0, 0.15) << "epsilon pair (" << epsilons[i] << ", " << epsilons[i + 1]
                                   << "), residuals (" << residual_norms[i] << ", " << residual_norms[i + 1] << ")";
  }
}

// Validates the full injected mechanism (Fx*delta_x + Fu*delta_u +
// residualToQpCorrection(...)) when nominal_pred is built with the TRUE
// stage-0 control (delta_u_k == 0 exactly, i.e. u_actual == u_ref -- not the
// degenerate delta_x_k == 0 case, so R_ref_next does NOT cancel against
// itself in the rotation composition and the test can still catch a sign
// error there). This isolates exactly the two things residualToQpCorrection
// actually claims to get right: the Fx/Fu Jacobian truncation (already
// characterized above) and composeFromResidual's exact SO3 round-trip (no
// linearization at all). Must converge at second order (slope ~2 on log-log
// residual-vs-epsilon), same rigor as the Jacobian test above. A previous
// version of residualToQpCorrection used a hand-derived linearized rotation
// term instead of composeFromResidual's exact composition and was caught
// here with a slope of ~1, not ~2 -- i.e. genuinely wrong, not just this
// test being too strict. If this regresses, do not fly it -- fix the
// correction formula until this passes again.
//
// A SEPARATE, known and bounded (not asymptotically vanishing) error source
// is covered by the BoundedControlSubstitutionError test below -- read its
// comment for why that one is a magnitude bound, not a convergence check.
TEST(ResidualQpInjection, MatchesSecondOrderTruncationWithExactControl)
{
  const double dt = 0.05;

  NominalState x_ref;
  x_ref.p = Eigen::Vector3d(1.0, -0.5, 1.5);
  x_ref.v = Eigen::Vector3d(0.3, 0.2, -0.1);
  x_ref.q = Eigen::Quaterniond(Eigen::AngleAxisd(0.3, Eigen::Vector3d(0.2, 0.5, 1.0).normalized()));
  x_ref.q.normalize();

  NominalInput u_ref;
  u_ref.thracc = ommpc::kGravity * 1.15;
  u_ref.omega = Eigen::Vector3d(0.4, -0.3, 0.15);

  MpcController controller;
  controller.init(makeTestParams());
  controller.setStateMatricesandBounds(0, x_ref.q, u_ref.omega, dt, u_ref.thracc);
  const Eigen::MatrixXd Fx = Eigen::MatrixXd(controller.Fx[0]);

  const NominalState x_ref_next = nominalStep(x_ref, u_ref, dt);

  std::mt19937 rng(20260802);
  std::normal_distribution<double> gauss(0.0, 1.0);
  Eigen::Matrix<double, 9, 1> dir_x;
  Eigen::Matrix<double, 9, 1> d; // fixed, non-shrinking -- a persistent disturbance estimate
  for (int i = 0; i < 9; ++i) dir_x(i) = gauss(rng);
  for (int i = 0; i < 9; ++i) d(i) = gauss(rng);
  dir_x.normalize();
  d.normalize();
  d *= 0.02; // representative windy |d_k| magnitude, see wind_generalization_experiment.md

  const std::vector<double> epsilons = {1e-1, 1e-2, 1e-3, 1e-4};
  std::vector<double> residual_norms;
  residual_norms.reserve(epsilons.size());

  for (double eps : epsilons)
  {
    const Eigen::Matrix<double, 9, 1> delta_x_k = eps * dir_x;
    const NominalState x_actual_k = actualFromError(x_ref, delta_x_k);

    // delta_u_k == 0 exactly: nominal_pred_true and nominal_pred_approx use
    // the SAME control (u_ref), so this test cannot see the control-
    // substitution error at all -- see BoundedControlSubstitutionError below
    // for that.
    const NominalState nominal_pred = nominalStep(x_actual_k, u_ref, dt);
    const NominalState x_true_next = ommpc::composeFromResidual(nominal_pred, d);
    const Eigen::Matrix<double, 9, 1> delta_x_next_true = ommpc::controllerErrorState(x_ref_next, x_true_next);

    const Eigen::Matrix<double, 9, 1> c = residualToQpCorrection(nominal_pred, x_ref_next, d);
    const Eigen::Matrix<double, 9, 1> delta_x_next_predicted = Fx * delta_x_k + c; // Fu*0

    residual_norms.push_back((delta_x_next_true - delta_x_next_predicted).norm());
  }

  for (size_t i = 0; i + 1 < epsilons.size(); ++i)
  {
    ASSERT_GT(residual_norms[i], 0.0) << "residual collapsed to exactly zero, slope undefined "
                                       << "(epsilon=" << epsilons[i] << ")";
    const double slope = (std::log(residual_norms[i + 1]) - std::log(residual_norms[i])) /
                          (std::log(epsilons[i + 1]) - std::log(epsilons[i]));
    EXPECT_NEAR(slope, 2.0, 0.15) << "epsilon pair (" << epsilons[i] << ", " << epsilons[i + 1]
                                   << "), residuals (" << residual_norms[i] << ", " << residual_norms[i + 1] << ")";
  }
}

// Bounds (does not asymptotically vanish) the OTHER approximation
// residualToQpCorrection makes: nominal_pred is built from u_ref, not the
// true stage-0 control u_actual_k = u_ref - delta_u_k, because delta_u_k is
// the QP's own decision variable and is genuinely unknown at
// constraint-build time (before mpc_wrapper_.solve() runs). Composing a
// FIXED, non-shrinking disturbance d with a nominal_pred that's off by
// delta_u_k leaves a leftover error of order (delta_u_k * |d|) -- this is a
// real, deliberately-accepted first-order coupling term, not a bug: it was
// checked with delta_u_k swept to zero alongside delta_x_k (matching
// MatchesSecondOrderTruncationWithExactControl's design above) and the
// residual-vs-epsilon slope came out ~1, not ~2, confirming it does NOT
// vanish at second order. Rather than chase a closed-form correction to
// Fu's own sensitivity to this term (which would reopen the same
// hand-derivation risk that MatchesSecondOrderTruncationWithExactControl's
// git history already caught once), this test instead checks the thing that
// actually matters for flight safety, the same way every other evaluation in
// this project is framed (see evaluate_model.py, evaluate_figure8_windy.py,
// docs/wind_generalization_experiment.md's "% RMS reduction vs. zero-residual
// baseline"): is the corrected prediction still meaningfully BETTER than
// applying no correction at all, at the tracking-error/disturbance scale
// actually observed in this project's own windy flights (~0.2-0.5 m
// position error, |d_k| ~0.0165-0.0177)? A single random operating point
// could get unlucky (the disturbance direction happens to interact badly
// with a specific delta_u_k draw), so this averages many random draws for a
// stable statistic rather than asserting on one.
TEST(ResidualQpInjection, ImprovesOnUncorrectedBaselineAtRealisticScale)
{
  const double dt = 0.05;

  NominalState x_ref;
  x_ref.p = Eigen::Vector3d(1.0, -0.5, 1.5);
  x_ref.v = Eigen::Vector3d(0.3, 0.2, -0.1);
  x_ref.q = Eigen::Quaterniond(Eigen::AngleAxisd(0.3, Eigen::Vector3d(0.2, 0.5, 1.0).normalized()));
  x_ref.q.normalize();

  NominalInput u_ref;
  u_ref.thracc = ommpc::kGravity * 1.15;
  u_ref.omega = Eigen::Vector3d(0.4, -0.3, 0.15);

  MpcController controller;
  controller.init(makeTestParams());
  controller.setStateMatricesandBounds(0, x_ref.q, u_ref.omega, dt, u_ref.thracc);
  const Eigen::MatrixXd Fx = Eigen::MatrixXd(controller.Fx[0]);
  const Eigen::MatrixXd Fu = Eigen::MatrixXd(controller.Fu[0]);

  const NominalState x_ref_next = nominalStep(x_ref, u_ref, dt);

  const double kTrackingErrorScale = 0.3; // representative of observed windy tracking error
  const double kResidualScale = 0.0177;   // observed windy |d_k| mean, wind_generalization_experiment.md
  const int kNumTrials = 200;

  std::mt19937 rng(20260803);
  std::normal_distribution<double> gauss(0.0, 1.0);

  double sum_sq_corrected = 0.0;
  double sum_sq_uncorrected = 0.0;

  for (int trial = 0; trial < kNumTrials; ++trial)
  {
    Eigen::Matrix<double, 9, 1> dir_x;
    Eigen::Matrix<double, 4, 1> dir_u;
    Eigen::Matrix<double, 9, 1> d;
    for (int i = 0; i < 9; ++i) dir_x(i) = gauss(rng);
    for (int i = 0; i < 4; ++i) dir_u(i) = gauss(rng);
    for (int i = 0; i < 9; ++i) d(i) = gauss(rng);
    dir_x.normalize();
    dir_u.normalize();
    d.normalize();
    d *= kResidualScale;

    const Eigen::Matrix<double, 9, 1> delta_x_k = kTrackingErrorScale * dir_x;
    const Eigen::Matrix<double, 4, 1> delta_u_k = kTrackingErrorScale * dir_u;

    const NominalState x_actual_k = actualFromError(x_ref, delta_x_k);
    NominalInput u_actual_k;
    u_actual_k.thracc = u_ref.thracc - delta_u_k(0);
    u_actual_k.omega = u_ref.omega - delta_u_k.segment<3>(1);

    const NominalState nominal_pred_true = nominalStep(x_actual_k, u_actual_k, dt);
    const NominalState x_true_next = ommpc::composeFromResidual(nominal_pred_true, d);
    const Eigen::Matrix<double, 9, 1> delta_x_next_true = ommpc::controllerErrorState(x_ref_next, x_true_next);

    const Eigen::Matrix<double, 9, 1> delta_x_next_uncorrected = Fx * delta_x_k + Fu * delta_u_k;

    const NominalState nominal_pred_approx = nominalStep(x_actual_k, u_ref, dt);
    const Eigen::Matrix<double, 9, 1> c = residualToQpCorrection(nominal_pred_approx, x_ref_next, d);
    const Eigen::Matrix<double, 9, 1> delta_x_next_corrected = delta_x_next_uncorrected + c;

    sum_sq_uncorrected += (delta_x_next_true - delta_x_next_uncorrected).squaredNorm();
    sum_sq_corrected += (delta_x_next_true - delta_x_next_corrected).squaredNorm();
  }

  const double rms_uncorrected = std::sqrt(sum_sq_uncorrected / kNumTrials);
  const double rms_corrected = std::sqrt(sum_sq_corrected / kNumTrials);
  const double reduction_pct = 100.0 * (1.0 - rms_corrected / rms_uncorrected);

  EXPECT_LT(rms_corrected, rms_uncorrected)
      << "corrected RMS (" << rms_corrected << ") vs uncorrected RMS (" << rms_uncorrected
      << ") over " << kNumTrials << " random trials -- correction is not helping on average";
  // Matches this project's own established bar for "does the correction
  // generalize" (docs/wind_generalization_experiment.md), applied here to
  // the injection formula itself rather than the trained network.
  EXPECT_GT(reduction_pct, 30.0) << "RMS reduction vs. uncorrected baseline is only " << reduction_pct
                                  << "%, weaker than expected for an exact-composition correction";
}

// Unlike the ResidualQpInjection cases above (pure math, no solve), this
// runs a real closed-loop simulation through MpcController::execMPC() (OSQP
// solve included) to directly prove the *observable effect* of
// enable_integral_trim, not just an internal-state convergence check: a
// simulated plant hovers under a constant unmodeled disturbance
// acceleration (the controller's internal dynamics model has no knowledge
// of it, same structure as real wind), and steady-state position error is
// compared with the trim off vs. on.
TEST(IntegralTrim, RejectsConstantDisturbanceDuringHover)
{
  const double dt = 0.01; // matches ommpc_node's 100Hz control-loop rate
  const int kSteps = 2000; // 20s simulated
  const int kSteadyStateWindow = 300; // last 3s
  const Eigen::Vector3d kDisturbanceAccel(0.15, 0.1, 0.0); // m/s^2, modest wind-drag-scale, unmodeled

  auto runSim = [&](bool enable_trim) -> double
  {
    Parameter_t p = makeTestParams();
    // Closer to the flight-tuned config/params.yaml (makeTestParams()'s
    // uniform 1.0 Q/R weights are only meant for the Jacobian-parity tests
    // above and are not representative of the actual closed-loop dynamics
    // this test needs).
    p.Q_pos_xy = 2.0;
    p.Q_pos_z = 2.0;
    p.Q_velocity = 1.0;
    p.Q_attitude_rp = 1.0;
    p.Q_attitude_yaw = 10.0;
    p.R_thrust = 0.01;
    p.R_pitchroll = 0.01;
    p.R_yaw = 0.1;
    p.enable_integral_trim = enable_trim;
    p.integral_trim_tau = 3.0; // shorter than the 10.0 flight default so it
                                // converges within this short simulated run
    p.residual_clip_norm = 0.08; // the flight-validated default -- keeps the
                                  // correction within the regime the QP's
                                  // small-angle Fx/Fu linearization is valid

    const Eigen::Vector3d kHoverRefP(0.0, 0.0, 1.0);

    MpcController controller;
    controller.init(p);

    // Start already at the hover setpoint (takeoff/settling is a separate
    // concern from disturbance rejection -- starting here avoids an initial
    // transient swamping the steady-state measurement below).
    ommpc::NominalState x_true;
    x_true.p = kHoverRefP;
    x_true.v = Eigen::Vector3d::Zero();
    x_true.q = Eigen::Quaterniond::Identity();

    rclcpp::Time t(0, 0, RCL_ROS_TIME);
    double sum_sq_steady_state = 0.0;
    int count_steady_state = 0;

    for (int i = 0; i < kSteps; ++i)
    {
      // solve() frees A_data_/A_indices_/A_indptr_ at the end of every call
      // (see MpcWrapper::solve) -- production code (ommpc_node.cpp's
      // controlLoop) always calls a setXReference() immediately before
      // every execMPC(), which rebuilds them fresh via buildConstraintMatrix
      // (mallocs new arrays, no dangling reuse). Match that here: a hover
      // setpoint frozen at the origin re-linearized identically every
      // cycle is still the right scenario (fixed hover point), it just has
      // to be re-issued each iteration to keep the QP buffers valid.
      controller.setHoverReference(Eigen::Vector4d(0.0, 0.0, 1.0, 0.0));

      Odom_Data_t odom;
      odom.p = x_true.p;
      odom.v = x_true.v;
      odom.q = x_true.q;
      odom.w = Eigen::Vector3d::Zero();

      Controller_Output_t u;
      c_int osqp_status;
      const bool ok = controller.execMPC(odom, u, t, osqp_status);
      EXPECT_TRUE(ok) << "MPC failed to solve at step " << i << " (enable_trim=" << enable_trim << ")";
      if (!ok)
      {
        return std::numeric_limits<double>::quiet_NaN();
      }

      ommpc::NominalInput u_applied;
      u_applied.thracc = u.thrust * controller.thr2acc;
      u_applied.omega = u.bodyrates;

      ommpc::NominalState x_next = ommpc::nominalStep(x_true, u_applied, dt);
      x_next.v += dt * kDisturbanceAccel; // unmodeled external acceleration
      x_true = x_next;

      if (!x_true.p.allFinite() || !x_true.v.allFinite())
      {
        ADD_FAILURE() << "x_true diverged (non-finite) at step " << i << " (enable_trim=" << enable_trim
                       << "): p=" << x_true.p.transpose() << " v=" << x_true.v.transpose()
                       << " u.thrust=" << u.thrust << " u.bodyrates=" << u.bodyrates.transpose();
        return std::numeric_limits<double>::quiet_NaN();
      }

      t = t + rclcpp::Duration::from_seconds(dt);

      if (i >= kSteps - kSteadyStateWindow)
      {
        sum_sq_steady_state += (x_true.p - kHoverRefP).squaredNorm();
        count_steady_state++;
      }
    }

    return std::sqrt(sum_sq_steady_state / count_steady_state);
  };

  const double rms_without_trim = runSim(false);
  const double rms_with_trim = runSim(true);
  ASSERT_FALSE(std::isnan(rms_without_trim)) << "sim without trim hit an MPC solve failure, see above";
  ASSERT_FALSE(std::isnan(rms_with_trim)) << "sim with trim hit an MPC solve failure, see above";

  EXPECT_LT(rms_with_trim, rms_without_trim)
      << "rms_with_trim=" << rms_with_trim << ", rms_without_trim=" << rms_without_trim;
  // Not just "some improvement" -- the whole point of the trim is closing a
  // steady-state offset, so demand it closes most of the gap.
  EXPECT_LT(rms_with_trim, 0.3 * rms_without_trim)
      << "integral trim only reduced steady-state error from " << rms_without_trim << " to "
      << rms_with_trim << " -- weaker than expected, retune before flying";
}

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
