// Validates that MpcController::setStateMatricesandBounds() produces Fx/Fu
// that are the true first-order Jacobians of ommpc::nominalStep(), under the
// controller's own (asymmetric) error-state convention documented in
// docs/residual_conventions.md §1. If this test fails after touching either
// nominal_dynamics.h or setStateMatricesandBounds(), the two have drifted
// out of sync and neither should be trusted as "the nominal model".
#include <gtest/gtest.h>

#include <cmath>
#include <random>
#include <vector>

#include <Eigen/Eigen>

#include "ommpc_ros2/nominal_dynamics.h"
#include "ommpc_ros2/ommpc_controller.hpp"

using ommpc::NominalInput;
using ommpc::NominalState;
using ommpc::nominalStep;

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

// Same convention, forward direction: delta_x = ref boxminus actual.
Eigen::Matrix<double, 9, 1> errorBetween(const NominalState &ref, const NominalState &actual)
{
  Eigen::Matrix<double, 9, 1> delta;
  delta.segment<3>(0) = ref.p - actual.p;
  delta.segment<3>(3) = ref.v - actual.v;
  delta.segment<3>(6) = SO3::log(actual.q.toRotationMatrix().transpose() * ref.q.toRotationMatrix());
  return delta;
}

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
    const Eigen::Matrix<double, 9, 1> delta_x_next_true = errorBetween(x_next_ref, x_next_actual);
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

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
