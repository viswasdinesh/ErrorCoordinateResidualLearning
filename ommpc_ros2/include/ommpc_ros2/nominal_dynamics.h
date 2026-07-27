#ifndef OMMPC_NOMINAL_DYNAMICS_H
#define OMMPC_NOMINAL_DYNAMICS_H

#include <Eigen/Eigen>

#include "ommpc_ros2/so3_math.h"

// Single source of truth for the nonlinear one-step nominal map whose
// Jacobians are hand-derived in MpcController::setStateMatricesandBounds()
// (ommpc_controller.hpp). See docs/residual_conventions.md for the
// derivation and for why this must stay in sync with
// scripts/residuals.py::nominal_step.
namespace ommpc
{

constexpr double kGravity = 9.81;

struct NominalState
{
  Eigen::Vector3d p;
  Eigen::Vector3d v;
  Eigen::Quaterniond q;
};

// u = [thracc, omega]. thracc: mass-normalized thrust acceleration along
// body +Z [m/s^2]. omega: commanded body rate [rad/s], body frame.
struct NominalInput
{
  double thracc;
  Eigen::Vector3d omega;
};

// p_{k+1} = p_k + dt*v_k
// v_{k+1} = v_k + dt*(R_k*e3*thracc_k - g*e3)
// R_{k+1} = R_k * Exp(omega_k*dt)
inline NominalState nominalStep(const NominalState &x, const NominalInput &u, double dt)
{
  NominalState x_next;
  const Eigen::Matrix3d R = x.q.toRotationMatrix();
  const Eigen::Vector3d e3(0.0, 0.0, 1.0);

  x_next.p = x.p + dt * x.v;
  x_next.v = x.v + dt * (R * e3 * u.thracc - kGravity * e3);
  x_next.q = Eigen::Quaterniond(R * SO3::exp(u.omega * dt));
  x_next.q.normalize();
  return x_next;
}

// boxminus(x_a, x_b) = [p_a - p_b ; v_a - v_b ; Log(R_a^T R_b)] in R^9.
// Right-perturbation difference: R_b = R_a * Exp(boxminus(x_a,x_b).tail<3>()).
// This is the residual-target operator (docs/residual_conventions.md §4) —
// NOT the same argument-order convention as the controller's internal error
// state delta_x (docs §1), which is asymmetric between its vector and
// rotation blocks. Do not conflate the two.
inline Eigen::Matrix<double, 9, 1> boxminus(const NominalState &a, const NominalState &b)
{
  Eigen::Matrix<double, 9, 1> out;
  out.segment<3>(0) = a.p - b.p;
  out.segment<3>(3) = a.v - b.v;
  out.segment<3>(6) = SO3::log(a.q.toRotationMatrix().transpose() * b.q.toRotationMatrix());
  return out;
}

} // namespace ommpc

#endif // OMMPC_NOMINAL_DYNAMICS_H
