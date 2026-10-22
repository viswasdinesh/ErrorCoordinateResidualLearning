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

// The controller's own (asymmetric) error state, docs/residual_conventions.md
// §1: delta_p = p_ref-p_actual, delta_v = v_ref-v_actual, delta_theta =
// Log(R_actual^T R_ref). NOT the same argument-order convention as
// boxminus() above (§4) -- see the comment there. Matches
// scripts/residuals.py::controller_error_state and the delta_x_init
// computation in MpcController::execMPC.
inline Eigen::Matrix<double, 9, 1> controllerErrorState(const NominalState &x_ref, const NominalState &x_actual)
{
  Eigen::Matrix<double, 9, 1> delta;
  delta.segment<3>(0) = x_ref.p - x_actual.p;
  delta.segment<3>(3) = x_ref.v - x_actual.v;
  delta.segment<3>(6) = SO3::log(x_actual.q.toRotationMatrix().transpose() * x_ref.q.toRotationMatrix());
  return delta;
}

// Exact inverse of boxminus(): given a nominal-model prediction and a §4
// residual d = boxminus(x_true, nominal_pred), reconstructs x_true. Used to
// fold a predicted residual back into a concrete corrected state so its
// effect on the QP's §1 error-state constraint can be computed exactly
// (docs/residual_conventions.md "Relationship to Task 4's affine QP term")
// rather than via a hand-derived linearized Jacobian -- this composition is
// exact (no small-angle approximation) for any d within Log()'s valid domain:
//   d.p = x_true.p - nominal_pred.p           => x_true.p = nominal_pred.p + d.p
//   d.v = x_true.v - nominal_pred.v           => x_true.v = nominal_pred.v + d.v
//   d.theta = Log(R_true^T R_nominal_pred)    => R_true = R_nominal_pred * Exp(-d.theta)
inline NominalState composeFromResidual(const NominalState &nominal_pred, const Eigen::Matrix<double, 9, 1> &d)
{
  NominalState x_true;
  x_true.p = nominal_pred.p + d.segment<3>(0);
  x_true.v = nominal_pred.v + d.segment<3>(3);
  const Eigen::Matrix3d R_true = nominal_pred.q.toRotationMatrix() * SO3::exp(-d.segment<3>(6));
  x_true.q = Eigen::Quaterniond(R_true);
  x_true.q.normalize();
  return x_true;
}

// The QP-injection correction (docs/residual_conventions.md "Relationship to
// Task 4's affine QP term"): given the nominal one-step prediction from the
// current MEASURED state (nominal_pred -- computed with the reference
// control u_ref as a zeroth-order stand-in for the QP's not-yet-solved
// stage-0 decision variable, since that's what's actually available at
// constraint-build time) and a predicted §4 residual d, returns the exact
// additive constant c such that controllerErrorState(x_ref_next,
// nominal_pred) + c exactly equals controllerErrorState(x_ref_next,
// composeFromResidual(nominal_pred, d)) -- i.e. the exact effect of folding
// d into the predicted next state, via composeFromResidual's exact SO3
// composition rather than a hand-derived linearized Jacobian (an earlier
// version used a linearized closed form for the rotation block and was
// caught wrong by test/test_nominal_dynamics.cpp's
// MatchesSecondOrderTruncationWithExactControl case -- see git history).
// The u_ref-for-u_0 substitution is a separate, deliberately-accepted
// approximation, bounded (not asymptotically eliminated) by that same test
// file's ImprovesOnUncorrectedBaselineAtRealisticScale case.
inline Eigen::Matrix<double, 9, 1> residualToQpCorrection(const NominalState &nominal_pred,
                                                           const NominalState &x_ref_next,
                                                           const Eigen::Matrix<double, 9, 1> &d)
{
  const NominalState x_true_pred = composeFromResidual(nominal_pred, d);
  return controllerErrorState(x_ref_next, x_true_pred) - controllerErrorState(x_ref_next, nominal_pred);
}

} // namespace ommpc

#endif // OMMPC_NOMINAL_DYNAMICS_H
