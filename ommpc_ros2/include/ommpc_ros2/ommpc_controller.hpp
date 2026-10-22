#ifndef OMMPC_CONTROLLER_HPP
#define OMMPC_CONTROLLER_HPP

#include <Eigen/Eigen>
#include <osqp/osqp.h>
#include <rclcpp/rclcpp.hpp>
#include <vector>
#include <memory>
#include <queue>
#include <deque>
#include <cmath>
#include <iostream>
#include <algorithm>
#include <chrono>

#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <px4_msgs/msg/sensor_combined.hpp>
#include "ommpc_ros2/polynomial_trajectory.h"
#include "ommpc_ros2/so3_math.h"
#include "ommpc_ros2/msg/poly_traj.hpp"
#include "ommpc_ros2/nominal_dynamics.h"
#include "ommpc_ros2/residual_model_ecrl.h"
#include "ommpc_ros2/residual_model_ambient.h"

static constexpr int nstep = 20;  // N steps
static constexpr int nx = 9;      // dimension of error state (δp, δv, δR)
static constexpr int nstate = 10; // dimension of state (pos quat vel)
static constexpr int nu = 4;      // dimension of control input (thrust omg)

struct Odom_Data_t
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Eigen::Vector3d p;
  Eigen::Vector3d v;
  Eigen::Quaterniond q;
  Eigen::Vector3d w;

  nav_msgs::msg::Odometry msg;
  rclcpp::Time rcv_stamp{0, 0, RCL_ROS_TIME};
  bool recv_new_msg;

  Odom_Data_t()
  {
    recv_new_msg = false;
  }

  void feed(const nav_msgs::msg::Odometry::SharedPtr pMsg, bool enu_frame, bool vel_in_body, rclcpp::Time now_time)
  {
    msg = *pMsg;
    rcv_stamp = now_time;
    recv_new_msg = true;

    p(0) = msg.pose.pose.position.x;
    p(1) = msg.pose.pose.position.y;
    p(2) = msg.pose.pose.position.z;

    v(0) = msg.twist.twist.linear.x;
    v(1) = msg.twist.twist.linear.y;
    v(2) = msg.twist.twist.linear.z;

    q.w() = msg.pose.pose.orientation.w;
    q.x() = msg.pose.pose.orientation.x;
    q.y() = msg.pose.pose.orientation.y;
    q.z() = msg.pose.pose.orientation.z;

    w(0) = msg.twist.twist.angular.x;
    w(1) = msg.twist.twist.angular.y;
    w(2) = msg.twist.twist.angular.z;

    if (!enu_frame)
    {
      Eigen::Matrix3d R_mid;
      R_mid << 0.0, 1.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, -1.0;
      Eigen::Quaterniond q_mid(R_mid);
      Eigen::Quaterniond q_body(0.0, 1.0, 0.0, 0.0);

      p = q_mid.toRotationMatrix() * p;
      v = q_mid.toRotationMatrix() * v;
      q = q_mid * q * q_body;
      q.normalize();
      w(1) = -w(1);
      w(2) = -w(2);
    }

    if (vel_in_body)
      v = q.toRotationMatrix() * v;
  }

  void feed(const px4_msgs::msg::VehicleOdometry::SharedPtr pMsg, bool enu_frame, bool vel_in_body, rclcpp::Time now_time)
  {
    rcv_stamp = now_time;
    recv_new_msg = true;

    p(0) = pMsg->position[0];
    p(1) = pMsg->position[1];
    p(2) = pMsg->position[2];

    v(0) = pMsg->velocity[0];
    v(1) = pMsg->velocity[1];
    v(2) = pMsg->velocity[2];

    q.w() = pMsg->q[0];
    q.x() = pMsg->q[1];
    q.y() = pMsg->q[2];
    q.z() = pMsg->q[3];

    w(0) = pMsg->angular_velocity[0];
    w(1) = pMsg->angular_velocity[1];
    w(2) = pMsg->angular_velocity[2];

    if (!enu_frame)
    {
      Eigen::Matrix3d R_mid;
      R_mid << 0.0, 1.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, -1.0;
      Eigen::Quaterniond q_mid(R_mid);
      Eigen::Quaterniond q_body(0.0, 1.0, 0.0, 0.0);

      p = q_mid.toRotationMatrix() * p;
      v = q_mid.toRotationMatrix() * v;
      q = q_mid * q * q_body;
      q.normalize();
      w(1) = -w(1);
      w(2) = -w(2);
    }

    if (vel_in_body)
      v = q.toRotationMatrix() * v;
  }
};

struct Controller_Output_t
{
  // Body rates in body frame
  Eigen::Vector3d bodyrates; // [rad/s]

  // Collective mass normalized thrust
  double thrust;
};

struct Imu_Data_t
{
  Eigen::Quaterniond q;
  Eigen::Vector3d w;
  Eigen::Vector3d a;

  sensor_msgs::msg::Imu msg;
  rclcpp::Time rcv_stamp{0, 0, RCL_ROS_TIME};
  bool recv_new_msg;

  Imu_Data_t()
  {
    recv_new_msg = false;
  }
  void feed(const sensor_msgs::msg::Imu::SharedPtr pMsg, bool enu_frame, rclcpp::Time now_time)
  {
    msg = *pMsg;
    rcv_stamp = now_time;
    recv_new_msg = true;

    a(0) = msg.linear_acceleration.x;
    a(1) = msg.linear_acceleration.y;
    a(2) = msg.linear_acceleration.z;

    q.x() = msg.orientation.x;
    q.y() = msg.orientation.y;
    q.z() = msg.orientation.z;
    q.w() = msg.orientation.w;

    w(0) = msg.angular_velocity.x;
    w(1) = msg.angular_velocity.y;
    w(2) = msg.angular_velocity.z;

    if (!enu_frame)
    {
      Eigen::Matrix3d R_mid;
      R_mid << 0.0, 1.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, -1.0;
      Eigen::Quaterniond q_mid(R_mid);

      a = q_mid.toRotationMatrix() * a;
      q = q_mid * q * q_mid;
      q.normalize();
      w = q_mid.toRotationMatrix() * w;
    }
  }

  void feed(const px4_msgs::msg::SensorCombined::SharedPtr pMsg, bool enu_frame, rclcpp::Time now_time)
  {
    rcv_stamp = now_time;
    recv_new_msg = true;

    a(0) = pMsg->accelerometer_m_s2[0];
    a(1) = pMsg->accelerometer_m_s2[1];
    a(2) = pMsg->accelerometer_m_s2[2];

    q.setIdentity();
    w.setZero();

    if (!enu_frame)
    {
      Eigen::Matrix3d R_mid;
      R_mid << 0.0, 1.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, -1.0;
      Eigen::Quaterniond q_mid(R_mid);

      a = q_mid.toRotationMatrix() * a;
    }
  }
};

struct oneTraj_Data_t
{
public:
  rclcpp::Time traj_start_time{0, 0, RCL_ROS_TIME};
  rclcpp::Time traj_end_time{0, 0, RCL_ROS_TIME};
  Trajectory traj;
  Trajectory yaw_traj;
};

class Trajectory_Data_t
{
public:
  rclcpp::Time total_traj_start_time{0, 0, RCL_ROS_TIME};
  rclcpp::Time total_traj_end_time{0, 0, RCL_ROS_TIME};
  int exec_traj = 0; // use for aborting the trajectory, 0 means no trajectory is executing
                     // -1 means the trajectory is aborting, 1 means the trajectory is executing
  std::deque<oneTraj_Data_t> traj_queue;

  Trajectory_Data_t()
  {
    exec_traj = 0;
  }
  void adjust_end_time()
  {
    if (traj_queue.size() < 2)
    {
      return;
    }
    for (auto it = traj_queue.begin(); it != (traj_queue.end() - 1); it++)
    {
      it->traj_end_time = (it + 1)->traj_start_time;
    }
  }
  // adapted for ego_planner_v2 (with its traj server)
  void feed_from_traj_utils(const ommpc_ros2::msg::PolyTraj::SharedPtr pMsg, rclcpp::Time now_time)
  {
    // #1. try to execute the action
    const ommpc_ros2::msg::PolyTraj &traj = *pMsg;

    if (traj.order >= 3)
    {
      std::cout << "[MPCCtrl] Loading the trajectory." << std::endl;
      if (traj.traj_id < 1)
      {
        std::cerr << "[MPCCtrl] The trajectory_id must start from 1" << std::endl;
        return;
      }
      oneTraj_Data_t traj_data;
      traj_data.traj_start_time = rclcpp::Time(pMsg->start_time);
      double t_total = 0;

      for (int i = 0; i < int(traj.duration.size()); ++i)
      {
        int num_dim = 3;
        int num_order = traj.order;
        t_total += traj.duration[i];

        Eigen::MatrixXd piece_coef;
        piece_coef.resize(num_dim, num_order + 1);
        for (int j = 0; j <= num_order; j++)
        {
          piece_coef(0, j) = traj.coef_x[i * (num_order + 1) + j];
          piece_coef(1, j) = traj.coef_y[i * (num_order + 1) + j];
          piece_coef(2, j) = traj.coef_z[i * (num_order + 1) + j];
        }
        traj_data.traj.emplace_back(traj.duration[i], piece_coef);
      }

      traj_data.traj_end_time = rclcpp::Time(pMsg->start_time) + rclcpp::Duration::from_seconds(t_total);

      // If traj_start_time is after current time, push_back the traj; else push_front
      if (now_time < traj_data.traj_start_time) // Future traj
      {
        // A future trajectory
        while ((!traj_queue.empty()) && traj_queue.back().traj_start_time > traj_data.traj_start_time)
        {
          traj_queue.pop_back();
        }
        traj_queue.push_back(traj_data);
        total_traj_end_time = traj_queue.back().traj_end_time;
        total_traj_start_time = traj_queue.front().traj_start_time;
      }
      else // older traj
      {
        while ((!traj_queue.empty()) && traj_queue.front().traj_start_time < traj_data.traj_start_time)
        {
          traj_queue.pop_front();
        }
        traj_queue.push_front(traj_data);
        total_traj_end_time = traj_queue.back().traj_end_time;
        total_traj_start_time = traj_queue.front().traj_start_time;
      }

      exec_traj = 1;
    }
    else
    {
      exec_traj = -1;
    }
  }
};

// MPC solver

struct Solution
{
  std::vector<Eigen::VectorXd> delta_u; // optimal control sequence
  std::vector<Eigen::VectorXd> delta_x; // optimal state sequence
  double optimal_cost;
};

class MpcWrapper
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  MpcWrapper()
  {
    // variables: [δx0, δu0, δx1, δu1, ..., δx{nstep-1}, δu{nstep-1}, δx{nstep}]
    total_vars_ = (nstep + 1) * nx + nstep * nu;

    // constraints: error dynamics (nstep) + init (1) + control bounds (nstep)
    total_constraints_ = nstep * nx + nx + nstep * nu;
  }

  ~MpcWrapper()
  {
    free(P_data_);
    free(P_indices_);
    free(P_indptr_);
  }

  void setInitValue(const Eigen::VectorXd &x0)
  {
    for (int i = 0; i < nx; ++i)
    {
      l_[i] = x0[i];
      u_[i] = x0[i];
    }
  }

  void setDesiredStart(const Eigen::VectorXd &xdes, const Eigen::VectorXd &udes)
  {
    x_des_start = xdes;
    u_des_start = udes;
  }
  void getDesiredStart(Eigen::VectorXd &xdes, Eigen::VectorXd &udes)
  {
    xdes = x_des_start;
    udes = u_des_start;
  }

  // solve QP problem. status_val_out receives the raw OSQP work->info->status_val
  // (e.g. OSQP_SOLVED, OSQP_MAX_ITER_REACHED, OSQP_PRIMAL_INFEASIBLE, ...) for
  // logging/diagnostics; it is left at OSQP_UNSOLVED-ish sentinel -1 if
  // osqp_setup() itself failed before a status was ever produced.
  bool solve(Solution &sol, c_int &status_val_out)
  {
    status_val_out = -1;
    // create OSQP data
    OSQPData osqp_data;
    OSQPSettings osqp_settings;
    OSQPWorkspace *work = nullptr;

    // fill in the data
    osqp_data.n = total_vars_;
    osqp_data.m = total_constraints_;
    osqp_data.P = csc_matrix(total_vars_, total_vars_, P_nnz_, P_data_, P_indices_, P_indptr_);
    osqp_data.q = q_.data();
    osqp_data.A = csc_matrix(total_constraints_, total_vars_, A_nnz_, A_data_, A_indices_, A_indptr_);
    osqp_data.l = l_.data();
    osqp_data.u = u_.data();

    // set OSQP params
    osqp_set_default_settings(&osqp_settings);
    osqp_settings.polish = true;
    osqp_settings.verbose = false;

    // create workspace and solve
    c_int exit_code = osqp_setup(&work, &osqp_data, &osqp_settings);
    if (exit_code != 0)
    {
      free(A_data_);
      free(A_indices_);
      free(A_indptr_);
      return false;
    }

    osqp_solve(work);

    // check solution status
    bool success = false;
    if (work != nullptr && work->info != nullptr)
    {
      status_val_out = work->info->status_val;
      // Determine success based on OSQP solution status
      if (work->info->status_val == OSQP_SOLVED ||
          work->info->status_val == OSQP_SOLVED_INACCURATE)
      {
        success = true;

        // Extract solution
        extractSolution(work->solution->x, sol);
        sol.optimal_cost = work->info->obj_val;
      }
    }

    // Clean up
    if (work != nullptr)
    {
      osqp_cleanup(work);
    }

    free(A_data_);
    free(A_indices_);
    free(A_indptr_);

    return success;
  }

  // Set weight matrix, only once
  void buildHessianMatrix(
      const Eigen::Matrix<double, nx, nx> &Q_diag,
      const Eigen::Matrix<double, nu, nu> &R_diag,
      const double state_cost_exponential,
      const double input_cost_exponential)
  {

    // Hessian matrix is diagonal, directly construct CSC format
    P_nnz_ = total_vars_;

    P_data_ = (c_float *)malloc(P_nnz_ * sizeof(c_float));
    P_indices_ = (c_int *)malloc(P_nnz_ * sizeof(c_int));
    P_indptr_ = (c_int *)malloc((total_vars_ + 1) * sizeof(c_int));

    int var_idx = 0;

    // Set column pointers
    for (int col = 0; col <= total_vars_; ++col)
    {
      P_indptr_[col] = col;
    }

    for (int k = 0; k < nstep; ++k)
    {
      // δxk part
      double state_decay = std::exp(-(double)k / (double)nstep * state_cost_exponential);
      for (int i = 0; i < nx; ++i)
      {
        P_indices_[var_idx] = var_idx;
        P_data_[var_idx] = Q_diag(i, i) * state_decay;
        var_idx++;
      }

      double input_decay = std::exp(-(double)k / (double)nstep * input_cost_exponential);
      // δuk part
      for (int i = 0; i < nu; ++i)
      {
        P_indices_[var_idx] = var_idx;
        P_data_[var_idx] = R_diag(i, i) * input_decay;
        var_idx++;
      }
    }

    const Eigen::Matrix<double, nx, nx> P_final_diag = Q_diag * std::exp(-state_cost_exponential);

    // δx{nstep} (final) part
    for (int i = 0; i < nx; ++i)
    {
      P_indices_[var_idx] = var_idx;
      P_data_[var_idx] = P_final_diag(i, i);
      var_idx++;
    }
  }

  // Set constraint matrix
  void buildConstraintMatrix(
      const std::vector<Eigen::SparseMatrix<double>> Fx,
      const std::vector<Eigen::SparseMatrix<double>> Fu)
  {
    // Calculate the number of non-zero elements
    A_nnz_ = nx; // Initial condition constraints

    // non-zero elements of dynamics constraints
    for (int k = 0; k < nstep; ++k)
    {
      A_nnz_ += Fx[k].nonZeros() + Fu[k].nonZeros() + nx;
    }

    // non-zero elements of control constraint (each constraint has 1 non-zero element)
    A_nnz_ += nstep * nu;

    // Allocate memory
    A_data_ = (c_float *)malloc(A_nnz_ * sizeof(c_float));
    A_indices_ = (c_int *)malloc(A_nnz_ * sizeof(c_int));
    A_indptr_ = (c_int *)malloc((total_vars_ + 1) * sizeof(c_int));

    // Initialize column pointers
    std::vector<int> col_nnz(total_vars_, 0);

    // First, count non-zero elements per column
    // Initial condition constraints: columns corresponding to δx0
    for (int i = 0; i < nx; ++i)
    {
      int col = i;
      col_nnz[col]++;
    }

    // Dynamics constraints
    int constraint_idx = nx;
    for (int k = 0; k < nstep; ++k)
    {
      int xk_offset = k * (nx + nu);
      int uk_offset = xk_offset + nx;
      int xkp1_offset = (k + 1) * (nx + nu);

      // -Fx[k] part (corresponding to δxk)
      for (int j = 0; j < Fx[k].outerSize(); ++j)
      {
        for (Eigen::SparseMatrix<double>::InnerIterator it(Fx[k], j); it; ++it)
        {
          int col = xk_offset + it.col();
          col_nnz[col]++;
        }
      }

      // -Fu[k] part (corresponding to δuk)
      for (int j = 0; j < Fu[k].outerSize(); ++j)
      {
        for (Eigen::SparseMatrix<double>::InnerIterator it(Fu[k], j); it; ++it)
        {
          int col = uk_offset + it.col();
          col_nnz[col]++;
        }
      }

      // I part (corresponding to δx{k+1})
      for (int i = 0; i < nx; ++i)
      {
        int col = xkp1_offset + i;
        col_nnz[col]++;
      }

      constraint_idx += nx;
    }

    // Control constraints
    for (int k = 0; k < nstep; ++k)
    {
      int uk_offset = k * (nx + nu) + nx;

      for (int i = 0; i < nu; ++i)
      {
        int col = uk_offset + i;
        col_nnz[col]++;
      }
    }

    // Set column pointers
    A_indptr_[0] = 0;
    for (int col = 0; col < total_vars_; ++col)
    {
      A_indptr_[col + 1] = A_indptr_[col] + col_nnz[col];
    }

    // Then, fill in data
    std::vector<int> col_pos(total_vars_, 0);
    std::vector<c_float> temp_data(A_nnz_);
    std::vector<c_int> temp_indices(A_nnz_);

    // Reset constraint index
    constraint_idx = 0;

    // Initial condition constraints
    for (int i = 0; i < nx; ++i)
    {
      int col = i;
      int pos = A_indptr_[col] + col_pos[col];
      temp_data[pos] = 1.0;
      temp_indices[pos] = constraint_idx++;
      col_pos[col]++;
    }

    // Dynamics constraints
    for (int k = 0; k < nstep; ++k)
    {
      int xk_offset = k * (nx + nu);
      int uk_offset = xk_offset + nx;
      int xkp1_offset = (k + 1) * (nx + nu);

      // -Fx[k]
      for (int j = 0; j < Fx[k].outerSize(); ++j)
      {
        for (Eigen::SparseMatrix<double>::InnerIterator it(Fx[k], j); it; ++it)
        {
          int col = xk_offset + it.col();
          int pos = A_indptr_[col] + col_pos[col];
          temp_data[pos] = -it.value();
          temp_indices[pos] = constraint_idx + it.row();
          col_pos[col]++;
        }
      }

      // -Fu[k]
      for (int j = 0; j < Fu[k].outerSize(); ++j)
      {
        for (Eigen::SparseMatrix<double>::InnerIterator it(Fu[k], j); it; ++it)
        {
          int col = uk_offset + it.col();
          int pos = A_indptr_[col] + col_pos[col];
          temp_data[pos] = -it.value();
          temp_indices[pos] = constraint_idx + it.row();
          col_pos[col]++;
        }
      }

      // I
      for (int i = 0; i < nx; ++i)
      {
        int col = xkp1_offset + i;
        int pos = A_indptr_[col] + col_pos[col];
        temp_data[pos] = 1.0;
        temp_indices[pos] = constraint_idx + i;
        col_pos[col]++;
      }

      constraint_idx += nx;
    }

    // Control constraints
    // int control_constraint_start = constraint_idx;
    for (int k = 0; k < nstep; ++k)
    {
      int uk_offset = k * (nx + nu) + nx;

      for (int i = 0; i < nu; ++i)
      {
        int col = uk_offset + i;
        int pos = A_indptr_[col] + col_pos[col];
        temp_data[pos] = 1.0;
        temp_indices[pos] = constraint_idx++;
        col_pos[col]++;
      }
    }

    // Copy to OSQP arrays
    memcpy(A_data_, temp_data.data(), A_nnz_ * sizeof(c_float));
    memcpy(A_indices_, temp_indices.data(), A_nnz_ * sizeof(c_int));
  }

  // Build constraint right-hand side vectors
  void buildConstraintVectors(
      const std::vector<Eigen::VectorXd> &u_min,
      const std::vector<Eigen::VectorXd> &u_max)
  {
    q_.resize(total_vars_, 0.0);
    l_.resize(total_constraints_, 0.0);
    u_.resize(total_constraints_, 0.0);

    // Dynamics constraints (equality constraints: l=0, u=0)
    int offset = nx;
    std::fill(l_.begin() + offset, l_.begin() + offset + nstep * nx, 0.0);
    std::fill(u_.begin() + offset, u_.begin() + offset + nstep * nx, 0.0);

    // Control constraints
    offset += nstep * nx;
    for (int k = 0; k < nstep; ++k)
    {
      for (int i = 0; i < nu; ++i)
      {
        l_[offset] = u_min[k][i];
        u_[offset] = u_max[k][i];
        offset++;
      }
    }
  }

  // Overwrites the dynamics-equality-constraint RHS (rows [nx, nx+nstep*nx),
  // currently pinned to zero by buildConstraintVectors()) with a per-cycle
  // residual-correction bias c, applied identically to every stage --
  // offset-free-MPC-style: the disturbance is assumed roughly constant over
  // the horizon, re-estimated fresh every control cycle. Must be called
  // AFTER buildConstraintVectors() (which resets these rows to zero) and
  // BEFORE solve(). See MpcController::execMPC and
  // docs/residual_conventions.md "Relationship to Task 4's affine QP term".
  void applyResidualCorrection(const Eigen::Matrix<double, nx, 1> &c)
  {
    int offset = nx;
    for (int k = 0; k < nstep; ++k)
    {
      for (int i = 0; i < nx; ++i)
      {
        l_[offset + i] = c(i);
        u_[offset + i] = c(i);
      }
      offset += nx;
    }
  }

private:
  // dimension of the problem
  int total_vars_;
  int total_constraints_;

  // data in OSQP CSC format matrix
  c_float *P_data_ = nullptr;
  c_int *P_indices_ = nullptr;
  c_int *P_indptr_ = nullptr;
  int P_nnz_ = 0;

  c_float *A_data_ = nullptr;
  c_int *A_indices_ = nullptr;
  c_int *A_indptr_ = nullptr;
  int A_nnz_ = 0;

  // data in vectors
  std::vector<c_float> q_;
  std::vector<c_float> l_;
  std::vector<c_float> u_;

  // desired state, for init error setting
  Eigen::VectorXd x_des_start;
  Eigen::VectorXd u_des_start;

  // Extract solution
  void extractSolution(const c_float *solution, Solution &sol)
  {
    sol.delta_u.resize(nstep);
    sol.delta_x.resize(nstep + 1);

    for (int k = 0; k < nstep; ++k)
    {
      int x_offset = k * (nx + nu);
      int u_offset = x_offset + nx;

      sol.delta_x[k] = Eigen::VectorXd(nx);
      sol.delta_u[k] = Eigen::VectorXd(nu);

      for (int i = 0; i < nx; ++i)
      {
        sol.delta_x[k][i] = solution[x_offset + i];
      }

      for (int i = 0; i < nu; ++i)
      {
        sol.delta_u[k][i] = solution[u_offset + i];
      }
    }

    // Final state
    int final_offset = nstep * (nx + nu);
    sol.delta_x[nstep] = Eigen::VectorXd(nx);
    for (int i = 0; i < nx; ++i)
    {
      sol.delta_x[nstep][i] = solution[final_offset + i];
    }
  }
};

struct Parameter_t
{
  double step_T;
  double hover_percent;
  double Q_pos_xy, Q_pos_z, Q_velocity, Q_attitude_rp, Q_attitude_yaw;
  double R_thrust, R_pitchroll, R_yaw;
  double state_cost_exponential, input_cost_exponential;
  double max_bodyrate_xy, max_bodyrate_z, min_thrust, max_thrust;

  bool use_fix_yaw, use_trajectory_ending_pos;

  bool use_ref_txt;
  std::string ref_filename;
  double ref_time_step;

  double takeoff_land_speed, takeoff_height;

  // Off by default: applies the trained `ecrl` residual model's prediction
  // as a per-cycle affine bias on the QP's dynamics-equality constraint
  // (docs/residual_conventions.md "Relationship to Task 4's affine QP
  // term"). See MpcController::execMPC. residual_clip_norm hard-caps the
  // 9-D correction's world-frame norm before it reaches the QP, as a safety
  // net against an out-of-distribution model output injecting a large
  // disturbance into a real-time control loop.
  bool enable_residual_correction = false;
  double residual_clip_norm = 0.08;

  // Off by default: the `ambient` adapter's alternative to
  // enable_residual_correction -- 8-D body-frame velocity/rate/thrust
  // features (residuals.py::ambient_features), no reference trajectory
  // involved, vs. ecrl's 16-D error-coordinate features. Mutually exclusive
  // with enable_residual_correction in practice (execMPC checks
  // enable_residual_correction first, so if both are set ecrl wins) --
  // the two are alternative feature adapters for the same correction slot,
  // not meant to be combined. Shares residual_clip_norm as its safety cap.
  bool enable_ambient_correction = false;

  // Off by default: a persistent, online-updated estimate of a roughly-
  // constant unmodeled disturbance (e.g. steady wind), summed with the
  // ecrl correction (if also enabled) before injection into the QP.
  // Targets the steady-state tracking offset a stateless per-cycle
  // correction cannot close -- see MpcController::execMPC's disturbance-
  // observer bookkeeping. integral_trim_tau is the leaky-integrator's
  // time constant in seconds: shorter tracks changing disturbances faster
  // but is noisier; longer is smoother but slower to adapt. Shares
  // residual_clip_norm as its safety cap (applied to the combined
  // ecrl + trim correction, and to the trim's own internal state).
  bool enable_integral_trim = false;
  double integral_trim_tau = 5.0;
};

class MpcController
{
private:
  const double gravity_ = 9.81;

  // MPC param wrapper
  MpcWrapper mpc_wrapper_;

  // params
  Parameter_t param_;

  // Last cycle's actually-applied control (thracc, body rates) -- matches
  // schema_version=2 logging's "u_k^applied" semantics (docs/
  // residual_conventions.md §4), used as the ecrl adapter's `du` feature.
  // Not the current cycle's about-to-be-solved control: that's the QP's own
  // decision variable and unknown until after solve().
  ommpc::NominalInput last_applied_u_;

  // Online constant-disturbance observer state (Parameter_t::
  // enable_integral_trim). Runs unconditionally every execMPC cycle (cheap:
  // one nominalStep + boxminus) so toggling the flag mid-flight never
  // starts from a cold estimate -- only its *use* is gated. See execMPC's
  // step-1.5 comment.
  Eigen::Matrix<double, 9, 1> d_hat_ = Eigen::Matrix<double, 9, 1>::Zero();
  ommpc::NominalState last_meas_state_;
  rclcpp::Time last_meas_time_{ 0, 0, RCL_ROS_TIME };
  bool have_last_meas_ = false;

  // for MPC timing
  double timing_feedback_;
	
  // calculate yaw
  double last_yaw_, last_yaw_dot_;
  // Helper functions for calculating yaw and yawdot
  double angle_limit(double ang)
  {
    while (ang > M_PI)
    {
        ang -= 2.0 * M_PI;
    }
    while (ang <= -M_PI)
    {
        ang += 2.0 * M_PI;
    }
    return ang;
  }

  double angle_diff(double a, double b)
  {
      double d1, d2;
      d1 = a - b;
      d2 = 2 * M_PI - fabs(d1);
      if (d1 > 0)
          d2 *= -1.0;
      if (fabs(d1) < fabs(d2))
          return (d1);
      else
          return (d2);
  }

  void calculate_yaw(Eigen::Vector3d &vel, const double dt, double &yaw, double &yawdot)
  {
    const double YAW_DOT_MAX_PER_SEC = param_.max_bodyrate_z * 0.90;
    const double YAW_DOT_DOT_MAX_PER_SEC = param_.max_bodyrate_z * 4.0;

    // tangent line
    double yaw_temp = vel.head(2).norm() > 0.1
                          ? atan2(vel(1), vel(0))
                          : last_yaw_;
    
    double d_yaw = angle_diff(yaw_temp, last_yaw_);

    const double YDM = d_yaw >= 0 ? YAW_DOT_MAX_PER_SEC : -YAW_DOT_MAX_PER_SEC;
    const double YDDM = d_yaw >= 0 ? YAW_DOT_DOT_MAX_PER_SEC : -YAW_DOT_DOT_MAX_PER_SEC;
    double d_yaw_max;
    if (fabs(last_yaw_dot_ + dt * YDDM) <= fabs(YDM))
    {
      d_yaw_max = last_yaw_dot_ * dt + 0.5 * YDDM * dt * dt;
    }
    else
    {
      // yawdot = YDM;
      double t1 = (YDM - last_yaw_dot_) / YDDM;
      d_yaw_max = ((dt - t1) + dt) * (YDM - last_yaw_dot_) / 2.0;
    }

    if (fabs(d_yaw) > fabs(d_yaw_max))
    {
      d_yaw = d_yaw_max;
    }
    yawdot = d_yaw / dt;

    yaw = last_yaw_ + d_yaw;
    if (yaw > M_PI)
      yaw -= 2 * M_PI;
    if (yaw < -M_PI)
      yaw += 2 * M_PI;

    last_yaw_ = yaw;
    last_yaw_dot_ = yawdot;
  }

public:
  std::queue<std::pair<rclcpp::Time, double>> timed_thrust;

  // Thrust-accel mapping params
  const double rho2 = 0.998; // do not change
  double thr2acc;
  double P;

  // Live diagnostics for the disturbance-observer trim / NN correction,
  // updated every execMPC cycle (read by OmmpcNode::publishReferenceState,
  // logged by ecrl_data_collector_node.cpp) so the terms' relative
  // contribution and clipping behavior can be inspected post-flight instead
  // of only inferred. trim_norm is d_hat_'s norm (post its own internal
  // clip, tracked unconditionally regardless of enable_integral_trim);
  // ecrl_norm/ecrl_time_us are named for the ecrl path but cover whichever
  // NN adapter is actually active this cycle (ecrlForward() if
  // enable_residual_correction, ambientForward() if
  // enable_ambient_correction -- the two are mutually exclusive in
  // practice, see enable_ambient_correction's doc comment), 0 when neither
  // is set; correction_clipped is whether the combined NN+trim correction
  // hit residual_clip_norm this cycle (only meaningful when at least one
  // correction path is enabled).
  double last_trim_norm = 0.0;
  double last_ecrl_norm = 0.0;
  bool last_correction_clipped = false;

  // Live per-cycle compute-time diagnostics (microseconds, std::chrono::
  // steady_clock wall time), for compute-cost benchmarking against the
  // 100Hz/10ms control-loop budget (ommpc_node.cpp's controlLoop timer).
  // last_total_time_us covers everything from the top of execMPC through
  // the OSQP solve (excludes the cheap post-solve result extraction);
  // trim/ecrl/qp are disjoint sub-slices of it, each 0 when that cycle's
  // path wasn't exercised (trim bookkeeping itself still runs whenever
  // have_last_meas_ is true, regardless of enable_integral_trim -- see
  // that member's doc comment -- so last_trim_time_us reflects that, not
  // the enable flag).
  double last_trim_time_us = 0.0;
  double last_ecrl_time_us = 0.0;
  double last_qp_solve_time_us = 0.0;
  double last_total_time_us = 0.0;

  // MPC reference state/input matrices
  std::vector<Eigen::SparseMatrix<double>> Fx;  // nstep，nx×nx
  std::vector<Eigen::SparseMatrix<double>> Fu;  // nstep，nx×nu

  // upper and lower bounds of control input
  std::vector<Eigen::VectorXd> u_lb;
  std::vector<Eigen::VectorXd> u_ub;

  void getDesiredStart(Eigen::VectorXd &xdes, Eigen::VectorXd &udes)
  {
    mpc_wrapper_.getDesiredStart(xdes, udes);
  }

  void init(Parameter_t param)
  {
    param_ = param;
    thr2acc = gravity_ / param_.hover_percent;
    P = 1e6;

    // Sane hover-ish default until the first real cycle updates it.
    last_applied_u_.thracc = gravity_;
    last_applied_u_.omega = Eigen::Vector3d::Zero();

    d_hat_.setZero();
    have_last_meas_ = false;

    // MPC Controller
    std::cout << "[MPCCtrl] mpc time_step: " << param_.step_T << std::endl;

    // set gains
    Eigen::Matrix<double, nx, nx> Q = (Eigen::Matrix<double, nx, 1>() 
      << param_.Q_pos_xy, param_.Q_pos_xy, param_.Q_pos_z,
        param_.Q_velocity, param_.Q_velocity, param_.Q_velocity,
        param_.Q_attitude_rp, param_.Q_attitude_rp, param_.Q_attitude_yaw).finished().asDiagonal();
    Eigen::Matrix<double, nu, nu> R = (Eigen::Matrix<double, nu, 1>()  
      << param_.R_thrust, param_.R_pitchroll, param_.R_pitchroll, param_.R_yaw).finished().asDiagonal();

    mpc_wrapper_.buildHessianMatrix(Q, R, param_.state_cost_exponential, param_.input_cost_exponential);

    Fx.resize(nstep);
    Fu.resize(nstep);
    u_lb.resize(nstep);
    u_ub.resize(nstep);
    for (int i = 0; i < nstep; i++)
    {
      u_ub[i].resize(nu);
      u_lb[i].resize(nu);
    }

    // Timing
    timing_feedback_ = 0.0;
  }

  void computeFlatInputwithHopfFibration(const Eigen::Vector3d &thr_acc,
                                  const Eigen::Vector3d &jer,
                                  const double &yaw,
                                  const double &yawd,
                                  const Eigen::Quaterniond &att_est,
                                  Eigen::Quaterniond &att,
                                  Eigen::Vector3d &omg) const
  {
    static Eigen::Vector3d omg_old(0.0, 0.0, 0.0);
    Eigen::Vector3d abc = thr_acc.normalized();
    double a = abc(0), b = abc(1), c = abc(2);
    Eigen::Vector3d abc_dot = (thr_acc.dot(thr_acc) * Eigen::MatrixXd::Identity(3, 3) - thr_acc * thr_acc.transpose()) / thr_acc.norm() / thr_acc.squaredNorm() * jer;
    double a_dot = abc_dot(0), b_dot = abc_dot(1), c_dot = abc_dot(2);
    
    if(1.0 + c > 1e-3 && thr_acc.norm() > 0.1){
      double norm = sqrt(2 * (1 + c));
      Eigen::Quaterniond q((1 + c) / norm, -b / norm, a / norm, 0);
      Eigen::Quaterniond q_yaw(cos(yaw / 2), 0, 0, sin(yaw / 2));
      att = q * q_yaw;
      double syaw = sin(yaw), cyaw = cos(yaw);
      omg(0) = syaw * a_dot - cyaw * b_dot - (a * syaw - b * cyaw) * c_dot / (c + 1);
      omg(1) = cyaw * a_dot + syaw * b_dot - (a * cyaw + b * syaw) * c_dot / (c + 1);
      omg(2) = (b * a_dot - a * b_dot) / (1 + c) + yawd;
    }else{
      std::cout << "Near singularity!!!!!" << std::endl;
      omg = omg_old;
      att = att_est;
    }
    omg_old = omg;
    return;
  }

  bool execMPC(const Odom_Data_t &odom,
              Controller_Output_t &u, rclcpp::Time now_time, c_int &osqp_status_out)
  {
    const clock_t start = clock();
    const auto t_total_start = std::chrono::steady_clock::now();

    // 1. set init error state of OMMPC
    auto rot_q = odom.q;
    rot_q.normalize();
    Eigen::VectorXd x_des_start(nstate), u_des_start(nu);
    mpc_wrapper_.getDesiredStart(x_des_start, u_des_start);
    const Eigen::Quaterniond est_q = rot_q;
    const Eigen::Quaterniond des_q = Eigen::Quaterniond(x_des_start(3), x_des_start(4), x_des_start(5), x_des_start(6));
    Eigen::Vector3d err_q = SO3::log(est_q.toRotationMatrix().transpose() * des_q.toRotationMatrix());
    Eigen::Vector3d err_p = x_des_start.head(3) - odom.p;
    Eigen::Vector3d err_v = x_des_start.tail(3) - odom.v;
    Eigen::VectorXd delta_x_init(nx);
    delta_x_init << err_p(0), err_p(1), err_p(2), err_v(0), err_v(1), err_v(2), err_q(0), err_q(1), err_q(2);
    mpc_wrapper_.setInitValue(delta_x_init);

    // 1.5. Disturbance handling: an optional per-cycle NN correction
    // (enable_residual_correction) and/or a persistent online disturbance-
    // observer trim (enable_integral_trim) are combined into one §4-
    // convention residual d_total, converted once via
    // residualToQpCorrection(), and injected into the QP's dynamics-
    // equality constraint (docs/residual_conventions.md "Relationship to
    // Task 4's affine QP term"). Both off by default.
    const ommpc::NominalState x_k_meas{odom.p, odom.v, rot_q};
    const ommpc::NominalState x_ref_k{x_des_start.head(3), x_des_start.tail(3), des_q};
    ommpc::NominalInput u_ref_k;
    u_ref_k.thracc = u_des_start(0);
    u_ref_k.omega = u_des_start.segment<3>(1);

    // --- online constant-disturbance observer bookkeeping -- runs
    // unconditionally (see d_hat_'s doc comment). Compares this cycle's
    // measured state against a one-step nominalStep() prediction seeded by
    // LAST cycle's measured state + control it actually applied, i.e. the
    // same causally-computable §4 "d_k" quantity
    // residuals.py::compute_targets() computes offline from logged flights.
    // Leaky-integrated (time constant integral_trim_tau) into a persistent
    // world-frame bias estimate d_hat_.
    const auto t_trim_start = std::chrono::steady_clock::now();
    if (have_last_meas_)
    {
      const double dt_obs = (now_time - last_meas_time_).seconds();
      if (dt_obs > 0.0 && dt_obs < 0.2)
      {
        const ommpc::NominalState nominal_pred_prev =
            ommpc::nominalStep(last_meas_state_, last_applied_u_, dt_obs);
        const Eigen::Matrix<double, 9, 1> d_raw = ommpc::boxminus(x_k_meas, nominal_pred_prev);
        // d_raw is defined relative to dt_obs (this cycle's actual
        // measurement interval, ~0.01s @ 100Hz -- same convention
        // scripts/residuals.py::compute_targets() uses, dt = row.dt_s).
        // residualToQpCorrection() below instead composes d onto a
        // nominal_pred built with param_.step_T (0.05s, the MPC's internal
        // horizon discretization) -- rescale linearly (a roughly-constant-
        // rate disturbance's effect over step_T is step_T/dt_obs times its
        // effect over dt_obs) so d_hat_ stays in the "per-step_T" units the
        // injection point expects, not 5x too small.
        const Eigen::Matrix<double, 9, 1> d_scaled = d_raw * (param_.step_T / dt_obs);
        const double alpha = std::clamp(dt_obs / param_.integral_trim_tau, 0.0, 1.0);
        d_hat_ = (1.0 - alpha) * d_hat_ + alpha * d_scaled;
        const double d_hat_norm = d_hat_.norm();
        if (d_hat_norm > param_.residual_clip_norm)
        {
          d_hat_ *= (param_.residual_clip_norm / d_hat_norm);
        }
        last_trim_norm = d_hat_.norm();
      }
    }
    last_trim_time_us = std::chrono::duration<double, std::micro>(
                             std::chrono::steady_clock::now() - t_trim_start)
                             .count();
    last_meas_state_ = x_k_meas;
    last_meas_time_ = now_time;
    have_last_meas_ = true;

    if (param_.enable_residual_correction || param_.enable_ambient_correction || param_.enable_integral_trim)
    {
      Eigen::VectorXd d_total = Eigen::VectorXd::Zero(9);

      if (param_.enable_residual_correction)
      {
        // ecrl adapter's 16-D input (residuals.py::ECRL_FEATURE_NAMES
        // order): dx (== delta_x_init, already computed above -- same
        // controller_error_state(x_ref_k, x_k) quantity, no need to
        // recompute), du (reference minus LAST cycle's actually-applied
        // control -- see last_applied_u_'s doc comment), r_ref.
        Eigen::VectorXd ecrl_x(16);
        ecrl_x.segment<9>(0) = delta_x_init;
        ecrl_x(9) = u_ref_k.thracc - last_applied_u_.thracc;
        ecrl_x.segment<3>(10) = u_ref_k.omega - last_applied_u_.omega;
        ecrl_x.segment<3>(13) = SO3::log(des_q.toRotationMatrix());
        const auto t_ecrl_start = std::chrono::steady_clock::now();
        const Eigen::VectorXd ecrl_pred = ommpc::residual::ecrlForward(ecrl_x);
        last_ecrl_time_us = std::chrono::duration<double, std::micro>(
                                 std::chrono::steady_clock::now() - t_ecrl_start)
                                 .count();
        last_ecrl_norm = ecrl_pred.norm();
        d_total += ecrl_pred;
      }
      else if (param_.enable_ambient_correction)
      {
        // ambient adapter's 8-D input (residuals.py::ambient_features
        // order): body-frame velocity (R_k^T v_k), body-frame MEASURED
        // angular rate (odom.w -- Odom_Data_t::feed() already applies the
        // same FRD->FLU sign flip as ecrl_data_collector_node.cpp's
        // wm_k_x/y/z, so this matches the training data's convention
        // exactly), last cycle's applied normalized thrust, and the live
        // thr2acc estimate. last_applied_u_.thracc / thr2acc is used in
        // place of the logged (post-clamp) thrust_norm_cmd_k -- the QP's
        // own u_lb/u_ub already bound thracc to
        // [min_thrust,max_thrust]*thr2acc (see setStateMatricesandBounds),
        // the same range ommpc_node.cpp's publishRatesSetpoint clamps to,
        // so this is effectively exact except in the rare cycle where the
        // clamp and the QP bound would have disagreed.
        Eigen::VectorXd ambient_x(8);
        ambient_x.segment<3>(0) = est_q.toRotationMatrix().transpose() * odom.v;
        ambient_x.segment<3>(3) = odom.w;
        ambient_x(6) = last_applied_u_.thracc / thr2acc;
        ambient_x(7) = thr2acc;
        const auto t_ecrl_start = std::chrono::steady_clock::now();
        const Eigen::VectorXd ambient_pred = ommpc::residual::ambientForward(ambient_x);
        last_ecrl_time_us = std::chrono::duration<double, std::micro>(
                                 std::chrono::steady_clock::now() - t_ecrl_start)
                                 .count();
        last_ecrl_norm = ambient_pred.norm();
        d_total += ambient_pred;
      }
      else
      {
        last_ecrl_norm = 0.0;
        last_ecrl_time_us = 0.0;
      }

      if (param_.enable_integral_trim)
      {
        d_total += d_hat_;
      }

      const double d_norm = d_total.norm();
      last_correction_clipped = d_norm > param_.residual_clip_norm;
      if (last_correction_clipped)
      {
        d_total *= (param_.residual_clip_norm / d_norm);
      }

      const ommpc::NominalState nominal_pred = ommpc::nominalStep(x_k_meas, u_ref_k, param_.step_T);
      const ommpc::NominalState x_ref_next = ommpc::nominalStep(x_ref_k, u_ref_k, param_.step_T);
      const Eigen::Matrix<double, nx, 1> c =
          ommpc::residualToQpCorrection(nominal_pred, x_ref_next, d_total);
      mpc_wrapper_.applyResidualCorrection(c);
    }

    // 2. solve MPC optimization problem
    Solution solution;
    const auto t_qp_start = std::chrono::steady_clock::now();
    bool mpc_solved = mpc_wrapper_.solve(solution, osqp_status_out);
    const auto t_qp_end = std::chrono::steady_clock::now();
    last_qp_solve_time_us = std::chrono::duration<double, std::micro>(t_qp_end - t_qp_start).count();
    last_total_time_us = std::chrono::duration<double, std::micro>(t_qp_end - t_total_start).count();

    if (!mpc_solved)
      return false;

    // 3. take first predicted state as input
    // 3.1 bodyrates
    u.bodyrates(0) = u_des_start(1) - solution.delta_u[0](1);
    u.bodyrates(1) = u_des_start(2) - solution.delta_u[0](2);
    u.bodyrates(2) = u_des_start(3) - solution.delta_u[0](3);
    // 3.2 thrustacc -> normalized thrust signal
    double thrustacc = u_des_start(0) - solution.delta_u[0](0);
    double normalized_thrust;
    normalized_thrust = thrustacc / thr2acc;
    u.thrust = normalized_thrust;

    // Cache what was actually applied this cycle for next cycle's `du`
    // feature (see last_applied_u_'s doc comment).
    last_applied_u_.thracc = thrustacc;
    last_applied_u_.omega = u.bodyrates;

    // Used for thrust-accel mapping estimation
    timed_thrust.push(std::pair<rclcpp::Time, double>(now_time, u.thrust));
    while (timed_thrust.size() > 100)
      timed_thrust.pop();

    const clock_t end = clock();
    timing_feedback_ = 0.9 * timing_feedback_ + 0.1 * double(end - start) / CLOCKS_PER_SEC;

    static rclcpp::Time last_print_time(0, 0, RCL_ROS_TIME);
    if ((now_time - last_print_time).seconds() > 2.0)
    {
      std::cout << "[MPCCtrl] MPC Timing: Latency: " << timing_feedback_ * 1000.0 << " ms" << std::endl;
      last_print_time = now_time;
    }
    return true;
  }

  void setStateMatricesandBounds(
                const int i,
                const Eigen::Quaterniond &q,
                const Eigen::Vector3d &omg,
                const double t_step,
                const double thracc)
  {
    Fx[i] = Eigen::SparseMatrix<double>(nx, nx);
    Fu[i] = Eigen::SparseMatrix<double>(nx, nu);

    // Fx
    std::vector<Eigen::Triplet<double>> tripletList;
    tripletList.reserve(27);

    // (0,0)
    for (int k = 0; k < 3; ++k) {
        tripletList.push_back(Eigen::Triplet<double>(k, k, 1.0));
    }

    // (3,3) 
    for (int k = 0; k < 3; ++k) {
        tripletList.push_back(Eigen::Triplet<double>(3+k, 3+k, 1.0));
    }

    // (0,3): t_step * I
    for (int k = 0; k < 3; ++k) {
        tripletList.push_back(Eigen::Triplet<double>(k, 3+k, t_step));
    }

    // (6,6)
    Eigen::Matrix3d exp_mat = SO3::exp(-omg * t_step);
    for (int row = 0; row < 3; ++row) {
        for (int col = 0; col < 3; ++col) {
            tripletList.push_back(Eigen::Triplet<double>(6+row, 6+col, exp_mat(row, col)));
        }
    }

    // (3,6)
    Eigen::Matrix3d mat_3_6 = t_step * q.toRotationMatrix() * SO3::hat(Eigen::Vector3d(0, 0, -thracc));
    for (int row = 0; row < 3; ++row) {
        for (int col = 0; col < 3; ++col) {
            tripletList.push_back(Eigen::Triplet<double>(3+row, 6+col, mat_3_6(row, col)));
        }
    }

    Fx[i].setFromTriplets(tripletList.begin(), tripletList.end());
    Fx[i].makeCompressed();

    // Fu
    tripletList.clear();
    tripletList.reserve(12);

    // (3,0)
    Eigen::Vector3d vec_3_0 = t_step * q.toRotationMatrix() * Eigen::Vector3d(0, 0, 1);
    for (int k = 0; k < 3; ++k) {
        tripletList.push_back(Eigen::Triplet<double>(3+k, 0, vec_3_0(k)));
    }

    // (6,1) 
    Eigen::Matrix3d mat_6_1 = SO3::leftJacobian(omg * t_step).transpose() * t_step;
    for (int row = 0; row < 3; ++row) {
        for (int col = 0; col < 3; ++col) {
            tripletList.push_back(Eigen::Triplet<double>(6+row, 1+col, mat_6_1(row, col)));
        }
    }

    Fu[i].setFromTriplets(tripletList.begin(), tripletList.end());
    Fu[i].makeCompressed();

    u_ub[i] << thracc - param_.min_thrust * thr2acc, 
                param_.max_bodyrate_xy + omg(0),
                param_.max_bodyrate_xy + omg(1),
                param_.max_bodyrate_z + omg(2); // lower
    u_lb[i] << -(param_.max_thrust * thr2acc - thracc), 
                omg(0) - param_.max_bodyrate_xy,
                omg(1) - param_.max_bodyrate_xy,
                omg(2) - param_.max_bodyrate_z; // upper 
  }

  // Case 1: HOVER reference
  // Eigen::Vector4d(px, py, pz, yaw)
  void setHoverReference(const Eigen::Vector4d &quad_pose)
  {
    double yaw = quad_pose(3);
    double thracc;
    Eigen::Vector3d des_acc_in_world = Eigen::Vector3d(0, 0, gravity_);
    thracc = gravity_;
    Eigen::Quaterniond identity_q(1, 0, 0, 0), q;
    Eigen::Vector3d omg;
    computeFlatInputwithHopfFibration(des_acc_in_world, Eigen::Vector3d::Zero(), yaw, 0, identity_q, q, omg);
    double t_step = param_.step_T;
    for (int i = 0; i < nstep; ++i)
    {
      setStateMatricesandBounds(i, q, omg, t_step, thracc);
    }
    mpc_wrapper_.buildConstraintMatrix(Fx, Fu);
    mpc_wrapper_.buildConstraintVectors(u_lb, u_ub);

    Eigen::VectorXd x_des_start(nstate);
    Eigen::VectorXd u_des_start(nu);
    x_des_start << quad_pose(0), quad_pose(1), quad_pose(2), 
                  q.w(), q.x(), q.y(), q.z(), 
                  0.0, 0.0, 0.0;
    u_des_start << gravity_, omg(0), omg(1), omg(2);
    mpc_wrapper_.setDesiredStart(x_des_start, u_des_start);
  }

  // Case 2: TXT reference
  void setTextReference(const std::vector<Eigen::Vector3d> &quad_positions, 
                                  const std::vector<Eigen::Vector3d> &quad_velocities,
                                  const Odom_Data_t &odom, 
                                  const double start_yaw,
                                  const std::vector<double> &yaws)
  {
    if (quad_positions.size() != nstep + 1 || yaws.size() != nstep + 1 || quad_velocities.size() != nstep + 1)
      std::cerr << "[MPCCtrl] Read reference Error!!" << std::endl;
    static Eigen::Vector3d last_quad_velocity = Eigen::Vector3d::Zero();
    static Eigen::Vector3d last_des_acc = Eigen::Vector3d::Zero(), last_acc;
    static Eigen::Vector3d last_des_jerk = Eigen::Vector3d::Zero();
    const double t_step = param_.step_T;
    Eigen::Quaterniond last_q = odom.q;
    Eigen::Vector3d body_z = last_q.toRotationMatrix() * Eigen::Vector3d(0, 0, 1);

    for (int i = 0; i < nstep; ++i)
    {
      Eigen::Vector3d quad_velocity = quad_velocities.at(i);
      Eigen::Vector3d quad_acc, quad_jerk;
      if (i == 0)
      {
        quad_acc = last_des_acc;
        last_acc = quad_acc;
        quad_jerk = last_des_jerk;
      }
      else
      {
        quad_acc = (quad_velocities[i] - quad_velocities[i-1]) / t_step;
        quad_jerk = (quad_acc - last_acc) / t_step;
        last_acc = quad_acc;
        if (i == 1)
        {
          last_des_acc = quad_acc;
          last_des_jerk = quad_jerk;
        }
      }
      double yaw, yaw_dot;
      double thracc;
      Eigen::Vector3d des_acc_in_world = Eigen::Vector3d(0, 0, gravity_) + quad_acc;
      thracc = des_acc_in_world.dot(body_z);
      Eigen::Quaterniond q;
      Eigen::Vector3d omg;
      if (param_.use_fix_yaw)
      {
          yaw = 0.0;
          yaw_dot = 0.0;
      }
      else  // directly compute from tangent line of traj
      {
        if (i == 0) // reset last_yaw_
        {
          last_yaw_ = start_yaw;
        }
        calculate_yaw(quad_velocity, t_step, yaw, yaw_dot);
        if (i == 0)
        {
          last_yaw_dot_ = yaw_dot;
        }
      }
      computeFlatInputwithHopfFibration(des_acc_in_world, Eigen::Vector3d::Zero(), yaw, yaw_dot, last_q, q, omg);

      q.normalize();
      last_q = q;
      if (i == 0)
      {
        Eigen::VectorXd x_des_start(nstate);
        Eigen::VectorXd u_des_start(nu);
        x_des_start << quad_positions[i](0), quad_positions[i](1), quad_positions[i](2), 
                      q.w(), q.x(), q.y(), q.z(), 
                      quad_velocities[i](0), quad_velocities[i](1), quad_velocities[i](2);
        u_des_start << thracc, omg(0), omg(1), omg(2);
        mpc_wrapper_.setDesiredStart(x_des_start, u_des_start);
      }
      
      body_z = q.toRotationMatrix() * Eigen::Vector3d(0, 0, 1);
      
      setStateMatricesandBounds(i, q, omg, t_step, thracc);
    }
    mpc_wrapper_.buildConstraintMatrix(Fx, Fu);
    mpc_wrapper_.buildConstraintVectors(u_lb, u_ub);
    last_quad_velocity = quad_velocities.at(0);
  }

  // Case 3: TRAJ reference
  void setTrajectoryReference(
          const Trajectory &traj, 
          const double tstart, 
          const double start_yaw, 
          const Trajectory &yaw_traj, 
          const Odom_Data_t &odom)
  {
    const double t_step = param_.step_T;
    double t_all = traj.getTotalDuration() - 1.0e-3;
    double t = tstart;
    double yaw, yaw_dot;
    Eigen::Vector3d pos_quad, vel_quad, acc_quad, jerk_quad; 
    Eigen::Quaterniond quat, last_quat;
    last_quat = odom.q;
    Eigen::Vector3d body_z = last_quat.toRotationMatrix() * Eigen::Vector3d(0, 0, 1);
    Eigen::Vector3d omg;

    for (int i = 0; i < nstep; i++)
    {
        Eigen::MatrixXd pvajs;
        if (t > t_all)
        { 
            t = t_all;
            pvajs = traj.getPVAJSC(t);
            pos_quad = pvajs.col(0);
            vel_quad = Eigen::Vector3d::Zero();
            acc_quad = Eigen::Vector3d::Zero();
            jerk_quad = Eigen::Vector3d::Zero();
        }
        else
        {
            pvajs = traj.getPVAJSC(t);
            pos_quad = pvajs.col(0);
            vel_quad = pvajs.col(1);
            acc_quad = pvajs.col(2);
            jerk_quad = pvajs.col(3);
        }
        
        if (param_.use_fix_yaw)
        {
            yaw = 0.0;
            yaw_dot = 0.0;
        }
        else // directly compute from tangent line of traj
        {
          if (i == 0) // reset last_yaw_
          {
            last_yaw_ = start_yaw;
          }
          calculate_yaw(vel_quad, t_step, yaw, yaw_dot);
          if (i == 0)
          {
            last_yaw_dot_ = yaw_dot;
          }
        }

        Eigen::Vector3d des_acc_in_world = acc_quad + Eigen::Vector3d(0, 0, gravity_); 
        double thracc = des_acc_in_world.dot(body_z);
        computeFlatInputwithHopfFibration(des_acc_in_world, jerk_quad, yaw, yaw_dot, last_quat, quat, omg);

        last_quat = quat;

        if (i == 0)
        {
          Eigen::VectorXd x_des_start(nstate);
          Eigen::VectorXd u_des_start(nu);
          x_des_start << pos_quad(0), pos_quad(1), pos_quad(2), 
                        last_quat.w(), last_quat.x(), last_quat.y(), last_quat.z(), 
                        vel_quad(0), vel_quad(1), vel_quad(2);
          u_des_start << thracc, omg(0), omg(1), omg(2);
          mpc_wrapper_.setDesiredStart(x_des_start, u_des_start);
        }
        
        body_z = last_quat.toRotationMatrix() * Eigen::Vector3d(0, 0, 1);

        setStateMatricesandBounds(i, last_quat, omg, t_step, thracc);
        t += t_step;
    }
    mpc_wrapper_.buildConstraintMatrix(Fx, Fu);
    mpc_wrapper_.buildConstraintVectors(u_lb, u_ub);
  }

  void estimateThrustModel(const Eigen::Vector3d &est_a, rclcpp::Time now_time)
  {
    rclcpp::Time t_now = now_time;
    while (timed_thrust.size() >= 1)
    {
      // Choose data before 35~45ms ago
      std::pair<rclcpp::Time, double> t_t = timed_thrust.front();
      double time_passed = (t_now - t_t.first).seconds();
      if (time_passed > 0.045) // 45ms
      {
        timed_thrust.pop();
        continue;
      }
      if (time_passed < 0.035) // 35ms
      {
        return;
      }

      /***********************************************************/
      /* Recursive least squares algorithm with vanishing memory */
      /***********************************************************/
      double thr = t_t.second;
      timed_thrust.pop();
      
      /***********************************/
      /* Model: est_a(2) = thr2acc * thr */
      /***********************************/
      double gamma = 1.0 / (rho2 + thr * P * thr);
      double K = gamma * P * thr;
      thr2acc = thr2acc + K * (est_a(2) - thr * thr2acc);
      P = (1.0 - K * thr) * P / rho2;
      const double hover_percentage = gravity_ / thr2acc;
      if ( hover_percentage > 0.8 || hover_percentage < 0.1 )
      {
        thr2acc = hover_percentage > 0.8 ? gravity_ / 0.8 : thr2acc;
        thr2acc = hover_percentage < 0.1 ? gravity_ / 0.1 : thr2acc;
      }
    }
  }
};

#endif
