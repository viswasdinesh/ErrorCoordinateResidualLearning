#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <px4_msgs/msg/vehicle_rates_setpoint.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <px4_msgs/msg/sensor_combined.hpp>
#include <ommpc_ros2/msg/reference_state.hpp>

#include <Eigen/Dense>
#include "ommpc_ros2/so3_math.h"

#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <memory>
#include <sys/stat.h>

class EcrlDataCollectorNode : public rclcpp::Node {
private:
  // Subscribers
  rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleRatesSetpoint>::SharedPtr setpoint_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr status_sub_;
  rclcpp::Subscription<ommpc_ros2::msg::ReferenceState>::SharedPtr ref_sub_;

  // Node Parameters
  std::string csv_filepath_;
  double hover_percent_ = 0.5;
  double thr2acc_ = 9.81 / 0.5;

  // Cached Latest Messages
  px4_msgs::msg::VehicleRatesSetpoint::SharedPtr latest_setpoint_;
  px4_msgs::msg::VehicleStatus::SharedPtr latest_status_;
  ommpc_ros2::msg::ReferenceState::SharedPtr latest_ref_;

  // File Stream for CSV
  std::ofstream csv_file_;
  bool header_written_ = false;

  // Tracking Previous States for discrete-time propagation
  bool has_prev_state_ = false;
  uint64_t prev_timestamp_ = 0;

  Eigen::Vector3d prev_p_meas_;
  Eigen::Vector3d prev_v_meas_;
  Eigen::Quaterniond prev_q_meas_;
  Eigen::Vector3d prev_w_meas_;

  // Previous Reference
  Eigen::Vector3d prev_p_ref_;
  Eigen::Vector3d prev_v_ref_;
  Eigen::Quaterniond prev_q_ref_;
  Eigen::Vector3d prev_w_ref_;
  double prev_thrust_ref_ = 0.0;

  // Previous Control setpoint
  double prev_thrust_cmd_ = 0.0;
  Eigen::Vector3d prev_w_cmd_;

  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr callback_handle_;

  rcl_interfaces::msg::SetParametersResult parametersCallback(const std::vector<rclcpp::Parameter> &parameters) {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    for (const auto &parameter : parameters) {
      if (parameter.get_name() == "csv_filepath") {
        std::string new_filepath = parameter.as_string();
        if (new_filepath != csv_filepath_) {
          RCLCPP_INFO(this->get_logger(), "Changing CSV filepath from %s to %s", csv_filepath_.c_str(), new_filepath.c_str());
          
          // Ensure output directory exists
          size_t last_slash = new_filepath.find_last_of('/');
          if (last_slash != std::string::npos) {
            std::string dir = new_filepath.substr(0, last_slash);
            mkdir(dir.c_str(), 0777);
          }

          // Close old file if open
          if (csv_file_.is_open()) {
            csv_file_.close();
          }

          // Open new file
          csv_filepath_ = new_filepath;
          csv_file_.open(csv_filepath_, std::ios::out | std::ios::app);
          header_written_ = false;

          if (!csv_file_.is_open()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open new CSV file: %s", csv_filepath_.c_str());
            result.successful = false;
            result.reason = "Failed to open new CSV file";
          } else {
            RCLCPP_INFO(this->get_logger(), "Logging ECRL data to new file: %s", csv_filepath_.c_str());
          }
        }
      }
    }
    return result;
  }

  void statusCallback(const px4_msgs::msg::VehicleStatus::SharedPtr msg) {
    latest_status_ = msg;
  }

  void setpointCallback(const px4_msgs::msg::VehicleRatesSetpoint::SharedPtr msg) {
    latest_setpoint_ = msg;
  }

  void referenceCallback(const ommpc_ros2::msg::ReferenceState::SharedPtr msg) {
    latest_ref_ = msg;
  }

  double getYawFromQuaternion(const Eigen::Quaterniond &q) {
    return atan2(2 * (q.w() * q.z() + q.x() * q.y()), 1 - 2 * (q.y() * q.y() + q.z() * q.z()));
  }

  void odomCallback(const px4_msgs::msg::VehicleOdometry::SharedPtr msg) {
    // Only record data if armed and in offboard mode
    if (!latest_status_ ||
        latest_status_->arming_state != px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED ||
        latest_status_->nav_state != px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD) {
      has_prev_state_ = false;
      prev_timestamp_ = msg->timestamp;
      return;
    }

    if (!latest_setpoint_ || !latest_ref_) {
      return; // Wait until we get reference and setpoints
    }

    // 1. Get current measured state in ENU/FLU
    Eigen::Vector3d curr_p_meas(msg->position[0], msg->position[1], msg->position[2]);
    Eigen::Vector3d curr_v_meas(msg->velocity[0], msg->velocity[1], msg->velocity[2]);
    Eigen::Quaterniond curr_q_meas(msg->q[0], msg->q[1], msg->q[2], msg->q[3]);
    Eigen::Vector3d curr_w_meas(msg->angular_velocity[0], msg->angular_velocity[1], msg->angular_velocity[2]);

    // NED/FRD to ENU/FLU frame conversion
    Eigen::Matrix3d R_mid;
    R_mid << 0.0, 1.0, 0.0,
             1.0, 0.0, 0.0,
             0.0, 0.0, -1.0;
    Eigen::Quaterniond q_mid(R_mid);
    Eigen::Quaterniond q_body(0.0, 1.0, 0.0, 0.0);

    curr_p_meas = R_mid * curr_p_meas;
    curr_v_meas = R_mid * curr_v_meas; // already in world frame
    curr_q_meas = q_mid * curr_q_meas * q_body;
    curr_q_meas.normalize();
    curr_w_meas(1) = -curr_w_meas(1);
    curr_w_meas(2) = -curr_w_meas(2);

    // 2. Get current reference state (in ENU/FLU)
    Eigen::Vector3d curr_p_ref(latest_ref_->position.x, latest_ref_->position.y, latest_ref_->position.z);
    Eigen::Vector3d curr_v_ref(latest_ref_->velocity.x, latest_ref_->velocity.y, latest_ref_->velocity.z);
    Eigen::Quaterniond curr_q_ref(latest_ref_->quaternion.w, latest_ref_->quaternion.x, latest_ref_->quaternion.y, latest_ref_->quaternion.z);
    curr_q_ref.normalize();
    Eigen::Vector3d curr_w_ref(latest_ref_->body_rates.x, latest_ref_->body_rates.y, latest_ref_->body_rates.z);
    double curr_thrust_ref = latest_ref_->thrust;

    // 3. Get current commanded control input (convert FRD to FLU/scale thrust)
    double curr_thrust_cmd = -latest_setpoint_->thrust_body[2] * thr2acc_;
    Eigen::Vector3d curr_w_cmd(latest_setpoint_->roll, -latest_setpoint_->pitch, -latest_setpoint_->yaw);

    // If we have previous state, compute time step and error propagation
    if (has_prev_state_) {
      double dt = (msg->timestamp - prev_timestamp_) * 1e-6; // in seconds
      
      if (dt > 0.0 && dt < 0.1) {
        // Calculate measured error at previous step
        Eigen::Vector3d dp_prev = prev_p_meas_ - prev_p_ref_;
        Eigen::Vector3d dv_prev = prev_v_meas_ - prev_v_ref_;
        Eigen::Vector3d dtheta_prev = SO3::log(prev_q_ref_.toRotationMatrix().transpose() * prev_q_meas_.toRotationMatrix());

        Eigen::Matrix<double, 9, 1> dx_prev;
        dx_prev << dp_prev, dv_prev, dtheta_prev;

        // Calculate control input error at previous step
        double du_thrust = prev_thrust_cmd_ - prev_thrust_ref_;
        Eigen::Vector3d du_w = prev_w_cmd_ - prev_w_ref_;

        Eigen::Matrix<double, 4, 1> du_prev;
        du_prev << du_thrust, du_w;

        // Build Fx and Fu matrices at previous step
        Eigen::Matrix<double, 9, 9> Fx = Eigen::Matrix<double, 9, 9>::Zero();
        for (int k = 0; k < 3; ++k) Fx(k, k) = 1.0;
        for (int k = 0; k < 3; ++k) Fx(3+k, 3+k) = 1.0;
        for (int k = 0; k < 3; ++k) Fx(k, 3+k) = dt;

        Eigen::Matrix3d exp_mat = SO3::exp(-prev_w_ref_ * dt);
        Fx.block<3, 3>(6, 6) = exp_mat;

        Eigen::Matrix3d mat_3_6 = dt * prev_q_ref_.toRotationMatrix() * SO3::hat(Eigen::Vector3d(0, 0, -prev_thrust_ref_));
        Fx.block<3, 3>(3, 6) = mat_3_6;

        Eigen::Matrix<double, 9, 4> Fu = Eigen::Matrix<double, 9, 4>::Zero();
        Eigen::Vector3d vec_3_0 = dt * prev_q_ref_.toRotationMatrix() * Eigen::Vector3d(0, 0, 1);
        Fu.block<3, 1>(3, 0) = vec_3_0;
        Fu.block<3, 3>(6, 1) = dt * SO3::leftJacobian(prev_w_ref_ * dt).transpose();

        // Calculate nominal next error state prediction
        Eigen::Matrix<double, 9, 1> dx_nom = Fx * dx_prev + Fu * du_prev;

        // Calculate actual next error state
        Eigen::Vector3d dp_curr = curr_p_meas - curr_p_ref;
        Eigen::Vector3d dv_curr = curr_v_meas - curr_v_ref;
        Eigen::Vector3d dtheta_curr = SO3::log(curr_q_ref.toRotationMatrix().transpose() * curr_q_meas.toRotationMatrix());

        Eigen::Matrix<double, 9, 1> dx_curr_meas;
        dx_curr_meas << dp_curr, dv_curr, dtheta_curr;

        // Calculate reference attitude Log(R_d)∨
        Eigen::Vector3d r_ref_prev = SO3::log(prev_q_ref_.toRotationMatrix());

        // Calculate residual (discrepancy)
        Eigen::Matrix<double, 9, 1> residual = dx_curr_meas - dx_nom;

        // Write to CSV
        logToCsv(msg->timestamp, dt, dx_prev, du_prev, residual, r_ref_prev);
      }
    }

    // Save current values as previous for the next callback
    prev_p_meas_ = curr_p_meas;
    prev_v_meas_ = curr_v_meas;
    prev_q_meas_ = curr_q_meas;
    prev_w_meas_ = curr_w_meas;

    prev_p_ref_ = curr_p_ref;
    prev_v_ref_ = curr_v_ref;
    prev_q_ref_ = curr_q_ref;
    prev_w_ref_ = curr_w_ref;
    prev_thrust_ref_ = curr_thrust_ref;

    prev_thrust_cmd_ = curr_thrust_cmd;
    prev_w_cmd_ = curr_w_cmd;

    prev_timestamp_ = msg->timestamp;
    has_prev_state_ = true;
  }

  void logToCsv(uint64_t timestamp, double dt, const Eigen::Matrix<double, 9, 1> &dx, const Eigen::Matrix<double, 4, 1> &du, const Eigen::Matrix<double, 9, 1> &res, const Eigen::Vector3d &r_ref) {
    if (!csv_file_.is_open()) return;

    if (!header_written_) {
      csv_file_ << "timestamp,dt,";
      for (int i = 0; i < 9; ++i) csv_file_ << "dx_" << i << ",";
      for (int i = 0; i < 4; ++i) csv_file_ << "du_" << i << ",";
      for (int i = 0; i < 9; ++i) csv_file_ << "res_" << i << ",";
      for (int i = 0; i < 3; ++i) csv_file_ << "r_ref_" << i << (i == 2 ? "" : ",");
      csv_file_ << "\n";
      header_written_ = true;
    }

    csv_file_ << timestamp << "," << dt << ",";
    for (int i = 0; i < 9; ++i) csv_file_ << dx(i) << ",";
    for (int i = 0; i < 4; ++i) csv_file_ << du(i) << ",";
    for (int i = 0; i < 9; ++i) csv_file_ << res(i) << ",";
    for (int i = 0; i < 3; ++i) csv_file_ << r_ref(i) << (i == 2 ? "" : ",");
    csv_file_ << "\n";
    csv_file_.flush();
  }

public:
  EcrlDataCollectorNode() : Node("ecrl_data_collector_node") {
    // Parameters
    this->declare_parameter("csv_filepath", "/home/dinesh/ecrl_datasets/figure_8.csv");
    this->declare_parameter("hover_percentage", 0.5);

    csv_filepath_ = this->get_parameter("csv_filepath").as_string();
    hover_percent_ = this->get_parameter("hover_percentage").as_double();
    thr2acc_ = 9.81 / hover_percent_;

    // Ensure output directory exists
    size_t last_slash = csv_filepath_.find_last_of('/');
    if (last_slash != std::string::npos) {
      std::string dir = csv_filepath_.substr(0, last_slash);
      mkdir(dir.c_str(), 0777);
    }

    // Open file
    csv_file_.open(csv_filepath_, std::ios::out | std::ios::app);
    if (!csv_file_.is_open()) {
      RCLCPP_ERROR(this->get_logger(), "Failed to open CSV file: %s", csv_filepath_.c_str());
    } else {
      RCLCPP_INFO(this->get_logger(), "Logging ECRL data to: %s", csv_filepath_.c_str());
    }

    callback_handle_ = this->add_on_set_parameters_callback(
        std::bind(&EcrlDataCollectorNode::parametersCallback, this, std::placeholders::_1));

    odom_sub_ = this->create_subscription<px4_msgs::msg::VehicleOdometry>(
        "/fmu/out/vehicle_odometry", rclcpp::QoS(10).best_effort(), std::bind(&EcrlDataCollectorNode::odomCallback, this, std::placeholders::_1));
    setpoint_sub_ = this->create_subscription<px4_msgs::msg::VehicleRatesSetpoint>(
        "/fmu/in/vehicle_rates_setpoint", rclcpp::QoS(10).best_effort(), std::bind(&EcrlDataCollectorNode::setpointCallback, this, std::placeholders::_1));
    status_sub_ = this->create_subscription<px4_msgs::msg::VehicleStatus>(
        "/fmu/out/vehicle_status_v1", rclcpp::QoS(10).best_effort(), std::bind(&EcrlDataCollectorNode::statusCallback, this, std::placeholders::_1));
    ref_sub_ = this->create_subscription<ommpc_ros2::msg::ReferenceState>(
        "/ommpc/current_reference", 10, std::bind(&EcrlDataCollectorNode::referenceCallback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "ECRL Data Collector Node Initialized.");
  }

  ~EcrlDataCollectorNode() {
    if (csv_file_.is_open()) {
      csv_file_.close();
    }
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<EcrlDataCollectorNode>());
  rclcpp::shutdown();
  return 0;
}
