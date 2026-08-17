// Raw-only ECRL data logger. Deliberately contains NO residual computation
// and NO copy of the controller's linearization -- see
// docs/residual_conventions.md. Every residual definition (the old
// linearized one, and the new nonlinear-model one) is an offline transform
// over this CSV in scripts/residuals.py. If you find yourself wanting to add
// math here, it belongs in that script instead.
#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <px4_msgs/msg/vehicle_rates_setpoint.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <px4_msgs/msg/sensor_combined.hpp>
#include <ommpc_ros2/msg/reference_state.hpp>

#include <Eigen/Dense>

#include <chrono>
#include <fstream>
#include <string>
#include <sys/stat.h>

namespace
{
constexpr int kSchemaVersion = 4;

int64_t steadyNowNs()
{
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
             std::chrono::steady_clock::now().time_since_epoch())
      .count();
}
} // namespace

class EcrlDataCollectorNode : public rclcpp::Node
{
private:
  // Subscribers
  rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleRatesSetpoint>::SharedPtr setpoint_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr status_sub_;
  rclcpp::Subscription<ommpc_ros2::msg::ReferenceState>::SharedPtr ref_sub_;

  // Node parameters
  std::string csv_filepath_;

  // Cached latest messages
  px4_msgs::msg::VehicleRatesSetpoint::SharedPtr latest_setpoint_;
  px4_msgs::msg::VehicleStatus::SharedPtr latest_status_;
  ommpc_ros2::msg::ReferenceState::SharedPtr latest_ref_;

  // File stream for CSV
  std::ofstream csv_file_;
  bool header_written_ = false;

  // Row k (previous tick) raw values, cached until row k+1 arrives so one CSV
  // row can carry the full (x_k -> x_{k+1}) transition.
  bool has_prev_ = false;
  int64_t prev_mono_ns_ = 0;
  uint64_t prev_odom_us_ = 0;

  Eigen::Vector3d prev_p_meas_, prev_v_meas_, prev_w_meas_;
  Eigen::Quaterniond prev_q_meas_;

  double prev_thracc_cmd_ = 0.0;
  Eigen::Vector3d prev_w_cmd_;
  double prev_thrust_norm_cmd_ = 0.0;
  double prev_thr2acc_ = 0.0;

  Eigen::Vector3d prev_p_ref_, prev_v_ref_;
  Eigen::Quaterniond prev_q_ref_;
  double prev_thracc_ref_ = 0.0;
  Eigen::Vector3d prev_w_ref_;

  uint8_t prev_fsm_state_ = 0;
  int32_t prev_osqp_status_ = -1;
  double prev_trim_norm_ = 0.0;
  double prev_ecrl_norm_ = 0.0;
  bool prev_correction_clipped_ = false;
  double prev_trim_time_us_ = 0.0;
  double prev_ecrl_time_us_ = 0.0;
  double prev_qp_solve_time_us_ = 0.0;
  double prev_total_time_us_ = 0.0;

  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr callback_handle_;

  rcl_interfaces::msg::SetParametersResult parametersCallback(const std::vector<rclcpp::Parameter> &parameters)
  {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    for (const auto &parameter : parameters)
    {
      if (parameter.get_name() == "csv_filepath")
      {
        std::string new_filepath = parameter.as_string();
        if (new_filepath != csv_filepath_)
        {
          RCLCPP_INFO(this->get_logger(), "Changing CSV filepath from %s to %s", csv_filepath_.c_str(),
                      new_filepath.c_str());

          size_t last_slash = new_filepath.find_last_of('/');
          if (last_slash != std::string::npos)
          {
            std::string dir = new_filepath.substr(0, last_slash);
            mkdir(dir.c_str(), 0777);
          }

          if (csv_file_.is_open())
          {
            csv_file_.close();
          }

          csv_filepath_ = new_filepath;
          csv_file_.open(csv_filepath_, std::ios::out | std::ios::app);
          header_written_ = false;
          has_prev_ = false; // new file: don't pair across the boundary

          if (!csv_file_.is_open())
          {
            RCLCPP_ERROR(this->get_logger(), "Failed to open new CSV file: %s", csv_filepath_.c_str());
            result.successful = false;
            result.reason = "Failed to open new CSV file";
          }
          else
          {
            RCLCPP_INFO(this->get_logger(), "Logging ECRL data to new file: %s", csv_filepath_.c_str());
          }
        }
      }
    }
    return result;
  }

  void statusCallback(const px4_msgs::msg::VehicleStatus::SharedPtr msg) { latest_status_ = msg; }
  void setpointCallback(const px4_msgs::msg::VehicleRatesSetpoint::SharedPtr msg) { latest_setpoint_ = msg; }
  void referenceCallback(const ommpc_ros2::msg::ReferenceState::SharedPtr msg) { latest_ref_ = msg; }

  void odomCallback(const px4_msgs::msg::VehicleOdometry::SharedPtr msg)
  {
    // Only record while armed and in offboard mode; don't pair across a gap.
    if (!latest_status_ || latest_status_->arming_state != px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED ||
        latest_status_->nav_state != px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD)
    {
      has_prev_ = false;
      return;
    }

    if (!latest_setpoint_ || !latest_ref_)
    {
      return; // wait until we have a commanded setpoint and a reference
    }

    // 1. Measured state, NED/FRD -> ENU/FLU (same convention as Odom_Data_t::feed).
    Eigen::Vector3d curr_p_meas(msg->position[0], msg->position[1], msg->position[2]);
    Eigen::Vector3d curr_v_meas(msg->velocity[0], msg->velocity[1], msg->velocity[2]);
    Eigen::Quaterniond curr_q_meas(msg->q[0], msg->q[1], msg->q[2], msg->q[3]);
    Eigen::Vector3d curr_w_meas(msg->angular_velocity[0], msg->angular_velocity[1], msg->angular_velocity[2]);

    Eigen::Matrix3d R_mid;
    R_mid << 0.0, 1.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, -1.0;
    Eigen::Quaterniond q_mid(R_mid);
    Eigen::Quaterniond q_body(0.0, 1.0, 0.0, 0.0);

    curr_p_meas = R_mid * curr_p_meas;
    curr_v_meas = R_mid * curr_v_meas;
    curr_q_meas = q_mid * curr_q_meas * q_body;
    curr_q_meas.normalize();
    // FRD -> FLU body rate: same sign-flip convention as Odom_Data_t::feed()
    // for px4_msgs::VehicleOdometry (no R_mid rotation needed here, body rate
    // is already in the body frame on both sides).
    curr_w_meas(1) = -curr_w_meas(1);
    curr_w_meas(2) = -curr_w_meas(2);

    // 2. Reference (already ENU/FLU as published by ommpc_node).
    Eigen::Vector3d curr_p_ref(latest_ref_->position.x, latest_ref_->position.y, latest_ref_->position.z);
    Eigen::Vector3d curr_v_ref(latest_ref_->velocity.x, latest_ref_->velocity.y, latest_ref_->velocity.z);
    Eigen::Quaterniond curr_q_ref(latest_ref_->quaternion.w, latest_ref_->quaternion.x, latest_ref_->quaternion.y,
                                  latest_ref_->quaternion.z);
    curr_q_ref.normalize();
    double curr_thracc_ref = latest_ref_->thrust;
    Eigen::Vector3d curr_w_ref(latest_ref_->body_rates.x, latest_ref_->body_rates.y, latest_ref_->body_rates.z);
    double curr_thr2acc = latest_ref_->thr2acc;
    uint8_t curr_fsm_state = latest_ref_->fsm_state;
    int32_t curr_osqp_status = latest_ref_->osqp_status;
    double curr_trim_norm = latest_ref_->trim_norm;
    double curr_ecrl_norm = latest_ref_->ecrl_norm;
    bool curr_correction_clipped = latest_ref_->correction_clipped;
    double curr_trim_time_us = latest_ref_->trim_time_us;
    double curr_ecrl_time_us = latest_ref_->ecrl_time_us;
    double curr_qp_solve_time_us = latest_ref_->qp_solve_time_us;
    double curr_total_time_us = latest_ref_->total_time_us;

    // 3. Applied control, FRD -> FLU (thrust_body[2] is negative-up, normalized [0,1]).
    double curr_thrust_norm_cmd = -latest_setpoint_->thrust_body[2];
    double curr_thracc_cmd = curr_thrust_norm_cmd * curr_thr2acc;
    Eigen::Vector3d curr_w_cmd(latest_setpoint_->roll, -latest_setpoint_->pitch, -latest_setpoint_->yaw);

    int64_t curr_mono_ns = steadyNowNs();

    if (has_prev_)
    {
      double dt = (msg->timestamp - prev_odom_us_) * 1e-6;
      // Sanity bound only -- the actual accept/reject-on-dt decision is made
      // offline in scripts/residuals.py from the raw timestamps logged below,
      // so it can be tuned without re-flying. This just avoids writing rows
      // across a total dropout (e.g. odometry loss, msg replay).
      if (dt > 0.0 && dt < 0.5)
      {
        logRow(prev_mono_ns_, curr_mono_ns, prev_odom_us_, msg->timestamp, prev_p_meas_, prev_v_meas_, prev_q_meas_,
               prev_w_meas_, curr_p_meas, curr_v_meas, curr_q_meas, prev_thracc_cmd_, prev_w_cmd_,
               prev_thrust_norm_cmd_, prev_thr2acc_, prev_p_ref_, prev_v_ref_, prev_q_ref_, prev_thracc_ref_,
               prev_w_ref_, prev_fsm_state_, prev_osqp_status_, prev_trim_norm_, prev_ecrl_norm_,
               prev_correction_clipped_, prev_trim_time_us_, prev_ecrl_time_us_, prev_qp_solve_time_us_,
               prev_total_time_us_);
      }
    }

    prev_mono_ns_ = curr_mono_ns;
    prev_odom_us_ = msg->timestamp;
    prev_p_meas_ = curr_p_meas;
    prev_v_meas_ = curr_v_meas;
    prev_q_meas_ = curr_q_meas;
    prev_w_meas_ = curr_w_meas;

    prev_thracc_cmd_ = curr_thracc_cmd;
    prev_w_cmd_ = curr_w_cmd;
    prev_thrust_norm_cmd_ = curr_thrust_norm_cmd;
    prev_thr2acc_ = curr_thr2acc;

    prev_p_ref_ = curr_p_ref;
    prev_v_ref_ = curr_v_ref;
    prev_q_ref_ = curr_q_ref;
    prev_thracc_ref_ = curr_thracc_ref;
    prev_w_ref_ = curr_w_ref;

    prev_fsm_state_ = curr_fsm_state;
    prev_osqp_status_ = curr_osqp_status;
    prev_trim_norm_ = curr_trim_norm;
    prev_ecrl_norm_ = curr_ecrl_norm;
    prev_correction_clipped_ = curr_correction_clipped;
    prev_trim_time_us_ = curr_trim_time_us;
    prev_ecrl_time_us_ = curr_ecrl_time_us;
    prev_qp_solve_time_us_ = curr_qp_solve_time_us;
    prev_total_time_us_ = curr_total_time_us;
    has_prev_ = true;
  }

  static void writeQuat(std::ofstream &f, const Eigen::Quaterniond &q)
  {
    f << q.w() << "," << q.x() << "," << q.y() << "," << q.z() << ",";
  }
  static void writeVec3(std::ofstream &f, const Eigen::Vector3d &v)
  {
    f << v(0) << "," << v(1) << "," << v(2) << ",";
  }

  void logRow(int64_t t_k_mono_ns, int64_t t_kp1_mono_ns, uint64_t odom_ts_k_us, uint64_t odom_ts_kp1_us,
              const Eigen::Vector3d &p_k, const Eigen::Vector3d &v_k, const Eigen::Quaterniond &q_k,
              const Eigen::Vector3d &w_meas_k, const Eigen::Vector3d &p_kp1, const Eigen::Vector3d &v_kp1,
              const Eigen::Quaterniond &q_kp1, double thracc_cmd_k, const Eigen::Vector3d &w_cmd_k,
              double thrust_norm_cmd_k, double thr2acc_k, const Eigen::Vector3d &p_ref_k,
              const Eigen::Vector3d &v_ref_k, const Eigen::Quaterniond &q_ref_k, double thracc_ref_k,
              const Eigen::Vector3d &w_ref_k, uint8_t fsm_state_k, int32_t osqp_status_k, double trim_norm_k,
              double ecrl_norm_k, bool correction_clipped_k, double trim_time_us_k, double ecrl_time_us_k,
              double qp_solve_time_us_k, double total_time_us_k)
  {
    if (!csv_file_.is_open())
      return;

    if (!header_written_)
    {
      csv_file_ << "# ecrl_schema_version=" << kSchemaVersion << "\n";
      csv_file_ << "t_k_mono_ns,t_kp1_mono_ns,odom_ts_k_us,odom_ts_kp1_us,dt_s,"
                << "p_k_x,p_k_y,p_k_z,v_k_x,v_k_y,v_k_z,q_k_w,q_k_x,q_k_y,q_k_z,wm_k_x,wm_k_y,wm_k_z,"
                << "p_kp1_x,p_kp1_y,p_kp1_z,v_kp1_x,v_kp1_y,v_kp1_z,q_kp1_w,q_kp1_x,q_kp1_y,q_kp1_z,"
                << "thracc_cmd_k,wx_cmd_k,wy_cmd_k,wz_cmd_k,thrust_norm_cmd_k,thr2acc_k,"
                << "p_ref_k_x,p_ref_k_y,p_ref_k_z,v_ref_k_x,v_ref_k_y,v_ref_k_z,"
                << "q_ref_k_w,q_ref_k_x,q_ref_k_y,q_ref_k_z,thracc_ref_k,wx_ref_k,wy_ref_k,wz_ref_k,"
                << "fsm_state_k,osqp_status_k,trim_norm_k,ecrl_norm_k,correction_clipped_k,"
                << "trim_time_us_k,ecrl_time_us_k,qp_solve_time_us_k,total_time_us_k\n";
      header_written_ = true;
    }

    csv_file_ << t_k_mono_ns << "," << t_kp1_mono_ns << "," << odom_ts_k_us << "," << odom_ts_kp1_us << ","
              << (odom_ts_kp1_us - odom_ts_k_us) * 1e-6 << ",";
    writeVec3(csv_file_, p_k);
    writeVec3(csv_file_, v_k);
    writeQuat(csv_file_, q_k);
    writeVec3(csv_file_, w_meas_k);
    writeVec3(csv_file_, p_kp1);
    writeVec3(csv_file_, v_kp1);
    writeQuat(csv_file_, q_kp1);
    csv_file_ << thracc_cmd_k << ",";
    writeVec3(csv_file_, w_cmd_k);
    csv_file_ << thrust_norm_cmd_k << "," << thr2acc_k << ",";
    writeVec3(csv_file_, p_ref_k);
    writeVec3(csv_file_, v_ref_k);
    writeQuat(csv_file_, q_ref_k);
    csv_file_ << thracc_ref_k << ",";
    writeVec3(csv_file_, w_ref_k);
    csv_file_ << static_cast<int>(fsm_state_k) << "," << osqp_status_k << "," << trim_norm_k << "," << ecrl_norm_k
              << "," << (correction_clipped_k ? 1 : 0) << "," << trim_time_us_k << "," << ecrl_time_us_k << ","
              << qp_solve_time_us_k << "," << total_time_us_k << "\n";
    csv_file_.flush();
  }

public:
  EcrlDataCollectorNode() : Node("ecrl_data_collector_node")
  {
    // ecrl_datasets_v2: schema_version=2 raw logs. Kept in a separate directory
    // from the old /home/dinesh/ecrl_datasets (schema_version=1, baked-in
    // linearized residuals) so the two formats can never be mixed by accident
    // -- residuals.py::load_raw_csv() would reject the old files anyway, but
    // a distinct directory means you never have to remember that.
    this->declare_parameter("csv_filepath", "/home/dinesh/ecrl_datasets_v2/figure_8.csv");
    csv_filepath_ = this->get_parameter("csv_filepath").as_string();

    size_t last_slash = csv_filepath_.find_last_of('/');
    if (last_slash != std::string::npos)
    {
      std::string dir = csv_filepath_.substr(0, last_slash);
      mkdir(dir.c_str(), 0777);
    }

    csv_file_.open(csv_filepath_, std::ios::out | std::ios::app);
    if (!csv_file_.is_open())
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to open CSV file: %s", csv_filepath_.c_str());
    }
    else
    {
      RCLCPP_INFO(this->get_logger(), "Logging raw ECRL data (schema_version=%d) to: %s", kSchemaVersion,
                  csv_filepath_.c_str());
    }

    callback_handle_ = this->add_on_set_parameters_callback(
        std::bind(&EcrlDataCollectorNode::parametersCallback, this, std::placeholders::_1));

    odom_sub_ = this->create_subscription<px4_msgs::msg::VehicleOdometry>(
        "/fmu/out/vehicle_odometry", rclcpp::QoS(10).best_effort(),
        std::bind(&EcrlDataCollectorNode::odomCallback, this, std::placeholders::_1));
    setpoint_sub_ = this->create_subscription<px4_msgs::msg::VehicleRatesSetpoint>(
        "/fmu/in/vehicle_rates_setpoint", rclcpp::QoS(10).best_effort(),
        std::bind(&EcrlDataCollectorNode::setpointCallback, this, std::placeholders::_1));
    status_sub_ = this->create_subscription<px4_msgs::msg::VehicleStatus>(
        "/fmu/out/vehicle_status_v1", rclcpp::QoS(10).best_effort(),
        std::bind(&EcrlDataCollectorNode::statusCallback, this, std::placeholders::_1));
    ref_sub_ = this->create_subscription<ommpc_ros2::msg::ReferenceState>(
        "/ommpc/current_reference", 10, std::bind(&EcrlDataCollectorNode::referenceCallback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "ECRL Data Collector Node Initialized.");
  }

  ~EcrlDataCollectorNode()
  {
    if (csv_file_.is_open())
    {
      csv_file_.close();
    }
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<EcrlDataCollectorNode>());
  rclcpp::shutdown();
  return 0;
}
