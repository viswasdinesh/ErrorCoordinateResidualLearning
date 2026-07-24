#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <px4_msgs/msg/sensor_combined.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/vehicle_rates_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include "ommpc_ros2/msg/reference_state.hpp"
#include "ommpc_ros2/ommpc_controller.hpp"
#include <fstream>
#include <sstream>

enum FsmState
{
  WAIT_OFFBOARD,
  ARMED_WAIT,
  TAKEOFF,
  HOVER,
  TRACKING,
  LAND
};

class OmmpcNode : public rclcpp::Node
{
private:
  // Subscribers
  rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<px4_msgs::msg::SensorCombined>::SharedPtr imu_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr status_sub_;
  rclcpp::Subscription<ommpc_ros2::msg::PolyTraj>::SharedPtr traj_sub_;

  // Publishers
  rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_control_mode_pub_;
  rclcpp::Publisher<px4_msgs::msg::VehicleRatesSetpoint>::SharedPtr rates_setpoint_pub_;
  rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_pub_;
  rclcpp::Publisher<ommpc_ros2::msg::ReferenceState>::SharedPtr reference_pub_;

  // Timers
  rclcpp::TimerBase::SharedPtr timer_;

  // FSM and Controller State
  FsmState fsm_state_ = WAIT_OFFBOARD;
  std::string trajectory_type_ = "hover";

  Odom_Data_t odom_data_;
  Imu_Data_t imu_data_;
  px4_msgs::msg::VehicleStatus vehicle_status_;

  Parameter_t param_;
  Trajectory_Data_t trajectory_data_;
  MpcController ommpc_controller_;

  // Control Flag and Timing
  bool enu_frame_ = false;  // NED internally in PX4 messages, let feed convert it
  bool vel_in_body_ = false;

  double thr2acc_ = 19.62;

  rclcpp::Time arming_detected_time_{ 0, 0, RCL_ROS_TIME };
  rclcpp::Time takeoff_start_time{ 0, 0, RCL_ROS_TIME };
  Eigen::Vector3d takeoff_start_pos_{ 0.0, 0.0, 0.0 };
  double takeoff_start_yaw_ = 0.0;

  rclcpp::Time tracking_start_time{ 0, 0, RCL_ROS_TIME };
  rclcpp::Time hover_start_time{ 0, 0, RCL_ROS_TIME };

  Eigen::Vector4d hover_pose_;  // [x, y, z, yaw] in ENU

  // Text trajectory data
  int line_cnt_ = 0;
  int number_of_steps_ = 0;
  std::vector<std::vector<double>> test_trajectory_;
  std::vector<Eigen::Vector3d> quad_positions_;
  std::vector<Eigen::Vector3d> quad_velocities_;
  std::vector<double> yaws_;

  void odomCallback(const px4_msgs::msg::VehicleOdometry::SharedPtr msg)
  {
    odom_data_.feed(msg, enu_frame_, vel_in_body_, this->get_clock()->now());
  }

  void imuCallback(const px4_msgs::msg::SensorCombined::SharedPtr msg)
  {
    imu_data_.feed(msg, enu_frame_, this->get_clock()->now());
  }

  void statusCallback(const px4_msgs::msg::VehicleStatus::SharedPtr msg)
  {
    vehicle_status_ = *msg;
  }

  void trajCallback(const ommpc_ros2::msg::PolyTraj::SharedPtr msg)
  {
    trajectory_data_.feed_from_traj_utils(msg, this->get_clock()->now());
  }

  void sendVehicleCommand(uint16_t command, float param1 = 0.0, float param2 = 0.0)
  {
    px4_msgs::msg::VehicleCommand msg;
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    msg.command = command;
    msg.param1 = param1;
    msg.param2 = param2;
    msg.target_system = 1;
    msg.target_component = 1;
    msg.source_system = 1;
    msg.source_component = 1;
    msg.from_external = true;
    vehicle_command_pub_->publish(msg);
  }

  void triggerArmAndOffboard()
  {
    if (vehicle_status_.arming_state != px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED)
    {
      RCLCPP_INFO(this->get_logger(), "Requesting ARM...");
      sendVehicleCommand(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);
    }
    if (vehicle_status_.nav_state != px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD)
    {
      RCLCPP_INFO(this->get_logger(), "Requesting OFFBOARD mode...");
      sendVehicleCommand(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0);
    }
  }

  void publishOffboardControlMode()
  {
    px4_msgs::msg::OffboardControlMode msg;
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    msg.position = false;
    msg.velocity = false;
    msg.acceleration = false;
    msg.attitude = false;
    msg.body_rate = true;
    offboard_control_mode_pub_->publish(msg);
  }

  void publishRatesSetpoint(const Controller_Output_t& output)
  {
    px4_msgs::msg::VehicleRatesSetpoint msg;
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;

    // FLU → FRD: flip pitch and yaw
    msg.roll = output.bodyrates(0);
    msg.pitch = -output.bodyrates(1);
    msg.yaw = -output.bodyrates(2);

    // MPC output.thrust is already normalized (range [0, 1]).
    double thrust_norm = std::max(0.1, std::min(0.9, output.thrust));

    msg.thrust_body[0] = 0.0f;
    msg.thrust_body[1] = 0.0f;
    msg.thrust_body[2] = -static_cast<float>(thrust_norm);

    rates_setpoint_pub_->publish(msg);
  }

  void publishReferenceState(const Eigen::VectorXd& xdes, const Eigen::VectorXd& udes)
  {
    if (xdes.size() < 10 || udes.size() < 4)
    {
      return;
    }
    ommpc_ros2::msg::ReferenceState msg;
    msg.timestamp = this->get_clock()->now();

    msg.position.x = xdes[0];
    msg.position.y = xdes[1];
    msg.position.z = xdes[2];

    msg.quaternion.w = xdes[3];
    msg.quaternion.x = xdes[4];
    msg.quaternion.y = xdes[5];
    msg.quaternion.z = xdes[6];

    msg.velocity.x = xdes[7];
    msg.velocity.y = xdes[8];
    msg.velocity.z = xdes[9];

    msg.thrust = udes[0];
    msg.body_rates.x = udes[1];
    msg.body_rates.y = udes[2];
    msg.body_rates.z = udes[3];

    reference_pub_->publish(msg);
  }

  double getYawFromQuaternion(const Eigen::Quaterniond& q)
  {
    return atan2(2 * (q.w() * q.z() + q.x() * q.y()), 1 - 2 * (q.y() * q.y() + q.z() * q.z()));
  }

  void getFigure8Reference(double t_since_start, std::vector<Eigen::Vector3d>& ref_p,
                           std::vector<Eigen::Vector3d>& ref_v, std::vector<double>& ref_yaw)
  {
    double omega = M_PI / 10.0;  // Period of 20 seconds
    double A = 1.5;
    double B = 1.5;
    double z_val = param_.takeoff_height;

    for (int i = 0; i <= nstep; ++i)
    {
      double t = t_since_start + i * param_.step_T;
      double x = A * sin(omega * t);
      double y = B * sin(2.0 * omega * t) / 2.0;

      double vx = A * omega * cos(omega * t);
      double vy = B * omega * cos(2.0 * omega * t);

      ref_p[i] = Eigen::Vector3d(takeoff_start_pos_(0) + x, takeoff_start_pos_(1) + y, z_val);
      ref_v[i] = Eigen::Vector3d(vx, vy, 0.0);
      ref_yaw[i] = atan2(vy, vx);
    }
  }

  void getCircleReference(double t_since_start, std::vector<Eigen::Vector3d>& ref_p,
                          std::vector<Eigen::Vector3d>& ref_v, std::vector<double>& ref_yaw)
  {
    double omega = M_PI / 10.0;  // Period of 20 seconds
    double R = 1.5;
    double z_val = param_.takeoff_height;

    for (int i = 0; i <= nstep; ++i)
    {
      double t = t_since_start + i * param_.step_T;
      double x = R * sin(omega * t);
      double y = R * (1.0 - cos(omega * t));

      double vx = R * omega * cos(omega * t);
      double vy = R * omega * sin(omega * t);

      ref_p[i] = Eigen::Vector3d(takeoff_start_pos_(0) + x, takeoff_start_pos_(1) + y, z_val);
      ref_v[i] = Eigen::Vector3d(vx, vy, 0.0);
      ref_yaw[i] = atan2(vy, vx);
    }
  }

  bool readDataFromFile()
  {
    try
    {
      std::string share_dir = ament_index_cpp::get_package_share_directory("ommpc_ros2");
      std::string traj_path = share_dir + "/" + param_.ref_filename;
      std::ifstream file(traj_path.c_str());
      std::string line;

      if (file.is_open())
      {
        RCLCPP_INFO(this->get_logger(), "Opened reference traj file: %s", traj_path.c_str());
        while (getline(file, line))
        {
          number_of_steps_++;
          std::istringstream linestream(line);
          std::vector<double> linedata;
          double number;

          while (linestream >> number)
          {
            linedata.push_back(number);
          }
          test_trajectory_.push_back(linedata);
        }
        file.close();
        RCLCPP_INFO(this->get_logger(), "Successfully read %d trajectory points.", number_of_steps_);
        return true;
      }
    }
    catch (const std::exception& e)
    {
      RCLCPP_ERROR(this->get_logger(), "Exception while loading traj file: %s", e.what());
    }
    return false;
  }

  void getTxtDes()
  {
    for (int cnt = line_cnt_; cnt <= line_cnt_ + nstep; cnt++)
    {
      int i = cnt - line_cnt_;
      if (nstep + cnt + 1 < number_of_steps_)
      {
        quad_positions_[i] = Eigen::Vector3d(test_trajectory_[line_cnt_ + i][0], test_trajectory_[line_cnt_ + i][1],
                                             test_trajectory_[line_cnt_ + i][2]);
        quad_velocities_[i] = Eigen::Vector3d(test_trajectory_[line_cnt_ + i][3], test_trajectory_[line_cnt_ + i][4],
                                              test_trajectory_[line_cnt_ + i][5]);
        yaws_[i] = test_trajectory_[line_cnt_ + i][6];
      }
      else
      {
        quad_positions_[i] =
            Eigen::Vector3d(test_trajectory_[number_of_steps_ - 1][0], test_trajectory_[number_of_steps_ - 1][1],
                            test_trajectory_[number_of_steps_ - 1][2]);
        quad_velocities_[i] = Eigen::Vector3d::Zero();
        yaws_[i] = test_trajectory_[number_of_steps_ - 1][6];
      }
    }

    line_cnt_++;
    if (line_cnt_ >= number_of_steps_)
    {
      line_cnt_ = number_of_steps_ - 1;
    }
  }

  void controlLoop()
  {
    publishOffboardControlMode();

    rclcpp::Time now = this->get_clock()->now();

    // Check if odometry feedback is received first
    if (!odom_data_.recv_new_msg)
    {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                           "[OmmpcNode] Waiting for odometry feedback...");
      return;
    }

    // 1. Handle Arming & Offboard Switching
    if (vehicle_status_.nav_state != px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD ||
        vehicle_status_.arming_state != px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED)
    {
      fsm_state_ = WAIT_OFFBOARD;

      // Publish a zero-thrust rates setpoint to allow entering offboard
      Controller_Output_t zero_u;
      zero_u.bodyrates.setZero();
      zero_u.thrust = 0.0;
      publishRatesSetpoint(zero_u);

      static int init_counter = 0;
      if (++init_counter >= 100)
      {  // Every 1s
        triggerArmAndOffboard();
        init_counter = 0;
      }
      return;
    }

    // 2. FSM Logic
    Controller_Output_t u;
    bool mpc_success = false;

    switch (fsm_state_)
    {
      case WAIT_OFFBOARD: {
        RCLCPP_INFO(this->get_logger(), "Offboard and Armed detected. Entering ARMED_WAIT for motor spool-up.");
        arming_detected_time_ = now;
        fsm_state_ = ARMED_WAIT;
        break;
      }

      case ARMED_WAIT: {
        // Send zero rates and idle/minimum thrust to keep drone on ground while motors spin up
        u.bodyrates.setZero();
        u.thrust = 0.0;
        mpc_success = true;

        if ((now - arming_detected_time_).seconds() >= 2.0)
        {
          RCLCPP_INFO(this->get_logger(), "Motors spooled up. Starting TAKEOFF.");
          takeoff_start_time = now;
          takeoff_start_pos_ = odom_data_.p;
          takeoff_start_yaw_ = getYawFromQuaternion(odom_data_.q);
          fsm_state_ = TAKEOFF;
        }
        break;
      }

      case TAKEOFF: {
        double t_since_takeoff = (now - takeoff_start_time).seconds();
        double target_z = takeoff_start_pos_(2) + param_.takeoff_land_speed * t_since_takeoff;
        if (target_z >= takeoff_start_pos_(2) + param_.takeoff_height)
        {
          target_z = takeoff_start_pos_(2) + param_.takeoff_height ;
        }

        // Closed-loop feedback: transition to HOVER only when actual altitude is close to target height
        double actual_altitude = odom_data_.p(2);
        double target_height = takeoff_start_pos_(2) + param_.takeoff_height;
        if (actual_altitude >= target_height - 0.05)
        {
          hover_pose_ << takeoff_start_pos_(0), takeoff_start_pos_(1), target_height, takeoff_start_yaw_;
          hover_start_time = now;
          fsm_state_ = HOVER;
          RCLCPP_INFO(this->get_logger(),
                      "Takeoff complete (feedback: actual altitude %.2f m reached target %.2f m). Entering HOVER.",
                      actual_altitude, target_height);
        }

        // Populate trajectory inputs for takeoff trajectory (ramp z)
        for (int i = 0; i <= nstep; ++i)
        {
          double future_t = t_since_takeoff + i * param_.step_T;
          double future_z = takeoff_start_pos_(2) + param_.takeoff_land_speed * future_t;
          double future_vz = param_.takeoff_land_speed;
          if (future_z > takeoff_start_pos_(2) + param_.takeoff_height)
          {
            future_z = takeoff_start_pos_(2) + param_.takeoff_height;
            future_vz = 0.0;
          }
          quad_positions_[i] = Eigen::Vector3d(takeoff_start_pos_(0), takeoff_start_pos_(1), future_z);
          quad_velocities_[i] = Eigen::Vector3d(0.0, 0.0, future_vz);
          yaws_[i] = takeoff_start_yaw_;
        }

        ommpc_controller_.setTextReference(quad_positions_, quad_velocities_, odom_data_, takeoff_start_yaw_, yaws_);
        mpc_success = ommpc_controller_.execMPC(odom_data_, u, now);
        break;
      }

      case HOVER: {
        // Run Hover NMPC
        ommpc_controller_.setHoverReference(hover_pose_);
        mpc_success = ommpc_controller_.execMPC(odom_data_, u, now);

        // Auto transition to tracking after 5s hover
        double t_since_hover = (now - hover_start_time).seconds();
        if (t_since_hover > 5.0 && trajectory_type_ != "hover")
        {
          RCLCPP_INFO(this->get_logger(), "Transitioning from HOVER to TRACKING (%s)", trajectory_type_.c_str());
          tracking_start_time = now;
          fsm_state_ = TRACKING;
        }
        break;
      }

      case TRACKING: {
        if (trajectory_type_ == "figure8")
        {
          double t_since_tracking = (now - tracking_start_time).seconds();
          getFigure8Reference(t_since_tracking, quad_positions_, quad_velocities_, yaws_);
          ommpc_controller_.setTextReference(quad_positions_, quad_velocities_, odom_data_, takeoff_start_yaw_, yaws_);
          mpc_success = ommpc_controller_.execMPC(odom_data_, u, now);
        }
        else if (trajectory_type_ == "circle")
        {
          double t_since_tracking = (now - tracking_start_time).seconds();
          getCircleReference(t_since_tracking, quad_positions_, quad_velocities_, yaws_);
          ommpc_controller_.setTextReference(quad_positions_, quad_velocities_, odom_data_, takeoff_start_yaw_, yaws_);
          mpc_success = ommpc_controller_.execMPC(odom_data_, u, now);
        }
        else if (trajectory_type_ == "text")
        {
          getTxtDes();
          ommpc_controller_.setTextReference(quad_positions_, quad_velocities_, odom_data_, takeoff_start_yaw_, yaws_);
          mpc_success = ommpc_controller_.execMPC(odom_data_, u, now);
        }
        else if (trajectory_type_ == "poly")
        {
          if (trajectory_data_.exec_traj == 1 && !trajectory_data_.traj_queue.empty())
          {
            oneTraj_Data_t& traj_info = trajectory_data_.traj_queue.front();
            double traj_time = (now - traj_info.traj_start_time).seconds();
            ommpc_controller_.setTrajectoryReference(traj_info.traj, traj_time, hover_pose_(3), traj_info.yaw_traj,
                                                     odom_data_);
            mpc_success = ommpc_controller_.execMPC(odom_data_, u, now);
          }
          else
          {
            // Trajectory ended, return to hover
            hover_pose_ << odom_data_.p(0), odom_data_.p(1), odom_data_.p(2), getYawFromQuaternion(odom_data_.q);
            fsm_state_ = HOVER;
            RCLCPP_INFO(this->get_logger(), "Poly Trajectory ended. Returning to HOVER.");
          }
        }
        break;
      }

      case LAND: {
        // Implement landing ramp if requested
        double t_since_land = (now - takeoff_start_time).seconds();
        double target_z = hover_pose_(2) - param_.takeoff_land_speed * t_since_land;
        if (target_z <= 0.05)
        {
          target_z = 0.0f;
          sendVehicleCommand(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0);
          fsm_state_ = WAIT_OFFBOARD;
          RCLCPP_INFO(this->get_logger(), "Landed and disarmed. Returning to WAIT_OFFBOARD.");
        }

        for (int i = 0; i <= nstep; ++i)
        {
          double future_t = t_since_land + i * param_.step_T;
          double future_z = hover_pose_(2) - param_.takeoff_land_speed * future_t;
          double future_vz = -param_.takeoff_land_speed;
          if (future_z < 0.0)
          {
            future_z = 0.0;
            future_vz = 0.0;
          }
          quad_positions_[i] = Eigen::Vector3d(hover_pose_(0), hover_pose_(1), future_z);
          quad_velocities_[i] = Eigen::Vector3d(0.0, 0.0, future_vz);
          yaws_[i] = hover_pose_(3);
        }

        ommpc_controller_.setTextReference(quad_positions_, quad_velocities_, odom_data_, hover_pose_(3), yaws_);
        mpc_success = ommpc_controller_.execMPC(odom_data_, u, now);
        break;
      }
    }

    // 3. Publish setpoint and update thrust model
    if (mpc_success)
    {
      publishRatesSetpoint(u);

      Eigen::VectorXd xdes(10);
      Eigen::VectorXd udes(4);
      ommpc_controller_.getDesiredStart(xdes, udes);
      publishReferenceState(xdes, udes);

      // Update recursive least squares thrust estimator when armed and in offboard
      if (vehicle_status_.arming_state == px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED &&
          vehicle_status_.nav_state == px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD &&
          (fsm_state_ != TAKEOFF || odom_data_.p(2) >= 0.15) && fsm_state_ != LAND)
      {
        ommpc_controller_.estimateThrustModel(imu_data_.a, now);
      }
    }
    else
    {
      RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "[OmmpcNode] MPC failed or not armed.");
    }
  }

public:
  OmmpcNode() : Node("ommpc_node")
  {
    // Declare and retrieve parameters
    this->declare_parameter("takeoff_height", 1.5);
    this->declare_parameter("takeoff_land_speed", 0.3);
    this->declare_parameter("trajectory", "hover");
    this->declare_parameter("ref_txt.enable", false);
    this->declare_parameter("ref_txt.time_step", 0.01);
    this->declare_parameter("ref_txt.ref_filename", "");
    this->declare_parameter("hover_percentage", 0.55);

    this->declare_parameter("MPC_params.Q_pos_xy", 2.0);
    this->declare_parameter("MPC_params.Q_pos_z", 2.0);
    this->declare_parameter("MPC_params.Q_velocity", 1.0);
    this->declare_parameter("MPC_params.Q_attitude_rp", 1.0);
    this->declare_parameter("MPC_params.Q_attitude_yaw", 10.0);
    this->declare_parameter("MPC_params.R_thrust", 0.1);
    this->declare_parameter("MPC_params.R_pitchroll", 0.01);
    this->declare_parameter("MPC_params.R_yaw", 0.1);
    this->declare_parameter("MPC_params.min_thrust", 0.1);
    this->declare_parameter("MPC_params.max_thrust", 0.9);
    this->declare_parameter("MPC_params.max_bodyrate_xy", 1.0);
    this->declare_parameter("MPC_params.max_bodyrate_z", 0.5);
    this->declare_parameter("MPC_params.state_cost_exponential", 0.0);
    this->declare_parameter("MPC_params.input_cost_exponential", 0.0);
    this->declare_parameter("MPC_params.step_T", 0.05);

    this->declare_parameter("use_fix_yaw", false);
    this->declare_parameter("use_trajectory_ending_pos", true);

    param_.takeoff_height = this->get_parameter("takeoff_height").as_double();
    param_.takeoff_land_speed = this->get_parameter("takeoff_land_speed").as_double();
    trajectory_type_ = this->get_parameter("trajectory").as_string();
    param_.use_ref_txt = this->get_parameter("ref_txt.enable").as_bool();
    param_.ref_time_step = this->get_parameter("ref_txt.time_step").as_double();
    param_.ref_filename = this->get_parameter("ref_txt.ref_filename").as_string();
    param_.hover_percent = this->get_parameter("hover_percentage").as_double();

    param_.Q_pos_xy = this->get_parameter("MPC_params.Q_pos_xy").as_double();
    param_.Q_pos_z = this->get_parameter("MPC_params.Q_pos_z").as_double();
    param_.Q_velocity = this->get_parameter("MPC_params.Q_velocity").as_double();
    param_.Q_attitude_rp = this->get_parameter("MPC_params.Q_attitude_rp").as_double();
    param_.Q_attitude_yaw = this->get_parameter("MPC_params.Q_attitude_yaw").as_double();
    param_.R_thrust = this->get_parameter("MPC_params.R_thrust").as_double();
    param_.R_pitchroll = this->get_parameter("MPC_params.R_pitchroll").as_double();
    param_.R_yaw = this->get_parameter("MPC_params.R_yaw").as_double();
    param_.min_thrust = this->get_parameter("MPC_params.min_thrust").as_double();
    param_.max_thrust = this->get_parameter("MPC_params.max_thrust").as_double();
    param_.max_bodyrate_xy = this->get_parameter("MPC_params.max_bodyrate_xy").as_double();
    param_.max_bodyrate_z = this->get_parameter("MPC_params.max_bodyrate_z").as_double();
    param_.state_cost_exponential = this->get_parameter("MPC_params.state_cost_exponential").as_double();
    param_.input_cost_exponential = this->get_parameter("MPC_params.input_cost_exponential").as_double();
    param_.step_T = this->get_parameter("MPC_params.step_T").as_double();

    param_.use_fix_yaw = this->get_parameter("use_fix_yaw").as_bool();
    param_.use_trajectory_ending_pos = this->get_parameter("use_trajectory_ending_pos").as_bool();

    thr2acc_ = 9.81 / param_.hover_percent;

    if (param_.use_ref_txt)
    {
      RCLCPP_INFO(this->get_logger(), "Loading text trajectory...");
      if (!readDataFromFile())
      {
        RCLCPP_ERROR(this->get_logger(), "Failed to read reference file!");
      }
    }

    // Allocate memory for vector trajectory
    quad_positions_.resize(nstep + 1);
    quad_velocities_.resize(nstep + 1);
    yaws_.resize(nstep + 1);

    // Initialize MPC Controller
    ommpc_controller_.init(param_);

    // Publishers
    offboard_control_mode_pub_ = this->create_publisher<px4_msgs::msg::OffboardControlMode>(
        "/fmu/in/offboard_control_mode", rclcpp::QoS(10).best_effort());
    rates_setpoint_pub_ = this->create_publisher<px4_msgs::msg::VehicleRatesSetpoint>("/fmu/in/vehicle_rates_setpoint",
                                                                                      rclcpp::QoS(10).best_effort());
    vehicle_command_pub_ =
        this->create_publisher<px4_msgs::msg::VehicleCommand>("/fmu/in/vehicle_command", rclcpp::QoS(10).best_effort());
    reference_pub_ = this->create_publisher<ommpc_ros2::msg::ReferenceState>("/ommpc/current_reference", 10);

    odom_sub_ = this->create_subscription<px4_msgs::msg::VehicleOdometry>(
        "/fmu/out/vehicle_odometry", rclcpp::QoS(10).best_effort(),
        std::bind(&OmmpcNode::odomCallback, this, std::placeholders::_1));
    imu_sub_ = this->create_subscription<px4_msgs::msg::SensorCombined>(
        "/fmu/out/sensor_combined", rclcpp::QoS(10).best_effort(),
        std::bind(&OmmpcNode::imuCallback, this, std::placeholders::_1));
    status_sub_ = this->create_subscription<px4_msgs::msg::VehicleStatus>(
        "/fmu/out/vehicle_status_v1", rclcpp::QoS(10).best_effort(),
        std::bind(&OmmpcNode::statusCallback, this, std::placeholders::_1));
    traj_sub_ = this->create_subscription<ommpc_ros2::msg::PolyTraj>(
        "/drone_0_planning/trajectory", 10, std::bind(&OmmpcNode::trajCallback, this, std::placeholders::_1));

    // Main Control Loop Timer (100 Hz = 10 ms)
    timer_ = this->create_wall_timer(std::chrono::milliseconds(10), std::bind(&OmmpcNode::controlLoop, this));

    RCLCPP_INFO(this->get_logger(), "OMMPC ROS 2 Controller Node Initialized.");
  }
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OmmpcNode>());
  rclcpp::shutdown();
  return 0;
}
