// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from ommpc_ros2:msg/PolyTraj.idl
// generated code does not contain a copyright notice

#ifndef OMMPC_ROS2__MSG__DETAIL__POLY_TRAJ__BUILDER_HPP_
#define OMMPC_ROS2__MSG__DETAIL__POLY_TRAJ__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "ommpc_ros2/msg/detail/poly_traj__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace ommpc_ros2
{

namespace msg
{

namespace builder
{

class Init_PolyTraj_duration
{
public:
  explicit Init_PolyTraj_duration(::ommpc_ros2::msg::PolyTraj & msg)
  : msg_(msg)
  {}
  ::ommpc_ros2::msg::PolyTraj duration(::ommpc_ros2::msg::PolyTraj::_duration_type arg)
  {
    msg_.duration = std::move(arg);
    return std::move(msg_);
  }

private:
  ::ommpc_ros2::msg::PolyTraj msg_;
};

class Init_PolyTraj_coef_z
{
public:
  explicit Init_PolyTraj_coef_z(::ommpc_ros2::msg::PolyTraj & msg)
  : msg_(msg)
  {}
  Init_PolyTraj_duration coef_z(::ommpc_ros2::msg::PolyTraj::_coef_z_type arg)
  {
    msg_.coef_z = std::move(arg);
    return Init_PolyTraj_duration(msg_);
  }

private:
  ::ommpc_ros2::msg::PolyTraj msg_;
};

class Init_PolyTraj_coef_y
{
public:
  explicit Init_PolyTraj_coef_y(::ommpc_ros2::msg::PolyTraj & msg)
  : msg_(msg)
  {}
  Init_PolyTraj_coef_z coef_y(::ommpc_ros2::msg::PolyTraj::_coef_y_type arg)
  {
    msg_.coef_y = std::move(arg);
    return Init_PolyTraj_coef_z(msg_);
  }

private:
  ::ommpc_ros2::msg::PolyTraj msg_;
};

class Init_PolyTraj_coef_x
{
public:
  explicit Init_PolyTraj_coef_x(::ommpc_ros2::msg::PolyTraj & msg)
  : msg_(msg)
  {}
  Init_PolyTraj_coef_y coef_x(::ommpc_ros2::msg::PolyTraj::_coef_x_type arg)
  {
    msg_.coef_x = std::move(arg);
    return Init_PolyTraj_coef_y(msg_);
  }

private:
  ::ommpc_ros2::msg::PolyTraj msg_;
};

class Init_PolyTraj_order
{
public:
  explicit Init_PolyTraj_order(::ommpc_ros2::msg::PolyTraj & msg)
  : msg_(msg)
  {}
  Init_PolyTraj_coef_x order(::ommpc_ros2::msg::PolyTraj::_order_type arg)
  {
    msg_.order = std::move(arg);
    return Init_PolyTraj_coef_x(msg_);
  }

private:
  ::ommpc_ros2::msg::PolyTraj msg_;
};

class Init_PolyTraj_start_time
{
public:
  explicit Init_PolyTraj_start_time(::ommpc_ros2::msg::PolyTraj & msg)
  : msg_(msg)
  {}
  Init_PolyTraj_order start_time(::ommpc_ros2::msg::PolyTraj::_start_time_type arg)
  {
    msg_.start_time = std::move(arg);
    return Init_PolyTraj_order(msg_);
  }

private:
  ::ommpc_ros2::msg::PolyTraj msg_;
};

class Init_PolyTraj_traj_id
{
public:
  explicit Init_PolyTraj_traj_id(::ommpc_ros2::msg::PolyTraj & msg)
  : msg_(msg)
  {}
  Init_PolyTraj_start_time traj_id(::ommpc_ros2::msg::PolyTraj::_traj_id_type arg)
  {
    msg_.traj_id = std::move(arg);
    return Init_PolyTraj_start_time(msg_);
  }

private:
  ::ommpc_ros2::msg::PolyTraj msg_;
};

class Init_PolyTraj_drone_id
{
public:
  Init_PolyTraj_drone_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_PolyTraj_traj_id drone_id(::ommpc_ros2::msg::PolyTraj::_drone_id_type arg)
  {
    msg_.drone_id = std::move(arg);
    return Init_PolyTraj_traj_id(msg_);
  }

private:
  ::ommpc_ros2::msg::PolyTraj msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::ommpc_ros2::msg::PolyTraj>()
{
  return ommpc_ros2::msg::builder::Init_PolyTraj_drone_id();
}

}  // namespace ommpc_ros2

#endif  // OMMPC_ROS2__MSG__DETAIL__POLY_TRAJ__BUILDER_HPP_
