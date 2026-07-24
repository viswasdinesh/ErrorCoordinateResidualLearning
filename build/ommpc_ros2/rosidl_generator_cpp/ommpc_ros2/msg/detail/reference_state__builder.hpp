// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from ommpc_ros2:msg/ReferenceState.idl
// generated code does not contain a copyright notice

#ifndef OMMPC_ROS2__MSG__DETAIL__REFERENCE_STATE__BUILDER_HPP_
#define OMMPC_ROS2__MSG__DETAIL__REFERENCE_STATE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "ommpc_ros2/msg/detail/reference_state__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace ommpc_ros2
{

namespace msg
{

namespace builder
{

class Init_ReferenceState_thrust
{
public:
  explicit Init_ReferenceState_thrust(::ommpc_ros2::msg::ReferenceState & msg)
  : msg_(msg)
  {}
  ::ommpc_ros2::msg::ReferenceState thrust(::ommpc_ros2::msg::ReferenceState::_thrust_type arg)
  {
    msg_.thrust = std::move(arg);
    return std::move(msg_);
  }

private:
  ::ommpc_ros2::msg::ReferenceState msg_;
};

class Init_ReferenceState_body_rates
{
public:
  explicit Init_ReferenceState_body_rates(::ommpc_ros2::msg::ReferenceState & msg)
  : msg_(msg)
  {}
  Init_ReferenceState_thrust body_rates(::ommpc_ros2::msg::ReferenceState::_body_rates_type arg)
  {
    msg_.body_rates = std::move(arg);
    return Init_ReferenceState_thrust(msg_);
  }

private:
  ::ommpc_ros2::msg::ReferenceState msg_;
};

class Init_ReferenceState_quaternion
{
public:
  explicit Init_ReferenceState_quaternion(::ommpc_ros2::msg::ReferenceState & msg)
  : msg_(msg)
  {}
  Init_ReferenceState_body_rates quaternion(::ommpc_ros2::msg::ReferenceState::_quaternion_type arg)
  {
    msg_.quaternion = std::move(arg);
    return Init_ReferenceState_body_rates(msg_);
  }

private:
  ::ommpc_ros2::msg::ReferenceState msg_;
};

class Init_ReferenceState_velocity
{
public:
  explicit Init_ReferenceState_velocity(::ommpc_ros2::msg::ReferenceState & msg)
  : msg_(msg)
  {}
  Init_ReferenceState_quaternion velocity(::ommpc_ros2::msg::ReferenceState::_velocity_type arg)
  {
    msg_.velocity = std::move(arg);
    return Init_ReferenceState_quaternion(msg_);
  }

private:
  ::ommpc_ros2::msg::ReferenceState msg_;
};

class Init_ReferenceState_position
{
public:
  explicit Init_ReferenceState_position(::ommpc_ros2::msg::ReferenceState & msg)
  : msg_(msg)
  {}
  Init_ReferenceState_velocity position(::ommpc_ros2::msg::ReferenceState::_position_type arg)
  {
    msg_.position = std::move(arg);
    return Init_ReferenceState_velocity(msg_);
  }

private:
  ::ommpc_ros2::msg::ReferenceState msg_;
};

class Init_ReferenceState_timestamp
{
public:
  Init_ReferenceState_timestamp()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ReferenceState_position timestamp(::ommpc_ros2::msg::ReferenceState::_timestamp_type arg)
  {
    msg_.timestamp = std::move(arg);
    return Init_ReferenceState_position(msg_);
  }

private:
  ::ommpc_ros2::msg::ReferenceState msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::ommpc_ros2::msg::ReferenceState>()
{
  return ommpc_ros2::msg::builder::Init_ReferenceState_timestamp();
}

}  // namespace ommpc_ros2

#endif  // OMMPC_ROS2__MSG__DETAIL__REFERENCE_STATE__BUILDER_HPP_
