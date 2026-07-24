// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from ommpc_ros2:msg/ReferenceState.idl
// generated code does not contain a copyright notice

#ifndef OMMPC_ROS2__MSG__DETAIL__REFERENCE_STATE__TRAITS_HPP_
#define OMMPC_ROS2__MSG__DETAIL__REFERENCE_STATE__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "ommpc_ros2/msg/detail/reference_state__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'timestamp'
#include "builtin_interfaces/msg/detail/time__traits.hpp"
// Member 'position'
#include "geometry_msgs/msg/detail/point__traits.hpp"
// Member 'velocity'
// Member 'body_rates'
#include "geometry_msgs/msg/detail/vector3__traits.hpp"
// Member 'quaternion'
#include "geometry_msgs/msg/detail/quaternion__traits.hpp"

namespace ommpc_ros2
{

namespace msg
{

inline void to_flow_style_yaml(
  const ReferenceState & msg,
  std::ostream & out)
{
  out << "{";
  // member: timestamp
  {
    out << "timestamp: ";
    to_flow_style_yaml(msg.timestamp, out);
    out << ", ";
  }

  // member: position
  {
    out << "position: ";
    to_flow_style_yaml(msg.position, out);
    out << ", ";
  }

  // member: velocity
  {
    out << "velocity: ";
    to_flow_style_yaml(msg.velocity, out);
    out << ", ";
  }

  // member: quaternion
  {
    out << "quaternion: ";
    to_flow_style_yaml(msg.quaternion, out);
    out << ", ";
  }

  // member: body_rates
  {
    out << "body_rates: ";
    to_flow_style_yaml(msg.body_rates, out);
    out << ", ";
  }

  // member: thrust
  {
    out << "thrust: ";
    rosidl_generator_traits::value_to_yaml(msg.thrust, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const ReferenceState & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: timestamp
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "timestamp:\n";
    to_block_style_yaml(msg.timestamp, out, indentation + 2);
  }

  // member: position
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "position:\n";
    to_block_style_yaml(msg.position, out, indentation + 2);
  }

  // member: velocity
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "velocity:\n";
    to_block_style_yaml(msg.velocity, out, indentation + 2);
  }

  // member: quaternion
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "quaternion:\n";
    to_block_style_yaml(msg.quaternion, out, indentation + 2);
  }

  // member: body_rates
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "body_rates:\n";
    to_block_style_yaml(msg.body_rates, out, indentation + 2);
  }

  // member: thrust
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "thrust: ";
    rosidl_generator_traits::value_to_yaml(msg.thrust, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const ReferenceState & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace ommpc_ros2

namespace rosidl_generator_traits
{

[[deprecated("use ommpc_ros2::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const ommpc_ros2::msg::ReferenceState & msg,
  std::ostream & out, size_t indentation = 0)
{
  ommpc_ros2::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use ommpc_ros2::msg::to_yaml() instead")]]
inline std::string to_yaml(const ommpc_ros2::msg::ReferenceState & msg)
{
  return ommpc_ros2::msg::to_yaml(msg);
}

template<>
inline const char * data_type<ommpc_ros2::msg::ReferenceState>()
{
  return "ommpc_ros2::msg::ReferenceState";
}

template<>
inline const char * name<ommpc_ros2::msg::ReferenceState>()
{
  return "ommpc_ros2/msg/ReferenceState";
}

template<>
struct has_fixed_size<ommpc_ros2::msg::ReferenceState>
  : std::integral_constant<bool, has_fixed_size<builtin_interfaces::msg::Time>::value && has_fixed_size<geometry_msgs::msg::Point>::value && has_fixed_size<geometry_msgs::msg::Quaternion>::value && has_fixed_size<geometry_msgs::msg::Vector3>::value> {};

template<>
struct has_bounded_size<ommpc_ros2::msg::ReferenceState>
  : std::integral_constant<bool, has_bounded_size<builtin_interfaces::msg::Time>::value && has_bounded_size<geometry_msgs::msg::Point>::value && has_bounded_size<geometry_msgs::msg::Quaternion>::value && has_bounded_size<geometry_msgs::msg::Vector3>::value> {};

template<>
struct is_message<ommpc_ros2::msg::ReferenceState>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // OMMPC_ROS2__MSG__DETAIL__REFERENCE_STATE__TRAITS_HPP_
