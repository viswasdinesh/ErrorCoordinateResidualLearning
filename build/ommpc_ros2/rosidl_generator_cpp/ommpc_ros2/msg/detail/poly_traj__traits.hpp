// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from ommpc_ros2:msg/PolyTraj.idl
// generated code does not contain a copyright notice

#ifndef OMMPC_ROS2__MSG__DETAIL__POLY_TRAJ__TRAITS_HPP_
#define OMMPC_ROS2__MSG__DETAIL__POLY_TRAJ__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "ommpc_ros2/msg/detail/poly_traj__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'start_time'
#include "builtin_interfaces/msg/detail/time__traits.hpp"

namespace ommpc_ros2
{

namespace msg
{

inline void to_flow_style_yaml(
  const PolyTraj & msg,
  std::ostream & out)
{
  out << "{";
  // member: drone_id
  {
    out << "drone_id: ";
    rosidl_generator_traits::value_to_yaml(msg.drone_id, out);
    out << ", ";
  }

  // member: traj_id
  {
    out << "traj_id: ";
    rosidl_generator_traits::value_to_yaml(msg.traj_id, out);
    out << ", ";
  }

  // member: start_time
  {
    out << "start_time: ";
    to_flow_style_yaml(msg.start_time, out);
    out << ", ";
  }

  // member: order
  {
    out << "order: ";
    rosidl_generator_traits::value_to_yaml(msg.order, out);
    out << ", ";
  }

  // member: coef_x
  {
    if (msg.coef_x.size() == 0) {
      out << "coef_x: []";
    } else {
      out << "coef_x: [";
      size_t pending_items = msg.coef_x.size();
      for (auto item : msg.coef_x) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: coef_y
  {
    if (msg.coef_y.size() == 0) {
      out << "coef_y: []";
    } else {
      out << "coef_y: [";
      size_t pending_items = msg.coef_y.size();
      for (auto item : msg.coef_y) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: coef_z
  {
    if (msg.coef_z.size() == 0) {
      out << "coef_z: []";
    } else {
      out << "coef_z: [";
      size_t pending_items = msg.coef_z.size();
      for (auto item : msg.coef_z) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: duration
  {
    if (msg.duration.size() == 0) {
      out << "duration: []";
    } else {
      out << "duration: [";
      size_t pending_items = msg.duration.size();
      for (auto item : msg.duration) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const PolyTraj & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: drone_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "drone_id: ";
    rosidl_generator_traits::value_to_yaml(msg.drone_id, out);
    out << "\n";
  }

  // member: traj_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "traj_id: ";
    rosidl_generator_traits::value_to_yaml(msg.traj_id, out);
    out << "\n";
  }

  // member: start_time
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "start_time:\n";
    to_block_style_yaml(msg.start_time, out, indentation + 2);
  }

  // member: order
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "order: ";
    rosidl_generator_traits::value_to_yaml(msg.order, out);
    out << "\n";
  }

  // member: coef_x
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.coef_x.size() == 0) {
      out << "coef_x: []\n";
    } else {
      out << "coef_x:\n";
      for (auto item : msg.coef_x) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: coef_y
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.coef_y.size() == 0) {
      out << "coef_y: []\n";
    } else {
      out << "coef_y:\n";
      for (auto item : msg.coef_y) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: coef_z
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.coef_z.size() == 0) {
      out << "coef_z: []\n";
    } else {
      out << "coef_z:\n";
      for (auto item : msg.coef_z) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: duration
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.duration.size() == 0) {
      out << "duration: []\n";
    } else {
      out << "duration:\n";
      for (auto item : msg.duration) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const PolyTraj & msg, bool use_flow_style = false)
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
  const ommpc_ros2::msg::PolyTraj & msg,
  std::ostream & out, size_t indentation = 0)
{
  ommpc_ros2::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use ommpc_ros2::msg::to_yaml() instead")]]
inline std::string to_yaml(const ommpc_ros2::msg::PolyTraj & msg)
{
  return ommpc_ros2::msg::to_yaml(msg);
}

template<>
inline const char * data_type<ommpc_ros2::msg::PolyTraj>()
{
  return "ommpc_ros2::msg::PolyTraj";
}

template<>
inline const char * name<ommpc_ros2::msg::PolyTraj>()
{
  return "ommpc_ros2/msg/PolyTraj";
}

template<>
struct has_fixed_size<ommpc_ros2::msg::PolyTraj>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<ommpc_ros2::msg::PolyTraj>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<ommpc_ros2::msg::PolyTraj>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // OMMPC_ROS2__MSG__DETAIL__POLY_TRAJ__TRAITS_HPP_
