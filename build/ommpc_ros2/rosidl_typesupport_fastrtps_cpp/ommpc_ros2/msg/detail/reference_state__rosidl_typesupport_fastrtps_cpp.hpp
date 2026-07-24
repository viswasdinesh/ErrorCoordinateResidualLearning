// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__rosidl_typesupport_fastrtps_cpp.hpp.em
// with input from ommpc_ros2:msg/ReferenceState.idl
// generated code does not contain a copyright notice

#ifndef OMMPC_ROS2__MSG__DETAIL__REFERENCE_STATE__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_
#define OMMPC_ROS2__MSG__DETAIL__REFERENCE_STATE__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_

#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_interface/macros.h"
#include "ommpc_ros2/msg/rosidl_typesupport_fastrtps_cpp__visibility_control.h"
#include "ommpc_ros2/msg/detail/reference_state__struct.hpp"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

#include "fastcdr/Cdr.h"

namespace ommpc_ros2
{

namespace msg
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_ommpc_ros2
cdr_serialize(
  const ommpc_ros2::msg::ReferenceState & ros_message,
  eprosima::fastcdr::Cdr & cdr);

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_ommpc_ros2
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  ommpc_ros2::msg::ReferenceState & ros_message);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_ommpc_ros2
get_serialized_size(
  const ommpc_ros2::msg::ReferenceState & ros_message,
  size_t current_alignment);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_ommpc_ros2
max_serialized_size_ReferenceState(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

}  // namespace typesupport_fastrtps_cpp

}  // namespace msg

}  // namespace ommpc_ros2

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_ommpc_ros2
const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, ommpc_ros2, msg, ReferenceState)();

#ifdef __cplusplus
}
#endif

#endif  // OMMPC_ROS2__MSG__DETAIL__REFERENCE_STATE__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_
