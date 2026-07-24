// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__type_support.cpp.em
// with input from ommpc_ros2:msg/ReferenceState.idl
// generated code does not contain a copyright notice
#include "ommpc_ros2/msg/detail/reference_state__rosidl_typesupport_fastrtps_cpp.hpp"
#include "ommpc_ros2/msg/detail/reference_state__struct.hpp"

#include <limits>
#include <stdexcept>
#include <string>
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_fastrtps_cpp/identifier.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_fastrtps_cpp/wstring_conversion.hpp"
#include "fastcdr/Cdr.h"


// forward declaration of message dependencies and their conversion functions
namespace builtin_interfaces
{
namespace msg
{
namespace typesupport_fastrtps_cpp
{
bool cdr_serialize(
  const builtin_interfaces::msg::Time &,
  eprosima::fastcdr::Cdr &);
bool cdr_deserialize(
  eprosima::fastcdr::Cdr &,
  builtin_interfaces::msg::Time &);
size_t get_serialized_size(
  const builtin_interfaces::msg::Time &,
  size_t current_alignment);
size_t
max_serialized_size_Time(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);
}  // namespace typesupport_fastrtps_cpp
}  // namespace msg
}  // namespace builtin_interfaces

namespace geometry_msgs
{
namespace msg
{
namespace typesupport_fastrtps_cpp
{
bool cdr_serialize(
  const geometry_msgs::msg::Point &,
  eprosima::fastcdr::Cdr &);
bool cdr_deserialize(
  eprosima::fastcdr::Cdr &,
  geometry_msgs::msg::Point &);
size_t get_serialized_size(
  const geometry_msgs::msg::Point &,
  size_t current_alignment);
size_t
max_serialized_size_Point(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);
}  // namespace typesupport_fastrtps_cpp
}  // namespace msg
}  // namespace geometry_msgs

namespace geometry_msgs
{
namespace msg
{
namespace typesupport_fastrtps_cpp
{
bool cdr_serialize(
  const geometry_msgs::msg::Vector3 &,
  eprosima::fastcdr::Cdr &);
bool cdr_deserialize(
  eprosima::fastcdr::Cdr &,
  geometry_msgs::msg::Vector3 &);
size_t get_serialized_size(
  const geometry_msgs::msg::Vector3 &,
  size_t current_alignment);
size_t
max_serialized_size_Vector3(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);
}  // namespace typesupport_fastrtps_cpp
}  // namespace msg
}  // namespace geometry_msgs

namespace geometry_msgs
{
namespace msg
{
namespace typesupport_fastrtps_cpp
{
bool cdr_serialize(
  const geometry_msgs::msg::Quaternion &,
  eprosima::fastcdr::Cdr &);
bool cdr_deserialize(
  eprosima::fastcdr::Cdr &,
  geometry_msgs::msg::Quaternion &);
size_t get_serialized_size(
  const geometry_msgs::msg::Quaternion &,
  size_t current_alignment);
size_t
max_serialized_size_Quaternion(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);
}  // namespace typesupport_fastrtps_cpp
}  // namespace msg
}  // namespace geometry_msgs

// functions for geometry_msgs::msg::Vector3 already declared above


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
  eprosima::fastcdr::Cdr & cdr)
{
  // Member: timestamp
  builtin_interfaces::msg::typesupport_fastrtps_cpp::cdr_serialize(
    ros_message.timestamp,
    cdr);
  // Member: position
  geometry_msgs::msg::typesupport_fastrtps_cpp::cdr_serialize(
    ros_message.position,
    cdr);
  // Member: velocity
  geometry_msgs::msg::typesupport_fastrtps_cpp::cdr_serialize(
    ros_message.velocity,
    cdr);
  // Member: quaternion
  geometry_msgs::msg::typesupport_fastrtps_cpp::cdr_serialize(
    ros_message.quaternion,
    cdr);
  // Member: body_rates
  geometry_msgs::msg::typesupport_fastrtps_cpp::cdr_serialize(
    ros_message.body_rates,
    cdr);
  // Member: thrust
  cdr << ros_message.thrust;
  return true;
}

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_ommpc_ros2
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  ommpc_ros2::msg::ReferenceState & ros_message)
{
  // Member: timestamp
  builtin_interfaces::msg::typesupport_fastrtps_cpp::cdr_deserialize(
    cdr, ros_message.timestamp);

  // Member: position
  geometry_msgs::msg::typesupport_fastrtps_cpp::cdr_deserialize(
    cdr, ros_message.position);

  // Member: velocity
  geometry_msgs::msg::typesupport_fastrtps_cpp::cdr_deserialize(
    cdr, ros_message.velocity);

  // Member: quaternion
  geometry_msgs::msg::typesupport_fastrtps_cpp::cdr_deserialize(
    cdr, ros_message.quaternion);

  // Member: body_rates
  geometry_msgs::msg::typesupport_fastrtps_cpp::cdr_deserialize(
    cdr, ros_message.body_rates);

  // Member: thrust
  cdr >> ros_message.thrust;

  return true;
}  // NOLINT(readability/fn_size)

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_ommpc_ros2
get_serialized_size(
  const ommpc_ros2::msg::ReferenceState & ros_message,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Member: timestamp

  current_alignment +=
    builtin_interfaces::msg::typesupport_fastrtps_cpp::get_serialized_size(
    ros_message.timestamp, current_alignment);
  // Member: position

  current_alignment +=
    geometry_msgs::msg::typesupport_fastrtps_cpp::get_serialized_size(
    ros_message.position, current_alignment);
  // Member: velocity

  current_alignment +=
    geometry_msgs::msg::typesupport_fastrtps_cpp::get_serialized_size(
    ros_message.velocity, current_alignment);
  // Member: quaternion

  current_alignment +=
    geometry_msgs::msg::typesupport_fastrtps_cpp::get_serialized_size(
    ros_message.quaternion, current_alignment);
  // Member: body_rates

  current_alignment +=
    geometry_msgs::msg::typesupport_fastrtps_cpp::get_serialized_size(
    ros_message.body_rates, current_alignment);
  // Member: thrust
  {
    size_t item_size = sizeof(ros_message.thrust);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_ommpc_ros2
max_serialized_size_ReferenceState(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  size_t last_member_size = 0;
  (void)last_member_size;
  (void)padding;
  (void)wchar_size;

  full_bounded = true;
  is_plain = true;


  // Member: timestamp
  {
    size_t array_size = 1;


    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size =
        builtin_interfaces::msg::typesupport_fastrtps_cpp::max_serialized_size_Time(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }

  // Member: position
  {
    size_t array_size = 1;


    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size =
        geometry_msgs::msg::typesupport_fastrtps_cpp::max_serialized_size_Point(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }

  // Member: velocity
  {
    size_t array_size = 1;


    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size =
        geometry_msgs::msg::typesupport_fastrtps_cpp::max_serialized_size_Vector3(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }

  // Member: quaternion
  {
    size_t array_size = 1;


    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size =
        geometry_msgs::msg::typesupport_fastrtps_cpp::max_serialized_size_Quaternion(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }

  // Member: body_rates
  {
    size_t array_size = 1;


    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size =
        geometry_msgs::msg::typesupport_fastrtps_cpp::max_serialized_size_Vector3(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }

  // Member: thrust
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = ommpc_ros2::msg::ReferenceState;
    is_plain =
      (
      offsetof(DataType, thrust) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static bool _ReferenceState__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  auto typed_message =
    static_cast<const ommpc_ros2::msg::ReferenceState *>(
    untyped_ros_message);
  return cdr_serialize(*typed_message, cdr);
}

static bool _ReferenceState__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  auto typed_message =
    static_cast<ommpc_ros2::msg::ReferenceState *>(
    untyped_ros_message);
  return cdr_deserialize(cdr, *typed_message);
}

static uint32_t _ReferenceState__get_serialized_size(
  const void * untyped_ros_message)
{
  auto typed_message =
    static_cast<const ommpc_ros2::msg::ReferenceState *>(
    untyped_ros_message);
  return static_cast<uint32_t>(get_serialized_size(*typed_message, 0));
}

static size_t _ReferenceState__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_ReferenceState(full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}

static message_type_support_callbacks_t _ReferenceState__callbacks = {
  "ommpc_ros2::msg",
  "ReferenceState",
  _ReferenceState__cdr_serialize,
  _ReferenceState__cdr_deserialize,
  _ReferenceState__get_serialized_size,
  _ReferenceState__max_serialized_size
};

static rosidl_message_type_support_t _ReferenceState__handle = {
  rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
  &_ReferenceState__callbacks,
  get_message_typesupport_handle_function,
};

}  // namespace typesupport_fastrtps_cpp

}  // namespace msg

}  // namespace ommpc_ros2

namespace rosidl_typesupport_fastrtps_cpp
{

template<>
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_EXPORT_ommpc_ros2
const rosidl_message_type_support_t *
get_message_type_support_handle<ommpc_ros2::msg::ReferenceState>()
{
  return &ommpc_ros2::msg::typesupport_fastrtps_cpp::_ReferenceState__handle;
}

}  // namespace rosidl_typesupport_fastrtps_cpp

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, ommpc_ros2, msg, ReferenceState)() {
  return &ommpc_ros2::msg::typesupport_fastrtps_cpp::_ReferenceState__handle;
}

#ifdef __cplusplus
}
#endif
