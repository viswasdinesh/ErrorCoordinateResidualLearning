// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__type_support.cpp.em
// with input from ommpc_ros2:msg/PolyTraj.idl
// generated code does not contain a copyright notice
#include "ommpc_ros2/msg/detail/poly_traj__rosidl_typesupport_fastrtps_cpp.hpp"
#include "ommpc_ros2/msg/detail/poly_traj__struct.hpp"

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


namespace ommpc_ros2
{

namespace msg
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_ommpc_ros2
cdr_serialize(
  const ommpc_ros2::msg::PolyTraj & ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Member: drone_id
  cdr << ros_message.drone_id;
  // Member: traj_id
  cdr << ros_message.traj_id;
  // Member: start_time
  builtin_interfaces::msg::typesupport_fastrtps_cpp::cdr_serialize(
    ros_message.start_time,
    cdr);
  // Member: order
  cdr << ros_message.order;
  // Member: coef_x
  {
    cdr << ros_message.coef_x;
  }
  // Member: coef_y
  {
    cdr << ros_message.coef_y;
  }
  // Member: coef_z
  {
    cdr << ros_message.coef_z;
  }
  // Member: duration
  {
    cdr << ros_message.duration;
  }
  return true;
}

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_ommpc_ros2
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  ommpc_ros2::msg::PolyTraj & ros_message)
{
  // Member: drone_id
  cdr >> ros_message.drone_id;

  // Member: traj_id
  cdr >> ros_message.traj_id;

  // Member: start_time
  builtin_interfaces::msg::typesupport_fastrtps_cpp::cdr_deserialize(
    cdr, ros_message.start_time);

  // Member: order
  cdr >> ros_message.order;

  // Member: coef_x
  {
    cdr >> ros_message.coef_x;
  }

  // Member: coef_y
  {
    cdr >> ros_message.coef_y;
  }

  // Member: coef_z
  {
    cdr >> ros_message.coef_z;
  }

  // Member: duration
  {
    cdr >> ros_message.duration;
  }

  return true;
}  // NOLINT(readability/fn_size)

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_ommpc_ros2
get_serialized_size(
  const ommpc_ros2::msg::PolyTraj & ros_message,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Member: drone_id
  {
    size_t item_size = sizeof(ros_message.drone_id);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: traj_id
  {
    size_t item_size = sizeof(ros_message.traj_id);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: start_time

  current_alignment +=
    builtin_interfaces::msg::typesupport_fastrtps_cpp::get_serialized_size(
    ros_message.start_time, current_alignment);
  // Member: order
  {
    size_t item_size = sizeof(ros_message.order);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: coef_x
  {
    size_t array_size = ros_message.coef_x.size();

    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    size_t item_size = sizeof(ros_message.coef_x[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: coef_y
  {
    size_t array_size = ros_message.coef_y.size();

    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    size_t item_size = sizeof(ros_message.coef_y[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: coef_z
  {
    size_t array_size = ros_message.coef_z.size();

    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    size_t item_size = sizeof(ros_message.coef_z[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: duration
  {
    size_t array_size = ros_message.duration.size();

    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    size_t item_size = sizeof(ros_message.duration[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_ommpc_ros2
max_serialized_size_PolyTraj(
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


  // Member: drone_id
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint16_t);
    current_alignment += array_size * sizeof(uint16_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint16_t));
  }

  // Member: traj_id
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: start_time
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

  // Member: order
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: coef_x
  {
    size_t array_size = 0;
    full_bounded = false;
    is_plain = false;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: coef_y
  {
    size_t array_size = 0;
    full_bounded = false;
    is_plain = false;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: coef_z
  {
    size_t array_size = 0;
    full_bounded = false;
    is_plain = false;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: duration
  {
    size_t array_size = 0;
    full_bounded = false;
    is_plain = false;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = ommpc_ros2::msg::PolyTraj;
    is_plain =
      (
      offsetof(DataType, duration) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static bool _PolyTraj__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  auto typed_message =
    static_cast<const ommpc_ros2::msg::PolyTraj *>(
    untyped_ros_message);
  return cdr_serialize(*typed_message, cdr);
}

static bool _PolyTraj__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  auto typed_message =
    static_cast<ommpc_ros2::msg::PolyTraj *>(
    untyped_ros_message);
  return cdr_deserialize(cdr, *typed_message);
}

static uint32_t _PolyTraj__get_serialized_size(
  const void * untyped_ros_message)
{
  auto typed_message =
    static_cast<const ommpc_ros2::msg::PolyTraj *>(
    untyped_ros_message);
  return static_cast<uint32_t>(get_serialized_size(*typed_message, 0));
}

static size_t _PolyTraj__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_PolyTraj(full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}

static message_type_support_callbacks_t _PolyTraj__callbacks = {
  "ommpc_ros2::msg",
  "PolyTraj",
  _PolyTraj__cdr_serialize,
  _PolyTraj__cdr_deserialize,
  _PolyTraj__get_serialized_size,
  _PolyTraj__max_serialized_size
};

static rosidl_message_type_support_t _PolyTraj__handle = {
  rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
  &_PolyTraj__callbacks,
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
get_message_type_support_handle<ommpc_ros2::msg::PolyTraj>()
{
  return &ommpc_ros2::msg::typesupport_fastrtps_cpp::_PolyTraj__handle;
}

}  // namespace rosidl_typesupport_fastrtps_cpp

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, ommpc_ros2, msg, PolyTraj)() {
  return &ommpc_ros2::msg::typesupport_fastrtps_cpp::_PolyTraj__handle;
}

#ifdef __cplusplus
}
#endif
