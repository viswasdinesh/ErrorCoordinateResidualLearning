// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from ommpc_ros2:msg/PolyTraj.idl
// generated code does not contain a copyright notice
#include "ommpc_ros2/msg/detail/poly_traj__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "ommpc_ros2/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "ommpc_ros2/msg/detail/poly_traj__struct.h"
#include "ommpc_ros2/msg/detail/poly_traj__functions.h"
#include "fastcdr/Cdr.h"

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

// includes and forward declarations of message dependencies and their conversion functions

#if defined(__cplusplus)
extern "C"
{
#endif

#include "builtin_interfaces/msg/detail/time__functions.h"  // start_time
#include "rosidl_runtime_c/primitives_sequence.h"  // coef_x, coef_y, coef_z, duration
#include "rosidl_runtime_c/primitives_sequence_functions.h"  // coef_x, coef_y, coef_z, duration

// forward declare type support functions
ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_ommpc_ros2
size_t get_serialized_size_builtin_interfaces__msg__Time(
  const void * untyped_ros_message,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_ommpc_ros2
size_t max_serialized_size_builtin_interfaces__msg__Time(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_ommpc_ros2
const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, builtin_interfaces, msg, Time)();


using _PolyTraj__ros_msg_type = ommpc_ros2__msg__PolyTraj;

static bool _PolyTraj__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _PolyTraj__ros_msg_type * ros_message = static_cast<const _PolyTraj__ros_msg_type *>(untyped_ros_message);
  // Field name: drone_id
  {
    cdr << ros_message->drone_id;
  }

  // Field name: traj_id
  {
    cdr << ros_message->traj_id;
  }

  // Field name: start_time
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, builtin_interfaces, msg, Time
      )()->data);
    if (!callbacks->cdr_serialize(
        &ros_message->start_time, cdr))
    {
      return false;
    }
  }

  // Field name: order
  {
    cdr << ros_message->order;
  }

  // Field name: coef_x
  {
    size_t size = ros_message->coef_x.size;
    auto array_ptr = ros_message->coef_x.data;
    cdr << static_cast<uint32_t>(size);
    cdr.serializeArray(array_ptr, size);
  }

  // Field name: coef_y
  {
    size_t size = ros_message->coef_y.size;
    auto array_ptr = ros_message->coef_y.data;
    cdr << static_cast<uint32_t>(size);
    cdr.serializeArray(array_ptr, size);
  }

  // Field name: coef_z
  {
    size_t size = ros_message->coef_z.size;
    auto array_ptr = ros_message->coef_z.data;
    cdr << static_cast<uint32_t>(size);
    cdr.serializeArray(array_ptr, size);
  }

  // Field name: duration
  {
    size_t size = ros_message->duration.size;
    auto array_ptr = ros_message->duration.data;
    cdr << static_cast<uint32_t>(size);
    cdr.serializeArray(array_ptr, size);
  }

  return true;
}

static bool _PolyTraj__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _PolyTraj__ros_msg_type * ros_message = static_cast<_PolyTraj__ros_msg_type *>(untyped_ros_message);
  // Field name: drone_id
  {
    cdr >> ros_message->drone_id;
  }

  // Field name: traj_id
  {
    cdr >> ros_message->traj_id;
  }

  // Field name: start_time
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, builtin_interfaces, msg, Time
      )()->data);
    if (!callbacks->cdr_deserialize(
        cdr, &ros_message->start_time))
    {
      return false;
    }
  }

  // Field name: order
  {
    cdr >> ros_message->order;
  }

  // Field name: coef_x
  {
    uint32_t cdrSize;
    cdr >> cdrSize;
    size_t size = static_cast<size_t>(cdrSize);

    // Check there are at least 'size' remaining bytes in the CDR stream before resizing
    auto old_state = cdr.getState();
    bool correct_size = cdr.jump(size);
    cdr.setState(old_state);
    if (!correct_size) {
      fprintf(stderr, "sequence size exceeds remaining buffer\n");
      return false;
    }

    if (ros_message->coef_x.data) {
      rosidl_runtime_c__float__Sequence__fini(&ros_message->coef_x);
    }
    if (!rosidl_runtime_c__float__Sequence__init(&ros_message->coef_x, size)) {
      fprintf(stderr, "failed to create array for field 'coef_x'");
      return false;
    }
    auto array_ptr = ros_message->coef_x.data;
    cdr.deserializeArray(array_ptr, size);
  }

  // Field name: coef_y
  {
    uint32_t cdrSize;
    cdr >> cdrSize;
    size_t size = static_cast<size_t>(cdrSize);

    // Check there are at least 'size' remaining bytes in the CDR stream before resizing
    auto old_state = cdr.getState();
    bool correct_size = cdr.jump(size);
    cdr.setState(old_state);
    if (!correct_size) {
      fprintf(stderr, "sequence size exceeds remaining buffer\n");
      return false;
    }

    if (ros_message->coef_y.data) {
      rosidl_runtime_c__float__Sequence__fini(&ros_message->coef_y);
    }
    if (!rosidl_runtime_c__float__Sequence__init(&ros_message->coef_y, size)) {
      fprintf(stderr, "failed to create array for field 'coef_y'");
      return false;
    }
    auto array_ptr = ros_message->coef_y.data;
    cdr.deserializeArray(array_ptr, size);
  }

  // Field name: coef_z
  {
    uint32_t cdrSize;
    cdr >> cdrSize;
    size_t size = static_cast<size_t>(cdrSize);

    // Check there are at least 'size' remaining bytes in the CDR stream before resizing
    auto old_state = cdr.getState();
    bool correct_size = cdr.jump(size);
    cdr.setState(old_state);
    if (!correct_size) {
      fprintf(stderr, "sequence size exceeds remaining buffer\n");
      return false;
    }

    if (ros_message->coef_z.data) {
      rosidl_runtime_c__float__Sequence__fini(&ros_message->coef_z);
    }
    if (!rosidl_runtime_c__float__Sequence__init(&ros_message->coef_z, size)) {
      fprintf(stderr, "failed to create array for field 'coef_z'");
      return false;
    }
    auto array_ptr = ros_message->coef_z.data;
    cdr.deserializeArray(array_ptr, size);
  }

  // Field name: duration
  {
    uint32_t cdrSize;
    cdr >> cdrSize;
    size_t size = static_cast<size_t>(cdrSize);

    // Check there are at least 'size' remaining bytes in the CDR stream before resizing
    auto old_state = cdr.getState();
    bool correct_size = cdr.jump(size);
    cdr.setState(old_state);
    if (!correct_size) {
      fprintf(stderr, "sequence size exceeds remaining buffer\n");
      return false;
    }

    if (ros_message->duration.data) {
      rosidl_runtime_c__float__Sequence__fini(&ros_message->duration);
    }
    if (!rosidl_runtime_c__float__Sequence__init(&ros_message->duration, size)) {
      fprintf(stderr, "failed to create array for field 'duration'");
      return false;
    }
    auto array_ptr = ros_message->duration.data;
    cdr.deserializeArray(array_ptr, size);
  }

  return true;
}  // NOLINT(readability/fn_size)

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_ommpc_ros2
size_t get_serialized_size_ommpc_ros2__msg__PolyTraj(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _PolyTraj__ros_msg_type * ros_message = static_cast<const _PolyTraj__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // field.name drone_id
  {
    size_t item_size = sizeof(ros_message->drone_id);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name traj_id
  {
    size_t item_size = sizeof(ros_message->traj_id);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name start_time

  current_alignment += get_serialized_size_builtin_interfaces__msg__Time(
    &(ros_message->start_time), current_alignment);
  // field.name order
  {
    size_t item_size = sizeof(ros_message->order);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name coef_x
  {
    size_t array_size = ros_message->coef_x.size;
    auto array_ptr = ros_message->coef_x.data;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name coef_y
  {
    size_t array_size = ros_message->coef_y.size;
    auto array_ptr = ros_message->coef_y.data;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name coef_z
  {
    size_t array_size = ros_message->coef_z.size;
    auto array_ptr = ros_message->coef_z.data;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name duration
  {
    size_t array_size = ros_message->duration.size;
    auto array_ptr = ros_message->duration.data;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

static uint32_t _PolyTraj__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_ommpc_ros2__msg__PolyTraj(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_ommpc_ros2
size_t max_serialized_size_ommpc_ros2__msg__PolyTraj(
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

  // member: drone_id
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint16_t);
    current_alignment += array_size * sizeof(uint16_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint16_t));
  }
  // member: traj_id
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: start_time
  {
    size_t array_size = 1;


    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size;
      inner_size =
        max_serialized_size_builtin_interfaces__msg__Time(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }
  // member: order
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: coef_x
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
  // member: coef_y
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
  // member: coef_z
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
  // member: duration
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
    using DataType = ommpc_ros2__msg__PolyTraj;
    is_plain =
      (
      offsetof(DataType, duration) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static size_t _PolyTraj__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_ommpc_ros2__msg__PolyTraj(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_PolyTraj = {
  "ommpc_ros2::msg",
  "PolyTraj",
  _PolyTraj__cdr_serialize,
  _PolyTraj__cdr_deserialize,
  _PolyTraj__get_serialized_size,
  _PolyTraj__max_serialized_size
};

static rosidl_message_type_support_t _PolyTraj__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_PolyTraj,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, ommpc_ros2, msg, PolyTraj)() {
  return &_PolyTraj__type_support;
}

#if defined(__cplusplus)
}
#endif
