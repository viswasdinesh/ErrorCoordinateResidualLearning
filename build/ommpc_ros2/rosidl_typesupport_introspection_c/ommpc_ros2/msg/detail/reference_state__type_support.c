// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from ommpc_ros2:msg/ReferenceState.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "ommpc_ros2/msg/detail/reference_state__rosidl_typesupport_introspection_c.h"
#include "ommpc_ros2/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "ommpc_ros2/msg/detail/reference_state__functions.h"
#include "ommpc_ros2/msg/detail/reference_state__struct.h"


// Include directives for member types
// Member `timestamp`
#include "builtin_interfaces/msg/time.h"
// Member `timestamp`
#include "builtin_interfaces/msg/detail/time__rosidl_typesupport_introspection_c.h"
// Member `position`
#include "geometry_msgs/msg/point.h"
// Member `position`
#include "geometry_msgs/msg/detail/point__rosidl_typesupport_introspection_c.h"
// Member `velocity`
// Member `body_rates`
#include "geometry_msgs/msg/vector3.h"
// Member `velocity`
// Member `body_rates`
#include "geometry_msgs/msg/detail/vector3__rosidl_typesupport_introspection_c.h"
// Member `quaternion`
#include "geometry_msgs/msg/quaternion.h"
// Member `quaternion`
#include "geometry_msgs/msg/detail/quaternion__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void ommpc_ros2__msg__ReferenceState__rosidl_typesupport_introspection_c__ReferenceState_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  ommpc_ros2__msg__ReferenceState__init(message_memory);
}

void ommpc_ros2__msg__ReferenceState__rosidl_typesupport_introspection_c__ReferenceState_fini_function(void * message_memory)
{
  ommpc_ros2__msg__ReferenceState__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember ommpc_ros2__msg__ReferenceState__rosidl_typesupport_introspection_c__ReferenceState_message_member_array[6] = {
  {
    "timestamp",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ommpc_ros2__msg__ReferenceState, timestamp),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "position",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ommpc_ros2__msg__ReferenceState, position),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "velocity",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ommpc_ros2__msg__ReferenceState, velocity),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "quaternion",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ommpc_ros2__msg__ReferenceState, quaternion),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "body_rates",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ommpc_ros2__msg__ReferenceState, body_rates),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "thrust",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ommpc_ros2__msg__ReferenceState, thrust),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers ommpc_ros2__msg__ReferenceState__rosidl_typesupport_introspection_c__ReferenceState_message_members = {
  "ommpc_ros2__msg",  // message namespace
  "ReferenceState",  // message name
  6,  // number of fields
  sizeof(ommpc_ros2__msg__ReferenceState),
  ommpc_ros2__msg__ReferenceState__rosidl_typesupport_introspection_c__ReferenceState_message_member_array,  // message members
  ommpc_ros2__msg__ReferenceState__rosidl_typesupport_introspection_c__ReferenceState_init_function,  // function to initialize message memory (memory has to be allocated)
  ommpc_ros2__msg__ReferenceState__rosidl_typesupport_introspection_c__ReferenceState_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t ommpc_ros2__msg__ReferenceState__rosidl_typesupport_introspection_c__ReferenceState_message_type_support_handle = {
  0,
  &ommpc_ros2__msg__ReferenceState__rosidl_typesupport_introspection_c__ReferenceState_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_ommpc_ros2
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, ommpc_ros2, msg, ReferenceState)() {
  ommpc_ros2__msg__ReferenceState__rosidl_typesupport_introspection_c__ReferenceState_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, builtin_interfaces, msg, Time)();
  ommpc_ros2__msg__ReferenceState__rosidl_typesupport_introspection_c__ReferenceState_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, geometry_msgs, msg, Point)();
  ommpc_ros2__msg__ReferenceState__rosidl_typesupport_introspection_c__ReferenceState_message_member_array[2].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, geometry_msgs, msg, Vector3)();
  ommpc_ros2__msg__ReferenceState__rosidl_typesupport_introspection_c__ReferenceState_message_member_array[3].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, geometry_msgs, msg, Quaternion)();
  ommpc_ros2__msg__ReferenceState__rosidl_typesupport_introspection_c__ReferenceState_message_member_array[4].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, geometry_msgs, msg, Vector3)();
  if (!ommpc_ros2__msg__ReferenceState__rosidl_typesupport_introspection_c__ReferenceState_message_type_support_handle.typesupport_identifier) {
    ommpc_ros2__msg__ReferenceState__rosidl_typesupport_introspection_c__ReferenceState_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &ommpc_ros2__msg__ReferenceState__rosidl_typesupport_introspection_c__ReferenceState_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
