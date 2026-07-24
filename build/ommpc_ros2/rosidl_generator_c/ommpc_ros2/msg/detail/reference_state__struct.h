// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from ommpc_ros2:msg/ReferenceState.idl
// generated code does not contain a copyright notice

#ifndef OMMPC_ROS2__MSG__DETAIL__REFERENCE_STATE__STRUCT_H_
#define OMMPC_ROS2__MSG__DETAIL__REFERENCE_STATE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'timestamp'
#include "builtin_interfaces/msg/detail/time__struct.h"
// Member 'position'
#include "geometry_msgs/msg/detail/point__struct.h"
// Member 'velocity'
// Member 'body_rates'
#include "geometry_msgs/msg/detail/vector3__struct.h"
// Member 'quaternion'
#include "geometry_msgs/msg/detail/quaternion__struct.h"

/// Struct defined in msg/ReferenceState in the package ommpc_ros2.
typedef struct ommpc_ros2__msg__ReferenceState
{
  builtin_interfaces__msg__Time timestamp;
  geometry_msgs__msg__Point position;
  geometry_msgs__msg__Vector3 velocity;
  geometry_msgs__msg__Quaternion quaternion;
  geometry_msgs__msg__Vector3 body_rates;
  double thrust;
} ommpc_ros2__msg__ReferenceState;

// Struct for a sequence of ommpc_ros2__msg__ReferenceState.
typedef struct ommpc_ros2__msg__ReferenceState__Sequence
{
  ommpc_ros2__msg__ReferenceState * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} ommpc_ros2__msg__ReferenceState__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // OMMPC_ROS2__MSG__DETAIL__REFERENCE_STATE__STRUCT_H_
