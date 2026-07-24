// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from ommpc_ros2:msg/PolyTraj.idl
// generated code does not contain a copyright notice

#ifndef OMMPC_ROS2__MSG__DETAIL__POLY_TRAJ__STRUCT_H_
#define OMMPC_ROS2__MSG__DETAIL__POLY_TRAJ__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'start_time'
#include "builtin_interfaces/msg/detail/time__struct.h"
// Member 'coef_x'
// Member 'coef_y'
// Member 'coef_z'
// Member 'duration'
#include "rosidl_runtime_c/primitives_sequence.h"

/// Struct defined in msg/PolyTraj in the package ommpc_ros2.
typedef struct ommpc_ros2__msg__PolyTraj
{
  int16_t drone_id;
  int32_t traj_id;
  builtin_interfaces__msg__Time start_time;
  uint8_t order;
  rosidl_runtime_c__float__Sequence coef_x;
  rosidl_runtime_c__float__Sequence coef_y;
  rosidl_runtime_c__float__Sequence coef_z;
  rosidl_runtime_c__float__Sequence duration;
} ommpc_ros2__msg__PolyTraj;

// Struct for a sequence of ommpc_ros2__msg__PolyTraj.
typedef struct ommpc_ros2__msg__PolyTraj__Sequence
{
  ommpc_ros2__msg__PolyTraj * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} ommpc_ros2__msg__PolyTraj__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // OMMPC_ROS2__MSG__DETAIL__POLY_TRAJ__STRUCT_H_
