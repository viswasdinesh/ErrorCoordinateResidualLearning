// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from ommpc_ros2:msg/PolyTraj.idl
// generated code does not contain a copyright notice

#ifndef OMMPC_ROS2__MSG__DETAIL__POLY_TRAJ__FUNCTIONS_H_
#define OMMPC_ROS2__MSG__DETAIL__POLY_TRAJ__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "ommpc_ros2/msg/rosidl_generator_c__visibility_control.h"

#include "ommpc_ros2/msg/detail/poly_traj__struct.h"

/// Initialize msg/PolyTraj message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * ommpc_ros2__msg__PolyTraj
 * )) before or use
 * ommpc_ros2__msg__PolyTraj__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_ommpc_ros2
bool
ommpc_ros2__msg__PolyTraj__init(ommpc_ros2__msg__PolyTraj * msg);

/// Finalize msg/PolyTraj message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_ommpc_ros2
void
ommpc_ros2__msg__PolyTraj__fini(ommpc_ros2__msg__PolyTraj * msg);

/// Create msg/PolyTraj message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * ommpc_ros2__msg__PolyTraj__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_ommpc_ros2
ommpc_ros2__msg__PolyTraj *
ommpc_ros2__msg__PolyTraj__create();

/// Destroy msg/PolyTraj message.
/**
 * It calls
 * ommpc_ros2__msg__PolyTraj__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_ommpc_ros2
void
ommpc_ros2__msg__PolyTraj__destroy(ommpc_ros2__msg__PolyTraj * msg);

/// Check for msg/PolyTraj message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_ommpc_ros2
bool
ommpc_ros2__msg__PolyTraj__are_equal(const ommpc_ros2__msg__PolyTraj * lhs, const ommpc_ros2__msg__PolyTraj * rhs);

/// Copy a msg/PolyTraj message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_ommpc_ros2
bool
ommpc_ros2__msg__PolyTraj__copy(
  const ommpc_ros2__msg__PolyTraj * input,
  ommpc_ros2__msg__PolyTraj * output);

/// Initialize array of msg/PolyTraj messages.
/**
 * It allocates the memory for the number of elements and calls
 * ommpc_ros2__msg__PolyTraj__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_ommpc_ros2
bool
ommpc_ros2__msg__PolyTraj__Sequence__init(ommpc_ros2__msg__PolyTraj__Sequence * array, size_t size);

/// Finalize array of msg/PolyTraj messages.
/**
 * It calls
 * ommpc_ros2__msg__PolyTraj__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_ommpc_ros2
void
ommpc_ros2__msg__PolyTraj__Sequence__fini(ommpc_ros2__msg__PolyTraj__Sequence * array);

/// Create array of msg/PolyTraj messages.
/**
 * It allocates the memory for the array and calls
 * ommpc_ros2__msg__PolyTraj__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_ommpc_ros2
ommpc_ros2__msg__PolyTraj__Sequence *
ommpc_ros2__msg__PolyTraj__Sequence__create(size_t size);

/// Destroy array of msg/PolyTraj messages.
/**
 * It calls
 * ommpc_ros2__msg__PolyTraj__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_ommpc_ros2
void
ommpc_ros2__msg__PolyTraj__Sequence__destroy(ommpc_ros2__msg__PolyTraj__Sequence * array);

/// Check for msg/PolyTraj message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_ommpc_ros2
bool
ommpc_ros2__msg__PolyTraj__Sequence__are_equal(const ommpc_ros2__msg__PolyTraj__Sequence * lhs, const ommpc_ros2__msg__PolyTraj__Sequence * rhs);

/// Copy an array of msg/PolyTraj messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_ommpc_ros2
bool
ommpc_ros2__msg__PolyTraj__Sequence__copy(
  const ommpc_ros2__msg__PolyTraj__Sequence * input,
  ommpc_ros2__msg__PolyTraj__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // OMMPC_ROS2__MSG__DETAIL__POLY_TRAJ__FUNCTIONS_H_
