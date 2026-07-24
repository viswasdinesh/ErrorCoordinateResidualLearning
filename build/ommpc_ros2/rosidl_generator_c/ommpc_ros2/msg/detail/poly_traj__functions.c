// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from ommpc_ros2:msg/PolyTraj.idl
// generated code does not contain a copyright notice
#include "ommpc_ros2/msg/detail/poly_traj__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `start_time`
#include "builtin_interfaces/msg/detail/time__functions.h"
// Member `coef_x`
// Member `coef_y`
// Member `coef_z`
// Member `duration`
#include "rosidl_runtime_c/primitives_sequence_functions.h"

bool
ommpc_ros2__msg__PolyTraj__init(ommpc_ros2__msg__PolyTraj * msg)
{
  if (!msg) {
    return false;
  }
  // drone_id
  // traj_id
  // start_time
  if (!builtin_interfaces__msg__Time__init(&msg->start_time)) {
    ommpc_ros2__msg__PolyTraj__fini(msg);
    return false;
  }
  // order
  // coef_x
  if (!rosidl_runtime_c__float__Sequence__init(&msg->coef_x, 0)) {
    ommpc_ros2__msg__PolyTraj__fini(msg);
    return false;
  }
  // coef_y
  if (!rosidl_runtime_c__float__Sequence__init(&msg->coef_y, 0)) {
    ommpc_ros2__msg__PolyTraj__fini(msg);
    return false;
  }
  // coef_z
  if (!rosidl_runtime_c__float__Sequence__init(&msg->coef_z, 0)) {
    ommpc_ros2__msg__PolyTraj__fini(msg);
    return false;
  }
  // duration
  if (!rosidl_runtime_c__float__Sequence__init(&msg->duration, 0)) {
    ommpc_ros2__msg__PolyTraj__fini(msg);
    return false;
  }
  return true;
}

void
ommpc_ros2__msg__PolyTraj__fini(ommpc_ros2__msg__PolyTraj * msg)
{
  if (!msg) {
    return;
  }
  // drone_id
  // traj_id
  // start_time
  builtin_interfaces__msg__Time__fini(&msg->start_time);
  // order
  // coef_x
  rosidl_runtime_c__float__Sequence__fini(&msg->coef_x);
  // coef_y
  rosidl_runtime_c__float__Sequence__fini(&msg->coef_y);
  // coef_z
  rosidl_runtime_c__float__Sequence__fini(&msg->coef_z);
  // duration
  rosidl_runtime_c__float__Sequence__fini(&msg->duration);
}

bool
ommpc_ros2__msg__PolyTraj__are_equal(const ommpc_ros2__msg__PolyTraj * lhs, const ommpc_ros2__msg__PolyTraj * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // drone_id
  if (lhs->drone_id != rhs->drone_id) {
    return false;
  }
  // traj_id
  if (lhs->traj_id != rhs->traj_id) {
    return false;
  }
  // start_time
  if (!builtin_interfaces__msg__Time__are_equal(
      &(lhs->start_time), &(rhs->start_time)))
  {
    return false;
  }
  // order
  if (lhs->order != rhs->order) {
    return false;
  }
  // coef_x
  if (!rosidl_runtime_c__float__Sequence__are_equal(
      &(lhs->coef_x), &(rhs->coef_x)))
  {
    return false;
  }
  // coef_y
  if (!rosidl_runtime_c__float__Sequence__are_equal(
      &(lhs->coef_y), &(rhs->coef_y)))
  {
    return false;
  }
  // coef_z
  if (!rosidl_runtime_c__float__Sequence__are_equal(
      &(lhs->coef_z), &(rhs->coef_z)))
  {
    return false;
  }
  // duration
  if (!rosidl_runtime_c__float__Sequence__are_equal(
      &(lhs->duration), &(rhs->duration)))
  {
    return false;
  }
  return true;
}

bool
ommpc_ros2__msg__PolyTraj__copy(
  const ommpc_ros2__msg__PolyTraj * input,
  ommpc_ros2__msg__PolyTraj * output)
{
  if (!input || !output) {
    return false;
  }
  // drone_id
  output->drone_id = input->drone_id;
  // traj_id
  output->traj_id = input->traj_id;
  // start_time
  if (!builtin_interfaces__msg__Time__copy(
      &(input->start_time), &(output->start_time)))
  {
    return false;
  }
  // order
  output->order = input->order;
  // coef_x
  if (!rosidl_runtime_c__float__Sequence__copy(
      &(input->coef_x), &(output->coef_x)))
  {
    return false;
  }
  // coef_y
  if (!rosidl_runtime_c__float__Sequence__copy(
      &(input->coef_y), &(output->coef_y)))
  {
    return false;
  }
  // coef_z
  if (!rosidl_runtime_c__float__Sequence__copy(
      &(input->coef_z), &(output->coef_z)))
  {
    return false;
  }
  // duration
  if (!rosidl_runtime_c__float__Sequence__copy(
      &(input->duration), &(output->duration)))
  {
    return false;
  }
  return true;
}

ommpc_ros2__msg__PolyTraj *
ommpc_ros2__msg__PolyTraj__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ommpc_ros2__msg__PolyTraj * msg = (ommpc_ros2__msg__PolyTraj *)allocator.allocate(sizeof(ommpc_ros2__msg__PolyTraj), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(ommpc_ros2__msg__PolyTraj));
  bool success = ommpc_ros2__msg__PolyTraj__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
ommpc_ros2__msg__PolyTraj__destroy(ommpc_ros2__msg__PolyTraj * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    ommpc_ros2__msg__PolyTraj__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
ommpc_ros2__msg__PolyTraj__Sequence__init(ommpc_ros2__msg__PolyTraj__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ommpc_ros2__msg__PolyTraj * data = NULL;

  if (size) {
    data = (ommpc_ros2__msg__PolyTraj *)allocator.zero_allocate(size, sizeof(ommpc_ros2__msg__PolyTraj), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = ommpc_ros2__msg__PolyTraj__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        ommpc_ros2__msg__PolyTraj__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
ommpc_ros2__msg__PolyTraj__Sequence__fini(ommpc_ros2__msg__PolyTraj__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      ommpc_ros2__msg__PolyTraj__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

ommpc_ros2__msg__PolyTraj__Sequence *
ommpc_ros2__msg__PolyTraj__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ommpc_ros2__msg__PolyTraj__Sequence * array = (ommpc_ros2__msg__PolyTraj__Sequence *)allocator.allocate(sizeof(ommpc_ros2__msg__PolyTraj__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = ommpc_ros2__msg__PolyTraj__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
ommpc_ros2__msg__PolyTraj__Sequence__destroy(ommpc_ros2__msg__PolyTraj__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    ommpc_ros2__msg__PolyTraj__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
ommpc_ros2__msg__PolyTraj__Sequence__are_equal(const ommpc_ros2__msg__PolyTraj__Sequence * lhs, const ommpc_ros2__msg__PolyTraj__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!ommpc_ros2__msg__PolyTraj__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
ommpc_ros2__msg__PolyTraj__Sequence__copy(
  const ommpc_ros2__msg__PolyTraj__Sequence * input,
  ommpc_ros2__msg__PolyTraj__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(ommpc_ros2__msg__PolyTraj);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    ommpc_ros2__msg__PolyTraj * data =
      (ommpc_ros2__msg__PolyTraj *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!ommpc_ros2__msg__PolyTraj__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          ommpc_ros2__msg__PolyTraj__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!ommpc_ros2__msg__PolyTraj__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
