// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from ommpc_ros2:msg/ReferenceState.idl
// generated code does not contain a copyright notice
#include "ommpc_ros2/msg/detail/reference_state__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `timestamp`
#include "builtin_interfaces/msg/detail/time__functions.h"
// Member `position`
#include "geometry_msgs/msg/detail/point__functions.h"
// Member `velocity`
// Member `body_rates`
#include "geometry_msgs/msg/detail/vector3__functions.h"
// Member `quaternion`
#include "geometry_msgs/msg/detail/quaternion__functions.h"

bool
ommpc_ros2__msg__ReferenceState__init(ommpc_ros2__msg__ReferenceState * msg)
{
  if (!msg) {
    return false;
  }
  // timestamp
  if (!builtin_interfaces__msg__Time__init(&msg->timestamp)) {
    ommpc_ros2__msg__ReferenceState__fini(msg);
    return false;
  }
  // position
  if (!geometry_msgs__msg__Point__init(&msg->position)) {
    ommpc_ros2__msg__ReferenceState__fini(msg);
    return false;
  }
  // velocity
  if (!geometry_msgs__msg__Vector3__init(&msg->velocity)) {
    ommpc_ros2__msg__ReferenceState__fini(msg);
    return false;
  }
  // quaternion
  if (!geometry_msgs__msg__Quaternion__init(&msg->quaternion)) {
    ommpc_ros2__msg__ReferenceState__fini(msg);
    return false;
  }
  // body_rates
  if (!geometry_msgs__msg__Vector3__init(&msg->body_rates)) {
    ommpc_ros2__msg__ReferenceState__fini(msg);
    return false;
  }
  // thrust
  return true;
}

void
ommpc_ros2__msg__ReferenceState__fini(ommpc_ros2__msg__ReferenceState * msg)
{
  if (!msg) {
    return;
  }
  // timestamp
  builtin_interfaces__msg__Time__fini(&msg->timestamp);
  // position
  geometry_msgs__msg__Point__fini(&msg->position);
  // velocity
  geometry_msgs__msg__Vector3__fini(&msg->velocity);
  // quaternion
  geometry_msgs__msg__Quaternion__fini(&msg->quaternion);
  // body_rates
  geometry_msgs__msg__Vector3__fini(&msg->body_rates);
  // thrust
}

bool
ommpc_ros2__msg__ReferenceState__are_equal(const ommpc_ros2__msg__ReferenceState * lhs, const ommpc_ros2__msg__ReferenceState * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // timestamp
  if (!builtin_interfaces__msg__Time__are_equal(
      &(lhs->timestamp), &(rhs->timestamp)))
  {
    return false;
  }
  // position
  if (!geometry_msgs__msg__Point__are_equal(
      &(lhs->position), &(rhs->position)))
  {
    return false;
  }
  // velocity
  if (!geometry_msgs__msg__Vector3__are_equal(
      &(lhs->velocity), &(rhs->velocity)))
  {
    return false;
  }
  // quaternion
  if (!geometry_msgs__msg__Quaternion__are_equal(
      &(lhs->quaternion), &(rhs->quaternion)))
  {
    return false;
  }
  // body_rates
  if (!geometry_msgs__msg__Vector3__are_equal(
      &(lhs->body_rates), &(rhs->body_rates)))
  {
    return false;
  }
  // thrust
  if (lhs->thrust != rhs->thrust) {
    return false;
  }
  return true;
}

bool
ommpc_ros2__msg__ReferenceState__copy(
  const ommpc_ros2__msg__ReferenceState * input,
  ommpc_ros2__msg__ReferenceState * output)
{
  if (!input || !output) {
    return false;
  }
  // timestamp
  if (!builtin_interfaces__msg__Time__copy(
      &(input->timestamp), &(output->timestamp)))
  {
    return false;
  }
  // position
  if (!geometry_msgs__msg__Point__copy(
      &(input->position), &(output->position)))
  {
    return false;
  }
  // velocity
  if (!geometry_msgs__msg__Vector3__copy(
      &(input->velocity), &(output->velocity)))
  {
    return false;
  }
  // quaternion
  if (!geometry_msgs__msg__Quaternion__copy(
      &(input->quaternion), &(output->quaternion)))
  {
    return false;
  }
  // body_rates
  if (!geometry_msgs__msg__Vector3__copy(
      &(input->body_rates), &(output->body_rates)))
  {
    return false;
  }
  // thrust
  output->thrust = input->thrust;
  return true;
}

ommpc_ros2__msg__ReferenceState *
ommpc_ros2__msg__ReferenceState__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ommpc_ros2__msg__ReferenceState * msg = (ommpc_ros2__msg__ReferenceState *)allocator.allocate(sizeof(ommpc_ros2__msg__ReferenceState), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(ommpc_ros2__msg__ReferenceState));
  bool success = ommpc_ros2__msg__ReferenceState__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
ommpc_ros2__msg__ReferenceState__destroy(ommpc_ros2__msg__ReferenceState * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    ommpc_ros2__msg__ReferenceState__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
ommpc_ros2__msg__ReferenceState__Sequence__init(ommpc_ros2__msg__ReferenceState__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ommpc_ros2__msg__ReferenceState * data = NULL;

  if (size) {
    data = (ommpc_ros2__msg__ReferenceState *)allocator.zero_allocate(size, sizeof(ommpc_ros2__msg__ReferenceState), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = ommpc_ros2__msg__ReferenceState__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        ommpc_ros2__msg__ReferenceState__fini(&data[i - 1]);
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
ommpc_ros2__msg__ReferenceState__Sequence__fini(ommpc_ros2__msg__ReferenceState__Sequence * array)
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
      ommpc_ros2__msg__ReferenceState__fini(&array->data[i]);
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

ommpc_ros2__msg__ReferenceState__Sequence *
ommpc_ros2__msg__ReferenceState__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ommpc_ros2__msg__ReferenceState__Sequence * array = (ommpc_ros2__msg__ReferenceState__Sequence *)allocator.allocate(sizeof(ommpc_ros2__msg__ReferenceState__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = ommpc_ros2__msg__ReferenceState__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
ommpc_ros2__msg__ReferenceState__Sequence__destroy(ommpc_ros2__msg__ReferenceState__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    ommpc_ros2__msg__ReferenceState__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
ommpc_ros2__msg__ReferenceState__Sequence__are_equal(const ommpc_ros2__msg__ReferenceState__Sequence * lhs, const ommpc_ros2__msg__ReferenceState__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!ommpc_ros2__msg__ReferenceState__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
ommpc_ros2__msg__ReferenceState__Sequence__copy(
  const ommpc_ros2__msg__ReferenceState__Sequence * input,
  ommpc_ros2__msg__ReferenceState__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(ommpc_ros2__msg__ReferenceState);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    ommpc_ros2__msg__ReferenceState * data =
      (ommpc_ros2__msg__ReferenceState *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!ommpc_ros2__msg__ReferenceState__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          ommpc_ros2__msg__ReferenceState__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!ommpc_ros2__msg__ReferenceState__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
