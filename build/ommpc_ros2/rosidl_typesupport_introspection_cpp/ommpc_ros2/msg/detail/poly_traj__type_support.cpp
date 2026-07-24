// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from ommpc_ros2:msg/PolyTraj.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "ommpc_ros2/msg/detail/poly_traj__struct.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace ommpc_ros2
{

namespace msg
{

namespace rosidl_typesupport_introspection_cpp
{

void PolyTraj_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) ommpc_ros2::msg::PolyTraj(_init);
}

void PolyTraj_fini_function(void * message_memory)
{
  auto typed_message = static_cast<ommpc_ros2::msg::PolyTraj *>(message_memory);
  typed_message->~PolyTraj();
}

size_t size_function__PolyTraj__coef_x(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<float> *>(untyped_member);
  return member->size();
}

const void * get_const_function__PolyTraj__coef_x(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<float> *>(untyped_member);
  return &member[index];
}

void * get_function__PolyTraj__coef_x(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<float> *>(untyped_member);
  return &member[index];
}

void fetch_function__PolyTraj__coef_x(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const float *>(
    get_const_function__PolyTraj__coef_x(untyped_member, index));
  auto & value = *reinterpret_cast<float *>(untyped_value);
  value = item;
}

void assign_function__PolyTraj__coef_x(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<float *>(
    get_function__PolyTraj__coef_x(untyped_member, index));
  const auto & value = *reinterpret_cast<const float *>(untyped_value);
  item = value;
}

void resize_function__PolyTraj__coef_x(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<float> *>(untyped_member);
  member->resize(size);
}

size_t size_function__PolyTraj__coef_y(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<float> *>(untyped_member);
  return member->size();
}

const void * get_const_function__PolyTraj__coef_y(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<float> *>(untyped_member);
  return &member[index];
}

void * get_function__PolyTraj__coef_y(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<float> *>(untyped_member);
  return &member[index];
}

void fetch_function__PolyTraj__coef_y(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const float *>(
    get_const_function__PolyTraj__coef_y(untyped_member, index));
  auto & value = *reinterpret_cast<float *>(untyped_value);
  value = item;
}

void assign_function__PolyTraj__coef_y(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<float *>(
    get_function__PolyTraj__coef_y(untyped_member, index));
  const auto & value = *reinterpret_cast<const float *>(untyped_value);
  item = value;
}

void resize_function__PolyTraj__coef_y(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<float> *>(untyped_member);
  member->resize(size);
}

size_t size_function__PolyTraj__coef_z(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<float> *>(untyped_member);
  return member->size();
}

const void * get_const_function__PolyTraj__coef_z(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<float> *>(untyped_member);
  return &member[index];
}

void * get_function__PolyTraj__coef_z(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<float> *>(untyped_member);
  return &member[index];
}

void fetch_function__PolyTraj__coef_z(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const float *>(
    get_const_function__PolyTraj__coef_z(untyped_member, index));
  auto & value = *reinterpret_cast<float *>(untyped_value);
  value = item;
}

void assign_function__PolyTraj__coef_z(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<float *>(
    get_function__PolyTraj__coef_z(untyped_member, index));
  const auto & value = *reinterpret_cast<const float *>(untyped_value);
  item = value;
}

void resize_function__PolyTraj__coef_z(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<float> *>(untyped_member);
  member->resize(size);
}

size_t size_function__PolyTraj__duration(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<float> *>(untyped_member);
  return member->size();
}

const void * get_const_function__PolyTraj__duration(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<float> *>(untyped_member);
  return &member[index];
}

void * get_function__PolyTraj__duration(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<float> *>(untyped_member);
  return &member[index];
}

void fetch_function__PolyTraj__duration(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const float *>(
    get_const_function__PolyTraj__duration(untyped_member, index));
  auto & value = *reinterpret_cast<float *>(untyped_value);
  value = item;
}

void assign_function__PolyTraj__duration(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<float *>(
    get_function__PolyTraj__duration(untyped_member, index));
  const auto & value = *reinterpret_cast<const float *>(untyped_value);
  item = value;
}

void resize_function__PolyTraj__duration(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<float> *>(untyped_member);
  member->resize(size);
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember PolyTraj_message_member_array[8] = {
  {
    "drone_id",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_INT16,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ommpc_ros2::msg::PolyTraj, drone_id),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "traj_id",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ommpc_ros2::msg::PolyTraj, traj_id),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "start_time",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<builtin_interfaces::msg::Time>(),  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ommpc_ros2::msg::PolyTraj, start_time),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "order",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ommpc_ros2::msg::PolyTraj, order),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "coef_x",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ommpc_ros2::msg::PolyTraj, coef_x),  // bytes offset in struct
    nullptr,  // default value
    size_function__PolyTraj__coef_x,  // size() function pointer
    get_const_function__PolyTraj__coef_x,  // get_const(index) function pointer
    get_function__PolyTraj__coef_x,  // get(index) function pointer
    fetch_function__PolyTraj__coef_x,  // fetch(index, &value) function pointer
    assign_function__PolyTraj__coef_x,  // assign(index, value) function pointer
    resize_function__PolyTraj__coef_x  // resize(index) function pointer
  },
  {
    "coef_y",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ommpc_ros2::msg::PolyTraj, coef_y),  // bytes offset in struct
    nullptr,  // default value
    size_function__PolyTraj__coef_y,  // size() function pointer
    get_const_function__PolyTraj__coef_y,  // get_const(index) function pointer
    get_function__PolyTraj__coef_y,  // get(index) function pointer
    fetch_function__PolyTraj__coef_y,  // fetch(index, &value) function pointer
    assign_function__PolyTraj__coef_y,  // assign(index, value) function pointer
    resize_function__PolyTraj__coef_y  // resize(index) function pointer
  },
  {
    "coef_z",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ommpc_ros2::msg::PolyTraj, coef_z),  // bytes offset in struct
    nullptr,  // default value
    size_function__PolyTraj__coef_z,  // size() function pointer
    get_const_function__PolyTraj__coef_z,  // get_const(index) function pointer
    get_function__PolyTraj__coef_z,  // get(index) function pointer
    fetch_function__PolyTraj__coef_z,  // fetch(index, &value) function pointer
    assign_function__PolyTraj__coef_z,  // assign(index, value) function pointer
    resize_function__PolyTraj__coef_z  // resize(index) function pointer
  },
  {
    "duration",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ommpc_ros2::msg::PolyTraj, duration),  // bytes offset in struct
    nullptr,  // default value
    size_function__PolyTraj__duration,  // size() function pointer
    get_const_function__PolyTraj__duration,  // get_const(index) function pointer
    get_function__PolyTraj__duration,  // get(index) function pointer
    fetch_function__PolyTraj__duration,  // fetch(index, &value) function pointer
    assign_function__PolyTraj__duration,  // assign(index, value) function pointer
    resize_function__PolyTraj__duration  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers PolyTraj_message_members = {
  "ommpc_ros2::msg",  // message namespace
  "PolyTraj",  // message name
  8,  // number of fields
  sizeof(ommpc_ros2::msg::PolyTraj),
  PolyTraj_message_member_array,  // message members
  PolyTraj_init_function,  // function to initialize message memory (memory has to be allocated)
  PolyTraj_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t PolyTraj_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &PolyTraj_message_members,
  get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace msg

}  // namespace ommpc_ros2


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<ommpc_ros2::msg::PolyTraj>()
{
  return &::ommpc_ros2::msg::rosidl_typesupport_introspection_cpp::PolyTraj_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, ommpc_ros2, msg, PolyTraj)() {
  return &::ommpc_ros2::msg::rosidl_typesupport_introspection_cpp::PolyTraj_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
