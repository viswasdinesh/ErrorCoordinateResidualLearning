// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from ommpc_ros2:msg/PolyTraj.idl
// generated code does not contain a copyright notice

#ifndef OMMPC_ROS2__MSG__DETAIL__POLY_TRAJ__STRUCT_HPP_
#define OMMPC_ROS2__MSG__DETAIL__POLY_TRAJ__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'start_time'
#include "builtin_interfaces/msg/detail/time__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__ommpc_ros2__msg__PolyTraj __attribute__((deprecated))
#else
# define DEPRECATED__ommpc_ros2__msg__PolyTraj __declspec(deprecated)
#endif

namespace ommpc_ros2
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct PolyTraj_
{
  using Type = PolyTraj_<ContainerAllocator>;

  explicit PolyTraj_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : start_time(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->drone_id = 0;
      this->traj_id = 0l;
      this->order = 0;
    }
  }

  explicit PolyTraj_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : start_time(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->drone_id = 0;
      this->traj_id = 0l;
      this->order = 0;
    }
  }

  // field types and members
  using _drone_id_type =
    int16_t;
  _drone_id_type drone_id;
  using _traj_id_type =
    int32_t;
  _traj_id_type traj_id;
  using _start_time_type =
    builtin_interfaces::msg::Time_<ContainerAllocator>;
  _start_time_type start_time;
  using _order_type =
    uint8_t;
  _order_type order;
  using _coef_x_type =
    std::vector<float, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<float>>;
  _coef_x_type coef_x;
  using _coef_y_type =
    std::vector<float, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<float>>;
  _coef_y_type coef_y;
  using _coef_z_type =
    std::vector<float, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<float>>;
  _coef_z_type coef_z;
  using _duration_type =
    std::vector<float, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<float>>;
  _duration_type duration;

  // setters for named parameter idiom
  Type & set__drone_id(
    const int16_t & _arg)
  {
    this->drone_id = _arg;
    return *this;
  }
  Type & set__traj_id(
    const int32_t & _arg)
  {
    this->traj_id = _arg;
    return *this;
  }
  Type & set__start_time(
    const builtin_interfaces::msg::Time_<ContainerAllocator> & _arg)
  {
    this->start_time = _arg;
    return *this;
  }
  Type & set__order(
    const uint8_t & _arg)
  {
    this->order = _arg;
    return *this;
  }
  Type & set__coef_x(
    const std::vector<float, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<float>> & _arg)
  {
    this->coef_x = _arg;
    return *this;
  }
  Type & set__coef_y(
    const std::vector<float, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<float>> & _arg)
  {
    this->coef_y = _arg;
    return *this;
  }
  Type & set__coef_z(
    const std::vector<float, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<float>> & _arg)
  {
    this->coef_z = _arg;
    return *this;
  }
  Type & set__duration(
    const std::vector<float, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<float>> & _arg)
  {
    this->duration = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    ommpc_ros2::msg::PolyTraj_<ContainerAllocator> *;
  using ConstRawPtr =
    const ommpc_ros2::msg::PolyTraj_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<ommpc_ros2::msg::PolyTraj_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<ommpc_ros2::msg::PolyTraj_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      ommpc_ros2::msg::PolyTraj_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<ommpc_ros2::msg::PolyTraj_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      ommpc_ros2::msg::PolyTraj_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<ommpc_ros2::msg::PolyTraj_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<ommpc_ros2::msg::PolyTraj_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<ommpc_ros2::msg::PolyTraj_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__ommpc_ros2__msg__PolyTraj
    std::shared_ptr<ommpc_ros2::msg::PolyTraj_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__ommpc_ros2__msg__PolyTraj
    std::shared_ptr<ommpc_ros2::msg::PolyTraj_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const PolyTraj_ & other) const
  {
    if (this->drone_id != other.drone_id) {
      return false;
    }
    if (this->traj_id != other.traj_id) {
      return false;
    }
    if (this->start_time != other.start_time) {
      return false;
    }
    if (this->order != other.order) {
      return false;
    }
    if (this->coef_x != other.coef_x) {
      return false;
    }
    if (this->coef_y != other.coef_y) {
      return false;
    }
    if (this->coef_z != other.coef_z) {
      return false;
    }
    if (this->duration != other.duration) {
      return false;
    }
    return true;
  }
  bool operator!=(const PolyTraj_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct PolyTraj_

// alias to use template instance with default allocator
using PolyTraj =
  ommpc_ros2::msg::PolyTraj_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace ommpc_ros2

#endif  // OMMPC_ROS2__MSG__DETAIL__POLY_TRAJ__STRUCT_HPP_
