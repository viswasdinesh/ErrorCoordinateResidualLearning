// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from ommpc_ros2:msg/ReferenceState.idl
// generated code does not contain a copyright notice

#ifndef OMMPC_ROS2__MSG__DETAIL__REFERENCE_STATE__STRUCT_HPP_
#define OMMPC_ROS2__MSG__DETAIL__REFERENCE_STATE__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'timestamp'
#include "builtin_interfaces/msg/detail/time__struct.hpp"
// Member 'position'
#include "geometry_msgs/msg/detail/point__struct.hpp"
// Member 'velocity'
// Member 'body_rates'
#include "geometry_msgs/msg/detail/vector3__struct.hpp"
// Member 'quaternion'
#include "geometry_msgs/msg/detail/quaternion__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__ommpc_ros2__msg__ReferenceState __attribute__((deprecated))
#else
# define DEPRECATED__ommpc_ros2__msg__ReferenceState __declspec(deprecated)
#endif

namespace ommpc_ros2
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct ReferenceState_
{
  using Type = ReferenceState_<ContainerAllocator>;

  explicit ReferenceState_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : timestamp(_init),
    position(_init),
    velocity(_init),
    quaternion(_init),
    body_rates(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->thrust = 0.0;
    }
  }

  explicit ReferenceState_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : timestamp(_alloc, _init),
    position(_alloc, _init),
    velocity(_alloc, _init),
    quaternion(_alloc, _init),
    body_rates(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->thrust = 0.0;
    }
  }

  // field types and members
  using _timestamp_type =
    builtin_interfaces::msg::Time_<ContainerAllocator>;
  _timestamp_type timestamp;
  using _position_type =
    geometry_msgs::msg::Point_<ContainerAllocator>;
  _position_type position;
  using _velocity_type =
    geometry_msgs::msg::Vector3_<ContainerAllocator>;
  _velocity_type velocity;
  using _quaternion_type =
    geometry_msgs::msg::Quaternion_<ContainerAllocator>;
  _quaternion_type quaternion;
  using _body_rates_type =
    geometry_msgs::msg::Vector3_<ContainerAllocator>;
  _body_rates_type body_rates;
  using _thrust_type =
    double;
  _thrust_type thrust;

  // setters for named parameter idiom
  Type & set__timestamp(
    const builtin_interfaces::msg::Time_<ContainerAllocator> & _arg)
  {
    this->timestamp = _arg;
    return *this;
  }
  Type & set__position(
    const geometry_msgs::msg::Point_<ContainerAllocator> & _arg)
  {
    this->position = _arg;
    return *this;
  }
  Type & set__velocity(
    const geometry_msgs::msg::Vector3_<ContainerAllocator> & _arg)
  {
    this->velocity = _arg;
    return *this;
  }
  Type & set__quaternion(
    const geometry_msgs::msg::Quaternion_<ContainerAllocator> & _arg)
  {
    this->quaternion = _arg;
    return *this;
  }
  Type & set__body_rates(
    const geometry_msgs::msg::Vector3_<ContainerAllocator> & _arg)
  {
    this->body_rates = _arg;
    return *this;
  }
  Type & set__thrust(
    const double & _arg)
  {
    this->thrust = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    ommpc_ros2::msg::ReferenceState_<ContainerAllocator> *;
  using ConstRawPtr =
    const ommpc_ros2::msg::ReferenceState_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<ommpc_ros2::msg::ReferenceState_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<ommpc_ros2::msg::ReferenceState_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      ommpc_ros2::msg::ReferenceState_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<ommpc_ros2::msg::ReferenceState_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      ommpc_ros2::msg::ReferenceState_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<ommpc_ros2::msg::ReferenceState_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<ommpc_ros2::msg::ReferenceState_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<ommpc_ros2::msg::ReferenceState_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__ommpc_ros2__msg__ReferenceState
    std::shared_ptr<ommpc_ros2::msg::ReferenceState_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__ommpc_ros2__msg__ReferenceState
    std::shared_ptr<ommpc_ros2::msg::ReferenceState_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const ReferenceState_ & other) const
  {
    if (this->timestamp != other.timestamp) {
      return false;
    }
    if (this->position != other.position) {
      return false;
    }
    if (this->velocity != other.velocity) {
      return false;
    }
    if (this->quaternion != other.quaternion) {
      return false;
    }
    if (this->body_rates != other.body_rates) {
      return false;
    }
    if (this->thrust != other.thrust) {
      return false;
    }
    return true;
  }
  bool operator!=(const ReferenceState_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct ReferenceState_

// alias to use template instance with default allocator
using ReferenceState =
  ommpc_ros2::msg::ReferenceState_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace ommpc_ros2

#endif  // OMMPC_ROS2__MSG__DETAIL__REFERENCE_STATE__STRUCT_HPP_
