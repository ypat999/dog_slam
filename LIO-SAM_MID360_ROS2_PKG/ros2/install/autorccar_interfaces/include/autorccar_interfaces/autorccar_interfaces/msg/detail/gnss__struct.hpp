// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from autorccar_interfaces:msg/Gnss.idl
// generated code does not contain a copyright notice

#ifndef AUTORCCAR_INTERFACES__MSG__DETAIL__GNSS__STRUCT_HPP_
#define AUTORCCAR_INTERFACES__MSG__DETAIL__GNSS__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'timestamp'
#include "builtin_interfaces/msg/detail/time__struct.hpp"
// Member 'position_ecef'
// Member 'velocity_ecef'
#include "geometry_msgs/msg/detail/vector3__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__autorccar_interfaces__msg__Gnss __attribute__((deprecated))
#else
# define DEPRECATED__autorccar_interfaces__msg__Gnss __declspec(deprecated)
#endif

namespace autorccar_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Gnss_
{
  using Type = Gnss_<ContainerAllocator>;

  explicit Gnss_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : timestamp(_init),
    position_ecef(_init),
    velocity_ecef(_init)
  {
    (void)_init;
  }

  explicit Gnss_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : timestamp(_alloc, _init),
    position_ecef(_alloc, _init),
    velocity_ecef(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _timestamp_type =
    builtin_interfaces::msg::Time_<ContainerAllocator>;
  _timestamp_type timestamp;
  using _position_ecef_type =
    geometry_msgs::msg::Vector3_<ContainerAllocator>;
  _position_ecef_type position_ecef;
  using _velocity_ecef_type =
    geometry_msgs::msg::Vector3_<ContainerAllocator>;
  _velocity_ecef_type velocity_ecef;

  // setters for named parameter idiom
  Type & set__timestamp(
    const builtin_interfaces::msg::Time_<ContainerAllocator> & _arg)
  {
    this->timestamp = _arg;
    return *this;
  }
  Type & set__position_ecef(
    const geometry_msgs::msg::Vector3_<ContainerAllocator> & _arg)
  {
    this->position_ecef = _arg;
    return *this;
  }
  Type & set__velocity_ecef(
    const geometry_msgs::msg::Vector3_<ContainerAllocator> & _arg)
  {
    this->velocity_ecef = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    autorccar_interfaces::msg::Gnss_<ContainerAllocator> *;
  using ConstRawPtr =
    const autorccar_interfaces::msg::Gnss_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<autorccar_interfaces::msg::Gnss_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<autorccar_interfaces::msg::Gnss_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      autorccar_interfaces::msg::Gnss_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<autorccar_interfaces::msg::Gnss_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      autorccar_interfaces::msg::Gnss_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<autorccar_interfaces::msg::Gnss_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<autorccar_interfaces::msg::Gnss_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<autorccar_interfaces::msg::Gnss_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__autorccar_interfaces__msg__Gnss
    std::shared_ptr<autorccar_interfaces::msg::Gnss_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__autorccar_interfaces__msg__Gnss
    std::shared_ptr<autorccar_interfaces::msg::Gnss_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Gnss_ & other) const
  {
    if (this->timestamp != other.timestamp) {
      return false;
    }
    if (this->position_ecef != other.position_ecef) {
      return false;
    }
    if (this->velocity_ecef != other.velocity_ecef) {
      return false;
    }
    return true;
  }
  bool operator!=(const Gnss_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Gnss_

// alias to use template instance with default allocator
using Gnss =
  autorccar_interfaces::msg::Gnss_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace autorccar_interfaces

#endif  // AUTORCCAR_INTERFACES__MSG__DETAIL__GNSS__STRUCT_HPP_
