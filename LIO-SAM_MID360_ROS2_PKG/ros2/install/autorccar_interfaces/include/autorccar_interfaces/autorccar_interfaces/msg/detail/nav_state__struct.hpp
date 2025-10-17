// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from autorccar_interfaces:msg/NavState.idl
// generated code does not contain a copyright notice

#ifndef AUTORCCAR_INTERFACES__MSG__DETAIL__NAV_STATE__STRUCT_HPP_
#define AUTORCCAR_INTERFACES__MSG__DETAIL__NAV_STATE__STRUCT_HPP_

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
// Member 'origin'
// Member 'position'
// Member 'velocity'
// Member 'acceleration'
// Member 'angular_velocity'
#include "geometry_msgs/msg/detail/vector3__struct.hpp"
// Member 'quaternion'
#include "geometry_msgs/msg/detail/quaternion__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__autorccar_interfaces__msg__NavState __attribute__((deprecated))
#else
# define DEPRECATED__autorccar_interfaces__msg__NavState __declspec(deprecated)
#endif

namespace autorccar_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct NavState_
{
  using Type = NavState_<ContainerAllocator>;

  explicit NavState_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : timestamp(_init),
    origin(_init),
    position(_init),
    velocity(_init),
    quaternion(_init),
    acceleration(_init),
    angular_velocity(_init)
  {
    (void)_init;
  }

  explicit NavState_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : timestamp(_alloc, _init),
    origin(_alloc, _init),
    position(_alloc, _init),
    velocity(_alloc, _init),
    quaternion(_alloc, _init),
    acceleration(_alloc, _init),
    angular_velocity(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _timestamp_type =
    builtin_interfaces::msg::Time_<ContainerAllocator>;
  _timestamp_type timestamp;
  using _origin_type =
    geometry_msgs::msg::Vector3_<ContainerAllocator>;
  _origin_type origin;
  using _position_type =
    geometry_msgs::msg::Vector3_<ContainerAllocator>;
  _position_type position;
  using _velocity_type =
    geometry_msgs::msg::Vector3_<ContainerAllocator>;
  _velocity_type velocity;
  using _quaternion_type =
    geometry_msgs::msg::Quaternion_<ContainerAllocator>;
  _quaternion_type quaternion;
  using _acceleration_type =
    geometry_msgs::msg::Vector3_<ContainerAllocator>;
  _acceleration_type acceleration;
  using _angular_velocity_type =
    geometry_msgs::msg::Vector3_<ContainerAllocator>;
  _angular_velocity_type angular_velocity;

  // setters for named parameter idiom
  Type & set__timestamp(
    const builtin_interfaces::msg::Time_<ContainerAllocator> & _arg)
  {
    this->timestamp = _arg;
    return *this;
  }
  Type & set__origin(
    const geometry_msgs::msg::Vector3_<ContainerAllocator> & _arg)
  {
    this->origin = _arg;
    return *this;
  }
  Type & set__position(
    const geometry_msgs::msg::Vector3_<ContainerAllocator> & _arg)
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
  Type & set__acceleration(
    const geometry_msgs::msg::Vector3_<ContainerAllocator> & _arg)
  {
    this->acceleration = _arg;
    return *this;
  }
  Type & set__angular_velocity(
    const geometry_msgs::msg::Vector3_<ContainerAllocator> & _arg)
  {
    this->angular_velocity = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    autorccar_interfaces::msg::NavState_<ContainerAllocator> *;
  using ConstRawPtr =
    const autorccar_interfaces::msg::NavState_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<autorccar_interfaces::msg::NavState_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<autorccar_interfaces::msg::NavState_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      autorccar_interfaces::msg::NavState_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<autorccar_interfaces::msg::NavState_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      autorccar_interfaces::msg::NavState_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<autorccar_interfaces::msg::NavState_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<autorccar_interfaces::msg::NavState_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<autorccar_interfaces::msg::NavState_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__autorccar_interfaces__msg__NavState
    std::shared_ptr<autorccar_interfaces::msg::NavState_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__autorccar_interfaces__msg__NavState
    std::shared_ptr<autorccar_interfaces::msg::NavState_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const NavState_ & other) const
  {
    if (this->timestamp != other.timestamp) {
      return false;
    }
    if (this->origin != other.origin) {
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
    if (this->acceleration != other.acceleration) {
      return false;
    }
    if (this->angular_velocity != other.angular_velocity) {
      return false;
    }
    return true;
  }
  bool operator!=(const NavState_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct NavState_

// alias to use template instance with default allocator
using NavState =
  autorccar_interfaces::msg::NavState_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace autorccar_interfaces

#endif  // AUTORCCAR_INTERFACES__MSG__DETAIL__NAV_STATE__STRUCT_HPP_
