// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from autorccar_interfaces:msg/ControlCommand.idl
// generated code does not contain a copyright notice

#ifndef AUTORCCAR_INTERFACES__MSG__DETAIL__CONTROL_COMMAND__STRUCT_HPP_
#define AUTORCCAR_INTERFACES__MSG__DETAIL__CONTROL_COMMAND__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__autorccar_interfaces__msg__ControlCommand __attribute__((deprecated))
#else
# define DEPRECATED__autorccar_interfaces__msg__ControlCommand __declspec(deprecated)
#endif

namespace autorccar_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct ControlCommand_
{
  using Type = ControlCommand_<ContainerAllocator>;

  explicit ControlCommand_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->speed = 0.0;
      this->steering_angle = 0.0;
    }
  }

  explicit ControlCommand_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->speed = 0.0;
      this->steering_angle = 0.0;
    }
  }

  // field types and members
  using _speed_type =
    double;
  _speed_type speed;
  using _steering_angle_type =
    double;
  _steering_angle_type steering_angle;

  // setters for named parameter idiom
  Type & set__speed(
    const double & _arg)
  {
    this->speed = _arg;
    return *this;
  }
  Type & set__steering_angle(
    const double & _arg)
  {
    this->steering_angle = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    autorccar_interfaces::msg::ControlCommand_<ContainerAllocator> *;
  using ConstRawPtr =
    const autorccar_interfaces::msg::ControlCommand_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<autorccar_interfaces::msg::ControlCommand_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<autorccar_interfaces::msg::ControlCommand_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      autorccar_interfaces::msg::ControlCommand_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<autorccar_interfaces::msg::ControlCommand_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      autorccar_interfaces::msg::ControlCommand_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<autorccar_interfaces::msg::ControlCommand_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<autorccar_interfaces::msg::ControlCommand_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<autorccar_interfaces::msg::ControlCommand_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__autorccar_interfaces__msg__ControlCommand
    std::shared_ptr<autorccar_interfaces::msg::ControlCommand_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__autorccar_interfaces__msg__ControlCommand
    std::shared_ptr<autorccar_interfaces::msg::ControlCommand_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const ControlCommand_ & other) const
  {
    if (this->speed != other.speed) {
      return false;
    }
    if (this->steering_angle != other.steering_angle) {
      return false;
    }
    return true;
  }
  bool operator!=(const ControlCommand_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct ControlCommand_

// alias to use template instance with default allocator
using ControlCommand =
  autorccar_interfaces::msg::ControlCommand_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace autorccar_interfaces

#endif  // AUTORCCAR_INTERFACES__MSG__DETAIL__CONTROL_COMMAND__STRUCT_HPP_
