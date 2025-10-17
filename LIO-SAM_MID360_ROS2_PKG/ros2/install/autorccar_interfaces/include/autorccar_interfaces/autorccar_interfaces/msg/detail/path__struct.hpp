// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from autorccar_interfaces:msg/Path.idl
// generated code does not contain a copyright notice

#ifndef AUTORCCAR_INTERFACES__MSG__DETAIL__PATH__STRUCT_HPP_
#define AUTORCCAR_INTERFACES__MSG__DETAIL__PATH__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'path_points'
#include "autorccar_interfaces/msg/detail/path_point__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__autorccar_interfaces__msg__Path __attribute__((deprecated))
#else
# define DEPRECATED__autorccar_interfaces__msg__Path __declspec(deprecated)
#endif

namespace autorccar_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Path_
{
  using Type = Path_<ContainerAllocator>;

  explicit Path_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_init;
  }

  explicit Path_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_init;
    (void)_alloc;
  }

  // field types and members
  using _path_points_type =
    std::vector<autorccar_interfaces::msg::PathPoint_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<autorccar_interfaces::msg::PathPoint_<ContainerAllocator>>>;
  _path_points_type path_points;

  // setters for named parameter idiom
  Type & set__path_points(
    const std::vector<autorccar_interfaces::msg::PathPoint_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<autorccar_interfaces::msg::PathPoint_<ContainerAllocator>>> & _arg)
  {
    this->path_points = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    autorccar_interfaces::msg::Path_<ContainerAllocator> *;
  using ConstRawPtr =
    const autorccar_interfaces::msg::Path_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<autorccar_interfaces::msg::Path_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<autorccar_interfaces::msg::Path_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      autorccar_interfaces::msg::Path_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<autorccar_interfaces::msg::Path_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      autorccar_interfaces::msg::Path_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<autorccar_interfaces::msg::Path_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<autorccar_interfaces::msg::Path_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<autorccar_interfaces::msg::Path_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__autorccar_interfaces__msg__Path
    std::shared_ptr<autorccar_interfaces::msg::Path_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__autorccar_interfaces__msg__Path
    std::shared_ptr<autorccar_interfaces::msg::Path_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Path_ & other) const
  {
    if (this->path_points != other.path_points) {
      return false;
    }
    return true;
  }
  bool operator!=(const Path_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Path_

// alias to use template instance with default allocator
using Path =
  autorccar_interfaces::msg::Path_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace autorccar_interfaces

#endif  // AUTORCCAR_INTERFACES__MSG__DETAIL__PATH__STRUCT_HPP_
