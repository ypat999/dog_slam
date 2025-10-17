// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from autorccar_interfaces:msg/BoundingBoxes.idl
// generated code does not contain a copyright notice

#ifndef AUTORCCAR_INTERFACES__MSG__DETAIL__BOUNDING_BOXES__STRUCT_HPP_
#define AUTORCCAR_INTERFACES__MSG__DETAIL__BOUNDING_BOXES__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'bounding_boxes'
#include "vision_msgs/msg/detail/bounding_box2_d__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__autorccar_interfaces__msg__BoundingBoxes __attribute__((deprecated))
#else
# define DEPRECATED__autorccar_interfaces__msg__BoundingBoxes __declspec(deprecated)
#endif

namespace autorccar_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct BoundingBoxes_
{
  using Type = BoundingBoxes_<ContainerAllocator>;

  explicit BoundingBoxes_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->num = 0ll;
    }
  }

  explicit BoundingBoxes_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->num = 0ll;
    }
  }

  // field types and members
  using _num_type =
    int64_t;
  _num_type num;
  using _bounding_boxes_type =
    std::vector<vision_msgs::msg::BoundingBox2D_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<vision_msgs::msg::BoundingBox2D_<ContainerAllocator>>>;
  _bounding_boxes_type bounding_boxes;

  // setters for named parameter idiom
  Type & set__num(
    const int64_t & _arg)
  {
    this->num = _arg;
    return *this;
  }
  Type & set__bounding_boxes(
    const std::vector<vision_msgs::msg::BoundingBox2D_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<vision_msgs::msg::BoundingBox2D_<ContainerAllocator>>> & _arg)
  {
    this->bounding_boxes = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    autorccar_interfaces::msg::BoundingBoxes_<ContainerAllocator> *;
  using ConstRawPtr =
    const autorccar_interfaces::msg::BoundingBoxes_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<autorccar_interfaces::msg::BoundingBoxes_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<autorccar_interfaces::msg::BoundingBoxes_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      autorccar_interfaces::msg::BoundingBoxes_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<autorccar_interfaces::msg::BoundingBoxes_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      autorccar_interfaces::msg::BoundingBoxes_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<autorccar_interfaces::msg::BoundingBoxes_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<autorccar_interfaces::msg::BoundingBoxes_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<autorccar_interfaces::msg::BoundingBoxes_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__autorccar_interfaces__msg__BoundingBoxes
    std::shared_ptr<autorccar_interfaces::msg::BoundingBoxes_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__autorccar_interfaces__msg__BoundingBoxes
    std::shared_ptr<autorccar_interfaces::msg::BoundingBoxes_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const BoundingBoxes_ & other) const
  {
    if (this->num != other.num) {
      return false;
    }
    if (this->bounding_boxes != other.bounding_boxes) {
      return false;
    }
    return true;
  }
  bool operator!=(const BoundingBoxes_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct BoundingBoxes_

// alias to use template instance with default allocator
using BoundingBoxes =
  autorccar_interfaces::msg::BoundingBoxes_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace autorccar_interfaces

#endif  // AUTORCCAR_INTERFACES__MSG__DETAIL__BOUNDING_BOXES__STRUCT_HPP_
