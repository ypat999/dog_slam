// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from explore_lite:msg/ExploreStatus.idl
// generated code does not contain a copyright notice

#ifndef EXPLORE_LITE__MSG__DETAIL__EXPLORE_STATUS__STRUCT_HPP_
#define EXPLORE_LITE__MSG__DETAIL__EXPLORE_STATUS__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__explore_lite__msg__ExploreStatus __attribute__((deprecated))
#else
# define DEPRECATED__explore_lite__msg__ExploreStatus __declspec(deprecated)
#endif

namespace explore_lite
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct ExploreStatus_
{
  using Type = ExploreStatus_<ContainerAllocator>;

  explicit ExploreStatus_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->state = 0;
      this->state_description = "";
      this->frontiers_explored = 0ul;
      this->frontiers_remaining = 0ul;
      this->progress_percentage = 0.0f;
      this->returning_to_init = false;
    }
  }

  explicit ExploreStatus_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init),
    state_description(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->state = 0;
      this->state_description = "";
      this->frontiers_explored = 0ul;
      this->frontiers_remaining = 0ul;
      this->progress_percentage = 0.0f;
      this->returning_to_init = false;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _state_type =
    uint8_t;
  _state_type state;
  using _state_description_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _state_description_type state_description;
  using _frontiers_explored_type =
    uint32_t;
  _frontiers_explored_type frontiers_explored;
  using _frontiers_remaining_type =
    uint32_t;
  _frontiers_remaining_type frontiers_remaining;
  using _progress_percentage_type =
    float;
  _progress_percentage_type progress_percentage;
  using _returning_to_init_type =
    bool;
  _returning_to_init_type returning_to_init;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__state(
    const uint8_t & _arg)
  {
    this->state = _arg;
    return *this;
  }
  Type & set__state_description(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->state_description = _arg;
    return *this;
  }
  Type & set__frontiers_explored(
    const uint32_t & _arg)
  {
    this->frontiers_explored = _arg;
    return *this;
  }
  Type & set__frontiers_remaining(
    const uint32_t & _arg)
  {
    this->frontiers_remaining = _arg;
    return *this;
  }
  Type & set__progress_percentage(
    const float & _arg)
  {
    this->progress_percentage = _arg;
    return *this;
  }
  Type & set__returning_to_init(
    const bool & _arg)
  {
    this->returning_to_init = _arg;
    return *this;
  }

  // constant declarations
  static constexpr uint8_t STATE_STARTING =
    0u;
  static constexpr uint8_t STATE_EXPLORING =
    1u;
  static constexpr uint8_t STATE_COMPLETED =
    2u;
  static constexpr uint8_t STATE_STOPPED =
    3u;

  // pointer types
  using RawPtr =
    explore_lite::msg::ExploreStatus_<ContainerAllocator> *;
  using ConstRawPtr =
    const explore_lite::msg::ExploreStatus_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<explore_lite::msg::ExploreStatus_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<explore_lite::msg::ExploreStatus_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      explore_lite::msg::ExploreStatus_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<explore_lite::msg::ExploreStatus_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      explore_lite::msg::ExploreStatus_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<explore_lite::msg::ExploreStatus_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<explore_lite::msg::ExploreStatus_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<explore_lite::msg::ExploreStatus_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__explore_lite__msg__ExploreStatus
    std::shared_ptr<explore_lite::msg::ExploreStatus_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__explore_lite__msg__ExploreStatus
    std::shared_ptr<explore_lite::msg::ExploreStatus_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const ExploreStatus_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->state != other.state) {
      return false;
    }
    if (this->state_description != other.state_description) {
      return false;
    }
    if (this->frontiers_explored != other.frontiers_explored) {
      return false;
    }
    if (this->frontiers_remaining != other.frontiers_remaining) {
      return false;
    }
    if (this->progress_percentage != other.progress_percentage) {
      return false;
    }
    if (this->returning_to_init != other.returning_to_init) {
      return false;
    }
    return true;
  }
  bool operator!=(const ExploreStatus_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct ExploreStatus_

// alias to use template instance with default allocator
using ExploreStatus =
  explore_lite::msg::ExploreStatus_<std::allocator<void>>;

// constant definitions
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t ExploreStatus_<ContainerAllocator>::STATE_STARTING;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t ExploreStatus_<ContainerAllocator>::STATE_EXPLORING;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t ExploreStatus_<ContainerAllocator>::STATE_COMPLETED;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t ExploreStatus_<ContainerAllocator>::STATE_STOPPED;
#endif  // __cplusplus < 201703L

}  // namespace msg

}  // namespace explore_lite

#endif  // EXPLORE_LITE__MSG__DETAIL__EXPLORE_STATUS__STRUCT_HPP_
