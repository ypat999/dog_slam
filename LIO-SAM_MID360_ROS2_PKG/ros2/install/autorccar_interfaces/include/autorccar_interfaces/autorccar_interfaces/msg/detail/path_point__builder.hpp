// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from autorccar_interfaces:msg/PathPoint.idl
// generated code does not contain a copyright notice

#ifndef AUTORCCAR_INTERFACES__MSG__DETAIL__PATH_POINT__BUILDER_HPP_
#define AUTORCCAR_INTERFACES__MSG__DETAIL__PATH_POINT__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "autorccar_interfaces/msg/detail/path_point__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace autorccar_interfaces
{

namespace msg
{

namespace builder
{

class Init_PathPoint_speed
{
public:
  explicit Init_PathPoint_speed(::autorccar_interfaces::msg::PathPoint & msg)
  : msg_(msg)
  {}
  ::autorccar_interfaces::msg::PathPoint speed(::autorccar_interfaces::msg::PathPoint::_speed_type arg)
  {
    msg_.speed = std::move(arg);
    return std::move(msg_);
  }

private:
  ::autorccar_interfaces::msg::PathPoint msg_;
};

class Init_PathPoint_y
{
public:
  explicit Init_PathPoint_y(::autorccar_interfaces::msg::PathPoint & msg)
  : msg_(msg)
  {}
  Init_PathPoint_speed y(::autorccar_interfaces::msg::PathPoint::_y_type arg)
  {
    msg_.y = std::move(arg);
    return Init_PathPoint_speed(msg_);
  }

private:
  ::autorccar_interfaces::msg::PathPoint msg_;
};

class Init_PathPoint_x
{
public:
  Init_PathPoint_x()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_PathPoint_y x(::autorccar_interfaces::msg::PathPoint::_x_type arg)
  {
    msg_.x = std::move(arg);
    return Init_PathPoint_y(msg_);
  }

private:
  ::autorccar_interfaces::msg::PathPoint msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::autorccar_interfaces::msg::PathPoint>()
{
  return autorccar_interfaces::msg::builder::Init_PathPoint_x();
}

}  // namespace autorccar_interfaces

#endif  // AUTORCCAR_INTERFACES__MSG__DETAIL__PATH_POINT__BUILDER_HPP_
