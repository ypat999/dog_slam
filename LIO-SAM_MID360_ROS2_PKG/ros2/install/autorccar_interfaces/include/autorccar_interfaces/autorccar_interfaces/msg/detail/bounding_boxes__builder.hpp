// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from autorccar_interfaces:msg/BoundingBoxes.idl
// generated code does not contain a copyright notice

#ifndef AUTORCCAR_INTERFACES__MSG__DETAIL__BOUNDING_BOXES__BUILDER_HPP_
#define AUTORCCAR_INTERFACES__MSG__DETAIL__BOUNDING_BOXES__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "autorccar_interfaces/msg/detail/bounding_boxes__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace autorccar_interfaces
{

namespace msg
{

namespace builder
{

class Init_BoundingBoxes_bounding_boxes
{
public:
  explicit Init_BoundingBoxes_bounding_boxes(::autorccar_interfaces::msg::BoundingBoxes & msg)
  : msg_(msg)
  {}
  ::autorccar_interfaces::msg::BoundingBoxes bounding_boxes(::autorccar_interfaces::msg::BoundingBoxes::_bounding_boxes_type arg)
  {
    msg_.bounding_boxes = std::move(arg);
    return std::move(msg_);
  }

private:
  ::autorccar_interfaces::msg::BoundingBoxes msg_;
};

class Init_BoundingBoxes_num
{
public:
  Init_BoundingBoxes_num()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_BoundingBoxes_bounding_boxes num(::autorccar_interfaces::msg::BoundingBoxes::_num_type arg)
  {
    msg_.num = std::move(arg);
    return Init_BoundingBoxes_bounding_boxes(msg_);
  }

private:
  ::autorccar_interfaces::msg::BoundingBoxes msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::autorccar_interfaces::msg::BoundingBoxes>()
{
  return autorccar_interfaces::msg::builder::Init_BoundingBoxes_num();
}

}  // namespace autorccar_interfaces

#endif  // AUTORCCAR_INTERFACES__MSG__DETAIL__BOUNDING_BOXES__BUILDER_HPP_
