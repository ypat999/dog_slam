// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from autorccar_interfaces:msg/Path.idl
// generated code does not contain a copyright notice

#ifndef AUTORCCAR_INTERFACES__MSG__DETAIL__PATH__BUILDER_HPP_
#define AUTORCCAR_INTERFACES__MSG__DETAIL__PATH__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "autorccar_interfaces/msg/detail/path__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace autorccar_interfaces
{

namespace msg
{

namespace builder
{

class Init_Path_path_points
{
public:
  Init_Path_path_points()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::autorccar_interfaces::msg::Path path_points(::autorccar_interfaces::msg::Path::_path_points_type arg)
  {
    msg_.path_points = std::move(arg);
    return std::move(msg_);
  }

private:
  ::autorccar_interfaces::msg::Path msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::autorccar_interfaces::msg::Path>()
{
  return autorccar_interfaces::msg::builder::Init_Path_path_points();
}

}  // namespace autorccar_interfaces

#endif  // AUTORCCAR_INTERFACES__MSG__DETAIL__PATH__BUILDER_HPP_
