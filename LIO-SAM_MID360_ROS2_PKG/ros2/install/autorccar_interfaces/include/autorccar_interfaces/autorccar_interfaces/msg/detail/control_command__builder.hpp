// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from autorccar_interfaces:msg/ControlCommand.idl
// generated code does not contain a copyright notice

#ifndef AUTORCCAR_INTERFACES__MSG__DETAIL__CONTROL_COMMAND__BUILDER_HPP_
#define AUTORCCAR_INTERFACES__MSG__DETAIL__CONTROL_COMMAND__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "autorccar_interfaces/msg/detail/control_command__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace autorccar_interfaces
{

namespace msg
{

namespace builder
{

class Init_ControlCommand_steering_angle
{
public:
  explicit Init_ControlCommand_steering_angle(::autorccar_interfaces::msg::ControlCommand & msg)
  : msg_(msg)
  {}
  ::autorccar_interfaces::msg::ControlCommand steering_angle(::autorccar_interfaces::msg::ControlCommand::_steering_angle_type arg)
  {
    msg_.steering_angle = std::move(arg);
    return std::move(msg_);
  }

private:
  ::autorccar_interfaces::msg::ControlCommand msg_;
};

class Init_ControlCommand_speed
{
public:
  Init_ControlCommand_speed()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ControlCommand_steering_angle speed(::autorccar_interfaces::msg::ControlCommand::_speed_type arg)
  {
    msg_.speed = std::move(arg);
    return Init_ControlCommand_steering_angle(msg_);
  }

private:
  ::autorccar_interfaces::msg::ControlCommand msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::autorccar_interfaces::msg::ControlCommand>()
{
  return autorccar_interfaces::msg::builder::Init_ControlCommand_speed();
}

}  // namespace autorccar_interfaces

#endif  // AUTORCCAR_INTERFACES__MSG__DETAIL__CONTROL_COMMAND__BUILDER_HPP_
