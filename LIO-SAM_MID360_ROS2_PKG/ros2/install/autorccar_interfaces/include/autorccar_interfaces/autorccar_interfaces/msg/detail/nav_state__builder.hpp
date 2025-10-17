// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from autorccar_interfaces:msg/NavState.idl
// generated code does not contain a copyright notice

#ifndef AUTORCCAR_INTERFACES__MSG__DETAIL__NAV_STATE__BUILDER_HPP_
#define AUTORCCAR_INTERFACES__MSG__DETAIL__NAV_STATE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "autorccar_interfaces/msg/detail/nav_state__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace autorccar_interfaces
{

namespace msg
{

namespace builder
{

class Init_NavState_angular_velocity
{
public:
  explicit Init_NavState_angular_velocity(::autorccar_interfaces::msg::NavState & msg)
  : msg_(msg)
  {}
  ::autorccar_interfaces::msg::NavState angular_velocity(::autorccar_interfaces::msg::NavState::_angular_velocity_type arg)
  {
    msg_.angular_velocity = std::move(arg);
    return std::move(msg_);
  }

private:
  ::autorccar_interfaces::msg::NavState msg_;
};

class Init_NavState_acceleration
{
public:
  explicit Init_NavState_acceleration(::autorccar_interfaces::msg::NavState & msg)
  : msg_(msg)
  {}
  Init_NavState_angular_velocity acceleration(::autorccar_interfaces::msg::NavState::_acceleration_type arg)
  {
    msg_.acceleration = std::move(arg);
    return Init_NavState_angular_velocity(msg_);
  }

private:
  ::autorccar_interfaces::msg::NavState msg_;
};

class Init_NavState_quaternion
{
public:
  explicit Init_NavState_quaternion(::autorccar_interfaces::msg::NavState & msg)
  : msg_(msg)
  {}
  Init_NavState_acceleration quaternion(::autorccar_interfaces::msg::NavState::_quaternion_type arg)
  {
    msg_.quaternion = std::move(arg);
    return Init_NavState_acceleration(msg_);
  }

private:
  ::autorccar_interfaces::msg::NavState msg_;
};

class Init_NavState_velocity
{
public:
  explicit Init_NavState_velocity(::autorccar_interfaces::msg::NavState & msg)
  : msg_(msg)
  {}
  Init_NavState_quaternion velocity(::autorccar_interfaces::msg::NavState::_velocity_type arg)
  {
    msg_.velocity = std::move(arg);
    return Init_NavState_quaternion(msg_);
  }

private:
  ::autorccar_interfaces::msg::NavState msg_;
};

class Init_NavState_position
{
public:
  explicit Init_NavState_position(::autorccar_interfaces::msg::NavState & msg)
  : msg_(msg)
  {}
  Init_NavState_velocity position(::autorccar_interfaces::msg::NavState::_position_type arg)
  {
    msg_.position = std::move(arg);
    return Init_NavState_velocity(msg_);
  }

private:
  ::autorccar_interfaces::msg::NavState msg_;
};

class Init_NavState_origin
{
public:
  explicit Init_NavState_origin(::autorccar_interfaces::msg::NavState & msg)
  : msg_(msg)
  {}
  Init_NavState_position origin(::autorccar_interfaces::msg::NavState::_origin_type arg)
  {
    msg_.origin = std::move(arg);
    return Init_NavState_position(msg_);
  }

private:
  ::autorccar_interfaces::msg::NavState msg_;
};

class Init_NavState_timestamp
{
public:
  Init_NavState_timestamp()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_NavState_origin timestamp(::autorccar_interfaces::msg::NavState::_timestamp_type arg)
  {
    msg_.timestamp = std::move(arg);
    return Init_NavState_origin(msg_);
  }

private:
  ::autorccar_interfaces::msg::NavState msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::autorccar_interfaces::msg::NavState>()
{
  return autorccar_interfaces::msg::builder::Init_NavState_timestamp();
}

}  // namespace autorccar_interfaces

#endif  // AUTORCCAR_INTERFACES__MSG__DETAIL__NAV_STATE__BUILDER_HPP_
