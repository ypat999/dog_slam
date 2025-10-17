// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from autorccar_interfaces:msg/Gnss.idl
// generated code does not contain a copyright notice

#ifndef AUTORCCAR_INTERFACES__MSG__DETAIL__GNSS__BUILDER_HPP_
#define AUTORCCAR_INTERFACES__MSG__DETAIL__GNSS__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "autorccar_interfaces/msg/detail/gnss__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace autorccar_interfaces
{

namespace msg
{

namespace builder
{

class Init_Gnss_velocity_ecef
{
public:
  explicit Init_Gnss_velocity_ecef(::autorccar_interfaces::msg::Gnss & msg)
  : msg_(msg)
  {}
  ::autorccar_interfaces::msg::Gnss velocity_ecef(::autorccar_interfaces::msg::Gnss::_velocity_ecef_type arg)
  {
    msg_.velocity_ecef = std::move(arg);
    return std::move(msg_);
  }

private:
  ::autorccar_interfaces::msg::Gnss msg_;
};

class Init_Gnss_position_ecef
{
public:
  explicit Init_Gnss_position_ecef(::autorccar_interfaces::msg::Gnss & msg)
  : msg_(msg)
  {}
  Init_Gnss_velocity_ecef position_ecef(::autorccar_interfaces::msg::Gnss::_position_ecef_type arg)
  {
    msg_.position_ecef = std::move(arg);
    return Init_Gnss_velocity_ecef(msg_);
  }

private:
  ::autorccar_interfaces::msg::Gnss msg_;
};

class Init_Gnss_timestamp
{
public:
  Init_Gnss_timestamp()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Gnss_position_ecef timestamp(::autorccar_interfaces::msg::Gnss::_timestamp_type arg)
  {
    msg_.timestamp = std::move(arg);
    return Init_Gnss_position_ecef(msg_);
  }

private:
  ::autorccar_interfaces::msg::Gnss msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::autorccar_interfaces::msg::Gnss>()
{
  return autorccar_interfaces::msg::builder::Init_Gnss_timestamp();
}

}  // namespace autorccar_interfaces

#endif  // AUTORCCAR_INTERFACES__MSG__DETAIL__GNSS__BUILDER_HPP_
