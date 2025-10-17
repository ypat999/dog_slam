// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from autorccar_interfaces:msg/Imu.idl
// generated code does not contain a copyright notice

#ifndef AUTORCCAR_INTERFACES__MSG__DETAIL__IMU__BUILDER_HPP_
#define AUTORCCAR_INTERFACES__MSG__DETAIL__IMU__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "autorccar_interfaces/msg/detail/imu__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace autorccar_interfaces
{

namespace msg
{

namespace builder
{

class Init_Imu_linear_acceleration
{
public:
  explicit Init_Imu_linear_acceleration(::autorccar_interfaces::msg::Imu & msg)
  : msg_(msg)
  {}
  ::autorccar_interfaces::msg::Imu linear_acceleration(::autorccar_interfaces::msg::Imu::_linear_acceleration_type arg)
  {
    msg_.linear_acceleration = std::move(arg);
    return std::move(msg_);
  }

private:
  ::autorccar_interfaces::msg::Imu msg_;
};

class Init_Imu_angular_velocity
{
public:
  explicit Init_Imu_angular_velocity(::autorccar_interfaces::msg::Imu & msg)
  : msg_(msg)
  {}
  Init_Imu_linear_acceleration angular_velocity(::autorccar_interfaces::msg::Imu::_angular_velocity_type arg)
  {
    msg_.angular_velocity = std::move(arg);
    return Init_Imu_linear_acceleration(msg_);
  }

private:
  ::autorccar_interfaces::msg::Imu msg_;
};

class Init_Imu_timestamp
{
public:
  Init_Imu_timestamp()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Imu_angular_velocity timestamp(::autorccar_interfaces::msg::Imu::_timestamp_type arg)
  {
    msg_.timestamp = std::move(arg);
    return Init_Imu_angular_velocity(msg_);
  }

private:
  ::autorccar_interfaces::msg::Imu msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::autorccar_interfaces::msg::Imu>()
{
  return autorccar_interfaces::msg::builder::Init_Imu_timestamp();
}

}  // namespace autorccar_interfaces

#endif  // AUTORCCAR_INTERFACES__MSG__DETAIL__IMU__BUILDER_HPP_
