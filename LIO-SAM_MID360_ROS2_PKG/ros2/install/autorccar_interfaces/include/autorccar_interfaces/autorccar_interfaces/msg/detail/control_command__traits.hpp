// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from autorccar_interfaces:msg/ControlCommand.idl
// generated code does not contain a copyright notice

#ifndef AUTORCCAR_INTERFACES__MSG__DETAIL__CONTROL_COMMAND__TRAITS_HPP_
#define AUTORCCAR_INTERFACES__MSG__DETAIL__CONTROL_COMMAND__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "autorccar_interfaces/msg/detail/control_command__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace autorccar_interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const ControlCommand & msg,
  std::ostream & out)
{
  out << "{";
  // member: speed
  {
    out << "speed: ";
    rosidl_generator_traits::value_to_yaml(msg.speed, out);
    out << ", ";
  }

  // member: steering_angle
  {
    out << "steering_angle: ";
    rosidl_generator_traits::value_to_yaml(msg.steering_angle, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const ControlCommand & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: speed
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "speed: ";
    rosidl_generator_traits::value_to_yaml(msg.speed, out);
    out << "\n";
  }

  // member: steering_angle
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "steering_angle: ";
    rosidl_generator_traits::value_to_yaml(msg.steering_angle, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const ControlCommand & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace autorccar_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use autorccar_interfaces::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const autorccar_interfaces::msg::ControlCommand & msg,
  std::ostream & out, size_t indentation = 0)
{
  autorccar_interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use autorccar_interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const autorccar_interfaces::msg::ControlCommand & msg)
{
  return autorccar_interfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<autorccar_interfaces::msg::ControlCommand>()
{
  return "autorccar_interfaces::msg::ControlCommand";
}

template<>
inline const char * name<autorccar_interfaces::msg::ControlCommand>()
{
  return "autorccar_interfaces/msg/ControlCommand";
}

template<>
struct has_fixed_size<autorccar_interfaces::msg::ControlCommand>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<autorccar_interfaces::msg::ControlCommand>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<autorccar_interfaces::msg::ControlCommand>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // AUTORCCAR_INTERFACES__MSG__DETAIL__CONTROL_COMMAND__TRAITS_HPP_
