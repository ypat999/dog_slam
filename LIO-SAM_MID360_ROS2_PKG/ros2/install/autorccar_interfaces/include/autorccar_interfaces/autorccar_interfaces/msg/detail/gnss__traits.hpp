// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from autorccar_interfaces:msg/Gnss.idl
// generated code does not contain a copyright notice

#ifndef AUTORCCAR_INTERFACES__MSG__DETAIL__GNSS__TRAITS_HPP_
#define AUTORCCAR_INTERFACES__MSG__DETAIL__GNSS__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "autorccar_interfaces/msg/detail/gnss__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'timestamp'
#include "builtin_interfaces/msg/detail/time__traits.hpp"
// Member 'position_ecef'
// Member 'velocity_ecef'
#include "geometry_msgs/msg/detail/vector3__traits.hpp"

namespace autorccar_interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const Gnss & msg,
  std::ostream & out)
{
  out << "{";
  // member: timestamp
  {
    out << "timestamp: ";
    to_flow_style_yaml(msg.timestamp, out);
    out << ", ";
  }

  // member: position_ecef
  {
    out << "position_ecef: ";
    to_flow_style_yaml(msg.position_ecef, out);
    out << ", ";
  }

  // member: velocity_ecef
  {
    out << "velocity_ecef: ";
    to_flow_style_yaml(msg.velocity_ecef, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Gnss & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: timestamp
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "timestamp:\n";
    to_block_style_yaml(msg.timestamp, out, indentation + 2);
  }

  // member: position_ecef
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "position_ecef:\n";
    to_block_style_yaml(msg.position_ecef, out, indentation + 2);
  }

  // member: velocity_ecef
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "velocity_ecef:\n";
    to_block_style_yaml(msg.velocity_ecef, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Gnss & msg, bool use_flow_style = false)
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
  const autorccar_interfaces::msg::Gnss & msg,
  std::ostream & out, size_t indentation = 0)
{
  autorccar_interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use autorccar_interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const autorccar_interfaces::msg::Gnss & msg)
{
  return autorccar_interfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<autorccar_interfaces::msg::Gnss>()
{
  return "autorccar_interfaces::msg::Gnss";
}

template<>
inline const char * name<autorccar_interfaces::msg::Gnss>()
{
  return "autorccar_interfaces/msg/Gnss";
}

template<>
struct has_fixed_size<autorccar_interfaces::msg::Gnss>
  : std::integral_constant<bool, has_fixed_size<builtin_interfaces::msg::Time>::value && has_fixed_size<geometry_msgs::msg::Vector3>::value> {};

template<>
struct has_bounded_size<autorccar_interfaces::msg::Gnss>
  : std::integral_constant<bool, has_bounded_size<builtin_interfaces::msg::Time>::value && has_bounded_size<geometry_msgs::msg::Vector3>::value> {};

template<>
struct is_message<autorccar_interfaces::msg::Gnss>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // AUTORCCAR_INTERFACES__MSG__DETAIL__GNSS__TRAITS_HPP_
