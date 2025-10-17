// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from autorccar_interfaces:msg/Path.idl
// generated code does not contain a copyright notice

#ifndef AUTORCCAR_INTERFACES__MSG__DETAIL__PATH__TRAITS_HPP_
#define AUTORCCAR_INTERFACES__MSG__DETAIL__PATH__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "autorccar_interfaces/msg/detail/path__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'path_points'
#include "autorccar_interfaces/msg/detail/path_point__traits.hpp"

namespace autorccar_interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const Path & msg,
  std::ostream & out)
{
  out << "{";
  // member: path_points
  {
    if (msg.path_points.size() == 0) {
      out << "path_points: []";
    } else {
      out << "path_points: [";
      size_t pending_items = msg.path_points.size();
      for (auto item : msg.path_points) {
        to_flow_style_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Path & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: path_points
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.path_points.size() == 0) {
      out << "path_points: []\n";
    } else {
      out << "path_points:\n";
      for (auto item : msg.path_points) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Path & msg, bool use_flow_style = false)
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
  const autorccar_interfaces::msg::Path & msg,
  std::ostream & out, size_t indentation = 0)
{
  autorccar_interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use autorccar_interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const autorccar_interfaces::msg::Path & msg)
{
  return autorccar_interfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<autorccar_interfaces::msg::Path>()
{
  return "autorccar_interfaces::msg::Path";
}

template<>
inline const char * name<autorccar_interfaces::msg::Path>()
{
  return "autorccar_interfaces/msg/Path";
}

template<>
struct has_fixed_size<autorccar_interfaces::msg::Path>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<autorccar_interfaces::msg::Path>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<autorccar_interfaces::msg::Path>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // AUTORCCAR_INTERFACES__MSG__DETAIL__PATH__TRAITS_HPP_
