// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from explore_lite:msg/ExploreStatus.idl
// generated code does not contain a copyright notice

#ifndef EXPLORE_LITE__MSG__DETAIL__EXPLORE_STATUS__TRAITS_HPP_
#define EXPLORE_LITE__MSG__DETAIL__EXPLORE_STATUS__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "explore_lite/msg/detail/explore_status__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"

namespace explore_lite
{

namespace msg
{

inline void to_flow_style_yaml(
  const ExploreStatus & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: state
  {
    out << "state: ";
    rosidl_generator_traits::value_to_yaml(msg.state, out);
    out << ", ";
  }

  // member: state_description
  {
    out << "state_description: ";
    rosidl_generator_traits::value_to_yaml(msg.state_description, out);
    out << ", ";
  }

  // member: frontiers_explored
  {
    out << "frontiers_explored: ";
    rosidl_generator_traits::value_to_yaml(msg.frontiers_explored, out);
    out << ", ";
  }

  // member: frontiers_remaining
  {
    out << "frontiers_remaining: ";
    rosidl_generator_traits::value_to_yaml(msg.frontiers_remaining, out);
    out << ", ";
  }

  // member: progress_percentage
  {
    out << "progress_percentage: ";
    rosidl_generator_traits::value_to_yaml(msg.progress_percentage, out);
    out << ", ";
  }

  // member: returning_to_init
  {
    out << "returning_to_init: ";
    rosidl_generator_traits::value_to_yaml(msg.returning_to_init, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const ExploreStatus & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: header
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "header:\n";
    to_block_style_yaml(msg.header, out, indentation + 2);
  }

  // member: state
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "state: ";
    rosidl_generator_traits::value_to_yaml(msg.state, out);
    out << "\n";
  }

  // member: state_description
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "state_description: ";
    rosidl_generator_traits::value_to_yaml(msg.state_description, out);
    out << "\n";
  }

  // member: frontiers_explored
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "frontiers_explored: ";
    rosidl_generator_traits::value_to_yaml(msg.frontiers_explored, out);
    out << "\n";
  }

  // member: frontiers_remaining
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "frontiers_remaining: ";
    rosidl_generator_traits::value_to_yaml(msg.frontiers_remaining, out);
    out << "\n";
  }

  // member: progress_percentage
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "progress_percentage: ";
    rosidl_generator_traits::value_to_yaml(msg.progress_percentage, out);
    out << "\n";
  }

  // member: returning_to_init
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "returning_to_init: ";
    rosidl_generator_traits::value_to_yaml(msg.returning_to_init, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const ExploreStatus & msg, bool use_flow_style = false)
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

}  // namespace explore_lite

namespace rosidl_generator_traits
{

[[deprecated("use explore_lite::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const explore_lite::msg::ExploreStatus & msg,
  std::ostream & out, size_t indentation = 0)
{
  explore_lite::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use explore_lite::msg::to_yaml() instead")]]
inline std::string to_yaml(const explore_lite::msg::ExploreStatus & msg)
{
  return explore_lite::msg::to_yaml(msg);
}

template<>
inline const char * data_type<explore_lite::msg::ExploreStatus>()
{
  return "explore_lite::msg::ExploreStatus";
}

template<>
inline const char * name<explore_lite::msg::ExploreStatus>()
{
  return "explore_lite/msg/ExploreStatus";
}

template<>
struct has_fixed_size<explore_lite::msg::ExploreStatus>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<explore_lite::msg::ExploreStatus>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<explore_lite::msg::ExploreStatus>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // EXPLORE_LITE__MSG__DETAIL__EXPLORE_STATUS__TRAITS_HPP_
