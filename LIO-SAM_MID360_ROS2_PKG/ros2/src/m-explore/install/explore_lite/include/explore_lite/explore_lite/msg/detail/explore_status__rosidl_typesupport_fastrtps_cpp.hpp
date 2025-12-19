// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__rosidl_typesupport_fastrtps_cpp.hpp.em
// with input from explore_lite:msg/ExploreStatus.idl
// generated code does not contain a copyright notice

#ifndef EXPLORE_LITE__MSG__DETAIL__EXPLORE_STATUS__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_
#define EXPLORE_LITE__MSG__DETAIL__EXPLORE_STATUS__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_

#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_interface/macros.h"
#include "explore_lite/msg/rosidl_typesupport_fastrtps_cpp__visibility_control.h"
#include "explore_lite/msg/detail/explore_status__struct.hpp"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

#include "fastcdr/Cdr.h"

namespace explore_lite
{

namespace msg
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_explore_lite
cdr_serialize(
  const explore_lite::msg::ExploreStatus & ros_message,
  eprosima::fastcdr::Cdr & cdr);

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_explore_lite
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  explore_lite::msg::ExploreStatus & ros_message);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_explore_lite
get_serialized_size(
  const explore_lite::msg::ExploreStatus & ros_message,
  size_t current_alignment);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_explore_lite
max_serialized_size_ExploreStatus(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

}  // namespace typesupport_fastrtps_cpp

}  // namespace msg

}  // namespace explore_lite

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_explore_lite
const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, explore_lite, msg, ExploreStatus)();

#ifdef __cplusplus
}
#endif

#endif  // EXPLORE_LITE__MSG__DETAIL__EXPLORE_STATUS__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_
