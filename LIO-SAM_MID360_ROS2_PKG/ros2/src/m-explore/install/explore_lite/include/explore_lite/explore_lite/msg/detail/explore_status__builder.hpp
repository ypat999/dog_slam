// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from explore_lite:msg/ExploreStatus.idl
// generated code does not contain a copyright notice

#ifndef EXPLORE_LITE__MSG__DETAIL__EXPLORE_STATUS__BUILDER_HPP_
#define EXPLORE_LITE__MSG__DETAIL__EXPLORE_STATUS__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "explore_lite/msg/detail/explore_status__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace explore_lite
{

namespace msg
{

namespace builder
{

class Init_ExploreStatus_returning_to_init
{
public:
  explicit Init_ExploreStatus_returning_to_init(::explore_lite::msg::ExploreStatus & msg)
  : msg_(msg)
  {}
  ::explore_lite::msg::ExploreStatus returning_to_init(::explore_lite::msg::ExploreStatus::_returning_to_init_type arg)
  {
    msg_.returning_to_init = std::move(arg);
    return std::move(msg_);
  }

private:
  ::explore_lite::msg::ExploreStatus msg_;
};

class Init_ExploreStatus_progress_percentage
{
public:
  explicit Init_ExploreStatus_progress_percentage(::explore_lite::msg::ExploreStatus & msg)
  : msg_(msg)
  {}
  Init_ExploreStatus_returning_to_init progress_percentage(::explore_lite::msg::ExploreStatus::_progress_percentage_type arg)
  {
    msg_.progress_percentage = std::move(arg);
    return Init_ExploreStatus_returning_to_init(msg_);
  }

private:
  ::explore_lite::msg::ExploreStatus msg_;
};

class Init_ExploreStatus_frontiers_remaining
{
public:
  explicit Init_ExploreStatus_frontiers_remaining(::explore_lite::msg::ExploreStatus & msg)
  : msg_(msg)
  {}
  Init_ExploreStatus_progress_percentage frontiers_remaining(::explore_lite::msg::ExploreStatus::_frontiers_remaining_type arg)
  {
    msg_.frontiers_remaining = std::move(arg);
    return Init_ExploreStatus_progress_percentage(msg_);
  }

private:
  ::explore_lite::msg::ExploreStatus msg_;
};

class Init_ExploreStatus_frontiers_explored
{
public:
  explicit Init_ExploreStatus_frontiers_explored(::explore_lite::msg::ExploreStatus & msg)
  : msg_(msg)
  {}
  Init_ExploreStatus_frontiers_remaining frontiers_explored(::explore_lite::msg::ExploreStatus::_frontiers_explored_type arg)
  {
    msg_.frontiers_explored = std::move(arg);
    return Init_ExploreStatus_frontiers_remaining(msg_);
  }

private:
  ::explore_lite::msg::ExploreStatus msg_;
};

class Init_ExploreStatus_state_description
{
public:
  explicit Init_ExploreStatus_state_description(::explore_lite::msg::ExploreStatus & msg)
  : msg_(msg)
  {}
  Init_ExploreStatus_frontiers_explored state_description(::explore_lite::msg::ExploreStatus::_state_description_type arg)
  {
    msg_.state_description = std::move(arg);
    return Init_ExploreStatus_frontiers_explored(msg_);
  }

private:
  ::explore_lite::msg::ExploreStatus msg_;
};

class Init_ExploreStatus_state
{
public:
  explicit Init_ExploreStatus_state(::explore_lite::msg::ExploreStatus & msg)
  : msg_(msg)
  {}
  Init_ExploreStatus_state_description state(::explore_lite::msg::ExploreStatus::_state_type arg)
  {
    msg_.state = std::move(arg);
    return Init_ExploreStatus_state_description(msg_);
  }

private:
  ::explore_lite::msg::ExploreStatus msg_;
};

class Init_ExploreStatus_header
{
public:
  Init_ExploreStatus_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ExploreStatus_state header(::explore_lite::msg::ExploreStatus::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_ExploreStatus_state(msg_);
  }

private:
  ::explore_lite::msg::ExploreStatus msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::explore_lite::msg::ExploreStatus>()
{
  return explore_lite::msg::builder::Init_ExploreStatus_header();
}

}  // namespace explore_lite

#endif  // EXPLORE_LITE__MSG__DETAIL__EXPLORE_STATUS__BUILDER_HPP_
