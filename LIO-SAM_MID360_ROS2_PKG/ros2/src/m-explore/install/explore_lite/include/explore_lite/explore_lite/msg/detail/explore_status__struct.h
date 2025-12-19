// NOLINT: This file starts with a BOM since it contain non-ASCII characters
// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from explore_lite:msg/ExploreStatus.idl
// generated code does not contain a copyright notice

#ifndef EXPLORE_LITE__MSG__DETAIL__EXPLORE_STATUS__STRUCT_H_
#define EXPLORE_LITE__MSG__DETAIL__EXPLORE_STATUS__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Constant 'STATE_STARTING'.
/**
  * 探索状态枚举
  * 启动中
 */
enum
{
  explore_lite__msg__ExploreStatus__STATE_STARTING = 0
};

/// Constant 'STATE_EXPLORING'.
/**
  * 探索中
 */
enum
{
  explore_lite__msg__ExploreStatus__STATE_EXPLORING = 1
};

/// Constant 'STATE_COMPLETED'.
/**
  * 探索完成
 */
enum
{
  explore_lite__msg__ExploreStatus__STATE_COMPLETED = 2
};

/// Constant 'STATE_STOPPED'.
/**
  * 已停止
 */
enum
{
  explore_lite__msg__ExploreStatus__STATE_STOPPED = 3
};

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.h"
// Member 'state_description'
#include "rosidl_runtime_c/string.h"

/// Struct defined in msg/ExploreStatus in the package explore_lite.
/**
  * 探索状态消息定义
 */
typedef struct explore_lite__msg__ExploreStatus
{
  /// 消息头
  std_msgs__msg__Header header;
  /// 当前状态
  uint8_t state;
  /// 状态描述
  rosidl_runtime_c__String state_description;
  /// 已探索的前沿数量
  uint32_t frontiers_explored;
  /// 剩余的前沿数量
  uint32_t frontiers_remaining;
  /// 探索进度百分比 (0-100)
  float progress_percentage;
  /// 是否正在返回初始位置
  bool returning_to_init;
} explore_lite__msg__ExploreStatus;

// Struct for a sequence of explore_lite__msg__ExploreStatus.
typedef struct explore_lite__msg__ExploreStatus__Sequence
{
  explore_lite__msg__ExploreStatus * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} explore_lite__msg__ExploreStatus__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // EXPLORE_LITE__MSG__DETAIL__EXPLORE_STATUS__STRUCT_H_
