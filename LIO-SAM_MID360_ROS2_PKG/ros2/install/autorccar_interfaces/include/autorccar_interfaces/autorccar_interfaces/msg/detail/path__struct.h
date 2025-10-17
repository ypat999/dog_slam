// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from autorccar_interfaces:msg/Path.idl
// generated code does not contain a copyright notice

#ifndef AUTORCCAR_INTERFACES__MSG__DETAIL__PATH__STRUCT_H_
#define AUTORCCAR_INTERFACES__MSG__DETAIL__PATH__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'path_points'
#include "autorccar_interfaces/msg/detail/path_point__struct.h"

/// Struct defined in msg/Path in the package autorccar_interfaces.
typedef struct autorccar_interfaces__msg__Path
{
  autorccar_interfaces__msg__PathPoint__Sequence path_points;
} autorccar_interfaces__msg__Path;

// Struct for a sequence of autorccar_interfaces__msg__Path.
typedef struct autorccar_interfaces__msg__Path__Sequence
{
  autorccar_interfaces__msg__Path * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} autorccar_interfaces__msg__Path__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // AUTORCCAR_INTERFACES__MSG__DETAIL__PATH__STRUCT_H_
