// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from autorccar_interfaces:msg/PathPoint.idl
// generated code does not contain a copyright notice

#ifndef AUTORCCAR_INTERFACES__MSG__DETAIL__PATH_POINT__STRUCT_H_
#define AUTORCCAR_INTERFACES__MSG__DETAIL__PATH_POINT__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/PathPoint in the package autorccar_interfaces.
typedef struct autorccar_interfaces__msg__PathPoint
{
  double x;
  double y;
  double speed;
} autorccar_interfaces__msg__PathPoint;

// Struct for a sequence of autorccar_interfaces__msg__PathPoint.
typedef struct autorccar_interfaces__msg__PathPoint__Sequence
{
  autorccar_interfaces__msg__PathPoint * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} autorccar_interfaces__msg__PathPoint__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // AUTORCCAR_INTERFACES__MSG__DETAIL__PATH_POINT__STRUCT_H_
