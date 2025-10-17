// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from autorccar_interfaces:msg/Gnss.idl
// generated code does not contain a copyright notice

#ifndef AUTORCCAR_INTERFACES__MSG__DETAIL__GNSS__STRUCT_H_
#define AUTORCCAR_INTERFACES__MSG__DETAIL__GNSS__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'timestamp'
#include "builtin_interfaces/msg/detail/time__struct.h"
// Member 'position_ecef'
// Member 'velocity_ecef'
#include "geometry_msgs/msg/detail/vector3__struct.h"

/// Struct defined in msg/Gnss in the package autorccar_interfaces.
typedef struct autorccar_interfaces__msg__Gnss
{
  builtin_interfaces__msg__Time timestamp;
  geometry_msgs__msg__Vector3 position_ecef;
  geometry_msgs__msg__Vector3 velocity_ecef;
} autorccar_interfaces__msg__Gnss;

// Struct for a sequence of autorccar_interfaces__msg__Gnss.
typedef struct autorccar_interfaces__msg__Gnss__Sequence
{
  autorccar_interfaces__msg__Gnss * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} autorccar_interfaces__msg__Gnss__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // AUTORCCAR_INTERFACES__MSG__DETAIL__GNSS__STRUCT_H_
