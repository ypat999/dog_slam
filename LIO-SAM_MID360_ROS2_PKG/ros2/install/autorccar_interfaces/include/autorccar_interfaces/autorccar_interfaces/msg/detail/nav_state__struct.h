// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from autorccar_interfaces:msg/NavState.idl
// generated code does not contain a copyright notice

#ifndef AUTORCCAR_INTERFACES__MSG__DETAIL__NAV_STATE__STRUCT_H_
#define AUTORCCAR_INTERFACES__MSG__DETAIL__NAV_STATE__STRUCT_H_

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
// Member 'origin'
// Member 'position'
// Member 'velocity'
// Member 'acceleration'
// Member 'angular_velocity'
#include "geometry_msgs/msg/detail/vector3__struct.h"
// Member 'quaternion'
#include "geometry_msgs/msg/detail/quaternion__struct.h"

/// Struct defined in msg/NavState in the package autorccar_interfaces.
typedef struct autorccar_interfaces__msg__NavState
{
  builtin_interfaces__msg__Time timestamp;
  geometry_msgs__msg__Vector3 origin;
  geometry_msgs__msg__Vector3 position;
  geometry_msgs__msg__Vector3 velocity;
  geometry_msgs__msg__Quaternion quaternion;
  geometry_msgs__msg__Vector3 acceleration;
  geometry_msgs__msg__Vector3 angular_velocity;
} autorccar_interfaces__msg__NavState;

// Struct for a sequence of autorccar_interfaces__msg__NavState.
typedef struct autorccar_interfaces__msg__NavState__Sequence
{
  autorccar_interfaces__msg__NavState * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} autorccar_interfaces__msg__NavState__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // AUTORCCAR_INTERFACES__MSG__DETAIL__NAV_STATE__STRUCT_H_
