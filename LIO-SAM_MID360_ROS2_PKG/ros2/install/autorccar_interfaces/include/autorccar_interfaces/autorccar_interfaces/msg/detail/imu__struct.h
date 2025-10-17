// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from autorccar_interfaces:msg/Imu.idl
// generated code does not contain a copyright notice

#ifndef AUTORCCAR_INTERFACES__MSG__DETAIL__IMU__STRUCT_H_
#define AUTORCCAR_INTERFACES__MSG__DETAIL__IMU__STRUCT_H_

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
// Member 'angular_velocity'
// Member 'linear_acceleration'
#include "geometry_msgs/msg/detail/vector3__struct.h"

/// Struct defined in msg/Imu in the package autorccar_interfaces.
typedef struct autorccar_interfaces__msg__Imu
{
  builtin_interfaces__msg__Time timestamp;
  geometry_msgs__msg__Vector3 angular_velocity;
  geometry_msgs__msg__Vector3 linear_acceleration;
} autorccar_interfaces__msg__Imu;

// Struct for a sequence of autorccar_interfaces__msg__Imu.
typedef struct autorccar_interfaces__msg__Imu__Sequence
{
  autorccar_interfaces__msg__Imu * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} autorccar_interfaces__msg__Imu__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // AUTORCCAR_INTERFACES__MSG__DETAIL__IMU__STRUCT_H_
