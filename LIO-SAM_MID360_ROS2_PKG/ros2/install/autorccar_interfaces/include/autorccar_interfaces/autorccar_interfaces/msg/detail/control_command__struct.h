// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from autorccar_interfaces:msg/ControlCommand.idl
// generated code does not contain a copyright notice

#ifndef AUTORCCAR_INTERFACES__MSG__DETAIL__CONTROL_COMMAND__STRUCT_H_
#define AUTORCCAR_INTERFACES__MSG__DETAIL__CONTROL_COMMAND__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/ControlCommand in the package autorccar_interfaces.
typedef struct autorccar_interfaces__msg__ControlCommand
{
  double speed;
  double steering_angle;
} autorccar_interfaces__msg__ControlCommand;

// Struct for a sequence of autorccar_interfaces__msg__ControlCommand.
typedef struct autorccar_interfaces__msg__ControlCommand__Sequence
{
  autorccar_interfaces__msg__ControlCommand * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} autorccar_interfaces__msg__ControlCommand__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // AUTORCCAR_INTERFACES__MSG__DETAIL__CONTROL_COMMAND__STRUCT_H_
