// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from autorccar_interfaces:msg/BoundingBoxes.idl
// generated code does not contain a copyright notice

#ifndef AUTORCCAR_INTERFACES__MSG__DETAIL__BOUNDING_BOXES__STRUCT_H_
#define AUTORCCAR_INTERFACES__MSG__DETAIL__BOUNDING_BOXES__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'bounding_boxes'
#include "vision_msgs/msg/detail/bounding_box2_d__struct.h"

/// Struct defined in msg/BoundingBoxes in the package autorccar_interfaces.
typedef struct autorccar_interfaces__msg__BoundingBoxes
{
  int64_t num;
  vision_msgs__msg__BoundingBox2D__Sequence bounding_boxes;
} autorccar_interfaces__msg__BoundingBoxes;

// Struct for a sequence of autorccar_interfaces__msg__BoundingBoxes.
typedef struct autorccar_interfaces__msg__BoundingBoxes__Sequence
{
  autorccar_interfaces__msg__BoundingBoxes * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} autorccar_interfaces__msg__BoundingBoxes__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // AUTORCCAR_INTERFACES__MSG__DETAIL__BOUNDING_BOXES__STRUCT_H_
