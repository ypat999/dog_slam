// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from autorccar_interfaces:msg/BoundingBoxes.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "autorccar_interfaces/msg/detail/bounding_boxes__rosidl_typesupport_introspection_c.h"
#include "autorccar_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "autorccar_interfaces/msg/detail/bounding_boxes__functions.h"
#include "autorccar_interfaces/msg/detail/bounding_boxes__struct.h"


// Include directives for member types
// Member `bounding_boxes`
#include "vision_msgs/msg/bounding_box2_d.h"
// Member `bounding_boxes`
#include "vision_msgs/msg/detail/bounding_box2_d__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void autorccar_interfaces__msg__BoundingBoxes__rosidl_typesupport_introspection_c__BoundingBoxes_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  autorccar_interfaces__msg__BoundingBoxes__init(message_memory);
}

void autorccar_interfaces__msg__BoundingBoxes__rosidl_typesupport_introspection_c__BoundingBoxes_fini_function(void * message_memory)
{
  autorccar_interfaces__msg__BoundingBoxes__fini(message_memory);
}

size_t autorccar_interfaces__msg__BoundingBoxes__rosidl_typesupport_introspection_c__size_function__BoundingBoxes__bounding_boxes(
  const void * untyped_member)
{
  const vision_msgs__msg__BoundingBox2D__Sequence * member =
    (const vision_msgs__msg__BoundingBox2D__Sequence *)(untyped_member);
  return member->size;
}

const void * autorccar_interfaces__msg__BoundingBoxes__rosidl_typesupport_introspection_c__get_const_function__BoundingBoxes__bounding_boxes(
  const void * untyped_member, size_t index)
{
  const vision_msgs__msg__BoundingBox2D__Sequence * member =
    (const vision_msgs__msg__BoundingBox2D__Sequence *)(untyped_member);
  return &member->data[index];
}

void * autorccar_interfaces__msg__BoundingBoxes__rosidl_typesupport_introspection_c__get_function__BoundingBoxes__bounding_boxes(
  void * untyped_member, size_t index)
{
  vision_msgs__msg__BoundingBox2D__Sequence * member =
    (vision_msgs__msg__BoundingBox2D__Sequence *)(untyped_member);
  return &member->data[index];
}

void autorccar_interfaces__msg__BoundingBoxes__rosidl_typesupport_introspection_c__fetch_function__BoundingBoxes__bounding_boxes(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const vision_msgs__msg__BoundingBox2D * item =
    ((const vision_msgs__msg__BoundingBox2D *)
    autorccar_interfaces__msg__BoundingBoxes__rosidl_typesupport_introspection_c__get_const_function__BoundingBoxes__bounding_boxes(untyped_member, index));
  vision_msgs__msg__BoundingBox2D * value =
    (vision_msgs__msg__BoundingBox2D *)(untyped_value);
  *value = *item;
}

void autorccar_interfaces__msg__BoundingBoxes__rosidl_typesupport_introspection_c__assign_function__BoundingBoxes__bounding_boxes(
  void * untyped_member, size_t index, const void * untyped_value)
{
  vision_msgs__msg__BoundingBox2D * item =
    ((vision_msgs__msg__BoundingBox2D *)
    autorccar_interfaces__msg__BoundingBoxes__rosidl_typesupport_introspection_c__get_function__BoundingBoxes__bounding_boxes(untyped_member, index));
  const vision_msgs__msg__BoundingBox2D * value =
    (const vision_msgs__msg__BoundingBox2D *)(untyped_value);
  *item = *value;
}

bool autorccar_interfaces__msg__BoundingBoxes__rosidl_typesupport_introspection_c__resize_function__BoundingBoxes__bounding_boxes(
  void * untyped_member, size_t size)
{
  vision_msgs__msg__BoundingBox2D__Sequence * member =
    (vision_msgs__msg__BoundingBox2D__Sequence *)(untyped_member);
  vision_msgs__msg__BoundingBox2D__Sequence__fini(member);
  return vision_msgs__msg__BoundingBox2D__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember autorccar_interfaces__msg__BoundingBoxes__rosidl_typesupport_introspection_c__BoundingBoxes_message_member_array[2] = {
  {
    "num",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT64,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(autorccar_interfaces__msg__BoundingBoxes, num),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "bounding_boxes",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(autorccar_interfaces__msg__BoundingBoxes, bounding_boxes),  // bytes offset in struct
    NULL,  // default value
    autorccar_interfaces__msg__BoundingBoxes__rosidl_typesupport_introspection_c__size_function__BoundingBoxes__bounding_boxes,  // size() function pointer
    autorccar_interfaces__msg__BoundingBoxes__rosidl_typesupport_introspection_c__get_const_function__BoundingBoxes__bounding_boxes,  // get_const(index) function pointer
    autorccar_interfaces__msg__BoundingBoxes__rosidl_typesupport_introspection_c__get_function__BoundingBoxes__bounding_boxes,  // get(index) function pointer
    autorccar_interfaces__msg__BoundingBoxes__rosidl_typesupport_introspection_c__fetch_function__BoundingBoxes__bounding_boxes,  // fetch(index, &value) function pointer
    autorccar_interfaces__msg__BoundingBoxes__rosidl_typesupport_introspection_c__assign_function__BoundingBoxes__bounding_boxes,  // assign(index, value) function pointer
    autorccar_interfaces__msg__BoundingBoxes__rosidl_typesupport_introspection_c__resize_function__BoundingBoxes__bounding_boxes  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers autorccar_interfaces__msg__BoundingBoxes__rosidl_typesupport_introspection_c__BoundingBoxes_message_members = {
  "autorccar_interfaces__msg",  // message namespace
  "BoundingBoxes",  // message name
  2,  // number of fields
  sizeof(autorccar_interfaces__msg__BoundingBoxes),
  autorccar_interfaces__msg__BoundingBoxes__rosidl_typesupport_introspection_c__BoundingBoxes_message_member_array,  // message members
  autorccar_interfaces__msg__BoundingBoxes__rosidl_typesupport_introspection_c__BoundingBoxes_init_function,  // function to initialize message memory (memory has to be allocated)
  autorccar_interfaces__msg__BoundingBoxes__rosidl_typesupport_introspection_c__BoundingBoxes_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t autorccar_interfaces__msg__BoundingBoxes__rosidl_typesupport_introspection_c__BoundingBoxes_message_type_support_handle = {
  0,
  &autorccar_interfaces__msg__BoundingBoxes__rosidl_typesupport_introspection_c__BoundingBoxes_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_autorccar_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, autorccar_interfaces, msg, BoundingBoxes)() {
  autorccar_interfaces__msg__BoundingBoxes__rosidl_typesupport_introspection_c__BoundingBoxes_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, vision_msgs, msg, BoundingBox2D)();
  if (!autorccar_interfaces__msg__BoundingBoxes__rosidl_typesupport_introspection_c__BoundingBoxes_message_type_support_handle.typesupport_identifier) {
    autorccar_interfaces__msg__BoundingBoxes__rosidl_typesupport_introspection_c__BoundingBoxes_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &autorccar_interfaces__msg__BoundingBoxes__rosidl_typesupport_introspection_c__BoundingBoxes_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
