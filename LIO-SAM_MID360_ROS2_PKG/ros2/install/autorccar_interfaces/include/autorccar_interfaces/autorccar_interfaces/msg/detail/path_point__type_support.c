// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from autorccar_interfaces:msg/PathPoint.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "autorccar_interfaces/msg/detail/path_point__rosidl_typesupport_introspection_c.h"
#include "autorccar_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "autorccar_interfaces/msg/detail/path_point__functions.h"
#include "autorccar_interfaces/msg/detail/path_point__struct.h"


#ifdef __cplusplus
extern "C"
{
#endif

void autorccar_interfaces__msg__PathPoint__rosidl_typesupport_introspection_c__PathPoint_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  autorccar_interfaces__msg__PathPoint__init(message_memory);
}

void autorccar_interfaces__msg__PathPoint__rosidl_typesupport_introspection_c__PathPoint_fini_function(void * message_memory)
{
  autorccar_interfaces__msg__PathPoint__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember autorccar_interfaces__msg__PathPoint__rosidl_typesupport_introspection_c__PathPoint_message_member_array[3] = {
  {
    "x",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(autorccar_interfaces__msg__PathPoint, x),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "y",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(autorccar_interfaces__msg__PathPoint, y),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "speed",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(autorccar_interfaces__msg__PathPoint, speed),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers autorccar_interfaces__msg__PathPoint__rosidl_typesupport_introspection_c__PathPoint_message_members = {
  "autorccar_interfaces__msg",  // message namespace
  "PathPoint",  // message name
  3,  // number of fields
  sizeof(autorccar_interfaces__msg__PathPoint),
  autorccar_interfaces__msg__PathPoint__rosidl_typesupport_introspection_c__PathPoint_message_member_array,  // message members
  autorccar_interfaces__msg__PathPoint__rosidl_typesupport_introspection_c__PathPoint_init_function,  // function to initialize message memory (memory has to be allocated)
  autorccar_interfaces__msg__PathPoint__rosidl_typesupport_introspection_c__PathPoint_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t autorccar_interfaces__msg__PathPoint__rosidl_typesupport_introspection_c__PathPoint_message_type_support_handle = {
  0,
  &autorccar_interfaces__msg__PathPoint__rosidl_typesupport_introspection_c__PathPoint_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_autorccar_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, autorccar_interfaces, msg, PathPoint)() {
  if (!autorccar_interfaces__msg__PathPoint__rosidl_typesupport_introspection_c__PathPoint_message_type_support_handle.typesupport_identifier) {
    autorccar_interfaces__msg__PathPoint__rosidl_typesupport_introspection_c__PathPoint_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &autorccar_interfaces__msg__PathPoint__rosidl_typesupport_introspection_c__PathPoint_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
