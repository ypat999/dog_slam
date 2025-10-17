// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from autorccar_interfaces:msg/Path.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "autorccar_interfaces/msg/detail/path__rosidl_typesupport_introspection_c.h"
#include "autorccar_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "autorccar_interfaces/msg/detail/path__functions.h"
#include "autorccar_interfaces/msg/detail/path__struct.h"


// Include directives for member types
// Member `path_points`
#include "autorccar_interfaces/msg/path_point.h"
// Member `path_points`
#include "autorccar_interfaces/msg/detail/path_point__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void autorccar_interfaces__msg__Path__rosidl_typesupport_introspection_c__Path_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  autorccar_interfaces__msg__Path__init(message_memory);
}

void autorccar_interfaces__msg__Path__rosidl_typesupport_introspection_c__Path_fini_function(void * message_memory)
{
  autorccar_interfaces__msg__Path__fini(message_memory);
}

size_t autorccar_interfaces__msg__Path__rosidl_typesupport_introspection_c__size_function__Path__path_points(
  const void * untyped_member)
{
  const autorccar_interfaces__msg__PathPoint__Sequence * member =
    (const autorccar_interfaces__msg__PathPoint__Sequence *)(untyped_member);
  return member->size;
}

const void * autorccar_interfaces__msg__Path__rosidl_typesupport_introspection_c__get_const_function__Path__path_points(
  const void * untyped_member, size_t index)
{
  const autorccar_interfaces__msg__PathPoint__Sequence * member =
    (const autorccar_interfaces__msg__PathPoint__Sequence *)(untyped_member);
  return &member->data[index];
}

void * autorccar_interfaces__msg__Path__rosidl_typesupport_introspection_c__get_function__Path__path_points(
  void * untyped_member, size_t index)
{
  autorccar_interfaces__msg__PathPoint__Sequence * member =
    (autorccar_interfaces__msg__PathPoint__Sequence *)(untyped_member);
  return &member->data[index];
}

void autorccar_interfaces__msg__Path__rosidl_typesupport_introspection_c__fetch_function__Path__path_points(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const autorccar_interfaces__msg__PathPoint * item =
    ((const autorccar_interfaces__msg__PathPoint *)
    autorccar_interfaces__msg__Path__rosidl_typesupport_introspection_c__get_const_function__Path__path_points(untyped_member, index));
  autorccar_interfaces__msg__PathPoint * value =
    (autorccar_interfaces__msg__PathPoint *)(untyped_value);
  *value = *item;
}

void autorccar_interfaces__msg__Path__rosidl_typesupport_introspection_c__assign_function__Path__path_points(
  void * untyped_member, size_t index, const void * untyped_value)
{
  autorccar_interfaces__msg__PathPoint * item =
    ((autorccar_interfaces__msg__PathPoint *)
    autorccar_interfaces__msg__Path__rosidl_typesupport_introspection_c__get_function__Path__path_points(untyped_member, index));
  const autorccar_interfaces__msg__PathPoint * value =
    (const autorccar_interfaces__msg__PathPoint *)(untyped_value);
  *item = *value;
}

bool autorccar_interfaces__msg__Path__rosidl_typesupport_introspection_c__resize_function__Path__path_points(
  void * untyped_member, size_t size)
{
  autorccar_interfaces__msg__PathPoint__Sequence * member =
    (autorccar_interfaces__msg__PathPoint__Sequence *)(untyped_member);
  autorccar_interfaces__msg__PathPoint__Sequence__fini(member);
  return autorccar_interfaces__msg__PathPoint__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember autorccar_interfaces__msg__Path__rosidl_typesupport_introspection_c__Path_message_member_array[1] = {
  {
    "path_points",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(autorccar_interfaces__msg__Path, path_points),  // bytes offset in struct
    NULL,  // default value
    autorccar_interfaces__msg__Path__rosidl_typesupport_introspection_c__size_function__Path__path_points,  // size() function pointer
    autorccar_interfaces__msg__Path__rosidl_typesupport_introspection_c__get_const_function__Path__path_points,  // get_const(index) function pointer
    autorccar_interfaces__msg__Path__rosidl_typesupport_introspection_c__get_function__Path__path_points,  // get(index) function pointer
    autorccar_interfaces__msg__Path__rosidl_typesupport_introspection_c__fetch_function__Path__path_points,  // fetch(index, &value) function pointer
    autorccar_interfaces__msg__Path__rosidl_typesupport_introspection_c__assign_function__Path__path_points,  // assign(index, value) function pointer
    autorccar_interfaces__msg__Path__rosidl_typesupport_introspection_c__resize_function__Path__path_points  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers autorccar_interfaces__msg__Path__rosidl_typesupport_introspection_c__Path_message_members = {
  "autorccar_interfaces__msg",  // message namespace
  "Path",  // message name
  1,  // number of fields
  sizeof(autorccar_interfaces__msg__Path),
  autorccar_interfaces__msg__Path__rosidl_typesupport_introspection_c__Path_message_member_array,  // message members
  autorccar_interfaces__msg__Path__rosidl_typesupport_introspection_c__Path_init_function,  // function to initialize message memory (memory has to be allocated)
  autorccar_interfaces__msg__Path__rosidl_typesupport_introspection_c__Path_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t autorccar_interfaces__msg__Path__rosidl_typesupport_introspection_c__Path_message_type_support_handle = {
  0,
  &autorccar_interfaces__msg__Path__rosidl_typesupport_introspection_c__Path_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_autorccar_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, autorccar_interfaces, msg, Path)() {
  autorccar_interfaces__msg__Path__rosidl_typesupport_introspection_c__Path_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, autorccar_interfaces, msg, PathPoint)();
  if (!autorccar_interfaces__msg__Path__rosidl_typesupport_introspection_c__Path_message_type_support_handle.typesupport_identifier) {
    autorccar_interfaces__msg__Path__rosidl_typesupport_introspection_c__Path_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &autorccar_interfaces__msg__Path__rosidl_typesupport_introspection_c__Path_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
