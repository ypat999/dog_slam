// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from autorccar_interfaces:msg/ControlCommand.idl
// generated code does not contain a copyright notice

#ifndef AUTORCCAR_INTERFACES__MSG__DETAIL__CONTROL_COMMAND__FUNCTIONS_H_
#define AUTORCCAR_INTERFACES__MSG__DETAIL__CONTROL_COMMAND__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "autorccar_interfaces/msg/rosidl_generator_c__visibility_control.h"

#include "autorccar_interfaces/msg/detail/control_command__struct.h"

/// Initialize msg/ControlCommand message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * autorccar_interfaces__msg__ControlCommand
 * )) before or use
 * autorccar_interfaces__msg__ControlCommand__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_autorccar_interfaces
bool
autorccar_interfaces__msg__ControlCommand__init(autorccar_interfaces__msg__ControlCommand * msg);

/// Finalize msg/ControlCommand message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_autorccar_interfaces
void
autorccar_interfaces__msg__ControlCommand__fini(autorccar_interfaces__msg__ControlCommand * msg);

/// Create msg/ControlCommand message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * autorccar_interfaces__msg__ControlCommand__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_autorccar_interfaces
autorccar_interfaces__msg__ControlCommand *
autorccar_interfaces__msg__ControlCommand__create();

/// Destroy msg/ControlCommand message.
/**
 * It calls
 * autorccar_interfaces__msg__ControlCommand__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_autorccar_interfaces
void
autorccar_interfaces__msg__ControlCommand__destroy(autorccar_interfaces__msg__ControlCommand * msg);

/// Check for msg/ControlCommand message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_autorccar_interfaces
bool
autorccar_interfaces__msg__ControlCommand__are_equal(const autorccar_interfaces__msg__ControlCommand * lhs, const autorccar_interfaces__msg__ControlCommand * rhs);

/// Copy a msg/ControlCommand message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_autorccar_interfaces
bool
autorccar_interfaces__msg__ControlCommand__copy(
  const autorccar_interfaces__msg__ControlCommand * input,
  autorccar_interfaces__msg__ControlCommand * output);

/// Initialize array of msg/ControlCommand messages.
/**
 * It allocates the memory for the number of elements and calls
 * autorccar_interfaces__msg__ControlCommand__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_autorccar_interfaces
bool
autorccar_interfaces__msg__ControlCommand__Sequence__init(autorccar_interfaces__msg__ControlCommand__Sequence * array, size_t size);

/// Finalize array of msg/ControlCommand messages.
/**
 * It calls
 * autorccar_interfaces__msg__ControlCommand__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_autorccar_interfaces
void
autorccar_interfaces__msg__ControlCommand__Sequence__fini(autorccar_interfaces__msg__ControlCommand__Sequence * array);

/// Create array of msg/ControlCommand messages.
/**
 * It allocates the memory for the array and calls
 * autorccar_interfaces__msg__ControlCommand__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_autorccar_interfaces
autorccar_interfaces__msg__ControlCommand__Sequence *
autorccar_interfaces__msg__ControlCommand__Sequence__create(size_t size);

/// Destroy array of msg/ControlCommand messages.
/**
 * It calls
 * autorccar_interfaces__msg__ControlCommand__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_autorccar_interfaces
void
autorccar_interfaces__msg__ControlCommand__Sequence__destroy(autorccar_interfaces__msg__ControlCommand__Sequence * array);

/// Check for msg/ControlCommand message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_autorccar_interfaces
bool
autorccar_interfaces__msg__ControlCommand__Sequence__are_equal(const autorccar_interfaces__msg__ControlCommand__Sequence * lhs, const autorccar_interfaces__msg__ControlCommand__Sequence * rhs);

/// Copy an array of msg/ControlCommand messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_autorccar_interfaces
bool
autorccar_interfaces__msg__ControlCommand__Sequence__copy(
  const autorccar_interfaces__msg__ControlCommand__Sequence * input,
  autorccar_interfaces__msg__ControlCommand__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // AUTORCCAR_INTERFACES__MSG__DETAIL__CONTROL_COMMAND__FUNCTIONS_H_
