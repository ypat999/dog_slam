// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from explore_lite:msg/ExploreStatus.idl
// generated code does not contain a copyright notice

#ifndef EXPLORE_LITE__MSG__DETAIL__EXPLORE_STATUS__FUNCTIONS_H_
#define EXPLORE_LITE__MSG__DETAIL__EXPLORE_STATUS__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "explore_lite/msg/rosidl_generator_c__visibility_control.h"

#include "explore_lite/msg/detail/explore_status__struct.h"

/// Initialize msg/ExploreStatus message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * explore_lite__msg__ExploreStatus
 * )) before or use
 * explore_lite__msg__ExploreStatus__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_explore_lite
bool
explore_lite__msg__ExploreStatus__init(explore_lite__msg__ExploreStatus * msg);

/// Finalize msg/ExploreStatus message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_explore_lite
void
explore_lite__msg__ExploreStatus__fini(explore_lite__msg__ExploreStatus * msg);

/// Create msg/ExploreStatus message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * explore_lite__msg__ExploreStatus__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_explore_lite
explore_lite__msg__ExploreStatus *
explore_lite__msg__ExploreStatus__create();

/// Destroy msg/ExploreStatus message.
/**
 * It calls
 * explore_lite__msg__ExploreStatus__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_explore_lite
void
explore_lite__msg__ExploreStatus__destroy(explore_lite__msg__ExploreStatus * msg);

/// Check for msg/ExploreStatus message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_explore_lite
bool
explore_lite__msg__ExploreStatus__are_equal(const explore_lite__msg__ExploreStatus * lhs, const explore_lite__msg__ExploreStatus * rhs);

/// Copy a msg/ExploreStatus message.
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
ROSIDL_GENERATOR_C_PUBLIC_explore_lite
bool
explore_lite__msg__ExploreStatus__copy(
  const explore_lite__msg__ExploreStatus * input,
  explore_lite__msg__ExploreStatus * output);

/// Initialize array of msg/ExploreStatus messages.
/**
 * It allocates the memory for the number of elements and calls
 * explore_lite__msg__ExploreStatus__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_explore_lite
bool
explore_lite__msg__ExploreStatus__Sequence__init(explore_lite__msg__ExploreStatus__Sequence * array, size_t size);

/// Finalize array of msg/ExploreStatus messages.
/**
 * It calls
 * explore_lite__msg__ExploreStatus__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_explore_lite
void
explore_lite__msg__ExploreStatus__Sequence__fini(explore_lite__msg__ExploreStatus__Sequence * array);

/// Create array of msg/ExploreStatus messages.
/**
 * It allocates the memory for the array and calls
 * explore_lite__msg__ExploreStatus__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_explore_lite
explore_lite__msg__ExploreStatus__Sequence *
explore_lite__msg__ExploreStatus__Sequence__create(size_t size);

/// Destroy array of msg/ExploreStatus messages.
/**
 * It calls
 * explore_lite__msg__ExploreStatus__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_explore_lite
void
explore_lite__msg__ExploreStatus__Sequence__destroy(explore_lite__msg__ExploreStatus__Sequence * array);

/// Check for msg/ExploreStatus message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_explore_lite
bool
explore_lite__msg__ExploreStatus__Sequence__are_equal(const explore_lite__msg__ExploreStatus__Sequence * lhs, const explore_lite__msg__ExploreStatus__Sequence * rhs);

/// Copy an array of msg/ExploreStatus messages.
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
ROSIDL_GENERATOR_C_PUBLIC_explore_lite
bool
explore_lite__msg__ExploreStatus__Sequence__copy(
  const explore_lite__msg__ExploreStatus__Sequence * input,
  explore_lite__msg__ExploreStatus__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // EXPLORE_LITE__MSG__DETAIL__EXPLORE_STATUS__FUNCTIONS_H_
