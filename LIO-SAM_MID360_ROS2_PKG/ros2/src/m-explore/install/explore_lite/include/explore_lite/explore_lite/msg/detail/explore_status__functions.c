// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from explore_lite:msg/ExploreStatus.idl
// generated code does not contain a copyright notice
#include "explore_lite/msg/detail/explore_status__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"
// Member `state_description`
#include "rosidl_runtime_c/string_functions.h"

bool
explore_lite__msg__ExploreStatus__init(explore_lite__msg__ExploreStatus * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    explore_lite__msg__ExploreStatus__fini(msg);
    return false;
  }
  // state
  // state_description
  if (!rosidl_runtime_c__String__init(&msg->state_description)) {
    explore_lite__msg__ExploreStatus__fini(msg);
    return false;
  }
  // frontiers_explored
  // frontiers_remaining
  // progress_percentage
  // returning_to_init
  return true;
}

void
explore_lite__msg__ExploreStatus__fini(explore_lite__msg__ExploreStatus * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // state
  // state_description
  rosidl_runtime_c__String__fini(&msg->state_description);
  // frontiers_explored
  // frontiers_remaining
  // progress_percentage
  // returning_to_init
}

bool
explore_lite__msg__ExploreStatus__are_equal(const explore_lite__msg__ExploreStatus * lhs, const explore_lite__msg__ExploreStatus * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__are_equal(
      &(lhs->header), &(rhs->header)))
  {
    return false;
  }
  // state
  if (lhs->state != rhs->state) {
    return false;
  }
  // state_description
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->state_description), &(rhs->state_description)))
  {
    return false;
  }
  // frontiers_explored
  if (lhs->frontiers_explored != rhs->frontiers_explored) {
    return false;
  }
  // frontiers_remaining
  if (lhs->frontiers_remaining != rhs->frontiers_remaining) {
    return false;
  }
  // progress_percentage
  if (lhs->progress_percentage != rhs->progress_percentage) {
    return false;
  }
  // returning_to_init
  if (lhs->returning_to_init != rhs->returning_to_init) {
    return false;
  }
  return true;
}

bool
explore_lite__msg__ExploreStatus__copy(
  const explore_lite__msg__ExploreStatus * input,
  explore_lite__msg__ExploreStatus * output)
{
  if (!input || !output) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__copy(
      &(input->header), &(output->header)))
  {
    return false;
  }
  // state
  output->state = input->state;
  // state_description
  if (!rosidl_runtime_c__String__copy(
      &(input->state_description), &(output->state_description)))
  {
    return false;
  }
  // frontiers_explored
  output->frontiers_explored = input->frontiers_explored;
  // frontiers_remaining
  output->frontiers_remaining = input->frontiers_remaining;
  // progress_percentage
  output->progress_percentage = input->progress_percentage;
  // returning_to_init
  output->returning_to_init = input->returning_to_init;
  return true;
}

explore_lite__msg__ExploreStatus *
explore_lite__msg__ExploreStatus__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  explore_lite__msg__ExploreStatus * msg = (explore_lite__msg__ExploreStatus *)allocator.allocate(sizeof(explore_lite__msg__ExploreStatus), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(explore_lite__msg__ExploreStatus));
  bool success = explore_lite__msg__ExploreStatus__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
explore_lite__msg__ExploreStatus__destroy(explore_lite__msg__ExploreStatus * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    explore_lite__msg__ExploreStatus__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
explore_lite__msg__ExploreStatus__Sequence__init(explore_lite__msg__ExploreStatus__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  explore_lite__msg__ExploreStatus * data = NULL;

  if (size) {
    data = (explore_lite__msg__ExploreStatus *)allocator.zero_allocate(size, sizeof(explore_lite__msg__ExploreStatus), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = explore_lite__msg__ExploreStatus__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        explore_lite__msg__ExploreStatus__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
explore_lite__msg__ExploreStatus__Sequence__fini(explore_lite__msg__ExploreStatus__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      explore_lite__msg__ExploreStatus__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

explore_lite__msg__ExploreStatus__Sequence *
explore_lite__msg__ExploreStatus__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  explore_lite__msg__ExploreStatus__Sequence * array = (explore_lite__msg__ExploreStatus__Sequence *)allocator.allocate(sizeof(explore_lite__msg__ExploreStatus__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = explore_lite__msg__ExploreStatus__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
explore_lite__msg__ExploreStatus__Sequence__destroy(explore_lite__msg__ExploreStatus__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    explore_lite__msg__ExploreStatus__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
explore_lite__msg__ExploreStatus__Sequence__are_equal(const explore_lite__msg__ExploreStatus__Sequence * lhs, const explore_lite__msg__ExploreStatus__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!explore_lite__msg__ExploreStatus__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
explore_lite__msg__ExploreStatus__Sequence__copy(
  const explore_lite__msg__ExploreStatus__Sequence * input,
  explore_lite__msg__ExploreStatus__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(explore_lite__msg__ExploreStatus);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    explore_lite__msg__ExploreStatus * data =
      (explore_lite__msg__ExploreStatus *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!explore_lite__msg__ExploreStatus__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          explore_lite__msg__ExploreStatus__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!explore_lite__msg__ExploreStatus__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
