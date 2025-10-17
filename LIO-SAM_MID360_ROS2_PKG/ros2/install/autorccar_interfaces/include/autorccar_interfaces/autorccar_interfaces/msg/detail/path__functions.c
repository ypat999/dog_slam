// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from autorccar_interfaces:msg/Path.idl
// generated code does not contain a copyright notice
#include "autorccar_interfaces/msg/detail/path__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `path_points`
#include "autorccar_interfaces/msg/detail/path_point__functions.h"

bool
autorccar_interfaces__msg__Path__init(autorccar_interfaces__msg__Path * msg)
{
  if (!msg) {
    return false;
  }
  // path_points
  if (!autorccar_interfaces__msg__PathPoint__Sequence__init(&msg->path_points, 0)) {
    autorccar_interfaces__msg__Path__fini(msg);
    return false;
  }
  return true;
}

void
autorccar_interfaces__msg__Path__fini(autorccar_interfaces__msg__Path * msg)
{
  if (!msg) {
    return;
  }
  // path_points
  autorccar_interfaces__msg__PathPoint__Sequence__fini(&msg->path_points);
}

bool
autorccar_interfaces__msg__Path__are_equal(const autorccar_interfaces__msg__Path * lhs, const autorccar_interfaces__msg__Path * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // path_points
  if (!autorccar_interfaces__msg__PathPoint__Sequence__are_equal(
      &(lhs->path_points), &(rhs->path_points)))
  {
    return false;
  }
  return true;
}

bool
autorccar_interfaces__msg__Path__copy(
  const autorccar_interfaces__msg__Path * input,
  autorccar_interfaces__msg__Path * output)
{
  if (!input || !output) {
    return false;
  }
  // path_points
  if (!autorccar_interfaces__msg__PathPoint__Sequence__copy(
      &(input->path_points), &(output->path_points)))
  {
    return false;
  }
  return true;
}

autorccar_interfaces__msg__Path *
autorccar_interfaces__msg__Path__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  autorccar_interfaces__msg__Path * msg = (autorccar_interfaces__msg__Path *)allocator.allocate(sizeof(autorccar_interfaces__msg__Path), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(autorccar_interfaces__msg__Path));
  bool success = autorccar_interfaces__msg__Path__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
autorccar_interfaces__msg__Path__destroy(autorccar_interfaces__msg__Path * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    autorccar_interfaces__msg__Path__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
autorccar_interfaces__msg__Path__Sequence__init(autorccar_interfaces__msg__Path__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  autorccar_interfaces__msg__Path * data = NULL;

  if (size) {
    data = (autorccar_interfaces__msg__Path *)allocator.zero_allocate(size, sizeof(autorccar_interfaces__msg__Path), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = autorccar_interfaces__msg__Path__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        autorccar_interfaces__msg__Path__fini(&data[i - 1]);
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
autorccar_interfaces__msg__Path__Sequence__fini(autorccar_interfaces__msg__Path__Sequence * array)
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
      autorccar_interfaces__msg__Path__fini(&array->data[i]);
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

autorccar_interfaces__msg__Path__Sequence *
autorccar_interfaces__msg__Path__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  autorccar_interfaces__msg__Path__Sequence * array = (autorccar_interfaces__msg__Path__Sequence *)allocator.allocate(sizeof(autorccar_interfaces__msg__Path__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = autorccar_interfaces__msg__Path__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
autorccar_interfaces__msg__Path__Sequence__destroy(autorccar_interfaces__msg__Path__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    autorccar_interfaces__msg__Path__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
autorccar_interfaces__msg__Path__Sequence__are_equal(const autorccar_interfaces__msg__Path__Sequence * lhs, const autorccar_interfaces__msg__Path__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!autorccar_interfaces__msg__Path__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
autorccar_interfaces__msg__Path__Sequence__copy(
  const autorccar_interfaces__msg__Path__Sequence * input,
  autorccar_interfaces__msg__Path__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(autorccar_interfaces__msg__Path);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    autorccar_interfaces__msg__Path * data =
      (autorccar_interfaces__msg__Path *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!autorccar_interfaces__msg__Path__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          autorccar_interfaces__msg__Path__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!autorccar_interfaces__msg__Path__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
