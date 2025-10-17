// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from autorccar_interfaces:msg/PathPoint.idl
// generated code does not contain a copyright notice
#include "autorccar_interfaces/msg/detail/path_point__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
autorccar_interfaces__msg__PathPoint__init(autorccar_interfaces__msg__PathPoint * msg)
{
  if (!msg) {
    return false;
  }
  // x
  // y
  // speed
  return true;
}

void
autorccar_interfaces__msg__PathPoint__fini(autorccar_interfaces__msg__PathPoint * msg)
{
  if (!msg) {
    return;
  }
  // x
  // y
  // speed
}

bool
autorccar_interfaces__msg__PathPoint__are_equal(const autorccar_interfaces__msg__PathPoint * lhs, const autorccar_interfaces__msg__PathPoint * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // x
  if (lhs->x != rhs->x) {
    return false;
  }
  // y
  if (lhs->y != rhs->y) {
    return false;
  }
  // speed
  if (lhs->speed != rhs->speed) {
    return false;
  }
  return true;
}

bool
autorccar_interfaces__msg__PathPoint__copy(
  const autorccar_interfaces__msg__PathPoint * input,
  autorccar_interfaces__msg__PathPoint * output)
{
  if (!input || !output) {
    return false;
  }
  // x
  output->x = input->x;
  // y
  output->y = input->y;
  // speed
  output->speed = input->speed;
  return true;
}

autorccar_interfaces__msg__PathPoint *
autorccar_interfaces__msg__PathPoint__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  autorccar_interfaces__msg__PathPoint * msg = (autorccar_interfaces__msg__PathPoint *)allocator.allocate(sizeof(autorccar_interfaces__msg__PathPoint), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(autorccar_interfaces__msg__PathPoint));
  bool success = autorccar_interfaces__msg__PathPoint__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
autorccar_interfaces__msg__PathPoint__destroy(autorccar_interfaces__msg__PathPoint * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    autorccar_interfaces__msg__PathPoint__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
autorccar_interfaces__msg__PathPoint__Sequence__init(autorccar_interfaces__msg__PathPoint__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  autorccar_interfaces__msg__PathPoint * data = NULL;

  if (size) {
    data = (autorccar_interfaces__msg__PathPoint *)allocator.zero_allocate(size, sizeof(autorccar_interfaces__msg__PathPoint), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = autorccar_interfaces__msg__PathPoint__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        autorccar_interfaces__msg__PathPoint__fini(&data[i - 1]);
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
autorccar_interfaces__msg__PathPoint__Sequence__fini(autorccar_interfaces__msg__PathPoint__Sequence * array)
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
      autorccar_interfaces__msg__PathPoint__fini(&array->data[i]);
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

autorccar_interfaces__msg__PathPoint__Sequence *
autorccar_interfaces__msg__PathPoint__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  autorccar_interfaces__msg__PathPoint__Sequence * array = (autorccar_interfaces__msg__PathPoint__Sequence *)allocator.allocate(sizeof(autorccar_interfaces__msg__PathPoint__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = autorccar_interfaces__msg__PathPoint__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
autorccar_interfaces__msg__PathPoint__Sequence__destroy(autorccar_interfaces__msg__PathPoint__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    autorccar_interfaces__msg__PathPoint__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
autorccar_interfaces__msg__PathPoint__Sequence__are_equal(const autorccar_interfaces__msg__PathPoint__Sequence * lhs, const autorccar_interfaces__msg__PathPoint__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!autorccar_interfaces__msg__PathPoint__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
autorccar_interfaces__msg__PathPoint__Sequence__copy(
  const autorccar_interfaces__msg__PathPoint__Sequence * input,
  autorccar_interfaces__msg__PathPoint__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(autorccar_interfaces__msg__PathPoint);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    autorccar_interfaces__msg__PathPoint * data =
      (autorccar_interfaces__msg__PathPoint *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!autorccar_interfaces__msg__PathPoint__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          autorccar_interfaces__msg__PathPoint__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!autorccar_interfaces__msg__PathPoint__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
