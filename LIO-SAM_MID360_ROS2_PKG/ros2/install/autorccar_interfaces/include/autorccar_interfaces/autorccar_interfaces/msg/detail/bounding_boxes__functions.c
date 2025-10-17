// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from autorccar_interfaces:msg/BoundingBoxes.idl
// generated code does not contain a copyright notice
#include "autorccar_interfaces/msg/detail/bounding_boxes__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `bounding_boxes`
#include "vision_msgs/msg/detail/bounding_box2_d__functions.h"

bool
autorccar_interfaces__msg__BoundingBoxes__init(autorccar_interfaces__msg__BoundingBoxes * msg)
{
  if (!msg) {
    return false;
  }
  // num
  // bounding_boxes
  if (!vision_msgs__msg__BoundingBox2D__Sequence__init(&msg->bounding_boxes, 0)) {
    autorccar_interfaces__msg__BoundingBoxes__fini(msg);
    return false;
  }
  return true;
}

void
autorccar_interfaces__msg__BoundingBoxes__fini(autorccar_interfaces__msg__BoundingBoxes * msg)
{
  if (!msg) {
    return;
  }
  // num
  // bounding_boxes
  vision_msgs__msg__BoundingBox2D__Sequence__fini(&msg->bounding_boxes);
}

bool
autorccar_interfaces__msg__BoundingBoxes__are_equal(const autorccar_interfaces__msg__BoundingBoxes * lhs, const autorccar_interfaces__msg__BoundingBoxes * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // num
  if (lhs->num != rhs->num) {
    return false;
  }
  // bounding_boxes
  if (!vision_msgs__msg__BoundingBox2D__Sequence__are_equal(
      &(lhs->bounding_boxes), &(rhs->bounding_boxes)))
  {
    return false;
  }
  return true;
}

bool
autorccar_interfaces__msg__BoundingBoxes__copy(
  const autorccar_interfaces__msg__BoundingBoxes * input,
  autorccar_interfaces__msg__BoundingBoxes * output)
{
  if (!input || !output) {
    return false;
  }
  // num
  output->num = input->num;
  // bounding_boxes
  if (!vision_msgs__msg__BoundingBox2D__Sequence__copy(
      &(input->bounding_boxes), &(output->bounding_boxes)))
  {
    return false;
  }
  return true;
}

autorccar_interfaces__msg__BoundingBoxes *
autorccar_interfaces__msg__BoundingBoxes__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  autorccar_interfaces__msg__BoundingBoxes * msg = (autorccar_interfaces__msg__BoundingBoxes *)allocator.allocate(sizeof(autorccar_interfaces__msg__BoundingBoxes), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(autorccar_interfaces__msg__BoundingBoxes));
  bool success = autorccar_interfaces__msg__BoundingBoxes__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
autorccar_interfaces__msg__BoundingBoxes__destroy(autorccar_interfaces__msg__BoundingBoxes * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    autorccar_interfaces__msg__BoundingBoxes__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
autorccar_interfaces__msg__BoundingBoxes__Sequence__init(autorccar_interfaces__msg__BoundingBoxes__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  autorccar_interfaces__msg__BoundingBoxes * data = NULL;

  if (size) {
    data = (autorccar_interfaces__msg__BoundingBoxes *)allocator.zero_allocate(size, sizeof(autorccar_interfaces__msg__BoundingBoxes), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = autorccar_interfaces__msg__BoundingBoxes__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        autorccar_interfaces__msg__BoundingBoxes__fini(&data[i - 1]);
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
autorccar_interfaces__msg__BoundingBoxes__Sequence__fini(autorccar_interfaces__msg__BoundingBoxes__Sequence * array)
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
      autorccar_interfaces__msg__BoundingBoxes__fini(&array->data[i]);
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

autorccar_interfaces__msg__BoundingBoxes__Sequence *
autorccar_interfaces__msg__BoundingBoxes__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  autorccar_interfaces__msg__BoundingBoxes__Sequence * array = (autorccar_interfaces__msg__BoundingBoxes__Sequence *)allocator.allocate(sizeof(autorccar_interfaces__msg__BoundingBoxes__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = autorccar_interfaces__msg__BoundingBoxes__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
autorccar_interfaces__msg__BoundingBoxes__Sequence__destroy(autorccar_interfaces__msg__BoundingBoxes__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    autorccar_interfaces__msg__BoundingBoxes__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
autorccar_interfaces__msg__BoundingBoxes__Sequence__are_equal(const autorccar_interfaces__msg__BoundingBoxes__Sequence * lhs, const autorccar_interfaces__msg__BoundingBoxes__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!autorccar_interfaces__msg__BoundingBoxes__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
autorccar_interfaces__msg__BoundingBoxes__Sequence__copy(
  const autorccar_interfaces__msg__BoundingBoxes__Sequence * input,
  autorccar_interfaces__msg__BoundingBoxes__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(autorccar_interfaces__msg__BoundingBoxes);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    autorccar_interfaces__msg__BoundingBoxes * data =
      (autorccar_interfaces__msg__BoundingBoxes *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!autorccar_interfaces__msg__BoundingBoxes__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          autorccar_interfaces__msg__BoundingBoxes__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!autorccar_interfaces__msg__BoundingBoxes__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
