// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from husarion_ugv_msgs:msg/LEDAnimation.idl
// generated code does not contain a copyright notice
#include "husarion_ugv_msgs/msg/detail/led_animation__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `param`
#include "rosidl_runtime_c/string_functions.h"

bool
husarion_ugv_msgs__msg__LEDAnimation__init(husarion_ugv_msgs__msg__LEDAnimation * msg)
{
  if (!msg) {
    return false;
  }
  // id
  // param
  if (!rosidl_runtime_c__String__init(&msg->param)) {
    husarion_ugv_msgs__msg__LEDAnimation__fini(msg);
    return false;
  }
  return true;
}

void
husarion_ugv_msgs__msg__LEDAnimation__fini(husarion_ugv_msgs__msg__LEDAnimation * msg)
{
  if (!msg) {
    return;
  }
  // id
  // param
  rosidl_runtime_c__String__fini(&msg->param);
}

bool
husarion_ugv_msgs__msg__LEDAnimation__are_equal(const husarion_ugv_msgs__msg__LEDAnimation * lhs, const husarion_ugv_msgs__msg__LEDAnimation * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // id
  if (lhs->id != rhs->id) {
    return false;
  }
  // param
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->param), &(rhs->param)))
  {
    return false;
  }
  return true;
}

bool
husarion_ugv_msgs__msg__LEDAnimation__copy(
  const husarion_ugv_msgs__msg__LEDAnimation * input,
  husarion_ugv_msgs__msg__LEDAnimation * output)
{
  if (!input || !output) {
    return false;
  }
  // id
  output->id = input->id;
  // param
  if (!rosidl_runtime_c__String__copy(
      &(input->param), &(output->param)))
  {
    return false;
  }
  return true;
}

husarion_ugv_msgs__msg__LEDAnimation *
husarion_ugv_msgs__msg__LEDAnimation__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  husarion_ugv_msgs__msg__LEDAnimation * msg = (husarion_ugv_msgs__msg__LEDAnimation *)allocator.allocate(sizeof(husarion_ugv_msgs__msg__LEDAnimation), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(husarion_ugv_msgs__msg__LEDAnimation));
  bool success = husarion_ugv_msgs__msg__LEDAnimation__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
husarion_ugv_msgs__msg__LEDAnimation__destroy(husarion_ugv_msgs__msg__LEDAnimation * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    husarion_ugv_msgs__msg__LEDAnimation__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
husarion_ugv_msgs__msg__LEDAnimation__Sequence__init(husarion_ugv_msgs__msg__LEDAnimation__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  husarion_ugv_msgs__msg__LEDAnimation * data = NULL;

  if (size) {
    data = (husarion_ugv_msgs__msg__LEDAnimation *)allocator.zero_allocate(size, sizeof(husarion_ugv_msgs__msg__LEDAnimation), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = husarion_ugv_msgs__msg__LEDAnimation__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        husarion_ugv_msgs__msg__LEDAnimation__fini(&data[i - 1]);
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
husarion_ugv_msgs__msg__LEDAnimation__Sequence__fini(husarion_ugv_msgs__msg__LEDAnimation__Sequence * array)
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
      husarion_ugv_msgs__msg__LEDAnimation__fini(&array->data[i]);
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

husarion_ugv_msgs__msg__LEDAnimation__Sequence *
husarion_ugv_msgs__msg__LEDAnimation__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  husarion_ugv_msgs__msg__LEDAnimation__Sequence * array = (husarion_ugv_msgs__msg__LEDAnimation__Sequence *)allocator.allocate(sizeof(husarion_ugv_msgs__msg__LEDAnimation__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = husarion_ugv_msgs__msg__LEDAnimation__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
husarion_ugv_msgs__msg__LEDAnimation__Sequence__destroy(husarion_ugv_msgs__msg__LEDAnimation__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    husarion_ugv_msgs__msg__LEDAnimation__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
husarion_ugv_msgs__msg__LEDAnimation__Sequence__are_equal(const husarion_ugv_msgs__msg__LEDAnimation__Sequence * lhs, const husarion_ugv_msgs__msg__LEDAnimation__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!husarion_ugv_msgs__msg__LEDAnimation__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
husarion_ugv_msgs__msg__LEDAnimation__Sequence__copy(
  const husarion_ugv_msgs__msg__LEDAnimation__Sequence * input,
  husarion_ugv_msgs__msg__LEDAnimation__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(husarion_ugv_msgs__msg__LEDAnimation);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    husarion_ugv_msgs__msg__LEDAnimation * data =
      (husarion_ugv_msgs__msg__LEDAnimation *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!husarion_ugv_msgs__msg__LEDAnimation__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          husarion_ugv_msgs__msg__LEDAnimation__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!husarion_ugv_msgs__msg__LEDAnimation__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
