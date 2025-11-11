// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from husarion_ugv_msgs:msg/LEDImageAnimation.idl
// generated code does not contain a copyright notice
#include "husarion_ugv_msgs/msg/detail/led_image_animation__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `image`
#include "rosidl_runtime_c/string_functions.h"

bool
husarion_ugv_msgs__msg__LEDImageAnimation__init(husarion_ugv_msgs__msg__LEDImageAnimation * msg)
{
  if (!msg) {
    return false;
  }
  // image
  if (!rosidl_runtime_c__String__init(&msg->image)) {
    husarion_ugv_msgs__msg__LEDImageAnimation__fini(msg);
    return false;
  }
  // duration
  // brightness
  // repeat
  // color
  return true;
}

void
husarion_ugv_msgs__msg__LEDImageAnimation__fini(husarion_ugv_msgs__msg__LEDImageAnimation * msg)
{
  if (!msg) {
    return;
  }
  // image
  rosidl_runtime_c__String__fini(&msg->image);
  // duration
  // brightness
  // repeat
  // color
}

bool
husarion_ugv_msgs__msg__LEDImageAnimation__are_equal(const husarion_ugv_msgs__msg__LEDImageAnimation * lhs, const husarion_ugv_msgs__msg__LEDImageAnimation * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // image
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->image), &(rhs->image)))
  {
    return false;
  }
  // duration
  if (lhs->duration != rhs->duration) {
    return false;
  }
  // brightness
  if (lhs->brightness != rhs->brightness) {
    return false;
  }
  // repeat
  if (lhs->repeat != rhs->repeat) {
    return false;
  }
  // color
  if (lhs->color != rhs->color) {
    return false;
  }
  return true;
}

bool
husarion_ugv_msgs__msg__LEDImageAnimation__copy(
  const husarion_ugv_msgs__msg__LEDImageAnimation * input,
  husarion_ugv_msgs__msg__LEDImageAnimation * output)
{
  if (!input || !output) {
    return false;
  }
  // image
  if (!rosidl_runtime_c__String__copy(
      &(input->image), &(output->image)))
  {
    return false;
  }
  // duration
  output->duration = input->duration;
  // brightness
  output->brightness = input->brightness;
  // repeat
  output->repeat = input->repeat;
  // color
  output->color = input->color;
  return true;
}

husarion_ugv_msgs__msg__LEDImageAnimation *
husarion_ugv_msgs__msg__LEDImageAnimation__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  husarion_ugv_msgs__msg__LEDImageAnimation * msg = (husarion_ugv_msgs__msg__LEDImageAnimation *)allocator.allocate(sizeof(husarion_ugv_msgs__msg__LEDImageAnimation), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(husarion_ugv_msgs__msg__LEDImageAnimation));
  bool success = husarion_ugv_msgs__msg__LEDImageAnimation__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
husarion_ugv_msgs__msg__LEDImageAnimation__destroy(husarion_ugv_msgs__msg__LEDImageAnimation * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    husarion_ugv_msgs__msg__LEDImageAnimation__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
husarion_ugv_msgs__msg__LEDImageAnimation__Sequence__init(husarion_ugv_msgs__msg__LEDImageAnimation__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  husarion_ugv_msgs__msg__LEDImageAnimation * data = NULL;

  if (size) {
    data = (husarion_ugv_msgs__msg__LEDImageAnimation *)allocator.zero_allocate(size, sizeof(husarion_ugv_msgs__msg__LEDImageAnimation), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = husarion_ugv_msgs__msg__LEDImageAnimation__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        husarion_ugv_msgs__msg__LEDImageAnimation__fini(&data[i - 1]);
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
husarion_ugv_msgs__msg__LEDImageAnimation__Sequence__fini(husarion_ugv_msgs__msg__LEDImageAnimation__Sequence * array)
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
      husarion_ugv_msgs__msg__LEDImageAnimation__fini(&array->data[i]);
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

husarion_ugv_msgs__msg__LEDImageAnimation__Sequence *
husarion_ugv_msgs__msg__LEDImageAnimation__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  husarion_ugv_msgs__msg__LEDImageAnimation__Sequence * array = (husarion_ugv_msgs__msg__LEDImageAnimation__Sequence *)allocator.allocate(sizeof(husarion_ugv_msgs__msg__LEDImageAnimation__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = husarion_ugv_msgs__msg__LEDImageAnimation__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
husarion_ugv_msgs__msg__LEDImageAnimation__Sequence__destroy(husarion_ugv_msgs__msg__LEDImageAnimation__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    husarion_ugv_msgs__msg__LEDImageAnimation__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
husarion_ugv_msgs__msg__LEDImageAnimation__Sequence__are_equal(const husarion_ugv_msgs__msg__LEDImageAnimation__Sequence * lhs, const husarion_ugv_msgs__msg__LEDImageAnimation__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!husarion_ugv_msgs__msg__LEDImageAnimation__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
husarion_ugv_msgs__msg__LEDImageAnimation__Sequence__copy(
  const husarion_ugv_msgs__msg__LEDImageAnimation__Sequence * input,
  husarion_ugv_msgs__msg__LEDImageAnimation__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(husarion_ugv_msgs__msg__LEDImageAnimation);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    husarion_ugv_msgs__msg__LEDImageAnimation * data =
      (husarion_ugv_msgs__msg__LEDImageAnimation *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!husarion_ugv_msgs__msg__LEDImageAnimation__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          husarion_ugv_msgs__msg__LEDImageAnimation__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!husarion_ugv_msgs__msg__LEDImageAnimation__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
