// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from husarion_ugv_msgs:msg/LEDAnimationQueue.idl
// generated code does not contain a copyright notice
#include "husarion_ugv_msgs/msg/detail/led_animation_queue__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `queue`
#include "rosidl_runtime_c/string_functions.h"

bool
husarion_ugv_msgs__msg__LEDAnimationQueue__init(husarion_ugv_msgs__msg__LEDAnimationQueue * msg)
{
  if (!msg) {
    return false;
  }
  // queue
  if (!rosidl_runtime_c__String__Sequence__init(&msg->queue, 0)) {
    husarion_ugv_msgs__msg__LEDAnimationQueue__fini(msg);
    return false;
  }
  return true;
}

void
husarion_ugv_msgs__msg__LEDAnimationQueue__fini(husarion_ugv_msgs__msg__LEDAnimationQueue * msg)
{
  if (!msg) {
    return;
  }
  // queue
  rosidl_runtime_c__String__Sequence__fini(&msg->queue);
}

bool
husarion_ugv_msgs__msg__LEDAnimationQueue__are_equal(const husarion_ugv_msgs__msg__LEDAnimationQueue * lhs, const husarion_ugv_msgs__msg__LEDAnimationQueue * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // queue
  if (!rosidl_runtime_c__String__Sequence__are_equal(
      &(lhs->queue), &(rhs->queue)))
  {
    return false;
  }
  return true;
}

bool
husarion_ugv_msgs__msg__LEDAnimationQueue__copy(
  const husarion_ugv_msgs__msg__LEDAnimationQueue * input,
  husarion_ugv_msgs__msg__LEDAnimationQueue * output)
{
  if (!input || !output) {
    return false;
  }
  // queue
  if (!rosidl_runtime_c__String__Sequence__copy(
      &(input->queue), &(output->queue)))
  {
    return false;
  }
  return true;
}

husarion_ugv_msgs__msg__LEDAnimationQueue *
husarion_ugv_msgs__msg__LEDAnimationQueue__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  husarion_ugv_msgs__msg__LEDAnimationQueue * msg = (husarion_ugv_msgs__msg__LEDAnimationQueue *)allocator.allocate(sizeof(husarion_ugv_msgs__msg__LEDAnimationQueue), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(husarion_ugv_msgs__msg__LEDAnimationQueue));
  bool success = husarion_ugv_msgs__msg__LEDAnimationQueue__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
husarion_ugv_msgs__msg__LEDAnimationQueue__destroy(husarion_ugv_msgs__msg__LEDAnimationQueue * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    husarion_ugv_msgs__msg__LEDAnimationQueue__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
husarion_ugv_msgs__msg__LEDAnimationQueue__Sequence__init(husarion_ugv_msgs__msg__LEDAnimationQueue__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  husarion_ugv_msgs__msg__LEDAnimationQueue * data = NULL;

  if (size) {
    data = (husarion_ugv_msgs__msg__LEDAnimationQueue *)allocator.zero_allocate(size, sizeof(husarion_ugv_msgs__msg__LEDAnimationQueue), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = husarion_ugv_msgs__msg__LEDAnimationQueue__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        husarion_ugv_msgs__msg__LEDAnimationQueue__fini(&data[i - 1]);
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
husarion_ugv_msgs__msg__LEDAnimationQueue__Sequence__fini(husarion_ugv_msgs__msg__LEDAnimationQueue__Sequence * array)
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
      husarion_ugv_msgs__msg__LEDAnimationQueue__fini(&array->data[i]);
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

husarion_ugv_msgs__msg__LEDAnimationQueue__Sequence *
husarion_ugv_msgs__msg__LEDAnimationQueue__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  husarion_ugv_msgs__msg__LEDAnimationQueue__Sequence * array = (husarion_ugv_msgs__msg__LEDAnimationQueue__Sequence *)allocator.allocate(sizeof(husarion_ugv_msgs__msg__LEDAnimationQueue__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = husarion_ugv_msgs__msg__LEDAnimationQueue__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
husarion_ugv_msgs__msg__LEDAnimationQueue__Sequence__destroy(husarion_ugv_msgs__msg__LEDAnimationQueue__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    husarion_ugv_msgs__msg__LEDAnimationQueue__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
husarion_ugv_msgs__msg__LEDAnimationQueue__Sequence__are_equal(const husarion_ugv_msgs__msg__LEDAnimationQueue__Sequence * lhs, const husarion_ugv_msgs__msg__LEDAnimationQueue__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!husarion_ugv_msgs__msg__LEDAnimationQueue__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
husarion_ugv_msgs__msg__LEDAnimationQueue__Sequence__copy(
  const husarion_ugv_msgs__msg__LEDAnimationQueue__Sequence * input,
  husarion_ugv_msgs__msg__LEDAnimationQueue__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(husarion_ugv_msgs__msg__LEDAnimationQueue);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    husarion_ugv_msgs__msg__LEDAnimationQueue * data =
      (husarion_ugv_msgs__msg__LEDAnimationQueue *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!husarion_ugv_msgs__msg__LEDAnimationQueue__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          husarion_ugv_msgs__msg__LEDAnimationQueue__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!husarion_ugv_msgs__msg__LEDAnimationQueue__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
