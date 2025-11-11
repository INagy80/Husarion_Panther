// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from husarion_ugv_msgs:msg/ScriptFlag.idl
// generated code does not contain a copyright notice
#include "husarion_ugv_msgs/msg/detail/script_flag__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
husarion_ugv_msgs__msg__ScriptFlag__init(husarion_ugv_msgs__msg__ScriptFlag * msg)
{
  if (!msg) {
    return false;
  }
  // loop_error
  // encoder_disconnected
  // amp_limiter
  return true;
}

void
husarion_ugv_msgs__msg__ScriptFlag__fini(husarion_ugv_msgs__msg__ScriptFlag * msg)
{
  if (!msg) {
    return;
  }
  // loop_error
  // encoder_disconnected
  // amp_limiter
}

bool
husarion_ugv_msgs__msg__ScriptFlag__are_equal(const husarion_ugv_msgs__msg__ScriptFlag * lhs, const husarion_ugv_msgs__msg__ScriptFlag * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // loop_error
  if (lhs->loop_error != rhs->loop_error) {
    return false;
  }
  // encoder_disconnected
  if (lhs->encoder_disconnected != rhs->encoder_disconnected) {
    return false;
  }
  // amp_limiter
  if (lhs->amp_limiter != rhs->amp_limiter) {
    return false;
  }
  return true;
}

bool
husarion_ugv_msgs__msg__ScriptFlag__copy(
  const husarion_ugv_msgs__msg__ScriptFlag * input,
  husarion_ugv_msgs__msg__ScriptFlag * output)
{
  if (!input || !output) {
    return false;
  }
  // loop_error
  output->loop_error = input->loop_error;
  // encoder_disconnected
  output->encoder_disconnected = input->encoder_disconnected;
  // amp_limiter
  output->amp_limiter = input->amp_limiter;
  return true;
}

husarion_ugv_msgs__msg__ScriptFlag *
husarion_ugv_msgs__msg__ScriptFlag__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  husarion_ugv_msgs__msg__ScriptFlag * msg = (husarion_ugv_msgs__msg__ScriptFlag *)allocator.allocate(sizeof(husarion_ugv_msgs__msg__ScriptFlag), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(husarion_ugv_msgs__msg__ScriptFlag));
  bool success = husarion_ugv_msgs__msg__ScriptFlag__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
husarion_ugv_msgs__msg__ScriptFlag__destroy(husarion_ugv_msgs__msg__ScriptFlag * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    husarion_ugv_msgs__msg__ScriptFlag__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
husarion_ugv_msgs__msg__ScriptFlag__Sequence__init(husarion_ugv_msgs__msg__ScriptFlag__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  husarion_ugv_msgs__msg__ScriptFlag * data = NULL;

  if (size) {
    data = (husarion_ugv_msgs__msg__ScriptFlag *)allocator.zero_allocate(size, sizeof(husarion_ugv_msgs__msg__ScriptFlag), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = husarion_ugv_msgs__msg__ScriptFlag__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        husarion_ugv_msgs__msg__ScriptFlag__fini(&data[i - 1]);
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
husarion_ugv_msgs__msg__ScriptFlag__Sequence__fini(husarion_ugv_msgs__msg__ScriptFlag__Sequence * array)
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
      husarion_ugv_msgs__msg__ScriptFlag__fini(&array->data[i]);
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

husarion_ugv_msgs__msg__ScriptFlag__Sequence *
husarion_ugv_msgs__msg__ScriptFlag__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  husarion_ugv_msgs__msg__ScriptFlag__Sequence * array = (husarion_ugv_msgs__msg__ScriptFlag__Sequence *)allocator.allocate(sizeof(husarion_ugv_msgs__msg__ScriptFlag__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = husarion_ugv_msgs__msg__ScriptFlag__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
husarion_ugv_msgs__msg__ScriptFlag__Sequence__destroy(husarion_ugv_msgs__msg__ScriptFlag__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    husarion_ugv_msgs__msg__ScriptFlag__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
husarion_ugv_msgs__msg__ScriptFlag__Sequence__are_equal(const husarion_ugv_msgs__msg__ScriptFlag__Sequence * lhs, const husarion_ugv_msgs__msg__ScriptFlag__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!husarion_ugv_msgs__msg__ScriptFlag__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
husarion_ugv_msgs__msg__ScriptFlag__Sequence__copy(
  const husarion_ugv_msgs__msg__ScriptFlag__Sequence * input,
  husarion_ugv_msgs__msg__ScriptFlag__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(husarion_ugv_msgs__msg__ScriptFlag);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    husarion_ugv_msgs__msg__ScriptFlag * data =
      (husarion_ugv_msgs__msg__ScriptFlag *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!husarion_ugv_msgs__msg__ScriptFlag__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          husarion_ugv_msgs__msg__ScriptFlag__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!husarion_ugv_msgs__msg__ScriptFlag__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
