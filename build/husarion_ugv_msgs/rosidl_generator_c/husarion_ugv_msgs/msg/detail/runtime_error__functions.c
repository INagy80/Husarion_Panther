// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from husarion_ugv_msgs:msg/RuntimeError.idl
// generated code does not contain a copyright notice
#include "husarion_ugv_msgs/msg/detail/runtime_error__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
husarion_ugv_msgs__msg__RuntimeError__init(husarion_ugv_msgs__msg__RuntimeError * msg)
{
  if (!msg) {
    return false;
  }
  // amps_limit_active
  // motor_stall
  // loop_error
  // safety_stop_active
  // forward_limit_triggered
  // reverse_limit_triggered
  // amps_trigger_activated
  return true;
}

void
husarion_ugv_msgs__msg__RuntimeError__fini(husarion_ugv_msgs__msg__RuntimeError * msg)
{
  if (!msg) {
    return;
  }
  // amps_limit_active
  // motor_stall
  // loop_error
  // safety_stop_active
  // forward_limit_triggered
  // reverse_limit_triggered
  // amps_trigger_activated
}

bool
husarion_ugv_msgs__msg__RuntimeError__are_equal(const husarion_ugv_msgs__msg__RuntimeError * lhs, const husarion_ugv_msgs__msg__RuntimeError * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // amps_limit_active
  if (lhs->amps_limit_active != rhs->amps_limit_active) {
    return false;
  }
  // motor_stall
  if (lhs->motor_stall != rhs->motor_stall) {
    return false;
  }
  // loop_error
  if (lhs->loop_error != rhs->loop_error) {
    return false;
  }
  // safety_stop_active
  if (lhs->safety_stop_active != rhs->safety_stop_active) {
    return false;
  }
  // forward_limit_triggered
  if (lhs->forward_limit_triggered != rhs->forward_limit_triggered) {
    return false;
  }
  // reverse_limit_triggered
  if (lhs->reverse_limit_triggered != rhs->reverse_limit_triggered) {
    return false;
  }
  // amps_trigger_activated
  if (lhs->amps_trigger_activated != rhs->amps_trigger_activated) {
    return false;
  }
  return true;
}

bool
husarion_ugv_msgs__msg__RuntimeError__copy(
  const husarion_ugv_msgs__msg__RuntimeError * input,
  husarion_ugv_msgs__msg__RuntimeError * output)
{
  if (!input || !output) {
    return false;
  }
  // amps_limit_active
  output->amps_limit_active = input->amps_limit_active;
  // motor_stall
  output->motor_stall = input->motor_stall;
  // loop_error
  output->loop_error = input->loop_error;
  // safety_stop_active
  output->safety_stop_active = input->safety_stop_active;
  // forward_limit_triggered
  output->forward_limit_triggered = input->forward_limit_triggered;
  // reverse_limit_triggered
  output->reverse_limit_triggered = input->reverse_limit_triggered;
  // amps_trigger_activated
  output->amps_trigger_activated = input->amps_trigger_activated;
  return true;
}

husarion_ugv_msgs__msg__RuntimeError *
husarion_ugv_msgs__msg__RuntimeError__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  husarion_ugv_msgs__msg__RuntimeError * msg = (husarion_ugv_msgs__msg__RuntimeError *)allocator.allocate(sizeof(husarion_ugv_msgs__msg__RuntimeError), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(husarion_ugv_msgs__msg__RuntimeError));
  bool success = husarion_ugv_msgs__msg__RuntimeError__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
husarion_ugv_msgs__msg__RuntimeError__destroy(husarion_ugv_msgs__msg__RuntimeError * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    husarion_ugv_msgs__msg__RuntimeError__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
husarion_ugv_msgs__msg__RuntimeError__Sequence__init(husarion_ugv_msgs__msg__RuntimeError__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  husarion_ugv_msgs__msg__RuntimeError * data = NULL;

  if (size) {
    data = (husarion_ugv_msgs__msg__RuntimeError *)allocator.zero_allocate(size, sizeof(husarion_ugv_msgs__msg__RuntimeError), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = husarion_ugv_msgs__msg__RuntimeError__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        husarion_ugv_msgs__msg__RuntimeError__fini(&data[i - 1]);
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
husarion_ugv_msgs__msg__RuntimeError__Sequence__fini(husarion_ugv_msgs__msg__RuntimeError__Sequence * array)
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
      husarion_ugv_msgs__msg__RuntimeError__fini(&array->data[i]);
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

husarion_ugv_msgs__msg__RuntimeError__Sequence *
husarion_ugv_msgs__msg__RuntimeError__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  husarion_ugv_msgs__msg__RuntimeError__Sequence * array = (husarion_ugv_msgs__msg__RuntimeError__Sequence *)allocator.allocate(sizeof(husarion_ugv_msgs__msg__RuntimeError__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = husarion_ugv_msgs__msg__RuntimeError__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
husarion_ugv_msgs__msg__RuntimeError__Sequence__destroy(husarion_ugv_msgs__msg__RuntimeError__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    husarion_ugv_msgs__msg__RuntimeError__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
husarion_ugv_msgs__msg__RuntimeError__Sequence__are_equal(const husarion_ugv_msgs__msg__RuntimeError__Sequence * lhs, const husarion_ugv_msgs__msg__RuntimeError__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!husarion_ugv_msgs__msg__RuntimeError__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
husarion_ugv_msgs__msg__RuntimeError__Sequence__copy(
  const husarion_ugv_msgs__msg__RuntimeError__Sequence * input,
  husarion_ugv_msgs__msg__RuntimeError__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(husarion_ugv_msgs__msg__RuntimeError);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    husarion_ugv_msgs__msg__RuntimeError * data =
      (husarion_ugv_msgs__msg__RuntimeError *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!husarion_ugv_msgs__msg__RuntimeError__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          husarion_ugv_msgs__msg__RuntimeError__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!husarion_ugv_msgs__msg__RuntimeError__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
