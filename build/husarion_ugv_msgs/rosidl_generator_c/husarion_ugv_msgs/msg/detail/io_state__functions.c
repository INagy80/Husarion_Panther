// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from husarion_ugv_msgs:msg/IOState.idl
// generated code does not contain a copyright notice
#include "husarion_ugv_msgs/msg/detail/io_state__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
husarion_ugv_msgs__msg__IOState__init(husarion_ugv_msgs__msg__IOState * msg)
{
  if (!msg) {
    return false;
  }
  // aux_power
  // charger_connected
  // charger_enabled
  // digital_power
  // fan
  // led_control
  // motor_on
  // power_button
  return true;
}

void
husarion_ugv_msgs__msg__IOState__fini(husarion_ugv_msgs__msg__IOState * msg)
{
  if (!msg) {
    return;
  }
  // aux_power
  // charger_connected
  // charger_enabled
  // digital_power
  // fan
  // led_control
  // motor_on
  // power_button
}

bool
husarion_ugv_msgs__msg__IOState__are_equal(const husarion_ugv_msgs__msg__IOState * lhs, const husarion_ugv_msgs__msg__IOState * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // aux_power
  if (lhs->aux_power != rhs->aux_power) {
    return false;
  }
  // charger_connected
  if (lhs->charger_connected != rhs->charger_connected) {
    return false;
  }
  // charger_enabled
  if (lhs->charger_enabled != rhs->charger_enabled) {
    return false;
  }
  // digital_power
  if (lhs->digital_power != rhs->digital_power) {
    return false;
  }
  // fan
  if (lhs->fan != rhs->fan) {
    return false;
  }
  // led_control
  if (lhs->led_control != rhs->led_control) {
    return false;
  }
  // motor_on
  if (lhs->motor_on != rhs->motor_on) {
    return false;
  }
  // power_button
  if (lhs->power_button != rhs->power_button) {
    return false;
  }
  return true;
}

bool
husarion_ugv_msgs__msg__IOState__copy(
  const husarion_ugv_msgs__msg__IOState * input,
  husarion_ugv_msgs__msg__IOState * output)
{
  if (!input || !output) {
    return false;
  }
  // aux_power
  output->aux_power = input->aux_power;
  // charger_connected
  output->charger_connected = input->charger_connected;
  // charger_enabled
  output->charger_enabled = input->charger_enabled;
  // digital_power
  output->digital_power = input->digital_power;
  // fan
  output->fan = input->fan;
  // led_control
  output->led_control = input->led_control;
  // motor_on
  output->motor_on = input->motor_on;
  // power_button
  output->power_button = input->power_button;
  return true;
}

husarion_ugv_msgs__msg__IOState *
husarion_ugv_msgs__msg__IOState__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  husarion_ugv_msgs__msg__IOState * msg = (husarion_ugv_msgs__msg__IOState *)allocator.allocate(sizeof(husarion_ugv_msgs__msg__IOState), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(husarion_ugv_msgs__msg__IOState));
  bool success = husarion_ugv_msgs__msg__IOState__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
husarion_ugv_msgs__msg__IOState__destroy(husarion_ugv_msgs__msg__IOState * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    husarion_ugv_msgs__msg__IOState__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
husarion_ugv_msgs__msg__IOState__Sequence__init(husarion_ugv_msgs__msg__IOState__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  husarion_ugv_msgs__msg__IOState * data = NULL;

  if (size) {
    data = (husarion_ugv_msgs__msg__IOState *)allocator.zero_allocate(size, sizeof(husarion_ugv_msgs__msg__IOState), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = husarion_ugv_msgs__msg__IOState__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        husarion_ugv_msgs__msg__IOState__fini(&data[i - 1]);
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
husarion_ugv_msgs__msg__IOState__Sequence__fini(husarion_ugv_msgs__msg__IOState__Sequence * array)
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
      husarion_ugv_msgs__msg__IOState__fini(&array->data[i]);
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

husarion_ugv_msgs__msg__IOState__Sequence *
husarion_ugv_msgs__msg__IOState__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  husarion_ugv_msgs__msg__IOState__Sequence * array = (husarion_ugv_msgs__msg__IOState__Sequence *)allocator.allocate(sizeof(husarion_ugv_msgs__msg__IOState__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = husarion_ugv_msgs__msg__IOState__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
husarion_ugv_msgs__msg__IOState__Sequence__destroy(husarion_ugv_msgs__msg__IOState__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    husarion_ugv_msgs__msg__IOState__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
husarion_ugv_msgs__msg__IOState__Sequence__are_equal(const husarion_ugv_msgs__msg__IOState__Sequence * lhs, const husarion_ugv_msgs__msg__IOState__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!husarion_ugv_msgs__msg__IOState__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
husarion_ugv_msgs__msg__IOState__Sequence__copy(
  const husarion_ugv_msgs__msg__IOState__Sequence * input,
  husarion_ugv_msgs__msg__IOState__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(husarion_ugv_msgs__msg__IOState);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    husarion_ugv_msgs__msg__IOState * data =
      (husarion_ugv_msgs__msg__IOState *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!husarion_ugv_msgs__msg__IOState__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          husarion_ugv_msgs__msg__IOState__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!husarion_ugv_msgs__msg__IOState__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
