// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from husarion_ugv_msgs:msg/DriverState.idl
// generated code does not contain a copyright notice
#include "husarion_ugv_msgs/msg/detail/driver_state__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `fault_flag`
#include "husarion_ugv_msgs/msg/detail/fault_flag__functions.h"
// Member `script_flag`
#include "husarion_ugv_msgs/msg/detail/script_flag__functions.h"
// Member `channel_1_motor_runtime_error`
// Member `channel_2_motor_runtime_error`
#include "husarion_ugv_msgs/msg/detail/runtime_error__functions.h"

bool
husarion_ugv_msgs__msg__DriverState__init(husarion_ugv_msgs__msg__DriverState * msg)
{
  if (!msg) {
    return false;
  }
  // voltage
  // current
  // temperature
  // heatsink_temperature
  // fault_flag
  if (!husarion_ugv_msgs__msg__FaultFlag__init(&msg->fault_flag)) {
    husarion_ugv_msgs__msg__DriverState__fini(msg);
    return false;
  }
  // script_flag
  if (!husarion_ugv_msgs__msg__ScriptFlag__init(&msg->script_flag)) {
    husarion_ugv_msgs__msg__DriverState__fini(msg);
    return false;
  }
  // channel_1_motor_runtime_error
  if (!husarion_ugv_msgs__msg__RuntimeError__init(&msg->channel_1_motor_runtime_error)) {
    husarion_ugv_msgs__msg__DriverState__fini(msg);
    return false;
  }
  // channel_2_motor_runtime_error
  if (!husarion_ugv_msgs__msg__RuntimeError__init(&msg->channel_2_motor_runtime_error)) {
    husarion_ugv_msgs__msg__DriverState__fini(msg);
    return false;
  }
  // motor_states_data_timed_out
  // driver_state_data_timed_out
  // can_error
  // heartbeat_timeout
  return true;
}

void
husarion_ugv_msgs__msg__DriverState__fini(husarion_ugv_msgs__msg__DriverState * msg)
{
  if (!msg) {
    return;
  }
  // voltage
  // current
  // temperature
  // heatsink_temperature
  // fault_flag
  husarion_ugv_msgs__msg__FaultFlag__fini(&msg->fault_flag);
  // script_flag
  husarion_ugv_msgs__msg__ScriptFlag__fini(&msg->script_flag);
  // channel_1_motor_runtime_error
  husarion_ugv_msgs__msg__RuntimeError__fini(&msg->channel_1_motor_runtime_error);
  // channel_2_motor_runtime_error
  husarion_ugv_msgs__msg__RuntimeError__fini(&msg->channel_2_motor_runtime_error);
  // motor_states_data_timed_out
  // driver_state_data_timed_out
  // can_error
  // heartbeat_timeout
}

bool
husarion_ugv_msgs__msg__DriverState__are_equal(const husarion_ugv_msgs__msg__DriverState * lhs, const husarion_ugv_msgs__msg__DriverState * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // voltage
  if (lhs->voltage != rhs->voltage) {
    return false;
  }
  // current
  if (lhs->current != rhs->current) {
    return false;
  }
  // temperature
  if (lhs->temperature != rhs->temperature) {
    return false;
  }
  // heatsink_temperature
  if (lhs->heatsink_temperature != rhs->heatsink_temperature) {
    return false;
  }
  // fault_flag
  if (!husarion_ugv_msgs__msg__FaultFlag__are_equal(
      &(lhs->fault_flag), &(rhs->fault_flag)))
  {
    return false;
  }
  // script_flag
  if (!husarion_ugv_msgs__msg__ScriptFlag__are_equal(
      &(lhs->script_flag), &(rhs->script_flag)))
  {
    return false;
  }
  // channel_1_motor_runtime_error
  if (!husarion_ugv_msgs__msg__RuntimeError__are_equal(
      &(lhs->channel_1_motor_runtime_error), &(rhs->channel_1_motor_runtime_error)))
  {
    return false;
  }
  // channel_2_motor_runtime_error
  if (!husarion_ugv_msgs__msg__RuntimeError__are_equal(
      &(lhs->channel_2_motor_runtime_error), &(rhs->channel_2_motor_runtime_error)))
  {
    return false;
  }
  // motor_states_data_timed_out
  if (lhs->motor_states_data_timed_out != rhs->motor_states_data_timed_out) {
    return false;
  }
  // driver_state_data_timed_out
  if (lhs->driver_state_data_timed_out != rhs->driver_state_data_timed_out) {
    return false;
  }
  // can_error
  if (lhs->can_error != rhs->can_error) {
    return false;
  }
  // heartbeat_timeout
  if (lhs->heartbeat_timeout != rhs->heartbeat_timeout) {
    return false;
  }
  return true;
}

bool
husarion_ugv_msgs__msg__DriverState__copy(
  const husarion_ugv_msgs__msg__DriverState * input,
  husarion_ugv_msgs__msg__DriverState * output)
{
  if (!input || !output) {
    return false;
  }
  // voltage
  output->voltage = input->voltage;
  // current
  output->current = input->current;
  // temperature
  output->temperature = input->temperature;
  // heatsink_temperature
  output->heatsink_temperature = input->heatsink_temperature;
  // fault_flag
  if (!husarion_ugv_msgs__msg__FaultFlag__copy(
      &(input->fault_flag), &(output->fault_flag)))
  {
    return false;
  }
  // script_flag
  if (!husarion_ugv_msgs__msg__ScriptFlag__copy(
      &(input->script_flag), &(output->script_flag)))
  {
    return false;
  }
  // channel_1_motor_runtime_error
  if (!husarion_ugv_msgs__msg__RuntimeError__copy(
      &(input->channel_1_motor_runtime_error), &(output->channel_1_motor_runtime_error)))
  {
    return false;
  }
  // channel_2_motor_runtime_error
  if (!husarion_ugv_msgs__msg__RuntimeError__copy(
      &(input->channel_2_motor_runtime_error), &(output->channel_2_motor_runtime_error)))
  {
    return false;
  }
  // motor_states_data_timed_out
  output->motor_states_data_timed_out = input->motor_states_data_timed_out;
  // driver_state_data_timed_out
  output->driver_state_data_timed_out = input->driver_state_data_timed_out;
  // can_error
  output->can_error = input->can_error;
  // heartbeat_timeout
  output->heartbeat_timeout = input->heartbeat_timeout;
  return true;
}

husarion_ugv_msgs__msg__DriverState *
husarion_ugv_msgs__msg__DriverState__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  husarion_ugv_msgs__msg__DriverState * msg = (husarion_ugv_msgs__msg__DriverState *)allocator.allocate(sizeof(husarion_ugv_msgs__msg__DriverState), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(husarion_ugv_msgs__msg__DriverState));
  bool success = husarion_ugv_msgs__msg__DriverState__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
husarion_ugv_msgs__msg__DriverState__destroy(husarion_ugv_msgs__msg__DriverState * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    husarion_ugv_msgs__msg__DriverState__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
husarion_ugv_msgs__msg__DriverState__Sequence__init(husarion_ugv_msgs__msg__DriverState__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  husarion_ugv_msgs__msg__DriverState * data = NULL;

  if (size) {
    data = (husarion_ugv_msgs__msg__DriverState *)allocator.zero_allocate(size, sizeof(husarion_ugv_msgs__msg__DriverState), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = husarion_ugv_msgs__msg__DriverState__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        husarion_ugv_msgs__msg__DriverState__fini(&data[i - 1]);
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
husarion_ugv_msgs__msg__DriverState__Sequence__fini(husarion_ugv_msgs__msg__DriverState__Sequence * array)
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
      husarion_ugv_msgs__msg__DriverState__fini(&array->data[i]);
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

husarion_ugv_msgs__msg__DriverState__Sequence *
husarion_ugv_msgs__msg__DriverState__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  husarion_ugv_msgs__msg__DriverState__Sequence * array = (husarion_ugv_msgs__msg__DriverState__Sequence *)allocator.allocate(sizeof(husarion_ugv_msgs__msg__DriverState__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = husarion_ugv_msgs__msg__DriverState__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
husarion_ugv_msgs__msg__DriverState__Sequence__destroy(husarion_ugv_msgs__msg__DriverState__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    husarion_ugv_msgs__msg__DriverState__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
husarion_ugv_msgs__msg__DriverState__Sequence__are_equal(const husarion_ugv_msgs__msg__DriverState__Sequence * lhs, const husarion_ugv_msgs__msg__DriverState__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!husarion_ugv_msgs__msg__DriverState__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
husarion_ugv_msgs__msg__DriverState__Sequence__copy(
  const husarion_ugv_msgs__msg__DriverState__Sequence * input,
  husarion_ugv_msgs__msg__DriverState__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(husarion_ugv_msgs__msg__DriverState);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    husarion_ugv_msgs__msg__DriverState * data =
      (husarion_ugv_msgs__msg__DriverState *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!husarion_ugv_msgs__msg__DriverState__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          husarion_ugv_msgs__msg__DriverState__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!husarion_ugv_msgs__msg__DriverState__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
