// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from husarion_ugv_msgs:msg/FaultFlag.idl
// generated code does not contain a copyright notice
#include "husarion_ugv_msgs/msg/detail/fault_flag__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
husarion_ugv_msgs__msg__FaultFlag__init(husarion_ugv_msgs__msg__FaultFlag * msg)
{
  if (!msg) {
    return false;
  }
  // overheat
  // overvoltage
  // undervoltage
  // short_circuit
  // emergency_stop
  // motor_or_sensor_setup_fault
  // mosfet_failure
  // default_config_loaded_at_startup
  return true;
}

void
husarion_ugv_msgs__msg__FaultFlag__fini(husarion_ugv_msgs__msg__FaultFlag * msg)
{
  if (!msg) {
    return;
  }
  // overheat
  // overvoltage
  // undervoltage
  // short_circuit
  // emergency_stop
  // motor_or_sensor_setup_fault
  // mosfet_failure
  // default_config_loaded_at_startup
}

bool
husarion_ugv_msgs__msg__FaultFlag__are_equal(const husarion_ugv_msgs__msg__FaultFlag * lhs, const husarion_ugv_msgs__msg__FaultFlag * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // overheat
  if (lhs->overheat != rhs->overheat) {
    return false;
  }
  // overvoltage
  if (lhs->overvoltage != rhs->overvoltage) {
    return false;
  }
  // undervoltage
  if (lhs->undervoltage != rhs->undervoltage) {
    return false;
  }
  // short_circuit
  if (lhs->short_circuit != rhs->short_circuit) {
    return false;
  }
  // emergency_stop
  if (lhs->emergency_stop != rhs->emergency_stop) {
    return false;
  }
  // motor_or_sensor_setup_fault
  if (lhs->motor_or_sensor_setup_fault != rhs->motor_or_sensor_setup_fault) {
    return false;
  }
  // mosfet_failure
  if (lhs->mosfet_failure != rhs->mosfet_failure) {
    return false;
  }
  // default_config_loaded_at_startup
  if (lhs->default_config_loaded_at_startup != rhs->default_config_loaded_at_startup) {
    return false;
  }
  return true;
}

bool
husarion_ugv_msgs__msg__FaultFlag__copy(
  const husarion_ugv_msgs__msg__FaultFlag * input,
  husarion_ugv_msgs__msg__FaultFlag * output)
{
  if (!input || !output) {
    return false;
  }
  // overheat
  output->overheat = input->overheat;
  // overvoltage
  output->overvoltage = input->overvoltage;
  // undervoltage
  output->undervoltage = input->undervoltage;
  // short_circuit
  output->short_circuit = input->short_circuit;
  // emergency_stop
  output->emergency_stop = input->emergency_stop;
  // motor_or_sensor_setup_fault
  output->motor_or_sensor_setup_fault = input->motor_or_sensor_setup_fault;
  // mosfet_failure
  output->mosfet_failure = input->mosfet_failure;
  // default_config_loaded_at_startup
  output->default_config_loaded_at_startup = input->default_config_loaded_at_startup;
  return true;
}

husarion_ugv_msgs__msg__FaultFlag *
husarion_ugv_msgs__msg__FaultFlag__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  husarion_ugv_msgs__msg__FaultFlag * msg = (husarion_ugv_msgs__msg__FaultFlag *)allocator.allocate(sizeof(husarion_ugv_msgs__msg__FaultFlag), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(husarion_ugv_msgs__msg__FaultFlag));
  bool success = husarion_ugv_msgs__msg__FaultFlag__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
husarion_ugv_msgs__msg__FaultFlag__destroy(husarion_ugv_msgs__msg__FaultFlag * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    husarion_ugv_msgs__msg__FaultFlag__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
husarion_ugv_msgs__msg__FaultFlag__Sequence__init(husarion_ugv_msgs__msg__FaultFlag__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  husarion_ugv_msgs__msg__FaultFlag * data = NULL;

  if (size) {
    data = (husarion_ugv_msgs__msg__FaultFlag *)allocator.zero_allocate(size, sizeof(husarion_ugv_msgs__msg__FaultFlag), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = husarion_ugv_msgs__msg__FaultFlag__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        husarion_ugv_msgs__msg__FaultFlag__fini(&data[i - 1]);
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
husarion_ugv_msgs__msg__FaultFlag__Sequence__fini(husarion_ugv_msgs__msg__FaultFlag__Sequence * array)
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
      husarion_ugv_msgs__msg__FaultFlag__fini(&array->data[i]);
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

husarion_ugv_msgs__msg__FaultFlag__Sequence *
husarion_ugv_msgs__msg__FaultFlag__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  husarion_ugv_msgs__msg__FaultFlag__Sequence * array = (husarion_ugv_msgs__msg__FaultFlag__Sequence *)allocator.allocate(sizeof(husarion_ugv_msgs__msg__FaultFlag__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = husarion_ugv_msgs__msg__FaultFlag__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
husarion_ugv_msgs__msg__FaultFlag__Sequence__destroy(husarion_ugv_msgs__msg__FaultFlag__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    husarion_ugv_msgs__msg__FaultFlag__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
husarion_ugv_msgs__msg__FaultFlag__Sequence__are_equal(const husarion_ugv_msgs__msg__FaultFlag__Sequence * lhs, const husarion_ugv_msgs__msg__FaultFlag__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!husarion_ugv_msgs__msg__FaultFlag__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
husarion_ugv_msgs__msg__FaultFlag__Sequence__copy(
  const husarion_ugv_msgs__msg__FaultFlag__Sequence * input,
  husarion_ugv_msgs__msg__FaultFlag__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(husarion_ugv_msgs__msg__FaultFlag);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    husarion_ugv_msgs__msg__FaultFlag * data =
      (husarion_ugv_msgs__msg__FaultFlag *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!husarion_ugv_msgs__msg__FaultFlag__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          husarion_ugv_msgs__msg__FaultFlag__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!husarion_ugv_msgs__msg__FaultFlag__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
