// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from husarion_ugv_msgs:msg/ChargingStatus.idl
// generated code does not contain a copyright notice
#include "husarion_ugv_msgs/msg/detail/charging_status__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"

bool
husarion_ugv_msgs__msg__ChargingStatus__init(husarion_ugv_msgs__msg__ChargingStatus * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    husarion_ugv_msgs__msg__ChargingStatus__fini(msg);
    return false;
  }
  // charging
  // current
  // current_battery_1
  // current_battery_2
  // charger_type
  return true;
}

void
husarion_ugv_msgs__msg__ChargingStatus__fini(husarion_ugv_msgs__msg__ChargingStatus * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // charging
  // current
  // current_battery_1
  // current_battery_2
  // charger_type
}

bool
husarion_ugv_msgs__msg__ChargingStatus__are_equal(const husarion_ugv_msgs__msg__ChargingStatus * lhs, const husarion_ugv_msgs__msg__ChargingStatus * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__are_equal(
      &(lhs->header), &(rhs->header)))
  {
    return false;
  }
  // charging
  if (lhs->charging != rhs->charging) {
    return false;
  }
  // current
  if (lhs->current != rhs->current) {
    return false;
  }
  // current_battery_1
  if (lhs->current_battery_1 != rhs->current_battery_1) {
    return false;
  }
  // current_battery_2
  if (lhs->current_battery_2 != rhs->current_battery_2) {
    return false;
  }
  // charger_type
  if (lhs->charger_type != rhs->charger_type) {
    return false;
  }
  return true;
}

bool
husarion_ugv_msgs__msg__ChargingStatus__copy(
  const husarion_ugv_msgs__msg__ChargingStatus * input,
  husarion_ugv_msgs__msg__ChargingStatus * output)
{
  if (!input || !output) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__copy(
      &(input->header), &(output->header)))
  {
    return false;
  }
  // charging
  output->charging = input->charging;
  // current
  output->current = input->current;
  // current_battery_1
  output->current_battery_1 = input->current_battery_1;
  // current_battery_2
  output->current_battery_2 = input->current_battery_2;
  // charger_type
  output->charger_type = input->charger_type;
  return true;
}

husarion_ugv_msgs__msg__ChargingStatus *
husarion_ugv_msgs__msg__ChargingStatus__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  husarion_ugv_msgs__msg__ChargingStatus * msg = (husarion_ugv_msgs__msg__ChargingStatus *)allocator.allocate(sizeof(husarion_ugv_msgs__msg__ChargingStatus), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(husarion_ugv_msgs__msg__ChargingStatus));
  bool success = husarion_ugv_msgs__msg__ChargingStatus__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
husarion_ugv_msgs__msg__ChargingStatus__destroy(husarion_ugv_msgs__msg__ChargingStatus * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    husarion_ugv_msgs__msg__ChargingStatus__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
husarion_ugv_msgs__msg__ChargingStatus__Sequence__init(husarion_ugv_msgs__msg__ChargingStatus__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  husarion_ugv_msgs__msg__ChargingStatus * data = NULL;

  if (size) {
    data = (husarion_ugv_msgs__msg__ChargingStatus *)allocator.zero_allocate(size, sizeof(husarion_ugv_msgs__msg__ChargingStatus), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = husarion_ugv_msgs__msg__ChargingStatus__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        husarion_ugv_msgs__msg__ChargingStatus__fini(&data[i - 1]);
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
husarion_ugv_msgs__msg__ChargingStatus__Sequence__fini(husarion_ugv_msgs__msg__ChargingStatus__Sequence * array)
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
      husarion_ugv_msgs__msg__ChargingStatus__fini(&array->data[i]);
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

husarion_ugv_msgs__msg__ChargingStatus__Sequence *
husarion_ugv_msgs__msg__ChargingStatus__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  husarion_ugv_msgs__msg__ChargingStatus__Sequence * array = (husarion_ugv_msgs__msg__ChargingStatus__Sequence *)allocator.allocate(sizeof(husarion_ugv_msgs__msg__ChargingStatus__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = husarion_ugv_msgs__msg__ChargingStatus__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
husarion_ugv_msgs__msg__ChargingStatus__Sequence__destroy(husarion_ugv_msgs__msg__ChargingStatus__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    husarion_ugv_msgs__msg__ChargingStatus__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
husarion_ugv_msgs__msg__ChargingStatus__Sequence__are_equal(const husarion_ugv_msgs__msg__ChargingStatus__Sequence * lhs, const husarion_ugv_msgs__msg__ChargingStatus__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!husarion_ugv_msgs__msg__ChargingStatus__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
husarion_ugv_msgs__msg__ChargingStatus__Sequence__copy(
  const husarion_ugv_msgs__msg__ChargingStatus__Sequence * input,
  husarion_ugv_msgs__msg__ChargingStatus__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(husarion_ugv_msgs__msg__ChargingStatus);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    husarion_ugv_msgs__msg__ChargingStatus * data =
      (husarion_ugv_msgs__msg__ChargingStatus *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!husarion_ugv_msgs__msg__ChargingStatus__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          husarion_ugv_msgs__msg__ChargingStatus__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!husarion_ugv_msgs__msg__ChargingStatus__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
