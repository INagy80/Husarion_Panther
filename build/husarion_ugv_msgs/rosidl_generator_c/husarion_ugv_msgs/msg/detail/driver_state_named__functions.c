// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from husarion_ugv_msgs:msg/DriverStateNamed.idl
// generated code does not contain a copyright notice
#include "husarion_ugv_msgs/msg/detail/driver_state_named__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `name`
#include "rosidl_runtime_c/string_functions.h"
// Member `state`
#include "husarion_ugv_msgs/msg/detail/driver_state__functions.h"

bool
husarion_ugv_msgs__msg__DriverStateNamed__init(husarion_ugv_msgs__msg__DriverStateNamed * msg)
{
  if (!msg) {
    return false;
  }
  // name
  if (!rosidl_runtime_c__String__init(&msg->name)) {
    husarion_ugv_msgs__msg__DriverStateNamed__fini(msg);
    return false;
  }
  // state
  if (!husarion_ugv_msgs__msg__DriverState__init(&msg->state)) {
    husarion_ugv_msgs__msg__DriverStateNamed__fini(msg);
    return false;
  }
  return true;
}

void
husarion_ugv_msgs__msg__DriverStateNamed__fini(husarion_ugv_msgs__msg__DriverStateNamed * msg)
{
  if (!msg) {
    return;
  }
  // name
  rosidl_runtime_c__String__fini(&msg->name);
  // state
  husarion_ugv_msgs__msg__DriverState__fini(&msg->state);
}

bool
husarion_ugv_msgs__msg__DriverStateNamed__are_equal(const husarion_ugv_msgs__msg__DriverStateNamed * lhs, const husarion_ugv_msgs__msg__DriverStateNamed * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // name
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->name), &(rhs->name)))
  {
    return false;
  }
  // state
  if (!husarion_ugv_msgs__msg__DriverState__are_equal(
      &(lhs->state), &(rhs->state)))
  {
    return false;
  }
  return true;
}

bool
husarion_ugv_msgs__msg__DriverStateNamed__copy(
  const husarion_ugv_msgs__msg__DriverStateNamed * input,
  husarion_ugv_msgs__msg__DriverStateNamed * output)
{
  if (!input || !output) {
    return false;
  }
  // name
  if (!rosidl_runtime_c__String__copy(
      &(input->name), &(output->name)))
  {
    return false;
  }
  // state
  if (!husarion_ugv_msgs__msg__DriverState__copy(
      &(input->state), &(output->state)))
  {
    return false;
  }
  return true;
}

husarion_ugv_msgs__msg__DriverStateNamed *
husarion_ugv_msgs__msg__DriverStateNamed__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  husarion_ugv_msgs__msg__DriverStateNamed * msg = (husarion_ugv_msgs__msg__DriverStateNamed *)allocator.allocate(sizeof(husarion_ugv_msgs__msg__DriverStateNamed), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(husarion_ugv_msgs__msg__DriverStateNamed));
  bool success = husarion_ugv_msgs__msg__DriverStateNamed__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
husarion_ugv_msgs__msg__DriverStateNamed__destroy(husarion_ugv_msgs__msg__DriverStateNamed * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    husarion_ugv_msgs__msg__DriverStateNamed__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
husarion_ugv_msgs__msg__DriverStateNamed__Sequence__init(husarion_ugv_msgs__msg__DriverStateNamed__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  husarion_ugv_msgs__msg__DriverStateNamed * data = NULL;

  if (size) {
    data = (husarion_ugv_msgs__msg__DriverStateNamed *)allocator.zero_allocate(size, sizeof(husarion_ugv_msgs__msg__DriverStateNamed), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = husarion_ugv_msgs__msg__DriverStateNamed__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        husarion_ugv_msgs__msg__DriverStateNamed__fini(&data[i - 1]);
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
husarion_ugv_msgs__msg__DriverStateNamed__Sequence__fini(husarion_ugv_msgs__msg__DriverStateNamed__Sequence * array)
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
      husarion_ugv_msgs__msg__DriverStateNamed__fini(&array->data[i]);
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

husarion_ugv_msgs__msg__DriverStateNamed__Sequence *
husarion_ugv_msgs__msg__DriverStateNamed__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  husarion_ugv_msgs__msg__DriverStateNamed__Sequence * array = (husarion_ugv_msgs__msg__DriverStateNamed__Sequence *)allocator.allocate(sizeof(husarion_ugv_msgs__msg__DriverStateNamed__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = husarion_ugv_msgs__msg__DriverStateNamed__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
husarion_ugv_msgs__msg__DriverStateNamed__Sequence__destroy(husarion_ugv_msgs__msg__DriverStateNamed__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    husarion_ugv_msgs__msg__DriverStateNamed__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
husarion_ugv_msgs__msg__DriverStateNamed__Sequence__are_equal(const husarion_ugv_msgs__msg__DriverStateNamed__Sequence * lhs, const husarion_ugv_msgs__msg__DriverStateNamed__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!husarion_ugv_msgs__msg__DriverStateNamed__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
husarion_ugv_msgs__msg__DriverStateNamed__Sequence__copy(
  const husarion_ugv_msgs__msg__DriverStateNamed__Sequence * input,
  husarion_ugv_msgs__msg__DriverStateNamed__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(husarion_ugv_msgs__msg__DriverStateNamed);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    husarion_ugv_msgs__msg__DriverStateNamed * data =
      (husarion_ugv_msgs__msg__DriverStateNamed *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!husarion_ugv_msgs__msg__DriverStateNamed__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          husarion_ugv_msgs__msg__DriverStateNamed__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!husarion_ugv_msgs__msg__DriverStateNamed__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
