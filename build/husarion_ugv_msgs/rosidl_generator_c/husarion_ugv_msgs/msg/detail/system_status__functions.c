// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from husarion_ugv_msgs:msg/SystemStatus.idl
// generated code does not contain a copyright notice
#include "husarion_ugv_msgs/msg/detail/system_status__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"
// Member `cpu_percent`
#include "rosidl_runtime_c/primitives_sequence_functions.h"

bool
husarion_ugv_msgs__msg__SystemStatus__init(husarion_ugv_msgs__msg__SystemStatus * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    husarion_ugv_msgs__msg__SystemStatus__fini(msg);
    return false;
  }
  // cpu_percent
  if (!rosidl_runtime_c__float__Sequence__init(&msg->cpu_percent, 0)) {
    husarion_ugv_msgs__msg__SystemStatus__fini(msg);
    return false;
  }
  // cpu_temp
  // avg_load_percent
  // ram_usage_percent
  // disc_usage_percent
  return true;
}

void
husarion_ugv_msgs__msg__SystemStatus__fini(husarion_ugv_msgs__msg__SystemStatus * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // cpu_percent
  rosidl_runtime_c__float__Sequence__fini(&msg->cpu_percent);
  // cpu_temp
  // avg_load_percent
  // ram_usage_percent
  // disc_usage_percent
}

bool
husarion_ugv_msgs__msg__SystemStatus__are_equal(const husarion_ugv_msgs__msg__SystemStatus * lhs, const husarion_ugv_msgs__msg__SystemStatus * rhs)
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
  // cpu_percent
  if (!rosidl_runtime_c__float__Sequence__are_equal(
      &(lhs->cpu_percent), &(rhs->cpu_percent)))
  {
    return false;
  }
  // cpu_temp
  if (lhs->cpu_temp != rhs->cpu_temp) {
    return false;
  }
  // avg_load_percent
  if (lhs->avg_load_percent != rhs->avg_load_percent) {
    return false;
  }
  // ram_usage_percent
  if (lhs->ram_usage_percent != rhs->ram_usage_percent) {
    return false;
  }
  // disc_usage_percent
  if (lhs->disc_usage_percent != rhs->disc_usage_percent) {
    return false;
  }
  return true;
}

bool
husarion_ugv_msgs__msg__SystemStatus__copy(
  const husarion_ugv_msgs__msg__SystemStatus * input,
  husarion_ugv_msgs__msg__SystemStatus * output)
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
  // cpu_percent
  if (!rosidl_runtime_c__float__Sequence__copy(
      &(input->cpu_percent), &(output->cpu_percent)))
  {
    return false;
  }
  // cpu_temp
  output->cpu_temp = input->cpu_temp;
  // avg_load_percent
  output->avg_load_percent = input->avg_load_percent;
  // ram_usage_percent
  output->ram_usage_percent = input->ram_usage_percent;
  // disc_usage_percent
  output->disc_usage_percent = input->disc_usage_percent;
  return true;
}

husarion_ugv_msgs__msg__SystemStatus *
husarion_ugv_msgs__msg__SystemStatus__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  husarion_ugv_msgs__msg__SystemStatus * msg = (husarion_ugv_msgs__msg__SystemStatus *)allocator.allocate(sizeof(husarion_ugv_msgs__msg__SystemStatus), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(husarion_ugv_msgs__msg__SystemStatus));
  bool success = husarion_ugv_msgs__msg__SystemStatus__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
husarion_ugv_msgs__msg__SystemStatus__destroy(husarion_ugv_msgs__msg__SystemStatus * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    husarion_ugv_msgs__msg__SystemStatus__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
husarion_ugv_msgs__msg__SystemStatus__Sequence__init(husarion_ugv_msgs__msg__SystemStatus__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  husarion_ugv_msgs__msg__SystemStatus * data = NULL;

  if (size) {
    data = (husarion_ugv_msgs__msg__SystemStatus *)allocator.zero_allocate(size, sizeof(husarion_ugv_msgs__msg__SystemStatus), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = husarion_ugv_msgs__msg__SystemStatus__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        husarion_ugv_msgs__msg__SystemStatus__fini(&data[i - 1]);
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
husarion_ugv_msgs__msg__SystemStatus__Sequence__fini(husarion_ugv_msgs__msg__SystemStatus__Sequence * array)
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
      husarion_ugv_msgs__msg__SystemStatus__fini(&array->data[i]);
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

husarion_ugv_msgs__msg__SystemStatus__Sequence *
husarion_ugv_msgs__msg__SystemStatus__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  husarion_ugv_msgs__msg__SystemStatus__Sequence * array = (husarion_ugv_msgs__msg__SystemStatus__Sequence *)allocator.allocate(sizeof(husarion_ugv_msgs__msg__SystemStatus__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = husarion_ugv_msgs__msg__SystemStatus__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
husarion_ugv_msgs__msg__SystemStatus__Sequence__destroy(husarion_ugv_msgs__msg__SystemStatus__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    husarion_ugv_msgs__msg__SystemStatus__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
husarion_ugv_msgs__msg__SystemStatus__Sequence__are_equal(const husarion_ugv_msgs__msg__SystemStatus__Sequence * lhs, const husarion_ugv_msgs__msg__SystemStatus__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!husarion_ugv_msgs__msg__SystemStatus__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
husarion_ugv_msgs__msg__SystemStatus__Sequence__copy(
  const husarion_ugv_msgs__msg__SystemStatus__Sequence * input,
  husarion_ugv_msgs__msg__SystemStatus__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(husarion_ugv_msgs__msg__SystemStatus);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    husarion_ugv_msgs__msg__SystemStatus * data =
      (husarion_ugv_msgs__msg__SystemStatus *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!husarion_ugv_msgs__msg__SystemStatus__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          husarion_ugv_msgs__msg__SystemStatus__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!husarion_ugv_msgs__msg__SystemStatus__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
