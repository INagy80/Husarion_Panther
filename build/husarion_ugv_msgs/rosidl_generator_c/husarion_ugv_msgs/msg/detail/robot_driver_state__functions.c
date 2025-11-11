// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from husarion_ugv_msgs:msg/RobotDriverState.idl
// generated code does not contain a copyright notice
#include "husarion_ugv_msgs/msg/detail/robot_driver_state__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"
// Member `driver_states`
#include "husarion_ugv_msgs/msg/detail/driver_state_named__functions.h"

bool
husarion_ugv_msgs__msg__RobotDriverState__init(husarion_ugv_msgs__msg__RobotDriverState * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    husarion_ugv_msgs__msg__RobotDriverState__fini(msg);
    return false;
  }
  // driver_states
  if (!husarion_ugv_msgs__msg__DriverStateNamed__Sequence__init(&msg->driver_states, 0)) {
    husarion_ugv_msgs__msg__RobotDriverState__fini(msg);
    return false;
  }
  // error
  // write_pdo_cmds_error
  // read_pdo_motor_states_error
  // read_pdo_driver_state_error
  return true;
}

void
husarion_ugv_msgs__msg__RobotDriverState__fini(husarion_ugv_msgs__msg__RobotDriverState * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // driver_states
  husarion_ugv_msgs__msg__DriverStateNamed__Sequence__fini(&msg->driver_states);
  // error
  // write_pdo_cmds_error
  // read_pdo_motor_states_error
  // read_pdo_driver_state_error
}

bool
husarion_ugv_msgs__msg__RobotDriverState__are_equal(const husarion_ugv_msgs__msg__RobotDriverState * lhs, const husarion_ugv_msgs__msg__RobotDriverState * rhs)
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
  // driver_states
  if (!husarion_ugv_msgs__msg__DriverStateNamed__Sequence__are_equal(
      &(lhs->driver_states), &(rhs->driver_states)))
  {
    return false;
  }
  // error
  if (lhs->error != rhs->error) {
    return false;
  }
  // write_pdo_cmds_error
  if (lhs->write_pdo_cmds_error != rhs->write_pdo_cmds_error) {
    return false;
  }
  // read_pdo_motor_states_error
  if (lhs->read_pdo_motor_states_error != rhs->read_pdo_motor_states_error) {
    return false;
  }
  // read_pdo_driver_state_error
  if (lhs->read_pdo_driver_state_error != rhs->read_pdo_driver_state_error) {
    return false;
  }
  return true;
}

bool
husarion_ugv_msgs__msg__RobotDriverState__copy(
  const husarion_ugv_msgs__msg__RobotDriverState * input,
  husarion_ugv_msgs__msg__RobotDriverState * output)
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
  // driver_states
  if (!husarion_ugv_msgs__msg__DriverStateNamed__Sequence__copy(
      &(input->driver_states), &(output->driver_states)))
  {
    return false;
  }
  // error
  output->error = input->error;
  // write_pdo_cmds_error
  output->write_pdo_cmds_error = input->write_pdo_cmds_error;
  // read_pdo_motor_states_error
  output->read_pdo_motor_states_error = input->read_pdo_motor_states_error;
  // read_pdo_driver_state_error
  output->read_pdo_driver_state_error = input->read_pdo_driver_state_error;
  return true;
}

husarion_ugv_msgs__msg__RobotDriverState *
husarion_ugv_msgs__msg__RobotDriverState__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  husarion_ugv_msgs__msg__RobotDriverState * msg = (husarion_ugv_msgs__msg__RobotDriverState *)allocator.allocate(sizeof(husarion_ugv_msgs__msg__RobotDriverState), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(husarion_ugv_msgs__msg__RobotDriverState));
  bool success = husarion_ugv_msgs__msg__RobotDriverState__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
husarion_ugv_msgs__msg__RobotDriverState__destroy(husarion_ugv_msgs__msg__RobotDriverState * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    husarion_ugv_msgs__msg__RobotDriverState__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
husarion_ugv_msgs__msg__RobotDriverState__Sequence__init(husarion_ugv_msgs__msg__RobotDriverState__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  husarion_ugv_msgs__msg__RobotDriverState * data = NULL;

  if (size) {
    data = (husarion_ugv_msgs__msg__RobotDriverState *)allocator.zero_allocate(size, sizeof(husarion_ugv_msgs__msg__RobotDriverState), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = husarion_ugv_msgs__msg__RobotDriverState__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        husarion_ugv_msgs__msg__RobotDriverState__fini(&data[i - 1]);
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
husarion_ugv_msgs__msg__RobotDriverState__Sequence__fini(husarion_ugv_msgs__msg__RobotDriverState__Sequence * array)
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
      husarion_ugv_msgs__msg__RobotDriverState__fini(&array->data[i]);
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

husarion_ugv_msgs__msg__RobotDriverState__Sequence *
husarion_ugv_msgs__msg__RobotDriverState__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  husarion_ugv_msgs__msg__RobotDriverState__Sequence * array = (husarion_ugv_msgs__msg__RobotDriverState__Sequence *)allocator.allocate(sizeof(husarion_ugv_msgs__msg__RobotDriverState__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = husarion_ugv_msgs__msg__RobotDriverState__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
husarion_ugv_msgs__msg__RobotDriverState__Sequence__destroy(husarion_ugv_msgs__msg__RobotDriverState__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    husarion_ugv_msgs__msg__RobotDriverState__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
husarion_ugv_msgs__msg__RobotDriverState__Sequence__are_equal(const husarion_ugv_msgs__msg__RobotDriverState__Sequence * lhs, const husarion_ugv_msgs__msg__RobotDriverState__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!husarion_ugv_msgs__msg__RobotDriverState__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
husarion_ugv_msgs__msg__RobotDriverState__Sequence__copy(
  const husarion_ugv_msgs__msg__RobotDriverState__Sequence * input,
  husarion_ugv_msgs__msg__RobotDriverState__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(husarion_ugv_msgs__msg__RobotDriverState);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    husarion_ugv_msgs__msg__RobotDriverState * data =
      (husarion_ugv_msgs__msg__RobotDriverState *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!husarion_ugv_msgs__msg__RobotDriverState__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          husarion_ugv_msgs__msg__RobotDriverState__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!husarion_ugv_msgs__msg__RobotDriverState__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
