// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from husarion_ugv_msgs:srv/SetLEDImageAnimation.idl
// generated code does not contain a copyright notice
#include "husarion_ugv_msgs/srv/detail/set_led_image_animation__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"

// Include directives for member types
// Member `front`
// Member `rear`
#include "husarion_ugv_msgs/msg/detail/led_image_animation__functions.h"

bool
husarion_ugv_msgs__srv__SetLEDImageAnimation_Request__init(husarion_ugv_msgs__srv__SetLEDImageAnimation_Request * msg)
{
  if (!msg) {
    return false;
  }
  // front
  if (!husarion_ugv_msgs__msg__LEDImageAnimation__init(&msg->front)) {
    husarion_ugv_msgs__srv__SetLEDImageAnimation_Request__fini(msg);
    return false;
  }
  // rear
  if (!husarion_ugv_msgs__msg__LEDImageAnimation__init(&msg->rear)) {
    husarion_ugv_msgs__srv__SetLEDImageAnimation_Request__fini(msg);
    return false;
  }
  // interrupting
  // repeating
  return true;
}

void
husarion_ugv_msgs__srv__SetLEDImageAnimation_Request__fini(husarion_ugv_msgs__srv__SetLEDImageAnimation_Request * msg)
{
  if (!msg) {
    return;
  }
  // front
  husarion_ugv_msgs__msg__LEDImageAnimation__fini(&msg->front);
  // rear
  husarion_ugv_msgs__msg__LEDImageAnimation__fini(&msg->rear);
  // interrupting
  // repeating
}

bool
husarion_ugv_msgs__srv__SetLEDImageAnimation_Request__are_equal(const husarion_ugv_msgs__srv__SetLEDImageAnimation_Request * lhs, const husarion_ugv_msgs__srv__SetLEDImageAnimation_Request * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // front
  if (!husarion_ugv_msgs__msg__LEDImageAnimation__are_equal(
      &(lhs->front), &(rhs->front)))
  {
    return false;
  }
  // rear
  if (!husarion_ugv_msgs__msg__LEDImageAnimation__are_equal(
      &(lhs->rear), &(rhs->rear)))
  {
    return false;
  }
  // interrupting
  if (lhs->interrupting != rhs->interrupting) {
    return false;
  }
  // repeating
  if (lhs->repeating != rhs->repeating) {
    return false;
  }
  return true;
}

bool
husarion_ugv_msgs__srv__SetLEDImageAnimation_Request__copy(
  const husarion_ugv_msgs__srv__SetLEDImageAnimation_Request * input,
  husarion_ugv_msgs__srv__SetLEDImageAnimation_Request * output)
{
  if (!input || !output) {
    return false;
  }
  // front
  if (!husarion_ugv_msgs__msg__LEDImageAnimation__copy(
      &(input->front), &(output->front)))
  {
    return false;
  }
  // rear
  if (!husarion_ugv_msgs__msg__LEDImageAnimation__copy(
      &(input->rear), &(output->rear)))
  {
    return false;
  }
  // interrupting
  output->interrupting = input->interrupting;
  // repeating
  output->repeating = input->repeating;
  return true;
}

husarion_ugv_msgs__srv__SetLEDImageAnimation_Request *
husarion_ugv_msgs__srv__SetLEDImageAnimation_Request__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  husarion_ugv_msgs__srv__SetLEDImageAnimation_Request * msg = (husarion_ugv_msgs__srv__SetLEDImageAnimation_Request *)allocator.allocate(sizeof(husarion_ugv_msgs__srv__SetLEDImageAnimation_Request), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(husarion_ugv_msgs__srv__SetLEDImageAnimation_Request));
  bool success = husarion_ugv_msgs__srv__SetLEDImageAnimation_Request__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
husarion_ugv_msgs__srv__SetLEDImageAnimation_Request__destroy(husarion_ugv_msgs__srv__SetLEDImageAnimation_Request * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    husarion_ugv_msgs__srv__SetLEDImageAnimation_Request__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
husarion_ugv_msgs__srv__SetLEDImageAnimation_Request__Sequence__init(husarion_ugv_msgs__srv__SetLEDImageAnimation_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  husarion_ugv_msgs__srv__SetLEDImageAnimation_Request * data = NULL;

  if (size) {
    data = (husarion_ugv_msgs__srv__SetLEDImageAnimation_Request *)allocator.zero_allocate(size, sizeof(husarion_ugv_msgs__srv__SetLEDImageAnimation_Request), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = husarion_ugv_msgs__srv__SetLEDImageAnimation_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        husarion_ugv_msgs__srv__SetLEDImageAnimation_Request__fini(&data[i - 1]);
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
husarion_ugv_msgs__srv__SetLEDImageAnimation_Request__Sequence__fini(husarion_ugv_msgs__srv__SetLEDImageAnimation_Request__Sequence * array)
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
      husarion_ugv_msgs__srv__SetLEDImageAnimation_Request__fini(&array->data[i]);
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

husarion_ugv_msgs__srv__SetLEDImageAnimation_Request__Sequence *
husarion_ugv_msgs__srv__SetLEDImageAnimation_Request__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  husarion_ugv_msgs__srv__SetLEDImageAnimation_Request__Sequence * array = (husarion_ugv_msgs__srv__SetLEDImageAnimation_Request__Sequence *)allocator.allocate(sizeof(husarion_ugv_msgs__srv__SetLEDImageAnimation_Request__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = husarion_ugv_msgs__srv__SetLEDImageAnimation_Request__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
husarion_ugv_msgs__srv__SetLEDImageAnimation_Request__Sequence__destroy(husarion_ugv_msgs__srv__SetLEDImageAnimation_Request__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    husarion_ugv_msgs__srv__SetLEDImageAnimation_Request__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
husarion_ugv_msgs__srv__SetLEDImageAnimation_Request__Sequence__are_equal(const husarion_ugv_msgs__srv__SetLEDImageAnimation_Request__Sequence * lhs, const husarion_ugv_msgs__srv__SetLEDImageAnimation_Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!husarion_ugv_msgs__srv__SetLEDImageAnimation_Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
husarion_ugv_msgs__srv__SetLEDImageAnimation_Request__Sequence__copy(
  const husarion_ugv_msgs__srv__SetLEDImageAnimation_Request__Sequence * input,
  husarion_ugv_msgs__srv__SetLEDImageAnimation_Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(husarion_ugv_msgs__srv__SetLEDImageAnimation_Request);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    husarion_ugv_msgs__srv__SetLEDImageAnimation_Request * data =
      (husarion_ugv_msgs__srv__SetLEDImageAnimation_Request *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!husarion_ugv_msgs__srv__SetLEDImageAnimation_Request__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          husarion_ugv_msgs__srv__SetLEDImageAnimation_Request__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!husarion_ugv_msgs__srv__SetLEDImageAnimation_Request__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `message`
#include "rosidl_runtime_c/string_functions.h"

bool
husarion_ugv_msgs__srv__SetLEDImageAnimation_Response__init(husarion_ugv_msgs__srv__SetLEDImageAnimation_Response * msg)
{
  if (!msg) {
    return false;
  }
  // success
  // message
  if (!rosidl_runtime_c__String__init(&msg->message)) {
    husarion_ugv_msgs__srv__SetLEDImageAnimation_Response__fini(msg);
    return false;
  }
  return true;
}

void
husarion_ugv_msgs__srv__SetLEDImageAnimation_Response__fini(husarion_ugv_msgs__srv__SetLEDImageAnimation_Response * msg)
{
  if (!msg) {
    return;
  }
  // success
  // message
  rosidl_runtime_c__String__fini(&msg->message);
}

bool
husarion_ugv_msgs__srv__SetLEDImageAnimation_Response__are_equal(const husarion_ugv_msgs__srv__SetLEDImageAnimation_Response * lhs, const husarion_ugv_msgs__srv__SetLEDImageAnimation_Response * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // success
  if (lhs->success != rhs->success) {
    return false;
  }
  // message
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->message), &(rhs->message)))
  {
    return false;
  }
  return true;
}

bool
husarion_ugv_msgs__srv__SetLEDImageAnimation_Response__copy(
  const husarion_ugv_msgs__srv__SetLEDImageAnimation_Response * input,
  husarion_ugv_msgs__srv__SetLEDImageAnimation_Response * output)
{
  if (!input || !output) {
    return false;
  }
  // success
  output->success = input->success;
  // message
  if (!rosidl_runtime_c__String__copy(
      &(input->message), &(output->message)))
  {
    return false;
  }
  return true;
}

husarion_ugv_msgs__srv__SetLEDImageAnimation_Response *
husarion_ugv_msgs__srv__SetLEDImageAnimation_Response__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  husarion_ugv_msgs__srv__SetLEDImageAnimation_Response * msg = (husarion_ugv_msgs__srv__SetLEDImageAnimation_Response *)allocator.allocate(sizeof(husarion_ugv_msgs__srv__SetLEDImageAnimation_Response), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(husarion_ugv_msgs__srv__SetLEDImageAnimation_Response));
  bool success = husarion_ugv_msgs__srv__SetLEDImageAnimation_Response__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
husarion_ugv_msgs__srv__SetLEDImageAnimation_Response__destroy(husarion_ugv_msgs__srv__SetLEDImageAnimation_Response * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    husarion_ugv_msgs__srv__SetLEDImageAnimation_Response__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
husarion_ugv_msgs__srv__SetLEDImageAnimation_Response__Sequence__init(husarion_ugv_msgs__srv__SetLEDImageAnimation_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  husarion_ugv_msgs__srv__SetLEDImageAnimation_Response * data = NULL;

  if (size) {
    data = (husarion_ugv_msgs__srv__SetLEDImageAnimation_Response *)allocator.zero_allocate(size, sizeof(husarion_ugv_msgs__srv__SetLEDImageAnimation_Response), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = husarion_ugv_msgs__srv__SetLEDImageAnimation_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        husarion_ugv_msgs__srv__SetLEDImageAnimation_Response__fini(&data[i - 1]);
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
husarion_ugv_msgs__srv__SetLEDImageAnimation_Response__Sequence__fini(husarion_ugv_msgs__srv__SetLEDImageAnimation_Response__Sequence * array)
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
      husarion_ugv_msgs__srv__SetLEDImageAnimation_Response__fini(&array->data[i]);
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

husarion_ugv_msgs__srv__SetLEDImageAnimation_Response__Sequence *
husarion_ugv_msgs__srv__SetLEDImageAnimation_Response__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  husarion_ugv_msgs__srv__SetLEDImageAnimation_Response__Sequence * array = (husarion_ugv_msgs__srv__SetLEDImageAnimation_Response__Sequence *)allocator.allocate(sizeof(husarion_ugv_msgs__srv__SetLEDImageAnimation_Response__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = husarion_ugv_msgs__srv__SetLEDImageAnimation_Response__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
husarion_ugv_msgs__srv__SetLEDImageAnimation_Response__Sequence__destroy(husarion_ugv_msgs__srv__SetLEDImageAnimation_Response__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    husarion_ugv_msgs__srv__SetLEDImageAnimation_Response__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
husarion_ugv_msgs__srv__SetLEDImageAnimation_Response__Sequence__are_equal(const husarion_ugv_msgs__srv__SetLEDImageAnimation_Response__Sequence * lhs, const husarion_ugv_msgs__srv__SetLEDImageAnimation_Response__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!husarion_ugv_msgs__srv__SetLEDImageAnimation_Response__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
husarion_ugv_msgs__srv__SetLEDImageAnimation_Response__Sequence__copy(
  const husarion_ugv_msgs__srv__SetLEDImageAnimation_Response__Sequence * input,
  husarion_ugv_msgs__srv__SetLEDImageAnimation_Response__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(husarion_ugv_msgs__srv__SetLEDImageAnimation_Response);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    husarion_ugv_msgs__srv__SetLEDImageAnimation_Response * data =
      (husarion_ugv_msgs__srv__SetLEDImageAnimation_Response *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!husarion_ugv_msgs__srv__SetLEDImageAnimation_Response__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          husarion_ugv_msgs__srv__SetLEDImageAnimation_Response__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!husarion_ugv_msgs__srv__SetLEDImageAnimation_Response__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `info`
#include "service_msgs/msg/detail/service_event_info__functions.h"
// Member `request`
// Member `response`
// already included above
// #include "husarion_ugv_msgs/srv/detail/set_led_image_animation__functions.h"

bool
husarion_ugv_msgs__srv__SetLEDImageAnimation_Event__init(husarion_ugv_msgs__srv__SetLEDImageAnimation_Event * msg)
{
  if (!msg) {
    return false;
  }
  // info
  if (!service_msgs__msg__ServiceEventInfo__init(&msg->info)) {
    husarion_ugv_msgs__srv__SetLEDImageAnimation_Event__fini(msg);
    return false;
  }
  // request
  if (!husarion_ugv_msgs__srv__SetLEDImageAnimation_Request__Sequence__init(&msg->request, 0)) {
    husarion_ugv_msgs__srv__SetLEDImageAnimation_Event__fini(msg);
    return false;
  }
  // response
  if (!husarion_ugv_msgs__srv__SetLEDImageAnimation_Response__Sequence__init(&msg->response, 0)) {
    husarion_ugv_msgs__srv__SetLEDImageAnimation_Event__fini(msg);
    return false;
  }
  return true;
}

void
husarion_ugv_msgs__srv__SetLEDImageAnimation_Event__fini(husarion_ugv_msgs__srv__SetLEDImageAnimation_Event * msg)
{
  if (!msg) {
    return;
  }
  // info
  service_msgs__msg__ServiceEventInfo__fini(&msg->info);
  // request
  husarion_ugv_msgs__srv__SetLEDImageAnimation_Request__Sequence__fini(&msg->request);
  // response
  husarion_ugv_msgs__srv__SetLEDImageAnimation_Response__Sequence__fini(&msg->response);
}

bool
husarion_ugv_msgs__srv__SetLEDImageAnimation_Event__are_equal(const husarion_ugv_msgs__srv__SetLEDImageAnimation_Event * lhs, const husarion_ugv_msgs__srv__SetLEDImageAnimation_Event * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // info
  if (!service_msgs__msg__ServiceEventInfo__are_equal(
      &(lhs->info), &(rhs->info)))
  {
    return false;
  }
  // request
  if (!husarion_ugv_msgs__srv__SetLEDImageAnimation_Request__Sequence__are_equal(
      &(lhs->request), &(rhs->request)))
  {
    return false;
  }
  // response
  if (!husarion_ugv_msgs__srv__SetLEDImageAnimation_Response__Sequence__are_equal(
      &(lhs->response), &(rhs->response)))
  {
    return false;
  }
  return true;
}

bool
husarion_ugv_msgs__srv__SetLEDImageAnimation_Event__copy(
  const husarion_ugv_msgs__srv__SetLEDImageAnimation_Event * input,
  husarion_ugv_msgs__srv__SetLEDImageAnimation_Event * output)
{
  if (!input || !output) {
    return false;
  }
  // info
  if (!service_msgs__msg__ServiceEventInfo__copy(
      &(input->info), &(output->info)))
  {
    return false;
  }
  // request
  if (!husarion_ugv_msgs__srv__SetLEDImageAnimation_Request__Sequence__copy(
      &(input->request), &(output->request)))
  {
    return false;
  }
  // response
  if (!husarion_ugv_msgs__srv__SetLEDImageAnimation_Response__Sequence__copy(
      &(input->response), &(output->response)))
  {
    return false;
  }
  return true;
}

husarion_ugv_msgs__srv__SetLEDImageAnimation_Event *
husarion_ugv_msgs__srv__SetLEDImageAnimation_Event__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  husarion_ugv_msgs__srv__SetLEDImageAnimation_Event * msg = (husarion_ugv_msgs__srv__SetLEDImageAnimation_Event *)allocator.allocate(sizeof(husarion_ugv_msgs__srv__SetLEDImageAnimation_Event), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(husarion_ugv_msgs__srv__SetLEDImageAnimation_Event));
  bool success = husarion_ugv_msgs__srv__SetLEDImageAnimation_Event__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
husarion_ugv_msgs__srv__SetLEDImageAnimation_Event__destroy(husarion_ugv_msgs__srv__SetLEDImageAnimation_Event * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    husarion_ugv_msgs__srv__SetLEDImageAnimation_Event__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
husarion_ugv_msgs__srv__SetLEDImageAnimation_Event__Sequence__init(husarion_ugv_msgs__srv__SetLEDImageAnimation_Event__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  husarion_ugv_msgs__srv__SetLEDImageAnimation_Event * data = NULL;

  if (size) {
    data = (husarion_ugv_msgs__srv__SetLEDImageAnimation_Event *)allocator.zero_allocate(size, sizeof(husarion_ugv_msgs__srv__SetLEDImageAnimation_Event), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = husarion_ugv_msgs__srv__SetLEDImageAnimation_Event__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        husarion_ugv_msgs__srv__SetLEDImageAnimation_Event__fini(&data[i - 1]);
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
husarion_ugv_msgs__srv__SetLEDImageAnimation_Event__Sequence__fini(husarion_ugv_msgs__srv__SetLEDImageAnimation_Event__Sequence * array)
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
      husarion_ugv_msgs__srv__SetLEDImageAnimation_Event__fini(&array->data[i]);
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

husarion_ugv_msgs__srv__SetLEDImageAnimation_Event__Sequence *
husarion_ugv_msgs__srv__SetLEDImageAnimation_Event__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  husarion_ugv_msgs__srv__SetLEDImageAnimation_Event__Sequence * array = (husarion_ugv_msgs__srv__SetLEDImageAnimation_Event__Sequence *)allocator.allocate(sizeof(husarion_ugv_msgs__srv__SetLEDImageAnimation_Event__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = husarion_ugv_msgs__srv__SetLEDImageAnimation_Event__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
husarion_ugv_msgs__srv__SetLEDImageAnimation_Event__Sequence__destroy(husarion_ugv_msgs__srv__SetLEDImageAnimation_Event__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    husarion_ugv_msgs__srv__SetLEDImageAnimation_Event__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
husarion_ugv_msgs__srv__SetLEDImageAnimation_Event__Sequence__are_equal(const husarion_ugv_msgs__srv__SetLEDImageAnimation_Event__Sequence * lhs, const husarion_ugv_msgs__srv__SetLEDImageAnimation_Event__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!husarion_ugv_msgs__srv__SetLEDImageAnimation_Event__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
husarion_ugv_msgs__srv__SetLEDImageAnimation_Event__Sequence__copy(
  const husarion_ugv_msgs__srv__SetLEDImageAnimation_Event__Sequence * input,
  husarion_ugv_msgs__srv__SetLEDImageAnimation_Event__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(husarion_ugv_msgs__srv__SetLEDImageAnimation_Event);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    husarion_ugv_msgs__srv__SetLEDImageAnimation_Event * data =
      (husarion_ugv_msgs__srv__SetLEDImageAnimation_Event *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!husarion_ugv_msgs__srv__SetLEDImageAnimation_Event__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          husarion_ugv_msgs__srv__SetLEDImageAnimation_Event__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!husarion_ugv_msgs__srv__SetLEDImageAnimation_Event__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
