// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from husarion_ugv_msgs:msg/DriverStateNamed.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "husarion_ugv_msgs/msg/driver_state_named.h"


#ifndef HUSARION_UGV_MSGS__MSG__DETAIL__DRIVER_STATE_NAMED__STRUCT_H_
#define HUSARION_UGV_MSGS__MSG__DETAIL__DRIVER_STATE_NAMED__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// Constants defined in the message

/// Constant 'NAME_DEFAULT'.
static const char * const husarion_ugv_msgs__msg__DriverStateNamed__NAME_DEFAULT = "default";

/// Constant 'NAME_FRONT'.
static const char * const husarion_ugv_msgs__msg__DriverStateNamed__NAME_FRONT = "front";

/// Constant 'NAME_REAR'.
static const char * const husarion_ugv_msgs__msg__DriverStateNamed__NAME_REAR = "rear";

// Include directives for member types
// Member 'name'
#include "rosidl_runtime_c/string.h"
// Member 'state'
#include "husarion_ugv_msgs/msg/detail/driver_state__struct.h"

/// Struct defined in msg/DriverStateNamed in the package husarion_ugv_msgs.
typedef struct husarion_ugv_msgs__msg__DriverStateNamed
{
  rosidl_runtime_c__String name;
  husarion_ugv_msgs__msg__DriverState state;
} husarion_ugv_msgs__msg__DriverStateNamed;

// Struct for a sequence of husarion_ugv_msgs__msg__DriverStateNamed.
typedef struct husarion_ugv_msgs__msg__DriverStateNamed__Sequence
{
  husarion_ugv_msgs__msg__DriverStateNamed * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} husarion_ugv_msgs__msg__DriverStateNamed__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // HUSARION_UGV_MSGS__MSG__DETAIL__DRIVER_STATE_NAMED__STRUCT_H_
