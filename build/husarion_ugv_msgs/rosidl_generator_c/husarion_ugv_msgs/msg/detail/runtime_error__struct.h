// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from husarion_ugv_msgs:msg/RuntimeError.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "husarion_ugv_msgs/msg/runtime_error.h"


#ifndef HUSARION_UGV_MSGS__MSG__DETAIL__RUNTIME_ERROR__STRUCT_H_
#define HUSARION_UGV_MSGS__MSG__DETAIL__RUNTIME_ERROR__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// Constants defined in the message

/// Struct defined in msg/RuntimeError in the package husarion_ugv_msgs.
typedef struct husarion_ugv_msgs__msg__RuntimeError
{
  bool amps_limit_active;
  bool motor_stall;
  bool loop_error;
  bool safety_stop_active;
  bool forward_limit_triggered;
  bool reverse_limit_triggered;
  bool amps_trigger_activated;
} husarion_ugv_msgs__msg__RuntimeError;

// Struct for a sequence of husarion_ugv_msgs__msg__RuntimeError.
typedef struct husarion_ugv_msgs__msg__RuntimeError__Sequence
{
  husarion_ugv_msgs__msg__RuntimeError * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} husarion_ugv_msgs__msg__RuntimeError__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // HUSARION_UGV_MSGS__MSG__DETAIL__RUNTIME_ERROR__STRUCT_H_
