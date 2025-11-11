// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from husarion_ugv_msgs:msg/IOState.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "husarion_ugv_msgs/msg/io_state.h"


#ifndef HUSARION_UGV_MSGS__MSG__DETAIL__IO_STATE__STRUCT_H_
#define HUSARION_UGV_MSGS__MSG__DETAIL__IO_STATE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// Constants defined in the message

/// Struct defined in msg/IOState in the package husarion_ugv_msgs.
typedef struct husarion_ugv_msgs__msg__IOState
{
  bool aux_power;
  bool charger_connected;
  bool charger_enabled;
  bool digital_power;
  bool fan;
  bool led_control;
  bool motor_on;
  bool power_button;
} husarion_ugv_msgs__msg__IOState;

// Struct for a sequence of husarion_ugv_msgs__msg__IOState.
typedef struct husarion_ugv_msgs__msg__IOState__Sequence
{
  husarion_ugv_msgs__msg__IOState * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} husarion_ugv_msgs__msg__IOState__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // HUSARION_UGV_MSGS__MSG__DETAIL__IO_STATE__STRUCT_H_
