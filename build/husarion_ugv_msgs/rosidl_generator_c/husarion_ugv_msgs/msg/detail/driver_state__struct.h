// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from husarion_ugv_msgs:msg/DriverState.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "husarion_ugv_msgs/msg/driver_state.h"


#ifndef HUSARION_UGV_MSGS__MSG__DETAIL__DRIVER_STATE__STRUCT_H_
#define HUSARION_UGV_MSGS__MSG__DETAIL__DRIVER_STATE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// Constants defined in the message

// Include directives for member types
// Member 'fault_flag'
#include "husarion_ugv_msgs/msg/detail/fault_flag__struct.h"
// Member 'script_flag'
#include "husarion_ugv_msgs/msg/detail/script_flag__struct.h"
// Member 'channel_1_motor_runtime_error'
// Member 'channel_2_motor_runtime_error'
#include "husarion_ugv_msgs/msg/detail/runtime_error__struct.h"

/// Struct defined in msg/DriverState in the package husarion_ugv_msgs.
typedef struct husarion_ugv_msgs__msg__DriverState
{
  float voltage;
  float current;
  float temperature;
  float heatsink_temperature;
  husarion_ugv_msgs__msg__FaultFlag fault_flag;
  husarion_ugv_msgs__msg__ScriptFlag script_flag;
  husarion_ugv_msgs__msg__RuntimeError channel_1_motor_runtime_error;
  husarion_ugv_msgs__msg__RuntimeError channel_2_motor_runtime_error;
  bool motor_states_data_timed_out;
  bool driver_state_data_timed_out;
  bool can_error;
  bool heartbeat_timeout;
} husarion_ugv_msgs__msg__DriverState;

// Struct for a sequence of husarion_ugv_msgs__msg__DriverState.
typedef struct husarion_ugv_msgs__msg__DriverState__Sequence
{
  husarion_ugv_msgs__msg__DriverState * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} husarion_ugv_msgs__msg__DriverState__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // HUSARION_UGV_MSGS__MSG__DETAIL__DRIVER_STATE__STRUCT_H_
