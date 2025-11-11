// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from husarion_ugv_msgs:msg/FaultFlag.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "husarion_ugv_msgs/msg/fault_flag.h"


#ifndef HUSARION_UGV_MSGS__MSG__DETAIL__FAULT_FLAG__STRUCT_H_
#define HUSARION_UGV_MSGS__MSG__DETAIL__FAULT_FLAG__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// Constants defined in the message

/// Struct defined in msg/FaultFlag in the package husarion_ugv_msgs.
typedef struct husarion_ugv_msgs__msg__FaultFlag
{
  bool overheat;
  bool overvoltage;
  bool undervoltage;
  bool short_circuit;
  bool emergency_stop;
  bool motor_or_sensor_setup_fault;
  bool mosfet_failure;
  bool default_config_loaded_at_startup;
} husarion_ugv_msgs__msg__FaultFlag;

// Struct for a sequence of husarion_ugv_msgs__msg__FaultFlag.
typedef struct husarion_ugv_msgs__msg__FaultFlag__Sequence
{
  husarion_ugv_msgs__msg__FaultFlag * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} husarion_ugv_msgs__msg__FaultFlag__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // HUSARION_UGV_MSGS__MSG__DETAIL__FAULT_FLAG__STRUCT_H_
