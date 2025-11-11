// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from husarion_ugv_msgs:msg/ScriptFlag.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "husarion_ugv_msgs/msg/script_flag.h"


#ifndef HUSARION_UGV_MSGS__MSG__DETAIL__SCRIPT_FLAG__STRUCT_H_
#define HUSARION_UGV_MSGS__MSG__DETAIL__SCRIPT_FLAG__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// Constants defined in the message

/// Struct defined in msg/ScriptFlag in the package husarion_ugv_msgs.
typedef struct husarion_ugv_msgs__msg__ScriptFlag
{
  bool loop_error;
  bool encoder_disconnected;
  bool amp_limiter;
} husarion_ugv_msgs__msg__ScriptFlag;

// Struct for a sequence of husarion_ugv_msgs__msg__ScriptFlag.
typedef struct husarion_ugv_msgs__msg__ScriptFlag__Sequence
{
  husarion_ugv_msgs__msg__ScriptFlag * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} husarion_ugv_msgs__msg__ScriptFlag__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // HUSARION_UGV_MSGS__MSG__DETAIL__SCRIPT_FLAG__STRUCT_H_
