// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from husarion_ugv_msgs:msg/RobotDriverState.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "husarion_ugv_msgs/msg/robot_driver_state.h"


#ifndef HUSARION_UGV_MSGS__MSG__DETAIL__ROBOT_DRIVER_STATE__STRUCT_H_
#define HUSARION_UGV_MSGS__MSG__DETAIL__ROBOT_DRIVER_STATE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// Constants defined in the message

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.h"
// Member 'driver_states'
#include "husarion_ugv_msgs/msg/detail/driver_state_named__struct.h"

/// Struct defined in msg/RobotDriverState in the package husarion_ugv_msgs.
typedef struct husarion_ugv_msgs__msg__RobotDriverState
{
  std_msgs__msg__Header header;
  husarion_ugv_msgs__msg__DriverStateNamed__Sequence driver_states;
  bool error;
  bool write_pdo_cmds_error;
  bool read_pdo_motor_states_error;
  bool read_pdo_driver_state_error;
} husarion_ugv_msgs__msg__RobotDriverState;

// Struct for a sequence of husarion_ugv_msgs__msg__RobotDriverState.
typedef struct husarion_ugv_msgs__msg__RobotDriverState__Sequence
{
  husarion_ugv_msgs__msg__RobotDriverState * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} husarion_ugv_msgs__msg__RobotDriverState__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // HUSARION_UGV_MSGS__MSG__DETAIL__ROBOT_DRIVER_STATE__STRUCT_H_
