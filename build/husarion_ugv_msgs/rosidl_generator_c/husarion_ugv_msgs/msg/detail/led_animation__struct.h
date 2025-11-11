// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from husarion_ugv_msgs:msg/LEDAnimation.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "husarion_ugv_msgs/msg/led_animation.h"


#ifndef HUSARION_UGV_MSGS__MSG__DETAIL__LED_ANIMATION__STRUCT_H_
#define HUSARION_UGV_MSGS__MSG__DETAIL__LED_ANIMATION__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// Constants defined in the message

/// Constant 'E_STOP'.
enum
{
  husarion_ugv_msgs__msg__LEDAnimation__E_STOP = 0
};

/// Constant 'READY'.
enum
{
  husarion_ugv_msgs__msg__LEDAnimation__READY = 1
};

/// Constant 'ERROR'.
enum
{
  husarion_ugv_msgs__msg__LEDAnimation__ERROR = 2
};

/// Constant 'NO_ERROR'.
enum
{
  husarion_ugv_msgs__msg__LEDAnimation__NO_ERROR = 3
};

/// Constant 'MANUAL_ACTION'.
enum
{
  husarion_ugv_msgs__msg__LEDAnimation__MANUAL_ACTION = 4
};

/// Constant 'LOW_BATTERY'.
enum
{
  husarion_ugv_msgs__msg__LEDAnimation__LOW_BATTERY = 5
};

/// Constant 'CRITICAL_BATTERY'.
enum
{
  husarion_ugv_msgs__msg__LEDAnimation__CRITICAL_BATTERY = 6
};

/// Constant 'CHARGING_BATTERY'.
enum
{
  husarion_ugv_msgs__msg__LEDAnimation__CHARGING_BATTERY = 7
};

/// Constant 'BATTERY_CHARGED'.
enum
{
  husarion_ugv_msgs__msg__LEDAnimation__BATTERY_CHARGED = 8
};

/// Constant 'CHARGER_INSERTED'.
enum
{
  husarion_ugv_msgs__msg__LEDAnimation__CHARGER_INSERTED = 9
};

/// Constant 'BATTERY_NOMINAL'.
enum
{
  husarion_ugv_msgs__msg__LEDAnimation__BATTERY_NOMINAL = 10
};

/// Constant 'AUTONOMOUS_READY'.
enum
{
  husarion_ugv_msgs__msg__LEDAnimation__AUTONOMOUS_READY = 11
};

/// Constant 'AUTONOMOUS_ACTION'.
enum
{
  husarion_ugv_msgs__msg__LEDAnimation__AUTONOMOUS_ACTION = 12
};

/// Constant 'GOAL_ACHIEVED'.
enum
{
  husarion_ugv_msgs__msg__LEDAnimation__GOAL_ACHIEVED = 13
};

/// Constant 'BLINKER_LEFT'.
enum
{
  husarion_ugv_msgs__msg__LEDAnimation__BLINKER_LEFT = 14
};

/// Constant 'BLINKER_RIGHT'.
enum
{
  husarion_ugv_msgs__msg__LEDAnimation__BLINKER_RIGHT = 15
};

// Include directives for member types
// Member 'param'
#include "rosidl_runtime_c/string.h"

/// Struct defined in msg/LEDAnimation in the package husarion_ugv_msgs.
typedef struct husarion_ugv_msgs__msg__LEDAnimation
{
  uint16_t id;
  rosidl_runtime_c__String param;
} husarion_ugv_msgs__msg__LEDAnimation;

// Struct for a sequence of husarion_ugv_msgs__msg__LEDAnimation.
typedef struct husarion_ugv_msgs__msg__LEDAnimation__Sequence
{
  husarion_ugv_msgs__msg__LEDAnimation * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} husarion_ugv_msgs__msg__LEDAnimation__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // HUSARION_UGV_MSGS__MSG__DETAIL__LED_ANIMATION__STRUCT_H_
