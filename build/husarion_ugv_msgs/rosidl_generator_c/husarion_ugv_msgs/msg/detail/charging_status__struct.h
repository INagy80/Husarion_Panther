// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from husarion_ugv_msgs:msg/ChargingStatus.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "husarion_ugv_msgs/msg/charging_status.h"


#ifndef HUSARION_UGV_MSGS__MSG__DETAIL__CHARGING_STATUS__STRUCT_H_
#define HUSARION_UGV_MSGS__MSG__DETAIL__CHARGING_STATUS__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// Constants defined in the message

/// Constant 'UNKNOWN'.
/**
  * Charger types
 */
enum
{
  husarion_ugv_msgs__msg__ChargingStatus__UNKNOWN = 0
};

/// Constant 'WIRED'.
enum
{
  husarion_ugv_msgs__msg__ChargingStatus__WIRED = 1
};

/// Constant 'WIRELESS'.
enum
{
  husarion_ugv_msgs__msg__ChargingStatus__WIRELESS = 2
};

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.h"

/// Struct defined in msg/ChargingStatus in the package husarion_ugv_msgs.
/**
  * This message defines the battery charging process status
 */
typedef struct husarion_ugv_msgs__msg__ChargingStatus
{
  std_msgs__msg__Header header;
  /// True if battery is being charged
  bool charging;
  /// Power supply total current (A)
  float current;
  /// Power supply current (A) for battery 1
  float current_battery_1;
  /// Power supply current (A) for battery 2 (NaN for single battery configuration)
  float current_battery_2;
  /// Determines the type of charger connection
  uint8_t charger_type;
} husarion_ugv_msgs__msg__ChargingStatus;

// Struct for a sequence of husarion_ugv_msgs__msg__ChargingStatus.
typedef struct husarion_ugv_msgs__msg__ChargingStatus__Sequence
{
  husarion_ugv_msgs__msg__ChargingStatus * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} husarion_ugv_msgs__msg__ChargingStatus__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // HUSARION_UGV_MSGS__MSG__DETAIL__CHARGING_STATUS__STRUCT_H_
