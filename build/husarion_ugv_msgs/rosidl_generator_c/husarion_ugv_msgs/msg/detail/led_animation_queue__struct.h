// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from husarion_ugv_msgs:msg/LEDAnimationQueue.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "husarion_ugv_msgs/msg/led_animation_queue.h"


#ifndef HUSARION_UGV_MSGS__MSG__DETAIL__LED_ANIMATION_QUEUE__STRUCT_H_
#define HUSARION_UGV_MSGS__MSG__DETAIL__LED_ANIMATION_QUEUE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// Constants defined in the message

// Include directives for member types
// Member 'queue'
#include "rosidl_runtime_c/string.h"

/// Struct defined in msg/LEDAnimationQueue in the package husarion_ugv_msgs.
typedef struct husarion_ugv_msgs__msg__LEDAnimationQueue
{
  rosidl_runtime_c__String__Sequence queue;
} husarion_ugv_msgs__msg__LEDAnimationQueue;

// Struct for a sequence of husarion_ugv_msgs__msg__LEDAnimationQueue.
typedef struct husarion_ugv_msgs__msg__LEDAnimationQueue__Sequence
{
  husarion_ugv_msgs__msg__LEDAnimationQueue * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} husarion_ugv_msgs__msg__LEDAnimationQueue__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // HUSARION_UGV_MSGS__MSG__DETAIL__LED_ANIMATION_QUEUE__STRUCT_H_
