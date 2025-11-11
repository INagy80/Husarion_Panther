// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from husarion_ugv_msgs:msg/LEDImageAnimation.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "husarion_ugv_msgs/msg/led_image_animation.h"


#ifndef HUSARION_UGV_MSGS__MSG__DETAIL__LED_IMAGE_ANIMATION__STRUCT_H_
#define HUSARION_UGV_MSGS__MSG__DETAIL__LED_IMAGE_ANIMATION__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// Constants defined in the message

// Include directives for member types
// Member 'image'
#include "rosidl_runtime_c/string.h"

/// Struct defined in msg/LEDImageAnimation in the package husarion_ugv_msgs.
typedef struct husarion_ugv_msgs__msg__LEDImageAnimation
{
  rosidl_runtime_c__String image;
  float duration;
  float brightness;
  uint8_t repeat;
  uint32_t color;
} husarion_ugv_msgs__msg__LEDImageAnimation;

// Struct for a sequence of husarion_ugv_msgs__msg__LEDImageAnimation.
typedef struct husarion_ugv_msgs__msg__LEDImageAnimation__Sequence
{
  husarion_ugv_msgs__msg__LEDImageAnimation * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} husarion_ugv_msgs__msg__LEDImageAnimation__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // HUSARION_UGV_MSGS__MSG__DETAIL__LED_IMAGE_ANIMATION__STRUCT_H_
