// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from husarion_ugv_msgs:msg/LEDAnimationQueue.idl
// generated code does not contain a copyright notice

#include "husarion_ugv_msgs/msg/detail/led_animation_queue__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_husarion_ugv_msgs
const rosidl_type_hash_t *
husarion_ugv_msgs__msg__LEDAnimationQueue__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x51, 0xf6, 0x53, 0x9a, 0xc0, 0x47, 0x5e, 0x1e,
      0x99, 0x54, 0xe9, 0x42, 0x69, 0xbb, 0x38, 0x9a,
      0xf4, 0x9d, 0x0d, 0xe6, 0x4a, 0xd8, 0x7d, 0xf3,
      0x26, 0xd5, 0xaa, 0x08, 0xf2, 0x2f, 0x30, 0x4e,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char husarion_ugv_msgs__msg__LEDAnimationQueue__TYPE_NAME[] = "husarion_ugv_msgs/msg/LEDAnimationQueue";

// Define type names, field names, and default values
static char husarion_ugv_msgs__msg__LEDAnimationQueue__FIELD_NAME__queue[] = "queue";

static rosidl_runtime_c__type_description__Field husarion_ugv_msgs__msg__LEDAnimationQueue__FIELDS[] = {
  {
    {husarion_ugv_msgs__msg__LEDAnimationQueue__FIELD_NAME__queue, 5, 5},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING_UNBOUNDED_SEQUENCE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
husarion_ugv_msgs__msg__LEDAnimationQueue__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {husarion_ugv_msgs__msg__LEDAnimationQueue__TYPE_NAME, 39, 39},
      {husarion_ugv_msgs__msg__LEDAnimationQueue__FIELDS, 1, 1},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "string[] queue";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
husarion_ugv_msgs__msg__LEDAnimationQueue__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {husarion_ugv_msgs__msg__LEDAnimationQueue__TYPE_NAME, 39, 39},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 15, 15},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
husarion_ugv_msgs__msg__LEDAnimationQueue__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *husarion_ugv_msgs__msg__LEDAnimationQueue__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
