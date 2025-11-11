// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from husarion_ugv_msgs:msg/LEDImageAnimation.idl
// generated code does not contain a copyright notice

#include "husarion_ugv_msgs/msg/detail/led_image_animation__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_husarion_ugv_msgs
const rosidl_type_hash_t *
husarion_ugv_msgs__msg__LEDImageAnimation__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x2f, 0x47, 0x57, 0x0d, 0x0f, 0xa7, 0xf3, 0x69,
      0xcd, 0x95, 0x22, 0xcf, 0x03, 0x2a, 0x15, 0x62,
      0xcd, 0xbd, 0xd2, 0x1d, 0x5e, 0x4b, 0x70, 0x20,
      0x4e, 0x3e, 0x2b, 0x4b, 0x9c, 0x68, 0x68, 0x6b,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char husarion_ugv_msgs__msg__LEDImageAnimation__TYPE_NAME[] = "husarion_ugv_msgs/msg/LEDImageAnimation";

// Define type names, field names, and default values
static char husarion_ugv_msgs__msg__LEDImageAnimation__FIELD_NAME__image[] = "image";
static char husarion_ugv_msgs__msg__LEDImageAnimation__FIELD_NAME__duration[] = "duration";
static char husarion_ugv_msgs__msg__LEDImageAnimation__FIELD_NAME__brightness[] = "brightness";
static char husarion_ugv_msgs__msg__LEDImageAnimation__FIELD_NAME__repeat[] = "repeat";
static char husarion_ugv_msgs__msg__LEDImageAnimation__FIELD_NAME__color[] = "color";

static rosidl_runtime_c__type_description__Field husarion_ugv_msgs__msg__LEDImageAnimation__FIELDS[] = {
  {
    {husarion_ugv_msgs__msg__LEDImageAnimation__FIELD_NAME__image, 5, 5},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {husarion_ugv_msgs__msg__LEDImageAnimation__FIELD_NAME__duration, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {husarion_ugv_msgs__msg__LEDImageAnimation__FIELD_NAME__brightness, 10, 10},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {husarion_ugv_msgs__msg__LEDImageAnimation__FIELD_NAME__repeat, 6, 6},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {husarion_ugv_msgs__msg__LEDImageAnimation__FIELD_NAME__color, 5, 5},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
husarion_ugv_msgs__msg__LEDImageAnimation__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {husarion_ugv_msgs__msg__LEDImageAnimation__TYPE_NAME, 39, 39},
      {husarion_ugv_msgs__msg__LEDImageAnimation__FIELDS, 5, 5},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "string image\n"
  "float32 duration\n"
  "float32 brightness\n"
  "uint8 repeat\n"
  "uint32 color";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
husarion_ugv_msgs__msg__LEDImageAnimation__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {husarion_ugv_msgs__msg__LEDImageAnimation__TYPE_NAME, 39, 39},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 75, 75},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
husarion_ugv_msgs__msg__LEDImageAnimation__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *husarion_ugv_msgs__msg__LEDImageAnimation__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
