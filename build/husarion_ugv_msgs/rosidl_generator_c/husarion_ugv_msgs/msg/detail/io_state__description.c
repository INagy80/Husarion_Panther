// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from husarion_ugv_msgs:msg/IOState.idl
// generated code does not contain a copyright notice

#include "husarion_ugv_msgs/msg/detail/io_state__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_husarion_ugv_msgs
const rosidl_type_hash_t *
husarion_ugv_msgs__msg__IOState__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xcf, 0x79, 0x76, 0x98, 0x91, 0x35, 0xfe, 0x6b,
      0x0d, 0x9d, 0x4c, 0xc4, 0xb5, 0xb0, 0x4e, 0xb7,
      0xd1, 0x28, 0xf8, 0xbd, 0xb7, 0xdc, 0xaa, 0xe5,
      0x34, 0xa5, 0x4a, 0x49, 0xb1, 0xac, 0xd3, 0x4b,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char husarion_ugv_msgs__msg__IOState__TYPE_NAME[] = "husarion_ugv_msgs/msg/IOState";

// Define type names, field names, and default values
static char husarion_ugv_msgs__msg__IOState__FIELD_NAME__aux_power[] = "aux_power";
static char husarion_ugv_msgs__msg__IOState__FIELD_NAME__charger_connected[] = "charger_connected";
static char husarion_ugv_msgs__msg__IOState__FIELD_NAME__charger_enabled[] = "charger_enabled";
static char husarion_ugv_msgs__msg__IOState__FIELD_NAME__digital_power[] = "digital_power";
static char husarion_ugv_msgs__msg__IOState__FIELD_NAME__fan[] = "fan";
static char husarion_ugv_msgs__msg__IOState__FIELD_NAME__led_control[] = "led_control";
static char husarion_ugv_msgs__msg__IOState__FIELD_NAME__motor_on[] = "motor_on";
static char husarion_ugv_msgs__msg__IOState__FIELD_NAME__power_button[] = "power_button";

static rosidl_runtime_c__type_description__Field husarion_ugv_msgs__msg__IOState__FIELDS[] = {
  {
    {husarion_ugv_msgs__msg__IOState__FIELD_NAME__aux_power, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {husarion_ugv_msgs__msg__IOState__FIELD_NAME__charger_connected, 17, 17},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {husarion_ugv_msgs__msg__IOState__FIELD_NAME__charger_enabled, 15, 15},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {husarion_ugv_msgs__msg__IOState__FIELD_NAME__digital_power, 13, 13},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {husarion_ugv_msgs__msg__IOState__FIELD_NAME__fan, 3, 3},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {husarion_ugv_msgs__msg__IOState__FIELD_NAME__led_control, 11, 11},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {husarion_ugv_msgs__msg__IOState__FIELD_NAME__motor_on, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {husarion_ugv_msgs__msg__IOState__FIELD_NAME__power_button, 12, 12},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
husarion_ugv_msgs__msg__IOState__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {husarion_ugv_msgs__msg__IOState__TYPE_NAME, 29, 29},
      {husarion_ugv_msgs__msg__IOState__FIELDS, 8, 8},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "bool aux_power\n"
  "bool charger_connected\n"
  "bool charger_enabled\n"
  "bool digital_power\n"
  "bool fan\n"
  "bool led_control\n"
  "bool motor_on\n"
  "bool power_button";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
husarion_ugv_msgs__msg__IOState__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {husarion_ugv_msgs__msg__IOState__TYPE_NAME, 29, 29},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 136, 136},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
husarion_ugv_msgs__msg__IOState__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *husarion_ugv_msgs__msg__IOState__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
