// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from husarion_ugv_msgs:msg/LEDAnimation.idl
// generated code does not contain a copyright notice

#include "husarion_ugv_msgs/msg/detail/led_animation__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_husarion_ugv_msgs
const rosidl_type_hash_t *
husarion_ugv_msgs__msg__LEDAnimation__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xd0, 0xc2, 0x03, 0xc2, 0x10, 0x86, 0xa3, 0x6d,
      0x2c, 0x0e, 0x61, 0xd1, 0x5a, 0x16, 0xb5, 0xd3,
      0x5c, 0x79, 0x1a, 0x88, 0x0b, 0xa4, 0xd8, 0x11,
      0x97, 0x8e, 0x0b, 0x87, 0xd7, 0xe8, 0x7a, 0x93,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char husarion_ugv_msgs__msg__LEDAnimation__TYPE_NAME[] = "husarion_ugv_msgs/msg/LEDAnimation";

// Define type names, field names, and default values
static char husarion_ugv_msgs__msg__LEDAnimation__FIELD_NAME__id[] = "id";
static char husarion_ugv_msgs__msg__LEDAnimation__FIELD_NAME__param[] = "param";

static rosidl_runtime_c__type_description__Field husarion_ugv_msgs__msg__LEDAnimation__FIELDS[] = {
  {
    {husarion_ugv_msgs__msg__LEDAnimation__FIELD_NAME__id, 2, 2},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT16,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {husarion_ugv_msgs__msg__LEDAnimation__FIELD_NAME__param, 5, 5},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
husarion_ugv_msgs__msg__LEDAnimation__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {husarion_ugv_msgs__msg__LEDAnimation__TYPE_NAME, 34, 34},
      {husarion_ugv_msgs__msg__LEDAnimation__FIELDS, 2, 2},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "uint16 E_STOP = 0\n"
  "uint16 READY = 1\n"
  "uint16 ERROR = 2\n"
  "uint16 NO_ERROR = 3\n"
  "uint16 MANUAL_ACTION = 4\n"
  "uint16 LOW_BATTERY = 5\n"
  "uint16 CRITICAL_BATTERY = 6\n"
  "uint16 CHARGING_BATTERY = 7\n"
  "uint16 BATTERY_CHARGED = 8\n"
  "uint16 CHARGER_INSERTED = 9\n"
  "uint16 BATTERY_NOMINAL = 10\n"
  "uint16 AUTONOMOUS_READY = 11\n"
  "uint16 AUTONOMOUS_ACTION = 12\n"
  "uint16 GOAL_ACHIEVED = 13\n"
  "uint16 BLINKER_LEFT = 14\n"
  "uint16 BLINKER_RIGHT = 15\n"
  "\n"
  "uint16 id\n"
  "string param";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
husarion_ugv_msgs__msg__LEDAnimation__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {husarion_ugv_msgs__msg__LEDAnimation__TYPE_NAME, 34, 34},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 419, 419},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
husarion_ugv_msgs__msg__LEDAnimation__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *husarion_ugv_msgs__msg__LEDAnimation__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
