// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from husarion_ugv_msgs:msg/RuntimeError.idl
// generated code does not contain a copyright notice

#include "husarion_ugv_msgs/msg/detail/runtime_error__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_husarion_ugv_msgs
const rosidl_type_hash_t *
husarion_ugv_msgs__msg__RuntimeError__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xbf, 0x3e, 0xd1, 0x57, 0x38, 0x9c, 0xdd, 0x75,
      0x39, 0x90, 0xdf, 0xb7, 0x4a, 0xa8, 0x45, 0x95,
      0xdc, 0xe8, 0xc5, 0x40, 0xd6, 0xed, 0x79, 0xb4,
      0x28, 0x83, 0x4d, 0xbd, 0xf7, 0x54, 0xfc, 0x99,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char husarion_ugv_msgs__msg__RuntimeError__TYPE_NAME[] = "husarion_ugv_msgs/msg/RuntimeError";

// Define type names, field names, and default values
static char husarion_ugv_msgs__msg__RuntimeError__FIELD_NAME__amps_limit_active[] = "amps_limit_active";
static char husarion_ugv_msgs__msg__RuntimeError__FIELD_NAME__motor_stall[] = "motor_stall";
static char husarion_ugv_msgs__msg__RuntimeError__FIELD_NAME__loop_error[] = "loop_error";
static char husarion_ugv_msgs__msg__RuntimeError__FIELD_NAME__safety_stop_active[] = "safety_stop_active";
static char husarion_ugv_msgs__msg__RuntimeError__FIELD_NAME__forward_limit_triggered[] = "forward_limit_triggered";
static char husarion_ugv_msgs__msg__RuntimeError__FIELD_NAME__reverse_limit_triggered[] = "reverse_limit_triggered";
static char husarion_ugv_msgs__msg__RuntimeError__FIELD_NAME__amps_trigger_activated[] = "amps_trigger_activated";

static rosidl_runtime_c__type_description__Field husarion_ugv_msgs__msg__RuntimeError__FIELDS[] = {
  {
    {husarion_ugv_msgs__msg__RuntimeError__FIELD_NAME__amps_limit_active, 17, 17},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {husarion_ugv_msgs__msg__RuntimeError__FIELD_NAME__motor_stall, 11, 11},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {husarion_ugv_msgs__msg__RuntimeError__FIELD_NAME__loop_error, 10, 10},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {husarion_ugv_msgs__msg__RuntimeError__FIELD_NAME__safety_stop_active, 18, 18},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {husarion_ugv_msgs__msg__RuntimeError__FIELD_NAME__forward_limit_triggered, 23, 23},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {husarion_ugv_msgs__msg__RuntimeError__FIELD_NAME__reverse_limit_triggered, 23, 23},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {husarion_ugv_msgs__msg__RuntimeError__FIELD_NAME__amps_trigger_activated, 22, 22},
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
husarion_ugv_msgs__msg__RuntimeError__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {husarion_ugv_msgs__msg__RuntimeError__TYPE_NAME, 34, 34},
      {husarion_ugv_msgs__msg__RuntimeError__FIELDS, 7, 7},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "bool amps_limit_active\n"
  "bool motor_stall\n"
  "bool loop_error\n"
  "bool safety_stop_active\n"
  "bool forward_limit_triggered\n"
  "bool reverse_limit_triggered\n"
  "bool amps_trigger_activated";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
husarion_ugv_msgs__msg__RuntimeError__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {husarion_ugv_msgs__msg__RuntimeError__TYPE_NAME, 34, 34},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 166, 166},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
husarion_ugv_msgs__msg__RuntimeError__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *husarion_ugv_msgs__msg__RuntimeError__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
