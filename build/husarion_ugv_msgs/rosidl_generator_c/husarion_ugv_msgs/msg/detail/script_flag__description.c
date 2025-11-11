// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from husarion_ugv_msgs:msg/ScriptFlag.idl
// generated code does not contain a copyright notice

#include "husarion_ugv_msgs/msg/detail/script_flag__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_husarion_ugv_msgs
const rosidl_type_hash_t *
husarion_ugv_msgs__msg__ScriptFlag__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x11, 0x80, 0xcb, 0x66, 0xd9, 0xe9, 0x6b, 0x1c,
      0xcf, 0xd5, 0xf5, 0x26, 0x8a, 0xa0, 0x9e, 0x6e,
      0xb9, 0x75, 0x1f, 0x69, 0x30, 0x25, 0xb5, 0x8a,
      0x91, 0xa5, 0x77, 0x4d, 0x45, 0xe1, 0x48, 0xa0,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char husarion_ugv_msgs__msg__ScriptFlag__TYPE_NAME[] = "husarion_ugv_msgs/msg/ScriptFlag";

// Define type names, field names, and default values
static char husarion_ugv_msgs__msg__ScriptFlag__FIELD_NAME__loop_error[] = "loop_error";
static char husarion_ugv_msgs__msg__ScriptFlag__FIELD_NAME__encoder_disconnected[] = "encoder_disconnected";
static char husarion_ugv_msgs__msg__ScriptFlag__FIELD_NAME__amp_limiter[] = "amp_limiter";

static rosidl_runtime_c__type_description__Field husarion_ugv_msgs__msg__ScriptFlag__FIELDS[] = {
  {
    {husarion_ugv_msgs__msg__ScriptFlag__FIELD_NAME__loop_error, 10, 10},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {husarion_ugv_msgs__msg__ScriptFlag__FIELD_NAME__encoder_disconnected, 20, 20},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {husarion_ugv_msgs__msg__ScriptFlag__FIELD_NAME__amp_limiter, 11, 11},
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
husarion_ugv_msgs__msg__ScriptFlag__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {husarion_ugv_msgs__msg__ScriptFlag__TYPE_NAME, 32, 32},
      {husarion_ugv_msgs__msg__ScriptFlag__FIELDS, 3, 3},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "bool loop_error\n"
  "bool encoder_disconnected\n"
  "bool amp_limiter";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
husarion_ugv_msgs__msg__ScriptFlag__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {husarion_ugv_msgs__msg__ScriptFlag__TYPE_NAME, 32, 32},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 59, 59},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
husarion_ugv_msgs__msg__ScriptFlag__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *husarion_ugv_msgs__msg__ScriptFlag__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
