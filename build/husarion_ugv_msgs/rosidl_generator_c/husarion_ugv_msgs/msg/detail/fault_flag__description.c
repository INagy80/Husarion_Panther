// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from husarion_ugv_msgs:msg/FaultFlag.idl
// generated code does not contain a copyright notice

#include "husarion_ugv_msgs/msg/detail/fault_flag__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_husarion_ugv_msgs
const rosidl_type_hash_t *
husarion_ugv_msgs__msg__FaultFlag__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xc6, 0x36, 0xba, 0x2e, 0x67, 0x06, 0x9a, 0x74,
      0x4a, 0xe3, 0xa0, 0x21, 0xf6, 0x69, 0x45, 0x6a,
      0xaf, 0x1b, 0x0d, 0x17, 0xbd, 0xba, 0xdd, 0xc2,
      0x0b, 0xd3, 0x35, 0x48, 0xcd, 0xe5, 0x4c, 0x55,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char husarion_ugv_msgs__msg__FaultFlag__TYPE_NAME[] = "husarion_ugv_msgs/msg/FaultFlag";

// Define type names, field names, and default values
static char husarion_ugv_msgs__msg__FaultFlag__FIELD_NAME__overheat[] = "overheat";
static char husarion_ugv_msgs__msg__FaultFlag__FIELD_NAME__overvoltage[] = "overvoltage";
static char husarion_ugv_msgs__msg__FaultFlag__FIELD_NAME__undervoltage[] = "undervoltage";
static char husarion_ugv_msgs__msg__FaultFlag__FIELD_NAME__short_circuit[] = "short_circuit";
static char husarion_ugv_msgs__msg__FaultFlag__FIELD_NAME__emergency_stop[] = "emergency_stop";
static char husarion_ugv_msgs__msg__FaultFlag__FIELD_NAME__motor_or_sensor_setup_fault[] = "motor_or_sensor_setup_fault";
static char husarion_ugv_msgs__msg__FaultFlag__FIELD_NAME__mosfet_failure[] = "mosfet_failure";
static char husarion_ugv_msgs__msg__FaultFlag__FIELD_NAME__default_config_loaded_at_startup[] = "default_config_loaded_at_startup";

static rosidl_runtime_c__type_description__Field husarion_ugv_msgs__msg__FaultFlag__FIELDS[] = {
  {
    {husarion_ugv_msgs__msg__FaultFlag__FIELD_NAME__overheat, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {husarion_ugv_msgs__msg__FaultFlag__FIELD_NAME__overvoltage, 11, 11},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {husarion_ugv_msgs__msg__FaultFlag__FIELD_NAME__undervoltage, 12, 12},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {husarion_ugv_msgs__msg__FaultFlag__FIELD_NAME__short_circuit, 13, 13},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {husarion_ugv_msgs__msg__FaultFlag__FIELD_NAME__emergency_stop, 14, 14},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {husarion_ugv_msgs__msg__FaultFlag__FIELD_NAME__motor_or_sensor_setup_fault, 27, 27},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {husarion_ugv_msgs__msg__FaultFlag__FIELD_NAME__mosfet_failure, 14, 14},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {husarion_ugv_msgs__msg__FaultFlag__FIELD_NAME__default_config_loaded_at_startup, 32, 32},
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
husarion_ugv_msgs__msg__FaultFlag__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {husarion_ugv_msgs__msg__FaultFlag__TYPE_NAME, 31, 31},
      {husarion_ugv_msgs__msg__FaultFlag__FIELDS, 8, 8},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "bool overheat\n"
  "bool overvoltage\n"
  "bool undervoltage\n"
  "bool short_circuit\n"
  "bool emergency_stop\n"
  "bool motor_or_sensor_setup_fault\n"
  "bool mosfet_failure\n"
  "bool default_config_loaded_at_startup";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
husarion_ugv_msgs__msg__FaultFlag__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {husarion_ugv_msgs__msg__FaultFlag__TYPE_NAME, 31, 31},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 179, 179},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
husarion_ugv_msgs__msg__FaultFlag__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *husarion_ugv_msgs__msg__FaultFlag__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
