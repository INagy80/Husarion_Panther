// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from husarion_ugv_msgs:msg/DriverStateNamed.idl
// generated code does not contain a copyright notice

#include "husarion_ugv_msgs/msg/detail/driver_state_named__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_husarion_ugv_msgs
const rosidl_type_hash_t *
husarion_ugv_msgs__msg__DriverStateNamed__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x7d, 0x94, 0x52, 0x06, 0xdb, 0x20, 0x46, 0x64,
      0x3f, 0xe8, 0x04, 0xe2, 0x92, 0x98, 0x91, 0x69,
      0x52, 0x5d, 0xdb, 0xd7, 0x76, 0xbf, 0x0b, 0x6d,
      0x27, 0x88, 0x12, 0x55, 0x6e, 0x22, 0xa5, 0x71,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types
#include "husarion_ugv_msgs/msg/detail/runtime_error__functions.h"
#include "husarion_ugv_msgs/msg/detail/script_flag__functions.h"
#include "husarion_ugv_msgs/msg/detail/driver_state__functions.h"
#include "husarion_ugv_msgs/msg/detail/fault_flag__functions.h"

// Hashes for external referenced types
#ifndef NDEBUG
static const rosidl_type_hash_t husarion_ugv_msgs__msg__DriverState__EXPECTED_HASH = {1, {
    0x0a, 0x86, 0x62, 0x00, 0x13, 0x20, 0x8a, 0x58,
    0xdb, 0x4f, 0xa7, 0x68, 0x5d, 0xa5, 0xfa, 0x49,
    0x79, 0xcb, 0x9b, 0x2b, 0x5c, 0x87, 0x7e, 0xce,
    0x36, 0x4b, 0xc3, 0x89, 0x46, 0xa5, 0x79, 0x7a,
  }};
static const rosidl_type_hash_t husarion_ugv_msgs__msg__FaultFlag__EXPECTED_HASH = {1, {
    0xc6, 0x36, 0xba, 0x2e, 0x67, 0x06, 0x9a, 0x74,
    0x4a, 0xe3, 0xa0, 0x21, 0xf6, 0x69, 0x45, 0x6a,
    0xaf, 0x1b, 0x0d, 0x17, 0xbd, 0xba, 0xdd, 0xc2,
    0x0b, 0xd3, 0x35, 0x48, 0xcd, 0xe5, 0x4c, 0x55,
  }};
static const rosidl_type_hash_t husarion_ugv_msgs__msg__RuntimeError__EXPECTED_HASH = {1, {
    0xbf, 0x3e, 0xd1, 0x57, 0x38, 0x9c, 0xdd, 0x75,
    0x39, 0x90, 0xdf, 0xb7, 0x4a, 0xa8, 0x45, 0x95,
    0xdc, 0xe8, 0xc5, 0x40, 0xd6, 0xed, 0x79, 0xb4,
    0x28, 0x83, 0x4d, 0xbd, 0xf7, 0x54, 0xfc, 0x99,
  }};
static const rosidl_type_hash_t husarion_ugv_msgs__msg__ScriptFlag__EXPECTED_HASH = {1, {
    0x11, 0x80, 0xcb, 0x66, 0xd9, 0xe9, 0x6b, 0x1c,
    0xcf, 0xd5, 0xf5, 0x26, 0x8a, 0xa0, 0x9e, 0x6e,
    0xb9, 0x75, 0x1f, 0x69, 0x30, 0x25, 0xb5, 0x8a,
    0x91, 0xa5, 0x77, 0x4d, 0x45, 0xe1, 0x48, 0xa0,
  }};
#endif

static char husarion_ugv_msgs__msg__DriverStateNamed__TYPE_NAME[] = "husarion_ugv_msgs/msg/DriverStateNamed";
static char husarion_ugv_msgs__msg__DriverState__TYPE_NAME[] = "husarion_ugv_msgs/msg/DriverState";
static char husarion_ugv_msgs__msg__FaultFlag__TYPE_NAME[] = "husarion_ugv_msgs/msg/FaultFlag";
static char husarion_ugv_msgs__msg__RuntimeError__TYPE_NAME[] = "husarion_ugv_msgs/msg/RuntimeError";
static char husarion_ugv_msgs__msg__ScriptFlag__TYPE_NAME[] = "husarion_ugv_msgs/msg/ScriptFlag";

// Define type names, field names, and default values
static char husarion_ugv_msgs__msg__DriverStateNamed__FIELD_NAME__name[] = "name";
static char husarion_ugv_msgs__msg__DriverStateNamed__FIELD_NAME__state[] = "state";

static rosidl_runtime_c__type_description__Field husarion_ugv_msgs__msg__DriverStateNamed__FIELDS[] = {
  {
    {husarion_ugv_msgs__msg__DriverStateNamed__FIELD_NAME__name, 4, 4},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {husarion_ugv_msgs__msg__DriverStateNamed__FIELD_NAME__state, 5, 5},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {husarion_ugv_msgs__msg__DriverState__TYPE_NAME, 33, 33},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription husarion_ugv_msgs__msg__DriverStateNamed__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {husarion_ugv_msgs__msg__DriverState__TYPE_NAME, 33, 33},
    {NULL, 0, 0},
  },
  {
    {husarion_ugv_msgs__msg__FaultFlag__TYPE_NAME, 31, 31},
    {NULL, 0, 0},
  },
  {
    {husarion_ugv_msgs__msg__RuntimeError__TYPE_NAME, 34, 34},
    {NULL, 0, 0},
  },
  {
    {husarion_ugv_msgs__msg__ScriptFlag__TYPE_NAME, 32, 32},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
husarion_ugv_msgs__msg__DriverStateNamed__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {husarion_ugv_msgs__msg__DriverStateNamed__TYPE_NAME, 38, 38},
      {husarion_ugv_msgs__msg__DriverStateNamed__FIELDS, 2, 2},
    },
    {husarion_ugv_msgs__msg__DriverStateNamed__REFERENCED_TYPE_DESCRIPTIONS, 4, 4},
  };
  if (!constructed) {
    assert(0 == memcmp(&husarion_ugv_msgs__msg__DriverState__EXPECTED_HASH, husarion_ugv_msgs__msg__DriverState__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = husarion_ugv_msgs__msg__DriverState__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&husarion_ugv_msgs__msg__FaultFlag__EXPECTED_HASH, husarion_ugv_msgs__msg__FaultFlag__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[1].fields = husarion_ugv_msgs__msg__FaultFlag__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&husarion_ugv_msgs__msg__RuntimeError__EXPECTED_HASH, husarion_ugv_msgs__msg__RuntimeError__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[2].fields = husarion_ugv_msgs__msg__RuntimeError__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&husarion_ugv_msgs__msg__ScriptFlag__EXPECTED_HASH, husarion_ugv_msgs__msg__ScriptFlag__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[3].fields = husarion_ugv_msgs__msg__ScriptFlag__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "string NAME_DEFAULT = \"default\"\n"
  "string NAME_FRONT = \"front\"\n"
  "string NAME_REAR = \"rear\"\n"
  "\n"
  "string name\n"
  "husarion_ugv_msgs/DriverState state";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
husarion_ugv_msgs__msg__DriverStateNamed__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {husarion_ugv_msgs__msg__DriverStateNamed__TYPE_NAME, 38, 38},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 135, 135},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
husarion_ugv_msgs__msg__DriverStateNamed__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[5];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 5, 5};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *husarion_ugv_msgs__msg__DriverStateNamed__get_individual_type_description_source(NULL),
    sources[1] = *husarion_ugv_msgs__msg__DriverState__get_individual_type_description_source(NULL);
    sources[2] = *husarion_ugv_msgs__msg__FaultFlag__get_individual_type_description_source(NULL);
    sources[3] = *husarion_ugv_msgs__msg__RuntimeError__get_individual_type_description_source(NULL);
    sources[4] = *husarion_ugv_msgs__msg__ScriptFlag__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}
