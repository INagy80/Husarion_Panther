// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from husarion_ugv_msgs:msg/RobotDriverState.idl
// generated code does not contain a copyright notice

#include "husarion_ugv_msgs/msg/detail/robot_driver_state__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_husarion_ugv_msgs
const rosidl_type_hash_t *
husarion_ugv_msgs__msg__RobotDriverState__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x28, 0xbd, 0x26, 0xde, 0x44, 0xd8, 0x81, 0x40,
      0x9c, 0x8d, 0x16, 0x5d, 0xd8, 0xa1, 0xff, 0xed,
      0x31, 0xda, 0xf4, 0x06, 0xa3, 0xaa, 0x6b, 0x26,
      0x4d, 0xa6, 0x9a, 0xe1, 0x93, 0xfa, 0xf5, 0x8d,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types
#include "builtin_interfaces/msg/detail/time__functions.h"
#include "husarion_ugv_msgs/msg/detail/script_flag__functions.h"
#include "husarion_ugv_msgs/msg/detail/driver_state_named__functions.h"
#include "husarion_ugv_msgs/msg/detail/fault_flag__functions.h"
#include "husarion_ugv_msgs/msg/detail/runtime_error__functions.h"
#include "std_msgs/msg/detail/header__functions.h"
#include "husarion_ugv_msgs/msg/detail/driver_state__functions.h"

// Hashes for external referenced types
#ifndef NDEBUG
static const rosidl_type_hash_t builtin_interfaces__msg__Time__EXPECTED_HASH = {1, {
    0xb1, 0x06, 0x23, 0x5e, 0x25, 0xa4, 0xc5, 0xed,
    0x35, 0x09, 0x8a, 0xa0, 0xa6, 0x1a, 0x3e, 0xe9,
    0xc9, 0xb1, 0x8d, 0x19, 0x7f, 0x39, 0x8b, 0x0e,
    0x42, 0x06, 0xce, 0xa9, 0xac, 0xf9, 0xc1, 0x97,
  }};
static const rosidl_type_hash_t husarion_ugv_msgs__msg__DriverState__EXPECTED_HASH = {1, {
    0x0a, 0x86, 0x62, 0x00, 0x13, 0x20, 0x8a, 0x58,
    0xdb, 0x4f, 0xa7, 0x68, 0x5d, 0xa5, 0xfa, 0x49,
    0x79, 0xcb, 0x9b, 0x2b, 0x5c, 0x87, 0x7e, 0xce,
    0x36, 0x4b, 0xc3, 0x89, 0x46, 0xa5, 0x79, 0x7a,
  }};
static const rosidl_type_hash_t husarion_ugv_msgs__msg__DriverStateNamed__EXPECTED_HASH = {1, {
    0x7d, 0x94, 0x52, 0x06, 0xdb, 0x20, 0x46, 0x64,
    0x3f, 0xe8, 0x04, 0xe2, 0x92, 0x98, 0x91, 0x69,
    0x52, 0x5d, 0xdb, 0xd7, 0x76, 0xbf, 0x0b, 0x6d,
    0x27, 0x88, 0x12, 0x55, 0x6e, 0x22, 0xa5, 0x71,
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
static const rosidl_type_hash_t std_msgs__msg__Header__EXPECTED_HASH = {1, {
    0xf4, 0x9f, 0xb3, 0xae, 0x2c, 0xf0, 0x70, 0xf7,
    0x93, 0x64, 0x5f, 0xf7, 0x49, 0x68, 0x3a, 0xc6,
    0xb0, 0x62, 0x03, 0xe4, 0x1c, 0x89, 0x1e, 0x17,
    0x70, 0x1b, 0x1c, 0xb5, 0x97, 0xce, 0x6a, 0x01,
  }};
#endif

static char husarion_ugv_msgs__msg__RobotDriverState__TYPE_NAME[] = "husarion_ugv_msgs/msg/RobotDriverState";
static char builtin_interfaces__msg__Time__TYPE_NAME[] = "builtin_interfaces/msg/Time";
static char husarion_ugv_msgs__msg__DriverState__TYPE_NAME[] = "husarion_ugv_msgs/msg/DriverState";
static char husarion_ugv_msgs__msg__DriverStateNamed__TYPE_NAME[] = "husarion_ugv_msgs/msg/DriverStateNamed";
static char husarion_ugv_msgs__msg__FaultFlag__TYPE_NAME[] = "husarion_ugv_msgs/msg/FaultFlag";
static char husarion_ugv_msgs__msg__RuntimeError__TYPE_NAME[] = "husarion_ugv_msgs/msg/RuntimeError";
static char husarion_ugv_msgs__msg__ScriptFlag__TYPE_NAME[] = "husarion_ugv_msgs/msg/ScriptFlag";
static char std_msgs__msg__Header__TYPE_NAME[] = "std_msgs/msg/Header";

// Define type names, field names, and default values
static char husarion_ugv_msgs__msg__RobotDriverState__FIELD_NAME__header[] = "header";
static char husarion_ugv_msgs__msg__RobotDriverState__FIELD_NAME__driver_states[] = "driver_states";
static char husarion_ugv_msgs__msg__RobotDriverState__FIELD_NAME__error[] = "error";
static char husarion_ugv_msgs__msg__RobotDriverState__FIELD_NAME__write_pdo_cmds_error[] = "write_pdo_cmds_error";
static char husarion_ugv_msgs__msg__RobotDriverState__FIELD_NAME__read_pdo_motor_states_error[] = "read_pdo_motor_states_error";
static char husarion_ugv_msgs__msg__RobotDriverState__FIELD_NAME__read_pdo_driver_state_error[] = "read_pdo_driver_state_error";

static rosidl_runtime_c__type_description__Field husarion_ugv_msgs__msg__RobotDriverState__FIELDS[] = {
  {
    {husarion_ugv_msgs__msg__RobotDriverState__FIELD_NAME__header, 6, 6},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {std_msgs__msg__Header__TYPE_NAME, 19, 19},
    },
    {NULL, 0, 0},
  },
  {
    {husarion_ugv_msgs__msg__RobotDriverState__FIELD_NAME__driver_states, 13, 13},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE_UNBOUNDED_SEQUENCE,
      0,
      0,
      {husarion_ugv_msgs__msg__DriverStateNamed__TYPE_NAME, 38, 38},
    },
    {NULL, 0, 0},
  },
  {
    {husarion_ugv_msgs__msg__RobotDriverState__FIELD_NAME__error, 5, 5},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {husarion_ugv_msgs__msg__RobotDriverState__FIELD_NAME__write_pdo_cmds_error, 20, 20},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {husarion_ugv_msgs__msg__RobotDriverState__FIELD_NAME__read_pdo_motor_states_error, 27, 27},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {husarion_ugv_msgs__msg__RobotDriverState__FIELD_NAME__read_pdo_driver_state_error, 27, 27},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription husarion_ugv_msgs__msg__RobotDriverState__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {builtin_interfaces__msg__Time__TYPE_NAME, 27, 27},
    {NULL, 0, 0},
  },
  {
    {husarion_ugv_msgs__msg__DriverState__TYPE_NAME, 33, 33},
    {NULL, 0, 0},
  },
  {
    {husarion_ugv_msgs__msg__DriverStateNamed__TYPE_NAME, 38, 38},
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
  {
    {std_msgs__msg__Header__TYPE_NAME, 19, 19},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
husarion_ugv_msgs__msg__RobotDriverState__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {husarion_ugv_msgs__msg__RobotDriverState__TYPE_NAME, 38, 38},
      {husarion_ugv_msgs__msg__RobotDriverState__FIELDS, 6, 6},
    },
    {husarion_ugv_msgs__msg__RobotDriverState__REFERENCED_TYPE_DESCRIPTIONS, 7, 7},
  };
  if (!constructed) {
    assert(0 == memcmp(&builtin_interfaces__msg__Time__EXPECTED_HASH, builtin_interfaces__msg__Time__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = builtin_interfaces__msg__Time__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&husarion_ugv_msgs__msg__DriverState__EXPECTED_HASH, husarion_ugv_msgs__msg__DriverState__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[1].fields = husarion_ugv_msgs__msg__DriverState__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&husarion_ugv_msgs__msg__DriverStateNamed__EXPECTED_HASH, husarion_ugv_msgs__msg__DriverStateNamed__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[2].fields = husarion_ugv_msgs__msg__DriverStateNamed__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&husarion_ugv_msgs__msg__FaultFlag__EXPECTED_HASH, husarion_ugv_msgs__msg__FaultFlag__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[3].fields = husarion_ugv_msgs__msg__FaultFlag__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&husarion_ugv_msgs__msg__RuntimeError__EXPECTED_HASH, husarion_ugv_msgs__msg__RuntimeError__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[4].fields = husarion_ugv_msgs__msg__RuntimeError__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&husarion_ugv_msgs__msg__ScriptFlag__EXPECTED_HASH, husarion_ugv_msgs__msg__ScriptFlag__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[5].fields = husarion_ugv_msgs__msg__ScriptFlag__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&std_msgs__msg__Header__EXPECTED_HASH, std_msgs__msg__Header__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[6].fields = std_msgs__msg__Header__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "std_msgs/Header header\n"
  "\n"
  "husarion_ugv_msgs/DriverStateNamed[] driver_states\n"
  "\n"
  "bool error\n"
  "bool write_pdo_cmds_error\n"
  "bool read_pdo_motor_states_error\n"
  "bool read_pdo_driver_state_error";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
husarion_ugv_msgs__msg__RobotDriverState__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {husarion_ugv_msgs__msg__RobotDriverState__TYPE_NAME, 38, 38},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 179, 179},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
husarion_ugv_msgs__msg__RobotDriverState__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[8];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 8, 8};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *husarion_ugv_msgs__msg__RobotDriverState__get_individual_type_description_source(NULL),
    sources[1] = *builtin_interfaces__msg__Time__get_individual_type_description_source(NULL);
    sources[2] = *husarion_ugv_msgs__msg__DriverState__get_individual_type_description_source(NULL);
    sources[3] = *husarion_ugv_msgs__msg__DriverStateNamed__get_individual_type_description_source(NULL);
    sources[4] = *husarion_ugv_msgs__msg__FaultFlag__get_individual_type_description_source(NULL);
    sources[5] = *husarion_ugv_msgs__msg__RuntimeError__get_individual_type_description_source(NULL);
    sources[6] = *husarion_ugv_msgs__msg__ScriptFlag__get_individual_type_description_source(NULL);
    sources[7] = *std_msgs__msg__Header__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}
