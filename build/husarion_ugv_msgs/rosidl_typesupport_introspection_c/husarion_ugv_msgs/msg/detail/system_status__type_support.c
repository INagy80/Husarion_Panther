// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from husarion_ugv_msgs:msg/SystemStatus.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "husarion_ugv_msgs/msg/detail/system_status__rosidl_typesupport_introspection_c.h"
#include "husarion_ugv_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "husarion_ugv_msgs/msg/detail/system_status__functions.h"
#include "husarion_ugv_msgs/msg/detail/system_status__struct.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/header.h"
// Member `header`
#include "std_msgs/msg/detail/header__rosidl_typesupport_introspection_c.h"
// Member `cpu_percent`
#include "rosidl_runtime_c/primitives_sequence_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void husarion_ugv_msgs__msg__SystemStatus__rosidl_typesupport_introspection_c__SystemStatus_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  husarion_ugv_msgs__msg__SystemStatus__init(message_memory);
}

void husarion_ugv_msgs__msg__SystemStatus__rosidl_typesupport_introspection_c__SystemStatus_fini_function(void * message_memory)
{
  husarion_ugv_msgs__msg__SystemStatus__fini(message_memory);
}

size_t husarion_ugv_msgs__msg__SystemStatus__rosidl_typesupport_introspection_c__size_function__SystemStatus__cpu_percent(
  const void * untyped_member)
{
  const rosidl_runtime_c__float__Sequence * member =
    (const rosidl_runtime_c__float__Sequence *)(untyped_member);
  return member->size;
}

const void * husarion_ugv_msgs__msg__SystemStatus__rosidl_typesupport_introspection_c__get_const_function__SystemStatus__cpu_percent(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__float__Sequence * member =
    (const rosidl_runtime_c__float__Sequence *)(untyped_member);
  return &member->data[index];
}

void * husarion_ugv_msgs__msg__SystemStatus__rosidl_typesupport_introspection_c__get_function__SystemStatus__cpu_percent(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__float__Sequence * member =
    (rosidl_runtime_c__float__Sequence *)(untyped_member);
  return &member->data[index];
}

void husarion_ugv_msgs__msg__SystemStatus__rosidl_typesupport_introspection_c__fetch_function__SystemStatus__cpu_percent(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const float * item =
    ((const float *)
    husarion_ugv_msgs__msg__SystemStatus__rosidl_typesupport_introspection_c__get_const_function__SystemStatus__cpu_percent(untyped_member, index));
  float * value =
    (float *)(untyped_value);
  *value = *item;
}

void husarion_ugv_msgs__msg__SystemStatus__rosidl_typesupport_introspection_c__assign_function__SystemStatus__cpu_percent(
  void * untyped_member, size_t index, const void * untyped_value)
{
  float * item =
    ((float *)
    husarion_ugv_msgs__msg__SystemStatus__rosidl_typesupport_introspection_c__get_function__SystemStatus__cpu_percent(untyped_member, index));
  const float * value =
    (const float *)(untyped_value);
  *item = *value;
}

bool husarion_ugv_msgs__msg__SystemStatus__rosidl_typesupport_introspection_c__resize_function__SystemStatus__cpu_percent(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__float__Sequence * member =
    (rosidl_runtime_c__float__Sequence *)(untyped_member);
  rosidl_runtime_c__float__Sequence__fini(member);
  return rosidl_runtime_c__float__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember husarion_ugv_msgs__msg__SystemStatus__rosidl_typesupport_introspection_c__SystemStatus_message_member_array[6] = {
  {
    "header",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(husarion_ugv_msgs__msg__SystemStatus, header),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "cpu_percent",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(husarion_ugv_msgs__msg__SystemStatus, cpu_percent),  // bytes offset in struct
    NULL,  // default value
    husarion_ugv_msgs__msg__SystemStatus__rosidl_typesupport_introspection_c__size_function__SystemStatus__cpu_percent,  // size() function pointer
    husarion_ugv_msgs__msg__SystemStatus__rosidl_typesupport_introspection_c__get_const_function__SystemStatus__cpu_percent,  // get_const(index) function pointer
    husarion_ugv_msgs__msg__SystemStatus__rosidl_typesupport_introspection_c__get_function__SystemStatus__cpu_percent,  // get(index) function pointer
    husarion_ugv_msgs__msg__SystemStatus__rosidl_typesupport_introspection_c__fetch_function__SystemStatus__cpu_percent,  // fetch(index, &value) function pointer
    husarion_ugv_msgs__msg__SystemStatus__rosidl_typesupport_introspection_c__assign_function__SystemStatus__cpu_percent,  // assign(index, value) function pointer
    husarion_ugv_msgs__msg__SystemStatus__rosidl_typesupport_introspection_c__resize_function__SystemStatus__cpu_percent  // resize(index) function pointer
  },
  {
    "cpu_temp",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(husarion_ugv_msgs__msg__SystemStatus, cpu_temp),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "avg_load_percent",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(husarion_ugv_msgs__msg__SystemStatus, avg_load_percent),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "ram_usage_percent",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(husarion_ugv_msgs__msg__SystemStatus, ram_usage_percent),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "disc_usage_percent",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(husarion_ugv_msgs__msg__SystemStatus, disc_usage_percent),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers husarion_ugv_msgs__msg__SystemStatus__rosidl_typesupport_introspection_c__SystemStatus_message_members = {
  "husarion_ugv_msgs__msg",  // message namespace
  "SystemStatus",  // message name
  6,  // number of fields
  sizeof(husarion_ugv_msgs__msg__SystemStatus),
  false,  // has_any_key_member_
  husarion_ugv_msgs__msg__SystemStatus__rosidl_typesupport_introspection_c__SystemStatus_message_member_array,  // message members
  husarion_ugv_msgs__msg__SystemStatus__rosidl_typesupport_introspection_c__SystemStatus_init_function,  // function to initialize message memory (memory has to be allocated)
  husarion_ugv_msgs__msg__SystemStatus__rosidl_typesupport_introspection_c__SystemStatus_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t husarion_ugv_msgs__msg__SystemStatus__rosidl_typesupport_introspection_c__SystemStatus_message_type_support_handle = {
  0,
  &husarion_ugv_msgs__msg__SystemStatus__rosidl_typesupport_introspection_c__SystemStatus_message_members,
  get_message_typesupport_handle_function,
  &husarion_ugv_msgs__msg__SystemStatus__get_type_hash,
  &husarion_ugv_msgs__msg__SystemStatus__get_type_description,
  &husarion_ugv_msgs__msg__SystemStatus__get_type_description_sources,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_husarion_ugv_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, husarion_ugv_msgs, msg, SystemStatus)() {
  husarion_ugv_msgs__msg__SystemStatus__rosidl_typesupport_introspection_c__SystemStatus_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, std_msgs, msg, Header)();
  if (!husarion_ugv_msgs__msg__SystemStatus__rosidl_typesupport_introspection_c__SystemStatus_message_type_support_handle.typesupport_identifier) {
    husarion_ugv_msgs__msg__SystemStatus__rosidl_typesupport_introspection_c__SystemStatus_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &husarion_ugv_msgs__msg__SystemStatus__rosidl_typesupport_introspection_c__SystemStatus_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
