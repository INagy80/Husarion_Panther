// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from husarion_ugv_msgs:msg/LEDAnimation.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "husarion_ugv_msgs/msg/detail/led_animation__rosidl_typesupport_introspection_c.h"
#include "husarion_ugv_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "husarion_ugv_msgs/msg/detail/led_animation__functions.h"
#include "husarion_ugv_msgs/msg/detail/led_animation__struct.h"


// Include directives for member types
// Member `param`
#include "rosidl_runtime_c/string_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void husarion_ugv_msgs__msg__LEDAnimation__rosidl_typesupport_introspection_c__LEDAnimation_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  husarion_ugv_msgs__msg__LEDAnimation__init(message_memory);
}

void husarion_ugv_msgs__msg__LEDAnimation__rosidl_typesupport_introspection_c__LEDAnimation_fini_function(void * message_memory)
{
  husarion_ugv_msgs__msg__LEDAnimation__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember husarion_ugv_msgs__msg__LEDAnimation__rosidl_typesupport_introspection_c__LEDAnimation_message_member_array[2] = {
  {
    "id",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT16,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(husarion_ugv_msgs__msg__LEDAnimation, id),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "param",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(husarion_ugv_msgs__msg__LEDAnimation, param),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers husarion_ugv_msgs__msg__LEDAnimation__rosidl_typesupport_introspection_c__LEDAnimation_message_members = {
  "husarion_ugv_msgs__msg",  // message namespace
  "LEDAnimation",  // message name
  2,  // number of fields
  sizeof(husarion_ugv_msgs__msg__LEDAnimation),
  false,  // has_any_key_member_
  husarion_ugv_msgs__msg__LEDAnimation__rosidl_typesupport_introspection_c__LEDAnimation_message_member_array,  // message members
  husarion_ugv_msgs__msg__LEDAnimation__rosidl_typesupport_introspection_c__LEDAnimation_init_function,  // function to initialize message memory (memory has to be allocated)
  husarion_ugv_msgs__msg__LEDAnimation__rosidl_typesupport_introspection_c__LEDAnimation_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t husarion_ugv_msgs__msg__LEDAnimation__rosidl_typesupport_introspection_c__LEDAnimation_message_type_support_handle = {
  0,
  &husarion_ugv_msgs__msg__LEDAnimation__rosidl_typesupport_introspection_c__LEDAnimation_message_members,
  get_message_typesupport_handle_function,
  &husarion_ugv_msgs__msg__LEDAnimation__get_type_hash,
  &husarion_ugv_msgs__msg__LEDAnimation__get_type_description,
  &husarion_ugv_msgs__msg__LEDAnimation__get_type_description_sources,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_husarion_ugv_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, husarion_ugv_msgs, msg, LEDAnimation)() {
  if (!husarion_ugv_msgs__msg__LEDAnimation__rosidl_typesupport_introspection_c__LEDAnimation_message_type_support_handle.typesupport_identifier) {
    husarion_ugv_msgs__msg__LEDAnimation__rosidl_typesupport_introspection_c__LEDAnimation_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &husarion_ugv_msgs__msg__LEDAnimation__rosidl_typesupport_introspection_c__LEDAnimation_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
