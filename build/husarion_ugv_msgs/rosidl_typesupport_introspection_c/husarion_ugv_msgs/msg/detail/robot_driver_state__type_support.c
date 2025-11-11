// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from husarion_ugv_msgs:msg/RobotDriverState.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "husarion_ugv_msgs/msg/detail/robot_driver_state__rosidl_typesupport_introspection_c.h"
#include "husarion_ugv_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "husarion_ugv_msgs/msg/detail/robot_driver_state__functions.h"
#include "husarion_ugv_msgs/msg/detail/robot_driver_state__struct.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/header.h"
// Member `header`
#include "std_msgs/msg/detail/header__rosidl_typesupport_introspection_c.h"
// Member `driver_states`
#include "husarion_ugv_msgs/msg/driver_state_named.h"
// Member `driver_states`
#include "husarion_ugv_msgs/msg/detail/driver_state_named__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void husarion_ugv_msgs__msg__RobotDriverState__rosidl_typesupport_introspection_c__RobotDriverState_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  husarion_ugv_msgs__msg__RobotDriverState__init(message_memory);
}

void husarion_ugv_msgs__msg__RobotDriverState__rosidl_typesupport_introspection_c__RobotDriverState_fini_function(void * message_memory)
{
  husarion_ugv_msgs__msg__RobotDriverState__fini(message_memory);
}

size_t husarion_ugv_msgs__msg__RobotDriverState__rosidl_typesupport_introspection_c__size_function__RobotDriverState__driver_states(
  const void * untyped_member)
{
  const husarion_ugv_msgs__msg__DriverStateNamed__Sequence * member =
    (const husarion_ugv_msgs__msg__DriverStateNamed__Sequence *)(untyped_member);
  return member->size;
}

const void * husarion_ugv_msgs__msg__RobotDriverState__rosidl_typesupport_introspection_c__get_const_function__RobotDriverState__driver_states(
  const void * untyped_member, size_t index)
{
  const husarion_ugv_msgs__msg__DriverStateNamed__Sequence * member =
    (const husarion_ugv_msgs__msg__DriverStateNamed__Sequence *)(untyped_member);
  return &member->data[index];
}

void * husarion_ugv_msgs__msg__RobotDriverState__rosidl_typesupport_introspection_c__get_function__RobotDriverState__driver_states(
  void * untyped_member, size_t index)
{
  husarion_ugv_msgs__msg__DriverStateNamed__Sequence * member =
    (husarion_ugv_msgs__msg__DriverStateNamed__Sequence *)(untyped_member);
  return &member->data[index];
}

void husarion_ugv_msgs__msg__RobotDriverState__rosidl_typesupport_introspection_c__fetch_function__RobotDriverState__driver_states(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const husarion_ugv_msgs__msg__DriverStateNamed * item =
    ((const husarion_ugv_msgs__msg__DriverStateNamed *)
    husarion_ugv_msgs__msg__RobotDriverState__rosidl_typesupport_introspection_c__get_const_function__RobotDriverState__driver_states(untyped_member, index));
  husarion_ugv_msgs__msg__DriverStateNamed * value =
    (husarion_ugv_msgs__msg__DriverStateNamed *)(untyped_value);
  *value = *item;
}

void husarion_ugv_msgs__msg__RobotDriverState__rosidl_typesupport_introspection_c__assign_function__RobotDriverState__driver_states(
  void * untyped_member, size_t index, const void * untyped_value)
{
  husarion_ugv_msgs__msg__DriverStateNamed * item =
    ((husarion_ugv_msgs__msg__DriverStateNamed *)
    husarion_ugv_msgs__msg__RobotDriverState__rosidl_typesupport_introspection_c__get_function__RobotDriverState__driver_states(untyped_member, index));
  const husarion_ugv_msgs__msg__DriverStateNamed * value =
    (const husarion_ugv_msgs__msg__DriverStateNamed *)(untyped_value);
  *item = *value;
}

bool husarion_ugv_msgs__msg__RobotDriverState__rosidl_typesupport_introspection_c__resize_function__RobotDriverState__driver_states(
  void * untyped_member, size_t size)
{
  husarion_ugv_msgs__msg__DriverStateNamed__Sequence * member =
    (husarion_ugv_msgs__msg__DriverStateNamed__Sequence *)(untyped_member);
  husarion_ugv_msgs__msg__DriverStateNamed__Sequence__fini(member);
  return husarion_ugv_msgs__msg__DriverStateNamed__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember husarion_ugv_msgs__msg__RobotDriverState__rosidl_typesupport_introspection_c__RobotDriverState_message_member_array[6] = {
  {
    "header",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(husarion_ugv_msgs__msg__RobotDriverState, header),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "driver_states",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is key
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(husarion_ugv_msgs__msg__RobotDriverState, driver_states),  // bytes offset in struct
    NULL,  // default value
    husarion_ugv_msgs__msg__RobotDriverState__rosidl_typesupport_introspection_c__size_function__RobotDriverState__driver_states,  // size() function pointer
    husarion_ugv_msgs__msg__RobotDriverState__rosidl_typesupport_introspection_c__get_const_function__RobotDriverState__driver_states,  // get_const(index) function pointer
    husarion_ugv_msgs__msg__RobotDriverState__rosidl_typesupport_introspection_c__get_function__RobotDriverState__driver_states,  // get(index) function pointer
    husarion_ugv_msgs__msg__RobotDriverState__rosidl_typesupport_introspection_c__fetch_function__RobotDriverState__driver_states,  // fetch(index, &value) function pointer
    husarion_ugv_msgs__msg__RobotDriverState__rosidl_typesupport_introspection_c__assign_function__RobotDriverState__driver_states,  // assign(index, value) function pointer
    husarion_ugv_msgs__msg__RobotDriverState__rosidl_typesupport_introspection_c__resize_function__RobotDriverState__driver_states  // resize(index) function pointer
  },
  {
    "error",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(husarion_ugv_msgs__msg__RobotDriverState, error),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "write_pdo_cmds_error",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(husarion_ugv_msgs__msg__RobotDriverState, write_pdo_cmds_error),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "read_pdo_motor_states_error",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(husarion_ugv_msgs__msg__RobotDriverState, read_pdo_motor_states_error),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "read_pdo_driver_state_error",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(husarion_ugv_msgs__msg__RobotDriverState, read_pdo_driver_state_error),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers husarion_ugv_msgs__msg__RobotDriverState__rosidl_typesupport_introspection_c__RobotDriverState_message_members = {
  "husarion_ugv_msgs__msg",  // message namespace
  "RobotDriverState",  // message name
  6,  // number of fields
  sizeof(husarion_ugv_msgs__msg__RobotDriverState),
  false,  // has_any_key_member_
  husarion_ugv_msgs__msg__RobotDriverState__rosidl_typesupport_introspection_c__RobotDriverState_message_member_array,  // message members
  husarion_ugv_msgs__msg__RobotDriverState__rosidl_typesupport_introspection_c__RobotDriverState_init_function,  // function to initialize message memory (memory has to be allocated)
  husarion_ugv_msgs__msg__RobotDriverState__rosidl_typesupport_introspection_c__RobotDriverState_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t husarion_ugv_msgs__msg__RobotDriverState__rosidl_typesupport_introspection_c__RobotDriverState_message_type_support_handle = {
  0,
  &husarion_ugv_msgs__msg__RobotDriverState__rosidl_typesupport_introspection_c__RobotDriverState_message_members,
  get_message_typesupport_handle_function,
  &husarion_ugv_msgs__msg__RobotDriverState__get_type_hash,
  &husarion_ugv_msgs__msg__RobotDriverState__get_type_description,
  &husarion_ugv_msgs__msg__RobotDriverState__get_type_description_sources,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_husarion_ugv_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, husarion_ugv_msgs, msg, RobotDriverState)() {
  husarion_ugv_msgs__msg__RobotDriverState__rosidl_typesupport_introspection_c__RobotDriverState_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, std_msgs, msg, Header)();
  husarion_ugv_msgs__msg__RobotDriverState__rosidl_typesupport_introspection_c__RobotDriverState_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, husarion_ugv_msgs, msg, DriverStateNamed)();
  if (!husarion_ugv_msgs__msg__RobotDriverState__rosidl_typesupport_introspection_c__RobotDriverState_message_type_support_handle.typesupport_identifier) {
    husarion_ugv_msgs__msg__RobotDriverState__rosidl_typesupport_introspection_c__RobotDriverState_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &husarion_ugv_msgs__msg__RobotDriverState__rosidl_typesupport_introspection_c__RobotDriverState_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
