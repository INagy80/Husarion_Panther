// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from husarion_ugv_msgs:msg/ScriptFlag.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "husarion_ugv_msgs/msg/detail/script_flag__functions.h"
#include "husarion_ugv_msgs/msg/detail/script_flag__struct.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace husarion_ugv_msgs
{

namespace msg
{

namespace rosidl_typesupport_introspection_cpp
{

void ScriptFlag_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) husarion_ugv_msgs::msg::ScriptFlag(_init);
}

void ScriptFlag_fini_function(void * message_memory)
{
  auto typed_message = static_cast<husarion_ugv_msgs::msg::ScriptFlag *>(message_memory);
  typed_message->~ScriptFlag();
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember ScriptFlag_message_member_array[3] = {
  {
    "loop_error",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(husarion_ugv_msgs::msg::ScriptFlag, loop_error),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "encoder_disconnected",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(husarion_ugv_msgs::msg::ScriptFlag, encoder_disconnected),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "amp_limiter",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(husarion_ugv_msgs::msg::ScriptFlag, amp_limiter),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers ScriptFlag_message_members = {
  "husarion_ugv_msgs::msg",  // message namespace
  "ScriptFlag",  // message name
  3,  // number of fields
  sizeof(husarion_ugv_msgs::msg::ScriptFlag),
  false,  // has_any_key_member_
  ScriptFlag_message_member_array,  // message members
  ScriptFlag_init_function,  // function to initialize message memory (memory has to be allocated)
  ScriptFlag_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t ScriptFlag_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &ScriptFlag_message_members,
  get_message_typesupport_handle_function,
  &husarion_ugv_msgs__msg__ScriptFlag__get_type_hash,
  &husarion_ugv_msgs__msg__ScriptFlag__get_type_description,
  &husarion_ugv_msgs__msg__ScriptFlag__get_type_description_sources,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace msg

}  // namespace husarion_ugv_msgs


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<husarion_ugv_msgs::msg::ScriptFlag>()
{
  return &::husarion_ugv_msgs::msg::rosidl_typesupport_introspection_cpp::ScriptFlag_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, husarion_ugv_msgs, msg, ScriptFlag)() {
  return &::husarion_ugv_msgs::msg::rosidl_typesupport_introspection_cpp::ScriptFlag_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
