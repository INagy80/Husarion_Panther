// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from husarion_ugv_msgs:msg/LEDImageAnimation.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "husarion_ugv_msgs/msg/detail/led_image_animation__functions.h"
#include "husarion_ugv_msgs/msg/detail/led_image_animation__struct.hpp"
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

void LEDImageAnimation_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) husarion_ugv_msgs::msg::LEDImageAnimation(_init);
}

void LEDImageAnimation_fini_function(void * message_memory)
{
  auto typed_message = static_cast<husarion_ugv_msgs::msg::LEDImageAnimation *>(message_memory);
  typed_message->~LEDImageAnimation();
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember LEDImageAnimation_message_member_array[5] = {
  {
    "image",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(husarion_ugv_msgs::msg::LEDImageAnimation, image),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "duration",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(husarion_ugv_msgs::msg::LEDImageAnimation, duration),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "brightness",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(husarion_ugv_msgs::msg::LEDImageAnimation, brightness),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "repeat",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(husarion_ugv_msgs::msg::LEDImageAnimation, repeat),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "color",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT32,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(husarion_ugv_msgs::msg::LEDImageAnimation, color),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers LEDImageAnimation_message_members = {
  "husarion_ugv_msgs::msg",  // message namespace
  "LEDImageAnimation",  // message name
  5,  // number of fields
  sizeof(husarion_ugv_msgs::msg::LEDImageAnimation),
  false,  // has_any_key_member_
  LEDImageAnimation_message_member_array,  // message members
  LEDImageAnimation_init_function,  // function to initialize message memory (memory has to be allocated)
  LEDImageAnimation_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t LEDImageAnimation_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &LEDImageAnimation_message_members,
  get_message_typesupport_handle_function,
  &husarion_ugv_msgs__msg__LEDImageAnimation__get_type_hash,
  &husarion_ugv_msgs__msg__LEDImageAnimation__get_type_description,
  &husarion_ugv_msgs__msg__LEDImageAnimation__get_type_description_sources,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace msg

}  // namespace husarion_ugv_msgs


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<husarion_ugv_msgs::msg::LEDImageAnimation>()
{
  return &::husarion_ugv_msgs::msg::rosidl_typesupport_introspection_cpp::LEDImageAnimation_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, husarion_ugv_msgs, msg, LEDImageAnimation)() {
  return &::husarion_ugv_msgs::msg::rosidl_typesupport_introspection_cpp::LEDImageAnimation_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
