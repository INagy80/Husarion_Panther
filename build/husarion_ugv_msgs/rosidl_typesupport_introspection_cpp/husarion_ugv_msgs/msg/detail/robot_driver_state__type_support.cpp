// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from husarion_ugv_msgs:msg/RobotDriverState.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "husarion_ugv_msgs/msg/detail/robot_driver_state__functions.h"
#include "husarion_ugv_msgs/msg/detail/robot_driver_state__struct.hpp"
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

void RobotDriverState_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) husarion_ugv_msgs::msg::RobotDriverState(_init);
}

void RobotDriverState_fini_function(void * message_memory)
{
  auto typed_message = static_cast<husarion_ugv_msgs::msg::RobotDriverState *>(message_memory);
  typed_message->~RobotDriverState();
}

size_t size_function__RobotDriverState__driver_states(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<husarion_ugv_msgs::msg::DriverStateNamed> *>(untyped_member);
  return member->size();
}

const void * get_const_function__RobotDriverState__driver_states(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<husarion_ugv_msgs::msg::DriverStateNamed> *>(untyped_member);
  return &member[index];
}

void * get_function__RobotDriverState__driver_states(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<husarion_ugv_msgs::msg::DriverStateNamed> *>(untyped_member);
  return &member[index];
}

void fetch_function__RobotDriverState__driver_states(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const husarion_ugv_msgs::msg::DriverStateNamed *>(
    get_const_function__RobotDriverState__driver_states(untyped_member, index));
  auto & value = *reinterpret_cast<husarion_ugv_msgs::msg::DriverStateNamed *>(untyped_value);
  value = item;
}

void assign_function__RobotDriverState__driver_states(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<husarion_ugv_msgs::msg::DriverStateNamed *>(
    get_function__RobotDriverState__driver_states(untyped_member, index));
  const auto & value = *reinterpret_cast<const husarion_ugv_msgs::msg::DriverStateNamed *>(untyped_value);
  item = value;
}

void resize_function__RobotDriverState__driver_states(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<husarion_ugv_msgs::msg::DriverStateNamed> *>(untyped_member);
  member->resize(size);
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember RobotDriverState_message_member_array[6] = {
  {
    "header",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<std_msgs::msg::Header>(),  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(husarion_ugv_msgs::msg::RobotDriverState, header),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "driver_states",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<husarion_ugv_msgs::msg::DriverStateNamed>(),  // members of sub message
    false,  // is key
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(husarion_ugv_msgs::msg::RobotDriverState, driver_states),  // bytes offset in struct
    nullptr,  // default value
    size_function__RobotDriverState__driver_states,  // size() function pointer
    get_const_function__RobotDriverState__driver_states,  // get_const(index) function pointer
    get_function__RobotDriverState__driver_states,  // get(index) function pointer
    fetch_function__RobotDriverState__driver_states,  // fetch(index, &value) function pointer
    assign_function__RobotDriverState__driver_states,  // assign(index, value) function pointer
    resize_function__RobotDriverState__driver_states  // resize(index) function pointer
  },
  {
    "error",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(husarion_ugv_msgs::msg::RobotDriverState, error),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "write_pdo_cmds_error",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(husarion_ugv_msgs::msg::RobotDriverState, write_pdo_cmds_error),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "read_pdo_motor_states_error",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(husarion_ugv_msgs::msg::RobotDriverState, read_pdo_motor_states_error),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "read_pdo_driver_state_error",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(husarion_ugv_msgs::msg::RobotDriverState, read_pdo_driver_state_error),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers RobotDriverState_message_members = {
  "husarion_ugv_msgs::msg",  // message namespace
  "RobotDriverState",  // message name
  6,  // number of fields
  sizeof(husarion_ugv_msgs::msg::RobotDriverState),
  false,  // has_any_key_member_
  RobotDriverState_message_member_array,  // message members
  RobotDriverState_init_function,  // function to initialize message memory (memory has to be allocated)
  RobotDriverState_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t RobotDriverState_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &RobotDriverState_message_members,
  get_message_typesupport_handle_function,
  &husarion_ugv_msgs__msg__RobotDriverState__get_type_hash,
  &husarion_ugv_msgs__msg__RobotDriverState__get_type_description,
  &husarion_ugv_msgs__msg__RobotDriverState__get_type_description_sources,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace msg

}  // namespace husarion_ugv_msgs


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<husarion_ugv_msgs::msg::RobotDriverState>()
{
  return &::husarion_ugv_msgs::msg::rosidl_typesupport_introspection_cpp::RobotDriverState_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, husarion_ugv_msgs, msg, RobotDriverState)() {
  return &::husarion_ugv_msgs::msg::rosidl_typesupport_introspection_cpp::RobotDriverState_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
