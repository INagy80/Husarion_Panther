// generated from rosidl_typesupport_fastrtps_c/resource/idl__rosidl_typesupport_fastrtps_c.h.em
// with input from husarion_ugv_msgs:msg/RuntimeError.idl
// generated code does not contain a copyright notice
#ifndef HUSARION_UGV_MSGS__MSG__DETAIL__RUNTIME_ERROR__ROSIDL_TYPESUPPORT_FASTRTPS_C_H_
#define HUSARION_UGV_MSGS__MSG__DETAIL__RUNTIME_ERROR__ROSIDL_TYPESUPPORT_FASTRTPS_C_H_


#include <stddef.h>
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_interface/macros.h"
#include "husarion_ugv_msgs/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "husarion_ugv_msgs/msg/detail/runtime_error__struct.h"
#include "fastcdr/Cdr.h"

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_husarion_ugv_msgs
bool cdr_serialize_husarion_ugv_msgs__msg__RuntimeError(
  const husarion_ugv_msgs__msg__RuntimeError * ros_message,
  eprosima::fastcdr::Cdr & cdr);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_husarion_ugv_msgs
bool cdr_deserialize_husarion_ugv_msgs__msg__RuntimeError(
  eprosima::fastcdr::Cdr &,
  husarion_ugv_msgs__msg__RuntimeError * ros_message);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_husarion_ugv_msgs
size_t get_serialized_size_husarion_ugv_msgs__msg__RuntimeError(
  const void * untyped_ros_message,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_husarion_ugv_msgs
size_t max_serialized_size_husarion_ugv_msgs__msg__RuntimeError(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_husarion_ugv_msgs
bool cdr_serialize_key_husarion_ugv_msgs__msg__RuntimeError(
  const husarion_ugv_msgs__msg__RuntimeError * ros_message,
  eprosima::fastcdr::Cdr & cdr);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_husarion_ugv_msgs
size_t get_serialized_size_key_husarion_ugv_msgs__msg__RuntimeError(
  const void * untyped_ros_message,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_husarion_ugv_msgs
size_t max_serialized_size_key_husarion_ugv_msgs__msg__RuntimeError(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_husarion_ugv_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, husarion_ugv_msgs, msg, RuntimeError)();

#ifdef __cplusplus
}
#endif

#endif  // HUSARION_UGV_MSGS__MSG__DETAIL__RUNTIME_ERROR__ROSIDL_TYPESUPPORT_FASTRTPS_C_H_
