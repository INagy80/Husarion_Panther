// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from husarion_ugv_msgs:msg/RuntimeError.idl
// generated code does not contain a copyright notice
#include "husarion_ugv_msgs/msg/detail/runtime_error__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <cstddef>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/serialization_helpers.hpp"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "husarion_ugv_msgs/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "husarion_ugv_msgs/msg/detail/runtime_error__struct.h"
#include "husarion_ugv_msgs/msg/detail/runtime_error__functions.h"
#include "fastcdr/Cdr.h"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

// includes and forward declarations of message dependencies and their conversion functions

#if defined(__cplusplus)
extern "C"
{
#endif


// forward declare type support functions


using _RuntimeError__ros_msg_type = husarion_ugv_msgs__msg__RuntimeError;


ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_husarion_ugv_msgs
bool cdr_serialize_husarion_ugv_msgs__msg__RuntimeError(
  const husarion_ugv_msgs__msg__RuntimeError * ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Field name: amps_limit_active
  {
    cdr << (ros_message->amps_limit_active ? true : false);
  }

  // Field name: motor_stall
  {
    cdr << (ros_message->motor_stall ? true : false);
  }

  // Field name: loop_error
  {
    cdr << (ros_message->loop_error ? true : false);
  }

  // Field name: safety_stop_active
  {
    cdr << (ros_message->safety_stop_active ? true : false);
  }

  // Field name: forward_limit_triggered
  {
    cdr << (ros_message->forward_limit_triggered ? true : false);
  }

  // Field name: reverse_limit_triggered
  {
    cdr << (ros_message->reverse_limit_triggered ? true : false);
  }

  // Field name: amps_trigger_activated
  {
    cdr << (ros_message->amps_trigger_activated ? true : false);
  }

  return true;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_husarion_ugv_msgs
bool cdr_deserialize_husarion_ugv_msgs__msg__RuntimeError(
  eprosima::fastcdr::Cdr & cdr,
  husarion_ugv_msgs__msg__RuntimeError * ros_message)
{
  // Field name: amps_limit_active
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->amps_limit_active = tmp ? true : false;
  }

  // Field name: motor_stall
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->motor_stall = tmp ? true : false;
  }

  // Field name: loop_error
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->loop_error = tmp ? true : false;
  }

  // Field name: safety_stop_active
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->safety_stop_active = tmp ? true : false;
  }

  // Field name: forward_limit_triggered
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->forward_limit_triggered = tmp ? true : false;
  }

  // Field name: reverse_limit_triggered
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->reverse_limit_triggered = tmp ? true : false;
  }

  // Field name: amps_trigger_activated
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->amps_trigger_activated = tmp ? true : false;
  }

  return true;
}  // NOLINT(readability/fn_size)


ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_husarion_ugv_msgs
size_t get_serialized_size_husarion_ugv_msgs__msg__RuntimeError(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _RuntimeError__ros_msg_type * ros_message = static_cast<const _RuntimeError__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Field name: amps_limit_active
  {
    size_t item_size = sizeof(ros_message->amps_limit_active);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: motor_stall
  {
    size_t item_size = sizeof(ros_message->motor_stall);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: loop_error
  {
    size_t item_size = sizeof(ros_message->loop_error);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: safety_stop_active
  {
    size_t item_size = sizeof(ros_message->safety_stop_active);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: forward_limit_triggered
  {
    size_t item_size = sizeof(ros_message->forward_limit_triggered);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: reverse_limit_triggered
  {
    size_t item_size = sizeof(ros_message->reverse_limit_triggered);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: amps_trigger_activated
  {
    size_t item_size = sizeof(ros_message->amps_trigger_activated);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}


ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_husarion_ugv_msgs
size_t max_serialized_size_husarion_ugv_msgs__msg__RuntimeError(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  size_t last_member_size = 0;
  (void)last_member_size;
  (void)padding;
  (void)wchar_size;

  full_bounded = true;
  is_plain = true;

  // Field name: amps_limit_active
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Field name: motor_stall
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Field name: loop_error
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Field name: safety_stop_active
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Field name: forward_limit_triggered
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Field name: reverse_limit_triggered
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Field name: amps_trigger_activated
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }


  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = husarion_ugv_msgs__msg__RuntimeError;
    is_plain =
      (
      offsetof(DataType, amps_trigger_activated) +
      last_member_size
      ) == ret_val;
  }
  return ret_val;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_husarion_ugv_msgs
bool cdr_serialize_key_husarion_ugv_msgs__msg__RuntimeError(
  const husarion_ugv_msgs__msg__RuntimeError * ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Field name: amps_limit_active
  {
    cdr << (ros_message->amps_limit_active ? true : false);
  }

  // Field name: motor_stall
  {
    cdr << (ros_message->motor_stall ? true : false);
  }

  // Field name: loop_error
  {
    cdr << (ros_message->loop_error ? true : false);
  }

  // Field name: safety_stop_active
  {
    cdr << (ros_message->safety_stop_active ? true : false);
  }

  // Field name: forward_limit_triggered
  {
    cdr << (ros_message->forward_limit_triggered ? true : false);
  }

  // Field name: reverse_limit_triggered
  {
    cdr << (ros_message->reverse_limit_triggered ? true : false);
  }

  // Field name: amps_trigger_activated
  {
    cdr << (ros_message->amps_trigger_activated ? true : false);
  }

  return true;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_husarion_ugv_msgs
size_t get_serialized_size_key_husarion_ugv_msgs__msg__RuntimeError(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _RuntimeError__ros_msg_type * ros_message = static_cast<const _RuntimeError__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;

  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Field name: amps_limit_active
  {
    size_t item_size = sizeof(ros_message->amps_limit_active);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: motor_stall
  {
    size_t item_size = sizeof(ros_message->motor_stall);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: loop_error
  {
    size_t item_size = sizeof(ros_message->loop_error);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: safety_stop_active
  {
    size_t item_size = sizeof(ros_message->safety_stop_active);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: forward_limit_triggered
  {
    size_t item_size = sizeof(ros_message->forward_limit_triggered);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: reverse_limit_triggered
  {
    size_t item_size = sizeof(ros_message->reverse_limit_triggered);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: amps_trigger_activated
  {
    size_t item_size = sizeof(ros_message->amps_trigger_activated);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_husarion_ugv_msgs
size_t max_serialized_size_key_husarion_ugv_msgs__msg__RuntimeError(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  size_t last_member_size = 0;
  (void)last_member_size;
  (void)padding;
  (void)wchar_size;

  full_bounded = true;
  is_plain = true;
  // Field name: amps_limit_active
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Field name: motor_stall
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Field name: loop_error
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Field name: safety_stop_active
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Field name: forward_limit_triggered
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Field name: reverse_limit_triggered
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Field name: amps_trigger_activated
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = husarion_ugv_msgs__msg__RuntimeError;
    is_plain =
      (
      offsetof(DataType, amps_trigger_activated) +
      last_member_size
      ) == ret_val;
  }
  return ret_val;
}


static bool _RuntimeError__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const husarion_ugv_msgs__msg__RuntimeError * ros_message = static_cast<const husarion_ugv_msgs__msg__RuntimeError *>(untyped_ros_message);
  (void)ros_message;
  return cdr_serialize_husarion_ugv_msgs__msg__RuntimeError(ros_message, cdr);
}

static bool _RuntimeError__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  husarion_ugv_msgs__msg__RuntimeError * ros_message = static_cast<husarion_ugv_msgs__msg__RuntimeError *>(untyped_ros_message);
  (void)ros_message;
  return cdr_deserialize_husarion_ugv_msgs__msg__RuntimeError(cdr, ros_message);
}

static uint32_t _RuntimeError__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_husarion_ugv_msgs__msg__RuntimeError(
      untyped_ros_message, 0));
}

static size_t _RuntimeError__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_husarion_ugv_msgs__msg__RuntimeError(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_RuntimeError = {
  "husarion_ugv_msgs::msg",
  "RuntimeError",
  _RuntimeError__cdr_serialize,
  _RuntimeError__cdr_deserialize,
  _RuntimeError__get_serialized_size,
  _RuntimeError__max_serialized_size,
  nullptr
};

static rosidl_message_type_support_t _RuntimeError__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_RuntimeError,
  get_message_typesupport_handle_function,
  &husarion_ugv_msgs__msg__RuntimeError__get_type_hash,
  &husarion_ugv_msgs__msg__RuntimeError__get_type_description,
  &husarion_ugv_msgs__msg__RuntimeError__get_type_description_sources,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, husarion_ugv_msgs, msg, RuntimeError)() {
  return &_RuntimeError__type_support;
}

#if defined(__cplusplus)
}
#endif
