// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from husarion_ugv_msgs:msg/ScriptFlag.idl
// generated code does not contain a copyright notice
#include "husarion_ugv_msgs/msg/detail/script_flag__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <cstddef>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/serialization_helpers.hpp"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "husarion_ugv_msgs/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "husarion_ugv_msgs/msg/detail/script_flag__struct.h"
#include "husarion_ugv_msgs/msg/detail/script_flag__functions.h"
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


using _ScriptFlag__ros_msg_type = husarion_ugv_msgs__msg__ScriptFlag;


ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_husarion_ugv_msgs
bool cdr_serialize_husarion_ugv_msgs__msg__ScriptFlag(
  const husarion_ugv_msgs__msg__ScriptFlag * ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Field name: loop_error
  {
    cdr << (ros_message->loop_error ? true : false);
  }

  // Field name: encoder_disconnected
  {
    cdr << (ros_message->encoder_disconnected ? true : false);
  }

  // Field name: amp_limiter
  {
    cdr << (ros_message->amp_limiter ? true : false);
  }

  return true;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_husarion_ugv_msgs
bool cdr_deserialize_husarion_ugv_msgs__msg__ScriptFlag(
  eprosima::fastcdr::Cdr & cdr,
  husarion_ugv_msgs__msg__ScriptFlag * ros_message)
{
  // Field name: loop_error
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->loop_error = tmp ? true : false;
  }

  // Field name: encoder_disconnected
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->encoder_disconnected = tmp ? true : false;
  }

  // Field name: amp_limiter
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->amp_limiter = tmp ? true : false;
  }

  return true;
}  // NOLINT(readability/fn_size)


ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_husarion_ugv_msgs
size_t get_serialized_size_husarion_ugv_msgs__msg__ScriptFlag(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _ScriptFlag__ros_msg_type * ros_message = static_cast<const _ScriptFlag__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Field name: loop_error
  {
    size_t item_size = sizeof(ros_message->loop_error);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: encoder_disconnected
  {
    size_t item_size = sizeof(ros_message->encoder_disconnected);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: amp_limiter
  {
    size_t item_size = sizeof(ros_message->amp_limiter);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}


ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_husarion_ugv_msgs
size_t max_serialized_size_husarion_ugv_msgs__msg__ScriptFlag(
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

  // Field name: loop_error
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Field name: encoder_disconnected
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Field name: amp_limiter
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
    using DataType = husarion_ugv_msgs__msg__ScriptFlag;
    is_plain =
      (
      offsetof(DataType, amp_limiter) +
      last_member_size
      ) == ret_val;
  }
  return ret_val;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_husarion_ugv_msgs
bool cdr_serialize_key_husarion_ugv_msgs__msg__ScriptFlag(
  const husarion_ugv_msgs__msg__ScriptFlag * ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Field name: loop_error
  {
    cdr << (ros_message->loop_error ? true : false);
  }

  // Field name: encoder_disconnected
  {
    cdr << (ros_message->encoder_disconnected ? true : false);
  }

  // Field name: amp_limiter
  {
    cdr << (ros_message->amp_limiter ? true : false);
  }

  return true;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_husarion_ugv_msgs
size_t get_serialized_size_key_husarion_ugv_msgs__msg__ScriptFlag(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _ScriptFlag__ros_msg_type * ros_message = static_cast<const _ScriptFlag__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;

  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Field name: loop_error
  {
    size_t item_size = sizeof(ros_message->loop_error);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: encoder_disconnected
  {
    size_t item_size = sizeof(ros_message->encoder_disconnected);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: amp_limiter
  {
    size_t item_size = sizeof(ros_message->amp_limiter);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_husarion_ugv_msgs
size_t max_serialized_size_key_husarion_ugv_msgs__msg__ScriptFlag(
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
  // Field name: loop_error
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Field name: encoder_disconnected
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Field name: amp_limiter
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
    using DataType = husarion_ugv_msgs__msg__ScriptFlag;
    is_plain =
      (
      offsetof(DataType, amp_limiter) +
      last_member_size
      ) == ret_val;
  }
  return ret_val;
}


static bool _ScriptFlag__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const husarion_ugv_msgs__msg__ScriptFlag * ros_message = static_cast<const husarion_ugv_msgs__msg__ScriptFlag *>(untyped_ros_message);
  (void)ros_message;
  return cdr_serialize_husarion_ugv_msgs__msg__ScriptFlag(ros_message, cdr);
}

static bool _ScriptFlag__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  husarion_ugv_msgs__msg__ScriptFlag * ros_message = static_cast<husarion_ugv_msgs__msg__ScriptFlag *>(untyped_ros_message);
  (void)ros_message;
  return cdr_deserialize_husarion_ugv_msgs__msg__ScriptFlag(cdr, ros_message);
}

static uint32_t _ScriptFlag__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_husarion_ugv_msgs__msg__ScriptFlag(
      untyped_ros_message, 0));
}

static size_t _ScriptFlag__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_husarion_ugv_msgs__msg__ScriptFlag(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_ScriptFlag = {
  "husarion_ugv_msgs::msg",
  "ScriptFlag",
  _ScriptFlag__cdr_serialize,
  _ScriptFlag__cdr_deserialize,
  _ScriptFlag__get_serialized_size,
  _ScriptFlag__max_serialized_size,
  nullptr
};

static rosidl_message_type_support_t _ScriptFlag__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_ScriptFlag,
  get_message_typesupport_handle_function,
  &husarion_ugv_msgs__msg__ScriptFlag__get_type_hash,
  &husarion_ugv_msgs__msg__ScriptFlag__get_type_description,
  &husarion_ugv_msgs__msg__ScriptFlag__get_type_description_sources,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, husarion_ugv_msgs, msg, ScriptFlag)() {
  return &_ScriptFlag__type_support;
}

#if defined(__cplusplus)
}
#endif
