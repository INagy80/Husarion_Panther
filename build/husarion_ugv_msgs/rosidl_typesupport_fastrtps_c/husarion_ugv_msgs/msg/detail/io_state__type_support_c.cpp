// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from husarion_ugv_msgs:msg/IOState.idl
// generated code does not contain a copyright notice
#include "husarion_ugv_msgs/msg/detail/io_state__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <cstddef>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/serialization_helpers.hpp"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "husarion_ugv_msgs/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "husarion_ugv_msgs/msg/detail/io_state__struct.h"
#include "husarion_ugv_msgs/msg/detail/io_state__functions.h"
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


using _IOState__ros_msg_type = husarion_ugv_msgs__msg__IOState;


ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_husarion_ugv_msgs
bool cdr_serialize_husarion_ugv_msgs__msg__IOState(
  const husarion_ugv_msgs__msg__IOState * ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Field name: aux_power
  {
    cdr << (ros_message->aux_power ? true : false);
  }

  // Field name: charger_connected
  {
    cdr << (ros_message->charger_connected ? true : false);
  }

  // Field name: charger_enabled
  {
    cdr << (ros_message->charger_enabled ? true : false);
  }

  // Field name: digital_power
  {
    cdr << (ros_message->digital_power ? true : false);
  }

  // Field name: fan
  {
    cdr << (ros_message->fan ? true : false);
  }

  // Field name: led_control
  {
    cdr << (ros_message->led_control ? true : false);
  }

  // Field name: motor_on
  {
    cdr << (ros_message->motor_on ? true : false);
  }

  // Field name: power_button
  {
    cdr << (ros_message->power_button ? true : false);
  }

  return true;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_husarion_ugv_msgs
bool cdr_deserialize_husarion_ugv_msgs__msg__IOState(
  eprosima::fastcdr::Cdr & cdr,
  husarion_ugv_msgs__msg__IOState * ros_message)
{
  // Field name: aux_power
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->aux_power = tmp ? true : false;
  }

  // Field name: charger_connected
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->charger_connected = tmp ? true : false;
  }

  // Field name: charger_enabled
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->charger_enabled = tmp ? true : false;
  }

  // Field name: digital_power
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->digital_power = tmp ? true : false;
  }

  // Field name: fan
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->fan = tmp ? true : false;
  }

  // Field name: led_control
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->led_control = tmp ? true : false;
  }

  // Field name: motor_on
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->motor_on = tmp ? true : false;
  }

  // Field name: power_button
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->power_button = tmp ? true : false;
  }

  return true;
}  // NOLINT(readability/fn_size)


ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_husarion_ugv_msgs
size_t get_serialized_size_husarion_ugv_msgs__msg__IOState(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _IOState__ros_msg_type * ros_message = static_cast<const _IOState__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Field name: aux_power
  {
    size_t item_size = sizeof(ros_message->aux_power);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: charger_connected
  {
    size_t item_size = sizeof(ros_message->charger_connected);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: charger_enabled
  {
    size_t item_size = sizeof(ros_message->charger_enabled);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: digital_power
  {
    size_t item_size = sizeof(ros_message->digital_power);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: fan
  {
    size_t item_size = sizeof(ros_message->fan);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: led_control
  {
    size_t item_size = sizeof(ros_message->led_control);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: motor_on
  {
    size_t item_size = sizeof(ros_message->motor_on);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: power_button
  {
    size_t item_size = sizeof(ros_message->power_button);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}


ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_husarion_ugv_msgs
size_t max_serialized_size_husarion_ugv_msgs__msg__IOState(
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

  // Field name: aux_power
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Field name: charger_connected
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Field name: charger_enabled
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Field name: digital_power
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Field name: fan
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Field name: led_control
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Field name: motor_on
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Field name: power_button
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
    using DataType = husarion_ugv_msgs__msg__IOState;
    is_plain =
      (
      offsetof(DataType, power_button) +
      last_member_size
      ) == ret_val;
  }
  return ret_val;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_husarion_ugv_msgs
bool cdr_serialize_key_husarion_ugv_msgs__msg__IOState(
  const husarion_ugv_msgs__msg__IOState * ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Field name: aux_power
  {
    cdr << (ros_message->aux_power ? true : false);
  }

  // Field name: charger_connected
  {
    cdr << (ros_message->charger_connected ? true : false);
  }

  // Field name: charger_enabled
  {
    cdr << (ros_message->charger_enabled ? true : false);
  }

  // Field name: digital_power
  {
    cdr << (ros_message->digital_power ? true : false);
  }

  // Field name: fan
  {
    cdr << (ros_message->fan ? true : false);
  }

  // Field name: led_control
  {
    cdr << (ros_message->led_control ? true : false);
  }

  // Field name: motor_on
  {
    cdr << (ros_message->motor_on ? true : false);
  }

  // Field name: power_button
  {
    cdr << (ros_message->power_button ? true : false);
  }

  return true;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_husarion_ugv_msgs
size_t get_serialized_size_key_husarion_ugv_msgs__msg__IOState(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _IOState__ros_msg_type * ros_message = static_cast<const _IOState__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;

  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Field name: aux_power
  {
    size_t item_size = sizeof(ros_message->aux_power);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: charger_connected
  {
    size_t item_size = sizeof(ros_message->charger_connected);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: charger_enabled
  {
    size_t item_size = sizeof(ros_message->charger_enabled);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: digital_power
  {
    size_t item_size = sizeof(ros_message->digital_power);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: fan
  {
    size_t item_size = sizeof(ros_message->fan);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: led_control
  {
    size_t item_size = sizeof(ros_message->led_control);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: motor_on
  {
    size_t item_size = sizeof(ros_message->motor_on);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: power_button
  {
    size_t item_size = sizeof(ros_message->power_button);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_husarion_ugv_msgs
size_t max_serialized_size_key_husarion_ugv_msgs__msg__IOState(
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
  // Field name: aux_power
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Field name: charger_connected
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Field name: charger_enabled
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Field name: digital_power
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Field name: fan
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Field name: led_control
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Field name: motor_on
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Field name: power_button
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
    using DataType = husarion_ugv_msgs__msg__IOState;
    is_plain =
      (
      offsetof(DataType, power_button) +
      last_member_size
      ) == ret_val;
  }
  return ret_val;
}


static bool _IOState__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const husarion_ugv_msgs__msg__IOState * ros_message = static_cast<const husarion_ugv_msgs__msg__IOState *>(untyped_ros_message);
  (void)ros_message;
  return cdr_serialize_husarion_ugv_msgs__msg__IOState(ros_message, cdr);
}

static bool _IOState__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  husarion_ugv_msgs__msg__IOState * ros_message = static_cast<husarion_ugv_msgs__msg__IOState *>(untyped_ros_message);
  (void)ros_message;
  return cdr_deserialize_husarion_ugv_msgs__msg__IOState(cdr, ros_message);
}

static uint32_t _IOState__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_husarion_ugv_msgs__msg__IOState(
      untyped_ros_message, 0));
}

static size_t _IOState__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_husarion_ugv_msgs__msg__IOState(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_IOState = {
  "husarion_ugv_msgs::msg",
  "IOState",
  _IOState__cdr_serialize,
  _IOState__cdr_deserialize,
  _IOState__get_serialized_size,
  _IOState__max_serialized_size,
  nullptr
};

static rosidl_message_type_support_t _IOState__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_IOState,
  get_message_typesupport_handle_function,
  &husarion_ugv_msgs__msg__IOState__get_type_hash,
  &husarion_ugv_msgs__msg__IOState__get_type_description,
  &husarion_ugv_msgs__msg__IOState__get_type_description_sources,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, husarion_ugv_msgs, msg, IOState)() {
  return &_IOState__type_support;
}

#if defined(__cplusplus)
}
#endif
