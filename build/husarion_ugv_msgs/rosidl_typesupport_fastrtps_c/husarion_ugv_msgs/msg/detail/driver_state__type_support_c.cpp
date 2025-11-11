// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from husarion_ugv_msgs:msg/DriverState.idl
// generated code does not contain a copyright notice
#include "husarion_ugv_msgs/msg/detail/driver_state__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <cstddef>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/serialization_helpers.hpp"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "husarion_ugv_msgs/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "husarion_ugv_msgs/msg/detail/driver_state__struct.h"
#include "husarion_ugv_msgs/msg/detail/driver_state__functions.h"
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

#include "husarion_ugv_msgs/msg/detail/fault_flag__functions.h"  // fault_flag
#include "husarion_ugv_msgs/msg/detail/runtime_error__functions.h"  // channel_1_motor_runtime_error, channel_2_motor_runtime_error
#include "husarion_ugv_msgs/msg/detail/script_flag__functions.h"  // script_flag

// forward declare type support functions

bool cdr_serialize_husarion_ugv_msgs__msg__FaultFlag(
  const husarion_ugv_msgs__msg__FaultFlag * ros_message,
  eprosima::fastcdr::Cdr & cdr);

bool cdr_deserialize_husarion_ugv_msgs__msg__FaultFlag(
  eprosima::fastcdr::Cdr & cdr,
  husarion_ugv_msgs__msg__FaultFlag * ros_message);

size_t get_serialized_size_husarion_ugv_msgs__msg__FaultFlag(
  const void * untyped_ros_message,
  size_t current_alignment);

size_t max_serialized_size_husarion_ugv_msgs__msg__FaultFlag(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

bool cdr_serialize_key_husarion_ugv_msgs__msg__FaultFlag(
  const husarion_ugv_msgs__msg__FaultFlag * ros_message,
  eprosima::fastcdr::Cdr & cdr);

size_t get_serialized_size_key_husarion_ugv_msgs__msg__FaultFlag(
  const void * untyped_ros_message,
  size_t current_alignment);

size_t max_serialized_size_key_husarion_ugv_msgs__msg__FaultFlag(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, husarion_ugv_msgs, msg, FaultFlag)();

bool cdr_serialize_husarion_ugv_msgs__msg__RuntimeError(
  const husarion_ugv_msgs__msg__RuntimeError * ros_message,
  eprosima::fastcdr::Cdr & cdr);

bool cdr_deserialize_husarion_ugv_msgs__msg__RuntimeError(
  eprosima::fastcdr::Cdr & cdr,
  husarion_ugv_msgs__msg__RuntimeError * ros_message);

size_t get_serialized_size_husarion_ugv_msgs__msg__RuntimeError(
  const void * untyped_ros_message,
  size_t current_alignment);

size_t max_serialized_size_husarion_ugv_msgs__msg__RuntimeError(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

bool cdr_serialize_key_husarion_ugv_msgs__msg__RuntimeError(
  const husarion_ugv_msgs__msg__RuntimeError * ros_message,
  eprosima::fastcdr::Cdr & cdr);

size_t get_serialized_size_key_husarion_ugv_msgs__msg__RuntimeError(
  const void * untyped_ros_message,
  size_t current_alignment);

size_t max_serialized_size_key_husarion_ugv_msgs__msg__RuntimeError(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, husarion_ugv_msgs, msg, RuntimeError)();

bool cdr_serialize_husarion_ugv_msgs__msg__ScriptFlag(
  const husarion_ugv_msgs__msg__ScriptFlag * ros_message,
  eprosima::fastcdr::Cdr & cdr);

bool cdr_deserialize_husarion_ugv_msgs__msg__ScriptFlag(
  eprosima::fastcdr::Cdr & cdr,
  husarion_ugv_msgs__msg__ScriptFlag * ros_message);

size_t get_serialized_size_husarion_ugv_msgs__msg__ScriptFlag(
  const void * untyped_ros_message,
  size_t current_alignment);

size_t max_serialized_size_husarion_ugv_msgs__msg__ScriptFlag(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

bool cdr_serialize_key_husarion_ugv_msgs__msg__ScriptFlag(
  const husarion_ugv_msgs__msg__ScriptFlag * ros_message,
  eprosima::fastcdr::Cdr & cdr);

size_t get_serialized_size_key_husarion_ugv_msgs__msg__ScriptFlag(
  const void * untyped_ros_message,
  size_t current_alignment);

size_t max_serialized_size_key_husarion_ugv_msgs__msg__ScriptFlag(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, husarion_ugv_msgs, msg, ScriptFlag)();


using _DriverState__ros_msg_type = husarion_ugv_msgs__msg__DriverState;


ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_husarion_ugv_msgs
bool cdr_serialize_husarion_ugv_msgs__msg__DriverState(
  const husarion_ugv_msgs__msg__DriverState * ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Field name: voltage
  {
    cdr << ros_message->voltage;
  }

  // Field name: current
  {
    cdr << ros_message->current;
  }

  // Field name: temperature
  {
    cdr << ros_message->temperature;
  }

  // Field name: heatsink_temperature
  {
    cdr << ros_message->heatsink_temperature;
  }

  // Field name: fault_flag
  {
    cdr_serialize_husarion_ugv_msgs__msg__FaultFlag(
      &ros_message->fault_flag, cdr);
  }

  // Field name: script_flag
  {
    cdr_serialize_husarion_ugv_msgs__msg__ScriptFlag(
      &ros_message->script_flag, cdr);
  }

  // Field name: channel_1_motor_runtime_error
  {
    cdr_serialize_husarion_ugv_msgs__msg__RuntimeError(
      &ros_message->channel_1_motor_runtime_error, cdr);
  }

  // Field name: channel_2_motor_runtime_error
  {
    cdr_serialize_husarion_ugv_msgs__msg__RuntimeError(
      &ros_message->channel_2_motor_runtime_error, cdr);
  }

  // Field name: motor_states_data_timed_out
  {
    cdr << (ros_message->motor_states_data_timed_out ? true : false);
  }

  // Field name: driver_state_data_timed_out
  {
    cdr << (ros_message->driver_state_data_timed_out ? true : false);
  }

  // Field name: can_error
  {
    cdr << (ros_message->can_error ? true : false);
  }

  // Field name: heartbeat_timeout
  {
    cdr << (ros_message->heartbeat_timeout ? true : false);
  }

  return true;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_husarion_ugv_msgs
bool cdr_deserialize_husarion_ugv_msgs__msg__DriverState(
  eprosima::fastcdr::Cdr & cdr,
  husarion_ugv_msgs__msg__DriverState * ros_message)
{
  // Field name: voltage
  {
    cdr >> ros_message->voltage;
  }

  // Field name: current
  {
    cdr >> ros_message->current;
  }

  // Field name: temperature
  {
    cdr >> ros_message->temperature;
  }

  // Field name: heatsink_temperature
  {
    cdr >> ros_message->heatsink_temperature;
  }

  // Field name: fault_flag
  {
    cdr_deserialize_husarion_ugv_msgs__msg__FaultFlag(cdr, &ros_message->fault_flag);
  }

  // Field name: script_flag
  {
    cdr_deserialize_husarion_ugv_msgs__msg__ScriptFlag(cdr, &ros_message->script_flag);
  }

  // Field name: channel_1_motor_runtime_error
  {
    cdr_deserialize_husarion_ugv_msgs__msg__RuntimeError(cdr, &ros_message->channel_1_motor_runtime_error);
  }

  // Field name: channel_2_motor_runtime_error
  {
    cdr_deserialize_husarion_ugv_msgs__msg__RuntimeError(cdr, &ros_message->channel_2_motor_runtime_error);
  }

  // Field name: motor_states_data_timed_out
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->motor_states_data_timed_out = tmp ? true : false;
  }

  // Field name: driver_state_data_timed_out
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->driver_state_data_timed_out = tmp ? true : false;
  }

  // Field name: can_error
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->can_error = tmp ? true : false;
  }

  // Field name: heartbeat_timeout
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->heartbeat_timeout = tmp ? true : false;
  }

  return true;
}  // NOLINT(readability/fn_size)


ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_husarion_ugv_msgs
size_t get_serialized_size_husarion_ugv_msgs__msg__DriverState(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _DriverState__ros_msg_type * ros_message = static_cast<const _DriverState__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Field name: voltage
  {
    size_t item_size = sizeof(ros_message->voltage);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: current
  {
    size_t item_size = sizeof(ros_message->current);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: temperature
  {
    size_t item_size = sizeof(ros_message->temperature);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: heatsink_temperature
  {
    size_t item_size = sizeof(ros_message->heatsink_temperature);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: fault_flag
  current_alignment += get_serialized_size_husarion_ugv_msgs__msg__FaultFlag(
    &(ros_message->fault_flag), current_alignment);

  // Field name: script_flag
  current_alignment += get_serialized_size_husarion_ugv_msgs__msg__ScriptFlag(
    &(ros_message->script_flag), current_alignment);

  // Field name: channel_1_motor_runtime_error
  current_alignment += get_serialized_size_husarion_ugv_msgs__msg__RuntimeError(
    &(ros_message->channel_1_motor_runtime_error), current_alignment);

  // Field name: channel_2_motor_runtime_error
  current_alignment += get_serialized_size_husarion_ugv_msgs__msg__RuntimeError(
    &(ros_message->channel_2_motor_runtime_error), current_alignment);

  // Field name: motor_states_data_timed_out
  {
    size_t item_size = sizeof(ros_message->motor_states_data_timed_out);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: driver_state_data_timed_out
  {
    size_t item_size = sizeof(ros_message->driver_state_data_timed_out);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: can_error
  {
    size_t item_size = sizeof(ros_message->can_error);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: heartbeat_timeout
  {
    size_t item_size = sizeof(ros_message->heartbeat_timeout);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}


ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_husarion_ugv_msgs
size_t max_serialized_size_husarion_ugv_msgs__msg__DriverState(
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

  // Field name: voltage
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Field name: current
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Field name: temperature
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Field name: heatsink_temperature
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Field name: fault_flag
  {
    size_t array_size = 1;
    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size;
      inner_size =
        max_serialized_size_husarion_ugv_msgs__msg__FaultFlag(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }

  // Field name: script_flag
  {
    size_t array_size = 1;
    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size;
      inner_size =
        max_serialized_size_husarion_ugv_msgs__msg__ScriptFlag(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }

  // Field name: channel_1_motor_runtime_error
  {
    size_t array_size = 1;
    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size;
      inner_size =
        max_serialized_size_husarion_ugv_msgs__msg__RuntimeError(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }

  // Field name: channel_2_motor_runtime_error
  {
    size_t array_size = 1;
    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size;
      inner_size =
        max_serialized_size_husarion_ugv_msgs__msg__RuntimeError(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }

  // Field name: motor_states_data_timed_out
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Field name: driver_state_data_timed_out
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Field name: can_error
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Field name: heartbeat_timeout
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
    using DataType = husarion_ugv_msgs__msg__DriverState;
    is_plain =
      (
      offsetof(DataType, heartbeat_timeout) +
      last_member_size
      ) == ret_val;
  }
  return ret_val;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_husarion_ugv_msgs
bool cdr_serialize_key_husarion_ugv_msgs__msg__DriverState(
  const husarion_ugv_msgs__msg__DriverState * ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Field name: voltage
  {
    cdr << ros_message->voltage;
  }

  // Field name: current
  {
    cdr << ros_message->current;
  }

  // Field name: temperature
  {
    cdr << ros_message->temperature;
  }

  // Field name: heatsink_temperature
  {
    cdr << ros_message->heatsink_temperature;
  }

  // Field name: fault_flag
  {
    cdr_serialize_key_husarion_ugv_msgs__msg__FaultFlag(
      &ros_message->fault_flag, cdr);
  }

  // Field name: script_flag
  {
    cdr_serialize_key_husarion_ugv_msgs__msg__ScriptFlag(
      &ros_message->script_flag, cdr);
  }

  // Field name: channel_1_motor_runtime_error
  {
    cdr_serialize_key_husarion_ugv_msgs__msg__RuntimeError(
      &ros_message->channel_1_motor_runtime_error, cdr);
  }

  // Field name: channel_2_motor_runtime_error
  {
    cdr_serialize_key_husarion_ugv_msgs__msg__RuntimeError(
      &ros_message->channel_2_motor_runtime_error, cdr);
  }

  // Field name: motor_states_data_timed_out
  {
    cdr << (ros_message->motor_states_data_timed_out ? true : false);
  }

  // Field name: driver_state_data_timed_out
  {
    cdr << (ros_message->driver_state_data_timed_out ? true : false);
  }

  // Field name: can_error
  {
    cdr << (ros_message->can_error ? true : false);
  }

  // Field name: heartbeat_timeout
  {
    cdr << (ros_message->heartbeat_timeout ? true : false);
  }

  return true;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_husarion_ugv_msgs
size_t get_serialized_size_key_husarion_ugv_msgs__msg__DriverState(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _DriverState__ros_msg_type * ros_message = static_cast<const _DriverState__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;

  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Field name: voltage
  {
    size_t item_size = sizeof(ros_message->voltage);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: current
  {
    size_t item_size = sizeof(ros_message->current);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: temperature
  {
    size_t item_size = sizeof(ros_message->temperature);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: heatsink_temperature
  {
    size_t item_size = sizeof(ros_message->heatsink_temperature);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: fault_flag
  current_alignment += get_serialized_size_key_husarion_ugv_msgs__msg__FaultFlag(
    &(ros_message->fault_flag), current_alignment);

  // Field name: script_flag
  current_alignment += get_serialized_size_key_husarion_ugv_msgs__msg__ScriptFlag(
    &(ros_message->script_flag), current_alignment);

  // Field name: channel_1_motor_runtime_error
  current_alignment += get_serialized_size_key_husarion_ugv_msgs__msg__RuntimeError(
    &(ros_message->channel_1_motor_runtime_error), current_alignment);

  // Field name: channel_2_motor_runtime_error
  current_alignment += get_serialized_size_key_husarion_ugv_msgs__msg__RuntimeError(
    &(ros_message->channel_2_motor_runtime_error), current_alignment);

  // Field name: motor_states_data_timed_out
  {
    size_t item_size = sizeof(ros_message->motor_states_data_timed_out);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: driver_state_data_timed_out
  {
    size_t item_size = sizeof(ros_message->driver_state_data_timed_out);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: can_error
  {
    size_t item_size = sizeof(ros_message->can_error);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: heartbeat_timeout
  {
    size_t item_size = sizeof(ros_message->heartbeat_timeout);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_husarion_ugv_msgs
size_t max_serialized_size_key_husarion_ugv_msgs__msg__DriverState(
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
  // Field name: voltage
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Field name: current
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Field name: temperature
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Field name: heatsink_temperature
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Field name: fault_flag
  {
    size_t array_size = 1;
    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size;
      inner_size =
        max_serialized_size_key_husarion_ugv_msgs__msg__FaultFlag(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }

  // Field name: script_flag
  {
    size_t array_size = 1;
    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size;
      inner_size =
        max_serialized_size_key_husarion_ugv_msgs__msg__ScriptFlag(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }

  // Field name: channel_1_motor_runtime_error
  {
    size_t array_size = 1;
    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size;
      inner_size =
        max_serialized_size_key_husarion_ugv_msgs__msg__RuntimeError(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }

  // Field name: channel_2_motor_runtime_error
  {
    size_t array_size = 1;
    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size;
      inner_size =
        max_serialized_size_key_husarion_ugv_msgs__msg__RuntimeError(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }

  // Field name: motor_states_data_timed_out
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Field name: driver_state_data_timed_out
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Field name: can_error
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Field name: heartbeat_timeout
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
    using DataType = husarion_ugv_msgs__msg__DriverState;
    is_plain =
      (
      offsetof(DataType, heartbeat_timeout) +
      last_member_size
      ) == ret_val;
  }
  return ret_val;
}


static bool _DriverState__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const husarion_ugv_msgs__msg__DriverState * ros_message = static_cast<const husarion_ugv_msgs__msg__DriverState *>(untyped_ros_message);
  (void)ros_message;
  return cdr_serialize_husarion_ugv_msgs__msg__DriverState(ros_message, cdr);
}

static bool _DriverState__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  husarion_ugv_msgs__msg__DriverState * ros_message = static_cast<husarion_ugv_msgs__msg__DriverState *>(untyped_ros_message);
  (void)ros_message;
  return cdr_deserialize_husarion_ugv_msgs__msg__DriverState(cdr, ros_message);
}

static uint32_t _DriverState__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_husarion_ugv_msgs__msg__DriverState(
      untyped_ros_message, 0));
}

static size_t _DriverState__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_husarion_ugv_msgs__msg__DriverState(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_DriverState = {
  "husarion_ugv_msgs::msg",
  "DriverState",
  _DriverState__cdr_serialize,
  _DriverState__cdr_deserialize,
  _DriverState__get_serialized_size,
  _DriverState__max_serialized_size,
  nullptr
};

static rosidl_message_type_support_t _DriverState__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_DriverState,
  get_message_typesupport_handle_function,
  &husarion_ugv_msgs__msg__DriverState__get_type_hash,
  &husarion_ugv_msgs__msg__DriverState__get_type_description,
  &husarion_ugv_msgs__msg__DriverState__get_type_description_sources,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, husarion_ugv_msgs, msg, DriverState)() {
  return &_DriverState__type_support;
}

#if defined(__cplusplus)
}
#endif
