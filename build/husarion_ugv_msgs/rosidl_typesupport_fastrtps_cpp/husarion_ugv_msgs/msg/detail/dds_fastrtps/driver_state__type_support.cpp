// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__type_support.cpp.em
// with input from husarion_ugv_msgs:msg/DriverState.idl
// generated code does not contain a copyright notice
#include "husarion_ugv_msgs/msg/detail/driver_state__rosidl_typesupport_fastrtps_cpp.hpp"
#include "husarion_ugv_msgs/msg/detail/driver_state__functions.h"
#include "husarion_ugv_msgs/msg/detail/driver_state__struct.hpp"

#include <cstddef>
#include <limits>
#include <stdexcept>
#include <string>
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_fastrtps_cpp/identifier.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_fastrtps_cpp/serialization_helpers.hpp"
#include "rosidl_typesupport_fastrtps_cpp/wstring_conversion.hpp"
#include "fastcdr/Cdr.h"


// forward declaration of message dependencies and their conversion functions
namespace husarion_ugv_msgs
{
namespace msg
{
namespace typesupport_fastrtps_cpp
{
bool cdr_serialize(
  const husarion_ugv_msgs::msg::FaultFlag &,
  eprosima::fastcdr::Cdr &);
bool cdr_deserialize(
  eprosima::fastcdr::Cdr &,
  husarion_ugv_msgs::msg::FaultFlag &);
size_t get_serialized_size(
  const husarion_ugv_msgs::msg::FaultFlag &,
  size_t current_alignment);
size_t
max_serialized_size_FaultFlag(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);
bool cdr_serialize_key(
  const husarion_ugv_msgs::msg::FaultFlag &,
  eprosima::fastcdr::Cdr &);
size_t get_serialized_size_key(
  const husarion_ugv_msgs::msg::FaultFlag &,
  size_t current_alignment);
size_t
max_serialized_size_key_FaultFlag(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);
}  // namespace typesupport_fastrtps_cpp
}  // namespace msg
}  // namespace husarion_ugv_msgs

namespace husarion_ugv_msgs
{
namespace msg
{
namespace typesupport_fastrtps_cpp
{
bool cdr_serialize(
  const husarion_ugv_msgs::msg::ScriptFlag &,
  eprosima::fastcdr::Cdr &);
bool cdr_deserialize(
  eprosima::fastcdr::Cdr &,
  husarion_ugv_msgs::msg::ScriptFlag &);
size_t get_serialized_size(
  const husarion_ugv_msgs::msg::ScriptFlag &,
  size_t current_alignment);
size_t
max_serialized_size_ScriptFlag(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);
bool cdr_serialize_key(
  const husarion_ugv_msgs::msg::ScriptFlag &,
  eprosima::fastcdr::Cdr &);
size_t get_serialized_size_key(
  const husarion_ugv_msgs::msg::ScriptFlag &,
  size_t current_alignment);
size_t
max_serialized_size_key_ScriptFlag(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);
}  // namespace typesupport_fastrtps_cpp
}  // namespace msg
}  // namespace husarion_ugv_msgs

namespace husarion_ugv_msgs
{
namespace msg
{
namespace typesupport_fastrtps_cpp
{
bool cdr_serialize(
  const husarion_ugv_msgs::msg::RuntimeError &,
  eprosima::fastcdr::Cdr &);
bool cdr_deserialize(
  eprosima::fastcdr::Cdr &,
  husarion_ugv_msgs::msg::RuntimeError &);
size_t get_serialized_size(
  const husarion_ugv_msgs::msg::RuntimeError &,
  size_t current_alignment);
size_t
max_serialized_size_RuntimeError(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);
bool cdr_serialize_key(
  const husarion_ugv_msgs::msg::RuntimeError &,
  eprosima::fastcdr::Cdr &);
size_t get_serialized_size_key(
  const husarion_ugv_msgs::msg::RuntimeError &,
  size_t current_alignment);
size_t
max_serialized_size_key_RuntimeError(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);
}  // namespace typesupport_fastrtps_cpp
}  // namespace msg
}  // namespace husarion_ugv_msgs

// functions for husarion_ugv_msgs::msg::RuntimeError already declared above


namespace husarion_ugv_msgs
{

namespace msg
{

namespace typesupport_fastrtps_cpp
{


bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_husarion_ugv_msgs
cdr_serialize(
  const husarion_ugv_msgs::msg::DriverState & ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Member: voltage
  cdr << ros_message.voltage;

  // Member: current
  cdr << ros_message.current;

  // Member: temperature
  cdr << ros_message.temperature;

  // Member: heatsink_temperature
  cdr << ros_message.heatsink_temperature;

  // Member: fault_flag
  husarion_ugv_msgs::msg::typesupport_fastrtps_cpp::cdr_serialize(
    ros_message.fault_flag,
    cdr);

  // Member: script_flag
  husarion_ugv_msgs::msg::typesupport_fastrtps_cpp::cdr_serialize(
    ros_message.script_flag,
    cdr);

  // Member: channel_1_motor_runtime_error
  husarion_ugv_msgs::msg::typesupport_fastrtps_cpp::cdr_serialize(
    ros_message.channel_1_motor_runtime_error,
    cdr);

  // Member: channel_2_motor_runtime_error
  husarion_ugv_msgs::msg::typesupport_fastrtps_cpp::cdr_serialize(
    ros_message.channel_2_motor_runtime_error,
    cdr);

  // Member: motor_states_data_timed_out
  cdr << (ros_message.motor_states_data_timed_out ? true : false);

  // Member: driver_state_data_timed_out
  cdr << (ros_message.driver_state_data_timed_out ? true : false);

  // Member: can_error
  cdr << (ros_message.can_error ? true : false);

  // Member: heartbeat_timeout
  cdr << (ros_message.heartbeat_timeout ? true : false);

  return true;
}

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_husarion_ugv_msgs
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  husarion_ugv_msgs::msg::DriverState & ros_message)
{
  // Member: voltage
  cdr >> ros_message.voltage;

  // Member: current
  cdr >> ros_message.current;

  // Member: temperature
  cdr >> ros_message.temperature;

  // Member: heatsink_temperature
  cdr >> ros_message.heatsink_temperature;

  // Member: fault_flag
  husarion_ugv_msgs::msg::typesupport_fastrtps_cpp::cdr_deserialize(
    cdr, ros_message.fault_flag);

  // Member: script_flag
  husarion_ugv_msgs::msg::typesupport_fastrtps_cpp::cdr_deserialize(
    cdr, ros_message.script_flag);

  // Member: channel_1_motor_runtime_error
  husarion_ugv_msgs::msg::typesupport_fastrtps_cpp::cdr_deserialize(
    cdr, ros_message.channel_1_motor_runtime_error);

  // Member: channel_2_motor_runtime_error
  husarion_ugv_msgs::msg::typesupport_fastrtps_cpp::cdr_deserialize(
    cdr, ros_message.channel_2_motor_runtime_error);

  // Member: motor_states_data_timed_out
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message.motor_states_data_timed_out = tmp ? true : false;
  }

  // Member: driver_state_data_timed_out
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message.driver_state_data_timed_out = tmp ? true : false;
  }

  // Member: can_error
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message.can_error = tmp ? true : false;
  }

  // Member: heartbeat_timeout
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message.heartbeat_timeout = tmp ? true : false;
  }

  return true;
}  // NOLINT(readability/fn_size)


size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_husarion_ugv_msgs
get_serialized_size(
  const husarion_ugv_msgs::msg::DriverState & ros_message,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Member: voltage
  {
    size_t item_size = sizeof(ros_message.voltage);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Member: current
  {
    size_t item_size = sizeof(ros_message.current);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Member: temperature
  {
    size_t item_size = sizeof(ros_message.temperature);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Member: heatsink_temperature
  {
    size_t item_size = sizeof(ros_message.heatsink_temperature);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Member: fault_flag
  current_alignment +=
    husarion_ugv_msgs::msg::typesupport_fastrtps_cpp::get_serialized_size(
    ros_message.fault_flag, current_alignment);

  // Member: script_flag
  current_alignment +=
    husarion_ugv_msgs::msg::typesupport_fastrtps_cpp::get_serialized_size(
    ros_message.script_flag, current_alignment);

  // Member: channel_1_motor_runtime_error
  current_alignment +=
    husarion_ugv_msgs::msg::typesupport_fastrtps_cpp::get_serialized_size(
    ros_message.channel_1_motor_runtime_error, current_alignment);

  // Member: channel_2_motor_runtime_error
  current_alignment +=
    husarion_ugv_msgs::msg::typesupport_fastrtps_cpp::get_serialized_size(
    ros_message.channel_2_motor_runtime_error, current_alignment);

  // Member: motor_states_data_timed_out
  {
    size_t item_size = sizeof(ros_message.motor_states_data_timed_out);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Member: driver_state_data_timed_out
  {
    size_t item_size = sizeof(ros_message.driver_state_data_timed_out);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Member: can_error
  {
    size_t item_size = sizeof(ros_message.can_error);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Member: heartbeat_timeout
  {
    size_t item_size = sizeof(ros_message.heartbeat_timeout);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}


size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_husarion_ugv_msgs
max_serialized_size_DriverState(
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

  // Member: voltage
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // Member: current
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // Member: temperature
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // Member: heatsink_temperature
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // Member: fault_flag
  {
    size_t array_size = 1;
    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size =
        husarion_ugv_msgs::msg::typesupport_fastrtps_cpp::max_serialized_size_FaultFlag(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }
  // Member: script_flag
  {
    size_t array_size = 1;
    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size =
        husarion_ugv_msgs::msg::typesupport_fastrtps_cpp::max_serialized_size_ScriptFlag(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }
  // Member: channel_1_motor_runtime_error
  {
    size_t array_size = 1;
    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size =
        husarion_ugv_msgs::msg::typesupport_fastrtps_cpp::max_serialized_size_RuntimeError(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }
  // Member: channel_2_motor_runtime_error
  {
    size_t array_size = 1;
    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size =
        husarion_ugv_msgs::msg::typesupport_fastrtps_cpp::max_serialized_size_RuntimeError(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }
  // Member: motor_states_data_timed_out
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // Member: driver_state_data_timed_out
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // Member: can_error
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // Member: heartbeat_timeout
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
    using DataType = husarion_ugv_msgs::msg::DriverState;
    is_plain =
      (
      offsetof(DataType, heartbeat_timeout) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_husarion_ugv_msgs
cdr_serialize_key(
  const husarion_ugv_msgs::msg::DriverState & ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Member: voltage
  cdr << ros_message.voltage;

  // Member: current
  cdr << ros_message.current;

  // Member: temperature
  cdr << ros_message.temperature;

  // Member: heatsink_temperature
  cdr << ros_message.heatsink_temperature;

  // Member: fault_flag
  husarion_ugv_msgs::msg::typesupport_fastrtps_cpp::cdr_serialize_key(
    ros_message.fault_flag,
    cdr);

  // Member: script_flag
  husarion_ugv_msgs::msg::typesupport_fastrtps_cpp::cdr_serialize_key(
    ros_message.script_flag,
    cdr);

  // Member: channel_1_motor_runtime_error
  husarion_ugv_msgs::msg::typesupport_fastrtps_cpp::cdr_serialize_key(
    ros_message.channel_1_motor_runtime_error,
    cdr);

  // Member: channel_2_motor_runtime_error
  husarion_ugv_msgs::msg::typesupport_fastrtps_cpp::cdr_serialize_key(
    ros_message.channel_2_motor_runtime_error,
    cdr);

  // Member: motor_states_data_timed_out
  cdr << (ros_message.motor_states_data_timed_out ? true : false);

  // Member: driver_state_data_timed_out
  cdr << (ros_message.driver_state_data_timed_out ? true : false);

  // Member: can_error
  cdr << (ros_message.can_error ? true : false);

  // Member: heartbeat_timeout
  cdr << (ros_message.heartbeat_timeout ? true : false);

  return true;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_husarion_ugv_msgs
get_serialized_size_key(
  const husarion_ugv_msgs::msg::DriverState & ros_message,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Member: voltage
  {
    size_t item_size = sizeof(ros_message.voltage);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Member: current
  {
    size_t item_size = sizeof(ros_message.current);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Member: temperature
  {
    size_t item_size = sizeof(ros_message.temperature);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Member: heatsink_temperature
  {
    size_t item_size = sizeof(ros_message.heatsink_temperature);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Member: fault_flag
  current_alignment +=
    husarion_ugv_msgs::msg::typesupport_fastrtps_cpp::get_serialized_size_key(
    ros_message.fault_flag, current_alignment);

  // Member: script_flag
  current_alignment +=
    husarion_ugv_msgs::msg::typesupport_fastrtps_cpp::get_serialized_size_key(
    ros_message.script_flag, current_alignment);

  // Member: channel_1_motor_runtime_error
  current_alignment +=
    husarion_ugv_msgs::msg::typesupport_fastrtps_cpp::get_serialized_size_key(
    ros_message.channel_1_motor_runtime_error, current_alignment);

  // Member: channel_2_motor_runtime_error
  current_alignment +=
    husarion_ugv_msgs::msg::typesupport_fastrtps_cpp::get_serialized_size_key(
    ros_message.channel_2_motor_runtime_error, current_alignment);

  // Member: motor_states_data_timed_out
  {
    size_t item_size = sizeof(ros_message.motor_states_data_timed_out);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Member: driver_state_data_timed_out
  {
    size_t item_size = sizeof(ros_message.driver_state_data_timed_out);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Member: can_error
  {
    size_t item_size = sizeof(ros_message.can_error);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Member: heartbeat_timeout
  {
    size_t item_size = sizeof(ros_message.heartbeat_timeout);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_husarion_ugv_msgs
max_serialized_size_key_DriverState(
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

  // Member: voltage
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: current
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: temperature
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: heatsink_temperature
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: fault_flag
  {
    size_t array_size = 1;
    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size =
        husarion_ugv_msgs::msg::typesupport_fastrtps_cpp::max_serialized_size_key_FaultFlag(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }

  // Member: script_flag
  {
    size_t array_size = 1;
    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size =
        husarion_ugv_msgs::msg::typesupport_fastrtps_cpp::max_serialized_size_key_ScriptFlag(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }

  // Member: channel_1_motor_runtime_error
  {
    size_t array_size = 1;
    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size =
        husarion_ugv_msgs::msg::typesupport_fastrtps_cpp::max_serialized_size_key_RuntimeError(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }

  // Member: channel_2_motor_runtime_error
  {
    size_t array_size = 1;
    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size =
        husarion_ugv_msgs::msg::typesupport_fastrtps_cpp::max_serialized_size_key_RuntimeError(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }

  // Member: motor_states_data_timed_out
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: driver_state_data_timed_out
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: can_error
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: heartbeat_timeout
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
    using DataType = husarion_ugv_msgs::msg::DriverState;
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
  auto typed_message =
    static_cast<const husarion_ugv_msgs::msg::DriverState *>(
    untyped_ros_message);
  return cdr_serialize(*typed_message, cdr);
}

static bool _DriverState__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  auto typed_message =
    static_cast<husarion_ugv_msgs::msg::DriverState *>(
    untyped_ros_message);
  return cdr_deserialize(cdr, *typed_message);
}

static uint32_t _DriverState__get_serialized_size(
  const void * untyped_ros_message)
{
  auto typed_message =
    static_cast<const husarion_ugv_msgs::msg::DriverState *>(
    untyped_ros_message);
  return static_cast<uint32_t>(get_serialized_size(*typed_message, 0));
}

static size_t _DriverState__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_DriverState(full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}

static message_type_support_callbacks_t _DriverState__callbacks = {
  "husarion_ugv_msgs::msg",
  "DriverState",
  _DriverState__cdr_serialize,
  _DriverState__cdr_deserialize,
  _DriverState__get_serialized_size,
  _DriverState__max_serialized_size,
  nullptr
};

static rosidl_message_type_support_t _DriverState__handle = {
  rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
  &_DriverState__callbacks,
  get_message_typesupport_handle_function,
  &husarion_ugv_msgs__msg__DriverState__get_type_hash,
  &husarion_ugv_msgs__msg__DriverState__get_type_description,
  &husarion_ugv_msgs__msg__DriverState__get_type_description_sources,
};

}  // namespace typesupport_fastrtps_cpp

}  // namespace msg

}  // namespace husarion_ugv_msgs

namespace rosidl_typesupport_fastrtps_cpp
{

template<>
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_EXPORT_husarion_ugv_msgs
const rosidl_message_type_support_t *
get_message_type_support_handle<husarion_ugv_msgs::msg::DriverState>()
{
  return &husarion_ugv_msgs::msg::typesupport_fastrtps_cpp::_DriverState__handle;
}

}  // namespace rosidl_typesupport_fastrtps_cpp

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, husarion_ugv_msgs, msg, DriverState)() {
  return &husarion_ugv_msgs::msg::typesupport_fastrtps_cpp::_DriverState__handle;
}

#ifdef __cplusplus
}
#endif
