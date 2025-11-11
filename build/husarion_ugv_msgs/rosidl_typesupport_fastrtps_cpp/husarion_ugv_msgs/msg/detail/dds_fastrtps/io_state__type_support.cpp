// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__type_support.cpp.em
// with input from husarion_ugv_msgs:msg/IOState.idl
// generated code does not contain a copyright notice
#include "husarion_ugv_msgs/msg/detail/io_state__rosidl_typesupport_fastrtps_cpp.hpp"
#include "husarion_ugv_msgs/msg/detail/io_state__functions.h"
#include "husarion_ugv_msgs/msg/detail/io_state__struct.hpp"

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


bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_husarion_ugv_msgs
cdr_serialize(
  const husarion_ugv_msgs::msg::IOState & ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Member: aux_power
  cdr << (ros_message.aux_power ? true : false);

  // Member: charger_connected
  cdr << (ros_message.charger_connected ? true : false);

  // Member: charger_enabled
  cdr << (ros_message.charger_enabled ? true : false);

  // Member: digital_power
  cdr << (ros_message.digital_power ? true : false);

  // Member: fan
  cdr << (ros_message.fan ? true : false);

  // Member: led_control
  cdr << (ros_message.led_control ? true : false);

  // Member: motor_on
  cdr << (ros_message.motor_on ? true : false);

  // Member: power_button
  cdr << (ros_message.power_button ? true : false);

  return true;
}

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_husarion_ugv_msgs
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  husarion_ugv_msgs::msg::IOState & ros_message)
{
  // Member: aux_power
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message.aux_power = tmp ? true : false;
  }

  // Member: charger_connected
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message.charger_connected = tmp ? true : false;
  }

  // Member: charger_enabled
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message.charger_enabled = tmp ? true : false;
  }

  // Member: digital_power
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message.digital_power = tmp ? true : false;
  }

  // Member: fan
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message.fan = tmp ? true : false;
  }

  // Member: led_control
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message.led_control = tmp ? true : false;
  }

  // Member: motor_on
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message.motor_on = tmp ? true : false;
  }

  // Member: power_button
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message.power_button = tmp ? true : false;
  }

  return true;
}  // NOLINT(readability/fn_size)


size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_husarion_ugv_msgs
get_serialized_size(
  const husarion_ugv_msgs::msg::IOState & ros_message,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Member: aux_power
  {
    size_t item_size = sizeof(ros_message.aux_power);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Member: charger_connected
  {
    size_t item_size = sizeof(ros_message.charger_connected);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Member: charger_enabled
  {
    size_t item_size = sizeof(ros_message.charger_enabled);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Member: digital_power
  {
    size_t item_size = sizeof(ros_message.digital_power);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Member: fan
  {
    size_t item_size = sizeof(ros_message.fan);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Member: led_control
  {
    size_t item_size = sizeof(ros_message.led_control);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Member: motor_on
  {
    size_t item_size = sizeof(ros_message.motor_on);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Member: power_button
  {
    size_t item_size = sizeof(ros_message.power_button);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}


size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_husarion_ugv_msgs
max_serialized_size_IOState(
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

  // Member: aux_power
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // Member: charger_connected
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // Member: charger_enabled
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // Member: digital_power
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // Member: fan
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // Member: led_control
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // Member: motor_on
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // Member: power_button
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
    using DataType = husarion_ugv_msgs::msg::IOState;
    is_plain =
      (
      offsetof(DataType, power_button) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_husarion_ugv_msgs
cdr_serialize_key(
  const husarion_ugv_msgs::msg::IOState & ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Member: aux_power
  cdr << (ros_message.aux_power ? true : false);

  // Member: charger_connected
  cdr << (ros_message.charger_connected ? true : false);

  // Member: charger_enabled
  cdr << (ros_message.charger_enabled ? true : false);

  // Member: digital_power
  cdr << (ros_message.digital_power ? true : false);

  // Member: fan
  cdr << (ros_message.fan ? true : false);

  // Member: led_control
  cdr << (ros_message.led_control ? true : false);

  // Member: motor_on
  cdr << (ros_message.motor_on ? true : false);

  // Member: power_button
  cdr << (ros_message.power_button ? true : false);

  return true;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_husarion_ugv_msgs
get_serialized_size_key(
  const husarion_ugv_msgs::msg::IOState & ros_message,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Member: aux_power
  {
    size_t item_size = sizeof(ros_message.aux_power);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Member: charger_connected
  {
    size_t item_size = sizeof(ros_message.charger_connected);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Member: charger_enabled
  {
    size_t item_size = sizeof(ros_message.charger_enabled);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Member: digital_power
  {
    size_t item_size = sizeof(ros_message.digital_power);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Member: fan
  {
    size_t item_size = sizeof(ros_message.fan);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Member: led_control
  {
    size_t item_size = sizeof(ros_message.led_control);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Member: motor_on
  {
    size_t item_size = sizeof(ros_message.motor_on);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Member: power_button
  {
    size_t item_size = sizeof(ros_message.power_button);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_husarion_ugv_msgs
max_serialized_size_key_IOState(
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

  // Member: aux_power
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: charger_connected
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: charger_enabled
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: digital_power
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: fan
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: led_control
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: motor_on
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: power_button
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
    using DataType = husarion_ugv_msgs::msg::IOState;
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
  auto typed_message =
    static_cast<const husarion_ugv_msgs::msg::IOState *>(
    untyped_ros_message);
  return cdr_serialize(*typed_message, cdr);
}

static bool _IOState__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  auto typed_message =
    static_cast<husarion_ugv_msgs::msg::IOState *>(
    untyped_ros_message);
  return cdr_deserialize(cdr, *typed_message);
}

static uint32_t _IOState__get_serialized_size(
  const void * untyped_ros_message)
{
  auto typed_message =
    static_cast<const husarion_ugv_msgs::msg::IOState *>(
    untyped_ros_message);
  return static_cast<uint32_t>(get_serialized_size(*typed_message, 0));
}

static size_t _IOState__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_IOState(full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}

static message_type_support_callbacks_t _IOState__callbacks = {
  "husarion_ugv_msgs::msg",
  "IOState",
  _IOState__cdr_serialize,
  _IOState__cdr_deserialize,
  _IOState__get_serialized_size,
  _IOState__max_serialized_size,
  nullptr
};

static rosidl_message_type_support_t _IOState__handle = {
  rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
  &_IOState__callbacks,
  get_message_typesupport_handle_function,
  &husarion_ugv_msgs__msg__IOState__get_type_hash,
  &husarion_ugv_msgs__msg__IOState__get_type_description,
  &husarion_ugv_msgs__msg__IOState__get_type_description_sources,
};

}  // namespace typesupport_fastrtps_cpp

}  // namespace msg

}  // namespace husarion_ugv_msgs

namespace rosidl_typesupport_fastrtps_cpp
{

template<>
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_EXPORT_husarion_ugv_msgs
const rosidl_message_type_support_t *
get_message_type_support_handle<husarion_ugv_msgs::msg::IOState>()
{
  return &husarion_ugv_msgs::msg::typesupport_fastrtps_cpp::_IOState__handle;
}

}  // namespace rosidl_typesupport_fastrtps_cpp

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, husarion_ugv_msgs, msg, IOState)() {
  return &husarion_ugv_msgs::msg::typesupport_fastrtps_cpp::_IOState__handle;
}

#ifdef __cplusplus
}
#endif
