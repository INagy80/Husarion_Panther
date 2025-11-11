// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__type_support.cpp.em
// with input from husarion_ugv_msgs:msg/RuntimeError.idl
// generated code does not contain a copyright notice
#include "husarion_ugv_msgs/msg/detail/runtime_error__rosidl_typesupport_fastrtps_cpp.hpp"
#include "husarion_ugv_msgs/msg/detail/runtime_error__functions.h"
#include "husarion_ugv_msgs/msg/detail/runtime_error__struct.hpp"

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
  const husarion_ugv_msgs::msg::RuntimeError & ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Member: amps_limit_active
  cdr << (ros_message.amps_limit_active ? true : false);

  // Member: motor_stall
  cdr << (ros_message.motor_stall ? true : false);

  // Member: loop_error
  cdr << (ros_message.loop_error ? true : false);

  // Member: safety_stop_active
  cdr << (ros_message.safety_stop_active ? true : false);

  // Member: forward_limit_triggered
  cdr << (ros_message.forward_limit_triggered ? true : false);

  // Member: reverse_limit_triggered
  cdr << (ros_message.reverse_limit_triggered ? true : false);

  // Member: amps_trigger_activated
  cdr << (ros_message.amps_trigger_activated ? true : false);

  return true;
}

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_husarion_ugv_msgs
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  husarion_ugv_msgs::msg::RuntimeError & ros_message)
{
  // Member: amps_limit_active
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message.amps_limit_active = tmp ? true : false;
  }

  // Member: motor_stall
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message.motor_stall = tmp ? true : false;
  }

  // Member: loop_error
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message.loop_error = tmp ? true : false;
  }

  // Member: safety_stop_active
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message.safety_stop_active = tmp ? true : false;
  }

  // Member: forward_limit_triggered
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message.forward_limit_triggered = tmp ? true : false;
  }

  // Member: reverse_limit_triggered
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message.reverse_limit_triggered = tmp ? true : false;
  }

  // Member: amps_trigger_activated
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message.amps_trigger_activated = tmp ? true : false;
  }

  return true;
}  // NOLINT(readability/fn_size)


size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_husarion_ugv_msgs
get_serialized_size(
  const husarion_ugv_msgs::msg::RuntimeError & ros_message,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Member: amps_limit_active
  {
    size_t item_size = sizeof(ros_message.amps_limit_active);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Member: motor_stall
  {
    size_t item_size = sizeof(ros_message.motor_stall);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Member: loop_error
  {
    size_t item_size = sizeof(ros_message.loop_error);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Member: safety_stop_active
  {
    size_t item_size = sizeof(ros_message.safety_stop_active);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Member: forward_limit_triggered
  {
    size_t item_size = sizeof(ros_message.forward_limit_triggered);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Member: reverse_limit_triggered
  {
    size_t item_size = sizeof(ros_message.reverse_limit_triggered);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Member: amps_trigger_activated
  {
    size_t item_size = sizeof(ros_message.amps_trigger_activated);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}


size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_husarion_ugv_msgs
max_serialized_size_RuntimeError(
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

  // Member: amps_limit_active
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // Member: motor_stall
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // Member: loop_error
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // Member: safety_stop_active
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // Member: forward_limit_triggered
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // Member: reverse_limit_triggered
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // Member: amps_trigger_activated
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
    using DataType = husarion_ugv_msgs::msg::RuntimeError;
    is_plain =
      (
      offsetof(DataType, amps_trigger_activated) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_husarion_ugv_msgs
cdr_serialize_key(
  const husarion_ugv_msgs::msg::RuntimeError & ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Member: amps_limit_active
  cdr << (ros_message.amps_limit_active ? true : false);

  // Member: motor_stall
  cdr << (ros_message.motor_stall ? true : false);

  // Member: loop_error
  cdr << (ros_message.loop_error ? true : false);

  // Member: safety_stop_active
  cdr << (ros_message.safety_stop_active ? true : false);

  // Member: forward_limit_triggered
  cdr << (ros_message.forward_limit_triggered ? true : false);

  // Member: reverse_limit_triggered
  cdr << (ros_message.reverse_limit_triggered ? true : false);

  // Member: amps_trigger_activated
  cdr << (ros_message.amps_trigger_activated ? true : false);

  return true;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_husarion_ugv_msgs
get_serialized_size_key(
  const husarion_ugv_msgs::msg::RuntimeError & ros_message,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Member: amps_limit_active
  {
    size_t item_size = sizeof(ros_message.amps_limit_active);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Member: motor_stall
  {
    size_t item_size = sizeof(ros_message.motor_stall);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Member: loop_error
  {
    size_t item_size = sizeof(ros_message.loop_error);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Member: safety_stop_active
  {
    size_t item_size = sizeof(ros_message.safety_stop_active);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Member: forward_limit_triggered
  {
    size_t item_size = sizeof(ros_message.forward_limit_triggered);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Member: reverse_limit_triggered
  {
    size_t item_size = sizeof(ros_message.reverse_limit_triggered);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Member: amps_trigger_activated
  {
    size_t item_size = sizeof(ros_message.amps_trigger_activated);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_husarion_ugv_msgs
max_serialized_size_key_RuntimeError(
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

  // Member: amps_limit_active
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: motor_stall
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: loop_error
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: safety_stop_active
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: forward_limit_triggered
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: reverse_limit_triggered
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: amps_trigger_activated
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
    using DataType = husarion_ugv_msgs::msg::RuntimeError;
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
  auto typed_message =
    static_cast<const husarion_ugv_msgs::msg::RuntimeError *>(
    untyped_ros_message);
  return cdr_serialize(*typed_message, cdr);
}

static bool _RuntimeError__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  auto typed_message =
    static_cast<husarion_ugv_msgs::msg::RuntimeError *>(
    untyped_ros_message);
  return cdr_deserialize(cdr, *typed_message);
}

static uint32_t _RuntimeError__get_serialized_size(
  const void * untyped_ros_message)
{
  auto typed_message =
    static_cast<const husarion_ugv_msgs::msg::RuntimeError *>(
    untyped_ros_message);
  return static_cast<uint32_t>(get_serialized_size(*typed_message, 0));
}

static size_t _RuntimeError__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_RuntimeError(full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}

static message_type_support_callbacks_t _RuntimeError__callbacks = {
  "husarion_ugv_msgs::msg",
  "RuntimeError",
  _RuntimeError__cdr_serialize,
  _RuntimeError__cdr_deserialize,
  _RuntimeError__get_serialized_size,
  _RuntimeError__max_serialized_size,
  nullptr
};

static rosidl_message_type_support_t _RuntimeError__handle = {
  rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
  &_RuntimeError__callbacks,
  get_message_typesupport_handle_function,
  &husarion_ugv_msgs__msg__RuntimeError__get_type_hash,
  &husarion_ugv_msgs__msg__RuntimeError__get_type_description,
  &husarion_ugv_msgs__msg__RuntimeError__get_type_description_sources,
};

}  // namespace typesupport_fastrtps_cpp

}  // namespace msg

}  // namespace husarion_ugv_msgs

namespace rosidl_typesupport_fastrtps_cpp
{

template<>
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_EXPORT_husarion_ugv_msgs
const rosidl_message_type_support_t *
get_message_type_support_handle<husarion_ugv_msgs::msg::RuntimeError>()
{
  return &husarion_ugv_msgs::msg::typesupport_fastrtps_cpp::_RuntimeError__handle;
}

}  // namespace rosidl_typesupport_fastrtps_cpp

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, husarion_ugv_msgs, msg, RuntimeError)() {
  return &husarion_ugv_msgs::msg::typesupport_fastrtps_cpp::_RuntimeError__handle;
}

#ifdef __cplusplus
}
#endif
