// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__type_support.cpp.em
// with input from husarion_ugv_msgs:msg/FaultFlag.idl
// generated code does not contain a copyright notice
#include "husarion_ugv_msgs/msg/detail/fault_flag__rosidl_typesupport_fastrtps_cpp.hpp"
#include "husarion_ugv_msgs/msg/detail/fault_flag__functions.h"
#include "husarion_ugv_msgs/msg/detail/fault_flag__struct.hpp"

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
  const husarion_ugv_msgs::msg::FaultFlag & ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Member: overheat
  cdr << (ros_message.overheat ? true : false);

  // Member: overvoltage
  cdr << (ros_message.overvoltage ? true : false);

  // Member: undervoltage
  cdr << (ros_message.undervoltage ? true : false);

  // Member: short_circuit
  cdr << (ros_message.short_circuit ? true : false);

  // Member: emergency_stop
  cdr << (ros_message.emergency_stop ? true : false);

  // Member: motor_or_sensor_setup_fault
  cdr << (ros_message.motor_or_sensor_setup_fault ? true : false);

  // Member: mosfet_failure
  cdr << (ros_message.mosfet_failure ? true : false);

  // Member: default_config_loaded_at_startup
  cdr << (ros_message.default_config_loaded_at_startup ? true : false);

  return true;
}

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_husarion_ugv_msgs
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  husarion_ugv_msgs::msg::FaultFlag & ros_message)
{
  // Member: overheat
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message.overheat = tmp ? true : false;
  }

  // Member: overvoltage
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message.overvoltage = tmp ? true : false;
  }

  // Member: undervoltage
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message.undervoltage = tmp ? true : false;
  }

  // Member: short_circuit
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message.short_circuit = tmp ? true : false;
  }

  // Member: emergency_stop
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message.emergency_stop = tmp ? true : false;
  }

  // Member: motor_or_sensor_setup_fault
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message.motor_or_sensor_setup_fault = tmp ? true : false;
  }

  // Member: mosfet_failure
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message.mosfet_failure = tmp ? true : false;
  }

  // Member: default_config_loaded_at_startup
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message.default_config_loaded_at_startup = tmp ? true : false;
  }

  return true;
}  // NOLINT(readability/fn_size)


size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_husarion_ugv_msgs
get_serialized_size(
  const husarion_ugv_msgs::msg::FaultFlag & ros_message,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Member: overheat
  {
    size_t item_size = sizeof(ros_message.overheat);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Member: overvoltage
  {
    size_t item_size = sizeof(ros_message.overvoltage);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Member: undervoltage
  {
    size_t item_size = sizeof(ros_message.undervoltage);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Member: short_circuit
  {
    size_t item_size = sizeof(ros_message.short_circuit);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Member: emergency_stop
  {
    size_t item_size = sizeof(ros_message.emergency_stop);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Member: motor_or_sensor_setup_fault
  {
    size_t item_size = sizeof(ros_message.motor_or_sensor_setup_fault);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Member: mosfet_failure
  {
    size_t item_size = sizeof(ros_message.mosfet_failure);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Member: default_config_loaded_at_startup
  {
    size_t item_size = sizeof(ros_message.default_config_loaded_at_startup);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}


size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_husarion_ugv_msgs
max_serialized_size_FaultFlag(
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

  // Member: overheat
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // Member: overvoltage
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // Member: undervoltage
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // Member: short_circuit
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // Member: emergency_stop
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // Member: motor_or_sensor_setup_fault
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // Member: mosfet_failure
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // Member: default_config_loaded_at_startup
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
    using DataType = husarion_ugv_msgs::msg::FaultFlag;
    is_plain =
      (
      offsetof(DataType, default_config_loaded_at_startup) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_husarion_ugv_msgs
cdr_serialize_key(
  const husarion_ugv_msgs::msg::FaultFlag & ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Member: overheat
  cdr << (ros_message.overheat ? true : false);

  // Member: overvoltage
  cdr << (ros_message.overvoltage ? true : false);

  // Member: undervoltage
  cdr << (ros_message.undervoltage ? true : false);

  // Member: short_circuit
  cdr << (ros_message.short_circuit ? true : false);

  // Member: emergency_stop
  cdr << (ros_message.emergency_stop ? true : false);

  // Member: motor_or_sensor_setup_fault
  cdr << (ros_message.motor_or_sensor_setup_fault ? true : false);

  // Member: mosfet_failure
  cdr << (ros_message.mosfet_failure ? true : false);

  // Member: default_config_loaded_at_startup
  cdr << (ros_message.default_config_loaded_at_startup ? true : false);

  return true;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_husarion_ugv_msgs
get_serialized_size_key(
  const husarion_ugv_msgs::msg::FaultFlag & ros_message,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Member: overheat
  {
    size_t item_size = sizeof(ros_message.overheat);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Member: overvoltage
  {
    size_t item_size = sizeof(ros_message.overvoltage);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Member: undervoltage
  {
    size_t item_size = sizeof(ros_message.undervoltage);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Member: short_circuit
  {
    size_t item_size = sizeof(ros_message.short_circuit);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Member: emergency_stop
  {
    size_t item_size = sizeof(ros_message.emergency_stop);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Member: motor_or_sensor_setup_fault
  {
    size_t item_size = sizeof(ros_message.motor_or_sensor_setup_fault);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Member: mosfet_failure
  {
    size_t item_size = sizeof(ros_message.mosfet_failure);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Member: default_config_loaded_at_startup
  {
    size_t item_size = sizeof(ros_message.default_config_loaded_at_startup);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_husarion_ugv_msgs
max_serialized_size_key_FaultFlag(
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

  // Member: overheat
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: overvoltage
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: undervoltage
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: short_circuit
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: emergency_stop
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: motor_or_sensor_setup_fault
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: mosfet_failure
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: default_config_loaded_at_startup
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
    using DataType = husarion_ugv_msgs::msg::FaultFlag;
    is_plain =
      (
      offsetof(DataType, default_config_loaded_at_startup) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}


static bool _FaultFlag__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  auto typed_message =
    static_cast<const husarion_ugv_msgs::msg::FaultFlag *>(
    untyped_ros_message);
  return cdr_serialize(*typed_message, cdr);
}

static bool _FaultFlag__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  auto typed_message =
    static_cast<husarion_ugv_msgs::msg::FaultFlag *>(
    untyped_ros_message);
  return cdr_deserialize(cdr, *typed_message);
}

static uint32_t _FaultFlag__get_serialized_size(
  const void * untyped_ros_message)
{
  auto typed_message =
    static_cast<const husarion_ugv_msgs::msg::FaultFlag *>(
    untyped_ros_message);
  return static_cast<uint32_t>(get_serialized_size(*typed_message, 0));
}

static size_t _FaultFlag__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_FaultFlag(full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}

static message_type_support_callbacks_t _FaultFlag__callbacks = {
  "husarion_ugv_msgs::msg",
  "FaultFlag",
  _FaultFlag__cdr_serialize,
  _FaultFlag__cdr_deserialize,
  _FaultFlag__get_serialized_size,
  _FaultFlag__max_serialized_size,
  nullptr
};

static rosidl_message_type_support_t _FaultFlag__handle = {
  rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
  &_FaultFlag__callbacks,
  get_message_typesupport_handle_function,
  &husarion_ugv_msgs__msg__FaultFlag__get_type_hash,
  &husarion_ugv_msgs__msg__FaultFlag__get_type_description,
  &husarion_ugv_msgs__msg__FaultFlag__get_type_description_sources,
};

}  // namespace typesupport_fastrtps_cpp

}  // namespace msg

}  // namespace husarion_ugv_msgs

namespace rosidl_typesupport_fastrtps_cpp
{

template<>
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_EXPORT_husarion_ugv_msgs
const rosidl_message_type_support_t *
get_message_type_support_handle<husarion_ugv_msgs::msg::FaultFlag>()
{
  return &husarion_ugv_msgs::msg::typesupport_fastrtps_cpp::_FaultFlag__handle;
}

}  // namespace rosidl_typesupport_fastrtps_cpp

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, husarion_ugv_msgs, msg, FaultFlag)() {
  return &husarion_ugv_msgs::msg::typesupport_fastrtps_cpp::_FaultFlag__handle;
}

#ifdef __cplusplus
}
#endif
