// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__type_support.cpp.em
// with input from husarion_ugv_msgs:msg/RobotDriverState.idl
// generated code does not contain a copyright notice
#include "husarion_ugv_msgs/msg/detail/robot_driver_state__rosidl_typesupport_fastrtps_cpp.hpp"
#include "husarion_ugv_msgs/msg/detail/robot_driver_state__functions.h"
#include "husarion_ugv_msgs/msg/detail/robot_driver_state__struct.hpp"

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
namespace std_msgs
{
namespace msg
{
namespace typesupport_fastrtps_cpp
{
bool cdr_serialize(
  const std_msgs::msg::Header &,
  eprosima::fastcdr::Cdr &);
bool cdr_deserialize(
  eprosima::fastcdr::Cdr &,
  std_msgs::msg::Header &);
size_t get_serialized_size(
  const std_msgs::msg::Header &,
  size_t current_alignment);
size_t
max_serialized_size_Header(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);
bool cdr_serialize_key(
  const std_msgs::msg::Header &,
  eprosima::fastcdr::Cdr &);
size_t get_serialized_size_key(
  const std_msgs::msg::Header &,
  size_t current_alignment);
size_t
max_serialized_size_key_Header(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);
}  // namespace typesupport_fastrtps_cpp
}  // namespace msg
}  // namespace std_msgs

namespace husarion_ugv_msgs
{
namespace msg
{
namespace typesupport_fastrtps_cpp
{
bool cdr_serialize(
  const husarion_ugv_msgs::msg::DriverStateNamed &,
  eprosima::fastcdr::Cdr &);
bool cdr_deserialize(
  eprosima::fastcdr::Cdr &,
  husarion_ugv_msgs::msg::DriverStateNamed &);
size_t get_serialized_size(
  const husarion_ugv_msgs::msg::DriverStateNamed &,
  size_t current_alignment);
size_t
max_serialized_size_DriverStateNamed(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);
bool cdr_serialize_key(
  const husarion_ugv_msgs::msg::DriverStateNamed &,
  eprosima::fastcdr::Cdr &);
size_t get_serialized_size_key(
  const husarion_ugv_msgs::msg::DriverStateNamed &,
  size_t current_alignment);
size_t
max_serialized_size_key_DriverStateNamed(
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


bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_husarion_ugv_msgs
cdr_serialize(
  const husarion_ugv_msgs::msg::RobotDriverState & ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Member: header
  std_msgs::msg::typesupport_fastrtps_cpp::cdr_serialize(
    ros_message.header,
    cdr);

  // Member: driver_states
  {
    size_t size = ros_message.driver_states.size();
    cdr << static_cast<uint32_t>(size);
    for (size_t i = 0; i < size; i++) {
      husarion_ugv_msgs::msg::typesupport_fastrtps_cpp::cdr_serialize(
        ros_message.driver_states[i],
        cdr);
    }
  }

  // Member: error
  cdr << (ros_message.error ? true : false);

  // Member: write_pdo_cmds_error
  cdr << (ros_message.write_pdo_cmds_error ? true : false);

  // Member: read_pdo_motor_states_error
  cdr << (ros_message.read_pdo_motor_states_error ? true : false);

  // Member: read_pdo_driver_state_error
  cdr << (ros_message.read_pdo_driver_state_error ? true : false);

  return true;
}

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_husarion_ugv_msgs
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  husarion_ugv_msgs::msg::RobotDriverState & ros_message)
{
  // Member: header
  std_msgs::msg::typesupport_fastrtps_cpp::cdr_deserialize(
    cdr, ros_message.header);

  // Member: driver_states
  {
    uint32_t cdrSize;
    cdr >> cdrSize;
    size_t size = static_cast<size_t>(cdrSize);

    // Check there are at least 'size' remaining bytes in the CDR stream before resizing
    auto old_state = cdr.get_state();
    bool correct_size = cdr.jump(size);
    cdr.set_state(old_state);
    if (!correct_size) {
      fprintf(stderr, "sequence size exceeds remaining buffer\n");
      return false;
    }

    ros_message.driver_states.resize(size);
    for (size_t i = 0; i < size; i++) {
      husarion_ugv_msgs::msg::typesupport_fastrtps_cpp::cdr_deserialize(
        cdr, ros_message.driver_states[i]);
    }
  }

  // Member: error
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message.error = tmp ? true : false;
  }

  // Member: write_pdo_cmds_error
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message.write_pdo_cmds_error = tmp ? true : false;
  }

  // Member: read_pdo_motor_states_error
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message.read_pdo_motor_states_error = tmp ? true : false;
  }

  // Member: read_pdo_driver_state_error
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message.read_pdo_driver_state_error = tmp ? true : false;
  }

  return true;
}  // NOLINT(readability/fn_size)


size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_husarion_ugv_msgs
get_serialized_size(
  const husarion_ugv_msgs::msg::RobotDriverState & ros_message,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Member: header
  current_alignment +=
    std_msgs::msg::typesupport_fastrtps_cpp::get_serialized_size(
    ros_message.header, current_alignment);

  // Member: driver_states
  {
    size_t array_size = ros_message.driver_states.size();
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment +=
        husarion_ugv_msgs::msg::typesupport_fastrtps_cpp::get_serialized_size(
        ros_message.driver_states[index], current_alignment);
    }
  }

  // Member: error
  {
    size_t item_size = sizeof(ros_message.error);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Member: write_pdo_cmds_error
  {
    size_t item_size = sizeof(ros_message.write_pdo_cmds_error);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Member: read_pdo_motor_states_error
  {
    size_t item_size = sizeof(ros_message.read_pdo_motor_states_error);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Member: read_pdo_driver_state_error
  {
    size_t item_size = sizeof(ros_message.read_pdo_driver_state_error);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}


size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_husarion_ugv_msgs
max_serialized_size_RobotDriverState(
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

  // Member: header
  {
    size_t array_size = 1;
    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size =
        std_msgs::msg::typesupport_fastrtps_cpp::max_serialized_size_Header(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }
  // Member: driver_states
  {
    size_t array_size = 0;
    full_bounded = false;
    is_plain = false;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size =
        husarion_ugv_msgs::msg::typesupport_fastrtps_cpp::max_serialized_size_DriverStateNamed(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }
  // Member: error
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // Member: write_pdo_cmds_error
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // Member: read_pdo_motor_states_error
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // Member: read_pdo_driver_state_error
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
    using DataType = husarion_ugv_msgs::msg::RobotDriverState;
    is_plain =
      (
      offsetof(DataType, read_pdo_driver_state_error) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_husarion_ugv_msgs
cdr_serialize_key(
  const husarion_ugv_msgs::msg::RobotDriverState & ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Member: header
  std_msgs::msg::typesupport_fastrtps_cpp::cdr_serialize_key(
    ros_message.header,
    cdr);

  // Member: driver_states
  {
    size_t size = ros_message.driver_states.size();
    cdr << static_cast<uint32_t>(size);
    for (size_t i = 0; i < size; i++) {
      husarion_ugv_msgs::msg::typesupport_fastrtps_cpp::cdr_serialize_key(
        ros_message.driver_states[i],
        cdr);
    }
  }

  // Member: error
  cdr << (ros_message.error ? true : false);

  // Member: write_pdo_cmds_error
  cdr << (ros_message.write_pdo_cmds_error ? true : false);

  // Member: read_pdo_motor_states_error
  cdr << (ros_message.read_pdo_motor_states_error ? true : false);

  // Member: read_pdo_driver_state_error
  cdr << (ros_message.read_pdo_driver_state_error ? true : false);

  return true;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_husarion_ugv_msgs
get_serialized_size_key(
  const husarion_ugv_msgs::msg::RobotDriverState & ros_message,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Member: header
  current_alignment +=
    std_msgs::msg::typesupport_fastrtps_cpp::get_serialized_size_key(
    ros_message.header, current_alignment);

  // Member: driver_states
  {
    size_t array_size = ros_message.driver_states.size();
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment +=
        husarion_ugv_msgs::msg::typesupport_fastrtps_cpp::get_serialized_size_key(
        ros_message.driver_states[index], current_alignment);
    }
  }

  // Member: error
  {
    size_t item_size = sizeof(ros_message.error);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Member: write_pdo_cmds_error
  {
    size_t item_size = sizeof(ros_message.write_pdo_cmds_error);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Member: read_pdo_motor_states_error
  {
    size_t item_size = sizeof(ros_message.read_pdo_motor_states_error);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Member: read_pdo_driver_state_error
  {
    size_t item_size = sizeof(ros_message.read_pdo_driver_state_error);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_husarion_ugv_msgs
max_serialized_size_key_RobotDriverState(
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

  // Member: header
  {
    size_t array_size = 1;
    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size =
        std_msgs::msg::typesupport_fastrtps_cpp::max_serialized_size_key_Header(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }

  // Member: driver_states
  {
    size_t array_size = 0;
    full_bounded = false;
    is_plain = false;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size =
        husarion_ugv_msgs::msg::typesupport_fastrtps_cpp::max_serialized_size_key_DriverStateNamed(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }

  // Member: error
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: write_pdo_cmds_error
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: read_pdo_motor_states_error
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: read_pdo_driver_state_error
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
    using DataType = husarion_ugv_msgs::msg::RobotDriverState;
    is_plain =
      (
      offsetof(DataType, read_pdo_driver_state_error) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}


static bool _RobotDriverState__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  auto typed_message =
    static_cast<const husarion_ugv_msgs::msg::RobotDriverState *>(
    untyped_ros_message);
  return cdr_serialize(*typed_message, cdr);
}

static bool _RobotDriverState__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  auto typed_message =
    static_cast<husarion_ugv_msgs::msg::RobotDriverState *>(
    untyped_ros_message);
  return cdr_deserialize(cdr, *typed_message);
}

static uint32_t _RobotDriverState__get_serialized_size(
  const void * untyped_ros_message)
{
  auto typed_message =
    static_cast<const husarion_ugv_msgs::msg::RobotDriverState *>(
    untyped_ros_message);
  return static_cast<uint32_t>(get_serialized_size(*typed_message, 0));
}

static size_t _RobotDriverState__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_RobotDriverState(full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}

static message_type_support_callbacks_t _RobotDriverState__callbacks = {
  "husarion_ugv_msgs::msg",
  "RobotDriverState",
  _RobotDriverState__cdr_serialize,
  _RobotDriverState__cdr_deserialize,
  _RobotDriverState__get_serialized_size,
  _RobotDriverState__max_serialized_size,
  nullptr
};

static rosidl_message_type_support_t _RobotDriverState__handle = {
  rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
  &_RobotDriverState__callbacks,
  get_message_typesupport_handle_function,
  &husarion_ugv_msgs__msg__RobotDriverState__get_type_hash,
  &husarion_ugv_msgs__msg__RobotDriverState__get_type_description,
  &husarion_ugv_msgs__msg__RobotDriverState__get_type_description_sources,
};

}  // namespace typesupport_fastrtps_cpp

}  // namespace msg

}  // namespace husarion_ugv_msgs

namespace rosidl_typesupport_fastrtps_cpp
{

template<>
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_EXPORT_husarion_ugv_msgs
const rosidl_message_type_support_t *
get_message_type_support_handle<husarion_ugv_msgs::msg::RobotDriverState>()
{
  return &husarion_ugv_msgs::msg::typesupport_fastrtps_cpp::_RobotDriverState__handle;
}

}  // namespace rosidl_typesupport_fastrtps_cpp

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, husarion_ugv_msgs, msg, RobotDriverState)() {
  return &husarion_ugv_msgs::msg::typesupport_fastrtps_cpp::_RobotDriverState__handle;
}

#ifdef __cplusplus
}
#endif
