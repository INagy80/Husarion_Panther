// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__type_support.cpp.em
// with input from husarion_ugv_msgs:msg/DriverStateNamed.idl
// generated code does not contain a copyright notice
#include "husarion_ugv_msgs/msg/detail/driver_state_named__rosidl_typesupport_fastrtps_cpp.hpp"
#include "husarion_ugv_msgs/msg/detail/driver_state_named__functions.h"
#include "husarion_ugv_msgs/msg/detail/driver_state_named__struct.hpp"

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
  const husarion_ugv_msgs::msg::DriverState &,
  eprosima::fastcdr::Cdr &);
bool cdr_deserialize(
  eprosima::fastcdr::Cdr &,
  husarion_ugv_msgs::msg::DriverState &);
size_t get_serialized_size(
  const husarion_ugv_msgs::msg::DriverState &,
  size_t current_alignment);
size_t
max_serialized_size_DriverState(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);
bool cdr_serialize_key(
  const husarion_ugv_msgs::msg::DriverState &,
  eprosima::fastcdr::Cdr &);
size_t get_serialized_size_key(
  const husarion_ugv_msgs::msg::DriverState &,
  size_t current_alignment);
size_t
max_serialized_size_key_DriverState(
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
  const husarion_ugv_msgs::msg::DriverStateNamed & ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Member: name
  cdr << ros_message.name;

  // Member: state
  husarion_ugv_msgs::msg::typesupport_fastrtps_cpp::cdr_serialize(
    ros_message.state,
    cdr);

  return true;
}

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_husarion_ugv_msgs
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  husarion_ugv_msgs::msg::DriverStateNamed & ros_message)
{
  // Member: name
  cdr >> ros_message.name;

  // Member: state
  husarion_ugv_msgs::msg::typesupport_fastrtps_cpp::cdr_deserialize(
    cdr, ros_message.state);

  return true;
}  // NOLINT(readability/fn_size)


size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_husarion_ugv_msgs
get_serialized_size(
  const husarion_ugv_msgs::msg::DriverStateNamed & ros_message,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Member: name
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message.name.size() + 1);

  // Member: state
  current_alignment +=
    husarion_ugv_msgs::msg::typesupport_fastrtps_cpp::get_serialized_size(
    ros_message.state, current_alignment);

  return current_alignment - initial_alignment;
}


size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_husarion_ugv_msgs
max_serialized_size_DriverStateNamed(
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

  // Member: name
  {
    size_t array_size = 1;
    full_bounded = false;
    is_plain = false;
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += padding +
        eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
        1;
    }
  }
  // Member: state
  {
    size_t array_size = 1;
    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size =
        husarion_ugv_msgs::msg::typesupport_fastrtps_cpp::max_serialized_size_DriverState(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }

  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = husarion_ugv_msgs::msg::DriverStateNamed;
    is_plain =
      (
      offsetof(DataType, state) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_husarion_ugv_msgs
cdr_serialize_key(
  const husarion_ugv_msgs::msg::DriverStateNamed & ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Member: name
  cdr << ros_message.name;

  // Member: state
  husarion_ugv_msgs::msg::typesupport_fastrtps_cpp::cdr_serialize_key(
    ros_message.state,
    cdr);

  return true;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_husarion_ugv_msgs
get_serialized_size_key(
  const husarion_ugv_msgs::msg::DriverStateNamed & ros_message,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Member: name
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message.name.size() + 1);

  // Member: state
  current_alignment +=
    husarion_ugv_msgs::msg::typesupport_fastrtps_cpp::get_serialized_size_key(
    ros_message.state, current_alignment);

  return current_alignment - initial_alignment;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_husarion_ugv_msgs
max_serialized_size_key_DriverStateNamed(
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

  // Member: name
  {
    size_t array_size = 1;
    full_bounded = false;
    is_plain = false;
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += padding +
        eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
        1;
    }
  }

  // Member: state
  {
    size_t array_size = 1;
    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size =
        husarion_ugv_msgs::msg::typesupport_fastrtps_cpp::max_serialized_size_key_DriverState(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }

  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = husarion_ugv_msgs::msg::DriverStateNamed;
    is_plain =
      (
      offsetof(DataType, state) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}


static bool _DriverStateNamed__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  auto typed_message =
    static_cast<const husarion_ugv_msgs::msg::DriverStateNamed *>(
    untyped_ros_message);
  return cdr_serialize(*typed_message, cdr);
}

static bool _DriverStateNamed__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  auto typed_message =
    static_cast<husarion_ugv_msgs::msg::DriverStateNamed *>(
    untyped_ros_message);
  return cdr_deserialize(cdr, *typed_message);
}

static uint32_t _DriverStateNamed__get_serialized_size(
  const void * untyped_ros_message)
{
  auto typed_message =
    static_cast<const husarion_ugv_msgs::msg::DriverStateNamed *>(
    untyped_ros_message);
  return static_cast<uint32_t>(get_serialized_size(*typed_message, 0));
}

static size_t _DriverStateNamed__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_DriverStateNamed(full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}

static message_type_support_callbacks_t _DriverStateNamed__callbacks = {
  "husarion_ugv_msgs::msg",
  "DriverStateNamed",
  _DriverStateNamed__cdr_serialize,
  _DriverStateNamed__cdr_deserialize,
  _DriverStateNamed__get_serialized_size,
  _DriverStateNamed__max_serialized_size,
  nullptr
};

static rosidl_message_type_support_t _DriverStateNamed__handle = {
  rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
  &_DriverStateNamed__callbacks,
  get_message_typesupport_handle_function,
  &husarion_ugv_msgs__msg__DriverStateNamed__get_type_hash,
  &husarion_ugv_msgs__msg__DriverStateNamed__get_type_description,
  &husarion_ugv_msgs__msg__DriverStateNamed__get_type_description_sources,
};

}  // namespace typesupport_fastrtps_cpp

}  // namespace msg

}  // namespace husarion_ugv_msgs

namespace rosidl_typesupport_fastrtps_cpp
{

template<>
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_EXPORT_husarion_ugv_msgs
const rosidl_message_type_support_t *
get_message_type_support_handle<husarion_ugv_msgs::msg::DriverStateNamed>()
{
  return &husarion_ugv_msgs::msg::typesupport_fastrtps_cpp::_DriverStateNamed__handle;
}

}  // namespace rosidl_typesupport_fastrtps_cpp

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, husarion_ugv_msgs, msg, DriverStateNamed)() {
  return &husarion_ugv_msgs::msg::typesupport_fastrtps_cpp::_DriverStateNamed__handle;
}

#ifdef __cplusplus
}
#endif
