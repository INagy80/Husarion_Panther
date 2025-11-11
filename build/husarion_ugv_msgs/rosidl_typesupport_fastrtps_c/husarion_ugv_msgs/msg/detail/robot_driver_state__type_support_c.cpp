// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from husarion_ugv_msgs:msg/RobotDriverState.idl
// generated code does not contain a copyright notice
#include "husarion_ugv_msgs/msg/detail/robot_driver_state__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <cstddef>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/serialization_helpers.hpp"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "husarion_ugv_msgs/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "husarion_ugv_msgs/msg/detail/robot_driver_state__struct.h"
#include "husarion_ugv_msgs/msg/detail/robot_driver_state__functions.h"
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

#include "husarion_ugv_msgs/msg/detail/driver_state_named__functions.h"  // driver_states
#include "std_msgs/msg/detail/header__functions.h"  // header

// forward declare type support functions

bool cdr_serialize_husarion_ugv_msgs__msg__DriverStateNamed(
  const husarion_ugv_msgs__msg__DriverStateNamed * ros_message,
  eprosima::fastcdr::Cdr & cdr);

bool cdr_deserialize_husarion_ugv_msgs__msg__DriverStateNamed(
  eprosima::fastcdr::Cdr & cdr,
  husarion_ugv_msgs__msg__DriverStateNamed * ros_message);

size_t get_serialized_size_husarion_ugv_msgs__msg__DriverStateNamed(
  const void * untyped_ros_message,
  size_t current_alignment);

size_t max_serialized_size_husarion_ugv_msgs__msg__DriverStateNamed(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

bool cdr_serialize_key_husarion_ugv_msgs__msg__DriverStateNamed(
  const husarion_ugv_msgs__msg__DriverStateNamed * ros_message,
  eprosima::fastcdr::Cdr & cdr);

size_t get_serialized_size_key_husarion_ugv_msgs__msg__DriverStateNamed(
  const void * untyped_ros_message,
  size_t current_alignment);

size_t max_serialized_size_key_husarion_ugv_msgs__msg__DriverStateNamed(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, husarion_ugv_msgs, msg, DriverStateNamed)();

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_husarion_ugv_msgs
bool cdr_serialize_std_msgs__msg__Header(
  const std_msgs__msg__Header * ros_message,
  eprosima::fastcdr::Cdr & cdr);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_husarion_ugv_msgs
bool cdr_deserialize_std_msgs__msg__Header(
  eprosima::fastcdr::Cdr & cdr,
  std_msgs__msg__Header * ros_message);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_husarion_ugv_msgs
size_t get_serialized_size_std_msgs__msg__Header(
  const void * untyped_ros_message,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_husarion_ugv_msgs
size_t max_serialized_size_std_msgs__msg__Header(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_husarion_ugv_msgs
bool cdr_serialize_key_std_msgs__msg__Header(
  const std_msgs__msg__Header * ros_message,
  eprosima::fastcdr::Cdr & cdr);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_husarion_ugv_msgs
size_t get_serialized_size_key_std_msgs__msg__Header(
  const void * untyped_ros_message,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_husarion_ugv_msgs
size_t max_serialized_size_key_std_msgs__msg__Header(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_husarion_ugv_msgs
const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, std_msgs, msg, Header)();


using _RobotDriverState__ros_msg_type = husarion_ugv_msgs__msg__RobotDriverState;


ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_husarion_ugv_msgs
bool cdr_serialize_husarion_ugv_msgs__msg__RobotDriverState(
  const husarion_ugv_msgs__msg__RobotDriverState * ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Field name: header
  {
    cdr_serialize_std_msgs__msg__Header(
      &ros_message->header, cdr);
  }

  // Field name: driver_states
  {
    size_t size = ros_message->driver_states.size;
    auto array_ptr = ros_message->driver_states.data;
    cdr << static_cast<uint32_t>(size);
    for (size_t i = 0; i < size; ++i) {
      cdr_serialize_husarion_ugv_msgs__msg__DriverStateNamed(
        &array_ptr[i], cdr);
    }
  }

  // Field name: error
  {
    cdr << (ros_message->error ? true : false);
  }

  // Field name: write_pdo_cmds_error
  {
    cdr << (ros_message->write_pdo_cmds_error ? true : false);
  }

  // Field name: read_pdo_motor_states_error
  {
    cdr << (ros_message->read_pdo_motor_states_error ? true : false);
  }

  // Field name: read_pdo_driver_state_error
  {
    cdr << (ros_message->read_pdo_driver_state_error ? true : false);
  }

  return true;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_husarion_ugv_msgs
bool cdr_deserialize_husarion_ugv_msgs__msg__RobotDriverState(
  eprosima::fastcdr::Cdr & cdr,
  husarion_ugv_msgs__msg__RobotDriverState * ros_message)
{
  // Field name: header
  {
    cdr_deserialize_std_msgs__msg__Header(cdr, &ros_message->header);
  }

  // Field name: driver_states
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

    if (ros_message->driver_states.data) {
      husarion_ugv_msgs__msg__DriverStateNamed__Sequence__fini(&ros_message->driver_states);
    }
    if (!husarion_ugv_msgs__msg__DriverStateNamed__Sequence__init(&ros_message->driver_states, size)) {
      fprintf(stderr, "failed to create array for field 'driver_states'");
      return false;
    }
    auto array_ptr = ros_message->driver_states.data;
    for (size_t i = 0; i < size; ++i) {
      cdr_deserialize_husarion_ugv_msgs__msg__DriverStateNamed(cdr, &array_ptr[i]);
    }
  }

  // Field name: error
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->error = tmp ? true : false;
  }

  // Field name: write_pdo_cmds_error
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->write_pdo_cmds_error = tmp ? true : false;
  }

  // Field name: read_pdo_motor_states_error
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->read_pdo_motor_states_error = tmp ? true : false;
  }

  // Field name: read_pdo_driver_state_error
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->read_pdo_driver_state_error = tmp ? true : false;
  }

  return true;
}  // NOLINT(readability/fn_size)


ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_husarion_ugv_msgs
size_t get_serialized_size_husarion_ugv_msgs__msg__RobotDriverState(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _RobotDriverState__ros_msg_type * ros_message = static_cast<const _RobotDriverState__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Field name: header
  current_alignment += get_serialized_size_std_msgs__msg__Header(
    &(ros_message->header), current_alignment);

  // Field name: driver_states
  {
    size_t array_size = ros_message->driver_states.size;
    auto array_ptr = ros_message->driver_states.data;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += get_serialized_size_husarion_ugv_msgs__msg__DriverStateNamed(
        &array_ptr[index], current_alignment);
    }
  }

  // Field name: error
  {
    size_t item_size = sizeof(ros_message->error);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: write_pdo_cmds_error
  {
    size_t item_size = sizeof(ros_message->write_pdo_cmds_error);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: read_pdo_motor_states_error
  {
    size_t item_size = sizeof(ros_message->read_pdo_motor_states_error);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: read_pdo_driver_state_error
  {
    size_t item_size = sizeof(ros_message->read_pdo_driver_state_error);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}


ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_husarion_ugv_msgs
size_t max_serialized_size_husarion_ugv_msgs__msg__RobotDriverState(
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

  // Field name: header
  {
    size_t array_size = 1;
    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size;
      inner_size =
        max_serialized_size_std_msgs__msg__Header(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }

  // Field name: driver_states
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
      size_t inner_size;
      inner_size =
        max_serialized_size_husarion_ugv_msgs__msg__DriverStateNamed(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }

  // Field name: error
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Field name: write_pdo_cmds_error
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Field name: read_pdo_motor_states_error
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Field name: read_pdo_driver_state_error
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
    using DataType = husarion_ugv_msgs__msg__RobotDriverState;
    is_plain =
      (
      offsetof(DataType, read_pdo_driver_state_error) +
      last_member_size
      ) == ret_val;
  }
  return ret_val;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_husarion_ugv_msgs
bool cdr_serialize_key_husarion_ugv_msgs__msg__RobotDriverState(
  const husarion_ugv_msgs__msg__RobotDriverState * ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Field name: header
  {
    cdr_serialize_key_std_msgs__msg__Header(
      &ros_message->header, cdr);
  }

  // Field name: driver_states
  {
    size_t size = ros_message->driver_states.size;
    auto array_ptr = ros_message->driver_states.data;
    cdr << static_cast<uint32_t>(size);
    for (size_t i = 0; i < size; ++i) {
      cdr_serialize_key_husarion_ugv_msgs__msg__DriverStateNamed(
        &array_ptr[i], cdr);
    }
  }

  // Field name: error
  {
    cdr << (ros_message->error ? true : false);
  }

  // Field name: write_pdo_cmds_error
  {
    cdr << (ros_message->write_pdo_cmds_error ? true : false);
  }

  // Field name: read_pdo_motor_states_error
  {
    cdr << (ros_message->read_pdo_motor_states_error ? true : false);
  }

  // Field name: read_pdo_driver_state_error
  {
    cdr << (ros_message->read_pdo_driver_state_error ? true : false);
  }

  return true;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_husarion_ugv_msgs
size_t get_serialized_size_key_husarion_ugv_msgs__msg__RobotDriverState(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _RobotDriverState__ros_msg_type * ros_message = static_cast<const _RobotDriverState__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;

  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Field name: header
  current_alignment += get_serialized_size_key_std_msgs__msg__Header(
    &(ros_message->header), current_alignment);

  // Field name: driver_states
  {
    size_t array_size = ros_message->driver_states.size;
    auto array_ptr = ros_message->driver_states.data;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += get_serialized_size_key_husarion_ugv_msgs__msg__DriverStateNamed(
        &array_ptr[index], current_alignment);
    }
  }

  // Field name: error
  {
    size_t item_size = sizeof(ros_message->error);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: write_pdo_cmds_error
  {
    size_t item_size = sizeof(ros_message->write_pdo_cmds_error);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: read_pdo_motor_states_error
  {
    size_t item_size = sizeof(ros_message->read_pdo_motor_states_error);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: read_pdo_driver_state_error
  {
    size_t item_size = sizeof(ros_message->read_pdo_driver_state_error);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_husarion_ugv_msgs
size_t max_serialized_size_key_husarion_ugv_msgs__msg__RobotDriverState(
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
  // Field name: header
  {
    size_t array_size = 1;
    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size;
      inner_size =
        max_serialized_size_key_std_msgs__msg__Header(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }

  // Field name: driver_states
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
      size_t inner_size;
      inner_size =
        max_serialized_size_key_husarion_ugv_msgs__msg__DriverStateNamed(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }

  // Field name: error
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Field name: write_pdo_cmds_error
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Field name: read_pdo_motor_states_error
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Field name: read_pdo_driver_state_error
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
    using DataType = husarion_ugv_msgs__msg__RobotDriverState;
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
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const husarion_ugv_msgs__msg__RobotDriverState * ros_message = static_cast<const husarion_ugv_msgs__msg__RobotDriverState *>(untyped_ros_message);
  (void)ros_message;
  return cdr_serialize_husarion_ugv_msgs__msg__RobotDriverState(ros_message, cdr);
}

static bool _RobotDriverState__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  husarion_ugv_msgs__msg__RobotDriverState * ros_message = static_cast<husarion_ugv_msgs__msg__RobotDriverState *>(untyped_ros_message);
  (void)ros_message;
  return cdr_deserialize_husarion_ugv_msgs__msg__RobotDriverState(cdr, ros_message);
}

static uint32_t _RobotDriverState__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_husarion_ugv_msgs__msg__RobotDriverState(
      untyped_ros_message, 0));
}

static size_t _RobotDriverState__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_husarion_ugv_msgs__msg__RobotDriverState(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_RobotDriverState = {
  "husarion_ugv_msgs::msg",
  "RobotDriverState",
  _RobotDriverState__cdr_serialize,
  _RobotDriverState__cdr_deserialize,
  _RobotDriverState__get_serialized_size,
  _RobotDriverState__max_serialized_size,
  nullptr
};

static rosidl_message_type_support_t _RobotDriverState__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_RobotDriverState,
  get_message_typesupport_handle_function,
  &husarion_ugv_msgs__msg__RobotDriverState__get_type_hash,
  &husarion_ugv_msgs__msg__RobotDriverState__get_type_description,
  &husarion_ugv_msgs__msg__RobotDriverState__get_type_description_sources,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, husarion_ugv_msgs, msg, RobotDriverState)() {
  return &_RobotDriverState__type_support;
}

#if defined(__cplusplus)
}
#endif
