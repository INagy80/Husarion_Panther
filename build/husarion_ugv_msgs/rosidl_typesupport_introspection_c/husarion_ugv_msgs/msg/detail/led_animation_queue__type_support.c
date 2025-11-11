// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from husarion_ugv_msgs:msg/LEDAnimationQueue.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "husarion_ugv_msgs/msg/detail/led_animation_queue__rosidl_typesupport_introspection_c.h"
#include "husarion_ugv_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "husarion_ugv_msgs/msg/detail/led_animation_queue__functions.h"
#include "husarion_ugv_msgs/msg/detail/led_animation_queue__struct.h"


// Include directives for member types
// Member `queue`
#include "rosidl_runtime_c/string_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void husarion_ugv_msgs__msg__LEDAnimationQueue__rosidl_typesupport_introspection_c__LEDAnimationQueue_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  husarion_ugv_msgs__msg__LEDAnimationQueue__init(message_memory);
}

void husarion_ugv_msgs__msg__LEDAnimationQueue__rosidl_typesupport_introspection_c__LEDAnimationQueue_fini_function(void * message_memory)
{
  husarion_ugv_msgs__msg__LEDAnimationQueue__fini(message_memory);
}

size_t husarion_ugv_msgs__msg__LEDAnimationQueue__rosidl_typesupport_introspection_c__size_function__LEDAnimationQueue__queue(
  const void * untyped_member)
{
  const rosidl_runtime_c__String__Sequence * member =
    (const rosidl_runtime_c__String__Sequence *)(untyped_member);
  return member->size;
}

const void * husarion_ugv_msgs__msg__LEDAnimationQueue__rosidl_typesupport_introspection_c__get_const_function__LEDAnimationQueue__queue(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__String__Sequence * member =
    (const rosidl_runtime_c__String__Sequence *)(untyped_member);
  return &member->data[index];
}

void * husarion_ugv_msgs__msg__LEDAnimationQueue__rosidl_typesupport_introspection_c__get_function__LEDAnimationQueue__queue(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__String__Sequence * member =
    (rosidl_runtime_c__String__Sequence *)(untyped_member);
  return &member->data[index];
}

void husarion_ugv_msgs__msg__LEDAnimationQueue__rosidl_typesupport_introspection_c__fetch_function__LEDAnimationQueue__queue(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const rosidl_runtime_c__String * item =
    ((const rosidl_runtime_c__String *)
    husarion_ugv_msgs__msg__LEDAnimationQueue__rosidl_typesupport_introspection_c__get_const_function__LEDAnimationQueue__queue(untyped_member, index));
  rosidl_runtime_c__String * value =
    (rosidl_runtime_c__String *)(untyped_value);
  *value = *item;
}

void husarion_ugv_msgs__msg__LEDAnimationQueue__rosidl_typesupport_introspection_c__assign_function__LEDAnimationQueue__queue(
  void * untyped_member, size_t index, const void * untyped_value)
{
  rosidl_runtime_c__String * item =
    ((rosidl_runtime_c__String *)
    husarion_ugv_msgs__msg__LEDAnimationQueue__rosidl_typesupport_introspection_c__get_function__LEDAnimationQueue__queue(untyped_member, index));
  const rosidl_runtime_c__String * value =
    (const rosidl_runtime_c__String *)(untyped_value);
  *item = *value;
}

bool husarion_ugv_msgs__msg__LEDAnimationQueue__rosidl_typesupport_introspection_c__resize_function__LEDAnimationQueue__queue(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__String__Sequence * member =
    (rosidl_runtime_c__String__Sequence *)(untyped_member);
  rosidl_runtime_c__String__Sequence__fini(member);
  return rosidl_runtime_c__String__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember husarion_ugv_msgs__msg__LEDAnimationQueue__rosidl_typesupport_introspection_c__LEDAnimationQueue_message_member_array[1] = {
  {
    "queue",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(husarion_ugv_msgs__msg__LEDAnimationQueue, queue),  // bytes offset in struct
    NULL,  // default value
    husarion_ugv_msgs__msg__LEDAnimationQueue__rosidl_typesupport_introspection_c__size_function__LEDAnimationQueue__queue,  // size() function pointer
    husarion_ugv_msgs__msg__LEDAnimationQueue__rosidl_typesupport_introspection_c__get_const_function__LEDAnimationQueue__queue,  // get_const(index) function pointer
    husarion_ugv_msgs__msg__LEDAnimationQueue__rosidl_typesupport_introspection_c__get_function__LEDAnimationQueue__queue,  // get(index) function pointer
    husarion_ugv_msgs__msg__LEDAnimationQueue__rosidl_typesupport_introspection_c__fetch_function__LEDAnimationQueue__queue,  // fetch(index, &value) function pointer
    husarion_ugv_msgs__msg__LEDAnimationQueue__rosidl_typesupport_introspection_c__assign_function__LEDAnimationQueue__queue,  // assign(index, value) function pointer
    husarion_ugv_msgs__msg__LEDAnimationQueue__rosidl_typesupport_introspection_c__resize_function__LEDAnimationQueue__queue  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers husarion_ugv_msgs__msg__LEDAnimationQueue__rosidl_typesupport_introspection_c__LEDAnimationQueue_message_members = {
  "husarion_ugv_msgs__msg",  // message namespace
  "LEDAnimationQueue",  // message name
  1,  // number of fields
  sizeof(husarion_ugv_msgs__msg__LEDAnimationQueue),
  false,  // has_any_key_member_
  husarion_ugv_msgs__msg__LEDAnimationQueue__rosidl_typesupport_introspection_c__LEDAnimationQueue_message_member_array,  // message members
  husarion_ugv_msgs__msg__LEDAnimationQueue__rosidl_typesupport_introspection_c__LEDAnimationQueue_init_function,  // function to initialize message memory (memory has to be allocated)
  husarion_ugv_msgs__msg__LEDAnimationQueue__rosidl_typesupport_introspection_c__LEDAnimationQueue_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t husarion_ugv_msgs__msg__LEDAnimationQueue__rosidl_typesupport_introspection_c__LEDAnimationQueue_message_type_support_handle = {
  0,
  &husarion_ugv_msgs__msg__LEDAnimationQueue__rosidl_typesupport_introspection_c__LEDAnimationQueue_message_members,
  get_message_typesupport_handle_function,
  &husarion_ugv_msgs__msg__LEDAnimationQueue__get_type_hash,
  &husarion_ugv_msgs__msg__LEDAnimationQueue__get_type_description,
  &husarion_ugv_msgs__msg__LEDAnimationQueue__get_type_description_sources,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_husarion_ugv_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, husarion_ugv_msgs, msg, LEDAnimationQueue)() {
  if (!husarion_ugv_msgs__msg__LEDAnimationQueue__rosidl_typesupport_introspection_c__LEDAnimationQueue_message_type_support_handle.typesupport_identifier) {
    husarion_ugv_msgs__msg__LEDAnimationQueue__rosidl_typesupport_introspection_c__LEDAnimationQueue_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &husarion_ugv_msgs__msg__LEDAnimationQueue__rosidl_typesupport_introspection_c__LEDAnimationQueue_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
