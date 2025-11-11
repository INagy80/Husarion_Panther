// generated from rosidl_generator_c/resource/idl__type_support.h.em
// with input from husarion_ugv_msgs:srv/SetLEDAnimation.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "husarion_ugv_msgs/srv/set_led_animation.h"


#ifndef HUSARION_UGV_MSGS__SRV__DETAIL__SET_LED_ANIMATION__TYPE_SUPPORT_H_
#define HUSARION_UGV_MSGS__SRV__DETAIL__SET_LED_ANIMATION__TYPE_SUPPORT_H_

#include "rosidl_typesupport_interface/macros.h"

#include "husarion_ugv_msgs/msg/rosidl_generator_c__visibility_control.h"

#ifdef __cplusplus
extern "C"
{
#endif

#include "rosidl_runtime_c/message_type_support_struct.h"

// Forward declare the get type support functions for this type.
ROSIDL_GENERATOR_C_PUBLIC_husarion_ugv_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
  rosidl_typesupport_c,
  husarion_ugv_msgs,
  srv,
  SetLEDAnimation_Request
)(void);

// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"

// Forward declare the get type support functions for this type.
ROSIDL_GENERATOR_C_PUBLIC_husarion_ugv_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
  rosidl_typesupport_c,
  husarion_ugv_msgs,
  srv,
  SetLEDAnimation_Response
)(void);

// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"

// Forward declare the get type support functions for this type.
ROSIDL_GENERATOR_C_PUBLIC_husarion_ugv_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
  rosidl_typesupport_c,
  husarion_ugv_msgs,
  srv,
  SetLEDAnimation_Event
)(void);

#include "rosidl_runtime_c/service_type_support_struct.h"

// Forward declare the get type support functions for this type.
ROSIDL_GENERATOR_C_PUBLIC_husarion_ugv_msgs
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(
  rosidl_typesupport_c,
  husarion_ugv_msgs,
  srv,
  SetLEDAnimation
)(void);

// Forward declare the function to create a service event message for this type.
ROSIDL_GENERATOR_C_PUBLIC_husarion_ugv_msgs
void *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_CREATE_EVENT_MESSAGE_SYMBOL_NAME(
  rosidl_typesupport_c,
  husarion_ugv_msgs,
  srv,
  SetLEDAnimation
)(
  const rosidl_service_introspection_info_t * info,
  rcutils_allocator_t * allocator,
  const void * request_message,
  const void * response_message);

// Forward declare the function to destroy a service event message for this type.
ROSIDL_GENERATOR_C_PUBLIC_husarion_ugv_msgs
bool
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_DESTROY_EVENT_MESSAGE_SYMBOL_NAME(
  rosidl_typesupport_c,
  husarion_ugv_msgs,
  srv,
  SetLEDAnimation
)(
  void * event_msg,
  rcutils_allocator_t * allocator);

#ifdef __cplusplus
}
#endif

#endif  // HUSARION_UGV_MSGS__SRV__DETAIL__SET_LED_ANIMATION__TYPE_SUPPORT_H_
