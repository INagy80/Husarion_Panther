// generated from rosidl_typesupport_c/resource/idl__type_support.cpp.em
// with input from husarion_ugv_msgs:msg/ChargingStatus.idl
// generated code does not contain a copyright notice

#include "cstddef"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "husarion_ugv_msgs/msg/detail/charging_status__struct.h"
#include "husarion_ugv_msgs/msg/detail/charging_status__type_support.h"
#include "husarion_ugv_msgs/msg/detail/charging_status__functions.h"
#include "rosidl_typesupport_c/identifier.h"
#include "rosidl_typesupport_c/message_type_support_dispatch.h"
#include "rosidl_typesupport_c/type_support_map.h"
#include "rosidl_typesupport_c/visibility_control.h"
#include "rosidl_typesupport_interface/macros.h"

namespace husarion_ugv_msgs
{

namespace msg
{

namespace rosidl_typesupport_c
{

typedef struct _ChargingStatus_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _ChargingStatus_type_support_ids_t;

static const _ChargingStatus_type_support_ids_t _ChargingStatus_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_c",  // ::rosidl_typesupport_fastrtps_c::typesupport_identifier,
    "rosidl_typesupport_introspection_c",  // ::rosidl_typesupport_introspection_c::typesupport_identifier,
  }
};

typedef struct _ChargingStatus_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _ChargingStatus_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _ChargingStatus_type_support_symbol_names_t _ChargingStatus_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, husarion_ugv_msgs, msg, ChargingStatus)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, husarion_ugv_msgs, msg, ChargingStatus)),
  }
};

typedef struct _ChargingStatus_type_support_data_t
{
  void * data[2];
} _ChargingStatus_type_support_data_t;

static _ChargingStatus_type_support_data_t _ChargingStatus_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _ChargingStatus_message_typesupport_map = {
  2,
  "husarion_ugv_msgs",
  &_ChargingStatus_message_typesupport_ids.typesupport_identifier[0],
  &_ChargingStatus_message_typesupport_symbol_names.symbol_name[0],
  &_ChargingStatus_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t ChargingStatus_message_type_support_handle = {
  rosidl_typesupport_c__typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_ChargingStatus_message_typesupport_map),
  rosidl_typesupport_c__get_message_typesupport_handle_function,
  &husarion_ugv_msgs__msg__ChargingStatus__get_type_hash,
  &husarion_ugv_msgs__msg__ChargingStatus__get_type_description,
  &husarion_ugv_msgs__msg__ChargingStatus__get_type_description_sources,
};

}  // namespace rosidl_typesupport_c

}  // namespace msg

}  // namespace husarion_ugv_msgs

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_c, husarion_ugv_msgs, msg, ChargingStatus)() {
  return &::husarion_ugv_msgs::msg::rosidl_typesupport_c::ChargingStatus_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
