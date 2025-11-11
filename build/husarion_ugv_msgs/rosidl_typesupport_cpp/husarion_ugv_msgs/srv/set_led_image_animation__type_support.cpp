// generated from rosidl_typesupport_cpp/resource/idl__type_support.cpp.em
// with input from husarion_ugv_msgs:srv/SetLEDImageAnimation.idl
// generated code does not contain a copyright notice

#include "cstddef"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "husarion_ugv_msgs/srv/detail/set_led_image_animation__functions.h"
#include "husarion_ugv_msgs/srv/detail/set_led_image_animation__struct.hpp"
#include "rosidl_typesupport_cpp/identifier.hpp"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_c/type_support_map.h"
#include "rosidl_typesupport_cpp/message_type_support_dispatch.hpp"
#include "rosidl_typesupport_cpp/visibility_control.h"
#include "rosidl_typesupport_interface/macros.h"

namespace husarion_ugv_msgs
{

namespace srv
{

namespace rosidl_typesupport_cpp
{

typedef struct _SetLEDImageAnimation_Request_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _SetLEDImageAnimation_Request_type_support_ids_t;

static const _SetLEDImageAnimation_Request_type_support_ids_t _SetLEDImageAnimation_Request_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _SetLEDImageAnimation_Request_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _SetLEDImageAnimation_Request_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _SetLEDImageAnimation_Request_type_support_symbol_names_t _SetLEDImageAnimation_Request_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, husarion_ugv_msgs, srv, SetLEDImageAnimation_Request)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, husarion_ugv_msgs, srv, SetLEDImageAnimation_Request)),
  }
};

typedef struct _SetLEDImageAnimation_Request_type_support_data_t
{
  void * data[2];
} _SetLEDImageAnimation_Request_type_support_data_t;

static _SetLEDImageAnimation_Request_type_support_data_t _SetLEDImageAnimation_Request_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _SetLEDImageAnimation_Request_message_typesupport_map = {
  2,
  "husarion_ugv_msgs",
  &_SetLEDImageAnimation_Request_message_typesupport_ids.typesupport_identifier[0],
  &_SetLEDImageAnimation_Request_message_typesupport_symbol_names.symbol_name[0],
  &_SetLEDImageAnimation_Request_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t SetLEDImageAnimation_Request_message_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_SetLEDImageAnimation_Request_message_typesupport_map),
  ::rosidl_typesupport_cpp::get_message_typesupport_handle_function,
  &husarion_ugv_msgs__srv__SetLEDImageAnimation_Request__get_type_hash,
  &husarion_ugv_msgs__srv__SetLEDImageAnimation_Request__get_type_description,
  &husarion_ugv_msgs__srv__SetLEDImageAnimation_Request__get_type_description_sources,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace srv

}  // namespace husarion_ugv_msgs

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<husarion_ugv_msgs::srv::SetLEDImageAnimation_Request>()
{
  return &::husarion_ugv_msgs::srv::rosidl_typesupport_cpp::SetLEDImageAnimation_Request_message_type_support_handle;
}

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_cpp, husarion_ugv_msgs, srv, SetLEDImageAnimation_Request)() {
  return get_message_type_support_handle<husarion_ugv_msgs::srv::SetLEDImageAnimation_Request>();
}

#ifdef __cplusplus
}
#endif
}  // namespace rosidl_typesupport_cpp

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "husarion_ugv_msgs/srv/detail/set_led_image_animation__functions.h"
// already included above
// #include "husarion_ugv_msgs/srv/detail/set_led_image_animation__struct.hpp"
// already included above
// #include "rosidl_typesupport_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support_dispatch.hpp"
// already included above
// #include "rosidl_typesupport_cpp/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace husarion_ugv_msgs
{

namespace srv
{

namespace rosidl_typesupport_cpp
{

typedef struct _SetLEDImageAnimation_Response_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _SetLEDImageAnimation_Response_type_support_ids_t;

static const _SetLEDImageAnimation_Response_type_support_ids_t _SetLEDImageAnimation_Response_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _SetLEDImageAnimation_Response_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _SetLEDImageAnimation_Response_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _SetLEDImageAnimation_Response_type_support_symbol_names_t _SetLEDImageAnimation_Response_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, husarion_ugv_msgs, srv, SetLEDImageAnimation_Response)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, husarion_ugv_msgs, srv, SetLEDImageAnimation_Response)),
  }
};

typedef struct _SetLEDImageAnimation_Response_type_support_data_t
{
  void * data[2];
} _SetLEDImageAnimation_Response_type_support_data_t;

static _SetLEDImageAnimation_Response_type_support_data_t _SetLEDImageAnimation_Response_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _SetLEDImageAnimation_Response_message_typesupport_map = {
  2,
  "husarion_ugv_msgs",
  &_SetLEDImageAnimation_Response_message_typesupport_ids.typesupport_identifier[0],
  &_SetLEDImageAnimation_Response_message_typesupport_symbol_names.symbol_name[0],
  &_SetLEDImageAnimation_Response_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t SetLEDImageAnimation_Response_message_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_SetLEDImageAnimation_Response_message_typesupport_map),
  ::rosidl_typesupport_cpp::get_message_typesupport_handle_function,
  &husarion_ugv_msgs__srv__SetLEDImageAnimation_Response__get_type_hash,
  &husarion_ugv_msgs__srv__SetLEDImageAnimation_Response__get_type_description,
  &husarion_ugv_msgs__srv__SetLEDImageAnimation_Response__get_type_description_sources,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace srv

}  // namespace husarion_ugv_msgs

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<husarion_ugv_msgs::srv::SetLEDImageAnimation_Response>()
{
  return &::husarion_ugv_msgs::srv::rosidl_typesupport_cpp::SetLEDImageAnimation_Response_message_type_support_handle;
}

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_cpp, husarion_ugv_msgs, srv, SetLEDImageAnimation_Response)() {
  return get_message_type_support_handle<husarion_ugv_msgs::srv::SetLEDImageAnimation_Response>();
}

#ifdef __cplusplus
}
#endif
}  // namespace rosidl_typesupport_cpp

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "husarion_ugv_msgs/srv/detail/set_led_image_animation__functions.h"
// already included above
// #include "husarion_ugv_msgs/srv/detail/set_led_image_animation__struct.hpp"
// already included above
// #include "rosidl_typesupport_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support_dispatch.hpp"
// already included above
// #include "rosidl_typesupport_cpp/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace husarion_ugv_msgs
{

namespace srv
{

namespace rosidl_typesupport_cpp
{

typedef struct _SetLEDImageAnimation_Event_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _SetLEDImageAnimation_Event_type_support_ids_t;

static const _SetLEDImageAnimation_Event_type_support_ids_t _SetLEDImageAnimation_Event_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _SetLEDImageAnimation_Event_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _SetLEDImageAnimation_Event_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _SetLEDImageAnimation_Event_type_support_symbol_names_t _SetLEDImageAnimation_Event_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, husarion_ugv_msgs, srv, SetLEDImageAnimation_Event)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, husarion_ugv_msgs, srv, SetLEDImageAnimation_Event)),
  }
};

typedef struct _SetLEDImageAnimation_Event_type_support_data_t
{
  void * data[2];
} _SetLEDImageAnimation_Event_type_support_data_t;

static _SetLEDImageAnimation_Event_type_support_data_t _SetLEDImageAnimation_Event_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _SetLEDImageAnimation_Event_message_typesupport_map = {
  2,
  "husarion_ugv_msgs",
  &_SetLEDImageAnimation_Event_message_typesupport_ids.typesupport_identifier[0],
  &_SetLEDImageAnimation_Event_message_typesupport_symbol_names.symbol_name[0],
  &_SetLEDImageAnimation_Event_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t SetLEDImageAnimation_Event_message_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_SetLEDImageAnimation_Event_message_typesupport_map),
  ::rosidl_typesupport_cpp::get_message_typesupport_handle_function,
  &husarion_ugv_msgs__srv__SetLEDImageAnimation_Event__get_type_hash,
  &husarion_ugv_msgs__srv__SetLEDImageAnimation_Event__get_type_description,
  &husarion_ugv_msgs__srv__SetLEDImageAnimation_Event__get_type_description_sources,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace srv

}  // namespace husarion_ugv_msgs

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<husarion_ugv_msgs::srv::SetLEDImageAnimation_Event>()
{
  return &::husarion_ugv_msgs::srv::rosidl_typesupport_cpp::SetLEDImageAnimation_Event_message_type_support_handle;
}

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_cpp, husarion_ugv_msgs, srv, SetLEDImageAnimation_Event)() {
  return get_message_type_support_handle<husarion_ugv_msgs::srv::SetLEDImageAnimation_Event>();
}

#ifdef __cplusplus
}
#endif
}  // namespace rosidl_typesupport_cpp

// already included above
// #include "cstddef"
#include "rosidl_runtime_c/service_type_support_struct.h"
#include "rosidl_typesupport_cpp/service_type_support.hpp"
// already included above
// #include "husarion_ugv_msgs/srv/detail/set_led_image_animation__struct.hpp"
// already included above
// #include "rosidl_typesupport_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
#include "rosidl_typesupport_cpp/service_type_support_dispatch.hpp"
// already included above
// #include "rosidl_typesupport_cpp/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace husarion_ugv_msgs
{

namespace srv
{

namespace rosidl_typesupport_cpp
{

typedef struct _SetLEDImageAnimation_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _SetLEDImageAnimation_type_support_ids_t;

static const _SetLEDImageAnimation_type_support_ids_t _SetLEDImageAnimation_service_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _SetLEDImageAnimation_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _SetLEDImageAnimation_type_support_symbol_names_t;
#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _SetLEDImageAnimation_type_support_symbol_names_t _SetLEDImageAnimation_service_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, husarion_ugv_msgs, srv, SetLEDImageAnimation)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, husarion_ugv_msgs, srv, SetLEDImageAnimation)),
  }
};

typedef struct _SetLEDImageAnimation_type_support_data_t
{
  void * data[2];
} _SetLEDImageAnimation_type_support_data_t;

static _SetLEDImageAnimation_type_support_data_t _SetLEDImageAnimation_service_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _SetLEDImageAnimation_service_typesupport_map = {
  2,
  "husarion_ugv_msgs",
  &_SetLEDImageAnimation_service_typesupport_ids.typesupport_identifier[0],
  &_SetLEDImageAnimation_service_typesupport_symbol_names.symbol_name[0],
  &_SetLEDImageAnimation_service_typesupport_data.data[0],
};

static const rosidl_service_type_support_t SetLEDImageAnimation_service_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_SetLEDImageAnimation_service_typesupport_map),
  ::rosidl_typesupport_cpp::get_service_typesupport_handle_function,
  ::rosidl_typesupport_cpp::get_message_type_support_handle<husarion_ugv_msgs::srv::SetLEDImageAnimation_Request>(),
  ::rosidl_typesupport_cpp::get_message_type_support_handle<husarion_ugv_msgs::srv::SetLEDImageAnimation_Response>(),
  ::rosidl_typesupport_cpp::get_message_type_support_handle<husarion_ugv_msgs::srv::SetLEDImageAnimation_Event>(),
  &::rosidl_typesupport_cpp::service_create_event_message<husarion_ugv_msgs::srv::SetLEDImageAnimation>,
  &::rosidl_typesupport_cpp::service_destroy_event_message<husarion_ugv_msgs::srv::SetLEDImageAnimation>,
  &husarion_ugv_msgs__srv__SetLEDImageAnimation__get_type_hash,
  &husarion_ugv_msgs__srv__SetLEDImageAnimation__get_type_description,
  &husarion_ugv_msgs__srv__SetLEDImageAnimation__get_type_description_sources,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace srv

}  // namespace husarion_ugv_msgs

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_service_type_support_t *
get_service_type_support_handle<husarion_ugv_msgs::srv::SetLEDImageAnimation>()
{
  return &::husarion_ugv_msgs::srv::rosidl_typesupport_cpp::SetLEDImageAnimation_service_type_support_handle;
}

}  // namespace rosidl_typesupport_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_cpp, husarion_ugv_msgs, srv, SetLEDImageAnimation)() {
  return ::rosidl_typesupport_cpp::get_service_type_support_handle<husarion_ugv_msgs::srv::SetLEDImageAnimation>();
}

#ifdef __cplusplus
}
#endif
