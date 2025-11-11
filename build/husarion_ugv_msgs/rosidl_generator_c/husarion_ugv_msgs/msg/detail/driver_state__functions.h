// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from husarion_ugv_msgs:msg/DriverState.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "husarion_ugv_msgs/msg/driver_state.h"


#ifndef HUSARION_UGV_MSGS__MSG__DETAIL__DRIVER_STATE__FUNCTIONS_H_
#define HUSARION_UGV_MSGS__MSG__DETAIL__DRIVER_STATE__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/action_type_support_struct.h"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_runtime_c/service_type_support_struct.h"
#include "rosidl_runtime_c/type_description/type_description__struct.h"
#include "rosidl_runtime_c/type_description/type_source__struct.h"
#include "rosidl_runtime_c/type_hash.h"
#include "rosidl_runtime_c/visibility_control.h"
#include "husarion_ugv_msgs/msg/rosidl_generator_c__visibility_control.h"

#include "husarion_ugv_msgs/msg/detail/driver_state__struct.h"

/// Initialize msg/DriverState message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * husarion_ugv_msgs__msg__DriverState
 * )) before or use
 * husarion_ugv_msgs__msg__DriverState__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_husarion_ugv_msgs
bool
husarion_ugv_msgs__msg__DriverState__init(husarion_ugv_msgs__msg__DriverState * msg);

/// Finalize msg/DriverState message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_husarion_ugv_msgs
void
husarion_ugv_msgs__msg__DriverState__fini(husarion_ugv_msgs__msg__DriverState * msg);

/// Create msg/DriverState message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * husarion_ugv_msgs__msg__DriverState__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_husarion_ugv_msgs
husarion_ugv_msgs__msg__DriverState *
husarion_ugv_msgs__msg__DriverState__create(void);

/// Destroy msg/DriverState message.
/**
 * It calls
 * husarion_ugv_msgs__msg__DriverState__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_husarion_ugv_msgs
void
husarion_ugv_msgs__msg__DriverState__destroy(husarion_ugv_msgs__msg__DriverState * msg);

/// Check for msg/DriverState message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_husarion_ugv_msgs
bool
husarion_ugv_msgs__msg__DriverState__are_equal(const husarion_ugv_msgs__msg__DriverState * lhs, const husarion_ugv_msgs__msg__DriverState * rhs);

/// Copy a msg/DriverState message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_husarion_ugv_msgs
bool
husarion_ugv_msgs__msg__DriverState__copy(
  const husarion_ugv_msgs__msg__DriverState * input,
  husarion_ugv_msgs__msg__DriverState * output);

/// Retrieve pointer to the hash of the description of this type.
ROSIDL_GENERATOR_C_PUBLIC_husarion_ugv_msgs
const rosidl_type_hash_t *
husarion_ugv_msgs__msg__DriverState__get_type_hash(
  const rosidl_message_type_support_t * type_support);

/// Retrieve pointer to the description of this type.
ROSIDL_GENERATOR_C_PUBLIC_husarion_ugv_msgs
const rosidl_runtime_c__type_description__TypeDescription *
husarion_ugv_msgs__msg__DriverState__get_type_description(
  const rosidl_message_type_support_t * type_support);

/// Retrieve pointer to the single raw source text that defined this type.
ROSIDL_GENERATOR_C_PUBLIC_husarion_ugv_msgs
const rosidl_runtime_c__type_description__TypeSource *
husarion_ugv_msgs__msg__DriverState__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support);

/// Retrieve pointer to the recursive raw sources that defined the description of this type.
ROSIDL_GENERATOR_C_PUBLIC_husarion_ugv_msgs
const rosidl_runtime_c__type_description__TypeSource__Sequence *
husarion_ugv_msgs__msg__DriverState__get_type_description_sources(
  const rosidl_message_type_support_t * type_support);

/// Initialize array of msg/DriverState messages.
/**
 * It allocates the memory for the number of elements and calls
 * husarion_ugv_msgs__msg__DriverState__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_husarion_ugv_msgs
bool
husarion_ugv_msgs__msg__DriverState__Sequence__init(husarion_ugv_msgs__msg__DriverState__Sequence * array, size_t size);

/// Finalize array of msg/DriverState messages.
/**
 * It calls
 * husarion_ugv_msgs__msg__DriverState__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_husarion_ugv_msgs
void
husarion_ugv_msgs__msg__DriverState__Sequence__fini(husarion_ugv_msgs__msg__DriverState__Sequence * array);

/// Create array of msg/DriverState messages.
/**
 * It allocates the memory for the array and calls
 * husarion_ugv_msgs__msg__DriverState__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_husarion_ugv_msgs
husarion_ugv_msgs__msg__DriverState__Sequence *
husarion_ugv_msgs__msg__DriverState__Sequence__create(size_t size);

/// Destroy array of msg/DriverState messages.
/**
 * It calls
 * husarion_ugv_msgs__msg__DriverState__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_husarion_ugv_msgs
void
husarion_ugv_msgs__msg__DriverState__Sequence__destroy(husarion_ugv_msgs__msg__DriverState__Sequence * array);

/// Check for msg/DriverState message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_husarion_ugv_msgs
bool
husarion_ugv_msgs__msg__DriverState__Sequence__are_equal(const husarion_ugv_msgs__msg__DriverState__Sequence * lhs, const husarion_ugv_msgs__msg__DriverState__Sequence * rhs);

/// Copy an array of msg/DriverState messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_husarion_ugv_msgs
bool
husarion_ugv_msgs__msg__DriverState__Sequence__copy(
  const husarion_ugv_msgs__msg__DriverState__Sequence * input,
  husarion_ugv_msgs__msg__DriverState__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // HUSARION_UGV_MSGS__MSG__DETAIL__DRIVER_STATE__FUNCTIONS_H_
