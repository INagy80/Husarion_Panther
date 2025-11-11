// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from husarion_ugv_msgs:msg/ScriptFlag.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "husarion_ugv_msgs/msg/script_flag.hpp"


#ifndef HUSARION_UGV_MSGS__MSG__DETAIL__SCRIPT_FLAG__TRAITS_HPP_
#define HUSARION_UGV_MSGS__MSG__DETAIL__SCRIPT_FLAG__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "husarion_ugv_msgs/msg/detail/script_flag__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace husarion_ugv_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const ScriptFlag & msg,
  std::ostream & out)
{
  out << "{";
  // member: loop_error
  {
    out << "loop_error: ";
    rosidl_generator_traits::value_to_yaml(msg.loop_error, out);
    out << ", ";
  }

  // member: encoder_disconnected
  {
    out << "encoder_disconnected: ";
    rosidl_generator_traits::value_to_yaml(msg.encoder_disconnected, out);
    out << ", ";
  }

  // member: amp_limiter
  {
    out << "amp_limiter: ";
    rosidl_generator_traits::value_to_yaml(msg.amp_limiter, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const ScriptFlag & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: loop_error
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "loop_error: ";
    rosidl_generator_traits::value_to_yaml(msg.loop_error, out);
    out << "\n";
  }

  // member: encoder_disconnected
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "encoder_disconnected: ";
    rosidl_generator_traits::value_to_yaml(msg.encoder_disconnected, out);
    out << "\n";
  }

  // member: amp_limiter
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "amp_limiter: ";
    rosidl_generator_traits::value_to_yaml(msg.amp_limiter, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const ScriptFlag & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace husarion_ugv_msgs

namespace rosidl_generator_traits
{

[[deprecated("use husarion_ugv_msgs::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const husarion_ugv_msgs::msg::ScriptFlag & msg,
  std::ostream & out, size_t indentation = 0)
{
  husarion_ugv_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use husarion_ugv_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const husarion_ugv_msgs::msg::ScriptFlag & msg)
{
  return husarion_ugv_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<husarion_ugv_msgs::msg::ScriptFlag>()
{
  return "husarion_ugv_msgs::msg::ScriptFlag";
}

template<>
inline const char * name<husarion_ugv_msgs::msg::ScriptFlag>()
{
  return "husarion_ugv_msgs/msg/ScriptFlag";
}

template<>
struct has_fixed_size<husarion_ugv_msgs::msg::ScriptFlag>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<husarion_ugv_msgs::msg::ScriptFlag>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<husarion_ugv_msgs::msg::ScriptFlag>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // HUSARION_UGV_MSGS__MSG__DETAIL__SCRIPT_FLAG__TRAITS_HPP_
