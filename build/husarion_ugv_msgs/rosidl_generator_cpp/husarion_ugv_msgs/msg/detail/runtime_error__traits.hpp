// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from husarion_ugv_msgs:msg/RuntimeError.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "husarion_ugv_msgs/msg/runtime_error.hpp"


#ifndef HUSARION_UGV_MSGS__MSG__DETAIL__RUNTIME_ERROR__TRAITS_HPP_
#define HUSARION_UGV_MSGS__MSG__DETAIL__RUNTIME_ERROR__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "husarion_ugv_msgs/msg/detail/runtime_error__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace husarion_ugv_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const RuntimeError & msg,
  std::ostream & out)
{
  out << "{";
  // member: amps_limit_active
  {
    out << "amps_limit_active: ";
    rosidl_generator_traits::value_to_yaml(msg.amps_limit_active, out);
    out << ", ";
  }

  // member: motor_stall
  {
    out << "motor_stall: ";
    rosidl_generator_traits::value_to_yaml(msg.motor_stall, out);
    out << ", ";
  }

  // member: loop_error
  {
    out << "loop_error: ";
    rosidl_generator_traits::value_to_yaml(msg.loop_error, out);
    out << ", ";
  }

  // member: safety_stop_active
  {
    out << "safety_stop_active: ";
    rosidl_generator_traits::value_to_yaml(msg.safety_stop_active, out);
    out << ", ";
  }

  // member: forward_limit_triggered
  {
    out << "forward_limit_triggered: ";
    rosidl_generator_traits::value_to_yaml(msg.forward_limit_triggered, out);
    out << ", ";
  }

  // member: reverse_limit_triggered
  {
    out << "reverse_limit_triggered: ";
    rosidl_generator_traits::value_to_yaml(msg.reverse_limit_triggered, out);
    out << ", ";
  }

  // member: amps_trigger_activated
  {
    out << "amps_trigger_activated: ";
    rosidl_generator_traits::value_to_yaml(msg.amps_trigger_activated, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const RuntimeError & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: amps_limit_active
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "amps_limit_active: ";
    rosidl_generator_traits::value_to_yaml(msg.amps_limit_active, out);
    out << "\n";
  }

  // member: motor_stall
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "motor_stall: ";
    rosidl_generator_traits::value_to_yaml(msg.motor_stall, out);
    out << "\n";
  }

  // member: loop_error
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "loop_error: ";
    rosidl_generator_traits::value_to_yaml(msg.loop_error, out);
    out << "\n";
  }

  // member: safety_stop_active
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "safety_stop_active: ";
    rosidl_generator_traits::value_to_yaml(msg.safety_stop_active, out);
    out << "\n";
  }

  // member: forward_limit_triggered
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "forward_limit_triggered: ";
    rosidl_generator_traits::value_to_yaml(msg.forward_limit_triggered, out);
    out << "\n";
  }

  // member: reverse_limit_triggered
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "reverse_limit_triggered: ";
    rosidl_generator_traits::value_to_yaml(msg.reverse_limit_triggered, out);
    out << "\n";
  }

  // member: amps_trigger_activated
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "amps_trigger_activated: ";
    rosidl_generator_traits::value_to_yaml(msg.amps_trigger_activated, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const RuntimeError & msg, bool use_flow_style = false)
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
  const husarion_ugv_msgs::msg::RuntimeError & msg,
  std::ostream & out, size_t indentation = 0)
{
  husarion_ugv_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use husarion_ugv_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const husarion_ugv_msgs::msg::RuntimeError & msg)
{
  return husarion_ugv_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<husarion_ugv_msgs::msg::RuntimeError>()
{
  return "husarion_ugv_msgs::msg::RuntimeError";
}

template<>
inline const char * name<husarion_ugv_msgs::msg::RuntimeError>()
{
  return "husarion_ugv_msgs/msg/RuntimeError";
}

template<>
struct has_fixed_size<husarion_ugv_msgs::msg::RuntimeError>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<husarion_ugv_msgs::msg::RuntimeError>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<husarion_ugv_msgs::msg::RuntimeError>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // HUSARION_UGV_MSGS__MSG__DETAIL__RUNTIME_ERROR__TRAITS_HPP_
