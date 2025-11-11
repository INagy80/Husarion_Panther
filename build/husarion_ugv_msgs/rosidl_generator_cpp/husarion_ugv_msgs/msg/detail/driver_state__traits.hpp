// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from husarion_ugv_msgs:msg/DriverState.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "husarion_ugv_msgs/msg/driver_state.hpp"


#ifndef HUSARION_UGV_MSGS__MSG__DETAIL__DRIVER_STATE__TRAITS_HPP_
#define HUSARION_UGV_MSGS__MSG__DETAIL__DRIVER_STATE__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "husarion_ugv_msgs/msg/detail/driver_state__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'fault_flag'
#include "husarion_ugv_msgs/msg/detail/fault_flag__traits.hpp"
// Member 'script_flag'
#include "husarion_ugv_msgs/msg/detail/script_flag__traits.hpp"
// Member 'channel_1_motor_runtime_error'
// Member 'channel_2_motor_runtime_error'
#include "husarion_ugv_msgs/msg/detail/runtime_error__traits.hpp"

namespace husarion_ugv_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const DriverState & msg,
  std::ostream & out)
{
  out << "{";
  // member: voltage
  {
    out << "voltage: ";
    rosidl_generator_traits::value_to_yaml(msg.voltage, out);
    out << ", ";
  }

  // member: current
  {
    out << "current: ";
    rosidl_generator_traits::value_to_yaml(msg.current, out);
    out << ", ";
  }

  // member: temperature
  {
    out << "temperature: ";
    rosidl_generator_traits::value_to_yaml(msg.temperature, out);
    out << ", ";
  }

  // member: heatsink_temperature
  {
    out << "heatsink_temperature: ";
    rosidl_generator_traits::value_to_yaml(msg.heatsink_temperature, out);
    out << ", ";
  }

  // member: fault_flag
  {
    out << "fault_flag: ";
    to_flow_style_yaml(msg.fault_flag, out);
    out << ", ";
  }

  // member: script_flag
  {
    out << "script_flag: ";
    to_flow_style_yaml(msg.script_flag, out);
    out << ", ";
  }

  // member: channel_1_motor_runtime_error
  {
    out << "channel_1_motor_runtime_error: ";
    to_flow_style_yaml(msg.channel_1_motor_runtime_error, out);
    out << ", ";
  }

  // member: channel_2_motor_runtime_error
  {
    out << "channel_2_motor_runtime_error: ";
    to_flow_style_yaml(msg.channel_2_motor_runtime_error, out);
    out << ", ";
  }

  // member: motor_states_data_timed_out
  {
    out << "motor_states_data_timed_out: ";
    rosidl_generator_traits::value_to_yaml(msg.motor_states_data_timed_out, out);
    out << ", ";
  }

  // member: driver_state_data_timed_out
  {
    out << "driver_state_data_timed_out: ";
    rosidl_generator_traits::value_to_yaml(msg.driver_state_data_timed_out, out);
    out << ", ";
  }

  // member: can_error
  {
    out << "can_error: ";
    rosidl_generator_traits::value_to_yaml(msg.can_error, out);
    out << ", ";
  }

  // member: heartbeat_timeout
  {
    out << "heartbeat_timeout: ";
    rosidl_generator_traits::value_to_yaml(msg.heartbeat_timeout, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const DriverState & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: voltage
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "voltage: ";
    rosidl_generator_traits::value_to_yaml(msg.voltage, out);
    out << "\n";
  }

  // member: current
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "current: ";
    rosidl_generator_traits::value_to_yaml(msg.current, out);
    out << "\n";
  }

  // member: temperature
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "temperature: ";
    rosidl_generator_traits::value_to_yaml(msg.temperature, out);
    out << "\n";
  }

  // member: heatsink_temperature
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "heatsink_temperature: ";
    rosidl_generator_traits::value_to_yaml(msg.heatsink_temperature, out);
    out << "\n";
  }

  // member: fault_flag
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "fault_flag:\n";
    to_block_style_yaml(msg.fault_flag, out, indentation + 2);
  }

  // member: script_flag
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "script_flag:\n";
    to_block_style_yaml(msg.script_flag, out, indentation + 2);
  }

  // member: channel_1_motor_runtime_error
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "channel_1_motor_runtime_error:\n";
    to_block_style_yaml(msg.channel_1_motor_runtime_error, out, indentation + 2);
  }

  // member: channel_2_motor_runtime_error
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "channel_2_motor_runtime_error:\n";
    to_block_style_yaml(msg.channel_2_motor_runtime_error, out, indentation + 2);
  }

  // member: motor_states_data_timed_out
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "motor_states_data_timed_out: ";
    rosidl_generator_traits::value_to_yaml(msg.motor_states_data_timed_out, out);
    out << "\n";
  }

  // member: driver_state_data_timed_out
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "driver_state_data_timed_out: ";
    rosidl_generator_traits::value_to_yaml(msg.driver_state_data_timed_out, out);
    out << "\n";
  }

  // member: can_error
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "can_error: ";
    rosidl_generator_traits::value_to_yaml(msg.can_error, out);
    out << "\n";
  }

  // member: heartbeat_timeout
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "heartbeat_timeout: ";
    rosidl_generator_traits::value_to_yaml(msg.heartbeat_timeout, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const DriverState & msg, bool use_flow_style = false)
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
  const husarion_ugv_msgs::msg::DriverState & msg,
  std::ostream & out, size_t indentation = 0)
{
  husarion_ugv_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use husarion_ugv_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const husarion_ugv_msgs::msg::DriverState & msg)
{
  return husarion_ugv_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<husarion_ugv_msgs::msg::DriverState>()
{
  return "husarion_ugv_msgs::msg::DriverState";
}

template<>
inline const char * name<husarion_ugv_msgs::msg::DriverState>()
{
  return "husarion_ugv_msgs/msg/DriverState";
}

template<>
struct has_fixed_size<husarion_ugv_msgs::msg::DriverState>
  : std::integral_constant<bool, has_fixed_size<husarion_ugv_msgs::msg::FaultFlag>::value && has_fixed_size<husarion_ugv_msgs::msg::RuntimeError>::value && has_fixed_size<husarion_ugv_msgs::msg::ScriptFlag>::value> {};

template<>
struct has_bounded_size<husarion_ugv_msgs::msg::DriverState>
  : std::integral_constant<bool, has_bounded_size<husarion_ugv_msgs::msg::FaultFlag>::value && has_bounded_size<husarion_ugv_msgs::msg::RuntimeError>::value && has_bounded_size<husarion_ugv_msgs::msg::ScriptFlag>::value> {};

template<>
struct is_message<husarion_ugv_msgs::msg::DriverState>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // HUSARION_UGV_MSGS__MSG__DETAIL__DRIVER_STATE__TRAITS_HPP_
