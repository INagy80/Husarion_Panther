// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from husarion_ugv_msgs:msg/IOState.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "husarion_ugv_msgs/msg/io_state.hpp"


#ifndef HUSARION_UGV_MSGS__MSG__DETAIL__IO_STATE__TRAITS_HPP_
#define HUSARION_UGV_MSGS__MSG__DETAIL__IO_STATE__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "husarion_ugv_msgs/msg/detail/io_state__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace husarion_ugv_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const IOState & msg,
  std::ostream & out)
{
  out << "{";
  // member: aux_power
  {
    out << "aux_power: ";
    rosidl_generator_traits::value_to_yaml(msg.aux_power, out);
    out << ", ";
  }

  // member: charger_connected
  {
    out << "charger_connected: ";
    rosidl_generator_traits::value_to_yaml(msg.charger_connected, out);
    out << ", ";
  }

  // member: charger_enabled
  {
    out << "charger_enabled: ";
    rosidl_generator_traits::value_to_yaml(msg.charger_enabled, out);
    out << ", ";
  }

  // member: digital_power
  {
    out << "digital_power: ";
    rosidl_generator_traits::value_to_yaml(msg.digital_power, out);
    out << ", ";
  }

  // member: fan
  {
    out << "fan: ";
    rosidl_generator_traits::value_to_yaml(msg.fan, out);
    out << ", ";
  }

  // member: led_control
  {
    out << "led_control: ";
    rosidl_generator_traits::value_to_yaml(msg.led_control, out);
    out << ", ";
  }

  // member: motor_on
  {
    out << "motor_on: ";
    rosidl_generator_traits::value_to_yaml(msg.motor_on, out);
    out << ", ";
  }

  // member: power_button
  {
    out << "power_button: ";
    rosidl_generator_traits::value_to_yaml(msg.power_button, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const IOState & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: aux_power
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "aux_power: ";
    rosidl_generator_traits::value_to_yaml(msg.aux_power, out);
    out << "\n";
  }

  // member: charger_connected
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "charger_connected: ";
    rosidl_generator_traits::value_to_yaml(msg.charger_connected, out);
    out << "\n";
  }

  // member: charger_enabled
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "charger_enabled: ";
    rosidl_generator_traits::value_to_yaml(msg.charger_enabled, out);
    out << "\n";
  }

  // member: digital_power
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "digital_power: ";
    rosidl_generator_traits::value_to_yaml(msg.digital_power, out);
    out << "\n";
  }

  // member: fan
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "fan: ";
    rosidl_generator_traits::value_to_yaml(msg.fan, out);
    out << "\n";
  }

  // member: led_control
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "led_control: ";
    rosidl_generator_traits::value_to_yaml(msg.led_control, out);
    out << "\n";
  }

  // member: motor_on
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "motor_on: ";
    rosidl_generator_traits::value_to_yaml(msg.motor_on, out);
    out << "\n";
  }

  // member: power_button
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "power_button: ";
    rosidl_generator_traits::value_to_yaml(msg.power_button, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const IOState & msg, bool use_flow_style = false)
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
  const husarion_ugv_msgs::msg::IOState & msg,
  std::ostream & out, size_t indentation = 0)
{
  husarion_ugv_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use husarion_ugv_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const husarion_ugv_msgs::msg::IOState & msg)
{
  return husarion_ugv_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<husarion_ugv_msgs::msg::IOState>()
{
  return "husarion_ugv_msgs::msg::IOState";
}

template<>
inline const char * name<husarion_ugv_msgs::msg::IOState>()
{
  return "husarion_ugv_msgs/msg/IOState";
}

template<>
struct has_fixed_size<husarion_ugv_msgs::msg::IOState>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<husarion_ugv_msgs::msg::IOState>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<husarion_ugv_msgs::msg::IOState>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // HUSARION_UGV_MSGS__MSG__DETAIL__IO_STATE__TRAITS_HPP_
