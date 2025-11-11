// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from husarion_ugv_msgs:msg/ChargingStatus.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "husarion_ugv_msgs/msg/charging_status.hpp"


#ifndef HUSARION_UGV_MSGS__MSG__DETAIL__CHARGING_STATUS__TRAITS_HPP_
#define HUSARION_UGV_MSGS__MSG__DETAIL__CHARGING_STATUS__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "husarion_ugv_msgs/msg/detail/charging_status__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"

namespace husarion_ugv_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const ChargingStatus & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: charging
  {
    out << "charging: ";
    rosidl_generator_traits::value_to_yaml(msg.charging, out);
    out << ", ";
  }

  // member: current
  {
    out << "current: ";
    rosidl_generator_traits::value_to_yaml(msg.current, out);
    out << ", ";
  }

  // member: current_battery_1
  {
    out << "current_battery_1: ";
    rosidl_generator_traits::value_to_yaml(msg.current_battery_1, out);
    out << ", ";
  }

  // member: current_battery_2
  {
    out << "current_battery_2: ";
    rosidl_generator_traits::value_to_yaml(msg.current_battery_2, out);
    out << ", ";
  }

  // member: charger_type
  {
    out << "charger_type: ";
    rosidl_generator_traits::value_to_yaml(msg.charger_type, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const ChargingStatus & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: header
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "header:\n";
    to_block_style_yaml(msg.header, out, indentation + 2);
  }

  // member: charging
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "charging: ";
    rosidl_generator_traits::value_to_yaml(msg.charging, out);
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

  // member: current_battery_1
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "current_battery_1: ";
    rosidl_generator_traits::value_to_yaml(msg.current_battery_1, out);
    out << "\n";
  }

  // member: current_battery_2
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "current_battery_2: ";
    rosidl_generator_traits::value_to_yaml(msg.current_battery_2, out);
    out << "\n";
  }

  // member: charger_type
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "charger_type: ";
    rosidl_generator_traits::value_to_yaml(msg.charger_type, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const ChargingStatus & msg, bool use_flow_style = false)
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
  const husarion_ugv_msgs::msg::ChargingStatus & msg,
  std::ostream & out, size_t indentation = 0)
{
  husarion_ugv_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use husarion_ugv_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const husarion_ugv_msgs::msg::ChargingStatus & msg)
{
  return husarion_ugv_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<husarion_ugv_msgs::msg::ChargingStatus>()
{
  return "husarion_ugv_msgs::msg::ChargingStatus";
}

template<>
inline const char * name<husarion_ugv_msgs::msg::ChargingStatus>()
{
  return "husarion_ugv_msgs/msg/ChargingStatus";
}

template<>
struct has_fixed_size<husarion_ugv_msgs::msg::ChargingStatus>
  : std::integral_constant<bool, has_fixed_size<std_msgs::msg::Header>::value> {};

template<>
struct has_bounded_size<husarion_ugv_msgs::msg::ChargingStatus>
  : std::integral_constant<bool, has_bounded_size<std_msgs::msg::Header>::value> {};

template<>
struct is_message<husarion_ugv_msgs::msg::ChargingStatus>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // HUSARION_UGV_MSGS__MSG__DETAIL__CHARGING_STATUS__TRAITS_HPP_
