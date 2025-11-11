// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from husarion_ugv_msgs:msg/FaultFlag.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "husarion_ugv_msgs/msg/fault_flag.hpp"


#ifndef HUSARION_UGV_MSGS__MSG__DETAIL__FAULT_FLAG__TRAITS_HPP_
#define HUSARION_UGV_MSGS__MSG__DETAIL__FAULT_FLAG__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "husarion_ugv_msgs/msg/detail/fault_flag__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace husarion_ugv_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const FaultFlag & msg,
  std::ostream & out)
{
  out << "{";
  // member: overheat
  {
    out << "overheat: ";
    rosidl_generator_traits::value_to_yaml(msg.overheat, out);
    out << ", ";
  }

  // member: overvoltage
  {
    out << "overvoltage: ";
    rosidl_generator_traits::value_to_yaml(msg.overvoltage, out);
    out << ", ";
  }

  // member: undervoltage
  {
    out << "undervoltage: ";
    rosidl_generator_traits::value_to_yaml(msg.undervoltage, out);
    out << ", ";
  }

  // member: short_circuit
  {
    out << "short_circuit: ";
    rosidl_generator_traits::value_to_yaml(msg.short_circuit, out);
    out << ", ";
  }

  // member: emergency_stop
  {
    out << "emergency_stop: ";
    rosidl_generator_traits::value_to_yaml(msg.emergency_stop, out);
    out << ", ";
  }

  // member: motor_or_sensor_setup_fault
  {
    out << "motor_or_sensor_setup_fault: ";
    rosidl_generator_traits::value_to_yaml(msg.motor_or_sensor_setup_fault, out);
    out << ", ";
  }

  // member: mosfet_failure
  {
    out << "mosfet_failure: ";
    rosidl_generator_traits::value_to_yaml(msg.mosfet_failure, out);
    out << ", ";
  }

  // member: default_config_loaded_at_startup
  {
    out << "default_config_loaded_at_startup: ";
    rosidl_generator_traits::value_to_yaml(msg.default_config_loaded_at_startup, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const FaultFlag & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: overheat
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "overheat: ";
    rosidl_generator_traits::value_to_yaml(msg.overheat, out);
    out << "\n";
  }

  // member: overvoltage
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "overvoltage: ";
    rosidl_generator_traits::value_to_yaml(msg.overvoltage, out);
    out << "\n";
  }

  // member: undervoltage
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "undervoltage: ";
    rosidl_generator_traits::value_to_yaml(msg.undervoltage, out);
    out << "\n";
  }

  // member: short_circuit
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "short_circuit: ";
    rosidl_generator_traits::value_to_yaml(msg.short_circuit, out);
    out << "\n";
  }

  // member: emergency_stop
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "emergency_stop: ";
    rosidl_generator_traits::value_to_yaml(msg.emergency_stop, out);
    out << "\n";
  }

  // member: motor_or_sensor_setup_fault
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "motor_or_sensor_setup_fault: ";
    rosidl_generator_traits::value_to_yaml(msg.motor_or_sensor_setup_fault, out);
    out << "\n";
  }

  // member: mosfet_failure
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "mosfet_failure: ";
    rosidl_generator_traits::value_to_yaml(msg.mosfet_failure, out);
    out << "\n";
  }

  // member: default_config_loaded_at_startup
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "default_config_loaded_at_startup: ";
    rosidl_generator_traits::value_to_yaml(msg.default_config_loaded_at_startup, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const FaultFlag & msg, bool use_flow_style = false)
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
  const husarion_ugv_msgs::msg::FaultFlag & msg,
  std::ostream & out, size_t indentation = 0)
{
  husarion_ugv_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use husarion_ugv_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const husarion_ugv_msgs::msg::FaultFlag & msg)
{
  return husarion_ugv_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<husarion_ugv_msgs::msg::FaultFlag>()
{
  return "husarion_ugv_msgs::msg::FaultFlag";
}

template<>
inline const char * name<husarion_ugv_msgs::msg::FaultFlag>()
{
  return "husarion_ugv_msgs/msg/FaultFlag";
}

template<>
struct has_fixed_size<husarion_ugv_msgs::msg::FaultFlag>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<husarion_ugv_msgs::msg::FaultFlag>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<husarion_ugv_msgs::msg::FaultFlag>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // HUSARION_UGV_MSGS__MSG__DETAIL__FAULT_FLAG__TRAITS_HPP_
