// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from husarion_ugv_msgs:msg/SystemStatus.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "husarion_ugv_msgs/msg/system_status.hpp"


#ifndef HUSARION_UGV_MSGS__MSG__DETAIL__SYSTEM_STATUS__TRAITS_HPP_
#define HUSARION_UGV_MSGS__MSG__DETAIL__SYSTEM_STATUS__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "husarion_ugv_msgs/msg/detail/system_status__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"

namespace husarion_ugv_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const SystemStatus & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: cpu_percent
  {
    if (msg.cpu_percent.size() == 0) {
      out << "cpu_percent: []";
    } else {
      out << "cpu_percent: [";
      size_t pending_items = msg.cpu_percent.size();
      for (auto item : msg.cpu_percent) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: cpu_temp
  {
    out << "cpu_temp: ";
    rosidl_generator_traits::value_to_yaml(msg.cpu_temp, out);
    out << ", ";
  }

  // member: avg_load_percent
  {
    out << "avg_load_percent: ";
    rosidl_generator_traits::value_to_yaml(msg.avg_load_percent, out);
    out << ", ";
  }

  // member: ram_usage_percent
  {
    out << "ram_usage_percent: ";
    rosidl_generator_traits::value_to_yaml(msg.ram_usage_percent, out);
    out << ", ";
  }

  // member: disc_usage_percent
  {
    out << "disc_usage_percent: ";
    rosidl_generator_traits::value_to_yaml(msg.disc_usage_percent, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const SystemStatus & msg,
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

  // member: cpu_percent
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.cpu_percent.size() == 0) {
      out << "cpu_percent: []\n";
    } else {
      out << "cpu_percent:\n";
      for (auto item : msg.cpu_percent) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: cpu_temp
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "cpu_temp: ";
    rosidl_generator_traits::value_to_yaml(msg.cpu_temp, out);
    out << "\n";
  }

  // member: avg_load_percent
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "avg_load_percent: ";
    rosidl_generator_traits::value_to_yaml(msg.avg_load_percent, out);
    out << "\n";
  }

  // member: ram_usage_percent
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "ram_usage_percent: ";
    rosidl_generator_traits::value_to_yaml(msg.ram_usage_percent, out);
    out << "\n";
  }

  // member: disc_usage_percent
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "disc_usage_percent: ";
    rosidl_generator_traits::value_to_yaml(msg.disc_usage_percent, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const SystemStatus & msg, bool use_flow_style = false)
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
  const husarion_ugv_msgs::msg::SystemStatus & msg,
  std::ostream & out, size_t indentation = 0)
{
  husarion_ugv_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use husarion_ugv_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const husarion_ugv_msgs::msg::SystemStatus & msg)
{
  return husarion_ugv_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<husarion_ugv_msgs::msg::SystemStatus>()
{
  return "husarion_ugv_msgs::msg::SystemStatus";
}

template<>
inline const char * name<husarion_ugv_msgs::msg::SystemStatus>()
{
  return "husarion_ugv_msgs/msg/SystemStatus";
}

template<>
struct has_fixed_size<husarion_ugv_msgs::msg::SystemStatus>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<husarion_ugv_msgs::msg::SystemStatus>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<husarion_ugv_msgs::msg::SystemStatus>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // HUSARION_UGV_MSGS__MSG__DETAIL__SYSTEM_STATUS__TRAITS_HPP_
