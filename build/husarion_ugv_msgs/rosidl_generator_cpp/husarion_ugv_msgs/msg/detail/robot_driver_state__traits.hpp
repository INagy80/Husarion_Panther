// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from husarion_ugv_msgs:msg/RobotDriverState.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "husarion_ugv_msgs/msg/robot_driver_state.hpp"


#ifndef HUSARION_UGV_MSGS__MSG__DETAIL__ROBOT_DRIVER_STATE__TRAITS_HPP_
#define HUSARION_UGV_MSGS__MSG__DETAIL__ROBOT_DRIVER_STATE__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "husarion_ugv_msgs/msg/detail/robot_driver_state__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"
// Member 'driver_states'
#include "husarion_ugv_msgs/msg/detail/driver_state_named__traits.hpp"

namespace husarion_ugv_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const RobotDriverState & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: driver_states
  {
    if (msg.driver_states.size() == 0) {
      out << "driver_states: []";
    } else {
      out << "driver_states: [";
      size_t pending_items = msg.driver_states.size();
      for (auto item : msg.driver_states) {
        to_flow_style_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: error
  {
    out << "error: ";
    rosidl_generator_traits::value_to_yaml(msg.error, out);
    out << ", ";
  }

  // member: write_pdo_cmds_error
  {
    out << "write_pdo_cmds_error: ";
    rosidl_generator_traits::value_to_yaml(msg.write_pdo_cmds_error, out);
    out << ", ";
  }

  // member: read_pdo_motor_states_error
  {
    out << "read_pdo_motor_states_error: ";
    rosidl_generator_traits::value_to_yaml(msg.read_pdo_motor_states_error, out);
    out << ", ";
  }

  // member: read_pdo_driver_state_error
  {
    out << "read_pdo_driver_state_error: ";
    rosidl_generator_traits::value_to_yaml(msg.read_pdo_driver_state_error, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const RobotDriverState & msg,
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

  // member: driver_states
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.driver_states.size() == 0) {
      out << "driver_states: []\n";
    } else {
      out << "driver_states:\n";
      for (auto item : msg.driver_states) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }

  // member: error
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "error: ";
    rosidl_generator_traits::value_to_yaml(msg.error, out);
    out << "\n";
  }

  // member: write_pdo_cmds_error
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "write_pdo_cmds_error: ";
    rosidl_generator_traits::value_to_yaml(msg.write_pdo_cmds_error, out);
    out << "\n";
  }

  // member: read_pdo_motor_states_error
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "read_pdo_motor_states_error: ";
    rosidl_generator_traits::value_to_yaml(msg.read_pdo_motor_states_error, out);
    out << "\n";
  }

  // member: read_pdo_driver_state_error
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "read_pdo_driver_state_error: ";
    rosidl_generator_traits::value_to_yaml(msg.read_pdo_driver_state_error, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const RobotDriverState & msg, bool use_flow_style = false)
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
  const husarion_ugv_msgs::msg::RobotDriverState & msg,
  std::ostream & out, size_t indentation = 0)
{
  husarion_ugv_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use husarion_ugv_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const husarion_ugv_msgs::msg::RobotDriverState & msg)
{
  return husarion_ugv_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<husarion_ugv_msgs::msg::RobotDriverState>()
{
  return "husarion_ugv_msgs::msg::RobotDriverState";
}

template<>
inline const char * name<husarion_ugv_msgs::msg::RobotDriverState>()
{
  return "husarion_ugv_msgs/msg/RobotDriverState";
}

template<>
struct has_fixed_size<husarion_ugv_msgs::msg::RobotDriverState>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<husarion_ugv_msgs::msg::RobotDriverState>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<husarion_ugv_msgs::msg::RobotDriverState>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // HUSARION_UGV_MSGS__MSG__DETAIL__ROBOT_DRIVER_STATE__TRAITS_HPP_
