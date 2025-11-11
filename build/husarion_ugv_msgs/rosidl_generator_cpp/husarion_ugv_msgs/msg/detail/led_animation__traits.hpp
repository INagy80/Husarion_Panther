// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from husarion_ugv_msgs:msg/LEDAnimation.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "husarion_ugv_msgs/msg/led_animation.hpp"


#ifndef HUSARION_UGV_MSGS__MSG__DETAIL__LED_ANIMATION__TRAITS_HPP_
#define HUSARION_UGV_MSGS__MSG__DETAIL__LED_ANIMATION__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "husarion_ugv_msgs/msg/detail/led_animation__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace husarion_ugv_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const LEDAnimation & msg,
  std::ostream & out)
{
  out << "{";
  // member: id
  {
    out << "id: ";
    rosidl_generator_traits::value_to_yaml(msg.id, out);
    out << ", ";
  }

  // member: param
  {
    out << "param: ";
    rosidl_generator_traits::value_to_yaml(msg.param, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const LEDAnimation & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "id: ";
    rosidl_generator_traits::value_to_yaml(msg.id, out);
    out << "\n";
  }

  // member: param
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "param: ";
    rosidl_generator_traits::value_to_yaml(msg.param, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const LEDAnimation & msg, bool use_flow_style = false)
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
  const husarion_ugv_msgs::msg::LEDAnimation & msg,
  std::ostream & out, size_t indentation = 0)
{
  husarion_ugv_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use husarion_ugv_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const husarion_ugv_msgs::msg::LEDAnimation & msg)
{
  return husarion_ugv_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<husarion_ugv_msgs::msg::LEDAnimation>()
{
  return "husarion_ugv_msgs::msg::LEDAnimation";
}

template<>
inline const char * name<husarion_ugv_msgs::msg::LEDAnimation>()
{
  return "husarion_ugv_msgs/msg/LEDAnimation";
}

template<>
struct has_fixed_size<husarion_ugv_msgs::msg::LEDAnimation>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<husarion_ugv_msgs::msg::LEDAnimation>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<husarion_ugv_msgs::msg::LEDAnimation>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // HUSARION_UGV_MSGS__MSG__DETAIL__LED_ANIMATION__TRAITS_HPP_
