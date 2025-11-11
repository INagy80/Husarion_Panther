// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from husarion_ugv_msgs:msg/LEDImageAnimation.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "husarion_ugv_msgs/msg/led_image_animation.hpp"


#ifndef HUSARION_UGV_MSGS__MSG__DETAIL__LED_IMAGE_ANIMATION__TRAITS_HPP_
#define HUSARION_UGV_MSGS__MSG__DETAIL__LED_IMAGE_ANIMATION__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "husarion_ugv_msgs/msg/detail/led_image_animation__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace husarion_ugv_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const LEDImageAnimation & msg,
  std::ostream & out)
{
  out << "{";
  // member: image
  {
    out << "image: ";
    rosidl_generator_traits::value_to_yaml(msg.image, out);
    out << ", ";
  }

  // member: duration
  {
    out << "duration: ";
    rosidl_generator_traits::value_to_yaml(msg.duration, out);
    out << ", ";
  }

  // member: brightness
  {
    out << "brightness: ";
    rosidl_generator_traits::value_to_yaml(msg.brightness, out);
    out << ", ";
  }

  // member: repeat
  {
    out << "repeat: ";
    rosidl_generator_traits::value_to_yaml(msg.repeat, out);
    out << ", ";
  }

  // member: color
  {
    out << "color: ";
    rosidl_generator_traits::value_to_yaml(msg.color, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const LEDImageAnimation & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: image
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "image: ";
    rosidl_generator_traits::value_to_yaml(msg.image, out);
    out << "\n";
  }

  // member: duration
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "duration: ";
    rosidl_generator_traits::value_to_yaml(msg.duration, out);
    out << "\n";
  }

  // member: brightness
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "brightness: ";
    rosidl_generator_traits::value_to_yaml(msg.brightness, out);
    out << "\n";
  }

  // member: repeat
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "repeat: ";
    rosidl_generator_traits::value_to_yaml(msg.repeat, out);
    out << "\n";
  }

  // member: color
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "color: ";
    rosidl_generator_traits::value_to_yaml(msg.color, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const LEDImageAnimation & msg, bool use_flow_style = false)
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
  const husarion_ugv_msgs::msg::LEDImageAnimation & msg,
  std::ostream & out, size_t indentation = 0)
{
  husarion_ugv_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use husarion_ugv_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const husarion_ugv_msgs::msg::LEDImageAnimation & msg)
{
  return husarion_ugv_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<husarion_ugv_msgs::msg::LEDImageAnimation>()
{
  return "husarion_ugv_msgs::msg::LEDImageAnimation";
}

template<>
inline const char * name<husarion_ugv_msgs::msg::LEDImageAnimation>()
{
  return "husarion_ugv_msgs/msg/LEDImageAnimation";
}

template<>
struct has_fixed_size<husarion_ugv_msgs::msg::LEDImageAnimation>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<husarion_ugv_msgs::msg::LEDImageAnimation>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<husarion_ugv_msgs::msg::LEDImageAnimation>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // HUSARION_UGV_MSGS__MSG__DETAIL__LED_IMAGE_ANIMATION__TRAITS_HPP_
